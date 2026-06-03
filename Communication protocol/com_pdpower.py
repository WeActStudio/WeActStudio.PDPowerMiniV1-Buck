import serial
import time
import threading
from queue import Queue, Empty, Full
from enum import IntEnum
import traceback


class Command(IntEnum):
    CMD_WHO_AM_I = 0x01
    CMD_OUTPUT_EN_STATE = 0x02
    CMD_OUTPUT_ID = 0x03
    CMD_OUTPUT_DATA = 0x04
    CMD_OUTPUT_DISPLAY = 0x05
    CMD_OUTPUT_OCP_EN = 0x06
    CMD_OUTPUT_OFFSET_EN = 0x07
    CMD_BRIGHTNESS = 0x08
    CMD_OUTPUT_DISCHARGE_EN = 0x09
    CMD_SYSTEM_RESET = 0x40
    CMD_SYSTEM_UPGRADE = 0x41
    CMD_SYSTEM_VERSION = 0x42
    CMD_SYSTEM_SERIAL_NUM = 0x43
    CMD_SYSTEM_CONFIG_SAVE = 0x44
    CMD_SYSTEM_FACTORY_RESET = 0x45
    CMD_SYSTEM_LCD_PANEL_TYPE = 0x46
    CMD_SYSTEM_FACTORY_DATA = 0x47
    CMD_END = 0x0A
    CMD_READ = 0x80

    @property
    def READ_LENGTH(self):
        return {
            Command.CMD_WHO_AM_I: 0,
            Command.CMD_OUTPUT_EN_STATE: 3,
            Command.CMD_OUTPUT_ID: 3,
            Command.CMD_OUTPUT_DATA: 7,
            Command.CMD_OUTPUT_DISPLAY: 6,
            Command.CMD_OUTPUT_OCP_EN: 3,
            Command.CMD_OUTPUT_OFFSET_EN: 3,
            Command.CMD_SYSTEM_FACTORY_DATA: 66,
            Command.CMD_BRIGHTNESS: 3,
            Command.CMD_SYSTEM_VERSION: 0,
            Command.CMD_SYSTEM_SERIAL_NUM: 0,
        }.get(self, 0)


class Com_PdPower:
    def __init__(
        self, port, baudrate=115200, timeout=1, queue_maxsize=100, use_crc8=False
    ):
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        if not self.ser.is_open:
            raise Exception(f"无法打开串口 Unable to open serial port {port}")
        time.sleep(0.1)
        self._stop_event = threading.Event()
        self._read_ok_event = threading.Event()
        self._write_ok_event = threading.Event()
        self._read_result = None
        self._read_thread = threading.Thread(target=self._read_serial, daemon=True)
        self._read_thread.start()
        self._send_queue = Queue(maxsize=queue_maxsize)
        self._send_thread = threading.Thread(target=self._send_commands, daemon=True)
        self._send_thread.start()
        self._use_crc8 = use_crc8

    def _read_serial(self):
        """
        线程函数，持续读取串口数据
        Thread function, continuously read serial port data
        """
        buffer = bytearray()
        time_last = time.time()
        while not self._stop_event.is_set():
            try:
                if self.ser.in_waiting:
                    buffer += self.ser.read(self.ser.in_waiting)
                    time_last = time.time()

                    while len(buffer) > 0:
                        if (buffer[0] & 0x7F) in [
                            Command.CMD_WHO_AM_I,
                            Command.CMD_SYSTEM_VERSION,
                            Command.CMD_SYSTEM_SERIAL_NUM,
                        ]:
                            if len(buffer) > 1 and buffer[1] == len(buffer) - 3 and self._use_crc8:
                                end_pos = 3 + buffer[1]
                                frame = buffer[:end_pos]
                                buffer = buffer[end_pos + 1 :]
                                # Check CRC if enabled
                                if self._use_crc8 and len(frame) > 1:
                                    # Extract data without CRC
                                    data = frame[:-1]
                                    received_crc = frame[-1]
                                    calculated_crc = self.calculate_crc8(data)

                                    if received_crc != calculated_crc:
                                        print(
                                            f"CRC校验失败 CRC check failed. Received: {received_crc:02X}, Calculated: {calculated_crc:02X}"
                                        )
                                        buffer = bytearray()
                                        break

                                    self._read_result = data[2:]
                                else:
                                    self._read_result = frame[2:-1]

                                self._read_ok_event.set()
                            elif Command.CMD_END in buffer:
                                end_pos = buffer.index(Command.CMD_END)
                                frame = buffer[:end_pos]
                                buffer = buffer[end_pos+1:]
                                if len(frame) > 0:
                                    try:
                                        cmd_name = Command(frame[0] & 0x7F).name
                                        self._read_result = frame[1:]
                                        self._read_ok_event.set()
                                    except ValueError:
                                        print(f"收到未知命令帧 Received an unknown command frame: {frame.hex()}")
                            else:
                                break
                        else:
                            cmd = buffer[0] & 0x7F
                            try:
                                if Command(cmd).READ_LENGTH == 0:
                                    break
                            except:
                                break
                            if len(buffer) >= Command(cmd).READ_LENGTH:
                                frame = buffer[: Command(cmd).READ_LENGTH]
                                if self._use_crc8:
                                    # Extract data without CRC
                                    data = frame[:-1]
                                    received_crc = frame[-1]
                                    calculated_crc = self.calculate_crc8(data)

                                    if received_crc != calculated_crc:
                                        print(
                                            f"CRC校验失败 CRC check failed. Received: {received_crc:02X}, Calculated: {calculated_crc:02X}"
                                        )
                                        buffer = bytearray()
                                        break  # Skip this frame

                                    self._read_result = data[1:]
                                else:
                                    self._read_result = frame[1:-1]

                                self._read_ok_event.set()
                                buffer = buffer[Command(cmd).READ_LENGTH :]
                            else:
                                break
                else:
                    if time.time() - time_last > 0.05:
                        buffer = bytearray()
                        time_last = time.time()

                time.sleep(0.01)

            except serial.SerialException as e:
                print(f"串口断开连接 Serial port disconnected: {e}")
                traceback.print_exc()
                self.ser.close()
                return
            except Exception as e:
                print(f"读取串口数据时出错 Error reading serial port data: {e}")
                traceback.print_exc()
                buffer = bytearray()

    def _send_commands(self):
        """
        线程函数，处理队列中的指令发送，队列空闲时每隔 100ms 发送坐标查询命令
        Thread function, handle command sending in the queue, and send coordinate query commands every 100ms when the queue is idle
        """
        while not self._stop_event.is_set():
            try:
                # 持续处理队列中的所有命令
                while True:
                    w_data = self._send_queue.get_nowait()
                    self.ser.write(w_data)
                    self._write_ok_event.set()
            except Empty:
                time.sleep(0.1)
            except serial.SerialException as e:
                print(f"串口断开连接 Serial port disconnected: {e}")
                traceback.print_exc()
                self.ser.close()
                return
            except Exception as e:
                print(f"发送指令时出错 Error sending command: {e}")
                pass

    def send_command(self, cmd):
        """
        发送指令到设备
        Send a command to the device
        """
        try:
            self._send_queue.put(cmd, timeout=0.5)
        except Full:
            print("发送指令队列已满，指令丢弃 Command queue is full, command discarded")

    def who_am_i(self):
        s = bytearray()
        s.append(Command.CMD_WHO_AM_I | Command.CMD_READ)

        # Add CRC if enabled
        if self._use_crc8:
            crc = self.calculate_crc8(s)
            s.append(crc)
        else:
            s.append(Command.CMD_END)

        self._read_ok_event.clear()
        self.send_command(s)
        if not self._read_ok_event.wait(timeout=1.0):  # 等待1秒超时
            print("等待响应超时 Timeout waiting for response")
            return None
        return self._read_result.decode("utf-8")

    def system_version(self):
        s = bytearray()
        s.append(Command.CMD_SYSTEM_VERSION | Command.CMD_READ)

        # Add CRC if enabled
        if self._use_crc8:
            crc = self.calculate_crc8(s)
            s.append(crc)
        else:
            s.append(Command.CMD_END)

        self._read_ok_event.clear()
        self.send_command(s)
        if not self._read_ok_event.wait(timeout=1.0):  # 等待1秒超时
            print("等待响应超时 Timeout waiting for response")
            return None
        return self._read_result.decode("utf-8")

    def system_serial_num(self):
        s = bytearray()
        s.append(Command.CMD_SYSTEM_SERIAL_NUM | Command.CMD_READ)

        # Add CRC if enabled
        if self._use_crc8:
            crc = self.calculate_crc8(s)
            s.append(crc)
        else:
            s.append(Command.CMD_END)

        self._read_ok_event.clear()
        self.send_command(s)
        if not self._read_ok_event.wait(timeout=1.0):  # 等待1秒超时
            print("等待响应超时 Timeout waiting for response")
            return None
        return self._read_result.decode("utf-8")

    def output_enable(self, enable=True):
        s = bytearray(3)
        s[0] = Command.CMD_OUTPUT_EN_STATE
        s[1] = 0x01 if enable else 0x00

        # Add CRC if enabled
        if self._use_crc8:
            crc = self.calculate_crc8(s[:-1])
            s[-1] = crc
        else:
            s[-1] = Command.CMD_END

        self._write_ok_event.clear()
        self.send_command(s)
        if not self._write_ok_event.wait(timeout=1.0):  # 等待1秒超时
            print("等待响应超时 Timeout waiting for response")

    def output_state(self):
        s = bytearray(2)
        s[0] = Command.CMD_OUTPUT_EN_STATE | Command.CMD_READ

        # Add CRC if enabled
        if self._use_crc8:
            crc = self.calculate_crc8(s[:-1])
            s[-1] = crc
        else:
            s[-1] = Command.CMD_END

        self._read_ok_event.clear()
        self.send_command(s)
        if not self._read_ok_event.wait(timeout=1.0):  # 等待1秒超时
            print("等待响应超时 Timeout waiting for response")
            return None
        state = self._read_result[0]
        return state

    def output_id_get(self):
        s = bytearray(2)
        s[0] = Command.CMD_OUTPUT_ID | Command.CMD_READ

        # Add CRC if enabled
        if self._use_crc8:
            crc = self.calculate_crc8(s[:-1])
            s[-1] = crc
        else:
            s[-1] = Command.CMD_END

        self._read_ok_event.clear()
        self.send_command(s)
        if not self._read_ok_event.wait(timeout=1.0):  # 等待1秒超时
            print("等待响应超时 Timeout waiting for response")
            return None
        return self._read_result

    def output_id_set(self, id):
        s = bytearray(3)
        s[0] = Command.CMD_OUTPUT_ID
        s[1] = id

        # Add CRC if enabled
        if self._use_crc8:
            crc = self.calculate_crc8(s[:-1])
            s[-1] = crc
        else:
            s[-1] = Command.CMD_END

        self._write_ok_event.clear()
        self.send_command(s)
        if not self._write_ok_event.wait(timeout=1.0):  # 等待1秒超时
            print("等待响应超时 Timeout waiting for response")

    def output_data_set(self, id, voltage, current):
        s = bytearray(7)
        s[0] = Command.CMD_OUTPUT_DATA
        s[1] = id
        s[2] = int(voltage & 0xFF)
        s[3] = int(voltage >> 8)
        s[4] = int(current & 0xFF)
        s[5] = int(current >> 8)

        # Add CRC if enabled
        if self._use_crc8:
            crc = self.calculate_crc8(s[:-1])
            s[-1] = crc
        else:
            s[-1] = Command.CMD_END

        self._write_ok_event.clear()
        self.send_command(s)
        if not self._write_ok_event.wait(timeout=1.0):  # 等待1秒超时
            print("等待响应超时 Timeout waiting for response")

    def output_data_get(self, id):
        s = bytearray(3)
        s[0] = Command.CMD_OUTPUT_DATA | Command.CMD_READ
        s[1] = id

        # Add CRC if enabled
        if self._use_crc8:
            crc = self.calculate_crc8(s[:-1])
            s[-1] = crc
        else:
            s[-1] = Command.CMD_END

        self._read_ok_event.clear()
        self.send_command(s)
        if not self._read_ok_event.wait(timeout=1.0):  # 等待1秒超时
            print("等待响应超时 Timeout waiting for response")
            return None
        voltage = self._read_result[1] | (self._read_result[2] << 8)
        current = self._read_result[3] | (self._read_result[4] << 8)
        return voltage, current

    def output_display_get(self):
        s = bytearray(2)
        s[0] = Command.CMD_OUTPUT_DISPLAY | Command.CMD_READ

        # Add CRC if enabled
        if self._use_crc8:
            crc = self.calculate_crc8(s[:-1])
            s[-1] = crc
        else:
            s[-1] = Command.CMD_END

        self._read_ok_event.clear()
        self.send_command(s)
        if not self._read_ok_event.wait(timeout=1.0):  # 等待1秒超时
            print("等待响应超时 Timeout waiting for response")
            return None
        voltage = self._read_result[0] | (self._read_result[1] << 8)
        current = self._read_result[2] | (self._read_result[3] << 8)
        return voltage, current

    def output_offset_enable(self, enable=True):
        s = bytearray(3)
        s[0] = Command.CMD_OUTPUT_OFFSET_EN
        s[1] = 0x01 if enable else 0x00
        # Add CRC if enabled
        if self._use_crc8:
            crc = self.calculate_crc8(s[:-1])
            s[-1] = crc
        else:
            s[-1] = Command.CMD_END

        self._write_ok_event.clear()
        self.send_command(s)
        if not self._write_ok_event.wait(timeout=1.0):  # 等待1秒超时
            print("等待响应超时 Timeout waiting for response")

    def factory_data_get(self):
        s = bytearray(2)
        s[0] = Command.CMD_SYSTEM_FACTORY_DATA | Command.CMD_READ

        # Add CRC if enabled
        if self._use_crc8:
            crc = self.calculate_crc8(s[:-1])
            s[-1] = crc
        else:
            s[-1] = Command.CMD_END

        self._read_ok_event.clear()
        self.send_command(s)
        if not self._read_ok_event.wait(timeout=1.0):  # 等待1秒超时
            print("等待响应超时 Timeout waiting for response")
            return None
        dis_voltage_offset = [
            self._read_result[0] | (self._read_result[1] << 8),
            self._read_result[4] | (self._read_result[5] << 8),
            self._read_result[8] | (self._read_result[9] << 8),
            self._read_result[12] | (self._read_result[13] << 8),
            self._read_result[16] | (self._read_result[17] << 8),
            self._read_result[20] | (self._read_result[21] << 8),
        ]

        set_voltage_offset = [
            self._read_result[2] | (self._read_result[3] << 8),
            self._read_result[6] | (self._read_result[7] << 8),
            self._read_result[10] | (self._read_result[11] << 8),
            self._read_result[14] | (self._read_result[15] << 8),
            self._read_result[18] | (self._read_result[19] << 8),
            self._read_result[22] | (self._read_result[23] << 8),
        ]

        dis_current_offset = [
            self._read_result[24] | (self._read_result[25] << 8),
            self._read_result[28] | (self._read_result[29] << 8),
            self._read_result[32] | (self._read_result[33] << 8),
            self._read_result[34] | (self._read_result[37] << 8),
            self._read_result[40] | (self._read_result[41] << 8),
            self._read_result[44] | (self._read_result[45] << 8),
        ]

        set_current_offset = [
            self._read_result[24] | (self._read_result[27] << 8),
            self._read_result[30] | (self._read_result[31] << 8),
            self._read_result[34] | (self._read_result[35] << 8),
            self._read_result[38] | (self._read_result[39] << 8),
            self._read_result[42] | (self._read_result[43] << 8),
            self._read_result[46] | (self._read_result[47] << 8),
        ]

        current_0a_offset = [
            self._read_result[50] | (self._read_result[51] << 8),
            self._read_result[48] | (self._read_result[49] << 8),
        ]

        lcd_panel_type = self._read_result[52] | (self._read_result[53] << 8)
        return (
            dis_voltage_offset,
            set_voltage_offset,
            dis_current_offset,
            set_current_offset,
            current_0a_offset,
            lcd_panel_type,
        )

    def output_discharge_enable(self, enable=True):
        s = bytearray(3)
        s[0] = Command.CMD_OUTPUT_DISCHARGE_EN
        s[1] = 0x01 if enable else 0x00

        # Add CRC if enabled
        if self._use_crc8:
            crc = self.calculate_crc8(s[:-1])
            s[-1] = crc
        else:
            s[-1] = Command.CMD_END

        self._write_ok_event.clear()
        self.send_command(s)
        if not self._write_ok_event.wait(timeout=1.0):  # 等待1秒超时
            print("等待响应超时 Timeout waiting for response")

    def factory_reset(self):
        s = bytearray(2)
        s[0] = Command.CMD_SYSTEM_FACTORY_RESET

        # Add CRC if enabled
        if self._use_crc8:
            crc = self.calculate_crc8(s[:-1])
            s[-1] = crc
        else:
            s[-1] = Command.CMD_END

        self._write_ok_event.clear()
        self.send_command(s)
        if not self._write_ok_event.wait(timeout=1.0):  # 等待1秒超时
            print("等待响应超时 Timeout waiting for response")

    def close(self):
        """
        关闭串口连接并停止读取线程和发送线程
        Close the serial port connection and stop the reading thread and the sending thread
        """
        self._stop_event.set()
        if self._read_thread.is_alive():
            self._read_thread.join()
        if self._send_thread.is_alive():
            self._send_thread.join()
        if self.ser.is_open:
            self.ser.close()

    def is_open(self):
        return self.ser.is_open

    @staticmethod
    def calculate_crc8(data):
        """
        Calculate CRC-8 checksum using polynomial 0x31 (x^8 + x^5 + x^4 + 1)
        with initial value 0xFF and bit-by-bit processing algorithm

        Args:
            data: Bytes or bytearray to calculate CRC for

        Returns:
            CRC-8 checksum as an integer
        """
        crc = 0xFF  # Initial value
        polynomial = 0x31  # x^8 + x^5 + x^4 + 1

        for byte in data:
            crc ^= byte  # XOR with the current byte
            for _ in range(8):  # Process each bit
                if crc & 0x80:  # If the most significant bit is set
                    crc = (crc << 1) ^ polynomial
                else:
                    crc <<= 1
                crc &= 0xFF  # Ensure CRC remains 8-bit

        return crc


if __name__ == "__main__":
    import sys

    if len(sys.argv) > 1:
        port = sys.argv[1]
        try:
            baudrate = int(sys.argv[2])
        except ValueError:
            print("波特率参数必须是整数 Please enter the baud rate as an integer")
            exit(1)
        if len(sys.argv) > 3 and sys.argv[3] == "crc8":
            use_crc8 = True
        else:
            use_crc8 = False
    else:
        print(
            "请输入串口端口号和波特率 Please enter the serial port number and baud rate"
        )
        print("可选参数 Optional parameter: crc8")
        print("示例: python com_PowerMonitorMiniV1.py COM19 115200 crc8")
        exit(1)

    print(f"use crc8 使用CRC8: {use_crc8}")
    print(f"波特率 baudrate: {baudrate}")

    c = Com_PdPower(port=port, baudrate=baudrate, use_crc8=use_crc8)

    print(f"who_am_i: {c.who_am_i()}")
    print(f"system_version: {c.system_version()}")
    print(f"system_serial_num: {c.system_serial_num()}")

    print(f"output_state: {c.output_state()}")

    print(f"output_id_set: {0x00}")
    c.output_id_set(0x00)
    print(f"output_id_get: {c.output_id_get()}")

    print(f"output_data_set: {0x00,3000,1000}")
    c.output_data_set(0x00, 3000, 1000)
    print(f"output_data_get: {c.output_data_get(0x00)}")

    print(f"output_enable: {True}")
    c.output_enable(True)
    print("sleep 1s")
    time.sleep(1)
    print(f"output_enable: {False}")
    c.output_enable(False)

    print(f"output_offset_enable: {True}")
    c.output_offset_enable(True)
    print("sleep 1s")
    time.sleep(1)
    print(f"output_offset_enable: {False}")
    c.output_offset_enable(False)

    print(f"factory_data_get: {c.factory_data_get()}")
    print(f"output_display_get: {c.output_display_get()}")
    # print(f"factory_reset: {True}")
    # c.factory_reset()
    c.close()
    time.sleep(1)
