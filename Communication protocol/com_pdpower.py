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
    CMD_INPUT_STATE = 0x0A
    CMD_OUTPUT_OC_DISCHARGE_EN = 0x0B
    CMD_OUTPUT_AUTO_EN = 0x0C
    CMD_FILTER_WINDOW_SIZE = 0x0D
    CMD_OUTPUT_COMP_EN = 0x0E
    CMD_TIMER_SET = 0x0F
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
            Command.CMD_INPUT_STATE: 7,
            Command.CMD_SYSTEM_FACTORY_DATA: 66,
            Command.CMD_BRIGHTNESS: 3,
            Command.CMD_SYSTEM_VERSION: 0,
            Command.CMD_SYSTEM_SERIAL_NUM: 0,
            Command.CMD_OUTPUT_OC_DISCHARGE_EN: 3,
            Command.CMD_OUTPUT_AUTO_EN: 3,
            Command.CMD_FILTER_WINDOW_SIZE: 3,
            Command.CMD_OUTPUT_COMP_EN: 3,
            Command.CMD_TIMER_SET: 7,
        }.get(self, 0)

class InputState(IntEnum):
    WAIT = 0
    WAIT_PD_OK = 1
    WAIT_QC_OK = 2
    ERR = 3
    QC = 4
    PD = 5
    DC = 6

class Com_PdPower:
    def __init__(
        self, port, baudrate=115200, timeout=1, queue_maxsize=100, use_crc8=False
    ):
        """
        打开串口并启动收、发两个后台线程
        Open the serial port and start the reader and writer threads
        """
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        if not self.ser.is_open:
            raise Exception(f"无法打开串口 Unable to open serial port {port}")
        time.sleep(0.1)
        self._stop_event = threading.Event()
        self._read_ok_event = threading.Event()
        self._write_ok_event = threading.Event()
        self._read_result = None
        self._read_thread = threading.Thread(target=self._read_serial, daemon=True)
        self._use_crc8 = use_crc8
        self._model = None
        # UART frames end with CRC8 and the device splits them by USART IDLE:
        # frames written back to back land in one buffer, fail CRC over the whole
        # buffer and are dropped silently. Four character times of idle is the
        # margin measured on hardware; below three the second frame is lost.
        # 设备靠 USART 空闲中断分帧，背靠背写出的两帧会落进同一段缓冲，
        # 整段校验失败后被静默丢弃。四个字符时间的间隔是实板测出的余量，
        # 低于三个字符时间第二帧就会丢。
        self._baudrate = float(baudrate)
        self._frame_gap = 40.0 / self._baudrate if use_crc8 else 0.0
        self._read_thread.start()
        self._send_queue = Queue(maxsize=queue_maxsize)
        self._send_thread = threading.Thread(target=self._send_commands, daemon=True)
        self._send_thread.start()

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
                                # 启用校验时校验 CRC / Check CRC when enabled
                                if self._use_crc8 and len(frame) > 1:
                                    # 去掉 CRC 后取数据 / Strip CRC from the reply
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
                                    # 去掉 CRC 后取数据 / Strip CRC from the reply
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

                # 1 ms keeps the reply latency near the device's own 3 ms at high
                # baud; 10 ms here capped the whole library at ~96 reads per second
                # 高波特率下 1 ms 轮询才能贴近设备本身的 3 ms 响应；
                # 这里原先用 10 ms，把整个库的读取速度压在每秒 96 次左右。
                time.sleep(0.001)

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
        线程函数，从队列取指令逐条发送
        Thread function, send queued commands one at a time
        """
        while not self._stop_event.is_set():
            try:
                w_data = self._send_queue.get(timeout=0.1)
            except Empty:
                continue
            except serial.SerialException as e:
                print(f"串口断开连接 Serial port disconnected: {e}")
                traceback.print_exc()
                self.ser.close()
                return

            try:
                self.ser.write(w_data)
                if self._frame_gap:
                    # The USB bridge takes bytes far faster than the line sends
                    # them, so flush() and out_waiting both go empty while the
                    # frame is still shifting out. Wait the frame's own airtime,
                    # then the idle gap, or the next frame merges into this one.
                    # USB 桥接收字节远快于线路发送，flush() 和 out_waiting 都会在
                    # 线路仍在发送时变为空。必须按帧自身的上线时长等待，再加上
                    # 空闲间隔，否则下一帧会与本帧粘连。
                    time.sleep(len(w_data) * 10.0 / self._baudrate + self._frame_gap)
                self._write_ok_event.set()
            except serial.SerialException as e:
                print(f"串口断开连接 Serial port disconnected: {e}")
                traceback.print_exc()
                self.ser.close()
                return
            except Exception as e:
                print(f"发送指令时出错 Error sending command: {e}")

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
        """
        读取型号标识字符串
        Read the model identifier string
        """
        s = bytearray()
        s.append(Command.CMD_WHO_AM_I | Command.CMD_READ)

        # 启用校验时追加 CRC / Append CRC when enabled
        if self._use_crc8:
            crc = self.calculate_crc8(s)
            s.append(crc)
        else:
            s.append(Command.CMD_END)

        self._read_ok_event.clear()
        self.send_command(s)
        if not self._read_ok_event.wait(timeout=1.0):  # 等待回应，超时 1 秒 / Wait up to 1s for the reply
            print("等待响应超时 Timeout waiting for response")
            return None
        return self._read_result.decode("utf-8")

    def system_version(self):
        """
        读取固件版本字符串
        Read the firmware version string
        """
        s = bytearray()
        s.append(Command.CMD_SYSTEM_VERSION | Command.CMD_READ)

        # 启用校验时追加 CRC / Append CRC when enabled
        if self._use_crc8:
            crc = self.calculate_crc8(s)
            s.append(crc)
        else:
            s.append(Command.CMD_END)

        self._read_ok_event.clear()
        self.send_command(s)
        if not self._read_ok_event.wait(timeout=1.0):  # 等待回应，超时 1 秒 / Wait up to 1s for the reply
            print("等待响应超时 Timeout waiting for response")
            return None
        return self._read_result.decode("utf-8")

    def system_serial_num(self):
        """
        读取设备序列号
        Read the device serial number
        """
        s = bytearray()
        s.append(Command.CMD_SYSTEM_SERIAL_NUM | Command.CMD_READ)

        # 启用校验时追加 CRC / Append CRC when enabled
        if self._use_crc8:
            crc = self.calculate_crc8(s)
            s.append(crc)
        else:
            s.append(Command.CMD_END)

        self._read_ok_event.clear()
        self.send_command(s)
        if not self._read_ok_event.wait(timeout=1.0):  # 等待回应，超时 1 秒 / Wait up to 1s for the reply
            print("等待响应超时 Timeout waiting for response")
            return None
        return self._read_result.decode("utf-8")

    def output_enable(self, enable=True):
        """
        设置输出使能
        Enable or disable the output
        """
        s = bytearray(3)
        s[0] = Command.CMD_OUTPUT_EN_STATE
        s[1] = 0x01 if enable else 0x00

        # 启用校验时追加 CRC / Append CRC when enabled
        if self._use_crc8:
            crc = self.calculate_crc8(s[:-1])
            s[-1] = crc
        else:
            s[-1] = Command.CMD_END

        self._write_ok_event.clear()
        self.send_command(s)
        if not self._write_ok_event.wait(timeout=1.0):  # 等待回应，超时 1 秒 / Wait up to 1s for the reply
            print("等待响应超时 Timeout waiting for response")

    def output_state(self):
        """
        读取输出状态字节
        Read the output state byte

        bit0 输出使能，bit1 恒流中，bit3 定时序列运行中
        序列断相位时 bit0 为 0、bit3 为 1，与手动关断可区分

        bit0 output enabled, bit1 constant current, bit3 timer sequence running
        In the off phase bit0 is 0 while bit3 stays 1, which tells a timer
        shutdown apart from a manual one
        """
        s = bytearray(2)
        s[0] = Command.CMD_OUTPUT_EN_STATE | Command.CMD_READ

        # 启用校验时追加 CRC / Append CRC when enabled
        if self._use_crc8:
            crc = self.calculate_crc8(s[:-1])
            s[-1] = crc
        else:
            s[-1] = Command.CMD_END

        self._read_ok_event.clear()
        self.send_command(s)
        if not self._read_ok_event.wait(timeout=1.0):  # 等待回应，超时 1 秒 / Wait up to 1s for the reply
            print("等待响应超时 Timeout waiting for response")
            return None
        state = self._read_result[0]
        return state

    def output_id_get(self):
        """
        读取当前输出组号
        Read the active output group index
        """
        s = bytearray(2)
        s[0] = Command.CMD_OUTPUT_ID | Command.CMD_READ

        # 启用校验时追加 CRC / Append CRC when enabled
        if self._use_crc8:
            crc = self.calculate_crc8(s[:-1])
            s[-1] = crc
        else:
            s[-1] = Command.CMD_END

        self._read_ok_event.clear()
        self.send_command(s)
        if not self._read_ok_event.wait(timeout=1.0):  # 等待回应，超时 1 秒 / Wait up to 1s for the reply
            print("等待响应超时 Timeout waiting for response")
            return None
        return self._read_result

    def output_id_set(self, id):
        """
        选择输出组
        Select the output group
        """
        s = bytearray(3)
        s[0] = Command.CMD_OUTPUT_ID
        s[1] = id

        # 启用校验时追加 CRC / Append CRC when enabled
        if self._use_crc8:
            crc = self.calculate_crc8(s[:-1])
            s[-1] = crc
        else:
            s[-1] = Command.CMD_END

        self._write_ok_event.clear()
        self.send_command(s)
        if not self._write_ok_event.wait(timeout=1.0):  # 等待回应，超时 1 秒 / Wait up to 1s for the reply
            print("等待响应超时 Timeout waiting for response")

    def output_data_set(self, id, voltage, current):
        """
        设置指定组的电压与电流
        Set voltage and current of a group
        """
        s = bytearray(7)
        s[0] = Command.CMD_OUTPUT_DATA
        s[1] = id
        s[2] = int(voltage & 0xFF)
        s[3] = int(voltage >> 8)
        s[4] = int(current & 0xFF)
        s[5] = int(current >> 8)

        # 启用校验时追加 CRC / Append CRC when enabled
        if self._use_crc8:
            crc = self.calculate_crc8(s[:-1])
            s[-1] = crc
        else:
            s[-1] = Command.CMD_END

        self._write_ok_event.clear()
        self.send_command(s)
        if not self._write_ok_event.wait(timeout=1.0):  # 等待回应，超时 1 秒 / Wait up to 1s for the reply
            print("等待响应超时 Timeout waiting for response")

    def output_data_get(self, id):
        """
        读取指定组的电压与电流
        Read voltage and current of a group
        """
        s = bytearray(3)
        s[0] = Command.CMD_OUTPUT_DATA | Command.CMD_READ
        s[1] = id

        # 启用校验时追加 CRC / Append CRC when enabled
        if self._use_crc8:
            crc = self.calculate_crc8(s[:-1])
            s[-1] = crc
        else:
            s[-1] = Command.CMD_END

        self._read_ok_event.clear()
        self.send_command(s)
        if not self._read_ok_event.wait(timeout=1.0):  # 等待回应，超时 1 秒 / Wait up to 1s for the reply
            print("等待响应超时 Timeout waiting for response")
            return None
        voltage = self._read_result[1] | (self._read_result[2] << 8)
        current = self._read_result[3] | (self._read_result[4] << 8)
        return voltage, current

    def output_display_get(self):
        """
        读取实时输出电压与电流
        Read live output voltage and current
        """
        s = bytearray(2)
        s[0] = Command.CMD_OUTPUT_DISPLAY | Command.CMD_READ

        # 启用校验时追加 CRC / Append CRC when enabled
        if self._use_crc8:
            crc = self.calculate_crc8(s[:-1])
            s[-1] = crc
        else:
            s[-1] = Command.CMD_END

        self._read_ok_event.clear()
        self.send_command(s)
        if not self._read_ok_event.wait(timeout=1.0):  # 等待回应，超时 1 秒 / Wait up to 1s for the reply
            print("等待响应超时 Timeout waiting for response")
            return None
        voltage = self._read_result[0] | (self._read_result[1] << 8)
        current = self._read_result[2] | (self._read_result[3] << 8)
        return voltage, current

    def input_state_get(self):
        """
        读取输入电源状态与输入电压
        Read input power state and voltage
        """
        s = bytearray(2)
        s[0] = Command.CMD_INPUT_STATE | Command.CMD_READ

        # 启用校验时追加 CRC / Append CRC when enabled
        if self._use_crc8:
            crc = self.calculate_crc8(s[:-1])
            s[-1] = crc
        else:
            s[-1] = Command.CMD_END

        self._read_ok_event.clear()
        self.send_command(s)
        if not self._read_ok_event.wait(timeout=1.0):  # 等待回应，超时 1 秒 / Wait up to 1s for the reply
            print("等待响应超时 Timeout waiting for response")
            return None
        state = InputState(self._read_result[0])
        voltage = self._read_result[1] | (self._read_result[2] << 8)
        pd_voltage = self._read_result[3] | (self._read_result[4] << 8)
        return state, voltage/1000, pd_voltage/10
    
    def input_pd_voltage_set(self, voltage):
        """
        设置 PD 请求电压
        Set the PD requested voltage

        unit: V
        """
        voltage = round(voltage * 10)
        voltage = int(voltage)
        s = bytearray(4)
        s[0] = Command.CMD_INPUT_STATE
        s[1] = int(voltage & 0xFF)
        s[2] = int(voltage >> 8)

        # 启用校验时追加 CRC / Append CRC when enabled
        if self._use_crc8:
            crc = self.calculate_crc8(s[:-1])
            s[-1] = crc
        else:
            s[-1] = Command.CMD_END

        self._write_ok_event.clear()
        self.send_command(s)
        if not self._write_ok_event.wait(timeout=1.0):  # 等待回应，超时 1 秒 / Wait up to 1s for the reply
            print("等待响应超时 Timeout waiting for response")
    
    def output_offset_enable(self, enable=True):
        """
        设置校准使能
        Enable or disable calibration
        """
        s = bytearray(3)
        s[0] = Command.CMD_OUTPUT_OFFSET_EN
        s[1] = 0x01 if enable else 0x00
        # 启用校验时追加 CRC / Append CRC when enabled
        if self._use_crc8:
            crc = self.calculate_crc8(s[:-1])
            s[-1] = crc
        else:
            s[-1] = Command.CMD_END

        self._write_ok_event.clear()
        self.send_command(s)
        if not self._write_ok_event.wait(timeout=1.0):  # 等待回应，超时 1 秒 / Wait up to 1s for the reply
            print("等待响应超时 Timeout waiting for response")

    def factory_data_get(self):
        """
        读取出厂校准数据
        Read factory calibration data
        """
        s = bytearray(2)
        s[0] = Command.CMD_SYSTEM_FACTORY_DATA | Command.CMD_READ

        # 启用校验时追加 CRC / Append CRC when enabled
        if self._use_crc8:
            crc = self.calculate_crc8(s[:-1])
            s[-1] = crc
        else:
            s[-1] = Command.CMD_END

        self._read_ok_event.clear()
        self.send_command(s)
        if not self._read_ok_event.wait(timeout=1.0):  # 等待回应，超时 1 秒 / Wait up to 1s for the reply
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
        """
        设置过压放电使能
        Enable or disable over-voltage discharge
        """
        s = bytearray(3)
        s[0] = Command.CMD_OUTPUT_DISCHARGE_EN
        s[1] = 0x01 if enable else 0x00

        # 启用校验时追加 CRC / Append CRC when enabled
        if self._use_crc8:
            crc = self.calculate_crc8(s[:-1])
            s[-1] = crc
        else:
            s[-1] = Command.CMD_END

        self._write_ok_event.clear()
        self.send_command(s)
        if not self._write_ok_event.wait(timeout=1.0):  # 等待回应，超时 1 秒 / Wait up to 1s for the reply
            print("等待响应超时 Timeout waiting for response")

    def factory_reset(self):
        """
        恢复出厂设置
        Restore factory defaults
        """
        s = bytearray(2)
        s[0] = Command.CMD_SYSTEM_FACTORY_RESET

        # 启用校验时追加 CRC / Append CRC when enabled
        if self._use_crc8:
            crc = self.calculate_crc8(s[:-1])
            s[-1] = crc
        else:
            s[-1] = Command.CMD_END

        self._write_ok_event.clear()
        self.send_command(s)
        if not self._write_ok_event.wait(timeout=1.0):  # 等待回应，超时 1 秒 / Wait up to 1s for the reply
            print("等待响应超时 Timeout waiting for response")

    # ---- 以下命令自固件 V1.3.0.0 起支持 ----
    # ---- Commands below require firmware V1.3.0.0 or later ----

    def _setting_set(self, cmd, value):
        """
        写入单个设置项
        Write a single setting item
        """
        s = bytearray(3)
        s[0] = cmd
        s[1] = int(value) & 0xFF
        if self._use_crc8:
            s[-1] = self.calculate_crc8(s[:-1])
        else:
            s[-1] = Command.CMD_END

        self._write_ok_event.clear()
        self.send_command(s)
        if not self._write_ok_event.wait(timeout=1.0):  # 等待回应，超时 1 秒 / Wait up to 1s for the reply
            print("等待响应超时 Timeout waiting for response")

    def _setting_get(self, cmd):
        """
        读取单个设置项
        Read a single setting item
        """
        s = bytearray(2)
        s[0] = cmd | Command.CMD_READ
        if self._use_crc8:
            s[-1] = self.calculate_crc8(s[:-1])
        else:
            s[-1] = Command.CMD_END

        self._read_ok_event.clear()
        self.send_command(s)
        if not self._read_ok_event.wait(timeout=1.0):  # 等待回应，超时 1 秒 / Wait up to 1s for the reply
            print("等待响应超时 Timeout waiting for response")
            return None
        # _read_result 已去掉命令码与帧尾，首字节即数据
        return self._read_result[0]

    def output_oc_discharge_enable(self, enable=True):
        """
        设置过流放电使能
        Enable or disable over-current discharge
        """
        self._setting_set(Command.CMD_OUTPUT_OC_DISCHARGE_EN, 1 if enable else 0)

    def output_oc_discharge_get(self):
        """
        读取过流放电使能
        Read over-current discharge enable
        """
        return self._setting_get(Command.CMD_OUTPUT_OC_DISCHARGE_EN)

    def output_auto_enable(self, enable=True):
        """
        设置上电自动输出使能
        Enable or disable auto output on power-up
        """
        self._setting_set(Command.CMD_OUTPUT_AUTO_EN, 1 if enable else 0)

    def output_auto_get(self):
        """
        读取上电自动输出使能
        Read auto output enable
        """
        return self._setting_get(Command.CMD_OUTPUT_AUTO_EN)

    def filter_window_size_set(self, size):
        """
        设置采样滤波窗口大小
        Set the moving filter window size
        """
        # The frame carries one byte, so an out-of-range value would be truncated
        # silently (300 becomes 44) instead of being rejected by the device.
        # 一字节字段，越界值会被静默截断（300 变成 44），设备不会拒绝
        if not 1 <= int(size) <= 200:
            print(f"滤波窗口超出范围，已忽略 filter window out of range, ignored: {size}")
            return
        self._setting_set(Command.CMD_FILTER_WINDOW_SIZE, size)

    def filter_window_size_get(self):
        """
        读取采样滤波窗口大小
        Read the moving filter window size
        """
        return self._setting_get(Command.CMD_FILTER_WINDOW_SIZE)

    def output_comp_enable(self, enable=True):
        """
        设置输出误差补偿使能
        Enable or disable output error compensation
        """
        self._setting_set(Command.CMD_OUTPUT_COMP_EN, 1 if enable else 0)

    def output_comp_get(self):
        """
        读取输出误差补偿使能
        Read output error compensation enable
        """
        return self._setting_get(Command.CMD_OUTPUT_COMP_EN)

    def timer_set(self, mode, enable_time, off_time):
        """
        设置定时开关
        mode: 0 关闭, 1 启用（设备菜单对应项：Timer En:）
        enable_time: 开启时间，秒，0-59999；0 表示不输出
        off_time: 关闭时间，秒；0 表示开完即结束（定时关闭），非 0 则开-关循环
        """
        # Both times travel as 16-bit fields, so an out-of-range value would be
        # truncated silently (70000 becomes 4464) instead of being rejected.
        # 两个时间都是 16 位字段，越界值会被静默截断
        # （70000 变成 4464），设备不会拒绝
        if not 0 <= int(mode) <= 1:
            print(f"定时模式超出范围，已忽略 timer mode out of range, ignored: {mode}")
            return
        if not 0 <= int(enable_time) <= 59999 or not 0 <= int(off_time) <= 59999:
            print(f"定时时间超出范围，已忽略 timer time out of range, ignored: {enable_time}, {off_time}")
            return

        s = bytearray(7)
        s[0] = Command.CMD_TIMER_SET
        s[1] = int(mode) & 0xFF
        s[2] = int(enable_time) & 0xFF
        s[3] = (int(enable_time) >> 8) & 0xFF
        s[4] = int(off_time) & 0xFF
        s[5] = (int(off_time) >> 8) & 0xFF
        s[6] = Command.CMD_END
        if self._use_crc8:
            s[-1] = self.calculate_crc8(s[:-1])

        self._write_ok_event.clear()
        self.send_command(s)
        if not self._write_ok_event.wait(timeout=1.0):  # 等待回应，超时 1 秒 / Wait up to 1s for the reply
            print("等待响应超时 Timeout waiting for response")

    def timer_get(self):
        """
        返回 (mode, enable_time, off_time)
        Returns (mode, enable_time, off_time)

        mode 为定时功能是否启用，两个时间的单位为秒
        mode: timer enabled or not, both times in seconds
        """
        s = bytearray(2)
        s[0] = Command.CMD_TIMER_SET | Command.CMD_READ
        if self._use_crc8:
            s[-1] = self.calculate_crc8(s[:-1])
        else:
            s[-1] = Command.CMD_END

        self._read_ok_event.clear()
        self.send_command(s)
        if not self._read_ok_event.wait(timeout=1.0):  # 等待回应，超时 1 秒 / Wait up to 1s for the reply
            print("等待响应超时 Timeout waiting for response")
            return None
        r = self._read_result
        return r[0], r[1] | (r[2] << 8), r[3] | (r[4] << 8)

    # 各型号支持定时开关与新增设置项的最低版本
    # Minimum firmware version per model for the timer and the new settings
    MIN_VERSION = {
        "V1": (1, 0, 3, 0),
        "V1.1": (1, 0, 2, 0),
    }

    def model(self):
        """按 WHO_AM_I 区分型号；V1.1 必须先判断，否则会被匹配成 V1"""
        if self._model is None:
            info = self.who_am_i() or ""
            if "V1.1" in info:
                self._model = "V1.1"
            elif "V1" in info:
                self._model = "V1"
            else:
                self._model = ""
        return self._model or None

    def version_tuple(self):
        """解析 "V1.0.3.0_20261116"；固件头为擦除态时返回 None"""
        text = (self.system_version() or "").strip()
        if not text.startswith("V"):
            return None
        head = text[1:].split("_")[0]
        try:
            parts = [int(x) for x in head.split(".")]
        except ValueError:
            return None
        if len(parts) != 4 or parts[0] == 255:
            return None
        return tuple(parts)

    def supports_timer(self):
        """
        判断固件版本是否支持定时开关与新增设置项
        Check whether the firmware version supports timer commands
        """
        version = self.version_tuple()
        need = self.MIN_VERSION.get(self.model())
        if version is None or need is None:
            return False
        return version >= need

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
        """
        串口是否已打开
        Whether the serial port is open
        """
        return self.ser.is_open

    @staticmethod
    def calculate_crc8(data):
        """
        计算 CRC-8 校验值
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

    # PD 电压设置要求 V1 >= v1.0.2.0，V1.1 >= v1.0.1.0
    # input_pd_voltage_set requires V1 >= v1.0.2.0, V1.1 >= v1.0.1.0
    # print(f"input_state_get: {c.input_state_get()}")
    # print(f"input_pd_voltage_set: 10V")
    # c.input_pd_voltage_set(10)

    print(f"output_state: {c.output_state()}")

    # 定时开关与新增设置项的最低版本：V1 为 v1.0.3.0，V1.1 为 v1.0.2.0
    # Minimum version for timer and new settings: V1 v1.0.3.0, V1.1 v1.0.2.0
    print(f"model: {c.model()}")
    if c.supports_timer():
        print("timer_set: mode=1, enable=5s, off=0s (one shot)")
        c.timer_set(1, 5, 0)
        print(f"timer_get: {c.timer_get()}")
    else:
        print("固件版本低于该型号的最低要求（V1: v1.0.3.0, V1.1: v1.0.2.0），跳过定时开关相关命令")
        print("Firmware is older than the minimum for this model, skip timer commands")

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
    print(f"factory_reset: {True}")
    c.factory_reset()
    c.close()
    time.sleep(1)
