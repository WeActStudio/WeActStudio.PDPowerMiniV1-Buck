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
            Command.CMD_SYSTEM_FACTORY_DATA: 58,
            Command.CMD_BRIGHTNESS: 3,
            Command.CMD_SYSTEM_VERSION: 0,
            Command.CMD_SYSTEM_SERIAL_NUM: 0,
        }.get(self, 0)


class Com_PdPower:
    def __init__(self, port, baudrate=115200, timeout=1, queue_maxsize=100):
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

    def _read_serial(self):
        """
        线程函数，持续读取串口数据
        Thread function, continuously read serial port data
        """
        buffer = bytearray()
        while not self._stop_event.is_set():
            try:
                if self.ser.in_waiting:
                    buffer += self.ser.read(self.ser.in_waiting)
                    
                    while len(buffer) > 0:
                        if len(buffer) > 0 and (buffer[0] & 0x7F) in [
                            Command.CMD_WHO_AM_I, 
                            Command.CMD_SYSTEM_VERSION,
                            Command.CMD_SYSTEM_SERIAL_NUM
                        ]:
                            if Command.CMD_END in buffer:
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
                            if len(buffer) > 0:
                                cmd = buffer[0] & 0x7F
                                expected_len = Command(cmd).READ_LENGTH - 1
                                if expected_len == -1:
                                    break
                                if len(buffer) >= expected_len + 1:
                                    if buffer[expected_len] == Command.CMD_END:
                                        frame = buffer[:expected_len]
                                        buffer = buffer[expected_len+1:]
                                        cmd_name = Command(cmd).name
                                        self._read_result = frame[1:]
                                        self._read_ok_event.set()
                                    else:
                                        break
                                else:
                                    break
                            else:
                                break
                                
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
        s = bytearray(2)
        s[0] = Command.CMD_WHO_AM_I | Command.CMD_READ
        s[1] = Command.CMD_END
        self._read_ok_event.clear()
        self.send_command(s)
        if not self._read_ok_event.wait(timeout=1.0):  # 等待1秒超时
            print("等待响应超时 Timeout waiting for response")
            return None
        return self._read_result
    
    def system_version(self):
        s = bytearray(2)
        s[0] = Command.CMD_SYSTEM_VERSION | Command.CMD_READ
        s[1] = Command.CMD_END
        self._read_ok_event.clear()
        self.send_command(s)
        if not self._read_ok_event.wait(timeout=1.0):  # 等待1秒超时
            print("等待响应超时 Timeout waiting for response")
            return None
        return self._read_result
    
    def system_serial_num(self):
        s = bytearray(2)
        s[0] = Command.CMD_SYSTEM_SERIAL_NUM | Command.CMD_READ
        s[1] = Command.CMD_END
        self._read_ok_event.clear()
        self.send_command(s)
        if not self._read_ok_event.wait(timeout=1.0):  # 等待1秒超时
            print("等待响应超时 Timeout waiting for response")
            return None
        return self._read_result

    def output_enable(self, enable=True):
        s = bytearray(3)
        s[0] = Command.CMD_OUTPUT_EN_STATE
        s[1] = 0x01 if enable else 0x00
        s[2] = Command.CMD_END
        self._write_ok_event.clear()
        self.send_command(s)
        if not self._write_ok_event.wait(timeout=1.0):  # 等待1秒超时
            print("等待响应超时 Timeout waiting for response")

    def output_state(self):
        s = bytearray(2)
        s[0] = Command.CMD_OUTPUT_EN_STATE | Command.CMD_READ
        s[1] = Command.CMD_END
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
        s[1] = Command.CMD_END
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
        s[2] = Command.CMD_END
        self._write_ok_event.clear()
        self.send_command(s)
        if not self._write_ok_event.wait(timeout=1.0):  # 等待1秒超时
            print("等待响应超时 Timeout waiting for response")

    def output_data_set(self,id,voltage,current):
        s = bytearray(7)
        s[0] = Command.CMD_OUTPUT_DATA
        s[1] = id
        s[2] = int(voltage & 0xFF)
        s[3] = int(voltage >> 8)
        s[4] = int(current & 0xFF)
        s[5] = int(current >> 8)
        s[6] = Command.CMD_END
        self._write_ok_event.clear()
        self.send_command(s)
        if not self._write_ok_event.wait(timeout=1.0):  # 等待1秒超时
            print("等待响应超时 Timeout waiting for response")

    def output_data_get(self,id):
        s = bytearray(3)
        s[0] = Command.CMD_OUTPUT_DATA | Command.CMD_READ
        s[1] = id
        s[2] = Command.CMD_END
        self._read_ok_event.clear()
        self.send_command(s)
        if not self._read_ok_event.wait(timeout=1.0):  # 等待1秒超时
            print("等待响应超时 Timeout waiting for response")
            return None
        voltage = self._read_result[1] | (self._read_result[2] << 8)
        current = self._read_result[3] | (self._read_result[4] << 8)
        return voltage,current
    
    def output_display_get(self):
        s = bytearray(2)
        s[0] = Command.CMD_OUTPUT_DISPLAY | Command.CMD_READ
        s[1] = Command.CMD_END
        self._read_ok_event.clear()
        self.send_command(s)
        if not self._read_ok_event.wait(timeout=1.0):  # 等待1秒超时
            print("等待响应超时 Timeout waiting for response")
            return None
        voltage = self._read_result[0] | (self._read_result[1] << 8)
        current = self._read_result[2] | (self._read_result[3] << 8)
        return voltage,current
    
    def output_offset_enable(self,enable=True):
        s = bytearray(3)
        s[0] = Command.CMD_OUTPUT_OFFSET_EN
        s[1] = 0x01 if enable else 0x00
        s[2] = Command.CMD_END
        self._write_ok_event.clear()
        self.send_command(s)
        if not self._write_ok_event.wait(timeout=1.0):  # 等待1秒超时
            print("等待响应超时 Timeout waiting for response")
    
    def factory_data_set(self,dis_voltage_offset,set_voltage_offset,dis_current_offset,set_current_offset,current_0a_offset,lcd_panel_type):
        s = bytearray(58)
        s[0] = Command.CMD_SYSTEM_FACTORY_DATA

        s[1] = dis_voltage_offset[0] & 0xFF
        s[2] = dis_voltage_offset[0] >> 8
        s[3] = set_voltage_offset[0] & 0xFF
        s[4] = set_voltage_offset[0] >> 8

        s[5] = dis_voltage_offset[1] & 0xFF
        s[6] = dis_voltage_offset[1] >> 8
        s[7] = set_voltage_offset[1] & 0xFF
        s[8] = set_voltage_offset[1] >> 8

        s[9] = dis_voltage_offset[2] & 0xFF
        s[10] = dis_voltage_offset[2] >> 8
        s[11] = set_voltage_offset[2] & 0xFF
        s[12] = set_voltage_offset[2] >> 8

        s[13] = dis_voltage_offset[3] & 0xFF
        s[14] = dis_voltage_offset[3] >> 8
        s[15] = set_voltage_offset[3] & 0xFF
        s[16] = set_voltage_offset[3] >> 8

        s[17] = dis_voltage_offset[4] & 0xFF
        s[18] = dis_voltage_offset[4] >> 8
        s[19] = set_voltage_offset[4] & 0xFF
        s[20] = set_voltage_offset[4] >> 8

        s[21] = dis_voltage_offset[5] & 0xFF
        s[22] = dis_voltage_offset[5] >> 8
        s[23] = set_voltage_offset[5] & 0xFF
        s[24] = set_voltage_offset[5] >> 8

        s[25] = dis_current_offset[0] & 0xFF
        s[26] = dis_current_offset[0] >> 8
        s[27] = set_current_offset[0] & 0xFF
        s[28] = set_current_offset[0] >> 8

        s[29] = dis_current_offset[1] & 0xFF
        s[30] = dis_current_offset[1] >> 8
        s[31] = set_current_offset[1] & 0xFF
        s[32] = set_current_offset[1] >> 8

        s[33] = dis_current_offset[2] & 0xFF
        s[34] = dis_current_offset[2] >> 8
        s[35] = set_current_offset[2] & 0xFF
        s[36] = set_current_offset[2] >> 8

        s[37] = dis_current_offset[3] & 0xFF
        s[38] = dis_current_offset[3] >> 8
        s[39] = set_current_offset[3] & 0xFF
        s[40] = set_current_offset[3] >> 8

        s[41] = dis_current_offset[4] & 0xFF
        s[42] = dis_current_offset[4] >> 8
        s[43] = set_current_offset[4] & 0xFF
        s[44] = set_current_offset[4] >> 8

        s[45] = dis_current_offset[5] & 0xFF
        s[46] = dis_current_offset[5] >> 8
        s[47] = set_current_offset[5] & 0xFF
        s[48] = set_current_offset[5] >> 8

        s[49] = current_0a_offset[1] & 0xFF
        s[50] = current_0a_offset[1] >> 8
        s[51] = current_0a_offset[0] & 0xFF
        s[52] = current_0a_offset[0] >> 8

        s[53] = lcd_panel_type & 0xFF
        s[54] = lcd_panel_type >> 8

        s[55] = 0
        s[56] = 0

        s[57] = Command.CMD_END

        self._write_ok_event.clear()
        self.send_command(s)
        if not self._write_ok_event.wait(timeout=1.0):  # 等待1秒超时
            print("等待响应超时 Timeout waiting for response")

    def factory_data_get(self):
        s = bytearray(2)
        s[0] = Command.CMD_SYSTEM_FACTORY_DATA | Command.CMD_READ
        s[1] = Command.CMD_END
        self._read_ok_event.clear()
        self.send_command(s)
        if not self._read_ok_event.wait(timeout=1.0):  # 等待1秒超时
            print("等待响应超时 Timeout waiting for response")
            return None
        dis_voltage_offset = [self._read_result[0] | (self._read_result[1] << 8),
                             self._read_result[4] | (self._read_result[5] << 8),
                             self._read_result[8] | (self._read_result[9] << 8),
                             self._read_result[12] | (self._read_result[13] << 8),
                             self._read_result[16] | (self._read_result[17] << 8),
                             self._read_result[20] | (self._read_result[21] << 8)]
                            
        set_voltage_offset = [self._read_result[2] | (self._read_result[3] << 8),
                             self._read_result[6] | (self._read_result[7] << 8),
                             self._read_result[10] | (self._read_result[11] << 8),
                             self._read_result[14] | (self._read_result[15] << 8),
                             self._read_result[18] | (self._read_result[19] << 8),
                             self._read_result[22] | (self._read_result[23] << 8)]
        
        dis_current_offset = [self._read_result[24] | (self._read_result[25] << 8),
                             self._read_result[28] | (self._read_result[29] << 8),
                             self._read_result[32] | (self._read_result[33] << 8),
                             self._read_result[34] | (self._read_result[37] << 8),
                             self._read_result[40] | (self._read_result[41] << 8),
                             self._read_result[44] | (self._read_result[45] << 8)]
                            
        set_current_offset = [self._read_result[24] | (self._read_result[27] << 8),
                             self._read_result[30] | (self._read_result[31] << 8),
                             self._read_result[34] | (self._read_result[35] << 8),
                             self._read_result[38] | (self._read_result[39] << 8),
                             self._read_result[42] | (self._read_result[43] << 8),
                             self._read_result[46] | (self._read_result[47] << 8)]
        
        current_0a_offset = [self._read_result[50] | (self._read_result[51] << 8),
                             self._read_result[48] | (self._read_result[49] << 8)]      
        
        lcd_panel_type = self._read_result[52] | (self._read_result[53] << 8)
        return dis_voltage_offset,set_voltage_offset,dis_current_offset,set_current_offset,current_0a_offset,lcd_panel_type
    
    def output_discharge_enable(self,enable=True):
        s = bytearray(3)
        s[0] = Command.CMD_OUTPUT_DISCHARGE_EN
        s[1] = 0x01 if enable else 0x00
        s[2] = Command.CMD_END
        self._write_ok_event.clear()
        self.send_command(s)
        if not self._write_ok_event.wait(timeout=1.0):  # 等待1秒超时
            print("等待响应超时 Timeout waiting for response")
    
    def factory_reset(self):
        s = bytearray(2)
        s[0] = Command.CMD_SYSTEM_FACTORY_RESET
        s[1] = Command.CMD_END
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

if __name__ == "__main__":
    c = Com_PdPower("COM19")

    print(c.who_am_i())
    print(c.system_version())
    print(c.system_serial_num())

    print(c.output_state())

    c.output_id_set(0x00)
    print(c.output_id_get())

    c.output_data_set(0x00,3000,1000)
    print(c.output_data_get(0x00))

    c.output_enable(True)
    time.sleep(1)
    c.output_enable(False)

    c.output_offset_enable(True)
    time.sleep(1)
    c.output_offset_enable(False)

    c.factory_data_set([1001,3001,5001,10001,15001,19001],[1000,3000,5000,10000,15000,19000],[51,351,751,1501,2501,3001],[50,350,750,1500,2500,3000],[1,3],0)
    print(c.factory_data_get())
    print(c.output_display_get())
    c.factory_reset()
    c.close()
    time.sleep(1)
