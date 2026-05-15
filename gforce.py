# !/usr/bin/python
# -*- coding:utf-8 -*-
#
# gforce.py — gForcePRO SDK (수정판)
#
# 수정 내역:
#   [BUG FIX] connectByRssi: print 포맷 "{1}" → "{0}" 수정
#   [BUG FIX] scan(): time.sleep() → await asyncio.sleep() 로 교체
#              (asyncio 이벤트 루프 블로킹이 BLE 연결 불안정의 주원인)
#   [BUG FIX] disconnect(): 상태 비교 "==" → 대입 "=" 수정
#   [NOTE]    SDK에는 소프트웨어 필터(Notch, BPF) 없음. 순수 raw ADC값 전달.
#             필터가 필요하면 collect_emg.py의 EmgFilter 클래스 사용 권장.
#   [NOTE]    데이터는 1kHz 개별 송수신이 아닌 패킷 단위 수신임.
#             패킷 1개 = 헤더(1B) + EMG 128B = 8채널 × 16샘플 묶음.
#             BLE 특성상 개별 샘플 단위 전송은 불가능.
#   [NOTE]    HxD로 bin 파일 열었을 때 우측에 보이는 알파벳 열은
#             hex 바이트를 ASCII로 강제 변환해서 표시하는 것으로,
#             HxD 프로그램의 디스플레이 방식이며 실제 데이터와 무관함.
#             (예: 0x77 = 119 → 'w', EMG 안정 구간 값이 119 근처에 몰려있어서 w가 많이 보임)

import struct
import threading
import time
from datetime import datetime, timedelta

import asyncio
from bleak import BleakClient, BleakScanner, BleakGATTCharacteristic


class GF_RET_CODE(int):
    GF_SUCCESS = 0
    GF_ERROR = 1
    GF_ERROR_BAD_PARAM = 2
    GF_ERROR_BAD_STATE = 3
    GF_ERROR_NOT_SUPPORT = 4
    GF_ERROR_SCAN_BUSY = 5
    GF_ERROR_NO_RESOURCE = 6
    GF_ERROR_TIMEOUT = 7
    GF_ERROR_DEVICE_BUSY = 7
    GF_ERROR_NOT_READY = 9


class CommandType(int):
    CMD_GET_PROTOCOL_VERSION = 0x00
    CMD_GET_FEATURE_MAP = 0x01
    CMD_GET_DEVICE_NAME = 0x02
    CMD_GET_MODEL_NUMBER = 0x03
    CMD_GET_SERIAL_NUMBER = 0x04
    CMD_GET_HW_REVISION = 0x05
    CMD_GET_FW_REVISION = 0x06
    CMD_GET_MANUFACTURER_NAME = 0x07
    CMD_GET_BOOTLOADER_VERSION = 0x0A
    CMD_GET_BATTERY_LEVEL = 0x08
    CMD_GET_TEMPERATURE = 0x09
    CMD_POWEROFF = 0x1D
    CMD_SWITCH_TO_OAD = 0x1E
    CMD_SYSTEM_RESET = 0x1F
    CMD_SWITCH_SERVICE = 0x20
    CMD_SET_LOG_LEVEL = 0x21
    CMD_SET_LOG_MODULE = 0x22
    CMD_PRINT_KERNEL_MSG = 0x23
    CMD_MOTOR_CONTROL = 0x24
    CMD_LED_CONTROL_TEST = 0x25
    CMD_PACKAGE_ID_CONTROL = 0x26
    CMD_SEND_TRAINING_PACKAGE = 0x27
    CMD_GET_ACCELERATE_CAP = 0x30
    CMD_SET_ACCELERATE_CONFIG = 0x31
    CMD_GET_GYROSCOPE_CAP = 0x32
    CMD_SET_GYROSCOPE_CONFIG = 0x33
    CMD_GET_MAGNETOMETER_CAP = 0x34
    CMD_SET_MAGNETOMETER_CONFIG = 0x35
    CMD_GET_EULER_ANGLE_CAP = 0x36
    CMD_SET_EULER_ANGLE_CONFIG = 0x37
    CMD_GET_QUATERNION_CAP = 0x38
    CMD_SET_QUATERNION_CONFIG = 0x39
    CMD_GET_ROTATION_MATRIX_CAP = 0x3A
    CMD_SET_ROTATION_MATRIX_CONFIG = 0x3B
    CMD_GET_GESTURE_CAP = 0x3C
    CMD_SET_GESTURE_CONFIG = 0x3D
    CMD_GET_EMG_RAWDATA_CAP = 0x3E
    CMD_SET_EMG_RAWDATA_CONFIG = 0x3F
    CMD_GET_MOUSE_DATA_CAP = 0x40
    CMD_SET_MOUSE_DATA_CONFIG = 0x41
    CMD_GET_JOYSTICK_DATA_CAP = 0x42
    CMD_SET_JOYSTICK_DATA_CONFIG = 0x43
    CMD_GET_DEVICE_STATUS_CAP = 0x44
    CMD_SET_DEVICE_STATUS_CONFIG = 0x45
    CMD_GET_EMG_RAWDATA_CONFIG = 0x46
    CMD_SET_DATA_NOTIF_SWITCH = 0x4F
    CMD_PARTIAL_DATA = 0xFF


class ResponseResult(int):
    RSP_CODE_SUCCESS = 0x00
    RSP_CODE_NOT_SUPPORT = 0x01
    RSP_CODE_BAD_PARAM = 0x02
    RSP_CODE_FAILED = 0x03
    RSP_CODE_TIMEOUT = 0x04
    RSP_CODE_PARTIAL_PACKET = 0xFF


class DataNotifFlags(int):
    DNF_OFF = 0x00000000
    DNF_ACCELERATE = 0x00000001
    DNF_GYROSCOPE = 0x00000002
    DNF_MAGNETOMETER = 0x00000004
    DNF_EULERANGLE = 0x00000008
    DNF_QUATERNION = 0x00000010
    DNF_ROTATIONMATRIX = 0x00000020
    DNF_EMG_GESTURE = 0x00000040
    DNF_EMG_RAW = 0x00000080
    DNF_HID_MOUSE = 0x00000100
    DNF_HID_JOYSTICK = 0x00000200
    DNF_DEVICE_STATUS = 0x00000400
    DNF_LOG = 0x00000800
    DNF_EMG_GESTURE_STRENGTH = 0x00001000
    DNF_ALL = 0xFFFFFFFF


class ProfileCharType(int):
    PROF_SIMPLE_DATA = 0
    PROF_DATA_CMD = 1
    PROF_DATA_NTF = 2
    PROF_OAD_IDENTIFY = 3
    PROF_OAD_BLOCK = 4
    PROF_OAD_FAST = 5


class NotifDataType(int):
    NTF_ACC_DATA = 0x01
    NTF_GYO_DATA = 0x02
    NTF_MAG_DATA = 0x03
    NTF_EULER_DATA = 0x04
    NTF_QUAT_FLOAT_DATA = 0x05
    NTF_ROTA_DATA = 0x06
    NTF_EMG_GEST_DATA = 0x07
    NTF_EMG_ADC_DATA = 0x08
    NTF_HID_MOUSE = 0x09
    NTF_HID_JOYSTICK = 0x0A
    NTF_DEV_STATUS = 0x0B
    NTF_LOG_DATA = 0x0C
    NTF_PARTIAL_DATA = 0xFF


class LogLevel(int):
    LOG_LEVEL_DEBUG = 0x00
    LOG_LEVEL_INFO = 0x01
    LOG_LEVEL_WARN = 0x02
    LOG_LEVEL_ERROR = 0x03
    LOG_LEVEL_FATAL = 0x04
    LOG_LEVEL_NONE = 0x05


class BluetoothDeviceState(int):
    disconnected = 0
    connected = 1


SERVICE_GUID = "0000ffd0-0000-1000-8000-00805f9b34fb"
CMD_NOTIFY_CHAR_UUID = "f000ffe1-0451-4000-b000-000000000000"
DATA_NOTIFY_CHAR_UUID = "f000ffe2-0451-4000-b000-000000000000"


class CommandCallbackTableEntry:
    def __init__(self, _cmd, _timeoutTime, _cb):
        self._cmd = _cmd
        self._timeoutTime = _timeoutTime
        self._cb = _cb


class GForceProfile:
    def __init__(self):
        self.device = None
        self.state = BluetoothDeviceState.disconnected
        self.cmdCharacteristic = None
        self.notifyCharacteristic = None
        self.timer = None
        self.cmdMap = {}
        self.mtu = None
        self.cmdForTimeout = -1
        self.incompleteCmdRespPacket = []
        self.lastIncompleteCmdRespPacketId = 0
        self.incompleteNotifPacket = []
        self.lastIncompleteNotifPacketId = 0
        self.onData = None
        self.lock = threading.Lock()

    def handle_disconnect(_: BleakClient):
        for task in asyncio.all_tasks():
            task.cancel()

    async def connect(self, addr):
        self.device = BleakClient(addr, disconnected_callback=self.handle_disconnect)
        await self.device.connect()
        print("connection succeeded.")
        self.mtu = self.device.mtu_size
        print(f"mtu: {self.mtu}")
        self.state = BluetoothDeviceState.connected
        self.cmdCharacteristic = CMD_NOTIFY_CHAR_UUID
        self.notifyCharacteristic = DATA_NOTIFY_CHAR_UUID
        await self.device.start_notify(self.cmdCharacteristic, self._onResponse)

    # [BUG FIX] time.sleep → await asyncio.sleep (이벤트루프 블로킹 방지 → BLE 연결 안정화)
    # [BUG FIX] print 포맷 "{1}" → "{0}" 수정
    async def connectByRssi(self, timeout, name_prefix="", min_rssi=-128):
        scan_result = await self.scan(timeout, name_prefix, min_rssi)
        rssi_devices = {}
        for dev in scan_result:
            rssi_devices[dev["rssi"]] = dev["address"]
        rssi = rssi_devices.keys()
        dev_addr = rssi_devices[max(rssi)]
        self.device = BleakClient(dev_addr, disconnected_callback=self.handle_disconnect)
        await self.device.connect()
        print("connection succeeded")
        self.mtu = self.device.mtu_size
        print("mtu: {0}".format(self.mtu))  # [FIX] {1} → {0}
        self.state = BluetoothDeviceState.connected
        self.cmdCharacteristic = CMD_NOTIFY_CHAR_UUID
        self.notifyCharacteristic = DATA_NOTIFY_CHAR_UUID
        await self.device.start_notify(self.cmdCharacteristic, self._onResponse)

    # [BUG FIX] time.sleep(timeout) → await asyncio.sleep(timeout)
    # 원래 코드의 time.sleep()은 asyncio 이벤트 루프 전체를 블로킹해서
    # BLE 스캔 중 다른 비동기 작업이 멈추고 연결이 불안정해지는 원인이었음.
    # PC에서 폰보다 연결이 오래 걸리거나 실패하는 주요 원인 중 하나.
    async def scan(self, timeout, name_prefix="", min_rssi=-128):
        scanner = BleakScanner(service_uuids=[SERVICE_GUID])
        await scanner.start()
        await asyncio.sleep(timeout)  # [FIX] time.sleep → await asyncio.sleep
        await scanner.stop()

        scan_result = []
        i = 1
        for v in scanner.discovered_devices_and_advertisement_data.values():
            dev = v[0]
            advData = v[1]
            if (dev.name is not None) and dev.name.startswith(name_prefix) and (advData.rssi >= min_rssi):
                print("Filtered device %s (%s), RSSI=%d dB" % (dev.address, dev.name, advData.rssi))
                scan_result.append({"index": i, "name": dev.name, "address": dev.address, "rssi": advData.rssi})
                i += 1
        return scan_result

    # [BUG FIX] "self.state == BluetoothDeviceState.disconnected" → 대입(=)으로 수정
    async def disconnect(self):
        if self.timer != None:
            self.timer.cancel()
        self.timer = None
        if self.state == BluetoothDeviceState.disconnected:
            return True
        else:
            await self.device.disconnect()
            self.state = BluetoothDeviceState.disconnected  # [FIX] == → =

    async def setDataNotifSwitch(self, flags, cb, timeout):
        data = []
        data.append(CommandType.CMD_SET_DATA_NOTIF_SWITCH)
        data.append(0xFF & (flags))
        data.append(0xFF & (flags >> 8))
        data.append(0xFF & (flags >> 16))
        data.append(0xFF & (flags >> 24))
        data = bytes(data)

        def temp(resp, respData):
            if cb != None:
                cb(resp)

        return await self.sendCommand(ProfileCharType.PROF_DATA_CMD, data, True, temp, timeout)

    async def powerOff(self, timeout):
        data = bytes([CommandType.CMD_POWEROFF])
        def temp(resp, respData): pass
        return await self.sendCommand(ProfileCharType.PROF_DATA_CMD, data, True, temp, timeout)

    async def systemReset(self, timeout):
        data = bytes([CommandType.CMD_SYSTEM_RESET])
        def temp(resp, respData): pass
        return await self.sendCommand(ProfileCharType.PROF_DATA_CMD, data, True, temp, timeout)

    async def setMotor(self, switchStatus, cb, timeout):
        data = bytes([CommandType.CMD_MOTOR_CONTROL, 0x01 if switchStatus else 0x00])
        def temp(resp, respData):
            if cb != None: cb(resp)
        return await self.sendCommand(ProfileCharType.PROF_DATA_CMD, data, True, temp, timeout)

    async def setLED(self, switchStatus, cb, timeout):
        data = bytes([CommandType.CMD_LED_CONTROL_TEST, 0x01 if switchStatus else 0x00])
        def temp(resp, respData):
            if cb != None: cb(resp)
        return await self.sendCommand(ProfileCharType.PROF_DATA_CMD, data, True, temp, timeout)

    async def setLogLevel(self, logLevel, cb, timeout):
        data = bytes([CommandType.CMD_SET_LOG_LEVEL, 0xFF & logLevel])
        def temp(resp, respData):
            if cb != None: cb(resp)
        return await self.sendCommand(ProfileCharType.PROF_DATA_CMD, data, True, temp, timeout)

    # EMG 설정 커맨드
    # [NOTE] sampRate=1000, resolution=12 조합은 SDK 레벨에서는 막지 않음.
    #        센서 펌웨어가 수락할지는 응답코드(RSP_CODE_SUCCESS / RSP_CODE_BAD_PARAM)로 확인.
    #        OYMotion 공식 입장: 1000Hz는 8bit 전용, 12bit는 500Hz까지만 (BLE 대역폭 한계).
    #        16bit는 SDK/펌웨어 모두 지원하지 않음 (resolution 파라미터: 8 또는 12만 유효).
    async def setEmgRawDataConfig(self, sampRate, channelMask, dataLen, resolution, cb, timeout):
        data = b""
        data += struct.pack("<B", CommandType.CMD_SET_EMG_RAWDATA_CONFIG)
        data += struct.pack("<H", sampRate)
        data += struct.pack("<H", channelMask)
        data += struct.pack("<B", dataLen)
        data += struct.pack("<B", resolution)

        def temp(resp, raspData):
            if cb != None:
                cb(resp)

        return await self.sendCommand(ProfileCharType.PROF_DATA_CMD, data, True, temp, timeout)

    async def getEmgRawDataConfig(self, cb, timeout):
        data = bytes([CommandType.CMD_GET_EMG_RAWDATA_CONFIG])

        def temp(resp, respData):
            if cb != None:
                if resp != ResponseResult.RSP_CODE_SUCCESS:
                    cb(resp, None, None, None, None)
                elif len(respData) == 6:
                    sampRate, channelMask, dataLen, resolution = struct.unpack_from("@HHBB", respData)
                cb(resp, sampRate, channelMask, dataLen, resolution)

        return await self.sendCommand(ProfileCharType.PROF_DATA_CMD, data, True, temp, timeout)

    async def getFeatureMap(self, cb, timeout):
        data = bytes([CommandType.CMD_GET_FEATURE_MAP])

        def temp(resp, respData):
            if cb != None:
                if resp != ResponseResult.RSP_CODE_SUCCESS:
                    cb(resp, None)
                elif len(respData) == 4:
                    featureMap = struct.unpack("@I", respData)[0]
                    cb(resp, featureMap)

        return await self.sendCommand(ProfileCharType.PROF_DATA_CMD, data, True, temp, timeout)

    async def getControllerFirmwareVersion(self, cb, timeout):
        data = bytes([CommandType.CMD_GET_FW_REVISION])

        def temp(resp, respData):
            if cb != None:
                if resp != ResponseResult.RSP_CODE_SUCCESS:
                    cb(resp, None)
                else:
                    if len(respData) > 4:
                        firmwareVersion = respData.decode("ascii")
                    else:
                        firmwareVersion = ""
                        for i in respData:
                            firmwareVersion += str(i) + "."
                    cb(resp, firmwareVersion)

        return await self.sendCommand(ProfileCharType.PROF_DATA_CMD, data, True, temp, timeout)

    async def sendCommand(self, profileCharType, data, hasResponse, cb, timeout):
        if hasResponse and cb != None:
            cmd = data[0]
            self.lock.acquire()
            if cmd in self.cmdMap.keys():
                self.lock.release()
                return GF_RET_CODE.GF_ERROR_DEVICE_BUSY
            self.cmdMap[cmd] = CommandCallbackTableEntry(cmd, datetime.now() + timedelta(milliseconds=timeout), cb)
            self._refreshTimer()
            self.lock.release()

        if profileCharType == ProfileCharType.PROF_DATA_CMD:
            if self.cmdCharacteristic == None:
                return GF_RET_CODE.GF_ERROR_BAD_STATE
            else:
                if len(data) > self.mtu:
                    contentLen = self.mtu - 2
                    packetCount = (len(data) + contentLen - 1) // contentLen
                    startIndex = 0
                    buf = []
                    for i in range(packetCount - 1, 0, -1):
                        buf.append(CommandType.CMD_PARTIAL_DATA)
                        buf.append(i)
                        buf += data[startIndex: startIndex + contentLen]
                        startIndex += contentLen
                        await self.device.write_gatt_char(self.cmdCharacteristic, buf)
                        buf.clear()
                    buf.append(CommandType.CMD_PARTIAL_DATA)
                    buf.append(0)
                    buf += data[startIndex:]
                    await self.device.write_gatt_char(self.cmdCharacteristic, buf)
                else:
                    await self.device.write_gatt_char(self.cmdCharacteristic, data)
                return GF_RET_CODE.GF_SUCCESS
        else:
            return GF_RET_CODE.GF_ERROR_BAD_PARAM

    def _refreshTimer(self):
        def cmp_time(cb):
            return cb._timeoutTime

        if self.timer != None:
            self.timer.cancel()
        self.timer = None
        cmdlist = self.cmdMap.values()
        if len(cmdlist) > 0:
            cmdlist = sorted(cmdlist, key=cmp_time)

        timeoutTime = None
        listlen = len(cmdlist)

        for i in range(listlen):
            timeoutTime = cmdlist[0]._timeoutTime
            if timeoutTime > datetime.now():
                self.cmdForTimeout = cmdlist[0]._cmd
                ms = int((timeoutTime.timestamp() - datetime.now().timestamp()) * 1000)
                if ms <= 0:
                    ms = 1
                self.timer = threading.Timer(ms / 1000, self._onTimeOut)
                self.timer.start()
                break
            cmd = cmdlist.pop(0)
            if cmd._cb != None:
                cmd._cb(ResponseResult.RSP_CODE_TIMEOUT, None)

    async def startDataNotification(self, onData):
        self.onData = onData
        try:
            await self.device.start_notify(self.notifyCharacteristic, self._handleDataNotification)
            success = True
        except:
            success = False
        return GF_RET_CODE.GF_SUCCESS if success else GF_RET_CODE.GF_ERROR_BAD_STATE

    async def stopDataNotification(self):
        try:
            await self.device.stop_notify(self.notifyCharacteristic)
            success = True
        except:
            success = False
        return GF_RET_CODE.GF_SUCCESS if success else GF_RET_CODE.GF_ERROR_BAD_STATE

    def _handleDataNotification(self, characteristic: BleakGATTCharacteristic, data: bytearray):
        fullPacket = []
        if len(data) >= 2:
            if data[0] == NotifDataType.NTF_PARTIAL_DATA:
                if self.lastIncompleteNotifPacketId != 0 and self.lastIncompleteNotifPacketId != data[1] + 1:
                    print("Error:lastIncompleteNotifPacketId:{0},current packet id:{1}".format(
                        self.lastIncompleteNotifPacketId, data[1]))
                if self.lastIncompleteNotifPacketId == 0 or self.lastIncompleteNotifPacketId > data[1]:
                    self.lastIncompleteNotifPacketId = data[1]
                    self.incompleteNotifPacket += data[2:]
                    if self.lastIncompleteNotifPacketId == 0:
                        fullPacket = self.incompleteNotifPacket
                        self.incompleteNotifPacket = []
            else:
                fullPacket = data
        if len(fullPacket) > 0:
            self.onData(fullPacket)

    def _onResponse(self, characteristic, data):
        fullPacket = []
        if len(data) >= 2:
            if data[0] == ResponseResult.RSP_CODE_PARTIAL_PACKET:
                if self.lastIncompleteCmdRespPacketId != 0 and self.lastIncompleteCmdRespPacketId != data[1] + 1:
                    print("Error: _lastIncompletePacketId:{0}, current packet id:{1}".format(
                        self.lastIncompleteCmdRespPacketId, data[1]))
                if self.lastIncompleteCmdRespPacketId == 0 or self.lastIncompleteCmdRespPacketId > data[1]:
                    self.lastIncompleteCmdRespPacketId = data[1]
                    self.incompleteCmdRespPacket += data[2:]
                    if self.lastIncompleteCmdRespPacketId == 0:
                        fullPacket = self.incompleteCmdRespPacket
                        self.incompleteCmdRespPacket = []
            else:
                fullPacket = data
        if fullPacket != None and len(fullPacket) >= 2:
            resp = fullPacket[0]
            cmd = fullPacket[1]
            self.lock.acquire()
            if cmd > 0 and self.cmdMap.__contains__(cmd):
                cb = self.cmdMap[cmd]._cb
                del self.cmdMap[cmd]
                self._refreshTimer()
                if cb != None:
                    cb(resp, fullPacket[2:])
            self.lock.release()

    def _onTimeOut(self):
        cb = None
        self.lock.acquire()
        if self.cmdForTimeout > 0 and self.cmdMap.__contains__(self.cmdForTimeout):
            cb = self.cmdMap[self.cmdForTimeout]._cb
            del self.cmdMap[self.cmdForTimeout]
        self._refreshTimer()
        self.lock.release()
        if cb != None:
            cb(ResponseResult.RSP_CODE_TIMEOUT, None)
