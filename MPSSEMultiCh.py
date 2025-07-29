#!/usr/bin/env python
# -*- coding: utf-8 -*-

from ctypes import *
import time

class MPSSEMultiCh:
    ver = '21.9.23'
    # %% Return value error codes
    # % 	0	FT_OK,
    # % 	1	FT_INVALID_HANDLE,
    # % 	2	FT_DEVICE_NOT_FOUND,
    # % 	3	FT_DEVICE_NOT_OPENED,
    # % 	4	FT_IO_ERROR,
    # % 	5	FT_INSUFFICIENT_RESOURCES,
    # % 	6	FT_INVALID_PARAMETER,
    # % 	7	FT_INVALID_BAUD_RATE,	 
    # % 	8	FT_DEVICE_NOT_OPENED_FOR_ERASE,
    # % 	9	FT_DEVICE_NOT_OPENED_FOR_WRITE,
    # % 	10	FT_FAILED_TO_WRITE_DEVICE,
    # % 	11	FT_EEPROM_READ_FAILED,
    # % 	12	FT_EEPROM_WRITE_FAILED,
    # % 	13	FT_EEPROM_ERASE_FAILED,
    # % 	14	FT_EEPROM_NOT_PRESENT,
    # % 	15	FT_EEPROM_NOT_PROGRAMMED,
    # % 	16	FT_INVALID_ARGS,
    # % 	17	FT_NOT_SUPPORTED,
    # % 	18	FT_OTHER_ERROR

    def __init__(self, arg_filename):
        self.dll = cdll.LoadLibrary(arg_filename)
        ii = c_long()
        self.dll.SPI_GetNumChannels(byref(ii))
        self.channelNum = ii.value
        self.channelOpend = [False]*self.channelNum

        self.deviceList = []
        device = self.FT_DEVICE_LIST_INFO_NODE()
        for i in range(self.channelNum):
            self.dll.SPI_GetChannelInfo( i, byref(device) )
            # print(type(device.SerialNumber))
            self.deviceList.append(self.DeviceInfo())
            self.deviceList[i].setup(device)
            # print(self.deviceList[i].Description)

        self.pChannelHandle = [c_void_p()]*self.channelNum

    def __del__(self):
        for i in range(self.channelNum):
            self.closeChannel(i)

    def showDevices(self):
        for i in range(self.channelNum):
            print('Num:%d, LocID:%d, TYPE:%s, Serial:%s'%(
                i,
                self.deviceList[i].LocID,
                self.deviceList[i].Description,
                self.deviceList[i].SerialNumber) )
    def openChannelFromSerial(self, ser, clock=2e6, latency=2, mode=0):
        for i in range(self.channelNum):
            if self.deviceList[i].SerialNumber == ser:
                return self.openChannel(i, clock, latency, mode)
        # print('Device was not found')
        return -1
    def openChannel(self, num, clock=2e6, latency=2, mode=0):
        num = int(num)
        if num > self.channelNum - 1:
            print('Channel Num should be less than %d.'%(self.channelNum))
            quit()
        # # # if not self.deviceList[num].Description=='FT232H':
        # # #      # print('The device is not FT232H.')
        # # #     #quit()
        self.dll.SPI_OpenChannel(num, byref(self.pChannelHandle[num]))
        config = self.ChannelConfig()
        config.ClockRate = c_uint32(int(clock))
        config.LatencyTimer = c_uint8(int(latency))
        config.configOptions = c_uint32(int(mode))
        self.dll.SPI_InitChannel(self.pChannelHandle[num], config)
        self.channelOpend[num] = True

        self.dll.FT_WriteGPIO(self.pChannelHandle[num], c_uint8(255), c_uint8(255))    # C0 to C7-High
        return num
        
    def closeChannel(self, ch):
        if self.channelOpend[ch]:
            self.dll.SPI_CloseChannel(self.pChannelHandle[ch])
            self.channelOpend[ch] = False
            return 0
        else:
            return 1
    def gpioWrite(self, ch, dir, value):
        self.dll.FT_WriteGPIO(self.pChannelHandle[ch], c_uint8(int(dir)), c_uint8(int(value)))
        
    def gpioRead(self, ch):
        readBuff = c_uint8(0)
        self.dll.FT_ReadGPIO(self.pChannelHandle[ch], byref(readBuff))
        return readBuff.value

    def spiReadWrite(self, ch, cs, value):
        cs = int(cs)
        writeBuff = (c_uint8*(len(value)+1))()
        for i in range(len(value)):
            writeBuff[i] = value[i]
        writeBuff[len(value)] = 0
        readBuff = (c_uint8*(len(value)+1))()
        numBytesTransferred = c_uint32(0)

        direction = c_uint8(int(255))
        gpioValue = (1<<8)-1-(1 << cs)
        self.dll.FT_WriteGPIO(self.pChannelHandle[ch], direction, gpioValue)
        # time.sleep(3.2) # for test
        self.dll.SPI_ReadWrite(self.pChannelHandle[ch], byref(readBuff), byref(writeBuff),
            c_uint8(len(writeBuff)), byref(numBytesTransferred), c_uint32(0) )
        self.dll.FT_WriteGPIO(self.pChannelHandle[ch], direction, direction)
        ret = [0]*len(value)
        for i in range(len(value)):
            ret[i] = readBuff[i+1]
        return ret
    def spiRead(self, ch, cs, length):
        cs = int(cs)
        direction = c_uint8(int(255))
        gpioValue = (1<<8)-1-(1 << cs)
        readBuff = (c_uint8*(length))()
        numBytesTransferred = c_uint32(0)
        self.dll.FT_WriteGPIO(self.pChannelHandle[ch], direction, gpioValue)
        # time.sleep(3.2) # for test
        self.dll.SPI_Read(self.pChannelHandle[ch], byref(readBuff), c_uint8(length),
            byref(numBytesTransferred), c_uint32(0))
        self.dll.FT_WriteGPIO(self.pChannelHandle[ch], direction, direction)
        ret = [0]*length
        for i in range(length):
            ret[i] = readBuff[i]
        return ret
    def spiWrite(self, ch, cs, value):
        cs = int(cs)
        direction = c_uint8(int(255))
        gpioValue = (1<<8)-1-(1 << cs)
        writeBuff = (c_uint8*(len(value)))()
        for i in range(len(value)):
            writeBuff[i] = value[i]
        numBytesTransferred = c_uint32(0)
        self.dll.FT_WriteGPIO(self.pChannelHandle[ch], direction, gpioValue)
        # time.sleep(3.2) # for test
        self.dll.SPI_Write(self.pChannelHandle[ch], byref(writeBuff),
            c_uint8(len(writeBuff)), byref(numBytesTransferred), c_uint32(0) )
        self.dll.FT_WriteGPIO(self.pChannelHandle[ch], direction, direction)

    class DeviceInfo:
        # def __init__(self):
        #     self.Flags = Z

        def setup(self, deviceInfoNode):
            self.Flags = int(deviceInfoNode.Flags)
            self.Type = int(deviceInfoNode.Type)
            self.ID = int(deviceInfoNode.ID)
            self.LocID = int(deviceInfoNode.LocID)
            self.SerialNumber = bytearray(deviceInfoNode.SerialNumber)[2:bytearray(deviceInfoNode.SerialNumber)[2:].find(b'\x00')+2].decode('UTF-8', errors='ignore')
            self.Description = bytearray(deviceInfoNode.Description)[2:bytearray(deviceInfoNode.Description)[2:].find(b'\x00')+2].decode('UTF-8', errors='ignore')
            # self.SerialNumber = bytearray(deviceInfoNode.SerialNumber)[2:bytearray(deviceInfoNode.SerialNumber)[2:].find(b'\x00\x00')+2].decode('UTF-8')
            # self.Description = bytearray(deviceInfoNode.Description)[2:bytearray(deviceInfoNode.Description)[2:].find(b'\x00\x00')+2].decode('UTF-8')
            self.ftHandle = deviceInfoNode.ftHandle

    class ChannelConfig(Structure):
        _fields_ = [
            ('ClockRate',c_uint32),
            ('LatencyTimer',c_uint8),
            ('configOptions',c_uint32),
            ('Pin',c_uint32),
            ('reserved',c_uint16)
        ]
    class FT_DEVICE_LIST_INFO_NODE(Structure):
        _fields_ = [
            ('Flags', c_ulong),
            ('Type', c_ulong),
            ('ID', c_ulong),
            ('LocID', c_ushort),
            ('SerialNumber', c_ubyte * 16), # char 16
            ('Description', c_ubyte * 64), # char 64
            ('ftHandle', c_void_p)
        ]

# if __name__ == '__main__':
#     mpsse = MPSSEMultiCh('./libMPSSE.dll')
#     mpsse.showDevices()
#     cs = [0]*2
#     cs[0] = mpsse.openChannel(0)
#     # cs[1] = mpsse.openChannel(1)

#     # direction = c_uint8(int(255))
#     # gpioValue = (1<<8)-1-(1 << 2)
#     # mpsse.dll.FT_WriteGPIO(mpsse.pChannelHandle, direction, gpioValue)
#     # time.sleep(1.0)
#     # mpsse.dll.FT_WriteGPIO(mpsse.pChannelHandle, direction, direction)

#     mpsse.closeChannel(cs[0])
#     # mpsse.closeChannel(cs[1])


# if __name__ == '__main__':
#     mpsse = MPSSEMultiCh('./libMPSSE.dll')
#     mpsse.showDevices()

#     ch = mpsse.openChannel(0)  # 0番チャネルを使用
#     cs = 0  # チップセレクト番号。回路によっては 1や2 かもしれません

#     # 送信データ（例えば、デバイスに依存するコマンド）
#     send_data = [0x9F]  # SPIフラッシュなどの JEDEC ID 読み取りコマンドの例
#     recv_data = mpsse.spiReadWrite(ch, cs, send_data)

#     print(f"Received data: {recv_data}")  # 結果を表示

#     mpsse.closeChannel(ch)


if __name__ == '__main__':
    mpsse = MPSSEMultiCh('./libMPSSE.dll')
    mpsse.showDevices()

    ch = mpsse.openChannel(0)  # チャンネル0を開く

    try:
        # ビット0のみ入力（dir=0）、他は出力（1）→方向: 0b11111110 = 0xFE
        dir_mask = 0b11111110
        out_value = 0b00000000  # 出力ピンは全てLowにする
        mpsse.gpioWrite(ch, dir_mask, out_value)

        while True:
            value = mpsse.gpioRead(ch)
            print(f"GPIO value: {bin(value)}")
            time.sleep(1)

    except KeyboardInterrupt:
        print("終了します。")

    mpsse.closeChannel(ch)
