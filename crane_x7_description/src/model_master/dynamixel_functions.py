#!/usr/bin/env python
# -*- coding: utf-8 -*-

################################################################################
# Copyright (c) 2016, ROBOTIS CO., LTD.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of ROBOTIS nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
################################################################################

# Author: Ryu Woon Jung (Leon)
import os
import ctypes
from ctypes import cdll
# dxl_lib = cdll.LoadLibrary("../../c/build/win32/output/dxl_x86_c.dll")  # for windows 32bit
# dxl_lib = cdll.LoadLibrary("../../c/build/win64/output/dxl_x64_c.dll")  # for windows 64bit
# dxl_lib = cdll.LoadLibrary("../../c/build/linux32/libdxl_x86_c.so")     # for linux 32bit
dxl_lib = cdll.LoadLibrary(os.path.dirname(__file__) + "/linux64/libdxl_x64_c.so")     # for linux 64bit
# dxl_lib = cdll.LoadLibrary("../../c/build/linux_sbc/libdxl_sbc_c.so")   # for SBC linux
# dxl_lib = cdll.LoadLibrary("../../c/build/mac/libdxl_mac_c.dylib")      # for Mac OS

# port_handler
portHandler = dxl_lib.portHandler

openPort = dxl_lib.openPort
closePort = dxl_lib.closePort
clearPort = dxl_lib.clearPort

setPortName = dxl_lib.setPortName
getPortName = dxl_lib.getPortName

setBaudRate = dxl_lib.setBaudRate
getBaudRate = dxl_lib.getBaudRate

readPort = dxl_lib.readPort
writePort = dxl_lib.writePort

setPacketTimeout = dxl_lib.setPacketTimeout
setPacketTimeoutMSec = dxl_lib.setPacketTimeoutMSec
isPacketTimeout = dxl_lib.isPacketTimeout

# packet_handler
packetHandler = dxl_lib.packetHandler

printTxRxResult = dxl_lib.printTxRxResult
getTxRxResult = dxl_lib.getTxRxResult
getTxRxResult.restype = ctypes.c_char_p
printRxPacketError = dxl_lib.printRxPacketError
getRxPacketError = dxl_lib.getRxPacketError
getRxPacketError.restype = ctypes.c_char_p

getLastTxRxResult = dxl_lib.getLastTxRxResult
getLastRxPacketError = dxl_lib.getLastRxPacketError

setDataWrite = dxl_lib.setDataWrite
getDataRead = dxl_lib.getDataRead

txPacket = dxl_lib.txPacket

rxPacket = dxl_lib.rxPacket

txRxPacket = dxl_lib.txRxPacket

ping = dxl_lib.ping

pingGetModelNum = dxl_lib.pingGetModelNum

broadcastPing = dxl_lib.broadcastPing
getBroadcastPingResult = dxl_lib.getBroadcastPingResult

reboot = dxl_lib.reboot

factoryReset = dxl_lib.factoryReset

readTx = dxl_lib.readTx
readRx = dxl_lib.readRx
readTxRx = dxl_lib.readTxRx

read1ByteTx = dxl_lib.read1ByteTx
read1ByteRx = dxl_lib.read1ByteRx
read1ByteTxRx = dxl_lib.read1ByteTxRx

read2ByteTx = dxl_lib.read2ByteTx
read2ByteRx = dxl_lib.read2ByteRx
read2ByteTxRx = dxl_lib.read2ByteTxRx

read4ByteTx = dxl_lib.read4ByteTx
read4ByteRx = dxl_lib.read4ByteRx
read4ByteTxRx = dxl_lib.read4ByteTxRx

writeTxOnly = dxl_lib.writeTxOnly
writeTxRx = dxl_lib.writeTxRx

write1ByteTxOnly = dxl_lib.write1ByteTxOnly
write1ByteTxRx = dxl_lib.write1ByteTxRx

write2ByteTxOnly = dxl_lib.write2ByteTxOnly
write2ByteTxRx = dxl_lib.write2ByteTxRx

write4ByteTxOnly = dxl_lib.write4ByteTxOnly
write4ByteTxRx = dxl_lib.write4ByteTxRx

regWriteTxOnly = dxl_lib.regWriteTxOnly
regWriteTxRx = dxl_lib.regWriteTxRx

syncReadTx = dxl_lib.syncReadTx
# syncReadRx   -> GroupSyncRead
# syncReadTxRx -> GroupSyncRead

syncWriteTxOnly = dxl_lib.syncWriteTxOnly

bulkReadTx = dxl_lib.bulkReadTx
# bulkReadRx   -> GroupBulkRead
# bulkReadTxRx -> GroupBulkRead

bulkWriteTxOnly = dxl_lib.bulkWriteTxOnly

# group_bulk_read
groupBulkRead = dxl_lib.groupBulkRead

groupBulkReadAddParam = dxl_lib.groupBulkReadAddParam
groupBulkReadRemoveParam = dxl_lib.groupBulkReadRemoveParam
groupBulkReadClearParam = dxl_lib.groupBulkReadClearParam

groupBulkReadTxPacket = dxl_lib.groupBulkReadTxPacket
groupBulkReadRxPacket = dxl_lib.groupBulkReadRxPacket
groupBulkReadTxRxPacket = dxl_lib.groupBulkReadTxRxPacket

groupBulkReadIsAvailable = dxl_lib.groupBulkReadIsAvailable
groupBulkReadGetData = dxl_lib.groupBulkReadGetData

#group_bulk_write
groupBulkWrite = dxl_lib.groupBulkWrite

groupBulkWriteAddParam = dxl_lib.groupBulkWriteAddParam
groupBulkWriteRemoveParam = dxl_lib.groupBulkWriteRemoveParam
groupBulkWriteChangeParam = dxl_lib.groupBulkWriteChangeParam
groupBulkWriteClearParam = dxl_lib.groupBulkWriteClearParam

groupBulkWriteTxPacket = dxl_lib.groupBulkWriteTxPacket

#group_sync_read
groupSyncRead = dxl_lib.groupSyncRead

groupSyncReadAddParam = dxl_lib.groupSyncReadAddParam
groupSyncReadRemoveParam = dxl_lib.groupSyncReadRemoveParam
groupSyncReadClearParam = dxl_lib.groupSyncReadClearParam

groupSyncReadTxPacket = dxl_lib.groupSyncReadTxPacket
groupSyncReadRxPacket = dxl_lib.groupSyncReadRxPacket
groupSyncReadTxRxPacket = dxl_lib.groupSyncReadTxRxPacket

groupSyncReadIsAvailable = dxl_lib.groupSyncReadIsAvailable
groupSyncReadGetData = dxl_lib.groupSyncReadGetData

#group_sync_write
groupSyncWrite = dxl_lib.groupSyncWrite

groupSyncWriteAddParam = dxl_lib.groupSyncWriteAddParam
groupSyncWriteRemoveParam = dxl_lib.groupSyncWriteRemoveParam
groupSyncWriteChangeParam = dxl_lib.groupSyncWriteChangeParam
groupSyncWriteClearParam = dxl_lib.groupSyncWriteClearParam

groupSyncWriteTxPacket = dxl_lib.groupSyncWriteTxPacket
