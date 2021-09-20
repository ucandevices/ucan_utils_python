# coding: utf-8

"""
Serial based interface. For example use over serial ports like
"/dev/ttyS1" or "/dev/ttyUSB0" on Linux machines or "COM1" on Windows.
The interface is a simple implementation for uCAN CAN/USB CFUC converter.
"""
from __future__ import absolute_import, division

import logging
import struct

from can import BusABC, Message
logger = logging.getLogger('can.serial')

try:
    import serial
except ImportError:
    logger.warning("You won't be able to use the serial can backend without "
                   "the serial module installed!")
    serial = None

ADLC = {
    0: 0x00000000,
    1: 0x00010000,
    2: 0x00020000,
    3: 0x00030000,
    4: 0x00040000,
    5: 0x00050000,
    6: 0x00060000,
    7: 0x00070000,
    8: 0x00080000,
    9: 0x00090000,
    10: 0x000A0000,
    11: 0x000B0000,
    12: 0x000C0000,
    13: 0x000D0000,
    14: 0x000E0000,
    15: 0x000F0000,
}


class CFUC(BusABC):
    can_frame_count = 0
    def __init__(self, channel, baudrate=115200, timeout=0.1, rtscts=False,
                 *args, **kwargs):
        if not channel:
            raise ValueError("Must specify a serial port.")

        self.channel_info = "Serial interface: " + channel
        self.ser = serial.serial_for_url(
            channel, baudrate=baudrate, timeout=timeout, rtscts=rtscts)

        self.can_frame_count = 0

        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()


        super(CFUC, self).__init__(channel=channel, *args, **kwargs)            

    def shutdown(self):
        """
        Close the serial interface.
        """
        self.ser.close()

    def send(self, msg, timeout=None):
        """
        Send a message over the serial device.

        :param can.Message msg:
            Message to send.

            .. note:: If the timestamp is a value it will be ignored

        :param timeout:
            This parameter will be ignored. The timeout value of the channel is
            used instead.

        """
        
        UCAN_FD_TX = struct.pack('<I', int(0x2))
        try:
            a_id = struct.pack('<I', msg.arbitration_id)
        except struct.error:
            raise ValueError('Arbitration Id is out of range')
        a_ex = b'\x00\x00\x00\x40' if (msg.is_extended_id) else b'\x00\x00\x00\x00'          
        a_rmt = b'\x00\x00\x00\x20' if (msg.is_remote_frame) else b'\x00\x00\x00\x00'
       
        a_dlc =struct.pack('<I', int(ADLC[msg.dlc]))


        #copy all structs to byte stream
        byte_msg = bytearray()
        # UCAN_FRAME_TYPE frame_type; /*!< Frame type is @ref UCAN_FD_TX.*/
        for i in range(0, 4):
            byte_msg.append(UCAN_FD_TX[i])
        # FDCAN_TxHeaderTypeDef can_tx_header; /*!< FDCAN Tx event FIFO structure definition @ref FDCAN_TxHeaderTypeDef.*/
        for i in range(0, 4):
            byte_msg.append(a_id[i])
        for i in range(0,4):
            byte_msg.append(a_ex[i])
        for i in range(0,4):
            byte_msg.append(a_rmt[i])
        for i in range(0,4):
            byte_msg.append(a_dlc[i])

        if (msg.error_state_indicator == False):
            byte_msg.extend( b'\x00\x00\x00\x00') #ErrorStateIndicator FDCAN_ESI_PASSIVE
        else: 
            byte_msg.extend( b'\x00\x00\x00\x80') #ErrorStateIndicator FDCAN_ESI_ACTIVE
        
        if (msg.bitrate_switch == False):
            byte_msg.extend( b'\x00\x00\x00\x00') #BitRateSwitch FDCAN_BRS_OFF
        else:
            byte_msg.extend( b'\x00\x00\x10\x00') #BitRateSwitch FDCAN_BRS_ON

        if (msg.is_fd == False):
            byte_msg.extend( b'\x00\x00\x00\x00') #FDFormat FDCAN_CLASSIC_CAN
        else:
            byte_msg.extend( b'\x00\x00\x20\x00') #FDFormat FDCAN_FD_CAN

        byte_msg.extend( b'\x00\x00\x00\x00') #TxEventFifoControl
        byte_msg.extend( b'\x00\x00\x00\x00') #MessageMarker

        # uint8_t can_data[64]; /* Data CAN buffer */
        for i in range(0, msg.dlc):                   
            byte_msg.append(msg.data[i])
        for i in range(msg.dlc,64):
            byte_msg.extend(b'\x00')
        self.ser.write(byte_msg)
    
    
    def _recv_internal(self, timeout):        
        """
        Read a message from the serial device.

        :param timeout:

            .. warning::
                This parameter will be ignored. The timeout value of the channel is used.

        :returns:
            Received message and False (because not filtering as taken place).

            .. warning::
                Flags like is_extended_id, is_remote_frame and is_error_frame
                will not be set over this function, the flags in the return
                message are the default values.

        :rtype:
            can.Message, bool
        """           
        if (self.can_frame_count == 0):
            # ser.read can return an empty string
            # or raise a SerialException  
            try:
                rx_byte = self.ser.read(4)
                rx_byte = (struct.unpack('<I', rx_byte))[0]
            except serial.SerialException:
                return None, False
            if rx_byte == 0x06:#UCAN_FD_RX:
                s = bytearray(self.ser.read(4))
                self.can_frame_count = (struct.unpack('<I', s))[0]
            # else   
            if (self.can_frame_count > 0) & (self.can_frame_count < 10):
                self.can_frame_count = self.can_frame_count - 1
                s = bytearray(self.ser.read(4)) 
                arb_id = (struct.unpack('<I', s))[0] #Identifier
                s = bytearray(self.ser.read(8)) #IdType #RxFrameType 
                s = bytearray(self.ser.read(4)) #DataLength
                len = (struct.unpack('<I', s))[0]
                dlc = (list(ADLC.values()).index(len))
                s = bytearray(self.ser.read(4)) #ErrorStateIndicator
                s = bytearray(self.ser.read(4)) #BitRateSwitch
                s = bytearray(self.ser.read(4)) #FDFormat
                s = bytearray(self.ser.read(4)) #RxTimestamp
                timestamp = (struct.unpack('<I', s))[0] 
                s = bytearray(self.ser.read(4)) #FilterIndex
                s = bytearray(self.ser.read(4)) #IsFilterMatchingFrame
                data = bytearray(self.ser.read(64)) #can_data
                s = bytearray(self.ser.read(4)) #packed_flags_and_error_counters  

                msg = Message(timestamp=timestamp/1000,
                            arbitration_id=arb_id,
                            dlc=dlc,
                            data=data)
                return msg, False
            else:
                self.can_frame_count = 0
                return None, False