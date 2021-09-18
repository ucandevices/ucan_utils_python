import ucan_cfuc
from can import Message

c = ucan_cfuc.CFUC("COM50")

m = Message(0,0x11223344,True,False,False,None,5,bytearray(b'\x01\x02\x03\x04\x05'),False,False,False,None,False)
c.send(m,None)

c.shutdown()