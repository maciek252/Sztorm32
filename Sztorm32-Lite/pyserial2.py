
from consoledraw import Console
from datetime import datetime

from time import sleep

import struct
import ctypes
import asyncio

import aioserial

import crc16
import serial
ser = serial.Serial('/dev/ttyUSB0',115200)  # open serial port
print(ser.name)         # check which port was really used
#values = bytearray([0x23, 9, 62, 144, 56, 30, 147, 3, 210, 89, 111, 78, 184, 151, 17, 129])
#values = bytearray([0xA5, 0x5A, 0x02 , 0x0D , 0x03 , 0x16 , 0xBC , 0x02 , 0x07 , 0xBB]) # horizon move
#values = bytearray([0xA5, 0x5A, 0x02 , 0x0D , 0x03 , 0x16 ,0x84 ,0x03 ,0x1A ,0x27]) # horizon move

#A5 5A 00 11 05 FF 01 FF 00 00 63 2F # joystick
values = bytearray([0xA5, 0x5A, 0x02 , 0x0D , 0x03 , 0x16 ,0x58 ,0x02 ,0x71 ,0x67]) # horizon move
#valuesWithoutCRC = bytearray([]) # horizon move
# 03 to liczba bajtow? no chyba tak!
#A5 5A 02 0D 03 0D 64 00 59 B4 // joystick max
#hexString = "020d03064400" 
#hexString = "041703165802" 
#hexString = "020D0323F100" # horizon
#hexString = "001105FF01ff0000" # 
#hexString = "001105000000ff00" # 
#hexString = "0011059FF0080000" # ok yaw right!
#hexString = "0011059000FF0000" # ok dziala yaw right!
#hexString =  "0011059000FF0000" # ok dziala there is some inertion
#hexString = "00110590000000FF" # ok  xx xx xx xx FF - pitch down
#hexString = "001105900000FF00" # ok xx xx xx FF xx - pitch up
hexString = "0011059000FF0000" # ok xx xx FF xx xx - yaw left
#hexString = "00110500FF000000" # ok xx FF xx xx xx - yaw right
#hexString = "001200" # 
#hexString = "001105FF00000000" # ok FF xx xx xx xx - CONTINUE? STOP?
# 
#hexString = "0011050000000000" # ok 00 00 00 00 00 - STOP
#hexString = "001105FF00000000" # ok 00 00 00 00 00 - STOP
#hexString = "00110500FF00FF00" # ok 
#hexString = "0011059FF00AF000" # ok yaw fast left
#hexString = "0011059FFFFAF000" # ok yaw fast right
#A5 5A 00 10 05 FF 00 00 00 00 D5 2E #?
#hexString = "001005ff00000000"  # starts TM from the gimbal!
#hexString = "0010050000000000"  # stops TM from the gimbal!
#hexString = "0011059FFFFAF0F0" # 
#hexString = "0012059FFFFAF0F0" # 
#hexStringWithCRC = "020d03165802"
#hexString = "020D0300F000" # horizon
prefix= "A55A"
b = bytes.fromhex(hexString)
barr = bytearray.fromhex(hexString)
barrPrefix = bytearray.fromhex(prefix)
barr = bytearray.fromhex(hexString)
#barrWithCrc = bytearray.fromhex(hexStringWithCRC)
#crc = crc16.crc16xmodem(b'\x02\x0D\x03\x16\x58\x02') # dobre!
crc = crc16.crc16xmodem(b)

print(crc)
print(hex(crc))
crcCale = hex(crc)
print(hex(crc & 0xFF))
print(hex((crc>>8) & 0xFF))
print(barr)

print(values)
c1 = (crc & 0xFF)
c2 = (crc>>8) & 0xFF
crsStr = f'{c1:x}' + f'{c2:x}'
sum = barrPrefix + barr + bytearray.fromhex(crsStr) #+ c1.to_bytes(2, 'big') + c2.to_bytes(2, 'big')


print(sum)
#ser.write(barrWithCrc)
ser.write(sum)
#ser.close()             # close port

console = Console()

format = """
    ╔══════════╗
    ║ {} ║
    ╚══════════╝
    {}
    {}
    {}
"""
licznik = ""
licznikRoll = ""
licznikYaw = ""
async def read_and_print(aioserial_instance: aioserial.AioSerial):
    while True:
        with console:
            chrum = await aioserial_instance.read_async()
            stru : bytes = chrum.decode(errors='ignore')
            global licznik
            global licznikRoll
            #licznik += len(str)
            #str2 = ''.join(format(str, '02x'))
            #print(''.join(format(x, '02x') for x in str))            )
            #licznik += " ".join(["{:02x}".format(x) for x in chrum])
            #licznik += hex(chrum)
            #licznik += str
            #licznik += ''.join(format(x, '02x') for x in chrum)
            for x in chrum:
                #licznik += "" + hex(x) + " "
                licznik += hex(x) + " "
            console.print(format.format(datetime.strftime(datetime.now(), "%H:%M:%S"), licznik))
            #licznik = 3

#asyncio.run(read_and_print(aioserial.AioSerial(port='/dev/ttyUSB0')))

bajty = []

def parseFrame(bajty):    
    #if(len(bajty) > 1 and (bajty[0] == 0xA5) and (bajty[1] == 0x5A)):
        
    if(len(bajty) < 5):        
        return False
    dlugosc = bajty[4] 
    #print("dlugosc=" + str(len(bajty)))   
    if(len(bajty) == 5 + dlugosc + 2):
        #print("kompletna!")
        return True
    return False    

def interpretFrame(bajty):
    #if(bajty[2] == ctypes.c_ubyte(0x03) and bajty[3] == ctypes.c_ubyte(0x10)):
    if(bajty[2] == 0x03 and bajty[3] == 0x10):
     #   print("pozycja!")
        i = 43
    else:
        return    
    #pos = struct.unpack(bytes(ctypes.c_ubyte(bajty[6])), bytes(ctypes.c_ubyte(bajty[7])))
    #pos = bajty[6]<<8 | bajty[7]
    #pos = bajty[7]<<8 | bajty[6]
    #baj = [bajty[6], bajty[7]]
    baj = [bajty[6]]
    pos = int.from_bytes(baj, "little", signed=True)
    #print("pos=" + str(pos))        
    

while True:
    with console:        
        line  = ser.readline(1)
        #licznik += ''.join(format(line, '02x'))
        #print(line)
        
        if(line != bytes(ctypes.c_ubyte(0xA5)) and len(bajty) == 0):
            continue        
#        if((line != 0xA5) and len(bajty) == 0):
 #           continue
        bajty += line
        if( parseFrame(bajty) == True):            
            interpretFrame(bajty)
            
            licznik = "" + str( (bajty[7]&0b01111111) * 256 + (bajty[6]&0b11111111)) + " " + str((bajty[7]&0b01111111) * 256 + bajty[6]*1) + " " + str(bajty[6])
            

            if(bajty[7] & 0b10000000):
                
                licznik = "" + str( (bajty[7]&0b01111111) * 256 + (bajty[6]&0b11111111)) + " " + str((bajty[7]&0b01111111) * 256 - bajty[6]*1) + " " + str(bajty[6])
                wyn = int.from_bytes([bajty[7]], "little", signed=True)
                licznik += "MINUS" + str(wyn * 256)

            licznikRoll = "" + str( (bajty[9]&0b01111111) * 256 + (bajty[8]&0b11111111)) + " " + str((bajty[9]&0b01111111) * 256 + bajty[8]*1) + " " + str(bajty[8])
            
            if(bajty[9] & 0b10000000):
                
                licznikRoll = "" + str( (bajty[9]&0b01111111) * 256 + (bajty[8]&0b11111111)) + " " + str((bajty[9]&0b01111111) * 256 - bajty[8]*1) + " " + str(bajty[8])
                licznikRoll += "MINUS"

            licznikYaw = "" + str( (bajty[11]&0b01111111) * 256 + (bajty[10]&0b11111111)) + " " + str((bajty[11]&0b01111111) * 256 + bajty[10]*1) + " " + str(bajty[10])
            
            if(bajty[11] & 0b10000000):
                
                licznikYaw = "" + str( (bajty[11]&0b01111111) * 256 + (bajty[10]&0b11111111)) + " " + str((bajty[11]&0b01111111) * 256 - bajty[10]*1) + " " + str(bajty[10])
                licznikYaw += "MINUS"


            bajty = []    
        #licznik += " " + line.hex()
        console.print(format.format(datetime.strftime(datetime.now(), "%H:%M:%S"), licznik, licznikRoll, licznikYaw))
    #sleep(0.5)

