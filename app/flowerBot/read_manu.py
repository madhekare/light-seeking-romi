#!/usr/bin/env python3

import struct
from bluepy.btle import ScanEntry, Scanner, DefaultDelegate
import argparse

parser = argparse.ArgumentParser(description='Print advertisement data from a BLE device')
parser.add_argument('addr', metavar='A', type=str, help='Address of the form XX:XX:XX:XX:XX:XX')
args = parser.parse_args()
addr = args.addr.lower()
if len(addr) != 17:
    raise ValueError("Invalid address supplied")

class ScanDelegate(DefaultDelegate):
    def __init__(self):
        DefaultDelegate.__init__(self)

    # when this python script discovers a BLE broadcast packet from a buckler
    # advertising light measurements, print out the data
    def handleDiscovery(self, dev, isNewDev, isNewData):
        if dev.addr == addr:
            print("found device", dev.addr)
            data = dev.getValue(ScanEntry.MANUFACTURER)
            try:
                # here we are unpacking a little-endian (<) halfword (h) and a
                # float (f), as well as 20 dummy bytes (20x)
                #print(data)
                #Value1 = data[:4]
                #Value2 = data[4:8]
                #Value3 = data[20:24]
                #val_1 = struct.unpack('f',Value1)
                #val_2 = struct.unpack('f',Value2)
                #val_3 = struct.unpack('f',Value3)
                values = struct.unpack('<hfff12x', data)
                # if the company ID matches that of Lab11
                #print(val_1)
                #print(val_2)
                #print(val_3)
                #if values[0] == 0x02e0:
                print("Value: " + str(values[1]) + " lux")
                print("Value: " + str(values[2]) )
                print("Value: " + str(values[3]) )

            except:
                print("Got bad packet format")

# create a scanner object that sends BLE broadcast packets to the ScanDelegate
scanner = Scanner().withDelegate(ScanDelegate())

# start the scanner and keep the process running
scanner.start()
while True:
    scanner.process()