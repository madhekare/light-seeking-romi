#!/usr/bin/env python3

import struct
from bluepy.btle import ScanEntry, Scanner, DefaultDelegate, Peripheral
import argparse

parser = argparse.ArgumentParser(description='Print advertisement data from a BLE device')
parser.add_argument('addr', metavar='A', type=str, help='Address of the form XX:XX:XX:XX:XX:XX')
args = parser.parse_args()
addr = args.addr.lower()
if len(addr) != 17:
    raise ValueError("Invalid address supplied")

SERVICE_UUID = "32e61089-2b22-4db5-a914-43ce41986c70"
CHAR_UUID    = "32e6108a-2b22-4db5-a914-43ce41986c70"
CHAR_UUID2    = "32e6108b-2b22-4db5-a914-43ce41986c70"
CHAR_UUID3    = "32e6108c-2b22-4db5-a914-43ce41986c70"


try:
    print("connecting")
    buckler = Peripheral(addr)

    print("connected")

    # Get service
    sv = buckler.getServiceByUUID(SERVICE_UUID)
    # Get characteristic
    ch = sv.getCharacteristics(CHAR_UUID)[0]
    ch2 = sv.getCharacteristics(CHAR_UUID2)[0]
    ch3 = sv.getCharacteristics(CHAR_UUID3)[0]
    f = open("ble_data.txt", "a")
    f.truncate(0)
    counter = 0
    while True:
        print(counter)
        counter += 1
        data =ch.read()
        print("data =")
        data = struct.unpack('f', data)
        print(data)


        data_front =ch2.read()
        print("data2 =")
        data_front = struct.unpack('f', data_front)
        print(data_front)
        print(type(data_front))

        data_right =ch3.read()
        print("data3 =")
        data_right = struct.unpack('f', data_right)
        print(data_right)

        f.write(str(data[0]) + " ")
        f.write(str(data_front[0]) + " ")
        f.write(str(data_right[0]) + " ")
        f.write("\n")
    f.close()
finally:
    buckler.disconnect()
