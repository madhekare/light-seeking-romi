#variant of read_advertisement ble based program from lab
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


try:
    print("connecting")
    buckler = Peripheral(addr)

    print("connected")

    # Get service
    sv = buckler.getServiceByUUID(SERVICE_UUID)
    # Get characteristic
    ch = sv.getCharacteristics(CHAR_UUID)[0]

    while True:

        data =ch.read()
        print("data =")
        print(data)
finally:
    buckler.disconnect()