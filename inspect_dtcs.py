#!/usr/bin/env python3
"""
Inspect Active DTCs on BMW SME
"""
import sys
import time
import serial
import threading
import struct
from dtc_definitions import decode_dtc, decode_status, print_dtc_report

CRC8_TABLE = [
    0x00, 0x1D, 0x3A, 0x27, 0x74, 0x69, 0x4E, 0x53, 0xE8, 0xF5, 0xD2, 0xCF, 0x9C, 0x81, 0xA6, 0xBB,
    0xCD, 0xD0, 0xF7, 0xEA, 0xB9, 0xA4, 0x83, 0x9E, 0x25, 0x38, 0x1F, 0x02, 0x51, 0x4C, 0x6B, 0x76,
    0x87, 0x9A, 0xBD, 0xA0, 0xF3, 0xEE, 0xC9, 0xD4, 0x6F, 0x72, 0x55, 0x48, 0x1B, 0x06, 0x21, 0x3C,
    0x4A, 0x57, 0x70, 0x6D, 0x3E, 0x23, 0x04, 0x19, 0xA2, 0xBF, 0x98, 0x85, 0xD6, 0xCB, 0xEC, 0xF1,
    0x13, 0x0E, 0x29, 0x34, 0x67, 0x7A, 0x5D, 0x40, 0xFB, 0xE6, 0xC1, 0xDC, 0x8F, 0x92, 0xB5, 0xA8,
    0xDE, 0xC3, 0xE4, 0xF9, 0xAA, 0xB7, 0x90, 0x8D, 0x36, 0x2B, 0x0C, 0x11, 0x42, 0x5F, 0x78, 0x65,
    0x94, 0x89, 0xAE, 0xB3, 0xE0, 0xFD, 0xDA, 0xC7, 0x7C, 0x61, 0x46, 0x5B, 0x08, 0x15, 0x32, 0x2F,
    0x59, 0x44, 0x63, 0x7E, 0x2D, 0x30, 0x17, 0x0A, 0xB1, 0xAC, 0x8B, 0x96, 0xC5, 0xD8, 0xFF, 0xE2,
    0x26, 0x3B, 0x1C, 0x01, 0x52, 0x4F, 0x68, 0x75, 0xCE, 0xD3, 0xF4, 0xE9, 0xBA, 0xA7, 0x80, 0x9D,
    0xEB, 0xF6, 0xD1, 0xCC, 0x9F, 0x82, 0xA5, 0xB8, 0x03, 0x1E, 0x39, 0x24, 0x77, 0x6A, 0x4D, 0x50,
    0xA1, 0xBC, 0x9B, 0x86, 0xD5, 0xC8, 0xEF, 0xF2, 0x49, 0x54, 0x73, 0x6E, 0x3D, 0x20, 0x07, 0x1A,
    0x6C, 0x71, 0x56, 0x4B, 0x18, 0x05, 0x22, 0x3F, 0x84, 0x99, 0xBE, 0xA3, 0xF0, 0xED, 0xCA, 0xD7,
    0x35, 0x28, 0x0F, 0x12, 0x41, 0x5C, 0x7B, 0x66, 0xDD, 0xC0, 0xE7, 0xFA, 0xA9, 0xB4, 0x93, 0x8E,
    0xF8, 0xE5, 0xC2, 0xDF, 0x8C, 0x91, 0xB6, 0xAB, 0x10, 0x0D, 0x2A, 0x37, 0x64, 0x79, 0x5E, 0x43,
    0xB2, 0xAF, 0x88, 0x95, 0xC6, 0xDB, 0xFC, 0xE1, 0x5A, 0x47, 0x60, 0x7D, 0x2E, 0x33, 0x14, 0x09,
    0x7F, 0x62, 0x45, 0x58, 0x0B, 0x16, 0x31, 0x2C, 0x97, 0x8A, 0xAD, 0xB0, 0xE3, 0xFE, 0xD9, 0xC4
]

def calculate_crc(data, length, init=0x3F):
    crc = init
    for b in data[1:length]:
        crc = CRC8_TABLE[(crc ^ b) & 0xFF]
    return crc

import glob

# Port parameter or auto-detection
ports = glob.glob("/dev/cu.usbmodem*") + glob.glob("/dev/tty.usbmodem*") + glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")
default_port = ports[0] if ports else "/dev/cu.usbmodem3101"
port = sys.argv[1] if len(sys.argv) > 1 else default_port

print(f"[*] Connecting to {port}...")
ser = serial.Serial(port, 115200, timeout=0.01)
time.sleep(0.05)

# 100k Wakeup
ser.write(b"C\r")
time.sleep(0.05)
ser.write(b"S3\r")
time.sleep(0.05)
ser.write(b"O\r")
time.sleep(0.05)
ser.write(b"t55445AA55AA5\r")
time.sleep(0.01)
ser.write(b"t55445AA55AA5\r")
time.sleep(0.05)

# 500k
ser.write(b"C\r")
time.sleep(0.05)
ser.write(b"S6\r")
time.sleep(0.05)
ser.write(b"O\r")
time.sleep(0.05)
ser.flushInput()

lock = threading.Lock()
running = True

def send_can(can_id, data):
    dlc = len(data)
    data_hex = "".join(f"{b:02X}" for b in data)
    cmd = f"t{can_id:03X}{dlc}{data_hex}\r".encode('ascii')
    with lock:
        ser.write(cmd)

def keepalive():
    alive_20 = 0
    alive_100 = 0
    clk_sec = 1000
    t_20 = t_100 = t_200 = t_1000 = 0
    while running:
        now = time.time()
        if now - t_20 >= 0.02:
            t_20 = now
            p = bytearray([0, alive_20, 0xFC])
            p[0] = calculate_crc(p, 3, 0x3F)
            send_can(0x10B, p)
            alive_20 = (alive_20 + 1) % 15
        if now - t_100 >= 0.1:
            t_100 = now
            p = bytearray([0, 0x20 + alive_100, 0x86, 0x1B, 0xF1, 0x35, 0x30, 0x02])
            p[0] = calculate_crc(p, 8, 0x3F)
            send_can(0x12F, p)
            alive_100 = (alive_100 + 1) % 15
        if now - t_200 >= 0.2:
            t_200 = now
            send_can(0x53A, [0x40, 0x3A, 0x04, 0x00, 0x00, 0x00, 0x01, 0x00])
        if now - t_1000 >= 1.0:
            t_1000 = now
            send_can(0x3A0, [0xFF, 0xFF, 0xCF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD])
            clk_sec += 1
            send_can(0x328, list(struct.pack("<I", clk_sec) + struct.pack("<H", 8800)))
            send_can(0x3CA, [0xB7, 0x60, 0x01, 0x0F, 0x0F, 0x31, 0xFF, 0xFF])
            send_can(0x3E8, [0xF1, 0xFF])
            send_can(0x2CA, [0x6E, 0x6F])
        time.sleep(0.005)

t = threading.Thread(target=keepalive, daemon=True)
t.start()
time.sleep(0.5)

uds_buf = bytearray()
uds_expected = 0
uds_active = False
completed_payload = None

def handle_rx_line(line):
    global uds_expected, uds_active, uds_buf, completed_payload
    if line.startswith('t') and len(line) >= 5:
        cid = int(line[1:4], 16)
        dlc = int(line[4])
        data_hex = line[5:5 + dlc*2]
        data = bytes.fromhex(data_hex)
        
        if cid == 0x607:
            pci = data[1]
            ftype = (pci & 0xF0) >> 4
            if ftype == 0:
                length = pci & 0x0F
                completed_payload = data[2:2+length]
            elif ftype == 1:
                uds_expected = ((pci & 0x0F) << 8) | data[2]
                uds_buf = bytearray(data[3:])
                uds_active = True
                send_can(0x6F1, [0x07, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
            elif ftype == 2:
                uds_buf.extend(data[2:])
                if len(uds_buf) >= uds_expected:
                    completed_payload = bytes(uds_buf[:uds_expected])
                    uds_active = False
                    uds_buf.clear()

def rx_loop():
    buf = bytearray()
    while running:
        c = ser.read(64)
        if c:
            for b in c:
                if b in (ord('\r'), ord('\n')):
                    if buf:
                        l = buf.decode('ascii', errors='ignore')
                        buf.clear()
                        for p in l.split('t'):
                            if p:
                                handle_rx_line('t' + p)
                else:
                    buf.append(b)
        else:
            time.sleep(0.002)

rx_t = threading.Thread(target=rx_loop, daemon=True)
rx_t.start()

def request_and_wait(can_data, timeout=5.0):
    global completed_payload
    completed_payload = None
    send_can(0x6F1, can_data)
    t0 = time.time()
    while time.time() - t0 < timeout:
        if completed_payload is not None:
            # If NRC 0x78 (Response Pending), reset timeout and keep waiting!
            if completed_payload[0] == 0x7F and len(completed_payload) >= 3 and completed_payload[2] == 0x78:
                print(f"[...] Received NRC 0x78 (Response Pending), waiting for final response...")
                completed_payload = None
                t0 = time.time() # Reset timeout
                continue
            res = completed_payload
            completed_payload = None
            return res
        time.sleep(0.01)
    return None

# Read DTCs
print("\n[*] Querying DTCs ($19 02 FF)...")
res = request_and_wait([0x07, 0x03, 0x19, 0x02, 0xFF, 0x00, 0x00, 0x00])
if res:
    print(f"[+] Response ({len(res)} bytes): {res.hex().upper()}")
    if res[0] == 0x59:
        status_mask = res[2]
        records = res[3:]
        print_dtc_report(records, status_mask, show_all=False)
else:
    print("[-] No response to Read DTCs.")

running = False
time.sleep(0.2)
ser.write(b"C\r")
ser.close()
