#!/usr/bin/env python3
"""
BMW SME BK2 AS4 Master Controller & Diagnostic Tool
===================================================
Reverse-engineered from BMW Gen3/Gen4 PHEV BMS implementation (Dala's Battery-Emulator).

Features:
1. Low-speed CAN Wakeup pulse sequence (100 kbps, ID 0x554)
2. High-speed 500 kbps full vehicle environment simulation:
   - 0x10B (20ms): Contactor driver / alive + CRC8 SAE J1850
   - 0x12F (100ms): Terminal 15 status / alive + CRC8 SAE J1850
   - 0x53A (200ms): Contactor state announcement (State A idle)
   - 0x3A0 (1000ms): BDC vehicle condition
   - 0x328 (1000ms): KOMBI relative time clock (live seconds & day counter)
   - 0x3CA (1000ms): Driving forecast
   - 0x3E8 (1000ms): OBD diagnostic idle
   - 0x2CA (1000ms): Ambient temperature (15°C)
3. ISO-TP / UDS Diagnostic client:
   - Physical addressing: Tester ID 0xF1 -> Target SME ID 0x07 via CAN ID 0x6F1
   - ECU responds on CAN ID 0x607!
   - Full Multi-Frame / Flow Control reassembly
   - Commands: Clear DTC ($14), Read DTC ($19), Read VIN ($22 F190), Stop Balancing Routine ($31 02 AD 6B)
4. Telemetry decoder:
   - 0x112: HV Pack Voltage, Current, Status
   - 0x432: State of Charge (SOC)
"""

import sys
import time
import serial
import glob
import threading
import struct
from dtc_definitions import decode_dtc, decode_status, print_dtc_report

# CRC8 Table for SAE J1850 (Poly 0x1D, Init 0x3F)
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

class SMEMaster:
    def __init__(self, port):
        self.port = port
        self.ser = None
        self.running = False
        self.lock = threading.Lock()
        
        # State variables
        self.alive_20ms = 0
        self.alive_100ms = 0
        self.clock_seconds = 1000
        self.clock_days = 8800
        
        # UDS Multi-frame reassembly
        self.uds_buffer = bytearray()
        self.uds_expected_len = 0
        self.uds_in_progress = False
        self.cf_batch_count = 0

        # Telemetry throttling
        self.last_112_print = 0
        self.last_432_print = 0
        self.verbose = False

    def open(self):
        print(f"[*] Connecting to {self.port}...")
        self.ser = serial.Serial(self.port, 115200, timeout=0.01)
        time.sleep(0.05)

    def send_slcan(self, cmd_str):
        with self.lock:
            if self.ser and self.ser.is_open:
                self.ser.write((cmd_str + "\r").encode('ascii'))

    def send_can(self, can_id, data):
        dlc = len(data)
        data_hex = "".join(f"{b:02X}" for b in data)
        cmd = f"t{can_id:03X}{dlc}{data_hex}"
        self.send_slcan(cmd)

    def pulse_100k_wakeup(self):
        print("[*] Initiating 100 kbps low-speed wake sequence...")
        with self.lock:
            self.ser.write(b"C\r")
            time.sleep(0.05)
            self.ser.write(b"S3\r") # 100 kbps
            time.sleep(0.05)
            self.ser.write(b"O\r")
            time.sleep(0.05)
            
            # Send 0x554 wakeup frame twice
            wakeup_frame = b"t55445AA55AA5\r"
            self.ser.write(wakeup_frame)
            time.sleep(0.01)
            self.ser.write(wakeup_frame)
            time.sleep(0.05)
            
            # Switch back to 500 kbps
            self.ser.write(b"C\r")
            time.sleep(0.05)
            self.ser.write(b"S6\r") # 500 kbps
            time.sleep(0.05)
            self.ser.write(b"O\r")
            time.sleep(0.05)
            self.ser.flushInput()
        print("[+] 500 kbps vehicle bus active!")

    def start_keepalive_loop(self):
        self.running = True
        
        t_last_20 = 0.0
        t_last_100 = 0.0
        t_last_200 = 0.0
        t_last_1000 = 0.0
        t_last_tester_present = 0.0

        print("[*] Starting keep-alive cyclic frames (0x10B, 0x12F, 0x53A, 0x3A0, 0x328, 0x3CA, 0x3E8, 0x2CA)...")

        while self.running:
            now = time.time()

            # 20ms Loop: 0x10B (Contactor Idle / Driver)
            if now - t_last_20 >= 0.020:
                t_last_20 = now
                payload_10b = bytearray([0x00, 0x00 + self.alive_20ms, 0xFC])
                payload_10b[0] = calculate_crc(payload_10b, 3, 0x3F)
                self.send_can(0x10B, payload_10b)
                self.alive_20ms = (self.alive_20ms + 1) % 15

            # 100ms Loop: 0x12F (Terminal Status / KL15)
            if now - t_last_100 >= 0.100:
                t_last_100 = now
                payload_12f = bytearray([0x00, 0x20 + self.alive_100ms, 0x86, 0x1B, 0xF1, 0x35, 0x30, 0x02])
                payload_12f[0] = calculate_crc(payload_12f, 8, 0x3F)
                self.send_can(0x12F, payload_12f)
                self.alive_100ms = (self.alive_100ms + 1) % 15

            # 200ms Loop: 0x53A (Vehicle Contactor State Announcement)
            if now - t_last_200 >= 0.200:
                t_last_200 = now
                self.send_can(0x53A, [0x40, 0x3A, 0x04, 0x00, 0x00, 0x00, 0x01, 0x00])

            # 1000ms Loop: Vehicle Environment (BDC condition, KOMBI clock, etc.)
            if now - t_last_1000 >= 1.000:
                t_last_1000 = now
                # 0x3A0 BDC Vehicle Condition
                self.send_can(0x3A0, [0xFF, 0xFF, 0xCF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD])
                
                # 0x328 KOMBI Clock (Seconds + Days LE)
                self.clock_seconds += 1
                sec_bytes = struct.pack("<I", self.clock_seconds)
                day_bytes = struct.pack("<H", self.clock_days)
                self.send_can(0x328, list(sec_bytes + day_bytes))
                
                # 0x3CA Driving Forecast
                self.send_can(0x3CA, [0xB7, 0x60, 0x01, 0x0F, 0x0F, 0x31, 0xFF, 0xFF])
                
                # 0x3E8 OBD Diagnosis EME Idle
                self.send_can(0x3E8, [0xF1, 0xFF])
                
                # 0x2CA Ambient Temp (15°C)
                self.send_can(0x2CA, [0x6E, 0x6F])

            # Tester Present every 2.0s (only when no UDS transfer is active)
            if now - t_last_tester_present >= 2.000 and not self.uds_in_progress:
                t_last_tester_present = now
                # Target 0x07, Single Frame len 2, 0x3E, 0x80 (SuppressPositiveResponse)
                self.send_can(0x6F1, [0x07, 0x02, 0x3E, 0x80, 0x00, 0x00, 0x00, 0x00])

            # Short sleep to prevent CPU hogging
            time.sleep(0.005)

    def send_uds(self, service_bytes):
        """Sends UDS request to SME (Target 0x07) via 0x6F1"""
        dlc = len(service_bytes)
        if dlc <= 6:
            # Single Frame: [Target, PCI_Len, Service...]
            frame = [0x07, dlc] + list(service_bytes)
            while len(frame) < 8:
                frame.append(0x00)
            self.send_can(0x6F1, frame)
        else:
            print("[!] Multi-frame request not implemented yet.")

    def clear_dtcs(self):
        print("\n[>>>] Sending Clear DTCs ($14 FF FF FF)...")
        # 0x6F1: [0x07, 0x04, 0x14, 0xFF, 0xFF, 0xFF, 0x00, 0x00]
        self.send_can(0x6F1, [0x07, 0x04, 0x14, 0xFF, 0xFF, 0xFF, 0x00, 0x00])

    def read_dtcs(self):
        print("\n[>>>] Sending Read DTCs ($19 02 FF)...")
        # 0x6F1: [0x07, 0x03, 0x19, 0x02, 0xFF, 0x00, 0x00, 0x00]
        self.send_can(0x6F1, [0x07, 0x03, 0x19, 0x02, 0xFF, 0x00, 0x00, 0x00])

    def read_vin(self):
        print("\n[>>>] Sending Read VIN ($22 F1 90)...")
        self.send_can(0x6F1, [0x07, 0x03, 0x22, 0xF1, 0x90, 0x00, 0x00, 0x00])

    def stop_balancing(self):
        print("\n[>>>] Sending Stop Latched Balancing Routine ($31 02 AD 6B)...")
        self.send_can(0x6F1, [0x07, 0x04, 0x31, 0x02, 0xAD, 0x6B, 0x00, 0x00])

    def hard_reset(self):
        print("\n[>>>] Sending SME Hard Reset ($11 01)...")
        self.send_can(0x6F1, [0x07, 0x02, 0x11, 0x01, 0x00, 0x00, 0x00, 0x00])

    def handle_rx_frame(self, can_id, dlc, data):
        # Ignore our own broadcast IDs
        if can_id in (0x10B, 0x12F, 0x53A, 0x3A0, 0x328, 0x3CA, 0x3E8, 0x2CA, 0x6F1, 0x554):
            return

        # SME Broadcast 0x112 (Status of HV Battery - 20ms)
        now = time.time()
        if can_id == 0x112 and dlc >= 4:
            if now - self.last_112_print >= 2.0: # Print once every 2 seconds
                self.last_112_print = now
                pack_dv = data[2] | (data[3] << 8)
                voltage_v = pack_dv * 0.1
                print(f"[SME TELEMETRY 0x112] HV Voltage: {voltage_v:.1f} V | Raw: {data.hex().upper()}")
            return

        # SME Broadcast 0x432 (SOC Info - 200ms)
        if can_id == 0x432 and dlc >= 5:
            if now - self.last_432_print >= 2.0:
                self.last_432_print = now
                soc_raw = data[4]
                soc_pct = soc_raw * 0.5
                mode = data[0] & 0x03
                print(f"[SME TELEMETRY 0x432] SOC: {soc_pct:.1f}% | Mode: {mode} | Raw: {data.hex().upper()}")
            return

        # SME UDS Response on 0x607
        if can_id == 0x607:
            # Extended addressing: data[0] = 0xF1 (tester target)
            pci = data[1]
            frame_type = (pci & 0xF0) >> 4

            if frame_type == 0: # Single Frame
                sf_len = pci & 0x0F
                payload = data[2:2 + sf_len]
                self.parse_uds_response(payload)

            elif frame_type == 1: # First Frame
                ff_len = ((pci & 0x0F) << 8) | data[2]
                self.uds_expected_len = ff_len
                self.uds_buffer = bytearray(data[3:8])
                self.uds_in_progress = True
                # Send Flow Control (ContinueToSend, BlockSize=0, STmin=0)
                # 0x6F1: [0x07, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
                self.send_can(0x6F1, [0x07, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])

            elif frame_type == 2: # Consecutive Frame
                if self.uds_in_progress:
                    self.uds_buffer.extend(data[2:8])
                    if len(self.uds_buffer) >= self.uds_expected_len:
                        full_payload = self.uds_buffer[:self.uds_expected_len]
                        self.uds_in_progress = False
                        self.uds_buffer.clear()
                        self.parse_uds_response(full_payload)
            return

        # Other SME frames (only print if verbose is enabled)
        if self.verbose:
            print(f"[SME CAN 0x{can_id:03X}] DLC: {dlc} DATA: {data.hex()}")

    def parse_uds_response(self, payload):
        hex_str = payload.hex().upper()
        if not payload:
            return
        sid = payload[0]
        if sid == 0x7F: # Negative Response
            req_sid = payload[1] if len(payload) > 1 else 0
            nrc = payload[2] if len(payload) > 2 else 0
            if nrc == 0x78:
                print(f"[UDS 0x607] NRC 0x78: ResponsePending (ECU processing request 0x{req_sid:02X})...")
                return
            nrc_names = {
                0x11: "ServiceNotSupported",
                0x12: "SubFunctionNotSupported",
                0x13: "IncorrectMessageLengthOrInvalidFormat",
                0x22: "ConditionsNotCorrect",
                0x31: "RequestOutOfRange",
                0x33: "SecurityAccessDenied",
                0x35: "InvalidKey",
                0x36: "ExceedNumberOfAttempts",
                0x7E: "SubFunctionNotSupportedInActiveSession"
            }
            nrc_str = nrc_names.get(nrc, f"NRC 0x{nrc:02X}")
            print(f"\n[!] UDS NEGATIVE RESPONSE: SID 0x{req_sid:02X} -> {nrc_str} (0x{nrc:02X})")
            return

        if sid == 0x54: # Positive Clear DTC
            print("\n[+] UDS SUCCESS: DTCs Cleared successfully ($54)!")
            return

        if sid == 0x59: # Positive Read DTC
            sub = payload[1] if len(payload) > 1 else 0
            status_mask = payload[2] if len(payload) > 2 else 0
            dtc_data = payload[3:]
            print_dtc_report(dtc_data, status_mask, show_all=False)
            return

        if sid == 0x62: # Positive ReadDataByIdentifier
            did = (payload[1] << 8) | payload[2] if len(payload) > 2 else 0
            data_val = payload[3:]
            print(f"\n[+] UDS DATA IDENTIFIER 0x{did:04X}:")
            if did == 0xF190: # VIN
                vin_ascii = "".join(chr(b) if 32 <= b <= 126 else "." for b in data_val)
                print(f"    VIN: {vin_ascii} (Hex: {data_val.hex().upper()})")
            elif did == 0xDDB4: # Pre-contactor voltage
                if len(data_val) >= 2:
                    v_raw = (data_val[0] << 8) | data_val[1]
                    print(f"    Pre-contactor Voltage: {v_raw * 0.1:.1f} V")
            else:
                print(f"    Data: {data_val.hex().upper()}")
            return

        if sid == 0x71: # Positive RoutineControl
            routine_type = payload[1] if len(payload) > 1 else 0
            rid = (payload[2] << 8) | payload[3] if len(payload) > 3 else 0
            print(f"\n[+] UDS ROUTINE CONTROL: Type {routine_type} RID 0x{rid:04X} -> Success! Data: {payload[4:].hex().upper()}")
            return

        print(f"\n[+] UDS POSITIVE RESPONSE: SID 0x{sid:02X} DATA: {hex_str}")

def main():
    ports = glob.glob("/dev/cu.usbmodem*") + glob.glob("/dev/tty.usbmodem*") + glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")
    port = sys.argv[1] if len(sys.argv) > 1 else (ports[0] if ports else "/dev/cu.usbmodem3101")

    sme = SMEMaster(port)
    sme.open()

    print("==========================================================")
    print("      BMW SME BK2 AS4 MASTER CONTROLLER & DIAGNOSTIC     ")
    print("==========================================================")
    
    # Pulse 100k wakeup
    sme.pulse_100k_wakeup()

    # Launch cyclic keepalive in background thread
    tx_thread = threading.Thread(target=sme.start_keepalive_loop, daemon=True)
    tx_thread.start()

    print("\nCommands:")
    print("  c : Clear DTCs (0x14 FF FF FF)")
    print("  d : Read DTCs (0x19 02 0C)")
    print("  v : Read VIN (0x22 F1 90)")
    print("  b : Stop Latched Balancing Routine (0x31 02 AD 6B)")
    print("  r : Hard Reset SME (0x11 01)")
    print("  w : Re-send 100k Wakeup Pulse")
    print("  m : Toggle Raw CAN Frame Stream (currently OFF)")
    print("  q : Quit\n")

    # Serial receive loop
    buf = bytearray()
    last_prompt = 0
    
    # Start keyboard listener in separate thread
    def input_loop():
        while sme.running:
            try:
                line = sys.stdin.readline().strip()
                if not line:
                    continue
                cmd = line.lower()
                if cmd == 'c':
                    sme.clear_dtcs()
                elif cmd == 'd':
                    sme.read_dtcs()
                elif cmd == 'v':
                    sme.read_vin()
                elif cmd == 'b':
                    sme.stop_balancing()
                elif cmd == 'r':
                    sme.hard_reset()
                elif cmd == 'w':
                    sme.pulse_100k_wakeup()
                elif cmd == 'm':
                    sme.verbose = not sme.verbose
                    print(f"\n[*] Raw CAN stream {'ENABLED' if sme.verbose else 'MUTED'}.")
                elif cmd == 'q':
                    sme.running = False
                    break
            except Exception as e:
                break

    in_thread = threading.Thread(target=input_loop, daemon=True)
    in_thread.start()

    try:
        while sme.running:
            chunk = sme.ser.read(128)
            if chunk:
                for b in chunk:
                    if b in (ord('\r'), ord('\n')):
                        if buf:
                            line_str = buf.decode('ascii', errors='ignore')
                            buf.clear()
                            for part in line_str.split('t'):
                                if len(part) >= 4:
                                    try:
                                        cid = int(part[0:3], 16)
                                        dlc = int(part[3])
                                        data_bytes = bytes.fromhex(part[4:4 + dlc * 2])
                                        sme.handle_rx_frame(cid, dlc, data_bytes)
                                    except Exception as e:
                                        pass
                    else:
                        buf.append(b)
            else:
                time.sleep(0.005)
    except KeyboardInterrupt:
        pass
    finally:
        sme.running = False
        time.sleep(0.1)
        with sme.lock:
            sme.ser.write(b"C\r")
            sme.ser.close()
        print("\n[*] Closed CANable and exited cleanly.")

if __name__ == "__main__":
    main()
