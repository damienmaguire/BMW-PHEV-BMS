"""
BMW SME BK2 AS4 Diagnostic Trouble Code (DTC) Definitions & Parser
"""

DTC_DESCRIPTIONS = {
    # High-Voltage Safety & Interlock
    "21F148": "HV Interlock Loop (HVIL): Interruption / Open Circuit (Pins 3/8)",
    "21F1F6": "HV Battery Contactors: Protective Switch-off after fault detected",
    "21F009": "HV Battery Negative Terminal: Contactor driver / ground line fault",
    "21F012": "Internal HV Power Supply: Voltage drop / main switch anomaly",
    "21F014": "Power Supply CSC: Short circuit / open line to Cell Supervision Circuits",
    "21F02D": "HV Battery Current Sensor / Pyrotechnic Disconnect: Implausible reading",
    "21F222": "Safety Box (S-Box): Line disconnection to voltage/current sensor",
    "21F22D": "SME Safety Concept: Shut-off path test failed",
    "21F22E": "HV Battery Safety: Disconnect / ripcord line state monitoring fault",
    "21F22F": "CSC Safety Circuit: Pyrofuse / ripcord PWM signal not OK",
    "21F37E": "CRASH LOCKOUT: Pyrofuse deployed / persistent impact shutdown latch",

    # Cell Modules 1-6 (CSC) Voltage Implausible / Missing
    "21F161": "Cell Module 01 (CSC 1): Voltage sensors implausible / no communication",
    "21F162": "Cell Module 02 (CSC 2): Voltage sensors implausible / no communication",
    "21F163": "Cell Module 03 (CSC 3): Voltage sensors implausible / no communication",
    "21F164": "Cell Module 04 (CSC 4): Voltage sensors implausible / no communication",
    "21F165": "Cell Module 05 (CSC 5): Voltage sensors implausible / no communication",
    "21F166": "Cell Module 06 (CSC 6): Voltage sensors implausible / no communication",

    # Cell Modules 1-6 (CSC) Temperature / Shutdown Path Tests
    "21F04D": "Cell Module 01 (CSC 1): Temperature sensor / low-side shutdown test failed",
    "21F04E": "Cell Module 02 (CSC 2): Temperature sensor / shutdown path test K1 failed",
    "21F04F": "Cell Module 03 (CSC 3): Temperature sensor / shutdown path test K2 failed",
    "21F050": "Cell Module 04 (CSC 4): Temperature sensor out of range / insulation threshold",
    "21F051": "Cell Module 05 (CSC 5): Temperature sensor reporting below nominal range",
    "21F052": "Cell Module 06 (CSC 6): Temperature sensor reporting below nominal range",
    "21F00F": "CSC Internal Network: Control module bus communication invalid",

    # Protective Throttling & Derating
    "21F182": "State of Charge: Critical upper SoC limit reached / calculation limited",
    "21F183": "HV Electrical System: Voltage unsuitable for closing switching contactors",
    "21F184": "HV Electrical System: Internal insulation resistance below warning threshold",
    "21F185": "State of Charge: SoC implausible due to missing cell data",
    "21F186": "Cell Monitoring: Cells severely unbalanced / CSC 3 temp anomaly",
    "21F187": "Cell Monitoring: Cell defect detected / CSC 4 temp anomaly",
    "21F188": "Power Throttling: Discharge current limit reduced to 0 A (protective mode)",
    "21F189": "Power Throttling: Charge current limit reduced to 0 A (protective mode)",
    "21F25A": "HV Battery Safety: Power limit triggered by cell temp/voltage irregularity",
    "21F159": "HV Battery Telemetry: Measurement tracking line 1 disconnected",
    "21F15B": "HV Battery Telemetry: Measurement tracking line 2 disconnected",

    # Vehicle Bus Missing Messages (CAD4xx)
    "CAD400": "Missing CAN Message: BDC (Vehicle state / network management)",
    "CAD401": "Missing CAN Message: KOMBI (Ambient temperature - ID 0x2CA)",
    "CAD402": "Missing CAN Message: KOMBI (Relative time clock - ID 0x328)",
    "CAD403": "Missing CAN Message: KOMBI (Display data)",
    "CAD404": "Missing CAN Message: DSC (Wheel speed / stability)",
    "CAD405": "Missing CAN Message: EME (OBD diagnosis / engine control - ID 0x3E8)",
    "CAD407": "Missing CAN Message: BDC (Terminal status / KL15 - ID 0x12F)",
    "CAD408": "Missing CAN Message: BDC (Vehicle condition - ID 0x3A0)",
    "CAD409": "Missing CAN Message: IHKA (Air conditioning / battery cooling request)",
    "CAD40A": "Missing CAN Message: DSC (Vehicle speed - ID 0x1A1)",
    "CAD40B": "Missing CAN Message: KOMBI (Secondary status)",
    "CAD40C": "Missing CAN Message: DSC (Brake pedal / standstill)",
    "CAD411": "Missing CAN Message: EME (Electric motor electronics status)",
    "CAD413": "Missing CAN Message: EME (HV battery specification - ID 0x433)",
    "CAD415": "Missing CAN Message: EME (Contactor state announcement - ID 0x53A)",
    "CAD416": "Missing CAN Message: EME (Contactor driver - ID 0x10B)",
    "CAD421": "Missing CAN Message: BDC (Central gateway)",
    "CAD426": "Missing CAN Message: DSC (Longitudinal acceleration)",
    "CAD428": "Missing CAN Message: KOMBI (Range / instrument cluster)",
    "CAD429": "Missing CAN Message: KOMBI (Driving forecast - ID 0x3CA)",

    # Network Management & Hardware Watchdogs
    "CAC47C": "K-CAN5: Communication fault / transceiver bus timeout",
    "CAC486": "K-CAN5: Network management node monitoring timeout",
    "CACBFF": "Internal Bus: Communication timeout / missing internal master heartbeat",
}

def decode_dtc(dtc_hex):
    dtc_hex = dtc_hex.upper()
    if dtc_hex in DTC_DESCRIPTIONS:
        return DTC_DESCRIPTIONS[dtc_hex]
    
    # Generic category heuristics
    if dtc_hex.startswith("CAD4"):
        return f"Missing CAN message (Vehicle Bus - Node 0x{dtc_hex[4:]})"
    if dtc_hex.startswith("21F0"):
        return "CSC / Internal SME Measurement Sensor Circuit fault"
    if dtc_hex.startswith("21F1"):
        return "High-Voltage Battery Cell / Contactor Supervision fault"
    if dtc_hex.startswith("21F2"):
        return "High-Voltage Safety Concept / Thermal Management fault"
    return "BMW Manufacturer Diagnostic Trouble Code"

def decode_status(status_byte):
    active = bool(status_byte & 0x01)
    confirmed = bool(status_byte & 0x08)
    failed_cycle = bool(status_byte & 0x02)
    failed_clear = bool(status_byte & 0x20)
    
    if active and confirmed:
        return "ACTIVE (Confirmed Failure)"
    elif active:
        return "ACTIVE (Unconfirmed)"
    elif confirmed and failed_cycle:
        return "PENDING/CONFIRMED (Cycle)"
    elif status_byte == 0x50:
        return "Not Tested Since Clear"
    elif failed_clear:
        return "Previously Failed (Healed)"
    else:
        return f"Status(0x{status_byte:02X})"

def print_dtc_report(dtc_data, status_mask, show_all=False):
    num_dtcs = len(dtc_data) // 4
    active_dtcs = []
    inactive_dtcs = []
    
    for i in range(num_dtcs):
        rec = dtc_data[i*4 : (i+1)*4]
        dtc_num = (rec[0] << 16) | (rec[1] << 8) | rec[2]
        dtc_hex = f"{dtc_num:06X}"
        status = rec[3]
        desc = decode_dtc(dtc_hex)
        status_text = decode_status(status)
        is_active = bool(status & 0x01)
        is_confirmed = bool(status & 0x08)
        
        entry = (dtc_hex, status, status_text, desc, is_active, is_confirmed)
        if is_active or is_confirmed:
            active_dtcs.append(entry)
        else:
            inactive_dtcs.append(entry)
            
    print(f"\n==========================================================================================")
    print(f"                      BMW SME DIAGNOSTIC TROUBLE CODE (DTC) REPORT                       ")
    print(f"==========================================================================================")
    print(f" Availability Mask: 0x{status_mask:02X} | Total DTC Records in SME Memory: {num_dtcs}")
    print(f" Active / Confirmed Faults: {len(active_dtcs)} | Inactive / Untested: {len(inactive_dtcs)}\n")
    
    if active_dtcs:
        print("--- ACTIVE & CONFIRMED BENCH FAULTS ---")
        for i, (dtc_hex, status, status_text, desc, is_active, is_confirmed) in enumerate(active_dtcs):
            marker = "🔴" if is_active else "🟡"
            print(f"  {marker} [{i+1:2d}] 0x{dtc_hex} (Status: 0x{status:02X}) | {status_text:24s} | {desc}")
    else:
        print("  ✅ NO ACTIVE FAULTS DETECTED! (All fault memories clear)")
        
    if show_all and inactive_dtcs:
        print("\n--- INACTIVE / UNTESTED CODES ---")
        for i, (dtc_hex, status, status_text, desc, _, _) in enumerate(inactive_dtcs):
            print(f"  ⚪ [{i+1:3d}] 0x{dtc_hex} (Status: 0x{status:02X}) | {status_text:24s} | {desc}")
    elif inactive_dtcs:
        print(f"\n  [i] {len(inactive_dtcs)} inactive / untested monitor entries omitted.")
        
    print(f"==========================================================================================\n")
