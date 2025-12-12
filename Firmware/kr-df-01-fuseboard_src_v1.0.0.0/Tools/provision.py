#
# Usage:
# py provision.py [STM32 Cube Programmer] [Serial number] [HW revision]
#

import sys
import os
import subprocess
import re

# Constants
OTP_BASE_ADDR = 0x08FFF000

# Connection arguments
CONN_ARGS = ["-c port=swd", "-q", "-vb 1"]

# Will be filled
STM32_PROG = ""

# ----------------------------------------------------------------------------------------------------------------------

# Error print function
def eprint(*args, **kwargs):
    print(*args, file=sys.stderr, **kwargs)

# Parse arguments
def parse_args(argv: []) -> (str, int, int):

    # Check arguments
    if len(argv) < 2:
        eprint("Provide path to STM32 programmer CLI executable")
        exit(-1)

    if len(argv) < 3:
        eprint("Provide Fuseboard serial number")
        exit(-1)

    if len(argv) < 4:
        eprint("Provide Fuseboard HW revision nmumber")
        exit(-1)

    # Extract arguments
    ser_arg = argv[2]
    hwr_arg = argv[3]

    # Check that CLI exists
    if not os.path.exists(argv[1]):
        eprint("STM32 programmer CLI executable not found")
        exit(-1)

    # Check serial number
    if not ser_arg.isdecimal() or int(ser_arg) < 1 or int(ser_arg) > 10_000_000:
        eprint("Fuseboard serial number is not valid")
        exit(-1)

    # Check HW revision number
    if not hwr_arg.isdecimal() or int(hwr_arg) < 1 or int(hwr_arg) > 100:
        eprint("Fuseboard HW revision number is not valid")
        exit(-1)

    # Convert arguments
    return argv[1], int(ser_arg), int(hwr_arg)

# Programmer command
def prog_cmd(args: [str]) -> (int, str):

    call_args = [STM32_PROG] + CONN_ARGS + args
    proc = subprocess.Popen(call_args, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    (out_bytes, err_bytes) = proc.communicate()

    out_txt = out_bytes.decode(errors="ignore") if out_bytes else ""
    err_txt = err_bytes.decode(errors="ignore") if err_bytes else ""
    if proc.returncode != 0:
        if err_txt:
            eprint(err_txt)
        else:
            eprint(out_txt)

    return proc.returncode, out_txt

# Check that correct MCU is connected
def check_connection(mcu_name: str) -> bool:

    code, out = prog_cmd([])
    if code != 0:
        eprint("Fuseboard SWD connection failed")
        return False

    # Parse device name and make sure correct MCU is connected
    match = re.search(r"^\s*Device name\s*:\s*([\w/]+)\s*$", out, re.MULTILINE)
    if not match:
        eprint("Could not parse MCU device name from output")
        return False

    dev_name = match.group(1)
    if dev_name != mcu_name:
        eprint(f"Invalid MCU found: {dev_name}")
        return False

    return True

# Write serial number and HW revision into OTP memory
def write_otp(serial: int, hw_rev: int) -> bool:

    # Read OTP to make sure device is not already programmed
    code, out = prog_cmd([f"-r16 0x{OTP_BASE_ADDR:08X} 4"])
    if code != 0:
        eprint("Reading OTP memory failed")
        return False

    match = re.search(rf"^\s*0x{OTP_BASE_ADDR:08X}\s*:\s*(\w+)\s+(\w+)\s*$", out, re.MULTILINE)
    if not match:
        eprint("Could not parse OTP content")
        return False

    if match.group(1) != "FFFF" or match.group(2) != "FFFF":
        eprint(f"Fuseboard MCU OTP memory is already written: {match.group(1)} {match.group(2)}")
        return False

    # Create OTP bytes as 16-bit little-endian half-words in hexadecimal format
    otp_w0 = f"{((serial >> 8) & 0xFF):02X}{((serial >> 16) & 0xFF):02X}"
    otp_w1 = f"{(hw_rev & 0xFF):02X}{(serial & 0xFF):02X}"

    # Write OTP bytes with two half-words and verify
    # STM32 Cube programmer requires 0x prefix on writing
    code, out = prog_cmd([f"-w16 0x{OTP_BASE_ADDR:08X} 0x{otp_w0} 0x{otp_w1}", "-v"])
    if code != 0:
        eprint("Error writing OTP memory")
        return False

    print(f"Fuseboard serial number {serial} and HW revision {hw_rev} written successfully!")
    return True

def config_ob() -> bool:
    code, out = prog_cmd(["-ob EDATA1_EN=0x1 EDATA1_STRT=0x7 EDATA2_EN=0x1 EDATA2_STRT=0x7"])
    if code != 0:
        eprint("Error writing option bytes")
        return False

    print(f"Fuseboard option bytes written successfully!")
    return True

# ----------------------------------------------------------------------------------------------------------------------

# Check arguments
STM32_PROG, serial, hw_rev = parse_args(sys.argv)

# Check connection
if not check_connection("STM32H56x/573"):
    exit(-1)

# Write OTP memory
write_otp(serial, hw_rev)

# Configure option bytes (OB)
config_ob()
