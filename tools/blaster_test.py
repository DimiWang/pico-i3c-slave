#!/usr/bin/env python3
"""
End-to-end test of pico-i3c-slave driven by an I3CBlaster
(https://github.com/xyphro/I3CBlaster).

It talks to the I3CBlaster's USB serial port with the same "@command" text
protocol that I3CBlaster's own python/i3cblaster.py module uses (one command
per line, answer "OK(0),0x..,0x.."), but is self-contained so it also works on
Linux/macOS and avoids two quirks of that module (see README).

    pip install pyserial
    python3 tools/blaster_test.py                 # auto-detect the blaster
    python3 tools/blaster_test.py --port /dev/ttyACM0 --clk 1000

Exit code 0 = all checks passed.
SPDX-License-Identifier: MIT
"""
import argparse
import sys
import time

import serial
import serial.tools.list_ports

PICO_VID, PICO_PID = 0x2E8A, 0x000A      # pico-sdk CDC (blaster *and* target!)
TARGET_PRODUCT = "pico-i3c-slave"        # USB product string of this firmware

STATIC_ADDR = 0x42
EXPECTED_ID = [0xFF, 0xFE, 0x3C, 0x01, 0x00, 0x01, 0x01, 0x00]  # PID, BCR, DCR


def find_blaster():
    """Return the first pico-sdk CDC port that is not this target firmware."""
    for p in serial.tools.list_ports.comports():
        if p.vid == PICO_VID and p.pid == PICO_PID and (p.product or "") != TARGET_PRODUCT:
            return p.device
    return None


class Blaster:
    def __init__(self, port):
        self.ser = serial.Serial(port, 115200, timeout=2)
        time.sleep(0.5)
        self.ser.reset_input_buffer()          # drop the welcome banner

    def cmd(self, line):
        """Send one I3CBlaster command, return (status, [values])."""
        self.ser.reset_input_buffer()
        self.ser.write(("@" + line + "\r").encode("ascii"))
        resp = self.ser.readline().decode("ascii", "replace").strip()
        status, _, rest = resp.partition(")")
        status += ")"
        vals = [int(v, 0) for v in rest.strip(",").split(",") if v.strip()]
        print(f"  > {line:45s} {resp}")
        return status, vals


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", help="I3CBlaster serial port (default: auto)")
    ap.add_argument("--clk", type=int, default=1000, help="I3C SCL in kHz (default 1000)")
    ap.add_argument("--da", type=lambda s: int(s, 0), default=0x30, help="DA for ENTDAA")
    args = ap.parse_args()

    port = args.port or find_blaster()
    if not port:
        sys.exit("I3CBlaster not found - use --port")
    print(f"I3CBlaster on {port}")
    b = Blaster(port)
    fails = 0

    def check(cond, what):
        nonlocal fails
        print(("  PASS " if cond else "  FAIL ") + what)
        fails += 0 if cond else 1

    da = args.da
    check(b.cmd(f"i3c_clk {args.clk}")[0] == "OK(0)", f"SCL = {args.clk} kHz")

    print("RSTDAA")
    check(b.cmd("i3c_rstdaa")[0] == "OK(0)", "RSTDAA")
    st, v = b.cmd("i3c_scan")
    check(STATIC_ADDR in v and da not in v, f"scan sees static 0x{STATIC_ADDR:02X} only")

    print("ENTDAA")
    st, v = b.cmd(f"i3c_entdaa 0x{da:02x}")
    check(st == "OK(0)" and v == EXPECTED_ID, "ENTDAA returns PID FFFE3C010001, BCR 01, DCR 00")
    st, v = b.cmd("i3c_scan")
    check(da in v, f"scan sees DA 0x{da:02X}")

    print("GET CCCs")
    st, v = b.cmd(f"i3c_sdr_ccc_direct_read 0x{da:02x} 0x8d 6")
    check(v == EXPECTED_ID[:6], "GETPID")
    st, v = b.cmd(f"i3c_sdr_ccc_direct_read 0x{da:02x} 0x8e 1")
    check(v == [0x01], "GETBCR")
    st, v = b.cmd(f"i3c_sdr_ccc_direct_read 0x{da:02x} 0x8f 1")
    check(v == [0x00], "GETDCR")
    st, v = b.cmd(f"i3c_sdr_ccc_direct_read 0x{da:02x} 0x90 2")
    check(st == "OK(0)" and len(v) == 2, "GETSTATUS")

    print("SDR private write + read back")
    data = [0xDE, 0xAD, 0xBE, 0xEF, 0x01, 0x02, 0x03, 0x04]
    payload = ",".join(f"0x{x:02x}" for x in [0x10] + data)
    check(b.cmd(f"i3c_sdr_write 0x{da:02x} {payload}")[0] == "OK(0)", "write 8 bytes at 0x10")
    st, v = b.cmd(f"i3c_sdr_writeread 0x{da:02x} 0x10 8")
    check(v == data, "writeread 8 bytes from 0x10")
    b.cmd(f"i3c_sdr_write 0x{da:02x} 0x10")           # pointer only
    st, v = b.cmd(f"i3c_sdr_read 0x{da:02x} 4")
    check(v == data[:4], "pointer write + plain read")

    print("RSTDAA + SETDASA")
    b.cmd("i3c_rstdaa")
    new = da + 1
    st, _ = b.cmd(f"i3c_sdr_ccc_direct_write 0x{STATIC_ADDR:02x} 0x87 0x{new << 1:02x}")
    check(st == "OK(0)", f"SETDASA 0x{STATIC_ADDR:02X} -> 0x{new:02X}")
    st, v = b.cmd(f"i3c_sdr_ccc_direct_read 0x{new:02x} 0x8d 6")
    check(v == EXPECTED_ID[:6], f"GETPID at 0x{new:02X}")

    print("SETNEWDA")
    newer = new + 1
    st, _ = b.cmd(f"i3c_sdr_ccc_direct_write 0x{new:02x} 0x88 0x{newer << 1:02x}")
    st, v = b.cmd("i3c_scan")
    check(newer in v and new not in v, f"SETNEWDA 0x{new:02X} -> 0x{newer:02X}")

    b.cmd("i3c_rstdaa")
    print(f"\n{'ALL PASSED' if fails == 0 else f'{fails} CHECK(S) FAILED'}")
    sys.exit(1 if fails else 0)


if __name__ == "__main__":
    main()
