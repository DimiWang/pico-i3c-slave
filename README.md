# pico-i3c-slave

**A Raspberry Pi Pico (RP2040) running as an I3C SDR *target* (slave), made for testing an [I3CBlaster](https://github.com/xyphro/I3CBlaster) controller.**

The RP2040 has no I3C hardware, so the bit level runs on two PIO state machines and the protocol runs on core 1. Core 0 prints every CCC and SDR transfer it sees to USB serial, so you can watch exactly what the blaster sent.

| | |
|---|---|
| Pins | **GPIO16 = SDA, GPIO17 = SCL** (same as the I3CBlaster), 3.3 V, common GND |
| Dynamic address | **ENTDAA**, **SETDASA**, **SETNEWDA**, **SETAASA**, **RSTDAA** (broadcast and direct) |
| Other CCCs | GETPID, GETBCR, GETDCR, GETSTATUS, GETMWL, GETMRL, GETMXDS, ENEC/DISEC, SETMWL/SETMRL |
| Transfers | SDR private write and read (8 data bits + T-bit) on a 256-byte register memory; legacy I2C on the static address |
| Logging | USB CDC: CCC code, address, direction, payload hex, DA changes, errors |
| Firmware | [`bin/pico_i3c_slave.uf2`](bin/pico_i3c_slave.uf2), ready to drag and drop |

> **Status:** this firmware builds cleanly with pico-sdk 2.1.1, and the real PIO programs and protocol code pass a bit-level simulation ([`test/sim`](test/sim)) that replays I3CBlaster-style transfers. **It has not yet been tested on a real bus.** If something misbehaves, the USB log and the [Limitations](#known-limitations) section are the first places to look. Issues and logic-analyzer traces are welcome.

---

## 1. Wiring

```
   I3CBlaster Pico (controller)              Target Pico (this firmware)
   +----------------------+                 +----------------------+
   | GPIO16 (SDA) --------+---------+-------+ GPIO16 (SDA)         |
   | GPIO17 (SCL) --------+-----+---|-------+ GPIO17 (SCL)         |
   | GND  ----------------+-----|---|-------+ GND                  |
   | 3V3(OUT) --+--[2k2]--------+   |       |                      |
   |            +--[2k2]------------+       |  (no extra pull-ups) |
   +----------------------+                 +----------------------+
        USB to PC                                USB to PC (log)
```

| I3CBlaster Pico | Target Pico | Notes |
|---|---|---|
| GPIO16 / pin 21 (SDA) | GPIO16 / pin 21 (SDA) | |
| GPIO17 / pin 22 (SCL) | GPIO17 / pin 22 (SCL) | the target never drives SCL |
| GND (for example pin 23) | GND (for example pin 23) | **required**, keep it short. Use two GND wires if you can |
| 3V3(OUT) pin 36 → 2.2 kΩ → GPIO16 and 2.2 kΩ → GPIO17 | – | the pull-ups the I3CBlaster README asks for |

* **Do not add a second pair of pull-ups on the target.** The I3CBlaster's 2.2 kΩ pull-ups are enough, and the firmware turns off the RP2040's internal pulls on GPIO16 and GPIO17.
* **3.3 V only.** Both boards run their IO at 3.3 V. Never connect 5 V or VBUS to SDA or SCL.
* **A shared ground is required.** If both Picos are plugged into the same PC, they already share USB ground, but still run a GND wire right next to SDA and SCL. The I3CBlaster README shows how badly a single long ground wire can ring.
* Power each Pico from its own USB cable. The target's USB port is also its log console. Do not connect the two 3V3 pins together.
* Keep the wires short (under 10 cm) and run at **1 MHz or slower** (see [Speed](#speed)).

## 2. Flash the firmware (no toolchain needed)

1. Download [`bin/pico_i3c_slave.uf2`](bin/pico_i3c_slave.uf2).
2. Hold **BOOTSEL** on the *target* Pico while you plug in USB. A drive called `RPI-RP2` appears.
3. Drag `pico_i3c_slave.uf2` onto that drive. The Pico reboots into the firmware.
4. Open its USB serial port (any baud rate) with PuTTY, Tera Term, `screen /dev/ttyACM1`, `minicom`, or `python -m serial.tools.miniterm`. You should see:

```
=== pico-i3c-slave v1.0.0 : RP2040 I3C SDR target ===
SDA=GPIO16 SCL=GPIO17  sys_clk=200000 kHz
PID=FFFE3C010001 BCR=0x01 DCR=0x00 static=0x42 dynamic=none
Type 'help' for console commands. Bus events follow:
```

> Both the I3CBlaster and this firmware use the standard pico-sdk USB ID **2E8A:000A**, so you will see two serial ports. This firmware's port reports the USB product string **`pico-i3c-slave`**. On Linux, `ls -l /dev/serial/by-id/` shows which port is which.

## 3. Build from source

Requirements: CMake ≥ 3.13, `arm-none-eabi-gcc`, and the [Pico SDK](https://github.com/raspberrypi/pico-sdk) (2.1.x recommended; 200 MHz on RP2040 needs SDK ≥ 2.1). You also need a host C/C++ compiler for pioasm and picotool.

```sh
git clone https://github.com/DimiWang/pico-i3c-slave.git
cd pico-i3c-slave
export PICO_SDK_PATH=/path/to/pico-sdk        # or add -DPICO_SDK_FETCH_FROM_GIT=ON
cmake -B build -DPICO_BOARD=pico
cmake --build build -j
# -> build/pico_i3c_slave.uf2
```

Compile-time options (see [`src/config.h`](src/config.h)):

| CMake option | Default | Meaning |
|---|---|---|
| `-DI3C_SDA_PIN=16` | 16 | SDA pin. **SCL is always SDA+1** (17) |
| `-DI3C_STATIC_ADDR=0x42` | 0x42 | static address |
| `-DI3C_PID=0xFFFE3C010001` | see below | 48-bit provisional ID |
| `-DI3C_BCR=0x01` / `-DI3C_DCR=0x00` | 0x01 / 0x00 | BCR / DCR |
| `-DI3C_RESPOND_ON_STATIC_ADDR=1` | 1 | answer private SDR transfers on the static address while no dynamic address is set |
| `-DI3C_LEGACY_I2C=1` | 1 | answer legacy I2C transfers on the static address while no dynamic address is set |
| `-DI3C_SYS_CLK_KHZ=200000` | 200000 | system clock (core voltage is raised to 1.15 V for >133 MHz). Use `125000` for the stock clock |

Simulation (host only, no hardware needed; uses the `pioasm` built in `build/` by the step above, or one on your PATH):

```sh
make -C test/sim          # runs the protocol test suite against the real PIO programs
make -C test/sim sweep    # pass/fail for a range of SCL speeds and CPU reaction times
```

## 4. Device identity

| Field | Value | Notes |
|---|---|---|
| **PID** (48 bit) | **`0xFFFE3C010001`**, sent as `FF FE 3C 01 00 01` | `[47:33]` manufacturer `0x7FFF` (a placeholder, **not an assigned MIPI ID**), `[32]` = 0 (fixed, not random), `[31:16]` part `0x3C01`, `[15:12]` instance 0, `[11:0]` `0x001` |
| **BCR** | **`0x01`** | I3C target; **no IBI** (`BCR[1]=0`), no IBI payload, not offline-capable, not virtual, no advanced capabilities; `BCR[0]=1` = has a speed limit (answers GETMXDS). **Hot-Join is not supported** |
| **DCR** | **`0x00`** | generic device |
| **Static address** | **`0x42`** | used by SETDASA and SETAASA |
| Dynamic address used in the examples | ENTDAA → **`0x30`**, SETDASA → **`0x31`**, SETNEWDA → **`0x32`** | your choice; the controller picks it |
| Memory | 256 bytes, filled with `00 01 02 … FF` at boot | first written byte = register pointer |
| MWL / MRL | 256 / 256 | a read stops itself with T=0 after 256 bytes |

Because the PID is all ones at the top, this target **loses ENTDAA arbitration to any real device** on the same bus and is assigned last.

## 5. Testing against the I3CBlaster

All commands below are the I3CBlaster's own command-line interface, taken from its [README](https://github.com/xyphro/I3CBlaster#commands) and [`src/main.c`](https://github.com/xyphro/I3CBlaster/blob/main/src/main.c). The blaster answers every command with one line, such as `OK(0)` or `OK(0),0x..,0x..`. Addresses are 7-bit. Lists of bytes are separated by commas with **no spaces**.

### 5.1 Open both consoles

* **I3CBlaster:** open its COM/tty port in PuTTY or Tera Term and press **F1** once so its line editor detects the terminal type (from the I3CBlaster README). `help` lists the commands, and `help i3c_sdr_write` shows the arguments for one.
* **Target:** open this firmware's port in a second window to watch the log.

### 5.2 Slow the clock down first

The I3CBlaster starts at **12.5 MHz**, which is much faster than this target can follow. Set it to 1 MHz, or 400 kHz for extra margin:

```
> i3c_clk 1000
OK(0)
```

### 5.3 RSTDAA, then scan

```
> i3c_rstdaa
OK(0)
> i3c_scan
OK(0),0x42,0x7e
```
Before it has a dynamic address, the target answers on its static address 0x42. 0x7E is the broadcast address.

Target log:
```
[   12.003101] CCC  bcast  0x06 RSTDAA
[   12.003101] DAA  RSTDAA: dynamic address cleared (was none)
[   14.511020] SDR  write  addr 0x42 len 0 (address probe, e.g. i3c_scan)
```

### 5.4 ENTDAA: assign a dynamic address and read the PID

```
> i3c_entdaa 0x30
OK(0),0xff,0xfe,0x3c,0x01,0x00,0x01,0x01,0x00
```
The 8 returned bytes are **PID[47:0]** (`ff fe 3c 01 00 01`), then **BCR** (`01`), then **DCR** (`00`): exactly what the target sent during arbitration. Running `i3c_entdaa` a second time returns `ERR_NAKED(3)`, because no unassigned targets are left.

```
> i3c_scan
OK(0),0x30,0x7e
```

Target log:
```
[   20.104230] CCC  bcast  0x07 ENTDAA
[   20.104346] DAA  ENTDAA: dynamic address 0x30 assigned (was none) PID+BCR+DCR: FF FE 3C 01 00 01 01 00
```

### 5.5 GET CCCs (direct read)

Syntax: `i3c_sdr_ccc_direct_read <addr> <ccc[,defining bytes]> <read length>`

```
> i3c_sdr_ccc_direct_read 0x30 0x8d 6        # GETPID
OK(0),0xff,0xfe,0x3c,0x01,0x00,0x01
> i3c_sdr_ccc_direct_read 0x30 0x8e 1        # GETBCR
OK(0),0x01
> i3c_sdr_ccc_direct_read 0x30 0x8f 1        # GETDCR
OK(0),0x00
> i3c_sdr_ccc_direct_read 0x30 0x90 2        # GETSTATUS
OK(0),0x00,0x00
> i3c_sdr_ccc_direct_read 0x30 0x8b 2        # GETMWL (256)
OK(0),0x01,0x00
> i3c_sdr_ccc_direct_read 0x30 0x94 2        # GETMXDS
OK(0),0x04,0x04
```
(Type the commands without the `# …` comments. They are only notes here.)

Target log:
```
[   25.300411] CCC  direct 0x8D GETPID
[   25.300475] CCC  direct 0x8D GETPID    <- addr 0x30 R len 6 data: FF FE 3C 01 00 01
[   26.811020] CCC  direct 0x8E GETBCR
[   26.811040] CCC  direct 0x8E GETBCR    <- addr 0x30 R len 1 data: 01
```
An unsupported direct CCC, such as `i3c_sdr_ccc_direct_read 0x30 0x95 2` (GETCAPS), is NACKed, so the blaster prints `ERR_NAKED(3)`.

### 5.6 SDR private write and read back (round trip)

The first byte of a write sets the register pointer, and the following bytes are stored from there. A read starts at the pointer and increments it.

```
> i3c_sdr_write 0x30 0x10,0xde,0xad,0xbe,0xef
OK(0)
> i3c_sdr_writeread 0x30 0x10 4
OK(0),0xde,0xad,0xbe,0xef
> i3c_sdr_read 0x30 4
OK(0),0x14,0x15,0x16,0x17
```
`i3c_sdr_read` continues where the last read stopped (0x14). Those bytes still hold the boot pattern.

Target log:
```
[   31.000804] SDR  write  addr 0x30 len 5 (ptr=0x10) data: 10 DE AD BE EF
[   33.000841] SDR  write  addr 0x30 len 1 (ptr=0x10) data: 10
[   33.000886] SDR  read   addr 0x30 len 4 (mem[0x10..]) data: DE AD BE EF
[   35.000940] SDR  read   addr 0x30 len 4 (mem[0x14..]) data: 14 15 16 17
```
On the target console, `dump 0x10 16` shows the memory. `fill 0x55` and `pattern` reset it.

### 5.7 RSTDAA + SETDASA (assign the address via the static address)

SETDASA is CCC 0x87, sent directly to the **static** address. Its data byte is **the new dynamic address shifted left by one** (0x31 → `0x62`).
Syntax: `i3c_sdr_ccc_direct_write <addr> <broadcast bytes = ccc> <direct bytes>`

```
> i3c_rstdaa
OK(0)
> i3c_sdr_ccc_direct_write 0x42 0x87 0x62
OK(0)
> i3c_scan
OK(0),0x31,0x7e
> i3c_sdr_ccc_direct_read 0x31 0x8d 6
OK(0),0xff,0xfe,0x3c,0x01,0x00,0x01
```

Target log:
```
[   40.003472] CCC  bcast  0x06 RSTDAA
[   40.003472] DAA  RSTDAA: dynamic address cleared (was 0x30)
[   41.003534] CCC  direct 0x87 SETDASA
[   41.003553] CCC  direct 0x87 SETDASA   -> addr 0x42 W len 1 data: 62
[   41.003553] DAA  SETDASA: dynamic address 0x31 assigned (was none)
```

### 5.8 More CCC examples

```
> i3c_sdr_ccc_direct_write 0x31 0x88 0x64     # SETNEWDA 0x31 -> 0x32
OK(0)
> i3c_sdr_ccc_bc_write 0x00,0x01              # ENEC broadcast, ENINT (stored only)
OK(0)
> i3c_sdr_ccc_bc_write 0x01,0x01              # DISEC broadcast
OK(0)
> i3c_rstdaa
OK(0)
> i3c_sdr_ccc_bc_write 0x29                   # SETAASA: dynamic = static = 0x42
OK(0)
> i3c_sdr_ccc_bc_write 0x06                   # RSTDAA written as a raw broadcast CCC
OK(0)
```

### 5.9 Legacy I2C (optional)

While the target has no dynamic address, it also answers plain I2C on 0x42. This uses the I3CBlaster's I2C commands:

```
> i3c_rstdaa
OK(0)
> i2c_write 0x42 0x40,0x11,0x22,0x33
OK(0)
> i2c_writeread 0x42 0x40 3
OK(0),0x11,0x22,0x33
```

### 5.10 Python

**Self-contained script (recommended, works on Windows, Linux and macOS):** [`tools/blaster_test.py`](tools/blaster_test.py) sends the same `@command` lines as the I3CBlaster Python module, runs every step above and checks the answers:

```sh
pip install pyserial
python3 tools/blaster_test.py                  # finds the blaster's port automatically
python3 tools/blaster_test.py --port COM7 --clk 400
```
```
I3CBlaster on /dev/ttyACM0
  > i3c_clk 1000                                  OK(0)
  PASS SCL = 1000 kHz
RSTDAA
  > i3c_rstdaa                                    OK(0)
  ...
  > i3c_entdaa 0x30                               OK(0),0xff,0xfe,0x3c,0x01,0x00,0x01,0x01,0x00
  PASS ENTDAA returns PID FFFE3C010001, BCR 01, DCR 00
  ...
ALL PASSED
```

**With the I3CBlaster module** ([`python/i3cblaster.py`](https://github.com/xyphro/I3CBlaster/blob/main/python/i3cblaster.py), copy it next to your script):

```python
import i3cblaster

print(i3cblaster.listdevices())      # serial numbers: blaster AND target show up (both 2E8A:000A)
i3c = i3cblaster.i3cblaster()
i3c.init('E6614103E7...')            # optional: pick the blaster by serial number

i3c._exec('i3c_clk 1000')            # see note 1
i3c.i3c_rstdaa()
print(i3c.i3c_scan())                               # [66, 126]  = 0x42, 0x7E
print([hex(x) for x in i3c.i3c_entdaa(0x30)])       # ['0xff','0xfe','0x3c','0x1','0x0','0x1','0x1','0x0']
print(i3c.i3c_sdr_ccc_direct_read(0x30, [0x8d], 6)) # GETPID -> [255, 254, 60, 1, 0, 1]
print(i3c.i3c_sdr_ccc_direct_read(0x30, [0x8e], 1)) # GETBCR -> [1]

i3c.i3c_sdr_write(0x30, [0x10, 0xde, 0xad, 0xbe, 0xef])
print(i3c.i3c_sdr_writeread(0x30, [0x10], 4))       # [222, 173, 190, 239]

i3c.i3c_rstdaa()
i3c._exec('i3c_sdr_ccc_direct_write 0x42 0x87 0x62')   # SETDASA -> 0x31, see note 2
print(i3c.i3c_scan())                               # [49, 126] = 0x31, 0x7E
```

Notes on the I3CBlaster Python module, current at the time of writing. Check the upstream code:
1. `i3c_clk()` sends `i3c_clock …`, which the firmware does not recognise (the command is `i3c_clk`), so it raises an exception. Use `_exec('i3c_clk 1000')` or the terminal instead.
2. `i3c_sdr_ccc_direct_write()` sends `i3c_sdr_ccc_bc_write …` instead of `i3c_sdr_ccc_direct_write …`, so SETDASA and SETNEWDA do not reach the target. Use `_exec(...)` as shown.
3. It encodes strings as `'ansi'`, a codec that exists only on Windows. On Linux and macOS, change `'ansi'` to `'ascii'` or use `tools/blaster_test.py`.
4. Its port auto-detection picks the first 2E8A:000A port, which **might be this target**. Pass the blaster's serial number with `init()`, or unplug the target's USB while you start the script.

## 6. Target USB console

| Command | Action |
|---|---|
| `help` | list commands |
| `status` | dynamic/static address, identity, ENEC mask, MWL/MRL, register pointer, counters (frames, NACKs, parity errors, ENTDAA losses, dropped log lines) |
| `dump [off] [len]` | hex dump of the 256-byte memory |
| `fill <byte>` | fill memory |
| `pattern` | refill with `00 01 … FF` |

Log line types: `CCC bcast`, `CCC direct` (header + defining bytes), `CCC direct … -> addr W` / `<- addr R` (direct segment to us), `SDR write` / `SDR read`, `I2C write` / `I2C read`, `DAA …` (every dynamic address change), `ERR …` (parity errors, invalid DA), `HDR …`.

## 7. Supported CCCs and transfers

| CCC | Code | Support |
|---|---|---|
| ENEC / DISEC | 0x00 / 0x01 bc, 0x80 / 0x81 direct | mask stored; IBI and Hot-Join are never raised |
| RSTDAA | 0x06 bc, 0x86 direct | clears the dynamic address (the static address works again) |
| ENTDAA | 0x07 | full open-drain arbitration on PID+BCR+DCR, ACKs the new address only if its parity is correct, then sits out later rounds |
| SETMWL / SETMRL | 0x09 / 0x0A bc, 0x89 / 0x8A direct | stored (2 bytes); GETMWL/GETMRL report them |
| SETAASA | 0x29 | dynamic address = static address |
| ENTHDR0-7 | 0x20-0x27 | logged; the target **ignores the bus** until SCL and SDA have been high for 50 µs |
| SETDASA | 0x87 | accepted on the static address while no dynamic address is set |
| SETNEWDA | 0x88 | accepted on the current dynamic address |
| GETMWL / GETMRL | 0x8B / 0x8C | 2 bytes each (no IBI payload byte, since BCR[2]=0) |
| GETPID / GETBCR / GETDCR | 0x8D / 0x8E / 0x8F | 6 / 1 / 1 bytes |
| GETSTATUS | 0x90 | 2 bytes; bit 5 = protocol error (parity) since the last GETSTATUS |
| GETMXDS | 0x94 | `0x04 0x04` (2 MHz max sustained; see Speed) |
| other broadcast CCCs | | logged with their payload, otherwise ignored |
| other direct CCCs | | address **NACKed** (as the spec requires for unsupported direct CCCs) |

Transfers:
* SDR private write: `addr/W`, ACK, then bytes with an odd-parity T-bit. A bad T-bit is logged, sets GETSTATUS bit 5, and the rest of that write is ignored.
* SDR private read: `addr/R`, ACK, then bytes with T=1 ("more"). The controller can stop the read after any byte; T=0 comes after MRL (256) bytes.
* The I3CBlaster's 0x7E arbitration header followed by Sr and an address is handled. A private transfer that starts directly with START + DA also works.
* Legacy I2C on the static address while no dynamic address is set: write (ACK per byte), read (open drain, controller ACK/NACK), and write + Sr + read.

## 8. How it works

```
 src/i3c_target.pio   PIO bit layer      bit engine (PIO0 SM0) + START/STOP detector (PIO1 SM0)
 src/i3c_phy.c        PIO set-up         FIFO access, reset, slot encoders (i3c_phy.h)
 src/i3c_target.c     protocol           address/ACK decisions, CCCs, DAA, memory (core 1)
 src/i3c_log.c        logging            lock-free ring core1 -> core0, text formatting
 src/main.c           USB + console      core 0
 test/sim/            host simulation    PIO interpreter + controller model + test suite
```

* The CPU feeds the **bit engine** with one *slot* per group of bits: receive N bits, drive N bits, ACK, an open-drain arbitration byte, or a T-bit. The engine changes SDA after each SCL falling edge and samples it on each rising edge. Bytes the controller will write are always queued two ahead, so the CPU only sits on the critical path for **ACK/NACK decisions** and **re-arming after a repeated START**.
* **ENTDAA arbitration** runs in PIO. For each '1' bit the target releases SDA, and if it then samples a '0', a device with a lower PID has won. The engine lets go of the bus immediately and the target tries again in the next round.
* **T-bit:** for T=1 the target drives SDA high and hands SDA back on the SCL rising edge. If the controller then pulls SDA low while SCL is high (repeated START), the read was aborted and the engine stops. For T=0 the target holds SDA low through the high phase and releases it on the next falling edge. Open drain is used for ACK and arbitration, push-pull for read data and the T-bit.
* START/Sr/STOP come from a separate PIO program that watches for SDA edges while SCL is high. After every STOP the bit engine is pre-armed for the next address, so the first frame after an idle bus does not depend on CPU latency.
* Everything runs from RAM (`copy_to_ram`), and core 1 does nothing but the bus, so it never waits on flash or USB.

## Speed

Each address byte needs a CPU decision within roughly one SCL half-period, and so does each repeated START. Results from `make -C test/sim sweep`, with a pessimistic model where the CPU reacts within the stated time:

| SCL | CPU reaction ≤ 200 ns | ≤ 500 ns | ≤ 1 µs |
|---|---|---|---|
| 0.4 MHz | pass | pass | pass |
| 1 MHz | pass | pass | fail |
| 2 MHz | pass | fail | fail |
| 4 MHz | fail | fail | fail |

**Recommendation: `i3c_clk 1000` (1 MHz) or lower. Use `i3c_clk 400` if anything looks flaky.** The I3CBlaster default of 12.5 MHz will not work. GETMXDS reports 2 MHz, the slowest value the I3C spec can encode.

## Known limitations

* **Not validated on hardware yet.** Only the simulation has passed so far; the timing figures above come from that model, not from measurements.
* **Speed:** about 1 to 2 MHz SDR at most (see above). No HDR (DDR/TSP/TSL); ENTHDRx makes the target ignore the bus until it is idle.
* **No IBI, no Hot-Join, no controller role** (BCR says so). ENEC/DISEC are only stored.
* No GETCAPS, GETACCCR, RSTACT, target reset pattern, ENTASx/ENTTM/SETXTIME/DEFTGTS processing, and no group addressing. Unknown broadcast CCCs are logged and ignored, and unknown direct CCCs are NACKed.
* The I3C spec has no clock stretching, and the target never holds SCL. If the CPU falls behind (clock too fast), bits are lost and the frame is garbled. The engine resynchronises at the next STOP.
* **Hand-off overlap:** after an ACK, the target releases SDA a few nanoseconds (about 15 ns at 200 MHz) after SCL falls, while the I3CBlaster starts driving at that same edge. This brief overlap is normal for this kind of hand-off but shows up on a scope.
* START/STOP detection is edge-based with a 2-cycle synchroniser, so a very noisy SDA during SCL high can create false conditions. Keep wires short and the ground solid.
* The private-transfer and I2C behaviour on the static address before a dynamic address exists (`I3C_RESPOND_ON_STATIC_ADDR`, `I3C_LEGACY_I2C`) is a convenience for bring-up. Turn both off for a strictly spec-like target.
* The PID manufacturer field (0x7FFF) is a placeholder. Change `I3C_PID` if you have a real MIPI manufacturer ID.

## Credits and license

* MIT license, see [LICENSE](LICENSE).
* Made to work with **[I3CBlaster](https://github.com/xyphro/I3CBlaster)** by xyphro / Kai Gossner (MIT). Its README, CLI and Python module were studied to match its command syntax and transfer sequences (0x7E arbitration header, ENTDAA framing, T-bit handling). **No I3CBlaster source code is copied into this repository.** All PIO and protocol code here is original target-side code.
* `pico_sdk_import.cmake` comes from the Raspberry Pi Pico SDK (BSD-3-Clause).
* I3C and MIPI are trademarks of MIPI Alliance. This is an independent hobby project, not affiliated with MIPI Alliance.
