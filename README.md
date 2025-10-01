# current3x3A_Artiq
3 channel current driver (up to 3A each channel) compatible with Artiq

<img src="./images/current3x3_back_view.png"  width="200" />
<img src="./images/current3x3_top_view.png"  width="200" />
<img src="./images/current3x3_bottom_view.png"  width="200" />
<img src="./images/current3x3_front_view.png"  width="200" />

# Ports

Main power supply (- and +) 10A, 15V.

3x compensating coils.

Power supply for low power control circuits (optional). This power supplay can be taken from the main power supply (using jumpers).

Ethernet - user interface.

Water cooling.

# Input configuration

`TTL0` - triger, rising edge triggers current state

`TTL1`, `TTL2`, `TTL3` - determines current state TTL1 is the least significant bit.   
(`TTL1`=0, `TTL2`=0, `TTL3`=0) -> `state 0`  
(`TTL1`=1, `TTL2`=0, `TTL3`=0) -> `state 1`  
(`TTL1`=0, `TTL2`=1, `TTL3`=0) -> `state 2`  
...  
(`TTL1`=1, `TTL2`=1, `TTL3`=1) -> `state 7`

TTLs inputs are configurated in pull-down mode.

# User Interface

The user interface is available via TCP/IP. TCP232 module is used for Ethernet TCP/IP communication. IP address is configurable on http server. After connecting to the device, the user can change settings by sending commands.

## Setting Ethernet
- type IP address in web browser (default-from factory TCP232 module IP is 192.168.0.7)
- switch to `Serial Port` tab and set `Work Mode` to `TCP Server` and `Local Port Number` which will be used to connect to device
- IP configuration may be changed in `Local IP Config` tab.

## Connecting to the device

Example in nc. In Windows you may need to switch to bash.
```
nc 192.168.3.96 10
```


## Commands

There are 8 ttl states available, s0 (ttl=000), s1 (ttl=001), ..., s7 (ttl=111).

For each state, there are parameters: v1 (current in 1st coil), v2 (current in 2nd coil), v3 (current in 3th coil), t (time in ms of ramp before reaching these values). Current can be set between -3A to 3A. Minimum time t is 1ms.

Example:

`S1:V2 1.3` - set current 1.3A in 2nd coil for ttl state 001

`S0:V3 -0.23` - set current -0.23A in 3th coil for ttl state 000

`S2:T 5` - set time of ramp to 5 ms

`S1:V1 ?` - display value of 1st coil for state 001

`STATE ?` - display current ttl state 

Commands can be joined using `;` separator: ex. `s1:v2 2.1; s1:t 3` (max 100 characters in one message)

---

# Flashing Firmware to STM32 with ST-LINK

The firmware file to upload is located here: firmware/current3x3A_Artiq.hex

## Using STM32CubeProgrammer (GUI)

1. Connect the STM32 board to your computer via **ST-LINK**.  
2. Open **STM32CubeProgrammer**.  
3. Select **ST-LINK** as the interface and click **Connect**.  
4. In **Erasing & Programming**:  
   - Select `firmware/current3x3A_Artiq.hex`.  
   - Enable **Verify programming** (recommended).  
   - Click **Start Programming**.  
5. The board will restart with the new firmware.  

---

## Using STM32CubeProgrammer (CLI)

```bash
STM32_Programmer_CLI -c port=SWD -w firmware/current3x3A_Artiq.hex -v -rst
```

# Issues

Please add your comments in the `Issues` Github section (at the top of this page).

### Known issues
- communication issues when not nc in linux is used