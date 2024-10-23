# current3x3A_Artiq
3 channel current driver (up to 3A each channel) compatible with Artiq



<img src="./images/current3x3_back_view.png"  width="200" />
<img src="./images/current3x3_top_view.png"  width="200" />
<img src="./images/current3x3_bottom_view.png"  width="200" />
<img src="./images/current3x3_front_view.png"  width="200" />

# User Interface

The user interface is available via TCP/IP on port 10. Depending on the configuration, the IP address can be static or assigned via DHCP; this must be set when programming the microcontroller. After connecting to the device, the user can change settings by sending commands.

## Commands

`DAC: state: val1; val2; val3; time`, where state is TTL state (000, 001, 010, 011, ...)

Values val1, val2, val3 should be between: -3.0 V - +3.0 V. 1V corresponds to 1A. 

Time is linear transition time (in ms) to this stete (min 1ms).

Example: `DAC: 000: 1.2565; -2.30; -0.065; 150`


# Issues

Please add your comments in the `Issues` Github section (at the top of this page).