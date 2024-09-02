2024-09-02

Checking DAC: bits to voltage

DACIN1 - [SetDAC(0, 65000);]  -> 2.955 V
DACIN1 - [SetDAC(0, 30000);]  -> 1.35 V
DACIN2 - [SetDAC(1, 25000);]  -> 1.125 V
DACIN3 - [SetDAC(2, 20000);]  -> 0.882 V
DACIN4 - [SetDAC(3, 10000);]  -> 0.425 V

Checking time factor
1st test (from Mehrdad program)
10ms in program ->  300ms

Test - transistors
val set in DAC table -> current measured by multimeter

L1:
start: -0.1A
0.0V -> 0.1A
0.1V -> -0.11A
1.0V -> -0.18A
2.0V -> -0.38A
-1.0V -> 0.2A
-2.0V -> 0.4A

L2:
start: -0.38A
0.0V -> 0.38A
0.1V -> -0.41A
1.0V -> -0.93A
2.0V -> -1.85
-1.0V -> 0.93A
-2.0V -> 1.86A

L3:
start: -0.36A
0.0V -> -0.36A
0.1V -> -0.39A
1.0V -> -0.9A
2.0V -> -1.8A
-1.0V -> 0.9A
-2.0V -> 1.8A