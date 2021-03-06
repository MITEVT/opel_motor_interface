EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:MITEVT_ANALOG
LIBS:MITEVT_CONTACTORS
LIBS:MITEVT_interface
LIBS:MITEVT_mcontrollers
LIBS:MITEVT_OPTO
LIBS:MITEVT_power
LIBS:MITEVT_REG
LIBS:final_outline-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 2 6
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L CONN_02X04 P001
U 1 1 56352AC8
P 2350 1950
F 0 "P001" H 2350 2200 50  0000 C CNN
F 1 "CANCONNECTOR" H 2350 1700 50  0000 C CNN
F 2 "MITEVT_AUTOMOTIVECON:776280-1" H 2350 750 60  0001 C CNN
F 3 "" H 2350 750 60  0000 C CNN
	1    2350 1950
	1    0    0    -1  
$EndComp
$Comp
L R R201
U 1 1 563E3F11
P 2350 1100
F 0 "R201" V 2430 1100 50  0000 C CNN
F 1 "120" V 2350 1100 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 2280 1100 30  0001 C CNN
F 3 "" H 2350 1100 30  0000 C CNN
	1    2350 1100
	0    -1   -1   0   
$EndComp
Text HLabel 2250 1300 0    60   BiDi ~ 0
CANP
Text HLabel 3400 1800 2    60   BiDi ~ 0
CANH
Text HLabel 1300 1800 0    60   BiDi ~ 0
CANL
Wire Wire Line
	2350 1300 2350 1500
Wire Wire Line
	1800 1900 2100 1900
Wire Wire Line
	1600 2100 2100 2100
Wire Wire Line
	2600 1800 3400 1800
Wire Wire Line
	1300 1800 2100 1800
Wire Wire Line
	3100 2100 2600 2100
Wire Wire Line
	2600 2000 2900 2000
Wire Wire Line
	2600 1900 2750 1900
Wire Wire Line
	2250 1300 2350 1300
Wire Wire Line
	2900 2000 2900 2450
$Comp
L GND #PWR04
U 1 1 5647C17F
P 2900 2450
F 0 "#PWR04" H 2900 2200 50  0001 C CNN
F 1 "GND" H 2900 2300 50  0000 C CNN
F 2 "" H 2900 2450 60  0000 C CNN
F 3 "" H 2900 2450 60  0000 C CNN
	1    2900 2450
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR05
U 1 1 5647C1FB
P 1800 2450
F 0 "#PWR05" H 1800 2200 50  0001 C CNN
F 1 "GND" H 1800 2300 50  0000 C CNN
F 2 "" H 1800 2450 60  0000 C CNN
F 3 "" H 1800 2450 60  0000 C CNN
	1    1800 2450
	1    0    0    -1  
$EndComp
Wire Wire Line
	1800 2450 1800 1900
Wire Wire Line
	2100 2000 1950 2000
Wire Wire Line
	1950 2000 1950 1500
Wire Wire Line
	1950 1500 2750 1500
Wire Wire Line
	2750 1500 2750 1900
Connection ~ 2350 1500
Wire Wire Line
	1600 1100 1600 2100
Wire Wire Line
	3100 1100 3100 2100
Wire Wire Line
	1600 1100 2200 1100
Wire Wire Line
	2500 1100 3100 1100
Connection ~ 3100 1800
Connection ~ 1600 1800
$Comp
L CONN_01X14 P002
U 1 1 56AD09BE
P 5050 1900
F 0 "P002" H 5050 2650 50  0000 C CNN
F 1 "CONN_01X14" V 5150 1900 50  0000 C CNN
F 2 "MITEVT_AUTOMOTIVECON:776267-1" H 5050 1900 60  0001 C CNN
F 3 "" H 5050 1900 60  0000 C CNN
	1    5050 1900
	1    0    0    -1  
$EndComp
NoConn ~ 4850 1750
NoConn ~ 4850 1850
NoConn ~ 4850 1950
NoConn ~ 4850 2150
NoConn ~ 4850 2250
NoConn ~ 4850 2350
NoConn ~ 4850 2450
NoConn ~ 4850 2550
Text HLabel 4350 1250 0    60   Input ~ 0
RS232_MSG
Text HLabel 4350 1350 0    60   Input ~ 0
DC/DC_CTRL
Wire Wire Line
	4350 1350 4850 1350
Wire Wire Line
	4200 1450 4850 1450
Wire Wire Line
	4850 1550 4500 1550
Wire Wire Line
	4500 1550 4500 1900
$Comp
L R R202
U 1 1 57629178
P 4350 1650
F 0 "R202" V 4430 1650 50  0000 C CNN
F 1 "120" V 4350 1650 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 4280 1650 50  0001 C CNN
F 3 "" H 4350 1650 50  0000 C CNN
	1    4350 1650
	1    0    0    -1  
$EndComp
Wire Wire Line
	4350 1450 4350 1500
Wire Wire Line
	4500 1900 4200 1900
Wire Wire Line
	4350 1900 4350 1800
Connection ~ 4350 1900
Connection ~ 4350 1450
Wire Wire Line
	4850 1650 4600 1650
Wire Wire Line
	4600 1650 4600 2000
Wire Wire Line
	4600 2000 4200 2000
Text HLabel 4200 1900 0    60   BiDi ~ 0
CANH2
Text HLabel 4200 1450 0    60   BiDi ~ 0
CANL2
Wire Wire Line
	4200 2000 4200 2200
$Comp
L GND #PWR06
U 1 1 5762B8AB
P 4200 2200
F 0 "#PWR06" H 4200 1950 50  0001 C CNN
F 1 "GND" H 4200 2050 50  0000 C CNN
F 2 "" H 4200 2200 50  0000 C CNN
F 3 "" H 4200 2200 50  0000 C CNN
	1    4200 2200
	1    0    0    -1  
$EndComp
Wire Wire Line
	4850 1250 4350 1250
NoConn ~ 4850 2050
$EndSCHEMATC
