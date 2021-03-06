EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
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
L power:+12V #PWR0103
U 1 1 60910FE4
P 2550 1150
F 0 "#PWR0103" H 2550 1000 50  0001 C CNN
F 1 "+12V" H 2565 1323 50  0000 C CNN
F 2 "" H 2550 1150 50  0001 C CNN
F 3 "" H 2550 1150 50  0001 C CNN
	1    2550 1150
	1    0    0    -1  
$EndComp
Text Label 1150 1900 2    50   ~ 0
POWER_ON
$Comp
L Device:R R102
U 1 1 60911597
P 1400 1900
F 0 "R102" V 1193 1900 50  0000 C CNN
F 1 "1k" V 1284 1900 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 1330 1900 50  0001 C CNN
F 3 "~" H 1400 1900 50  0001 C CNN
	1    1400 1900
	0    1    1    0   
$EndComp
$Comp
L Device:R R103
U 1 1 60911B80
P 1600 2200
F 0 "R103" H 1670 2246 50  0000 L CNN
F 1 "10k" H 1670 2155 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 1530 2200 50  0001 C CNN
F 3 "~" H 1600 2200 50  0001 C CNN
	1    1600 2200
	1    0    0    -1  
$EndComp
$Comp
L Device:R R104
U 1 1 60911FD4
P 1950 1450
F 0 "R104" H 2020 1496 50  0000 L CNN
F 1 "1k" H 2020 1405 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 1880 1450 50  0001 C CNN
F 3 "~" H 1950 1450 50  0001 C CNN
	1    1950 1450
	1    0    0    -1  
$EndComp
$Comp
L Transistor_FET:Si2371EDS Q102
U 1 1 60913394
P 2450 1650
F 0 "Q102" H 2654 1604 50  0000 L CNN
F 1 "DMP3098L" H 2654 1695 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 2650 1575 50  0001 L CIN
F 3 "" H 2450 1650 50  0001 L CNN
	1    2450 1650
	1    0    0    1   
$EndComp
$Comp
L Transistor_BJT:BC817 Q101
U 1 1 6091717C
P 1850 1900
F 0 "Q101" H 2040 1946 50  0000 L CNN
F 1 "S8050" H 2040 1855 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 2050 1825 50  0001 L CIN
F 3 "https://www.onsemi.com/pub/Collateral/BC818-D.pdf" H 1850 1900 50  0001 L CNN
	1    1850 1900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0102
U 1 1 60917F72
P 1950 2550
F 0 "#PWR0102" H 1950 2300 50  0001 C CNN
F 1 "GND" H 1955 2377 50  0000 C CNN
F 2 "" H 1950 2550 50  0001 C CNN
F 3 "" H 1950 2550 50  0001 C CNN
	1    1950 2550
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0104
U 1 1 6091823D
P 2550 2550
F 0 "#PWR0104" H 2550 2300 50  0001 C CNN
F 1 "GND" H 2555 2377 50  0000 C CNN
F 2 "" H 2550 2550 50  0001 C CNN
F 3 "" H 2550 2550 50  0001 C CNN
	1    2550 2550
	1    0    0    -1  
$EndComp
Wire Wire Line
	1950 2550 1950 2450
Wire Wire Line
	1600 2350 1600 2450
Wire Wire Line
	1600 2450 1950 2450
Connection ~ 1950 2450
Wire Wire Line
	1950 2450 1950 2100
Wire Wire Line
	1600 2050 1600 1900
Wire Wire Line
	1600 1900 1650 1900
Wire Wire Line
	1600 1900 1550 1900
Connection ~ 1600 1900
Wire Wire Line
	1250 1900 1200 1900
Wire Wire Line
	1950 1700 1950 1650
Wire Wire Line
	1950 1300 1950 1250
Wire Wire Line
	1950 1250 2550 1250
Wire Wire Line
	2550 1250 2550 1150
Wire Wire Line
	2550 1250 2550 1450
Connection ~ 2550 1250
Wire Wire Line
	2250 1650 1950 1650
Connection ~ 1950 1650
Wire Wire Line
	1950 1650 1950 1600
Text Label 2750 2100 0    50   ~ 0
DUT_12V
Wire Wire Line
	2550 2100 2550 1850
$Comp
L Device:D D101
U 1 1 6091B25C
P 2550 2250
F 0 "D101" V 2504 2330 50  0000 L CNN
F 1 "SM4007PL" V 2595 2330 50  0000 L CNN
F 2 "Diode_SMD:D_SOD-123F" H 2550 2250 50  0001 C CNN
F 3 "~" H 2550 2250 50  0001 C CNN
	1    2550 2250
	0    1    1    0   
$EndComp
Connection ~ 2550 2100
Wire Wire Line
	2550 2400 2550 2450
Text Notes 1800 750  0    50   ~ 0
DUT power switch
Text Label 1400 4250 0    50   ~ 0
DUT_PRESENT
$Comp
L Device:R R101
U 1 1 6092647B
P 1250 3850
F 0 "R101" H 1320 3896 50  0000 L CNN
F 1 "10k" H 1320 3805 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 1180 3850 50  0001 C CNN
F 3 "~" H 1250 3850 50  0001 C CNN
	1    1250 3850
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0101
U 1 1 609270BD
P 1250 3500
F 0 "#PWR0101" H 1250 3350 50  0001 C CNN
F 1 "+3.3V" H 1265 3673 50  0000 C CNN
F 2 "" H 1250 3500 50  0001 C CNN
F 3 "" H 1250 3500 50  0001 C CNN
	1    1250 3500
	1    0    0    -1  
$EndComp
Text Label 1250 4600 0    50   ~ 0
DUT_GPIO_GND
Wire Wire Line
	1250 4600 1250 4250
Wire Wire Line
	1400 4250 1250 4250
Connection ~ 1250 4250
Wire Wire Line
	1250 4250 1250 4000
Wire Wire Line
	1250 3700 1250 3500
Text Notes 1000 5050 0    50   ~ 0
If DUT is on the bed, PCB_PRESENT\nis pulled low.
Text Notes 1000 3150 0    50   ~ 0
DUT presence detection
Text Label 3300 3450 2    50   ~ 0
CONTROLLER_GPIO12
Text Label 3550 3450 0    50   ~ 0
READY_TO_FLASH
Wire Wire Line
	3550 3450 3300 3450
Text Label 5400 3450 2    50   ~ 0
READY_TO_FLASH
Text Label 5650 3450 0    50   ~ 0
FLASHER_GPIO17
Wire Wire Line
	5650 3450 5400 3450
Text Label 5400 3600 2    50   ~ 0
FLASH_OK
Text Label 5650 3600 0    50   ~ 0
FLASHER_GPIO27
Wire Wire Line
	5650 3600 5400 3600
Text Label 3300 3600 2    50   ~ 0
CONTROLLER_GPIO13
Text Label 3550 3600 0    50   ~ 0
FLASH_OK
Wire Wire Line
	3550 3600 3300 3600
$Comp
L SH-RPi-test-jig:ADS1115-module U101
U 1 1 6092EAB5
P 5850 1800
F 0 "U101" H 5950 2250 50  0000 C CNN
F 1 "ADS1115-module" H 6250 2150 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x10_P2.54mm_Vertical" H 5850 1800 50  0001 C CNN
F 3 "" H 5850 1800 50  0001 C CNN
	1    5850 1800
	1    0    0    -1  
$EndComp
Text Label 5350 1650 2    50   ~ 0
DUT_Vsup
Text Label 3850 1200 2    50   ~ 0
DUT_5V
Wire Wire Line
	5350 1650 5500 1650
$Comp
L power:+3.3V #PWR0107
U 1 1 60931888
P 5850 1300
F 0 "#PWR0107" H 5850 1150 50  0001 C CNN
F 1 "+3.3V" H 5865 1473 50  0000 C CNN
F 2 "" H 5850 1300 50  0001 C CNN
F 3 "" H 5850 1300 50  0001 C CNN
	1    5850 1300
	1    0    0    -1  
$EndComp
Wire Wire Line
	5850 1300 5850 1350
$Comp
L power:GND #PWR0108
U 1 1 609324DF
P 5850 2250
F 0 "#PWR0108" H 5850 2000 50  0001 C CNN
F 1 "GND" H 5855 2077 50  0000 C CNN
F 2 "" H 5850 2250 50  0001 C CNN
F 3 "" H 5850 2250 50  0001 C CNN
	1    5850 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	5850 2250 5850 2200
Text Label 6350 1650 0    50   ~ 0
SCL
Text Label 6350 1750 0    50   ~ 0
SDA
$Comp
L Device:R R105
U 1 1 60936622
P 3850 1600
F 0 "R105" H 3920 1646 50  0000 L CNN
F 1 "1k" H 3920 1555 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 3780 1600 50  0001 C CNN
F 3 "~" H 3850 1600 50  0001 C CNN
	1    3850 1600
	1    0    0    -1  
$EndComp
$Comp
L Device:R R106
U 1 1 60936C2B
P 3850 1900
F 0 "R106" H 3920 1946 50  0000 L CNN
F 1 "1k" H 3920 1855 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 3780 1900 50  0001 C CNN
F 3 "~" H 3850 1900 50  0001 C CNN
	1    3850 1900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0105
U 1 1 60936FF6
P 3850 2250
F 0 "#PWR0105" H 3850 2000 50  0001 C CNN
F 1 "GND" H 3855 2077 50  0000 C CNN
F 2 "" H 3850 2250 50  0001 C CNN
F 3 "" H 3850 2250 50  0001 C CNN
	1    3850 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	3850 2250 3850 2200
Wire Wire Line
	3850 1200 3850 1450
Wire Wire Line
	3850 1750 4100 1750
Connection ~ 3850 1750
Text Label 4100 1750 0    50   ~ 0
A2
Text Label 5350 1850 2    50   ~ 0
A2
Wire Wire Line
	5350 1950 5500 1950
Text Label 5350 1950 2    50   ~ 0
A3
Text Label 4350 1200 2    50   ~ 0
DUT_12V
$Comp
L Device:R R107
U 1 1 60940615
P 4350 1600
F 0 "R107" H 4420 1646 50  0000 L CNN
F 1 "4.7k" H 4420 1555 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 4280 1600 50  0001 C CNN
F 3 "~" H 4350 1600 50  0001 C CNN
	1    4350 1600
	1    0    0    -1  
$EndComp
$Comp
L Device:R R108
U 1 1 6094061B
P 4350 1900
F 0 "R108" H 4420 1946 50  0000 L CNN
F 1 "1k" H 4420 1855 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 4280 1900 50  0001 C CNN
F 3 "~" H 4350 1900 50  0001 C CNN
	1    4350 1900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0106
U 1 1 60940621
P 4350 2250
F 0 "#PWR0106" H 4350 2000 50  0001 C CNN
F 1 "GND" H 4355 2077 50  0000 C CNN
F 2 "" H 4350 2250 50  0001 C CNN
F 3 "" H 4350 2250 50  0001 C CNN
	1    4350 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	4350 2250 4350 2200
Wire Wire Line
	4350 1200 4350 1450
Wire Wire Line
	4350 1750 4600 1750
Connection ~ 4350 1750
Text Label 4600 1750 0    50   ~ 0
A3
Text Notes 4500 850  0    50   ~ 0
DUT voltage testing
Text Label 5400 4100 2    50   ~ 0
SCL
Text Label 5650 4100 0    50   ~ 0
DUT_SCL
Wire Wire Line
	5650 4100 5400 4100
Text Label 3300 4100 2    50   ~ 0
CONTROLLER_SCL
Text Label 3550 4100 0    50   ~ 0
SCL
Wire Wire Line
	3550 4100 3300 4100
Text Label 5400 4250 2    50   ~ 0
SDA
Text Label 5650 4250 0    50   ~ 0
DUT_SDA
Wire Wire Line
	5650 4250 5400 4250
Text Label 3300 4250 2    50   ~ 0
CONTROLLER_SDA
Text Label 3550 4250 0    50   ~ 0
SDA
Wire Wire Line
	3550 4250 3300 4250
Text Label 6600 5300 0    50   ~ 0
RESPONDER_POWER
Text Label 6350 5300 2    50   ~ 0
DUT_5V
Wire Wire Line
	6350 5300 6600 5300
Text Label 6350 5450 2    50   ~ 0
CAN_H
Text Label 6600 5450 0    50   ~ 0
RESPONDER_CAN_H
Text Label 6350 5600 2    50   ~ 0
CAN_L
Text Label 6600 5600 0    50   ~ 0
RESPONDER_CAN_L
Text Label 6350 5750 2    50   ~ 0
CAN_VCC
Text Label 6600 5750 0    50   ~ 0
RESPONDER_CAN_VCC
Text Label 6350 5900 2    50   ~ 0
CAN_GND
Text Label 6600 5900 0    50   ~ 0
RESPONDER_CAN_GND
Wire Wire Line
	6350 5450 6600 5450
Wire Wire Line
	6350 5600 6600 5600
Wire Wire Line
	6350 5750 6600 5750
Wire Wire Line
	6350 5900 6600 5900
Text Label 8400 3650 0    50   ~ 0
FLASHER_GPIO18
Text Label 8400 3850 0    50   ~ 0
FLASHER_GPIO14
Text Label 8150 3850 2    50   ~ 0
TX
Text Label 8400 4050 0    50   ~ 0
FLASHER_GPIO15
Text Label 8150 4050 2    50   ~ 0
RX
Text Label 7300 3850 2    50   ~ 0
UPDI
$Comp
L Device:R R109
U 1 1 6094970D
P 7700 3850
F 0 "R109" V 7493 3850 50  0000 C CNN
F 1 "1k" V 7584 3850 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 7630 3850 50  0001 C CNN
F 3 "~" H 7700 3850 50  0001 C CNN
	1    7700 3850
	0    1    1    0   
$EndComp
Wire Wire Line
	7450 3850 7450 4050
Connection ~ 7450 3850
Wire Wire Line
	7450 3850 7550 3850
Text Label 8150 3650 2    50   ~ 0
ATTVcc
Wire Wire Line
	8150 3650 8400 3650
Text Label 8400 4250 0    50   ~ 0
FLASHER_GND
$Comp
L power:GND #PWR0110
U 1 1 6094E6F0
P 8200 4400
F 0 "#PWR0110" H 8200 4150 50  0001 C CNN
F 1 "GND" H 8205 4227 50  0000 C CNN
F 2 "" H 8200 4400 50  0001 C CNN
F 3 "" H 8200 4400 50  0001 C CNN
	1    8200 4400
	1    0    0    -1  
$EndComp
Wire Wire Line
	8200 4400 8200 4250
Text Notes 7800 3150 0    50   ~ 0
UPDI flashing connection
Text Notes 8400 3500 0    50   ~ 0
Assuming that RPi GPIO current capability\nis sufficient to power the ATtiny and four LEDs
Text Label 5000 5450 2    50   ~ 0
CONTROLLER_CAN_H
Text Label 5000 5600 2    50   ~ 0
CONTROLLER_CAN_L
Text Label 5000 5750 2    50   ~ 0
CONTROLLER_CAN_VCC
Text Label 5000 5900 2    50   ~ 0
CONTROLLER_CAN_GND
Text Label 5300 5450 0    50   ~ 0
CAN_H
Text Label 5300 5600 0    50   ~ 0
CAN_L
Text Label 5300 5750 0    50   ~ 0
CAN_VCC
Text Label 5300 5900 0    50   ~ 0
CAN_GND
Wire Wire Line
	5300 5450 5000 5450
Wire Wire Line
	5300 5600 5000 5600
Wire Wire Line
	5300 5750 5000 5750
Wire Wire Line
	5300 5900 5000 5900
$Comp
L SH-RPi-test-jig:HT75xx-1-SOT23 U102
U 1 1 6095F055
P 8500 1750
F 0 "U102" H 8500 2117 50  0000 C CNN
F 1 "H7533" H 8500 2026 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 8500 2075 50  0001 C CIN
F 3 "https://www.holtek.com/documents/10179/116711/HT75xx-1v250.pdf" H 8500 1850 50  0001 C CNN
	1    8500 1750
	1    0    0    -1  
$EndComp
$Comp
L Device:C C101
U 1 1 6095FC17
P 7850 1950
F 0 "C101" H 7965 1996 50  0000 L CNN
F 1 "2.2uF" H 7965 1905 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 7888 1800 50  0001 C CNN
F 3 "~" H 7850 1950 50  0001 C CNN
	1    7850 1950
	1    0    0    -1  
$EndComp
$Comp
L Device:C C102
U 1 1 6096032A
P 9100 1950
F 0 "C102" H 9215 1996 50  0000 L CNN
F 1 "2.2uF" H 9215 1905 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 9138 1800 50  0001 C CNN
F 3 "~" H 9100 1950 50  0001 C CNN
	1    9100 1950
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0111
U 1 1 609605D2
P 8500 2250
F 0 "#PWR0111" H 8500 2000 50  0001 C CNN
F 1 "GND" H 8505 2077 50  0000 C CNN
F 2 "" H 8500 2250 50  0001 C CNN
F 3 "" H 8500 2250 50  0001 C CNN
	1    8500 2250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0109
U 1 1 60960C1C
P 7850 2250
F 0 "#PWR0109" H 7850 2000 50  0001 C CNN
F 1 "GND" H 7855 2077 50  0000 C CNN
F 2 "" H 7850 2250 50  0001 C CNN
F 3 "" H 7850 2250 50  0001 C CNN
	1    7850 2250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0112
U 1 1 60960F62
P 9100 2250
F 0 "#PWR0112" H 9100 2000 50  0001 C CNN
F 1 "GND" H 9105 2077 50  0000 C CNN
F 2 "" H 9100 2250 50  0001 C CNN
F 3 "" H 9100 2250 50  0001 C CNN
	1    9100 2250
	1    0    0    -1  
$EndComp
Text Label 7650 1650 2    50   ~ 0
DUT_5V
Text Label 9450 1650 0    50   ~ 0
DUT_3V3
Wire Wire Line
	7650 1650 7850 1650
Wire Wire Line
	7850 1800 7850 1650
Connection ~ 7850 1650
Wire Wire Line
	7850 1650 8200 1650
Wire Wire Line
	7850 2250 7850 2150
Wire Wire Line
	8500 2250 8500 1950
Wire Wire Line
	8800 1650 9100 1650
Wire Wire Line
	9100 1800 9100 1650
Connection ~ 9100 1650
Wire Wire Line
	9100 1650 9450 1650
Wire Wire Line
	9100 2250 9100 2200
Text Notes 8100 1150 0    50   ~ 0
LDO providing DUT 3.3V normally\nprovided by the Raspberry Pi
$Comp
L Connector_Generic:Conn_01x01 J101
U 1 1 6096E550
P 1050 4250
F 0 "J101" H 968 4025 50  0000 C CNN
F 1 "Conn_01x01" H 968 4116 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 1050 4250 50  0001 C CNN
F 3 "~" H 1050 4250 50  0001 C CNN
	1    1050 4250
	-1   0    0    1   
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J103
U 1 1 609702BE
P 1200 2100
F 0 "J103" V 1072 2180 50  0000 L CNN
F 1 "Conn_01x01" V 1163 2180 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 1200 2100 50  0001 C CNN
F 3 "~" H 1200 2100 50  0001 C CNN
	1    1200 2100
	0    1    1    0   
$EndComp
Connection ~ 1200 1900
Wire Wire Line
	1200 1900 1150 1900
$Comp
L Connector_Generic:Conn_01x01 J108
U 1 1 60970C28
P 3150 2300
F 0 "J108" V 3022 2380 50  0000 L CNN
F 1 "Conn_01x01" V 3113 2380 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 3150 2300 50  0001 C CNN
F 3 "~" H 3150 2300 50  0001 C CNN
	1    3150 2300
	0    1    1    0   
$EndComp
Wire Wire Line
	2550 2100 3150 2100
$Comp
L Connector_Generic:Conn_01x01 J109
U 1 1 609726F1
P 3850 1000
F 0 "J109" V 3814 912 50  0000 R CNN
F 1 "Conn_01x01" V 3723 912 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 3850 1000 50  0001 C CNN
F 3 "~" H 3850 1000 50  0001 C CNN
	1    3850 1000
	0    -1   -1   0   
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J111
U 1 1 60973073
P 4350 1000
F 0 "J111" V 4314 912 50  0000 R CNN
F 1 "Conn_01x01" V 4223 912 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 4350 1000 50  0001 C CNN
F 3 "~" H 4350 1000 50  0001 C CNN
	1    4350 1000
	0    -1   -1   0   
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J115
U 1 1 60973575
P 6600 1950
F 0 "J115" V 6472 2030 50  0000 L CNN
F 1 "Conn_01x01" V 6563 2030 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 6600 1950 50  0001 C CNN
F 3 "~" H 6600 1950 50  0001 C CNN
	1    6600 1950
	0    1    1    0   
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J116
U 1 1 609748F8
P 6800 1850
F 0 "J116" V 6672 1930 50  0000 L CNN
F 1 "Conn_01x01" V 6763 1930 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 6800 1850 50  0001 C CNN
F 3 "~" H 6800 1850 50  0001 C CNN
	1    6800 1850
	0    1    1    0   
$EndComp
Wire Wire Line
	6200 1650 6800 1650
Wire Wire Line
	6200 1750 6600 1750
$Comp
L Connector_Generic:Conn_01x01 J121
U 1 1 60977BA7
P 9450 1850
F 0 "J121" V 9322 1930 50  0000 L CNN
F 1 "Conn_01x01" V 9413 1930 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 9450 1850 50  0001 C CNN
F 3 "~" H 9450 1850 50  0001 C CNN
	1    9450 1850
	0    1    1    0   
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J117
U 1 1 609784D3
P 6850 3850
F 0 "J117" H 6768 3625 50  0000 C CNN
F 1 "Conn_01x01" H 6768 3716 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 6850 3850 50  0001 C CNN
F 3 "~" H 6850 3850 50  0001 C CNN
	1    6850 3850
	-1   0    0    1   
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J118
U 1 1 60979133
P 9400 3850
F 0 "J118" H 9480 3892 50  0000 L CNN
F 1 "Conn_01x01" H 9480 3801 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 9400 3850 50  0001 C CNN
F 3 "~" H 9400 3850 50  0001 C CNN
	1    9400 3850
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J119
U 1 1 609798AF
P 9400 4050
F 0 "J119" H 9480 4092 50  0000 L CNN
F 1 "Conn_01x01" H 9480 4001 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 9400 4050 50  0001 C CNN
F 3 "~" H 9400 4050 50  0001 C CNN
	1    9400 4050
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J120
U 1 1 60979B68
P 9400 4250
F 0 "J120" H 9480 4292 50  0000 L CNN
F 1 "Conn_01x01" H 9480 4201 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 9400 4250 50  0001 C CNN
F 3 "~" H 9400 4250 50  0001 C CNN
	1    9400 4250
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J110
U 1 1 6097AA4D
P 4100 1950
F 0 "J110" V 3972 2030 50  0000 L CNN
F 1 "Conn_01x01" V 4063 2030 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 4100 1950 50  0001 C CNN
F 3 "~" H 4100 1950 50  0001 C CNN
	1    4100 1950
	0    1    1    0   
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J112
U 1 1 6097B502
P 4600 1950
F 0 "J112" V 4472 2030 50  0000 L CNN
F 1 "Conn_01x01" V 4563 2030 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 4600 1950 50  0001 C CNN
F 3 "~" H 4600 1950 50  0001 C CNN
	1    4600 1950
	0    1    1    0   
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J107
U 1 1 6097E88B
P 2750 1250
F 0 "J107" H 2830 1292 50  0000 L CNN
F 1 "Conn_01x01" H 2830 1201 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 2750 1250 50  0001 C CNN
F 3 "~" H 2750 1250 50  0001 C CNN
	1    2750 1250
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J105
U 1 1 6097F051
P 1750 1650
F 0 "J105" H 1668 1425 50  0000 C CNN
F 1 "Conn_01x01" H 1668 1516 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 1750 1650 50  0001 C CNN
F 3 "~" H 1750 1650 50  0001 C CNN
	1    1750 1650
	-1   0    0    1   
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J104
U 1 1 60980B80
P 1600 1700
F 0 "J104" V 1564 1612 50  0000 R CNN
F 1 "Conn_01x01" V 1473 1612 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 1600 1700 50  0001 C CNN
F 3 "~" H 1600 1700 50  0001 C CNN
	1    1600 1700
	0    -1   -1   0   
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J106
U 1 1 609814F0
P 2150 2450
F 0 "J106" H 2230 2492 50  0000 L CNN
F 1 "Conn_01x01" H 2230 2401 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 2150 2450 50  0001 C CNN
F 3 "~" H 2150 2450 50  0001 C CNN
	1    2150 2450
	1    0    0    -1  
$EndComp
Wire Wire Line
	7850 3850 9200 3850
Wire Wire Line
	7450 4050 9200 4050
Wire Wire Line
	8200 4250 9200 4250
Wire Wire Line
	7050 3850 7450 3850
$Comp
L Connector_Generic:Conn_01x01 J124
U 1 1 609B0650
P 4550 2200
F 0 "J124" H 4630 2242 50  0000 L CNN
F 1 "Conn_01x01" H 4630 2151 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 4550 2200 50  0001 C CNN
F 3 "~" H 4550 2200 50  0001 C CNN
	1    4550 2200
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J123
U 1 1 609B1BE5
P 4050 2200
F 0 "J123" H 4130 2242 50  0000 L CNN
F 1 "Conn_01x01" H 4130 2151 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 4050 2200 50  0001 C CNN
F 3 "~" H 4050 2200 50  0001 C CNN
	1    4050 2200
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J125
U 1 1 609B3136
P 6050 2200
F 0 "J125" H 6130 2242 50  0000 L CNN
F 1 "Conn_01x01" H 6130 2151 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 6050 2200 50  0001 C CNN
F 3 "~" H 6050 2200 50  0001 C CNN
	1    6050 2200
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J126
U 1 1 609B73FA
P 2750 2450
F 0 "J126" H 2830 2492 50  0000 L CNN
F 1 "Conn_01x01" H 2830 2401 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 2750 2450 50  0001 C CNN
F 3 "~" H 2750 2450 50  0001 C CNN
	1    2750 2450
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J127
U 1 1 609BDEFA
P 9300 2200
F 0 "J127" V 9172 2280 50  0000 L CNN
F 1 "Conn_01x01" V 9263 2280 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 9300 2200 50  0001 C CNN
F 3 "~" H 9300 2200 50  0001 C CNN
	1    9300 2200
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J113
U 1 1 609D0E01
P 7850 1450
F 0 "J113" V 7814 1362 50  0000 R CNN
F 1 "Conn_01x01" V 7723 1362 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 7850 1450 50  0001 C CNN
F 3 "~" H 7850 1450 50  0001 C CNN
	1    7850 1450
	0    -1   -1   0   
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J114
U 1 1 609D761B
P 5650 1350
F 0 "J114" H 5730 1392 50  0000 L CNN
F 1 "Conn_01x01" H 5730 1301 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 5650 1350 50  0001 C CNN
F 3 "~" H 5650 1350 50  0001 C CNN
	1    5650 1350
	-1   0    0    1   
$EndComp
Connection ~ 5850 1350
Wire Wire Line
	5850 1350 5850 1450
Connection ~ 5850 2200
Wire Wire Line
	5850 2200 5850 2150
$Comp
L Connector_Generic:Conn_01x01 J128
U 1 1 609E66AD
P 5350 2150
F 0 "J128" V 5222 2230 50  0000 L CNN
F 1 "Conn_01x01" V 5313 2230 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 5350 2150 50  0001 C CNN
F 3 "~" H 5350 2150 50  0001 C CNN
	1    5350 2150
	0    1    1    0   
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J102
U 1 1 609E6DDC
P 5200 2050
F 0 "J102" V 5072 2130 50  0000 L CNN
F 1 "Conn_01x01" V 5163 2130 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 5200 2050 50  0001 C CNN
F 3 "~" H 5200 2050 50  0001 C CNN
	1    5200 2050
	0    1    1    0   
$EndComp
Wire Wire Line
	5200 1850 5500 1850
Connection ~ 9100 2200
Wire Wire Line
	9100 2200 9100 2100
Connection ~ 3850 2200
Wire Wire Line
	3850 2200 3850 2050
Connection ~ 4350 2200
Wire Wire Line
	4350 2200 4350 2050
Connection ~ 2550 2450
Wire Wire Line
	2550 2450 2550 2550
$Comp
L Connector_Generic:Conn_01x01 J122
U 1 1 60A00206
P 7650 2150
F 0 "J122" H 7730 2192 50  0000 L CNN
F 1 "Conn_01x01" H 7730 2101 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 7650 2150 50  0001 C CNN
F 3 "~" H 7650 2150 50  0001 C CNN
	1    7650 2150
	-1   0    0    1   
$EndComp
Connection ~ 7850 2150
Wire Wire Line
	7850 2150 7850 2100
Text Label 9150 5300 0    50   ~ 0
RESPONDER_GPIO13
Text Label 8400 5300 2    50   ~ 0
DUT_GPIO10
Wire Wire Line
	8400 5300 9150 5300
Text Label 8400 5450 2    50   ~ 0
DUT_GPIO9
Text Label 9150 5450 0    50   ~ 0
RESPONDER_GPIO12
Text Label 8400 5600 2    50   ~ 0
DUT_GPIO11
Text Label 9150 5600 0    50   ~ 0
RESPONDER_GPIO14
Text Label 8400 5750 2    50   ~ 0
DUT_GPIO6
Text Label 9150 5750 0    50   ~ 0
RESPONDER_GPIO15
Text Label 8400 5900 2    50   ~ 0
DUT_GPIO5
Text Label 9150 5900 0    50   ~ 0
RESPONDER_GPIO27
Wire Wire Line
	8400 5450 9150 5450
Wire Wire Line
	8400 5600 9150 5600
Wire Wire Line
	8400 5750 9150 5750
Wire Wire Line
	8400 5900 9150 5900
Text Label 8750 5300 0    50   ~ 0
SI
Text Label 8750 5450 0    50   ~ 0
SO
Text Label 8750 5600 0    50   ~ 0
SCK
Text Label 8750 5750 0    50   ~ 0
CS
Text Label 8750 5900 0    50   ~ 0
INT
Text Notes 8300 5100 0    50   ~ 0
SPI connection to DUT MCP2515
Text Notes 4900 5100 0    50   ~ 0
Controller CAN
Text Notes 6200 5000 0    50   ~ 0
Responder CAN
Text Label 3300 4650 2    50   ~ 0
CONTROLLER_GPIO14
Text Label 3550 4650 0    50   ~ 0
POWER_ON
Wire Wire Line
	3550 4650 3300 4650
Text Label 3300 4800 2    50   ~ 0
CONTROLLER_GPIO15
Text Label 3550 4800 0    50   ~ 0
DUT_PRESENT
Wire Wire Line
	3550 4800 3300 4800
NoConn ~ 5500 1750
$EndSCHEMATC
