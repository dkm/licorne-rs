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
L custom:EK-TM4C123GXL U1
U 1 1 5FD16A49
P 5300 3100
F 0 "U1" H 5300 4365 50  0000 C CNN
F 1 "EK-TM4C123GXL" H 5300 4274 50  0000 C CNN
F 2 "custom:Launchpad_4x10" H 5300 3100 50  0001 C CNN
F 3 "" H 5300 3100 50  0001 C CNN
	1    5300 3100
	1    0    0    -1  
$EndComp
Wire Wire Line
	4500 2400 4050 2400
NoConn ~ 4500 3600
NoConn ~ 4500 3400
Text GLabel 4500 3900 0    50   Input ~ 0
pe2_epd_busy
Text GLabel 4500 3500 0    50   Input ~ 0
pd1_epd_rst
Text GLabel 4500 2500 0    50   Input ~ 0
pe4_epd_dc
NoConn ~ 6100 2800
$Comp
L power:GND #PWR0101
U 1 1 5FD46763
P 7600 3300
F 0 "#PWR0101" H 7600 3050 50  0001 C CNN
F 1 "GND" H 7605 3127 50  0000 C CNN
F 2 "" H 7600 3300 50  0001 C CNN
F 3 "" H 7600 3300 50  0001 C CNN
	1    7600 3300
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0102
U 1 1 5FD47FB7
P 7250 2850
F 0 "#PWR0102" H 7250 2700 50  0001 C CNN
F 1 "+3.3V" H 7265 3023 50  0000 C CNN
F 2 "" H 7250 2850 50  0001 C CNN
F 3 "" H 7250 2850 50  0001 C CNN
	1    7250 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	4350 2100 4350 2050
$Comp
L power:+3.3V #PWR0103
U 1 1 5FD4CA00
P 4350 2050
F 0 "#PWR0103" H 4350 1900 50  0001 C CNN
F 1 "+3.3V" H 4365 2223 50  0000 C CNN
F 2 "" H 4350 2050 50  0001 C CNN
F 3 "" H 4350 2050 50  0001 C CNN
	1    4350 2050
	1    0    0    -1  
$EndComp
Text GLabel 4500 2700 0    50   Input ~ 0
pb4_rotary_pb
Text GLabel 2050 1600 0    50   Input ~ 0
pb4_rotary_pb
Text GLabel 4050 2400 0    50   Input ~ 0
pb1_rotary_pa
Text GLabel 2050 1700 0    50   Input ~ 0
pb1_rotary_pa
Text GLabel 6100 3500 2    50   Input ~ 0
pc4_rotary_switch
Text GLabel 2050 1800 0    50   Input ~ 0
pc4_rotary_switch
Wire Wire Line
	2050 2000 2000 2000
Wire Wire Line
	1800 2000 1800 2150
$Comp
L power:GND #PWR0104
U 1 1 5FD203F3
P 1800 2150
F 0 "#PWR0104" H 1800 1900 50  0001 C CNN
F 1 "GND" H 1805 1977 50  0000 C CNN
F 2 "" H 1800 2150 50  0001 C CNN
F 3 "" H 1800 2150 50  0001 C CNN
	1    1800 2150
	1    0    0    -1  
$EndComp
NoConn ~ 2550 1600
NoConn ~ 2550 1700
NoConn ~ 2550 1800
NoConn ~ 2550 1900
Text GLabel 6100 3000 2    50   Input ~ 0
pa2_ssi0_mosi
Text GLabel 8300 3000 2    50   Input ~ 0
pa2_ssi0_mosi
Text GLabel 4500 2800 0    50   Input ~ 0
pa5_ssi0_sck
Text GLabel 8300 3100 2    50   Input ~ 0
pa5_ssi0_sck
Text GLabel 8300 3200 2    50   Input ~ 0
pe2_epd_busy
Text GLabel 6100 3700 2    50   Input ~ 0
pc6_epd_cs
Text GLabel 8300 3300 2    50   Input ~ 0
pc6_epd_cs
Text GLabel 8300 3400 2    50   Input ~ 0
pd1_epd_rst
Text GLabel 7800 3000 0    50   Input ~ 0
pe4_epd_dc
NoConn ~ 7800 3300
NoConn ~ 7800 3400
$Comp
L Connector_Generic:Conn_02x05_Counter_Clockwise J6
U 1 1 5FD25B96
P 8000 3200
F 0 "J6" H 8050 3617 50  0000 C CNN
F 1 "epd_conn" H 8050 3526 50  0000 C CNN
F 2 "Connector_IDC:IDC-Header_2x05_P2.54mm_Vertical" H 8000 3200 50  0001 C CNN
F 3 "~" H 8000 3200 50  0001 C CNN
	1    8000 3200
	1    0    0    -1  
$EndComp
Wire Wire Line
	7800 3200 7600 3200
Wire Wire Line
	7600 3200 7600 3300
Wire Wire Line
	7800 3100 7250 3100
Wire Wire Line
	7250 3100 7250 2850
$Comp
L power:+3.3V #PWR0105
U 1 1 5FD37559
P 2600 3850
F 0 "#PWR0105" H 2600 3700 50  0001 C CNN
F 1 "+3.3V" H 2615 4023 50  0000 C CNN
F 2 "" H 2600 3850 50  0001 C CNN
F 3 "" H 2600 3850 50  0001 C CNN
	1    2600 3850
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0106
U 1 1 5FD37E4D
P 1800 3650
F 0 "#PWR0106" H 1800 3400 50  0001 C CNN
F 1 "GND" H 1805 3477 50  0000 C CNN
F 2 "" H 1800 3650 50  0001 C CNN
F 3 "" H 1800 3650 50  0001 C CNN
	1    1800 3650
	1    0    0    -1  
$EndComp
Text GLabel 4500 2900 0    50   Input ~ 0
pa6_i2c1_bme680_scl
Text GLabel 2600 4050 0    50   Input ~ 0
pa6_i2c1_bme680_scl
Text GLabel 4500 3000 0    50   Input ~ 0
pa7_i2c1_bme680_sda
Text GLabel 2600 4150 0    50   Input ~ 0
pa7_i2c1_bme680_sda
Text GLabel 6100 2200 2    50   Input ~ 0
pb2_i2c0_rtc_scl
Text GLabel 6100 3400 2    50   Input ~ 0
pb3_i2c0_rtc_sda
Text GLabel 6100 3900 2    50   Input ~ 0
pd6_rtc_int_sqw
$Comp
L Connector_Generic:Conn_02x05_Counter_Clockwise J4
U 1 1 5FD4B85E
P 4000 1200
F 0 "J4" H 4050 1617 50  0000 C CNN
F 1 "ws2812_conn" H 4050 1526 50  0000 C CNN
F 2 "Connector_IDC:IDC-Header_2x05_P2.54mm_Vertical" H 4000 1200 50  0001 C CNN
F 3 "~" H 4000 1200 50  0001 C CNN
	1    4000 1200
	1    0    0    -1  
$EndComp
Text GLabel 4500 3700 0    50   Input ~ 0
pd3_ssi1_ws2812_mosi
Text GLabel 3800 1000 0    50   Input ~ 0
pd3_ssi1_ws2812_mosi
Wire Wire Line
	4300 1000 4850 1000
Wire Wire Line
	4850 1000 4850 900 
$Comp
L power:+5V #PWR0109
U 1 1 5FD51326
P 4850 900
F 0 "#PWR0109" H 4850 750 50  0001 C CNN
F 1 "+5V" H 4865 1073 50  0000 C CNN
F 2 "" H 4850 900 50  0001 C CNN
F 3 "" H 4850 900 50  0001 C CNN
	1    4850 900 
	1    0    0    -1  
$EndComp
Wire Wire Line
	4300 1100 4650 1100
Wire Wire Line
	4650 1100 4650 1350
$Comp
L power:GND #PWR0110
U 1 1 5FD5288D
P 4650 1350
F 0 "#PWR0110" H 4650 1100 50  0001 C CNN
F 1 "GND" H 4655 1177 50  0000 C CNN
F 2 "" H 4650 1350 50  0001 C CNN
F 3 "" H 4650 1350 50  0001 C CNN
	1    4650 1350
	1    0    0    -1  
$EndComp
NoConn ~ 4300 1200
NoConn ~ 4300 1300
NoConn ~ 4300 1400
NoConn ~ 3800 1400
NoConn ~ 3800 1300
NoConn ~ 3800 1200
NoConn ~ 3800 1100
$Comp
L Connector_Generic:Conn_02x05_Counter_Clockwise J3
U 1 1 5FD5974D
P 3200 1650
F 0 "J3" H 3308 1831 50  0000 C CNN
F 1 "toggle_conn" H 3308 1740 50  0000 C CNN
F 2 "Connector_IDC:IDC-Header_2x05_P2.54mm_Vertical" H 3200 1650 50  0001 C CNN
F 3 "~" H 3200 1650 50  0001 C CNN
	1    3200 1650
	-1   0    0    1   
$EndComp
Wire Wire Line
	4500 2200 3800 2200
Wire Wire Line
	3800 2200 3800 1750
Wire Wire Line
	3800 1750 3400 1750
Wire Wire Line
	3400 1850 3500 1850
Wire Wire Line
	3500 1850 3500 2050
$Comp
L power:GND #PWR0111
U 1 1 5FD5B8E2
P 3500 2050
F 0 "#PWR0111" H 3500 1800 50  0001 C CNN
F 1 "GND" H 3505 1877 50  0000 C CNN
F 2 "" H 3500 2050 50  0001 C CNN
F 3 "" H 3500 2050 50  0001 C CNN
	1    3500 2050
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0112
U 1 1 5FD5F714
P 3500 3200
F 0 "#PWR0112" H 3500 3050 50  0001 C CNN
F 1 "+5V" H 3515 3373 50  0000 C CNN
F 2 "" H 3500 3200 50  0001 C CNN
F 3 "" H 3500 3200 50  0001 C CNN
	1    3500 3200
	1    0    0    -1  
$EndComp
$Comp
L power:PWR_FLAG #FLG0101
U 1 1 5FD6019B
P 3600 3450
F 0 "#FLG0101" H 3600 3525 50  0001 C CNN
F 1 "PWR_FLAG" H 3600 3623 50  0000 C CNN
F 2 "" H 3600 3450 50  0001 C CNN
F 3 "~" H 3600 3450 50  0001 C CNN
	1    3600 3450
	1    0    0    -1  
$EndComp
Wire Wire Line
	4500 3200 3900 3200
Wire Wire Line
	3900 3200 3900 3450
Wire Wire Line
	3900 3450 3600 3450
Wire Wire Line
	3500 3450 3500 3200
Connection ~ 3600 3450
Wire Wire Line
	3600 3450 3500 3450
$Comp
L power:GND #PWR0113
U 1 1 5FD656F2
P 4150 3300
F 0 "#PWR0113" H 4150 3050 50  0001 C CNN
F 1 "GND" H 4155 3127 50  0000 C CNN
F 2 "" H 4150 3300 50  0001 C CNN
F 3 "" H 4150 3300 50  0001 C CNN
	1    4150 3300
	1    0    0    -1  
$EndComp
NoConn ~ 4500 3800
NoConn ~ 4500 4000
NoConn ~ 4500 4100
NoConn ~ 6100 2300
NoConn ~ 6100 2400
NoConn ~ 6100 2700
NoConn ~ 6100 2900
NoConn ~ 6100 3200
NoConn ~ 6100 3300
NoConn ~ 6100 3600
NoConn ~ 6100 3800
NoConn ~ 6100 4000
NoConn ~ 6100 4100
Wire Wire Line
	6100 2100 6150 2100
Wire Wire Line
	6150 2100 6150 1550
Wire Wire Line
	6150 1550 6200 1550
Wire Wire Line
	6300 1550 6300 1700
$Comp
L power:GND #PWR0114
U 1 1 5FD72639
P 6300 1700
F 0 "#PWR0114" H 6300 1450 50  0001 C CNN
F 1 "GND" H 6305 1527 50  0000 C CNN
F 2 "" H 6300 1700 50  0001 C CNN
F 3 "" H 6300 1700 50  0001 C CNN
	1    6300 1700
	1    0    0    -1  
$EndComp
$Comp
L power:PWR_FLAG #FLG0103
U 1 1 5FD72DCA
P 6200 1550
F 0 "#FLG0103" H 6200 1625 50  0001 C CNN
F 1 "PWR_FLAG" H 6200 1723 50  0000 C CNN
F 2 "" H 6200 1550 50  0001 C CNN
F 3 "~" H 6200 1550 50  0001 C CNN
	1    6200 1550
	1    0    0    -1  
$EndComp
Connection ~ 6200 1550
Wire Wire Line
	6200 1550 6300 1550
NoConn ~ 6100 2500
NoConn ~ 4500 2600
Wire Wire Line
	4350 2100 4500 2100
Wire Wire Line
	4150 3300 4500 3300
Wire Wire Line
	4500 3300 5450 3300
Wire Wire Line
	5450 3300 5450 2100
Connection ~ 4500 3300
Wire Wire Line
	5450 2100 6100 2100
Connection ~ 6100 2100
NoConn ~ 4500 2300
$Comp
L Connector_Generic:Conn_02x05_Counter_Clockwise J2
U 1 1 5FD12971
P 2250 1800
F 0 "J2" H 2300 2217 50  0000 C CNN
F 1 "rotary_conn" H 2300 2126 50  0000 C CNN
F 2 "Connector_IDC:IDC-Header_2x05_P2.54mm_Vertical" H 2250 1800 50  0001 C CNN
F 3 "~" H 2250 1800 50  0001 C CNN
	1    2250 1800
	1    0    0    -1  
$EndComp
NoConn ~ 2050 1900
Wire Wire Line
	2550 2000 2550 2150
Wire Wire Line
	2550 2150 2000 2150
Wire Wire Line
	2000 2150 2000 2000
Connection ~ 2000 2000
Wire Wire Line
	2000 2000 1800 2000
Text GLabel 7100 750  0    50   Input ~ 0
pd6_rtc_int_sqw
$Comp
L power:GND #PWR0108
U 1 1 5FD46773
P 7100 1150
F 0 "#PWR0108" H 7100 900 50  0001 C CNN
F 1 "GND" H 7105 977 50  0000 C CNN
F 2 "" H 7100 1150 50  0001 C CNN
F 3 "" H 7100 1150 50  0001 C CNN
	1    7100 1150
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0107
U 1 1 5FD45C2F
P 6200 1000
F 0 "#PWR0107" H 6200 850 50  0001 C CNN
F 1 "+3.3V" H 6215 1173 50  0000 C CNN
F 2 "" H 6200 1000 50  0001 C CNN
F 3 "" H 6200 1000 50  0001 C CNN
	1    6200 1000
	1    0    0    -1  
$EndComp
Text GLabel 7100 950  0    50   Input ~ 0
pb3_i2c0_rtc_sda
Text GLabel 7100 850  0    50   Input ~ 0
pb2_i2c0_rtc_scl
$Comp
L Connector_Generic:Conn_01x06 J5
U 1 1 601E28B5
P 7300 850
F 0 "J5" H 7380 842 50  0000 L CNN
F 1 "RTC" H 7380 751 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x06_P2.54mm_Vertical" H 7300 850 50  0001 C CNN
F 3 "~" H 7300 850 50  0001 C CNN
	1    7300 850 
	1    0    0    -1  
$EndComp
Wire Wire Line
	7100 1050 6200 1050
Wire Wire Line
	6200 1050 6200 1000
$Comp
L Connector_Generic:Conn_01x06 J1
U 1 1 6020DB75
P 2800 4050
F 0 "J1" H 2880 4042 50  0000 L CNN
F 1 "BME680" H 2880 3951 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x06_P2.54mm_Vertical" H 2800 4050 50  0001 C CNN
F 3 "~" H 2800 4050 50  0001 C CNN
	1    2800 4050
	1    0    0    -1  
$EndComp
Wire Wire Line
	1800 3650 2300 3650
NoConn ~ 2600 4250
NoConn ~ 2900 1450
NoConn ~ 2900 1550
NoConn ~ 2900 1650
NoConn ~ 2900 1750
NoConn ~ 2900 1850
NoConn ~ 3400 1550
NoConn ~ 3400 1650
NoConn ~ 3400 1450
NoConn ~ 7100 650 
Wire Wire Line
	2300 3650 2300 3950
Wire Wire Line
	2300 3950 2600 3950
NoConn ~ 2600 4350
$EndSCHEMATC
