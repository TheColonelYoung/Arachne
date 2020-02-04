EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 9 9
Title "VRASSEO - Main sensoric board"
Date "2018-09-09"
Rev "0.2"
Comp "VUT FIT - STRaDe"
Comment1 "Author: Petr Malaník"
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text HLabel 4550 2300 2    50   Input ~ 0
USB_D+
Text GLabel 2650 1800 2    50   Input ~ 0
USB_VCC
$Comp
L Connector:USB_B J?
U 1 1 5E44C3FC
P 1850 2400
F 0 "J?" H 1905 2867 50  0000 C CNN
F 1 "USB_B" H 1905 2776 50  0000 C CNN
F 2 "Connector_USB:USB_B_OST_USB-B1HSxx_Horizontal" H 2000 2350 50  0001 C CNN
F 3 " ~" H 2000 2350 50  0001 C CNN
	1    1850 2400
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5E44C3FD
P 1850 3200
F 0 "#PWR?" H 1850 2950 50  0001 C CNN
F 1 "GND" H 1855 3027 50  0000 C CNN
F 2 "" H 1850 3200 50  0001 C CNN
F 3 "" H 1850 3200 50  0001 C CNN
	1    1850 3200
	1    0    0    -1  
$EndComp
$Comp
L Device:Ferrite_Bead L?
U 1 1 5E442BE2
P 1550 3000
F 0 "L?" V 1276 3000 50  0000 C CNN
F 1 "50R@100MHz, 3A" V 1367 3000 50  0000 C CNN
F 2 "Inductor_SMD:L_1206_3216Metric" V 1480 3000 50  0001 C CNN
F 3 "~" H 1550 3000 50  0001 C CNN
	1    1550 3000
	1    0    0    -1  
$EndComp
Wire Wire Line
	1850 2800 1850 3200
Wire Wire Line
	1750 2800 1550 2800
Wire Wire Line
	1550 2800 1550 2850
$Comp
L power:GND #PWR?
U 1 1 5E442BE3
P 1550 3200
F 0 "#PWR?" H 1550 2950 50  0001 C CNN
F 1 "GND" H 1555 3027 50  0000 C CNN
F 2 "" H 1550 3200 50  0001 C CNN
F 3 "" H 1550 3200 50  0001 C CNN
	1    1550 3200
	1    0    0    -1  
$EndComp
Wire Wire Line
	1550 3150 1550 3200
Text Notes 2650 1600 0    79   ~ 0
USB interface
Wire Wire Line
	2150 2400 2200 2400
Wire Wire Line
	2150 2500 2200 2500
Wire Wire Line
	3550 2500 3550 2400
Wire Wire Line
	3700 2900 3700 2500
Wire Wire Line
	3150 2900 3700 2900
Wire Wire Line
	3150 1900 3700 1900
Wire Wire Line
	2550 2400 2550 2200
$Comp
L power:GND #PWR?
U 1 1 5E442BE4
P 3550 2500
F 0 "#PWR?" H 3550 2250 50  0001 C CNN
F 1 "GND" H 3555 2327 50  0000 C CNN
F 2 "" H 3550 2500 50  0001 C CNN
F 3 "" H 3550 2500 50  0001 C CNN
	1    3550 2500
	1    0    0    -1  
$EndComp
Wire Wire Line
	2200 2900 2950 2900
Wire Wire Line
	2200 2500 2200 2900
Wire Wire Line
	3700 1900 3700 2300
Wire Wire Line
	2950 1900 2200 1900
Wire Wire Line
	2200 1900 2200 2400
Text Label 4250 2500 0    50   ~ 0
USB_D-
Text Label 4250 2300 0    50   ~ 0
USB_D+
Wire Wire Line
	4200 2500 4550 2500
Wire Wire Line
	4200 2300 4550 2300
Text Label 2200 2900 0    50   ~ 0
USB-
Text Label 2200 1900 0    50   ~ 0
USB+
Text Label 2550 2200 2    50   ~ 0
USB_VCC
Wire Wire Line
	3700 2500 3900 2500
Wire Wire Line
	3700 2300 3900 2300
$Comp
L Device:R R?
U 1 1 5E44C3FF
P 4050 2500
F 0 "R?" V 4250 2500 50  0000 C CNN
F 1 "22R" V 4150 2500 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 3980 2500 50  0001 C CNN
F 3 "~" H 4050 2500 50  0001 C CNN
	1    4050 2500
	0    1    1    0   
$EndComp
$Comp
L Device:R R?
U 1 1 5E44C3FE
P 4050 2300
F 0 "R?" V 3843 2300 50  0000 C CNN
F 1 "22R" V 3934 2300 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 3980 2300 50  0001 C CNN
F 3 "~" H 4050 2300 50  0001 C CNN
	1    4050 2300
	0    1    1    0   
$EndComp
$Comp
L vrasseo-rescue:USBLC6-2SC6-Power_Protection U5
U 1 1 5E442BDD
P 3050 2400
AR Path="/5E442BDD" Ref="U5"  Part="1" 
AR Path="/5BA4084F/5E442BDD" Ref="U5"  Part="1" 
AR Path="/5DA29F03/5E442BDD" Ref="U5"  Part="1" 
AR Path="/5E43C48F/5E442BDD" Ref="U?"  Part="1" 
F 0 "U?" V 2800 2850 50  0000 C CNN
F 1 "USBLC6-2SC6" V 2700 2850 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23-6" H 2300 2800 50  0001 C CNN
F 3 "http://www2.st.com/resource/en/datasheet/CD00050750.pdf" H 3250 2750 50  0001 C CNN
	1    3050 2400
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2150 2200 2550 2200
Text HLabel 4550 2500 2    50   Input ~ 0
USB_D-
Connection ~ 2550 2200
Wire Wire Line
	2650 1800 2550 1800
Wire Wire Line
	2550 1800 2550 2200
$Comp
L TCY_connectors:PROG_CONN_STM_6p J?
U 1 1 5E442BE5
P 3050 4800
F 0 "J?" H 3050 5215 50  0000 C CNN
F 1 "PROG_CONN_STM_6p" H 3050 5124 50  0000 C CNN
F 2 "Connector_IDC:IDC-Header_2x03_P2.54mm_Horizontal" H 2850 5200 50  0001 C CNN
F 3 "" H 2850 5200 50  0001 C CNN
	1    3050 4800
	1    0    0    -1  
$EndComp
Wire Wire Line
	3550 4800 3700 4800
Text HLabel 3700 4800 2    79   Input ~ 0
RESET
Text HLabel 3700 4950 2    79   Input ~ 0
SWDIO
Text HLabel 2400 4950 0    79   Input ~ 0
SWCLK
$Comp
L power:+5V #PWR?
U 1 1 5E44C400
P 2400 4800
F 0 "#PWR?" H 2400 4650 50  0001 C CNN
F 1 "+5V" V 2415 4928 50  0000 L CNN
F 2 "" H 2400 4800 50  0001 C CNN
F 3 "" H 2400 4800 50  0001 C CNN
	1    2400 4800
	0    -1   -1   0   
$EndComp
$Comp
L power:+3.3V #PWR?
U 1 1 5E442BE7
P 3700 4650
F 0 "#PWR?" H 3700 4500 50  0001 C CNN
F 1 "+3.3V" V 3715 4778 50  0000 L CNN
F 2 "" H 3700 4650 50  0001 C CNN
F 3 "" H 3700 4650 50  0001 C CNN
	1    3700 4650
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5E44C401
P 2400 4650
F 0 "#PWR?" H 2400 4400 50  0001 C CNN
F 1 "GND" V 2405 4522 50  0000 R CNN
F 2 "" H 2400 4650 50  0001 C CNN
F 3 "" H 2400 4650 50  0001 C CNN
	1    2400 4650
	0    1    1    0   
$EndComp
Wire Wire Line
	2400 4650 2550 4650
Wire Wire Line
	2400 4800 2550 4800
Wire Wire Line
	2550 4950 2400 4950
Wire Wire Line
	3550 4950 3700 4950
Wire Wire Line
	3700 4650 3550 4650
Text Notes 2350 4300 0    79   ~ 0
Programming connector
$Comp
L Connector_Generic:Conn_01x03 J?
U 1 1 5E442BE9
P 2900 6600
F 0 "J?" H 2818 6275 50  0000 C CNN
F 1 "DBG_UART" H 2818 6366 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 2900 6600 50  0001 C CNN
F 3 "~" H 2900 6600 50  0001 C CNN
	1    2900 6600
	-1   0    0    1   
$EndComp
Wire Wire Line
	3100 6500 3300 6500
Wire Wire Line
	3100 6600 3300 6600
Wire Wire Line
	3100 6700 3200 6700
Wire Wire Line
	3200 6700 3200 6800
$Comp
L power:GND #PWR?
U 1 1 5E442BEA
P 3200 6800
F 0 "#PWR?" H 3200 6550 50  0001 C CNN
F 1 "GND" H 3205 6627 50  0000 C CNN
F 2 "" H 3200 6800 50  0001 C CNN
F 3 "" H 3200 6800 50  0001 C CNN
	1    3200 6800
	1    0    0    -1  
$EndComp
Text HLabel 3300 6500 2    79   Input ~ 0
DBG_RX
Text HLabel 3300 6600 2    79   Input ~ 0
DBG_TX
$Comp
L Device:R_Pack04 RN?
U 1 1 5E442BEB
P 7675 2450
F 0 "RN?" V 7258 2450 50  0000 C CNN
F 1 "R_Pack04" V 7349 2450 50  0000 C CNN
F 2 "Resistor_SMD:R_Array_Convex_4x0603" V 7950 2450 50  0001 C CNN
F 3 "~" H 7675 2450 50  0001 C CNN
	1    7675 2450
	0    1    1    0   
$EndComp
$Comp
L Device:LED D?
U 1 1 5E442BEC
P 7075 2250
F 0 "D?" H 6975 2300 50  0000 C CNN
F 1 "YELLOW" H 7050 2125 50  0000 C CNN
F 2 "LED_SMD:LED_0603_1608Metric_Castellated" H 7075 2250 50  0001 C CNN
F 3 "~" H 7075 2250 50  0001 C CNN
	1    7075 2250
	-1   0    0    1   
$EndComp
$Comp
L Device:LED D?
U 1 1 5E442BED
P 7075 2550
F 0 "D?" H 6975 2600 50  0000 C CNN
F 1 "BLUE" H 7075 2450 50  0000 C CNN
F 2 "LED_SMD:LED_0603_1608Metric_Castellated" H 7075 2550 50  0001 C CNN
F 3 "~" H 7075 2550 50  0001 C CNN
	1    7075 2550
	-1   0    0    1   
$EndComp
$Comp
L Device:LED D?
U 1 1 5E442BEE
P 7075 2850
F 0 "D?" H 6975 2900 50  0000 C CNN
F 1 "BLUE" H 7075 2750 50  0000 C CNN
F 2 "LED_SMD:LED_0603_1608Metric_Castellated" H 7075 2850 50  0001 C CNN
F 3 "~" H 7075 2850 50  0001 C CNN
	1    7075 2850
	-1   0    0    1   
$EndComp
$Comp
L Device:LED D?
U 1 1 5E442BEF
P 7075 1950
F 0 "D?" H 6975 2000 50  0000 C CNN
F 1 "GREEN" H 7075 1825 50  0000 C CNN
F 2 "LED_SMD:LED_0603_1608Metric_Castellated" H 7075 1950 50  0001 C CNN
F 3 "~" H 7075 1950 50  0001 C CNN
	1    7075 1950
	-1   0    0    1   
$EndComp
Wire Wire Line
	6925 2250 6925 2550
Connection ~ 6925 2550
Wire Wire Line
	6925 2550 6925 2850
Wire Wire Line
	6925 1950 6925 2250
Connection ~ 6925 2250
Connection ~ 6925 1950
$Comp
L power:+3.3V #PWR?
U 1 1 5DAA918B
P 6925 1950
F 0 "#PWR?" H 6925 1800 50  0001 C CNN
F 1 "+3.3V" V 6940 2078 50  0000 L CNN
F 2 "" H 6925 1950 50  0001 C CNN
F 3 "" H 6925 1950 50  0001 C CNN
	1    6925 1950
	0    -1   -1   0   
$EndComp
Wire Wire Line
	7475 2450 7300 2450
Wire Wire Line
	7300 2450 7300 2550
Wire Wire Line
	7300 2550 7225 2550
Wire Wire Line
	7475 2350 7300 2350
Wire Wire Line
	7300 2350 7300 2250
Wire Wire Line
	7300 2250 7225 2250
Wire Wire Line
	7225 2850 7375 2850
Wire Wire Line
	7475 2550 7375 2550
Wire Wire Line
	7375 2550 7375 2850
Wire Wire Line
	7475 2250 7375 2250
Wire Wire Line
	7375 2250 7375 1950
Wire Wire Line
	7375 1950 7225 1950
Wire Wire Line
	7875 2550 8150 2550
Wire Wire Line
	7875 2450 8150 2450
Text HLabel 8150 2450 2    79   Input ~ 0
DBG_RX
Text HLabel 8150 2550 2    79   Input ~ 0
DBG_TX
Text Notes 7000 1650 0    79   ~ 0
LED signalization
$Comp
L power:GND #PWR?
U 1 1 5DAC4CBF
P 8150 2250
F 0 "#PWR?" H 8150 2000 50  0001 C CNN
F 1 "GND" V 8155 2122 50  0000 R CNN
F 2 "" H 8150 2250 50  0001 C CNN
F 3 "" H 8150 2250 50  0001 C CNN
	1    8150 2250
	0    -1   -1   0   
$EndComp
Wire Wire Line
	8150 2250 7875 2250
Text HLabel 8150 2350 2    79   Input ~ 0
STAT
Wire Wire Line
	7875 2350 8150 2350
Text Label 3575 1900 0    50   ~ 0
D+
Text Label 3575 2900 0    50   ~ 0
D-
Text Notes 6375 2625 0    79   ~ 0
POWER\nSTAT\nDBG_RX\nDBG_TX
$EndSCHEMATC
