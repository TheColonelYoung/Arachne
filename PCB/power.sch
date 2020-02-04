EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 8 9
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
L power:GND #PWR?
U 1 1 5CDAE692
P 1700 1500
F 0 "#PWR?" H 1700 1250 50  0001 C CNN
F 1 "GND" H 1700 1325 50  0000 C CNN
F 2 "" H 1700 1500 50  0001 C CNN
F 3 "" H 1700 1500 50  0001 C CNN
	1    1700 1500
	1    0    0    -1  
$EndComp
Wire Wire Line
	1450 1400 1550 1400
Wire Wire Line
	1700 1400 1700 1500
$Comp
L Connector:TestPoint TP?
U 1 1 5CDB03CA
P 1550 1450
F 0 "TP?" H 1750 1600 50  0000 R CNN
F 1 "GND" H 1750 1700 50  0000 R CNN
F 2 "TestPoint:TestPoint_THTPad_D1.5mm_Drill0.7mm" H 1750 1450 50  0001 C CNN
F 3 "~" H 1750 1450 50  0001 C CNN
	1    1550 1450
	-1   0    0    1   
$EndComp
Wire Wire Line
	1550 1400 1700 1400
Connection ~ 1550 1400
$Comp
L Device:Q_PMOS_GDS Q?
U 1 1 5CDB36E0
P 2000 1400
F 0 "Q?" V 2350 1350 50  0000 C CNN
F 1 "AOD4189" V 2250 1350 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:TO-252-2" H 2200 1500 50  0001 C CNN
F 3 "~" H 2000 1400 50  0001 C CNN
	1    2000 1400
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2000 1600 2000 1650
$Comp
L Device:R R?
U 1 1 5CDB7101
P 2000 1850
F 0 "R?" H 2100 1900 50  0000 L CNN
F 1 "100k" H 2100 1800 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 1930 1850 50  0001 C CNN
F 3 "~" H 2000 1850 50  0001 C CNN
	1    2000 1850
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5CDB7AAF
P 2000 2250
F 0 "#PWR?" H 2000 2000 50  0001 C CNN
F 1 "GND" H 2150 2175 50  0000 C CNN
F 2 "" H 2000 2250 50  0001 C CNN
F 3 "" H 2000 2250 50  0001 C CNN
	1    2000 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	2200 1300 2250 1300
$Comp
L Device:D_Zener D?
U 1 1 5CDB8D93
P 2250 1450
F 0 "D?" V 2200 1500 50  0000 L CNN
F 1 "1SMB5925" V 2300 1500 50  0000 L CNN
F 2 "Diode_SMD:D_SMB" H 2250 1450 50  0001 C CNN
F 3 "~" H 2250 1450 50  0001 C CNN
	1    2250 1450
	0    1    1    0   
$EndComp
Wire Wire Line
	2250 1600 2250 1650
Wire Wire Line
	2250 1650 2000 1650
Connection ~ 2000 1650
Wire Wire Line
	2000 1650 2000 1700
$Comp
L Connector:TestPoint TP?
U 1 1 5CDBA330
P 2700 1700
F 0 "TP?" H 2750 1900 50  0000 L CNN
F 1 "24V" H 2750 1800 50  0000 L CNN
F 2 "TestPoint:TestPoint_THTPad_D1.5mm_Drill0.7mm" H 2900 1700 50  0001 C CNN
F 3 "~" H 2900 1700 50  0001 C CNN
	1    2700 1700
	1    0    0    -1  
$EndComp
Wire Wire Line
	1550 1450 1550 1400
$Comp
L Regulator_Switching:L5973D U?
U 1 1 5CE05CCA
P 2750 4300
F 0 "U?" H 2950 4650 50  0000 C CNN
F 1 "L5973D" H 2950 4550 50  0000 C CNN
F 2 "Package_SO:HSOP-8-1EP_3.9x4.9mm_P1.27mm_EP2.41x3.1mm_ThermalVias" H 2900 3850 50  0001 L CNN
F 3 "http://www.st.com/resource/en/datasheet/l5973d.pdf" H 2750 4300 50  0001 C CNN
	1    2750 4300
	1    0    0    -1  
$EndComp
NoConn ~ 3250 4400
NoConn ~ 2250 4200
$Comp
L Device:C C?
U 1 1 5CE05CD9
P 2150 4700
F 0 "C?" H 2150 4800 50  0000 L CNN
F 1 "22nF" H 2150 4600 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 2188 4550 50  0001 C CNN
F 3 "~" H 2150 4700 50  0001 C CNN
	1    2150 4700
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 5CE05CE3
P 1900 4700
F 0 "C?" H 1900 4800 50  0000 L CNN
F 1 "220nF" H 1900 4600 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 1938 4550 50  0001 C CNN
F 3 "~" H 1900 4700 50  0001 C CNN
	1    1900 4700
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 5CE05CED
P 2150 5150
F 0 "R?" H 2200 5200 50  0000 L CNN
F 1 "4.7k" H 2200 5100 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 2080 5150 50  0001 C CNN
F 3 "~" H 2150 5150 50  0001 C CNN
	1    2150 5150
	1    0    0    -1  
$EndComp
$Comp
L Device:D_Schottky D?
U 1 1 5CE05CF7
P 3450 4800
F 0 "D?" V 3400 4600 50  0000 L CNN
F 1 "STPS340U" V 3500 4450 50  0000 L CNN
F 2 "Diode_SMD:D_SMB" H 3450 4800 50  0001 C CNN
F 3 "~" H 3450 4800 50  0001 C CNN
	1    3450 4800
	0    1    1    0   
$EndComp
$Comp
L Device:L L?
U 1 1 5CE05D01
P 3700 4200
F 0 "L?" V 3800 4300 50  0000 C CNN
F 1 "15uH" V 3800 4100 50  0000 C CNN
F 2 "Inductor_SMD:L_Neosid_Ms85" H 3700 4200 50  0001 C CNN
F 3 "~" H 3700 4200 50  0001 C CNN
	1    3700 4200
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R?
U 1 1 5CE05D0B
P 3950 4350
F 0 "R?" H 4000 4400 50  0000 L CNN
F 1 "20k" H 4000 4300 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 3880 4350 50  0001 C CNN
F 3 "~" H 3950 4350 50  0001 C CNN
	1    3950 4350
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 5CE05D15
P 3950 5150
F 0 "R?" H 4000 5200 50  0000 L CNN
F 1 "10k" H 4000 5100 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 3880 5150 50  0001 C CNN
F 3 "~" H 3950 5150 50  0001 C CNN
	1    3950 5150
	1    0    0    -1  
$EndComp
$Comp
L Device:R_POT RV?
U 1 1 5CE05D1F
P 3950 4750
F 0 "RV?" H 3850 4800 50  0000 R CNN
F 1 "20k" H 3850 4700 50  0000 R CNN
F 2 "Potentiometer_SMD:Potentiometer_Bourns_3214J_Horizontal" H 3950 4750 50  0001 C CNN
F 3 "~" H 3950 4750 50  0001 C CNN
	1    3950 4750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5CE05D29
P 2150 5300
F 0 "#PWR?" H 2150 5050 50  0001 C CNN
F 1 "GND" H 2150 5150 50  0000 C CNN
F 2 "" H 2150 5300 50  0001 C CNN
F 3 "" H 2150 5300 50  0001 C CNN
	1    2150 5300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5CE05D33
P 1900 5300
F 0 "#PWR?" H 1900 5050 50  0001 C CNN
F 1 "GND" H 1900 5150 50  0000 C CNN
F 2 "" H 1900 5300 50  0001 C CNN
F 3 "" H 1900 5300 50  0001 C CNN
	1    1900 5300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5CE05D3D
P 3950 5300
F 0 "#PWR?" H 3950 5050 50  0001 C CNN
F 1 "GND" H 3950 5150 50  0000 C CNN
F 2 "" H 3950 5300 50  0001 C CNN
F 3 "" H 3950 5300 50  0001 C CNN
	1    3950 5300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5CE05D47
P 3450 5300
F 0 "#PWR?" H 3450 5050 50  0001 C CNN
F 1 "GND" H 3450 5150 50  0000 C CNN
F 2 "" H 3450 5300 50  0001 C CNN
F 3 "" H 3450 5300 50  0001 C CNN
	1    3450 5300
	1    0    0    -1  
$EndComp
Wire Wire Line
	3950 4600 3950 4550
Wire Wire Line
	3950 4550 4150 4550
Wire Wire Line
	4150 4550 4150 4750
Wire Wire Line
	4150 4750 4100 4750
Connection ~ 3950 4550
Wire Wire Line
	3950 4550 3950 4500
Wire Wire Line
	3250 4200 3450 4200
Wire Wire Line
	3450 4200 3450 4650
Connection ~ 3450 4200
Wire Wire Line
	3950 4900 3950 4950
Wire Wire Line
	3950 4950 3600 4950
Wire Wire Line
	3600 4950 3600 4300
Wire Wire Line
	3600 4300 3250 4300
Connection ~ 3950 4950
Wire Wire Line
	3950 4950 3950 5000
Wire Wire Line
	3450 4950 3450 5300
Wire Wire Line
	2150 5000 2150 4850
Wire Wire Line
	2150 4550 2150 4400
Wire Wire Line
	2150 4400 2250 4400
Wire Wire Line
	2150 4400 1900 4400
Wire Wire Line
	1900 4400 1900 4550
Connection ~ 2150 4400
Wire Wire Line
	1900 4850 1900 5300
$Comp
L power:GND #PWR?
U 1 1 5CE05D68
P 2800 4750
F 0 "#PWR?" H 2800 4500 50  0001 C CNN
F 1 "GND" H 2800 4600 50  0000 C CNN
F 2 "" H 2800 4750 50  0001 C CNN
F 3 "" H 2800 4750 50  0001 C CNN
	1    2800 4750
	1    0    0    -1  
$EndComp
Wire Wire Line
	2750 4600 2800 4600
Wire Wire Line
	2800 4750 2800 4600
Connection ~ 2800 4600
Wire Wire Line
	2800 4600 2850 4600
Wire Wire Line
	2250 4300 1750 4300
Wire Wire Line
	1750 4300 1750 4400
$Comp
L power:GND #PWR?
U 1 1 5CE05D78
P 1750 4400
F 0 "#PWR?" H 1750 4150 50  0001 C CNN
F 1 "GND" H 1750 4250 50  0000 C CNN
F 2 "" H 1750 4400 50  0001 C CNN
F 3 "" H 1750 4400 50  0001 C CNN
	1    1750 4400
	1    0    0    -1  
$EndComp
$Comp
L Device:D_TVS D?
U 1 1 5CE05D82
P 4700 4500
F 0 "D?" V 4650 4600 50  0000 L CNN
F 1 "7V" V 4750 4600 50  0000 L CNN
F 2 "Diode_SMD:D_SMB" H 4700 4500 50  0001 C CNN
F 3 "~" H 4700 4500 50  0001 C CNN
	1    4700 4500
	0    1    1    0   
$EndComp
Wire Wire Line
	4350 4200 4350 4350
Wire Wire Line
	3950 4200 4350 4200
Wire Wire Line
	4700 4200 4700 4350
$Comp
L Device:CP C?
U 1 1 5CE05D8F
P 4350 4500
F 0 "C?" H 4250 4650 50  0000 L CNN
F 1 "300uF 16V" V 4500 4250 50  0000 L CNN
F 2 "Capacitor_SMD:CP_Elec_8x10" H 4388 4350 50  0001 C CNN
F 3 "~" H 4350 4500 50  0001 C CNN
	1    4350 4500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5CE05D99
P 4350 4650
F 0 "#PWR?" H 4350 4400 50  0001 C CNN
F 1 "GND" H 4350 4500 50  0000 C CNN
F 2 "" H 4350 4650 50  0001 C CNN
F 3 "" H 4350 4650 50  0001 C CNN
	1    4350 4650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5CE05DA3
P 4700 4650
F 0 "#PWR?" H 4700 4400 50  0001 C CNN
F 1 "GND" H 4700 4500 50  0000 C CNN
F 2 "" H 4700 4650 50  0001 C CNN
F 3 "" H 4700 4650 50  0001 C CNN
	1    4700 4650
	1    0    0    -1  
$EndComp
$Comp
L Device:Fuse F?
U 1 1 5CE05DAD
P 4900 4200
F 0 "F?" V 4800 4100 50  0000 C CNN
F 1 "2A" V 4800 4250 50  0000 C CNN
F 2 "Fuse:Fuse_1206_3216Metric" V 4830 4200 50  0001 C CNN
F 3 "~" H 4900 4200 50  0001 C CNN
	1    4900 4200
	0    1    1    0   
$EndComp
Wire Wire Line
	4700 4200 4750 4200
Wire Wire Line
	3450 4200 3550 4200
Wire Wire Line
	3850 4200 3950 4200
Connection ~ 3950 4200
$Comp
L Device:D D?
U 1 1 5CE05DBB
P 4550 4200
F 0 "D?" H 4550 4000 50  0000 C CNN
F 1 "SKL310" H 4600 4100 50  0000 C CNN
F 2 "Diode_SMD:D_SOD-123F" H 4550 4200 50  0001 C CNN
F 3 "~" H 4550 4200 50  0001 C CNN
	1    4550 4200
	-1   0    0    1   
$EndComp
Connection ~ 4700 4200
Wire Wire Line
	4400 4200 4350 4200
Connection ~ 4350 4200
Wire Wire Line
	5050 4200 5100 4200
$Comp
L Connector:TestPoint TP?
U 1 1 5CE05DD3
P 5100 4200
F 0 "TP?" H 5050 4500 50  0000 L CNN
F 1 "6V" H 5050 4400 50  0000 L CNN
F 2 "TestPoint:TestPoint_THTPad_D1.5mm_Drill0.7mm" H 5300 4200 50  0001 C CNN
F 3 "~" H 5300 4200 50  0001 C CNN
	1    5100 4200
	1    0    0    -1  
$EndComp
Connection ~ 5100 4200
$Comp
L Device:CP C?
U 1 1 5CE8E00A
P 6250 1825
F 0 "C?" V 6300 1875 50  0000 L CNN
F 1 "10uF 50V (ceramic)" V 6400 1475 50  0000 L CNN
F 2 "Capacitor_SMD:C_2816_7142Metric_Pad3.20x4.45mm_HandSolder" H 6288 1675 50  0001 C CNN
F 3 "~" H 6250 1825 50  0001 C CNN
	1    6250 1825
	1    0    0    -1  
$EndComp
Wire Wire Line
	6250 1975 6250 2175
$Comp
L power:GND #PWR?
U 1 1 5D43EC49
P 6250 2175
F 0 "#PWR?" H 6250 1925 50  0001 C CNN
F 1 "GND" H 6250 2025 50  0000 C CNN
F 2 "" H 6250 2175 50  0001 C CNN
F 3 "" H 6250 2175 50  0001 C CNN
	1    6250 2175
	1    0    0    -1  
$EndComp
Wire Wire Line
	6250 1675 6250 1375
$Comp
L Device:R R?
U 1 1 5D43EC4A
P 6550 1575
F 0 "R?" H 6650 1625 50  0000 L CNN
F 1 "180k" H 6650 1525 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 6480 1575 50  0001 C CNN
F 3 "~" H 6550 1575 50  0001 C CNN
	1    6550 1575
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 5D43EC4B
P 6550 1975
F 0 "R?" H 6650 2025 50  0000 L CNN
F 1 "10k" H 6650 1925 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 6480 1975 50  0001 C CNN
F 3 "~" H 6550 1975 50  0001 C CNN
	1    6550 1975
	1    0    0    -1  
$EndComp
Wire Wire Line
	6550 1425 6550 1375
Connection ~ 6550 1375
Wire Wire Line
	6550 1725 6550 1775
$Comp
L power:GND #PWR?
U 1 1 5D43EC4C
P 6550 2175
F 0 "#PWR?" H 6550 1925 50  0001 C CNN
F 1 "GND" H 6550 2025 50  0000 C CNN
F 2 "" H 6550 2175 50  0001 C CNN
F 3 "" H 6550 2175 50  0001 C CNN
	1    6550 2175
	1    0    0    -1  
$EndComp
Wire Wire Line
	6550 2125 6550 2175
Connection ~ 6550 1775
Wire Wire Line
	6550 1775 6550 1825
$Comp
L Regulator_Linear:NCP1117-3.3_SOT223 U?
U 1 1 5CDDA441
P 8850 5350
F 0 "U?" H 8850 5650 50  0000 C CNN
F 1 "NCP1117-3.3_SOT223" H 8850 5550 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-223-3_TabPin2" H 8850 5550 50  0001 C CNN
F 3 "http://www.onsemi.com/pub_link/Collateral/NCP1117-D.PDF" H 8950 5100 50  0001 C CNN
	1    8850 5350
	1    0    0    -1  
$EndComp
$Comp
L Regulator_Linear:NCP1117-5.0_SOT223 U?
U 1 1 5CDDE15A
P 8850 3500
F 0 "U?" H 8850 3800 50  0000 C CNN
F 1 "NCP1117-5.0_SOT223" H 8850 3700 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-223-3_TabPin2" H 8850 3700 50  0001 C CNN
F 3 "http://www.onsemi.com/pub_link/Collateral/NCP1117-D.PDF" H 8950 3250 50  0001 C CNN
	1    8850 3500
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 5CDE4694
P 8300 5500
F 0 "C?" H 8450 5500 50  0000 L CNN
F 1 "4.7uF (X7R)" H 8300 5400 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 8338 5350 50  0001 C CNN
F 3 "~" H 8300 5500 50  0001 C CNN
	1    8300 5500
	1    0    0    -1  
$EndComp
Connection ~ 8300 5350
Wire Wire Line
	8300 5350 8550 5350
$Comp
L power:+5V #PWR?
U 1 1 5CDF38D3
P 9900 3500
F 0 "#PWR?" H 9900 3350 50  0001 C CNN
F 1 "+5V" V 9900 3650 50  0000 L CNN
F 2 "" H 9900 3500 50  0001 C CNN
F 3 "" H 9900 3500 50  0001 C CNN
	1    9900 3500
	0    1    1    0   
$EndComp
Wire Wire Line
	9150 3500 9400 3500
Wire Wire Line
	9150 5350 9400 5350
$Comp
L Device:C C?
U 1 1 5CDF7CC9
P 9400 5500
F 0 "C?" H 9150 5500 50  0000 L CNN
F 1 "4.7uF (X7R)" H 8950 5400 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 9438 5350 50  0001 C CNN
F 3 "~" H 9400 5500 50  0001 C CNN
	1    9400 5500
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 5CDF811F
P 9400 3650
F 0 "C?" H 9150 3650 50  0000 L CNN
F 1 "4.7uF (X7R)" H 8950 3550 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 9438 3500 50  0001 C CNN
F 3 "~" H 9400 3650 50  0001 C CNN
	1    9400 3650
	1    0    0    -1  
$EndComp
Connection ~ 9400 3500
$Comp
L Device:C C?
U 1 1 5CDF8469
P 8300 3650
F 0 "C?" H 8450 3650 50  0000 L CNN
F 1 "4.7uF (X7R)" H 8300 3550 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 8338 3500 50  0001 C CNN
F 3 "~" H 8300 3650 50  0001 C CNN
	1    8300 3650
	1    0    0    -1  
$EndComp
Wire Wire Line
	8300 3500 8550 3500
$Comp
L power:GND #PWR?
U 1 1 5CDF9BC9
P 9400 5650
F 0 "#PWR?" H 9400 5400 50  0001 C CNN
F 1 "GND" H 9450 5450 50  0000 C CNN
F 2 "" H 9400 5650 50  0001 C CNN
F 3 "" H 9400 5650 50  0001 C CNN
	1    9400 5650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5CDF9FFC
P 8300 5650
F 0 "#PWR?" H 8300 5400 50  0001 C CNN
F 1 "GND" H 8350 5450 50  0000 C CNN
F 2 "" H 8300 5650 50  0001 C CNN
F 3 "" H 8300 5650 50  0001 C CNN
	1    8300 5650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5CDFA3C0
P 8300 3800
F 0 "#PWR?" H 8300 3550 50  0001 C CNN
F 1 "GND" H 8350 3600 50  0000 C CNN
F 2 "" H 8300 3800 50  0001 C CNN
F 3 "" H 8300 3800 50  0001 C CNN
	1    8300 3800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5CDFA75C
P 9400 3800
F 0 "#PWR?" H 9400 3550 50  0001 C CNN
F 1 "GND" H 9450 3600 50  0000 C CNN
F 2 "" H 9400 3800 50  0001 C CNN
F 3 "" H 9400 3800 50  0001 C CNN
	1    9400 3800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5CDFAB21
P 8850 3800
F 0 "#PWR?" H 8850 3550 50  0001 C CNN
F 1 "GND" H 8900 3600 50  0000 C CNN
F 2 "" H 8850 3800 50  0001 C CNN
F 3 "" H 8850 3800 50  0001 C CNN
	1    8850 3800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5CDFCAF2
P 8850 5650
F 0 "#PWR?" H 8850 5400 50  0001 C CNN
F 1 "GND" H 8900 5450 50  0000 C CNN
F 2 "" H 8850 5650 50  0001 C CNN
F 3 "" H 8850 5650 50  0001 C CNN
	1    8850 5650
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR?
U 1 1 5CE30936
P 9600 5350
F 0 "#PWR?" H 9600 5200 50  0001 C CNN
F 1 "+3.3V" V 9600 5500 50  0000 L CNN
F 2 "" H 9600 5350 50  0001 C CNN
F 3 "" H 9600 5350 50  0001 C CNN
	1    9600 5350
	0    1    1    0   
$EndComp
$Comp
L Device:R R?
U 1 1 5CE5A6FB
P 9650 3650
F 0 "R?" H 9750 3700 50  0000 L CNN
F 1 "10k" H 9750 3600 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 9580 3650 50  0001 C CNN
F 3 "~" H 9650 3650 50  0001 C CNN
	1    9650 3650
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 5CE5A705
P 9650 4150
F 0 "R?" H 9750 4200 50  0000 L CNN
F 1 "10k" H 9750 4100 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 9580 4150 50  0001 C CNN
F 3 "~" H 9650 4150 50  0001 C CNN
	1    9650 4150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5CE5A713
P 9650 4300
F 0 "#PWR?" H 9650 4050 50  0001 C CNN
F 1 "GND" H 9650 4150 50  0000 C CNN
F 2 "" H 9650 4300 50  0001 C CNN
F 3 "" H 9650 4300 50  0001 C CNN
	1    9650 4300
	1    0    0    -1  
$EndComp
Wire Wire Line
	9400 3500 9650 3500
Connection ~ 9650 3500
Wire Wire Line
	9650 3500 9900 3500
Connection ~ 9400 5350
Wire Wire Line
	6250 1375 6550 1375
Connection ~ 1700 1300
Wire Wire Line
	1700 1300 1450 1300
Wire Wire Line
	1700 1300 1800 1300
$Comp
L Connector:TestPoint TP?
U 1 1 5CDAF9A5
P 1700 1300
F 0 "TP?" H 1550 1550 50  0000 L CNN
F 1 "PWR_IN" H 1400 1450 50  0000 L CNN
F 2 "TestPoint:TestPoint_THTPad_D1.5mm_Drill0.7mm" H 1900 1300 50  0001 C CNN
F 3 "~" H 1900 1300 50  0001 C CNN
	1    1700 1300
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint TP?
U 1 1 5CFE62E2
P 9650 3400
F 0 "TP?" H 9750 3550 50  0000 L CNN
F 1 "5V" H 9750 3450 50  0000 L CNN
F 2 "TestPoint:TestPoint_THTPad_D1.5mm_Drill0.7mm" H 9850 3400 50  0001 C CNN
F 3 "~" H 9850 3400 50  0001 C CNN
	1    9650 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	9650 3400 9650 3500
$Comp
L Connector:TestPoint TP?
U 1 1 5CFEFE42
P 9400 5275
F 0 "TP?" H 9500 5425 50  0000 L CNN
F 1 "3V3" H 9500 5325 50  0000 L CNN
F 2 "TestPoint:TestPoint_THTPad_D1.5mm_Drill0.7mm" H 9600 5275 50  0001 C CNN
F 3 "~" H 9600 5275 50  0001 C CNN
	1    9400 5275
	1    0    0    -1  
$EndComp
Wire Wire Line
	5100 4200 5300 4200
Wire Wire Line
	5300 4200 5300 3800
Wire Wire Line
	5300 3800 5150 3800
Connection ~ 5300 4200
Text GLabel 5150 3800 0    50   Input ~ 0
USB_VCC
Text HLabel 9450 1450 2    50   Input ~ 0
5V_MON
Text HLabel 9450 1750 2    50   Input ~ 0
PWR_MON
Wire Wire Line
	6550 1375 6900 1375
Wire Wire Line
	5300 4200 5550 4200
Text Label 5550 4200 2    50   ~ 0
6V
Wire Wire Line
	8050 5350 8300 5350
Text Label 8050 5350 0    50   ~ 0
6V
Text Label 8050 3500 0    50   ~ 0
6V
Wire Wire Line
	8050 3500 8300 3500
Connection ~ 8300 3500
Text Notes 1700 850  0    118  ~ 0
Power input
Text Notes 2550 3750 0    118  ~ 0
6V step-down
Text Notes 8600 2950 0    118  ~ 0
5V regulator
Text Notes 8600 4800 0    118  ~ 0
3V3 regulator
$Comp
L Device:C C?
U 1 1 5CFDE4F9
P 6950 1975
F 0 "C?" H 7065 2021 50  0000 L CNN
F 1 "100nF" H 7065 1930 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 6988 1825 50  0001 C CNN
F 3 "~" H 6950 1975 50  0001 C CNN
	1    6950 1975
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5D43EC4F
P 6950 2175
F 0 "#PWR?" H 6950 1925 50  0001 C CNN
F 1 "GND" H 6950 2025 50  0000 C CNN
F 2 "" H 6950 2175 50  0001 C CNN
F 3 "" H 6950 2175 50  0001 C CNN
	1    6950 2175
	1    0    0    -1  
$EndComp
Wire Wire Line
	6550 1775 6950 1775
Wire Wire Line
	6950 1825 6950 1775
Connection ~ 6950 1775
Wire Wire Line
	6950 2125 6950 2175
Text Notes 8550 950  0    118  ~ 0
ESD protection
$Comp
L power:GND #PWR?
U 1 1 5D43EC50
P 9100 2300
F 0 "#PWR?" H 9100 2050 50  0001 C CNN
F 1 "GND" H 9100 2150 50  0000 C CNN
F 2 "" H 9100 2300 50  0001 C CNN
F 3 "" H 9100 2300 50  0001 C CNN
	1    9100 2300
	1    0    0    -1  
$EndComp
Wire Wire Line
	9450 1750 8900 1750
Wire Wire Line
	8900 1750 8900 1900
Wire Wire Line
	8900 1750 8450 1750
Connection ~ 8900 1750
Text Label 7450 1775 2    50   ~ 0
MAIN_V_MON
Text Label 8450 1750 0    50   ~ 0
PWR_MON
$Comp
L Device:C C?
U 1 1 5D0D94FC
P 10050 4150
F 0 "C?" H 10165 4196 50  0000 L CNN
F 1 "100nF" H 10165 4105 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 10088 4000 50  0001 C CNN
F 3 "~" H 10050 4150 50  0001 C CNN
	1    10050 4150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5D0D9506
P 10050 4300
F 0 "#PWR?" H 10050 4050 50  0001 C CNN
F 1 "GND" H 10050 4150 50  0000 C CNN
F 2 "" H 10050 4300 50  0001 C CNN
F 3 "" H 10050 4300 50  0001 C CNN
	1    10050 4300
	1    0    0    -1  
$EndComp
Wire Wire Line
	9650 3900 10050 3900
Connection ~ 10050 3900
Text Label 10450 3900 2    50   ~ 0
5V_MON
Wire Wire Line
	10050 3900 10450 3900
Wire Wire Line
	10050 3900 10050 4000
Wire Wire Line
	9650 3800 9650 3900
Connection ~ 9650 3900
Wire Wire Line
	9650 3900 9650 4000
Wire Wire Line
	9100 1900 9100 1450
Wire Wire Line
	9100 1450 9450 1450
Wire Wire Line
	8450 1450 9100 1450
Connection ~ 9100 1450
Text Label 8450 1450 0    50   ~ 0
5V_MON
$Comp
L Sensor_Current:ACS712xLCTR-20A U?
U 1 1 5D030689
P 3300 1500
AR Path="/5C6B1C46/5D030689" Ref="U?"  Part="1" 
AR Path="/5CDAD7AE/5D030689" Ref="U?"  Part="1" 
AR Path="/5D3A517C/5D030689" Ref="U?"  Part="1" 
AR Path="/5D436602/5D030689" Ref="U?"  Part="1" 
F 0 "U?" H 2900 1200 50  0000 C CNN
F 1 "ACS712xLCTR-20A" H 2900 1100 50  0000 C CNN
F 2 "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm" H 3400 1150 50  0001 L CIN
F 3 "http://www.allegromicro.com/~/media/Files/Datasheets/ACS712-Datasheet.ashx?la=en" H 3300 1500 50  0001 C CNN
	1    3300 1500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5D03068F
P 3300 2250
AR Path="/5C6B1C46/5D03068F" Ref="#PWR?"  Part="1" 
AR Path="/5CDAD7AE/5D03068F" Ref="#PWR?"  Part="1" 
AR Path="/5D3A517C/5D03068F" Ref="#PWR?"  Part="1" 
AR Path="/5D436602/5D03068F" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 3300 2000 50  0001 C CNN
F 1 "GND" H 3450 2175 50  0000 C CNN
F 2 "" H 3300 2250 50  0001 C CNN
F 3 "" H 3300 2250 50  0001 C CNN
	1    3300 2250
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 5D030695
P 3300 850
AR Path="/5C6B1C46/5D030695" Ref="#PWR?"  Part="1" 
AR Path="/5CDAD7AE/5D030695" Ref="#PWR?"  Part="1" 
AR Path="/5D3A517C/5D030695" Ref="#PWR?"  Part="1" 
AR Path="/5D436602/5D030695" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 3300 700 50  0001 C CNN
F 1 "+5V" H 3350 1050 50  0000 C CNN
F 2 "" H 3300 850 50  0001 C CNN
F 3 "" H 3300 850 50  0001 C CNN
	1    3300 850 
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 5D03069B
P 3550 950
AR Path="/5C6B1C46/5D03069B" Ref="C?"  Part="1" 
AR Path="/5CDAD7AE/5D03069B" Ref="C?"  Part="1" 
AR Path="/5D3A517C/5D03069B" Ref="C?"  Part="1" 
AR Path="/5D436602/5D03069B" Ref="C?"  Part="1" 
F 0 "C?" V 3250 950 50  0000 C CNN
F 1 "1uF (X7R)" V 3350 950 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 3588 800 50  0001 C CNN
F 3 "~" H 3550 950 50  0001 C CNN
	1    3550 950 
	0    1    1    0   
$EndComp
Wire Wire Line
	3300 850  3300 950 
Wire Wire Line
	3300 950  3400 950 
Wire Wire Line
	3300 950  3300 1100
Connection ~ 3300 950 
$Comp
L power:GND #PWR?
U 1 1 5D0306A5
P 3800 1050
AR Path="/5C6B1C46/5D0306A5" Ref="#PWR?"  Part="1" 
AR Path="/5CDAD7AE/5D0306A5" Ref="#PWR?"  Part="1" 
AR Path="/5D3A517C/5D0306A5" Ref="#PWR?"  Part="1" 
AR Path="/5D436602/5D0306A5" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 3800 800 50  0001 C CNN
F 1 "GND" H 3800 900 50  0000 C CNN
F 2 "" H 3800 1050 50  0001 C CNN
F 3 "" H 3800 1050 50  0001 C CNN
	1    3800 1050
	1    0    0    -1  
$EndComp
Wire Wire Line
	3700 950  3800 950 
Wire Wire Line
	3800 950  3800 1050
$Comp
L Device:C C?
U 1 1 5D0306AD
P 3800 1950
AR Path="/5C6B1C46/5D0306AD" Ref="C?"  Part="1" 
AR Path="/5CDAD7AE/5D0306AD" Ref="C?"  Part="1" 
AR Path="/5D3A517C/5D0306AD" Ref="C?"  Part="1" 
AR Path="/5D436602/5D0306AD" Ref="C?"  Part="1" 
F 0 "C?" H 3575 1950 50  0000 L CNN
F 1 "1uF (X7R)" H 3400 1850 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 3838 1800 50  0001 C CNN
F 3 "~" H 3800 1950 50  0001 C CNN
	1    3800 1950
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5D0306B3
P 3800 2250
AR Path="/5C6B1C46/5D0306B3" Ref="#PWR?"  Part="1" 
AR Path="/5CDAD7AE/5D0306B3" Ref="#PWR?"  Part="1" 
AR Path="/5D3A517C/5D0306B3" Ref="#PWR?"  Part="1" 
AR Path="/5D436602/5D0306B3" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 3800 2000 50  0001 C CNN
F 1 "GND" H 3950 2175 50  0000 C CNN
F 2 "" H 3800 2250 50  0001 C CNN
F 3 "" H 3800 2250 50  0001 C CNN
	1    3800 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	3700 1600 3800 1600
Wire Wire Line
	2900 1700 2700 1700
Connection ~ 2700 1700
Text HLabel 9450 1150 2    50   Input ~ 0
I_MON
Connection ~ 2250 1300
$Comp
L Connector:TestPoint TP?
U 1 1 5D0DE3DB
P 4000 1500
F 0 "TP?" H 4050 1750 50  0000 L CNN
F 1 "I_MAIN" H 4050 1650 50  0000 L CNN
F 2 "TestPoint:TestPoint_THTPad_D1.5mm_Drill0.7mm" H 4200 1500 50  0001 C CNN
F 3 "~" H 4200 1500 50  0001 C CNN
	1    4000 1500
	1    0    0    -1  
$EndComp
Wire Wire Line
	3700 1500 4000 1500
Wire Wire Line
	2750 3950 2750 4000
Wire Wire Line
	8450 1150 9300 1150
Wire Wire Line
	9300 1900 9300 1150
Connection ~ 9300 1150
Wire Wire Line
	9300 1150 9450 1150
Text Label 8450 1150 0    50   ~ 0
I_MON
Text Label 5000 1850 2    50   ~ 0
I_MON
Wire Wire Line
	1300 1500 1450 1500
Wire Wire Line
	1300 1200 1450 1200
$Comp
L Connector_Generic:Conn_01x04 J?
U 1 1 5D40B943
P 1100 1400
F 0 "J?" H 1000 950 50  0000 C CNN
F 1 "V_IN" H 1000 1050 50  0000 C CNN
F 2 "Connector_Phoenix_MC:PhoenixContact_MC_1,5_4-G-3.5_1x04_P3.50mm_Horizontal" H 1100 1400 50  0001 C CNN
F 3 "~" H 1100 1400 50  0001 C CNN
	1    1100 1400
	-1   0    0    1   
$EndComp
Wire Wire Line
	1300 1300 1450 1300
Wire Wire Line
	1450 1300 1450 1200
Wire Wire Line
	1300 1400 1450 1400
Wire Wire Line
	1450 1400 1450 1500
Connection ~ 1450 1300
Connection ~ 1450 1400
$Comp
L power:+24V #PWR?
U 1 1 5D42D8FA
P 6900 1375
F 0 "#PWR?" H 6900 1225 50  0001 C CNN
F 1 "+24V" V 6800 1325 50  0000 L CNN
F 2 "" H 6900 1375 50  0001 C CNN
F 3 "" H 6900 1375 50  0001 C CNN
	1    6900 1375
	0    1    1    0   
$EndComp
Wire Wire Line
	2250 1300 2900 1300
Connection ~ 6250 1375
$Comp
L Device:Ferrite_Bead FB?
U 1 1 5D478B04
P 6575 975
F 0 "FB?" V 6301 975 50  0000 C CNN
F 1 "22R@100MHz" V 6392 975 50  0000 C CNN
F 2 "" V 6505 975 50  0001 C CNN
F 3 "~" H 6575 975 50  0001 C CNN
	1    6575 975 
	0    1    1    0   
$EndComp
Text GLabel 7025 975  2    50   Input ~ 0
VMOT
Wire Wire Line
	6725 975  7025 975 
Wire Wire Line
	6950 1775 7450 1775
$Comp
L power:+24V #PWR?
U 1 1 5D4903EF
P 2750 3950
F 0 "#PWR?" H 2750 3800 50  0001 C CNN
F 1 "+24V" H 2765 4123 50  0000 C CNN
F 2 "" H 2750 3950 50  0001 C CNN
F 3 "" H 2750 3950 50  0001 C CNN
	1    2750 3950
	1    0    0    -1  
$EndComp
Text Label 5800 1375 0    50   ~ 0
MAIN_V
Text Label 2350 1700 0    50   ~ 0
MAIN_V
Wire Wire Line
	2350 1700 2700 1700
$Comp
L Power_Protection:ESDA6V1-5SC6 D?
U 1 1 5E3A73A9
P 9100 2100
F 0 "D?" H 9430 2146 50  0000 L CNN
F 1 "ESDA6V1-5SC6" H 9430 2055 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23-6" H 9800 1850 50  0001 C CNN
F 3 "www.st.com/resource/en/datasheet/esda6v1-5sc6.pdf" V 9100 2100 50  0001 C CNN
	1    9100 2100
	1    0    0    -1  
$EndComp
Wire Wire Line
	6250 1375 6250 975 
Wire Wire Line
	6250 975  6425 975 
Text Notes 6150 2525 0    50   ~ 0
Main voltage monitoring
Text Notes 6900 850  0    50   ~ 0
Motor power supply
Text Notes 3700 2500 0    50   ~ 0
Current measuring
Text Notes 1450 2500 0    50   ~ 0
Reverse polarity protection
$Comp
L Device:R R?
U 1 1 5E3D22F6
P 4325 1650
F 0 "R?" H 4425 1700 50  0000 L CNN
F 1 "10k" H 4425 1600 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 4255 1650 50  0001 C CNN
F 3 "~" H 4325 1650 50  0001 C CNN
	1    4325 1650
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 5E3D2300
P 4325 2050
F 0 "R?" H 4425 2100 50  0000 L CNN
F 1 "10k" H 4425 2000 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 4255 2050 50  0001 C CNN
F 3 "~" H 4325 2050 50  0001 C CNN
	1    4325 2050
	1    0    0    -1  
$EndComp
Wire Wire Line
	4325 1800 4325 1850
$Comp
L power:GND #PWR?
U 1 1 5E3D230B
P 4325 2250
F 0 "#PWR?" H 4325 2000 50  0001 C CNN
F 1 "GND" H 4475 2175 50  0000 C CNN
F 2 "" H 4325 2250 50  0001 C CNN
F 3 "" H 4325 2250 50  0001 C CNN
	1    4325 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	4325 2200 4325 2250
Connection ~ 4325 1850
Wire Wire Line
	4325 1850 4325 1900
$Comp
L Device:C C?
U 1 1 5E3D2318
P 4725 2050
F 0 "C?" H 4840 2096 50  0000 L CNN
F 1 "100nF" H 4840 2005 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 4763 1900 50  0001 C CNN
F 3 "~" H 4725 2050 50  0001 C CNN
	1    4725 2050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5E3D2322
P 4725 2250
F 0 "#PWR?" H 4725 2000 50  0001 C CNN
F 1 "GND" H 4875 2175 50  0000 C CNN
F 2 "" H 4725 2250 50  0001 C CNN
F 3 "" H 4725 2250 50  0001 C CNN
	1    4725 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	4725 2200 4725 2250
Wire Wire Line
	4725 1850 5000 1850
Connection ~ 4725 1850
Wire Wire Line
	3300 1900 3300 2250
Wire Wire Line
	3800 2100 3800 2250
Wire Wire Line
	3800 1600 3800 1800
Wire Wire Line
	4000 1500 4325 1500
Connection ~ 4000 1500
Wire Wire Line
	2000 2000 2000 2250
$Comp
L Connector:TestPoint TP?
U 1 1 5E419C38
P 4725 1850
F 0 "TP?" H 4775 2050 50  0000 L CNN
F 1 "I_DIV" H 4775 1975 50  0000 L CNN
F 2 "" H 4925 1850 50  0001 C CNN
F 3 "~" H 4925 1850 50  0001 C CNN
	1    4725 1850
	1    0    0    -1  
$EndComp
Wire Wire Line
	5800 1375 6250 1375
Wire Wire Line
	4725 1850 4725 1900
Wire Wire Line
	4325 1850 4725 1850
Wire Wire Line
	9400 5275 9400 5350
Wire Wire Line
	9400 5350 9600 5350
Text Notes 9725 5775 0    50   ~ 0
3.3V rail is monitored\n via internal voltage\n reference in MCU
$EndSCHEMATC
