EESchema Schematic File Version 4
LIBS:emitter-cache
EELAYER 29 0
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
L Connector:Conn_01x05_Male J2
U 1 1 5D4F8E78
P 3500 3200
F 0 "J2" H 3600 2900 50  0000 C CNN
F 1 "Conn_01x05_Male" H 3608 3490 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x05_P2.54mm_Vertical" H 3500 3200 50  0001 C CNN
F 3 "~" H 3500 3200 50  0001 C CNN
	1    3500 3200
	1    0    0    -1  
$EndComp
$Comp
L Device:LED_ALT D2
U 1 1 5D4FA4C4
P 1650 3200
F 0 "D2" H 1650 3000 50  0000 C CNN
F 1 "LED_ALT" H 1650 2900 50  0000 C CNN
F 2 "LED_THT:LED_D5.0mm" H 1650 3200 50  0001 C CNN
F 3 "~" H 1650 3200 50  0001 C CNN
	1    1650 3200
	1    0    0    -1  
$EndComp
$Comp
L Device:LED_ALT D1
U 1 1 5D4FB16E
P 1250 3200
F 0 "D1" H 1250 3000 50  0000 C CNN
F 1 "LED_ALT" H 1250 2900 50  0000 C CNN
F 2 "LED_THT:LED_D5.0mm" H 1250 3200 50  0001 C CNN
F 3 "~" H 1250 3200 50  0001 C CNN
	1    1250 3200
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R1
U 1 1 5D4FB79D
P 2050 3200
F 0 "R1" V 2250 3200 50  0000 C CNN
F 1 "R_Small" V 2350 3200 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" H 2050 3200 50  0001 C CNN
F 3 "~" H 2050 3200 50  0001 C CNN
	1    2050 3200
	0    1    1    0   
$EndComp
$Comp
L Connector:Conn_01x05_Male J1
U 1 1 5D4F534A
P 2500 3200
F 0 "J1" H 2650 2900 50  0000 R CNN
F 1 "Conn_01x05_Male" H 2472 3133 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x05_P2.54mm_Vertical" H 2500 3200 50  0001 C CNN
F 3 "~" H 2500 3200 50  0001 C CNN
	1    2500 3200
	-1   0    0    -1  
$EndComp
Wire Wire Line
	2300 3100 1000 3100
Wire Wire Line
	1000 3100 1000 3200
Wire Wire Line
	2150 3200 2300 3200
Wire Wire Line
	1800 2850 2300 2850
Wire Wire Line
	2300 2850 2300 3000
Wire Wire Line
	2300 3300 2200 3300
Wire Wire Line
	2200 3300 2200 2950
Wire Wire Line
	2200 2950 1400 2950
Wire Wire Line
	1400 2950 1400 2850
Wire Wire Line
	1400 2850 1500 2850
$Comp
L Switch:SW_MEC_5G SW1
U 1 1 5D5009AF
P 1600 2850
F 0 "SW1" H 1600 3135 50  0000 C CNN
F 1 "SW_MEC_5G" H 1600 3044 50  0000 C CNN
F 2 "Button_Switch_THT:SW_TH_Tactile_Omron_B3F-10xx" H 1600 3050 50  0001 C CNN
F 3 "http://www.apem.com/int/index.php?controller=attachment&id_attachment=488" H 1600 3050 50  0001 C CNN
	1    1600 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	1400 3200 1500 3200
Wire Wire Line
	1000 3200 1100 3200
Wire Wire Line
	1800 3200 1950 3200
$EndSCHEMATC
