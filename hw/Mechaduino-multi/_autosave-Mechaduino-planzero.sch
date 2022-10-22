EESchema Schematic File Version 5
EELAYER 36 0
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
Comment5 ""
Comment6 ""
Comment7 ""
Comment8 ""
Comment9 ""
$EndDescr
Connection ~ 2600 1190
Connection ~ 2790 1580
Connection ~ 2950 1190
Connection ~ 5390 1340
Connection ~ 6350 770 
Connection ~ 7000 2330
Wire Wire Line
	2600 1150 2600 1190
Wire Wire Line
	2600 1190 2600 1250
Wire Wire Line
	2600 1550 2600 1580
Wire Wire Line
	2600 1580 2790 1580
Wire Wire Line
	2790 1580 2790 1600
Wire Wire Line
	2790 1580 2950 1580
Wire Wire Line
	2950 1190 2600 1190
Wire Wire Line
	2950 1250 2950 1190
Wire Wire Line
	2950 1580 2950 1550
Wire Wire Line
	4450 1190 2950 1190
Wire Wire Line
	4450 1800 4450 1190
Wire Wire Line
	4450 3600 4450 3660
Wire Wire Line
	5050 2100 5140 2100
Wire Wire Line
	5050 2200 5140 2200
Wire Wire Line
	5050 2300 5140 2300
Wire Wire Line
	5050 2400 5140 2400
Wire Wire Line
	5050 2500 5140 2500
Wire Wire Line
	5050 2600 5140 2600
Wire Wire Line
	5050 2700 5140 2700
Wire Wire Line
	5050 2800 5140 2800
Wire Wire Line
	5050 3000 5140 3000
Wire Wire Line
	5050 3100 5140 3100
Wire Wire Line
	5050 3200 5140 3200
Wire Wire Line
	5050 3300 5140 3300
Wire Wire Line
	5060 1280 5060 1340
Wire Wire Line
	5360 1340 5390 1340
Wire Wire Line
	5390 1340 5420 1340
Wire Wire Line
	5390 1800 5390 1340
Wire Wire Line
	5720 1230 5720 1340
Wire Wire Line
	6170 770  6350 770 
Wire Wire Line
	6170 2000 6170 770 
Wire Wire Line
	6350 690  6350 770 
Wire Wire Line
	6350 770  6350 820 
Wire Wire Line
	6350 1120 6350 1200
Wire Wire Line
	6410 2100 6410 2330
Wire Wire Line
	6410 2330 7000 2330
Wire Wire Line
	6450 1500 6500 1500
Wire Wire Line
	6450 1600 6500 1600
Wire Wire Line
	6450 1700 6500 1700
Wire Wire Line
	6500 1800 5390 1800
Wire Wire Line
	6500 2000 6170 2000
Wire Wire Line
	6500 2100 6410 2100
Wire Wire Line
	6800 770  6350 770 
Wire Wire Line
	6800 1300 6800 770 
Wire Wire Line
	7000 1300 6800 1300
Wire Wire Line
	7000 2300 7000 2330
Wire Wire Line
	7000 2330 7000 2350
Text GLabel 5060 1280 1    50   Input ~ 0
at24CSn
Text GLabel 5140 2100 2    50   Input ~ 0
IN4
Text GLabel 5140 2200 2    50   Input ~ 0
IN3
Text GLabel 5140 2300 2    50   Input ~ 0
IN2
Text GLabel 5140 2400 2    50   Input ~ 0
IN1
Text GLabel 5140 2500 2    50   Input ~ 0
SCK
Text GLabel 5140 2600 2    50   Input ~ 0
MISO
Text GLabel 5140 2700 2    50   Input ~ 0
MOSI
Text GLabel 5140 2800 2    50   Input ~ 0
VREF1
Text GLabel 5140 3000 2    50   Input ~ 0
at24CSn
Text GLabel 5140 3100 2    50   Input ~ 0
at24CSn2
Text GLabel 5140 3200 2    50   Input ~ 0
VREF2
Text GLabel 5140 3300 2    50   Input ~ 0
Reset
Text GLabel 6450 1500 0    50   Input ~ 0
MOSI
Text GLabel 6450 1600 0    50   Input ~ 0
MISO
Text GLabel 6450 1700 0    50   Input ~ 0
SCK
$Comp
L power:+3.3V #PWR?
U 1 1 00000000
P 2600 1150
F 0 "#PWR?" H 2600 1000 50  0001 C CNN
F 1 "+3.3V" H 2600 1291 50  0000 C CNN
F 2 "" H 2600 1150 50  0001 C CNN
F 3 "" H 2600 1150 50  0001 C CNN
	1    2600 1150
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR?
U 1 1 00000000
P 6350 690
F 0 "#PWR?" H 6350 540 50  0001 C CNN
F 1 "+3.3V" H 6350 831 50  0000 C CNN
F 2 "" H 6350 690 50  0001 C CNN
F 3 "" H 6350 690 50  0001 C CNN
	1    6350 690 
	1    0    0    -1  
$EndComp
$Comp
L power:GNDREF #PWR?
U 1 1 00000000
P 2790 1600
F 0 "#PWR?" H 2790 1350 50  0001 C CNN
F 1 "GNDREF" H 2790 1425 50  0000 C CNN
F 2 "" H 2790 1600 50  0001 C CNN
F 3 "" H 2790 1600 50  0001 C CNN
	1    2790 1600
	1    0    0    -1  
$EndComp
$Comp
L power:GNDREF #PWR?
U 1 1 00000000
P 4450 3660
F 0 "#PWR?" H 4450 3410 50  0001 C CNN
F 1 "GNDREF" H 4450 3485 50  0000 C CNN
F 2 "" H 4450 3660 50  0001 C CNN
F 3 "" H 4450 3660 50  0001 C CNN
	1    4450 3660
	1    0    0    -1  
$EndComp
$Comp
L power:GNDREF #PWR?
U 1 1 00000000
P 6350 1200
F 0 "#PWR?" H 6350 950 50  0001 C CNN
F 1 "GNDREF" H 6350 1025 50  0000 C CNN
F 2 "" H 6350 1200 50  0001 C CNN
F 3 "" H 6350 1200 50  0001 C CNN
	1    6350 1200
	1    0    0    -1  
$EndComp
$Comp
L power:GNDREF #PWR?
U 1 1 00000000
P 7000 2350
F 0 "#PWR?" H 7000 2100 50  0001 C CNN
F 1 "GNDREF" H 7000 2175 50  0000 C CNN
F 2 "" H 7000 2350 50  0001 C CNN
F 3 "" H 7000 2350 50  0001 C CNN
	1    7000 2350
	1    0    0    -1  
$EndComp
$Comp
L Jumper:SolderJumper_2_Open JP?
U 1 1 00000000
P 5210 1340
F 0 "JP?" H 5210 1521 50  0000 C CNN
F 1 " " H 5210 1421 50  0000 C CNN
F 2 "" H 5210 1340 50  0001 C CNN
F 3 "~" H 5210 1340 50  0001 C CNN
	1    5210 1340
	1    0    0    -1  
$EndComp
$Comp
L Jumper:SolderJumper_2_Open JP?
U 1 1 00000000
P 5570 1340
F 0 "JP?" H 5570 1521 50  0000 C CNN
F 1 " " H 5570 1421 50  0000 C CNN
F 2 "" H 5570 1340 50  0001 C CNN
F 3 "~" H 5570 1340 50  0001 C CNN
	1    5570 1340
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J?
U 1 1 00000000
P 5720 1030
F 0 "J?" V 5753 950 50  0000 R CNN
F 1 "EXT_CSn" V 5653 950 50  0000 R CNN
F 2 "" H 5720 1030 50  0001 C CNN
F 3 "~" H 5720 1030 50  0001 C CNN
	1    5720 1030
	0    -1   -1   0   
$EndComp
$Comp
L Device:C C?
U 1 1 00000000
P 2600 1400
F 0 "C?" H 2715 1433 50  0000 L CNN
F 1 "100n" H 2715 1333 50  0000 L CNN
F 2 "" H 2638 1250 50  0001 C CNN
F 3 "~" H 2600 1400 50  0001 C CNN
	1    2600 1400
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Polarized C?
U 1 1 00000000
P 2950 1400
F 0 "C?" H 3065 1468 50  0000 L CNN
F 1 "4.7u" H 3065 1368 50  0000 L CNN
F 2 "" H 2988 1250 50  0001 C CNN
F 3 "~" H 2950 1400 50  0001 C CNN
	1    2950 1400
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 00000000
P 6350 970
F 0 "C?" H 6465 1003 50  0000 L CNN
F 1 "100n" H 6465 903 50  0000 L CNN
F 2 "" H 6388 820 50  0001 C CNN
F 3 "~" H 6350 970 50  0001 C CNN
	1    6350 970 
	1    0    0    -1  
$EndComp
$Comp
L Sensor_Magnetic:AS5047D U?
U 1 1 634C169E
P 7000 1800
F 0 "U?" H 7000 2481 50  0000 C CNN
F 1 "AS5047D" H 7000 2390 50  0000 C CNN
F 2 "Package_SO:TSSOP-14_4.4x5mm_P0.65mm" H 7000 1200 50  0001 C CNN
F 3 "https://ams.com/documents/20143/36005/AS5047D_DS000394_2-00.pdf" H 6250 1250 50  0001 C CNN
	1    7000 1800
	1    0    0    -1  
$EndComp
$Comp
L Mechaduino-planzero-rescue:A4954ELPTR-T-a4954 IC?
U 1 1 634C03B4
P 6700 3250
F 0 "IC?" H 7350 3515 50  0000 C CNN
F 1 "A4954ELPTR-T" H 7350 3424 50  0000 C CNN
F 2 "A2550KLPTRT" H 7850 3350 50  0001 L CNN
F 3 "https://datasheet.datasheetarchive.com/originals/distributors/Datasheets-DGA5/485249.pdf" H 7850 3250 50  0001 L CNN
F 4 "Dual DMOS Full Bridge Motor Driver" H 7850 3150 50  0001 L CNN "Description"
F 5 "1.2" H 7850 3050 50  0001 L CNN "Height"
F 6 "Allegro Microsystems" H 7850 2950 50  0001 L CNN "Manufacturer_Name"
F 7 "A4954ELPTR-T" H 7850 2850 50  0001 L CNN "Manufacturer_Part_Number"
F 8 "" H 7850 2750 50  0001 L CNN "Mouser Part Number"
F 9 "" H 7850 2650 50  0001 L CNN "Mouser Price/Stock"
F 10 "A4954ELPTR-T" H 7850 2550 50  0001 L CNN "Arrow Part Number"
F 11 "https://www.arrow.com/en/products/a4954elptr-t/allegro-microsystems" H 7850 2450 50  0001 L CNN "Arrow Price/Stock"
F 12 "" H 7850 2350 50  0001 L CNN "Mouser Testing Part Number"
F 13 "" H 7850 2250 50  0001 L CNN "Mouser Testing Price/Stock"
	1    6700 3250
	1    0    0    -1  
$EndComp
$Comp
L Mechaduino-planzero-rescue:ATtiny24-20MU-MCU_Microchip_ATtiny U?
U 1 1 634C0410
P 4450 2700
F 0 "U?" H 3907 2746 50  0000 R CNN
F 1 "ATtiny24-20MU" H 3907 2655 50  0000 R CNN
F 2 "Package_DFN_QFN:QFN-20-1EP_4x4mm_P0.5mm_EP2.6x2.6mm" H 4450 2700 50  0001 C CIN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/doc8006.pdf" H 4450 2700 50  0001 C CNN
	1    4450 2700
	1    0    0    -1  
$EndComp
$EndSCHEMATC
