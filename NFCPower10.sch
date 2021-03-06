EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A 11000 8500
encoding utf-8
Sheet 1 12
Title "NFC Power 10 Transmitters - YSO"
Date "2021-01-27"
Rev "2"
Comp "NthDegree"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Sheet
S 7850 4700 750  700 
U 5FBD86FB
F0 "TX1" 50
F1 "transmitter.sch" 50
F2 "VS" U L 7850 4800 50 
F3 "GND" U L 7850 5300 50 
F4 "SCL" U L 7850 4900 50 
F5 "SDA" U L 7850 5000 50 
F6 "IRQ" U L 7850 5100 50 
F7 "CLK" I L 7850 5200 50 
F8 "RX" I R 8600 4900 50 
F9 "TX" O R 8600 4800 50 
F10 "A0" U R 8600 5000 50 
F11 "A1" U R 8600 5100 50 
F12 "A2" U R 8600 5200 50 
F13 "A3" U R 8600 5300 50 
$EndSheet
Wire Wire Line
	7550 1800 7850 1800
Wire Wire Line
	7550 2300 7850 2300
$Comp
L power:GND #PWR048
U 1 1 5FF408B8
P 7550 2300
F 0 "#PWR048" H 7550 2050 50  0001 C CNN
F 1 "GND" V 7555 2172 50  0000 R CNN
F 2 "" H 7550 2300 50  0001 C CNN
F 3 "" H 7550 2300 50  0001 C CNN
	1    7550 2300
	0    1    1    0   
$EndComp
Wire Wire Line
	7850 1900 7550 1900
Wire Wire Line
	7850 2000 7550 2000
Wire Wire Line
	7850 2100 7550 2100
Wire Wire Line
	7850 2200 7550 2200
Text Label 7750 1900 2    50   ~ 0
SCLP
Text Label 7750 2000 2    50   ~ 0
SDAP
Text Label 7750 2100 2    50   ~ 0
IRQP
Wire Wire Line
	7550 800  7850 800 
Wire Wire Line
	7550 1300 7850 1300
$Comp
L power:GND #PWR046
U 1 1 5FF41E3A
P 7550 1300
F 0 "#PWR046" H 7550 1050 50  0001 C CNN
F 1 "GND" V 7555 1172 50  0000 R CNN
F 2 "" H 7550 1300 50  0001 C CNN
F 3 "" H 7550 1300 50  0001 C CNN
	1    7550 1300
	0    1    1    0   
$EndComp
Wire Wire Line
	7850 900  7550 900 
Wire Wire Line
	7850 1000 7550 1000
Wire Wire Line
	7850 1100 7550 1100
Wire Wire Line
	7850 1200 7550 1200
Text Label 7750 900  2    50   ~ 0
SCLP
Text Label 7750 1000 2    50   ~ 0
SDAP
Text Label 7750 1100 2    50   ~ 0
IRQP
Wire Wire Line
	7550 2800 7850 2800
Wire Wire Line
	7550 3300 7850 3300
$Comp
L power:GND #PWR050
U 1 1 5FF44216
P 7550 3300
F 0 "#PWR050" H 7550 3050 50  0001 C CNN
F 1 "GND" V 7555 3172 50  0000 R CNN
F 2 "" H 7550 3300 50  0001 C CNN
F 3 "" H 7550 3300 50  0001 C CNN
	1    7550 3300
	0    1    1    0   
$EndComp
Wire Wire Line
	7850 2900 7550 2900
Wire Wire Line
	7850 3000 7550 3000
Wire Wire Line
	7850 3100 7550 3100
Wire Wire Line
	7850 3200 7550 3200
Text Label 7750 2900 2    50   ~ 0
SCLP
Text Label 7750 3000 2    50   ~ 0
SDAP
Text Label 7750 3100 2    50   ~ 0
IRQP
Wire Wire Line
	7550 3800 7850 3800
Wire Wire Line
	7550 4300 7850 4300
$Comp
L power:GND #PWR052
U 1 1 5FF453AD
P 7550 4300
F 0 "#PWR052" H 7550 4050 50  0001 C CNN
F 1 "GND" V 7555 4172 50  0000 R CNN
F 2 "" H 7550 4300 50  0001 C CNN
F 3 "" H 7550 4300 50  0001 C CNN
	1    7550 4300
	0    1    1    0   
$EndComp
Wire Wire Line
	7850 3900 7550 3900
Wire Wire Line
	7850 4000 7550 4000
Wire Wire Line
	7850 4100 7550 4100
Wire Wire Line
	7850 4200 7550 4200
Text Label 7750 3900 2    50   ~ 0
SCLP
Text Label 7750 4000 2    50   ~ 0
SDAP
Text Label 7750 4100 2    50   ~ 0
IRQP
Wire Wire Line
	7550 4800 7850 4800
Wire Wire Line
	7550 5300 7850 5300
$Comp
L power:GND #PWR054
U 1 1 5FF466A8
P 7550 5300
F 0 "#PWR054" H 7550 5050 50  0001 C CNN
F 1 "GND" V 7555 5172 50  0000 R CNN
F 2 "" H 7550 5300 50  0001 C CNN
F 3 "" H 7550 5300 50  0001 C CNN
	1    7550 5300
	0    1    1    0   
$EndComp
Wire Wire Line
	7850 4900 7550 4900
Wire Wire Line
	7850 5000 7550 5000
Wire Wire Line
	7850 5100 7550 5100
Wire Wire Line
	7850 5200 7550 5200
Text Label 7750 4900 2    50   ~ 0
SCLP
Text Label 7750 5000 2    50   ~ 0
SDAP
Text Label 7750 5100 2    50   ~ 0
IRQP
Text Label 7750 5200 2    50   ~ 0
CLK1
Text Label 7750 4200 2    50   ~ 0
CLK2
Text Label 7750 3200 2    50   ~ 0
CLK3
Text Label 7750 2200 2    50   ~ 0
CLK4
Text Label 7750 1200 2    50   ~ 0
CLK5
Wire Wire Line
	9150 1800 9450 1800
Wire Wire Line
	9150 2300 9450 2300
$Comp
L power:GND #PWR060
U 1 1 5FF4BA53
P 9150 2300
F 0 "#PWR060" H 9150 2050 50  0001 C CNN
F 1 "GND" V 9155 2172 50  0000 R CNN
F 2 "" H 9150 2300 50  0001 C CNN
F 3 "" H 9150 2300 50  0001 C CNN
	1    9150 2300
	0    1    1    0   
$EndComp
Wire Wire Line
	9450 1900 9150 1900
Wire Wire Line
	9450 2000 9150 2000
Wire Wire Line
	9450 2100 9150 2100
Wire Wire Line
	9450 2200 9150 2200
Text Label 9350 1900 2    50   ~ 0
SCLP
Text Label 9350 2000 2    50   ~ 0
SDAP
Text Label 9350 2100 2    50   ~ 0
IRQP
Wire Wire Line
	9150 800  9450 800 
Wire Wire Line
	9150 1300 9450 1300
$Comp
L power:GND #PWR058
U 1 1 5FF4BA68
P 9150 1300
F 0 "#PWR058" H 9150 1050 50  0001 C CNN
F 1 "GND" V 9155 1172 50  0000 R CNN
F 2 "" H 9150 1300 50  0001 C CNN
F 3 "" H 9150 1300 50  0001 C CNN
	1    9150 1300
	0    1    1    0   
$EndComp
Wire Wire Line
	9450 900  9150 900 
Wire Wire Line
	9450 1000 9150 1000
Wire Wire Line
	9450 1100 9150 1100
Wire Wire Line
	9450 1200 9150 1200
Text Label 9350 900  2    50   ~ 0
SCLP
Text Label 9350 1000 2    50   ~ 0
SDAP
Text Label 9350 1100 2    50   ~ 0
IRQP
Wire Wire Line
	9150 2800 9450 2800
Wire Wire Line
	9150 3300 9450 3300
$Comp
L power:GND #PWR062
U 1 1 5FF4BA7D
P 9150 3300
F 0 "#PWR062" H 9150 3050 50  0001 C CNN
F 1 "GND" V 9155 3172 50  0000 R CNN
F 2 "" H 9150 3300 50  0001 C CNN
F 3 "" H 9150 3300 50  0001 C CNN
	1    9150 3300
	0    1    1    0   
$EndComp
Wire Wire Line
	9450 2900 9150 2900
Wire Wire Line
	9450 3000 9150 3000
Wire Wire Line
	9450 3100 9150 3100
Wire Wire Line
	9450 3200 9150 3200
Text Label 9350 2900 2    50   ~ 0
SCLP
Text Label 9350 3000 2    50   ~ 0
SDAP
Text Label 9350 3100 2    50   ~ 0
IRQP
Wire Wire Line
	9150 3800 9450 3800
Wire Wire Line
	9150 4300 9450 4300
$Comp
L power:GND #PWR064
U 1 1 5FF4BA92
P 9150 4300
F 0 "#PWR064" H 9150 4050 50  0001 C CNN
F 1 "GND" V 9155 4172 50  0000 R CNN
F 2 "" H 9150 4300 50  0001 C CNN
F 3 "" H 9150 4300 50  0001 C CNN
	1    9150 4300
	0    1    1    0   
$EndComp
Wire Wire Line
	9450 3900 9150 3900
Wire Wire Line
	9450 4000 9150 4000
Wire Wire Line
	9450 4100 9150 4100
Wire Wire Line
	9450 4200 9150 4200
Text Label 9350 3900 2    50   ~ 0
SCLP
Text Label 9350 4000 2    50   ~ 0
SDAP
Text Label 9350 4100 2    50   ~ 0
IRQP
Wire Wire Line
	9150 4800 9450 4800
Wire Wire Line
	9150 5300 9450 5300
$Comp
L power:GND #PWR066
U 1 1 5FF4BAA7
P 9150 5300
F 0 "#PWR066" H 9150 5050 50  0001 C CNN
F 1 "GND" V 9155 5172 50  0000 R CNN
F 2 "" H 9150 5300 50  0001 C CNN
F 3 "" H 9150 5300 50  0001 C CNN
	1    9150 5300
	0    1    1    0   
$EndComp
Wire Wire Line
	9450 4900 9150 4900
Wire Wire Line
	9450 5000 9150 5000
Wire Wire Line
	9450 5100 9150 5100
Wire Wire Line
	9450 5200 9150 5200
Text Label 9350 4900 2    50   ~ 0
SCLP
Text Label 9350 5000 2    50   ~ 0
SDAP
Text Label 9350 5100 2    50   ~ 0
IRQP
Text Label 9350 5200 2    50   ~ 0
CLK1
Text Label 9350 4200 2    50   ~ 0
CLK2
Text Label 9350 3200 2    50   ~ 0
CLK3
Text Label 9350 2200 2    50   ~ 0
CLK4
Text Label 9350 1200 2    50   ~ 0
CLK5
$Comp
L Connector_Generic:Conn_02x03_Odd_Even J?
U 1 1 6007BA26
P 1900 7700
AR Path="/5FC1E32F/6007BA26" Ref="J?"  Part="1" 
AR Path="/5FBD86FB/6007BA26" Ref="J?"  Part="1" 
AR Path="/6007BA26" Ref="J3"  Part="1" 
F 0 "J3" H 1950 8017 50  0000 C CNN
F 1 "Conn_02x03_Odd_Even" H 1950 7926 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_2x03_Pitch2.54mm" H 1900 7700 50  0001 C CNN
F 3 "~" H 1900 7700 50  0001 C CNN
	1    1900 7700
	1    0    0    -1  
$EndComp
$Comp
L project:ATMEGA328P-MU U?
U 1 1 6007BA3E
P 1150 4300
AR Path="/5FC1E32F/6007BA3E" Ref="U?"  Part="1" 
AR Path="/5FBD86FB/6007BA3E" Ref="U?"  Part="1" 
AR Path="/6007BA3E" Ref="U1"  Part="1" 
F 0 "U1" H 2150 4655 50  0000 C CNN
F 1 "ATMEGA328P-MU" H 2150 4564 50  0000 C CNN
F 2 "project:Microchip___Atmel-ATMEGA328P-MU-Level_A" H 1150 4700 50  0001 L CNN
F 3 "" H 1150 4800 50  0001 L CNN
F 4 "MO-220-VHHD-2" H 1150 4900 50  0001 L CNN "Code  JEDEC"
F 5 "Manufacturer URL" H 1150 5000 50  0001 L CNN "Component Link 1 Description"
F 6 "http://www.atmel.com/" H 1150 5100 50  0001 L CNN "Component Link 1 URL"
F 7 "ATmega48A/48PA/88A/88PA/168A/168PA/328/328P Datasheet" H 1150 5200 50  0001 L CNN "Component Link 2 Description"
F 8 "http://www.atmel.com/dyn/resources/prod_documents/8271.pdf" H 1150 5300 50  0001 L CNN "Component Link 2 URL"
F 9 "ATmega48A/48PA/88A/88PA/168A/168PA/328/328P Summary" H 1150 5400 50  0001 L CNN "Component Link 3 Description"
F 10 "http://www.atmel.com/dyn/resources/prod_documents/8271S.pdf" H 1150 5500 50  0001 L CNN "Component Link 3 URL"
F 11 "revD, May-2011" H 1150 5600 50  0001 L CNN "Datasheet Version"
F 12 "32-Pin Micro Lead Frame Package (MLF),  5 x 5 x 1.0 mm Body, Pitch 0.50 mm, 3.10mm Exposed Pad" H 1150 5700 50  0001 L CNN "Package Description"
F 13 "revE, May-2006" H 1150 5800 50  0001 L CNN "Package Version"
F 14 "IC" H 1150 5900 50  0001 L CNN "category"
F 15 "973187" H 1150 6000 50  0001 L CNN "ciiva ids"
F 16 "7d83c92a723a3371" H 1150 6100 50  0001 L CNN "library id"
F 17 "Microchip / Atmel" H 1150 6200 50  0001 L CNN "manufacturer"
F 18 "QFN-32" H 2150 4473 50  0000 C CNN "package"
F 19 "1329192673" H 1150 6400 50  0001 L CNN "release date"
F 20 "417A53D3-7F28-4C25-88A7-47DF99564268" H 1150 6500 50  0001 L CNN "vault revision"
F 21 "yes" H 1150 6600 50  0001 L CNN "imported"
	1    1150 4300
	1    0    0    -1  
$EndComp
Wire Wire Line
	1250 6600 1200 6600
Wire Wire Line
	1200 6600 1200 6700
Wire Wire Line
	1250 6700 1200 6700
Connection ~ 1200 6700
Wire Wire Line
	1200 6700 1200 6800
Wire Wire Line
	1250 6800 1200 6800
Connection ~ 1200 6800
Wire Wire Line
	1200 6800 1200 7000
Wire Wire Line
	1250 7000 1200 7000
Wire Wire Line
	1250 4600 1200 4600
Wire Wire Line
	1200 4600 1200 4400
Wire Wire Line
	1200 4300 1250 4300
Wire Wire Line
	1250 4400 1200 4400
Connection ~ 1200 4400
Wire Wire Line
	1200 4400 1200 4300
Wire Wire Line
	1250 4800 900  4800
Wire Wire Line
	900  4800 900  4900
$Comp
L project:CC0603KPX7R9BB104 C?
U 1 1 6007BA6E
P 900 4900
AR Path="/5FC1E32F/6007BA6E" Ref="C?"  Part="1" 
AR Path="/5FBD86FB/6007BA6E" Ref="C?"  Part="1" 
AR Path="/6007BA6E" Ref="C2"  Part="1" 
F 0 "C2" H 1028 4937 50  0000 L CNN
F 1 "CC0603KPX7R9BB104" H 900 5100 50  0001 L CNN
F 2 "project:YAGEO-CC0603-0-0-0" H 900 5200 50  0001 L CNN
F 3 "http://www.yageo.com/documents/recent/UPY-GPHC_X7R_6.3V-to-50V_18.pdf" H 900 5300 50  0001 L CNN
F 4 "No" H 900 5400 50  0001 L CNN "automotive"
F 5 "100 nF" H 1028 4846 50  0000 L CNN "capacitance"
F 6 "Cap" H 900 5600 50  0001 L CNN "category"
F 7 "Passive Components" H 900 5700 50  0001 L CNN "device class L1"
F 8 "Capacitors" H 900 5800 50  0001 L CNN "device class L2"
F 9 "Ceramic Capacitors" H 900 5900 50  0001 L CNN "device class L3"
F 10 "CAP CER 0.1UF 50V X7R 0603" H 900 6000 50  0001 L CNN "digikey description"
F 11 "CC0603KPX7R9BB104-ND" H 900 6100 50  0001 L CNN "digikey part number"
F 12 "http://www.yageo.com/exep/pages/download/literatures/UPY-C_GEN_24.pdf" H 900 6200 50  0001 L CNN "footprint url"
F 13 "0.9mm" H 900 6300 50  0001 L CNN "height"
F 14 "Yes" H 900 6400 50  0001 L CNN "lead free"
F 15 "7cd692b09ff39865" H 900 6500 50  0001 L CNN "library id"
F 16 "YAGEO" H 900 6600 50  0001 L CNN "manufacturer"
F 17 "Ceramic" H 900 6700 50  0001 L CNN "material"
F 18 "Multilayer Ceramic Capacitors MLCC - SMD/SMT 100nF 50V X7R 10%" H 900 6800 50  0001 L CNN "mouser description"
F 19 "603-CC603KPX7R9BB104" H 900 6900 50  0001 L CNN "mouser part number"
F 20 "0603" H 1028 4755 50  0000 L CNN "package"
F 21 "Yes" H 900 7100 50  0001 L CNN "rohs"
F 22 "X7R" H 900 7200 50  0001 L CNN "temperature characteristic"
F 23 "15%" H 900 7300 50  0001 L CNN "temperature coefficient"
F 24 "+125°C" H 900 7400 50  0001 L CNN "temperature range high"
F 25 "-55°C" H 900 7500 50  0001 L CNN "temperature range low"
F 26 "0.1" H 900 7600 50  0001 L CNN "tolerance"
F 27 "50 V" H 1028 4664 50  0000 L CNN "voltage"
F 28 "50 V" H 900 7800 50  0001 L CNN "voltage rating"
	1    900  4900
	1    0    0    -1  
$EndComp
Wire Wire Line
	900  5100 900  5150
$Comp
L project:CC0603KPX7R9BB104 C?
U 1 1 6007BA8E
P 750 4400
AR Path="/5FC1E32F/6007BA8E" Ref="C?"  Part="1" 
AR Path="/5FBD86FB/6007BA8E" Ref="C?"  Part="1" 
AR Path="/6007BA8E" Ref="C1"  Part="1" 
F 0 "C1" H 878 4437 50  0000 L CNN
F 1 "CC0603KPX7R9BB104" H 750 4600 50  0001 L CNN
F 2 "project:YAGEO-CC0603-0-0-0" H 750 4700 50  0001 L CNN
F 3 "http://www.yageo.com/documents/recent/UPY-GPHC_X7R_6.3V-to-50V_18.pdf" H 750 4800 50  0001 L CNN
F 4 "No" H 750 4900 50  0001 L CNN "automotive"
F 5 "100 nF" H 878 4346 50  0000 L CNN "capacitance"
F 6 "Cap" H 750 5100 50  0001 L CNN "category"
F 7 "Passive Components" H 750 5200 50  0001 L CNN "device class L1"
F 8 "Capacitors" H 750 5300 50  0001 L CNN "device class L2"
F 9 "Ceramic Capacitors" H 750 5400 50  0001 L CNN "device class L3"
F 10 "CAP CER 0.1UF 50V X7R 0603" H 750 5500 50  0001 L CNN "digikey description"
F 11 "CC0603KPX7R9BB104-ND" H 750 5600 50  0001 L CNN "digikey part number"
F 12 "http://www.yageo.com/exep/pages/download/literatures/UPY-C_GEN_24.pdf" H 750 5700 50  0001 L CNN "footprint url"
F 13 "0.9mm" H 750 5800 50  0001 L CNN "height"
F 14 "Yes" H 750 5900 50  0001 L CNN "lead free"
F 15 "7cd692b09ff39865" H 750 6000 50  0001 L CNN "library id"
F 16 "YAGEO" H 750 6100 50  0001 L CNN "manufacturer"
F 17 "Ceramic" H 750 6200 50  0001 L CNN "material"
F 18 "Multilayer Ceramic Capacitors MLCC - SMD/SMT 100nF 50V X7R 10%" H 750 6300 50  0001 L CNN "mouser description"
F 19 "603-CC603KPX7R9BB104" H 750 6400 50  0001 L CNN "mouser part number"
F 20 "0603" H 878 4255 50  0000 L CNN "package"
F 21 "Yes" H 750 6600 50  0001 L CNN "rohs"
F 22 "X7R" H 750 6700 50  0001 L CNN "temperature characteristic"
F 23 "15%" H 750 6800 50  0001 L CNN "temperature coefficient"
F 24 "+125°C" H 750 6900 50  0001 L CNN "temperature range high"
F 25 "-55°C" H 750 7000 50  0001 L CNN "temperature range low"
F 26 "0.1" H 750 7100 50  0001 L CNN "tolerance"
F 27 "50 V" H 878 4164 50  0000 L CNN "voltage"
F 28 "50 V" H 750 7300 50  0001 L CNN "voltage rating"
	1    750  4400
	1    0    0    -1  
$EndComp
Wire Wire Line
	1200 4300 1100 4300
Wire Wire Line
	750  4300 750  4400
Connection ~ 1200 4300
Wire Wire Line
	750  4600 750  5150
Wire Wire Line
	750  5150 900  5150
Text Label 3100 5000 0    50   ~ 0
SDA
Text Label 3100 5100 0    50   ~ 0
SCL
Wire Wire Line
	2200 7700 2450 7700
Wire Wire Line
	1450 7600 1700 7600
Wire Wire Line
	1450 7700 1700 7700
Text Label 1450 7600 0    50   ~ 0
MISO
Text Label 1450 7700 0    50   ~ 0
SCK
Text Label 1450 7800 0    50   ~ 0
RES
Text Label 2200 7700 0    50   ~ 0
MOSI
Text Label 3100 6600 0    50   ~ 0
MOSI
Wire Wire Line
	3050 6600 3300 6600
Text Label 3100 6700 0    50   ~ 0
MISO
Text Label 3100 6800 0    50   ~ 0
SCK
Text Label 3100 5200 0    50   ~ 0
RES
$Comp
L project:TSX-3225_16.0000MF09Z-AC3 Y?
U 1 1 6007BAC2
P 3250 7150
AR Path="/5FC1E32F/6007BAC2" Ref="Y?"  Part="1" 
AR Path="/5FBD86FB/6007BAC2" Ref="Y?"  Part="1" 
AR Path="/6007BAC2" Ref="Y1"  Part="1" 
F 0 "Y1" H 3350 7300 50  0000 C CNN
F 1 "TSX-3225_16.0000MF09Z-AC3" H 3250 7450 50  0001 L CNN
F 2 "project:Epson-TSX-3225_16.0000MF09Z-AC3-0" H 3250 7550 50  0001 L CNN
F 3 "https://support.epson.biz/td/api/doc_check.php?dl=brief_FA-238V_en.pdf" H 3250 7650 50  0001 L CNN
F 4 "Crys" H 3250 7750 50  0001 L CNN "category"
F 5 "16MHz ±10ppm Crystal 9pF 60 Ohms 4-SMD, No Lead" H 3250 7850 50  0001 L CNN "digikey description"
F 6 "SER3628CT-ND" H 3250 7950 50  0001 L CNN "digikey part number"
F 7 "yes" H 3250 8050 50  0001 L CNN "lead free"
F 8 "36bb9251123dabb2" H 3250 8150 50  0001 L CNN "library id"
F 9 "Epson" H 3250 8250 50  0001 L CNN "manufacturer"
F 10 "732-TX325-16F09Z-AC3" H 3250 8350 50  0001 L CNN "mouser part number"
F 11 "XTAL_SMD_3MM2_2MM5" H 3250 8450 50  0001 L CNN "package"
F 12 "yes" H 3250 8550 50  0001 L CNN "rohs"
F 13 "+75°C" H 3250 8650 50  0001 L CNN "temperature range high"
F 14 "-20°C" H 3250 8750 50  0001 L CNN "temperature range low"
F 15 "16MHz" H 3550 7300 50  0000 C CNN "Frequency"
F 16 "9pF" H 3750 7300 50  0000 C CNN "Capacitance"
	1    3250 7150
	1    0    0    -1  
$EndComp
Wire Wire Line
	3050 6700 3300 6700
$Comp
L project:CGA3E2C0G1H060D080AA C?
U 1 1 6007BADE
P 3850 7250
AR Path="/5FC1E32F/6007BADE" Ref="C?"  Part="1" 
AR Path="/5FBD86FB/6007BADE" Ref="C?"  Part="1" 
AR Path="/6007BADE" Ref="C8"  Part="1" 
F 0 "C8" H 3978 7287 50  0000 L CNN
F 1 "CGA3E2C0G1H060D080AA" H 3850 7450 50  0001 L CNN
F 2 "project:TDK-CGA3E-0.3-0.1-0-0-IPC_A" H 3850 7550 50  0001 L CNN
F 3 "https://product.tdk.com/info/en/catalog/spec/mlccspec_automotive_general_en.pdf" H 3850 7650 50  0001 L CNN
F 4 "Yes" H 3850 7750 50  0001 L CNN "automotive"
F 5 "Grade 1" H 3850 7850 50  0001 L CNN "automotive grade"
F 6 "6pF" H 3978 7196 50  0000 L CNN "capacitance"
F 7 "Cap" H 3850 8050 50  0001 L CNN "category"
F 8 "Passive Components" H 3850 8150 50  0001 L CNN "device class L1"
F 9 "Capacitors" H 3850 8250 50  0001 L CNN "device class L2"
F 10 "Ceramic Capacitors" H 3850 8350 50  0001 L CNN "device class L3"
F 11 "1.1mm" H 3850 8450 50  0001 L CNN "height"
F 12 "CAPC16080X80" H 3850 8550 50  0001 L CNN "ipc land pattern name"
F 13 "Yes" H 3850 8650 50  0001 L CNN "lead free"
F 14 "9251288ec5c7c3b4" H 3850 8750 50  0001 L CNN "library id"
F 15 "TDK" H 3850 8850 50  0001 L CNN "manufacturer"
F 16 "Ceramic" H 3850 8950 50  0001 L CNN "material"
F 17 "0603" H 3978 7105 50  0000 L CNN "package"
F 18 "Yes" H 3850 9250 50  0001 L CNN "rohs"
F 19 "C0G" H 3850 9350 50  0001 L CNN "temperature characteristic"
F 20 "30ppm/°C" H 3850 9450 50  0001 L CNN "temperature coefficient"
F 21 "+125°C" H 3850 9550 50  0001 L CNN "temperature range high"
F 22 "-55°C" H 3850 9650 50  0001 L CNN "temperature range low"
F 23 "0.5pF" H 3850 9750 50  0001 L CNN "tolerance"
F 24 "50V" H 3978 7014 50  0000 L CNN "voltage rating"
	1    3850 7250
	1    0    0    -1  
$EndComp
$Comp
L project:CGA3E2C0G1H060D080AA C?
U 1 1 6007BAF9
P 3050 7250
AR Path="/5FC1E32F/6007BAF9" Ref="C?"  Part="1" 
AR Path="/5FBD86FB/6007BAF9" Ref="C?"  Part="1" 
AR Path="/6007BAF9" Ref="C6"  Part="1" 
F 0 "C6" H 3178 7287 50  0000 L CNN
F 1 "CGA3E2C0G1H060D080AA" H 3050 7450 50  0001 L CNN
F 2 "project:TDK-CGA3E-0.3-0.1-0-0-IPC_A" H 3050 7550 50  0001 L CNN
F 3 "https://product.tdk.com/info/en/catalog/spec/mlccspec_automotive_general_en.pdf" H 3050 7650 50  0001 L CNN
F 4 "Yes" H 3050 7750 50  0001 L CNN "automotive"
F 5 "Grade 1" H 3050 7850 50  0001 L CNN "automotive grade"
F 6 "6pF" H 3178 7196 50  0000 L CNN "capacitance"
F 7 "Cap" H 3050 8050 50  0001 L CNN "category"
F 8 "Passive Components" H 3050 8150 50  0001 L CNN "device class L1"
F 9 "Capacitors" H 3050 8250 50  0001 L CNN "device class L2"
F 10 "Ceramic Capacitors" H 3050 8350 50  0001 L CNN "device class L3"
F 11 "1.1mm" H 3050 8450 50  0001 L CNN "height"
F 12 "CAPC16080X80" H 3050 8550 50  0001 L CNN "ipc land pattern name"
F 13 "Yes" H 3050 8650 50  0001 L CNN "lead free"
F 14 "9251288ec5c7c3b4" H 3050 8750 50  0001 L CNN "library id"
F 15 "TDK" H 3050 8850 50  0001 L CNN "manufacturer"
F 16 "Ceramic" H 3050 8950 50  0001 L CNN "material"
F 17 "0603" H 3178 7105 50  0000 L CNN "package"
F 18 "Yes" H 3050 9250 50  0001 L CNN "rohs"
F 19 "C0G" H 3050 9350 50  0001 L CNN "temperature characteristic"
F 20 "30ppm/°C" H 3050 9450 50  0001 L CNN "temperature coefficient"
F 21 "+125°C" H 3050 9550 50  0001 L CNN "temperature range high"
F 22 "-55°C" H 3050 9650 50  0001 L CNN "temperature range low"
F 23 "0.5pF" H 3050 9750 50  0001 L CNN "tolerance"
F 24 "50V" H 3178 7014 50  0000 L CNN "voltage rating"
	1    3050 7250
	1    0    0    -1  
$EndComp
Wire Wire Line
	3350 7150 3050 7150
Wire Wire Line
	3050 7150 3050 7250
Wire Wire Line
	3750 7150 3850 7150
Wire Wire Line
	3850 7150 3850 7250
Wire Wire Line
	3050 7450 3050 7550
Wire Wire Line
	3850 7550 3850 7450
Wire Wire Line
	3650 7350 3700 7350
Wire Wire Line
	3700 7350 3700 7550
Connection ~ 3700 7550
Wire Wire Line
	3700 7550 3850 7550
Wire Wire Line
	3450 7350 3400 7350
Wire Wire Line
	3400 7350 3400 7550
Connection ~ 3400 7550
Wire Wire Line
	3400 7550 3700 7550
Wire Wire Line
	3050 7000 3050 7150
Connection ~ 3050 7150
Wire Wire Line
	3850 6900 3850 7150
Connection ~ 3850 7150
NoConn ~ 3050 4300
NoConn ~ 3050 4400
Wire Wire Line
	3050 5400 3300 5400
Wire Wire Line
	3050 5500 3300 5500
Text Label 3100 5400 0    50   ~ 0
RX
Text Label 3100 5500 0    50   ~ 0
TX
Wire Wire Line
	3300 5600 3050 5600
Text Label 3100 5600 0    50   ~ 0
IRQ
Wire Wire Line
	3050 5000 3300 5000
Wire Wire Line
	3050 5100 3300 5100
$Comp
L power:GND #PWR016
U 1 1 6009E038
P 3050 7600
F 0 "#PWR016" H 3050 7350 50  0001 C CNN
F 1 "GND" H 3055 7427 50  0000 C CNN
F 2 "" H 3050 7600 50  0001 C CNN
F 3 "" H 3050 7600 50  0001 C CNN
	1    3050 7600
	1    0    0    -1  
$EndComp
Wire Wire Line
	3050 7600 3050 7550
Connection ~ 3050 7550
$Comp
L power:GND #PWR04
U 1 1 600A3F35
P 1200 7050
F 0 "#PWR04" H 1200 6800 50  0001 C CNN
F 1 "GND" H 1205 6877 50  0000 C CNN
F 2 "" H 1200 7050 50  0001 C CNN
F 3 "" H 1200 7050 50  0001 C CNN
	1    1200 7050
	1    0    0    -1  
$EndComp
Wire Wire Line
	1200 7050 1200 7000
$Comp
L power:+5V #PWR01
U 1 1 600A9CA1
P 750 4250
F 0 "#PWR01" H 750 4100 50  0001 C CNN
F 1 "+5V" H 765 4423 50  0000 C CNN
F 2 "" H 750 4250 50  0001 C CNN
F 3 "" H 750 4250 50  0001 C CNN
	1    750  4250
	1    0    0    -1  
$EndComp
Wire Wire Line
	750  4250 750  4300
Connection ~ 750  4300
$Comp
L power:+5V #PWR012
U 1 1 600AFE22
P 2550 7600
F 0 "#PWR012" H 2550 7450 50  0001 C CNN
F 1 "+5V" H 2565 7773 50  0000 C CNN
F 2 "" H 2550 7600 50  0001 C CNN
F 3 "" H 2550 7600 50  0001 C CNN
	1    2550 7600
	0    1    1    0   
$EndComp
Wire Wire Line
	2200 7600 2550 7600
$Comp
L power:GND #PWR013
U 1 1 600B5CAF
P 2550 7800
F 0 "#PWR013" H 2550 7550 50  0001 C CNN
F 1 "GND" H 2555 7627 50  0000 C CNN
F 2 "" H 2550 7800 50  0001 C CNN
F 3 "" H 2550 7800 50  0001 C CNN
	1    2550 7800
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3050 5200 3300 5200
$Comp
L project:RC0603FR-0710KL R1
U 1 1 6011056E
P 700 7800
F 0 "R1" H 1050 8105 50  0000 C CNN
F 1 "RC0603FR-0710KL" H 700 8100 50  0001 L CNN
F 2 "project:Yageo-RC0603-0-0-IPC_A" H 700 8200 50  0001 L CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RC_Group_51_RoHS_L_9.pdf" H 700 8300 50  0001 L CNN
F 4 "541-1.00HHCT-ND" H 700 8400 50  0001 L CNN "approved alternate part number"
F 5 "Res" H 700 8500 50  0001 L CNN "category"
F 6 "Thick Film" H 700 8600 50  0001 L CNN "composition"
F 7 "Passive Components" H 700 8700 50  0001 L CNN "device class L1"
F 8 "Resistors" H 700 8800 50  0001 L CNN "device class L2"
F 9 "Chip SMD Resistors" H 700 8900 50  0001 L CNN "device class L3"
F 10 "RES SMD 10K OHM 1% 1/10W 0603" H 700 9000 50  0001 L CNN "digikey description"
F 11 "311-10.0KHRTR-ND" H 700 9100 50  0001 L CNN "digikey part number"
F 12 "0.5mm" H 700 9200 50  0001 L CNN "height"
F 13 "RESC15585X45" H 700 9300 50  0001 L CNN "ipc land pattern name"
F 14 "yes" H 700 9400 50  0001 L CNN "lead free"
F 15 "368bdb3a487a5469" H 700 9500 50  0001 L CNN "library id"
F 16 "Yageo" H 700 9600 50  0001 L CNN "manufacturer"
F 17 "Surface Mount Thick Film Resistor, RC Series, 10 kohm, 100 mW, - 1%, 50 V, 0603 [1608 Metric]" H 700 9700 50  0001 L CNN "mouser description"
F 18 "603-RC0603FR-0710KL" H 700 9800 50  0001 L CNN "mouser part number"
F 19 "0603" H 1050 8014 50  0000 C CNN "package"
F 20 "100mW" H 700 10000 50  0001 L CNN "power"
F 21 "0.1W" H 700 10100 50  0001 L CNN "power rating"
F 22 "10kΩ" H 1050 7923 50  0000 C CNN "resistance"
F 23 "yes" H 700 10300 50  0001 L CNN "rohs"
F 24 "RC" H 700 10400 50  0001 L CNN "series"
F 25 "0mm" H 700 10500 50  0001 L CNN "standoff height"
F 26 "100ppm/°C" H 700 10600 50  0001 L CNN "temperature coefficient"
F 27 "+155°C" H 700 10700 50  0001 L CNN "temperature range high"
F 28 "-55°C" H 700 10800 50  0001 L CNN "temperature range low"
F 29 "1%" H 700 10900 50  0001 L CNN "tolerance"
F 30 "75V" H 700 11000 50  0001 L CNN "voltage"
F 31 "75V" H 700 11100 50  0001 L CNN "voltage rating"
	1    700  7800
	1    0    0    -1  
$EndComp
$Comp
L project:RC0603FR-071K65L R?
U 1 1 6014440A
P 3850 6100
AR Path="/5FBD86FB/6014440A" Ref="R?"  Part="1" 
AR Path="/5FC1E32F/6014440A" Ref="R?"  Part="1" 
AR Path="/6014440A" Ref="R3"  Part="1" 
F 0 "R3" H 4200 6405 50  0000 C CNN
F 1 "RC0603FR-071K65L" H 4200 6224 50  0001 C CNN
F 2 "project:Yageo-RC0603-0-0-IPC_A" H 3850 6500 50  0001 L CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RC_Group_51_RoHS_L_9.pdf" H 3850 6600 50  0001 L CNN
F 4 "541-1.00HHCT-ND" H 3850 6700 50  0001 L CNN "approved alternate part number"
F 5 "Res" H 3850 6800 50  0001 L CNN "category"
F 6 "Thick Film" H 3850 6900 50  0001 L CNN "composition"
F 7 "Passive Components" H 3850 7000 50  0001 L CNN "device class L1"
F 8 "Resistors" H 3850 7100 50  0001 L CNN "device class L2"
F 9 "Chip SMD Resistors" H 3850 7200 50  0001 L CNN "device class L3"
F 10 "RES SMD 1.65K OHM 1% 1/10W 0603" H 3850 7300 50  0001 L CNN "digikey description"
F 11 "311-1.65KHRTR-ND" H 3850 7400 50  0001 L CNN "digikey part number"
F 12 "0.5mm" H 3850 7500 50  0001 L CNN "height"
F 13 "RESC15585X45" H 3850 7600 50  0001 L CNN "ipc land pattern name"
F 14 "yes" H 3850 7700 50  0001 L CNN "lead free"
F 15 "ce4140018ac48837" H 3850 7800 50  0001 L CNN "library id"
F 16 "Yageo" H 3850 7900 50  0001 L CNN "manufacturer"
F 17 "Thick Film Resistors - SMD 1.65K OHM 1%\\n" H 3850 8000 50  0001 L CNN "mouser description"
F 18 "603-RC0603FR-071K65L" H 3850 8100 50  0001 L CNN "mouser part number"
F 19 "0603" H 4200 6314 50  0000 C CNN "package"
F 20 "100mW" H 3850 8300 50  0001 L CNN "power"
F 21 "0.1W" H 3850 8400 50  0001 L CNN "power rating"
F 22 "1.65kΩ" H 4200 6223 50  0000 C CNN "resistance"
F 23 "yes" H 3850 8600 50  0001 L CNN "rohs"
F 24 "RC" H 3850 8700 50  0001 L CNN "series"
F 25 "0mm" H 3850 8800 50  0001 L CNN "standoff height"
F 26 "100ppm/°C" H 3850 8900 50  0001 L CNN "temperature coefficient"
F 27 "+155°C" H 3850 9000 50  0001 L CNN "temperature range high"
F 28 "-55°C" H 3850 9100 50  0001 L CNN "temperature range low"
F 29 "1%" H 3850 9200 50  0001 L CNN "tolerance"
F 30 "75V" H 3850 9300 50  0001 L CNN "voltage"
F 31 "75V" H 3850 9400 50  0001 L CNN "voltage rating"
	1    3850 6100
	1    0    0    -1  
$EndComp
Wire Wire Line
	3950 6100 3900 6100
Wire Wire Line
	4450 6100 4500 6100
$Comp
L project:LTST-C190KRKT D?
U 1 1 6014442F
P 3500 6100
AR Path="/5FBD86FB/6014442F" Ref="D?"  Part="1" 
AR Path="/5FC1E32F/6014442F" Ref="D?"  Part="1" 
AR Path="/6014442F" Ref="D4"  Part="1" 
F 0 "D4" H 3700 6700 50  0000 C CNN
F 1 "LTST-C190KRKT" H 3700 6600 50  0000 C CNN
F 2 "project:Lite-On-LTST-C190KRKT-MFG" H 3500 6700 50  0001 L CNN
F 3 "https://optoelectronics.liteon.com/upload/download/DS-22-99-0151/LTST-C190KRKT.pdf" H 3500 6800 50  0001 L CNN
F 4 "No" H 3500 6900 50  0001 L CNN "automotive"
F 5 "Diode" H 3500 7000 50  0001 L CNN "category"
F 6 "Red" H 3500 6500 50  0000 C CNN "colour"
F 7 "Optoelectronics" H 3500 7200 50  0001 L CNN "device class L1"
F 8 "LEDs" H 3500 7300 50  0001 L CNN "device class L2"
F 9 "unset" H 3500 7400 50  0001 L CNN "device class L3"
F 10 "LED RED CLEAR CHIP SMD" H 3500 7500 50  0001 L CNN "digikey description"
F 11 "160-1436-1-ND" H 3500 7600 50  0001 L CNN "digikey part number"
F 12 "20mA" H 3900 6500 50  0000 C CNN "forward current"
F 13 "2.4V" H 3700 6500 50  0000 C CNN "forward voltage"
F 14 "0.9mm" H 3500 7900 50  0001 L CNN "height"
F 15 "Yes" H 3500 8000 50  0001 L CNN "lead free"
F 16 "Top View" H 3500 8100 50  0001 L CNN "led orientation"
F 17 "3c5f6cfddbd1ff06" H 3500 8200 50  0001 L CNN "library id"
F 18 "18-180mcd" H 3500 8300 50  0001 L CNN "luminous intensity"
F 19 "Lite-On" H 3500 8400 50  0001 L CNN "manufacturer"
F 20 "859-LTST-C190KRKT" H 3500 8500 50  0001 L CNN "mouser part number"
F 21 "0603" H 3700 6394 50  0000 C CNN "package"
F 22 "639nm" H 3500 8700 50  0001 L CNN "peak emmision wavelength"
F 23 "62.5mW" H 3500 8800 50  0001 L CNN "power dissipation"
F 24 "5V" H 3500 8900 50  0001 L CNN "reverse voltage"
F 25 "Yes" H 3500 9000 50  0001 L CNN "rohs"
F 26 "+85°C" H 3500 9100 50  0001 L CNN "temperature range high"
F 27 "-30°C" H 3500 9200 50  0001 L CNN "temperature range low"
F 28 "130°" H 3500 9300 50  0001 L CNN "viewing angle"
F 29 "631nm" H 3500 9400 50  0001 L CNN "wavelength"
	1    3500 6100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR022
U 1 1 60163A79
P 4500 6100
F 0 "#PWR022" H 4500 5850 50  0001 C CNN
F 1 "GND" H 4505 5927 50  0000 C CNN
F 2 "" H 4500 6100 50  0001 C CNN
F 3 "" H 4500 6100 50  0001 C CNN
	1    4500 6100
	0    -1   -1   0   
$EndComp
$Comp
L project:SSW-106-02-T-S J5
U 1 1 6016C588
P 5200 5500
F 0 "J5" V 4950 5800 50  0000 L CNN
F 1 "SSW-106-02-T-S" V 5050 5150 50  0000 L CNN
F 2 "project:Samtec-SSW-106-02-T-S-Manufacturer_Recommended" H 5200 6200 50  0001 L CNN
F 3 "http://www.samtec.com/documents/webfiles/pdf/SSW_TH.PDF" H 5200 6300 50  0001 L CNN
F 4 "Manufacturer URL" H 5200 6400 50  0001 L CNN "Component Link 1 Description"
F 5 "http://www.samtec.com" H 5200 6500 50  0001 L CNN "Component Link 1 URL"
F 6 "Package Specification" H 5200 6600 50  0001 L CNN "Component Link 3 Description"
F 7 "http://www.samtec.com/documents/webfiles/cpdf/SSW-1XX-XX-XXX-X-XX-XXX-XX-MKT.pdf" H 5200 6700 50  0001 L CNN "Component Link 3 URL"
F 8 "F-214" H 5200 6800 50  0001 L CNN "Datasheet Version"
F 9 "Through-hole" H 5200 6900 50  0001 L CNN "Mounting Technology"
F 10 "Vertical" H 5200 7000 50  0001 L CNN "Orientation"
F 11 "Assembly, Socket Strip .016 X .031 Tail" H 5200 7100 50  0001 L CNN "Package Description"
F 12 "CA, 6/1987" H 5200 7200 50  0001 L CNN "Package Version"
F 13 "2.54 mm" H 5200 7300 50  0001 L CNN "Pitch"
F 14 "-55 to 105 degC" H 5200 7400 50  0001 L CNN "Temperature Range"
F 15 "Conn" H 5200 7500 50  0001 L CNN "category"
F 16 "679223" H 5200 7600 50  0001 L CNN "ciiva ids"
F 17 "68252b6a81f3dbac" H 5200 7700 50  0001 L CNN "library id"
F 18 "Samtec" H 5200 7800 50  0001 L CNN "manufacturer"
F 19 "SSW-106-02-X-S" H 5200 7900 50  0001 L CNN "package"
F 20 "1404374511" H 5200 8000 50  0001 L CNN "release date"
F 21 "515F7714-EB50-4157-8AB6-9F808CB470BE" H 5200 8100 50  0001 L CNN "vault revision"
F 22 "yes" H 5200 8200 50  0001 L CNN "imported"
	1    5200 5500
	0    1    1    0   
$EndComp
Wire Wire Line
	5300 5500 5050 5500
$Comp
L power:GND #PWR035
U 1 1 60174894
P 5050 5500
F 0 "#PWR035" H 5050 5250 50  0001 C CNN
F 1 "GND" V 5055 5372 50  0000 R CNN
F 2 "" H 5050 5500 50  0001 C CNN
F 3 "" H 5050 5500 50  0001 C CNN
	1    5050 5500
	0    1    1    0   
$EndComp
Wire Wire Line
	5300 5600 5000 5600
Wire Wire Line
	5300 5700 5000 5700
Wire Wire Line
	5300 5800 5000 5800
Wire Wire Line
	5300 5900 5000 5900
Wire Wire Line
	5300 6000 5000 6000
$Comp
L power:+5V #PWR032
U 1 1 6019C552
P 5000 5600
F 0 "#PWR032" H 5000 5450 50  0001 C CNN
F 1 "+5V" V 5015 5728 50  0000 L CNN
F 2 "" H 5000 5600 50  0001 C CNN
F 3 "" H 5000 5600 50  0001 C CNN
	1    5000 5600
	0    -1   -1   0   
$EndComp
Text Label 5050 5700 0    50   ~ 0
MISO
Text Label 5050 5800 0    50   ~ 0
MOSI
Text Label 5050 5900 0    50   ~ 0
SCK
Text Label 5050 6000 0    50   ~ 0
SDCS
Wire Wire Line
	3050 6100 3500 6100
Text Notes 4950 6200 0    50   ~ 0
Socket for SD Card
Wire Wire Line
	3050 6500 3300 6500
Text Label 3100 6500 0    50   ~ 0
SDCS
$Comp
L power:+5VP #PWR045
U 1 1 601BEE21
P 7550 800
F 0 "#PWR045" H 7550 650 50  0001 C CNN
F 1 "+5VP" V 7565 928 50  0000 L CNN
F 2 "" H 7550 800 50  0001 C CNN
F 3 "" H 7550 800 50  0001 C CNN
	1    7550 800 
	0    -1   -1   0   
$EndComp
$Comp
L power:+5VP #PWR057
U 1 1 601C6425
P 9150 800
F 0 "#PWR057" H 9150 650 50  0001 C CNN
F 1 "+5VP" V 9165 928 50  0000 L CNN
F 2 "" H 9150 800 50  0001 C CNN
F 3 "" H 9150 800 50  0001 C CNN
	1    9150 800 
	0    -1   -1   0   
$EndComp
$Comp
L power:+5VP #PWR047
U 1 1 601C6A1A
P 7550 1800
F 0 "#PWR047" H 7550 1650 50  0001 C CNN
F 1 "+5VP" V 7565 1928 50  0000 L CNN
F 2 "" H 7550 1800 50  0001 C CNN
F 3 "" H 7550 1800 50  0001 C CNN
	1    7550 1800
	0    -1   -1   0   
$EndComp
$Comp
L power:+5VP #PWR059
U 1 1 601C6CE7
P 9150 1800
F 0 "#PWR059" H 9150 1650 50  0001 C CNN
F 1 "+5VP" V 9165 1928 50  0000 L CNN
F 2 "" H 9150 1800 50  0001 C CNN
F 3 "" H 9150 1800 50  0001 C CNN
	1    9150 1800
	0    -1   -1   0   
$EndComp
$Comp
L power:+5VP #PWR061
U 1 1 601C730A
P 9150 2800
F 0 "#PWR061" H 9150 2650 50  0001 C CNN
F 1 "+5VP" V 9165 2928 50  0000 L CNN
F 2 "" H 9150 2800 50  0001 C CNN
F 3 "" H 9150 2800 50  0001 C CNN
	1    9150 2800
	0    -1   -1   0   
$EndComp
$Comp
L power:+5VP #PWR049
U 1 1 601C7684
P 7550 2800
F 0 "#PWR049" H 7550 2650 50  0001 C CNN
F 1 "+5VP" V 7565 2928 50  0000 L CNN
F 2 "" H 7550 2800 50  0001 C CNN
F 3 "" H 7550 2800 50  0001 C CNN
	1    7550 2800
	0    -1   -1   0   
$EndComp
$Comp
L power:+5VP #PWR051
U 1 1 601C7B98
P 7550 3800
F 0 "#PWR051" H 7550 3650 50  0001 C CNN
F 1 "+5VP" V 7565 3928 50  0000 L CNN
F 2 "" H 7550 3800 50  0001 C CNN
F 3 "" H 7550 3800 50  0001 C CNN
	1    7550 3800
	0    -1   -1   0   
$EndComp
$Comp
L power:+5VP #PWR063
U 1 1 601C7F7C
P 9150 3800
F 0 "#PWR063" H 9150 3650 50  0001 C CNN
F 1 "+5VP" V 9165 3928 50  0000 L CNN
F 2 "" H 9150 3800 50  0001 C CNN
F 3 "" H 9150 3800 50  0001 C CNN
	1    9150 3800
	0    -1   -1   0   
$EndComp
$Comp
L power:+5VP #PWR065
U 1 1 601C81FC
P 9150 4800
F 0 "#PWR065" H 9150 4650 50  0001 C CNN
F 1 "+5VP" V 9165 4928 50  0000 L CNN
F 2 "" H 9150 4800 50  0001 C CNN
F 3 "" H 9150 4800 50  0001 C CNN
	1    9150 4800
	0    -1   -1   0   
$EndComp
$Comp
L power:+5VP #PWR053
U 1 1 601C85BC
P 7550 4800
F 0 "#PWR053" H 7550 4650 50  0001 C CNN
F 1 "+5VP" V 7565 4928 50  0000 L CNN
F 2 "" H 7550 4800 50  0001 C CNN
F 3 "" H 7550 4800 50  0001 C CNN
	1    7550 4800
	0    -1   -1   0   
$EndComp
$Comp
L project:BAT54T1G D1
U 1 1 601CADAC
P 2400 1100
F 0 "D1" H 2550 1637 50  0000 C CNN
F 1 "BAT54T1G" H 2550 1546 50  0000 C CNN
F 2 "project:On_Semiconductor-BAT54T1G-Manufacturer_Recommended" H 2400 1500 50  0001 L CNN
F 3 "http://www.onsemi.cn/pub_link/Collateral/BAT54T1-D.PDF" H 2400 1600 50  0001 L CNN
F 4 "Manufacturer URL" H 2400 1700 50  0001 L CNN "Component Link 1 Description"
F 5 "http://www.onsemi.com/" H 2400 1800 50  0001 L CNN "Component Link 1 URL"
F 6 "Package Specification" H 2400 1900 50  0001 L CNN "Component Link 3 Description"
F 7 "http://www.onsemi.com/pub_link/Collateral/425-04.PDF" H 2400 2000 50  0001 L CNN "Component Link 3 URL"
F 8 "Single" H 2400 2100 50  0001 L CNN "Configuration"
F 9 "Rev. 9" H 2400 2200 50  0001 L CNN "Datasheet Version"
F 10 "0.6" H 2400 2300 50  0001 L CNN "IFSM Max A"
F 11 "Surface Mount" H 2400 2400 50  0001 L CNN "Mounting Technology"
F 12 "2-Pin SOD, Body 1.6 x 2.69 mm" H 2400 2500 50  0001 L CNN "Package Description"
F 13 "Rev. G, 10/2009" H 2400 2600 50  0001 L CNN "Package Version"
F 14 "Tape and Reel" H 2400 2700 50  0001 L CNN "Packing"
F 15 "Diode" H 2400 2800 50  0001 L CNN "category"
F 16 "19353990" H 2400 2900 50  0001 L CNN "ciiva ids"
F 17 "f50e6730bced61e6" H 2400 3000 50  0001 L CNN "library id"
F 18 "On Semiconductor" H 2400 3100 50  0001 L CNN "manufacturer"
F 19 "SOD-123" H 2550 1455 50  0000 C CNN "package"
F 20 "1407123099" H 2400 3300 50  0001 L CNN "release date"
F 21 "Yes" H 2400 3400 50  0001 L CNN "rohs"
F 22 "089F5CB3-91C9-4093-8502-D57A14867F47" H 2400 3500 50  0001 L CNN "vault revision"
F 23 "yes" H 2400 3600 50  0001 L CNN "imported"
F 24 "5" H 2400 3700 50  0001 L CNN "trr Max ns"
F 25 "30V" H 2550 1364 50  0000 C CNN "Reverse Breakdown Voltage"
F 26 "200mA" H 2550 1273 50  0000 C CNN "Current"
	1    2400 1100
	1    0    0    -1  
$EndComp
Wire Wire Line
	2700 1100 2850 1100
$Comp
L power:+5V #PWR014
U 1 1 601DB0CD
P 2850 1100
F 0 "#PWR014" H 2850 950 50  0001 C CNN
F 1 "+5V" H 2865 1273 50  0000 C CNN
F 2 "" H 2850 1100 50  0001 C CNN
F 3 "" H 2850 1100 50  0001 C CNN
	1    2850 1100
	1    0    0    -1  
$EndComp
Text Notes 2150 1750 0    50   ~ 0
Auto Select Voltage
$Comp
L project:FT231XQ-R U2
U 1 1 602086E6
P 2200 2150
F 0 "U2" H 3150 2505 50  0000 C CNN
F 1 "FT231XQ-R" H 3150 2414 50  0000 C CNN
F 2 "project:FTDI-QFN-20-0-0-MFG" H 2200 2550 50  0001 L CNN
F 3 "https://www.ftdichip.com/Support/Documents/DataSheets/ICs/DS_FT231X.pdf" H 2200 2650 50  0001 L CNN
F 4 "+85°C" H 2200 2750 50  0001 L CNN "ambient temperature range high"
F 5 "-40°C" H 2200 2850 50  0001 L CNN "ambient temperature range low"
F 6 "No" H 2200 2950 50  0001 L CNN "automotive"
F 7 "IC" H 2200 3050 50  0001 L CNN "category"
F 8 "Integrated Circuits (ICs)" H 2200 3150 50  0001 L CNN "device class L1"
F 9 "Interface ICs" H 2200 3250 50  0001 L CNN "device class L2"
F 10 "Controllers" H 2200 3350 50  0001 L CNN "device class L3"
F 11 "IC USB SERIAL FULL UART 20QFN" H 2200 3450 50  0001 L CNN "digikey description"
F 12 "768-1128-1-ND" H 2200 3550 50  0001 L CNN "digikey part number"
F 13 "https://www.ftdichip.com/Support/Documents/TechnicalNotes/TN_166%20FTDI%20Example%20IC%20PCB%20Footprints.pdf" H 2200 3650 50  0001 L CNN "footprint url"
F 14 "USB to UART" H 2200 3750 50  0001 L CNN "function"
F 15 "0.8mm" H 2200 3850 50  0001 L CNN "height"
F 16 "UART,USB" H 2200 3950 50  0001 L CNN "interface"
F 17 "QFN50P400X400X75-20" H 2200 4050 50  0001 L CNN "ipc land pattern name"
F 18 "Yes" H 2200 4150 50  0001 L CNN "lead free"
F 19 "d3944f6512027fdf" H 2200 4250 50  0001 L CNN "library id"
F 20 "FTDI" H 2200 4350 50  0001 L CNN "manufacturer"
F 21 "5.5V" H 2200 4450 50  0001 L CNN "max supply voltage"
F 22 "1.62V" H 2200 4550 50  0001 L CNN "min supply voltage"
F 23 "895-FT231XQ-R" H 2200 4650 50  0001 L CNN "mouser part number"
F 24 "125-8000uA" H 2200 4750 50  0001 L CNN "nominal supply current"
F 25 "QFN20" H 3150 2323 50  0000 C CNN "package"
F 26 "Yes" H 2200 4950 50  0001 L CNN "rohs"
F 27 "0mm" H 2200 5050 50  0001 L CNN "standoff height"
F 28 "+85°C" H 2200 5150 50  0001 L CNN "temperature range high"
F 29 "-40°C" H 2200 5250 50  0001 L CNN "temperature range low"
	1    2200 2150
	1    0    0    -1  
$EndComp
Wire Wire Line
	2300 2250 2200 2250
Wire Wire Line
	2200 2250 2200 2150
Wire Wire Line
	2200 2150 2300 2150
$Comp
L power:+5V #PWR011
U 1 1 60236616
P 2200 2100
F 0 "#PWR011" H 2200 1950 50  0001 C CNN
F 1 "+5V" H 2215 2273 50  0000 C CNN
F 2 "" H 2200 2100 50  0001 C CNN
F 3 "" H 2200 2100 50  0001 C CNN
	1    2200 2100
	1    0    0    -1  
$EndComp
Wire Wire Line
	2200 2100 2200 2150
Connection ~ 2200 2150
$Comp
L project:475890001 J2
U 1 1 602402C9
P 1750 2250
F 0 "J2" H 1300 2705 50  0000 C CNN
F 1 "475890001" H 1300 2614 50  0000 C CNN
F 2 "project:Molex-475890001-MFG" H 1750 2750 50  0001 L CNN
F 3 "https://www.molex.com/pdm_docs/ps/PS-47589-001-001.pdf" H 1750 2850 50  0001 L CNN
F 4 "No" H 1750 2950 50  0001 L CNN "automotive"
F 5 "Conn" H 1750 3050 50  0001 L CNN "category"
F 6 "Gold,Nickel" H 1750 3150 50  0001 L CNN "contact material"
F 7 "1/1.8A" H 1750 3250 50  0001 L CNN "current rating"
F 8 "Connectors" H 1750 3350 50  0001 L CNN "device class L1"
F 9 "USB Connectors" H 1750 3450 50  0001 L CNN "device class L2"
F 10 "unset" H 1750 3550 50  0001 L CNN "device class L3"
F 11 "CONN RCPT MICRO USB AB 5P SMD RA" H 1300 2523 50  0000 C CNN "digikey description"
F 12 "WM17143CT-ND" H 1750 3750 50  0001 L CNN "digikey part number"
F 13 "https://www.molex.com/pdm_docs/sd/475890001_sd.pdf" H 1750 3850 50  0001 L CNN "footprint url"
F 14 "2.86mm" H 1750 3950 50  0001 L CNN "height"
F 15 "yes" H 1750 4050 50  0001 L CNN "is connector"
F 16 "yes" H 1750 4150 50  0001 L CNN "is female"
F 17 "Yes" H 1750 4250 50  0001 L CNN "lead free"
F 18 "35c17e53108c3912" H 1750 4350 50  0001 L CNN "library id"
F 19 "Molex" H 1750 4450 50  0001 L CNN "manufacturer"
F 20 "538-47589-0001" H 1750 4550 50  0001 L CNN "mouser part number"
F 21 "5" H 1750 4650 50  0001 L CNN "number of contacts"
F 22 "CONN_USB_8MM2_5MM6" H 1750 4750 50  0001 L CNN "package"
F 23 "0.65mm" H 1750 4850 50  0001 L CNN "pitch"
F 24 "Yes" H 1750 4950 50  0001 L CNN "rohs"
F 25 "true" H 1750 5050 50  0001 L CNN "shielding"
F 26 "+85°C" H 1750 5150 50  0001 L CNN "temperature range high"
F 27 "-30°C" H 1750 5250 50  0001 L CNN "temperature range low"
F 28 "USB 2.0" H 1750 5350 50  0001 L CNN "usb standard"
F 29 "30V" H 1750 5450 50  0001 L CNN "voltage rating"
	1    1750 2250
	1    0    0    -1  
$EndComp
$Comp
L project:BAT54T1G D2
U 1 1 602A166B
P 3350 1100
F 0 "D2" H 3500 563 50  0000 C CNN
F 1 "BAT54T1G" H 3500 654 50  0000 C CNN
F 2 "project:On_Semiconductor-BAT54T1G-Manufacturer_Recommended" H 3350 1500 50  0001 L CNN
F 3 "http://www.onsemi.cn/pub_link/Collateral/BAT54T1-D.PDF" H 3350 1600 50  0001 L CNN
F 4 "Manufacturer URL" H 3350 1700 50  0001 L CNN "Component Link 1 Description"
F 5 "http://www.onsemi.com/" H 3350 1800 50  0001 L CNN "Component Link 1 URL"
F 6 "Package Specification" H 3350 1900 50  0001 L CNN "Component Link 3 Description"
F 7 "http://www.onsemi.com/pub_link/Collateral/425-04.PDF" H 3350 2000 50  0001 L CNN "Component Link 3 URL"
F 8 "Single" H 3350 2100 50  0001 L CNN "Configuration"
F 9 "Rev. 9" H 3350 2200 50  0001 L CNN "Datasheet Version"
F 10 "0.6" H 3350 2300 50  0001 L CNN "IFSM Max A"
F 11 "Surface Mount" H 3350 2400 50  0001 L CNN "Mounting Technology"
F 12 "2-Pin SOD, Body 1.6 x 2.69 mm" H 3350 2500 50  0001 L CNN "Package Description"
F 13 "Rev. G, 10/2009" H 3350 2600 50  0001 L CNN "Package Version"
F 14 "Tape and Reel" H 3350 2700 50  0001 L CNN "Packing"
F 15 "Diode" H 3350 2800 50  0001 L CNN "category"
F 16 "19353990" H 3350 2900 50  0001 L CNN "ciiva ids"
F 17 "f50e6730bced61e6" H 3350 3000 50  0001 L CNN "library id"
F 18 "On Semiconductor" H 3350 3100 50  0001 L CNN "manufacturer"
F 19 "SOD-123" H 3500 745 50  0000 C CNN "package"
F 20 "1407123099" H 3350 3300 50  0001 L CNN "release date"
F 21 "Yes" H 3350 3400 50  0001 L CNN "rohs"
F 22 "089F5CB3-91C9-4093-8502-D57A14867F47" H 3350 3500 50  0001 L CNN "vault revision"
F 23 "yes" H 3350 3600 50  0001 L CNN "imported"
F 24 "5" H 3350 3700 50  0001 L CNN "trr Max ns"
F 25 "30V" H 3500 836 50  0000 C CNN "Reverse Breakdown Voltage"
F 26 "200mA" H 3500 927 50  0000 C CNN "Current"
	1    3350 1100
	-1   0    0    1   
$EndComp
Wire Wire Line
	3050 1100 2850 1100
Connection ~ 2850 1100
$Comp
L power:VBUS #PWR06
U 1 1 602BAE4A
P 1750 2250
F 0 "#PWR06" H 1750 2100 50  0001 C CNN
F 1 "VBUS" H 1765 2423 50  0000 C CNN
F 2 "" H 1750 2250 50  0001 C CNN
F 3 "" H 1750 2250 50  0001 C CNN
	1    1750 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	1750 2250 1650 2250
$Comp
L power:VBUS #PWR017
U 1 1 602C401A
P 3500 1100
F 0 "#PWR017" H 3500 950 50  0001 C CNN
F 1 "VBUS" H 3515 1273 50  0000 C CNN
F 2 "" H 3500 1100 50  0001 C CNN
F 3 "" H 3500 1100 50  0001 C CNN
	1    3500 1100
	1    0    0    -1  
$EndComp
Wire Wire Line
	3500 1100 3350 1100
Wire Wire Line
	2300 2450 1650 2450
Wire Wire Line
	2300 2550 1850 2550
Wire Wire Line
	1850 2550 1850 2650
Wire Wire Line
	1850 2650 1650 2650
NoConn ~ 2300 2750
Wire Wire Line
	4000 3550 4050 3550
Wire Wire Line
	4050 3550 4050 3650
Wire Wire Line
	4000 3650 4050 3650
Connection ~ 4050 3650
Wire Wire Line
	4050 3650 4050 3750
Wire Wire Line
	4000 3750 4050 3750
Connection ~ 4050 3750
Wire Wire Line
	4050 3750 4050 3850
$Comp
L power:GND #PWR07
U 1 1 60339AA9
P 1750 3050
F 0 "#PWR07" H 1750 2800 50  0001 C CNN
F 1 "GND" H 1755 2877 50  0000 C CNN
F 2 "" H 1750 3050 50  0001 C CNN
F 3 "" H 1750 3050 50  0001 C CNN
	1    1750 3050
	1    0    0    -1  
$EndComp
NoConn ~ 1650 2850
$Comp
L power:GND #PWR019
U 1 1 6034D910
P 4050 3850
F 0 "#PWR019" H 4050 3600 50  0001 C CNN
F 1 "GND" H 4055 3677 50  0000 C CNN
F 2 "" H 4050 3850 50  0001 C CNN
F 3 "" H 4050 3850 50  0001 C CNN
	1    4050 3850
	1    0    0    -1  
$EndComp
NoConn ~ 950  2150
NoConn ~ 950  2350
NoConn ~ 950  2550
NoConn ~ 950  2750
NoConn ~ 950  2950
NoConn ~ 950  3150
$Comp
L project:RC0603FR-071K65L R?
U 1 1 60388A27
P 4300 2150
AR Path="/5FBD86FB/60388A27" Ref="R?"  Part="1" 
AR Path="/5FC1E32F/60388A27" Ref="R?"  Part="1" 
AR Path="/60388A27" Ref="R4"  Part="1" 
F 0 "R4" H 4350 2300 50  0000 C CNN
F 1 "RC0603FR-071K65L" H 4650 2274 50  0001 C CNN
F 2 "project:Yageo-RC0603-0-0-IPC_A" H 4300 2550 50  0001 L CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RC_Group_51_RoHS_L_9.pdf" H 4300 2650 50  0001 L CNN
F 4 "541-1.00HHCT-ND" H 4300 2750 50  0001 L CNN "approved alternate part number"
F 5 "Res" H 4300 2850 50  0001 L CNN "category"
F 6 "Thick Film" H 4300 2950 50  0001 L CNN "composition"
F 7 "Passive Components" H 4300 3050 50  0001 L CNN "device class L1"
F 8 "Resistors" H 4300 3150 50  0001 L CNN "device class L2"
F 9 "Chip SMD Resistors" H 4300 3250 50  0001 L CNN "device class L3"
F 10 "RES SMD 1.65K OHM 1% 1/10W 0603" H 4300 3350 50  0001 L CNN "digikey description"
F 11 "311-1.65KHRTR-ND" H 4300 3450 50  0001 L CNN "digikey part number"
F 12 "0.5mm" H 4300 3550 50  0001 L CNN "height"
F 13 "RESC15585X45" H 4300 3650 50  0001 L CNN "ipc land pattern name"
F 14 "yes" H 4300 3750 50  0001 L CNN "lead free"
F 15 "ce4140018ac48837" H 4300 3850 50  0001 L CNN "library id"
F 16 "Yageo" H 4300 3950 50  0001 L CNN "manufacturer"
F 17 "Thick Film Resistors - SMD 1.65K OHM 1%\\n" H 4300 4050 50  0001 L CNN "mouser description"
F 18 "603-RC0603FR-071K65L" H 4300 4150 50  0001 L CNN "mouser part number"
F 19 "0603" H 4900 2300 50  0000 C CNN "package"
F 20 "100mW" H 4300 4350 50  0001 L CNN "power"
F 21 "0.1W" H 4300 4450 50  0001 L CNN "power rating"
F 22 "1.65kΩ" H 4600 2300 50  0000 C CNN "resistance"
F 23 "yes" H 4300 4650 50  0001 L CNN "rohs"
F 24 "RC" H 4300 4750 50  0001 L CNN "series"
F 25 "0mm" H 4300 4850 50  0001 L CNN "standoff height"
F 26 "100ppm/°C" H 4300 4950 50  0001 L CNN "temperature coefficient"
F 27 "+155°C" H 4300 5050 50  0001 L CNN "temperature range high"
F 28 "-55°C" H 4300 5150 50  0001 L CNN "temperature range low"
F 29 "1%" H 4300 5250 50  0001 L CNN "tolerance"
F 30 "75V" H 4300 5350 50  0001 L CNN "voltage"
F 31 "75V" H 4300 5450 50  0001 L CNN "voltage rating"
	1    4300 2150
	1    0    0    -1  
$EndComp
$Comp
L project:RC0603FR-071K65L R?
U 1 1 6038A2EC
P 4300 2400
AR Path="/5FBD86FB/6038A2EC" Ref="R?"  Part="1" 
AR Path="/5FC1E32F/6038A2EC" Ref="R?"  Part="1" 
AR Path="/6038A2EC" Ref="R5"  Part="1" 
F 0 "R5" H 4350 2550 50  0000 C CNN
F 1 "RC0603FR-071K65L" H 4650 2524 50  0001 C CNN
F 2 "project:Yageo-RC0603-0-0-IPC_A" H 4300 2800 50  0001 L CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RC_Group_51_RoHS_L_9.pdf" H 4300 2900 50  0001 L CNN
F 4 "541-1.00HHCT-ND" H 4300 3000 50  0001 L CNN "approved alternate part number"
F 5 "Res" H 4300 3100 50  0001 L CNN "category"
F 6 "Thick Film" H 4300 3200 50  0001 L CNN "composition"
F 7 "Passive Components" H 4300 3300 50  0001 L CNN "device class L1"
F 8 "Resistors" H 4300 3400 50  0001 L CNN "device class L2"
F 9 "Chip SMD Resistors" H 4300 3500 50  0001 L CNN "device class L3"
F 10 "RES SMD 1.65K OHM 1% 1/10W 0603" H 4300 3600 50  0001 L CNN "digikey description"
F 11 "311-1.65KHRTR-ND" H 4300 3700 50  0001 L CNN "digikey part number"
F 12 "0.5mm" H 4300 3800 50  0001 L CNN "height"
F 13 "RESC15585X45" H 4300 3900 50  0001 L CNN "ipc land pattern name"
F 14 "yes" H 4300 4000 50  0001 L CNN "lead free"
F 15 "ce4140018ac48837" H 4300 4100 50  0001 L CNN "library id"
F 16 "Yageo" H 4300 4200 50  0001 L CNN "manufacturer"
F 17 "Thick Film Resistors - SMD 1.65K OHM 1%\\n" H 4300 4300 50  0001 L CNN "mouser description"
F 18 "603-RC0603FR-071K65L" H 4300 4400 50  0001 L CNN "mouser part number"
F 19 "0603" H 4900 2550 50  0000 C CNN "package"
F 20 "100mW" H 4300 4600 50  0001 L CNN "power"
F 21 "0.1W" H 4300 4700 50  0001 L CNN "power rating"
F 22 "1.65kΩ" H 4600 2550 50  0000 C CNN "resistance"
F 23 "yes" H 4300 4900 50  0001 L CNN "rohs"
F 24 "RC" H 4300 5000 50  0001 L CNN "series"
F 25 "0mm" H 4300 5100 50  0001 L CNN "standoff height"
F 26 "100ppm/°C" H 4300 5200 50  0001 L CNN "temperature coefficient"
F 27 "+155°C" H 4300 5300 50  0001 L CNN "temperature range high"
F 28 "-55°C" H 4300 5400 50  0001 L CNN "temperature range low"
F 29 "1%" H 4300 5500 50  0001 L CNN "tolerance"
F 30 "75V" H 4300 5600 50  0001 L CNN "voltage"
F 31 "75V" H 4300 5700 50  0001 L CNN "voltage rating"
	1    4300 2400
	1    0    0    -1  
$EndComp
Wire Wire Line
	4000 2150 4400 2150
Wire Wire Line
	4000 2250 4250 2250
Wire Wire Line
	4250 2250 4250 2400
Wire Wire Line
	4250 2400 4400 2400
Wire Wire Line
	4900 2150 5150 2150
Wire Wire Line
	4900 2400 5150 2400
Text Label 4950 2150 0    50   ~ 0
RX
Text Label 4950 2400 0    50   ~ 0
TX
$Comp
L project:CC0603KPX7R9BB104 C?
U 1 1 603C99D4
P 1950 3200
AR Path="/5FC1E32F/603C99D4" Ref="C?"  Part="1" 
AR Path="/5FBD86FB/603C99D4" Ref="C?"  Part="1" 
AR Path="/603C99D4" Ref="C3"  Part="1" 
F 0 "C3" H 2078 3237 50  0000 L CNN
F 1 "CC0603KPX7R9BB104" H 1950 3400 50  0001 L CNN
F 2 "project:YAGEO-CC0603-0-0-0" H 1950 3500 50  0001 L CNN
F 3 "http://www.yageo.com/documents/recent/UPY-GPHC_X7R_6.3V-to-50V_18.pdf" H 1950 3600 50  0001 L CNN
F 4 "No" H 1950 3700 50  0001 L CNN "automotive"
F 5 "100 nF" H 2078 3146 50  0000 L CNN "capacitance"
F 6 "Cap" H 1950 3900 50  0001 L CNN "category"
F 7 "Passive Components" H 1950 4000 50  0001 L CNN "device class L1"
F 8 "Capacitors" H 1950 4100 50  0001 L CNN "device class L2"
F 9 "Ceramic Capacitors" H 1950 4200 50  0001 L CNN "device class L3"
F 10 "CAP CER 0.1UF 50V X7R 0603" H 1950 4300 50  0001 L CNN "digikey description"
F 11 "CC0603KPX7R9BB104-ND" H 1950 4400 50  0001 L CNN "digikey part number"
F 12 "http://www.yageo.com/exep/pages/download/literatures/UPY-C_GEN_24.pdf" H 1950 4500 50  0001 L CNN "footprint url"
F 13 "0.9mm" H 1950 4600 50  0001 L CNN "height"
F 14 "Yes" H 1950 4700 50  0001 L CNN "lead free"
F 15 "7cd692b09ff39865" H 1950 4800 50  0001 L CNN "library id"
F 16 "YAGEO" H 1950 4900 50  0001 L CNN "manufacturer"
F 17 "Ceramic" H 1950 5000 50  0001 L CNN "material"
F 18 "Multilayer Ceramic Capacitors MLCC - SMD/SMT 100nF 50V X7R 10%" H 1950 5100 50  0001 L CNN "mouser description"
F 19 "603-CC603KPX7R9BB104" H 1950 5200 50  0001 L CNN "mouser part number"
F 20 "0603" H 2078 3055 50  0000 L CNN "package"
F 21 "Yes" H 1950 5400 50  0001 L CNN "rohs"
F 22 "X7R" H 1950 5500 50  0001 L CNN "temperature characteristic"
F 23 "15%" H 1950 5600 50  0001 L CNN "temperature coefficient"
F 24 "+125°C" H 1950 5700 50  0001 L CNN "temperature range high"
F 25 "-55°C" H 1950 5800 50  0001 L CNN "temperature range low"
F 26 "0.1" H 1950 5900 50  0001 L CNN "tolerance"
F 27 "50 V" H 2078 2964 50  0000 L CNN "voltage"
F 28 "50 V" H 1950 6100 50  0001 L CNN "voltage rating"
	1    1950 3200
	1    0    0    -1  
$EndComp
Wire Wire Line
	2300 3050 1950 3050
Wire Wire Line
	1950 3050 1950 3200
Wire Wire Line
	1950 3400 1950 3450
$Comp
L power:GND #PWR08
U 1 1 603E02D6
P 1950 3450
F 0 "#PWR08" H 1950 3200 50  0001 C CNN
F 1 "GND" H 1955 3277 50  0000 C CNN
F 2 "" H 1950 3450 50  0001 C CNN
F 3 "" H 1950 3450 50  0001 C CNN
	1    1950 3450
	1    0    0    -1  
$EndComp
NoConn ~ 4000 2450
NoConn ~ 4000 2650
NoConn ~ 4000 2750
NoConn ~ 4000 2850
NoConn ~ 4000 3050
NoConn ~ 4000 3350
Text Label 4950 2650 0    50   ~ 0
RES
Wire Wire Line
	4000 2550 4250 2550
Wire Wire Line
	4250 2550 4250 2650
$Comp
L project:RC0603FR-071K65L R?
U 1 1 6045B8B1
P 5000 3750
AR Path="/5FBD86FB/6045B8B1" Ref="R?"  Part="1" 
AR Path="/5FC1E32F/6045B8B1" Ref="R?"  Part="1" 
AR Path="/6045B8B1" Ref="R9"  Part="1" 
F 0 "R9" H 5350 4055 50  0000 C CNN
F 1 "RC0603FR-071K65L" H 5350 3874 50  0001 C CNN
F 2 "project:Yageo-RC0603-0-0-IPC_A" H 5000 4150 50  0001 L CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RC_Group_51_RoHS_L_9.pdf" H 5000 4250 50  0001 L CNN
F 4 "541-1.00HHCT-ND" H 5000 4350 50  0001 L CNN "approved alternate part number"
F 5 "Res" H 5000 4450 50  0001 L CNN "category"
F 6 "Thick Film" H 5000 4550 50  0001 L CNN "composition"
F 7 "Passive Components" H 5000 4650 50  0001 L CNN "device class L1"
F 8 "Resistors" H 5000 4750 50  0001 L CNN "device class L2"
F 9 "Chip SMD Resistors" H 5000 4850 50  0001 L CNN "device class L3"
F 10 "RES SMD 1.65K OHM 1% 1/10W 0603" H 5000 4950 50  0001 L CNN "digikey description"
F 11 "311-1.65KHRTR-ND" H 5000 5050 50  0001 L CNN "digikey part number"
F 12 "0.5mm" H 5000 5150 50  0001 L CNN "height"
F 13 "RESC15585X45" H 5000 5250 50  0001 L CNN "ipc land pattern name"
F 14 "yes" H 5000 5350 50  0001 L CNN "lead free"
F 15 "ce4140018ac48837" H 5000 5450 50  0001 L CNN "library id"
F 16 "Yageo" H 5000 5550 50  0001 L CNN "manufacturer"
F 17 "Thick Film Resistors - SMD 1.65K OHM 1%\\n" H 5000 5650 50  0001 L CNN "mouser description"
F 18 "603-RC0603FR-071K65L" H 5000 5750 50  0001 L CNN "mouser part number"
F 19 "0603" H 5350 3964 50  0000 C CNN "package"
F 20 "100mW" H 5000 5950 50  0001 L CNN "power"
F 21 "0.1W" H 5000 6050 50  0001 L CNN "power rating"
F 22 "1.65kΩ" H 5350 3873 50  0000 C CNN "resistance"
F 23 "yes" H 5000 6250 50  0001 L CNN "rohs"
F 24 "RC" H 5000 6350 50  0001 L CNN "series"
F 25 "0mm" H 5000 6450 50  0001 L CNN "standoff height"
F 26 "100ppm/°C" H 5000 6550 50  0001 L CNN "temperature coefficient"
F 27 "+155°C" H 5000 6650 50  0001 L CNN "temperature range high"
F 28 "-55°C" H 5000 6750 50  0001 L CNN "temperature range low"
F 29 "1%" H 5000 6850 50  0001 L CNN "tolerance"
F 30 "75V" H 5000 6950 50  0001 L CNN "voltage"
F 31 "75V" H 5000 7050 50  0001 L CNN "voltage rating"
	1    5000 3750
	-1   0    0    -1  
$EndComp
Wire Wire Line
	4900 3750 5000 3750
$Comp
L project:LTST-C190KSKT D?
U 1 1 6045B8D3
P 5500 3750
AR Path="/5FBD86FB/6045B8D3" Ref="D?"  Part="1" 
AR Path="/5FC1E32F/6045B8D3" Ref="D?"  Part="1" 
AR Path="/6045B8D3" Ref="D6"  Part="1" 
F 0 "D6" H 5800 4100 50  0000 C CNN
F 1 "LTST-C190KSKT" H 5800 4000 50  0000 C CNN
F 2 "project:Lite-On-LTST-C190KSKT-MFG" H 5500 4350 50  0001 L CNN
F 3 "https://optoelectronics.liteon.com/upload/download/DS-22-99-0189/LTST-C190KSKT.pdf" H 5500 4450 50  0001 L CNN
F 4 "No" H 5500 4550 50  0001 L CNN "automotive"
F 5 "Diode" H 5500 4650 50  0001 L CNN "category"
F 6 "Yellow" H 6100 3600 50  0000 C CNN "colour"
F 7 "Optoelectronics" H 5500 4850 50  0001 L CNN "device class L1"
F 8 "LEDs" H 5500 4950 50  0001 L CNN "device class L2"
F 9 "unset" H 5500 5050 50  0001 L CNN "device class L3"
F 10 "LED YELLOW CLEAR CHIP SMD" H 5500 5150 50  0001 L CNN "digikey description"
F 11 "160-1437-1-ND" H 5500 5250 50  0001 L CNN "digikey part number"
F 12 "20mA" H 5650 3600 50  0000 C CNN "forward current"
F 13 "2.4V" H 5850 3600 50  0000 C CNN "forward voltage"
F 14 "0.9mm" H 5500 5550 50  0001 L CNN "height"
F 15 "Yes" H 5500 5650 50  0001 L CNN "lead free"
F 16 "Top View" H 5500 5750 50  0001 L CNN "led orientation"
F 17 "d70eef253d640e08" H 5500 5850 50  0001 L CNN "library id"
F 18 "28-180mcd" H 5500 5950 50  0001 L CNN "luminous intensity"
F 19 "Lite-On" H 5500 6050 50  0001 L CNN "manufacturer"
F 20 "859-LTST-C190KSKT" H 5500 6150 50  0001 L CNN "mouser part number"
F 21 "0603" H 5400 3600 50  0000 C CNN "package"
F 22 "588nm" H 5500 6350 50  0001 L CNN "peak emmision wavelength"
F 23 "75mW" H 5500 6450 50  0001 L CNN "power dissipation"
F 24 "5V" H 5500 6550 50  0001 L CNN "reverse voltage"
F 25 "Yes" H 5500 6650 50  0001 L CNN "rohs"
F 26 "+85°C" H 5500 6750 50  0001 L CNN "temperature range high"
F 27 "-30°C" H 5500 6850 50  0001 L CNN "temperature range low"
F 28 "130°" H 5500 6950 50  0001 L CNN "viewing angle"
F 29 "597nm" H 5500 7050 50  0001 L CNN "wavelength"
	1    5500 3750
	-1   0    0    -1  
$EndComp
$Comp
L project:LTST-C190KGKT D?
U 1 1 6046B423
P 5400 3150
AR Path="/5FBD86FB/6046B423" Ref="D?"  Part="1" 
AR Path="/5FC1E32F/6046B423" Ref="D?"  Part="1" 
AR Path="/6046B423" Ref="D5"  Part="1" 
F 0 "D5" H 5600 3500 50  0000 C CNN
F 1 "LTST-C190KGKT" H 5650 3400 50  0000 C CNN
F 2 "project:Lite-On-LTST-C190KGKT-MFG" H 5400 3750 50  0001 L CNN
F 3 "https://optoelectronics.liteon.com/upload/download/DS22-2000-074/LTST-C190KGKT.PDF" H 5400 3850 50  0001 L CNN
F 4 "No" H 5400 3950 50  0001 L CNN "automotive"
F 5 "Diode" H 5400 4050 50  0001 L CNN "category"
F 6 "Green" H 5950 3000 50  0000 C CNN "colour"
F 7 "Optoelectronics" H 5400 4250 50  0001 L CNN "device class L1"
F 8 "LEDs" H 5400 4350 50  0001 L CNN "device class L2"
F 9 "unset" H 5400 4450 50  0001 L CNN "device class L3"
F 10 "LED GREEN CLEAR CHIP SMD" H 5400 4550 50  0001 L CNN "digikey description"
F 11 "160-1435-1-ND" H 5400 4650 50  0001 L CNN "digikey part number"
F 12 "20mA" H 5500 3000 50  0000 C CNN "forward current"
F 13 "2.4V" H 5700 3000 50  0000 C CNN "forward voltage"
F 14 "0.9mm" H 5400 4950 50  0001 L CNN "height"
F 15 "Yes" H 5400 5050 50  0001 L CNN "lead free"
F 16 "Top View" H 5400 5150 50  0001 L CNN "led orientation"
F 17 "4c107ec0dbc12b09" H 5400 5250 50  0001 L CNN "library id"
F 18 "18-71mcd" H 5400 5350 50  0001 L CNN "luminous intensity"
F 19 "Lite-On" H 5400 5450 50  0001 L CNN "manufacturer"
F 20 "859-LTST-C190KGKT" H 5400 5550 50  0001 L CNN "mouser part number"
F 21 "0603" H 5250 3000 50  0000 C CNN "package"
F 22 "574nm" H 5400 5750 50  0001 L CNN "peak emmision wavelength"
F 23 "75mW" H 5400 5850 50  0001 L CNN "power dissipation"
F 24 "5V" H 5400 5950 50  0001 L CNN "reverse voltage"
F 25 "Yes" H 5400 6050 50  0001 L CNN "rohs"
F 26 "+85°C" H 5400 6150 50  0001 L CNN "temperature range high"
F 27 "-30°C" H 5400 6250 50  0001 L CNN "temperature range low"
F 28 "130°" H 5400 6350 50  0001 L CNN "viewing angle"
F 29 "576.5nm" H 5400 6450 50  0001 L CNN "wavelength"
	1    5400 3150
	-1   0    0    -1  
$EndComp
$Comp
L project:RC0603FR-071K65L R?
U 1 1 6046B445
P 5000 3150
AR Path="/5FBD86FB/6046B445" Ref="R?"  Part="1" 
AR Path="/5FC1E32F/6046B445" Ref="R?"  Part="1" 
AR Path="/6046B445" Ref="R8"  Part="1" 
F 0 "R8" H 5350 3455 50  0000 C CNN
F 1 "RC0603FR-071K65L" H 5350 3364 50  0001 C CNN
F 2 "project:Yageo-RC0603-0-0-IPC_A" H 5000 3550 50  0001 L CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RC_Group_51_RoHS_L_9.pdf" H 5000 3650 50  0001 L CNN
F 4 "541-1.00HHCT-ND" H 5000 3750 50  0001 L CNN "approved alternate part number"
F 5 "Res" H 5000 3850 50  0001 L CNN "category"
F 6 "Thick Film" H 5000 3950 50  0001 L CNN "composition"
F 7 "Passive Components" H 5000 4050 50  0001 L CNN "device class L1"
F 8 "Resistors" H 5000 4150 50  0001 L CNN "device class L2"
F 9 "Chip SMD Resistors" H 5000 4250 50  0001 L CNN "device class L3"
F 10 "RES SMD 1.65K OHM 1% 1/10W 0603" H 5000 4350 50  0001 L CNN "digikey description"
F 11 "311-1.65KHRTR-ND" H 5000 4450 50  0001 L CNN "digikey part number"
F 12 "0.5mm" H 5000 4550 50  0001 L CNN "height"
F 13 "RESC15585X45" H 5000 4650 50  0001 L CNN "ipc land pattern name"
F 14 "yes" H 5000 4750 50  0001 L CNN "lead free"
F 15 "ce4140018ac48837" H 5000 4850 50  0001 L CNN "library id"
F 16 "Yageo" H 5000 4950 50  0001 L CNN "manufacturer"
F 17 "Thick Film Resistors - SMD 1.65K OHM 1%\\n" H 5000 5050 50  0001 L CNN "mouser description"
F 18 "603-RC0603FR-071K65L" H 5000 5150 50  0001 L CNN "mouser part number"
F 19 "0603" H 5350 3364 50  0000 C CNN "package"
F 20 "100mW" H 5000 5350 50  0001 L CNN "power"
F 21 "0.1W" H 5000 5450 50  0001 L CNN "power rating"
F 22 "1.65kΩ" H 5350 3273 50  0000 C CNN "resistance"
F 23 "yes" H 5000 5650 50  0001 L CNN "rohs"
F 24 "RC" H 5000 5750 50  0001 L CNN "series"
F 25 "0mm" H 5000 5850 50  0001 L CNN "standoff height"
F 26 "100ppm/°C" H 5000 5950 50  0001 L CNN "temperature coefficient"
F 27 "+155°C" H 5000 6050 50  0001 L CNN "temperature range high"
F 28 "-55°C" H 5000 6150 50  0001 L CNN "temperature range low"
F 29 "1%" H 5000 6250 50  0001 L CNN "tolerance"
F 30 "75V" H 5000 6350 50  0001 L CNN "voltage"
F 31 "75V" H 5000 6450 50  0001 L CNN "voltage rating"
	1    5000 3150
	-1   0    0    -1  
$EndComp
Wire Wire Line
	4900 3150 5000 3150
$Comp
L project:RC0603FR-071K65L R?
U 1 1 6046B46C
P 3800 6800
AR Path="/5FBD86FB/6046B46C" Ref="R?"  Part="1" 
AR Path="/5FC1E32F/6046B46C" Ref="R?"  Part="1" 
AR Path="/6046B46C" Ref="R2"  Part="1" 
F 0 "R2" H 4150 7105 50  0000 C CNN
F 1 "RC0603FR-071K65L" H 4150 7014 50  0001 C CNN
F 2 "project:Yageo-RC0603-0-0-IPC_A" H 3800 7200 50  0001 L CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RC_Group_51_RoHS_L_9.pdf" H 3800 7300 50  0001 L CNN
F 4 "541-1.00HHCT-ND" H 3800 7400 50  0001 L CNN "approved alternate part number"
F 5 "Res" H 3800 7500 50  0001 L CNN "category"
F 6 "Thick Film" H 3800 7600 50  0001 L CNN "composition"
F 7 "Passive Components" H 3800 7700 50  0001 L CNN "device class L1"
F 8 "Resistors" H 3800 7800 50  0001 L CNN "device class L2"
F 9 "Chip SMD Resistors" H 3800 7900 50  0001 L CNN "device class L3"
F 10 "RES SMD 1.65K OHM 1% 1/10W 0603" H 3800 8000 50  0001 L CNN "digikey description"
F 11 "311-1.65KHRTR-ND" H 3800 8100 50  0001 L CNN "digikey part number"
F 12 "0.5mm" H 3800 8200 50  0001 L CNN "height"
F 13 "RESC15585X45" H 3800 8300 50  0001 L CNN "ipc land pattern name"
F 14 "yes" H 3800 8400 50  0001 L CNN "lead free"
F 15 "ce4140018ac48837" H 3800 8500 50  0001 L CNN "library id"
F 16 "Yageo" H 3800 8600 50  0001 L CNN "manufacturer"
F 17 "Thick Film Resistors - SMD 1.65K OHM 1%\\n" H 3800 8700 50  0001 L CNN "mouser description"
F 18 "603-RC0603FR-071K65L" H 3800 8800 50  0001 L CNN "mouser part number"
F 19 "0603" H 4150 7014 50  0000 C CNN "package"
F 20 "100mW" H 3800 9000 50  0001 L CNN "power"
F 21 "0.1W" H 3800 9100 50  0001 L CNN "power rating"
F 22 "1.65kΩ" H 4150 6923 50  0000 C CNN "resistance"
F 23 "yes" H 3800 9300 50  0001 L CNN "rohs"
F 24 "RC" H 3800 9400 50  0001 L CNN "series"
F 25 "0mm" H 3800 9500 50  0001 L CNN "standoff height"
F 26 "100ppm/°C" H 3800 9600 50  0001 L CNN "temperature coefficient"
F 27 "+155°C" H 3800 9700 50  0001 L CNN "temperature range high"
F 28 "-55°C" H 3800 9800 50  0001 L CNN "temperature range low"
F 29 "1%" H 3800 9900 50  0001 L CNN "tolerance"
F 30 "75V" H 3800 10000 50  0001 L CNN "voltage"
F 31 "75V" H 3800 10100 50  0001 L CNN "voltage rating"
	1    3800 6800
	1    0    0    -1  
$EndComp
Wire Wire Line
	3900 6800 3850 6800
$Comp
L project:LTST-C190TBKT D?
U 1 1 6046B494
P 3450 6800
AR Path="/5FBD86FB/6046B494" Ref="D?"  Part="1" 
AR Path="/5FC1E32F/6046B494" Ref="D?"  Part="1" 
AR Path="/6046B494" Ref="D3"  Part="1" 
F 0 "D3" H 3650 7350 50  0000 C CNN
F 1 "LTST-C190TBKT" H 3650 7250 50  0000 C CNN
F 2 "project:Vishay_Lite-On-LTST-C190TBKT-Manufacturer_Recommended" H 3450 7400 50  0001 L CNN
F 3 "http://optoelectronics.liteon.com/upload/download/DS-22-99-0224/S_110_LTST-C190TBKT(0630).pdf" H 3450 7500 50  0001 L CNN
F 4 "DFN" H 3450 7600 50  0001 L CNN "Case Package"
F 5 "Manufacturer URL" H 3450 7700 50  0001 L CNN "Component Link 1 Description"
F 6 "http://www.us.liteon.com/" H 3450 7800 50  0001 L CNN "Component Link 1 URL"
F 7 "Rev. K, 08/2011" H 3450 7900 50  0001 L CNN "Datasheet Version"
F 8 "1.1 mm" H 3450 8000 50  0001 L CNN "Height"
F 9 "Serial" H 3450 8100 50  0001 L CNN "Interface"
F 10 "3 mm" H 3450 8200 50  0001 L CNN "Length"
F 11 "125 degC" H 3450 8300 50  0001 L CNN "Max Operating Temperature"
F 12 "3.6 V" H 3450 8400 50  0001 L CNN "Max Supply Voltage"
F 13 "-40 degC" H 3450 8500 50  0001 L CNN "Min Operating Temperature"
F 14 "2.1 V" H 3450 8600 50  0001 L CNN "Min Supply Voltage"
F 15 "Surface Mount" H 3450 8700 50  0001 L CNN "Mount"
F 16 "3.3V" H 3650 7150 50  0000 C CNN "Nominal Supply Voltage"
F 17 "2-Pin Surface Mount Device, Body 1.6 x 0.8 mm" H 3450 8900 50  0001 L CNN "Package Description"
F 18 "Cut Tape" H 3450 9000 50  0001 L CNN "Packaging"
F 19 "6" H 3450 9100 50  0001 L CNN "Pins"
F 20 "No SVHC" H 3450 9200 50  0001 L CNN "REACH SVHC"
F 21 "true" H 3450 9300 50  0001 L CNN "Ro HSCompliant"
F 22 "3 mm" H 3450 9400 50  0001 L CNN "Width"
F 23 "Disp" H 3450 9500 50  0001 L CNN "category"
F 24 "1545016" H 3450 9600 50  0001 L CNN "ciiva ids"
F 25 "01d263fa86e1fb49" H 3450 9700 50  0001 L CNN "library id"
F 26 "Vishay Lite-On" H 3450 9800 50  0001 L CNN "manufacturer"
F 27 "0603" H 3650 7050 50  0000 C CNN "package"
F 28 "1475043740" H 3450 10000 50  0001 L CNN "release date"
F 29 "6CBC32C1-F554-43EB-9EEA-D4C78589E111" H 3450 10100 50  0001 L CNN "vault revision"
F 30 "yes" H 3450 10200 50  0001 L CNN "imported"
F 31 "Blue" H 3450 7150 50  0000 C CNN "Color"
F 32 "20mA" H 3850 7150 50  0000 C CNN "Current"
	1    3450 6800
	1    0    0    -1  
$EndComp
Wire Wire Line
	5600 3150 5400 3150
$Comp
L power:VBUS #PWR033
U 1 1 605A189C
P 5600 3100
F 0 "#PWR033" H 5600 2950 50  0001 C CNN
F 1 "VBUS" H 5615 3273 50  0000 C CNN
F 2 "" H 5600 3100 50  0001 C CNN
F 3 "" H 5600 3100 50  0001 C CNN
	1    5600 3100
	1    0    0    -1  
$EndComp
Wire Wire Line
	5600 3100 5600 3150
Text Label 4000 3150 0    50   ~ 0
RXLED#
Text Label 4000 3250 0    50   ~ 0
TXLED#
Text Label 4000 2150 0    50   ~ 0
TXD
Text Label 4000 2250 0    50   ~ 0
RXD
Text Label 4000 2550 0    50   ~ 0
DTRD
Wire Wire Line
	4400 6800 4450 6800
$Comp
L power:GND #PWR021
U 1 1 605EC326
P 4450 6800
F 0 "#PWR021" H 4450 6550 50  0001 C CNN
F 1 "GND" H 4455 6627 50  0000 C CNN
F 2 "" H 4450 6800 50  0001 C CNN
F 3 "" H 4450 6800 50  0001 C CNN
	1    4450 6800
	0    -1   -1   0   
$EndComp
Connection ~ 3500 1100
Wire Wire Line
	3500 1100 3500 1200
$Comp
L project:CC0603KPX7R9BB104 C?
U 1 1 6074652A
P 3500 1200
AR Path="/5FC1E32F/6074652A" Ref="C?"  Part="1" 
AR Path="/5FBD86FB/6074652A" Ref="C?"  Part="1" 
AR Path="/6074652A" Ref="C7"  Part="1" 
F 0 "C7" H 3628 1237 50  0000 L CNN
F 1 "CC0603KPX7R9BB104" H 3500 1400 50  0001 L CNN
F 2 "project:YAGEO-CC0603-0-0-0" H 3500 1500 50  0001 L CNN
F 3 "http://www.yageo.com/documents/recent/UPY-GPHC_X7R_6.3V-to-50V_18.pdf" H 3500 1600 50  0001 L CNN
F 4 "No" H 3500 1700 50  0001 L CNN "automotive"
F 5 "100 nF" H 3628 1146 50  0000 L CNN "capacitance"
F 6 "Cap" H 3500 1900 50  0001 L CNN "category"
F 7 "Passive Components" H 3500 2000 50  0001 L CNN "device class L1"
F 8 "Capacitors" H 3500 2100 50  0001 L CNN "device class L2"
F 9 "Ceramic Capacitors" H 3500 2200 50  0001 L CNN "device class L3"
F 10 "CAP CER 0.1UF 50V X7R 0603" H 3500 2300 50  0001 L CNN "digikey description"
F 11 "CC0603KPX7R9BB104-ND" H 3500 2400 50  0001 L CNN "digikey part number"
F 12 "http://www.yageo.com/exep/pages/download/literatures/UPY-C_GEN_24.pdf" H 3500 2500 50  0001 L CNN "footprint url"
F 13 "0.9mm" H 3500 2600 50  0001 L CNN "height"
F 14 "Yes" H 3500 2700 50  0001 L CNN "lead free"
F 15 "7cd692b09ff39865" H 3500 2800 50  0001 L CNN "library id"
F 16 "YAGEO" H 3500 2900 50  0001 L CNN "manufacturer"
F 17 "Ceramic" H 3500 3000 50  0001 L CNN "material"
F 18 "Multilayer Ceramic Capacitors MLCC - SMD/SMT 100nF 50V X7R 10%" H 3500 3100 50  0001 L CNN "mouser description"
F 19 "603-CC603KPX7R9BB104" H 3500 3200 50  0001 L CNN "mouser part number"
F 20 "0603" H 3628 1055 50  0000 L CNN "package"
F 21 "Yes" H 3500 3400 50  0001 L CNN "rohs"
F 22 "X7R" H 3500 3500 50  0001 L CNN "temperature characteristic"
F 23 "15%" H 3500 3600 50  0001 L CNN "temperature coefficient"
F 24 "+125°C" H 3500 3700 50  0001 L CNN "temperature range high"
F 25 "-55°C" H 3500 3800 50  0001 L CNN "temperature range low"
F 26 "0.1" H 3500 3900 50  0001 L CNN "tolerance"
F 27 "50 V" H 3628 964 50  0000 L CNN "voltage"
F 28 "50 V" H 3500 4100 50  0001 L CNN "voltage rating"
	1    3500 1200
	1    0    0    -1  
$EndComp
Wire Wire Line
	3500 1400 3500 1450
$Comp
L power:GND #PWR018
U 1 1 60746531
P 3500 1450
F 0 "#PWR018" H 3500 1200 50  0001 C CNN
F 1 "GND" H 3505 1277 50  0000 C CNN
F 2 "" H 3500 1450 50  0001 C CNN
F 3 "" H 3500 1450 50  0001 C CNN
	1    3500 1450
	1    0    0    -1  
$EndComp
Wire Wire Line
	2850 1400 2850 1450
$Comp
L power:GND #PWR015
U 1 1 6076421D
P 2850 1450
F 0 "#PWR015" H 2850 1200 50  0001 C CNN
F 1 "GND" H 2855 1277 50  0000 C CNN
F 2 "" H 2850 1450 50  0001 C CNN
F 3 "" H 2850 1450 50  0001 C CNN
	1    2850 1450
	1    0    0    -1  
$EndComp
$Comp
L project:CC0805ZKY5V6BB106 C?
U 1 1 60764231
P 2850 1200
AR Path="/5FC1E32F/60764231" Ref="C?"  Part="1" 
AR Path="/60764231" Ref="C5"  Part="1" 
F 0 "C5" H 2978 1237 50  0000 L CNN
F 1 "CC0805ZKY5V6BB106" H 2850 1400 50  0001 L CNN
F 2 "project:YAGEO-CC0805-0-0-0" H 2850 1500 50  0001 L CNN
F 3 "http://www.yageo.com/documents/recent/UPY-GPHC_Y5V_6.3V-to-50V_8.pdf" H 2850 1600 50  0001 L CNN
F 4 "10uF" H 2978 1146 50  0000 L CNN "capacitance"
F 5 "Cap" H 2850 1800 50  0001 L CNN "category"
F 6 "CAP CER 10UF 10V Y5V 0805" H 2850 1900 50  0001 L CNN "digikey description"
F 7 "311-1355-1-ND" H 2850 2000 50  0001 L CNN "digikey part number"
F 8 "yes" H 2850 2100 50  0001 L CNN "lead free"
F 9 "852e7d0cf36849be" H 2850 2200 50  0001 L CNN "library id"
F 10 "YAGEO" H 2850 2300 50  0001 L CNN "manufacturer"
F 11 "Multilayer Ceramic Capacitors MLCC - SMD/SMT 10uF 10V Y5V-20/+80%" H 2850 2400 50  0001 L CNN "mouser description"
F 12 "603-CC805ZKY5V6BB106" H 2850 2500 50  0001 L CNN "mouser part number"
F 13 "0805" H 2978 1055 50  0000 L CNN "package"
F 14 "yes" H 2850 2700 50  0001 L CNN "rohs"
F 15 "+85°C" H 2850 2800 50  0001 L CNN "temperature range high"
F 16 "-30°C" H 2850 2900 50  0001 L CNN "temperature range low"
F 17 "10V" H 2978 964 50  0000 L CNN "voltage"
	1    2850 1200
	1    0    0    -1  
$EndComp
Wire Wire Line
	2850 1200 2850 1100
$Comp
L power:+5V #PWR02
U 1 1 60108B7B
P 750 7800
F 0 "#PWR02" H 750 7650 50  0001 C CNN
F 1 "+5V" H 765 7973 50  0000 C CNN
F 2 "" H 750 7800 50  0001 C CNN
F 3 "" H 750 7800 50  0001 C CNN
	1    750  7800
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5300 6500 5050 6500
$Comp
L power:GND #PWR024
U 1 1 6088431E
P 5050 6500
F 0 "#PWR024" H 5050 6250 50  0001 C CNN
F 1 "GND" V 5055 6372 50  0000 R CNN
F 2 "" H 5050 6500 50  0001 C CNN
F 3 "" H 5050 6500 50  0001 C CNN
	1    5050 6500
	0    1    1    0   
$EndComp
Wire Wire Line
	5300 6600 5000 6600
Wire Wire Line
	5300 6700 5000 6700
Wire Wire Line
	5300 6800 5000 6800
$Comp
L power:+5V #PWR023
U 1 1 60884327
P 5000 6600
F 0 "#PWR023" H 5000 6450 50  0001 C CNN
F 1 "+5V" V 5015 6728 50  0000 L CNN
F 2 "" H 5000 6600 50  0001 C CNN
F 3 "" H 5000 6600 50  0001 C CNN
	1    5000 6600
	0    -1   -1   0   
$EndComp
Text Label 5050 6700 0    50   ~ 0
SCL
Text Label 5050 6800 0    50   ~ 0
SDA
Text Notes 4850 7050 0    50   ~ 0
Socket for I2C Display
$Comp
L project:FSM2JSMAA SW1
U 1 1 608B5173
P 4550 1000
F 0 "SW1" V 4154 800 50  0000 C CNN
F 1 "FSM2JSMAA" V 4245 800 50  0000 C CNN
F 2 "project:TE_Connectivity-FSM2JSMAA-MFG" H 4550 1200 50  0001 L CNN
F 3 "https://www.te.com/commerce/DocumentDelivery/DDEController?Action=showdoc&DocId=Customer+Drawing%7F2-1437565-7%7FV%7Fpdf%7FEnglish%7FENG_CD_2-1437565-7_V.pdf%7F2-1437565-8" H 4550 1300 50  0001 L CNN
F 4 "No" H 4550 1400 50  0001 L CNN "automotive"
F 5 "Switch" H 4550 1500 50  0001 L CNN "category"
F 6 "50mA" H 4550 1600 50  0001 L CNN "contact current rating"
F 7 "100mΩ" H 4550 1700 50  0001 L CNN "contact resistance"
F 8 "Electromechanical" H 4550 1800 50  0001 L CNN "device class L1"
F 9 "Switches" H 4550 1900 50  0001 L CNN "device class L2"
F 10 "Pushbutton Switches" H 4550 2000 50  0001 L CNN "device class L3"
F 11 "SWITCH TACTILE SPST-NO 0.05A 24V" H 4550 2100 50  0001 L CNN "digikey description"
F 12 "450-3355-ND" H 4550 2200 50  0001 L CNN "digikey part number"
F 13 "100000Cycles" H 4550 2300 50  0001 L CNN "electromechanical life"
F 14 "4.59mm" H 4550 2400 50  0001 L CNN "height"
F 15 "Yes" H 4550 2500 50  0001 L CNN "lead free"
F 16 "7458e20eae944245" H 4550 2600 50  0001 L CNN "library id"
F 17 "TE Connectivity" H 4550 2700 50  0001 L CNN "manufacturer"
F 18 "Surface Mount" H 4550 2800 50  0001 L CNN "mount"
F 19 "Tactile Switch 260G" V 4336 800 50  0000 C CNN "mouser description"
F 20 "506-2-1437565-8" H 4550 3000 50  0001 L CNN "mouser part number"
F 21 "260gf" H 4550 3100 50  0001 L CNN "operating force"
F 22 "SMT_SW_6MM00_6MM00" V 4427 800 50  0000 C CNN "package"
F 23 "Yes" H 4550 3300 50  0001 L CNN "rohs"
F 24 "0.05mm" H 4550 3400 50  0001 L CNN "standoff height"
F 25 "+85°C" H 4550 3500 50  0001 L CNN "temperature range high"
F 26 "-35°C" H 4550 3600 50  0001 L CNN "temperature range low"
F 27 "SPST-NO" H 4550 3700 50  0001 L CNN "throw configuration"
F 28 "24V" H 4550 3800 50  0001 L CNN "voltage rating DC"
	1    4550 1000
	0    1    1    0   
$EndComp
Wire Wire Line
	4250 1000 4000 1000
NoConn ~ 4450 1000
NoConn ~ 4450 1400
Wire Wire Line
	4250 1400 4200 1400
Wire Wire Line
	4200 1400 4200 1450
$Comp
L power:GND #PWR020
U 1 1 608F6F28
P 4200 1450
F 0 "#PWR020" H 4200 1200 50  0001 C CNN
F 1 "GND" H 4205 1277 50  0000 C CNN
F 2 "" H 4200 1450 50  0001 C CNN
F 3 "" H 4200 1450 50  0001 C CNN
	1    4200 1450
	1    0    0    -1  
$EndComp
Text Label 4050 1000 0    50   ~ 0
RES
$Comp
L project:FSM2JSMAA SW2
U 1 1 6091A4AB
P 5500 1000
F 0 "SW2" V 5104 800 50  0000 C CNN
F 1 "FSM2JSMAA" V 5195 800 50  0000 C CNN
F 2 "project:TE_Connectivity-FSM2JSMAA-MFG" H 5500 1200 50  0001 L CNN
F 3 "https://www.te.com/commerce/DocumentDelivery/DDEController?Action=showdoc&DocId=Customer+Drawing%7F2-1437565-7%7FV%7Fpdf%7FEnglish%7FENG_CD_2-1437565-7_V.pdf%7F2-1437565-8" H 5500 1300 50  0001 L CNN
F 4 "No" H 5500 1400 50  0001 L CNN "automotive"
F 5 "Switch" H 5500 1500 50  0001 L CNN "category"
F 6 "50mA" H 5500 1600 50  0001 L CNN "contact current rating"
F 7 "100mΩ" H 5500 1700 50  0001 L CNN "contact resistance"
F 8 "Electromechanical" H 5500 1800 50  0001 L CNN "device class L1"
F 9 "Switches" H 5500 1900 50  0001 L CNN "device class L2"
F 10 "Pushbutton Switches" H 5500 2000 50  0001 L CNN "device class L3"
F 11 "SWITCH TACTILE SPST-NO 0.05A 24V" H 5500 2100 50  0001 L CNN "digikey description"
F 12 "450-3355-ND" H 5500 2200 50  0001 L CNN "digikey part number"
F 13 "100000Cycles" H 5500 2300 50  0001 L CNN "electromechanical life"
F 14 "4.59mm" H 5500 2400 50  0001 L CNN "height"
F 15 "Yes" H 5500 2500 50  0001 L CNN "lead free"
F 16 "7458e20eae944245" H 5500 2600 50  0001 L CNN "library id"
F 17 "TE Connectivity" H 5500 2700 50  0001 L CNN "manufacturer"
F 18 "Surface Mount" H 5500 2800 50  0001 L CNN "mount"
F 19 "Tactile Switch 260G" V 5286 800 50  0000 C CNN "mouser description"
F 20 "506-2-1437565-8" H 5500 3000 50  0001 L CNN "mouser part number"
F 21 "260gf" H 5500 3100 50  0001 L CNN "operating force"
F 22 "SMT_SW_6MM00_6MM00" V 5377 800 50  0000 C CNN "package"
F 23 "Yes" H 5500 3300 50  0001 L CNN "rohs"
F 24 "0.05mm" H 5500 3400 50  0001 L CNN "standoff height"
F 25 "+85°C" H 5500 3500 50  0001 L CNN "temperature range high"
F 26 "-35°C" H 5500 3600 50  0001 L CNN "temperature range low"
F 27 "SPST-NO" H 5500 3700 50  0001 L CNN "throw configuration"
F 28 "24V" H 5500 3800 50  0001 L CNN "voltage rating DC"
	1    5500 1000
	0    1    1    0   
$EndComp
Wire Wire Line
	5200 1000 4950 1000
NoConn ~ 5400 1000
NoConn ~ 5400 1400
Wire Wire Line
	5200 1400 5150 1400
Wire Wire Line
	5150 1400 5150 1450
$Comp
L power:GND #PWR029
U 1 1 6091A4B6
P 5150 1450
F 0 "#PWR029" H 5150 1200 50  0001 C CNN
F 1 "GND" H 5155 1277 50  0000 C CNN
F 2 "" H 5150 1450 50  0001 C CNN
F 3 "" H 5150 1450 50  0001 C CNN
	1    5150 1450
	1    0    0    -1  
$EndComp
Text Label 5000 1000 0    50   ~ 0
IRQ
Text Notes 4000 1750 0    50   ~ 0
Button Reset
Text Notes 4800 1750 0    50   ~ 0
Button Interrupt Request
Text Notes 1000 3350 0    50   ~ 0
Micro USB Port
Text Notes 1350 7950 0    50   ~ 0
In Circuit Serial Programing
Text Notes 2900 3950 0    50   ~ 0
UART to USB
Text Notes 2000 7250 0    50   ~ 0
Host MCU
$Comp
L project:SSW-104-01-T-S J4
U 1 1 60D6DB05
P 5200 6500
F 0 "J4" V 4950 6800 50  0000 L CNN
F 1 "SSW-104-01-T-S" V 5050 6150 50  0000 L CNN
F 2 "project:Samtec-SSW-104-01-T-S-Manufacturer_Recommended" H 5200 7200 50  0001 L CNN
F 3 "http://www.samtec.com/documents/webfiles/pdf/SLW.PDF" H 5200 7300 50  0001 L CNN
F 4 "Manufacturer URL" H 5200 7400 50  0001 L CNN "Component Link 1 Description"
F 5 "http://www.samtec.com" H 5200 7500 50  0001 L CNN "Component Link 1 URL"
F 6 "Package Specification" H 5200 7600 50  0001 L CNN "Component Link 3 Description"
F 7 "http://www.samtec.com/documents/webfiles/cpdf/SLW-1XX-01-X-X-MKT.pdf" H 5200 7700 50  0001 L CNN "Component Link 3 URL"
F 8 "F-214" H 5200 7800 50  0001 L CNN "Datasheet Version"
F 9 "Through-hole" H 5200 7900 50  0001 L CNN "Mounting Technology"
F 10 "Vertical" H 5200 8000 50  0001 L CNN "Orientation"
F 11 "Low Profile Socket Strip" H 5200 8100 50  0001 L CNN "Package Description"
F 12 "X, 9/1987" H 5200 8200 50  0001 L CNN "Package Version"
F 13 "2.54 mm" H 5200 8300 50  0001 L CNN "Pitch"
F 14 "-55 to 105 degC" H 5200 8400 50  0001 L CNN "Temperature Range"
F 15 "Conn" H 5200 8500 50  0001 L CNN "category"
F 16 "349254" H 5200 8600 50  0001 L CNN "ciiva ids"
F 17 "e244508a65ae41fc" H 5200 8700 50  0001 L CNN "library id"
F 18 "Samtec" H 5200 8800 50  0001 L CNN "manufacturer"
F 19 "SSW-104-01-X-S" H 5200 8900 50  0001 L CNN "package"
F 20 "1404374325" H 5200 9000 50  0001 L CNN "release date"
F 21 "3E9B3488-2A9B-444C-BFB7-BA4271584146" H 5200 9100 50  0001 L CNN "vault revision"
F 22 "yes" H 5200 9200 50  0001 L CNN "imported"
	1    5200 6500
	0    1    1    0   
$EndComp
Wire Wire Line
	800  7800 750  7800
Text Label 3100 6100 0    50   ~ 0
RED
$Comp
L project:RC0603FR-0710KL R14
U 1 1 6129E034
P 6850 700
F 0 "R14" H 7200 1005 50  0000 C CNN
F 1 "RC0603FR-0710KL" H 6850 1000 50  0001 L CNN
F 2 "project:Yageo-RC0603-0-0-IPC_A" H 6850 1100 50  0001 L CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RC_Group_51_RoHS_L_9.pdf" H 6850 1200 50  0001 L CNN
F 4 "541-1.00HHCT-ND" H 6850 1300 50  0001 L CNN "approved alternate part number"
F 5 "Res" H 6850 1400 50  0001 L CNN "category"
F 6 "Thick Film" H 6850 1500 50  0001 L CNN "composition"
F 7 "Passive Components" H 6850 1600 50  0001 L CNN "device class L1"
F 8 "Resistors" H 6850 1700 50  0001 L CNN "device class L2"
F 9 "Chip SMD Resistors" H 6850 1800 50  0001 L CNN "device class L3"
F 10 "RES SMD 10K OHM 1% 1/10W 0603" H 6850 1900 50  0001 L CNN "digikey description"
F 11 "311-10.0KHRTR-ND" H 6850 2000 50  0001 L CNN "digikey part number"
F 12 "0.5mm" H 6850 2100 50  0001 L CNN "height"
F 13 "RESC15585X45" H 6850 2200 50  0001 L CNN "ipc land pattern name"
F 14 "yes" H 6850 2300 50  0001 L CNN "lead free"
F 15 "368bdb3a487a5469" H 6850 2400 50  0001 L CNN "library id"
F 16 "Yageo" H 6850 2500 50  0001 L CNN "manufacturer"
F 17 "Surface Mount Thick Film Resistor, RC Series, 10 kohm, 100 mW, - 1%, 50 V, 0603 [1608 Metric]" H 6850 2600 50  0001 L CNN "mouser description"
F 18 "603-RC0603FR-0710KL" H 6850 2700 50  0001 L CNN "mouser part number"
F 19 "0603" H 7200 914 50  0000 C CNN "package"
F 20 "100mW" H 6850 2900 50  0001 L CNN "power"
F 21 "0.1W" H 6850 3000 50  0001 L CNN "power rating"
F 22 "10kΩ" H 7200 823 50  0000 C CNN "resistance"
F 23 "yes" H 6850 3200 50  0001 L CNN "rohs"
F 24 "RC" H 6850 3300 50  0001 L CNN "series"
F 25 "0mm" H 6850 3400 50  0001 L CNN "standoff height"
F 26 "100ppm/°C" H 6850 3500 50  0001 L CNN "temperature coefficient"
F 27 "+155°C" H 6850 3600 50  0001 L CNN "temperature range high"
F 28 "-55°C" H 6850 3700 50  0001 L CNN "temperature range low"
F 29 "1%" H 6850 3800 50  0001 L CNN "tolerance"
F 30 "75V" H 6850 3900 50  0001 L CNN "voltage"
F 31 "75V" H 6850 4000 50  0001 L CNN "voltage rating"
	1    6850 700 
	0    1    1    0   
$EndComp
Wire Wire Line
	6850 1300 6850 1350
Wire Wire Line
	6850 1350 6650 1350
Wire Wire Line
	6050 1300 6050 1350
Wire Wire Line
	6050 1350 6250 1350
$Comp
L power:+5VP #PWR041
U 1 1 612C9B3A
P 6850 750
F 0 "#PWR041" H 6850 600 50  0001 C CNN
F 1 "+5VP" H 6700 900 50  0000 L CNN
F 2 "" H 6850 750 50  0001 C CNN
F 3 "" H 6850 750 50  0001 C CNN
	1    6850 750 
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR037
U 1 1 612C9E4F
P 6050 750
F 0 "#PWR037" H 6050 600 50  0001 C CNN
F 1 "+5V" H 6065 923 50  0000 C CNN
F 2 "" H 6050 750 50  0001 C CNN
F 3 "" H 6050 750 50  0001 C CNN
	1    6050 750 
	1    0    0    -1  
$EndComp
Wire Wire Line
	6050 750  6050 800 
Wire Wire Line
	6850 750  6850 800 
Wire Wire Line
	6450 1050 6450 800 
Text Label 6050 1350 0    50   ~ 0
SDA
Text Label 6650 1350 0    50   ~ 0
SDAP
$Comp
L project:RC0603FR-0710KL R15
U 1 1 614310CA
P 6850 1550
F 0 "R15" H 7200 1855 50  0000 C CNN
F 1 "RC0603FR-0710KL" H 6850 1850 50  0001 L CNN
F 2 "project:Yageo-RC0603-0-0-IPC_A" H 6850 1950 50  0001 L CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RC_Group_51_RoHS_L_9.pdf" H 6850 2050 50  0001 L CNN
F 4 "541-1.00HHCT-ND" H 6850 2150 50  0001 L CNN "approved alternate part number"
F 5 "Res" H 6850 2250 50  0001 L CNN "category"
F 6 "Thick Film" H 6850 2350 50  0001 L CNN "composition"
F 7 "Passive Components" H 6850 2450 50  0001 L CNN "device class L1"
F 8 "Resistors" H 6850 2550 50  0001 L CNN "device class L2"
F 9 "Chip SMD Resistors" H 6850 2650 50  0001 L CNN "device class L3"
F 10 "RES SMD 10K OHM 1% 1/10W 0603" H 6850 2750 50  0001 L CNN "digikey description"
F 11 "311-10.0KHRTR-ND" H 6850 2850 50  0001 L CNN "digikey part number"
F 12 "0.5mm" H 6850 2950 50  0001 L CNN "height"
F 13 "RESC15585X45" H 6850 3050 50  0001 L CNN "ipc land pattern name"
F 14 "yes" H 6850 3150 50  0001 L CNN "lead free"
F 15 "368bdb3a487a5469" H 6850 3250 50  0001 L CNN "library id"
F 16 "Yageo" H 6850 3350 50  0001 L CNN "manufacturer"
F 17 "Surface Mount Thick Film Resistor, RC Series, 10 kohm, 100 mW, - 1%, 50 V, 0603 [1608 Metric]" H 6850 3450 50  0001 L CNN "mouser description"
F 18 "603-RC0603FR-0710KL" H 6850 3550 50  0001 L CNN "mouser part number"
F 19 "0603" H 7200 1764 50  0000 C CNN "package"
F 20 "100mW" H 6850 3750 50  0001 L CNN "power"
F 21 "0.1W" H 6850 3850 50  0001 L CNN "power rating"
F 22 "10kΩ" H 7200 1673 50  0000 C CNN "resistance"
F 23 "yes" H 6850 4050 50  0001 L CNN "rohs"
F 24 "RC" H 6850 4150 50  0001 L CNN "series"
F 25 "0mm" H 6850 4250 50  0001 L CNN "standoff height"
F 26 "100ppm/°C" H 6850 4350 50  0001 L CNN "temperature coefficient"
F 27 "+155°C" H 6850 4450 50  0001 L CNN "temperature range high"
F 28 "-55°C" H 6850 4550 50  0001 L CNN "temperature range low"
F 29 "1%" H 6850 4650 50  0001 L CNN "tolerance"
F 30 "75V" H 6850 4750 50  0001 L CNN "voltage"
F 31 "75V" H 6850 4850 50  0001 L CNN "voltage rating"
	1    6850 1550
	0    1    1    0   
$EndComp
Wire Wire Line
	6850 2150 6850 2200
Wire Wire Line
	6850 2200 6650 2200
Wire Wire Line
	6050 2150 6050 2200
Wire Wire Line
	6050 2200 6250 2200
$Comp
L power:+5VP #PWR042
U 1 1 614310D4
P 6850 1600
F 0 "#PWR042" H 6850 1450 50  0001 C CNN
F 1 "+5VP" H 6700 1750 50  0000 L CNN
F 2 "" H 6850 1600 50  0001 C CNN
F 3 "" H 6850 1600 50  0001 C CNN
	1    6850 1600
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR038
U 1 1 614310DA
P 6050 1600
F 0 "#PWR038" H 6050 1450 50  0001 C CNN
F 1 "+5V" H 6065 1773 50  0000 C CNN
F 2 "" H 6050 1600 50  0001 C CNN
F 3 "" H 6050 1600 50  0001 C CNN
	1    6050 1600
	1    0    0    -1  
$EndComp
Wire Wire Line
	6050 1600 6050 1650
Wire Wire Line
	6850 1600 6850 1650
Wire Wire Line
	6450 1900 6450 1650
Text Label 6050 2200 0    50   ~ 0
SCL
Text Label 6650 2200 0    50   ~ 0
SCLP
Wire Wire Line
	1300 7800 1700 7800
Wire Wire Line
	2200 7800 2550 7800
Wire Wire Line
	6450 1650 6850 1650
Connection ~ 6850 1650
Wire Wire Line
	6450 800  6850 800 
Connection ~ 6850 800 
$Comp
L power:VBUS #PWR034
U 1 1 5FCCB70B
P 5650 3700
F 0 "#PWR034" H 5650 3550 50  0001 C CNN
F 1 "VBUS" H 5665 3873 50  0000 C CNN
F 2 "" H 5650 3700 50  0001 C CNN
F 3 "" H 5650 3700 50  0001 C CNN
	1    5650 3700
	-1   0    0    -1  
$EndComp
Wire Wire Line
	5400 3750 5650 3750
Wire Wire Line
	5650 3750 5650 3700
$Comp
L project:RC0603FR-0710KL R10
U 1 1 5FD87EB2
P 6050 2450
F 0 "R10" H 6400 2755 50  0000 C CNN
F 1 "RC0603FR-0710KL" H 6050 2750 50  0001 L CNN
F 2 "project:Yageo-RC0603-0-0-IPC_A" H 6050 2850 50  0001 L CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RC_Group_51_RoHS_L_9.pdf" H 6050 2950 50  0001 L CNN
F 4 "541-1.00HHCT-ND" H 6050 3050 50  0001 L CNN "approved alternate part number"
F 5 "Res" H 6050 3150 50  0001 L CNN "category"
F 6 "Thick Film" H 6050 3250 50  0001 L CNN "composition"
F 7 "Passive Components" H 6050 3350 50  0001 L CNN "device class L1"
F 8 "Resistors" H 6050 3450 50  0001 L CNN "device class L2"
F 9 "Chip SMD Resistors" H 6050 3550 50  0001 L CNN "device class L3"
F 10 "RES SMD 10K OHM 1% 1/10W 0603" H 6050 3650 50  0001 L CNN "digikey description"
F 11 "311-10.0KHRTR-ND" H 6050 3750 50  0001 L CNN "digikey part number"
F 12 "0.5mm" H 6050 3850 50  0001 L CNN "height"
F 13 "RESC15585X45" H 6050 3950 50  0001 L CNN "ipc land pattern name"
F 14 "yes" H 6050 4050 50  0001 L CNN "lead free"
F 15 "368bdb3a487a5469" H 6050 4150 50  0001 L CNN "library id"
F 16 "Yageo" H 6050 4250 50  0001 L CNN "manufacturer"
F 17 "Surface Mount Thick Film Resistor, RC Series, 10 kohm, 100 mW, - 1%, 50 V, 0603 [1608 Metric]" H 6050 4350 50  0001 L CNN "mouser description"
F 18 "603-RC0603FR-0710KL" H 6050 4450 50  0001 L CNN "mouser part number"
F 19 "0603" H 6400 2664 50  0000 C CNN "package"
F 20 "100mW" H 6050 4650 50  0001 L CNN "power"
F 21 "0.1W" H 6050 4750 50  0001 L CNN "power rating"
F 22 "10kΩ" H 6400 2573 50  0000 C CNN "resistance"
F 23 "yes" H 6050 4950 50  0001 L CNN "rohs"
F 24 "RC" H 6050 5050 50  0001 L CNN "series"
F 25 "0mm" H 6050 5150 50  0001 L CNN "standoff height"
F 26 "100ppm/°C" H 6050 5250 50  0001 L CNN "temperature coefficient"
F 27 "+155°C" H 6050 5350 50  0001 L CNN "temperature range high"
F 28 "-55°C" H 6050 5450 50  0001 L CNN "temperature range low"
F 29 "1%" H 6050 5550 50  0001 L CNN "tolerance"
F 30 "75V" H 6050 5650 50  0001 L CNN "voltage"
F 31 "75V" H 6050 5750 50  0001 L CNN "voltage rating"
	1    6050 2450
	0    1    1    0   
$EndComp
$Comp
L project:RC0603FR-0710KL R13
U 1 1 5FD87ED4
P 6850 2450
F 0 "R13" H 7200 2755 50  0000 C CNN
F 1 "RC0603FR-0710KL" H 6850 2750 50  0001 L CNN
F 2 "project:Yageo-RC0603-0-0-IPC_A" H 6850 2850 50  0001 L CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RC_Group_51_RoHS_L_9.pdf" H 6850 2950 50  0001 L CNN
F 4 "541-1.00HHCT-ND" H 6850 3050 50  0001 L CNN "approved alternate part number"
F 5 "Res" H 6850 3150 50  0001 L CNN "category"
F 6 "Thick Film" H 6850 3250 50  0001 L CNN "composition"
F 7 "Passive Components" H 6850 3350 50  0001 L CNN "device class L1"
F 8 "Resistors" H 6850 3450 50  0001 L CNN "device class L2"
F 9 "Chip SMD Resistors" H 6850 3550 50  0001 L CNN "device class L3"
F 10 "RES SMD 10K OHM 1% 1/10W 0603" H 6850 3650 50  0001 L CNN "digikey description"
F 11 "311-10.0KHRTR-ND" H 6850 3750 50  0001 L CNN "digikey part number"
F 12 "0.5mm" H 6850 3850 50  0001 L CNN "height"
F 13 "RESC15585X45" H 6850 3950 50  0001 L CNN "ipc land pattern name"
F 14 "yes" H 6850 4050 50  0001 L CNN "lead free"
F 15 "368bdb3a487a5469" H 6850 4150 50  0001 L CNN "library id"
F 16 "Yageo" H 6850 4250 50  0001 L CNN "manufacturer"
F 17 "Surface Mount Thick Film Resistor, RC Series, 10 kohm, 100 mW, - 1%, 50 V, 0603 [1608 Metric]" H 6850 4350 50  0001 L CNN "mouser description"
F 18 "603-RC0603FR-0710KL" H 6850 4450 50  0001 L CNN "mouser part number"
F 19 "0603" H 7200 2664 50  0000 C CNN "package"
F 20 "100mW" H 6850 4650 50  0001 L CNN "power"
F 21 "0.1W" H 6850 4750 50  0001 L CNN "power rating"
F 22 "10kΩ" H 7200 2573 50  0000 C CNN "resistance"
F 23 "yes" H 6850 4950 50  0001 L CNN "rohs"
F 24 "RC" H 6850 5050 50  0001 L CNN "series"
F 25 "0mm" H 6850 5150 50  0001 L CNN "standoff height"
F 26 "100ppm/°C" H 6850 5250 50  0001 L CNN "temperature coefficient"
F 27 "+155°C" H 6850 5350 50  0001 L CNN "temperature range high"
F 28 "-55°C" H 6850 5450 50  0001 L CNN "temperature range low"
F 29 "1%" H 6850 5550 50  0001 L CNN "tolerance"
F 30 "75V" H 6850 5650 50  0001 L CNN "voltage"
F 31 "75V" H 6850 5750 50  0001 L CNN "voltage rating"
	1    6850 2450
	0    1    1    0   
$EndComp
Wire Wire Line
	6850 3050 6850 3100
Wire Wire Line
	6850 3100 6650 3100
Wire Wire Line
	6050 3050 6050 3100
Wire Wire Line
	6050 3100 6250 3100
$Comp
L power:+5VP #PWR040
U 1 1 5FD87EDE
P 6850 2500
F 0 "#PWR040" H 6850 2350 50  0001 C CNN
F 1 "+5VP" H 6700 2650 50  0000 L CNN
F 2 "" H 6850 2500 50  0001 C CNN
F 3 "" H 6850 2500 50  0001 C CNN
	1    6850 2500
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR036
U 1 1 5FD87EE4
P 6050 2500
F 0 "#PWR036" H 6050 2350 50  0001 C CNN
F 1 "+5V" H 6065 2673 50  0000 C CNN
F 2 "" H 6050 2500 50  0001 C CNN
F 3 "" H 6050 2500 50  0001 C CNN
	1    6050 2500
	1    0    0    -1  
$EndComp
Wire Wire Line
	6050 2500 6050 2550
Wire Wire Line
	6850 2500 6850 2550
Wire Wire Line
	6450 2800 6450 2550
Text Label 6050 3100 0    50   ~ 0
IRQ
Text Label 6650 3100 0    50   ~ 0
IRQP
Wire Wire Line
	6450 2550 6850 2550
Connection ~ 6850 2550
Wire Wire Line
	4300 3750 4400 3750
Wire Wire Line
	4300 3250 4300 3750
Wire Wire Line
	3050 6900 3850 6900
Connection ~ 1200 7000
$Comp
L power:GND #PWR03
U 1 1 5FD73FBD
P 900 5200
F 0 "#PWR03" H 900 4950 50  0001 C CNN
F 1 "GND" H 905 5027 50  0000 C CNN
F 2 "" H 900 5200 50  0001 C CNN
F 3 "" H 900 5200 50  0001 C CNN
	1    900  5200
	1    0    0    -1  
$EndComp
Wire Wire Line
	900  5200 900  5150
Connection ~ 900  5150
Wire Wire Line
	3050 7550 3400 7550
Wire Wire Line
	3050 6800 3450 6800
Wire Wire Line
	1650 3050 1750 3050
Wire Wire Line
	8600 4800 8650 4800
Wire Wire Line
	8650 4800 8650 3900
Wire Wire Line
	8600 3900 8650 3900
Wire Wire Line
	8600 3800 8650 3800
Wire Wire Line
	8650 3800 8650 2900
Wire Wire Line
	8650 2900 8600 2900
Wire Wire Line
	8600 2800 8650 2800
Wire Wire Line
	8650 2800 8650 1900
Wire Wire Line
	8650 1900 8600 1900
Wire Wire Line
	8600 1800 8650 1800
Wire Wire Line
	8650 1800 8650 900 
Wire Wire Line
	8600 900  8650 900 
Wire Wire Line
	10200 4800 10250 4800
Wire Wire Line
	10250 4800 10250 3900
Wire Wire Line
	10250 3900 10200 3900
Wire Wire Line
	10200 3800 10250 3800
Wire Wire Line
	10250 3800 10250 2900
Wire Wire Line
	10250 2900 10200 2900
Wire Wire Line
	10200 2800 10250 2800
Wire Wire Line
	10250 2800 10250 1900
Wire Wire Line
	10250 1900 10200 1900
Wire Wire Line
	10200 1800 10250 1800
Wire Wire Line
	10250 1800 10250 900 
Wire Wire Line
	10250 900  10200 900 
Wire Wire Line
	8600 800  8700 800 
Wire Wire Line
	8700 800  8700 5550
Wire Wire Line
	8700 5550 10250 5550
Wire Wire Line
	10250 5550 10250 4900
Wire Wire Line
	10250 4900 10200 4900
$Comp
L project:RC0603FR-0710KL R18
U 1 1 604E696D
P 9050 5750
F 0 "R18" H 9400 6055 50  0000 C CNN
F 1 "RC0603FR-0710KL" H 9050 6050 50  0001 L CNN
F 2 "project:Yageo-RC0603-0-0-IPC_A" H 9050 6150 50  0001 L CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RC_Group_51_RoHS_L_9.pdf" H 9050 6250 50  0001 L CNN
F 4 "541-1.00HHCT-ND" H 9050 6350 50  0001 L CNN "approved alternate part number"
F 5 "Res" H 9050 6450 50  0001 L CNN "category"
F 6 "Thick Film" H 9050 6550 50  0001 L CNN "composition"
F 7 "Passive Components" H 9050 6650 50  0001 L CNN "device class L1"
F 8 "Resistors" H 9050 6750 50  0001 L CNN "device class L2"
F 9 "Chip SMD Resistors" H 9050 6850 50  0001 L CNN "device class L3"
F 10 "RES SMD 10K OHM 1% 1/10W 0603" H 9050 6950 50  0001 L CNN "digikey description"
F 11 "311-10.0KHRTR-ND" H 9050 7050 50  0001 L CNN "digikey part number"
F 12 "0.5mm" H 9050 7150 50  0001 L CNN "height"
F 13 "RESC15585X45" H 9050 7250 50  0001 L CNN "ipc land pattern name"
F 14 "yes" H 9050 7350 50  0001 L CNN "lead free"
F 15 "368bdb3a487a5469" H 9050 7450 50  0001 L CNN "library id"
F 16 "Yageo" H 9050 7550 50  0001 L CNN "manufacturer"
F 17 "Surface Mount Thick Film Resistor, RC Series, 10 kohm, 100 mW, - 1%, 50 V, 0603 [1608 Metric]" H 9050 7650 50  0001 L CNN "mouser description"
F 18 "603-RC0603FR-0710KL" H 9050 7750 50  0001 L CNN "mouser part number"
F 19 "0603" H 9400 5964 50  0000 C CNN "package"
F 20 "100mW" H 9050 7950 50  0001 L CNN "power"
F 21 "0.1W" H 9050 8050 50  0001 L CNN "power rating"
F 22 "10kΩ" H 9400 5873 50  0000 C CNN "resistance"
F 23 "yes" H 9050 8250 50  0001 L CNN "rohs"
F 24 "RC" H 9050 8350 50  0001 L CNN "series"
F 25 "0mm" H 9050 8450 50  0001 L CNN "standoff height"
F 26 "100ppm/°C" H 9050 8550 50  0001 L CNN "temperature coefficient"
F 27 "+155°C" H 9050 8650 50  0001 L CNN "temperature range high"
F 28 "-55°C" H 9050 8750 50  0001 L CNN "temperature range low"
F 29 "1%" H 9050 8850 50  0001 L CNN "tolerance"
F 30 "75V" H 9050 8950 50  0001 L CNN "voltage"
F 31 "75V" H 9050 9050 50  0001 L CNN "voltage rating"
	1    9050 5750
	0    1    1    0   
$EndComp
$Comp
L project:RC0603FR-0710KL R19
U 1 1 604E698F
P 9850 5750
F 0 "R19" H 10200 6055 50  0000 C CNN
F 1 "RC0603FR-0710KL" H 9850 6050 50  0001 L CNN
F 2 "project:Yageo-RC0603-0-0-IPC_A" H 9850 6150 50  0001 L CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RC_Group_51_RoHS_L_9.pdf" H 9850 6250 50  0001 L CNN
F 4 "541-1.00HHCT-ND" H 9850 6350 50  0001 L CNN "approved alternate part number"
F 5 "Res" H 9850 6450 50  0001 L CNN "category"
F 6 "Thick Film" H 9850 6550 50  0001 L CNN "composition"
F 7 "Passive Components" H 9850 6650 50  0001 L CNN "device class L1"
F 8 "Resistors" H 9850 6750 50  0001 L CNN "device class L2"
F 9 "Chip SMD Resistors" H 9850 6850 50  0001 L CNN "device class L3"
F 10 "RES SMD 10K OHM 1% 1/10W 0603" H 9850 6950 50  0001 L CNN "digikey description"
F 11 "311-10.0KHRTR-ND" H 9850 7050 50  0001 L CNN "digikey part number"
F 12 "0.5mm" H 9850 7150 50  0001 L CNN "height"
F 13 "RESC15585X45" H 9850 7250 50  0001 L CNN "ipc land pattern name"
F 14 "yes" H 9850 7350 50  0001 L CNN "lead free"
F 15 "368bdb3a487a5469" H 9850 7450 50  0001 L CNN "library id"
F 16 "Yageo" H 9850 7550 50  0001 L CNN "manufacturer"
F 17 "Surface Mount Thick Film Resistor, RC Series, 10 kohm, 100 mW, - 1%, 50 V, 0603 [1608 Metric]" H 9850 7650 50  0001 L CNN "mouser description"
F 18 "603-RC0603FR-0710KL" H 9850 7750 50  0001 L CNN "mouser part number"
F 19 "0603" H 10200 5964 50  0000 C CNN "package"
F 20 "100mW" H 9850 7950 50  0001 L CNN "power"
F 21 "0.1W" H 9850 8050 50  0001 L CNN "power rating"
F 22 "10kΩ" H 10200 5873 50  0000 C CNN "resistance"
F 23 "yes" H 9850 8250 50  0001 L CNN "rohs"
F 24 "RC" H 9850 8350 50  0001 L CNN "series"
F 25 "0mm" H 9850 8450 50  0001 L CNN "standoff height"
F 26 "100ppm/°C" H 9850 8550 50  0001 L CNN "temperature coefficient"
F 27 "+155°C" H 9850 8650 50  0001 L CNN "temperature range high"
F 28 "-55°C" H 9850 8750 50  0001 L CNN "temperature range low"
F 29 "1%" H 9850 8850 50  0001 L CNN "tolerance"
F 30 "75V" H 9850 8950 50  0001 L CNN "voltage"
F 31 "75V" H 9850 9050 50  0001 L CNN "voltage rating"
	1    9850 5750
	0    1    1    0   
$EndComp
Wire Wire Line
	9850 6350 9850 6400
Wire Wire Line
	9850 6400 9650 6400
Wire Wire Line
	9050 6350 9050 6400
Wire Wire Line
	9050 6400 9250 6400
$Comp
L power:+5VP #PWR067
U 1 1 604E6999
P 9850 5800
F 0 "#PWR067" H 9850 5650 50  0001 C CNN
F 1 "+5VP" H 9700 5950 50  0000 L CNN
F 2 "" H 9850 5800 50  0001 C CNN
F 3 "" H 9850 5800 50  0001 C CNN
	1    9850 5800
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR056
U 1 1 604E699F
P 9050 5800
F 0 "#PWR056" H 9050 5650 50  0001 C CNN
F 1 "+5V" H 9065 5973 50  0000 C CNN
F 2 "" H 9050 5800 50  0001 C CNN
F 3 "" H 9050 5800 50  0001 C CNN
	1    9050 5800
	1    0    0    -1  
$EndComp
Wire Wire Line
	9050 5800 9050 5850
Wire Wire Line
	9850 5800 9850 5850
Wire Wire Line
	9450 6100 9450 5850
Text Label 9050 6400 0    50   ~ 0
RXT
Text Label 9650 6400 0    50   ~ 0
TXP
Wire Wire Line
	9450 5850 9850 5850
$Comp
L project:BSS138 Q5
U 1 1 604E69D4
P 9450 6000
F 0 "Q5" V 10000 6150 50  0000 C CNN
F 1 "BSS138" V 10000 5900 50  0000 C CNN
F 2 "project:ON_Semiconductor-318-08-01_2018-AS-MFG" H 9450 6500 50  0001 L CNN
F 3 "https://www.onsemi.com/pub/Collateral/BSS138-D.PDF" H 9450 6600 50  0001 L CNN
F 4 "No" H 9450 6700 50  0001 L CNN "automotive"
F 5 "Trans" H 9450 6800 50  0001 L CNN "category"
F 6 "0.22A" V 10100 5900 50  0000 C CNN "continuous drain current"
F 7 "False" H 9450 7000 50  0001 L CNN "depletion mode"
F 8 "Discrete Semiconductors" H 9450 7100 50  0001 L CNN "device class L1"
F 9 "Transistors" H 9450 7200 50  0001 L CNN "device class L2"
F 10 "MOSFETs" H 9450 7300 50  0001 L CNN "device class L3"
F 11 "MOSFET N-CH 50V 220MA SOT-23" H 9450 7400 50  0001 L CNN "digikey description"
F 12 "BSS138CT-ND" H 9450 7500 50  0001 L CNN "digikey part number"
F 13 "50V" V 10100 5700 50  0000 C CNN "drain to source breakdown voltage"
F 14 "0.7Ω" H 9450 7700 50  0001 L CNN "drain to source resistance"
F 15 "50V" H 9450 7800 50  0001 L CNN "drain to source voltage"
F 16 "https://www.onsemi.com/pub/Collateral/318-08.PDF" H 9450 7900 50  0001 L CNN "footprint url"
F 17 "1.7nC @ 10V" H 9450 8000 50  0001 L CNN "gate charge at vgs"
F 18 "20V" H 9450 8100 50  0001 L CNN "gate to source voltage"
F 19 "1.11mm" H 9450 8200 50  0001 L CNN "height"
F 20 "27pF @ 25V" H 9450 8300 50  0001 L CNN "input capacitace at vds"
F 21 "Yes" H 9450 8400 50  0001 L CNN "lead free"
F 22 "3f36dc230686dcb0" H 9450 8500 50  0001 L CNN "library id"
F 23 "ON Semiconductor" H 9450 8600 50  0001 L CNN "manufacturer"
F 24 "1.4V" H 9450 8700 50  0001 L CNN "max forward diode voltage"
F 25 "+150°C" H 9450 8800 50  0001 L CNN "max junction temp"
F 26 "N-Channel Logic Level Enhancement Mode Field Effect Transistor, 50V, 220mA" H 9450 8900 50  0001 L CNN "mouser description"
F 27 "512-BSS138" H 9450 9000 50  0001 L CNN "mouser part number"
F 28 "1" H 9450 9100 50  0001 L CNN "number of N channels"
F 29 "1" H 9450 9200 50  0001 L CNN "number of channels"
F 30 "SOT23-3" V 10100 6200 50  0000 C CNN "package"
F 31 "0.36W" H 9450 9400 50  0001 L CNN "power dissipation"
F 32 "0.88A" H 9450 9500 50  0001 L CNN "pulse drain current"
F 33 "Yes" H 9450 9600 50  0001 L CNN "rohs"
F 34 "350°C/W" H 9450 9700 50  0001 L CNN "rthja max"
F 35 "0.01mm" H 9450 9800 50  0001 L CNN "standoff height"
F 36 "+150°C" H 9450 9900 50  0001 L CNN "temperature range high"
F 37 "-55°C" H 9450 10000 50  0001 L CNN "temperature range low"
F 38 "1.5V" H 9450 10100 50  0001 L CNN "threshold vgs max"
F 39 "0.8V" H 9450 10200 50  0001 L CNN "threshold vgs min"
F 40 "0.5S" H 9450 10300 50  0001 L CNN "transconductance"
F 41 "20ns" H 9450 10400 50  0001 L CNN "turn off delay time"
F 42 "2.5ns" H 9450 10500 50  0001 L CNN "turn on delay time"
F 43 "0.8V" H 9450 10600 50  0001 L CNN "typ forward diode voltage"
	1    9450 6000
	0    -1   1    0   
$EndComp
Wire Wire Line
	9850 6400 10300 6400
Wire Wire Line
	10300 800  10200 800 
Connection ~ 9850 6400
$Comp
L project:RC0603FR-0710KL R17
U 1 1 60558730
P 8200 5750
F 0 "R17" H 8550 6055 50  0000 C CNN
F 1 "RC0603FR-0710KL" H 8200 6050 50  0001 L CNN
F 2 "project:Yageo-RC0603-0-0-IPC_A" H 8200 6150 50  0001 L CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RC_Group_51_RoHS_L_9.pdf" H 8200 6250 50  0001 L CNN
F 4 "541-1.00HHCT-ND" H 8200 6350 50  0001 L CNN "approved alternate part number"
F 5 "Res" H 8200 6450 50  0001 L CNN "category"
F 6 "Thick Film" H 8200 6550 50  0001 L CNN "composition"
F 7 "Passive Components" H 8200 6650 50  0001 L CNN "device class L1"
F 8 "Resistors" H 8200 6750 50  0001 L CNN "device class L2"
F 9 "Chip SMD Resistors" H 8200 6850 50  0001 L CNN "device class L3"
F 10 "RES SMD 10K OHM 1% 1/10W 0603" H 8200 6950 50  0001 L CNN "digikey description"
F 11 "311-10.0KHRTR-ND" H 8200 7050 50  0001 L CNN "digikey part number"
F 12 "0.5mm" H 8200 7150 50  0001 L CNN "height"
F 13 "RESC15585X45" H 8200 7250 50  0001 L CNN "ipc land pattern name"
F 14 "yes" H 8200 7350 50  0001 L CNN "lead free"
F 15 "368bdb3a487a5469" H 8200 7450 50  0001 L CNN "library id"
F 16 "Yageo" H 8200 7550 50  0001 L CNN "manufacturer"
F 17 "Surface Mount Thick Film Resistor, RC Series, 10 kohm, 100 mW, - 1%, 50 V, 0603 [1608 Metric]" H 8200 7650 50  0001 L CNN "mouser description"
F 18 "603-RC0603FR-0710KL" H 8200 7750 50  0001 L CNN "mouser part number"
F 19 "0603" H 8550 5964 50  0000 C CNN "package"
F 20 "100mW" H 8200 7950 50  0001 L CNN "power"
F 21 "0.1W" H 8200 8050 50  0001 L CNN "power rating"
F 22 "10kΩ" H 8550 5873 50  0000 C CNN "resistance"
F 23 "yes" H 8200 8250 50  0001 L CNN "rohs"
F 24 "RC" H 8200 8350 50  0001 L CNN "series"
F 25 "0mm" H 8200 8450 50  0001 L CNN "standoff height"
F 26 "100ppm/°C" H 8200 8550 50  0001 L CNN "temperature coefficient"
F 27 "+155°C" H 8200 8650 50  0001 L CNN "temperature range high"
F 28 "-55°C" H 8200 8750 50  0001 L CNN "temperature range low"
F 29 "1%" H 8200 8850 50  0001 L CNN "tolerance"
F 30 "75V" H 8200 8950 50  0001 L CNN "voltage"
F 31 "75V" H 8200 9050 50  0001 L CNN "voltage rating"
	1    8200 5750
	0    1    1    0   
$EndComp
Wire Wire Line
	8200 6350 8200 6400
$Comp
L power:+5VP #PWR055
U 1 1 6055873E
P 8200 5800
F 0 "#PWR055" H 8200 5650 50  0001 C CNN
F 1 "+5VP" H 8050 5950 50  0000 L CNN
F 2 "" H 8200 5800 50  0001 C CNN
F 3 "" H 8200 5800 50  0001 C CNN
	1    8200 5800
	1    0    0    -1  
$EndComp
Wire Wire Line
	8200 5800 8200 5850
Text Label 7400 6400 0    50   ~ 0
TXT
Text Label 8000 6400 0    50   ~ 0
RXP
Wire Wire Line
	8200 6400 8650 6400
Wire Wire Line
	8650 6400 8650 4900
Wire Wire Line
	8650 4900 8600 4900
Connection ~ 8200 6400
Wire Wire Line
	7400 6400 7600 6400
Wire Wire Line
	8000 6400 8200 6400
Connection ~ 9850 5850
Wire Wire Line
	7800 6100 7800 5850
Wire Wire Line
	7800 5850 8200 5850
$Comp
L project:BSS138 Q4
U 1 1 60760EB0
P 7800 6000
F 0 "Q4" V 8350 6150 50  0000 C CNN
F 1 "BSS138" V 8350 5900 50  0000 C CNN
F 2 "project:ON_Semiconductor-318-08-01_2018-AS-MFG" H 7800 6500 50  0001 L CNN
F 3 "https://www.onsemi.com/pub/Collateral/BSS138-D.PDF" H 7800 6600 50  0001 L CNN
F 4 "No" H 7800 6700 50  0001 L CNN "automotive"
F 5 "Trans" H 7800 6800 50  0001 L CNN "category"
F 6 "0.22A" V 8450 5900 50  0000 C CNN "continuous drain current"
F 7 "False" H 7800 7000 50  0001 L CNN "depletion mode"
F 8 "Discrete Semiconductors" H 7800 7100 50  0001 L CNN "device class L1"
F 9 "Transistors" H 7800 7200 50  0001 L CNN "device class L2"
F 10 "MOSFETs" H 7800 7300 50  0001 L CNN "device class L3"
F 11 "MOSFET N-CH 50V 220MA SOT-23" H 7800 7400 50  0001 L CNN "digikey description"
F 12 "BSS138CT-ND" H 7800 7500 50  0001 L CNN "digikey part number"
F 13 "50V" V 8450 5700 50  0000 C CNN "drain to source breakdown voltage"
F 14 "0.7Ω" H 7800 7700 50  0001 L CNN "drain to source resistance"
F 15 "50V" H 7800 7800 50  0001 L CNN "drain to source voltage"
F 16 "https://www.onsemi.com/pub/Collateral/318-08.PDF" H 7800 7900 50  0001 L CNN "footprint url"
F 17 "1.7nC @ 10V" H 7800 8000 50  0001 L CNN "gate charge at vgs"
F 18 "20V" H 7800 8100 50  0001 L CNN "gate to source voltage"
F 19 "1.11mm" H 7800 8200 50  0001 L CNN "height"
F 20 "27pF @ 25V" H 7800 8300 50  0001 L CNN "input capacitace at vds"
F 21 "Yes" H 7800 8400 50  0001 L CNN "lead free"
F 22 "3f36dc230686dcb0" H 7800 8500 50  0001 L CNN "library id"
F 23 "ON Semiconductor" H 7800 8600 50  0001 L CNN "manufacturer"
F 24 "1.4V" H 7800 8700 50  0001 L CNN "max forward diode voltage"
F 25 "+150°C" H 7800 8800 50  0001 L CNN "max junction temp"
F 26 "N-Channel Logic Level Enhancement Mode Field Effect Transistor, 50V, 220mA" H 7800 8900 50  0001 L CNN "mouser description"
F 27 "512-BSS138" H 7800 9000 50  0001 L CNN "mouser part number"
F 28 "1" H 7800 9100 50  0001 L CNN "number of N channels"
F 29 "1" H 7800 9200 50  0001 L CNN "number of channels"
F 30 "SOT23-3" V 8450 6200 50  0000 C CNN "package"
F 31 "0.36W" H 7800 9400 50  0001 L CNN "power dissipation"
F 32 "0.88A" H 7800 9500 50  0001 L CNN "pulse drain current"
F 33 "Yes" H 7800 9600 50  0001 L CNN "rohs"
F 34 "350°C/W" H 7800 9700 50  0001 L CNN "rthja max"
F 35 "0.01mm" H 7800 9800 50  0001 L CNN "standoff height"
F 36 "+150°C" H 7800 9900 50  0001 L CNN "temperature range high"
F 37 "-55°C" H 7800 10000 50  0001 L CNN "temperature range low"
F 38 "1.5V" H 7800 10100 50  0001 L CNN "threshold vgs max"
F 39 "0.8V" H 7800 10200 50  0001 L CNN "threshold vgs min"
F 40 "0.5S" H 7800 10300 50  0001 L CNN "transconductance"
F 41 "20ns" H 7800 10400 50  0001 L CNN "turn off delay time"
F 42 "2.5ns" H 7800 10500 50  0001 L CNN "turn on delay time"
F 43 "0.8V" H 7800 10600 50  0001 L CNN "typ forward diode voltage"
	1    7800 6000
	0    -1   1    0   
$EndComp
$Comp
L project:INA219AIDCNR U24
U 1 1 5FC372D1
P 5900 3750
F 0 "U24" H 6500 4105 50  0000 C CNN
F 1 "INA219AIDCNR" H 6500 4014 50  0000 C CNN
F 2 "project:Texas_Instruments-DCN0008A-0-0-MFG" H 5900 4150 50  0001 L CNN
F 3 "https://www.ti.com/lit/ds/symlink/ina219.pdf?ts=1599696930953&ref_url=https%253A%252F%252Fwww.ti.com%252Fstore%252Fti%252Fen%252Fp%252Fproduct%252F%253Fp%253DINA219AIDCNR%2526keyMatch%253DINA219AIDCNR%2526tisearch%253DSearch-EN-everything%2526usecase%253DOPN" H 5900 4250 50  0001 L CNN
F 4 "+85°C" H 5900 4350 50  0001 L CNN "ambient temperature range high"
F 5 "-25°C" H 5900 4450 50  0001 L CNN "ambient temperature range low"
F 6 "No" H 5900 4550 50  0001 L CNN "automotive"
F 7 "IC" H 5900 4650 50  0001 L CNN "category"
F 8 "Integrated Circuits (ICs)" H 5900 4750 50  0001 L CNN "device class L1"
F 9 "Power Management ICs" H 5900 4850 50  0001 L CNN "device class L2"
F 10 "unset" H 5900 4950 50  0001 L CNN "device class L3"
F 11 "IC CURRENT MONITOR 1% SOT23-8" H 5900 5050 50  0001 L CNN "digikey description"
F 12 "296-23770-1-ND" H 5900 5150 50  0001 L CNN "digikey part number"
F 13 "1.45mm" H 5900 5250 50  0001 L CNN "height"
F 14 "Yes" H 5900 5350 50  0001 L CNN "lead free"
F 15 "d9964224a5575dbc" H 5900 5450 50  0001 L CNN "library id"
F 16 "Texas Instruments" H 5900 5550 50  0001 L CNN "manufacturer"
F 17 "+150°C" H 5900 5650 50  0001 L CNN "max junction temp"
F 18 "5.5V" H 5900 5750 50  0001 L CNN "max supply voltage"
F 19 "3V" H 5900 5850 50  0001 L CNN "min supply voltage"
F 20 "595-INA219AIDCNR" H 5900 5950 50  0001 L CNN "mouser part number"
F 21 "0.7mA" H 5900 6050 50  0001 L CNN "nominal supply current"
F 22 "1" H 5900 6150 50  0001 L CNN "number of outputs"
F 23 "10mA" H 5900 6250 50  0001 L CNN "output current"
F 24 "SOT23-8" H 6500 3923 50  0000 C CNN "package"
F 25 "Yes" H 5900 6450 50  0001 L CNN "rohs"
F 26 "0mm" H 5900 6550 50  0001 L CNN "standoff height"
F 27 "+125°C" H 5900 6650 50  0001 L CNN "temperature range high"
F 28 "-40°C" H 5900 6750 50  0001 L CNN "temperature range low"
	1    5900 3750
	1    0    0    -1  
$EndComp
$Comp
L project:BSS138 Q1
U 1 1 60520BD4
P 6450 2700
F 0 "Q1" V 6350 2600 50  0000 L CNN
F 1 "BSS138" V 6450 2400 50  0000 L CNN
F 2 "project:ON_Semiconductor-318-08-01_2018-AS-MFG" H 6450 3200 50  0001 L CNN
F 3 "https://www.onsemi.com/pub/Collateral/BSS138-D.PDF" H 6450 3300 50  0001 L CNN
F 4 "No" H 6450 3400 50  0001 L CNN "automotive"
F 5 "Trans" H 6450 3500 50  0001 L CNN "category"
F 6 "0.22A" V 6650 2450 50  0000 L CNN "continuous drain current"
F 7 "False" H 6450 3700 50  0001 L CNN "depletion mode"
F 8 "Discrete Semiconductors" H 6450 3800 50  0001 L CNN "device class L1"
F 9 "Transistors" H 6450 3900 50  0001 L CNN "device class L2"
F 10 "MOSFETs" H 6450 4000 50  0001 L CNN "device class L3"
F 11 "MOSFET N-CH 50V 220MA SOT-23" H 6450 4100 50  0001 L CNN "digikey description"
F 12 "BSS138CT-ND" H 6450 4200 50  0001 L CNN "digikey part number"
F 13 "50V" V 6750 2450 50  0000 L CNN "drain to source breakdown voltage"
F 14 "0.7Ω" H 6450 4400 50  0001 L CNN "drain to source resistance"
F 15 "50V" H 6450 4500 50  0001 L CNN "drain to source voltage"
F 16 "https://www.onsemi.com/pub/Collateral/318-08.PDF" H 6450 4600 50  0001 L CNN "footprint url"
F 17 "1.7nC @ 10V" H 6450 4700 50  0001 L CNN "gate charge at vgs"
F 18 "20V" H 6450 4800 50  0001 L CNN "gate to source voltage"
F 19 "1.11mm" H 6450 4900 50  0001 L CNN "height"
F 20 "27pF @ 25V" H 6450 5000 50  0001 L CNN "input capacitace at vds"
F 21 "Yes" H 6450 5100 50  0001 L CNN "lead free"
F 22 "3f36dc230686dcb0" H 6450 5200 50  0001 L CNN "library id"
F 23 "ON Semiconductor" H 6450 5300 50  0001 L CNN "manufacturer"
F 24 "1.4V" H 6450 5400 50  0001 L CNN "max forward diode voltage"
F 25 "+150°C" H 6450 5500 50  0001 L CNN "max junction temp"
F 26 "N-Channel Logic Level Enhancement Mode Field Effect Transistor, 50V, 220mA" H 6450 5600 50  0001 L CNN "mouser description"
F 27 "512-BSS138" H 6450 5700 50  0001 L CNN "mouser part number"
F 28 "1" H 6450 5800 50  0001 L CNN "number of N channels"
F 29 "1" H 6450 5900 50  0001 L CNN "number of channels"
F 30 "SOT23-3" V 6550 2350 50  0000 L CNN "package"
F 31 "0.36W" H 6450 6100 50  0001 L CNN "power dissipation"
F 32 "0.88A" H 6450 6200 50  0001 L CNN "pulse drain current"
F 33 "Yes" H 6450 6300 50  0001 L CNN "rohs"
F 34 "350°C/W" H 6450 6400 50  0001 L CNN "rthja max"
F 35 "0.01mm" H 6450 6500 50  0001 L CNN "standoff height"
F 36 "+150°C" H 6450 6600 50  0001 L CNN "temperature range high"
F 37 "-55°C" H 6450 6700 50  0001 L CNN "temperature range low"
F 38 "1.5V" H 6450 6800 50  0001 L CNN "threshold vgs max"
F 39 "0.8V" H 6450 6900 50  0001 L CNN "threshold vgs min"
F 40 "0.5S" H 6450 7000 50  0001 L CNN "transconductance"
F 41 "20ns" H 6450 7100 50  0001 L CNN "turn off delay time"
F 42 "2.5ns" H 6450 7200 50  0001 L CNN "turn on delay time"
F 43 "0.8V" H 6450 7300 50  0001 L CNN "typ forward diode voltage"
	1    6450 2700
	0    -1   1    0   
$EndComp
$Comp
L project:BSS138 Q2
U 1 1 5FC59223
P 6450 950
F 0 "Q2" V 6350 850 50  0000 L CNN
F 1 "BSS138" V 6450 650 50  0000 L CNN
F 2 "project:ON_Semiconductor-318-08-01_2018-AS-MFG" H 6450 1450 50  0001 L CNN
F 3 "https://www.onsemi.com/pub/Collateral/BSS138-D.PDF" H 6450 1550 50  0001 L CNN
F 4 "No" H 6450 1650 50  0001 L CNN "automotive"
F 5 "Trans" H 6450 1750 50  0001 L CNN "category"
F 6 "0.22A" V 6650 700 50  0000 L CNN "continuous drain current"
F 7 "False" H 6450 1950 50  0001 L CNN "depletion mode"
F 8 "Discrete Semiconductors" H 6450 2050 50  0001 L CNN "device class L1"
F 9 "Transistors" H 6450 2150 50  0001 L CNN "device class L2"
F 10 "MOSFETs" H 6450 2250 50  0001 L CNN "device class L3"
F 11 "MOSFET N-CH 50V 220MA SOT-23" H 6450 2350 50  0001 L CNN "digikey description"
F 12 "BSS138CT-ND" H 6450 2450 50  0001 L CNN "digikey part number"
F 13 "50V" V 6750 700 50  0000 L CNN "drain to source breakdown voltage"
F 14 "0.7Ω" H 6450 2650 50  0001 L CNN "drain to source resistance"
F 15 "50V" H 6450 2750 50  0001 L CNN "drain to source voltage"
F 16 "https://www.onsemi.com/pub/Collateral/318-08.PDF" H 6450 2850 50  0001 L CNN "footprint url"
F 17 "1.7nC @ 10V" H 6450 2950 50  0001 L CNN "gate charge at vgs"
F 18 "20V" H 6450 3050 50  0001 L CNN "gate to source voltage"
F 19 "1.11mm" H 6450 3150 50  0001 L CNN "height"
F 20 "27pF @ 25V" H 6450 3250 50  0001 L CNN "input capacitace at vds"
F 21 "Yes" H 6450 3350 50  0001 L CNN "lead free"
F 22 "3f36dc230686dcb0" H 6450 3450 50  0001 L CNN "library id"
F 23 "ON Semiconductor" H 6450 3550 50  0001 L CNN "manufacturer"
F 24 "1.4V" H 6450 3650 50  0001 L CNN "max forward diode voltage"
F 25 "+150°C" H 6450 3750 50  0001 L CNN "max junction temp"
F 26 "N-Channel Logic Level Enhancement Mode Field Effect Transistor, 50V, 220mA" H 6450 3850 50  0001 L CNN "mouser description"
F 27 "512-BSS138" H 6450 3950 50  0001 L CNN "mouser part number"
F 28 "1" H 6450 4050 50  0001 L CNN "number of N channels"
F 29 "1" H 6450 4150 50  0001 L CNN "number of channels"
F 30 "SOT23-3" V 6550 600 50  0000 L CNN "package"
F 31 "0.36W" H 6450 4350 50  0001 L CNN "power dissipation"
F 32 "0.88A" H 6450 4450 50  0001 L CNN "pulse drain current"
F 33 "Yes" H 6450 4550 50  0001 L CNN "rohs"
F 34 "350°C/W" H 6450 4650 50  0001 L CNN "rthja max"
F 35 "0.01mm" H 6450 4750 50  0001 L CNN "standoff height"
F 36 "+150°C" H 6450 4850 50  0001 L CNN "temperature range high"
F 37 "-55°C" H 6450 4950 50  0001 L CNN "temperature range low"
F 38 "1.5V" H 6450 5050 50  0001 L CNN "threshold vgs max"
F 39 "0.8V" H 6450 5150 50  0001 L CNN "threshold vgs min"
F 40 "0.5S" H 6450 5250 50  0001 L CNN "transconductance"
F 41 "20ns" H 6450 5350 50  0001 L CNN "turn off delay time"
F 42 "2.5ns" H 6450 5450 50  0001 L CNN "turn on delay time"
F 43 "0.8V" H 6450 5550 50  0001 L CNN "typ forward diode voltage"
	1    6450 950 
	0    -1   1    0   
$EndComp
$Comp
L project:BSS138 Q3
U 1 1 5FC5B348
P 6450 1800
F 0 "Q3" V 6350 1700 50  0000 L CNN
F 1 "BSS138" V 6450 1500 50  0000 L CNN
F 2 "project:ON_Semiconductor-318-08-01_2018-AS-MFG" H 6450 2300 50  0001 L CNN
F 3 "https://www.onsemi.com/pub/Collateral/BSS138-D.PDF" H 6450 2400 50  0001 L CNN
F 4 "No" H 6450 2500 50  0001 L CNN "automotive"
F 5 "Trans" H 6450 2600 50  0001 L CNN "category"
F 6 "0.22A" V 6650 1550 50  0000 L CNN "continuous drain current"
F 7 "False" H 6450 2800 50  0001 L CNN "depletion mode"
F 8 "Discrete Semiconductors" H 6450 2900 50  0001 L CNN "device class L1"
F 9 "Transistors" H 6450 3000 50  0001 L CNN "device class L2"
F 10 "MOSFETs" H 6450 3100 50  0001 L CNN "device class L3"
F 11 "MOSFET N-CH 50V 220MA SOT-23" H 6450 3200 50  0001 L CNN "digikey description"
F 12 "BSS138CT-ND" H 6450 3300 50  0001 L CNN "digikey part number"
F 13 "50V" V 6750 1550 50  0000 L CNN "drain to source breakdown voltage"
F 14 "0.7Ω" H 6450 3500 50  0001 L CNN "drain to source resistance"
F 15 "50V" H 6450 3600 50  0001 L CNN "drain to source voltage"
F 16 "https://www.onsemi.com/pub/Collateral/318-08.PDF" H 6450 3700 50  0001 L CNN "footprint url"
F 17 "1.7nC @ 10V" H 6450 3800 50  0001 L CNN "gate charge at vgs"
F 18 "20V" H 6450 3900 50  0001 L CNN "gate to source voltage"
F 19 "1.11mm" H 6450 4000 50  0001 L CNN "height"
F 20 "27pF @ 25V" H 6450 4100 50  0001 L CNN "input capacitace at vds"
F 21 "Yes" H 6450 4200 50  0001 L CNN "lead free"
F 22 "3f36dc230686dcb0" H 6450 4300 50  0001 L CNN "library id"
F 23 "ON Semiconductor" H 6450 4400 50  0001 L CNN "manufacturer"
F 24 "1.4V" H 6450 4500 50  0001 L CNN "max forward diode voltage"
F 25 "+150°C" H 6450 4600 50  0001 L CNN "max junction temp"
F 26 "N-Channel Logic Level Enhancement Mode Field Effect Transistor, 50V, 220mA" H 6450 4700 50  0001 L CNN "mouser description"
F 27 "512-BSS138" H 6450 4800 50  0001 L CNN "mouser part number"
F 28 "1" H 6450 4900 50  0001 L CNN "number of N channels"
F 29 "1" H 6450 5000 50  0001 L CNN "number of channels"
F 30 "SOT23-3" V 6550 1450 50  0000 L CNN "package"
F 31 "0.36W" H 6450 5200 50  0001 L CNN "power dissipation"
F 32 "0.88A" H 6450 5300 50  0001 L CNN "pulse drain current"
F 33 "Yes" H 6450 5400 50  0001 L CNN "rohs"
F 34 "350°C/W" H 6450 5500 50  0001 L CNN "rthja max"
F 35 "0.01mm" H 6450 5600 50  0001 L CNN "standoff height"
F 36 "+150°C" H 6450 5700 50  0001 L CNN "temperature range high"
F 37 "-55°C" H 6450 5800 50  0001 L CNN "temperature range low"
F 38 "1.5V" H 6450 5900 50  0001 L CNN "threshold vgs max"
F 39 "0.8V" H 6450 6000 50  0001 L CNN "threshold vgs min"
F 40 "0.5S" H 6450 6100 50  0001 L CNN "transconductance"
F 41 "20ns" H 6450 6200 50  0001 L CNN "turn off delay time"
F 42 "2.5ns" H 6450 6300 50  0001 L CNN "turn on delay time"
F 43 "0.8V" H 6450 6400 50  0001 L CNN "typ forward diode voltage"
	1    6450 1800
	0    -1   1    0   
$EndComp
$Comp
L power:+5V #PWR0101
U 1 1 5FC9B34B
P 5950 3700
F 0 "#PWR0101" H 5950 3550 50  0001 C CNN
F 1 "+5V" H 5965 3873 50  0000 C CNN
F 2 "" H 5950 3700 50  0001 C CNN
F 3 "" H 5950 3700 50  0001 C CNN
	1    5950 3700
	1    0    0    -1  
$EndComp
Wire Wire Line
	6000 3750 5950 3750
Wire Wire Line
	5950 3750 5950 3700
$Comp
L project:CC0805ZKY5V6BB106 C?
U 1 1 606FC690
P 2050 1200
AR Path="/5FC1E32F/606FC690" Ref="C?"  Part="1" 
AR Path="/606FC690" Ref="C4"  Part="1" 
F 0 "C4" H 2178 1237 50  0000 L CNN
F 1 "CC0805ZKY5V6BB106" H 2050 1400 50  0001 L CNN
F 2 "project:YAGEO-CC0805-0-0-0" H 2050 1500 50  0001 L CNN
F 3 "http://www.yageo.com/documents/recent/UPY-GPHC_Y5V_6.3V-to-50V_8.pdf" H 2050 1600 50  0001 L CNN
F 4 "10uF" H 2178 1146 50  0000 L CNN "capacitance"
F 5 "Cap" H 2050 1800 50  0001 L CNN "category"
F 6 "CAP CER 10UF 10V Y5V 0805" H 2050 1900 50  0001 L CNN "digikey description"
F 7 "311-1355-1-ND" H 2050 2000 50  0001 L CNN "digikey part number"
F 8 "yes" H 2050 2100 50  0001 L CNN "lead free"
F 9 "852e7d0cf36849be" H 2050 2200 50  0001 L CNN "library id"
F 10 "YAGEO" H 2050 2300 50  0001 L CNN "manufacturer"
F 11 "Multilayer Ceramic Capacitors MLCC - SMD/SMT 10uF 10V Y5V-20/+80%" H 2050 2400 50  0001 L CNN "mouser description"
F 12 "603-CC805ZKY5V6BB106" H 2050 2500 50  0001 L CNN "mouser part number"
F 13 "0805" H 2178 1055 50  0000 L CNN "package"
F 14 "yes" H 2050 2700 50  0001 L CNN "rohs"
F 15 "+85°C" H 2050 2800 50  0001 L CNN "temperature range high"
F 16 "-30°C" H 2050 2900 50  0001 L CNN "temperature range low"
F 17 "10V" H 2178 964 50  0000 L CNN "voltage"
	1    2050 1200
	1    0    0    -1  
$EndComp
Wire Wire Line
	2050 1100 2400 1100
Wire Wire Line
	2050 1100 2050 1200
$Comp
L power:GND #PWR010
U 1 1 60638D2F
P 2050 1450
F 0 "#PWR010" H 2050 1200 50  0001 C CNN
F 1 "GND" H 2055 1277 50  0000 C CNN
F 2 "" H 2050 1450 50  0001 C CNN
F 3 "" H 2050 1450 50  0001 C CNN
	1    2050 1450
	1    0    0    -1  
$EndComp
Wire Wire Line
	2050 1400 2050 1450
Connection ~ 2050 1100
$Comp
L power:+5VP #PWR09
U 1 1 601CC086
P 2050 1100
F 0 "#PWR09" H 2050 950 50  0001 C CNN
F 1 "+5VP" H 2065 1273 50  0000 C CNN
F 2 "" H 2050 1100 50  0001 C CNN
F 3 "" H 2050 1100 50  0001 C CNN
	1    2050 1100
	1    0    0    -1  
$EndComp
$Comp
L project:PJ-002AH J1
U 1 1 601F46CA
P 1400 1100
F 0 "J1" H 1050 1400 50  0000 C CNN
F 1 "PJ-002AH" H 1050 1300 50  0000 C CNN
F 2 "project:CUI_PJ-002AH" H 1400 1400 50  0001 L CNN
F 3 "https://www.cui.com/product/resource/pj-002ah.pdf" H 1400 1500 50  0001 L CNN
F 4 "Conn" H 1400 1600 50  0001 L CNN "category"
F 5 "Silver" H 1400 1700 50  0001 L CNN "contact material"
F 6 "5A" H 1150 1200 50  0000 C CNN "current rating"
F 7 "Connectors" H 1400 1900 50  0001 L CNN "device class L1"
F 8 "Power Connectors" H 1400 2000 50  0001 L CNN "device class L2"
F 9 "unset" H 1400 2100 50  0001 L CNN "device class L3"
F 10 "CONN PWR JACK 2X5.5MM SOLDER" H 1200 550 50  0000 C CNN "digikey description"
F 11 "CP-002AH-ND" H 1400 2300 50  0001 L CNN "digikey part number"
F 12 "11.2mm" H 1400 2400 50  0001 L CNN "height"
F 13 "yes" H 1400 2500 50  0001 L CNN "lead free"
F 14 "9faf6f1e7549c614" H 1400 2600 50  0001 L CNN "library id"
F 15 "CUI" H 1400 2700 50  0001 L CNN "manufacturer"
F 16 "DC Power Connectors Power Jacks" H 1400 2800 50  0001 L CNN "mouser description"
F 17 "490-PJ-002AH" H 1400 2900 50  0001 L CNN "mouser part number"
F 18 "2" H 1400 3000 50  0001 L CNN "number of contacts"
F 19 "CONN_PWR_JACK" H 1400 3100 50  0001 L CNN "package"
F 20 "yes" H 1400 3200 50  0001 L CNN "rohs"
F 21 "+85°C" H 1400 3300 50  0001 L CNN "temperature range high"
F 22 "-25°C" H 1400 3400 50  0001 L CNN "temperature range low"
F 23 "24V" H 950 1200 50  0000 C CNN "voltage rating"
	1    1400 1100
	1    0    0    -1  
$EndComp
Wire Wire Line
	1300 1300 1400 1300
Wire Wire Line
	1400 1300 1400 1350
$Comp
L power:GND #PWR05
U 1 1 60205E8B
P 1400 1350
F 0 "#PWR05" H 1400 1100 50  0001 C CNN
F 1 "GND" H 1405 1177 50  0000 C CNN
F 2 "" H 1400 1350 50  0001 C CNN
F 3 "" H 1400 1350 50  0001 C CNN
	1    1400 1350
	1    0    0    -1  
$EndComp
$Comp
L project:RL2512FK-070R1L R16
U 1 1 5FD2D8C3
P 1500 1100
F 0 "R16" H 1750 1497 50  0000 C CNN
F 1 "RL2512FK-070R1L" H 1500 1400 50  0001 L CNN
F 2 "project:YAGEO-RL2512-02_2018-10-MFG" H 1500 1500 50  0001 L CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RL_Group_521_RoHS_L_2.pdf" H 1500 1600 50  0001 L CNN
F 4 "Yes" H 1500 1700 50  0001 L CNN "automotive"
F 5 "Grade 0" H 1500 1800 50  0001 L CNN "automotive grade"
F 6 "Res" H 1500 1900 50  0001 L CNN "category"
F 7 "Thick Film" H 1500 2000 50  0001 L CNN "composition"
F 8 "Passive Components" H 1500 2100 50  0001 L CNN "device class L1"
F 9 "Resistors" H 1500 2200 50  0001 L CNN "device class L2"
F 10 "Chip SMD Resistors" H 1500 2300 50  0001 L CNN "device class L3"
F 11 "RES 0.1 OHM 1% 1W 2512" H 1500 2400 50  0001 L CNN "digikey description"
F 12 "311-0.1TCT-ND" H 1500 2500 50  0001 L CNN "digikey part number"
F 13 "http://www.yageo.com/exep/pages/download/literatures/PYu-R_Mount_10.pdf" H 1500 2600 50  0001 L CNN "footprint url"
F 14 "0.65mm" H 1500 2700 50  0001 L CNN "height"
F 15 "RESC635320X55" H 1500 2800 50  0001 L CNN "ipc land pattern name"
F 16 "Yes" H 1500 2900 50  0001 L CNN "lead free"
F 17 "15048a7efe84bced" H 1500 3000 50  0001 L CNN "library id"
F 18 "YAGEO" H 1500 3100 50  0001 L CNN "manufacturer"
F 19 "Current Sense Resistors 0.1ohm 1% 1W" H 1500 3200 50  0001 L CNN "mouser description"
F 20 "603-RL252FK070R1L" H 1500 3300 50  0001 L CNN "mouser part number"
F 21 "2512" H 1750 1406 50  0000 C CNN "package"
F 22 "1W" H 1750 1315 50  0000 C CNN "power rating"
F 23 "100mΩ" H 1750 1224 50  0000 C CNN "resistance"
F 24 "Yes" H 1500 3700 50  0001 L CNN "rohs"
F 25 "RL" H 1500 3800 50  0001 L CNN "series"
F 26 "600ppm/°C" H 1500 3900 50  0001 L CNN "temperature coefficient"
F 27 "+155°C" H 1500 4000 50  0001 L CNN "temperature range high"
F 28 "-55°C" H 1500 4100 50  0001 L CNN "temperature range low"
F 29 "1%" H 1500 4200 50  0001 L CNN "tolerance"
	1    1500 1100
	1    0    0    -1  
$EndComp
Wire Wire Line
	1500 1100 1300 1100
Wire Wire Line
	2000 1100 2050 1100
Text Label 1350 1100 0    50   ~ 0
VIN
Wire Wire Line
	6000 4050 5800 4050
Text Label 5850 4050 0    50   ~ 0
VIN
$Comp
L power:+5VP #PWR0102
U 1 1 5FDC3834
P 5950 4150
F 0 "#PWR0102" H 5950 4000 50  0001 C CNN
F 1 "+5VP" V 5965 4278 50  0000 L CNN
F 2 "" H 5950 4150 50  0001 C CNN
F 3 "" H 5950 4150 50  0001 C CNN
	1    5950 4150
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5950 4150 6000 4150
Wire Wire Line
	7000 3750 7200 3750
Wire Wire Line
	7000 3850 7200 3850
Wire Wire Line
	7000 4050 7050 4050
Wire Wire Line
	7050 4050 7050 4150
Wire Wire Line
	7000 4350 7050 4350
Connection ~ 7050 4350
Wire Wire Line
	7050 4350 7050 4400
Wire Wire Line
	7000 4150 7050 4150
Connection ~ 7050 4150
Wire Wire Line
	7050 4150 7050 4350
$Comp
L power:GND #PWR0103
U 1 1 5FE7B82E
P 7050 4400
F 0 "#PWR0103" H 7050 4150 50  0001 C CNN
F 1 "GND" H 7100 4250 50  0000 R CNN
F 2 "" H 7050 4400 50  0001 C CNN
F 3 "" H 7050 4400 50  0001 C CNN
	1    7050 4400
	1    0    0    -1  
$EndComp
Text Label 7000 3750 0    50   ~ 0
SCL
Text Label 7000 3850 0    50   ~ 0
SDA
$Comp
L project:CC0603KPX7R9BB104 C?
U 1 1 600E4A92
P 4550 2650
AR Path="/5FC1E32F/600E4A92" Ref="C?"  Part="1" 
AR Path="/5FBD86FB/600E4A92" Ref="C?"  Part="1" 
AR Path="/600E4A92" Ref="C39"  Part="1" 
F 0 "C39" V 4700 2800 50  0000 L CNN
F 1 "CC0603KPX7R9BB104" H 4550 2850 50  0001 L CNN
F 2 "project:YAGEO-CC0603-0-0-0" H 4550 2950 50  0001 L CNN
F 3 "http://www.yageo.com/documents/recent/UPY-GPHC_X7R_6.3V-to-50V_18.pdf" H 4550 3050 50  0001 L CNN
F 4 "No" H 4550 3150 50  0001 L CNN "automotive"
F 5 "100 nF" V 4700 2450 50  0000 L CNN "capacitance"
F 6 "Cap" H 4550 3350 50  0001 L CNN "category"
F 7 "Passive Components" H 4550 3450 50  0001 L CNN "device class L1"
F 8 "Capacitors" H 4550 3550 50  0001 L CNN "device class L2"
F 9 "Ceramic Capacitors" H 4550 3650 50  0001 L CNN "device class L3"
F 10 "CAP CER 0.1UF 50V X7R 0603" H 4550 3750 50  0001 L CNN "digikey description"
F 11 "CC0603KPX7R9BB104-ND" H 4550 3850 50  0001 L CNN "digikey part number"
F 12 "http://www.yageo.com/exep/pages/download/literatures/UPY-C_GEN_24.pdf" H 4550 3950 50  0001 L CNN "footprint url"
F 13 "0.9mm" H 4550 4050 50  0001 L CNN "height"
F 14 "Yes" H 4550 4150 50  0001 L CNN "lead free"
F 15 "7cd692b09ff39865" H 4550 4250 50  0001 L CNN "library id"
F 16 "YAGEO" H 4550 4350 50  0001 L CNN "manufacturer"
F 17 "Ceramic" H 4550 4450 50  0001 L CNN "material"
F 18 "Multilayer Ceramic Capacitors MLCC - SMD/SMT 100nF 50V X7R 10%" H 4550 4550 50  0001 L CNN "mouser description"
F 19 "603-CC603KPX7R9BB104" H 4550 4650 50  0001 L CNN "mouser part number"
F 20 "0603" V 4700 2200 50  0000 L CNN "package"
F 21 "Yes" H 4550 4850 50  0001 L CNN "rohs"
F 22 "X7R" H 4550 4950 50  0001 L CNN "temperature characteristic"
F 23 "15%" H 4550 5050 50  0001 L CNN "temperature coefficient"
F 24 "+125°C" H 4550 5150 50  0001 L CNN "temperature range high"
F 25 "-55°C" H 4550 5250 50  0001 L CNN "temperature range low"
F 26 "0.1" H 4550 5350 50  0001 L CNN "tolerance"
F 27 "50 V" V 4700 2250 50  0001 L CNN "voltage"
F 28 "50 V" H 4550 5550 50  0001 L CNN "voltage rating"
	1    4550 2650
	0    -1   -1   0   
$EndComp
NoConn ~ 4000 2350
Wire Wire Line
	5150 2650 4750 2650
Wire Wire Line
	4250 2650 4550 2650
Wire Wire Line
	4000 3250 4300 3250
Wire Wire Line
	4000 3150 4400 3150
$Comp
L project:RC0603FR-071K65L R?
U 1 1 609EA832
P 6050 700
AR Path="/5FBD86FB/609EA832" Ref="R?"  Part="1" 
AR Path="/5FC1E32F/609EA832" Ref="R?"  Part="1" 
AR Path="/609EA832" Ref="R11"  Part="1" 
F 0 "R11" H 6400 1005 50  0000 C CNN
F 1 "RC0603FR-071K65L" H 6400 914 50  0001 C CNN
F 2 "project:Yageo-RC0603-0-0-IPC_A" H 6050 1100 50  0001 L CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RC_Group_51_RoHS_L_9.pdf" H 6050 1200 50  0001 L CNN
F 4 "541-1.00HHCT-ND" H 6050 1300 50  0001 L CNN "approved alternate part number"
F 5 "Res" H 6050 1400 50  0001 L CNN "category"
F 6 "Thick Film" H 6050 1500 50  0001 L CNN "composition"
F 7 "Passive Components" H 6050 1600 50  0001 L CNN "device class L1"
F 8 "Resistors" H 6050 1700 50  0001 L CNN "device class L2"
F 9 "Chip SMD Resistors" H 6050 1800 50  0001 L CNN "device class L3"
F 10 "RES SMD 1.65K OHM 1% 1/10W 0603" H 6050 1900 50  0001 L CNN "digikey description"
F 11 "311-1.65KHRTR-ND" H 6050 2000 50  0001 L CNN "digikey part number"
F 12 "0.5mm" H 6050 2100 50  0001 L CNN "height"
F 13 "RESC15585X45" H 6050 2200 50  0001 L CNN "ipc land pattern name"
F 14 "yes" H 6050 2300 50  0001 L CNN "lead free"
F 15 "ce4140018ac48837" H 6050 2400 50  0001 L CNN "library id"
F 16 "Yageo" H 6050 2500 50  0001 L CNN "manufacturer"
F 17 "Thick Film Resistors - SMD 1.65K OHM 1%\\n" H 6050 2600 50  0001 L CNN "mouser description"
F 18 "603-RC0603FR-071K65L" H 6050 2700 50  0001 L CNN "mouser part number"
F 19 "0603" H 6400 914 50  0000 C CNN "package"
F 20 "100mW" H 6050 2900 50  0001 L CNN "power"
F 21 "0.1W" H 6050 3000 50  0001 L CNN "power rating"
F 22 "1.65kΩ" H 6400 823 50  0000 C CNN "resistance"
F 23 "yes" H 6050 3200 50  0001 L CNN "rohs"
F 24 "RC" H 6050 3300 50  0001 L CNN "series"
F 25 "0mm" H 6050 3400 50  0001 L CNN "standoff height"
F 26 "100ppm/°C" H 6050 3500 50  0001 L CNN "temperature coefficient"
F 27 "+155°C" H 6050 3600 50  0001 L CNN "temperature range high"
F 28 "-55°C" H 6050 3700 50  0001 L CNN "temperature range low"
F 29 "1%" H 6050 3800 50  0001 L CNN "tolerance"
F 30 "75V" H 6050 3900 50  0001 L CNN "voltage"
F 31 "75V" H 6050 4000 50  0001 L CNN "voltage rating"
	1    6050 700 
	0    1    1    0   
$EndComp
$Comp
L project:RC0603FR-071K65L R?
U 1 1 60A76BA5
P 6050 1550
AR Path="/5FBD86FB/60A76BA5" Ref="R?"  Part="1" 
AR Path="/5FC1E32F/60A76BA5" Ref="R?"  Part="1" 
AR Path="/60A76BA5" Ref="R12"  Part="1" 
F 0 "R12" H 6400 1855 50  0000 C CNN
F 1 "RC0603FR-071K65L" H 6400 1764 50  0001 C CNN
F 2 "project:Yageo-RC0603-0-0-IPC_A" H 6050 1950 50  0001 L CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RC_Group_51_RoHS_L_9.pdf" H 6050 2050 50  0001 L CNN
F 4 "541-1.00HHCT-ND" H 6050 2150 50  0001 L CNN "approved alternate part number"
F 5 "Res" H 6050 2250 50  0001 L CNN "category"
F 6 "Thick Film" H 6050 2350 50  0001 L CNN "composition"
F 7 "Passive Components" H 6050 2450 50  0001 L CNN "device class L1"
F 8 "Resistors" H 6050 2550 50  0001 L CNN "device class L2"
F 9 "Chip SMD Resistors" H 6050 2650 50  0001 L CNN "device class L3"
F 10 "RES SMD 1.65K OHM 1% 1/10W 0603" H 6050 2750 50  0001 L CNN "digikey description"
F 11 "311-1.65KHRTR-ND" H 6050 2850 50  0001 L CNN "digikey part number"
F 12 "0.5mm" H 6050 2950 50  0001 L CNN "height"
F 13 "RESC15585X45" H 6050 3050 50  0001 L CNN "ipc land pattern name"
F 14 "yes" H 6050 3150 50  0001 L CNN "lead free"
F 15 "ce4140018ac48837" H 6050 3250 50  0001 L CNN "library id"
F 16 "Yageo" H 6050 3350 50  0001 L CNN "manufacturer"
F 17 "Thick Film Resistors - SMD 1.65K OHM 1%\\n" H 6050 3450 50  0001 L CNN "mouser description"
F 18 "603-RC0603FR-071K65L" H 6050 3550 50  0001 L CNN "mouser part number"
F 19 "0603" H 6400 1764 50  0000 C CNN "package"
F 20 "100mW" H 6050 3750 50  0001 L CNN "power"
F 21 "0.1W" H 6050 3850 50  0001 L CNN "power rating"
F 22 "1.65kΩ" H 6400 1673 50  0000 C CNN "resistance"
F 23 "yes" H 6050 4050 50  0001 L CNN "rohs"
F 24 "RC" H 6050 4150 50  0001 L CNN "series"
F 25 "0mm" H 6050 4250 50  0001 L CNN "standoff height"
F 26 "100ppm/°C" H 6050 4350 50  0001 L CNN "temperature coefficient"
F 27 "+155°C" H 6050 4450 50  0001 L CNN "temperature range high"
F 28 "-55°C" H 6050 4550 50  0001 L CNN "temperature range low"
F 29 "1%" H 6050 4650 50  0001 L CNN "tolerance"
F 30 "75V" H 6050 4750 50  0001 L CNN "voltage"
F 31 "75V" H 6050 4850 50  0001 L CNN "voltage rating"
	1    6050 1550
	0    1    1    0   
$EndComp
Wire Wire Line
	3050 5700 3300 5700
Wire Wire Line
	3050 5800 3300 5800
Text Label 3100 5800 0    50   ~ 0
TXT
Text Label 3100 5700 0    50   ~ 0
RXT
Wire Wire Line
	8600 5000 8800 5000
Wire Wire Line
	8800 5000 8800 5100
Wire Wire Line
	8800 5300 8600 5300
Wire Wire Line
	8600 5200 8800 5200
Connection ~ 8800 5200
Wire Wire Line
	8800 5200 8800 5300
Wire Wire Line
	8600 5100 8800 5100
Connection ~ 8800 5100
Wire Wire Line
	8800 5100 8800 5200
$Comp
L power:GND #PWR071
U 1 1 6106D1F5
P 8800 5300
F 0 "#PWR071" H 8800 5050 50  0001 C CNN
F 1 "GND" H 8900 5150 50  0000 R CNN
F 2 "" H 8800 5300 50  0001 C CNN
F 3 "" H 8800 5300 50  0001 C CNN
	1    8800 5300
	1    0    0    -1  
$EndComp
Connection ~ 8800 5300
$Sheet
S 7850 3700 750  700 
U 611D0642
F0 "TX2" 50
F1 "transmitter.sch" 50
F2 "VS" U L 7850 3800 50 
F3 "GND" U L 7850 4300 50 
F4 "SCL" U L 7850 3900 50 
F5 "SDA" U L 7850 4000 50 
F6 "IRQ" U L 7850 4100 50 
F7 "CLK" I L 7850 4200 50 
F8 "RX" I R 8600 3900 50 
F9 "TX" O R 8600 3800 50 
F10 "A0" U R 8600 4000 50 
F11 "A1" U R 8600 4100 50 
F12 "A2" U R 8600 4200 50 
F13 "A3" U R 8600 4300 50 
$EndSheet
$Sheet
S 7850 2700 750  700 
U 611F2B9B
F0 "TX3" 50
F1 "transmitter.sch" 50
F2 "VS" U L 7850 2800 50 
F3 "GND" U L 7850 3300 50 
F4 "SCL" U L 7850 2900 50 
F5 "SDA" U L 7850 3000 50 
F6 "IRQ" U L 7850 3100 50 
F7 "CLK" I L 7850 3200 50 
F8 "RX" I R 8600 2900 50 
F9 "TX" O R 8600 2800 50 
F10 "A0" U R 8600 3000 50 
F11 "A1" U R 8600 3100 50 
F12 "A2" U R 8600 3200 50 
F13 "A3" U R 8600 3300 50 
$EndSheet
$Sheet
S 7850 1700 750  700 
U 612151C4
F0 "TX4" 50
F1 "transmitter.sch" 50
F2 "VS" U L 7850 1800 50 
F3 "GND" U L 7850 2300 50 
F4 "SCL" U L 7850 1900 50 
F5 "SDA" U L 7850 2000 50 
F6 "IRQ" U L 7850 2100 50 
F7 "CLK" I L 7850 2200 50 
F8 "RX" I R 8600 1900 50 
F9 "TX" O R 8600 1800 50 
F10 "A0" U R 8600 2000 50 
F11 "A1" U R 8600 2100 50 
F12 "A2" U R 8600 2200 50 
F13 "A3" U R 8600 2300 50 
$EndSheet
$Sheet
S 7850 700  750  700 
U 612375E5
F0 "TX5" 50
F1 "transmitter.sch" 50
F2 "VS" U L 7850 800 50 
F3 "GND" U L 7850 1300 50 
F4 "SCL" U L 7850 900 50 
F5 "SDA" U L 7850 1000 50 
F6 "IRQ" U L 7850 1100 50 
F7 "CLK" I L 7850 1200 50 
F8 "RX" I R 8600 900 50 
F9 "TX" O R 8600 800 50 
F10 "A0" U R 8600 1000 50 
F11 "A1" U R 8600 1100 50 
F12 "A2" U R 8600 1200 50 
F13 "A3" U R 8600 1300 50 
$EndSheet
$Sheet
S 9450 4700 750  700 
U 61259C35
F0 "TX6" 50
F1 "transmitter.sch" 50
F2 "VS" U L 9450 4800 50 
F3 "GND" U L 9450 5300 50 
F4 "SCL" U L 9450 4900 50 
F5 "SDA" U L 9450 5000 50 
F6 "IRQ" U L 9450 5100 50 
F7 "CLK" I L 9450 5200 50 
F8 "RX" I R 10200 4900 50 
F9 "TX" O R 10200 4800 50 
F10 "A0" U R 10200 5000 50 
F11 "A1" U R 10200 5100 50 
F12 "A2" U R 10200 5200 50 
F13 "A3" U R 10200 5300 50 
$EndSheet
$Sheet
S 9450 3700 750  700 
U 6127C0CB
F0 "TX7" 50
F1 "transmitter.sch" 50
F2 "VS" U L 9450 3800 50 
F3 "GND" U L 9450 4300 50 
F4 "SCL" U L 9450 3900 50 
F5 "SDA" U L 9450 4000 50 
F6 "IRQ" U L 9450 4100 50 
F7 "CLK" I L 9450 4200 50 
F8 "RX" I R 10200 3900 50 
F9 "TX" O R 10200 3800 50 
F10 "A0" U R 10200 4000 50 
F11 "A1" U R 10200 4100 50 
F12 "A2" U R 10200 4200 50 
F13 "A3" U R 10200 4300 50 
$EndSheet
$Sheet
S 9450 2700 750  700 
U 6129E6CD
F0 "TX8" 50
F1 "transmitter.sch" 50
F2 "VS" U L 9450 2800 50 
F3 "GND" U L 9450 3300 50 
F4 "SCL" U L 9450 2900 50 
F5 "SDA" U L 9450 3000 50 
F6 "IRQ" U L 9450 3100 50 
F7 "CLK" I L 9450 3200 50 
F8 "RX" I R 10200 2900 50 
F9 "TX" O R 10200 2800 50 
F10 "A0" U R 10200 3000 50 
F11 "A1" U R 10200 3100 50 
F12 "A2" U R 10200 3200 50 
F13 "A3" U R 10200 3300 50 
$EndSheet
$Sheet
S 9450 1700 750  700 
U 612C0A79
F0 "TX9" 50
F1 "transmitter.sch" 50
F2 "VS" U L 9450 1800 50 
F3 "GND" U L 9450 2300 50 
F4 "SCL" U L 9450 1900 50 
F5 "SDA" U L 9450 2000 50 
F6 "IRQ" U L 9450 2100 50 
F7 "CLK" I L 9450 2200 50 
F8 "RX" I R 10200 1900 50 
F9 "TX" O R 10200 1800 50 
F10 "A0" U R 10200 2000 50 
F11 "A1" U R 10200 2100 50 
F12 "A2" U R 10200 2200 50 
F13 "A3" U R 10200 2300 50 
$EndSheet
$Sheet
S 9450 700  750  700 
U 612E307B
F0 "TX10" 50
F1 "transmitter.sch" 50
F2 "VS" U L 9450 800 50 
F3 "GND" U L 9450 1300 50 
F4 "SCL" U L 9450 900 50 
F5 "SDA" U L 9450 1000 50 
F6 "IRQ" U L 9450 1100 50 
F7 "CLK" I L 9450 1200 50 
F8 "RX" I R 10200 900 50 
F9 "TX" O R 10200 800 50 
F10 "A0" U R 10200 1000 50 
F11 "A1" U R 10200 1100 50 
F12 "A2" U R 10200 1200 50 
F13 "A3" U R 10200 1300 50 
$EndSheet
Wire Wire Line
	8600 4000 8750 4000
Wire Wire Line
	8800 4300 8600 4300
Wire Wire Line
	8600 4200 8800 4200
Connection ~ 8800 4200
Wire Wire Line
	8800 4200 8800 4300
Wire Wire Line
	8600 4100 8800 4100
Wire Wire Line
	8800 4100 8800 4200
$Comp
L power:GND #PWR070
U 1 1 617D0D17
P 8800 4300
F 0 "#PWR070" H 8800 4050 50  0001 C CNN
F 1 "GND" H 8900 4150 50  0000 R CNN
F 2 "" H 8800 4300 50  0001 C CNN
F 3 "" H 8800 4300 50  0001 C CNN
	1    8800 4300
	1    0    0    -1  
$EndComp
Connection ~ 8800 4300
Wire Wire Line
	8600 3000 8800 3000
Wire Wire Line
	8800 3300 8600 3300
Wire Wire Line
	8600 3200 8800 3200
Connection ~ 8800 3200
Wire Wire Line
	8800 3200 8800 3300
$Comp
L power:GND #PWR069
U 1 1 617F482A
P 8800 3300
F 0 "#PWR069" H 8800 3050 50  0001 C CNN
F 1 "GND" H 8900 3150 50  0000 R CNN
F 2 "" H 8800 3300 50  0001 C CNN
F 3 "" H 8800 3300 50  0001 C CNN
	1    8800 3300
	1    0    0    -1  
$EndComp
Connection ~ 8800 3300
Wire Wire Line
	8600 2000 8750 2000
Wire Wire Line
	8750 2000 8750 2100
Wire Wire Line
	8800 2300 8600 2300
Wire Wire Line
	8600 2200 8800 2200
Wire Wire Line
	8800 2200 8800 2300
Wire Wire Line
	8600 2100 8750 2100
$Comp
L power:GND #PWR068
U 1 1 6181A00D
P 8800 2300
F 0 "#PWR068" H 8800 2050 50  0001 C CNN
F 1 "GND" H 8900 2150 50  0000 R CNN
F 2 "" H 8800 2300 50  0001 C CNN
F 3 "" H 8800 2300 50  0001 C CNN
	1    8800 2300
	1    0    0    -1  
$EndComp
Connection ~ 8800 2300
Wire Wire Line
	8600 1000 8800 1000
Wire Wire Line
	8800 1000 8800 1100
Wire Wire Line
	8800 1300 8600 1300
Wire Wire Line
	8600 1100 8800 1100
Connection ~ 8800 1100
$Comp
L power:GND #PWR044
U 1 1 6183FDD9
P 8800 1300
F 0 "#PWR044" H 8800 1050 50  0001 C CNN
F 1 "GND" H 8900 1150 50  0000 R CNN
F 2 "" H 8800 1300 50  0001 C CNN
F 3 "" H 8800 1300 50  0001 C CNN
	1    8800 1300
	1    0    0    -1  
$EndComp
Connection ~ 8800 1300
Wire Wire Line
	10200 5000 10350 5000
Wire Wire Line
	10400 5300 10200 5300
Wire Wire Line
	10200 5100 10400 5100
$Comp
L power:GND #PWR081
U 1 1 618674F5
P 10400 5300
F 0 "#PWR081" H 10400 5050 50  0001 C CNN
F 1 "GND" H 10500 5150 50  0000 R CNN
F 2 "" H 10400 5300 50  0001 C CNN
F 3 "" H 10400 5300 50  0001 C CNN
	1    10400 5300
	1    0    0    -1  
$EndComp
Connection ~ 10400 5300
Wire Wire Line
	10200 4000 10400 4000
Wire Wire Line
	10400 4300 10200 4300
$Comp
L power:GND #PWR080
U 1 1 6188F909
P 10400 4300
F 0 "#PWR080" H 10400 4050 50  0001 C CNN
F 1 "GND" H 10500 4150 50  0000 R CNN
F 2 "" H 10400 4300 50  0001 C CNN
F 3 "" H 10400 4300 50  0001 C CNN
	1    10400 4300
	1    0    0    -1  
$EndComp
Connection ~ 10400 4300
Wire Wire Line
	10200 3000 10350 3000
Wire Wire Line
	10350 3000 10350 3100
Wire Wire Line
	10400 3300 10200 3300
Wire Wire Line
	10200 3200 10350 3200
Wire Wire Line
	10200 3100 10350 3100
Connection ~ 10350 3100
Wire Wire Line
	10350 3100 10350 3200
$Comp
L power:GND #PWR079
U 1 1 618B8909
P 10400 3300
F 0 "#PWR079" H 10400 3050 50  0001 C CNN
F 1 "GND" H 10500 3150 50  0000 R CNN
F 2 "" H 10400 3300 50  0001 C CNN
F 3 "" H 10400 3300 50  0001 C CNN
	1    10400 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	10200 2000 10400 2000
Wire Wire Line
	10400 2000 10400 2100
Wire Wire Line
	10200 2200 10400 2200
Connection ~ 10400 2200
Wire Wire Line
	10400 2200 10400 2300
Wire Wire Line
	10200 2100 10400 2100
Connection ~ 10400 2100
Wire Wire Line
	10400 2100 10400 2200
$Comp
L power:GND #PWR078
U 1 1 618E2E2A
P 10400 2300
F 0 "#PWR078" H 10400 2050 50  0001 C CNN
F 1 "GND" H 10500 2150 50  0000 R CNN
F 2 "" H 10400 2300 50  0001 C CNN
F 3 "" H 10400 2300 50  0001 C CNN
	1    10400 2300
	1    0    0    -1  
$EndComp
Connection ~ 8200 5850
$Comp
L power:+5VP #PWR043
U 1 1 619452D5
P 8750 3950
F 0 "#PWR043" H 8750 3800 50  0001 C CNN
F 1 "+5VP" H 8800 4000 50  0000 L CNN
F 2 "" H 8750 3950 50  0001 C CNN
F 3 "" H 8750 3950 50  0001 C CNN
	1    8750 3950
	1    0    0    -1  
$EndComp
Wire Wire Line
	8750 3950 8750 4000
$Comp
L power:+5VP #PWR039
U 1 1 619F3C4C
P 8750 2950
F 0 "#PWR039" H 8750 2800 50  0001 C CNN
F 1 "+5VP" H 8800 3000 50  0000 L CNN
F 2 "" H 8750 2950 50  0001 C CNN
F 3 "" H 8750 2950 50  0001 C CNN
	1    8750 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	8800 3000 8800 3200
Wire Wire Line
	8600 3100 8750 3100
Wire Wire Line
	8750 3100 8750 2950
$Comp
L power:+5VP #PWR031
U 1 1 61A4B5DB
P 8750 1950
F 0 "#PWR031" H 8750 1800 50  0001 C CNN
F 1 "+5VP" H 8800 2000 50  0000 L CNN
F 2 "" H 8750 1950 50  0001 C CNN
F 3 "" H 8750 1950 50  0001 C CNN
	1    8750 1950
	1    0    0    -1  
$EndComp
Wire Wire Line
	8750 2000 8750 1950
Connection ~ 8750 2000
$Comp
L power:+5VP #PWR030
U 1 1 61ACEA0D
P 8750 950
F 0 "#PWR030" H 8750 800 50  0001 C CNN
F 1 "+5VP" H 8800 1000 50  0000 L CNN
F 2 "" H 8750 950 50  0001 C CNN
F 3 "" H 8750 950 50  0001 C CNN
	1    8750 950 
	1    0    0    -1  
$EndComp
Wire Wire Line
	8800 1100 8800 1300
Wire Wire Line
	8600 1200 8750 1200
Wire Wire Line
	8750 1200 8750 950 
$Comp
L power:+5VP #PWR076
U 1 1 61B264AE
P 10350 4950
F 0 "#PWR076" H 10350 4800 50  0001 C CNN
F 1 "+5VP" H 10300 5100 50  0000 L CNN
F 2 "" H 10350 4950 50  0001 C CNN
F 3 "" H 10350 4950 50  0001 C CNN
	1    10350 4950
	1    0    0    -1  
$EndComp
Wire Wire Line
	10400 5100 10400 5300
Wire Wire Line
	10300 6400 10300 800 
Wire Wire Line
	10200 5200 10350 5200
Wire Wire Line
	10350 5200 10350 5000
Connection ~ 10350 5000
Wire Wire Line
	10350 5000 10350 4950
$Comp
L power:+5VP #PWR075
U 1 1 61C01B5F
P 10350 3950
F 0 "#PWR075" H 10350 3800 50  0001 C CNN
F 1 "+5VP" H 10300 4100 50  0000 L CNN
F 2 "" H 10350 3950 50  0001 C CNN
F 3 "" H 10350 3950 50  0001 C CNN
	1    10350 3950
	1    0    0    -1  
$EndComp
Wire Wire Line
	10400 4000 10400 4300
Wire Wire Line
	10200 4200 10350 4200
Wire Wire Line
	10350 4200 10350 4100
Wire Wire Line
	10200 4100 10350 4100
Connection ~ 10350 4100
Wire Wire Line
	10350 4100 10350 3950
$Comp
L power:+5VP #PWR074
U 1 1 61CAFD0B
P 10350 2950
F 0 "#PWR074" H 10350 2800 50  0001 C CNN
F 1 "+5VP" H 10300 3100 50  0000 L CNN
F 2 "" H 10350 2950 50  0001 C CNN
F 3 "" H 10350 2950 50  0001 C CNN
	1    10350 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	10350 3000 10350 2950
Connection ~ 10350 3000
$Comp
L power:+5VP #PWR073
U 1 1 61D34187
P 10350 1950
F 0 "#PWR073" H 10350 1800 50  0001 C CNN
F 1 "+5VP" H 10300 2100 50  0000 L CNN
F 2 "" H 10350 1950 50  0001 C CNN
F 3 "" H 10350 1950 50  0001 C CNN
	1    10350 1950
	1    0    0    -1  
$EndComp
Wire Wire Line
	10200 2300 10350 2300
Wire Wire Line
	10350 2300 10350 1950
Wire Wire Line
	10200 1000 10350 1000
Wire Wire Line
	10200 1200 10400 1200
Connection ~ 10400 1200
Wire Wire Line
	10400 1200 10400 1300
Wire Wire Line
	10200 1100 10400 1100
Wire Wire Line
	10400 1100 10400 1200
$Comp
L power:GND #PWR077
U 1 1 61DC4F64
P 10400 1300
F 0 "#PWR077" H 10400 1050 50  0001 C CNN
F 1 "GND" H 10500 1150 50  0000 R CNN
F 2 "" H 10400 1300 50  0001 C CNN
F 3 "" H 10400 1300 50  0001 C CNN
	1    10400 1300
	1    0    0    -1  
$EndComp
$Comp
L power:+5VP #PWR072
U 1 1 61DC4F6E
P 10350 950
F 0 "#PWR072" H 10350 800 50  0001 C CNN
F 1 "+5VP" H 10300 1100 50  0000 L CNN
F 2 "" H 10350 950 50  0001 C CNN
F 3 "" H 10350 950 50  0001 C CNN
	1    10350 950 
	1    0    0    -1  
$EndComp
Wire Wire Line
	10200 1300 10350 1300
Wire Wire Line
	10350 1300 10350 1000
Connection ~ 10350 1000
Wire Wire Line
	10350 1000 10350 950 
Wire Wire Line
	4450 4600 4200 4600
$Comp
L power:GND #PWR026
U 1 1 61E59ABB
P 4200 4600
F 0 "#PWR026" H 4200 4350 50  0001 C CNN
F 1 "GND" V 4205 4472 50  0000 R CNN
F 2 "" H 4200 4600 50  0001 C CNN
F 3 "" H 4200 4600 50  0001 C CNN
	1    4200 4600
	0    1    1    0   
$EndComp
Wire Wire Line
	4450 4700 4150 4700
Wire Wire Line
	4450 4800 4150 4800
Wire Wire Line
	4450 4900 4150 4900
$Comp
L power:+5V #PWR025
U 1 1 61E59AC4
P 4150 4700
F 0 "#PWR025" H 4150 4550 50  0001 C CNN
F 1 "+5V" V 4165 4828 50  0000 L CNN
F 2 "" H 4150 4700 50  0001 C CNN
F 3 "" H 4150 4700 50  0001 C CNN
	1    4150 4700
	0    -1   -1   0   
$EndComp
Text Label 4200 4800 0    50   ~ 0
ENCA
Text Label 4200 4900 0    50   ~ 0
ENCB
Text Notes 4150 5100 0    50   ~ 0
Socket for Encoder
$Comp
L project:SSW-104-01-T-S J9
U 1 1 61E59AE0
P 4350 4600
F 0 "J9" V 4100 4900 50  0000 L CNN
F 1 "SSW-104-01-T-S" V 4200 4250 50  0000 L CNN
F 2 "project:Samtec-SSW-104-01-T-S-Manufacturer_Recommended" H 4350 5300 50  0001 L CNN
F 3 "http://www.samtec.com/documents/webfiles/pdf/SLW.PDF" H 4350 5400 50  0001 L CNN
F 4 "Manufacturer URL" H 4350 5500 50  0001 L CNN "Component Link 1 Description"
F 5 "http://www.samtec.com" H 4350 5600 50  0001 L CNN "Component Link 1 URL"
F 6 "Package Specification" H 4350 5700 50  0001 L CNN "Component Link 3 Description"
F 7 "http://www.samtec.com/documents/webfiles/cpdf/SLW-1XX-01-X-X-MKT.pdf" H 4350 5800 50  0001 L CNN "Component Link 3 URL"
F 8 "F-214" H 4350 5900 50  0001 L CNN "Datasheet Version"
F 9 "Through-hole" H 4350 6000 50  0001 L CNN "Mounting Technology"
F 10 "Vertical" H 4350 6100 50  0001 L CNN "Orientation"
F 11 "Low Profile Socket Strip" H 4350 6200 50  0001 L CNN "Package Description"
F 12 "X, 9/1987" H 4350 6300 50  0001 L CNN "Package Version"
F 13 "2.54 mm" H 4350 6400 50  0001 L CNN "Pitch"
F 14 "-55 to 105 degC" H 4350 6500 50  0001 L CNN "Temperature Range"
F 15 "Conn" H 4350 6600 50  0001 L CNN "category"
F 16 "349254" H 4350 6700 50  0001 L CNN "ciiva ids"
F 17 "e244508a65ae41fc" H 4350 6800 50  0001 L CNN "library id"
F 18 "Samtec" H 4350 6900 50  0001 L CNN "manufacturer"
F 19 "SSW-104-01-X-S" H 4350 7000 50  0001 L CNN "package"
F 20 "1404374325" H 4350 7100 50  0001 L CNN "release date"
F 21 "3E9B3488-2A9B-444C-BFB7-BA4271584146" H 4350 7200 50  0001 L CNN "vault revision"
F 22 "yes" H 4350 7300 50  0001 L CNN "imported"
	1    4350 4600
	0    1    1    0   
$EndComp
Wire Wire Line
	3350 4600 3050 4600
Wire Wire Line
	3350 4700 3050 4700
Text Label 3100 4600 0    50   ~ 0
ENCA
Text Label 3100 4700 0    50   ~ 0
ENCB
Wire Wire Line
	6900 5100 7150 5100
Text Label 6950 5100 0    50   ~ 0
CLK1
Wire Wire Line
	6900 5200 7150 5200
Text Label 6950 5200 0    50   ~ 0
CLK2
Wire Wire Line
	6900 5300 7150 5300
Text Label 6950 5300 0    50   ~ 0
CLK3
Wire Wire Line
	6900 5400 7150 5400
Text Label 6950 5400 0    50   ~ 0
CLK4
Wire Wire Line
	6900 5500 7150 5500
Text Label 6950 5500 0    50   ~ 0
CLK5
Text Label 6000 5200 0    50   ~ 0
SDA
Text Label 6000 5300 0    50   ~ 0
SCL
Wire Wire Line
	5950 5200 6200 5200
Wire Wire Line
	5950 5300 6200 5300
Wire Wire Line
	5900 5100 6200 5100
$Comp
L power:GND #PWR028
U 1 1 6216F595
P 5900 5400
F 0 "#PWR028" H 5900 5150 50  0001 C CNN
F 1 "GND" H 5905 5227 50  0000 C CNN
F 2 "" H 5900 5400 50  0001 C CNN
F 3 "" H 5900 5400 50  0001 C CNN
	1    5900 5400
	0    1    1    0   
$EndComp
Wire Wire Line
	5900 5400 6200 5400
Wire Wire Line
	3050 6400 3300 6400
Text Label 3100 6400 0    50   ~ 0
CSS
Wire Wire Line
	6200 5500 5950 5500
Wire Wire Line
	6200 5600 5950 5600
Wire Wire Line
	6200 5700 5950 5700
Wire Wire Line
	6200 5800 5950 5800
Text Label 5950 5500 0    50   ~ 0
MOSI
Text Label 5950 5600 0    50   ~ 0
MISO
Text Label 5950 5700 0    50   ~ 0
SCK
Text Label 5950 5800 0    50   ~ 0
CSS
$Comp
L power:+5V #PWR027
U 1 1 606C03AD
P 5900 5100
F 0 "#PWR027" H 5900 4950 50  0001 C CNN
F 1 "+5V" H 5915 5273 50  0000 C CNN
F 2 "" H 5900 5100 50  0001 C CNN
F 3 "" H 5900 5100 50  0001 C CNN
	1    5900 5100
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3050 6300 3300 6300
Wire Wire Line
	3050 4800 3350 4800
Wire Wire Line
	3050 4900 3350 4900
Wire Wire Line
	3050 6000 3350 6000
Text Label 3100 6300 0    50   ~ 0
IED
Text Label 3100 6000 0    50   ~ 0
GRN
Text Label 3100 4800 0    50   ~ 0
ENED1
Text Label 3100 4900 0    50   ~ 0
ENED2
Wire Wire Line
	5950 5900 6200 5900
Wire Wire Line
	5950 6000 6200 6000
Wire Wire Line
	5950 6100 6200 6100
Text Label 5950 5900 0    50   ~ 0
GRN
Text Label 5950 6000 0    50   ~ 0
ENED1
Text Label 5950 6100 0    50   ~ 0
ENED2
NoConn ~ 1300 1200
Text Label 5950 6200 0    50   ~ 0
IED
Wire Wire Line
	5950 6200 6200 6200
$Sheet
S 6200 5000 700  1300
U 61F18BE9
F0 "DEV1" 50
F1 "lowVdevices.sch" 50
F2 "SDA" I L 6200 5200 50 
F3 "SCL" I L 6200 5300 50 
F4 "CLK1" O R 6900 5100 50 
F5 "CLK2" O R 6900 5200 50 
F6 "CLK3" O R 6900 5300 50 
F7 "CLK4" O R 6900 5400 50 
F8 "CLK5" O R 6900 5500 50 
F9 "GND" U L 6200 5400 50 
F10 "VH" U L 6200 5100 50 
F11 "MOSI" I L 6200 5500 50 
F12 "MISO" I L 6200 5600 50 
F13 "SCK" I L 6200 5700 50 
F14 "CSS" I L 6200 5800 50 
F15 "GRN" I L 6200 5900 50 
F16 "ENED1" I L 6200 6000 50 
F17 "ENED2" I L 6200 6100 50 
F18 "IED" U L 6200 6200 50 
$EndSheet
$Comp
L power:PWR_FLAG #FLG0101
U 1 1 601C27A8
P 1100 4250
F 0 "#FLG0101" H 1100 4325 50  0001 C CNN
F 1 "PWR_FLAG" H 1100 4423 50  0000 C CNN
F 2 "" H 1100 4250 50  0001 C CNN
F 3 "~" H 1100 4250 50  0001 C CNN
	1    1100 4250
	1    0    0    -1  
$EndComp
Wire Wire Line
	1100 4250 1100 4300
Connection ~ 1100 4300
Wire Wire Line
	1100 4300 750  4300
$Comp
L power:PWR_FLAG #FLG0102
U 1 1 601F8873
P 1100 6550
F 0 "#FLG0102" H 1100 6625 50  0001 C CNN
F 1 "PWR_FLAG" H 1100 6723 50  0000 C CNN
F 2 "" H 1100 6550 50  0001 C CNN
F 3 "~" H 1100 6550 50  0001 C CNN
	1    1100 6550
	1    0    0    -1  
$EndComp
Wire Wire Line
	1200 6600 1100 6600
Wire Wire Line
	1100 6600 1100 6550
Connection ~ 1200 6600
$Comp
L power:PWR_FLAG #FLG0103
U 1 1 602ED962
P 6450 750
F 0 "#FLG0103" H 6450 825 50  0001 C CNN
F 1 "PWR_FLAG" H 6450 923 50  0000 C CNN
F 2 "" H 6450 750 50  0001 C CNN
F 3 "~" H 6450 750 50  0001 C CNN
	1    6450 750 
	1    0    0    -1  
$EndComp
Wire Wire Line
	6450 750  6450 800 
Connection ~ 6450 800 
NoConn ~ 5350 7400
NoConn ~ 5350 7550
NoConn ~ 5350 7700
NoConn ~ 5350 7850
$Comp
L Connector_Generic:Conn_01x01 MT?
U 1 1 6045B68B
P 5550 7850
AR Path="/5FBD86FB/6045B68B" Ref="MT?"  Part="1" 
AR Path="/611D0642/6045B68B" Ref="MT?"  Part="1" 
AR Path="/611F2B9B/6045B68B" Ref="MT?"  Part="1" 
AR Path="/612151C4/6045B68B" Ref="MT?"  Part="1" 
AR Path="/612375E5/6045B68B" Ref="MT?"  Part="1" 
AR Path="/61259C35/6045B68B" Ref="MT?"  Part="1" 
AR Path="/6127C0CB/6045B68B" Ref="MT?"  Part="1" 
AR Path="/6129E6CD/6045B68B" Ref="MT?"  Part="1" 
AR Path="/612C0A79/6045B68B" Ref="MT?"  Part="1" 
AR Path="/612E307B/6045B68B" Ref="MT?"  Part="1" 
AR Path="/6045B68B" Ref="MT4"  Part="1" 
F 0 "MT4" H 5630 7892 50  0000 L CNN
F 1 "Mount Hole M3" H 5630 7801 50  0000 L CNN
F 2 "Mounting_Holes:MountingHole_3.2mm_M3" H 5550 7850 50  0001 C CNN
F 3 "~" H 5550 7850 50  0001 C CNN
	1    5550 7850
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x01 MT?
U 1 1 6045B691
P 5550 7700
AR Path="/5FBD86FB/6045B691" Ref="MT?"  Part="1" 
AR Path="/611D0642/6045B691" Ref="MT?"  Part="1" 
AR Path="/611F2B9B/6045B691" Ref="MT?"  Part="1" 
AR Path="/612151C4/6045B691" Ref="MT?"  Part="1" 
AR Path="/612375E5/6045B691" Ref="MT?"  Part="1" 
AR Path="/61259C35/6045B691" Ref="MT?"  Part="1" 
AR Path="/6127C0CB/6045B691" Ref="MT?"  Part="1" 
AR Path="/6129E6CD/6045B691" Ref="MT?"  Part="1" 
AR Path="/612C0A79/6045B691" Ref="MT?"  Part="1" 
AR Path="/612E307B/6045B691" Ref="MT?"  Part="1" 
AR Path="/6045B691" Ref="MT3"  Part="1" 
F 0 "MT3" H 5630 7742 50  0000 L CNN
F 1 "Mount Hole M3" H 5630 7651 50  0000 L CNN
F 2 "Mounting_Holes:MountingHole_3.2mm_M3" H 5550 7700 50  0001 C CNN
F 3 "~" H 5550 7700 50  0001 C CNN
	1    5550 7700
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x01 MT?
U 1 1 6045B697
P 5550 7550
AR Path="/5FBD86FB/6045B697" Ref="MT?"  Part="1" 
AR Path="/611D0642/6045B697" Ref="MT?"  Part="1" 
AR Path="/611F2B9B/6045B697" Ref="MT?"  Part="1" 
AR Path="/612151C4/6045B697" Ref="MT?"  Part="1" 
AR Path="/612375E5/6045B697" Ref="MT?"  Part="1" 
AR Path="/61259C35/6045B697" Ref="MT?"  Part="1" 
AR Path="/6127C0CB/6045B697" Ref="MT?"  Part="1" 
AR Path="/6129E6CD/6045B697" Ref="MT?"  Part="1" 
AR Path="/612C0A79/6045B697" Ref="MT?"  Part="1" 
AR Path="/612E307B/6045B697" Ref="MT?"  Part="1" 
AR Path="/6045B697" Ref="MT2"  Part="1" 
F 0 "MT2" H 5630 7592 50  0000 L CNN
F 1 "Mount Hole M3" H 5630 7501 50  0000 L CNN
F 2 "Mounting_Holes:MountingHole_3.2mm_M3" H 5550 7550 50  0001 C CNN
F 3 "~" H 5550 7550 50  0001 C CNN
	1    5550 7550
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x01 MT?
U 1 1 6045B69D
P 5550 7400
AR Path="/5FBD86FB/6045B69D" Ref="MT?"  Part="1" 
AR Path="/611D0642/6045B69D" Ref="MT?"  Part="1" 
AR Path="/611F2B9B/6045B69D" Ref="MT?"  Part="1" 
AR Path="/612151C4/6045B69D" Ref="MT?"  Part="1" 
AR Path="/612375E5/6045B69D" Ref="MT?"  Part="1" 
AR Path="/61259C35/6045B69D" Ref="MT?"  Part="1" 
AR Path="/6127C0CB/6045B69D" Ref="MT?"  Part="1" 
AR Path="/6129E6CD/6045B69D" Ref="MT?"  Part="1" 
AR Path="/612C0A79/6045B69D" Ref="MT?"  Part="1" 
AR Path="/612E307B/6045B69D" Ref="MT?"  Part="1" 
AR Path="/6045B69D" Ref="MT1"  Part="1" 
F 0 "MT1" H 5630 7442 50  0000 L CNN
F 1 "Mount Hole M3" H 5630 7351 50  0000 L CNN
F 2 "Mounting_Holes:MountingHole_3.2mm_M3" H 5550 7400 50  0001 C CNN
F 3 "~" H 5550 7400 50  0001 C CNN
	1    5550 7400
	1    0    0    -1  
$EndComp
$Comp
L project:fiducial FID1
U 1 1 6063C3CC
P 4800 7400
F 0 "FID1" H 4928 7446 50  0000 L CNN
F 1 "fiducial" H 4928 7355 50  0000 L CNN
F 2 "Fiducials:Fiducial_1mm_Dia_2.54mm_Outer_CopperTop" H 4950 7250 50  0001 C CNN
F 3 "" H 4800 7400 50  0001 C CNN
	1    4800 7400
	1    0    0    -1  
$EndComp
$Comp
L project:fiducial FID2
U 1 1 6063C8E8
P 4800 7600
F 0 "FID2" H 4928 7646 50  0000 L CNN
F 1 "fiducial" H 4928 7555 50  0000 L CNN
F 2 "Fiducials:Fiducial_1mm_Dia_2.54mm_Outer_CopperTop" H 4950 7450 50  0001 C CNN
F 3 "" H 4800 7600 50  0001 C CNN
	1    4800 7600
	1    0    0    -1  
$EndComp
$Comp
L project:fiducial FID3
U 1 1 6063CCDD
P 4800 7800
F 0 "FID3" H 4928 7846 50  0000 L CNN
F 1 "fiducial" H 4928 7755 50  0000 L CNN
F 2 "Fiducials:Fiducial_1mm_Dia_2.54mm_Outer_CopperTop" H 4950 7650 50  0001 C CNN
F 3 "" H 4800 7800 50  0001 C CNN
	1    4800 7800
	1    0    0    -1  
$EndComp
NoConn ~ 3050 5900
Text Notes 6050 6600 0    50   ~ 0
I2C and SPI Devices \nOperating at 3.3V and 2.8V
Text Notes 5850 4700 0    50   ~ 0
5VP Rail Voltage and Current Sensor
$EndSCHEMATC
