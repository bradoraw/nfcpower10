EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A 11000 8500
encoding utf-8
Sheet 6 11
Title "NFC Power Transmitter"
Date "2020-11-24"
Rev "1"
Comp "NthDegree"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text HLabel 4700 850  1    50   UnSpc ~ 0
VS
Text HLabel 6700 4100 3    50   UnSpc ~ 0
GND
Text HLabel 4550 2550 0    50   UnSpc ~ 0
SCL
Text HLabel 4550 2450 0    50   UnSpc ~ 0
SDA
Text HLabel 3600 5900 2    50   UnSpc ~ 0
IRQ
$Comp
L project:ST25R3916-AQWT U?
U 1 1 5FBD9A74
P 4850 1050
AR Path="/5FBD86FB/5FBD9A74" Ref="U?"  Part="1" 
AR Path="/5FC1DCBA/5FBD9A74" Ref="U?"  Part="1" 
AR Path="/5FC1E11E/5FBD9A74" Ref="U?"  Part="1" 
AR Path="/5FC1E1DD/5FBD9A74" Ref="U?"  Part="1" 
AR Path="/5FC1E32F/5FBD9A74" Ref="U?"  Part="1" 
AR Path="/5FC1FA4C/5FBD9A74" Ref="U?"  Part="1" 
AR Path="/5FC1FA54/5FBD9A74" Ref="U?"  Part="1" 
AR Path="/5FC1FA5C/5FBD9A74" Ref="U?"  Part="1" 
AR Path="/5FC1FA64/5FBD9A74" Ref="U?"  Part="1" 
AR Path="/5FC1FA6C/5FBD9A74" Ref="U?"  Part="1" 
F 0 "U?" H 5650 1315 50  0000 C CNN
F 1 "ST25R3916-AQWT" H 5650 1224 50  0000 C CNN
F 2 "STMicroelectronics-VFQFPN32-B04R-0-1-*" H 4850 1450 50  0001 L CNN
F 3 "https://www.st.com/content/st_com/en/products/nfc/st25-nfc-rfid-tags-readers/st25-nfc-rfid-readers/st25r3916.html#resource" H 4850 1550 50  0001 L CNN
F 4 "No" H 4850 1650 50  0001 L CNN "automotive"
F 5 "IC" H 4850 1750 50  0001 L CNN "category"
F 6 "848kbits/s" H 4850 1850 50  0001 L CNN "data rate"
F 7 "Integrated Circuits (ICs)" H 4850 1950 50  0001 L CNN "device class L1"
F 8 "RF Semiconductors and Devices" H 4850 2050 50  0001 L CNN "device class L2"
F 9 "Transceivers" H 4850 2150 50  0001 L CNN "device class L3"
F 10 "IC RFID READER/TRANSP 32VFQFN" H 4850 2250 50  0001 L CNN "digikey description"
F 11 "497-18366-1-ND" H 4850 2350 50  0001 L CNN "digikey part number"
F 12 "1mm" H 4850 2450 50  0001 L CNN "height"
F 13 "I2C,SPI,Other" H 4850 2550 50  0001 L CNN "interface"
F 14 "QFN50P500X500X90-32" H 4850 2650 50  0001 L CNN "ipc land pattern name"
F 15 "Yes" H 4850 2750 50  0001 L CNN "lead free"
F 16 "80ef14bff18074aa" H 4850 2850 50  0001 L CNN "library id"
F 17 "STMicroelectronics" H 4850 2950 50  0001 L CNN "manufacturer"
F 18 "13.56MHz" H 4850 3050 50  0001 L CNN "max frequency"
F 19 "+125°C" H 4850 3150 50  0001 L CNN "max junction temp"
F 20 "5.5V" H 4850 3250 50  0001 L CNN "max supply voltage"
F 21 "1.65V" H 4850 3350 50  0001 L CNN "min supply voltage"
F 22 "511-ST25R3916-AQWT" H 4850 3450 50  0001 L CNN "mouser part number"
F 23 "1.8-20uA" H 4850 3550 50  0001 L CNN "nominal supply current"
F 24 "VFQFPN32" H 4850 3650 50  0001 L CNN "package"
F 25 "Yes" H 4850 3750 50  0001 L CNN "rohs"
F 26 "0mm" H 4850 3850 50  0001 L CNN "standoff height"
F 27 "+125°C" H 4850 3950 50  0001 L CNN "temperature range high"
F 28 "-40°C" H 4850 4050 50  0001 L CNN "temperature range low"
	1    4850 1050
	1    0    0    -1  
$EndComp
$Comp
L project:CGA3E2C0G1H060D080AA C?
U 1 1 5FBDBDE3
P 4450 3650
AR Path="/5FBD86FB/5FBDBDE3" Ref="C?"  Part="1" 
AR Path="/5FC1DCBA/5FBDBDE3" Ref="C?"  Part="1" 
AR Path="/5FC1E11E/5FBDBDE3" Ref="C?"  Part="1" 
AR Path="/5FC1E1DD/5FBDBDE3" Ref="C?"  Part="1" 
AR Path="/5FC1E32F/5FBDBDE3" Ref="C?"  Part="1" 
AR Path="/5FC1FA4C/5FBDBDE3" Ref="C?"  Part="1" 
AR Path="/5FC1FA54/5FBDBDE3" Ref="C?"  Part="1" 
AR Path="/5FC1FA5C/5FBDBDE3" Ref="C?"  Part="1" 
AR Path="/5FC1FA64/5FBDBDE3" Ref="C?"  Part="1" 
AR Path="/5FC1FA6C/5FBDBDE3" Ref="C?"  Part="1" 
F 0 "C?" H 4578 3687 50  0000 L CNN
F 1 "CGA3E2C0G1H060D080AA" H 4578 3596 50  0001 L CNN
F 2 "TDK-CGA3E-0.3-0.1-0-0-*" H 4450 3950 50  0001 L CNN
F 3 "https://product.tdk.com/info/en/catalog/spec/mlccspec_automotive_general_en.pdf" H 4450 4050 50  0001 L CNN
F 4 "Yes" H 4450 4150 50  0001 L CNN "automotive"
F 5 "Grade 1" H 4450 4250 50  0001 L CNN "automotive grade"
F 6 "6pF" H 4578 3596 50  0000 L CNN "capacitance"
F 7 "Cap" H 4450 4450 50  0001 L CNN "category"
F 8 "Passive Components" H 4450 4550 50  0001 L CNN "device class L1"
F 9 "Capacitors" H 4450 4650 50  0001 L CNN "device class L2"
F 10 "Ceramic Capacitors" H 4450 4750 50  0001 L CNN "device class L3"
F 11 "1.1mm" H 4450 4850 50  0001 L CNN "height"
F 12 "CAPC16080X80" H 4450 4950 50  0001 L CNN "ipc land pattern name"
F 13 "Yes" H 4450 5050 50  0001 L CNN "lead free"
F 14 "9251288ec5c7c3b4" H 4450 5150 50  0001 L CNN "library id"
F 15 "TDK" H 4450 5250 50  0001 L CNN "manufacturer"
F 16 "Ceramic" H 4450 5350 50  0001 L CNN "material"
F 17 "0603" H 4578 3505 50  0000 L CNN "package"
F 18 "Yes" H 4450 5650 50  0001 L CNN "rohs"
F 19 "C0G" H 4450 5750 50  0001 L CNN "temperature characteristic"
F 20 "30ppm/°C" H 4450 5850 50  0001 L CNN "temperature coefficient"
F 21 "+125°C" H 4450 5950 50  0001 L CNN "temperature range high"
F 22 "-55°C" H 4450 6050 50  0001 L CNN "temperature range low"
F 23 "0.5pF" H 4450 6150 50  0001 L CNN "tolerance"
F 24 "50V" H 4578 3414 50  0000 L CNN "voltage rating"
	1    4450 3650
	1    0    0    -1  
$EndComp
Wire Wire Line
	4950 3550 4450 3550
Wire Wire Line
	4450 3550 4450 3650
Wire Wire Line
	6350 2950 6700 2950
Wire Wire Line
	6700 2950 6700 3150
Wire Wire Line
	6350 3150 6700 3150
Connection ~ 6700 3150
Wire Wire Line
	6700 3150 6700 3250
Wire Wire Line
	6350 3250 6700 3250
Connection ~ 6700 3250
Wire Wire Line
	6700 3250 6700 3350
Wire Wire Line
	6350 3350 6700 3350
Connection ~ 6700 3350
Wire Wire Line
	6700 3350 6700 3450
Wire Wire Line
	6350 3450 6700 3450
Connection ~ 6700 3450
Wire Wire Line
	6700 3450 6700 3550
Wire Wire Line
	6350 3550 6700 3550
Connection ~ 6700 3550
Wire Wire Line
	6700 3550 6700 4000
Wire Wire Line
	4450 4000 6700 4000
Wire Wire Line
	4450 3850 4450 4000
Connection ~ 6700 4000
Wire Wire Line
	6700 4000 6700 4100
$Comp
L project:CC0603MRX5R5BB225 C?
U 1 1 5FBE0122
P 6700 2200
AR Path="/5FBD86FB/5FBE0122" Ref="C?"  Part="1" 
AR Path="/5FC1DCBA/5FBE0122" Ref="C?"  Part="1" 
AR Path="/5FC1E11E/5FBE0122" Ref="C?"  Part="1" 
AR Path="/5FC1E1DD/5FBE0122" Ref="C?"  Part="1" 
AR Path="/5FC1E32F/5FBE0122" Ref="C?"  Part="1" 
AR Path="/5FC1FA4C/5FBE0122" Ref="C?"  Part="1" 
AR Path="/5FC1FA54/5FBE0122" Ref="C?"  Part="1" 
AR Path="/5FC1FA5C/5FBE0122" Ref="C?"  Part="1" 
AR Path="/5FC1FA64/5FBE0122" Ref="C?"  Part="1" 
AR Path="/5FC1FA6C/5FBE0122" Ref="C?"  Part="1" 
F 0 "C?" H 6828 2237 50  0000 L CNN
F 1 "CC0603MRX5R5BB225" H 6828 2055 50  0001 L CNN
F 2 "YAGEO-CC0603-DA-05_2018-24-*" H 6700 2500 50  0001 L CNN
F 3 "http://www.yageo.com/documents/recent/UPY-GPHC_X7R_6.3V-to-50V_18.pdf" H 6700 2600 50  0001 L CNN
F 4 "No" H 6700 2700 50  0001 L CNN "automotive"
F 5 "2.2 uF" H 6828 2146 50  0000 L CNN "capacitance"
F 6 "Cap" H 6700 2900 50  0001 L CNN "category"
F 7 "Passive Components" H 6700 3000 50  0001 L CNN "device class L1"
F 8 "Capacitors" H 6700 3100 50  0001 L CNN "device class L2"
F 9 "Ceramic Capacitors" H 6700 3200 50  0001 L CNN "device class L3"
F 10 "CAP CER 2.2UF 6.3V X5R 0603" H 6700 3300 50  0001 L CNN "digikey description"
F 11 "311-1814-2-ND" H 6700 3400 50  0001 L CNN "digikey part number"
F 12 "http://www.yageo.com/exep/pages/download/literatures/UPY-C_GEN_24.pdf" H 6700 3500 50  0001 L CNN "footprint url"
F 13 "0.9mm" H 6700 3600 50  0001 L CNN "height"
F 14 "Yes" H 6700 3700 50  0001 L CNN "lead free"
F 15 "d4a18ccc754c13a9" H 6700 3800 50  0001 L CNN "library id"
F 16 "YAGEO" H 6700 3900 50  0001 L CNN "manufacturer"
F 17 "Ceramic" H 6700 4000 50  0001 L CNN "material"
F 18 "0603 2.2 uF 6.3 V ±20% Tolerance X5R SMT Multilayer Ceramic Capacitor" H 6700 4100 50  0001 L CNN "mouser description"
F 19 "603-CC603MRX5R5BB225" H 6700 4200 50  0001 L CNN "mouser part number"
F 20 "0603" H 6828 2055 50  0000 L CNN "package"
F 21 "Yes" H 6700 4400 50  0001 L CNN "rohs"
F 22 "X5R" H 6700 4500 50  0001 L CNN "temperature characteristic"
F 23 "15%" H 6700 4600 50  0001 L CNN "temperature coefficient"
F 24 "+125°C" H 6700 4700 50  0001 L CNN "temperature range high"
F 25 "-55°C" H 6700 4800 50  0001 L CNN "temperature range low"
F 26 "0.2" H 6700 4900 50  0001 L CNN "tolerance"
F 27 "6.3 V" H 6828 1964 50  0000 L CNN "voltage"
F 28 "6.3 V" H 6700 5100 50  0001 L CNN "voltage rating"
	1    6700 2200
	1    0    0    -1  
$EndComp
NoConn ~ 6350 2350
NoConn ~ 6350 2450
NoConn ~ 6350 2650
NoConn ~ 6350 1750
NoConn ~ 6350 1650
NoConn ~ 6350 1450
NoConn ~ 6350 1350
Wire Wire Line
	6350 2050 6700 2050
Wire Wire Line
	6700 2050 6700 2200
Wire Wire Line
	6700 2400 6700 2600
Connection ~ 6700 2950
$Comp
L project:CC0603KRX7R9BB103 C?
U 1 1 5FBE555E
P 7200 2200
AR Path="/5FBD86FB/5FBE555E" Ref="C?"  Part="1" 
AR Path="/5FC1DCBA/5FBE555E" Ref="C?"  Part="1" 
AR Path="/5FC1E11E/5FBE555E" Ref="C?"  Part="1" 
AR Path="/5FC1E1DD/5FBE555E" Ref="C?"  Part="1" 
AR Path="/5FC1E32F/5FBE555E" Ref="C?"  Part="1" 
AR Path="/5FC1FA4C/5FBE555E" Ref="C?"  Part="1" 
AR Path="/5FC1FA54/5FBE555E" Ref="C?"  Part="1" 
AR Path="/5FC1FA5C/5FBE555E" Ref="C?"  Part="1" 
AR Path="/5FC1FA64/5FBE555E" Ref="C?"  Part="1" 
AR Path="/5FC1FA6C/5FBE555E" Ref="C?"  Part="1" 
F 0 "C?" H 7328 2237 50  0000 L CNN
F 1 "CC0603KRX7R9BB103" H 7200 2400 50  0001 L CNN
F 2 "YAGEO-CC0603-DA-05_2018-24-*" H 7200 2500 50  0001 L CNN
F 3 "http://www.yageo.com/documents/recent/UPY-GPHC_X7R_6.3V-to-50V_18.pdf" H 7200 2600 50  0001 L CNN
F 4 "No" H 7200 2700 50  0001 L CNN "automotive"
F 5 "10 nF" H 7328 2146 50  0000 L CNN "capacitance"
F 6 "Cap" H 7200 2900 50  0001 L CNN "category"
F 7 "Passive Components" H 7200 3000 50  0001 L CNN "device class L1"
F 8 "Capacitors" H 7200 3100 50  0001 L CNN "device class L2"
F 9 "Ceramic Capacitors" H 7200 3200 50  0001 L CNN "device class L3"
F 10 "CAP CER 10000PF 50V X7R 0603" H 7200 3300 50  0001 L CNN "digikey description"
F 11 "311-1085-2-ND" H 7200 3400 50  0001 L CNN "digikey part number"
F 12 "http://www.yageo.com/exep/pages/download/literatures/UPY-C_GEN_24.pdf" H 7200 3500 50  0001 L CNN "footprint url"
F 13 "0.9mm" H 7200 3600 50  0001 L CNN "height"
F 14 "Yes" H 7200 3700 50  0001 L CNN "lead free"
F 15 "2a4c50056f7872c7" H 7200 3800 50  0001 L CNN "library id"
F 16 "YAGEO" H 7200 3900 50  0001 L CNN "manufacturer"
F 17 "Ceramic" H 7200 4000 50  0001 L CNN "material"
F 18 "Multilayer Ceramic Capacitor, CC Series, 0.01 - F, - 10%, X7R, 50 V, 0603 [1608 Metric]" H 7200 4100 50  0001 L CNN "mouser description"
F 19 "603-CC603KRX7R9BB103" H 7200 4200 50  0001 L CNN "mouser part number"
F 20 "0603" H 7328 2055 50  0000 L CNN "package"
F 21 "Yes" H 7200 4400 50  0001 L CNN "rohs"
F 22 "X7R" H 7200 4500 50  0001 L CNN "temperature characteristic"
F 23 "15%" H 7200 4600 50  0001 L CNN "temperature coefficient"
F 24 "+125°C" H 7200 4700 50  0001 L CNN "temperature range high"
F 25 "-55°C" H 7200 4800 50  0001 L CNN "temperature range low"
F 26 "0.1" H 7200 4900 50  0001 L CNN "tolerance"
F 27 "50 V" H 7328 1964 50  0000 L CNN "voltage"
F 28 "50 V" H 7200 5100 50  0001 L CNN "voltage rating"
	1    7200 2200
	1    0    0    -1  
$EndComp
Wire Wire Line
	6700 2050 7200 2050
Wire Wire Line
	7200 2050 7200 2200
Connection ~ 6700 2050
Wire Wire Line
	7200 2400 7200 2600
Wire Wire Line
	7200 2600 6700 2600
Connection ~ 6700 2600
Wire Wire Line
	6700 2600 6700 2950
$Comp
L project:MLJ1608WR27JT000 L?
U 1 1 5FBE783B
P 7050 1050
AR Path="/5FBD86FB/5FBE783B" Ref="L?"  Part="1" 
AR Path="/5FC1DCBA/5FBE783B" Ref="L?"  Part="1" 
AR Path="/5FC1E11E/5FBE783B" Ref="L?"  Part="1" 
AR Path="/5FC1E1DD/5FBE783B" Ref="L?"  Part="1" 
AR Path="/5FC1E32F/5FBE783B" Ref="L?"  Part="1" 
AR Path="/5FC1FA4C/5FBE783B" Ref="L?"  Part="1" 
AR Path="/5FC1FA54/5FBE783B" Ref="L?"  Part="1" 
AR Path="/5FC1FA5C/5FBE783B" Ref="L?"  Part="1" 
AR Path="/5FC1FA64/5FBE783B" Ref="L?"  Part="1" 
AR Path="/5FC1FA6C/5FBE783B" Ref="L?"  Part="1" 
F 0 "L?" H 7050 1424 50  0000 C CNN
F 1 "MLJ1608WR27JT000" H 6850 1600 50  0001 L BNN
F 2 "INDC1608X95N" H 6850 1500 50  0001 L BNN
F 3 "" H 7050 1050 50  0001 C CNN
F 4 "270nH" H 7050 1333 50  0000 C CNN "Inductance"
F 5 "550mA" H 7050 1242 50  0000 C CNN "Current"
F 6 "0603" H 7050 1151 50  0000 C CNN "Package"
F 7 "260MHz" H 7000 1700 50  0001 C CNN "Self Resonance"
	1    7050 1050
	1    0    0    -1  
$EndComp
$Comp
L project:MLJ1608WR27JT000 L?
U 1 1 5FBE93D3
P 7050 1850
AR Path="/5FBD86FB/5FBE93D3" Ref="L?"  Part="1" 
AR Path="/5FC1DCBA/5FBE93D3" Ref="L?"  Part="1" 
AR Path="/5FC1E11E/5FBE93D3" Ref="L?"  Part="1" 
AR Path="/5FC1E1DD/5FBE93D3" Ref="L?"  Part="1" 
AR Path="/5FC1E32F/5FBE93D3" Ref="L?"  Part="1" 
AR Path="/5FC1FA4C/5FBE93D3" Ref="L?"  Part="1" 
AR Path="/5FC1FA54/5FBE93D3" Ref="L?"  Part="1" 
AR Path="/5FC1FA5C/5FBE93D3" Ref="L?"  Part="1" 
AR Path="/5FC1FA64/5FBE93D3" Ref="L?"  Part="1" 
AR Path="/5FC1FA6C/5FBE93D3" Ref="L?"  Part="1" 
F 0 "L?" H 7050 2224 50  0000 C CNN
F 1 "MLJ1608WR27JT000" H 6850 2400 50  0001 L BNN
F 2 "INDC1608X95N" H 6850 2300 50  0001 L BNN
F 3 "" H 7050 1850 50  0001 C CNN
F 4 "270nH" H 7050 2133 50  0000 C CNN "Inductance"
F 5 "550mA" H 7050 2042 50  0000 C CNN "Current"
F 6 "0603" H 7050 1951 50  0000 C CNN "Package"
F 7 "260MHz" H 7000 2500 50  0001 C CNN "Self Resonance"
	1    7050 1850
	1    0    0    -1  
$EndComp
Wire Wire Line
	6350 1050 6750 1050
Wire Wire Line
	6350 1150 6600 1150
Wire Wire Line
	6600 1850 6750 1850
$Comp
L project:CC0603KRX7R9BB102 C?
U 1 1 5FBECF7A
P 7500 1150
AR Path="/5FBD86FB/5FBECF7A" Ref="C?"  Part="1" 
AR Path="/5FC1DCBA/5FBECF7A" Ref="C?"  Part="1" 
AR Path="/5FC1E11E/5FBECF7A" Ref="C?"  Part="1" 
AR Path="/5FC1E1DD/5FBECF7A" Ref="C?"  Part="1" 
AR Path="/5FC1E32F/5FBECF7A" Ref="C?"  Part="1" 
AR Path="/5FC1FA4C/5FBECF7A" Ref="C?"  Part="1" 
AR Path="/5FC1FA54/5FBECF7A" Ref="C?"  Part="1" 
AR Path="/5FC1FA5C/5FBECF7A" Ref="C?"  Part="1" 
AR Path="/5FC1FA64/5FBECF7A" Ref="C?"  Part="1" 
AR Path="/5FC1FA6C/5FBECF7A" Ref="C?"  Part="1" 
F 0 "C?" H 7628 1187 50  0000 L CNN
F 1 "CC0603KRX7R9BB102" H 7500 1350 50  0001 L CNN
F 2 "YAGEO-CC0603-DA-05_2018-24-*" H 7500 1450 50  0001 L CNN
F 3 "http://www.yageo.com/documents/recent/UPY-GPHC_X7R_6.3V-to-50V_18.pdf" H 7500 1550 50  0001 L CNN
F 4 "No" H 7500 1650 50  0001 L CNN "automotive"
F 5 "1.0 nF" H 7628 1096 50  0000 L CNN "capacitance"
F 6 "Cap" H 7500 1850 50  0001 L CNN "category"
F 7 "Passive Components" H 7500 1950 50  0001 L CNN "device class L1"
F 8 "Capacitors" H 7500 2050 50  0001 L CNN "device class L2"
F 9 "Ceramic Capacitors" H 7500 2150 50  0001 L CNN "device class L3"
F 10 "CAP CER 1000PF 50V X7R 0603" H 7500 2250 50  0001 L CNN "digikey description"
F 11 "311-1080-2-ND" H 7500 2350 50  0001 L CNN "digikey part number"
F 12 "http://www.yageo.com/exep/pages/download/literatures/UPY-C_GEN_24.pdf" H 7500 2450 50  0001 L CNN "footprint url"
F 13 "0.9mm" H 7500 2550 50  0001 L CNN "height"
F 14 "Yes" H 7500 2650 50  0001 L CNN "lead free"
F 15 "1aeb194455b4965b" H 7500 2750 50  0001 L CNN "library id"
F 16 "YAGEO" H 7500 2850 50  0001 L CNN "manufacturer"
F 17 "Ceramic" H 7500 2950 50  0001 L CNN "material"
F 18 "Multilayer Ceramic Capacitors MLCC - SMD/SMT 1.0nF 50V X7R 10%" H 7500 3050 50  0001 L CNN "mouser description"
F 19 "603-CC603KRX7R9BB102" H 7500 3150 50  0001 L CNN "mouser part number"
F 20 "0603" H 7628 1005 50  0000 L CNN "package"
F 21 "Yes" H 7500 3350 50  0001 L CNN "rohs"
F 22 "X7R" H 7500 3450 50  0001 L CNN "temperature characteristic"
F 23 "15%" H 7500 3550 50  0001 L CNN "temperature coefficient"
F 24 "+125°C" H 7500 3650 50  0001 L CNN "temperature range high"
F 25 "-55°C" H 7500 3750 50  0001 L CNN "temperature range low"
F 26 "0.1" H 7500 3850 50  0001 L CNN "tolerance"
F 27 "50 V" H 7628 914 50  0000 L CNN "voltage"
F 28 "50 V" H 7500 4050 50  0001 L CNN "voltage rating"
	1    7500 1150
	1    0    0    -1  
$EndComp
$Comp
L project:CC0603KRX7R9BB102 C?
U 1 1 5FBEFCB6
P 7500 1550
AR Path="/5FBD86FB/5FBEFCB6" Ref="C?"  Part="1" 
AR Path="/5FC1DCBA/5FBEFCB6" Ref="C?"  Part="1" 
AR Path="/5FC1E11E/5FBEFCB6" Ref="C?"  Part="1" 
AR Path="/5FC1E1DD/5FBEFCB6" Ref="C?"  Part="1" 
AR Path="/5FC1E32F/5FBEFCB6" Ref="C?"  Part="1" 
AR Path="/5FC1FA4C/5FBEFCB6" Ref="C?"  Part="1" 
AR Path="/5FC1FA54/5FBEFCB6" Ref="C?"  Part="1" 
AR Path="/5FC1FA5C/5FBEFCB6" Ref="C?"  Part="1" 
AR Path="/5FC1FA64/5FBEFCB6" Ref="C?"  Part="1" 
AR Path="/5FC1FA6C/5FBEFCB6" Ref="C?"  Part="1" 
F 0 "C?" H 7628 1587 50  0000 L CNN
F 1 "CC0603KRX7R9BB102" H 7500 1750 50  0001 L CNN
F 2 "YAGEO-CC0603-DA-05_2018-24-*" H 7500 1850 50  0001 L CNN
F 3 "http://www.yageo.com/documents/recent/UPY-GPHC_X7R_6.3V-to-50V_18.pdf" H 7500 1950 50  0001 L CNN
F 4 "No" H 7500 2050 50  0001 L CNN "automotive"
F 5 "1.0 nF" H 7628 1496 50  0000 L CNN "capacitance"
F 6 "Cap" H 7500 2250 50  0001 L CNN "category"
F 7 "Passive Components" H 7500 2350 50  0001 L CNN "device class L1"
F 8 "Capacitors" H 7500 2450 50  0001 L CNN "device class L2"
F 9 "Ceramic Capacitors" H 7500 2550 50  0001 L CNN "device class L3"
F 10 "CAP CER 1000PF 50V X7R 0603" H 7500 2650 50  0001 L CNN "digikey description"
F 11 "311-1080-2-ND" H 7500 2750 50  0001 L CNN "digikey part number"
F 12 "http://www.yageo.com/exep/pages/download/literatures/UPY-C_GEN_24.pdf" H 7500 2850 50  0001 L CNN "footprint url"
F 13 "0.9mm" H 7500 2950 50  0001 L CNN "height"
F 14 "Yes" H 7500 3050 50  0001 L CNN "lead free"
F 15 "1aeb194455b4965b" H 7500 3150 50  0001 L CNN "library id"
F 16 "YAGEO" H 7500 3250 50  0001 L CNN "manufacturer"
F 17 "Ceramic" H 7500 3350 50  0001 L CNN "material"
F 18 "Multilayer Ceramic Capacitors MLCC - SMD/SMT 1.0nF 50V X7R 10%" H 7500 3450 50  0001 L CNN "mouser description"
F 19 "603-CC603KRX7R9BB102" H 7500 3550 50  0001 L CNN "mouser part number"
F 20 "0603" H 7628 1405 50  0000 L CNN "package"
F 21 "Yes" H 7500 3750 50  0001 L CNN "rohs"
F 22 "X7R" H 7500 3850 50  0001 L CNN "temperature characteristic"
F 23 "15%" H 7500 3950 50  0001 L CNN "temperature coefficient"
F 24 "+125°C" H 7500 4050 50  0001 L CNN "temperature range high"
F 25 "-55°C" H 7500 4150 50  0001 L CNN "temperature range low"
F 26 "0.1" H 7500 4250 50  0001 L CNN "tolerance"
F 27 "50 V" H 7628 1314 50  0000 L CNN "voltage"
F 28 "50 V" H 7500 4450 50  0001 L CNN "voltage rating"
	1    7500 1550
	1    0    0    -1  
$EndComp
Wire Wire Line
	7350 1850 7500 1850
Wire Wire Line
	7500 1550 7500 1450
Wire Wire Line
	7350 1050 7500 1050
Wire Wire Line
	7500 1450 7300 1450
Connection ~ 7500 1450
Wire Wire Line
	7500 1450 7500 1350
Text Label 7300 1450 0    50   ~ 0
GND
$Comp
L project:Coil_49_18_6T_2Layer Y?
U 1 1 5FBF2B0F
P 8750 1650
AR Path="/5FBD86FB/5FBF2B0F" Ref="Y?"  Part="1" 
AR Path="/5FC1DCBA/5FBF2B0F" Ref="Y?"  Part="1" 
AR Path="/5FC1E11E/5FBF2B0F" Ref="Y?"  Part="1" 
AR Path="/5FC1E1DD/5FBF2B0F" Ref="Y?"  Part="1" 
AR Path="/5FC1E32F/5FBF2B0F" Ref="Y?"  Part="1" 
AR Path="/5FC1FA4C/5FBF2B0F" Ref="Y?"  Part="1" 
AR Path="/5FC1FA54/5FBF2B0F" Ref="Y?"  Part="1" 
AR Path="/5FC1FA5C/5FBF2B0F" Ref="Y?"  Part="1" 
AR Path="/5FC1FA64/5FBF2B0F" Ref="Y?"  Part="1" 
AR Path="/5FC1FA6C/5FBF2B0F" Ref="Y?"  Part="1" 
F 0 "Y?" H 9278 1696 50  0000 L CNN
F 1 "Coil_49_18_6T_2Layer" H 9278 1605 50  0000 L CNN
F 2 "NFCPower:Coil_49_18_6T_2Layer" H 9750 1450 50  0001 C CNN
F 3 "" H 8750 1650 50  0001 C CNN
	1    8750 1650
	1    0    0    -1  
$EndComp
Wire Wire Line
	7500 1050 7500 1150
Wire Wire Line
	7500 1750 7500 1850
Wire Wire Line
	6600 1150 6600 1850
Wire Wire Line
	8750 1700 8600 1700
Wire Wire Line
	8600 1850 7500 1850
Connection ~ 7500 1850
Wire Wire Line
	4950 1650 4850 1650
Wire Wire Line
	4850 1650 4850 1450
Wire Wire Line
	4850 1450 4950 1450
Wire Wire Line
	4950 1750 4700 1750
Wire Wire Line
	4700 1750 4700 1550
Wire Wire Line
	4700 1550 4950 1550
Wire Wire Line
	4950 1050 4700 1050
Wire Wire Line
	4700 1050 4700 1550
Connection ~ 4700 1550
NoConn ~ 4950 2050
NoConn ~ 4950 2150
NoConn ~ 4950 2350
NoConn ~ 4950 3050
Text HLabel 3200 3450 0    50   Input ~ 0
CLK
$Comp
L project:CGA3E2C0G1H060D080AA C?
U 1 1 5FBFF6D7
P 3950 3650
AR Path="/5FBD86FB/5FBFF6D7" Ref="C?"  Part="1" 
AR Path="/5FC1DCBA/5FBFF6D7" Ref="C?"  Part="1" 
AR Path="/5FC1E11E/5FBFF6D7" Ref="C?"  Part="1" 
AR Path="/5FC1E1DD/5FBFF6D7" Ref="C?"  Part="1" 
AR Path="/5FC1E32F/5FBFF6D7" Ref="C?"  Part="1" 
AR Path="/5FC1FA4C/5FBFF6D7" Ref="C?"  Part="1" 
AR Path="/5FC1FA54/5FBFF6D7" Ref="C?"  Part="1" 
AR Path="/5FC1FA5C/5FBFF6D7" Ref="C?"  Part="1" 
AR Path="/5FC1FA64/5FBFF6D7" Ref="C?"  Part="1" 
AR Path="/5FC1FA6C/5FBFF6D7" Ref="C?"  Part="1" 
F 0 "C?" H 4078 3687 50  0000 L CNN
F 1 "CGA3E2C0G1H060D080AA" H 4078 3596 50  0001 L CNN
F 2 "TDK-CGA3E-0.3-0.1-0-0-*" H 3950 3950 50  0001 L CNN
F 3 "https://product.tdk.com/info/en/catalog/spec/mlccspec_automotive_general_en.pdf" H 3950 4050 50  0001 L CNN
F 4 "Yes" H 3950 4150 50  0001 L CNN "automotive"
F 5 "Grade 1" H 3950 4250 50  0001 L CNN "automotive grade"
F 6 "6pF" H 4078 3596 50  0000 L CNN "capacitance"
F 7 "Cap" H 3950 4450 50  0001 L CNN "category"
F 8 "Passive Components" H 3950 4550 50  0001 L CNN "device class L1"
F 9 "Capacitors" H 3950 4650 50  0001 L CNN "device class L2"
F 10 "Ceramic Capacitors" H 3950 4750 50  0001 L CNN "device class L3"
F 11 "1.1mm" H 3950 4850 50  0001 L CNN "height"
F 12 "CAPC16080X80" H 3950 4950 50  0001 L CNN "ipc land pattern name"
F 13 "Yes" H 3950 5050 50  0001 L CNN "lead free"
F 14 "9251288ec5c7c3b4" H 3950 5150 50  0001 L CNN "library id"
F 15 "TDK" H 3950 5250 50  0001 L CNN "manufacturer"
F 16 "Ceramic" H 3950 5350 50  0001 L CNN "material"
F 17 "0603" H 4078 3505 50  0000 L CNN "package"
F 18 "Yes" H 3950 5650 50  0001 L CNN "rohs"
F 19 "C0G" H 3950 5750 50  0001 L CNN "temperature characteristic"
F 20 "30ppm/°C" H 3950 5850 50  0001 L CNN "temperature coefficient"
F 21 "+125°C" H 3950 5950 50  0001 L CNN "temperature range high"
F 22 "-55°C" H 3950 6050 50  0001 L CNN "temperature range low"
F 23 "0.5pF" H 3950 6150 50  0001 L CNN "tolerance"
F 24 "50V" H 4078 3414 50  0000 L CNN "voltage rating"
	1    3950 3650
	1    0    0    -1  
$EndComp
Wire Wire Line
	4450 4000 3950 4000
Wire Wire Line
	3950 4000 3950 3850
Connection ~ 4450 4000
Wire Wire Line
	4950 3450 3950 3450
Wire Wire Line
	3950 3450 3950 3650
Wire Wire Line
	3200 3450 3950 3450
Connection ~ 3950 3450
Wire Wire Line
	4700 1050 4700 850 
Connection ~ 4700 1050
$Comp
L project:CC0603KRX7R9BB181 C?
U 1 1 5FC15023
P 8600 1150
AR Path="/5FBD86FB/5FC15023" Ref="C?"  Part="1" 
AR Path="/5FC1DCBA/5FC15023" Ref="C?"  Part="1" 
AR Path="/5FC1E11E/5FC15023" Ref="C?"  Part="1" 
AR Path="/5FC1E1DD/5FC15023" Ref="C?"  Part="1" 
AR Path="/5FC1E32F/5FC15023" Ref="C?"  Part="1" 
AR Path="/5FC1FA4C/5FC15023" Ref="C?"  Part="1" 
AR Path="/5FC1FA54/5FC15023" Ref="C?"  Part="1" 
AR Path="/5FC1FA5C/5FC15023" Ref="C?"  Part="1" 
AR Path="/5FC1FA64/5FC15023" Ref="C?"  Part="1" 
AR Path="/5FC1FA6C/5FC15023" Ref="C?"  Part="1" 
F 0 "C?" H 8728 1187 50  0000 L CNN
F 1 "CC0603KRX7R9BB181" H 8600 1350 50  0001 L CNN
F 2 "YAGEO-CC0603-DA-05_2018-24-*" H 8600 1450 50  0001 L CNN
F 3 "http://www.yageo.com/documents/recent/UPY-GPHC_X7R_6.3V-to-50V_18.pdf" H 8600 1550 50  0001 L CNN
F 4 "No" H 8600 1650 50  0001 L CNN "automotive"
F 5 "180 pF" H 8728 1096 50  0000 L CNN "capacitance"
F 6 "Cap" H 8600 1850 50  0001 L CNN "category"
F 7 "Passive Components" H 8600 1950 50  0001 L CNN "device class L1"
F 8 "Capacitors" H 8600 2050 50  0001 L CNN "device class L2"
F 9 "Ceramic Capacitors" H 8600 2150 50  0001 L CNN "device class L3"
F 10 "CAP CER 180PF 50V X7R 0603" H 8600 2250 50  0001 L CNN "digikey description"
F 11 "CC0603KRX7R9BB181-ND" H 8600 2350 50  0001 L CNN "digikey part number"
F 12 "http://www.yageo.com/exep/pages/download/literatures/UPY-C_GEN_24.pdf" H 8600 2450 50  0001 L CNN "footprint url"
F 13 "0.9mm" H 8600 2550 50  0001 L CNN "height"
F 14 "Yes" H 8600 2650 50  0001 L CNN "lead free"
F 15 "118bd706f38603c6" H 8600 2750 50  0001 L CNN "library id"
F 16 "YAGEO" H 8600 2850 50  0001 L CNN "manufacturer"
F 17 "Ceramic" H 8600 2950 50  0001 L CNN "material"
F 18 "Multilayer Ceramic Capacitors MLCC - SMD/SMT 180pF 50V X7R 10%" H 8600 3050 50  0001 L CNN "mouser description"
F 19 "603-CC603KRX7R9BB181" H 8600 3150 50  0001 L CNN "mouser part number"
F 20 "0603" H 8728 1005 50  0000 L CNN "package"
F 21 "Yes" H 8600 3350 50  0001 L CNN "rohs"
F 22 "X7R" H 8600 3450 50  0001 L CNN "temperature characteristic"
F 23 "15%" H 8600 3550 50  0001 L CNN "temperature coefficient"
F 24 "+125°C" H 8600 3650 50  0001 L CNN "temperature range high"
F 25 "-55°C" H 8600 3750 50  0001 L CNN "temperature range low"
F 26 "0.1" H 8600 3850 50  0001 L CNN "tolerance"
F 27 "50 V" H 8728 914 50  0000 L CNN "voltage"
F 28 "50 V" H 8600 4050 50  0001 L CNN "voltage rating"
	1    8600 1150
	1    0    0    -1  
$EndComp
Wire Wire Line
	8600 1700 8600 1850
Wire Wire Line
	8750 1600 8600 1600
Wire Wire Line
	8600 1600 8600 1350
Wire Wire Line
	8600 1150 8600 1050
Wire Wire Line
	8600 1050 7500 1050
Connection ~ 7500 1050
$Comp
L project:CC0603MRX5R5BB225 C?
U 1 1 5FC212B3
P 1250 1400
AR Path="/5FBD86FB/5FC212B3" Ref="C?"  Part="1" 
AR Path="/5FC1DCBA/5FC212B3" Ref="C?"  Part="1" 
AR Path="/5FC1E11E/5FC212B3" Ref="C?"  Part="1" 
AR Path="/5FC1E1DD/5FC212B3" Ref="C?"  Part="1" 
AR Path="/5FC1E32F/5FC212B3" Ref="C?"  Part="1" 
AR Path="/5FC1FA4C/5FC212B3" Ref="C?"  Part="1" 
AR Path="/5FC1FA54/5FC212B3" Ref="C?"  Part="1" 
AR Path="/5FC1FA5C/5FC212B3" Ref="C?"  Part="1" 
AR Path="/5FC1FA64/5FC212B3" Ref="C?"  Part="1" 
AR Path="/5FC1FA6C/5FC212B3" Ref="C?"  Part="1" 
F 0 "C?" H 1378 1437 50  0000 L CNN
F 1 "CC0603MRX5R5BB225" H 1378 1255 50  0001 L CNN
F 2 "YAGEO-CC0603-DA-05_2018-24-*" H 1250 1700 50  0001 L CNN
F 3 "http://www.yageo.com/documents/recent/UPY-GPHC_X7R_6.3V-to-50V_18.pdf" H 1250 1800 50  0001 L CNN
F 4 "No" H 1250 1900 50  0001 L CNN "automotive"
F 5 "2.2 uF" H 1378 1346 50  0000 L CNN "capacitance"
F 6 "Cap" H 1250 2100 50  0001 L CNN "category"
F 7 "Passive Components" H 1250 2200 50  0001 L CNN "device class L1"
F 8 "Capacitors" H 1250 2300 50  0001 L CNN "device class L2"
F 9 "Ceramic Capacitors" H 1250 2400 50  0001 L CNN "device class L3"
F 10 "CAP CER 2.2UF 6.3V X5R 0603" H 1250 2500 50  0001 L CNN "digikey description"
F 11 "311-1814-2-ND" H 1250 2600 50  0001 L CNN "digikey part number"
F 12 "http://www.yageo.com/exep/pages/download/literatures/UPY-C_GEN_24.pdf" H 1250 2700 50  0001 L CNN "footprint url"
F 13 "0.9mm" H 1250 2800 50  0001 L CNN "height"
F 14 "Yes" H 1250 2900 50  0001 L CNN "lead free"
F 15 "d4a18ccc754c13a9" H 1250 3000 50  0001 L CNN "library id"
F 16 "YAGEO" H 1250 3100 50  0001 L CNN "manufacturer"
F 17 "Ceramic" H 1250 3200 50  0001 L CNN "material"
F 18 "0603 2.2 uF 6.3 V ±20% Tolerance X5R SMT Multilayer Ceramic Capacitor" H 1250 3300 50  0001 L CNN "mouser description"
F 19 "603-CC603MRX5R5BB225" H 1250 3400 50  0001 L CNN "mouser part number"
F 20 "0603" H 1378 1255 50  0000 L CNN "package"
F 21 "Yes" H 1250 3600 50  0001 L CNN "rohs"
F 22 "X5R" H 1250 3700 50  0001 L CNN "temperature characteristic"
F 23 "15%" H 1250 3800 50  0001 L CNN "temperature coefficient"
F 24 "+125°C" H 1250 3900 50  0001 L CNN "temperature range high"
F 25 "-55°C" H 1250 4000 50  0001 L CNN "temperature range low"
F 26 "0.2" H 1250 4100 50  0001 L CNN "tolerance"
F 27 "6.3 V" H 1378 1164 50  0000 L CNN "voltage"
F 28 "6.3 V" H 1250 4300 50  0001 L CNN "voltage rating"
	1    1250 1400
	1    0    0    -1  
$EndComp
Wire Wire Line
	1250 1250 1250 1400
Wire Wire Line
	1250 1600 1250 1800
$Comp
L project:CC0603KRX7R9BB103 C?
U 1 1 5FC212D5
P 1750 1400
AR Path="/5FBD86FB/5FC212D5" Ref="C?"  Part="1" 
AR Path="/5FC1DCBA/5FC212D5" Ref="C?"  Part="1" 
AR Path="/5FC1E11E/5FC212D5" Ref="C?"  Part="1" 
AR Path="/5FC1E1DD/5FC212D5" Ref="C?"  Part="1" 
AR Path="/5FC1E32F/5FC212D5" Ref="C?"  Part="1" 
AR Path="/5FC1FA4C/5FC212D5" Ref="C?"  Part="1" 
AR Path="/5FC1FA54/5FC212D5" Ref="C?"  Part="1" 
AR Path="/5FC1FA5C/5FC212D5" Ref="C?"  Part="1" 
AR Path="/5FC1FA64/5FC212D5" Ref="C?"  Part="1" 
AR Path="/5FC1FA6C/5FC212D5" Ref="C?"  Part="1" 
F 0 "C?" H 1878 1437 50  0000 L CNN
F 1 "CC0603KRX7R9BB103" H 1750 1600 50  0001 L CNN
F 2 "YAGEO-CC0603-DA-05_2018-24-*" H 1750 1700 50  0001 L CNN
F 3 "http://www.yageo.com/documents/recent/UPY-GPHC_X7R_6.3V-to-50V_18.pdf" H 1750 1800 50  0001 L CNN
F 4 "No" H 1750 1900 50  0001 L CNN "automotive"
F 5 "10 nF" H 1878 1346 50  0000 L CNN "capacitance"
F 6 "Cap" H 1750 2100 50  0001 L CNN "category"
F 7 "Passive Components" H 1750 2200 50  0001 L CNN "device class L1"
F 8 "Capacitors" H 1750 2300 50  0001 L CNN "device class L2"
F 9 "Ceramic Capacitors" H 1750 2400 50  0001 L CNN "device class L3"
F 10 "CAP CER 10000PF 50V X7R 0603" H 1750 2500 50  0001 L CNN "digikey description"
F 11 "311-1085-2-ND" H 1750 2600 50  0001 L CNN "digikey part number"
F 12 "http://www.yageo.com/exep/pages/download/literatures/UPY-C_GEN_24.pdf" H 1750 2700 50  0001 L CNN "footprint url"
F 13 "0.9mm" H 1750 2800 50  0001 L CNN "height"
F 14 "Yes" H 1750 2900 50  0001 L CNN "lead free"
F 15 "2a4c50056f7872c7" H 1750 3000 50  0001 L CNN "library id"
F 16 "YAGEO" H 1750 3100 50  0001 L CNN "manufacturer"
F 17 "Ceramic" H 1750 3200 50  0001 L CNN "material"
F 18 "Multilayer Ceramic Capacitor, CC Series, 0.01 - F, - 10%, X7R, 50 V, 0603 [1608 Metric]" H 1750 3300 50  0001 L CNN "mouser description"
F 19 "603-CC603KRX7R9BB103" H 1750 3400 50  0001 L CNN "mouser part number"
F 20 "0603" H 1878 1255 50  0000 L CNN "package"
F 21 "Yes" H 1750 3600 50  0001 L CNN "rohs"
F 22 "X7R" H 1750 3700 50  0001 L CNN "temperature characteristic"
F 23 "15%" H 1750 3800 50  0001 L CNN "temperature coefficient"
F 24 "+125°C" H 1750 3900 50  0001 L CNN "temperature range high"
F 25 "-55°C" H 1750 4000 50  0001 L CNN "temperature range low"
F 26 "0.1" H 1750 4100 50  0001 L CNN "tolerance"
F 27 "50 V" H 1878 1164 50  0000 L CNN "voltage"
F 28 "50 V" H 1750 4300 50  0001 L CNN "voltage rating"
	1    1750 1400
	1    0    0    -1  
$EndComp
Wire Wire Line
	1250 1250 1750 1250
Wire Wire Line
	1750 1250 1750 1400
Wire Wire Line
	1750 1600 1750 1800
Wire Wire Line
	1750 1800 1250 1800
$Comp
L project:CC0603MRX5R5BB225 C?
U 1 1 5FC2C871
P 3250 1400
AR Path="/5FBD86FB/5FC2C871" Ref="C?"  Part="1" 
AR Path="/5FC1DCBA/5FC2C871" Ref="C?"  Part="1" 
AR Path="/5FC1E11E/5FC2C871" Ref="C?"  Part="1" 
AR Path="/5FC1E1DD/5FC2C871" Ref="C?"  Part="1" 
AR Path="/5FC1E32F/5FC2C871" Ref="C?"  Part="1" 
AR Path="/5FC1FA4C/5FC2C871" Ref="C?"  Part="1" 
AR Path="/5FC1FA54/5FC2C871" Ref="C?"  Part="1" 
AR Path="/5FC1FA5C/5FC2C871" Ref="C?"  Part="1" 
AR Path="/5FC1FA64/5FC2C871" Ref="C?"  Part="1" 
AR Path="/5FC1FA6C/5FC2C871" Ref="C?"  Part="1" 
F 0 "C?" H 3378 1437 50  0000 L CNN
F 1 "CC0603MRX5R5BB225" H 3378 1255 50  0001 L CNN
F 2 "YAGEO-CC0603-DA-05_2018-24-*" H 3250 1700 50  0001 L CNN
F 3 "http://www.yageo.com/documents/recent/UPY-GPHC_X7R_6.3V-to-50V_18.pdf" H 3250 1800 50  0001 L CNN
F 4 "No" H 3250 1900 50  0001 L CNN "automotive"
F 5 "2.2 uF" H 3378 1346 50  0000 L CNN "capacitance"
F 6 "Cap" H 3250 2100 50  0001 L CNN "category"
F 7 "Passive Components" H 3250 2200 50  0001 L CNN "device class L1"
F 8 "Capacitors" H 3250 2300 50  0001 L CNN "device class L2"
F 9 "Ceramic Capacitors" H 3250 2400 50  0001 L CNN "device class L3"
F 10 "CAP CER 2.2UF 6.3V X5R 0603" H 3250 2500 50  0001 L CNN "digikey description"
F 11 "311-1814-2-ND" H 3250 2600 50  0001 L CNN "digikey part number"
F 12 "http://www.yageo.com/exep/pages/download/literatures/UPY-C_GEN_24.pdf" H 3250 2700 50  0001 L CNN "footprint url"
F 13 "0.9mm" H 3250 2800 50  0001 L CNN "height"
F 14 "Yes" H 3250 2900 50  0001 L CNN "lead free"
F 15 "d4a18ccc754c13a9" H 3250 3000 50  0001 L CNN "library id"
F 16 "YAGEO" H 3250 3100 50  0001 L CNN "manufacturer"
F 17 "Ceramic" H 3250 3200 50  0001 L CNN "material"
F 18 "0603 2.2 uF 6.3 V ±20% Tolerance X5R SMT Multilayer Ceramic Capacitor" H 3250 3300 50  0001 L CNN "mouser description"
F 19 "603-CC603MRX5R5BB225" H 3250 3400 50  0001 L CNN "mouser part number"
F 20 "0603" H 3378 1255 50  0000 L CNN "package"
F 21 "Yes" H 3250 3600 50  0001 L CNN "rohs"
F 22 "X5R" H 3250 3700 50  0001 L CNN "temperature characteristic"
F 23 "15%" H 3250 3800 50  0001 L CNN "temperature coefficient"
F 24 "+125°C" H 3250 3900 50  0001 L CNN "temperature range high"
F 25 "-55°C" H 3250 4000 50  0001 L CNN "temperature range low"
F 26 "0.2" H 3250 4100 50  0001 L CNN "tolerance"
F 27 "6.3 V" H 3378 1164 50  0000 L CNN "voltage"
F 28 "6.3 V" H 3250 4300 50  0001 L CNN "voltage rating"
	1    3250 1400
	1    0    0    -1  
$EndComp
Wire Wire Line
	3250 1250 3250 1400
Wire Wire Line
	3250 1600 3250 1800
$Comp
L project:CC0603KRX7R9BB103 C?
U 1 1 5FC2C893
P 3750 1400
AR Path="/5FBD86FB/5FC2C893" Ref="C?"  Part="1" 
AR Path="/5FC1DCBA/5FC2C893" Ref="C?"  Part="1" 
AR Path="/5FC1E11E/5FC2C893" Ref="C?"  Part="1" 
AR Path="/5FC1E1DD/5FC2C893" Ref="C?"  Part="1" 
AR Path="/5FC1E32F/5FC2C893" Ref="C?"  Part="1" 
AR Path="/5FC1FA4C/5FC2C893" Ref="C?"  Part="1" 
AR Path="/5FC1FA54/5FC2C893" Ref="C?"  Part="1" 
AR Path="/5FC1FA5C/5FC2C893" Ref="C?"  Part="1" 
AR Path="/5FC1FA64/5FC2C893" Ref="C?"  Part="1" 
AR Path="/5FC1FA6C/5FC2C893" Ref="C?"  Part="1" 
F 0 "C?" H 3878 1437 50  0000 L CNN
F 1 "CC0603KRX7R9BB103" H 3750 1600 50  0001 L CNN
F 2 "YAGEO-CC0603-DA-05_2018-24-*" H 3750 1700 50  0001 L CNN
F 3 "http://www.yageo.com/documents/recent/UPY-GPHC_X7R_6.3V-to-50V_18.pdf" H 3750 1800 50  0001 L CNN
F 4 "No" H 3750 1900 50  0001 L CNN "automotive"
F 5 "10 nF" H 3878 1346 50  0000 L CNN "capacitance"
F 6 "Cap" H 3750 2100 50  0001 L CNN "category"
F 7 "Passive Components" H 3750 2200 50  0001 L CNN "device class L1"
F 8 "Capacitors" H 3750 2300 50  0001 L CNN "device class L2"
F 9 "Ceramic Capacitors" H 3750 2400 50  0001 L CNN "device class L3"
F 10 "CAP CER 10000PF 50V X7R 0603" H 3750 2500 50  0001 L CNN "digikey description"
F 11 "311-1085-2-ND" H 3750 2600 50  0001 L CNN "digikey part number"
F 12 "http://www.yageo.com/exep/pages/download/literatures/UPY-C_GEN_24.pdf" H 3750 2700 50  0001 L CNN "footprint url"
F 13 "0.9mm" H 3750 2800 50  0001 L CNN "height"
F 14 "Yes" H 3750 2900 50  0001 L CNN "lead free"
F 15 "2a4c50056f7872c7" H 3750 3000 50  0001 L CNN "library id"
F 16 "YAGEO" H 3750 3100 50  0001 L CNN "manufacturer"
F 17 "Ceramic" H 3750 3200 50  0001 L CNN "material"
F 18 "Multilayer Ceramic Capacitor, CC Series, 0.01 - F, - 10%, X7R, 50 V, 0603 [1608 Metric]" H 3750 3300 50  0001 L CNN "mouser description"
F 19 "603-CC603KRX7R9BB103" H 3750 3400 50  0001 L CNN "mouser part number"
F 20 "0603" H 3878 1255 50  0000 L CNN "package"
F 21 "Yes" H 3750 3600 50  0001 L CNN "rohs"
F 22 "X7R" H 3750 3700 50  0001 L CNN "temperature characteristic"
F 23 "15%" H 3750 3800 50  0001 L CNN "temperature coefficient"
F 24 "+125°C" H 3750 3900 50  0001 L CNN "temperature range high"
F 25 "-55°C" H 3750 4000 50  0001 L CNN "temperature range low"
F 26 "0.1" H 3750 4100 50  0001 L CNN "tolerance"
F 27 "50 V" H 3878 1164 50  0000 L CNN "voltage"
F 28 "50 V" H 3750 4300 50  0001 L CNN "voltage rating"
	1    3750 1400
	1    0    0    -1  
$EndComp
Wire Wire Line
	3250 1250 3750 1250
Wire Wire Line
	3750 1250 3750 1400
Wire Wire Line
	3750 1600 3750 1800
Wire Wire Line
	3750 1800 3250 1800
$Comp
L project:CC0603MRX5R5BB225 C?
U 1 1 5FC32725
P 2250 1400
AR Path="/5FBD86FB/5FC32725" Ref="C?"  Part="1" 
AR Path="/5FC1DCBA/5FC32725" Ref="C?"  Part="1" 
AR Path="/5FC1E11E/5FC32725" Ref="C?"  Part="1" 
AR Path="/5FC1E1DD/5FC32725" Ref="C?"  Part="1" 
AR Path="/5FC1E32F/5FC32725" Ref="C?"  Part="1" 
AR Path="/5FC1FA4C/5FC32725" Ref="C?"  Part="1" 
AR Path="/5FC1FA54/5FC32725" Ref="C?"  Part="1" 
AR Path="/5FC1FA5C/5FC32725" Ref="C?"  Part="1" 
AR Path="/5FC1FA64/5FC32725" Ref="C?"  Part="1" 
AR Path="/5FC1FA6C/5FC32725" Ref="C?"  Part="1" 
F 0 "C?" H 2378 1437 50  0000 L CNN
F 1 "CC0603MRX5R5BB225" H 2378 1255 50  0001 L CNN
F 2 "YAGEO-CC0603-DA-05_2018-24-*" H 2250 1700 50  0001 L CNN
F 3 "http://www.yageo.com/documents/recent/UPY-GPHC_X7R_6.3V-to-50V_18.pdf" H 2250 1800 50  0001 L CNN
F 4 "No" H 2250 1900 50  0001 L CNN "automotive"
F 5 "2.2 uF" H 2378 1346 50  0000 L CNN "capacitance"
F 6 "Cap" H 2250 2100 50  0001 L CNN "category"
F 7 "Passive Components" H 2250 2200 50  0001 L CNN "device class L1"
F 8 "Capacitors" H 2250 2300 50  0001 L CNN "device class L2"
F 9 "Ceramic Capacitors" H 2250 2400 50  0001 L CNN "device class L3"
F 10 "CAP CER 2.2UF 6.3V X5R 0603" H 2250 2500 50  0001 L CNN "digikey description"
F 11 "311-1814-2-ND" H 2250 2600 50  0001 L CNN "digikey part number"
F 12 "http://www.yageo.com/exep/pages/download/literatures/UPY-C_GEN_24.pdf" H 2250 2700 50  0001 L CNN "footprint url"
F 13 "0.9mm" H 2250 2800 50  0001 L CNN "height"
F 14 "Yes" H 2250 2900 50  0001 L CNN "lead free"
F 15 "d4a18ccc754c13a9" H 2250 3000 50  0001 L CNN "library id"
F 16 "YAGEO" H 2250 3100 50  0001 L CNN "manufacturer"
F 17 "Ceramic" H 2250 3200 50  0001 L CNN "material"
F 18 "0603 2.2 uF 6.3 V ±20% Tolerance X5R SMT Multilayer Ceramic Capacitor" H 2250 3300 50  0001 L CNN "mouser description"
F 19 "603-CC603MRX5R5BB225" H 2250 3400 50  0001 L CNN "mouser part number"
F 20 "0603" H 2378 1255 50  0000 L CNN "package"
F 21 "Yes" H 2250 3600 50  0001 L CNN "rohs"
F 22 "X5R" H 2250 3700 50  0001 L CNN "temperature characteristic"
F 23 "15%" H 2250 3800 50  0001 L CNN "temperature coefficient"
F 24 "+125°C" H 2250 3900 50  0001 L CNN "temperature range high"
F 25 "-55°C" H 2250 4000 50  0001 L CNN "temperature range low"
F 26 "0.2" H 2250 4100 50  0001 L CNN "tolerance"
F 27 "6.3 V" H 2378 1164 50  0000 L CNN "voltage"
F 28 "6.3 V" H 2250 4300 50  0001 L CNN "voltage rating"
	1    2250 1400
	1    0    0    -1  
$EndComp
Wire Wire Line
	2250 1250 2250 1400
Wire Wire Line
	2250 1600 2250 1800
$Comp
L project:CC0603KRX7R9BB103 C?
U 1 1 5FC32746
P 2750 1400
AR Path="/5FBD86FB/5FC32746" Ref="C?"  Part="1" 
AR Path="/5FC1DCBA/5FC32746" Ref="C?"  Part="1" 
AR Path="/5FC1E11E/5FC32746" Ref="C?"  Part="1" 
AR Path="/5FC1E1DD/5FC32746" Ref="C?"  Part="1" 
AR Path="/5FC1E32F/5FC32746" Ref="C?"  Part="1" 
AR Path="/5FC1FA4C/5FC32746" Ref="C?"  Part="1" 
AR Path="/5FC1FA54/5FC32746" Ref="C?"  Part="1" 
AR Path="/5FC1FA5C/5FC32746" Ref="C?"  Part="1" 
AR Path="/5FC1FA64/5FC32746" Ref="C?"  Part="1" 
AR Path="/5FC1FA6C/5FC32746" Ref="C?"  Part="1" 
F 0 "C?" H 2878 1437 50  0000 L CNN
F 1 "CC0603KRX7R9BB103" H 2750 1600 50  0001 L CNN
F 2 "YAGEO-CC0603-DA-05_2018-24-*" H 2750 1700 50  0001 L CNN
F 3 "http://www.yageo.com/documents/recent/UPY-GPHC_X7R_6.3V-to-50V_18.pdf" H 2750 1800 50  0001 L CNN
F 4 "No" H 2750 1900 50  0001 L CNN "automotive"
F 5 "10 nF" H 2878 1346 50  0000 L CNN "capacitance"
F 6 "Cap" H 2750 2100 50  0001 L CNN "category"
F 7 "Passive Components" H 2750 2200 50  0001 L CNN "device class L1"
F 8 "Capacitors" H 2750 2300 50  0001 L CNN "device class L2"
F 9 "Ceramic Capacitors" H 2750 2400 50  0001 L CNN "device class L3"
F 10 "CAP CER 10000PF 50V X7R 0603" H 2750 2500 50  0001 L CNN "digikey description"
F 11 "311-1085-2-ND" H 2750 2600 50  0001 L CNN "digikey part number"
F 12 "http://www.yageo.com/exep/pages/download/literatures/UPY-C_GEN_24.pdf" H 2750 2700 50  0001 L CNN "footprint url"
F 13 "0.9mm" H 2750 2800 50  0001 L CNN "height"
F 14 "Yes" H 2750 2900 50  0001 L CNN "lead free"
F 15 "2a4c50056f7872c7" H 2750 3000 50  0001 L CNN "library id"
F 16 "YAGEO" H 2750 3100 50  0001 L CNN "manufacturer"
F 17 "Ceramic" H 2750 3200 50  0001 L CNN "material"
F 18 "Multilayer Ceramic Capacitor, CC Series, 0.01 - F, - 10%, X7R, 50 V, 0603 [1608 Metric]" H 2750 3300 50  0001 L CNN "mouser description"
F 19 "603-CC603KRX7R9BB103" H 2750 3400 50  0001 L CNN "mouser part number"
F 20 "0603" H 2878 1255 50  0000 L CNN "package"
F 21 "Yes" H 2750 3600 50  0001 L CNN "rohs"
F 22 "X7R" H 2750 3700 50  0001 L CNN "temperature characteristic"
F 23 "15%" H 2750 3800 50  0001 L CNN "temperature coefficient"
F 24 "+125°C" H 2750 3900 50  0001 L CNN "temperature range high"
F 25 "-55°C" H 2750 4000 50  0001 L CNN "temperature range low"
F 26 "0.1" H 2750 4100 50  0001 L CNN "tolerance"
F 27 "50 V" H 2878 1164 50  0000 L CNN "voltage"
F 28 "50 V" H 2750 4300 50  0001 L CNN "voltage rating"
	1    2750 1400
	1    0    0    -1  
$EndComp
Wire Wire Line
	2250 1250 2750 1250
Wire Wire Line
	2750 1250 2750 1400
Wire Wire Line
	2750 1600 2750 1800
Wire Wire Line
	2750 1800 2250 1800
Wire Wire Line
	4950 1250 3750 1250
Connection ~ 3750 1250
Wire Wire Line
	4950 1150 2750 1150
Wire Wire Line
	2750 1150 2750 1250
Connection ~ 2750 1250
Wire Wire Line
	4700 1050 1750 1050
Wire Wire Line
	1750 1050 1750 1250
Connection ~ 1750 1250
$Comp
L project:CC0603MRX5R5BB225 C?
U 1 1 5FC679AB
P 1250 2250
AR Path="/5FBD86FB/5FC679AB" Ref="C?"  Part="1" 
AR Path="/5FC1DCBA/5FC679AB" Ref="C?"  Part="1" 
AR Path="/5FC1E11E/5FC679AB" Ref="C?"  Part="1" 
AR Path="/5FC1E1DD/5FC679AB" Ref="C?"  Part="1" 
AR Path="/5FC1E32F/5FC679AB" Ref="C?"  Part="1" 
AR Path="/5FC1FA4C/5FC679AB" Ref="C?"  Part="1" 
AR Path="/5FC1FA54/5FC679AB" Ref="C?"  Part="1" 
AR Path="/5FC1FA5C/5FC679AB" Ref="C?"  Part="1" 
AR Path="/5FC1FA64/5FC679AB" Ref="C?"  Part="1" 
AR Path="/5FC1FA6C/5FC679AB" Ref="C?"  Part="1" 
F 0 "C?" H 1378 2287 50  0000 L CNN
F 1 "CC0603MRX5R5BB225" H 1378 2105 50  0001 L CNN
F 2 "YAGEO-CC0603-DA-05_2018-24-*" H 1250 2550 50  0001 L CNN
F 3 "http://www.yageo.com/documents/recent/UPY-GPHC_X7R_6.3V-to-50V_18.pdf" H 1250 2650 50  0001 L CNN
F 4 "No" H 1250 2750 50  0001 L CNN "automotive"
F 5 "2.2 uF" H 1378 2196 50  0000 L CNN "capacitance"
F 6 "Cap" H 1250 2950 50  0001 L CNN "category"
F 7 "Passive Components" H 1250 3050 50  0001 L CNN "device class L1"
F 8 "Capacitors" H 1250 3150 50  0001 L CNN "device class L2"
F 9 "Ceramic Capacitors" H 1250 3250 50  0001 L CNN "device class L3"
F 10 "CAP CER 2.2UF 6.3V X5R 0603" H 1250 3350 50  0001 L CNN "digikey description"
F 11 "311-1814-2-ND" H 1250 3450 50  0001 L CNN "digikey part number"
F 12 "http://www.yageo.com/exep/pages/download/literatures/UPY-C_GEN_24.pdf" H 1250 3550 50  0001 L CNN "footprint url"
F 13 "0.9mm" H 1250 3650 50  0001 L CNN "height"
F 14 "Yes" H 1250 3750 50  0001 L CNN "lead free"
F 15 "d4a18ccc754c13a9" H 1250 3850 50  0001 L CNN "library id"
F 16 "YAGEO" H 1250 3950 50  0001 L CNN "manufacturer"
F 17 "Ceramic" H 1250 4050 50  0001 L CNN "material"
F 18 "0603 2.2 uF 6.3 V ±20% Tolerance X5R SMT Multilayer Ceramic Capacitor" H 1250 4150 50  0001 L CNN "mouser description"
F 19 "603-CC603MRX5R5BB225" H 1250 4250 50  0001 L CNN "mouser part number"
F 20 "0603" H 1378 2105 50  0000 L CNN "package"
F 21 "Yes" H 1250 4450 50  0001 L CNN "rohs"
F 22 "X5R" H 1250 4550 50  0001 L CNN "temperature characteristic"
F 23 "15%" H 1250 4650 50  0001 L CNN "temperature coefficient"
F 24 "+125°C" H 1250 4750 50  0001 L CNN "temperature range high"
F 25 "-55°C" H 1250 4850 50  0001 L CNN "temperature range low"
F 26 "0.2" H 1250 4950 50  0001 L CNN "tolerance"
F 27 "6.3 V" H 1378 2014 50  0000 L CNN "voltage"
F 28 "6.3 V" H 1250 5150 50  0001 L CNN "voltage rating"
	1    1250 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	1250 2100 1250 2250
Wire Wire Line
	1250 2450 1250 2650
$Comp
L project:CC0603KRX7R9BB103 C?
U 1 1 5FC679CC
P 1750 2250
AR Path="/5FBD86FB/5FC679CC" Ref="C?"  Part="1" 
AR Path="/5FC1DCBA/5FC679CC" Ref="C?"  Part="1" 
AR Path="/5FC1E11E/5FC679CC" Ref="C?"  Part="1" 
AR Path="/5FC1E1DD/5FC679CC" Ref="C?"  Part="1" 
AR Path="/5FC1E32F/5FC679CC" Ref="C?"  Part="1" 
AR Path="/5FC1FA4C/5FC679CC" Ref="C?"  Part="1" 
AR Path="/5FC1FA54/5FC679CC" Ref="C?"  Part="1" 
AR Path="/5FC1FA5C/5FC679CC" Ref="C?"  Part="1" 
AR Path="/5FC1FA64/5FC679CC" Ref="C?"  Part="1" 
AR Path="/5FC1FA6C/5FC679CC" Ref="C?"  Part="1" 
F 0 "C?" H 1878 2287 50  0000 L CNN
F 1 "CC0603KRX7R9BB103" H 1750 2450 50  0001 L CNN
F 2 "YAGEO-CC0603-DA-05_2018-24-*" H 1750 2550 50  0001 L CNN
F 3 "http://www.yageo.com/documents/recent/UPY-GPHC_X7R_6.3V-to-50V_18.pdf" H 1750 2650 50  0001 L CNN
F 4 "No" H 1750 2750 50  0001 L CNN "automotive"
F 5 "10 nF" H 1878 2196 50  0000 L CNN "capacitance"
F 6 "Cap" H 1750 2950 50  0001 L CNN "category"
F 7 "Passive Components" H 1750 3050 50  0001 L CNN "device class L1"
F 8 "Capacitors" H 1750 3150 50  0001 L CNN "device class L2"
F 9 "Ceramic Capacitors" H 1750 3250 50  0001 L CNN "device class L3"
F 10 "CAP CER 10000PF 50V X7R 0603" H 1750 3350 50  0001 L CNN "digikey description"
F 11 "311-1085-2-ND" H 1750 3450 50  0001 L CNN "digikey part number"
F 12 "http://www.yageo.com/exep/pages/download/literatures/UPY-C_GEN_24.pdf" H 1750 3550 50  0001 L CNN "footprint url"
F 13 "0.9mm" H 1750 3650 50  0001 L CNN "height"
F 14 "Yes" H 1750 3750 50  0001 L CNN "lead free"
F 15 "2a4c50056f7872c7" H 1750 3850 50  0001 L CNN "library id"
F 16 "YAGEO" H 1750 3950 50  0001 L CNN "manufacturer"
F 17 "Ceramic" H 1750 4050 50  0001 L CNN "material"
F 18 "Multilayer Ceramic Capacitor, CC Series, 0.01 - F, - 10%, X7R, 50 V, 0603 [1608 Metric]" H 1750 4150 50  0001 L CNN "mouser description"
F 19 "603-CC603KRX7R9BB103" H 1750 4250 50  0001 L CNN "mouser part number"
F 20 "0603" H 1878 2105 50  0000 L CNN "package"
F 21 "Yes" H 1750 4450 50  0001 L CNN "rohs"
F 22 "X7R" H 1750 4550 50  0001 L CNN "temperature characteristic"
F 23 "15%" H 1750 4650 50  0001 L CNN "temperature coefficient"
F 24 "+125°C" H 1750 4750 50  0001 L CNN "temperature range high"
F 25 "-55°C" H 1750 4850 50  0001 L CNN "temperature range low"
F 26 "0.1" H 1750 4950 50  0001 L CNN "tolerance"
F 27 "50 V" H 1878 2014 50  0000 L CNN "voltage"
F 28 "50 V" H 1750 5150 50  0001 L CNN "voltage rating"
	1    1750 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	1250 2100 1750 2100
Wire Wire Line
	1750 2100 1750 2250
Wire Wire Line
	1750 2450 1750 2650
Wire Wire Line
	1750 2650 1250 2650
$Comp
L project:CC0603MRX5R5BB225 C?
U 1 1 5FC67A34
P 2250 2250
AR Path="/5FBD86FB/5FC67A34" Ref="C?"  Part="1" 
AR Path="/5FC1DCBA/5FC67A34" Ref="C?"  Part="1" 
AR Path="/5FC1E11E/5FC67A34" Ref="C?"  Part="1" 
AR Path="/5FC1E1DD/5FC67A34" Ref="C?"  Part="1" 
AR Path="/5FC1E32F/5FC67A34" Ref="C?"  Part="1" 
AR Path="/5FC1FA4C/5FC67A34" Ref="C?"  Part="1" 
AR Path="/5FC1FA54/5FC67A34" Ref="C?"  Part="1" 
AR Path="/5FC1FA5C/5FC67A34" Ref="C?"  Part="1" 
AR Path="/5FC1FA64/5FC67A34" Ref="C?"  Part="1" 
AR Path="/5FC1FA6C/5FC67A34" Ref="C?"  Part="1" 
F 0 "C?" H 2378 2287 50  0000 L CNN
F 1 "CC0603MRX5R5BB225" H 2378 2105 50  0001 L CNN
F 2 "YAGEO-CC0603-DA-05_2018-24-*" H 2250 2550 50  0001 L CNN
F 3 "http://www.yageo.com/documents/recent/UPY-GPHC_X7R_6.3V-to-50V_18.pdf" H 2250 2650 50  0001 L CNN
F 4 "No" H 2250 2750 50  0001 L CNN "automotive"
F 5 "2.2 uF" H 2378 2196 50  0000 L CNN "capacitance"
F 6 "Cap" H 2250 2950 50  0001 L CNN "category"
F 7 "Passive Components" H 2250 3050 50  0001 L CNN "device class L1"
F 8 "Capacitors" H 2250 3150 50  0001 L CNN "device class L2"
F 9 "Ceramic Capacitors" H 2250 3250 50  0001 L CNN "device class L3"
F 10 "CAP CER 2.2UF 6.3V X5R 0603" H 2250 3350 50  0001 L CNN "digikey description"
F 11 "311-1814-2-ND" H 2250 3450 50  0001 L CNN "digikey part number"
F 12 "http://www.yageo.com/exep/pages/download/literatures/UPY-C_GEN_24.pdf" H 2250 3550 50  0001 L CNN "footprint url"
F 13 "0.9mm" H 2250 3650 50  0001 L CNN "height"
F 14 "Yes" H 2250 3750 50  0001 L CNN "lead free"
F 15 "d4a18ccc754c13a9" H 2250 3850 50  0001 L CNN "library id"
F 16 "YAGEO" H 2250 3950 50  0001 L CNN "manufacturer"
F 17 "Ceramic" H 2250 4050 50  0001 L CNN "material"
F 18 "0603 2.2 uF 6.3 V ±20% Tolerance X5R SMT Multilayer Ceramic Capacitor" H 2250 4150 50  0001 L CNN "mouser description"
F 19 "603-CC603MRX5R5BB225" H 2250 4250 50  0001 L CNN "mouser part number"
F 20 "0603" H 2378 2105 50  0000 L CNN "package"
F 21 "Yes" H 2250 4450 50  0001 L CNN "rohs"
F 22 "X5R" H 2250 4550 50  0001 L CNN "temperature characteristic"
F 23 "15%" H 2250 4650 50  0001 L CNN "temperature coefficient"
F 24 "+125°C" H 2250 4750 50  0001 L CNN "temperature range high"
F 25 "-55°C" H 2250 4850 50  0001 L CNN "temperature range low"
F 26 "0.2" H 2250 4950 50  0001 L CNN "tolerance"
F 27 "6.3 V" H 2378 2014 50  0000 L CNN "voltage"
F 28 "6.3 V" H 2250 5150 50  0001 L CNN "voltage rating"
	1    2250 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	2250 2100 2250 2250
Wire Wire Line
	2250 2450 2250 2650
$Comp
L project:CC0603KRX7R9BB103 C?
U 1 1 5FC67A55
P 2750 2250
AR Path="/5FBD86FB/5FC67A55" Ref="C?"  Part="1" 
AR Path="/5FC1DCBA/5FC67A55" Ref="C?"  Part="1" 
AR Path="/5FC1E11E/5FC67A55" Ref="C?"  Part="1" 
AR Path="/5FC1E1DD/5FC67A55" Ref="C?"  Part="1" 
AR Path="/5FC1E32F/5FC67A55" Ref="C?"  Part="1" 
AR Path="/5FC1FA4C/5FC67A55" Ref="C?"  Part="1" 
AR Path="/5FC1FA54/5FC67A55" Ref="C?"  Part="1" 
AR Path="/5FC1FA5C/5FC67A55" Ref="C?"  Part="1" 
AR Path="/5FC1FA64/5FC67A55" Ref="C?"  Part="1" 
AR Path="/5FC1FA6C/5FC67A55" Ref="C?"  Part="1" 
F 0 "C?" H 2878 2287 50  0000 L CNN
F 1 "CC0603KRX7R9BB103" H 2750 2450 50  0001 L CNN
F 2 "YAGEO-CC0603-DA-05_2018-24-*" H 2750 2550 50  0001 L CNN
F 3 "http://www.yageo.com/documents/recent/UPY-GPHC_X7R_6.3V-to-50V_18.pdf" H 2750 2650 50  0001 L CNN
F 4 "No" H 2750 2750 50  0001 L CNN "automotive"
F 5 "10 nF" H 2878 2196 50  0000 L CNN "capacitance"
F 6 "Cap" H 2750 2950 50  0001 L CNN "category"
F 7 "Passive Components" H 2750 3050 50  0001 L CNN "device class L1"
F 8 "Capacitors" H 2750 3150 50  0001 L CNN "device class L2"
F 9 "Ceramic Capacitors" H 2750 3250 50  0001 L CNN "device class L3"
F 10 "CAP CER 10000PF 50V X7R 0603" H 2750 3350 50  0001 L CNN "digikey description"
F 11 "311-1085-2-ND" H 2750 3450 50  0001 L CNN "digikey part number"
F 12 "http://www.yageo.com/exep/pages/download/literatures/UPY-C_GEN_24.pdf" H 2750 3550 50  0001 L CNN "footprint url"
F 13 "0.9mm" H 2750 3650 50  0001 L CNN "height"
F 14 "Yes" H 2750 3750 50  0001 L CNN "lead free"
F 15 "2a4c50056f7872c7" H 2750 3850 50  0001 L CNN "library id"
F 16 "YAGEO" H 2750 3950 50  0001 L CNN "manufacturer"
F 17 "Ceramic" H 2750 4050 50  0001 L CNN "material"
F 18 "Multilayer Ceramic Capacitor, CC Series, 0.01 - F, - 10%, X7R, 50 V, 0603 [1608 Metric]" H 2750 4150 50  0001 L CNN "mouser description"
F 19 "603-CC603KRX7R9BB103" H 2750 4250 50  0001 L CNN "mouser part number"
F 20 "0603" H 2878 2105 50  0000 L CNN "package"
F 21 "Yes" H 2750 4450 50  0001 L CNN "rohs"
F 22 "X7R" H 2750 4550 50  0001 L CNN "temperature characteristic"
F 23 "15%" H 2750 4650 50  0001 L CNN "temperature coefficient"
F 24 "+125°C" H 2750 4750 50  0001 L CNN "temperature range high"
F 25 "-55°C" H 2750 4850 50  0001 L CNN "temperature range low"
F 26 "0.1" H 2750 4950 50  0001 L CNN "tolerance"
F 27 "50 V" H 2878 2014 50  0000 L CNN "voltage"
F 28 "50 V" H 2750 5150 50  0001 L CNN "voltage rating"
	1    2750 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	2250 2100 2750 2100
Wire Wire Line
	2750 2100 2750 2250
Wire Wire Line
	2750 2450 2750 2650
Wire Wire Line
	2750 2650 2250 2650
Wire Wire Line
	2750 2000 2750 2100
Connection ~ 2750 2100
Wire Wire Line
	1750 1900 1750 2100
Connection ~ 1750 2100
Wire Wire Line
	1750 1900 4200 1900
Wire Wire Line
	4200 1900 4200 1350
Wire Wire Line
	2750 2000 4350 2000
Wire Wire Line
	4350 2000 4350 1450
Wire Wire Line
	4350 1450 4850 1450
Connection ~ 4850 1450
Wire Wire Line
	4200 1350 4950 1350
Wire Wire Line
	4200 2750 4200 1900
Wire Wire Line
	4200 2750 4950 2750
Connection ~ 4200 1900
Wire Wire Line
	3250 1800 2750 1800
Connection ~ 3250 1800
Connection ~ 2750 1800
Wire Wire Line
	2250 1800 1750 1800
Connection ~ 2250 1800
Connection ~ 1750 1800
Wire Wire Line
	1250 1800 950  1800
Wire Wire Line
	950  1800 950  2650
Wire Wire Line
	950  2650 1250 2650
Connection ~ 1250 1800
Connection ~ 1250 2650
Wire Wire Line
	1750 2650 2250 2650
Connection ~ 1750 2650
Connection ~ 2250 2650
Wire Wire Line
	3950 4000 950  4000
Wire Wire Line
	950  4000 950  2650
Connection ~ 3950 4000
Connection ~ 950  2650
Text Label 4650 2450 0    50   ~ 0
SDA
Text Label 4650 2550 0    50   ~ 0
SCL
$Comp
L Connector_Generic:Conn_02x03_Odd_Even J?
U 1 1 5FCBDD08
P 5400 7350
AR Path="/5FC1E32F/5FCBDD08" Ref="J?"  Part="1" 
AR Path="/5FBD86FB/5FCBDD08" Ref="J?"  Part="1" 
F 0 "J?" H 5450 7667 50  0000 C CNN
F 1 "Conn_02x03_Odd_Even" H 5450 7576 50  0000 C CNN
F 2 "" H 5400 7350 50  0001 C CNN
F 3 "~" H 5400 7350 50  0001 C CNN
	1    5400 7350
	1    0    0    -1  
$EndComp
$Comp
L project:ATMEGA328P-MU U?
U 1 1 5FCC0393
P 1450 4600
AR Path="/5FC1E32F/5FCC0393" Ref="U?"  Part="1" 
AR Path="/5FBD86FB/5FCC0393" Ref="U?"  Part="1" 
F 0 "U?" H 2450 4955 50  0000 C CNN
F 1 "ATMEGA328P-MU" H 2450 4864 50  0000 C CNN
F 2 "Microchip___Atmel-ATMEGA328P-MU-*" H 1450 5000 50  0001 L CNN
F 3 "" H 1450 5100 50  0001 L CNN
F 4 "MO-220-VHHD-2" H 1450 5200 50  0001 L CNN "Code  JEDEC"
F 5 "Manufacturer URL" H 1450 5300 50  0001 L CNN "Component Link 1 Description"
F 6 "http://www.atmel.com/" H 1450 5400 50  0001 L CNN "Component Link 1 URL"
F 7 "ATmega48A/48PA/88A/88PA/168A/168PA/328/328P Datasheet" H 1450 5500 50  0001 L CNN "Component Link 2 Description"
F 8 "http://www.atmel.com/dyn/resources/prod_documents/8271.pdf" H 1450 5600 50  0001 L CNN "Component Link 2 URL"
F 9 "ATmega48A/48PA/88A/88PA/168A/168PA/328/328P Summary" H 1450 5700 50  0001 L CNN "Component Link 3 Description"
F 10 "http://www.atmel.com/dyn/resources/prod_documents/8271S.pdf" H 1450 5800 50  0001 L CNN "Component Link 3 URL"
F 11 "revD, May-2011" H 1450 5900 50  0001 L CNN "Datasheet Version"
F 12 "32-Pin Micro Lead Frame Package (MLF),  5 x 5 x 1.0 mm Body, Pitch 0.50 mm, 3.10mm Exposed Pad" H 1450 6000 50  0001 L CNN "Package Description"
F 13 "revE, May-2006" H 1450 6100 50  0001 L CNN "Package Version"
F 14 "IC" H 1450 6200 50  0001 L CNN "category"
F 15 "973187" H 1450 6300 50  0001 L CNN "ciiva ids"
F 16 "7d83c92a723a3371" H 1450 6400 50  0001 L CNN "library id"
F 17 "Microchip / Atmel" H 1450 6500 50  0001 L CNN "manufacturer"
F 18 "QFN-32" H 2450 4773 50  0000 C CNN "package"
F 19 "1329192673" H 1450 6700 50  0001 L CNN "release date"
F 20 "417A53D3-7F28-4C25-88A7-47DF99564268" H 1450 6800 50  0001 L CNN "vault revision"
F 21 "yes" H 1450 6900 50  0001 L CNN "imported"
	1    1450 4600
	1    0    0    -1  
$EndComp
Wire Wire Line
	1550 6900 1400 6900
Wire Wire Line
	1400 6900 1400 7000
Wire Wire Line
	1550 7000 1400 7000
Connection ~ 1400 7000
Wire Wire Line
	1400 7000 1400 7100
Wire Wire Line
	1550 7100 1400 7100
Connection ~ 1400 7100
Wire Wire Line
	1400 7100 1400 7300
Wire Wire Line
	1550 7300 1400 7300
Text Label 5250 4000 0    50   ~ 0
GND
Wire Wire Line
	1550 4900 1450 4900
Wire Wire Line
	1450 4900 1450 4700
Wire Wire Line
	1450 4600 1550 4600
Wire Wire Line
	1550 4700 1450 4700
Connection ~ 1450 4700
Wire Wire Line
	1450 4700 1450 4600
Wire Wire Line
	1550 5100 1200 5100
Wire Wire Line
	1200 5100 1200 5200
$Comp
L project:CC0603KPX7R9BB104 C?
U 1 1 5FD114BD
P 1200 5200
AR Path="/5FC1E32F/5FD114BD" Ref="C?"  Part="1" 
AR Path="/5FBD86FB/5FD114BD" Ref="C?"  Part="1" 
F 0 "C?" H 1328 5237 50  0000 L CNN
F 1 "CC0603KPX7R9BB104" H 1200 5400 50  0001 L CNN
F 2 "project:YAGEO-CC0603-0-0-0" H 1200 5500 50  0001 L CNN
F 3 "http://www.yageo.com/documents/recent/UPY-GPHC_X7R_6.3V-to-50V_18.pdf" H 1200 5600 50  0001 L CNN
F 4 "No" H 1200 5700 50  0001 L CNN "automotive"
F 5 "100 nF" H 1328 5146 50  0000 L CNN "capacitance"
F 6 "Cap" H 1200 5900 50  0001 L CNN "category"
F 7 "Passive Components" H 1200 6000 50  0001 L CNN "device class L1"
F 8 "Capacitors" H 1200 6100 50  0001 L CNN "device class L2"
F 9 "Ceramic Capacitors" H 1200 6200 50  0001 L CNN "device class L3"
F 10 "CAP CER 0.1UF 50V X7R 0603" H 1200 6300 50  0001 L CNN "digikey description"
F 11 "CC0603KPX7R9BB104-ND" H 1200 6400 50  0001 L CNN "digikey part number"
F 12 "http://www.yageo.com/exep/pages/download/literatures/UPY-C_GEN_24.pdf" H 1200 6500 50  0001 L CNN "footprint url"
F 13 "0.9mm" H 1200 6600 50  0001 L CNN "height"
F 14 "Yes" H 1200 6700 50  0001 L CNN "lead free"
F 15 "7cd692b09ff39865" H 1200 6800 50  0001 L CNN "library id"
F 16 "YAGEO" H 1200 6900 50  0001 L CNN "manufacturer"
F 17 "Ceramic" H 1200 7000 50  0001 L CNN "material"
F 18 "Multilayer Ceramic Capacitors MLCC - SMD/SMT 100nF 50V X7R 10%" H 1200 7100 50  0001 L CNN "mouser description"
F 19 "603-CC603KPX7R9BB104" H 1200 7200 50  0001 L CNN "mouser part number"
F 20 "0603" H 1328 5055 50  0000 L CNN "package"
F 21 "Yes" H 1200 7400 50  0001 L CNN "rohs"
F 22 "X7R" H 1200 7500 50  0001 L CNN "temperature characteristic"
F 23 "15%" H 1200 7600 50  0001 L CNN "temperature coefficient"
F 24 "+125°C" H 1200 7700 50  0001 L CNN "temperature range high"
F 25 "-55°C" H 1200 7800 50  0001 L CNN "temperature range low"
F 26 "0.1" H 1200 7900 50  0001 L CNN "tolerance"
F 27 "50 V" H 1328 4964 50  0000 L CNN "voltage"
F 28 "50 V" H 1200 8100 50  0001 L CNN "voltage rating"
	1    1200 5200
	1    0    0    -1  
$EndComp
Wire Wire Line
	1200 5400 1200 5600
$Comp
L project:CC0603KPX7R9BB104 C?
U 1 1 5FD25CA5
P 950 4700
AR Path="/5FC1E32F/5FD25CA5" Ref="C?"  Part="1" 
AR Path="/5FBD86FB/5FD25CA5" Ref="C?"  Part="1" 
F 0 "C?" H 1078 4737 50  0000 L CNN
F 1 "CC0603KPX7R9BB104" H 950 4900 50  0001 L CNN
F 2 "project:YAGEO-CC0603-0-0-0" H 950 5000 50  0001 L CNN
F 3 "http://www.yageo.com/documents/recent/UPY-GPHC_X7R_6.3V-to-50V_18.pdf" H 950 5100 50  0001 L CNN
F 4 "No" H 950 5200 50  0001 L CNN "automotive"
F 5 "100 nF" H 1078 4646 50  0000 L CNN "capacitance"
F 6 "Cap" H 950 5400 50  0001 L CNN "category"
F 7 "Passive Components" H 950 5500 50  0001 L CNN "device class L1"
F 8 "Capacitors" H 950 5600 50  0001 L CNN "device class L2"
F 9 "Ceramic Capacitors" H 950 5700 50  0001 L CNN "device class L3"
F 10 "CAP CER 0.1UF 50V X7R 0603" H 950 5800 50  0001 L CNN "digikey description"
F 11 "CC0603KPX7R9BB104-ND" H 950 5900 50  0001 L CNN "digikey part number"
F 12 "http://www.yageo.com/exep/pages/download/literatures/UPY-C_GEN_24.pdf" H 950 6000 50  0001 L CNN "footprint url"
F 13 "0.9mm" H 950 6100 50  0001 L CNN "height"
F 14 "Yes" H 950 6200 50  0001 L CNN "lead free"
F 15 "7cd692b09ff39865" H 950 6300 50  0001 L CNN "library id"
F 16 "YAGEO" H 950 6400 50  0001 L CNN "manufacturer"
F 17 "Ceramic" H 950 6500 50  0001 L CNN "material"
F 18 "Multilayer Ceramic Capacitors MLCC - SMD/SMT 100nF 50V X7R 10%" H 950 6600 50  0001 L CNN "mouser description"
F 19 "603-CC603KPX7R9BB104" H 950 6700 50  0001 L CNN "mouser part number"
F 20 "0603" H 1078 4555 50  0000 L CNN "package"
F 21 "Yes" H 950 6900 50  0001 L CNN "rohs"
F 22 "X7R" H 950 7000 50  0001 L CNN "temperature characteristic"
F 23 "15%" H 950 7100 50  0001 L CNN "temperature coefficient"
F 24 "+125°C" H 950 7200 50  0001 L CNN "temperature range high"
F 25 "-55°C" H 950 7300 50  0001 L CNN "temperature range low"
F 26 "0.1" H 950 7400 50  0001 L CNN "tolerance"
F 27 "50 V" H 1078 4464 50  0000 L CNN "voltage"
F 28 "50 V" H 950 7600 50  0001 L CNN "voltage rating"
	1    950  4700
	1    0    0    -1  
$EndComp
Wire Wire Line
	1450 4600 950  4600
Wire Wire Line
	950  4600 950  4700
Connection ~ 1450 4600
Wire Wire Line
	950  4900 950  5600
Wire Wire Line
	950  5600 1200 5600
Wire Wire Line
	1400 6900 950  6900
Wire Wire Line
	950  6900 950  5600
Connection ~ 1400 6900
Connection ~ 950  5600
Text Label 1100 6900 0    50   ~ 0
GND
Text Label 3400 5300 0    50   ~ 0
SDA
Text Label 3400 5400 0    50   ~ 0
SCL
Text Label 4400 1050 0    50   ~ 0
VDD
Text Label 4400 1150 0    50   ~ 0
VDD_A
Text Label 4400 1250 0    50   ~ 0
VDD_AM
Text Label 4400 1350 0    50   ~ 0
VDD_D
Text Label 4400 1450 0    50   ~ 0
VDD_RF
Text Label 6400 1050 0    50   ~ 0
RFO1
Text Label 6400 1150 0    50   ~ 0
RFO2
Text Label 8400 1050 0    50   ~ 0
RF1
Text Label 8400 1850 0    50   ~ 0
RF2
Text Label 6400 2050 0    50   ~ 0
AGDC
Wire Wire Line
	5700 7250 5950 7250
Wire Wire Line
	5700 7350 5950 7350
Wire Wire Line
	5700 7450 5950 7450
Wire Wire Line
	4950 7250 5200 7250
Wire Wire Line
	4950 7350 5200 7350
Wire Wire Line
	4950 7450 5200 7450
Text Label 4950 7250 0    50   ~ 0
MISO
Text Label 4950 7350 0    50   ~ 0
SCK
Text Label 4950 7450 0    50   ~ 0
RES
Text Label 5700 7250 0    50   ~ 0
VDD
Text Label 5700 7350 0    50   ~ 0
MOSI
Text Label 5700 7450 0    50   ~ 0
GND
Text Label 3400 6900 0    50   ~ 0
MOSI
Wire Wire Line
	3350 6900 3600 6900
Wire Wire Line
	3350 7100 3600 7100
Text Label 3400 7000 0    50   ~ 0
MISO
Text Label 3400 7100 0    50   ~ 0
SCK
Text Label 3400 5500 0    50   ~ 0
RES
$Comp
L project:TSX-3225_16.0000MF09Z-AC3 Y?
U 1 1 5FDA9FE5
P 3800 7400
AR Path="/5FC1E32F/5FDA9FE5" Ref="Y?"  Part="1" 
AR Path="/5FBD86FB/5FDA9FE5" Ref="Y?"  Part="1" 
F 0 "Y?" H 4100 7755 50  0000 C CNN
F 1 "TSX-3225_16.0000MF09Z-AC3" H 3800 7700 50  0001 L CNN
F 2 "Epson-TSX-3225_16.0000MF09Z-AC3-*" H 3800 7800 50  0001 L CNN
F 3 "https://support.epson.biz/td/api/doc_check.php?dl=brief_FA-238V_en.pdf" H 3800 7900 50  0001 L CNN
F 4 "Crys" H 3800 8000 50  0001 L CNN "category"
F 5 "16MHz ±10ppm Crystal 9pF 60 Ohms 4-SMD, No Lead" H 3800 8100 50  0001 L CNN "digikey description"
F 6 "SER3628CT-ND" H 3800 8200 50  0001 L CNN "digikey part number"
F 7 "yes" H 3800 8300 50  0001 L CNN "lead free"
F 8 "36bb9251123dabb2" H 3800 8400 50  0001 L CNN "library id"
F 9 "Epson" H 3800 8500 50  0001 L CNN "manufacturer"
F 10 "732-TX325-16F09Z-AC3" H 3800 8600 50  0001 L CNN "mouser part number"
F 11 "XTAL_SMD_3MM2_2MM5" H 3800 8700 50  0001 L CNN "package"
F 12 "yes" H 3800 8800 50  0001 L CNN "rohs"
F 13 "+75°C" H 3800 8900 50  0001 L CNN "temperature range high"
F 14 "-20°C" H 3800 9000 50  0001 L CNN "temperature range low"
F 15 "16MHz" H 4100 7664 50  0000 C CNN "Frequency"
F 16 "9pF" H 4100 7573 50  0000 C CNN "Capacitance"
	1    3800 7400
	1    0    0    -1  
$EndComp
Wire Wire Line
	3350 7000 3600 7000
$Comp
L project:CGA3E2C0G1H060D080AA C?
U 1 1 5FE26FEC
P 4450 7500
AR Path="/5FC1E32F/5FE26FEC" Ref="C?"  Part="1" 
AR Path="/5FBD86FB/5FE26FEC" Ref="C?"  Part="1" 
F 0 "C?" H 4578 7537 50  0000 L CNN
F 1 "CGA3E2C0G1H060D080AA" H 4450 7700 50  0001 L CNN
F 2 "TDK-CGA3E-0.3-0.1-0-0-*" H 4450 7800 50  0001 L CNN
F 3 "https://product.tdk.com/info/en/catalog/spec/mlccspec_automotive_general_en.pdf" H 4450 7900 50  0001 L CNN
F 4 "Yes" H 4450 8000 50  0001 L CNN "automotive"
F 5 "Grade 1" H 4450 8100 50  0001 L CNN "automotive grade"
F 6 "6pF" H 4578 7446 50  0000 L CNN "capacitance"
F 7 "Cap" H 4450 8300 50  0001 L CNN "category"
F 8 "Passive Components" H 4450 8400 50  0001 L CNN "device class L1"
F 9 "Capacitors" H 4450 8500 50  0001 L CNN "device class L2"
F 10 "Ceramic Capacitors" H 4450 8600 50  0001 L CNN "device class L3"
F 11 "1.1mm" H 4450 8700 50  0001 L CNN "height"
F 12 "CAPC16080X80" H 4450 8800 50  0001 L CNN "ipc land pattern name"
F 13 "Yes" H 4450 8900 50  0001 L CNN "lead free"
F 14 "9251288ec5c7c3b4" H 4450 9000 50  0001 L CNN "library id"
F 15 "TDK" H 4450 9100 50  0001 L CNN "manufacturer"
F 16 "Ceramic" H 4450 9200 50  0001 L CNN "material"
F 17 "0603" H 4578 7355 50  0000 L CNN "package"
F 18 "Yes" H 4450 9500 50  0001 L CNN "rohs"
F 19 "C0G" H 4450 9600 50  0001 L CNN "temperature characteristic"
F 20 "30ppm/°C" H 4450 9700 50  0001 L CNN "temperature coefficient"
F 21 "+125°C" H 4450 9800 50  0001 L CNN "temperature range high"
F 22 "-55°C" H 4450 9900 50  0001 L CNN "temperature range low"
F 23 "0.5pF" H 4450 10000 50  0001 L CNN "tolerance"
F 24 "50V" H 4578 7264 50  0000 L CNN "voltage rating"
	1    4450 7500
	1    0    0    -1  
$EndComp
$Comp
L project:CGA3E2C0G1H060D080AA C?
U 1 1 5FE39FC9
P 3550 7500
AR Path="/5FC1E32F/5FE39FC9" Ref="C?"  Part="1" 
AR Path="/5FBD86FB/5FE39FC9" Ref="C?"  Part="1" 
F 0 "C?" H 3678 7537 50  0000 L CNN
F 1 "CGA3E2C0G1H060D080AA" H 3550 7700 50  0001 L CNN
F 2 "TDK-CGA3E-0.3-0.1-0-0-*" H 3550 7800 50  0001 L CNN
F 3 "https://product.tdk.com/info/en/catalog/spec/mlccspec_automotive_general_en.pdf" H 3550 7900 50  0001 L CNN
F 4 "Yes" H 3550 8000 50  0001 L CNN "automotive"
F 5 "Grade 1" H 3550 8100 50  0001 L CNN "automotive grade"
F 6 "6pF" H 3678 7446 50  0000 L CNN "capacitance"
F 7 "Cap" H 3550 8300 50  0001 L CNN "category"
F 8 "Passive Components" H 3550 8400 50  0001 L CNN "device class L1"
F 9 "Capacitors" H 3550 8500 50  0001 L CNN "device class L2"
F 10 "Ceramic Capacitors" H 3550 8600 50  0001 L CNN "device class L3"
F 11 "1.1mm" H 3550 8700 50  0001 L CNN "height"
F 12 "CAPC16080X80" H 3550 8800 50  0001 L CNN "ipc land pattern name"
F 13 "Yes" H 3550 8900 50  0001 L CNN "lead free"
F 14 "9251288ec5c7c3b4" H 3550 9000 50  0001 L CNN "library id"
F 15 "TDK" H 3550 9100 50  0001 L CNN "manufacturer"
F 16 "Ceramic" H 3550 9200 50  0001 L CNN "material"
F 17 "0603" H 3678 7355 50  0000 L CNN "package"
F 18 "Yes" H 3550 9500 50  0001 L CNN "rohs"
F 19 "C0G" H 3550 9600 50  0001 L CNN "temperature characteristic"
F 20 "30ppm/°C" H 3550 9700 50  0001 L CNN "temperature coefficient"
F 21 "+125°C" H 3550 9800 50  0001 L CNN "temperature range high"
F 22 "-55°C" H 3550 9900 50  0001 L CNN "temperature range low"
F 23 "0.5pF" H 3550 10000 50  0001 L CNN "tolerance"
F 24 "50V" H 3678 7264 50  0000 L CNN "voltage rating"
	1    3550 7500
	1    0    0    -1  
$EndComp
Wire Wire Line
	3900 7400 3550 7400
Wire Wire Line
	3550 7400 3550 7500
Wire Wire Line
	4300 7400 4450 7400
Wire Wire Line
	4450 7400 4450 7500
Wire Wire Line
	3550 7700 3550 7800
Wire Wire Line
	3550 7800 3950 7800
Wire Wire Line
	4450 7800 4450 7700
Wire Wire Line
	4200 7600 4250 7600
Wire Wire Line
	4250 7600 4250 7800
Connection ~ 4250 7800
Wire Wire Line
	4250 7800 4450 7800
Wire Wire Line
	4000 7600 3950 7600
Wire Wire Line
	3950 7600 3950 7800
Connection ~ 3950 7800
Wire Wire Line
	3950 7800 4250 7800
Wire Wire Line
	3350 7300 3550 7300
Wire Wire Line
	3550 7300 3550 7400
Connection ~ 3550 7400
Wire Wire Line
	3350 7200 3850 7200
Wire Wire Line
	3850 7200 3850 6950
Wire Wire Line
	3850 6950 4450 6950
Wire Wire Line
	4450 6950 4450 7400
Connection ~ 4450 7400
Text Label 4000 7800 0    50   ~ 0
GND
Wire Wire Line
	4000 5400 4000 5750
Wire Wire Line
	4000 5750 4100 5750
Wire Wire Line
	4600 5300 4850 5300
Wire Wire Line
	4850 5300 4850 5750
Wire Wire Line
	4850 5750 4600 5750
Text Label 4650 5300 0    50   ~ 0
VDD
Text Label 3400 4900 0    50   ~ 0
VDET1
Text Label 3400 5000 0    50   ~ 0
VDET2
NoConn ~ 3350 4600
NoConn ~ 3350 4700
Wire Wire Line
	3350 5700 3600 5700
Wire Wire Line
	3350 5800 3600 5800
Text Label 3400 5700 0    50   ~ 0
RX
Text Label 3400 5800 0    50   ~ 0
TX
Wire Wire Line
	3350 5300 4100 5300
Wire Wire Line
	3350 5400 4000 5400
Text HLabel 3600 5700 2    50   Input ~ 0
RX
Text HLabel 3600 5800 2    50   Output ~ 0
TX
Wire Wire Line
	3600 5900 3350 5900
Text Label 3400 5900 0    50   ~ 0
IRQ
Wire Wire Line
	4550 2450 4950 2450
Wire Wire Line
	4550 2550 4950 2550
Text Label 1300 4600 2    50   ~ 0
VDD
Wire Wire Line
	3350 6000 3600 6000
Wire Wire Line
	3350 6100 3600 6100
Wire Wire Line
	3350 6200 3600 6200
Text Label 3400 6000 0    50   ~ 0
RED
Text Label 3400 6100 0    50   ~ 0
GRN
Text Label 3400 6200 0    50   ~ 0
BLU
NoConn ~ 3350 6300
NoConn ~ 3350 6400
NoConn ~ 3350 6600
NoConn ~ 3350 6700
NoConn ~ 3350 6800
NoConn ~ 3350 5200
NoConn ~ 3350 5100
$Comp
L project:RC0603FR-071K65L R?
U 1 1 5FFC6139
P 9050 3750
AR Path="/5FBD86FB/5FFC6139" Ref="R?"  Part="1" 
AR Path="/5FC1E32F/5FFC6139" Ref="R?"  Part="1" 
F 0 "R?" H 9400 4055 50  0000 C CNN
F 1 "RC0603FR-071K65L" H 9400 3874 50  0001 C CNN
F 2 "Yageo-RC0603-0-0-*" H 9050 4150 50  0001 L CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RC_Group_51_RoHS_L_9.pdf" H 9050 4250 50  0001 L CNN
F 4 "541-1.00HHCT-ND" H 9050 4350 50  0001 L CNN "approved alternate part number"
F 5 "Res" H 9050 4450 50  0001 L CNN "category"
F 6 "Thick Film" H 9050 4550 50  0001 L CNN "composition"
F 7 "Passive Components" H 9050 4650 50  0001 L CNN "device class L1"
F 8 "Resistors" H 9050 4750 50  0001 L CNN "device class L2"
F 9 "Chip SMD Resistors" H 9050 4850 50  0001 L CNN "device class L3"
F 10 "RES SMD 1.65K OHM 1% 1/10W 0603" H 9050 4950 50  0001 L CNN "digikey description"
F 11 "311-1.65KHRTR-ND" H 9050 5050 50  0001 L CNN "digikey part number"
F 12 "0.5mm" H 9050 5150 50  0001 L CNN "height"
F 13 "RESC15585X45" H 9050 5250 50  0001 L CNN "ipc land pattern name"
F 14 "yes" H 9050 5350 50  0001 L CNN "lead free"
F 15 "ce4140018ac48837" H 9050 5450 50  0001 L CNN "library id"
F 16 "Yageo" H 9050 5550 50  0001 L CNN "manufacturer"
F 17 "Thick Film Resistors - SMD 1.65K OHM 1%
F 18 "603-RC0603FR-071K65L" H 9050 5750 50  0001 L CNN "mouser part number"
F 19 "0603" H 9400 3964 50  0000 C CNN "package"
F 20 "100mW" H 9050 5950 50  0001 L CNN "power"
F 21 "0.1W" H 9050 6050 50  0001 L CNN "power rating"
F 22 "1.65kΩ" H 9400 3873 50  0000 C CNN "resistance"
F 23 "yes" H 9050 6250 50  0001 L CNN "rohs"
F 24 "RC" H 9050 6350 50  0001 L CNN "series"
F 25 "0mm" H 9050 6450 50  0001 L CNN "standoff height"
F 26 "100ppm/°C" H 9050 6550 50  0001 L CNN "temperature coefficient"
F 27 "+155°C" H 9050 6650 50  0001 L CNN "temperature range high"
F 28 "-55°C" H 9050 6750 50  0001 L CNN "temperature range low"
F 29 "1%" H 9050 6850 50  0001 L CNN "tolerance"
F 30 "75V" H 9050 6950 50  0001 L CNN "voltage"
F 31 "75V" H 9050 7050 50  0001 L CNN "voltage rating"
	1    9050 3750
	1    0    0    -1  
$EndComp
Wire Wire Line
	9150 3750 8900 3750
Wire Wire Line
	8500 3750 8000 3750
Wire Wire Line
	9650 3750 10150 3750
Text Label 8150 3750 0    50   ~ 0
RF1
Text Label 9900 3750 0    50   ~ 0
RF2
$Comp
L project:RC0603FR-071K65L R?
U 1 1 5FFF2D8C
P 9050 4650
AR Path="/5FBD86FB/5FFF2D8C" Ref="R?"  Part="1" 
AR Path="/5FC1E32F/5FFF2D8C" Ref="R?"  Part="1" 
F 0 "R?" H 9400 4955 50  0000 C CNN
F 1 "RC0603FR-071K65L" H 9400 4774 50  0001 C CNN
F 2 "Yageo-RC0603-0-0-*" H 9050 5050 50  0001 L CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RC_Group_51_RoHS_L_9.pdf" H 9050 5150 50  0001 L CNN
F 4 "541-1.00HHCT-ND" H 9050 5250 50  0001 L CNN "approved alternate part number"
F 5 "Res" H 9050 5350 50  0001 L CNN "category"
F 6 "Thick Film" H 9050 5450 50  0001 L CNN "composition"
F 7 "Passive Components" H 9050 5550 50  0001 L CNN "device class L1"
F 8 "Resistors" H 9050 5650 50  0001 L CNN "device class L2"
F 9 "Chip SMD Resistors" H 9050 5750 50  0001 L CNN "device class L3"
F 10 "RES SMD 1.65K OHM 1% 1/10W 0603" H 9050 5850 50  0001 L CNN "digikey description"
F 11 "311-1.65KHRTR-ND" H 9050 5950 50  0001 L CNN "digikey part number"
F 12 "0.5mm" H 9050 6050 50  0001 L CNN "height"
F 13 "RESC15585X45" H 9050 6150 50  0001 L CNN "ipc land pattern name"
F 14 "yes" H 9050 6250 50  0001 L CNN "lead free"
F 15 "ce4140018ac48837" H 9050 6350 50  0001 L CNN "library id"
F 16 "Yageo" H 9050 6450 50  0001 L CNN "manufacturer"
F 17 "Thick Film Resistors - SMD 1.65K OHM 1%
F 18 "603-RC0603FR-071K65L" H 9050 6650 50  0001 L CNN "mouser part number"
F 19 "0603" H 9400 4864 50  0000 C CNN "package"
F 20 "100mW" H 9050 6850 50  0001 L CNN "power"
F 21 "0.1W" H 9050 6950 50  0001 L CNN "power rating"
F 22 "1.65kΩ" H 9400 4773 50  0000 C CNN "resistance"
F 23 "yes" H 9050 7150 50  0001 L CNN "rohs"
F 24 "RC" H 9050 7250 50  0001 L CNN "series"
F 25 "0mm" H 9050 7350 50  0001 L CNN "standoff height"
F 26 "100ppm/°C" H 9050 7450 50  0001 L CNN "temperature coefficient"
F 27 "+155°C" H 9050 7550 50  0001 L CNN "temperature range high"
F 28 "-55°C" H 9050 7650 50  0001 L CNN "temperature range low"
F 29 "1%" H 9050 7750 50  0001 L CNN "tolerance"
F 30 "75V" H 9050 7850 50  0001 L CNN "voltage"
F 31 "75V" H 9050 7950 50  0001 L CNN "voltage rating"
	1    9050 4650
	1    0    0    -1  
$EndComp
Wire Wire Line
	9150 4650 8900 4650
Wire Wire Line
	8500 4650 8000 4650
Wire Wire Line
	9650 4650 10150 4650
Text Label 8150 4650 0    50   ~ 0
RED
Text Label 9900 4650 0    50   ~ 0
GND
$Comp
L project:LTST-C190KGKT D?
U 1 1 5FFFF3B5
P 8500 5550
AR Path="/5FBD86FB/5FFFF3B5" Ref="D?"  Part="1" 
AR Path="/5FC1E32F/5FFFF3B5" Ref="D?"  Part="1" 
F 0 "D?" H 8700 6299 50  0000 C CNN
F 1 "LTST-C190KGKT" H 8700 6208 50  0000 C CNN
F 2 "Lite-On-LTST-C190KGKT-*" H 8500 6150 50  0001 L CNN
F 3 "https://optoelectronics.liteon.com/upload/download/DS22-2000-074/LTST-C190KGKT.PDF" H 8500 6250 50  0001 L CNN
F 4 "No" H 8500 6350 50  0001 L CNN "automotive"
F 5 "Diode" H 8500 6450 50  0001 L CNN "category"
F 6 "Green" H 8700 6117 50  0000 C CNN "colour"
F 7 "Optoelectronics" H 8500 6650 50  0001 L CNN "device class L1"
F 8 "LEDs" H 8500 6750 50  0001 L CNN "device class L2"
F 9 "unset" H 8500 6850 50  0001 L CNN "device class L3"
F 10 "LED GREEN CLEAR CHIP SMD" H 8500 6950 50  0001 L CNN "digikey description"
F 11 "160-1435-1-ND" H 8500 7050 50  0001 L CNN "digikey part number"
F 12 "20mA" H 8700 6026 50  0000 C CNN "forward current"
F 13 "2.4V" H 8700 5935 50  0000 C CNN "forward voltage"
F 14 "0.9mm" H 8500 7350 50  0001 L CNN "height"
F 15 "Yes" H 8500 7450 50  0001 L CNN "lead free"
F 16 "Top View" H 8500 7550 50  0001 L CNN "led orientation"
F 17 "4c107ec0dbc12b09" H 8500 7650 50  0001 L CNN "library id"
F 18 "18-71mcd" H 8500 7750 50  0001 L CNN "luminous intensity"
F 19 "Lite-On" H 8500 7850 50  0001 L CNN "manufacturer"
F 20 "859-LTST-C190KGKT" H 8500 7950 50  0001 L CNN "mouser part number"
F 21 "0603" H 8700 5844 50  0000 C CNN "package"
F 22 "574nm" H 8500 8150 50  0001 L CNN "peak emmision wavelength"
F 23 "75mW" H 8500 8250 50  0001 L CNN "power dissipation"
F 24 "5V" H 8500 8350 50  0001 L CNN "reverse voltage"
F 25 "Yes" H 8500 8450 50  0001 L CNN "rohs"
F 26 "+85°C" H 8500 8550 50  0001 L CNN "temperature range high"
F 27 "-30°C" H 8500 8650 50  0001 L CNN "temperature range low"
F 28 "130°" H 8500 8750 50  0001 L CNN "viewing angle"
F 29 "576.5nm" H 8500 8850 50  0001 L CNN "wavelength"
	1    8500 5550
	1    0    0    -1  
$EndComp
$Comp
L project:RC0603FR-071K65L R?
U 1 1 5FFFF3D7
P 9050 5550
AR Path="/5FBD86FB/5FFFF3D7" Ref="R?"  Part="1" 
AR Path="/5FC1E32F/5FFFF3D7" Ref="R?"  Part="1" 
F 0 "R?" H 9400 5855 50  0000 C CNN
F 1 "RC0603FR-071K65L" H 9400 5764 50  0001 C CNN
F 2 "Yageo-RC0603-0-0-*" H 9050 5950 50  0001 L CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RC_Group_51_RoHS_L_9.pdf" H 9050 6050 50  0001 L CNN
F 4 "541-1.00HHCT-ND" H 9050 6150 50  0001 L CNN "approved alternate part number"
F 5 "Res" H 9050 6250 50  0001 L CNN "category"
F 6 "Thick Film" H 9050 6350 50  0001 L CNN "composition"
F 7 "Passive Components" H 9050 6450 50  0001 L CNN "device class L1"
F 8 "Resistors" H 9050 6550 50  0001 L CNN "device class L2"
F 9 "Chip SMD Resistors" H 9050 6650 50  0001 L CNN "device class L3"
F 10 "RES SMD 1.65K OHM 1% 1/10W 0603" H 9050 6750 50  0001 L CNN "digikey description"
F 11 "311-1.65KHRTR-ND" H 9050 6850 50  0001 L CNN "digikey part number"
F 12 "0.5mm" H 9050 6950 50  0001 L CNN "height"
F 13 "RESC15585X45" H 9050 7050 50  0001 L CNN "ipc land pattern name"
F 14 "yes" H 9050 7150 50  0001 L CNN "lead free"
F 15 "ce4140018ac48837" H 9050 7250 50  0001 L CNN "library id"
F 16 "Yageo" H 9050 7350 50  0001 L CNN "manufacturer"
F 17 "Thick Film Resistors - SMD 1.65K OHM 1%
F 18 "603-RC0603FR-071K65L" H 9050 7550 50  0001 L CNN "mouser part number"
F 19 "0603" H 9400 5764 50  0000 C CNN "package"
F 20 "100mW" H 9050 7750 50  0001 L CNN "power"
F 21 "0.1W" H 9050 7850 50  0001 L CNN "power rating"
F 22 "1.65kΩ" H 9400 5673 50  0000 C CNN "resistance"
F 23 "yes" H 9050 8050 50  0001 L CNN "rohs"
F 24 "RC" H 9050 8150 50  0001 L CNN "series"
F 25 "0mm" H 9050 8250 50  0001 L CNN "standoff height"
F 26 "100ppm/°C" H 9050 8350 50  0001 L CNN "temperature coefficient"
F 27 "+155°C" H 9050 8450 50  0001 L CNN "temperature range high"
F 28 "-55°C" H 9050 8550 50  0001 L CNN "temperature range low"
F 29 "1%" H 9050 8650 50  0001 L CNN "tolerance"
F 30 "75V" H 9050 8750 50  0001 L CNN "voltage"
F 31 "75V" H 9050 8850 50  0001 L CNN "voltage rating"
	1    9050 5550
	1    0    0    -1  
$EndComp
Wire Wire Line
	9150 5550 8900 5550
Wire Wire Line
	8500 5550 8000 5550
Wire Wire Line
	9650 5550 10150 5550
Text Label 8150 5550 0    50   ~ 0
GRN
Text Label 9900 5550 0    50   ~ 0
GND
$Comp
L project:RC0603FR-071K65L R?
U 1 1 6003E356
P 9050 6450
AR Path="/5FBD86FB/6003E356" Ref="R?"  Part="1" 
AR Path="/5FC1E32F/6003E356" Ref="R?"  Part="1" 
F 0 "R?" H 9400 6755 50  0000 C CNN
F 1 "RC0603FR-071K65L" H 9400 6664 50  0001 C CNN
F 2 "Yageo-RC0603-0-0-*" H 9050 6850 50  0001 L CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RC_Group_51_RoHS_L_9.pdf" H 9050 6950 50  0001 L CNN
F 4 "541-1.00HHCT-ND" H 9050 7050 50  0001 L CNN "approved alternate part number"
F 5 "Res" H 9050 7150 50  0001 L CNN "category"
F 6 "Thick Film" H 9050 7250 50  0001 L CNN "composition"
F 7 "Passive Components" H 9050 7350 50  0001 L CNN "device class L1"
F 8 "Resistors" H 9050 7450 50  0001 L CNN "device class L2"
F 9 "Chip SMD Resistors" H 9050 7550 50  0001 L CNN "device class L3"
F 10 "RES SMD 1.65K OHM 1% 1/10W 0603" H 9050 7650 50  0001 L CNN "digikey description"
F 11 "311-1.65KHRTR-ND" H 9050 7750 50  0001 L CNN "digikey part number"
F 12 "0.5mm" H 9050 7850 50  0001 L CNN "height"
F 13 "RESC15585X45" H 9050 7950 50  0001 L CNN "ipc land pattern name"
F 14 "yes" H 9050 8050 50  0001 L CNN "lead free"
F 15 "ce4140018ac48837" H 9050 8150 50  0001 L CNN "library id"
F 16 "Yageo" H 9050 8250 50  0001 L CNN "manufacturer"
F 17 "Thick Film Resistors - SMD 1.65K OHM 1%
F 18 "603-RC0603FR-071K65L" H 9050 8450 50  0001 L CNN "mouser part number"
F 19 "0603" H 9400 6664 50  0000 C CNN "package"
F 20 "100mW" H 9050 8650 50  0001 L CNN "power"
F 21 "0.1W" H 9050 8750 50  0001 L CNN "power rating"
F 22 "1.65kΩ" H 9400 6573 50  0000 C CNN "resistance"
F 23 "yes" H 9050 8950 50  0001 L CNN "rohs"
F 24 "RC" H 9050 9050 50  0001 L CNN "series"
F 25 "0mm" H 9050 9150 50  0001 L CNN "standoff height"
F 26 "100ppm/°C" H 9050 9250 50  0001 L CNN "temperature coefficient"
F 27 "+155°C" H 9050 9350 50  0001 L CNN "temperature range high"
F 28 "-55°C" H 9050 9450 50  0001 L CNN "temperature range low"
F 29 "1%" H 9050 9550 50  0001 L CNN "tolerance"
F 30 "75V" H 9050 9650 50  0001 L CNN "voltage"
F 31 "75V" H 9050 9750 50  0001 L CNN "voltage rating"
	1    9050 6450
	1    0    0    -1  
$EndComp
Wire Wire Line
	9150 6450 8900 6450
Wire Wire Line
	8500 6450 8000 6450
Wire Wire Line
	9650 6450 10150 6450
Text Label 8150 6450 0    50   ~ 0
BLU
Text Label 9900 6450 0    50   ~ 0
GND
$Comp
L project:LTST-C190KRKT D?
U 1 1 6004A743
P 8500 4650
AR Path="/5FBD86FB/6004A743" Ref="D?"  Part="1" 
AR Path="/5FC1E32F/6004A743" Ref="D?"  Part="1" 
F 0 "D?" H 8700 5399 50  0000 C CNN
F 1 "LTST-C190KRKT" H 8700 5308 50  0000 C CNN
F 2 "Lite-On-LTST-C190KRKT-*" H 8500 5250 50  0001 L CNN
F 3 "https://optoelectronics.liteon.com/upload/download/DS-22-99-0151/LTST-C190KRKT.pdf" H 8500 5350 50  0001 L CNN
F 4 "No" H 8500 5450 50  0001 L CNN "automotive"
F 5 "Diode" H 8500 5550 50  0001 L CNN "category"
F 6 "Red" H 8700 5217 50  0000 C CNN "colour"
F 7 "Optoelectronics" H 8500 5750 50  0001 L CNN "device class L1"
F 8 "LEDs" H 8500 5850 50  0001 L CNN "device class L2"
F 9 "unset" H 8500 5950 50  0001 L CNN "device class L3"
F 10 "LED RED CLEAR CHIP SMD" H 8500 6050 50  0001 L CNN "digikey description"
F 11 "160-1436-1-ND" H 8500 6150 50  0001 L CNN "digikey part number"
F 12 "20mA" H 8700 5126 50  0000 C CNN "forward current"
F 13 "2.4V" H 8700 5035 50  0000 C CNN "forward voltage"
F 14 "0.9mm" H 8500 6450 50  0001 L CNN "height"
F 15 "Yes" H 8500 6550 50  0001 L CNN "lead free"
F 16 "Top View" H 8500 6650 50  0001 L CNN "led orientation"
F 17 "3c5f6cfddbd1ff06" H 8500 6750 50  0001 L CNN "library id"
F 18 "18-180mcd" H 8500 6850 50  0001 L CNN "luminous intensity"
F 19 "Lite-On" H 8500 6950 50  0001 L CNN "manufacturer"
F 20 "859-LTST-C190KRKT" H 8500 7050 50  0001 L CNN "mouser part number"
F 21 "0603" H 8700 4944 50  0000 C CNN "package"
F 22 "639nm" H 8500 7250 50  0001 L CNN "peak emmision wavelength"
F 23 "62.5mW" H 8500 7350 50  0001 L CNN "power dissipation"
F 24 "5V" H 8500 7450 50  0001 L CNN "reverse voltage"
F 25 "Yes" H 8500 7550 50  0001 L CNN "rohs"
F 26 "+85°C" H 8500 7650 50  0001 L CNN "temperature range high"
F 27 "-30°C" H 8500 7750 50  0001 L CNN "temperature range low"
F 28 "130°" H 8500 7850 50  0001 L CNN "viewing angle"
F 29 "631nm" H 8500 7950 50  0001 L CNN "wavelength"
	1    8500 4650
	1    0    0    -1  
$EndComp
$Comp
L project:LTST-C190TBKT DS?
U 1 1 6004D7FB
P 8500 6450
AR Path="/5FBD86FB/6004D7FB" Ref="DS?"  Part="1" 
AR Path="/5FC1E32F/6004D7FB" Ref="DS?"  Part="1" 
F 0 "DS?" H 8700 7189 50  0000 C CNN
F 1 "LTST-C190TBKT" H 8700 7098 50  0000 C CNN
F 2 "Vishay_Lite-On-LTST-C190TBKT-*" H 8500 7050 50  0001 L CNN
F 3 "http://optoelectronics.liteon.com/upload/download/DS-22-99-0224/S_110_LTST-C190TBKT(0630).pdf" H 8500 7150 50  0001 L CNN
F 4 "DFN" H 8500 7250 50  0001 L CNN "Case Package"
F 5 "Manufacturer URL" H 8500 7350 50  0001 L CNN "Component Link 1 Description"
F 6 "http://www.us.liteon.com/" H 8500 7450 50  0001 L CNN "Component Link 1 URL"
F 7 "Rev. K, 08/2011" H 8500 7550 50  0001 L CNN "Datasheet Version"
F 8 "1.1 mm" H 8500 7650 50  0001 L CNN "Height"
F 9 "Serial" H 8500 7750 50  0001 L CNN "Interface"
F 10 "3 mm" H 8500 7850 50  0001 L CNN "Length"
F 11 "125 degC" H 8500 7950 50  0001 L CNN "Max Operating Temperature"
F 12 "3.6 V" H 8500 8050 50  0001 L CNN "Max Supply Voltage"
F 13 "-40 degC" H 8500 8150 50  0001 L CNN "Min Operating Temperature"
F 14 "2.1 V" H 8500 8250 50  0001 L CNN "Min Supply Voltage"
F 15 "Surface Mount" H 8500 8350 50  0001 L CNN "Mount"
F 16 "3.3V" H 8700 7007 50  0000 C CNN "Nominal Supply Voltage"
F 17 "2-Pin Surface Mount Device, Body 1.6 x 0.8 mm" H 8500 8550 50  0001 L CNN "Package Description"
F 18 "Cut Tape" H 8500 8650 50  0001 L CNN "Packaging"
F 19 "6" H 8500 8750 50  0001 L CNN "Pins"
F 20 "No SVHC" H 8500 8850 50  0001 L CNN "REACH SVHC"
F 21 "true" H 8500 8950 50  0001 L CNN "Ro HSCompliant"
F 22 "3 mm" H 8500 9050 50  0001 L CNN "Width"
F 23 "Disp" H 8500 9150 50  0001 L CNN "category"
F 24 "1545016" H 8500 9250 50  0001 L CNN "ciiva ids"
F 25 "01d263fa86e1fb49" H 8500 9350 50  0001 L CNN "library id"
F 26 "Vishay Lite-On" H 8500 9450 50  0001 L CNN "manufacturer"
F 27 "0603" H 8700 6916 50  0000 C CNN "package"
F 28 "1475043740" H 8500 9650 50  0001 L CNN "release date"
F 29 "6CBC32C1-F554-43EB-9EEA-D4C78589E111" H 8500 9750 50  0001 L CNN "vault revision"
F 30 "yes" H 8500 9850 50  0001 L CNN "imported"
F 31 "Blue" H 8700 6825 50  0000 C CNN "Color"
F 32 "20mA" H 8700 6734 50  0000 C CNN "Current"
	1    8500 6450
	1    0    0    -1  
$EndComp
$Comp
L project:LTST-C190KSKT D?
U 1 1 60052F87
P 8400 3750
AR Path="/5FBD86FB/60052F87" Ref="D?"  Part="1" 
AR Path="/5FC1E32F/60052F87" Ref="D?"  Part="1" 
F 0 "D?" H 8700 4499 50  0000 C CNN
F 1 "LTST-C190KSKT" H 8700 4408 50  0000 C CNN
F 2 "Lite-On-LTST-C190KSKT-*" H 8400 4350 50  0001 L CNN
F 3 "https://optoelectronics.liteon.com/upload/download/DS-22-99-0189/LTST-C190KSKT.pdf" H 8400 4450 50  0001 L CNN
F 4 "No" H 8400 4550 50  0001 L CNN "automotive"
F 5 "Diode" H 8400 4650 50  0001 L CNN "category"
F 6 "Yellow" H 8700 4317 50  0000 C CNN "colour"
F 7 "Optoelectronics" H 8400 4850 50  0001 L CNN "device class L1"
F 8 "LEDs" H 8400 4950 50  0001 L CNN "device class L2"
F 9 "unset" H 8400 5050 50  0001 L CNN "device class L3"
F 10 "LED YELLOW CLEAR CHIP SMD" H 8400 5150 50  0001 L CNN "digikey description"
F 11 "160-1437-1-ND" H 8400 5250 50  0001 L CNN "digikey part number"
F 12 "20mA" H 8700 4226 50  0000 C CNN "forward current"
F 13 "2.4V" H 8700 4135 50  0000 C CNN "forward voltage"
F 14 "0.9mm" H 8400 5550 50  0001 L CNN "height"
F 15 "Yes" H 8400 5650 50  0001 L CNN "lead free"
F 16 "Top View" H 8400 5750 50  0001 L CNN "led orientation"
F 17 "d70eef253d640e08" H 8400 5850 50  0001 L CNN "library id"
F 18 "28-180mcd" H 8400 5950 50  0001 L CNN "luminous intensity"
F 19 "Lite-On" H 8400 6050 50  0001 L CNN "manufacturer"
F 20 "859-LTST-C190KSKT" H 8400 6150 50  0001 L CNN "mouser part number"
F 21 "0603" H 8700 4044 50  0000 C CNN "package"
F 22 "588nm" H 8400 6350 50  0001 L CNN "peak emmision wavelength"
F 23 "75mW" H 8400 6450 50  0001 L CNN "power dissipation"
F 24 "5V" H 8400 6550 50  0001 L CNN "reverse voltage"
F 25 "Yes" H 8400 6650 50  0001 L CNN "rohs"
F 26 "+85°C" H 8400 6750 50  0001 L CNN "temperature range high"
F 27 "-30°C" H 8400 6850 50  0001 L CNN "temperature range low"
F 28 "130°" H 8400 6950 50  0001 L CNN "viewing angle"
F 29 "597nm" H 8400 7050 50  0001 L CNN "wavelength"
	1    8400 3750
	1    0    0    -1  
$EndComp
Wire Wire Line
	4100 6200 3900 6200
Wire Wire Line
	3900 6200 3900 5500
Wire Wire Line
	3350 5500 3900 5500
Wire Wire Line
	4600 6200 4850 6200
Wire Wire Line
	4850 6200 4850 5750
Connection ~ 4850 5750
$Comp
L project:RC0603FR-0710KL R?
U 1 1 601131B9
P 4000 5300
AR Path="/601131B9" Ref="R?"  Part="1" 
AR Path="/5FBD86FB/601131B9" Ref="R?"  Part="1" 
AR Path="/5FC1E32F/601131B9" Ref="R?"  Part="1" 
F 0 "R?" H 4350 5605 50  0000 C CNN
F 1 "RC0603FR-0710KL" H 4000 5600 50  0001 L CNN
F 2 "Yageo-RC0603-0-0-*" H 4000 5700 50  0001 L CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RC_Group_51_RoHS_L_9.pdf" H 4000 5800 50  0001 L CNN
F 4 "541-1.00HHCT-ND" H 4000 5900 50  0001 L CNN "approved alternate part number"
F 5 "Res" H 4000 6000 50  0001 L CNN "category"
F 6 "Thick Film" H 4000 6100 50  0001 L CNN "composition"
F 7 "Passive Components" H 4000 6200 50  0001 L CNN "device class L1"
F 8 "Resistors" H 4000 6300 50  0001 L CNN "device class L2"
F 9 "Chip SMD Resistors" H 4000 6400 50  0001 L CNN "device class L3"
F 10 "RES SMD 10K OHM 1% 1/10W 0603" H 4000 6500 50  0001 L CNN "digikey description"
F 11 "311-10.0KHRTR-ND" H 4000 6600 50  0001 L CNN "digikey part number"
F 12 "0.5mm" H 4000 6700 50  0001 L CNN "height"
F 13 "RESC15585X45" H 4000 6800 50  0001 L CNN "ipc land pattern name"
F 14 "yes" H 4000 6900 50  0001 L CNN "lead free"
F 15 "368bdb3a487a5469" H 4000 7000 50  0001 L CNN "library id"
F 16 "Yageo" H 4000 7100 50  0001 L CNN "manufacturer"
F 17 "Surface Mount Thick Film Resistor, RC Series, 10 kohm, 100 mW, - 1%, 50 V, 0603 [1608 Metric]" H 4000 7200 50  0001 L CNN "mouser description"
F 18 "603-RC0603FR-0710KL" H 4000 7300 50  0001 L CNN "mouser part number"
F 19 "0603" H 4350 5514 50  0000 C CNN "package"
F 20 "100mW" H 4000 7500 50  0001 L CNN "power"
F 21 "0.1W" H 4000 7600 50  0001 L CNN "power rating"
F 22 "10kΩ" H 4350 5423 50  0000 C CNN "resistance"
F 23 "yes" H 4000 7800 50  0001 L CNN "rohs"
F 24 "RC" H 4000 7900 50  0001 L CNN "series"
F 25 "0mm" H 4000 8000 50  0001 L CNN "standoff height"
F 26 "100ppm/°C" H 4000 8100 50  0001 L CNN "temperature coefficient"
F 27 "+155°C" H 4000 8200 50  0001 L CNN "temperature range high"
F 28 "-55°C" H 4000 8300 50  0001 L CNN "temperature range low"
F 29 "1%" H 4000 8400 50  0001 L CNN "tolerance"
F 30 "75V" H 4000 8500 50  0001 L CNN "voltage"
F 31 "75V" H 4000 8600 50  0001 L CNN "voltage rating"
	1    4000 5300
	1    0    0    -1  
$EndComp
$Comp
L project:RC0603FR-0710KL R?
U 1 1 6011F144
P 4000 5750
AR Path="/6011F144" Ref="R?"  Part="1" 
AR Path="/5FBD86FB/6011F144" Ref="R?"  Part="1" 
AR Path="/5FC1E32F/6011F144" Ref="R?"  Part="1" 
F 0 "R?" H 4350 6055 50  0000 C CNN
F 1 "RC0603FR-0710KL" H 4000 6050 50  0001 L CNN
F 2 "Yageo-RC0603-0-0-*" H 4000 6150 50  0001 L CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RC_Group_51_RoHS_L_9.pdf" H 4000 6250 50  0001 L CNN
F 4 "541-1.00HHCT-ND" H 4000 6350 50  0001 L CNN "approved alternate part number"
F 5 "Res" H 4000 6450 50  0001 L CNN "category"
F 6 "Thick Film" H 4000 6550 50  0001 L CNN "composition"
F 7 "Passive Components" H 4000 6650 50  0001 L CNN "device class L1"
F 8 "Resistors" H 4000 6750 50  0001 L CNN "device class L2"
F 9 "Chip SMD Resistors" H 4000 6850 50  0001 L CNN "device class L3"
F 10 "RES SMD 10K OHM 1% 1/10W 0603" H 4000 6950 50  0001 L CNN "digikey description"
F 11 "311-10.0KHRTR-ND" H 4000 7050 50  0001 L CNN "digikey part number"
F 12 "0.5mm" H 4000 7150 50  0001 L CNN "height"
F 13 "RESC15585X45" H 4000 7250 50  0001 L CNN "ipc land pattern name"
F 14 "yes" H 4000 7350 50  0001 L CNN "lead free"
F 15 "368bdb3a487a5469" H 4000 7450 50  0001 L CNN "library id"
F 16 "Yageo" H 4000 7550 50  0001 L CNN "manufacturer"
F 17 "Surface Mount Thick Film Resistor, RC Series, 10 kohm, 100 mW, - 1%, 50 V, 0603 [1608 Metric]" H 4000 7650 50  0001 L CNN "mouser description"
F 18 "603-RC0603FR-0710KL" H 4000 7750 50  0001 L CNN "mouser part number"
F 19 "0603" H 4350 5964 50  0000 C CNN "package"
F 20 "100mW" H 4000 7950 50  0001 L CNN "power"
F 21 "0.1W" H 4000 8050 50  0001 L CNN "power rating"
F 22 "10kΩ" H 4350 5873 50  0000 C CNN "resistance"
F 23 "yes" H 4000 8250 50  0001 L CNN "rohs"
F 24 "RC" H 4000 8350 50  0001 L CNN "series"
F 25 "0mm" H 4000 8450 50  0001 L CNN "standoff height"
F 26 "100ppm/°C" H 4000 8550 50  0001 L CNN "temperature coefficient"
F 27 "+155°C" H 4000 8650 50  0001 L CNN "temperature range high"
F 28 "-55°C" H 4000 8750 50  0001 L CNN "temperature range low"
F 29 "1%" H 4000 8850 50  0001 L CNN "tolerance"
F 30 "75V" H 4000 8950 50  0001 L CNN "voltage"
F 31 "75V" H 4000 9050 50  0001 L CNN "voltage rating"
	1    4000 5750
	1    0    0    -1  
$EndComp
$Comp
L project:RC0603FR-0710KL R?
U 1 1 6012AA35
P 4000 6200
AR Path="/6012AA35" Ref="R?"  Part="1" 
AR Path="/5FBD86FB/6012AA35" Ref="R?"  Part="1" 
AR Path="/5FC1E32F/6012AA35" Ref="R?"  Part="1" 
F 0 "R?" H 4350 6505 50  0000 C CNN
F 1 "RC0603FR-0710KL" H 4000 6500 50  0001 L CNN
F 2 "Yageo-RC0603-0-0-*" H 4000 6600 50  0001 L CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RC_Group_51_RoHS_L_9.pdf" H 4000 6700 50  0001 L CNN
F 4 "541-1.00HHCT-ND" H 4000 6800 50  0001 L CNN "approved alternate part number"
F 5 "Res" H 4000 6900 50  0001 L CNN "category"
F 6 "Thick Film" H 4000 7000 50  0001 L CNN "composition"
F 7 "Passive Components" H 4000 7100 50  0001 L CNN "device class L1"
F 8 "Resistors" H 4000 7200 50  0001 L CNN "device class L2"
F 9 "Chip SMD Resistors" H 4000 7300 50  0001 L CNN "device class L3"
F 10 "RES SMD 10K OHM 1% 1/10W 0603" H 4000 7400 50  0001 L CNN "digikey description"
F 11 "311-10.0KHRTR-ND" H 4000 7500 50  0001 L CNN "digikey part number"
F 12 "0.5mm" H 4000 7600 50  0001 L CNN "height"
F 13 "RESC15585X45" H 4000 7700 50  0001 L CNN "ipc land pattern name"
F 14 "yes" H 4000 7800 50  0001 L CNN "lead free"
F 15 "368bdb3a487a5469" H 4000 7900 50  0001 L CNN "library id"
F 16 "Yageo" H 4000 8000 50  0001 L CNN "manufacturer"
F 17 "Surface Mount Thick Film Resistor, RC Series, 10 kohm, 100 mW, - 1%, 50 V, 0603 [1608 Metric]" H 4000 8100 50  0001 L CNN "mouser description"
F 18 "603-RC0603FR-0710KL" H 4000 8200 50  0001 L CNN "mouser part number"
F 19 "0603" H 4350 6414 50  0000 C CNN "package"
F 20 "100mW" H 4000 8400 50  0001 L CNN "power"
F 21 "0.1W" H 4000 8500 50  0001 L CNN "power rating"
F 22 "10kΩ" H 4350 6323 50  0000 C CNN "resistance"
F 23 "yes" H 4000 8700 50  0001 L CNN "rohs"
F 24 "RC" H 4000 8800 50  0001 L CNN "series"
F 25 "0mm" H 4000 8900 50  0001 L CNN "standoff height"
F 26 "100ppm/°C" H 4000 9000 50  0001 L CNN "temperature coefficient"
F 27 "+155°C" H 4000 9100 50  0001 L CNN "temperature range high"
F 28 "-55°C" H 4000 9200 50  0001 L CNN "temperature range low"
F 29 "1%" H 4000 9300 50  0001 L CNN "tolerance"
F 30 "75V" H 4000 9400 50  0001 L CNN "voltage"
F 31 "75V" H 4000 9500 50  0001 L CNN "voltage rating"
	1    4000 6200
	1    0    0    -1  
$EndComp
Text Notes 8750 2350 0    50   ~ 0
In plane field sensor
$Comp
L project:CC0805ZKY5V6BB106 C?
U 1 1 606CA4FF
P 750 1400
F 0 "C?" H 878 1437 50  0000 L CNN
F 1 "CC0805ZKY5V6BB106" H 750 1600 50  0001 L CNN
F 2 "YAGEO-CC0805-0-0-*" H 750 1700 50  0001 L CNN
F 3 "http://www.yageo.com/documents/recent/UPY-GPHC_Y5V_6.3V-to-50V_8.pdf" H 750 1800 50  0001 L CNN
F 4 "10uF" H 878 1346 50  0000 L CNN "capacitance"
F 5 "Cap" H 750 2000 50  0001 L CNN "category"
F 6 "CAP CER 10UF 10V Y5V 0805" H 750 2100 50  0001 L CNN "digikey description"
F 7 "311-1355-1-ND" H 750 2200 50  0001 L CNN "digikey part number"
F 8 "yes" H 750 2300 50  0001 L CNN "lead free"
F 9 "852e7d0cf36849be" H 750 2400 50  0001 L CNN "library id"
F 10 "YAGEO" H 750 2500 50  0001 L CNN "manufacturer"
F 11 "Multilayer Ceramic Capacitors MLCC - SMD/SMT 10uF 10V Y5V-20/+80%" H 750 2600 50  0001 L CNN "mouser description"
F 12 "603-CC805ZKY5V6BB106" H 750 2700 50  0001 L CNN "mouser part number"
F 13 "0805" H 878 1255 50  0000 L CNN "package"
F 14 "yes" H 750 2900 50  0001 L CNN "rohs"
F 15 "+85°C" H 750 3000 50  0001 L CNN "temperature range high"
F 16 "-30°C" H 750 3100 50  0001 L CNN "temperature range low"
F 17 "10V" H 878 1164 50  0000 L CNN "voltage"
	1    750  1400
	1    0    0    -1  
$EndComp
Wire Wire Line
	750  1050 1750 1050
Wire Wire Line
	750  1050 750  1400
Connection ~ 1750 1050
Wire Wire Line
	750  1600 750  1800
Wire Wire Line
	750  1800 950  1800
Connection ~ 950  1800
$Comp
L project:Field_Sensor_54_22 Y?
U 1 1 60985765
P 8750 2100
F 0 "Y?" H 9278 2146 50  0000 L CNN
F 1 "Field_Sensor_54_22" H 9278 2055 50  0000 L CNN
F 2 "project:Field_Sensor_54_22" H 9750 1900 50  0001 C CNN
F 3 "" H 8750 2100 50  0001 C CNN
	1    8750 2100
	1    0    0    -1  
$EndComp
$Comp
L project:BAS40-7-F D?
U 1 1 609B0CD2
P 7400 6100
F 0 "D?" H 7900 5545 50  0000 C CNN
F 1 "BAS40-7-F" H 7900 5636 50  0000 C CNN
F 2 "Diodes_Inc.-SOT23-01_2017-0-*" H 7400 6600 50  0001 L CNN
F 3 "https://www.diodes.com/assets/Datasheets/ds11006.pdf" H 7400 6700 50  0001 L CNN
F 4 "Yes" H 7400 6800 50  0001 L CNN "automotive"
F 5 "Grade 1" H 7400 6900 50  0001 L CNN "automotive grade"
F 6 "4pF" H 7400 7000 50  0001 L CNN "capacitance"
F 7 "Diode" H 7400 7100 50  0001 L CNN "category"
F 8 "Discrete Semiconductors" H 7400 7200 50  0001 L CNN "device class L1"
F 9 "Diodes" H 7400 7300 50  0001 L CNN "device class L2"
F 10 "Schottky Diodes" H 7400 7400 50  0001 L CNN "device class L3"
F 11 "DIODE SCHOTTKY 40V 200MA SOT23-3" H 7400 7500 50  0001 L CNN "digikey description"
F 12 "BAS40-FDICT-ND" H 7400 7600 50  0001 L CNN "digikey part number"
F 13 "https://www.diodes.com/assets/Package-Files/SOT23.pdf" H 7400 7700 50  0001 L CNN "footprint url"
F 14 "200mA" H 7400 7800 50  0001 L CNN "forward current"
F 15 "1.1mm" H 7400 7900 50  0001 L CNN "height"
F 16 "Yes" H 7400 8000 50  0001 L CNN "lead free"
F 17 "c4a7d7c409903337" H 7400 8100 50  0001 L CNN "library id"
F 18 "Diodes Inc." H 7400 8200 50  0001 L CNN "manufacturer"
F 19 "+125°C" H 7400 8300 50  0001 L CNN "max junction temp"
F 20 "40V" H 7400 8400 50  0001 L CNN "max repetitive reverse voltage"
F 21 "600mA" H 7400 8500 50  0001 L CNN "max surge forward current"
F 22 "Schottky Diodes & Rectifiers 40V 350mW" H 7400 8600 50  0001 L CNN "mouser description"
F 23 "621-BAS40-F" H 7400 8700 50  0001 L CNN "mouser part number"
F 24 "SOT23" H 7900 5727 50  0000 C CNN "package"
F 25 "0.2uA" H 7400 8900 50  0001 L CNN "peak reverse current"
F 26 "40V" H 7400 9000 50  0001 L CNN "reverse voltage"
F 27 "Yes" H 7400 9100 50  0001 L CNN "rohs"
F 28 "0.013mm" H 7400 9200 50  0001 L CNN "standoff height"
F 29 "+125°C" H 7400 9300 50  0001 L CNN "temperature range high"
F 30 "-55°C" H 7400 9400 50  0001 L CNN "temperature range low"
	1    7400 6100
	-1   0    0    1   
$EndComp
$Comp
L project:CC0603KRX7R9BB102 C?
U 1 1 609B2086
P 6200 6250
AR Path="/5FBD86FB/609B2086" Ref="C?"  Part="1" 
AR Path="/5FC1DCBA/609B2086" Ref="C?"  Part="1" 
AR Path="/5FC1E11E/609B2086" Ref="C?"  Part="1" 
AR Path="/5FC1E1DD/609B2086" Ref="C?"  Part="1" 
AR Path="/5FC1E32F/609B2086" Ref="C?"  Part="1" 
AR Path="/5FC1FA4C/609B2086" Ref="C?"  Part="1" 
AR Path="/5FC1FA54/609B2086" Ref="C?"  Part="1" 
AR Path="/5FC1FA5C/609B2086" Ref="C?"  Part="1" 
AR Path="/5FC1FA64/609B2086" Ref="C?"  Part="1" 
AR Path="/5FC1FA6C/609B2086" Ref="C?"  Part="1" 
F 0 "C?" H 6328 6287 50  0000 L CNN
F 1 "CC0603KRX7R9BB102" H 6200 6450 50  0001 L CNN
F 2 "YAGEO-CC0603-DA-05_2018-24-*" H 6200 6550 50  0001 L CNN
F 3 "http://www.yageo.com/documents/recent/UPY-GPHC_X7R_6.3V-to-50V_18.pdf" H 6200 6650 50  0001 L CNN
F 4 "No" H 6200 6750 50  0001 L CNN "automotive"
F 5 "1.0 nF" H 6328 6196 50  0000 L CNN "capacitance"
F 6 "Cap" H 6200 6950 50  0001 L CNN "category"
F 7 "Passive Components" H 6200 7050 50  0001 L CNN "device class L1"
F 8 "Capacitors" H 6200 7150 50  0001 L CNN "device class L2"
F 9 "Ceramic Capacitors" H 6200 7250 50  0001 L CNN "device class L3"
F 10 "CAP CER 1000PF 50V X7R 0603" H 6200 7350 50  0001 L CNN "digikey description"
F 11 "311-1080-2-ND" H 6200 7450 50  0001 L CNN "digikey part number"
F 12 "http://www.yageo.com/exep/pages/download/literatures/UPY-C_GEN_24.pdf" H 6200 7550 50  0001 L CNN "footprint url"
F 13 "0.9mm" H 6200 7650 50  0001 L CNN "height"
F 14 "Yes" H 6200 7750 50  0001 L CNN "lead free"
F 15 "1aeb194455b4965b" H 6200 7850 50  0001 L CNN "library id"
F 16 "YAGEO" H 6200 7950 50  0001 L CNN "manufacturer"
F 17 "Ceramic" H 6200 8050 50  0001 L CNN "material"
F 18 "Multilayer Ceramic Capacitors MLCC - SMD/SMT 1.0nF 50V X7R 10%" H 6200 8150 50  0001 L CNN "mouser description"
F 19 "603-CC603KRX7R9BB102" H 6200 8250 50  0001 L CNN "mouser part number"
F 20 "0603" H 6328 6105 50  0000 L CNN "package"
F 21 "Yes" H 6200 8450 50  0001 L CNN "rohs"
F 22 "X7R" H 6200 8550 50  0001 L CNN "temperature characteristic"
F 23 "15%" H 6200 8650 50  0001 L CNN "temperature coefficient"
F 24 "+125°C" H 6200 8750 50  0001 L CNN "temperature range high"
F 25 "-55°C" H 6200 8850 50  0001 L CNN "temperature range low"
F 26 "0.1" H 6200 8950 50  0001 L CNN "tolerance"
F 27 "50 V" H 6328 6014 50  0000 L CNN "voltage"
F 28 "50 V" H 6200 9150 50  0001 L CNN "voltage rating"
	1    6200 6250
	1    0    0    -1  
$EndComp
Wire Wire Line
	6500 6100 6200 6100
Wire Wire Line
	6200 6100 6200 6250
Wire Wire Line
	8750 2050 8350 2050
Wire Wire Line
	8750 2150 8350 2150
Text Label 8400 2050 0    50   ~ 0
HSEN1
Text Label 8400 2150 0    50   ~ 0
GND
Wire Wire Line
	7300 6100 7350 6100
Text Label 7350 6100 0    50   ~ 0
HSEN2
$Comp
L project:RC0603FR-0710KL R?
U 1 1 60A56DFB
P 5550 6100
AR Path="/60A56DFB" Ref="R?"  Part="1" 
AR Path="/5FBD86FB/60A56DFB" Ref="R?"  Part="1" 
AR Path="/5FC1E32F/60A56DFB" Ref="R?"  Part="1" 
F 0 "R?" H 5900 6405 50  0000 C CNN
F 1 "RC0603FR-0710KL" H 5550 6400 50  0001 L CNN
F 2 "Yageo-RC0603-0-0-*" H 5550 6500 50  0001 L CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RC_Group_51_RoHS_L_9.pdf" H 5550 6600 50  0001 L CNN
F 4 "541-1.00HHCT-ND" H 5550 6700 50  0001 L CNN "approved alternate part number"
F 5 "Res" H 5550 6800 50  0001 L CNN "category"
F 6 "Thick Film" H 5550 6900 50  0001 L CNN "composition"
F 7 "Passive Components" H 5550 7000 50  0001 L CNN "device class L1"
F 8 "Resistors" H 5550 7100 50  0001 L CNN "device class L2"
F 9 "Chip SMD Resistors" H 5550 7200 50  0001 L CNN "device class L3"
F 10 "RES SMD 10K OHM 1% 1/10W 0603" H 5550 7300 50  0001 L CNN "digikey description"
F 11 "311-10.0KHRTR-ND" H 5550 7400 50  0001 L CNN "digikey part number"
F 12 "0.5mm" H 5550 7500 50  0001 L CNN "height"
F 13 "RESC15585X45" H 5550 7600 50  0001 L CNN "ipc land pattern name"
F 14 "yes" H 5550 7700 50  0001 L CNN "lead free"
F 15 "368bdb3a487a5469" H 5550 7800 50  0001 L CNN "library id"
F 16 "Yageo" H 5550 7900 50  0001 L CNN "manufacturer"
F 17 "Surface Mount Thick Film Resistor, RC Series, 10 kohm, 100 mW, - 1%, 50 V, 0603 [1608 Metric]" H 5550 8000 50  0001 L CNN "mouser description"
F 18 "603-RC0603FR-0710KL" H 5550 8100 50  0001 L CNN "mouser part number"
F 19 "0603" H 5900 6314 50  0000 C CNN "package"
F 20 "100mW" H 5550 8300 50  0001 L CNN "power"
F 21 "0.1W" H 5550 8400 50  0001 L CNN "power rating"
F 22 "10kΩ" H 5900 6223 50  0000 C CNN "resistance"
F 23 "yes" H 5550 8600 50  0001 L CNN "rohs"
F 24 "RC" H 5550 8700 50  0001 L CNN "series"
F 25 "0mm" H 5550 8800 50  0001 L CNN "standoff height"
F 26 "100ppm/°C" H 5550 8900 50  0001 L CNN "temperature coefficient"
F 27 "+155°C" H 5550 9000 50  0001 L CNN "temperature range high"
F 28 "-55°C" H 5550 9100 50  0001 L CNN "temperature range low"
F 29 "1%" H 5550 9200 50  0001 L CNN "tolerance"
F 30 "75V" H 5550 9300 50  0001 L CNN "voltage"
F 31 "75V" H 5550 9400 50  0001 L CNN "voltage rating"
	1    5550 6100
	1    0    0    -1  
$EndComp
Wire Wire Line
	6150 6100 6200 6100
Connection ~ 6200 6100
$Comp
L project:RC0603FR-0710KL R?
U 1 1 60A64AEF
P 5650 6050
AR Path="/60A64AEF" Ref="R?"  Part="1" 
AR Path="/5FBD86FB/60A64AEF" Ref="R?"  Part="1" 
AR Path="/5FC1E32F/60A64AEF" Ref="R?"  Part="1" 
F 0 "R?" V 5909 6128 50  0000 L CNN
F 1 "RC0603FR-0710KL" H 5650 6350 50  0001 L CNN
F 2 "Yageo-RC0603-0-0-*" H 5650 6450 50  0001 L CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RC_Group_51_RoHS_L_9.pdf" H 5650 6550 50  0001 L CNN
F 4 "541-1.00HHCT-ND" H 5650 6650 50  0001 L CNN "approved alternate part number"
F 5 "Res" H 5650 6750 50  0001 L CNN "category"
F 6 "Thick Film" H 5650 6850 50  0001 L CNN "composition"
F 7 "Passive Components" H 5650 6950 50  0001 L CNN "device class L1"
F 8 "Resistors" H 5650 7050 50  0001 L CNN "device class L2"
F 9 "Chip SMD Resistors" H 5650 7150 50  0001 L CNN "device class L3"
F 10 "RES SMD 10K OHM 1% 1/10W 0603" H 5650 7250 50  0001 L CNN "digikey description"
F 11 "311-10.0KHRTR-ND" H 5650 7350 50  0001 L CNN "digikey part number"
F 12 "0.5mm" H 5650 7450 50  0001 L CNN "height"
F 13 "RESC15585X45" H 5650 7550 50  0001 L CNN "ipc land pattern name"
F 14 "yes" H 5650 7650 50  0001 L CNN "lead free"
F 15 "368bdb3a487a5469" H 5650 7750 50  0001 L CNN "library id"
F 16 "Yageo" H 5650 7850 50  0001 L CNN "manufacturer"
F 17 "Surface Mount Thick Film Resistor, RC Series, 10 kohm, 100 mW, - 1%, 50 V, 0603 [1608 Metric]" H 5650 7950 50  0001 L CNN "mouser description"
F 18 "603-RC0603FR-0710KL" H 5650 8050 50  0001 L CNN "mouser part number"
F 19 "0603" V 6000 6128 50  0000 L CNN "package"
F 20 "100mW" H 5650 8250 50  0001 L CNN "power"
F 21 "0.1W" H 5650 8350 50  0001 L CNN "power rating"
F 22 "10kΩ" V 6091 6128 50  0000 L CNN "resistance"
F 23 "yes" H 5650 8550 50  0001 L CNN "rohs"
F 24 "RC" H 5650 8650 50  0001 L CNN "series"
F 25 "0mm" H 5650 8750 50  0001 L CNN "standoff height"
F 26 "100ppm/°C" H 5650 8850 50  0001 L CNN "temperature coefficient"
F 27 "+155°C" H 5650 8950 50  0001 L CNN "temperature range high"
F 28 "-55°C" H 5650 9050 50  0001 L CNN "temperature range low"
F 29 "1%" H 5650 9150 50  0001 L CNN "tolerance"
F 30 "75V" H 5650 9250 50  0001 L CNN "voltage"
F 31 "75V" H 5650 9350 50  0001 L CNN "voltage rating"
	1    5650 6050
	0    1    1    0   
$EndComp
Wire Wire Line
	5650 6100 5650 6150
Wire Wire Line
	5650 6650 6200 6650
Wire Wire Line
	6200 6450 6200 6650
$Comp
L project:C0603C209C5GACTU C?
U 1 1 60AAAD59
P 7350 6250
F 0 "C?" H 7478 6287 50  0000 L CNN
F 1 "C0603C209C5GACTU" H 7478 6241 50  0001 L CNN
F 2 "KEMET-C0603C-CF-0-0-*" H 7350 6550 50  0001 L CNN
F 3 "https://api.kemet.com/component-edge/download/datasheet/C0603C100J5GACAUTO.pdf" H 7350 6650 50  0001 L CNN
F 4 "Yes" H 7350 6750 50  0001 L CNN "automotive"
F 5 "Grade 1" H 7350 6850 50  0001 L CNN "automotive grade"
F 6 "2.0 pF" H 7478 6196 50  0000 L CNN "capacitance"
F 7 "Cap" H 7350 7050 50  0001 L CNN "category"
F 8 "Passive Components" H 7350 7150 50  0001 L CNN "device class L1"
F 9 "Capacitors" H 7350 7250 50  0001 L CNN "device class L2"
F 10 "Ceramic Capacitors" H 7350 7350 50  0001 L CNN "device class L3"
F 11 "CAP CER 2PF 50V C0G/NP0 0603" H 7350 7450 50  0001 L CNN "digikey description"
F 12 "399-1047-2-ND" H 7350 7550 50  0001 L CNN "digikey part number"
F 13 "0.87mm" H 7350 7650 50  0001 L CNN "height"
F 14 "Yes" H 7350 7750 50  0001 L CNN "lead free"
F 15 "4271a13c665e43de" H 7350 7850 50  0001 L CNN "library id"
F 16 "KEMET" H 7350 7950 50  0001 L CNN "manufacturer"
F 17 "Ceramic" H 7350 8050 50  0001 L CNN "material"
F 18 "Multilayer Ceramic Capacitors MLCC - SMD/SMT 50volts 2.0pF C0G 0.25%" H 7350 8150 50  0001 L CNN "mouser description"
F 19 "80-C0603C209C5G" H 7350 8250 50  0001 L CNN "mouser part number"
F 20 "0603" H 7478 6105 50  0000 L CNN "package"
F 21 "Yes" H 7350 8450 50  0001 L CNN "rohs"
F 22 "C0G" H 7350 8550 50  0001 L CNN "temperature characteristic"
F 23 "30ppm/°C" H 7350 8650 50  0001 L CNN "temperature coefficient"
F 24 "+125°C" H 7350 8750 50  0001 L CNN "temperature range high"
F 25 "-55°C" H 7350 8850 50  0001 L CNN "temperature range low"
F 26 "0.25 pF" H 7350 8950 50  0001 L CNN "tolerance"
F 27 "50 V" H 7478 6014 50  0000 L CNN "voltage rating"
	1    7350 6250
	1    0    0    -1  
$EndComp
$Comp
L project:CC0603KRX7R9BB103 C?
U 1 1 60AAD48F
P 5200 6250
AR Path="/5FBD86FB/60AAD48F" Ref="C?"  Part="1" 
AR Path="/5FC1DCBA/60AAD48F" Ref="C?"  Part="1" 
AR Path="/5FC1E11E/60AAD48F" Ref="C?"  Part="1" 
AR Path="/5FC1E1DD/60AAD48F" Ref="C?"  Part="1" 
AR Path="/5FC1E32F/60AAD48F" Ref="C?"  Part="1" 
AR Path="/5FC1FA4C/60AAD48F" Ref="C?"  Part="1" 
AR Path="/5FC1FA54/60AAD48F" Ref="C?"  Part="1" 
AR Path="/5FC1FA5C/60AAD48F" Ref="C?"  Part="1" 
AR Path="/5FC1FA64/60AAD48F" Ref="C?"  Part="1" 
AR Path="/5FC1FA6C/60AAD48F" Ref="C?"  Part="1" 
F 0 "C?" H 5328 6287 50  0000 L CNN
F 1 "CC0603KRX7R9BB103" H 5200 6450 50  0001 L CNN
F 2 "YAGEO-CC0603-DA-05_2018-24-*" H 5200 6550 50  0001 L CNN
F 3 "http://www.yageo.com/documents/recent/UPY-GPHC_X7R_6.3V-to-50V_18.pdf" H 5200 6650 50  0001 L CNN
F 4 "No" H 5200 6750 50  0001 L CNN "automotive"
F 5 "10 nF" H 5328 6196 50  0000 L CNN "capacitance"
F 6 "Cap" H 5200 6950 50  0001 L CNN "category"
F 7 "Passive Components" H 5200 7050 50  0001 L CNN "device class L1"
F 8 "Capacitors" H 5200 7150 50  0001 L CNN "device class L2"
F 9 "Ceramic Capacitors" H 5200 7250 50  0001 L CNN "device class L3"
F 10 "CAP CER 10000PF 50V X7R 0603" H 5200 7350 50  0001 L CNN "digikey description"
F 11 "311-1085-2-ND" H 5200 7450 50  0001 L CNN "digikey part number"
F 12 "http://www.yageo.com/exep/pages/download/literatures/UPY-C_GEN_24.pdf" H 5200 7550 50  0001 L CNN "footprint url"
F 13 "0.9mm" H 5200 7650 50  0001 L CNN "height"
F 14 "Yes" H 5200 7750 50  0001 L CNN "lead free"
F 15 "2a4c50056f7872c7" H 5200 7850 50  0001 L CNN "library id"
F 16 "YAGEO" H 5200 7950 50  0001 L CNN "manufacturer"
F 17 "Ceramic" H 5200 8050 50  0001 L CNN "material"
F 18 "Multilayer Ceramic Capacitor, CC Series, 0.01 - F, - 10%, X7R, 50 V, 0603 [1608 Metric]" H 5200 8150 50  0001 L CNN "mouser description"
F 19 "603-CC603KRX7R9BB103" H 5200 8250 50  0001 L CNN "mouser part number"
F 20 "0603" H 5328 6105 50  0000 L CNN "package"
F 21 "Yes" H 5200 8450 50  0001 L CNN "rohs"
F 22 "X7R" H 5200 8550 50  0001 L CNN "temperature characteristic"
F 23 "15%" H 5200 8650 50  0001 L CNN "temperature coefficient"
F 24 "+125°C" H 5200 8750 50  0001 L CNN "temperature range high"
F 25 "-55°C" H 5200 8850 50  0001 L CNN "temperature range low"
F 26 "0.1" H 5200 8950 50  0001 L CNN "tolerance"
F 27 "50 V" H 5328 6014 50  0000 L CNN "voltage"
F 28 "50 V" H 5200 9150 50  0001 L CNN "voltage rating"
	1    5200 6250
	1    0    0    -1  
$EndComp
Wire Wire Line
	5200 6650 5650 6650
Connection ~ 5650 6650
Wire Wire Line
	7350 6250 7350 6100
Connection ~ 7350 6100
Wire Wire Line
	7350 6100 7600 6100
Wire Wire Line
	7350 6450 7350 6650
Wire Wire Line
	7350 6650 6200 6650
Connection ~ 6200 6650
Connection ~ 5650 6100
Wire Wire Line
	5650 6100 5200 6100
Wire Wire Line
	5200 6650 5200 6450
Wire Wire Line
	5200 6250 5200 6100
NoConn ~ 7300 5900
Text Label 6750 6650 0    50   ~ 0
GND
$Comp
L project:BAS40-7-F D?
U 1 1 60B9BE1A
P 7400 4900
F 0 "D?" H 7900 4345 50  0000 C CNN
F 1 "BAS40-7-F" H 7900 4436 50  0000 C CNN
F 2 "Diodes_Inc.-SOT23-01_2017-0-*" H 7400 5400 50  0001 L CNN
F 3 "https://www.diodes.com/assets/Datasheets/ds11006.pdf" H 7400 5500 50  0001 L CNN
F 4 "Yes" H 7400 5600 50  0001 L CNN "automotive"
F 5 "Grade 1" H 7400 5700 50  0001 L CNN "automotive grade"
F 6 "4pF" H 7400 5800 50  0001 L CNN "capacitance"
F 7 "Diode" H 7400 5900 50  0001 L CNN "category"
F 8 "Discrete Semiconductors" H 7400 6000 50  0001 L CNN "device class L1"
F 9 "Diodes" H 7400 6100 50  0001 L CNN "device class L2"
F 10 "Schottky Diodes" H 7400 6200 50  0001 L CNN "device class L3"
F 11 "DIODE SCHOTTKY 40V 200MA SOT23-3" H 7400 6300 50  0001 L CNN "digikey description"
F 12 "BAS40-FDICT-ND" H 7400 6400 50  0001 L CNN "digikey part number"
F 13 "https://www.diodes.com/assets/Package-Files/SOT23.pdf" H 7400 6500 50  0001 L CNN "footprint url"
F 14 "200mA" H 7400 6600 50  0001 L CNN "forward current"
F 15 "1.1mm" H 7400 6700 50  0001 L CNN "height"
F 16 "Yes" H 7400 6800 50  0001 L CNN "lead free"
F 17 "c4a7d7c409903337" H 7400 6900 50  0001 L CNN "library id"
F 18 "Diodes Inc." H 7400 7000 50  0001 L CNN "manufacturer"
F 19 "+125°C" H 7400 7100 50  0001 L CNN "max junction temp"
F 20 "40V" H 7400 7200 50  0001 L CNN "max repetitive reverse voltage"
F 21 "600mA" H 7400 7300 50  0001 L CNN "max surge forward current"
F 22 "Schottky Diodes & Rectifiers 40V 350mW" H 7400 7400 50  0001 L CNN "mouser description"
F 23 "621-BAS40-F" H 7400 7500 50  0001 L CNN "mouser part number"
F 24 "SOT23" H 7900 4527 50  0000 C CNN "package"
F 25 "0.2uA" H 7400 7700 50  0001 L CNN "peak reverse current"
F 26 "40V" H 7400 7800 50  0001 L CNN "reverse voltage"
F 27 "Yes" H 7400 7900 50  0001 L CNN "rohs"
F 28 "0.013mm" H 7400 8000 50  0001 L CNN "standoff height"
F 29 "+125°C" H 7400 8100 50  0001 L CNN "temperature range high"
F 30 "-55°C" H 7400 8200 50  0001 L CNN "temperature range low"
	1    7400 4900
	-1   0    0    1   
$EndComp
$Comp
L project:CC0603KRX7R9BB102 C?
U 1 1 60B9BE39
P 6200 5050
AR Path="/5FBD86FB/60B9BE39" Ref="C?"  Part="1" 
AR Path="/5FC1DCBA/60B9BE39" Ref="C?"  Part="1" 
AR Path="/5FC1E11E/60B9BE39" Ref="C?"  Part="1" 
AR Path="/5FC1E1DD/60B9BE39" Ref="C?"  Part="1" 
AR Path="/5FC1E32F/60B9BE39" Ref="C?"  Part="1" 
AR Path="/5FC1FA4C/60B9BE39" Ref="C?"  Part="1" 
AR Path="/5FC1FA54/60B9BE39" Ref="C?"  Part="1" 
AR Path="/5FC1FA5C/60B9BE39" Ref="C?"  Part="1" 
AR Path="/5FC1FA64/60B9BE39" Ref="C?"  Part="1" 
AR Path="/5FC1FA6C/60B9BE39" Ref="C?"  Part="1" 
F 0 "C?" H 6328 5087 50  0000 L CNN
F 1 "CC0603KRX7R9BB102" H 6200 5250 50  0001 L CNN
F 2 "YAGEO-CC0603-DA-05_2018-24-*" H 6200 5350 50  0001 L CNN
F 3 "http://www.yageo.com/documents/recent/UPY-GPHC_X7R_6.3V-to-50V_18.pdf" H 6200 5450 50  0001 L CNN
F 4 "No" H 6200 5550 50  0001 L CNN "automotive"
F 5 "1.0 nF" H 6328 4996 50  0000 L CNN "capacitance"
F 6 "Cap" H 6200 5750 50  0001 L CNN "category"
F 7 "Passive Components" H 6200 5850 50  0001 L CNN "device class L1"
F 8 "Capacitors" H 6200 5950 50  0001 L CNN "device class L2"
F 9 "Ceramic Capacitors" H 6200 6050 50  0001 L CNN "device class L3"
F 10 "CAP CER 1000PF 50V X7R 0603" H 6200 6150 50  0001 L CNN "digikey description"
F 11 "311-1080-2-ND" H 6200 6250 50  0001 L CNN "digikey part number"
F 12 "http://www.yageo.com/exep/pages/download/literatures/UPY-C_GEN_24.pdf" H 6200 6350 50  0001 L CNN "footprint url"
F 13 "0.9mm" H 6200 6450 50  0001 L CNN "height"
F 14 "Yes" H 6200 6550 50  0001 L CNN "lead free"
F 15 "1aeb194455b4965b" H 6200 6650 50  0001 L CNN "library id"
F 16 "YAGEO" H 6200 6750 50  0001 L CNN "manufacturer"
F 17 "Ceramic" H 6200 6850 50  0001 L CNN "material"
F 18 "Multilayer Ceramic Capacitors MLCC - SMD/SMT 1.0nF 50V X7R 10%" H 6200 6950 50  0001 L CNN "mouser description"
F 19 "603-CC603KRX7R9BB102" H 6200 7050 50  0001 L CNN "mouser part number"
F 20 "0603" H 6328 4905 50  0000 L CNN "package"
F 21 "Yes" H 6200 7250 50  0001 L CNN "rohs"
F 22 "X7R" H 6200 7350 50  0001 L CNN "temperature characteristic"
F 23 "15%" H 6200 7450 50  0001 L CNN "temperature coefficient"
F 24 "+125°C" H 6200 7550 50  0001 L CNN "temperature range high"
F 25 "-55°C" H 6200 7650 50  0001 L CNN "temperature range low"
F 26 "0.1" H 6200 7750 50  0001 L CNN "tolerance"
F 27 "50 V" H 6328 4814 50  0000 L CNN "voltage"
F 28 "50 V" H 6200 7950 50  0001 L CNN "voltage rating"
	1    6200 5050
	1    0    0    -1  
$EndComp
Wire Wire Line
	6500 4900 6200 4900
Wire Wire Line
	6200 4900 6200 5050
Wire Wire Line
	7300 4900 7350 4900
Text Label 7350 4900 0    50   ~ 0
HSEN1
$Comp
L project:RC0603FR-0710KL R?
U 1 1 60B9BE5F
P 5550 4900
AR Path="/60B9BE5F" Ref="R?"  Part="1" 
AR Path="/5FBD86FB/60B9BE5F" Ref="R?"  Part="1" 
AR Path="/5FC1E32F/60B9BE5F" Ref="R?"  Part="1" 
F 0 "R?" H 5900 5205 50  0000 C CNN
F 1 "RC0603FR-0710KL" H 5550 5200 50  0001 L CNN
F 2 "Yageo-RC0603-0-0-*" H 5550 5300 50  0001 L CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RC_Group_51_RoHS_L_9.pdf" H 5550 5400 50  0001 L CNN
F 4 "541-1.00HHCT-ND" H 5550 5500 50  0001 L CNN "approved alternate part number"
F 5 "Res" H 5550 5600 50  0001 L CNN "category"
F 6 "Thick Film" H 5550 5700 50  0001 L CNN "composition"
F 7 "Passive Components" H 5550 5800 50  0001 L CNN "device class L1"
F 8 "Resistors" H 5550 5900 50  0001 L CNN "device class L2"
F 9 "Chip SMD Resistors" H 5550 6000 50  0001 L CNN "device class L3"
F 10 "RES SMD 10K OHM 1% 1/10W 0603" H 5550 6100 50  0001 L CNN "digikey description"
F 11 "311-10.0KHRTR-ND" H 5550 6200 50  0001 L CNN "digikey part number"
F 12 "0.5mm" H 5550 6300 50  0001 L CNN "height"
F 13 "RESC15585X45" H 5550 6400 50  0001 L CNN "ipc land pattern name"
F 14 "yes" H 5550 6500 50  0001 L CNN "lead free"
F 15 "368bdb3a487a5469" H 5550 6600 50  0001 L CNN "library id"
F 16 "Yageo" H 5550 6700 50  0001 L CNN "manufacturer"
F 17 "Surface Mount Thick Film Resistor, RC Series, 10 kohm, 100 mW, - 1%, 50 V, 0603 [1608 Metric]" H 5550 6800 50  0001 L CNN "mouser description"
F 18 "603-RC0603FR-0710KL" H 5550 6900 50  0001 L CNN "mouser part number"
F 19 "0603" H 5900 5114 50  0000 C CNN "package"
F 20 "100mW" H 5550 7100 50  0001 L CNN "power"
F 21 "0.1W" H 5550 7200 50  0001 L CNN "power rating"
F 22 "10kΩ" H 5900 5023 50  0000 C CNN "resistance"
F 23 "yes" H 5550 7400 50  0001 L CNN "rohs"
F 24 "RC" H 5550 7500 50  0001 L CNN "series"
F 25 "0mm" H 5550 7600 50  0001 L CNN "standoff height"
F 26 "100ppm/°C" H 5550 7700 50  0001 L CNN "temperature coefficient"
F 27 "+155°C" H 5550 7800 50  0001 L CNN "temperature range high"
F 28 "-55°C" H 5550 7900 50  0001 L CNN "temperature range low"
F 29 "1%" H 5550 8000 50  0001 L CNN "tolerance"
F 30 "75V" H 5550 8100 50  0001 L CNN "voltage"
F 31 "75V" H 5550 8200 50  0001 L CNN "voltage rating"
	1    5550 4900
	1    0    0    -1  
$EndComp
Wire Wire Line
	6150 4900 6200 4900
Connection ~ 6200 4900
$Comp
L project:RC0603FR-0710KL R?
U 1 1 60B9BE83
P 5650 4850
AR Path="/60B9BE83" Ref="R?"  Part="1" 
AR Path="/5FBD86FB/60B9BE83" Ref="R?"  Part="1" 
AR Path="/5FC1E32F/60B9BE83" Ref="R?"  Part="1" 
F 0 "R?" V 5909 4928 50  0000 L CNN
F 1 "RC0603FR-0710KL" H 5650 5150 50  0001 L CNN
F 2 "Yageo-RC0603-0-0-*" H 5650 5250 50  0001 L CNN
F 3 "http://www.yageo.com/documents/recent/PYu-RC_Group_51_RoHS_L_9.pdf" H 5650 5350 50  0001 L CNN
F 4 "541-1.00HHCT-ND" H 5650 5450 50  0001 L CNN "approved alternate part number"
F 5 "Res" H 5650 5550 50  0001 L CNN "category"
F 6 "Thick Film" H 5650 5650 50  0001 L CNN "composition"
F 7 "Passive Components" H 5650 5750 50  0001 L CNN "device class L1"
F 8 "Resistors" H 5650 5850 50  0001 L CNN "device class L2"
F 9 "Chip SMD Resistors" H 5650 5950 50  0001 L CNN "device class L3"
F 10 "RES SMD 10K OHM 1% 1/10W 0603" H 5650 6050 50  0001 L CNN "digikey description"
F 11 "311-10.0KHRTR-ND" H 5650 6150 50  0001 L CNN "digikey part number"
F 12 "0.5mm" H 5650 6250 50  0001 L CNN "height"
F 13 "RESC15585X45" H 5650 6350 50  0001 L CNN "ipc land pattern name"
F 14 "yes" H 5650 6450 50  0001 L CNN "lead free"
F 15 "368bdb3a487a5469" H 5650 6550 50  0001 L CNN "library id"
F 16 "Yageo" H 5650 6650 50  0001 L CNN "manufacturer"
F 17 "Surface Mount Thick Film Resistor, RC Series, 10 kohm, 100 mW, - 1%, 50 V, 0603 [1608 Metric]" H 5650 6750 50  0001 L CNN "mouser description"
F 18 "603-RC0603FR-0710KL" H 5650 6850 50  0001 L CNN "mouser part number"
F 19 "0603" V 6000 4928 50  0000 L CNN "package"
F 20 "100mW" H 5650 7050 50  0001 L CNN "power"
F 21 "0.1W" H 5650 7150 50  0001 L CNN "power rating"
F 22 "10kΩ" V 6091 4928 50  0000 L CNN "resistance"
F 23 "yes" H 5650 7350 50  0001 L CNN "rohs"
F 24 "RC" H 5650 7450 50  0001 L CNN "series"
F 25 "0mm" H 5650 7550 50  0001 L CNN "standoff height"
F 26 "100ppm/°C" H 5650 7650 50  0001 L CNN "temperature coefficient"
F 27 "+155°C" H 5650 7750 50  0001 L CNN "temperature range high"
F 28 "-55°C" H 5650 7850 50  0001 L CNN "temperature range low"
F 29 "1%" H 5650 7950 50  0001 L CNN "tolerance"
F 30 "75V" H 5650 8050 50  0001 L CNN "voltage"
F 31 "75V" H 5650 8150 50  0001 L CNN "voltage rating"
	1    5650 4850
	0    1    1    0   
$EndComp
Wire Wire Line
	5650 4900 5650 4950
Wire Wire Line
	5650 5450 6200 5450
Wire Wire Line
	6200 5250 6200 5450
$Comp
L project:C0603C209C5GACTU C?
U 1 1 60B9BEA4
P 7350 5050
F 0 "C?" H 7478 5087 50  0000 L CNN
F 1 "C0603C209C5GACTU" H 7478 5041 50  0001 L CNN
F 2 "KEMET-C0603C-CF-0-0-*" H 7350 5350 50  0001 L CNN
F 3 "https://api.kemet.com/component-edge/download/datasheet/C0603C100J5GACAUTO.pdf" H 7350 5450 50  0001 L CNN
F 4 "Yes" H 7350 5550 50  0001 L CNN "automotive"
F 5 "Grade 1" H 7350 5650 50  0001 L CNN "automotive grade"
F 6 "2.0 pF" H 7478 4996 50  0000 L CNN "capacitance"
F 7 "Cap" H 7350 5850 50  0001 L CNN "category"
F 8 "Passive Components" H 7350 5950 50  0001 L CNN "device class L1"
F 9 "Capacitors" H 7350 6050 50  0001 L CNN "device class L2"
F 10 "Ceramic Capacitors" H 7350 6150 50  0001 L CNN "device class L3"
F 11 "CAP CER 2PF 50V C0G/NP0 0603" H 7350 6250 50  0001 L CNN "digikey description"
F 12 "399-1047-2-ND" H 7350 6350 50  0001 L CNN "digikey part number"
F 13 "0.87mm" H 7350 6450 50  0001 L CNN "height"
F 14 "Yes" H 7350 6550 50  0001 L CNN "lead free"
F 15 "4271a13c665e43de" H 7350 6650 50  0001 L CNN "library id"
F 16 "KEMET" H 7350 6750 50  0001 L CNN "manufacturer"
F 17 "Ceramic" H 7350 6850 50  0001 L CNN "material"
F 18 "Multilayer Ceramic Capacitors MLCC - SMD/SMT 50volts 2.0pF C0G 0.25%" H 7350 6950 50  0001 L CNN "mouser description"
F 19 "80-C0603C209C5G" H 7350 7050 50  0001 L CNN "mouser part number"
F 20 "0603" H 7478 4905 50  0000 L CNN "package"
F 21 "Yes" H 7350 7250 50  0001 L CNN "rohs"
F 22 "C0G" H 7350 7350 50  0001 L CNN "temperature characteristic"
F 23 "30ppm/°C" H 7350 7450 50  0001 L CNN "temperature coefficient"
F 24 "+125°C" H 7350 7550 50  0001 L CNN "temperature range high"
F 25 "-55°C" H 7350 7650 50  0001 L CNN "temperature range low"
F 26 "0.25 pF" H 7350 7750 50  0001 L CNN "tolerance"
F 27 "50 V" H 7478 4814 50  0000 L CNN "voltage rating"
	1    7350 5050
	1    0    0    -1  
$EndComp
$Comp
L project:CC0603KRX7R9BB103 C?
U 1 1 60B9BEC3
P 5200 5050
AR Path="/5FBD86FB/60B9BEC3" Ref="C?"  Part="1" 
AR Path="/5FC1DCBA/60B9BEC3" Ref="C?"  Part="1" 
AR Path="/5FC1E11E/60B9BEC3" Ref="C?"  Part="1" 
AR Path="/5FC1E1DD/60B9BEC3" Ref="C?"  Part="1" 
AR Path="/5FC1E32F/60B9BEC3" Ref="C?"  Part="1" 
AR Path="/5FC1FA4C/60B9BEC3" Ref="C?"  Part="1" 
AR Path="/5FC1FA54/60B9BEC3" Ref="C?"  Part="1" 
AR Path="/5FC1FA5C/60B9BEC3" Ref="C?"  Part="1" 
AR Path="/5FC1FA64/60B9BEC3" Ref="C?"  Part="1" 
AR Path="/5FC1FA6C/60B9BEC3" Ref="C?"  Part="1" 
F 0 "C?" H 5328 5087 50  0000 L CNN
F 1 "CC0603KRX7R9BB103" H 5200 5250 50  0001 L CNN
F 2 "YAGEO-CC0603-DA-05_2018-24-*" H 5200 5350 50  0001 L CNN
F 3 "http://www.yageo.com/documents/recent/UPY-GPHC_X7R_6.3V-to-50V_18.pdf" H 5200 5450 50  0001 L CNN
F 4 "No" H 5200 5550 50  0001 L CNN "automotive"
F 5 "10 nF" H 5328 4996 50  0000 L CNN "capacitance"
F 6 "Cap" H 5200 5750 50  0001 L CNN "category"
F 7 "Passive Components" H 5200 5850 50  0001 L CNN "device class L1"
F 8 "Capacitors" H 5200 5950 50  0001 L CNN "device class L2"
F 9 "Ceramic Capacitors" H 5200 6050 50  0001 L CNN "device class L3"
F 10 "CAP CER 10000PF 50V X7R 0603" H 5200 6150 50  0001 L CNN "digikey description"
F 11 "311-1085-2-ND" H 5200 6250 50  0001 L CNN "digikey part number"
F 12 "http://www.yageo.com/exep/pages/download/literatures/UPY-C_GEN_24.pdf" H 5200 6350 50  0001 L CNN "footprint url"
F 13 "0.9mm" H 5200 6450 50  0001 L CNN "height"
F 14 "Yes" H 5200 6550 50  0001 L CNN "lead free"
F 15 "2a4c50056f7872c7" H 5200 6650 50  0001 L CNN "library id"
F 16 "YAGEO" H 5200 6750 50  0001 L CNN "manufacturer"
F 17 "Ceramic" H 5200 6850 50  0001 L CNN "material"
F 18 "Multilayer Ceramic Capacitor, CC Series, 0.01 - F, - 10%, X7R, 50 V, 0603 [1608 Metric]" H 5200 6950 50  0001 L CNN "mouser description"
F 19 "603-CC603KRX7R9BB103" H 5200 7050 50  0001 L CNN "mouser part number"
F 20 "0603" H 5328 4905 50  0000 L CNN "package"
F 21 "Yes" H 5200 7250 50  0001 L CNN "rohs"
F 22 "X7R" H 5200 7350 50  0001 L CNN "temperature characteristic"
F 23 "15%" H 5200 7450 50  0001 L CNN "temperature coefficient"
F 24 "+125°C" H 5200 7550 50  0001 L CNN "temperature range high"
F 25 "-55°C" H 5200 7650 50  0001 L CNN "temperature range low"
F 26 "0.1" H 5200 7750 50  0001 L CNN "tolerance"
F 27 "50 V" H 5328 4814 50  0000 L CNN "voltage"
F 28 "50 V" H 5200 7950 50  0001 L CNN "voltage rating"
	1    5200 5050
	1    0    0    -1  
$EndComp
Wire Wire Line
	5200 5450 5650 5450
Connection ~ 5650 5450
Wire Wire Line
	7350 5050 7350 4900
Connection ~ 7350 4900
Wire Wire Line
	7350 4900 7600 4900
Wire Wire Line
	7350 5250 7350 5450
Wire Wire Line
	7350 5450 6200 5450
Connection ~ 6200 5450
Connection ~ 5650 4900
Wire Wire Line
	5650 4900 5200 4900
Wire Wire Line
	5200 5450 5200 5250
Wire Wire Line
	5200 5050 5200 4900
NoConn ~ 7300 4700
Text Label 6750 5450 0    50   ~ 0
GND
Wire Wire Line
	3350 4900 5200 4900
Connection ~ 5200 4900
Wire Wire Line
	3350 5000 3750 5000
Wire Wire Line
	3750 5000 3750 4950
Wire Wire Line
	3750 4950 5000 4950
Wire Wire Line
	5000 4950 5000 6100
Wire Wire Line
	5000 6100 5200 6100
Connection ~ 5200 6100
Text Label 8400 2600 0    50   ~ 0
HSEN2
Text Label 8400 2700 0    50   ~ 0
GND
Wire Wire Line
	8350 2600 8850 2600
Wire Wire Line
	8350 2700 8850 2700
Text Notes 8750 2900 0    50   ~ 0
Offset field sensor
$Comp
L project:SSW-102-01-T-S J?
U 1 1 60D4A0E1
P 8750 2600
F 0 "J?" V 8754 3028 50  0000 L CNN
F 1 "SSW-102-01-T-S" V 8845 3028 50  0000 L CNN
F 2 "project:Samtec-SSW-102-01-T-S-Manufacturer_Recommended" H 8750 3300 50  0001 L CNN
F 3 "http://www.samtec.com/documents/webfiles/pdf/SLW.PDF" H 8750 3400 50  0001 L CNN
F 4 "Manufacturer URL" H 8750 3500 50  0001 L CNN "Component Link 1 Description"
F 5 "http://www.samtec.com" H 8750 3600 50  0001 L CNN "Component Link 1 URL"
F 6 "Package Specification" H 8750 3700 50  0001 L CNN "Component Link 3 Description"
F 7 "http://www.samtec.com/documents/webfiles/cpdf/SLW-1XX-01-X-X-MKT.pdf" H 8750 3800 50  0001 L CNN "Component Link 3 URL"
F 8 "F-214" H 8750 3900 50  0001 L CNN "Datasheet Version"
F 9 "Through-hole" H 8750 4000 50  0001 L CNN "Mounting Technology"
F 10 "Vertical" H 8750 4100 50  0001 L CNN "Orientation"
F 11 "Low Profile Socket Strip" H 8750 4200 50  0001 L CNN "Package Description"
F 12 "X, 9/1987" H 8750 4300 50  0001 L CNN "Package Version"
F 13 "2.54 mm" H 8750 4400 50  0001 L CNN "Pitch"
F 14 "-55 to 105 degC" H 8750 4500 50  0001 L CNN "Temperature Range"
F 15 "Conn" H 8750 4600 50  0001 L CNN "category"
F 16 "349239" H 8750 4700 50  0001 L CNN "ciiva ids"
F 17 "97f31bfa5e778d3a" H 8750 4800 50  0001 L CNN "library id"
F 18 "Samtec" H 8750 4900 50  0001 L CNN "manufacturer"
F 19 "SSW-102-01-X-S" H 8750 5000 50  0001 L CNN "package"
F 20 "1404374322" H 8750 5100 50  0001 L CNN "release date"
F 21 "FDE8A73C-755C-42A3-8DB0-EB605DA0F6D5" H 8750 5200 50  0001 L CNN "vault revision"
F 22 "yes" H 8750 5300 50  0001 L CNN "imported"
	1    8750 2600
	0    1    1    0   
$EndComp
$Comp
L project:SSW-102-01-T-S J?
U 1 1 60D4C115
P 10350 2700
F 0 "J?" V 10715 2942 50  0000 C CNN
F 1 "SSW-102-01-T-S" V 10624 2942 50  0000 C CNN
F 2 "project:Samtec-SSW-102-01-T-S-Manufacturer_Recommended" H 10350 3400 50  0001 L CNN
F 3 "http://www.samtec.com/documents/webfiles/pdf/SLW.PDF" H 10350 3500 50  0001 L CNN
F 4 "Manufacturer URL" H 10350 3600 50  0001 L CNN "Component Link 1 Description"
F 5 "http://www.samtec.com" H 10350 3700 50  0001 L CNN "Component Link 1 URL"
F 6 "Package Specification" H 10350 3800 50  0001 L CNN "Component Link 3 Description"
F 7 "http://www.samtec.com/documents/webfiles/cpdf/SLW-1XX-01-X-X-MKT.pdf" H 10350 3900 50  0001 L CNN "Component Link 3 URL"
F 8 "F-214" H 10350 4000 50  0001 L CNN "Datasheet Version"
F 9 "Through-hole" H 10350 4100 50  0001 L CNN "Mounting Technology"
F 10 "Vertical" H 10350 4200 50  0001 L CNN "Orientation"
F 11 "Low Profile Socket Strip" H 10350 4300 50  0001 L CNN "Package Description"
F 12 "X, 9/1987" H 10350 4400 50  0001 L CNN "Package Version"
F 13 "2.54 mm" H 10350 4500 50  0001 L CNN "Pitch"
F 14 "-55 to 105 degC" H 10350 4600 50  0001 L CNN "Temperature Range"
F 15 "Conn" H 10350 4700 50  0001 L CNN "category"
F 16 "349239" H 10350 4800 50  0001 L CNN "ciiva ids"
F 17 "97f31bfa5e778d3a" H 10350 4900 50  0001 L CNN "library id"
F 18 "Samtec" H 10350 5000 50  0001 L CNN "manufacturer"
F 19 "SSW-102-01-X-S" H 10350 5100 50  0001 L CNN "package"
F 20 "1404374322" H 10350 5200 50  0001 L CNN "release date"
F 21 "FDE8A73C-755C-42A3-8DB0-EB605DA0F6D5" H 10350 5300 50  0001 L CNN "vault revision"
F 22 "yes" H 10350 5400 50  0001 L CNN "imported"
	1    10350 2700
	0    -1   -1   0   
$EndComp
NoConn ~ 10250 2600
NoConn ~ 10250 2700
$EndSCHEMATC