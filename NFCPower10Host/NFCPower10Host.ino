/*
 * NFCPower10Host
 *  - Microcontroller ATMEGA328
 *  2021/02/15
 *    - Create a trigger signal using the transmitter field sensors
 *	  - print Min value or Max value when we detect the min and max field
 *    - also print the time in ms since last min/or max
 *    - mask the trigger while duration is less than 20ms
 *  2021/02/16
 *    - Field signal period is 750ms but lamp is in view for only about 50ms
 *      so time scales are different. Sampling field at 10ms and then delaying
 *      min field detection 5 samples would exceed the duration the lamp is in view.
 *      So instead we will change to a photo gate to trigger the lamp rows.
 *  2021/02/17
 *    - Use IRQ driven by a photo gate to trigger on each lamp row.
 *    - Print "Trig" when IRQ interrupt detected
 *    - Then read the 10 field values and print "Field " followed by 10 space separated values
 */
#define IDN "NFCPower10Host"
#define VERSION "1.0.0"

#include <Wire.h>
#include <Adafruit_INA219.h>
#include <Adafruit_SI5351.h>

#define ADDRESS_TX1 0x30
uint16_t val_field_detectors[20];
enum{TX_REG_LEDS,TX_REG_VDET1_MSB,TX_REG_VDET1_LSB,TX_REG_VDET2_MSB,TX_REG_VDET2_LSB};

Adafruit_INA219 ina219;
#define ADDRESS_INA219 0x40
float shuntvoltage = 0;
float busvoltage = 0;
float current_mA = 0;
float loadvoltage = 0;
float power_mW = 0;

#define NFC_ADDRESS 0x50
#define NFC_DC_SET_DEFAULT        0xC0
#define NFC_DC_INITIAL_FIELD_ON   0xC8
#define NFC_A_TX_DRIVER           0x28
#define NFC_A_REGULATOR_CONTROL   0x2C
#define NFC_B_REGULATOR_DISPLAY   0x2C

Adafruit_SI5351 clockgen = Adafruit_SI5351();
#define ADDRESS_SI5351 0x60
#define PLL_M 15
#define PLL_N 0
#define PLL_D 1
#define PLL "375 MHz"
#define FOUT_DIV 12
#define FOUT_N 1000
#define FOUT_D1 570
#define FOUT_D2 560
#define FOUT_D3 550
#define FOUT_D4 540
#define FOUT_D5 530
#define F1 "27.264 MHz"
#define F2 "27.202 MHz"
#define F3 "27.138 MHz"
#define F4 "27.072 MHz"
#define F5 "27.004 MHz"

#define PIN_LED_BLU 13 // PB5, SCK
#define PIN_LED_RED 7 // PD7
#define PIN_OEB 17 // PC3
#define PIN_SSEN 16 // PC2
#define PIN_SDA 18 // PC4

#define PIN_IRQ 2 // PD2, INT0

String inputString = "";            // incoming serial data buffer
bool stringComplete = false;        // string complete flag
bool read_field_detectors = true;   // read field detectors flag
uint16_t read_field_detectors_counter = 0;

uint16_t demo_pattern_counter = 0;      // demo pattern counter
uint8_t demo_pattern = 0;

bool blue_toggle = false;

int16_t field_signal = 0;
int16_t field_signal_lpf2 = 0;
int16_t field_signal_lpf4 = 0;
int16_t field_signal_lpf32 = 0;
int16_t field_diff = 0;
int16_t field_diff_lpf2 = 0;
int16_t field_diff_lpf4 = 0;
int16_t field_diff2 = 0;
int16_t field_diff2_lpf2 = 0;
int16_t field_diff2_lpf4 = 0;
bool field_diff2_positive = false;
bool field_diff2_negative = false;
bool last_field_diff2_positive = false;
bool last_field_diff2_negative = false;
int16_t last_field_diff_lpf4 = 0;
int16_t field_duration = 0;

String msg_field_values = "";
bool irq_flag = false;
long irq_start_time = 0;
bool step_demo_pattern = false;

void setup() {
  pinMode(PIN_LED_RED, OUTPUT); 
  digitalWrite(PIN_LED_RED, HIGH);   
  Serial.begin(115200); // create serial
  printIDNVERSION(); // print identifier and firmware version
  delay(500); // wait for slaves to startup
  Serial.println("Assign Transmitter Addresses");
  assignTXAddresses(); // assign I2C TX addresses
  Serial.println("Start I2C");  
  Wire.begin(); // join i2c bus (address optional for master)
  Serial.println("Start Current Sensor");
  ina219.begin(); // start current sensor
  Serial.println("Start Clock Generator");
  setupClockGen(); // setup clock generator frequencies
  if(nfc_connected()){
    Serial.println("Turn on NFC Field");
    nfc_field_on();
  }
  else{
    Serial.println("NFC Readers not Connected");
  }
  //Wire.setClock(100000); // standard
  //Wire.setClock(400000); // fast
  //Wire.setClock(1000000); // fast plus
  Wire.setClock(3400000); // high speed
  Serial.println("Set Transmitter LEDs Test Pattern");
  setTransmitterLEDsTestPattern(demo_pattern);
  setup_timer2(); // setup timer2 1ms interrupt  
  Serial.println("Ready");  
  pinMode(PIN_LED_BLU, OUTPUT);
  digitalWrite(PIN_LED_BLU, LOW);
  pinMode(PIN_IRQ, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_IRQ), isrIRQ, FALLING);
}

void loop() {
  if(stringComplete){
    handleMessage(inputString);
    stringComplete = false;
    inputString = "";
  }
  if(read_field_detectors){
	  //calculateTrigger(); // calculate the trigger
    if(++read_field_detectors_counter>10){ // 10ms read
      //msg_field_values = readTransmitterFieldDetectors();
      //calculateTrigger(); // calculate the trigger      
      read_field_detectors_counter = 0;
    }
    //measureCurrentVoltage();
    //printCurrentVoltage();
    read_field_detectors = false;
  }
  if(step_demo_pattern){    
    blue_toggle = !blue_toggle;
    digitalWrite(PIN_LED_BLU, (blue_toggle)?(HIGH):(LOW));
    if(++demo_pattern>2){
      demo_pattern = 0;
    }
    setTransmitterLEDsTestPattern(demo_pattern);
    step_demo_pattern = false; // clear flag  
  }
  if(irq_flag){
    long irq_stop_time = millis();
    Serial.println("Trig"); // print trigger message
    long start_time = millis();
    msg_field_values = readTransmitterFieldDetectors();
    long read_time = millis();
    Serial.print("Fields ");
    Serial.println(msg_field_values); // print field values
    long stop_time = millis();
    #define DO_PRINT_TIMES false
    if(DO_PRINT_TIMES){
      Serial.print("IRQ Time ");
      Serial.print((irq_stop_time-irq_start_time));
      Serial.print(" Read Time ");
      Serial.print((read_time-start_time));
      Serial.print(" Send time ");
      Serial.println((stop_time-read_time));
    }
    irq_flag = 0; // clear flag
  }
}

void isrIRQ(){
  irq_flag = true; // set flag, handle in loop
  irq_start_time = millis();
}

void setup_timer2(){ // setup timer2, compare A, 1ms interrupt
  TCCR2A=(1<<WGM21);    //Set the CTC mode   
  TCCR2B|=(1<<CS22);    //Set the prescale 1/64 clock, 16MHz/64 = 250kHz
  OCR2A=0xFA;           //Value for 1ms, 250kHz/250 = 1kHz
  TIMSK2|=(1<<OCIE2A);  //Set the interrupt request
  sei();                //Enable interrupt
}

ISR(TIMER2_COMPA_vect){ // Handle Timer2 COMPA interrupt
  //read_field_detectors = true; // set the flag, handle in loop
  if(++demo_pattern_counter>1000){
    step_demo_pattern = true; // set the flag, handle in loop
    demo_pattern_counter = 0; // reset counter
  }
}

void serialEvent(){
  while(Serial.available()){
    char inChar = (char)Serial.read(); // read a character
    if(inChar == '\r'){ // detect line terminator carriage return
      stringComplete = true; // set flag
    }
    else if(inChar != '\n'){ // ignore new line characters
      inputString += inChar; // add character to string
    }
  }
}

void calculateTrigger(void){
	field_duration++; // increment duration
	// 1. Read the field strength signal
	field_signal = (int16_t)readTransmitterFieldDetector(5)*4; // read first transmitter field strength
	field_signal_lpf32 = (field_signal*8 - field_signal_lpf32)/32 + field_signal_lpf32;
	// 2. Calculate signal low pass filter 2
	field_signal_lpf2 = (field_signal - field_signal_lpf2)/2 + field_signal_lpf2;
	// 3. Calculate signal low pass filter 4
	field_signal_lpf4 = (field_signal - field_signal_lpf4)/4 + field_signal_lpf4;
	// 4. Calculate difference of low pass 2 - 4
	field_diff = (field_signal_lpf2 - field_signal_lpf4)*4;
	// 5. Calculate difference low pass filter 2
	field_diff_lpf2 = (field_diff - field_diff_lpf2)/2 + field_diff_lpf2;
	// 6. Calculate difference low pass filter 4
	field_diff_lpf4 = (field_diff - field_diff_lpf4)/4 + field_diff_lpf4;
	// 7. Calculate 2nd difference of low pass 2 - 4
	field_diff2 = (field_diff_lpf2 - field_diff_lpf4)*4;
	// 8. Calculate 2nd difference low pass filter 2
	field_diff2_lpf2 = (field_diff2 - field_diff2_lpf2)/2 + field_diff2_lpf2;
	// 9. Calculate 2nd difference low pass filter 4
	field_diff2_lpf4 = (field_diff2 - field_diff2_lpf4)/4 + field_diff2_lpf4;
	// 10. Calculate 2nd difference positivity
	field_diff2_positive = field_diff2_lpf4>0;
	// 11. Calculate 2nd difference negativity
	field_diff2_negative = field_diff2_lpf4<0;  
	// 12. Calculate is minimum
	bool is_positive = field_diff2_positive || last_field_diff2_positive;
	bool is_minimum = (field_diff_lpf4 > 0) && (last_field_diff_lpf4 < -2) && is_positive;  
	// 13. Calculate is maximum
	bool is_negative = field_diff2_negative || last_field_diff2_negative;
	bool is_maximum = (field_diff_lpf4 < 0) && (last_field_diff_lpf4 > 2) && is_negative;  

  // print min max triggers
  #define DO_PRINT_MIN_MAX false
	if(DO_PRINT_MIN_MAX && is_minimum && field_duration > 20){ // mask trigger if duration is not greater than 20ms
		Serial.print("Min "); // print minimum trigger
		Serial.print(field_signal_lpf4); // print the signal low pass 4 value
		Serial.print(" "); // print a space
		Serial.println(field_duration); // print duration (ms) since last trigger
		field_duration = 0; // clear duration
	}
	else if(DO_PRINT_MIN_MAX && is_maximum && field_duration > 20){ // mask trigger if duration is not greater than 20ms
		Serial.print("Max "); // print maximum trigger
		Serial.print(field_signal_lpf4); // print the signal low pass 4 value
		Serial.print(" "); // print a space
		Serial.println(field_duration); // print duration (ms) since last trigger
		field_duration = 0; // clear duration
	}
	
	last_field_diff2_positive = field_diff2_positive; // set last
	last_field_diff2_negative = field_diff2_negative; // set last
	last_field_diff_lpf4 = field_diff_lpf4; // set last

  // debug signals with serial plotter
  #define DO_PRINT_SERIAL_PLOTTER true
  if(DO_PRINT_SERIAL_PLOTTER){
    //Serial.print(field_signal);Serial.print(" ");
    //Serial.print(field_signal_lpf2);Serial.print(" ");
    //Serial.print(field_signal_lpf4/2);Serial.print(" ");
    //Serial.print(field_signal_lpf32/16);Serial.print(" ");
    //Serial.print(field_diff);Serial.print(" ");
    //Serial.print(field_diff_lpf2);Serial.print(" ");
    //Serial.print(field_diff_lpf4);Serial.print(" ");
    Serial.print(field_diff_lpf4);Serial.print(" ");
    Serial.print(field_diff2_lpf4);Serial.print(" ");
    //Serial.print(is_positive?("1000"):("0"));Serial.print(" ");
    Serial.print(is_minimum?("1000"):("0"));Serial.print(" ");
    Serial.println(""); 
  }
}

void handleMessage(String input_msg){
  input_msg.toUpperCase();
  if(input_msg=="READ CURRENT VOLTAGE"){
    measureCurrentVoltage();
    printCurrentVoltage();
  }
}

// print identifier and firmware version
void printIDNVERSION(){
  Serial.print(IDN);
  Serial.print(" v");
  Serial.println(VERSION);
}

// Purpose is to assign a sequential I2C addresses to the 10 transmitters.
// The 10 transmitter uart tx and rx are cascaded.
// When a transmitter receives the hop message it assigns its own address to the hop number. 
// Then it increments the hop number and sends to the next transmitter.
bool assignTXAddresses(){
  stringComplete = false; // clear flag
  inputString = ""; // clear string
  Serial.print("ADDRESS HOP "); // print the transmitter address hop message
  Serial.println(ADDRESS_TX1); // print first address
  uint16_t timeout_counter = 0;
  while(!stringComplete){ // wait for response message
    serialEvent(); // call serialEvent to check for received message
    if(++timeout_counter>100){
      Serial.println("Timeout Assigning Addresses");
      return false;
    }
    delay(1);
  }
  bool ret_val = true;
  if(inputString != "ADDRESS HOP "+String(ADDRESS_TX1+10)){
    Serial.print("Error Assigning Addresses: "); // print error message
    Serial.println(inputString); // echo input string
    ret_val = false;
  }
  else{    
    Serial.print("Success Assigning Addresses: ");
    Serial.println(inputString); // echo input string
  }
  stringComplete = false; // clear flag
  inputString = ""; // clear string
  return ret_val;
}

uint16_t readTransmitterFieldDetector(uint8_t ntx){	
	uint16_t val = 0; // initialize the return value to zero
	if(ntx>0 && ntx <= 10){ // verify ntx is between 1 and 10
		int addr = ADDRESS_TX1 + ntx-1; // set the transmitter address
		Wire.beginTransmission(addr); // transmit to device at addr
		Wire.write(TX_REG_VDET1_MSB); // sends one byte, set the register to VDET1_MSB
		Wire.endTransmission(); // stop transmitting
		Wire.requestFrom(addr, 2);    // request 2 bytes, VDET1_MSB and VDET1_LSB
		val = Wire.read()<<8; // read VDET1_MSB
		val += Wire.read(); // read VDET1_LSB		
	}
	return val; // return the value
}

String readTransmitterFieldDetectors(){  
  String msg_HSEN1 = "";//"HSEN1: ";
  String msg_HSEN2 = "";//"HSEN2: ";
  for(int addr = ADDRESS_TX1; addr<(ADDRESS_TX1+10); addr++){
    Wire.beginTransmission(addr); // transmit to device at addr
    Wire.write(TX_REG_VDET1_MSB); // sends one byte
    Wire.endTransmission(); // stop transmitting
    //Wire.requestFrom(addr, 4);    // request 4 bytes
    Wire.requestFrom(addr, 2);    // request 2 bytes
    uint16_t val = Wire.read()<<8; // read MSB
    val += Wire.read(); // read LSB
    val_field_detectors[(addr-ADDRESS_TX1)*2] = val;
    if(true){//if(addr<(ADDRESS_TX1+5)){
      msg_HSEN1 += String(val) + " ";
    }
    continue;
    val = Wire.read()<<8; // read MSB
    val += Wire.read(); // read LSB
    val_field_detectors[(addr-ADDRESS_TX1)*2+1] = val;
    if(addr>(ADDRESS_TX1+5)){
      msg_HSEN2 += String(val) + " ";
    }
  }
  //Serial.println(msg_HSEN1);
  //Serial.println(msg_HSEN2);
  return msg_HSEN1;
}

void setTransmitterLEDsTestPattern(uint8_t offset){
  uint8_t leds = 1 << offset;
  for(int addr = ADDRESS_TX1; addr<(ADDRESS_TX1+11); addr++){    
    setTransmitterLEDs(addr,leds);
    leds <<=1;
    if(leds>4){leds = 1;}
  }
}

void setTransmitterLEDs(int addr, uint8_t leds){
  Wire.beginTransmission(addr); // transmit to device at addr
  Wire.write(TX_REG_LEDS); // sends one byte
  Wire.write(leds); // sends one byte
  Wire.endTransmission(); // stop transmitting
}

// measure current and voltage
void measureCurrentVoltage(){
  shuntvoltage = ina219.getShuntVoltage_mV();
  busvoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  power_mW = ina219.getPower_mW();
  loadvoltage = busvoltage + (shuntvoltage / 1000);
}

// print current and voltage
void printCurrentVoltage(){
  Serial.print("Bus Voltage:   "); Serial.print(busvoltage); Serial.println(" V");
  Serial.print("Shunt Voltage: "); Serial.print(shuntvoltage); Serial.println(" mV");
  Serial.print("Load Voltage:  "); Serial.print(loadvoltage); Serial.println(" V");
  Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
  Serial.print("Power:         "); Serial.print(power_mW); Serial.println(" mW");
  Serial.println("");
}

// setup clock generator frequencies
bool setupClockGen(){
  //Serial.println("Connecting to SI5351...");
  //pinMode(PIN_SSEN, OUTPUT);
  //digitalWrite(PIN_SSEN, LOW); // disable spread spectrum
  //pinMode(PIN_OEB, OUTPUT);
  //digitalWrite(PIN_OEB, HIGH); // enable outputs, active low
  if (clockgen.begin() != ERROR_NONE){ // Initialise the sensor
    Serial.println("SI5351 Not Detected.");
    //while(1);
    return false;
  }
  Serial.println("SI5351 Connected.");
  //Serial.print("Set PLLA to "); Serial.println(PLL);
  clockgen.setupPLLInt(SI5351_PLL_A, PLL_M);
  Serial.print("Set Output 1 to "); Serial.println(F1);  
  clockgen.setupMultisynth(1, SI5351_PLL_A, FOUT_DIV, FOUT_N, FOUT_D1);
  Serial.print("Set Output 2 to "); Serial.println(F2);  
  clockgen.setupMultisynth(2, SI5351_PLL_A, FOUT_DIV, FOUT_N, FOUT_D2);
  Serial.print("Set Output 3 to "); Serial.println(F3);  
  clockgen.setupMultisynth(3, SI5351_PLL_A, FOUT_DIV, FOUT_N, FOUT_D3);
  Serial.print("Set Output 4 to "); Serial.println(F4);  
  clockgen.setupMultisynth(4, SI5351_PLL_A, FOUT_DIV, FOUT_N, FOUT_D4);
  Serial.print("Set Output 5 to "); Serial.println(F5);  
  clockgen.setupMultisynth(5, SI5351_PLL_A, FOUT_DIV, FOUT_N, FOUT_D5);
  clockgen.enableOutputs(true); // Enable the clocks
  return true;
}

void nfc_field_on(){
  char ready_mode[] = {0x02,0x81};
  nfc_write(ready_mode,2);
  nfc_write_byte(NFC_DC_INITIAL_FIELD_ON);
}

void nfc_field_off(){
  nfc_write_byte(NFC_DC_SET_DEFAULT);
}

void nfc_write(char *d, char n){
  Wire.beginTransmission(NFC_ADDRESS); // transmit to device
  for(char i=0; i<n; i++){
    byte b = *d;
    d++;
    Wire.write(b);
  }
  Wire.endTransmission();    // stop transmitting
}

void nfc_write_byte(byte b){
  Wire.beginTransmission(NFC_ADDRESS); // transmit to device
  Wire.write((uint8_t)b);
  Wire.endTransmission();    // stop transmitting
}

bool nfc_connected(){
  Wire.beginTransmission(NFC_ADDRESS);
  byte error = Wire.endTransmission();
  return !error;
}
