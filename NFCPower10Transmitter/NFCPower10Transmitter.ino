/*
 * NFCPower10Transmitter
 *  - Microcontroller ATMEGA328
 */
#define IDN "NFCPower10Transmitter"
#define VERSION "1.0.0"

#include <Wire.h>

uint8_t address = 0;
uint8_t reg = 0;
uint8_t registers[5] = {0,0,0,0,0};
enum{TX_REG_LEDS,TX_REG_VDET1_MSB,TX_REG_VDET1_LSB,TX_REG_VDET2_MSB,TX_REG_VDET2_LSB};

bool measure_field = false;
#define ANALOG_PIN_VDET1 A0
#define ANALOG_PIN_VDET2 A1
uint16_t val_field_detector1 = 0;
uint16_t val_field_detector2 = 0;

#define PIN_IRQ 2
#define PIN_LED_RED 3
#define PIN_LED_GRN 4
#define PIN_LED_BLU 5

bool update_LEDs = true; 

String inputString = "";            // incoming serial data buffer
bool stringComplete = false;        // string complete flag

void setup() {
  Serial.begin(115200); // create serial
  setup_timer2(); // setup timer2 1ms interrupt
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_GRN, OUTPUT);
  pinMode(PIN_LED_BLU, OUTPUT);
  digitalWrite(PIN_LED_RED, HIGH);
  digitalWrite(PIN_LED_GRN, HIGH);
  digitalWrite(PIN_LED_BLU, HIGH); 
}

void loop() {
  if(stringComplete){
    handleMessage(inputString);
    stringComplete = false;
    inputString = "";
  }
  if(measure_field){
    measureFieldDetectors();
    measure_field = false;
    updateState();
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

void handleMessage(String input_msg){
  if(input_msg.startsWith("ADDRESS HOP")){
    String val_str = input_msg.substring(11); // parse value
    address = val_str.toInt(); // set address
    Wire.begin(address); // join i2c bus with address
    Wire.onReceive(receiveEvent); // register event
    Wire.onRequest(requestEvent); // register event
    Serial.print("ADDRESS HOP "); // print the address hop message
    Serial.println(address+1); // increment next address
  }
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany){
  reg = Wire.read();    // receive byte as an integer
  if(Wire.available()){
    if(reg==TX_REG_LEDS){
      registers[reg] = Wire.read();
      update_LEDs = true;      
    }
  }  
}

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void requestEvent() {
  Wire.write(registers[reg]);  
  if(reg>TX_REG_LEDS){
    Wire.write(registers[reg+1]);  
  }  
}

// update state of LEDs
void updateState(){
  if(registers[TX_REG_LEDS]&0x01>0){
    digitalWrite(PIN_LED_RED,HIGH);
  }
  else{
    digitalWrite(PIN_LED_RED,LOW);
  }
  if(registers[TX_REG_LEDS]&0x02>0){
    digitalWrite(PIN_LED_GRN,HIGH);
  }
  else{
    digitalWrite(PIN_LED_GRN,LOW);
  }
  if(registers[TX_REG_LEDS]&0x41>0){
    digitalWrite(PIN_LED_BLU,HIGH);
  }
  else{
    digitalWrite(PIN_LED_BLU,LOW);
  }
}

// print identifier and firmware version
void printIDNVERSION(){
  Serial.print(IDN);
  Serial.print(" v");
  Serial.println(VERSION);
}

void setup_timer2(){ // setup timer2, compare A, 1ms interrupt
  TCCR2A=(1<<WGM21);    //Set the CTC mode   
  TCCR2B|=(1<<CS22);    //Set the prescale 1/64 clock, 16MHz/64 = 250kHz
  OCR2A=0xFA;           //Value for 1ms, 250kHz/250 = 1kHz
  TIMSK2|=(1<<OCIE2A);  //Set the interrupt request
  sei();                //Enable interrupt
}

ISR(TIMER2_COMPA_vect){ // Handle Timer2 COMPA interrupt
  measure_field = true; // set the flag, handle in loop
}

void measureFieldDetectors(){
  val_field_detector1 = analogRead(ANALOG_PIN_VDET1);  // read the input pin
  val_field_detector2 = analogRead(ANALOG_PIN_VDET2);  // read the input pin
}
