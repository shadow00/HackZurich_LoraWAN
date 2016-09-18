#include "Sodaq_RN2483.h"
#include "Arduino.h"
#include <math.h>
#include <OneWire.h> 

//int DS18S20_Pin = 13; //DS18S20 Signal pin on digital 13
#define DS18S20_Pin 13
//Temperature chip i/o
OneWire ds(DS18S20_Pin); // on digital pin 13

#define debugSerial SerialUSB
#define loraSerial Serial1

/* The number of the device: 1,2,3,4 */
#define deviceNo 1

#define beePin ENABLE_PIN_IO

#define LOUDNESS_SENSOR 0
#define LIGHT_SENSOR 2
#define WATER_SENSOR 6
#define BUZZER 8
#define MAGNETIC_SWITCH 12
//#define VIBRATION_SENSOR 4

int loudness;

void BLUE() {
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_BLUE, LOW);
}

void RED() {
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_BLUE, HIGH);
}

void YELLOW() {
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_BLUE, HIGH);
}

void WHITE() {
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_BLUE, LOW);
}


void GREEN() {
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_BLUE, HIGH);
}

void CLEAR() {
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_BLUE, HIGH);
}

void blink(int length) {
    switch(deviceNo) {
    case 1:
        BLUE();
        break;
    case 2:
        RED();
        break;
    case 3:
        GREEN();
        break;
    case 4:
        WHITE();
        break;
    }
    delay(length);
    CLEAR();
}

void beep(int howlong) {
     buzzOn();
     delay(howlong);
     buzzOff();
}

void buzzOn() {
    digitalWrite(BUZZER, HIGH);
}

void buzzOff() {
    digitalWrite(BUZZER, LOW);
}


void setupLED() {
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
}


// OTAA
// Random numbers chosen + device id
uint8_t DevEUI[8] = { 0x9c, 0xd9, 0x0b, 0xb5, 0x2b, 0x6a, 0x1d, deviceNo };

uint8_t AppEUI[8] = { 0xd4, 0x16, 0xcd, 0x0b, 0x7b, 0xcf, 0x2d, 0x5c };

uint8_t AppKey[16] = { 0xa9, 0xbc, 0x8b, 0x6a, 0x81, 0x75, 0xf6, 0x33,
0xe0, 0xd6, 0x64, 0xd9, 0x2b, 0xcb, 0x13, 0x78 };

uint8_t counter;

void setupLoRaOTAA(){
  if (LoRaBee.initOTA(loraSerial, DevEUI, AppEUI, AppKey, true))
  {
    debugSerial.println("Communication to LoRaBEE successful.");
  }
  else
  {
    debugSerial.println("OTAA Setup failed!");
  }
}

/*boolean readVibration()
{
  int sensorValue=analoglRead(VIBRATION_SENSOR);
  if (sensorValue>1000) {
    return true;
  } else {
    return false;
  }
}*/

int readLoudness()
{
	return analogRead(LOUDNESS_SENSOR);
}

int readLight()
{
    int sensorValue = analogRead(LIGHT_SENSOR);
    return map(sensorValue, 11, 27333, 0, 413);
}

/*void setupVibration() {
     pinMode(VIBRATION_SENSOR, INPUT);
}*/

void setupWater() {
     pinMode(WATER_SENSOR, INPUT);
}

boolean hasWater()
{
    if(digitalRead(WATER_SENSOR) == LOW) {
        return true;
    } else {
        return false;
    }
}

/*void setupTemp() {
     pinMode(TEMPERATURE, INPUT);
}*/

float readTemp()
{
    /*int B=44000;                 // B value of the thermistor
    int R0 = 88000;          
    int a=analogRead(TEMPERATURE);
    float R = 99000.0*(1023.0/((float)a)-1.0);
 
    float temperature = (1.0/(((log(R/R0))/B+(1/298.15))))-273.15;//convert to temperature via datasheet ;
 
    //Serial.print("temperature = ");
    //Serial.println(temperature);*/
    byte data[12];
    byte addr[8];
  
   if ( !ds.search(addr)) {
     //no more sensors on chain, reset search
     ds.reset_search();
     return -1;
   }
  
   if ( OneWire::crc8( addr, 7) != addr[7]) {
     return -1000; //"CRC is not valid!"
   }
  
   if ( addr[0] != 0x10 && addr[0] != 0x28) {
     return -1000; //Device is not recognized
   }
  
   ds.reset();
   ds.select(addr);
   ds.write(0x44,1); // start conversion, with parasite power on at the end
  
   byte present = ds.reset();
   ds.select(addr);  
   ds.write(0xBE); // Read Scratchpad
  
   
   for (int i = 0; i < 9; i++) { // we need 9 bytes
    data[i] = ds.read();
   }
   
   ds.reset_search();
   
   byte MSB = data[1];
   byte LSB = data[0];
  
   float tempRead = ((MSB << 8) | LSB); //using two's compliment
   float TemperatureSum = tempRead / 16;
   
   return TemperatureSum;
  
}


void setupBuzzer()
{
    pinMode(BUZZER, OUTPUT);
}


void setupMagnet()
{
    pinMode(MAGNETIC_SWITCH, INPUT);
}

boolean isMagnetic()
{
  if(digitalRead(MAGNETIC_SWITCH) == HIGH)
      return true;
  else
      return false;
}



void setup() {
  //Power up the LoRaBEE - on loraone/sodaq one
  pinMode(ENABLE_PIN_IO, OUTPUT); // ONE

  digitalWrite(beePin, HIGH); // ONE
  delay(3000);

  /* Enable the pins 2/3, 6/7 and 8/9 */
  pinMode(11, OUTPUT);
  digitalWrite(11, HIGH);

  while ((!SerialUSB) && (millis() < 10000)){
    // Wait 10 seconds for the Serial Monitor
  }

  //Set baud rate
  debugSerial.begin(57600);
  loraSerial.begin(LoRaBee.getDefaultBaudRate());

  // Debug output from LoRaBee
  // LoRaBee.setDiag(debugSerial); // optional

  setupLED();
  blink(60);

  /* used for blinking */
  counter=0;

  loudness = 0;

  //connect to the LoRa Network
  setupLoRa();

  setupWater();
  setupBuzzer();
  setupMagnet();
  //setupVibration();

}

void setupLoRa(){
  setupLoRaOTAA();
}

void sendPacket(String packet){
  switch (LoRaBee.sendReqAck(1, (uint8_t*)packet.c_str(), packet.length(), 8))
    {
    case NoError:
      debugSerial.println("Successful transmission.");
      break;
    case NoResponse:
      debugSerial.println("There was no response from the device.");
      setupLoRa();
      break;
    case Timeout:
      debugSerial.println("Connection timed-out. Check your serial connection to the device! Sleeping for 20sec.");
      delay(20000);
      break;
    case PayloadSizeError:
      debugSerial.println("The size of the payload is greater than allowed. Transmission failed!");
      break;
    case InternalError:
      debugSerial.println("Oh No! This shouldn't happen. Something is really wrong! Try restarting the device!\r\nThe network connection will reset.");
      setupLoRa();
      break;
    case Busy:
      debugSerial.println("The device is busy. Sleeping for 10 extra seconds.");
      delay(10000);
      break;
    case NetworkFatalError:
      debugSerial.println("There is a non-recoverable error with the network connection. You should re-connect.\r\nThe network connection will reset.");
      setupLoRa();
      break;
    case NotConnected:
      debugSerial.println("The device is not connected to the network. Please connect to the network before attempting to send data.\r\nThe network connection will reset.");
      setupLoRa();
      break;
    case NoAcknowledgment:
      debugSerial.println("There was no acknowledgment sent back!");
      // When you this message you are probaly out of range of the network.
      break;
    default:
      break;
    }
}

void loop() {


  /* Announce begin of code */
  blink(20); delay(50);
  blink(20); delay(50);
  blink(20); delay(50);

  loudness = readLoudness();

  String data_loudness = String("loudness=" + String(loudness, DEC));
  debugSerial.println(data_loudness);

  String data_light = String("light=" + String(readLight(), 3));
  debugSerial.println(data_light);

  String data_temp = String("temp=" + String(readTemp(), 3));
  debugSerial.println(data_temp);


  String data_water;
  if(hasWater()) {
      data_water = String("water=1");
      buzzOn();
  } else {
      data_water = String("water=0");
      buzzOff();
  }
  debugSerial.println(data_water);

  String data_magnet;
  if(isMagnetic()) {
      data_magnet = String("magnet=1");
  } else {
      data_magnet = String("magnet=0");
  }
  debugSerial.println(data_magnet);

  /*String data_vibration;
  if (readVibration()) {
    data_vibration = String("vibration=1");
    buzzOn();
  } else {
    data_vibration = String("vibration=0");
    buzzOff();
  }
  debugSerial.println(data_vibration);*/

  //sendPacket(data_vibration);
  //blink(20); delay(2980);
  sendPacket(data_loudness);
  blink(20); delay(2980);
  sendPacket(data_temp);
  blink(20); delay(2980);
  sendPacket(data_light);
  blink(20); delay(2980);
  sendPacket(data_water);
  blink(20); delay(2980);
  sendPacket(data_magnet);


  /* Blink long after sending packet
  if(counter >= 10) {
      // Beep(20);
      blink(20);
      delay(10);
      blink(20);
      sendPacket(data_temp);
      blink(500);
      sendPacket(data_loudness);
      blink(500);
      sendPacket(data_light);
      blink(500);
      sendPacket(data_water);
      blink(500);
      sendPacket(data_magnet);
      counter = 0;
  } else {
      blink(30);
      counter++;
  }
  */

  // delay(1000);
}
