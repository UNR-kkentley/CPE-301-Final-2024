
#include <DHT.h> //Humidity and Temp
#include <DHT_U.h>

#define DHTPIN 13          // Define the pin for DHT
#define DHTTYPE DHT11 
DHT dht(DHTPIN, DHTTYPE); // Initialize DHT sensor


const int motorInput1 = 11;
const int motorInput2 = 12; 
const int motorEnablePin = 10;

void setup() {
  pinMode(motorInput1, OUTPUT); // motor input can be used to control fan direction, both off = Fan off, one on = fan spinning 
  pinMode(motorInput2, OUTPUT); //dont need both motor input since we just need ON & off and do not need to control direction 
  pinMode(motorEnablePin, OUTPUT); //controls Dc motor speed 
  dht.begin();

  analogWrite(motorEnablePin, 100);
}

void loop() {
  dc_motor();
}

void dc_motor(){
  float temperature = (dht.readTemperature() * 9 / 5 + 32); 
  
  while (temperature < 100.0) { 
    digitalWrite(motorInput1, HIGH); // Turn motor on
    digitalWrite(motorInput2, LOW); // Turn motor on
  } 
}


