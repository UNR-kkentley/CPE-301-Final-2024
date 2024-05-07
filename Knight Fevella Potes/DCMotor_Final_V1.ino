
#include <DHT.h> //Humidity and Temp
#include <DHT_U.h>

////PORT B INPUT/OUTPUT
volatile unsigned char* port_b = (unsigned char*) 0x25;
volatile unsigned char* ddr_b = (unsigned char*) 0x24;
volatile unsigned char* pin_b = (unsigned char*) 0x23; // DHT PB7 D13 // DC controller (PB4-PB6) (D10-D12) 

#define DHTPIN 13          // Define the pin for DHT
#define DHTTYPE DHT11 
DHT dht(DHTPIN, DHTTYPE); // Initialize DHT sensort

//DC motor pins 
#define motorInput1 11
#define motorInput2 12
#define motorEnablePin 10


void setup() {
  dht.begin();

  set_PB_as_output(11);
  set_PB_as_output(12);
  set_PB_as_output(10);
}

void loop() {
  
  dc_motor(1); //call function for motor control 1 - motor on 0 - motor off

}

void dc_motor(int motorCondition){ //function to control dc motor output

  float temperature = (dht.readTemperature() * 9 / 5 + 32); // convert temperature reading to fahrenheit

  if (motorCondition == 1){ 
    while(temperature > 70.0) { // dc motor will only power on if DHT11 temperature is above threshold 
      write_PB_high(motorInput1); // provide power to dc motor 
      //write_PB_low(motorInput2);
      write_pin(motorEnablePin, 225); // set dc motor speed via enable pin on L293D chip 

      temperature = (dht.readTemperature() * 9 / 5 + 32); //re-measure temperature for while loop condition 
    }
  }
}

void set_PB_as_output(unsigned char pin_num) { //function to set port B to output 
  *ddr_b |= (0x01 << pin_num);
}

void write_PB_high(unsigned char pin_num){  //function to set specified port B pin HIGH 
  *port_b |= (0x01 << pin_num);
}

void set_PB_as_input(unsigned char pin_num) { //function to set port B to output 
  *ddr_b &= ~(1 << pin_num); 
}

void write_PB_low(unsigned char pin_num) { //function to set specified port B pin LOW
  *port_b &= ~(1 << pin_num); 
}

void write_pin(unsigned char pin_num, int value) { //function to set specified value at specified port&pin
  *port_b = (*port_b & ~(1 << pin_num)) | value;
}


