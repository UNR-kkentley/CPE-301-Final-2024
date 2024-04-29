// CPE Final Project April 2024
//Kieran Kentley, Knight Fevella Potes, Eric Cordova Gonzalez, Abigail Ganze

//include libraries that are allowed

////PORT B INPUT/OUTPUT
volatile unsigned char* port_b = (unsigned char*) 0x25;
volatile unsigned char* ddr_b = (unsigned char*) 0x24;
volatile unsigned char* pin_b = (unsigned char*) 0x23;

////PORT A INPUT / OUTPUT
volatile unsigned char* port_h = (unsigned char*) 0x25;
volatile unsigned char* ddr_h = (unsigned char*) 0x24;
volatile unsigned char* pin_h = (unsigned char*) 0x23;

////PORT E INPUT/OUTPUT
volatile unsigned char* port_e = (unsigned char*) 0x25;
volatile unsigned char* ddr_e = (unsigned char*) 0x24;
volatile unsigned char* pin_e = (unsigned char*) 0x23;

////PORT D INPUT/OUTPUT
volatile unsigned char* port_d = (unsigned char*) 0x25;
volatile unsigned char* ddr_d = (unsigned char*) 0x24;
volatile unsigned char* pin_d = (unsigned char*) 0x23;

////PORT A INPUT/OUTPUT
volatile unsigned char* port_a = (unsigned char*) 0x25;
volatile unsigned char* ddr_a = (unsigned char*) 0x24;
volatile unsigned char* pin_a = (unsigned char*) 0x23;

//Place holder for Analog?
//volatile unsigned char* port_b = (unsigned char*) 0x25;
//volatile unsigned char* ddr_b = (unsigned char*) 0x24;
//volatile unsigned char* pin_b = (unsigned char*) 0x23;

volatile unsigned char* myTCCR1A = (unsigned char*) 0x80;
volatile unsigned char* myTCCR1B = (unsigned char*) 0x81;
volatile unsigned char* myTCCR1C = (unsigned char*) 0x82;
volatile unsigned char* myTIMSK1 = (unsigned char*) 0x6F;
volatile unsigned int* myTCNT1  = (unsigned  int*) 0x84;
volatile unsigned char* myTIFR1 =  (unsigned char*) 0x36;

volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;


void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
