// CPE Final Project April 2024
//Kieran Kentley, Knight Fevella Potes, Eric Cordova Gonzalez, Abigail Ganze

//include libraries that are allowed
#include <Stepper.h> //Includes the Arduino Stepper Library

volatile bool currentStatus = false;
volatile bool previousStatus = true;

//const int stepsPerRevolution = 2038;
#define step_move 16
#define STEPS 64

//Stepper potentiometer variables
int previous_pot_val = 0;
Stepper  myStepper(STEPS, 6, 8, 7, 9) ; //declaring  stepper motor  //this is the correct order 6,8,7,9



void U0init(int U0baud);
void adc_init();


////PORT E INPUT/OUTPUT
volatile unsigned char* port_e = (unsigned char*) 0x2E;
volatile unsigned char* ddr_e = (unsigned char*) 0x2D;
volatile unsigned char* pin_e = (unsigned char*) 0x2C; // Pushbutton Stepper D3 PE5. //Pushbutton on-off D2 PE4 Must use: ISR 

// Port F Addresses for Input/Output
volatile unsigned char* port_f = (unsigned char*) 0x31; // Data Register for PORT F
volatile unsigned char* ddr_f = (unsigned char*) 0x30;  // Data Direction Register for PORT F
volatile unsigned char* pin_f = (unsigned char*) 0x2F;  // Input Pins Address for PORT F

//PORT L INPUT/OUTPUT
volatile unsigned char* port_l = (unsigned char*) 0x10B;
volatile unsigned char* ddr_l = (unsigned char*) 0x10A;
volatile unsigned char* pin_l = (unsigned char*) 0x109; //LCD Pins 

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

#define RDA 0x80
#define TBE 0x20  
volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;
volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;

volatile unsigned char* myEIMSK = (unsigned char*) 0x1D; 
volatile unsigned char* myEIFR = (unsigned char*) 0x1C;  
volatile unsigned char* mySREG = (unsigned char*) 0x3F; 
volatile unsigned char* myEICRB = (unsigned char*) 0x6A;


void setup() {
  myStepper.setSpeed(128) ;
}

void loop() {
  stepperMotor(1);
}


void stepperMotor(int stepperCondition) {
  if (stepperCondition == 1){
    int current_pot_val = analogRead(9); //adc_read does not work, stepper does not loop if using adc_read

    myStepper.step(current_pot_val - previous_pot_val);
    
    previous_pot_val = current_pot_val;
  }
}


void set_PF_as_input(unsigned char pin_num) { //function to set port B to output 
  *ddr_f &= ~(1 << pin_num); 
}
void set_PE_as_input(unsigned char pin_num) { //function to set port B to output 
  *ddr_e &= ~(1 << pin_num); 
}







