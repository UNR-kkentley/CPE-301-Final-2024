// CPE Final Project April 2024
//Kieran Kentley, Knight Fevella Potes, Eric Cordova Gonzalez, Abigail Ganze

//include libraries that are allowed
#include <Stepper.h> //Includes the Arduino Stepper Library

volatile bool currentStatus = false;
volatile bool previousStatus = true;

//Stepper potentiometer variables 
float old_step_count = 0.0; //number of previous step count 
float current_step_count = 0.0; //number of current step count 
const int stepper_total_steps = 64; //number of steps in full cycle //might need to change to 2048
Stepper  myStepper(stepper_total_steps, 6, 7, 8, 9) ; //set up for the stepper motor 

//angleStepperVariables 
signed int stepper_move_back = -256;   //move the motor -256 steps (-45 degrees)
signed int stepper_move_forward = 256; //move the motor 256 steps (45 degrees)
int angle_limit = 90;      //start in the "middle"


void U0init(int U0baud);
void adc_init();

//Error State
bool errorState = false;


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
  myStepper.setSpeed(300) ; //sets stepper motor movement speed to 30rpms
}

void loop() {
  while(errorState == false){ //this part good and loops 
    stepper_motor_potentiometer(1);

    if(errorState == true) { //exit condition 
      return;
    }
    //my_delay(1000);
    errorState == false;
  }
  
}

void stepper_motor_potentiometer(int stepperCondition) {// stepper motor for potentiometer configuration 
    //potentiometer code

    //read potentiometer //taking potentiometer analog reading from analog port 1 //do not need to setup to be input, analogRead does that automatically
    current_step_count = adc_read(9);

    //current_step_count = map(adc_read(9),0,1024,0, stepper_total_steps);
    //map(value, fromLow, fromHigh, toLow, toHigh)  

    if(stepperCondition == 1){ //this part not good 
      //if(current_step_count != old_step_count){
        myStepper.step(current_step_count - old_step_count); //move stepper with respect to potentiometer readings 
        old_step_count = current_step_count; //save latest step count 
        //my_delay(1000);
     //}
    }
    return;
}

void set_PF_as_input(unsigned char pin_num) { //function to set port B to output 
  *ddr_f &= ~(1 << pin_num); 
}
void set_PE_as_input(unsigned char pin_num) { //function to set port B to output 
  *ddr_e &= ~(1 << pin_num); 
}

unsigned int adc_read(unsigned char adc_channel_num)
{
  // clear the channel selection bits (MUX 4:0)
  *my_ADMUX  &= 0b11100000;
  // clear the channel selection bits (MUX 5)
  *my_ADCSRB &= 0b11110111;
  // set the channel number
  if(adc_channel_num > 7)
  {
    // set the channel selection bits, but remove the most significant bit (bit 3)
    adc_channel_num -= 8;
    // set MUX bit 5
    *my_ADCSRB |= 0b00001000;
  }
  // set the channel selection bits
  *my_ADMUX  += adc_channel_num;
  // set bit 6 of ADCSRA to 1 to start a conversion
  *my_ADCSRA |= 0x40;
  // wait for the conversion to complete
  while((*my_ADCSRA & 0x40) != 0);
  // return the result in the ADC data register
  return *my_ADC_DATA;
}

void my_delay(unsigned int milliseconds)
{
    double period = milliseconds / 1000.0;
    double half_period = period / 2.0;
    double clk_period = 0.0000000625;
    unsigned int ticks = half_period / clk_period;
    *myTCCR1B &= 0xF8;
    *myTCNT1 = (unsigned int) (65535 - ticks);
    *myTCCR1B |= 0x02;
    *myTCCR1A = 0x00;

    while ((*myTIFR1 & 0x01) == 0);
    *myTCCR1B &= 0xF8;
    *myTIFR1 |= 0x01;
}





