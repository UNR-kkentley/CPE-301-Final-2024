//include libraries that are allowed
#include <Stepper.h> //Includes the Arduino Stepper Library
#include <LiquidCrystal.h> //LCD Library
#include <RTClib.h> //RTC
#include <Wire.h> //I2C
#include <DHT.h> //Humidity and Temp
#include <DHT_U.h>

#define DHTPIN 13          // Define the pin for DHT
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE); // Initialize DHT sensor

void U0init(int U0baud);
void adc_init();
unsigned int adc_read(unsigned char adc_channel_num);
void enableLED(int color);
void set_PE_as_input(unsigned char pin_num);

// LCD pins <--> Arduino pins
const int RS = 49, RW = 48, E = 47, D4 = 53, D5 = 51, D6 = 50, D7 = 52;
LiquidCrystal lcd(RS, RW, E, D4, D5, D6, D7);

unsigned long previousMillis = 0; //Previous state
const long interval = 60000; //1-min delay

unsigned int adc_read(unsigned char adc_channel_num);
unsigned int water_value = adc_read(7); //Read from ADC channel 7
#define temp_value 20

#define RDA 0x80
#define TBE 0x20

//---------------------------------------------------------

////PORT B INPUT/OUTPUT
volatile unsigned char* port_b = (unsigned char*) 0x25;
volatile unsigned char* ddr_b = (unsigned char*) 0x24;
volatile unsigned char* pin_b = (unsigned char*) 0x23; // DHT PB7 D13 // DC controller (PB4-PB6) (D10-D12)

////PORT H INPUT / OUTPUT
volatile unsigned char* port_h = (unsigned char*) 0x102;
volatile unsigned char* ddr_h = (unsigned char*) 0x101;
volatile unsigned char* pin_h = (unsigned char*) 0x100;// DC controller (PH5-PH6) (D6-D7)

////PORT E INPUT/OUTPUT
volatile unsigned char* port_e = (unsigned char*) 0x2E;
volatile unsigned char* ddr_e = (unsigned char*) 0x2D;
volatile unsigned char* pin_e = (unsigned char*) 0x2C; // Pushbutton Stepper D3 PE5. //Pushbutton on-off D2 PE4 Must use: ISR

////PORT D INPUT/OUTPUT
volatile unsigned char* port_d = (unsigned char*) 0x2B;
volatile unsigned char* ddr_d = (unsigned char*) 0x2A;
volatile unsigned char* pin_d = (unsigned char*) 0x29; // Real Time Clock RTC D20-D21 PD0-PD1

////PORT A INPUT/OUTPUT
volatile unsigned char* port_a = (unsigned char*) 0x22;
volatile unsigned char* ddr_a = (unsigned char*) 0x21;
volatile unsigned char* pin_a = (unsigned char*) 0x20; //Water Sensor A7 PF7 //Green LED D23 PA1 //Yellow LED D24 PA2 //Blue LED D25 PA3 //Red LED D27 PA5

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

volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;
volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;

volatile unsigned char* myEIMSK = (unsigned char*) 0x1D;
volatile unsigned char* myEIFR = (unsigned char*) 0x1C;
volatile unsigned char* mySREG = (unsigned char*) 0x3F;
volatile unsigned char* myEICRB = (unsigned char*) 0x6A;



//------------------------------------------------------

const int DISABLED_ST = 1;
const int IDLE_ST = 0;
const int RUNNING_ST = 2;
const int ERROR_ST = 3;

int presentState;
int nextState;

volatile bool buttonPressed = false;

void setup() {
  U0init(9600);
  adc_init();
  dht.begin();
  lcd.begin(16, 2); // set up number of columns and rows
  *ddr_a |= 0b00001111;

  set_PE_as_input(4);

  presentState = DISABLED_ST;
  nextState = IDLE_ST;
  enableLED(presentState);

  attachInterrupt(digitalPinToInterrupt(2), hInterrupt, FALLING);

}

void loop() {

  unsigned int water_value = adc_read(7); //Read from ADC channel 7
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();

  while (presentState == DISABLED_ST) {
    lcd.clear();
    lcd.setCursor(0, 0);
    //time print for motor disable
    lcd.print("Disabled");

    nextState = IDLE_ST;
  }


  if (buttonPressed) {
    buttonPressed = false;
    if (presentState == IDLE_ST) {
      //print something
      nextState = RUNNING_ST;
    }

    else if (presentState == ERROR_ST) {
      //print error state message time
      nextState = IDLE_ST;
    }
    else {
      nextState = IDLE_ST;
      //print idle message to time
    }
  }
  if (water_value <= 120) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Error Message:");
    lcd.setCursor(0, 1);
    lcd.print("Water Level Low");

    nextState = ERROR_ST;
  }
  else if (water_value > 120 && temperature < temp_value) {

    unsigned long currentMillis = millis(); // Get the current time

    if (currentMillis - previousMillis >= interval) { // Check if it's time to update the LCD
      previousMillis = currentMillis; // Save the last time the LCD was updated

      // Display temperature and humidity on LCD
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Temp: ");
      lcd.print(temperature * 9 / 5 + 32);
      lcd.print(" F");

      lcd.setCursor(0, 1);
      lcd.print("Humidity: ");
      lcd.print(humidity);
      lcd.print("%");

    }

    presentState = RUNNING_ST;
    //anything needed for running state
  }

  presentState = nextState;
  enableLED(presentState); //Find another way to get this into func
}

//--------------------------------------------------------------------------------


void adc_init()
{
  // setup the A register
  *my_ADCSRA |= 0b10000000; // set bit   7 to 1 to enable the ADC
  *my_ADCSRA &= 0b11011111; // clear bit 6 to 0 to disable the ADC trigger mode
  *my_ADCSRA &= 0b11110111; // clear bit 5 to 0 to disable the ADC interrupt
  *my_ADCSRA &= 0b11111000; // clear bit 0-2 to 0 to set prescaler selection to slow reading
  //the prescaler bit is the division factor between the clocks
  // setup the B register
  *my_ADCSRB &= 0b11110111; // clear bit 3 to 0 to reset the channel and gain bits
  *my_ADCSRB &= 0b11111000; // clear bit 2-0 to 0 to set free running mode
  // setup the MUX Register
  *my_ADMUX  &= 0b01111111; // clear bit 7 to 0 for AVCC analog reference
  *my_ADMUX  |= 0b01000000; // set bit   6 to 1 for AVCC analog reference
  *my_ADMUX  &= 0b11011111; // clear bit 5 to 0 for right adjust result
  *my_ADMUX  &= 0b11100000; // clear bit 4-0 to 0 to reset the channel and gain bits
}

unsigned int adc_read(unsigned char adc_channel_num)
{
  // clear the channel selection bits (MUX 4:0)
  *my_ADMUX  &= 0b11100000;
  // clear the channel selection bits (MUX 5)
  *my_ADCSRB &= 0b11110111;
  // set the channel number
  if (adc_channel_num > 7)
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
  while ((*my_ADCSRA & 0x40) != 0);
  // return the result in the ADC data register
  return *my_ADC_DATA;
}

void U0init(int U0baud)
{
  unsigned long FCPU = 16000000;
  unsigned int tbaud;
  tbaud = (FCPU / 16 / U0baud - 1);
  // Same as (FCPU / (16 * U0baud)) - 1;
  *myUCSR0A = 0x20;
  *myUCSR0B = 0x18;
  *myUCSR0C = 0x06;
  *myUBRR0  = tbaud;
}

unsigned char U0kbhit()
{
  return *myUCSR0A & RDA;
}

void U0putchar(unsigned char U0pdata)
{
  while ((*myUCSR0A & TBE) == 0);
  *myUDR0 = U0pdata;
}


void my_delay(unsigned int freq)
{
  // calc period
  double period = 1.0 / double(freq);
  // 50% duty cycle
  double half_period = period / 2.0f;
  // clock period def
  double clk_period = 0.0000000625;
  // calc ticks
  unsigned int ticks = half_period / clk_period;
  //Serial.println(ticks);
  // stop the timer
  *myTCCR1B &= 0xF8;
  // set the counts
  *myTCNT1 = (unsigned int) (65535 - ticks); //(65536 - ticks)
  // start the timer
  * myTCCR1B |= 0b00000010;
  //prescaler 0b00000001 - changed to be before TCCR1A is set to 0 for start
  * myTCCR1A = 0x0;

  // wait for overflow
  while ((*myTIFR1 & 0x01) == 0); // 0b 0000 0000
  // stop the timer
  *myTCCR1B &= 0xF8;   // 0b 0000 0000
  // reset TOV
  *myTIFR1 |= 0x01;
}

void enableLED(int state) {
    *port_a &= ~(0b00111110); // Turn off all LEDs
    switch (state) {
        case DISABLED_ST:
            *port_a |= (1 << 2); // Yellow LED
            break;
        case IDLE_ST:
            *port_a |= (1 << 1); // Green LED
            break;
        case RUNNING_ST:
            *port_a |= (1 << 3); // Blue LED
            break;
        case ERROR_ST:
            *port_a |= (1 << 5); // Red LED
            break;
    }
}

//------------------------------------------------------

void set_PE_as_input(unsigned char pin_num) {
  *ddr_e &= ~(0x01 << pin_num);
  *port_e |= (1 << pin_num);

}

void hInterrupt()
{
  static unsigned long lastInterruptTime = 0;
  unsigned long interruptTime = millis();

  if (interruptTime - lastInterruptTime > 50) {
    buttonPressed = true;
    lastInterruptTime = interruptTime;
  }
}
