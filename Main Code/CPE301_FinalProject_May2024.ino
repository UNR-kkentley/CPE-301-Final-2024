
// CPE Final Project May 2024
//Kieran Kentley, Knight Fevella Potes, Eric Cordova Gonzalez, Abigail Ganze

#include <Stepper.h>
#include <LiquidCrystal.h>
#include <RTClib.h>
#include <Wire.h> //I2C
#include <DHT.h>
#include <DHT_U.h>

#define DHTPIN 13
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE); // Initialize DHT sensor
RTC_DS1307 RTC;
DateTime currentTime;

void U0init(int U0baud);
void U0putchar(unsigned char U0pdata);
void adc_init();
unsigned int adc_read(unsigned char adc_channel_num);
void enableLED(int color);
void set_PE_as_input(unsigned char pin_num);
void hInterrupt();

// LCD pins
const int RS = 49, RW = 48, E = 47, D4 = 53, D5 = 51, D6 = 50, D7 = 52;
LiquidCrystal lcd(RS, RW, E, D4, D5, D6, D7);

unsigned long previousMillis = 0; //Previous time
const long interval = 60000; //1-minute delay

unsigned int water_value = adc_read(7); 
#define temp_value 25 

#define RDA 0x80
#define TBE 0x20

#define step_move 16
#define STEPS 64

//Stepper potentiometer variables
int previous_pot_val = 0;
Stepper  myStepper(STEPS, 6, 8, 7, 9); //this is the correct order 6,8,7,9

//DC motor pins
#define motorInput1 11
#define motorInput2 12
#define motorEnablePin 10

#define DEBOUNCE_INT 780  // 50ms

//---------------------------------------------------------

////PORT B INPUT/OUTPUT
volatile unsigned char* port_b = (unsigned char*) 0x25;
volatile unsigned char* ddr_b = (unsigned char*) 0x24;
volatile unsigned char* pin_b = (unsigned char*) 0x23; // DHT PB7 D13 // DC controller (PB4-PB6) (D10-D12)

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

// Port F Addresses for Input/Output
volatile unsigned char* port_f = (unsigned char*) 0x31; 
volatile unsigned char* ddr_f = (unsigned char*) 0x30;  
volatile unsigned char* pin_f = (unsigned char*) 0x2F;  

//PORT L INPUT/OUTPUT
volatile unsigned char* port_l = (unsigned char*) 0x10B;
volatile unsigned char* ddr_l = (unsigned char*) 0x10A;
volatile unsigned char* pin_l = (unsigned char*) 0x109; //LCD Pins

volatile unsigned char* myTCCR1A = (unsigned char*) 0x80;
volatile unsigned char* myTCCR1B = (unsigned char*) 0x81;
volatile unsigned char* myTCCR1C = (unsigned char*) 0x82;
volatile unsigned char* myTIMSK1 = (unsigned char*) 0x6F;
volatile unsigned int* myTCNT1  = (unsigned int*) 0x84;
volatile unsigned char* myTIFR1 = (unsigned char*) 0x36;

volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;
volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;

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
  lcd.begin(16, 2); // Set up the LCD
  *ddr_a |= 0b00111110; // Set LED pins PA# as outputs
  myStepper.setSpeed(128);
  Wire.begin();
  RTC.begin();

  if (!RTC.isrunning()) {
    RTC.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  presentState = DISABLED_ST;
  nextState = DISABLED_ST;
  enableLED(presentState);

  set_PE_as_input(4); // Set up button as input
  set_PB_as_output(11);//DC motor pins
  set_PB_as_output(12);
  set_PB_as_output(10);

  attachInterrupt(digitalPinToInterrupt(2), hInterrupt, FALLING);
}
void loop() {
  unsigned int water_value = adc_read(7);
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();
  currentTime = RTC.now();

  if (presentState == RUNNING_ST) {
    dc_motor(1);
  } else {
    dc_motor(0);
  }

  if (presentState == RUNNING_ST || presentState == IDLE_ST) {
    stepperMotor(1);
  } else {
    stepperMotor(0);
  }

  switch (presentState) {
    case DISABLED_ST:
      if (buttonPressed) {
        buttonPressed = false;
        nextState = IDLE_ST;
      }
      U0PrintTime("DISABLED STATE");
      dc_motor(0);
      lcd.clear();
      lcd.print("Disabled");
      break;

    case IDLE_ST:
      htLCD(humidity, temperature, presentState);
      U0PrintTime("IDLE STATE ");
      dc_motor(0);
      if (buttonPressed) {
        buttonPressed = false;
        nextState = DISABLED_ST;
      } else if (water_value <= 102) {
        nextState = ERROR_ST;
      } else if (water_value > 102 && temperature > temp_value) {
        nextState = RUNNING_ST;
      }
      break;

    case RUNNING_ST:
      htLCD(humidity, temperature, presentState);
      U0PrintTime("RUNNING STATE");
      dc_motor(1);
      if (temperature < temp_value) {
        nextState = IDLE_ST;
      } else if (buttonPressed) {
        buttonPressed = false;
        nextState = DISABLED_ST;
      } else if (water_value <= 102) {
        nextState = ERROR_ST;
      }
      break;

    case ERROR_ST:
      U0PrintTime("Water low, ERROR STATE");
      dc_motor(0);
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Error Message:");
      lcd.setCursor(0, 1);
      lcd.print("Water Level Low");
      enableLED(ERROR_ST);
      if (buttonPressed) {
        buttonPressed = false;
        nextState = DISABLED_ST;
      }
      break;
  }

  if (nextState != presentState) {
    presentState = nextState;
    enableLED(presentState);
    if (presentState != IDLE_ST && presentState != RUNNING_ST) {
      stepperMotor(0); // Turn off stepper motor when not in IDLE_ST or RUNNING_ST
    }
  }
}

void U0PrintTime(String s) {
  String U0Send = String(currentTime.year());
  for (int i = 0; i < U0Send.length(); i++) {
    U0putchar(U0Send[i]);
  }

  U0putchar('/');
  U0Send = currentTime.month();

  for (int i = 0; i < U0Send.length(); i++) {
    U0putchar(U0Send[i]);
  }

  U0putchar('/');
  U0Send = currentTime.day();

  for (int i = 0; i < U0Send.length(); i++) {
    U0putchar(U0Send[i]);
  }

  U0putchar(' ');
  U0Send = currentTime.hour();

  for (int i = 0; i < U0Send.length(); i++) {
    U0putchar(U0Send[i]);
  }

  U0putchar(':');
  U0Send = currentTime.minute();

  for (int i = 0; i < U0Send.length(); i++) {
    U0putchar(U0Send[i]);
  }

  U0putchar(':');
  U0Send = currentTime.second();

  for (int i = 0; i < U0Send.length(); i++) {
    U0putchar(U0Send[i]);
  }

  s += "\n";

  for (int i = 0; i < s.length(); i++) {
    U0putchar(s[i]);
  }
}

//--------------------------------------------------------------------------------

void adc_init()
{
  // setup the A register
  *my_ADCSRA |= 0b10000000; 
  *my_ADCSRA &= 0b11011111; 
  *my_ADCSRA &= 0b11110111; 
  *my_ADCSRA &= 0b11111000; 
  
  *my_ADCSRB &= 0b11110111; 
  *my_ADCSRB &= 0b11111000; 
  
  *my_ADMUX  &= 0b01111111; 
  *my_ADMUX  |= 0b01000000; 
  *my_ADMUX  &= 0b11011111; 
  *my_ADMUX  &= 0b11100000; 
}

unsigned int adc_read(unsigned char adc_channel_num)
{
  *my_ADMUX  &= 0b11100000;
  *my_ADCSRB &= 0b11110111;
  // set the channel number
  if (adc_channel_num > 7)
  {
    adc_channel_num -= 8;
    *my_ADCSRB |= 0b00001000;
  }
  *my_ADMUX  += adc_channel_num;
  *my_ADCSRA |= 0x40;
  while ((*my_ADCSRA & 0x40) != 0);
  // return the result
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

void htLCD(float humidity, float temp, int presentState) {
  static unsigned long previousMillis = 0; //Previous state
  const long interval = 60000; //1-minute delay

  static int previousState = DISABLED_ST;
  unsigned long currentMillis = millis();

  if (presentState == RUNNING_ST || presentState == IDLE_ST) {
    if ((presentState != previousState) || (currentMillis - previousMillis >= interval)) {
      previousMillis = currentMillis; 
      previousState = presentState;
      lcd.setCursor(0, 0);
      lcd.print("Temp: ");
      lcd.print(temp);
      lcd.print("C");
      lcd.setCursor(0, 1);
      lcd.print("Humidity: ");
      lcd.print(humidity);
      lcd.print("%");

    }
  }

  else if (presentState == DISABLED_ST){
    lcd.clear();
    lcd.print("Disabled");
  }
}
//------------------

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

void set_PF_as_input(unsigned char pin_num) {
  *ddr_f &= ~(1 << pin_num);
}

void set_PE_as_input(unsigned char pin_num) {
  *ddr_e &= ~(0x01 << pin_num);

}

void set_PB_as_output(unsigned char pin_num) {
  *ddr_b |= (0x01 << pin_num);
}

void write_PB_high(unsigned char pin_num) {
  *port_b |= (0x01 << pin_num);
}

void set_PB_as_input(unsigned char pin_num) {
  *ddr_b &= ~(1 << pin_num);
}

void write_PB_low(unsigned char pin_num) {
  *port_b &= ~(1 << pin_num);
}

void dc_motor(int motorCondition) {
  static bool motorRunning = false;

  //motorEnablePin
  //motorInput1,2
  if (motorCondition == 1 && !motorRunning) {
    write_PB_high(11);
    write_PB_low(12);
    analogWrite(10, 225);
    U0Send("Fan motor turned on ");
    motorRunning = true;
  } else if (motorCondition == 0 && motorRunning) {
    write_PB_low(10);
    U0Send("Fan motor turned off ");
    motorRunning = false;
  }
}

void stepperMotor(int stepperState) {
  static int lastSeg = -1;
  int current_pot_val = adc_read(9);
  int segSize = 1023 / 15;
  int currentSeg = current_pot_val / segSize;

  if (stepperState == 1 && currentSeg != lastSeg) {
    int stepCount = (currentSeg - lastSeg) * step_move;  // Calculate steps to move
    myStepper.step(stepCount);
    U0Send("Stepper motor moved position");
    lastSeg = currentSeg;
  } else if (stepperState == 0) {
    lastSeg = -1;  // Reset
  }
}

void U0Send (String s) {
  String U0Print = String(currentTime.year());
  for (int i = 0; i < U0Print.length(); i++) {
    U0putchar(U0Print[i]);
  }
  
  U0putchar('/');
  U0Print = currentTime.month();

  for (int i = 0; i < U0Print.length(); i++) {
    U0putchar(U0Print[i]);
  }

  U0putchar('/');
  U0Print = currentTime.day();

  for (int i = 0; i < U0Print.length(); i++) {
    U0putchar(U0Print[i]);
  }

  U0putchar(' ');
  U0Print = currentTime.hour();

  for (int i = 0; i < U0Print.length(); i++) {
    U0putchar(U0Print[i]);
  }

  U0putchar(':');
  U0Print = currentTime.minute();

  for (int i = 0; i < U0Print.length(); i++) {
    U0putchar(U0Print[i]);
  }

  U0putchar(':');
  U0Print = currentTime.second();

  for (int i = 0; i < U0Print.length(); i++) {
    U0putchar(U0Print[i]);
  }

  s += "\n";

  for (int i = 0; i < s.length(); i++) {
    U0putchar(s[i]);
  }
}

void InterruptTimer() {
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 65535 - DEBOUNCE_INT;  // Set the timer count
  TIMSK1 &= ~(1 << TOIE1);
  TCCR1B |= (1 << CS12);    // Set the prescaler and start timer
  TIMSK1 |= (1 << TOIE1);
}

ISR(TIMER1_OVF_vect) {
  TCNT1 = 65535 - DEBOUNCE_INT;  // Reload timer
  buttonPressed = true;
  TIMSK1 &= ~(1 << TOIE1);
}
//Checking this to make sure the debounce is good, seeing mixed results on scope
void hInterrupt() {
  TIMSK1 &= ~(1 << TOIE1);
  TCNT1 = 65535 - DEBOUNCE_INT;  // Restart the debounce timer
  TIMSK1 |= (1 << TOIE1);
}
