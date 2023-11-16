/*
Breadboard Setup
Start Button = PK2 (Analog In A10)
Vent (Stepper Motor) Button = PK3 (Analog In A11)
Yellow LED = PD0 (Communication 21)
Green LED = PD1 (Communication 20)
Blue LED = PD2 (Communication 19)
Red LED = PD3 (Communication 18)
LCD RS (PWM 11) = 11
LCD EN (PWM 12) = 12
LCD D4 (PWM 2) = 2
LCD D5 (PWM 3) = 3
LCD D6 (PWM 4) = 4
LCD D7 (PWM 5) = 5
Stepper Motor IN1 = 7
Stepper Motor IN2 = 8
Stepper Motor IN3 = 9
Stepper Motor IN4 = 10
DC (Fan) Motor Speed Pin: 1 = PD4 (Communication 17)
DC (Fan) Motor IN1: 2 = PD5 (Communication 16)
DC (Fan) Motor IN2: 7 = PD6 (Communication 15)
SDA Clock = A4 (Analog In)
SCL Clock = A5 (Analog In)
*/


/*
To Test:
LEDs
Stepper Motor
Fan
Buttons
Serial Port
Clock
LCD
*/

/*
  Missing code:
  Water level and Temp (and their thresholds)
*/

// Download time library 1.6.1
// Download stepper library
#include <LiquidCrystal.h>
#include <time.h>
#include <Stepper.h>
#include <RTClib.h>
#include <Wire.h>

#define WRITE_HIGH_PD(pin_num)  *port_d |= (0x01 << pin_num);
#define WRITE_LOW_PD(pin_num)  *port_d &= ~(0x01 << pin_num);

#define RDA 0x80
#define TBE 0x20

int state;
// 0 = DISABLED
// 1 = IDLE
// 2 = RUNNING
// 3 = ERROR

// Define Port K Register Pointers
volatile unsigned char* port_k = (unsigned char*) 0x108; 
volatile unsigned char* ddr_k  = (unsigned char*) 0x107; 
volatile unsigned char* pin_k  = (unsigned char*) 0x106;

// Define Port D Register Pointers
volatile unsigned char* port_d = (unsigned char*) 0x2B; 
volatile unsigned char* ddr_d  = (unsigned char*) 0x2A; 
volatile unsigned char* pin_d  = (unsigned char*) 0x29;

// Setup UART
volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;
volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;

// Defines the number of steps per rotation
const int stepsPerRevolution = 2038;

// Creates an instance of stepper class
// Pins entered in sequence IN1-IN3-IN2-IN4 for proper step sequence
Stepper myStepper = Stepper(stepsPerRevolution, 7, 9, 8, 10);

//DC (Fan) Motor Pins
int speedPin = 4;
int dir1 = 5;
int dir2 = 6;
//DC (Fan) Motor Speed
int mSpeed = 90;

const int RS = 11, EN = 12, D4 = 2, D5 = 3, D6 = 4, D7 = 5;
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);

RTC_DS3231 rtc;
char t[32];

void setup()
{
  //Serial.begin(9600);
  // setup the UART
  U0init(9600);

  Wire.begin();
  rtc.begin();
  rtc.adjust(DateTime(F(__DATE__),F(__TIME__)));

  lcd.begin(16, 2);
  
  //set PD0 to OUTPUT
  *ddr_d |= 0x01;
  //set PD1 to OUTPUT
  *ddr_d |= 0x02;
  //set PD2 to OUTPUT
  *ddr_d |= 0x04;
  //set PD3 to OUTPUT
  *ddr_d |= 0x08;
  
  
  //set PD4 to OUTPUT
  *ddr_d |= 0x10;
  //set PD5 to OUTPUT
  *ddr_d |= 0x20;
  //set PD6 to OUTPUT
  *ddr_d |= 0x40;

  //set PK2 to INPUT
  *ddr_k &= 0xFB;


  //set PK3 to INPUT
  *ddr_k &= 0xF7;
  
  // enable the pullup resistor on PK2
  *port_k |= 0x04;
}



void loop()
{
  bool startButton;
  if(*pin_k & 0x04) {
    startButton = true;
    // To keep true without holding button. Test to make sure it works
    *pin_k = *pin_k & 0x04;
  } else {
    startButton = false;
    // To keep false without holding button. Test to make sure it works
    *pin_k = *pin_k & 0xFB;
  }

  bool ventButton;
  if(*pin_k & 0x08) {
    ventButton = true;
    // To keep true without holding button. Test to make sure it works
    // PD3
    *pin_k = *pin_k & 0x08;
  } else {
    ventButton = false;
    // To keep false without holding button. Test to make sure it works
    // PD3
    *pin_k = *pin_k & 0xF7;
  }

  switch (state)
  {
    case 0: // Time of state change and motor position change to serial port
              timeToSerial();
              stateToSerial(state);
              U0putchar('\n');
            // Fan OFF
              stopFan();
            // Yellow LED ON
              // drive PD0 HIGH
              WRITE_HIGH_PD(0);
            // Stop stepper motor
              stopStepperMotor();
            // Monitor start button
              if (startButton) {
                state = 1;
              }
            break;
    case 1: // Time of state change and motor position change to serial port
              timeToSerial();
              stateToSerial(state);
              U0putchar('\n');
            // Fan OFF
              stopFan();
            // Green LED ON
              // drive PD1 HIGH
              WRITE_HIGH_PD(1);
            // LCDTempAndHumidity() ON
            // Monitor water level
            // Respond to change in vent control
              if (ventButton) {
                startStepperMotor();
              } else {
                stopStepperMotor();
              }
            // if (temp>threshhold) {state=2}
            // if (waterLevel<=threshold) {state=3}
            // if (stopButtonPressed) {state=0}
              if (!startButton) {
                stopFan();
                state = 0;
              }
            break;
    case 2: // Time of state change and motor position change to serial port
              timeToSerial();
              stateToSerial(state);
              U0putchar('\n');
            // LCDTempAndHumidity() ON
            // Start fan motor
              startFan();
            // Blue LED ON
              // drive PD2 HIGH
              WRITE_HIGH_PD(2);
              // turn off other LEDs
              WRITE_LOW_PD(0);
              WRITE_LOW_PD(1);
              WRITE_LOW_PD(3);
            // Monitor water level
            // Respond to change in vent control
              if (ventButton) {
                startStepperMotor();
              } else {
                stopStepperMotor();
              }
            // if (temp<=threshhold) {state=1}
            // if (waterLevel<threshold) {state=3}
            // if (stopButtonPressed) {state=0}
              if (!startButton) {
                stopFan();
                state = 0;
              }
            break;
    case 3: // Time of state change and motor position change to serial port
              timeToSerial();
              stateToSerial(state);
              U0putchar('\n');
            // Motor OFF
              stopStepperMotor();
              stopFan();
            // Write error to LCD
              LCDError();
            // Red LED ON (All other leds off)
              // drive PD3 HIGH
              WRITE_HIGH_PD(3);
              // turn off other LEDs
              WRITE_LOW_PD(0);
              WRITE_LOW_PD(1);
              WRITE_LOW_PD(2);
            // Respond to change in vent control
              if (ventButton) {
                startStepperMotor();
              } else {
                stopStepperMotor();
              }
            // if(resetPressed) {state=1}
            // if (stopButtonPressed) {state=0}
              if (!startButton) {
                stopFan();
                state = 0;
              }
            break;
  }
  // LED Delay
  delay(1);
}

void LCDTempAndHumidity(int temp, int humidity) {
  lcd.clear();
  lcd.setCursor(0, 0);
  /*
  lcd.write('A');
  lcd.setCursor(0, 1);
  lcd.write('i');
  lcd.setCursor(0, 2);
  lcd.write('r');
  lcd.setCursor(0, 3);
  lcd.write(' ');
  lcd.setCursor(0, 4);
  lcd.write('T');
  lcd.setCursor(0, 5);
  lcd.write('e');
  lcd.setCursor(0, 6);
  lcd.write('m');
  lcd.setCursor(0, 7);
  lcd.write('p');
  lcd.setCursor(0, 8);
  lcd.write(':');
  lcd.setCursor(0, 9);
  lcd.write(' ');
  lcd.setCursor(0, 10);
  lcd.write(29);
  */

  lcd.print("Air Temp (C): " + temp);

  lcd.setCursor(1, 0);
  lcd.print("Humidity: " + humidity);

}

void LCDError() {
  lcd.clear();
  lcd.setCursor(0, 0);
  /*
  lcd.write('A');
  lcd.setCursor(0, 1);
  lcd.write('i');
  lcd.setCursor(0, 2);
  lcd.write('r');
  lcd.setCursor(0, 3);
  lcd.write(' ');
  lcd.setCursor(0, 4);
  lcd.write('T');
  lcd.setCursor(0, 5);
  lcd.write('e');
  lcd.setCursor(0, 6);
  lcd.write('m');
  lcd.setCursor(0, 7);
  lcd.write('p');
  lcd.setCursor(0, 8);
  lcd.write(':');
  lcd.setCursor(0, 9);
  lcd.write(' ');
  lcd.setCursor(0, 10);
  lcd.write(29);
  */

  lcd.print("ERROR!");

}

void timeToSerial(){
  /*
  time_t t = now();
  unsigned int hour = hour(t);
  unsigned int minute = time.minute();
  unsigned int second = time.second();
  U0putchar(hour + '0');
  U0putchar(':');
  U0putchar(minute + '0');
  U0putchar(':');
  U0putchar(second + '0');
  U0putchar(' ');
  */


  DateTime now = rtc.now();
  char hour = now.hour() + '0';
  char minute = now.minute() + '0';
  char second = now.second() + '0';
  U0putchar(hour);
  U0putchar(':');
  U0putchar(minute);
  U0putchar(':');
  U0putchar(second);
  U0putchar(' ');
}

void stateToSerial(int state){
 if (state==0) {
  U0putchar('D');
  U0putchar('I');
  U0putchar('S');
  U0putchar('A');
  U0putchar('B');
  U0putchar('L');
  U0putchar('E');
  U0putchar('D ');
 } else if (state==1) {
  U0putchar('I');
  U0putchar('D');
  U0putchar('L');
  U0putchar('E ');
 } else if (state==2) {
  U0putchar('R');
  U0putchar('U');
  U0putchar('N');
  U0putchar('N');
  U0putchar('I');
  U0putchar('N');
  U0putchar('G ');
 }  else if (state==3) {
  U0putchar('E');
  U0putchar('R');
  U0putchar('R');
  U0putchar('O');
  U0putchar('R ');
 }
}

void startStepperMotor() {
  // Rotate CW slowly at 5 RPM
  myStepper.setSpeed(5);
  myStepper.step(stepsPerRevolution);
  delay(1000);

  // Print vent position change to serial port
  U0putchar('S');
  U0putchar('T');
  U0putchar('E');
  U0putchar('P');
  U0putchar('P');
  U0putchar('E');
  U0putchar('R ');
  U0putchar('P');
  U0putchar('O');
  U0putchar('S ');
  U0putchar('C');
  U0putchar('H');
  U0putchar('A');
  U0putchar('N');
  U0putchar('G');
  U0putchar('E');
  U0putchar('D');
}

void stopStepperMotor() {
  // Rotate CW at 0 RPM
  myStepper.setSpeed(0);
  myStepper.step(stepsPerRevolution);
  delay(1000);
}


void startFan() {
  WRITE_LOW_PD(dir1);
  WRITE_HIGH_PD(dir2);
  WRITE_HIGH_PD(speedPin);
  delay(25);
}

void stopFan() {
  WRITE_LOW_PD(dir1);
  WRITE_LOW_PD(dir2);
  WRITE_LOW_PD(speedPin);
}

// UART Stuff
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
unsigned char U0getchar()
{
  return *myUDR0;
}
void U0putchar(unsigned char U0pdata)
{
  while((*myUCSR0A & TBE)==0);
  *myUDR0 = U0pdata;
}
