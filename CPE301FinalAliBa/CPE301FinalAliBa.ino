/*
Breadboard Setup (in order)
Yellow LED = PD0 (Communication 21)
Green LED = PD1 (Communication 20)
Blue LED = PD2 (Communication 19)
Red LED = PD3 (Communication 18)
  330 Resistor for LEDs
  Long side is positive on LEDs
  Positive into resistor
  Make sure resistors aren't touching
Start Button = PK2 (Analog In A10)
  1K resistor for button into negative side
  Positive into other side
Stop Button = PK3 (Analog In A11)
  1K resistor for button into negative side
  Positive into other side
LCD RS (PWM 11) = 11
LCD EN (E) (PWM 12) = 12
LCD D4 (PWM 2) = 2
LCD D5 (PWM 3) = 3
LCD D6 (PWM 4) = 4
LCD D7 (PWM 5) = 5
  A into (light side) 330 resistor (dark side) into positive
  VSS into negative
  VDD into positive
  RW into negative
  K into negative
  NEED Pontentiometer (Side w/ 1 by itself is to VO, right side to negative and left to positive)
Power Supply: https://www.youtube.com/watch?v=1er6XQ-BMp4
  Press white button to power on/off
Temp/Humidity Sensor = 6 (PWM 6)
  right to left: (signal, positive, negative)
  keep touching it to make work
Water sensor info: https://lastminuteengineers.com/water-level-sensor-arduino-tutorial/
Water Sensor Signal = PK4: A12 (Analog In)
Water Sensor Power = PB7: 13 (PWM 13) 
  Into 13 NOT 7
  Use female to male wires
How to work clock: https://lastminuteengineers.com/ds1307-rtc-arduino-tutorial/
SDA Clock = A4 (Analog In)
SCL Clock = A5 (Analog In)
Vent (Stepper Motor) Button = PK5 (Analog In A13)
Stepper Motor IN1 = 7
Stepper Motor IN2 = 8
Stepper Motor IN3 = 9
Stepper Motor IN4 = 10
DC (Fan) Motor Speed Pin: 1 = PD4 (Communication 17)
DC (Fan) Motor IN1: 2 = PD5 (Communication 16)
DC (Fan) Motor IN2: 7 = PD6 (Communication 15)
*/

/*
Breadboard Setup V2 (in order)
Pinout: https://www.electronicshub.org/wp-content/uploads/2021/01/Arduino-Mega-Pinout.jpg
Yellow LED = PK0 (Analog In A8)
Green LED = PK1 (Analog In A9)
Blue LED = PK2 (Analog In A10)
Red LED = PK3 (Analog In A11)
  330 Resistor for LEDs
  Long side is positive on LEDs
  Positive into resistor
  Make sure resistors aren't touching
Start Button = PK4 (Analog In A12)
  1K resistor for button into negative side
  Positive into other side
Stop Button = PK5 (Analog In A13)
  1K resistor for button into negative side
  Positive into other side
LCD RS (PWM 11) = 11
LCD EN (E) (PWM 12) = 12
LCD D4 (PWM 2) = 2
LCD D5 (PWM 3) = 3
LCD D6 (PWM 4) = 4
LCD D7 (PWM 5) = 5
  A into (light side) 330 resistor (dark side) into positive
  VSS into negative
  VDD into positive
  RW into negative
  K into negative
  NEED Pontentiometer (Side w/ 1 by itself is to VO, right side to negative and left to positive)
Power Supply: https://www.youtube.com/watch?v=1er6XQ-BMp4
  Press WHITE button to power on/off
Temp/Humidity Sensor = 6 (PWM 6)
  right to left: (signal, positive, negative)
  keep touching it to make work
Water sensor info: https://lastminuteengineers.com/water-level-sensor-arduino-tutorial/
Water Sensor Signal = PK6: A14 (Analog In)
Water Sensor Power = PB7: 13 (PWM 13) 
  Into 13 NOT 7
  Use female to male wires
How to work clock: https://lastminuteengineers.com/ds1307-rtc-arduino-tutorial/
Clock code: https://circuitdigest.com/microcontroller-projects/interfacing-ds3231-rtc-with-arduino-and-diy-digital-clock
SDA Clock = 20 (Communication 20)
SCL Clock = 21 (Communication 21)
Vent (Stepper Motor) Button = PK7 (Analog In A15)
Stepper Motor IN1 = 7
Stepper Motor IN2 = 8
Stepper Motor IN3 = 9
Stepper Motor IN4 = 10
DC (Fan) Motor Speed Pin: 1 = PD4 (Communication 17)
DC (Fan) Motor IN1: 2 = PD5 (Communication 16)
DC (Fan) Motor IN2: 7 = PD6 (Communication 15)

*/

/*
To Test:
LEDs*
Stepper start and stop don't work rn
Stepper Motor
Vent Button
Fan
Start Button*
Stop Button*
Serial Port*
Clock*
LCD*
Water Sensor*
Temp and Humidity*
*/

/*
  Missing code:
*/

// Download time library 1.6.1
// Download stepper library
// Download RTCLib
// Download DS3231
// Download DHTLib library
// Download uRTCLib by Naguissa
#include <LiquidCrystal.h>
#include <time.h>
#include <Stepper.h>
#include <RTClib.h>
#include <Wire.h>
#include <dht.h>
// #include <uRTCLib.h>

#define WRITE_HIGH_PD(pin_num)  *port_d |= (0x01 << pin_num);
#define WRITE_LOW_PD(pin_num)  *port_d &= ~(0x01 << pin_num);

#define WRITE_HIGH_PB(pin_num)  *port_b |= (0x01 << pin_num);
#define WRITE_LOW_PB(pin_num)  *port_b &= ~(0x01 << pin_num);

#define WRITE_HIGH_PK(pin_num)  *port_k |= (0x01 << pin_num);
#define WRITE_LOW_PK(pin_num)  *port_k &= ~(0x01 << pin_num);

#define RDA 0x80
#define TBE 0x20

#define DHT11_PIN 6

int state = 0;
//int state;
// 0 = DISABLED
// 1 = IDLE
// 2 = RUNNING
// 3 = ERROR

float tempThreshold = 40.0;
int waterThreshold = 1;

// Define Port K Register Pointers
volatile unsigned char* port_k = (unsigned char*) 0x108; 
volatile unsigned char* ddr_k  = (unsigned char*) 0x107; 
volatile unsigned char* pin_k  = (unsigned char*) 0x106;

// Define Port D Register Pointers
volatile unsigned char* port_d = (unsigned char*) 0x2B; 
volatile unsigned char* ddr_d  = (unsigned char*) 0x2A; 
volatile unsigned char* pin_d  = (unsigned char*) 0x29;

// Define Port B Register Pointers
volatile unsigned char* port_b = (unsigned char*) 0x25; 
volatile unsigned char* ddr_b  = (unsigned char*) 0x24; 
volatile unsigned char* pin_b  = (unsigned char*) 0x23;

// Setup UART
volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;
volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;

// Setup ADC
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

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

// Setup LCD pins and create LCD instance
const int RS = 11, EN = 12, D4 = 2, D5 = 3, D6 = 4, D7 = 5;
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);

// Time/Clock
RTC_DS3231 rtc;
char dt[32];
char t[14];

//uRTCLib rtc(0x68);
//char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

// Temp and humidity sensor
dht DHT;

// Water level from sensor
int waterLevel;

// Start/Stop button
bool startButton = false;
bool stopButton = false;

void setup()
{
  //Serial.begin(9600);
  // setup the UART
  U0init(9600);
  
  // setup the ADC
  adc_init();

  // setup time/clock
  Wire.begin();
  rtc.begin();
  rtc.adjust(DateTime(F(__DATE__),F(__TIME__)));
  
  //URTCLIB_WIRE.begin();
  //rtc.set(0, 56, 12, 5, 13, 1, 22);

  // Wake up LCD
  lcd.begin(16, 2);
  
  //set PK0 to OUTPUT
  *ddr_k |= 0x01;
  //set PK1 to OUTPUT
  *ddr_k |= 0x02;
  //set PK2 to OUTPUT
  *ddr_k |= 0x04;
  //set PK3 to OUTPUT
  *ddr_k |= 0x08;
  
  
  //set PD4 to OUTPUT
  //*ddr_d |= 0x10;
  //set PD5 to OUTPUT
  //*ddr_d |= 0x20;
  //set PD6 to OUTPUT
  //*ddr_d |= 0x40;

  //set PK4 to INPUT
  *ddr_k &= 0xEF;
  //set PK5 to INPUT
  *ddr_k &= 0xDF;
  //set PK6 to INPUT
  *ddr_k &= 0xBF;
  //set PK7 to INPUT
  *ddr_k &= 0x7F;

  //set PB7 to OUTPUT
  *ddr_b |= 0x80;

  // Water sensor OFF
  WRITE_LOW_PB(7);
  
  // enable the pullup resistor on PK4
  *port_k |= 0x10;
  // enable the pullup resistor on PK5
  *port_k |= 0x20;

  // enable the pullup resistor on PK6
  *port_k |= 0x40;

  // enable the pullup resistor on PK7
  *port_k |= 0x80;
}



void loop()
{
  // If clicked will start
  if(*pin_k & 0x10) {
    delay(100);
    startButton = true;
  } else {
    startButton = false;
  }

  // If clicked will stop
  if(*pin_k & 0x20) {
    delay(100);
    stopButton = true;
  } else {
    stopButton = false;
  }

  // If pressed vent moves slowly then stops when let go
  // bool ventButton;
  // if(*pin_k & 0x80) {
  //   ventButton = true;
  //   // To keep true without holding button. SHOULDN'T NEED SINCE WE WANT HOLDING BUTTON
  // } else {
  //   ventButton = false;
  //   // To keep false without holding button. SHOULDN'T NEED SINCE WE WANT HOLDING BUTTON
  // }

  // Get tempurature and humidity
  int chk = DHT.read11(DHT11_PIN);
  float temperature = DHT.temperature;
  float humidity = DHT.humidity;
  // float temperature = 22.0;
  // float humidity = 78.0;

  // rtc.refresh();

  // Moniter Water Level
  waterLevel = readWaterSensor();

  switch (state)
  {
    case 0: // Time of state change and motor position change to serial port
              timeToSerial();
              stateToSerial(state);
              U0putchar('\n');
              
            // Fan OFF
              //stopFan();
              
            // Yellow LED ON
              turnOnLED(state);
              
            // Stop stepper motor
              //stopStepperMotor();
              
            // LCD
              LCDDisabled();
              
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
              //stopFan();
              
            // Green LED ON
              turnOnLED(state);
              
            // LCDTempAndHumidity() ON
              LCDTempAndHumidity(temperature, humidity);
              
            // Monitor water level
            waterLevel = readWaterSensor();

            // Temperary to check if water sensor works
            Serial.println(waterLevel); 
            
            // Respond to change in vent control
              // if (ventButton) {
              //   startStepperMotor();
              // } else {
              //   stopStepperMotor();
              // }
              
            // if (temp>threshhold) {state=2}
              if (temperature>tempThreshold) {
                state=2;
              }
              
            // if (waterLevel<=threshold) {state=3}
              if (waterLevel <= waterThreshold) {
                state = 3;
              }
              
            // if (stopButtonPressed) {state=0}
              if (stopButton) {
                //stopFan();
                state = 0;
              }
            break;
    case 2: // Time of state change and motor position change to serial port
              timeToSerial();
              stateToSerial(state);
              U0putchar('\n');

             // Blue LED ON
              turnOnLED(state);
              
            // LCDTempAndHumidity() ON
              LCDTempAndHumidity(temperature, humidity);
              
            // Start fan motor
              //startFan();
           
            // Monitor water level
            waterLevel = readWaterSensor();
              
            // Respond to change in vent control
              // if (ventButton) {
              //   startStepperMotor();
              // } else {
              //   stopStepperMotor();
              // }
              
            // if (temp<=threshhold) {state=1}
              if (temperature<=tempThreshold) {
                state=1;
              }
              
            // if (waterLevel<threshold) {state=3}
              if (waterLevel < waterThreshold) {
                state = 3;
              }
              
            // if (stopButtonPressed) {state=0}
              if (stopButton) {
                //stopFan();
                state = 0;
              }
            break;
    case 3: // Time of state change and motor position change to serial port
              timeToSerial();
              stateToSerial(state);
              U0putchar('\n');
              
            // Motor OFF
              //stopStepperMotor();
              //stopFan();
              
            // Write error to LCD
              LCDError();
              
            // Red LED ON (All other leds off)
              turnOnLED(state);
              
            // Respond to change in vent control
              // if (ventButton) {
              //   startStepperMotor();
              // } else {
              //   stopStepperMotor();
              // }
              
            // if(resetPressed) {state=1}
              if (startButton) {
                state = 1;
              }
            
            // if (stopButtonPressed) {state=0}
              if (stopButton) {
                //stopFan();
                state = 0;
              }
            break;
  }
  // LED Delay
  delay(100);
}

void turnOnLED(int state) {
  
  // turn off ALL LEDs
  WRITE_LOW_PK(0);
  WRITE_LOW_PK(1);
  WRITE_LOW_PK(2);
  WRITE_LOW_PK(3);

  // drive LED HIGH
  WRITE_HIGH_PK(state);
  
//   if (state==0) {
//   // Yellow LED ON
//     // drive PK0 HIGH
//     WRITE_HIGH_PK(0);
//     // turn off other LEDs
//     WRITE_LOW_PK(1);
//     WRITE_LOW_PK(2);
//     WRITE_LOW_PK(3);
//  } else if (state==1) {
//   // Green LED ON
//     // drive PK1 HIGH
//     WRITE_HIGH_PK(1);
//     // turn off other LEDs
//     WRITE_LOW_PK(0);
//     WRITE_LOW_PK(2);
//     WRITE_LOW_PK(3);
//  } else if (state==2) {
//   // Blue LED ON
//     // drive PK2 HIGH
//     WRITE_HIGH_PK(2);
//     // turn off other LEDs
//     WRITE_LOW_PK(0);
//     WRITE_LOW_PK(1);
//     WRITE_LOW_PK(3);
//  }  else if (state==3) {
//   // Red LED ON
//     // drive PK3 HIGH
//     WRITE_HIGH_PK(3);
//     // turn off other LEDs
//     WRITE_LOW_PK(0);
//     WRITE_LOW_PK(1);
//     WRITE_LOW_PK(2);
//  }
}

void LCDTempAndHumidity(float temp, float humidity) {
  lcd.clear();
  lcd.setCursor(0, 0);

  lcd.print("Air Temp (C): ");
  lcd.println((float)temp, 2);

  lcd.setCursor(0, 1);
  lcd.print("Humidity (%): ");
  lcd.println((float)humidity, 2);
  // Delay for 1 min
  //delay(60000);
}

void LCDError() {
  lcd.clear();
  lcd.setCursor(0, 0);

  lcd.print("ERROR!");

}

void LCDDisabled() {
  lcd.clear();
  lcd.setCursor(0, 0);

  lcd.print("DISABLED");

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


  // DateTime now = rtc.now();
  // char hour = now.hour() + '0';
  // char minute = now.minute() + '0';
  // char second = now.second() + '0';
  // U0putchar(hour);
  // U0putchar(':');
  // U0putchar(minute);
  // U0putchar(':');
  // U0putchar(second);
  // U0putchar(' ');

  DateTime now = rtc.now();
  sprintf(t, "Time: %02d:%02d:%02d", now.hour(), now.minute(), now.second());
  for (int i=0; i<sizeof(t); i++) {
    U0putchar(t[i]);
  }
  U0putchar(' ');
  delay(10);
}

void stateToSerial(int state){
  U0putchar('S');
  U0putchar('t');
  U0putchar('a');
  U0putchar('t');
  U0putchar('e');
  U0putchar(':');
  U0putchar(' ');
 if (state==0) {
  U0putchar('D');
  U0putchar('I');
  U0putchar('S');
  U0putchar('A');
  U0putchar('B');
  U0putchar('L');
  U0putchar('E');
  U0putchar('D');
  U0putchar(' ');
 } else if (state==1) {
  U0putchar('I');
  U0putchar('D');
  U0putchar('L');
  U0putchar('E');
  U0putchar(' ');
 } else if (state==2) {
  U0putchar('R');
  U0putchar('U');
  U0putchar('N');
  U0putchar('N');
  U0putchar('I');
  U0putchar('N');
  U0putchar('G');
  U0putchar(' ');
 }  else if (state==3) {
  U0putchar('E');
  U0putchar('R');
  U0putchar('R');
  U0putchar('O');
  U0putchar('R');
  U0putchar(' ');
 }
}

void startStepperMotor() {
  // Rotate CW slowly at 5 RPM
  myStepper.setSpeed(5);
  myStepper.step(stepsPerRevolution);
  delay(1000);

  // Print date and time
  DateTime now = rtc.now();
  sprintf(dt, "Date/Time: %02d:%02d:%02d %02d/%02d/%02d", now.hour(), now.minute(), now.second(), now.day(), now.month(), now.year());
  for (int i=0; i<sizeof(dt); i++) {
    U0putchar(dt[i]);
  }
  U0putchar(' ');
  delay(10);

  // Print vent position change to serial port
  U0putchar('S');
  U0putchar('T');
  U0putchar('E');
  U0putchar('P');
  U0putchar('P');
  U0putchar('E');
  U0putchar('R');
  U0putchar(' ');
  U0putchar('P');
  U0putchar('O');
  U0putchar('S');
  U0putchar(' ');
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

  // Print date and time
  DateTime now = rtc.now();
  sprintf(dt, "Date/Time: %02d:%02d:%02d %02d/%02d/%02d", now.hour(), now.minute(), now.second(), now.day(), now.month(), now.year());
  for (int i=0; i<sizeof(dt); i++) {
    U0putchar(dt[i]);
  }
  U0putchar(' ');
  delay(10);
}

int readWaterSensor() {
  int waterLevel;
  // Water sensor ON
  WRITE_HIGH_PB(7);
  delay(10);
  //Read water level
  waterLevel = adc_read(14);
  // Water sensor OFF
  WRITE_LOW_PB(7);
  return waterLevel;
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

// ADC Stuff
void adc_init()
{
  // setup the A register
  *my_ADCSRA |= 0b10000000; // set bit   7 to 1 to enable the ADC
  *my_ADCSRA &= 0b11011111; // clear bit 6 to 0 to disable the ADC trigger mode
  *my_ADCSRA &= 0b11110111; // clear bit 5 to 0 to disable the ADC interrupt
  *my_ADCSRA &= 0b11111000; // clear bit 0-2 to 0 to set prescaler selection to slow reading
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
