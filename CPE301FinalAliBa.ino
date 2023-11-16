/*
Breadboard Setup
PK2 (Analog In A10) = Button
PD0 (Communication 21) = Yellow LED
PD1 (Communication 20) = Green LED
PD2 (Communication 19) = Blue LED
PD3 (Communication 18) = Red LED
LCD RS (PWM 11) = 11
LCD EN (PWM 12) = 12
LCD D4 (PWM 2) = 2
LCD D5 (PWM 3) = 3
LCD D6 (PWM 4) = 4
LCD D7 (PWM 5) = 5
*/

// Download time library 1.6.1

#include <LiquidCrystal.h>
#include <time.h>

#define WRITE_HIGH_PD(pin_num)  *port_d |= (0x01 << pin_num);
#define WRITE_LOW_PD(pin_num)  *port_d &= ~(0x01 << pin_num);

#define RDA 0x80
#define TBE 0x20

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

int state;
// 0 = DISABLED
// 1 = IDLE
// 2 = RUNNING
// 3 = ERROR

const int RS = 11, EN = 12, D4 = 2, D5 = 3, D6 = 4, D7 = 5;
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);

void setup()
{
  //Serial.begin(9600);
  // setup the UART
  U0init(9600);

  lcd.begin(16, 2);
  
  //set PD0 to OUTPUT
  *ddr_d |= 0x01;

  //set PK2 to INPUT
  *ddr_k &= 0xFB;
  
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

  switch (state)
  {
    case 0: // Time of state change and motor position change to serial port
              //timeToSerial();
              stateToSerial(state);

              U0putchar('\n');
            // Fan OFF
            // Yellow LED ON
              // drive PD0 HIGH
              WRITE_HIGH_PD(0);
            // Stop fan motor
            // Monitor start button
            break;
    case 1: // Time of state change and motor position change to serial port
              //timeToSerial();
              stateToSerial(state);
              
              U0putchar('\n');
            // Fan OFF
            // Green LED ON
              // drive PD1 HIGH
              WRITE_HIGH_PD(1);
            // LCDTempAndHumidity() ON
            // Stop fan motor
            // Monitor water level
            // Respond to change in vent control
            // if (temp>threshhold) {state=2}
            // if (waterLevel<=threshold) {state=3}
            // if (stopButtonPressed) {state=0}
              if (!startButton) {
                state = 0;
              }
            break;
    case 2: // Time of state change and motor position change to serial port
              //timeToSerial();
              stateToSerial(state);
              
              U0putchar('\n');
            // LCDTempAndHumidity() ON
            // Start fan motor
            // Blue LED ON
              // drive PD2 HIGH
              WRITE_HIGH_PD(2);
              // turn off other LEDs
              WRITE_LOW_PD(0);
              WRITE_LOW_PD(1);
              WRITE_LOW_PD(3);
            // Monitor water level
            // Respond to change in vent control
            // if (temp<=threshhold) {state=1}
            // if (waterLevel<threshold) {state=3}
            // if (stopButtonPressed) {state=0}
              if (!startButton) {
                state = 0;
              }
            break;
    case 3: // Time of state change and motor position change to serial port
              //timeToSerial();
              stateToSerial(state);
              
              U0putchar('\n');
            // Motor OFF
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
            // if(resetPressed) {state=1}
            // if (stopButtonPressed) {state=0}
              if (!startButton) {
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
