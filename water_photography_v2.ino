

// Include the Bounce2 library found here :
// https://github.com/thomasfredericks/Bounce2
#include <Bounce2.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>




// functions
ISR(TIMER1_COMPA_vect); // accurate delay using timer 1 interrupt.
void init_timer1_delay_with_interrupts();
void delay_with_interrupts(int delayValue); // delayValue is in ms
void turnSolenoidOn();
void turnSolenoidOff();
void takePicture();
void displayUpdate();
void turnShutterON();
void turnShutterOFF();
void turnFocusON();
void turnFocusOFF();

// pins
#define shutterPin 3
#define FocusPin 4
#define waterValvePin 5
#define POTpin A0
#define POTpin2 A1
#define BUTTON_PIN 7 // the pin which is used to trigger the water dropping sequence
#define LED_PIN 13
#define shutterResponseTime 0 // shutter response time when shutter pin is activated in ms or relay delay

//constants
#define I2C_address 0x3F
#define DISPLAY_DELAY 1000

//objects
LiquidCrystal_I2C lcd(I2C_address, 16, 2);
Bounce debouncer = Bounce(); // Instantiate a Bounce object

//global variables
unsigned long lastDisplayUpdate = 0;
int dropletSizeDelayValue = 0;
int shutterDelayValue = 200;
volatile bool delayCompleteFlag = false;
int ledState = LOW;
char message ;


void setup() {

  init_timer1_delay_with_interrupts(); 
  Serial.begin(9600);
  Serial.println("options");
  Serial.println("A Focus ON");
  Serial.println("B Focus OFF");
  Serial.println("C Shutter ON");
  Serial.println("D Shutter OFF");
  Serial.println("E takePicture");

  lcd.begin();

  
  lcd.backlight(); // Turn on the blacklight and print a message.
  debouncer.attach(BUTTON_PIN, INPUT_PULLUP); // Attach the debouncer to a pin with INPUT_PULLUP mode
  debouncer.interval(25); // Use a debounce interval of 25 milliseconds


  pinMode(LED_PIN, OUTPUT); // Setup the LED
  pinMode(shutterPin, OUTPUT);
  pinMode(FocusPin, OUTPUT);
  pinMode(waterValvePin, OUTPUT);
  digitalWrite(LED_PIN, ledState);

}



void loop() {


  if (Serial.available())
  {
    message = Serial.read();
    switch (message)
    {
      case 'A': turnFocusON();    break;
      case 'B': turnFocusOFF();   break;
      case 'C': turnShutterON();   break;
      case 'D': turnShutterOFF();  break;
      case 'E': takePicture();
    }
  }
  debouncer.update(); // Update the Bounce instance

  dropletSizeDelayValue = map(analogRead(POTpin2), 0, 1023, 5, 250);
  shutterDelayValue = map(analogRead(POTpin), 0, 1023, 20, 200);

  if ( debouncer.fell() ) {  // Call code if button transitions from HIGH to LOW
    ledState = !ledState; // Toggle LED state
    digitalWrite(LED_PIN, ledState); // Apply new LED state
    turnFocusON(); // comment out this line if using MANUAL FOCUS
    turnSolenoidOn();
    delay_with_interrupts(dropletSizeDelayValue); // later change delay to millis() // turns on solenoid for dropletSizeDelayValue ms
    turnSolenoidOff();
    delay_with_interrupts(shutterDelayValue  - shutterResponseTime);
    turnShutterON();
    delay_with_interrupts(20);
    turnShutterOFF();
    turnFocusOFF();

  }
  else
  {
    displayUpdate(); // only display when switch is not pressed, improves timing.
  }

}

void turnSolenoidOn()
{
  Serial.println("waterValve ON");
  digitalWrite(waterValvePin, HIGH);

}

void turnSolenoidOff()
{
  Serial.println("waterValve OFF");
  digitalWrite(waterValvePin, LOW);

}



void turnShutterON()
{
  Serial.println("Shutter ON");
  digitalWrite(shutterPin, HIGH);
}
void turnShutterOFF()
{
  Serial.println(" Shutter OFF");
  digitalWrite(shutterPin, LOW);
}
void turnFocusON()
{
  Serial.println("Focus ON");
  digitalWrite(FocusPin, HIGH);
}
void turnFocusOFF()
{
  Serial.println(" Focus OFF");
  digitalWrite(FocusPin, LOW);
}

void takePicture()
{
  Serial.println("Picture taken");
  turnShutterON();
  delay(10);
  turnShutterOFF();
}


void displayUpdate() // updates I2C display and Serial monitor display if enabled in "SerialDebugging"
{
  if ((millis() - lastDisplayUpdate) > DISPLAY_DELAY) {
    lastDisplayUpdate = millis();
    lcd.clear();
    lcd.print("delay:");
    lcd.print(dropletSizeDelayValue);
    lcd.print("ms");
    lcd.setCursor(0, 1);
    lcd.print("S Delay:");
    lcd.print(shutterDelayValue);
    lcd.print("ms");


  }

}


void init_timer1_delay_with_interrupts()
{
  cli();
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B


  TCCR1B = (1 << WGM12); // turn on CTC mode
  TCCR1B |= (1 << CS12) | (1 << CS10);  // Set CS10 and CS12 bits for 1024 prescaler
  OCR1A = 65532;// = (16*10^6) / (1*1024) - 1 (must be <65536)
  sei();

}

void delay_with_interrupts(int delay_value)
{

  float freq = 1000 / delay_value;
  float temp_value = 15625.0; // (F_clk) / /(1024)
  OCR1A = (uint16_t) 15625.0 / freq - 1; // = (16*10^6) / (freq*1024) - 1 (must be <65536)
  // Serial.print("OCR1A = ");
  // Serial.println(OCR1A);

  TCNT1  = 0;
  TIMSK1 |= (1 << OCIE1A);
  TCNT1  = 0;//initialize counter value to 0
  delayCompleteFlag = false;
  sei();
  // Serial.print("delayCompleteFlag = ");
  // Serial.println(delayCompleteFlag);
  while (delayCompleteFlag == false)
  {
   // for (int i = 0; i < 9; i++); // waits until delay period is finished

  }

  delayCompleteFlag = false;
  TIMSK1 &= ~(1 << OCIE1A); // disables timer1 interrupt
  cli();

}

ISR(TIMER1_COMPA_vect)
{
  delayCompleteFlag = true;

}
