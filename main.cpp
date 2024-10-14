#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"

// UNO and NANO Oled Display PINS:	SDA/A4	SCL/A5

// for debugging, serialprint on = true, off = false, slowing down the Atmega chip!
bool debug = false;

// Defining arduino pins:
#define SERVOLOCK_PIN 5
#define SERVOHACK_PIN 6
#define LIGHTSENSOR_PIN A0
#define GATESENSOR_PIN A1
#define SECONDGATESENSOR_PIN A2 // in case a second gate sensor is connected
#define LEDGREEN_PIN 3          // Kathode, Anode to 5V, switched by GND
#define LEDRED_PIN 4            // Kathode, Anode to 5V, switched by GND
#define REEDOPEN_PIN 7
#define REEDCLOSE_PIN 8
#define ENCODER_CLK 11
#define ENCODER_DT 12
#define ENCODER_SW 13
#define LEDIR_PIN 9

// Defining parameters (all in milli-seconds)
#define LIGHTSENSOR_INTERVAL 100 // how oft a messurement is executed; every 100 ms
#define GATESENSOR_INTERVAL 100  // how oft a messurement is executed; every 100 ms
#define TIME_MAX_RANGE 120000    // 1 minute per Step: 7200000    // border between light and dark testvalue: 1 second per step: 120000
#define LED_FLASHRATE 300        // Flashrate on/off 300 ms
// #define IR_FLASHRATE 26
#define BUTTON_PRESSED_TIME_MIN 50
#define BUTTON_PRESSED_TIME_BETWEEN 2000
#define BUTTON_PRESSED_TIME_MAX 2100

#define FOTOSENSOR_TIME_BELOW_ABOVE 2000 // how long must the fotosensorvalue be below above the thresholdvalue, this must prevent the system from oscillating wenn the fotoSensorValue is more or less the same as the threshold value, default is 1 second

// Defining step counter or factor values
#define SERVOLOCK_SPEED 10
#define SERVOHACK_SPEED 2
#define LIGHTSENSOR_THRESHOLD_MAX 1024 // range from 0 till 1024?? needs calibrating with the fotoresistor!
#define GATESENSOR_THRESHOLD 512       // is set to 50% as default
#define RTE_MAX_TIME_STEPS 120
#define RTE_MULTIPLIER 10
#define RTE_MAX_LIGHT_STEPS 10

// Servo settings in degrees
#define SERVOLOCK_MIN 0
#define SERVOLOCK_MAX 180
#define SERVOHACK_STOP 90
#define SERVOHACK_OPEN 180
#define SERVOHACK_CLOSE 0

// Settings for OLED Display 0X3C+SA0 - 0x3C or 0x3D
#define I2C_ADDRESS 0x3C
// Define proper RST_PIN if required.
#define RST_PIN -1

// defining the lines in the Oled Display, max 4
char *line0;
char *line1;
char *line2;
char *line3;

char *line3B; // backup a line text in display to save display text between status changes

// defining all variables needed:
unsigned long LightSensorThreshold = 512; // border between light and dark initial value 50% = 512
unsigned long timeSunset = 4000;          // time after Sunset, initial value = 4
unsigned long timeSunrise = 4000;         // time after Sunrise, initial value = 4
int RotaryLightSensorThreshold = 5;       // border between light and dark initial value 50% = 5
int RotarytimeSunset = 4;                 // time after Sunset, initial value = 4
int RotarytimeSunrise = 4;                // time after Sunrise, initial value = 4

// defining variables for loops and status
unsigned long previousMillis;
unsigned long currentMillis;
int caseNumber;
bool open;
bool doorway;

unsigned int fotoSensorValue = 512; // initial value of 50%
unsigned int gateSensorValue = 512;
unsigned int secondgateSensorValue = 512; // in case a second gate sensor is connected
bool reedStatusClose;
bool reedStatusOpen;

int fontIndexNumber = 0; // select font Oled display
int rotaryCounter;       // counting the steps of the rotary encoder
bool buttonStatus = 0;   // stores the last button status
char buffer[8];          // needed to display an system integer on the oled display
int menuItem;            // defines which menu is displayed

bool printFlag; // refreshes display if it is set true
bool openMenu;  // opens and closes the menu on the Oled display
bool prevLightSensorThreshold;
bool prevGateSensorThreshold;

bool flashred;
bool flashgreen;
bool closeCompleted;
bool openCompleted;
bool calibrationCompleted;

// bool oneloop;
bool oneloop;  // bool if a line of code should be executed just once
bool oneloop1; // iteration to count system loops if a line of code should be executed just once
bool oneloop2;
bool oneloop3;

long timebuttonrelease = 0; // sets the time point if the button is released
long timebuttonpress = 0;   // same but visa versa
long timepressed = 0;       // stores the duration time of the pressed button

long timeFotoLightSensorValueBelow1 = 0; // sets the time point if the button is released
long timeFotoLightSensorValueAbove1 = 0; // same but visa versa
long timeFotoLightSensorValueBelow2 = 0; // sets the time point if the button is released
long timeFotoLightSensorValueAbove2 = 0; // same but visa versa
long timeBelow = 0;                      // stores the duration time of the pressed button
long timeAbove = 0;                      // stores the duration time of the pressed button

int prevRotCounter = 0;      // checks if values have changed in 1 loop
bool prevButtonStatus = LOW; // checks if values have changed in 1 loop

// font list for the oled display
const char *fontName[] = {
    "Arial10",
    "Arial_bold_14",
    "Callibri11",
    "Callibri11_bold",
    "Callibri11_italic",
    "Callibri13",
    "Corsiva_12",
    "fixed_bold10x15",
    "font5x7",
    "font8x8",
    "Iain5x7",
    "lcd5x7",
    "Stang5x7",
    "System5x7",
    "TimesNewRoman16",
    "TimesNewRoman16_bold",
    "TimesNewRoman16_italic",
    "utf8font10x16",
    "Verdana12",
    "Verdana12_bold",
    "Verdana12_italic",
    "X11fixed7x14",
    "X11fixed7x14B",
    "ZevvPeep8x16"};
const uint8_t *fontList[] = {
    Arial14,
    Arial_bold_14,
    Callibri11,
    Callibri11_bold,
    Callibri11_italic,
    Callibri15,
    Corsiva_12,
    fixed_bold10x15,
    font5x7,
    font8x8,
    Iain5x7,
    lcd5x7,
    Stang5x7,
    System5x7,
    TimesNewRoman16,
    TimesNewRoman16_bold,
    TimesNewRoman16_italic,
    utf8font10x16,
    Verdana12,
    Verdana12_bold,
    Verdana12_italic,
    X11fixed7x14,
    X11fixed7x14B,
    ZevvPeep8x16};
// uint8_t nFont = sizeof(fontList)/sizeof(uint8_t*);

// defining the classes:

class ServoMove
{
  Servo servo; // the servo

  int increment;            // increment to move for each interval
  int updateInterval;       // interval between updates
  unsigned long lastUpdate; // last update of position
  int *ppos = &pos;         // pointer to current servo position

public:
  int pos = 90; // current servo position
  ServoMove(int interval)
  {
    updateInterval = interval;
    increment = 1;
  }

  void Attach(int pin) // detach and attach, turns the PWM signal on and off, it prevents the servos from buzzing, and getting warm in case a load is put on the servo output spline
  {
    servo.attach(pin);
  }

  void Detach()
  {
    servo.detach();
  }

  void MovePos(int setpos)
  {
    *ppos > setpos ? increment = -1 : increment = 1;                 // set direction
    if ((millis() - lastUpdate) > updateInterval && *ppos != setpos) // time to update
    {
      lastUpdate = millis();
      *ppos += increment;
      servo.write(*ppos);
      if (debug == true)
      {
        Serial.println("Servo position = ");
        Serial.println(*ppos);
      }
    }
  }
};

class LED
{
  // Class Member Variables
  // These are initialized at startup
  int ledPin;     // the number of the LED pin
  long flashrate; // milliseconds of on-time

public:
  LED(int pin, long flash)
  {
    ledPin = pin;
    pinMode(ledPin, OUTPUT);
    flashrate = flash;
    digitalWrite(ledPin, LOW);
  }

  void flash()
  {
    timer(flashrate) == 1 ? digitalWrite(ledPin, !digitalRead(ledPin)) : void();
  }
  void on()
  {
    digitalRead(ledPin) == HIGH ? digitalWrite(ledPin, LOW) : void();
  }
  void off()
  {
    digitalRead(ledPin) == LOW ? digitalWrite(ledPin, HIGH) : void();
  }

private:
  unsigned long timervar;
  unsigned long millisold = 0;
  unsigned int timer(long timerset)
  {
    timervar = millis();
    if (timervar - millisold > timerset)
    {
      millisold = timervar;
      return 1;
    }
    else
      return 0;
  }
};

class FotoSensor
{
  int sensorPin;
  long messureInterval;
  int *pmessuredValue = &messuredValue;

public:
  int messuredValue = 0;
  FotoSensor(int pin, long interval)
  {
    sensorPin = pin;
    pinMode(sensorPin, INPUT);
    messureInterval = interval;
  }
  void messure()
  {
    timer(messureInterval) == 1 ? *pmessuredValue = analogRead(sensorPin) : *pmessuredValue;
    if (debug == true)
    {
      Serial.println("Sensor messured value = ");
      Serial.println(messuredValue);
    }
  }

private:
  unsigned long timervar;
  unsigned long millisold = 0;
  unsigned int timer(long timerset)
  {
    timervar = millis();
    if (timervar - millisold > timerset)
    {
      millisold = timervar;
      return 1;
    }
    else
      return 0;
  }
};

class ReedSensor
{
  int reedPin;

public:
  int reedState;
  int *preedState = &reedState;
  ReedSensor(int pin)
  {
    reedPin = pin;
    pinMode(reedPin, INPUT_PULLUP);
  }
  int messure()
  {
    *preedState = digitalRead(reedPin);
    if (debug == true)
    {
      Serial.println("Reedstatus =");
      Serial.println(reedState);
    }
    return reedState;
  }
};

// Creating objects:
ServoMove servolock(SERVOLOCK_SPEED);
ServoMove servohack(SERVOHACK_SPEED);
LED ledgreen(LEDGREEN_PIN, LED_FLASHRATE);
LED ledred(LEDRED_PIN, LED_FLASHRATE);
FotoSensor lightfotosensor(LIGHTSENSOR_PIN, LIGHTSENSOR_INTERVAL);
FotoSensor gatefotosensor(GATESENSOR_PIN, GATESENSOR_INTERVAL);
FotoSensor secondgatefotosensor(SECONDGATESENSOR_PIN, GATESENSOR_INTERVAL); // in case a second gate sensor is connected
ReedSensor reedsensorOpen(REEDOPEN_PIN);
ReedSensor reedsensorClose(REEDCLOSE_PIN);
SSD1306AsciiWire oled;

// defined functions

// function reading rotary encoder (no bouncing?)
void readEncoder()
{
  static uint8_t state = 0;
  bool CLKstate = digitalRead(ENCODER_CLK);
  bool DTstate = digitalRead(ENCODER_DT);
  switch (state)
  {
  case 0: // Idle state, encoder not turning
    if (!CLKstate)
    { // Turn clockwise and CLK goes low first
      state = 1;
    }
    else if (!DTstate)
    { // Turn anticlockwise and DT goes low first
      state = 4;
    }
    break;
  // Clockwise rotation
  case 1:
    if (!DTstate)
    { // Continue clockwise and DT will go low after CLK
      state = 2;
    }
    break;
  case 2:
    if (CLKstate)
    { // Turn further and CLK will go high first
      state = 3;
    }
    break;
  case 3:
    if (CLKstate && DTstate)
    { // Both CLK and DT now high as the encoder completes one step clockwise
      state = 0;
      ++rotaryCounter;
    }
    break;
  // Anticlockwise rotation
  case 4: // As for clockwise but with CLK and DT reversed
    if (!CLKstate)
    {
      state = 5;
    }
    break;
  case 5:
    if (DTstate)
    {
      state = 6;
    }
    break;
  case 6:
    if (CLKstate && DTstate)
    {
      state = 0;
      --rotaryCounter;
    }
    break;
  }
}

// function refreshes display
void printScreen()
{
  oled.setFont(fontList[fontIndexNumber]);
  oled.clear();
  oled.println(line0);
  oled.println(line1);
  oled.println(line2);
  oled.println(line3);
  printFlag = false;
}

// function to determine and calculate the duration of the pressed button (=timepressed)
void buttonPressedTimeCounter()
{
  if (buttonStatus == LOW)
  {
    timebuttonpress = millis();
  }
  if (buttonStatus == HIGH)
  {
    timebuttonrelease = millis();
    timepressed = timebuttonrelease - timebuttonpress;
  }
}

// functions to determine and calculate the duration of the fotoSensorValue being above or below the LightSensorThreshold value (returning timeAbove and timeBelow)
void fotoSensorValueBelowTimeCounter()
{
  if (fotoSensorValue <= LightSensorThreshold)
  {
    timeFotoLightSensorValueBelow1 = millis();
  }
  if (fotoSensorValue > LightSensorThreshold)
  {
    timeFotoLightSensorValueAbove1 = millis();
    timeBelow = timeFotoLightSensorValueAbove1 - timeFotoLightSensorValueBelow1;
  }
}

void fotoSensorValueAboveTimeCounter()
{
  if (fotoSensorValue > LightSensorThreshold)
  {
    timeFotoLightSensorValueAbove2 = millis();
  }
  if (fotoSensorValue <= LightSensorThreshold)
  {
    timeFotoLightSensorValueBelow2 = millis();
    timeAbove = timeFotoLightSensorValueBelow2 - timeFotoLightSensorValueAbove2;
  }
}

void ledsFlashing()
{
  // to make the led flashing these two functions (ledgreen.flash() and ledred.flash()) have to be looped permanently:
  flashgreen ? ledgreen.flash() : void();
  flashred ? ledred.flash() : void();
}

void meassureThresholdLEDoutput()
{
  // letting the LED flash dependending on the sensor threshold value is being crossed by the meassured value from the fotosensor
  if (fotoSensorValue > LightSensorThreshold)
  {
    while (prevLightSensorThreshold == false)
    {
      flashgreen = true;
      flashred = false;
      ledred.off();
      prevLightSensorThreshold = true;
    }
  }

  if (fotoSensorValue <= LightSensorThreshold)
  {
    while (prevLightSensorThreshold == true)
    {
      flashred = true;
      flashgreen = false;
      ledgreen.off();
      prevLightSensorThreshold = false;
    }
  }
}

void switchMenu()
{
  switch (menuItem)
  {
  case 0:                // Setting time Sunset
    if (oneloop == true) // is run only one loop
    {
      RotaryLightSensorThreshold = rotaryCounter;                                                                   // saving the rotary encoder position
      rotaryCounter = RotarytimeSunset;                                                                             // loading the next variable value
      LightSensorThreshold = map(RotaryLightSensorThreshold, 0, RTE_MAX_LIGHT_STEPS, 0, LIGHTSENSOR_THRESHOLD_MAX); // mapping the saved rotary encoder value to the actual sensor threshold value
      if (debug == true)
      {
        Serial.println("Sensor Threshold value =");
        Serial.println(LightSensorThreshold);
      }
      oneloop = false;
    }
    rotaryCounter < 0 ? rotaryCounter = 0 : rotaryCounter;                                   // prevents to count down to negative values
    rotaryCounter > RTE_MAX_TIME_STEPS ? rotaryCounter = RTE_MAX_TIME_STEPS : rotaryCounter; // sets a maximum value
    line0 = "Zeit Sonnenuntergang:";                                                         // text to show in display
    itoa(rotaryCounter, buffer, 10);                                                         // turns the integer into a displayable number
    line1 = buffer;                                                                          // contains the integer to be displayed
    line2 = "Minuten";
    rotaryCounter != prevRotCounter || buttonStatus != prevButtonStatus ? printFlag = true : printFlag = false; // only refresh display after the rotaryencoder has been turned or the button has been pressed
    break;
  case 1: // Setting time Sundrise
    if (oneloop == true)
    {
      RotarytimeSunset = rotaryCounter;
      rotaryCounter = RotarytimeSunrise;
      timeSunset = map(RotarytimeSunset, 0, RTE_MAX_TIME_STEPS, 0, TIME_MAX_RANGE);
      if (debug == true)
      {
        Serial.println("Sensor timeSunset value =");
        Serial.println(timeSunset);
      }
      oneloop = false;
    }
    rotaryCounter < 0 ? rotaryCounter = 0 : rotaryCounter;
    rotaryCounter > RTE_MAX_TIME_STEPS ? rotaryCounter = RTE_MAX_TIME_STEPS : rotaryCounter;
    line0 = "Zeit Sonnenaufgang:";
    itoa(rotaryCounter, buffer, 10);
    line1 = buffer;
    line2 = "Minuten";
    rotaryCounter != prevRotCounter || buttonStatus != prevButtonStatus ? printFlag = true : printFlag = false;
    break;
  case 2: // Setting Fotosensor Threshold
    if (oneloop == true)
    {
      RotarytimeSunrise = rotaryCounter;
      rotaryCounter = RotaryLightSensorThreshold;
      timeSunrise = map(RotarytimeSunrise, 0, RTE_MAX_TIME_STEPS, 0, TIME_MAX_RANGE);
      if (debug == true)
      {
        Serial.println("Sensor timeSunrise value =");
        Serial.println(timeSunrise);
      }
      oneloop = false;
    }
    rotaryCounter < 0 ? rotaryCounter = 0 : rotaryCounter;
    rotaryCounter > RTE_MAX_LIGHT_STEPS ? rotaryCounter = RTE_MAX_LIGHT_STEPS : rotaryCounter;
    line0 = "Foto Sensor:";
    itoa((rotaryCounter * RTE_MULTIPLIER), buffer, 10);
    line1 = buffer;
    line2 = "%";
    rotaryCounter != prevRotCounter || buttonStatus != prevButtonStatus ? printFlag = true : printFlag = false;
    break;
  default:
    break;
  }
}

void evaluateAndDecide()
{
  switch (caseNumber)
  {
  case 0: // evaluating state
    if (open == false && timeBelow >= FOTOSENSOR_TIME_BELOW_ABOVE)
    {
      caseNumber = 1; // opening
      timeAbove = 0;
      flashgreen = true;
      flashred = false;
      ledred.off();
      previousMillis = currentMillis;
    }
    if (open == true && timeAbove >= FOTOSENSOR_TIME_BELOW_ABOVE)
    {
      caseNumber = 2; // closing
      timeBelow = 0;
      flashred = true;
      flashgreen = false;
      ledgreen.off();
      previousMillis = currentMillis;
    }
    break;
  case 1: // time waiting to open
    if (millis() - previousMillis >= timeSunrise)
      caseNumber = 3;
    break;
  case 2: // time waiting to close
    if (millis() - previousMillis >= timeSunset)
      caseNumber = 4;
    break;
  case 3: // start opening procedure
    if (debug == true)
    {
      Serial.println("opening, bool open = true");
    }
    line3 = "oeffnend";
    printFlag = true;
    oneloop = true;
    oneloop1 = true;
    open = true;
    servolock.Attach(SERVOLOCK_PIN); // attaching the servos, turning the PWM signals on
    servohack.Attach(SERVOHACK_PIN);
    caseNumber = 0; // Value has changed, back to init state
    break;
  case 4: // start closing procedure
    if (debug == true)
    {
      Serial.println("closing, bool open = false");
    }
    line3 = "schliessend";
    printFlag = true;
    oneloop = true;
    oneloop2 = true;
    open = false;
    servolock.Attach(SERVOLOCK_PIN);
    servohack.Attach(SERVOHACK_PIN);
    caseNumber = 0; // Value has changed, back to init state
    break;
  }
}

void openingProcedure()
{
  servolock.MovePos(SERVOLOCK_MIN);   // Servolock moves the open position
  if (servolock.pos == SERVOLOCK_MIN) // wenn the open position is reached, means the doorlock is fully open/unlocked
  {
    if (reedStatusOpen == HIGH)
    {
      servohack.MovePos(SERVOHACK_OPEN); // as long a the reed is high, the servo is motor turning in the open direction
    }
    if (reedStatusOpen == LOW)
    {
      servohack.MovePos(SERVOHACK_STOP); // as soon as the reed contact switches low, the door is open, the servomotor stops turning

      if (servohack.pos == SERVOHACK_STOP)
      {
        servolock.Detach(); // the door is open and the servomotor has stopped, both servos are disconnected
        servohack.Detach();

        if (oneloop1 == true) // info is send to LEDS and OLED display, one loop
        {
          ledgreen.on();
          ledred.off();
          flashgreen = false;
          flashred = false;
          line3 = "TOR OFFEN";
          printFlag = true;
          oneloop1 = false;
          openCompleted = true;
          closeCompleted = false;
        }
      }
    }
  }
}

void closingProcedure()
{
  if (reedStatusClose == HIGH)
  {
    servohack.MovePos(SERVOHACK_CLOSE); // as long as the reed close is high, the servomotor is turning in the closing direction
  }
  if (reedStatusClose == LOW)
  {
    servohack.MovePos(SERVOHACK_STOP);   // is the door closed, the reed close is turned to low, the motor stops running
    if (servohack.pos == SERVOHACK_STOP) // has the motor been stopped
    {
      servolock.MovePos(SERVOLOCK_MAX); // the lock servo is moving into lock posisition and locking the door

      if (servolock.pos == SERVOLOCK_MAX)
      {
        servolock.Detach(); // after the lock has moved into place, the servos are detached
        servohack.Detach();

        if (oneloop2 == true) // info is send to LEDS and OLED display, one loop
        {
          ledred.on();
          ledgreen.off();
          flashgreen = false;
          flashred = false;
          line3 = "TOR GESCHLOSSEN";
          printFlag = true;
          oneloop2 = false;
          closeCompleted = true;
          openCompleted = false;
        }
      }
    }
  }
}

void checkingDoorway()
{
  if (gateSensorValue < GATESENSOR_THRESHOLD) // change if just one gate sensor is installed
  // if (gateSensorValue > GATESENSOR_THRESHOLD || secondgateSensorValue > GATESENSOR_THRESHOLD) // If one of both gate sensors are sending a negative signal (the light path is blocked)
  {
    doorway = false; // doorway blocked

    (reedStatusOpen == HIGH && servolock.pos == SERVOLOCK_MIN) ? servohack.MovePos(SERVOHACK_OPEN) : servohack.MovePos(SERVOHACK_STOP); // open door to free entrance, only working while closing procedure is activ!

    ledgreen.flash();
    ledred.flash();

    while (prevGateSensorThreshold == true) // one loop to refresh display
    {
      // line3B = line3; // saving last message and led status before entrance was blocked
      line3 = "DG BLOCKIERT!";
      printFlag = true;
      prevGateSensorThreshold = false;
    }
  }

  if (gateSensorValue >= GATESENSOR_THRESHOLD) // change if just one gate sensor is installed
  // if (gateSensorValue <= GATESENSOR_THRESHOLD && secondgateSensorValue <= GATESENSOR_THRESHOLD) // both sensors have to give a positive signal (the light path is free)
  {
    doorway = true; // doorway free

    while (prevGateSensorThreshold == false) // one loop to refresh display
    {
      // closeCompleted ? ledred.on() : ledred.off(); // resetting the original LED status before the door was blocked
      // openCompleted ? ledgreen.on() : ledgreen.off();
      // line3 = line3B; // puts the orginal message and the original led status after doorway is set free back into the display and led modus
      open == true &&openCompleted == false &&reedStatusClose &&reedStatusOpen ? line3 = "oeffnend" : NULL;
      open == false &&closeCompleted == false &&reedStatusClose &&reedStatusOpen ? line3 = "schliessend" : NULL;
      open == true &&openCompleted == true ? line3 = "TOR OFFEN", ledgreen.on() : ledgreen.off();
      open == false &&closeCompleted == true ? line3 = "TOR GESCHLOSSEN", ledred.on() : ledred.off();
      printFlag = true;
      prevGateSensorThreshold = true;
    }
  }
}

// initializing after turning the system on
bool calibration()
{
  if (oneloop3 == true)
  {
    servolock.Attach(SERVOLOCK_PIN);
    servohack.Attach(SERVOHACK_PIN);
    line3 = "calibrieren...";
    printFlag = true;
    oneloop3 = false;
  }

  servolock.MovePos(SERVOLOCK_MIN); // moving the lock servo to the open position
  if (servolock.pos == SERVOLOCK_MIN)
  {
    if (reedStatusOpen == HIGH)
    {
      servohack.MovePos(SERVOHACK_OPEN); // as long as the reed close is high, the servomotor is turning in the opening direction
    }
    if (reedStatusOpen == LOW) // has the door been opened, servohack has to stop, put to 90 position
    {
      servohack.MovePos(SERVOHACK_STOP);

      if (servohack.pos == SERVOHACK_STOP) // has the door been opened and lock been opened?
      {
        servohack.Detach();
        servolock.Detach();
        line3 = "Calib beendet";
        printFlag = true;
        gateSensorValue >= GATESENSOR_THRESHOLD ? prevGateSensorThreshold = false : NULL; // setting bool values depending on current sample values
        gateSensorValue < GATESENSOR_THRESHOLD ? prevGateSensorThreshold = true : NULL;
        fotoSensorValue > LightSensorThreshold ? open = false, prevLightSensorThreshold = false, closeCompleted = true : NULL;
        fotoSensorValue <= LightSensorThreshold ? open = true, prevLightSensorThreshold = true, openCompleted = true : NULL;
        calibrationCompleted = true; // quitting calibration and going to operation mode
      }
    }
  }
  return calibrationCompleted;
}

// first loop
void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);

  // setting all the initial values:
  servolock.Detach();
  servohack.Detach();
  ledgreen.off();
  ledred.off();
  previousMillis = 0;
  caseNumber = 0;
  open = false;
  doorway = true;
  printFlag = false;
  menuItem = 0;
  oneloop = false;
  oneloop1 = false;
  oneloop2 = false;
  oneloop3 = true;
  openMenu = false;
  prevLightSensorThreshold = false;
  flashgreen = false;
  flashred = false;
  openCompleted = false;
  closeCompleted = false;
  calibrationCompleted = false;

  // Initialize encoder pins
  pinMode(ENCODER_CLK, INPUT_PULLUP);
  pinMode(ENCODER_DT, INPUT_PULLUP);
  pinMode(ENCODER_SW, INPUT_PULLUP);

  // Set OLED Display
  Wire.begin();
  Wire.setClock(400000L);
#if RST_PIN >= 0
  oled.begin(&Adafruit128x64, I2C_ADDRESS, RST_PIN);
#else  // RST_PIN >= 0
  oled.begin(&Adafruit128x64, I2C_ADDRESS);
#endif // RST_PIN >= 0
  line0 = NULL;
  line1 = NULL;
  line2 = NULL;
  line3 = NULL;
  line3B = line3;
  printFlag = true;
}

// continuous loop
void loop()
{
  // putting in; reading status, sample and hold:
  lightfotosensor.messure();              // reading daylight fotosensor
  gatefotosensor.messure();               // reading gate fotosensor
  secondgatefotosensor.messure();         // reading second gate fotosensor
  readEncoder();                          // reading rotary encoder turning steps and direction
  buttonStatus = digitalRead(ENCODER_SW); // reading the status of the RTE button

  fotoSensorValue = lightfotosensor.messuredValue;            // sample and hold the fotosensor meassured value
  gateSensorValue = gatefotosensor.messuredValue;             // sample and hold the gatesensor meassured value
  secondgateSensorValue = secondgatefotosensor.messuredValue; // sample and hold the second gatesensor meassured value
  reedStatusClose = reedsensorClose.messure();                // sample and hold the reed status
  reedStatusOpen = reedsensorOpen.messure();
  currentMillis = millis(); // sample and hold the the current time, time stamp

  calibrationCompleted == false ? calibration() : calibrationCompleted; // system is first calibrated to start position: door is closed and lock is opened

  // putting out signals:
  tone(LEDIR_PIN, 38000); // setting a 38 kHz signal on pin for the emitting IR LED , needed for the photoelectric gate, resistor necessary!
  ledsFlashing();         // make the leds flashing
  // meassureThresholdLEDoutput(); // making the leds flashing depending on the cross direction of the lightsensorthreshold value

  // setting one loop
  oneloop = false; // resets the oneloop boolian value

  // only display refresh if: printFlag = true, this avoids flickering
  if (printFlag)
    printScreen();

  // function to determine how long the rotary button is being pressed saving that time in the variable "timepressed":
  if (buttonStatus != prevButtonStatus)
    buttonPressedTimeCounter();

  // opens setup menu on oled display after the button is pressed longer than ca 3000 mS
  if (buttonStatus == HIGH && prevButtonStatus == LOW && timepressed > BUTTON_PRESSED_TIME_MAX)
  {
    openMenu = !openMenu; // toggles the menu opening/closing
    oneloop = true;       // setting the oneloop
    if (openMenu == true)
    {
      menuItem = 0;                               // sets the first menu to open
      rotaryCounter = RotaryLightSensorThreshold; // setting the rotary counter to the saved value of the first menu item, in this case the saved lightsensor threshold in the RAM
      servolock.Detach();                         // preventing servos from moving when menu is open
      line3B = line3;                             // saving last message and led status before menu is being opened
      line3 = "System ist inaktiv";
    }
    if (openMenu == false)
    {
      servolock.Attach(SERVOLOCK_PIN);                                                                          // reconnect lockservo after leaving settings menu
      oneloop == true ? line0 = NULL, line1 = NULL, line2 = NULL, line3 = line3B, printFlag = true : printFlag; // cleans display and returns message in line3 after exiting the menu
    }
  }

  // loops through the menus after short (= between ca 50 and 2000 mS) pressing the rotary button
  if (buttonStatus == LOW && prevButtonStatus == HIGH && timepressed > BUTTON_PRESSED_TIME_MIN && timepressed < BUTTON_PRESSED_TIME_BETWEEN)
  {
    menuItem++;                             // stepping through the menus
    menuItem > 2 ? menuItem = 0 : menuItem; // max 3 menus are determined: 0 1 2.
    oneloop = true;                         // to set one loop
  }

  // the 3 menus to define the parameters from user input by the rotary encoder, it outputs: LightSensorThreshold (%), timeSunrise, timeSunset (Minutes)
  if (openMenu == true)
  {
    servohack.MovePos(SERVOHACK_STOP); // stops hackservo from moving wenn settings menu is open
    switchMenu();                      // looping through the menu settings and setting the variables
  }

  if (openMenu == false && calibrationCompleted == true)
  {
    // checking how long the fotosensor value is below or above the threshold value
    fotoSensorValueAboveTimeCounter();
    fotoSensorValueBelowTimeCounter();

    // evaluating the duration of the messured sensor values and making decisions what action should be taken:
    evaluateAndDecide();

    // checking if the entrance is free or blocked
    checkingDoorway();

    // Servos opening procedure
    if (open == true)
    {
      openingProcedure();
    }
    // Servos closing procedure
    if (open == false && doorway == true)
    {
      closingProcedure();
    }
  }

  // necessary to check if the status of the button and rotary encoder have changed between two loops
  prevButtonStatus = buttonStatus;
  prevRotCounter = rotaryCounter;

  // for debugging:
  if (debug == true)
  {
    Serial.println("Fotosensor value =");
    Serial.println(fotoSensorValue);
    Serial.println("Timeabove =");
    Serial.println(timeAbove);
    Serial.println("Timebelow =");
    Serial.println(timeBelow);
    // Serial.println("Gatesensor value =");
    // Serial.println(gateSensorValue);
    //
  }
}