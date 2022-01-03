/*
 * Program urine detector AS7262
 * 
 * author: Mas Sagita 2020
 */


#include <OneWire.h>
#include <AverageValue.h>

#include <dht11.h>
#include <Wire.h>
#define DHT11_PIN     A1
dht11 DHT;

#define ONE_WIRE_BUS  A0  //Pin data DS18B20 1
OneWire ds(ONE_WIRE_BUS);

#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);

#include "Adafruit_AS726x.h"
Adafruit_AS726x ams;

uint16_t sensorValues[AS726x_NUM_CHANNELS];

//Rotary Encoder PIN
#define pinRotaryEncoderCLK     2
#define pinRotaryEncoderDT      3
#define pinRotaryEncoderSwitch  4

//Variables for Rotary Encoder
const int buttonPin = pinRotaryEncoderSwitch; // the number of the pushbutton pin
const int SHORT_PRESS_TIME = 800; // 1000 milliseconds
const int LONG_PRESS_TIME  = 800; // 1000 milliseconds

//Variables for switch Rotary Encoder
int lastState = LOW;  // the previous state from the input pin
int currentState;     // the current reading from the input pin
unsigned long pressedTime  = 0;
unsigned long releasedTime = 0;
bool isPressing = false;
bool isLongDetected = false;
bool longPressBtn = false;
bool shortPressBtn = false;
bool isHeating = false;
long pressDuration;

//Variables Rotary Switch will change:
int ledState = HIGH;         // the current state of the output pin
int buttonState;             // the current reading from the input pin
int lastButtonState = LOW;   // the previous reading from the input pin
// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

//Variables for set interrupt encoder
uint8_t maskSensorA;
uint8_t maskSensorB;
uint8_t *pinSensorA;
uint8_t *pinSensorB;
volatile bool encoderAFlag = 0;
volatile bool encoderBFlag = 0;
int8_t nilaiEncoder = 0;
int nilaiSetting[4];
int countEcd;
String currentDir = "";

float dsVal; //Store ds18b20 value
float dht11Val; //Store DHT11 value

const int BUZZER_PIN = 10; //Buzzer pin
const int ledPin = 13; //Led pin

const int relayPin[] = {6, 7};
const int buttonStart = 8;

int buttonStartState;

//Variables for menu
int countSend;
int manualScreen;
int countDisplay = 0;
int menu = 0;
int cursorSet = 0;
int cursorState;
//Variables for menu
static bool isMenu = false;
static bool isPhSetup = false;
static bool isScreenSetup = false;
static bool isPressSetup = false;
static bool isManualScreen = false;
static bool isJsonSend = false;
static bool isLimitHeater = false;
static bool isModeFan = false;
//Variables for menu
String screenSetup = "";
String jsonStatus = "";
String modeFan = "";

int refresh = 0; //For refresh screen

float limitHeater = 40;

uint8_t tempAS;
bool rdy;
int tempSensorWarna;

float calibratedValues[AS726x_NUM_CHANNELS];

bool colorRead;

float dallas(OneWire& ds, byte start = false) {
  int16_t temp;
  do {
    ds.reset();
    ds.write(0xCC);
    ds.write(0xBE);
    ds.read_bytes((uint8_t*) &temp, sizeof(temp));
    ds.reset();
    ds.write(0xCC);
    ds.write(0x44, 1);
    if (start) delay(1000);
  } while (start--);
  return (temp * 0.0625);
}

void setup() {
  // Initialize "debug" serial port
  // The data rate must be much higher than the "link" serial port
  Serial.begin(115200);
  lcd.init();

  Serial.println("-------BGV1-------");
  dallas(ds, true);

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(buttonStart, INPUT_PULLUP);

  pinMode(pinRotaryEncoderCLK, INPUT_PULLUP);
  pinMode(pinRotaryEncoderDT, INPUT_PULLUP);
  pinMode(pinRotaryEncoderSwitch, INPUT_PULLUP);

  for (int i = 0; i < 3; i++) {
    pinMode(relayPin[i], OUTPUT);
  }

  relay(0, 1);
  relay(1, 1);

  attachInterrupt(digitalPinToInterrupt(pinRotaryEncoderCLK), encoderARising, RISING);
  attachInterrupt(digitalPinToInterrupt(pinRotaryEncoderDT), encoderBRising, RISING);

  maskSensorA  = digitalPinToBitMask(pinRotaryEncoderCLK);
  pinSensorA = portInputRegister(digitalPinToPort(pinRotaryEncoderCLK));
  maskSensorB  = digitalPinToBitMask(pinRotaryEncoderDT);
  pinSensorB = portInputRegister(digitalPinToPort(pinRotaryEncoderDT));

  for (int i = 0; i < 2; i++) {
    tone(BUZZER_PIN, 2394, 125); //2394
    digitalWrite(ledPin, 1);
    delay(130);
    tone(BUZZER_PIN, 3136, 125); //3136
    digitalWrite(ledPin , 0);
    delay(125);
  }

  manualScreen = 0;
  screenSetup = "Manual"; // start screen manual scroll
  jsonStatus = "ON";
  modeFan = "auto"; // start fan mode manual

  longPressBtn = true;
  
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("NANO AS7262"));
  lcd.setCursor(0, 1);
  lcd.print(F("Trial Version"));
  delay(1000);

  if (!ams.begin()) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("AS7262 OFF"));
    lcd.setCursor(0, 1);
    lcd.print(F("CHECK WIRING"));
    while (1);
  }
  ams.setConversionType(MODE_2);
}

void loop() {

  digitalWrite(BUZZER_PIN, HIGH);

  isMenu = false;
  isPhSetup = false;
  isScreenSetup = false;
  isPressSetup = false;
  isJsonSend = false;
  isHeating = false;
  isModeFan = false;

  getRotaryBtn();
  readColorSensor();

  hitungSuhuDS();
  hitungDHT11();

  buttonStartState = digitalRead(buttonStart);

  if (screenSetup == "Auto") {
    isManualScreen = false;
    if (isManualScreen == false) {
      countDisplay++;
      manualScreen = 10;
      if (++refresh > 1) {
        lcd.clear();
        refresh = 0;
      }
      lcd.setCursor(15, 0);
      lcd.print(F("A"));
      lcd.setCursor(15, 1);
      lcd.print(buttonStartState);
      displayScreen();
    }
  }

  if (screenSetup == "Manual") {
    isManualScreen = true;
    if (++refresh > 1) {
      lcd.clear();
      refresh = 0;
    }
    lcd.setCursor(15, 0);
    lcd.print(F("M"));

    lcd.setCursor(15, 1);
    lcd.print(buttonStartState);
    countDisplay = -1;
    if (manualScreen > 4) manualScreen = 0;
    if (manualScreen < 0) manualScreen = 4;
    displayScreen();
  }

  if (shortPressBtn) {
    displayMenu();
    countDisplay = 0;
  }

  Serial.println((String) countSend + "\t" + screenSetup + "\t"
                 + countDisplay + "\t" + manualScreen);
}

void readColorSensor() {
  if (colorRead) {
    tempAS = ams.readTemperature();
    ams.startMeasurement();

    rdy = false;
    while (!rdy) {
      rdy = ams.dataReady();
    }
    ams.readCalibratedValues(calibratedValues);
    //ams.readRawValues(sensorValues);
  }
  if (!colorRead){
    
  }
}
int rel0 = 1, rel1 = 1;
int statusRelay;

void hitungSuhuDS() {
  dsVal = dallas(ds);
  if (buttonStartState == 1) {
    isHeating = true;
  }

  if (isHeating) {
    screenSetup = "heating";
    rel0 = 0;
  }

  if (dsVal > limitHeater) {
    screenSetup = "scanning";
    rel0 = 1;
  }

  if (screenSetup == "scanning") {
    lcd.clear();
    lcd.print(F("SCANNING"));
    colorRead = true;
  }

  if (screenSetup == "heating") {
    colorRead = false;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("HEATING:"));
    lcd.print(dsVal); lcd.print((char)223); lcd.print("C");

    lcd.setCursor(0, 1);
    lcd.print("set:"); lcd.print((int)limitHeater);
    lcd.print(" fan:"); lcd.print(modeFan);
  }

  if (modeFan == "manual") rel1 = 0;
  if (modeFan == "auto") {
    if (dsVal > 35) rel1 = 0;
    else rel1 = 1;
  }

  //  lcd.setCursor(13, 1);
  //  lcd.print(isHeating);

  relay(0, rel0);
  relay(1, rel1);
}

void hitungDHT11() {
  dht11Val = DHT.read(DHT11_PIN);
}

void displayScreen() {
  //display the screen according to condition manual or auto
  //for setting find in menu
  if (countDisplay >= 0 && countDisplay <= 9 || manualScreen == 0) {
    ams.drvOff();
    lcd.setCursor(0, 0); lcd.print(F("DS18B20:"));
    lcd.setCursor(0, 1); lcd.print(F("T:")); lcd.print(dsVal);
    lcd.print((char)223); lcd.print("C");
  }

  if (countDisplay >= 10 && countDisplay <= 20 || manualScreen == 1) {
    ams.drvOff();
    lcd.setCursor(0, 0); lcd.print(F("BiPo_DHT11:"));
    lcd.setCursor(0, 1);
    lcd.print(F("T:")); lcd.print(DHT.temperature, 1);
    lcd.print(F(" H:"));  lcd.print(DHT.humidity, 1);  lcd.print(F("%"));
  }

  if (countDisplay >= 21 && countDisplay <= 30 || manualScreen == 2) {
    lcd.setCursor(0, 1);
    lcd.print(F("V:")); lcd.print(sensorValues[AS726x_VIOLET]);
    lcd.setCursor(6, 1);
    lcd.print(F("B:")); lcd.print(sensorValues[AS726x_BLUE]);
    tempSensorWarna = 1;
  }

  if (countDisplay >= 31 && countDisplay <= 40 || manualScreen == 3) {
    lcd.setCursor(0, 1);
    lcd.print(F("H:")); lcd.print(sensorValues[AS726x_GREEN]);
    lcd.setCursor(6, 1);
    lcd.print(F("K:")); lcd.print(sensorValues[AS726x_YELLOW]);
    tempSensorWarna = 1;
  }

  if (countDisplay >= 41 && countDisplay <= 50 || manualScreen == 4) {    
    lcd.setCursor(0, 1);
    lcd.print(F("O:")); lcd.print(sensorValues[AS726x_ORANGE]);
    lcd.setCursor(6, 1);
    lcd.print(F("M:")); lcd.print(sensorValues[AS726x_RED]);
    tempSensorWarna = 1;
  }

  if (tempSensorWarna == 1) {
    colorRead = true;
    ams.drvOn();
    lcd.setCursor(0, 0);
    lcd.print("AS7262 "); lcd.print("T:");
    lcd.print(tempAS);
    lcd.print((char)223); lcd.print("C");
  }

  if (countDisplay > 50) countDisplay = 0; tempSensorWarna = 0;
}

void displayMenu() {
  isMenu = true;
  isPhSetup = false;
  isScreenSetup = false;
  isPressSetup = false;
  isJsonSend = false;
  isManualScreen = false;
  isModeFan = false;
  menu = 0;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Setup Screen"));

  tone(BUZZER_PIN, 3136, 125); //3136
  digitalWrite(ledPin, 1);
  delay(200);
  tone(BUZZER_PIN, 2394, 125); //2394
  digitalWrite(ledPin , 0);
  delay(300);

  delay(500);

  while (1) {
    readColorSensor();
    getRotaryBtn();
    if (ledState == 0) {
      isMenu = false;
      cursorSet = 9;
    }
    if (ledState == 1) {
      isMenu = true;
      isPhSetup = false;
      isScreenSetup = false;
      isPressSetup = false;
      isJsonSend = false;
      isLimitHeater = false;
      isModeFan = false;
      cursorSet = 0;
    }

    if (++refresh > 5) {
      lcd.clear();
      refresh = 0;
    }
    //Serial.println(menu);

    //limit menu
    if (menu > 6) menu = 0;
    if (menu < 0) menu = 6;

    if (menu == 0) {
      tempSensorWarna = 0;
      isScreenSetup = true;
      isPhSetup = false;
      isPressSetup = false;
      isJsonSend = false;
      isLimitHeater = false;
      isModeFan = false;

      lcd.setCursor(0, 0);
      lcd.print(menu); lcd.print(F(".Screen Setup"));

      lcd.setCursor(cursorSet, 1);
      lcd.print(F(">"));
      lcd.setCursor(1, 1);
      lcd.print(F("Scroll"));
      lcd.setCursor(10, 1);
      lcd.print(screenSetup);
    }

    if (menu == 1) {
      tempSensorWarna = 0;
      isScreenSetup = false;
      isPhSetup = true;
      isPressSetup = false;
      isJsonSend = false;
      isLimitHeater = true;
      isModeFan = false;

      lcd.setCursor(0, 0);
      lcd.print(menu); lcd.print(F(".Limit Heater"));

      lcd.setCursor(cursorSet, 1);
      lcd.print(F(">"));
      lcd.setCursor(1, 1);
      lcd.print("calVal:");
      lcd.setCursor(10, 1);
      lcd.print(limitHeater);
    }

    if (menu == 2) {
      tempSensorWarna = 0;
      isScreenSetup = false;
      isPhSetup = false;
      isPressSetup = false;
      isJsonSend = false;
      isLimitHeater = false;
      isModeFan = true;

      lcd.setCursor(0, 0);
      lcd.print(menu); lcd.print(F(".Fan Mode"));

      lcd.setCursor(cursorSet, 1);
      lcd.print(F(">"));
      lcd.setCursor(1, 1);
      lcd.print("mode:");
      lcd.setCursor(10, 1);
      lcd.print(modeFan);
    }

    if (menu == 3) {
      tempSensorWarna = 1;
      lcd.setCursor(0, 1);
      lcd.print(F("V:")); lcd.print(sensorValues[AS726x_VIOLET]);
      lcd.setCursor(6, 1);
      lcd.print(F("B:")); lcd.print(sensorValues[AS726x_BLUE]);
    }

    if (menu == 4) {
      tempSensorWarna = 1;
      lcd.setCursor(0, 1);
      lcd.print(F("H:")); lcd.print(sensorValues[AS726x_GREEN]);
      lcd.setCursor(6, 1);
      lcd.print(F("K:")); lcd.print(sensorValues[AS726x_YELLOW]);
    }

    if (menu == 5) {
      tempSensorWarna = 1;
      lcd.setCursor(0, 1);
      lcd.print(F("O:")); lcd.print(sensorValues[AS726x_ORANGE]);
      lcd.setCursor(6, 1);
      lcd.print(F("M:")); lcd.print(sensorValues[AS726x_RED]);
    }

    if (tempSensorWarna == 1) {
      ams.drvOn();
      colorRead = true;
      lcd.setCursor(0, 0);
      lcd.print(menu); lcd.print(".S-AS7262 ");
      lcd.print("T:"); lcd.print(tempAS);
    }

    if (menu == 6) {
      lcd.setCursor(0, 0);
      lcd.print(menu); lcd.print(F(".versions"));
      lcd.setCursor(0, 1);
      lcd.print("AS7262 V0.1");
    }

    if(tempSensorWarna == 0){
      ams.drvOff();
      colorRead = false;
    }

    //need eeprom to save setting value
    if (longPressBtn) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Saving..");
      tone(BUZZER_PIN, 2394, 125); //2394
      digitalWrite(ledPin, 1);
      delay(200);
      tone(BUZZER_PIN, 3136, 125); //3136
      digitalWrite(ledPin , 0);
      delay(300);
      delay(500);
      break;
    }
  }
}

void relay(int ch, bool on) {
  digitalWrite(relayPin[ch], on);
}

void getRotaryBtn() {
  currentState = digitalRead(buttonPin);

  // If the switch changed, due to noise or pressing:
  if (currentState != lastButtonState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (currentState != buttonState) {
      buttonState = currentState;

      // only toggle the LED if the new button state is HIGH
      if (buttonState == HIGH) {
        ledState = !ledState;
      }
    }
  }

  lastButtonState = currentState;

  if (lastState == HIGH && currentState == LOW) {       // button is pressed
    pressedTime = millis();
    isPressing = true;
    isLongDetected = false;
  } else if (lastState == LOW && currentState == HIGH) { // button is released
    isPressing = false;
    releasedTime = millis();

    pressDuration = releasedTime - pressedTime;

    if ( pressDuration < SHORT_PRESS_TIME ) {
      longPressBtn = false;
      shortPressBtn = true;
      Serial.print((String)"A short press is detected \t");
      Serial.println((String) longPressBtn + shortPressBtn);
    }
  }

  if (isPressing == true && isLongDetected == false) {
    pressDuration = millis() - pressedTime;

    if ( pressDuration > LONG_PRESS_TIME ) {
      longPressBtn = true;
      shortPressBtn = false;
      Serial.print((String)"A long press is detected \t");
      Serial.println((String) longPressBtn + shortPressBtn);
      isLongDetected = true;
    }
  }
  digitalWrite(ledPin, isLongDetected);
  // save the the last state
  lastState = currentState;
}

void encoderARising() {
  if ((*pinSensorA & maskSensorA) &&  (*pinSensorB & maskSensorB) && encoderAFlag)
  {
    nilaiEncoder = -1;
    countEcd --;
    currentDir = "CCW";
    if (isMenu)menu --;
    else if (isManualScreen)manualScreen--;
    else if (isScreenSetup)screenSetup = "Manual";
    else if (isJsonSend)jsonStatus = "OFF";
    else if (isLimitHeater) limitHeater -= 0.5;
    else if (isModeFan) modeFan = "auto";
    encoderAFlag = false;
    encoderBFlag = false;
  }
  else if (*pinSensorA & maskSensorA)
  {
    encoderBFlag = true;
  }
  EIFR = 0xFF;
}

void encoderBRising() {
  if ((*pinSensorA & maskSensorA) &&  (*pinSensorB & maskSensorB) && encoderBFlag)
  {
    nilaiEncoder = 1;
    countEcd ++;
    currentDir = "CW";
    if (isMenu)menu ++;
    else if (isManualScreen)manualScreen++;
    else if (isScreenSetup)screenSetup = "Auto";
    else if (isJsonSend)jsonStatus = "ON";
    else if (isLimitHeater) limitHeater += 0.5;
    else if (isModeFan) modeFan = "manual";
    encoderAFlag = false;
    encoderBFlag = false;
  }
  else if (*pinSensorB & maskSensorB)
  {
    encoderAFlag = true;
  }
  EIFR = 0xFF;
}

