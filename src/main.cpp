#include "Printable.h"
#include "esp32-hal-gpio.h"
#include "esp32-hal-ledc.h"
#include "esp32-hal.h"
#include <Arduino.h>

#include <WiFi.h>
#include <AsyncTCP.h>

#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// #include <AiEsp32RotaryEncoder.h>

#include <OneWire.h>
#include <DallasTemperature.h>

#include <Adafruit_MAX31865.h>
#include <cstdint>

// I2C 2004 display, default pins
#define PIN_I2C_SCL 22
#define PIN_I2C_SDA 21

// SPI MAX31865 PT100 amplifier, default SPI pins
#define PIN_SPI_CS 5
#define PIN_SPI_SCK 18
#define PIN_SPI_MISO 19
#define PIN_SPI_MOSI 23

// Rotary encoder
#define PIN_ROTARY_ENCODER_CLK 17
#define PIN_ROTARY_ENCODER_DATA 16
#define PIN_ROTARY_ENCODER_BUTTON 4

// Relays
#define PIN_HEATER_300 26
#define PIN_HEATER_800 25
#define PIN_HEATER_1500 33
#define PIN_PUMP 32

// Pump PWM, with 4.7k pullup to 5V
#define PIN_PUMP_PWM 13

// DS18B20 temperature sensors, with 4.7k pullup to 3.3V
#define PIN_ONEWIRE 15

#define PIN_FLOW_SENSOR_SOLAR 14
#define PIN_FLOW_SENSOR_WATER 27

#define relayOn(pin)  digitalWrite(pin, LOW)
#define relayOff(pin) digitalWrite(pin, HIGH)

/*
Pump PWM mode 1:
   0     No PWM signal, auto mode
   1-25  Maximum speed
  26-214 Decreasing speed
 215-232 Lowest speed
 233-242 Hysteresis, pump starts in intervals to avoid clocking?
 243-255 Pump stops
*/

void setPumpSpeed(int speed) {
  if (speed <= 0) {
    relayOff(PIN_PUMP);
    analogWrite(PIN_PUMP_PWM, 255);
  } else {
    relayOn(PIN_PUMP);
    if (speed > 100) {
      speed = 100;
    }
    analogWrite(PIN_PUMP_PWM, 220 - 2 * speed);
  }
}


const char* ssid = "Brouwketel";
const char* password = "levedromen";

LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display

// AiEsp32RotaryEncoder rotaryEncoder = AiEsp32RotaryEncoder(PIN_ROTARY_ENCODER_A, PIN_ROTARY_ENCODER_B, PIN_ROTARY_ENCODER_BUTTON, -1, 4);

OneWire oneWire(PIN_ONEWIRE);        // Set up a OneWire instance to communicate with OneWire devices
DallasTemperature tempSensors(&oneWire); // Create an instance of the temperature sensor class
int DS_delay = 750;

// Assign address manually. The addresses below will need to be changed
// to valid device addresses on your bus. Device address can be retrieved
// by using either oneWire.search(deviceAddress) or individually via
// sensors.getAddress(deviceAddress, index)
// DeviceAddress insideThermometer = { 0x28, 0x1D, 0x39, 0x31, 0x2, 0x0, 0x0, 0xF0 };
// DeviceAddress outsideThermometer   = { 0x28, 0x3F, 0x1C, 0x31, 0x2, 0x0, 0x0, 0x2 };
struct {
  DeviceAddress boilerTop;
  DeviceAddress boilerMiddle;
  DeviceAddress boilerBottom;
  DeviceAddress solarFrom;
  DeviceAddress solarTo;
  DeviceAddress tapWater;
} thermo = {
  .boilerTop    = {},
  .boilerMiddle = {0x28, 0xA3, 0x78, 0x79, 0x97, 0x08, 0x03, 0xB5},
  .boilerBottom = {},
  .solarFrom    = {},
  .solarTo      = {},
  .tapWater     = {},
};

// use hardware SPI, just pass in the CS pin
Adafruit_MAX31865 pt100 = Adafruit_MAX31865(PIN_SPI_CS);
// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF      430.0
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL  100.0
int RTDoffset = 0;

#define BUTTON_DEBOUNCE 250

struct {
  bool buttonPressed;
  long int timeLastPress;
  bool encoderChanged;
  int encoder;
} input;

struct {
  float tSolar;
  float tSolarFrom;
  float tSolarTo;
  float tBoilerTop;
  float tBoilerMiddle;
  float tBoilerBottom;
  volatile float flowWater;
  volatile float flowSolar;
} sensors;

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
AsyncEventSource events("/events");

// void IRAM_ATTR readEncoderISR() {
// 	rotaryEncoder.readEncoder_ISR();
// }

// void rotary_loop() {
// 	//dont print anything unless value changed
// 	if (rotaryEncoder.encoderChanged()) 	{
// 		Serial.print("Value: ");
// 		Serial.println(rotaryEncoder.readEncoder());
// 	}
// 	if (rotaryEncoder.isEncoderButtonClicked()) {
//     Serial.println("Button pressed");
// 	}
// }
// function to print a device address

void printAddress(int index)
{
  DeviceAddress deviceAddress;
  tempSensors.getAddress(deviceAddress, index);
  Serial.print("{");
  for (uint8_t i = 0; i < 8; i++)
  {
    Serial.print("0x");
    // zero pad the address if necessary
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
    Serial.print(", ");
  }
  Serial.println("}");
}

void IRAM_ATTR flowSolarInterrupt() {
  sensors.flowSolar += 1;
}

void IRAM_ATTR flowWaterInterrupt() {
  sensors.flowWater += 1;
}

void IRAM_ATTR buttonInterrupt() {
  unsigned long interruptTime = millis();
  if (interruptTime > input.timeLastPress + BUTTON_DEBOUNCE) {
    input.timeLastPress = interruptTime;
    input.buttonPressed = true;
  }
}

// void IRAM_ATTR encoderInterrupta() {
//   static unsigned long lastInterruptTime = 0;
//   static int lastEncoderState = 0;
//   int st = !digitalRead(PIN_ROTARY_ENCODER_CLK) | (!digitalRead(PIN_ROTARY_ENCODER_DATA) << 1);
//   if (st == 0 || st == 3)

// }

void IRAM_ATTR encoderInterrupt() {
  detachInterrupt(digitalPinToInterrupt(PIN_ROTARY_ENCODER_CLK));
  static unsigned long lastInterruptTime = 0;
  unsigned long interruptTime = millis();
  input.encoderChanged = true;
  // If interrupts come faster than 5ms, assume it's a bounce and ignore
  if (interruptTime - lastInterruptTime > 5) {
    if (digitalRead(PIN_ROTARY_ENCODER_DATA)) {
      input.encoder -= 1;
    } else {
      input.encoder += 1;
    }
  }
  lastInterruptTime = interruptTime;
  attachInterrupt(digitalPinToInterrupt(PIN_ROTARY_ENCODER_CLK), encoderInterrupt, RISING);
}

void setup(void) {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("");

  lcd.init();
  // Print a message to the LCD.
  lcd.backlight();
  lcd.setCursor(3,0);
  lcd.print("Hello, world!");

  pinMode(PIN_ROTARY_ENCODER_CLK, INPUT);
  pinMode(PIN_ROTARY_ENCODER_DATA, INPUT);
  pinMode(PIN_ROTARY_ENCODER_BUTTON, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ROTARY_ENCODER_CLK), encoderInterrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ROTARY_ENCODER_BUTTON), buttonInterrupt, FALLING);

  pinMode(PIN_HEATER_300, OUTPUT_OPEN_DRAIN);
  pinMode(PIN_HEATER_800, OUTPUT_OPEN_DRAIN);
  pinMode(PIN_HEATER_1500, OUTPUT_OPEN_DRAIN);
  pinMode(PIN_PUMP, OUTPUT_OPEN_DRAIN);

  relayOff(PIN_HEATER_300);
  relayOff(PIN_HEATER_800);
  relayOff(PIN_HEATER_1500);
  relayOff(PIN_PUMP);

  setPumpSpeed(0);

	// rotaryEncoder.begin();
	// rotaryEncoder.setup(readEncoderISR);
	// rotaryEncoder.setBoundaries(0, 1000, false); //minValue, maxValue, circleValues true|false (when max go to min and vice versa)

  tempSensors.setWaitForConversion(false); // Don't block the program while the temperature sensor is reading
  tempSensors.begin();                     // Start the temperature sensor
  DS_delay = tempSensors.millisToWaitForConversion();

  // locate devices on the bus
  Serial.print("Found ");
  Serial.print(tempSensors.getDeviceCount(), DEC);
  Serial.println(" DS18x20 devices.");
  printAddress(0);

  pt100.begin(MAX31865_2WIRE);  // set to 2WIRE or 4WIRE as necessary
  pt100.enable50Hz(true); // default filters 60Hz, set to 50Hz for europe

  // flow sensor
  pinMode(PIN_FLOW_SENSOR_WATER, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_FLOW_SENSOR_WATER), flowWaterInterrupt, RISING);
  pinMode(PIN_FLOW_SENSOR_SOLAR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_FLOW_SENSOR_SOLAR), flowSolarInterrupt, RISING);

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "Hi! This is a sample response.");
  });

  AsyncElegantOTA.begin(&server);    // Start AsyncElegantOTA
  server.begin();
  Serial.println("HTTP server started");
}

unsigned long timeSensePrev = 0.0;
unsigned long senseInterval = 1000;

void loop(void) {
  if (input.encoderChanged) {
    Serial.println(input.encoder);
    input.encoderChanged = false;
    int val = constrain(input.encoder, 0, 255);
    input.encoder = val;
    setPumpSpeed(val);
    // analogWrite(PIN_PUMP_PWM, val);
    lcd.setCursor(0, 2);
    lcd.print("Pompsnelheid: ");lcd.print(val);lcd.print("  ");
  }
  if (input.buttonPressed) {
    input.buttonPressed = false;
    Serial.println("Button pressed");
  }

  unsigned long currentMillis = millis();

  if (currentMillis > timeSensePrev + senseInterval - DS_delay) {
    tempSensors.requestTemperatures(); // Request the temperature from the sensor (it takes some time to read it)
  }

  if (currentMillis > timeSensePrev + senseInterval) {
    timeSensePrev = currentMillis;
    uint16_t val = pt100.readRTD();
    uint8_t fault = pt100.readFault();
    if (fault) {
      Serial.print("Fault 0x"); Serial.println(fault, HEX);
      pt100.clearFault();
    } else {
      sensors.tSolar = pt100.calculateTemperature(val - RTDoffset, RNOMINAL, RREF);
    }
    Serial.print("Temperature = "); Serial.println(sensors.tSolar);
    sensors.tBoilerMiddle = tempSensors.getTempC(thermo.boilerMiddle);
    Serial.println(sensors.tBoilerMiddle);
    lcd.setCursor(0, 3);
    lcd.print(sensors.tBoilerMiddle);
    lcd.print(",");
    lcd.print(sensors.tSolar);
    lcd.print(",");
    lcd.print(sensors.flowSolar);
    lcd.print(",");
    lcd.print(sensors.flowWater);
  }

  lcd.setCursor(0,1);
  lcd.print(currentMillis);

}
