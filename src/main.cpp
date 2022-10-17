#include "esp32-hal.h"
#include <Arduino.h>

#include <WiFi.h>
#include <AsyncTCP.h>

#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#include <OneWire.h>
#include <DallasTemperature.h>

#include <Adafruit_MAX31865.h>
#include <cstdint>

/* Board setup */

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
#define PIN_HEATER_300 25
#define PIN_HEATER_800 33
#define PIN_HEATER_1500 26
#define PIN_PUMP 32

// Pump PWM, with 4.7k pullup to 5V
#define PIN_PUMP_PWM 13

// DS18B20 temperature sensors, with 4.7k pullup to 3.3V
#define PIN_ONEWIRE 15

#define PIN_FLOW_SENSOR_SOLAR 14
#define PIN_FLOW_SENSOR_WATER 27

#define relayOn(pin)  digitalWrite(pin, LOW)
#define relayOff(pin) digitalWrite(pin, HIGH)

#define pow2(x) ((x)<<1)

void setPumpSpeed(int speed) {
  /*
  Pump PWM mode 1:
    0     No PWM signal, auto mode
    1-25  Maximum speed
    26-214 Decreasing speed
  215-232 Lowest speed
  233-242 Hysteresis, pump starts in intervals to avoid clocking?
  243-255 Pump stops
  */
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

// Temperature sensors
OneWire oneWire(PIN_ONEWIRE);        // Set up a OneWire instance to communicate with OneWire devices
DallasTemperature tempSensors(&oneWire); // Create an instance of the temperature sensor class
int DS_delay = 750;

// DS18B20 addresses
// Device address can be retrieved by using sensors.getAddress(deviceAddress, index)
struct {
  DeviceAddress tBoilerTop, tBoilerMiddle, tBoilerBottom, tSolarFrom, tSolarTo, tTapWater;
} thermo = {
  .tBoilerTop    = {0x28, 0xB7, 0xED, 0x79, 0x97, 0x08, 0x03, 0x57},
  .tBoilerMiddle = {0x28, 0xA3, 0x78, 0x79, 0x97, 0x08, 0x03, 0xB5},
  .tBoilerBottom = {0x28, 0x1E, 0x34, 0x79, 0x97, 0x08, 0x03, 0x30},
  .tSolarFrom    = {0x28, 0x4C, 0x29, 0x79, 0x97, 0x07, 0x03, 0xF8},
  .tSolarTo      = {0x28, 0x27, 0xA8, 0x79, 0x97, 0x07, 0x03, 0xC5},
  .tTapWater     = {0x28, 0xB4, 0x23, 0x79, 0x97, 0x08, 0x03, 0x8E},
};

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

// pt100 temperature sensor in solar collector
Adafruit_MAX31865 pt100 = Adafruit_MAX31865(PIN_SPI_CS);
// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF      430.0
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL  100.0
// Wire resistance as int, Ohm/430*2^16
int RTDoffset = 100;

#define BUTTON_DEBOUNCE 5
uint8_t char_pipe[8] = {
  B00100,
  B00100,
  B00100,
  B00100,
  B00100,
  B00100,
  B00100,
  B00100
};
uint8_t char_topleft[8] = {
  B00000,
  B00000,
  B00000,
  B00111,
  B00100,
  B00100,
  B00100,
  B00100
};
uint8_t char_topright[8] = {
  B00000,
  B00000,
  B00000,
  B11100,
  B00100,
  B00100,
  B00100,
  B00100
};
uint8_t char_bottomleft[8] = {
  B00100,
  B00100,
  B00100,
  B00111,
  B00000,
  B00000,
  B00000,
  B00000
};uint8_t char_bottomright[8] = {
  B00100,
  B00100,
  B00100,
  B11100,
  B00000,
  B00000,
  B00000,
  B00000
};
uint8_t char_degree[8] = {
  B11100,
  B10100,
  B11100,
  B00000,
  B00000,
  B00000,
  B00000,
  B00000
};
uint8_t char_lpm[8] = {
  B10000,
  B10001,
  B10010,
  B00100,
  B00000,
  B11010,
  B10101,
  B10101
};

struct {
  bool buttonPressed;
  bool buttonClick;
  bool encoderChanged;
  int encoder;
} input;

struct {
  float tSolar;
  float tBoilerTop;
  float tBoilerMiddle;
  float tBoilerBottom;
  float tSolarFrom;
  float tSolarTo;
  float tTapWater;
  volatile int32_t flowWater;
  volatile int32_t flowSolar;
} sensors;

class FlowMeter {
  public:
  volatile uint32_t lastTime;
  volatile uint32_t count;
  volatile uint32_t sum;
  volatile uint32_t varSum;
  volatile uint32_t meanEst;
  volatile uint32_t mind;
  volatile uint32_t maxd;
  uint32_t mean;
  float std;

  FlowMeter() {}

  void calcAndReset() {
    if (count) {
      mean = sum/count;
      float a = (float)varSum/count - pow2((int)mean - (int)meanEst);
      if (a >= 0) {
        std = sqrt(a);
      } else std = 999;
    } else {
      mean = 0;
    }
    count = 0;
    sum = 0;
    varSum = 0;
  }

  void IRAM_ATTR isr() {
    uint32_t time = millis();
    uint32_t delta = time - lastTime;
    if (count == 0) {
      meanEst = delta;
    }
    count++;
    sum += delta;
    varSum += pow2(delta - meanEst);
    lastTime = time;
  }
};

void IRAM_ATTR isr(FlowMeter* f) {
  uint32_t time = millis();
  uint32_t delta = time - f->lastTime;
  if (f->count == 0) {
    f->meanEst = delta;
    f->sum = 0;
    f->varSum = 0;
    f->maxd = delta;
    f->mind = delta;
  }
  f->count++;
  f->sum += delta;
  if (delta < f->mind) f->mind = delta;
  if (delta > f->maxd) f->maxd = delta;
  f->varSum += pow2(delta - f->meanEst);
  f->lastTime = time;
}

FlowMeter flowWater, flowSolar;

int pumpSpeed = 0;
int heater = 0;

bool safeMode = false;

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
AsyncEventSource events("/events");

void IRAM_ATTR flowSolarInterrupt() {
  // flowSolar.isr();
  isr(&flowSolar);
}

void IRAM_ATTR flowWaterInterrupt() {
  // flowWater.isr();
  isr(&flowWater);
}

void IRAM_ATTR encoderInterrupt() {
  static uint8_t lastEncoderState = 0;
  static uint8_t lastEncoderState2 = 0;
  uint8_t a = !digitalRead(PIN_ROTARY_ENCODER_CLK);
  uint8_t b = !digitalRead(PIN_ROTARY_ENCODER_DATA);
  int st = a | (b << 1);
  if (st != lastEncoderState && st != lastEncoderState2 && a == b) {
    if (st) {
      input.encoderChanged = true;
      if (lastEncoderState == 1) {
        input.encoder += 1;
      } else {
        input.encoder -= 1;
      }
    }
    lastEncoderState2 = st;
  }
  lastEncoderState = st;

  static uint8_t lastButtonState = 0;
  static uint8_t buttonDebounce = 0;
  uint8_t but = !digitalRead(PIN_ROTARY_ENCODER_BUTTON);
  if (but != input.buttonPressed) {
    if (but != lastButtonState) {
      buttonDebounce = BUTTON_DEBOUNCE;
    } else {
      buttonDebounce--;
      if (buttonDebounce == 0) {
        input.buttonPressed = but;
        if (but) {
          input.buttonClick = true;
        }
      }
    }
  }
  lastButtonState = but;
}

void setup(void) {
  Serial.begin(115200);

  pinMode(PIN_ROTARY_ENCODER_CLK, INPUT);
  pinMode(PIN_ROTARY_ENCODER_DATA, INPUT);
  pinMode(PIN_ROTARY_ENCODER_BUTTON, INPUT_PULLUP);

  pinMode(PIN_FLOW_SENSOR_SOLAR, INPUT_PULLUP);
  pinMode(PIN_FLOW_SENSOR_WATER, INPUT_PULLUP);

  lcd.init();
  // Print a message to the LCD.
  lcd.backlight();
  lcd.setCursor(3,0);
  lcd.print("Zonneboiler");

  if (!digitalRead(PIN_ROTARY_ENCODER_BUTTON)) {
    safeMode = true;

    WiFi.mode(WIFI_MODE_APSTA);
    WiFi.softAP("Zonneboiler", "password");
    Serial.println ("Access Point created");
    WiFi.begin(ssid, password);

    lcd.setCursor(3,0);
    lcd.print(" Update mode ");

    int n = 0;
    while (WiFi.status() != WL_CONNECTED) {  // Wait for the Wi-Fi to connect
      delay(250);
      n += 250;
      if (n > 5000) { // | !digitalRead(SEAT_SENSOR_PIN)) {
        Serial.println("Connection failed");
        break;
      }
      Serial.print('.');
    }

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
      request->send(200, "text/plain", "Safe mode. Go to /update to update firmware.");
    });

    AsyncElegantOTA.begin(&server);    // Start AsyncElegantOTA
    server.begin();

    lcd.setCursor(0,1);
    lcd.print("Wifi pass: password");
    lcd.setCursor(0,2);
    lcd.print("192.168.4.1/update");
    return;
  }

  lcd.createChar(1, char_topleft);
  // lcd.createChar(2, char_topright);
  lcd.createChar(3, char_bottomleft);
  // lcd.createChar(4, char_bottomright);
  lcd.createChar(5, char_pipe);
  lcd.createChar(6, char_degree);
  lcd.createChar(7, char_lpm);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("");

  hw_timer_t * timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, encoderInterrupt, true);
  timerAlarmWrite(timer, 1000, true);
  timerAlarmEnable(timer);

  pinMode(PIN_HEATER_300, OUTPUT_OPEN_DRAIN);
  pinMode(PIN_HEATER_800, OUTPUT_OPEN_DRAIN);
  pinMode(PIN_HEATER_1500, OUTPUT_OPEN_DRAIN);
  pinMode(PIN_PUMP, OUTPUT_OPEN_DRAIN);

  relayOff(PIN_HEATER_300);
  relayOff(PIN_HEATER_800);
  relayOff(PIN_HEATER_1500);
  relayOff(PIN_PUMP);

  setPumpSpeed(0);

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

  AsyncElegantOTA.begin(&server);
  server.begin();
  Serial.println("HTTP server started");
}

unsigned long timeSensePrev = 0.0;
unsigned long senseInterval = 1000;

int selected = 0;

void loop(void) {
  if (safeMode) return;

  if (input.buttonClick) {
    input.buttonClick = false;
    selected++;
    if (selected == 4) selected = 0;
    lcd.setCursor(0, 1);
    switch (selected) {
    case 0:
      lcd.print("Pomp:  ");
      input.encoder = pumpSpeed;
      break;
    case 1:
      lcd.print("Heater:");
      input.encoder = heater;
      break;
    case 2:
      lcd.print("RTD:   ");
      input.encoder = RTDoffset;
      break;
    }
    lcd.setCursor(0, 2);
    lcd.print(input.encoder);
    Serial.println("Button pressed");
  }
  if (input.encoderChanged) {
    Serial.println(input.encoder);
    input.encoderChanged = false;
    lcd.setCursor(0, 2); lcd.print("        ");
    lcd.setCursor(0, 2);
    if (selected == 0) {
      int val = constrain(input.encoder, 0, 100);
      pumpSpeed = input.encoder = val;
      setPumpSpeed(val);
      lcd.print(val);
    } else if (selected == 1) {
      int val = constrain(input.encoder, 0, 3);
      heater = input.encoder = val;
      switch (val) {
        case 0:
          lcd.print(" off");
          relayOff(PIN_HEATER_300);
          relayOff(PIN_HEATER_800);
          relayOff(PIN_HEATER_1500);
          break;
        case 1:
          lcd.print(" 300W");
          relayOn(PIN_HEATER_300);
          relayOff(PIN_HEATER_800);
          relayOff(PIN_HEATER_1500);
          break;
        case 2:
          lcd.print(" 800W");
          relayOff(PIN_HEATER_300);
          relayOn(PIN_HEATER_800);
          relayOff(PIN_HEATER_1500);
          break;
        case 3:
          lcd.print("1500W");
          relayOff(PIN_HEATER_300);
          relayOff(PIN_HEATER_800);
          relayOn(PIN_HEATER_1500);
          break;
      }
    } else if (selected == 2) {
      int val = input.encoder;
      RTDoffset = val;
      lcd.print(val);
    } else if (selected == 3) {
      int val = constrain(input.encoder, 0, 12);
      input.encoder = val;
      lcd.clear();
      for (int n = 0; n < 4; n++) {
        lcd.setCursor(0, n); lcd.print(val + n, HEX);
        lcd.setCursor(3, n);
        for (int x = 0; x < 16; x++) {
          lcd.write((val+n)*16 + x);
        }
      }
    }
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
    sensors.tBoilerTop    = tempSensors.getTempC(thermo.tBoilerTop);
    sensors.tBoilerMiddle = tempSensors.getTempC(thermo.tBoilerMiddle);
    sensors.tBoilerBottom = tempSensors.getTempC(thermo.tBoilerBottom);
    sensors.tSolarFrom    = tempSensors.getTempC(thermo.tSolarFrom);
    sensors.tSolarTo      = tempSensors.getTempC(thermo.tSolarTo);
    sensors.tTapWater     = tempSensors.getTempC(thermo.tTapWater);
    Serial.print("tBoilerTop:    "); Serial.println(sensors.tBoilerTop);
    Serial.print("tBoilerMiddle: "); Serial.println(sensors.tBoilerMiddle);
    Serial.print("tBoilerBottom: "); Serial.println(sensors.tBoilerBottom);
    Serial.print("tSolarFrom:    "); Serial.println(sensors.tSolarFrom);
    Serial.print("tSolarTo:      "); Serial.println(sensors.tSolarTo);
    Serial.print("tTapWater:     "); Serial.println(sensors.tTapWater);

//      ----------------------
//      |                ___ |
//      |         ⌜-75°-|65°||
//      |        86° 12`|36°||
//      |         ⌞-32°-|23°||
//      ----------------------
    if (selected != 3) {
    lcd.setCursor(15, 0); lcd.print(" ___ ");
    lcd.setCursor(8, 1); lcd.print(" \1---\6-\5  \6\5");
    lcd.setCursor(8, 2); lcd.print("  \6   \7\5  \6\5");
    lcd.setCursor(8, 3); lcd.print(" \3---\6-\5  \6\5");
    lcd.setCursor(11, 1); lcd.print(int(0.5f+sensors.tSolarFrom));
    lcd.setCursor(16, 1); lcd.print(int(0.5f+sensors.tBoilerTop));
    lcd.setCursor(8, 2); lcd.print(int(0.5f+sensors.tSolar));
    // lcd.setCursor(5, 2); lcd.print((sensors.tSolar));
    lcd.setCursor(12, 2); lcd.print(" 0");//int(0.5f+sensors.flowSolar));
    lcd.setCursor(16, 2); lcd.print(int(0.5f+sensors.tBoilerMiddle));
    lcd.setCursor(11, 3); lcd.print(int(0.5f+sensors.tSolarTo));
    lcd.setCursor(16, 3); lcd.print(int(0.5f+sensors.tBoilerBottom));

    }
    lcd.setCursor(0, 0); lcd.print(flowSolar.count);
    // lcd.print(","); lcd.print(flowWater.count);
    flowSolar.calcAndReset();
    flowWater.calcAndReset();
    lcd.print("m"); lcd.print(flowSolar.mean);
    lcd.print("i"); lcd.print(flowSolar.mind);
    lcd.print("a"); lcd.print(flowSolar.maxd);
    lcd.print("d"); lcd.print(flowSolar.std);
    // lcd.print(","); lcd.print(flowWater.mean);
    // lcd.print(","); lcd.print(flowWater.std);
    // tempSensors.begin();
    // if (tempSensors.getDeviceCount())
    //   printAddress(0);

  }
}
