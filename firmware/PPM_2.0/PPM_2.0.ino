#include <EncButton.h>
#include <EEPROM.h>
#include <Disp595.h>
#include <Disp595_symbols.h>
#include <SevenSegmentsDisp.h>

#define SENSOR_PIN A1
#define BASE_C02 420.07
#define SENSOR_R 1000
#define BUTTON_PIN 7
#define SCLK_PIN 4
#define RCLK_PIN 3
#define DATA_PIN 2
#define RGB_RED_PIN 5
#define RGB_GREEN_PIN 6
#define RGB_BLUE_PIN 9
#define HEATING_TIME 30000
#define TOTAL_CAL_COUNT 120
#define R0_EEPROM_ADDR 4

float A = 110.9169;
float B = -2.715707;
float R0 = 0.0;
float filtData = 0.0;
float k = 0.005;
bool isHeating = false;
bool isCalibrating = false;
int mode = 0;
float brightness = 1.0;
int updatePeriod = 1000;
int pointPos = 0;

Button button(BUTTON_PIN, INPUT_PULLUP, LOW);

Disp595 disp(DATA_PIN, SCLK_PIN, RCLK_PIN);

void setup() {
  Serial.begin(9600);
  delay(100);

  button.setHoldTimeout(1000);

  pinMode(RGB_RED_PIN, OUTPUT);
  pinMode(RGB_GREEN_PIN, OUTPUT);
  pinMode(RGB_BLUE_PIN, OUTPUT);

  startHeating();

  if (EEPROM.read(R0_EEPROM_ADDR) == 0xFF) {
    startCalibrating();
  } else {
    EEPROM.get(R0_EEPROM_ADDR, R0);
    Serial.print("R0: ");
    Serial.println(R0);
  }
}

void loop() {
  button.tick();
  disp.tick();
  static uint32_t tmr;
  static uint32_t tmr2;

  if (isHeating) {
    if (millis() - tmr2 >= 500) {
      tmr2 = millis();

      shiftPoint();
    }

    if (millis() - tmr >= HEATING_TIME) {
      tmr = millis();

      isHeating = false;
      setColor(0, 0, 0);
      disp.clear();
    }
  } else if (isCalibrating) {
    static uint16_t calCount = 0;

    if (millis() - tmr2 >= 500) {
      tmr2 = millis();

      shiftPoint();
    }

    if (millis() - tmr >= 100) {
      tmr = millis();

      calCount += 1;
      R0 += getR0();
    }

    if (calCount == TOTAL_CAL_COUNT) {
      R0 = R0 / TOTAL_CAL_COUNT;
      Serial.print("R0: ");
      Serial.println(R0);
      EEPROM.put(R0_EEPROM_ADDR, R0);
      isCalibrating = false;
      setColor(0, 0, 0);
      disp.clear();
    }
  } else {
    if (millis() - tmr >= 10) {
      tmr = millis();

      filtData += (getPPM() - filtData) * k;
    }

    if (millis() - tmr2 >= updatePeriod) {
      tmr2 = millis();

      displayPPM(filtData);

      Serial.print(getPPM());
      Serial.print(",");
      Serial.println(filtData);
    }

    if (button.click()) {
      changeMode();
    }

    if (button.hold()) {
      startCalibrating();
    }
  }
}

void startHeating() {
  isHeating = true;
  pointPos = 0;
  setColor(255, 80, 0);
  disp.clear();
  uint8_t HEA[] = { _H, _E, _A };
  disp.displayBytes(HEA, sizeof(HEA));
}

void startCalibrating() {
  isCalibrating = true;
  pointPos = 0;
  setColor(0, 140, 255);
  disp.clear();
  uint8_t CAL[] = { _C, _A, _L };
  disp.displayBytes(CAL, sizeof(CAL));
}

void changeMode() {
  if (mode == 0) {
    mode = 1;
    brightness = 0.8;
    updatePeriod = 3000;
    displayPPM(filtData);
  } else if (mode == 1) {
    mode = 2;
    brightness = 0.5;
    updatePeriod = 5000;
    displayPPM(filtData);
  } else if (mode == 2) {
    mode = 0;
    brightness = 1.0;
    updatePeriod = 1000;
    displayPPM(filtData);
  }
}

void displayPPM(float val) {
  if (val <= 800) setColor(0, 255, 0);          // green
  else if (val <= 1000) setColor(255, 255, 0);  // yellow
  else setColor(255, 0, 0);                     // red

  if (int(val) > 9999){
    disp.displayInt(9999);
  } else if (int(val) < 0) {
    disp.displayInt(0);
  } else {
    disp.displayInt(int(val));
  }
}

void setColor(int redVal, int greenVal, int blueVal) {
  analogWrite(RGB_RED_PIN, redVal * brightness);
  analogWrite(RGB_GREEN_PIN, greenVal * brightness);
  analogWrite(RGB_BLUE_PIN, blueVal * brightness);
}

float getRS() {
  float data = analogRead(SENSOR_PIN);
  return ((1023 / data) - 1) * SENSOR_R;
}

float getR0() {
  return getRS() * pow(A / BASE_C02, (1 / B));
}

float getPPM() {
  return A * pow((getRS() / R0), B);
}

void shiftPoint() {
  if (pointPos == 0) {
    disp.point(3, false);
  } else {
    disp.point(pointPos - 1, false);
  }

  disp.point(pointPos, true);

  if (pointPos != 3) {
    pointPos += 1;
  } else {
    pointPos = 0;
  }
}
