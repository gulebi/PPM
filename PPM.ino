#include <EncButton.h>
#include <EEPROM.h>
#include <GyverTimers.h>
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
#define DISP_PERIOD 3000
#define RGB_RED_PIN 5
#define RGB_GREEN_PIN 6
#define RGB_BLUE_PIN 9
#define HEATING_TIME 5000  // 120000  // 2 min
#define CAL_TIME 120000    // 2 min
#define UPDATE_PERIOD 5    // 5 sec
#define R0_EEPROM_ADDR 4

float A = 110.9169;
float B = -2.715707;
float R0 = 0;
unsigned long previousMillis = 0;
float smoothedPPM = 0;
int iter = 0;
int mode = 0;  // 0 - screen + led, 1 - led, 2 - screen.

Button button(BUTTON_PIN, INPUT_PULLUP, LOW);

Disp595 disp(DATA_PIN, SCLK_PIN, RCLK_PIN);

void setup() {
  Serial.begin(9600);
  delay(1000);

  Timer1.setPeriod(DISP_PERIOD);
  Timer1.enableISR();

  pinMode(RGB_RED_PIN, OUTPUT);
  pinMode(RGB_GREEN_PIN, OUTPUT);
  pinMode(RGB_BLUE_PIN, OUTPUT);

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  heating();

  if (EEPROM.read(R0_EEPROM_ADDR) == 0xFF) {
    calibrating();
  } else {
    EEPROM.get(R0_EEPROM_ADDR, R0);
  }

  Serial.println("R0: ");
  Serial.println(R0);
}

void loop() {
  button.tick();
  unsigned long currentMillis = millis();
  float PPM;

  if (currentMillis - previousMillis >= 1000) {
    PPM = getPPM();
    Serial.println("Real: ");
    Serial.println(PPM);
    smoothedPPM = smoothedPPM + PPM;
    previousMillis = currentMillis;
    iter++;

    if (iter == UPDATE_PERIOD) {
      smoothedPPM = smoothedPPM / UPDATE_PERIOD;
      Serial.println("Smoothed: ");
      Serial.println(smoothedPPM);

      displayPPM(smoothedPPM);
      smoothedPPM = 0;
      iter = 0;
    }
  }


  if (button.click()) {
    changeMode(PPM);
  }

  if (button.hold()) {
    calibrating();
  }
}

ISR(TIMER1_A) {
  disp.tickManual();
}

void changeMode(float val) {
  if (mode == 0) {
    mode = 1;
    disp.clear();
    displayPPM(val);
  } else if (mode == 1) {
    mode = 2;
    setColor(0, 0, 0);
    displayPPM(val);
  } else if (mode == 2) {
    mode = 0;
    displayPPM(val);
  }
}

void displayPPM(float val) {
  if (mode != 2) {
    if (val <= 800) setColor(0, 255, 0);          // green
    else if (val <= 1000) setColor(255, 255, 0);  // yellow
    else setColor(255, 0, 0);                     // red
  }

  if (mode != 1) {
    disp.displayInt(int(val));
  }
}

void heating() {
  setColor(255, 80, 0);
  disp.clear();
  uint8_t HEA[] = { _H, _E, _A };
  disp.displayBytes(HEA, sizeof(HEA));
  Serial.println("HEA");

  delay(HEATING_TIME);
  setColor(0, 0, 0);
  disp.clear();
}

void calibrating() {
  setColor(0, 140, 255);
  disp.clear();
  uint8_t CAL[] = { _C, _A, _L };
  disp.displayBytes(CAL, sizeof(CAL));
  Serial.println("CAL");

  for (int i = 1; i <= (CAL_TIME / 1000); i++) {
    R0 += getR0();
    delay(1000);
  };

  R0 = R0 / (CAL_TIME / 1000);
  Serial.println("R0: ");
  Serial.println(R0);
  EEPROM.put(R0_EEPROM_ADDR, R0);
  setColor(0, 0, 0);
  disp.clear();
}

void setColor(int redVal, int greenVal, int blueVal) {
  analogWrite(RGB_RED_PIN, redVal);
  analogWrite(RGB_GREEN_PIN, greenVal);
  analogWrite(RGB_BLUE_PIN, blueVal);
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
