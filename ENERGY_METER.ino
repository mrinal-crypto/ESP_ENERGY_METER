#include <Adafruit_ADS1X15.h>

Adafruit_ADS1115 ads;

const int sampleSize = 300;
float sumOfVSamples;
float sumOfISamples;
float gridVolt;
float rmsCurrent;
float real_power;
float energykWh;

unsigned long lastTime = 0;
const unsigned long interval = 500;

TaskHandle_t Task1;

void setup() {
  Serial.begin(115200);

  if (!ads.begin()) {
    Serial.println("Failed to initialize ADS.");
    while (1);
  }

  xTaskCreatePinnedToCore(
    loop1,
    "Task1",
    10000,
    NULL,
    1,
    &Task1,
    1);
  delay(500);

}

////////////////////////////////////////////////////////////////////////////////////////
void findRmsVoltCurrent(int samplesNo) {

  sumOfVSamples = 0;
  sumOfISamples = 0;

  for (int i = 1; i <= samplesNo; i++) {
    int16_t analogSamples0 = ads.readADC_SingleEnded(0);
    int16_t analogSamples1 = ads.readADC_SingleEnded(1);

    float readVolt0 = ads.computeVolts(analogSamples0);
    float readVolt1 = ads.computeVolts(analogSamples1);

    sumOfVSamples += readVolt0 * readVolt0;
    sumOfISamples += readVolt1 * readVolt1;

    //    Serial.print("AIN0: ");
    //    Serial.print(analogSamples);
    //    Serial.print("  ");
    //    Serial.print(readVolt);
    //    Serial.print("V ");
    //
    //    Serial.println(sumOfVSamples);
  }

  float meanSqVolt0 = sumOfVSamples / samplesNo;
  float meanSqVolt1 = sumOfISamples / samplesNo;

  float rmsVolt0 = sqrt(meanSqVolt0);
  float rmsVolt1 = sqrt(meanSqVolt1);

  gridVolt = (146.269 * rmsVolt0) + 14.161; //{(y-y1)/(y2-y1)}/{(x-x1)/(x2-x1)}
  rmsCurrent = (13.136 * rmsVolt1) + 0.0156; //{(y-y1)/(y2-y1)}/{(x-x1)/(x2-x1)}
  real_power = gridVolt * rmsCurrent;

  Serial.println("---------------------------------------------------");
  Serial.print("VOLTAGE(V): ");
  Serial.print(gridVolt, 0);
  Serial.print(" CURRENT(A): ");
  Serial.print(rmsCurrent, 2);
  Serial.print(" POWER(W): ");
  Serial.println(real_power, 1);
  Serial.println("---------------------------------------------------");

}
////////////////////////////////////////////////////////////////////////////////
void loop() {
  findRmsVoltCurrent(sampleSize);
}

void loop1(void * parameter) {

  for (;;) {
    //    calculateFrequency(freqSamples);
  }
}
