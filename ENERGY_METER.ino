#include "EmonLib.h"
#include "Ticker.h"

#define vSensorPin 39
#define iSensorPin 35
#define currCalibration 1

const int sampleSize = 200;
const int avgOf = 20;
const int freqSamples = 10;
const int currentOffset = 1280; //offset value be measured when no current passing throgh CT

int analogVSamples[sampleSize];
int analogISamples[sampleSize];
unsigned long sumOfVSamples;
unsigned long sumOfISamples;

unsigned long lastZeroCrossingTime = 0;
unsigned long period = 0;
float sumFrequency = 0; // Sum of frequency values
float avgFreq;

int rmsVolt;
float vrmsCalibration = 142.2818;
float rmsCurrent;
float power;

int sampleCount = 0;

EnergyMonitor emon;
TaskHandle_t Task1;

Ticker rmsVoltCalTimer;

void setup() {
  Serial.begin(115200);

  pinMode(vSensorPin, INPUT);
  pinMode(iSensorPin, INPUT);

  //  emon.current(iSensorPin, currCalibration);

  rmsVoltCalTimer.attach(1, intervalOfRmsMeasure);

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
void intervalOfRmsMeasure() {
  findRmsVoltCurrent(analogVSamples, analogISamples, avgOf, sampleSize);
}

void findRmsVoltCurrent(int vValues[], int iValues[], int avgOf, int size) {

  sumOfVSamples = 0;
  sumOfISamples = 0;

  for (int j = 0; j < avgOf; j++) {

    for (int i = 0; i < size; i++) {
      analogVSamples[i] = analogRead(vSensorPin);
      analogISamples[i] = analogRead(iSensorPin);
    }

    int highestVvalue = vValues[0];
    int highestIvalue = iValues[0];

    for (int i = 0; i < size; i++) {
      if (vValues[i] > highestVvalue) {
        highestVvalue = vValues[i];
      }
      if (iValues[i] > highestIvalue) {
        highestIvalue = iValues[i];
      }
    }

    sumOfVSamples += highestVvalue;
    sumOfISamples += highestIvalue;
  }

  int peakVvalue = sumOfVSamples / avgOf;
  int peakIvalue = sumOfISamples / avgOf;
  int trueIvalue = abs(peakIvalue - currentOffset);

  rmsVolt = ((peakVvalue * (3.3 / 4095)) / 1.4142) * vrmsCalibration;
  rmsCurrent = trueIvalue * 0.0168;
  power = rmsVolt * rmsCurrent;

  Serial.print("ANALOG: ");
  Serial.print(peakVvalue);
  Serial.print(" ANALOG VOLT: ");
  Serial.print((peakVvalue * 3.3 / 4095)/1.4142, 4);
  Serial.print(" RMS VOLT: ");
  Serial.print(rmsVolt);
  Serial.print(" CURRENT OFFSET: ");
  Serial.print(peakIvalue);
  Serial.print(" TRUE ANALOG I: ");
  Serial.print(trueIvalue);
  Serial.print(" RMS CURRENT: ");
  Serial.print(rmsCurrent, 2);
  Serial.print(" POWER(W): ");
  Serial.println(power);

}
void intervalFreqMeasure() {
  calculateFrequency(freqSamples);
}
void calculateFrequency(int samples) {
  float frequency;
  int analogValue = analogRead(vSensorPin);
  float voltage = analogValue * (3.3 / 4095.0);

  if (voltage > 1) { // Assuming signal crosses zero at 1V (adjust as needed)
    if (millis() - lastZeroCrossingTime > 10) { // Ignore noise and debounce
      period = millis() - lastZeroCrossingTime;
      frequency = 1000.0 / period; // Calculate frequency in Hz
      lastZeroCrossingTime = millis();
      sumFrequency += frequency;
      sampleCount++;
    }
  }
  if (sampleCount == samples) {
    avgFreq = sumFrequency / sampleCount;
    sumFrequency = 0;
    sampleCount = 0;
    //    Serial.print("Frequency: ");
    //    Serial.print(avgFreq);
    //    Serial.println(" Hz");
  }
}

void loop() {
  //    findRmsVoltCurrent(analogVSamples, analogISamples, avgOf, sampleSize);

  //  Serial.print(" RMS: ");
  //  Serial.print(rmsVolt);
  //  Serial.print("V");

  //    Serial.print(" POWER: ");


  //  Serial.print(" Frequency: ");
  //  Serial.print(avgFreq, 1);
  //  Serial.println("Hz");
  //  delay(500);
}

void loop1(void * parameter) {

  for (;;) {
    //    calculateFrequency(freqSamples);
  }
}
