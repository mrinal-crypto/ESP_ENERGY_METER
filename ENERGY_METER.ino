
#define sensorPin 34


const int sampleSize = 1000;
const int avgOf = 10;
int analogSamples[sampleSize];
unsigned long sumOfSamples;

unsigned long lastZeroCrossingTime = 0;
unsigned long period = 0;
float sumFrequency = 0; // Sum of frequency values
int avgFreq;
int rmsVolt;
int sampleCount = 0;

TaskHandle_t Task1;

void setup() {
  Serial.begin(115200);
  pinMode(sensorPin, INPUT);

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

void findRmsVolt(int values[], int avgOf, int size) {

  sumOfSamples = 0;
  for (int j = 0; j < avgOf; j++) {

    for (int i = 0; i < size; i++) {
      analogSamples[i] = analogRead(sensorPin);
    }

    int highestValue = values[0];
    for (int i = 0; i < size; i++) {
      if (values[i] > highestValue) {
        highestValue = values[i];
      }
    }

    sumOfSamples += highestValue;
  }

  int peakValue = sumOfSamples / avgOf;
  rmsVolt = peakValue * 0.065753;

  //  Serial.print("ANALOG: ");
  //  Serial.print(peakValue);
  //  Serial.print(" RMS: ");
  //  Serial.println(rmsVolt);

}


void calculateFrequency(int samples) {
  float frequency;
  int analogValue = analogRead(sensorPin);
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

  findRmsVolt(analogSamples, avgOf, sampleSize);
  
  Serial.print("RMS: ");
  Serial.print(rmsVolt);
  Serial.print("V");
  Serial.print(" Frequency: ");
  Serial.print(avgFreq);
  Serial.println("Hz");
  //    delay(1000);
}

void loop1(void * parameter) {

  for (;;) {
    calculateFrequency(40);
  }
}
