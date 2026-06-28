#define SENSOR_PIN 34

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
}

void loop() {
  long suma = 0;
  const int N = 100;
  for (int i = 0; i < N; i++) { suma += analogRead(SENSOR_PIN); delay(2); }
  float adc = suma / (float)N;
  Serial.print("ADC: "); Serial.print(adc, 1);
  Serial.print("   V: "); Serial.println(adc * 3.3 / 4095.0, 3);
  delay(500);
}