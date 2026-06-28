#include <Arduino.h>
#include <ESP32Servo.h>
#include <math.h>

#define SERVO_PIN     13
#define SENSOR_PIN    34

const int US_NIVEL = 1420; 
const int US_TEST  = 1900;   // El pulso que genera los 2.85 grados

// --- Configuración del Sensor Sharp ---
#define N_MUESTRAS   4      // Menos muestras para mayor velocidad de muestreo
#define ALPHA        0.70f  
const int N_PTS = 19;
const float dist_cm[N_PTS] = { 3, 5, 7, 9, 11, 13, 15, 17, 19, 21, 23, 25, 27, 29, 31, 33, 35, 37, 39 };
const float adc_val[N_PTS] = { 2760, 2350, 1940, 1680, 1470, 1360, 1190, 1080, 970, 895, 820, 760, 710, 660, 620, 600, 570, 540, 516 };
const float CENTRO_CM     = 22.5f;
const float RADIO_BOLA_CM = 2.0f;

float distFiltrada = 0.0f;
bool primera = true;
Servo servo;

float leerADC() {
  long s = 0;
  for (int i = 0; i < N_MUESTRAS; i++) { s += analogRead(SENSOR_PIN); delayMicroseconds(100); }
  return s / (float)N_MUESTRAS;
}

float adcADistancia(float adc) {
  if (adc >= adc_val[0])       return dist_cm[0];
  if (adc <= adc_val[N_PTS-1]) return dist_cm[N_PTS-1];
  for (int i = 0; i < N_PTS - 1; i++) {
    if (adc <= adc_val[i] && adc >= adc_val[i+1]) {
      float t = (adc - adc_val[i]) / (adc_val[i+1] - adc_val[i]);
      return dist_cm[i] + t * (dist_cm[i+1] - dist_cm[i]);
    }
  }
  return -1.0f;
}

float leerPosicion() {
  float d = adcADistancia(leerADC());
  if (d < 0) d = distFiltrada;
  if (primera) { distFiltrada = d; primera = false; }
  else         { distFiltrada = ALPHA * d + (1.0f - ALPHA) * distFiltrada; }
  return ((distFiltrada + RADIO_BOLA_CM) - CENTRO_CM) / 100.0f; // Retorna en METROS
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  
  servo.setPeriodHertz(50);
  servo.attach(SERVO_PIN, 500, 2500);
  
  servo.writeMicroseconds(US_NIVEL);
  Serial.println("\n--- Sistema Listo ---");
  Serial.println("Envia 'g' para iniciar el test de aceleracion.");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'g') {
      Serial.println("# Inclinando barra... ¡Pon la bola en el extremo alto!");
      servo.writeMicroseconds(US_TEST);
      
      // Cuenta regresiva para que te prepares
      for(int i=3; i>0; i--) {
        Serial.print("# "); Serial.println(i);
        delay(1000);
      }
      Serial.println("# ¡SUELTA LA BOLA AHORA!");
      Serial.println("tiempo_s,posicion_m"); // Cabecera para Python
      
      unsigned long t0 = millis();
      primera = true; // Reiniciar filtro para el test
      
      // Registra datos durante 2 segundos
      while (millis() - t0 < 2000) {
        float t_seg = (millis() - t0) / 1000.0f;
        float pos = leerPosicion();
        
        Serial.print(t_seg, 4);
        Serial.print(",");
        Serial.println(pos, 4);
        
        delay(15); // Muestreo cercano a ~60Hz
      }
      
      // Regresa a nivel al terminar
      servo.writeMicroseconds(US_NIVEL);
      Serial.println("# FIN DEL TEST. Copia los datos de arriba.");
    }
  }
}