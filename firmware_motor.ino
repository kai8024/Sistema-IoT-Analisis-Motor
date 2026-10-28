/*
  Monitoreo IoT de motor trifásico - firmware ESP32 (Arduino IDE)
  Tarea "sensado" (núcleo 1): lee DS18B20, ADXL345 y ADS1115 -> cola
  Tarea "envio"   (núcleo 0): toma de la cola y hace POST JSON al backend Flask, con reintentos

  Librerías (Gestor de librerías): OneWire, DallasTemperature, Adafruit ADXL345,
  Adafruit Unified Sensor, Adafruit ADS1X15 (>= 2.4), ArduinoJson (v7).
  Placa: "ESP32 Dev Module" (paquete esp32 de Espressif).
*/
#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_ADS1X15.h>
#include <ArduinoJson.h>
#include "config.h"

struct Muestra {
  float temperatura;
  float ax, ay, az;        // en g (última lectura de la ventana)
  float vib_rms;           // RMS dinámico de |a| en la ventana (extra, el backend aún lo ignora)
  float corriente[3];      // A
  float tension[3];        // V
};

static QueueHandle_t colaMuestras;
static OneWire oneWire(PIN_ONEWIRE);
static DallasTemperature ds18(&oneWire);
static Adafruit_ADXL345_Unified adxl(12345);
#if USAR_ELECTRICO
static Adafruit_ADS1115 ads1, ads2;
#endif

// ---------------- Lectura de sensores ----------------
bool leerTemperatura(float &t) {
  ds18.requestTemperatures();
  float v = ds18.getTempCByIndex(0);
  if (v == DEVICE_DISCONNECTED_C || v < -50.0f || v > 125.0f) return false;
  t = v;
  return true;
}

void leerAcelerometro(Muestra &m) {
  const float G = 9.80665f;
  double s = 0, s2 = 0;
  sensors_event_t e;
  for (int i = 0; i < N_MUESTRAS_ACEL; i++) {
    adxl.getEvent(&e);
    m.ax = e.acceleration.x / G;
    m.ay = e.acceleration.y / G;
    m.az = e.acceleration.z / G;
    float mag = sqrtf(m.ax * m.ax + m.ay * m.ay + m.az * m.az);
    s += mag;
    s2 += (double)mag * mag;
    vTaskDelay(pdMS_TO_TICKS(INTERVALO_ACEL_MS));
  }
  double media = s / N_MUESTRAS_ACEL;
  double var = s2 / N_MUESTRAS_ACEL - media * media;
  m.vib_rms = var > 0 ? (float)sqrt(var) : 0.0f;
}

#if USAR_ELECTRICO
// RMS de la componente alterna (quita el offset DC) de un canal del ADS1115, en voltios.
float rmsCanal(Adafruit_ADS1115 &ads, uint8_t canal) {
  double s = 0, s2 = 0;
  for (int i = 0; i < N_MUESTRAS_RMS; i++) {
    float v = ads.computeVolts(ads.readADC_SingleEnded(canal));
    s += v;
    s2 += (double)v * v;
  }
  double media = s / N_MUESTRAS_RMS;
  double var = s2 / N_MUESTRAS_RMS - media * media;
  return var > 0 ? (float)sqrt(var) : 0.0f;
}
#endif

void leerElectrico(Muestra &m) {
#if USAR_ELECTRICO
  float vi[3] = {rmsCanal(ads1, 0), rmsCanal(ads1, 1), rmsCanal(ads1, 2)};
  float vv[3] = {rmsCanal(ads1, 3), rmsCanal(ads2, 0), rmsCanal(ads2, 1)};
  for (int i = 0; i < 3; i++) {
    m.corriente[i] = (vi[i] < RUIDO_MIN_V) ? 0.0f : vi[i] * CAL_I[i];
    m.tension[i]   = (vv[i] < RUIDO_MIN_V) ? 0.0f : vv[i] * CAL_V[i];
  }
#else
  for (int i = 0; i < 3; i++) { m.corriente[i] = 0; m.tension[i] = 0; }
#endif
}

// ---------------- Tarea de sensado ----------------
void tareaSensado(void *) {
  TickType_t ultimo = xTaskGetTickCount();
  float ultimaTemp = NAN;
  for (;;) {
    Muestra m{};
    bool ok = leerTemperatura(m.temperatura);
    if (ok) ultimaTemp = m.temperatura;
    else if (!isnan(ultimaTemp)) m.temperatura = ultimaTemp;   // reutiliza la última válida
    else {
      Serial.println("[sensado] DS18B20 sin lectura valida; se omite el ciclo");
      vTaskDelayUntil(&ultimo, pdMS_TO_TICKS(PERIODO_MUESTREO_MS));
      continue;
    }
    leerAcelerometro(m);
    leerElectrico(m);

    Serial.printf("[sensado] T=%.1fC |a|~(%.2f,%.2f,%.2f)g vibRMS=%.3f I=%.2f/%.2f/%.2f V=%.0f/%.0f/%.0f\n",
                  m.temperatura, m.ax, m.ay, m.az, m.vib_rms,
                  m.corriente[0], m.corriente[1], m.corriente[2],
                  m.tension[0], m.tension[1], m.tension[2]);

    if (xQueueSend(colaMuestras, &m, 0) != pdTRUE) {           // cola llena: descarta la más antigua
      Muestra vieja;
      xQueueReceive(colaMuestras, &vieja, 0);
      xQueueSend(colaMuestras, &m, 0);
      Serial.println("[sensado] cola llena, se descarto la medicion mas antigua");
    }
    vTaskDelayUntil(&ultimo, pdMS_TO_TICKS(PERIODO_MUESTREO_MS));
  }
}

// ---------------- Comunicación ----------------
bool conectarWiFi() {
  if (WiFi.status() == WL_CONNECTED) return true;
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) vTaskDelay(pdMS_TO_TICKS(500));
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("[wifi] conectado, IP %s\n", WiFi.localIP().toString().c_str());
    return true;
  }
  Serial.println("[wifi] sin conexion");
  return false;
}

int enviar(const Muestra &m) {
  JsonDocument doc;
  doc["temperatura"]   = m.temperatura;
  doc["aceleracion_x"] = m.ax;
  doc["aceleracion_y"] = m.ay;
  doc["aceleracion_z"] = m.az;
  doc["vibracion_rms_g"] = m.vib_rms;      // el backend actual ignora campos desconocidos
  for (int i = 0; i < 3; i++) {
    doc[String("corriente_l") + (i + 1)] = m.corriente[i];
    doc[String("tension_l") + (i + 1)]   = m.tension[i];
  }
  String cuerpo;
  serializeJson(doc, cuerpo);

  HTTPClient http;
  http.setTimeout(5000);
  if (!http.begin(SERVER_URL)) return -1;
  http.addHeader("Content-Type", "application/json");
  http.addHeader("X-API-Key", API_KEY);
  int code = http.POST(cuerpo);
  http.end();
  return code;
}

void tareaEnvio(void *) {
  WiFi.setAutoReconnect(true);
  conectarWiFi();
  Muestra m;
  for (;;) {
    if (xQueuePeek(colaMuestras, &m, pdMS_TO_TICKS(1000)) != pdTRUE) continue;  // nada pendiente
    if (!conectarWiFi()) { vTaskDelay(pdMS_TO_TICKS(2000)); continue; }

    int code = enviar(m);
    Serial.printf("[envio] HTTP %d (pendientes: %u)\n", code, (unsigned)uxQueueMessagesWaiting(colaMuestras));
    if ((code >= 200 && code < 300) || code == 400) {   // 400 = dato invalido: no reintentar
      xQueueReceive(colaMuestras, &m, 0);               // se saca de la cola solo si ya se resolvio
    } else {
      if (code == 401) Serial.println("[envio] API_KEY incorrecta");
      vTaskDelay(pdMS_TO_TICKS(2000));                  // fallo de red/servidor: reintenta
    }
  }
}

// ---------------- Arranque ----------------
void setup() {
  Serial.begin(115200);
  delay(500);
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);

  ds18.begin();
  ds18.setResolution(11);
  if (ds18.getDeviceCount() == 0) Serial.println("[init] ATENCION: no se detecta el DS18B20 (revisa pull-up 4.7k)");

  while (!adxl.begin(0x53)) {                           // 0x53 si SDO a GND; 0x1D si SDO a VCC
    Serial.println("[init] ADXL345 no responde, revisa SDA/SCL/alimentacion");
    delay(1000);
  }
  adxl.setRange(ADXL345_RANGE_4_G);
  adxl.setDataRate(ADXL345_DATARATE_200_HZ);

#if USAR_ELECTRICO
  if (!ads1.begin(DIR_ADS1) || !ads2.begin(DIR_ADS2)) {
    Serial.println("[init] ADS1115 no responde (¿direcciones 0x48/0x49?)");
    while (true) delay(1000);
  }
  ads1.setGain(GAIN_ONE);  ads2.setGain(GAIN_ONE);      // +-4.096 V
  ads1.setDataRate(RATE_ADS1115_860SPS);
  ads2.setDataRate(RATE_ADS1115_860SPS);
#endif

  colaMuestras = xQueueCreate(TAM_COLA, sizeof(Muestra));
  xTaskCreatePinnedToCore(tareaSensado, "sensado", 6144, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(tareaEnvio, "envio", 10240, NULL, 1, NULL, 0);
}

void loop() { vTaskDelay(pdMS_TO_TICKS(1000)); }       // todo el trabajo ocurre en las tareas
