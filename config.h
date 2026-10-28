#pragma once
// ====== CONFIGURACIÓN DEL USUARIO ======
#define WIFI_SSID   "TU_WIFI"
#define WIFI_PASS   "TU_CLAVE_WIFI"
// IP del PC donde corre Flask (en CMD: ipconfig -> "Dirección IPv4"). Mismo Wi-Fi que el ESP32.
#define SERVER_URL  "http://192.168.1.50:5000/api/mediciones"
#define API_KEY     "cambia-esta-clave"          // igual a API_KEY del .env del backend

// ====== PINES (verificar con el pinout de tu placa) ======
#define PIN_SDA      21
#define PIN_SCL      22
#define PIN_ONEWIRE   4                          // DS18B20 + pull-up 4.7k a 3V3

// ====== MUESTREO ======
#define PERIODO_MUESTREO_MS   5000               // un envío cada 5 s
#define N_MUESTRAS_ACEL        100               // ventana de aceleración
#define INTERVALO_ACEL_MS        5               // ~200 Hz
#define N_MUESTRAS_RMS         240               // por canal eléctrico (~20 ciclos a 60 Hz)
#define TAM_COLA                30               // mediciones en espera si cae el Wi-Fi

// ====== MÓDULO ELÉCTRICO ======
// 0 = no hay sensores de corriente/tensión aún: se envían ceros (el backend lo tolera).
#define USAR_ELECTRICO 1
#define DIR_ADS1  0x48                           // ADDR a GND  -> I1, I2, I3, V1
#define DIR_ADS2  0x49                           // ADDR a VDD  -> V2, V3

// Factores de calibración: valor_real = Vrms_medido_en_ADC * factor. CALIBRAR con multímetro/pinza.
// SCT-013-030 (salida 1 V = 30 A): 30.0. Si tu SCT lleva resistencia burden, recalcula.
constexpr float CAL_I[3] = {30.0f, 30.0f, 30.0f};   // A por voltio RMS
constexpr float CAL_V[3] = {100.0f, 100.0f, 100.0f}; // V por voltio RMS (PLACEHOLDER)
constexpr float RUIDO_MIN_V = 0.005f;               // por debajo = canal apagado (0)
