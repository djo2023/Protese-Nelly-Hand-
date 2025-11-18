#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP23X17.h>
#include <Adafruit_ADS1X15.h>

// --- Objetos ---
Adafruit_MCP23X17 mcp;
Adafruit_ADS1115 ads2, ads5, ads6, ads9;

// Handle para o Mutex (NÃO UTILIZADO)
// SemaphoreHandle_t serialMutex;

// --- Definições de Pinos e Constantes ---
int pinoLED = 2; 

const int frequenciaPwm = 20000; // 20 kHz
const int resolucaoPwm = 10;     // Resolução de 10 bits (0-1023)
 
// ... (Definições dos pinos dos motores M1 a M5 permanecem iguais) ...
// Motor 1
#define M1_ENA  25 // Pino PWM do ESP32
#define M1_IN1  12 // Pino 8 do MCP23X17
#define M1_IN2  13 // Pino 9 do MCP23X17
const int canalPwm_M1 = 0;
// Motor 2
#define M2_ENA  26 // Pino PWM do ESP32
#define M2_IN1  14 // Pino 10 do MCP23X17
#define M2_IN2  15 // Pino 11 do MCP23X17
const int canalPwm_M2 = 1;
// Motor 3
#define M3_ENA  32 // Pino PWM do ESP32
#define M3_IN1  10 // Pino 12 do MCP23X17
#define M3_IN2  11 // Pino 13 do MCP23X17
const int canalPwm_M3 = 2;
// Motor 4
#define M4_ENA  33 // Pino PWM do ESP32
#define M4_IN1  8 // Pino 14 do MCP23X17
#define M4_IN2  9 // Pino 15 do MCP23X17
const int canalPwm_M4 = 3;
// Motor 5
#define M5_ENA  27 // Pino PWM do ESP32
#define M5_IN1  1  // Pino 7 do MCP23X17
#define M5_IN2  0  // Pino 6 do MCP23X17
const int canalPwm_M5 = 4;


// --- Constantes do NTC (Função lerTemp) ---
const float RESISTOR_FIXO = 10000.0;
const float NTC_NOMINAL = 10000.0;
const float TEMP_NOMINAL_C = 25.0;
const float BETA = 3950.0;
const float V_IN = 3.3;

void Motor(int M_IN1, int M_IN2, int canalPWM, int Dir, int dutyCycle){
  if (Dir == 0){
    mcp.digitalWrite(M_IN1, LOW);
    mcp.digitalWrite(M_IN2, LOW);
    ledcWrite(canalPWM, dutyCycle);
  }
  else if (Dir == 1){
    mcp.digitalWrite(M_IN1, HIGH);
    mcp.digitalWrite(M_IN2, LOW);
    ledcWrite(canalPWM, dutyCycle);
  }
  else {
    mcp.digitalWrite(M_IN1, LOW);
    mcp.digitalWrite(M_IN2, HIGH);
    ledcWrite(canalPWM, dutyCycle);   
  }
}


float lerTemp(float volt) {
  
  // A conversão de ADC raw para volts agora é feita no loop()

  float resistencia_ntc = (RESISTOR_FIXO * volt) / (V_IN - volt);
  
  // Evita divisão por zero ou log de número negativo se a voltagem for >= V_IN
  if (resistencia_ntc <= 0) {
    return -999.0; // Retorna um valor de erro
  }

  float T0_kelvin = TEMP_NOMINAL_C + 273.15;
  float temp_kelvin = 1.0 / ( (1.0 / T0_kelvin) + (1.0 / BETA) * log(resistencia_ntc / NTC_NOMINAL) );
  float temp_celsius = temp_kelvin - 273.15;
  
  return temp_celsius; // Retorna o valor calculado
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n\n--- Iniciando Setup ---");

  Wire.begin(); 
  if (!mcp.begin_I2C()) {
    Serial.println("ERRO: MCP23X17 nao encontrado!");
    while (1); 
  }
  Serial.println("MCP23X17 encontrado!");

  // Inicializa o sensor ADS1115.
  ads2.begin(0x48);
  ads5.begin(0X4A);
  ads6.begin(0x4B);
  ads9.begin(0x49);
  ads2.setGain(GAIN_TWOTHIRDS); // FSR: +/- 6.144V
  ads5.setGain(GAIN_TWOTHIRDS);
  ads6.setGain(GAIN_TWOTHIRDS);
  ads9.setGain(GAIN_TWOTHIRDS);
  Serial.println("ADS1115 configurado!");

  // ... (Configuração dos pinos do MCP e PWM permanece a mesma) ...
  //Motor 1
  mcp.pinMode(M1_IN1, OUTPUT);
  mcp.pinMode(M1_IN2, OUTPUT);
  // Motor 2
  mcp.pinMode(M2_IN1, OUTPUT);
  mcp.pinMode(M2_IN2, OUTPUT);
  // Motor 3
  mcp.pinMode(M3_IN1, OUTPUT);
  mcp.pinMode(M3_IN2, OUTPUT);
  // Motor 4
  mcp.pinMode(M4_IN1, OUTPUT);
  mcp.pinMode(M4_IN2, OUTPUT);
  // Motor 5
  mcp.pinMode(M5_IN1, OUTPUT);
  mcp.pinMode(M5_IN2, OUTPUT);
  Serial.println("Pinos do MCP configurados como SAIDA.");

  // Configura o canal do LEDC (PWM) para os pinos ENA do ESP32
  ledcSetup(canalPwm_M1, frequenciaPwm, resolucaoPwm);
  ledcSetup(canalPwm_M2, frequenciaPwm, resolucaoPwm);
  ledcSetup(canalPwm_M3, frequenciaPwm, resolucaoPwm);
  ledcSetup(canalPwm_M4, frequenciaPwm, resolucaoPwm);
  ledcSetup(canalPwm_M5, frequenciaPwm, resolucaoPwm);
  
  // Anexa os pinos ENA do ESP32 aos canais de PWM
  ledcAttachPin(M1_ENA, canalPwm_M1);
  ledcAttachPin(M2_ENA, canalPwm_M2);
  ledcAttachPin(M3_ENA, canalPwm_M3);
  ledcAttachPin(M4_ENA, canalPwm_M4);
  ledcAttachPin(M5_ENA, canalPwm_M5);
  Serial.println("Canais PWM configurados e anexados.");
  
  Serial.println("--- Setup completo. Iniciando loop() ... ---");

}


void loop() {
  int duty = 1024;
  
    delay(1000);

    Motor(M1_IN1, M1_IN2, canalPwm_M1, 2, duty);
    Motor(M2_IN1, M2_IN2, canalPwm_M2, 2, duty);
    Motor(M3_IN1, M3_IN2, canalPwm_M3, 1, duty);
    Motor(M4_IN1, M4_IN2, canalPwm_M4, 1, duty);
    Motor(M5_IN1, M5_IN2, canalPwm_M5, 1, duty);

    delay(2700);

    Motor(M1_IN1, M1_IN2, canalPwm_M1, 0, duty);
    Motor(M2_IN1, M2_IN2, canalPwm_M2, 0, duty);
    Motor(M3_IN1, M3_IN2, canalPwm_M3, 0, duty);
    Motor(M4_IN1, M4_IN2, canalPwm_M4, 0, duty);
    Motor(M5_IN1, M5_IN2, canalPwm_M5, 0, duty);

    delay(1000);

    Motor(M1_IN1, M1_IN2, canalPwm_M1, 1, duty);
    Motor(M2_IN1, M2_IN2, canalPwm_M2, 1, duty);
    Motor(M3_IN1, M3_IN2, canalPwm_M3, 2, duty);
    Motor(M4_IN1, M4_IN2, canalPwm_M4, 2, duty);
    Motor(M5_IN1, M5_IN2, canalPwm_M5, 2, duty);

    delay(2560);

    Motor(M1_IN1, M1_IN2, canalPwm_M1, 0, duty);
    Motor(M2_IN1, M2_IN2, canalPwm_M2, 0, duty);
    Motor(M3_IN1, M3_IN2, canalPwm_M3, 0, duty);
    Motor(M4_IN1, M4_IN2, canalPwm_M4, 0, duty);
    Motor(M5_IN1, M5_IN2, canalPwm_M5, 0, duty);
    delay(1000);
}