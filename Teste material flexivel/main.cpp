#include <Wire.h>
#include <Adafruit_ADS1X15.h>

Adafruit_ADS1115 ads;

unsigned long long cont = 0;

const int frequenciaPwm = 20000; // 25 kHz
// Resolução de 10 bits
const int resolucaoPwm = 10;

// Motor 1
#define M1_ENA  19
#define M1_IN1  13
#define M1_IN2  14
// Canal do periférico LEDC (0-15)
const int canalPwm_M1 = 0;

void Motor(int M_IN1, int M_IN2, int canalPWM, int Dir, int dutyCycle){
  if (Dir == 0){
    digitalWrite(M_IN1, LOW);
    digitalWrite(M_IN2, LOW);
    ledcWrite(canalPWM, dutyCycle);
    //Serial.println("00");
  }
  else if (Dir == 1){
    digitalWrite(M_IN1, HIGH);
    digitalWrite(M_IN2, LOW);
    ledcWrite(canalPWM, dutyCycle);
    //Serial.println("11");
  }
  else {
    digitalWrite(M_IN1, LOW);
    digitalWrite(M_IN2, HIGH);
    ledcWrite(canalPWM, dutyCycle);  
    //Serial.println("22");
  }
}

void Ligamotor(){
  // Gira todos os 5 motores para FRENTE
Motor(M1_IN1, M1_IN2, canalPwm_M1, 1, 1024);
}

void Desligamotor(){
Motor(M1_IN1, M1_IN2, canalPwm_M1, 0, 512);
}


void setup() {
  Serial.begin(115200);

  // Configura o canal do LEDC
  ledcSetup(canalPwm_M1, frequenciaPwm, resolucaoPwm);
  // Anexa o pino ao canal
  ledcAttachPin(M1_ENA, canalPwm_M1);

  ads.begin(0x48);
  //Serial.println("teste");
  Serial.println("Iniciando sensor...");
  if (!ads.begin()) { // exemplo com ADS1115
    Serial.println("Falha ao detectar dispositivo I2C!");
  } else {
    Serial.println("Sensor encontrado!");
  }
  ads.setGain(GAIN_ONE);       // FSR: +/- 4.096V
  // Motor 1
  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);

  delay(5000);

}


void loop() {

  //Serial.println("tudo certo");

  int16_t adc0; 
  float volts0;

  //adc0 = ads.readADC_SingleEnded(1);
  //volts0 = ads.computeVolts(adc0);
  //Serial.println(volts0, 4);

  Motor(M1_IN1, M1_IN2, canalPwm_M1, 1, 1024);
  delay(1400);
  Motor(M1_IN1, M1_IN2, canalPwm_M1, 0, 1024);
  delay(1000);
  Motor(M1_IN1, M1_IN2, canalPwm_M1, 2, 1024);
  delay(1340);
  Motor(M1_IN1, M1_IN2, canalPwm_M1, 0, 1024);
  delay(1000);
  cont = cont+1;
  Serial.println(cont);
}
