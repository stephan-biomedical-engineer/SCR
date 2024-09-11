/**
**************************************************************************************************************
* @file main_control.cpp
* @author Stephan Costa Barros <stephan.barros@ufu.br> Gustavo Valle Sato <...> Bruno ... <...> Allan ... <...>
* @version V0.1.0
* @date 31-Aug-2024
* @brief code for "Sistemas de Controle Realimentado"
*************************************************************************************************************
*/

/* Includes ----------------------------------------------------------------------------------------------------------*/
#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>

/* Defines ----------------------------------------------------------------------------------------------------------*/
#define TEMPERATURE_SENSOR 4 // Define pino do sensor DS18B20
#define FLOW_SENSOR 33 // Define pino do sensor de fluxo YF-S401
#define FAN 27 // Define pino do cooler Hyper T4
#define LIGHT 21 // Define pino da luz/resistência
#define PUSH_BUTTON 34 // Define pino do botão

/* Objects declarations and functions prototyping ----------------------------------------------------------------------------------------------------------*/
OneWire oneWire(TEMPERATURE_SENSOR); // Cria um objeto OneWire
DallasTemperature sensor(&oneWire); // Cria objeto DallasTemperature
DeviceAddress temp_address; // Endereço temporário do sensor
void pulseCounter(); // Declaração da função para contar pulsos
float readTemperature(); // Declaração da função para ler a temperatura
float calculateFlowRate(); // Declaração da função para calcular a vazão

/* Variables ----------------------------------------------------------------------------------------------------------*/
float tempC; // Variável para armazenar a temperatura
volatile uint16_t duty_cycle; // Variável para armazenar o ciclo de trabalho do PWM
bool flag = false; // Flag para controle do botão
volatile unsigned long previousMicros = 0;
const long interval = 1000; // Intervalo em milissegundos para calcular a vazão
volatile unsigned long pulseCount;
volatile unsigned int PULSES_PER_LITER = 450; //**************************** Calibrar sensor de fluxo para definir esta variável ****************************
float flowRate;
float flowRateSmoothed = 0.0; // Variável para suavização da vazão
unsigned long lastDebounceTime = 0;  // Tempo da última alteração de estado do botão
unsigned long debounceDelay = 50;    // Atraso de debounce

void setup() {
  Serial.begin(115200);
    /*-----------------------------------------------  Connecting temperature sensor  -----------------------------------------------*/
  sensor.begin(); // Inicia o sensor de temperatura
  if (!sensor.getAddress(temp_address, 0)) {
    Serial.println("SENSOR NÃO CONECTADO");
    while (true); // Para o código se o sensor não estiver conectado
  }
  /*-----------------------------------------------  PIN Modes  -----------------------------------------------*/
  pinMode(FAN, OUTPUT); // Define o pino do cooler como saída
  pinMode(LIGHT, OUTPUT); // Define o pino da luz/resistência como saída
  pinMode(FLOW_SENSOR, INPUT_PULLUP); // Configura o pino do sensor de fluxo como entrada com pull-up
  pinMode(PUSH_BUTTON, INPUT_PULLUP); // Configura o pino do botão como entrada com pull-up

    /*-----------------------------------------------  Interrupts definitions  -----------------------------------------------*/
  attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR), pulseCounter, FALLING);

  /*-----------------------------------------------  PWM definitions  -----------------------------------------------*/
  ledcAttachPin(FAN, 0);    
  ledcSetup(0, 1000, 8); // Configura o PWM no canal 0, 1kHz, resolução de 8 bits
  duty_cycle = 0; // Inicializa o ciclo de trabalho

  /*-----------------------------------------------  Initial state  -----------------------------------------------*/
  digitalWrite(LIGHT, HIGH); // Liga a luz/resistência
  delay(5000); // Aguarda 5 segundos para o sensor de temperatura estabilizar
}
  
void loop() {
  // Implementação do debounce para o botão
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (digitalRead(PUSH_BUTTON) == HIGH) { // Verifica se o botão foi pressionado
      flag = !flag; // Alterna a flag
      lastDebounceTime = millis(); // Reseta o tempo de debounce
    }
  }

  tempC = readTemperature(); // Lê a temperatura do sensor de forma assíncrona
  Serial.print("Temperatura = "); 
  Serial.println(tempC);

  flowRate = calculateFlowRate(); // Calcula a vazão com suavização
  Serial.print("Vazão = ");
  Serial.println(flowRate);

  if (flag) { // Verifica se a flag foi ativada
    digitalWrite(LIGHT, LOW); // Desliga a luz
  } else {
    digitalWrite(LIGHT, HIGH); // Liga a luz
  }

   // Ajuste dinâmico do ciclo de trabalho com base na temperatura e na vazão
  if (flowRate < 10) {
    duty_cycle = 100; // Se a vazão for baixa, cooler na velocidade máxima
  } else {
    duty_cycle = map(tempC, 20, 50, 25, 100); // Mapeia a temperatura para um ciclo de trabalho entre 25% e 100%
    if (duty_cycle > 100) duty_cycle = 100; // Limita o ciclo de trabalho máximo a 100%
    else if (duty_cycle < 25) duty_cycle = 25; // Limita o ciclo de trabalho mínimo a 25%
    
    // Ajusta o ciclo de trabalho com base na vazão (quanto maior a vazão, menor a necessidade de resfriamento adicional)
    duty_cycle = duty_cycle * (1.0 - min(flowRate, 100.0f) / 100.0f);
  }

  ledcWrite(0, 255*duty_cycle/100); // Ajusta o PWM do cooler

  delay(100); // Pequeno atraso para a próxima leitura
}

/* Functions ----------------------------------------------------------------------------------------------------------*/

// Função assíncrona para ler a temperatura
float readTemperature() {
  static unsigned long lastRequest = 0;
  static float lastTemperature = 0.0;

  if (millis() - lastRequest >= 750) { // Tempo mínimo para a conversão
    lastRequest = millis();
    sensor.requestTemperatures();
    lastTemperature = sensor.getTempC(temp_address);
  }

  if (isnan(lastTemperature)) {
    Serial.println("Falha na leitura da temperatura");
    return -1; // Retorna um valor inválido em caso de erro
  } else {
    return lastTemperature;
  }
}

// Função para calcular a vazão com suavização
float calculateFlowRate() {
  unsigned long currentMicros = micros();
  float deltaTime = (float)(currentMicros - previousMicros) / 1000000.0; // Tempo em segundos
  previousMicros = currentMicros;

  float currentFlowRate = (float)pulseCount / deltaTime / PULSES_PER_LITER * 60; // Vazão em L/min
  pulseCount = 0; // Reinicia a contagem de pulsos

  // Suaviza a leitura da vazão usando suavização exponencial
  flowRateSmoothed = (0.7 * currentFlowRate) + (0.3 * flowRateSmoothed);

  return flowRateSmoothed;
}

void pulseCounter() {
  pulseCount++;
}
