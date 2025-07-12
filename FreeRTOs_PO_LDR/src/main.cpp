#include <WiFi.h>
#include <PubSubClient.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <Servo.h>

// Configurações de rede e MQTT
const char* ssid = "API_001";               //Nome do Wifi
const char* password = "SOLARtracker";      //Senha do wifi
const char* mqtt_server = "10.0.0.155";     //Endereço IP do mosquitto
const int mqtt_port = 1883;                 //Porta do mosquitto
const char* mqtt_user = "solar";            //Usuário mosquitto
const char* mqtt_password = "tracker";      //Senha mosquitto

// Variáveis MQTT
WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastReconnectAttempt = 0;
const unsigned long reconnectInterval = 5000;

// Variáveis contagem do tempo para PID
volatile float hora = 0.0;
volatile float minuto = 0.0;
volatile float segundo = 0.0;
char horaminutosegundo[20] = {0}; 

// Variáveis Algoritmo Perturba e Observa
Servo myservo;      
int pinoServo = 18;
int posServo;
int passoMovimento = 1;
int limiteMax = 150;
int limiteMin = 50;
int leituraAnterior = 0;
int direcao = 1;
unsigned long tempoAnterior = 0;
const int intervaloPerturbacao = 250;


//Função de Callback para mensagens MQTT recebidas
void callback(char* topic, byte* payload, unsigned int length) {

  char message[50] = {0};
  unsigned int copyLength = length < sizeof(message)-1 ? length : sizeof(message)-1;
  strncpy(message, (char*)payload, copyLength);
  message[copyLength] = '\0';

  if (strcmp(topic, "topic/horaminutosegundo") == 0) {
    strncpy(horaminutosegundo, message, sizeof(horaminutosegundo)-1);
    Serial.printf("HoraMinutoSegundo recebido: %s\n", horaminutosegundo);
  }
  else if (strcmp(topic, "topic/hora") == 0) {
    hora = atof(message);
    Serial.printf("Hora recebida: %.2f\n", hora);
  }
  else if (strcmp(topic, "topic/minuto") == 0) {
    minuto = atof(message);
    Serial.printf("Minuto recebido: %.2f\n", minuto);
  }
  else if (strcmp(topic, "topic/segundo") == 0) {
    segundo = atof(message);
    Serial.printf("Segundo recebido: %.2f\n", segundo);
  }
}

//Função Conectar WiFi
bool connectWiFi() {

  Serial.begin(115200);
  Serial.println("\nConectando a " + String(ssid));
  
  WiFi.disconnect(true);
  WiFi.begin(ssid, password);
  
  unsigned long startAttemptTime = millis();
  const unsigned long timeout = 30000;
  
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < timeout) {
    vTaskDelay(500 / portTICK_PERIOD_MS);
    Serial.print(".");
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi conectado. IP: " + WiFi.localIP().toString());
    return true;
  } else {
    Serial.println("\nFalha ao conectar WiFi");
    return false;
  }
}

//Função de Reconexão MQTT
bool reconnectMQTT() {

  if (!client.connected()) {
    Serial.println("Tentando conexão MQTT...");
    
    if (client.connect("ESP32Client", mqtt_user, mqtt_password)) {
      
      client.subscribe("topic/horaminutosegundo", 1);
      client.subscribe("topic/hora", 1);
      client.subscribe("topic/minuto", 1);
      client.subscribe("topic/segundo", 1);
      return true;
    } else {
      Serial.printf("Falha na conexão. Código: %d\n", client.state());
      return false;
    }
  }
  return true;
}

// Task MQTT
void mqttTask(void *pvParameters) {

  while (1) {
    if (WiFi.status() == WL_CONNECTED) {
      if (!client.connected()) {
        if (millis() - lastReconnectAttempt > reconnectInterval) {
          lastReconnectAttempt = millis();
          if (!reconnectMQTT()) {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
          }
        }
      } else {
        client.loop();
      }
    } else {
      Serial.println("WiFi desconectado, tentando reconectar...");
      connectWiFi();
    }
    
    vTaskDelay(100 / portTICK_PERIOD_MS); 
  }
}

// Task de leitura dos dados do Painel
void dadosPainelTask(void *pvParameters) {

  analogReadResolution(12); 
  analogSetAttenuation(ADC_11db);

  while (1) {
    if (WiFi.status() == WL_CONNECTED && client.connected()) {
      int valorAnalogico = analogRead(PINO_DADOS_PAINEL); 
      float tensao3V = (valorAnalogico * 3.39) / 4095.0; 
      float tensao12V = tensao3V * 4.0;

      char mensagem[40];
      snprintf(mensagem, sizeof(mensagem), "%.2f", tensao12V);
      
      if (!client.publish("topic/painel", mensagem)) {
        Serial.println("Falha ao publicar mensagem MQTT");
      }
    }
    
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

// Task de leitura de dados do LDR
void dadosLDRTask(void *pvParameters) {

  analogReadResolution(12); 
  analogSetAttenuation(ADC_11db);

  while (1) {
    if (WiFi.status() == WL_CONNECTED && client.connected()) {
      int valorAnalogico_ldr = analogRead(LDR); 
     float tensao3Vldr = (valorAnalogico_ldr * 4.99) / 4095.0; 
     float tensao12Vldr = tensao3Vldr * 6.0;

      char mensagem[40];
      snprintf(mensagem, sizeof(mensagem), "%.2f", tensao12Vldr);
      
      if (!client.publish("topic/ldr", mensagem)) {
        Serial.println("Falha ao publicar mensagem MQTT");
      }
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

// Task Perturba Observa LDR
void servoTask(void *pvParameters) {

  myservo.attach(pinoServo);
  leituraAnterior = analogRead(LDR);
  posServo = map(leituraAnterior, 0, 4095, limiteMin, limiteMax);

  if (posServo > limiteMax) posServo = limiteMax;
  if (posServo < limiteMin) posServo = limiteMin;
  
  while (1) {
    if (millis() - tempoAnterior >= intervaloPerturbacao) {

      posServo += direcao * passoMovimento;

      if (posServo >= limiteMax) {
        posServo = limiteMax;
        direcao = -1;
      }
      if (posServo <= limiteMin) {
        posServo = limiteMin;
        direcao = 1;
      }

      myservo.write(posServo);
      vTaskDelay(50 / portTICK_PERIOD_MS);

      int leituraAtual = analogRead(LDR);
    
      if (leituraAtual < leituraAnterior) {
        direcao *= -1;
      }

      leituraAnterior = leituraAtual;
      tempoAnterior = millis();
    }
    
    vTaskDelay(10 / portTICK_PERIOD_MS); 
  }
}


void setup() {

  // Inicializa WiFi
  if (!connectWiFi()) {
    Serial.println("Falha crítica ao conectar WiFi. Reiniciando...");
    ESP.restart();
  }

  // Configura cliente MQTT
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  client.setBufferSize(512);
  client.setSocketTimeout(10); 

// Criação da Task MQTT
xTaskCreatePinnedToCore(
    mqttTask,       
    "MQTTTask",    
    2560,          
    NULL,          
    1,              // Prioridade 
    NULL,           
    1               // Core onde a task será executada (0 ou 1)
);

// Task de leitura dos dados do Painel
xTaskCreatePinnedToCore(
   dadosPainelTask,
    "DadosLDRTask",
    2560,
    NULL,
    1, 
    NULL,
    0               
);

//Criação da Task de leitura de dados do LDR
xTaskCreatePinnedToCore(
    dadosLDRTask,
    "DadosPainelTask",
    2560,
    NULL,
    2, 
    NULL,
    0               
);  

//Criação da Task Perturba Observa Painel
xTaskCreatePinnedToCore(
    servoTask,          
    "Servo Control Task", 
    2560,
    NULL,
    3, 
    NULL,
    0               
);  

}

void loop() {

}
