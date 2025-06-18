#include <WiFi.h>
#include <PubSubClient.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Configurações de rede e MQTT
const char* ssid = "API_001";
const char* password = "SOLARtracker";
const char* mqtt_server = "10.0.0.155";
const int mqtt_port = 1883;
const char* mqtt_user = "solar";       
const char* mqtt_password = "tracker";       

// Variáveis globais com proteção
volatile float hora = 0.0;
volatile float minuto = 0.0;
volatile float segundo = 0.0;
char horaminutosegundo[20] = {0};

WiFiClient espClient;
PubSubClient client(espClient);

unsigned long lastReconnectAttempt = 0;
const unsigned long reconnectInterval = 5000;

// Callback para mensagens MQTT recebidas
void callback(char* topic, byte* payload, unsigned int length) {
  // Buffer seguro para a mensagem
  char message[50] = {0};
  unsigned int copyLength = length < sizeof(message)-1 ? length : sizeof(message)-1;
  strncpy(message, (char*)payload, copyLength);
  message[copyLength] = '\0';

  // Processa os tópicos recebidos
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

// Conectar WiFi com timeout
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

// Reconexão MQTT com proteção
bool reconnectMQTT() {
  if (!client.connected()) {
    Serial.println("Tentando conexão MQTT...");
    
    if (client.connect("ESP32Client", mqtt_user, mqtt_password)) {
      Serial.println("Conectado ao broker MQTT!");
      
      // Subscreve com QoS 1 para maior confiabilidade
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

// Task MQTT com proteção
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

// Task de leitura do painel com proteção
void dadosPainelTask(void *pvParameters) {
  analogReadResolution(12); 
  analogSetAttenuation(ADC_11db);

  while (1) {
    if (WiFi.status() == WL_CONNECTED && client.connected()) {
      int valorAnalogico = analogRead(PINO_DADOS_PAINEL); 
      float tensao3V = (valorAnalogico * 3.39) / 4095.0; 
      float tensao12V = tensao3V * 4.0;

      char mensagem[20];
      snprintf(mensagem, sizeof(mensagem), "%.2f", tensao12V);
      
      if (!client.publish("topic/painel", mensagem)) {
        Serial.println("Falha ao publicar mensagem MQTT");
      }
    }
    
    vTaskDelay(1000 / portTICK_PERIOD_MS);
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

  // Cria tasks com stacks adequados
  xTaskCreate(
    mqttTask,
    "MQTTTask",
    3072, // Stack reduzida
    NULL,
    2, // Prioridade média
    NULL
  );

  xTaskCreate(
    dadosPainelTask,
    "DadosPainelTask",
    2048, // Stack reduzida
    NULL,
    1, // Prioridade mais baixa
    NULL
  );


}

void loop() {

}
