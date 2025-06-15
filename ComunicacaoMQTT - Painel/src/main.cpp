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

String horaminutosegundo = "";
float hora = 0.0;
float minuto = 0.0;
float segundo = 0.0;

WiFiClient espClient;
PubSubClient client(espClient);

unsigned long lastReconnectAttempt = 0;
const unsigned long reconnectInterval = 5000; 

// Callback para mensagens MQTT recebidas
void callback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  // Processa os tópicos recebidos
  if (String(topic) == "topic/horaminutosegundo") {
    horaminutosegundo = message;
    Serial.println("HoraMinutoSegundo recebido: " + horaminutosegundo);
  }
  else if (String(topic) == "topic/hora") {
    hora = message.toFloat();
    Serial.println("Hora recebida: " + String(hora));
  }
  else if (String(topic) == "topic/minuto") {
    minuto = message.toFloat();
    Serial.println("Minuto recebido: " + String(minuto));
  }
  else if (String(topic) == "topic/segundo") {
    segundo = message.toFloat();
    Serial.println("Segundo recebido: " + String(segundo));
  }
}

// Task para conectar WiFi 
void connectWiFi() {
  Serial.begin(115200);
  Serial.println("\nConectando a " + String(ssid));
  
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(500 / portTICK_PERIOD_MS);
    Serial.print(".");
  }
  
  Serial.println("\nWiFi conectado. IP: " + WiFi.localIP().toString());
}

// Task para reconexão MQTT
bool reconnectMQTT() {
  if (!client.connected()) {
    Serial.println("Tentando conexão MQTT...");
    
    if (client.connect("ESP32Client", mqtt_user, mqtt_password)) {
      Serial.println("Conectado ao broker MQTT!");
      client.subscribe("topic/horaminutosegundo");
      client.subscribe("topic/hora");
      client.subscribe("topic/minuto");
      client.subscribe("topic/segundo");
      return true;
    } else {
      Serial.println("Falha na conexão. Código: " + String(client.state()));
      return false;
    }
  }
  return true;
}

// Task para gerenciar conexão MQTT
void mqttTask(void *pvParameters) {
  while (1) {
    if (!client.connected() && millis() - lastReconnectAttempt > reconnectInterval) {
      lastReconnectAttempt = millis();
      reconnectMQTT();
    }
    client.loop();
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// Lê a tensão do painel
void dadosPainelTask(void *pvParameters) {
  
  analogReadResolution(12); 
  analogSetAttenuation(ADC_11db);

  while (1) {
    int valorAnalogico = analogRead(PINO_DADOS_PAINEL); 
    float tensao3V = (valorAnalogico * 3.58) / 4095.0; 
    float tensao12V = tensao3V * 4.0;

    if (client.connected()) {
      String mensagem = String(tensao12V, 2);
      client.publish("topic/painel", mensagem.c_str());
    }
    
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void setup() {
  connectWiFi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  // Cria tasks no FreeRTOS
  xTaskCreate(
    mqttTask,
    "MQTTTask",
    4096,
    NULL,
    2,
    NULL
  );

  xTaskCreate(
    dadosPainelTask,
    "DadosPainelTask",
    4096,
    NULL,
    1,
    NULL
  );
}

void loop() {
  
}