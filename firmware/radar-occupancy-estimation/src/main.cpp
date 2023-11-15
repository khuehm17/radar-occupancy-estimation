
/**************************************************************************************************
 *                                          Include
 **************************************************************************************************/
#include <Arduino.h>

#include <WiFi.h>
#include <AsyncMqttClient.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/timers.h>

#include <DFRobot_mmWave_Radar.h>

/**************************************************************************************************
 *                                      Macro Definition
 **************************************************************************************************/
// #define WIFI_SSID "HCMUS-VLDT-SV"
// #define WIFI_PASSWORD "svvldt38300595"
#define WIFI_SSID           "DOM-COFFEE 2"
#define WIFI_PASSWORD       "294221550"

#define MQTT_HOST           IPAddress(192, 168, 0, 105)
#define MQTT_SERVER         "test.mosquitto.org"
#define MQTT_PORT           1883
#define MQTT_DATA_TOPIC     "/dpeeroedata"
#define MQTT_CTRL_TOPIC     "/dpeeroectrl"

#define STATE_IDLE          0
#define STATE_CONNECTED     1

#define NO_ONE_DETECTED     0
#define PRESENCE_DETECTED   1

/**************************************************************************************************
 *                                     Global declaration
 **************************************************************************************************/
void connectToWifi();
void connectToMqtt();
void WiFiEvent(WiFiEvent_t event);
void connectToMqtt();
void WiFiEvent(WiFiEvent_t event);
void onMqttDisconnect(AsyncMqttClientDisconnectReason reason);
void onMqttSubscribe(uint16_t packetId, uint8_t qos);
void onMqttUnsubscribe(uint16_t packetId);
void onMqttPublish(uint16_t packetId);
void onMqttMessage(char *topic, char *payload, 
                    AsyncMqttClientMessageProperties properties, 
                    size_t len, size_t index, size_t total);
void onMqttPublish(uint16_t packetId);

/* RTOS tasks */
void led_blink(void *pvParameter);
void mqtt_sayHello(void *pvParameter);
void mqtt_mmWaveRadar_Public(void *pvParameter);
void onControlMessage(char * ctrlMsg, uint16_t len);

/* Variables for Wifi and MQTT */
int g_state = STATE_IDLE;
AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;
void onMqttConnect(bool sessionPresent);

char mmWaveSensorDataBuffer[50];
char ctrlMessageDataBuffer[50];

/* Variable for mmWaveSensor */
DFRobot_mmWave_Radar mmWaveSensor(&Serial2);

/**************************************************************************************************
 *                                      Main application
 **************************************************************************************************/
void setup()
{
    pinMode(BUILTIN_LED, OUTPUT);
    /* Start UART interface for Log terminal */
    Serial.begin(115200);

    /* Init mmWave Radar sensor*/
    /* Start UART interface for mmWave Radar sensor */
    Serial2.begin(115200);
    /* Config mmWave Radar sensor */
    mmWaveSensor.factoryReset();        // Restore to the factory settings 
    mmWaveSensor.DetRangeCfg(0, 4);     // The detection range is as far as 9m
    mmWaveSensor.OutputLatency(0, 0);

    /* Init Wifi connection */
    Serial.println("Initializing...");
    wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), 
                                        pdFALSE, (void *)0, 
                                        reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));
    WiFi.onEvent(WiFiEvent);
    /* Start Wifi connection */
    connectToWifi();
   
    /* Init MQTT */
    mqttReconnectTimer = xTimerCreate("mqttTimer", 
                                    pdMS_TO_TICKS(2000), pdFALSE, 
                                    (void *)0, 
                                    reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
    mqttClient.onConnect(onMqttConnect);
    mqttClient.onDisconnect(onMqttDisconnect);
    mqttClient.onSubscribe(onMqttSubscribe);
    mqttClient.onUnsubscribe(onMqttUnsubscribe);
    mqttClient.onMessage(onMqttMessage);
    // mqttClient.onPublish(onMqttPublish);
    mqttClient.setServer(MQTT_SERVER, MQTT_PORT);

    /* Start Say hello task */
    // xTaskCreate(&mqtt_sayHello, "mqtt_sayHello", 2048, NULL, 5, NULL);
    xTaskCreate(&mqtt_mmWaveRadar_Public, "mqtt_mmWaveRadar_Public", 2048, NULL, 5, NULL);
}

void loop()
{
    ;
}

/**************************************************************************************************
 *	                                    Task functions
 **************************************************************************************************/
void mqtt_sayHello(void *pvParameter)
{
    int msgCount = 0;
    while (1)
    {
        if (g_state == STATE_CONNECTED)
        {
            sprintf(mmWaveSensorDataBuffer, "Hello world %d", msgCount);
            mqttClient.publish(MQTT_DATA_TOPIC, 2, true, mmWaveSensorDataBuffer);
            Serial.println(mmWaveSensorDataBuffer);
            msgCount ++;
        }
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
}

void mqtt_mmWaveRadar_Public(void *pvParameter)
{
    int msgCount = 0;
    uint8_t humanPresenseDectVal = NO_ONE_DETECTED;

    while (1)
    {
        if (g_state == STATE_CONNECTED)
        {
            // humanPresenseDectVal = mmWaveSensor.readPresenceDetection();

            if (humanPresenseDectVal == NO_ONE_DETECTED)
            {
                sprintf(mmWaveSensorDataBuffer, "[mmWaveMsg%d] No one detected!", msgCount);
            } else
            {
                sprintf(mmWaveSensorDataBuffer, "[mmWaveMsg%d] Human Presence detected!", msgCount);
            }
            
            mqttClient.publish(MQTT_DATA_TOPIC, 2, true, mmWaveSensorDataBuffer);
            // Serial.println(mmWaveSensorDataBuffer);
            msgCount ++;
        }
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
}

void onControlMessage(char * ctrlMsg, uint16_t len)
{
    Serial.println("\n on Control message");
    memcpy(ctrlMessageDataBuffer, ctrlMsg, len);
    Serial.println(ctrlMessageDataBuffer);
    if (strcmp(ctrlMessageDataBuffer, "toggleled") == 0)
    {
        digitalWrite(BUILTIN_LED, !digitalRead(BUILTIN_LED));
    }
}

/**************************************************************************************************
 *	                                    Sub-Functions
 **************************************************************************************************/
void connectToWifi()
{
    Serial.println("Connecting to Wi-Fi...");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt()
{
    Serial.println("Connecting to MQTT...");
    mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event)
{
    Serial.printf("[WiFi-event] event: %d\n", event);
    switch (event)
    {
    case SYSTEM_EVENT_STA_GOT_IP:
        Serial.println("WiFi connected");
        Serial.println("IP address: ");
        Serial.println(WiFi.localIP());
        connectToMqtt();
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        Serial.println("WiFi lost connection");
        /* ensure we don't reconnect to MQTT while reconnecting to Wi-Fi */
        xTimerStop(mqttReconnectTimer, 0);
        xTimerStart(wifiReconnectTimer, 0);
        break;
    }
}

void onMqttConnect(bool sessionPresent)
{
    Serial.println("Connected to MQTT.");
    Serial.print("Session present: ");
    Serial.println(sessionPresent);
    g_state = STATE_CONNECTED;

    uint16_t packetIdSub = mqttClient.subscribe(MQTT_CTRL_TOPIC, 2);
    Serial.print("Subscribing at QoS 2, packetId: ");
    Serial.println(packetIdSub);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason)
{
    g_state = STATE_IDLE;
    Serial.println("Disconnected from MQTT.");

    if (WiFi.isConnected())
    {
        xTimerStart(mqttReconnectTimer, 0);
    }
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos)
{
    Serial.println("Subscribe acknowledged.");
    Serial.print("  packetId: ");
    Serial.println(packetId);
    Serial.print("  qos: ");
    Serial.println(qos);
}

void onMqttUnsubscribe(uint16_t packetId)
{
    Serial.println("Unsubscribe acknowledged.");
    Serial.print("  packetId: ");
    Serial.println(packetId);
}

void onMqttMessage(char *topic, char *payload, AsyncMqttClientMessageProperties properties, 
                    size_t len, size_t index, size_t total)
{
    Serial.println("Message received.");
    Serial.print("topic: ");
    Serial.print(topic);
    Serial.print("\t len: ");
    Serial.print(len);
    // Serial.print("\t qos: ");
    // Serial.print(properties.qos);
    // Serial.print("\t dup: ");
    // Serial.print(properties.dup);
    // Serial.print("\t retain: ");
    // Serial.print(properties.retain);
    // Serial.print("\t index: ");
    // Serial.print(index);
    // Serial.print("\t total: ");
    // Serial.print(total);
    // Serial.print("\t payload: ");
    // Serial.print(payload);
    // Serial.print("\t EoM.");

    if (strcmp(topic, MQTT_CTRL_TOPIC) == 0)
    {
        onControlMessage(payload, len);
    }
    
}

void onMqttPublish(uint16_t packetId)
{
    Serial.println("Publish acknowledged.");
    Serial.print("  packetId: ");
    Serial.println(packetId);
}


   // WiFi.mode(WIFI_AP_STA);
    // /* start SmartConfig */
    // WiFi.beginSmartConfig();
    
    // /* Wait for SmartConfig packet from mobile */
    // Serial.println("Waiting for SmartConfig.");
    // while (!WiFi.smartConfigDone()) 
    // {
    //     delay(500);
    //     Serial.print(".");
    // }
    // Serial.println("");
    // Serial.println("SmartConfig done.");
    
    // /* Wait for WiFi to connect to AP */
    // Serial.println("Waiting for WiFi");
    // while (WiFi.status() != WL_CONNECTED) {
    //     delay(500);
    //     Serial.print(".");
    // }
    // Serial.println("WiFi Connected.");
    // Serial.print("IP Address: ");
    // Serial.println(WiFi.localIP());