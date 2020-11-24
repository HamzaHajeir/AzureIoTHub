#define USE_TLS
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <Ticker.h>
#include <PangolinMQTT.h>
#include <string>


#define WIFI_SSID "YourSSID"
#define WIFI_PASSWORD "YourPASS"

#define MQTT_HOST "[yourIoTHub].azure-devices.net"
#define MQTT_PORT 8883
const char *mqtt_server = "[yourIoTHub].azure-devices.net";
const char *deviceName = "Device-ID";
const char *deviceSAS = "SharedAccessSignature sr=[yourIoTHub].azure-devices.net....";


#define RECONNECT_DELAY_W 5

uint32_t elapsed = 0;

PangolinMQTT mqttClient;
Ticker mqttReconnectTimer;
Ticker wifiReconnectTimer;

WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;

void connectToMqtt()
{
    Serial.println("Connecting to MQTT...");

    const uint8_t cert[20] = {0x5C, 0xB9, 0xEE, 0xC6, 0xE4, 0x5D, 0x0A, 0x69, 0x3A, 0x7A, 0xCA, 0x76, 0x83, 0x87, 0xC7, 0x87, 0xA5, 0x44, 0xB7, 0x03};
    // const uint8_t cert[20] = {0xD4, 0xDE, 0x20, 0xD0, 0x5E, 0x66, 0xFC, 0x53, 0xFE, 0x1A, 0x50, 0x88, 0x2C, 0x78, 0xDB, 0x28, 0x52, 0xCA, 0xE4, 0x74};
    mqttClient.serverFingerprint(cert);
    mqttClient.connect();
}
void connectToWifi()
{
    Serial.println("Connecting to Wi-Fi...");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void onWifiConnect(const WiFiEventStationModeGotIP &event)
{
    Serial.println("Connected to Wi-Fi.");
    connectToMqtt();
}

void onWifiDisconnect(const WiFiEventStationModeDisconnected &event)
{
    Serial.println("Disconnected from Wi-Fi.");
    mqttReconnectTimer.detach(); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
    wifiReconnectTimer.once(RECONNECT_DELAY_W, connectToWifi);
}

float pload0 = 0.0f;
bool mqttConnected=false;

void onMqttConnect(bool sessionPresent)
{
    elapsed = millis();

    // std::string rootTopic = "devices/";
    // rootTopic += deviceName;

    // std::string subscribeTopic = rootTopic += "/messages/devicebound/#";


    std::string publishTopic = "devices/";
    publishTopic += deviceName;
    publishTopic += "/messages/events/";
    // mqttClient.subscribe(subscribeTopic.c_str(), 2);
    // mqttClient.subscribe(publishTopic.c_str(), 2);

    Serial.printf("Connected to MQTT session=%d max payload size=%d\n", sessionPresent, mqttClient.getMaxPayloadSize());
    // Serial.println("Subscribing at QoS 2");
    Serial.printf("T=%u Publishing at QoS 0\n", millis());

    
    mqttClient.publish(publishTopic.c_str(), 0, false, String(pload0));
    mqttConnected = true;
}

void onMqttDisconnect(int8_t reason)
{
    Serial.printf("Disconnected from MQTT reason=%d\n", reason);

    if (WiFi.isConnected())
    {
        mqttReconnectTimer.once(2, connectToMqtt);
    }

    mqttConnected = false;
}

void setup()
{
    Serial.begin(115200);
    Serial.printf("\nPangolinMQTT v0.0.7\n");

    wifiConnectHandler = WiFi.onStationModeGotIP(onWifiConnect);
    wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);

    mqttClient.onConnect(onMqttConnect);
    mqttClient.onDisconnect(onMqttDisconnect);

    String hubUserString = mqtt_server;
    hubUserString += "/";
    hubUserString += deviceName;
    
    mqttClient.setClientId(deviceName);
    mqttClient.setCredentials(hubUserString.c_str(), deviceSAS);
    mqttClient.setServer(MQTT_HOST, MQTT_PORT);
    mqttClient.setCleanSession(true);
    connectToWifi();
}

void loop() {

    static uint32_t prevmillis = 0 ;
    if(millis() - prevmillis >= 10000 && mqttConnected)
    {
        float voltage = analogRead(A0) * 3.3/1023;
        float temperature = voltage / 0.01 ;
        Serial.println(temperature);

        std::string publishTopic = "devices/";
        publishTopic += deviceName;
        publishTopic += "/messages/events/Temperature";

        mqttClient.publish(publishTopic.c_str(),0,false,String(temperature));
        
        prevmillis = millis();
    }

}
