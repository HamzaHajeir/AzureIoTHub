#include <Arduino.h>

#ifndef STASSID
#define STASSID "ORANGE_4"
#define STAPSK  "123456789"
#endif
#if 1

#define USE_TLS

const char *mqtt_server = "";
const char *deviceName = "";
const char *deviceSAS = "";

#include <ESP8266WiFi.h>

#include <Ticker.h>
#include <PangolinMQTT.h>
#include <string>

#define WIFI_SSID STASSID
#define WIFI_PASSWORD STAPSK

#define MQTT_HOST "iotcoursehub123.azure-devices.net"
#define MQTT_PORT 8883

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

float pload0 = 25.5f;

void onMqttConnect(bool sessionPresent)
{
    elapsed = millis();

    std::string rootTopic = "devices/";
    rootTopic += deviceName;

    std::string subscribeTopic = rootTopic += "/messages/devicebound/#";


    std::string publishTopic = "devices/";
    publishTopic += deviceName;
    publishTopic += "/messages/events/";

    Serial.printf("Connected to MQTT session=%d max payload size=%d\n", sessionPresent, mqttClient.getMaxPayloadSize());
    Serial.println("Subscribing at QoS 2");
    mqttClient.subscribe(subscribeTopic.c_str(), 2);
    mqttClient.subscribe(publishTopic.c_str(), 2);
    Serial.printf("T=%u Publishing at QoS 0\n", millis());
    mqttClient.publish(publishTopic.c_str(), 0, false, String(pload0));
}

void onMqttDisconnect(int8_t reason)
{
    Serial.printf("Disconnected from MQTT reason=%d\n", reason);

    if (WiFi.isConnected())
    {
        mqttReconnectTimer.once(2, connectToMqtt);
    }
}

void onMqttMessage(const char *topic, uint8_t *payload, struct PANGO_PROPS props, size_t len, size_t index, size_t total)
{
    Serial.printf("T=%u Message %s qos%d dup=%d retain=%d len=%d elapsed=%u\n", millis(), topic, props.qos, props.dup, props.retain, len, millis() - elapsed);
    PANGO::dumphex(payload, len, 16);
}

void setup()
{
    Serial.begin(115200);
    Serial.printf("\nPangolinMQTT v0.0.7\n");

    wifiConnectHandler = WiFi.onStationModeGotIP(onWifiConnect);
    wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);

    mqttClient.onConnect(onMqttConnect);
    mqttClient.onDisconnect(onMqttDisconnect);
    mqttClient.onMessage(onMqttMessage);
    String hubUserString = mqtt_server;
    hubUserString += "/";
    hubUserString += deviceName;
    mqttClient.setClientId(deviceName);
    mqttClient.setCredentials(hubUserString.c_str(), deviceSAS);
    mqttClient.setServer(MQTT_HOST, MQTT_PORT);
    mqttClient.setCleanSession(true);
    connectToWifi();
}

void loop() {}

#elif 0

#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
// #include <DHT.h>

/* WiFi Settings */
const char *ssid = STASSID;
const char *password = STAPSK;

/* Azure IOT Hub Settings */
const char *mqtt_server = "iotcourse2020.azure-devices.net";
const char *deviceName = "NodeMCU";
const char *deviceSAS = "SharedAccessSignature sr=iotcourse2020.azure-devices.net%2Fdevices%2FNodeMCU&sig=21ypg1MZ2Jlz8ZZXWFVOYnspv5akHPtkBDO%2BOcmEQno%3D&se=1601573349";
long interval = 15000; //(ms) - 15 seconds between reports
/* Azure IOT Hub Settings */

#define DHTTYPE DHT11 // DHT11 or DHT22
#define DHTPIN D1     //

WiFiClientSecure espClient;
PubSubClient client(espClient);
// DHT dht(DHTPIN, DHTTYPE);


const char trustRoot[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDdzCCAl+gAwIBAgIEAgAAuTANBgkqhkiG9w0BAQUFADBaMQswCQYDVQQGEwJJ
RTESMBAGA1UEChMJQmFsdGltb3JlMRMwEQYDVQQLEwpDeWJlclRydXN0MSIwIAYD
VQQDExlCYWx0aW1vcmUgQ3liZXJUcnVzdCBSb290MB4XDTAwMDUxMjE4NDYwMFoX
DTI1MDUxMjIzNTkwMFowWjELMAkGA1UEBhMCSUUxEjAQBgNVBAoTCUJhbHRpbW9y
ZTETMBEGA1UECxMKQ3liZXJUcnVzdDEiMCAGA1UEAxMZQmFsdGltb3JlIEN5YmVy
VHJ1c3QgUm9vdDCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBAKMEuyKr
mD1X6CZymrV51Cni4eiVgLGw41uOKymaZN+hXe2wCQVt2yguzmKiYv60iNoS6zjr
IZ3AQSsBUnuId9Mcj8e6uYi1agnnc+gRQKfRzMpijS3ljwumUNKoUMMo6vWrJYeK
mpYcqWe4PwzV9/lSEy/CG9VwcPCPwBLKBsua4dnKM3p31vjsufFoREJIE9LAwqSu
XmD+tqYF/LTdB1kC1FkYmGP1pWPgkAx9XbIGevOF6uvUA65ehD5f/xXtabz5OTZy
dc93Uk3zyZAsuT3lySNTPx8kmCFcB5kpvcY67Oduhjprl3RjM71oGDHweI12v/ye
jl0qhqdNkNwnGjkCAwEAAaNFMEMwHQYDVR0OBBYEFOWdWTCCR1jMrPoIVDaGezq1
BE3wMBIGA1UdEwEB/wQIMAYBAf8CAQMwDgYDVR0PAQH/BAQDAgEGMA0GCSqGSIb3
DQEBBQUAA4IBAQCFDF2O5G9RaEIFoN27TyclhAO992T9Ldcw46QQF+vaKSm2eT92
9hkTI7gQCvlYpNRhcL0EYWoSihfVCr3FvDB81ukMJY2GQE/szKN+OMY3EU/t3Wgx
jkzSswF07r51XgdIGn9w/xZchMB5hbgF/X++ZRGjD8ACtPhSNzkE1akxehi/oCr0
Epn3o0WC4zxe9Z2etciefC7IpJ5OCBRLbf1wbWsaY71k5h+3zvDyny67G7fyUIhz
ksLi4xaNmjICq44Y3ekQEe5+NauQrz4wlHrQMz2nZQ/1/I6eYs9HRCwBXbsdtTLS
R9I4LtD+gdwyah617jzV/OeBHRnDJELqYzmp
-----END CERTIFICATE-----
)EOF";
X509List cert(trustRoot);


long lastMsg = 0;
int value = 0;
void callback(char *topic, byte *payload, unsigned int length);

void LEDOn()
{
    digitalWrite(BUILTIN_LED, LOW); // Turn the LED on (Note that LOW is the voltage level
}

void LEDOff()
{
    digitalWrite(BUILTIN_LED, HIGH); // Turn the LED off (Note that HIGH is the voltage level
}

void setup()
{
    pinMode(BUILTIN_LED, OUTPUT); // Initialize the BUILTIN_LED pin as an output
    LEDOff();
    // dht.begin();
    Serial.begin(115200);
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }

    configTime(3 * 3600, 0, "pool.ntp.org", "time.nist.gov");


    espClient.setCACert((const uint8_t*)trustRoot,1262);
    Serial.println(WiFi.localIP());
    client.setServer(mqtt_server, 8883);
    client.setCallback(callback);
}




void callback(char *topic, byte *payload, unsigned int length)
{
    LEDOn();
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");
    String message;
    for (int i = 0; i < length; i++)
    {
        message += (char)payload[i];
    }
    Serial.println(message);

    LEDOff();
}

void reconnect()
{
    // Loop until we're reconnected
    while (!client.connected())
    {
        Serial.print("Attempting MQTT connection...");
        // Attempt to connect
        String hubUserString = mqtt_server;
        hubUserString += "/";
        hubUserString += deviceName;

        if (client.connect(deviceName, hubUserString.c_str(), deviceSAS))
        {
            /*Serial.println("connected");*/
            String subscribestring = "devices/";
            subscribestring += deviceName;
            subscribestring += "/messages/devicebound/#";

            client.subscribe(subscribestring.c_str());
        }
        else
        {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 5 seconds");
            // Wait 5 seconds before retrying
            delay(5000);
        }
    }
}

void publishTemprature()
{
    LEDOn();
    //  float heatIndex   = dht.computeHeatIndex(temperature, humidity);
/* 
    float humidity = dht.readHumidity();
    float temperature = dht.readTemperature(true);
    if (isnan(humidity) || isnan(temperature))
    {
        Serial.println("Failed to read from DHT sensor!");
        return;
    }
 */
    float humidity = 53.8f;
    float temperature = 16.8f;
    String json = "{ \"DeviceId\":\"";
    json += deviceName;
    json += "\", \"Temperature\":";
    json += temperature;
    json += ", \"Humidity\": ";
    json += humidity;
    json += " }";

    String publishstring = "devices/";
    publishstring += deviceName;
    publishstring += "/messages/events/";

    Serial.println(json);
    client.publish(publishstring.c_str(), json.c_str());
    LEDOff();
}

void loop()
{
    if (!client.connected())
    {
        reconnect();
    }
    client.loop();

    long now = millis();
    if (now - lastMsg > interval)
    {
        lastMsg = now;
        ++value;
        // Do the real work here
        publishTemprature();
    }
}

#elif 0

// Copyright (c) Microsoft. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

// CAVEAT: This sample is to demonstrate azure IoT client concepts only and is not a guide design principles or style
// Checking of return codes and error values shall be omitted for brevity.  Please practice sound engineering practices
// when writing production code.

// Note: PLEASE see https://github.com/Azure/azure-iot-arduino#simple-sample-instructions for detailed sample setup instructions.
// Note2: To use this sample with the esp32, you MUST build the AzureIoTSocket_WiFi library by using the make_sdk.py,
//        found in https://github.com/Azure/azure-iot-pal-arduino/tree/master/build_all. 
//        Command line example: python3 make_sdk.py -o <your-lib-folder>
#undef round
#include <AzureIoTHub.h>
#include <stdio.h>
#include <stdlib.h>


#include "iot_configs.h" // You must set your wifi SSID, wifi PWD, and your IoTHub Device Connection String in iot_configs.h
#include "sample_init.h"

#ifdef is_esp_board
  #include "Esp.h"
#endif

#ifdef SAMPLE_MQTT
    #include "AzureIoTProtocol_MQTT.h"
    #include "iothubtransportmqtt.h"
#endif // SAMPLE_MQTT
#ifdef SAMPLE_HTTP
    #include "AzureIoTProtocol_HTTP.h"
    #include "iothubtransporthttp.h"
#endif // SAMPLE_HTTP

static const char ssid[] = IOT_CONFIG_WIFI_SSID;
static const char pass[] = IOT_CONFIG_WIFI_PASSWORD;

/* Define several constants/global variables */
static const char* connectionString = DEVICE_CONNECTION_STRING;
static bool g_continueRunning = true; // defines whether or not the device maintains its IoT Hub connection after sending (think receiving messages from the cloud)
static size_t g_message_count_send_confirmations = 0;
static bool g_run_demo = true;

IOTHUB_MESSAGE_HANDLE message_handle;
size_t messages_sent = 0;
#define MESSAGE_COUNT 5 // determines the number of times the device tries to send a message to the IoT Hub in the cloud.
const char* telemetry_msg = "test_message";
const char* quit_msg = "quit";
const char* exit_msg = "exit";

IOTHUB_DEVICE_CLIENT_LL_HANDLE device_ll_handle;

static int callbackCounter;
int receiveContext = 0;

/* -- receive_message_callback --
 * Callback method which executes upon receipt of a message originating from the IoT Hub in the cloud. 
 * Note: Modifying the contents of this method allows one to command the device from the cloud. 
 */
static IOTHUBMESSAGE_DISPOSITION_RESULT receive_message_callback(IOTHUB_MESSAGE_HANDLE message, void* userContextCallback)
{
    int* counter = (int*)userContextCallback;
    const unsigned char* buffer;
    size_t size;
    const char* messageId;

    // Message properties
    if ((messageId = IoTHubMessage_GetMessageId(message)) == NULL)
    {
        messageId = "<null>";
    }

    // Message content
    if (IoTHubMessage_GetByteArray(message, (const unsigned char**)&buffer, &size) != IOTHUB_MESSAGE_OK)
    {
        LogInfo("unable to retrieve the message data\r\n");
    }
    else
    {
        LogInfo("Received Message [%d]\r\n Message ID: %s\r\n Data: <<<%s>>> & Size=%d\r\n", *counter, messageId, buffer, (int)size);
        // LogInfo("Received Message [%d]\r\n Message ID: %s\r\n Data: <<<%.*s>>> & Size=%d\r\n", *counter, messageId, (int)size, buffer, (int)size);
        // If we receive the word 'quit' then we stop running
        if (size == (strlen(quit_msg) * sizeof(char)) && memcmp(buffer, quit_msg, size) == 0)
        {
            g_continueRunning = false;
        }
    }

    /* Some device specific action code goes here... */
    (*counter)++;
    return IOTHUBMESSAGE_ACCEPTED;
}


/* -- send_confirm_callback --
 * Callback method which executes upon confirmation that a message originating from this device has been received by the IoT Hub in the cloud.
 */
static void send_confirm_callback(IOTHUB_CLIENT_CONFIRMATION_RESULT result, void* userContextCallback)
{
    (void)userContextCallback;
    // When a message is sent this callback will get envoked
    g_message_count_send_confirmations++;
    LogInfo("Confirmation callback received for message %lu with result %s\r\n", (unsigned long)g_message_count_send_confirmations, MU_ENUM_TO_STRING(IOTHUB_CLIENT_CONFIRMATION_RESULT, result));
}

/* -- connection_status_callback --
 * Callback method which executes on receipt of a connection status message from the IoT Hub in the cloud.
 */
static void connection_status_callback(IOTHUB_CLIENT_CONNECTION_STATUS result, IOTHUB_CLIENT_CONNECTION_STATUS_REASON reason, void* user_context)
{
    (void)reason;
    (void)user_context;
    // This sample DOES NOT take into consideration network outages.
    if (result == IOTHUB_CLIENT_CONNECTION_AUTHENTICATED)
    {
        LogInfo("The device client is connected to iothub\r\n");
    }
    else
    {
        LogInfo("The device client has been disconnected\r\n");
    }
}

/* -- reset_esp_helper -- 
 * waits for call of exit_msg over Serial line to reset device
 */
static void reset_esp_helper()
{
#ifdef is_esp_board
    // Read from local serial 
    if (Serial.available()){
        String s1 = Serial.readStringUntil('\n');// s1 is String type variable.
        Serial.print("Received Data: ");
        Serial.println(s1);//display same received Data back in serial monitor.

        // Restart device upon receipt of 'exit' call.
        int e_start = s1.indexOf('e');
        String ebit = (String) s1.substring(e_start, e_start+4);
        if(ebit == exit_msg)
        {
            ESP.restart();
        }
    }
#endif // is_esp_board
}

/* -- run_demo --
 * Runs active task of sending telemetry to IoTHub
 * WARNING: only call this function once, as it includes steps to destroy handles and clean up at the end.
 */
static void run_demo()
{
    int result = 0;

    // action phase of the program, sending messages to the IoT Hub in the cloud.
    do
    {
        if (messages_sent < MESSAGE_COUNT)
        {
            // Construct the iothub message from a string or a byte array
            message_handle = IoTHubMessage_CreateFromString(telemetry_msg);
            //message_handle = IoTHubMessage_CreateFromByteArray((const unsigned char*)msgText, strlen(msgText)));

            // Set Message property
            /*(void)IoTHubMessage_SetMessageId(message_handle, "MSG_ID");
            (void)IoTHubMessage_SetCorrelationId(message_handle, "CORE_ID");
            (void)IoTHubMessage_SetContentTypeSystemProperty(message_handle, "application%2fjson");
            (void)IoTHubMessage_SetContentEncodingSystemProperty(message_handle, "utf-8");*/

            // Add custom properties to message
            // (void)IoTHubMessage_SetProperty(message_handle, "property_key", "property_value");

            LogInfo("Sending message %d to IoTHub\r\n", (int)(messages_sent + 1));
            result = IoTHubDeviceClient_LL_SendEventAsync(device_ll_handle, message_handle, send_confirm_callback, NULL);
            // The message is copied to the sdk so the we can destroy it
            IoTHubMessage_Destroy(message_handle);

            messages_sent++;
        }
        else if (g_message_count_send_confirmations >= MESSAGE_COUNT)
        {
            // After all messages are all received stop running
            g_continueRunning = false;
        }

        IoTHubDeviceClient_LL_DoWork(device_ll_handle);
        ThreadAPI_Sleep(3);
        reset_esp_helper();
      
    } while (g_continueRunning);

    // Clean up the iothub sdk handle
    IoTHubDeviceClient_LL_Destroy(device_ll_handle);
    // Free all the sdk subsystem
    IoTHub_Deinit();

    LogInfo("done with sending");
    return;
}

void setup() {
  
    // Select the Protocol to use with the connection
#ifdef SAMPLE_MQTT
    IOTHUB_CLIENT_TRANSPORT_PROVIDER protocol = MQTT_Protocol;
#endif // SAMPLE_MQTT
#ifdef SAMPLE_HTTP
   IOTHUB_CLIENT_TRANSPORT_PROVIDER protocol = HTTP_Protocol;
#endif // SAMPLE_HTTP

    sample_init(ssid, pass);

    // Used to initialize IoTHub SDK subsystem
    (void)IoTHub_Init();
    // Create the iothub handle here
    device_ll_handle = IoTHubDeviceClient_LL_CreateFromConnectionString(connectionString, protocol);
    LogInfo("Creating IoTHub Device handle\r\n");

    if (device_ll_handle == NULL)
    {
        LogInfo("Error AZ002: Failure creating Iothub device. Hint: Check you connection string.\r\n");
    }
    else
    {
        // Set any option that are neccessary.
        // For available options please see the iothub_sdk_options.md documentation in the main C SDK
        // turn off diagnostic sampling
        int diag_off = 0;
        IoTHubDeviceClient_LL_SetOption(device_ll_handle, OPTION_DIAGNOSTIC_SAMPLING_PERCENTAGE, &diag_off);

#ifndef SAMPLE_HTTP
        // Example sdk status tracing for troubleshooting
        bool traceOn = true;
        IoTHubDeviceClient_LL_SetOption(device_ll_handle, OPTION_LOG_TRACE, &traceOn);
#endif // SAMPLE_HTTP

        // Setting the Trusted Certificate.
        IoTHubDeviceClient_LL_SetOption(device_ll_handle, OPTION_TRUSTED_CERT, certificates);

#if defined SAMPLE_MQTT
        //Setting the auto URL Encoder (recommended for MQTT). Please use this option unless
        //you are URL Encoding inputs yourself.
        //ONLY valid for use with MQTT
        bool urlEncodeOn = true;
        IoTHubDeviceClient_LL_SetOption(device_ll_handle, OPTION_AUTO_URL_ENCODE_DECODE, &urlEncodeOn);
        /* Setting Message call back, so we can receive Commands. */
        if (IoTHubClient_LL_SetMessageCallback(device_ll_handle, receive_message_callback, &receiveContext) != IOTHUB_CLIENT_OK)
        {
            LogInfo("ERROR: IoTHubClient_LL_SetMessageCallback..........FAILED!\r\n");
        }
#endif // SAMPLE_MQTT

        // Setting connection status callback to get indication of connection to iothub
        (void)IoTHubDeviceClient_LL_SetConnectionStatusCallback(device_ll_handle, connection_status_callback, NULL);

    }
}

void loop(void)
{
  if (g_run_demo)
  {
      run_demo();
      g_run_demo = false;
  }
  reset_esp_helper();
}


#elif 0

#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>


const char* ssid = STASSID;
const char* password = STAPSK;

const char* host = "IotCourse.servicebus.windows.net";
const int httpsPort = 443;

// DigiCert High Assurance EV Root CA
const char trustRoot[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDdzCCAl+gAwIBAgIEAgAAuTANBgkqhkiG9w0BAQUFADBaMQswCQYDVQQGEwJJ
RTESMBAGA1UEChMJQmFsdGltb3JlMRMwEQYDVQQLEwpDeWJlclRydXN0MSIwIAYD
VQQDExlCYWx0aW1vcmUgQ3liZXJUcnVzdCBSb290MB4XDTAwMDUxMjE4NDYwMFoX
DTI1MDUxMjIzNTkwMFowWjELMAkGA1UEBhMCSUUxEjAQBgNVBAoTCUJhbHRpbW9y
ZTETMBEGA1UECxMKQ3liZXJUcnVzdDEiMCAGA1UEAxMZQmFsdGltb3JlIEN5YmVy
VHJ1c3QgUm9vdDCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBAKMEuyKr
mD1X6CZymrV51Cni4eiVgLGw41uOKymaZN+hXe2wCQVt2yguzmKiYv60iNoS6zjr
IZ3AQSsBUnuId9Mcj8e6uYi1agnnc+gRQKfRzMpijS3ljwumUNKoUMMo6vWrJYeK
mpYcqWe4PwzV9/lSEy/CG9VwcPCPwBLKBsua4dnKM3p31vjsufFoREJIE9LAwqSu
XmD+tqYF/LTdB1kC1FkYmGP1pWPgkAx9XbIGevOF6uvUA65ehD5f/xXtabz5OTZy
dc93Uk3zyZAsuT3lySNTPx8kmCFcB5kpvcY67Oduhjprl3RjM71oGDHweI12v/ye
jl0qhqdNkNwnGjkCAwEAAaNFMEMwHQYDVR0OBBYEFOWdWTCCR1jMrPoIVDaGezq1
BE3wMBIGA1UdEwEB/wQIMAYBAf8CAQMwDgYDVR0PAQH/BAQDAgEGMA0GCSqGSIb3
DQEBBQUAA4IBAQCFDF2O5G9RaEIFoN27TyclhAO992T9Ldcw46QQF+vaKSm2eT92
9hkTI7gQCvlYpNRhcL0EYWoSihfVCr3FvDB81ukMJY2GQE/szKN+OMY3EU/t3Wgx
jkzSswF07r51XgdIGn9w/xZchMB5hbgF/X++ZRGjD8ACtPhSNzkE1akxehi/oCr0
Epn3o0WC4zxe9Z2etciefC7IpJ5OCBRLbf1wbWsaY71k5h+3zvDyny67G7fyUIhz
ksLi4xaNmjICq44Y3ekQEe5+NauQrz4wlHrQMz2nZQ/1/I6eYs9HRCwBXbsdtTLS
R9I4LtD+gdwyah617jzV/OeBHRnDJELqYzmp
-----END CERTIFICATE-----
)EOF";
X509List cert(trustRoot);

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  // Set time via NTP, as required for x.509 validation
  configTime(3 * 3600, 0, "pool.ntp.org", "time.nist.gov");

  Serial.print("Waiting for NTP time sync: ");
  time_t now = time(nullptr);
  while (now < 8 * 3600 * 2) {
    delay(500);
    Serial.print(".");
    now = time(nullptr);
  }
  Serial.println("");
  struct tm timeinfo;
  gmtime_r(&now, &timeinfo);
  Serial.print("Current time: ");
  Serial.print(asctime(&timeinfo));

  // Use WiFiClientSecure class to create TLS connection
  WiFiClientSecure client;
  Serial.print("Connecting to ");
  Serial.println(host);

  Serial.printf("Using certificate: %s\n", trustRoot);
  client.setTrustAnchors(&cert);

  if (!client.connect(host, httpsPort)) {
    Serial.println("Connection failed");
    return;
  }

  String url = "/repos/esp8266/Arduino/commits/master/status";
  Serial.print("Requesting URL: ");
  Serial.println(url);

  client.print(String("GET ") + url + " HTTP/1.1\r\n" +
               "Host: " + host + "\r\n" +
               "User-Agent: BuildFailureDetectorESP8266\r\n" +
               "Connection: close\r\n\r\n");

  Serial.println("Request sent");
  while (client.connected()) {
    String line = client.readStringUntil('\n');
    if (line == "\r") {
      Serial.println("Headers received");
      break;
    }
  }
  String line = client.readStringUntil('\n');
  if (line.startsWith("{\"state\":\"success\"")) {
    Serial.println("esp8266/Arduino CI successfull!");
  } else {
    Serial.println("esp8266/Arduino CI has failed");
  }
  Serial.println("Reply was:");
  Serial.println("==========");
  Serial.println(line);
  Serial.println("==========");
  Serial.println("Closing connection");
}

void loop() {
}

#else 
#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>
#include "sha256.h"
#include "Base64.h"
#include <ArduinoJson.h>

// START: Azure Evet Hub settings

const char *KEY = "9GMj6WHjUw3ySJy10xMqFN5KytaW1OOKyZ9sNj9I7Hw=";                      // event hub access key
const char *KEY_NAME = "RootManageSharedAccessKey";                  // event hub key name  ( policy name)
const char *HOST = "IotCourse.servicebus.windows.net"; // event hub name (name of service bus)
const char *END_POINT = "/test1/messages";           // name of the evnthub which we create inside eventhub namespace
// END: Azure Evet Hub settings
// START: WiFi settings
const char *SSID = "ORANGE_1";
const char *PASSWORD = "123456789";
const char* ssid = STASSID;
const char* password = STAPSK;
// END: WiFi settings
String request;
String data;
String fullSas;
WiFiClientSecure client;

const char trustRoot[] = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDdzCCAl+gAwIBAgIEAgAAuTANBgkqhkiG9w0BAQUFADBaMQswCQYDVQQGEwJJ
RTESMBAGA1UEChMJQmFsdGltb3JlMRMwEQYDVQQLEwpDeWJlclRydXN0MSIwIAYD
VQQDExlCYWx0aW1vcmUgQ3liZXJUcnVzdCBSb290MB4XDTAwMDUxMjE4NDYwMFoX
DTI1MDUxMjIzNTkwMFowWjELMAkGA1UEBhMCSUUxEjAQBgNVBAoTCUJhbHRpbW9y
ZTETMBEGA1UECxMKQ3liZXJUcnVzdDEiMCAGA1UEAxMZQmFsdGltb3JlIEN5YmVy
VHJ1c3QgUm9vdDCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBAKMEuyKr
mD1X6CZymrV51Cni4eiVgLGw41uOKymaZN+hXe2wCQVt2yguzmKiYv60iNoS6zjr
IZ3AQSsBUnuId9Mcj8e6uYi1agnnc+gRQKfRzMpijS3ljwumUNKoUMMo6vWrJYeK
mpYcqWe4PwzV9/lSEy/CG9VwcPCPwBLKBsua4dnKM3p31vjsufFoREJIE9LAwqSu
XmD+tqYF/LTdB1kC1FkYmGP1pWPgkAx9XbIGevOF6uvUA65ehD5f/xXtabz5OTZy
dc93Uk3zyZAsuT3lySNTPx8kmCFcB5kpvcY67Oduhjprl3RjM71oGDHweI12v/ye
jl0qhqdNkNwnGjkCAwEAAaNFMEMwHQYDVR0OBBYEFOWdWTCCR1jMrPoIVDaGezq1
BE3wMBIGA1UdEwEB/wQIMAYBAf8CAQMwDgYDVR0PAQH/BAQDAgEGMA0GCSqGSIb3
DQEBBQUAA4IBAQCFDF2O5G9RaEIFoN27TyclhAO992T9Ldcw46QQF+vaKSm2eT92
9hkTI7gQCvlYpNRhcL0EYWoSihfVCr3FvDB81ukMJY2GQE/szKN+OMY3EU/t3Wgx
jkzSswF07r51XgdIGn9w/xZchMB5hbgF/X++ZRGjD8ACtPhSNzkE1akxehi/oCr0
Epn3o0WC4zxe9Z2etciefC7IpJ5OCBRLbf1wbWsaY71k5h+3zvDyny67G7fyUIhz
ksLi4xaNmjICq44Y3ekQEe5+NauQrz4wlHrQMz2nZQ/1/I6eYs9HRCwBXbsdtTLS
R9I4LtD+gdwyah617jzV/OeBHRnDJELqYzmp
-----END CERTIFICATE-----
)EOF";
X509List cert(trustRoot);

void setup()
{

    Serial.begin(115200);
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    // Set time via NTP, as required for x.509 validation
    
    configTime(3 * 3600, 0, "pool.ntp.org", "time.nist.gov");
/* 
    Serial.print("Waiting for NTP time sync: ");
    time_t now = time(nullptr);
    while (now < 8 * 3600 * 2)
    {
        delay(500);
        Serial.print(".");
        now = time(nullptr);
    }
    Serial.println("");
    struct tm timeinfo;
    gmtime_r(&now, &timeinfo);
    Serial.print("Current time: ");
    Serial.print(asctime(&timeinfo));
 */


    // START: Naive URL Encode
    String url = "https://" + (String)HOST + (String)END_POINT;
    url.replace(":", "%3A");
    url.replace("/", "%2F");
    Serial.println(url);
    // END: Naive URL Encode

    // START: Create SAS
    // https://azure.microsoft.com/en-us/documentation/articles/service-bus-sas-overview/
    // Where to get secods since the epoch: local service, SNTP, RTC
    int expire = 1711104241;
    String stringToSign = url + "\n" + expire;

    // START: Create signature
    Sha256.initHmac((const uint8_t *)KEY, 44);
    Sha256.print(stringToSign);
    char *sign = (char *)Sha256.resultHmac();

    //Serial.println(String (Sha256.resultHmac));
    int signLen = 32;
    // END: Create signature

    // START: Get base64 of signature
      
  int encodedSignLen = base64_enc_len(signLen);
  char encodedSign[encodedSignLen];
  base64_encode(encodedSign, sign, signLen); 
  String encodedSas = (String) encodedSign;

    // int encodedSignLen = (signLen + 2 - ((signLen + 2) % 3)) / 3 * 4;
    // String encodedSas = base64::encode(sign,encodedSignLen);
    // Naive URL encode
    encodedSas.replace("=", "%3D");
    //Serial.println(encodedSas);
    // END: Get base64 of signature

    // SharedAccessSignature
    fullSas = "sr=" + url + "&sig=" + encodedSas + "&se=" + expire + "&skn=" + KEY_NAME;
    // END: create SAS
    //Serial.println("SAS below");
    //Serial.println(fullSas);
    //Serial.println();
    // START: Wifi connection
    // END: Wifi connection

    // client.setCACert((const uint8_t*)trustRoot,1262);


    Serial.print("Connecting to ");
    Serial.println(HOST);
    Serial.printf("Using certificate: %s\n", trustRoot);
}

void loop()
{

    client.setTrustAnchors(&cert);

    if (!client.connect(HOST, 443))
    {
        Serial.println("connection failed");
        return;
    }
    int r = random(100, 10000);
    float temp = random(25, 45);
    int soilm = random(10, 70);
    const size_t capacity = JSON_OBJECT_SIZE(3);
    DynamicJsonDocument doc(capacity);

    doc["temp"] = temp;
    doc["humidity"] = soilm;

    serializeJson(doc, Serial);
    serializeJson(doc,data);

    request = String("POST ") + END_POINT + " HTTP/1.1\r\n" +
              "Host: " + HOST + "\r\n" +
              "Authorization: SharedAccessSignature " + fullSas + "\r\n" +
              "Content-Type: application/atom+xml;type=entry;charset=utf-8\r\n" +
              "Content-Length: " + data.length() + "\r\n\r\n" +
              data;
    Serial.println(request);
    client.print(request);
    data.clear();
    delay(100);

/*     String response = "";
    while (client.connected())
    {
        response = client.readString();
    }

    Serial.println();
    Serial.print("Response code: ");
    Serial.println(response.substring(9, 12)); */
}
#endif