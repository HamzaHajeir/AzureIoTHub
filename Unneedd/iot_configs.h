// Copyright (c) Microsoft. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef IOT_CONFIGS_H_
#define IOT_CONFIGS_H_

/**
 * WiFi setup
 */
#define IOT_CONFIG_WIFI_SSID            "ORANGE_1"
#define IOT_CONFIG_WIFI_PASSWORD        "123456789"

/**
 * IoT Hub Device Connection String setup
 * Find your Device Connection String by going to your Azure portal, creating (or navigating to) an IoT Hub, 
 * navigating to IoT Devices tab on the left, and creating (or selecting an existing) IoT Device. 
 * Then click on the named Device ID, and you will have able to copy the Primary or Secondary Device Connection String to this sample.
 */
#define DEVICE_CONNECTION_STRING    "HostName=iotcourse2020.azure-devices.net;DeviceId=NodeMCU;SharedAccessKey=xHpTg1AmGR/T6FYEhPZe6INeziirLUP6lGifunx5SOo="

// The protocol you wish to use should be uncommented
//
#define SAMPLE_MQTT
//#define SAMPLE_HTTP

#endif /* IOT_CONFIGS_H_ */
