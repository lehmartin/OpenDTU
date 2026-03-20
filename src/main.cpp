// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2022-2025 Thomas Basler and others
 */
#include "Configuration.h"
#include "Datastore.h"
#include "Display_Graphic.h"
#include "I18n.h"
#include "InverterSettings.h"
#include "Led_Single.h"
#include "Logging.h"
#include "MessageOutput.h"
#include "MqttHandleDtu.h"
#include "MqttHandleHass.h"
#include "MqttHandleInverter.h"
#include "MqttHandleInverterTotal.h"
#include "MqttSettings.h"
#include "NetworkSettings.h"
#include "NtpSettings.h"
#include "PinMapping.h"
#include "RestartHelper.h"
#include "Scheduler.h"
#include "SunPosition.h"
#include "Utils.h"
#include "WebApi.h"
#include "defaults.h"
#include <Arduino.h>
#include <LittleFS.h>
#include <SpiManager.h>
#include <TaskScheduler.h>
#include <esp_heap_caps.h>

#undef TAG
static const char* TAG = "main";

#define DEBUG 0

unsigned long millisSinceLastRequest;
unsigned long millisSinceLastResponse;
bool foundClientAddress = false;
int clientAddress = 1; // subtract 1 when using as index for array access
int commandIndex = 0;
unsigned long pollingIntv = 500;
int lastCommandIndex = 3;
int bytesReadSinceLastRequest = 0;

byte commandByte1[7] = { 0xF8, 0x13, 0x13, 0x13, 0x13, 0x13, 0x13 };
byte commandByte2[7] = { 0x0B, 0x2A, 0x1C, 0x02, 0x00, 0x20, 0x40 };
byte commandByte3[7] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
byte commandByte4[7] = { 0x01, 0x18, 0x04, 0x0A, 0x20, 0x20, 0x0A };
byte crcByte1[7] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
byte crcByte2[7] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

byte response[80];

extern float batteryVoltage = 48;
extern float batteryCurrent = 0;
extern long cellVoltage[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
extern long cellTemperature[4] = { 0, 0, 0, 0 };
extern int8_t bmsTemperature = 0;
extern int8_t SOC = 99;
extern int8_t cycles = 0;

int counter = 0;

byte StrtoByte(String str_value)
{
    char char_buff[3];
    str_value.toCharArray(char_buff, 3);
    byte byte_value = strtoul(char_buff, NULL, 16);
    return byte_value;
}

String ModRTU_CRC(String raw_msg_data)
{
    byte raw_msg_data_byte[raw_msg_data.length() / 2];
    // Convert the raw_msg_data to a byte array raw_msg_data
    for (int i = 0; i < raw_msg_data.length() / 2; i++) {
        raw_msg_data_byte[i] = StrtoByte(raw_msg_data.substring(2 * i, 2 * i + 2));
    }
    // Calc the raw_msg_data_byte CRC code
    uint16_t crc = 0xFFFF;
    String crc_string = "";
    for (int pos = 0; pos < raw_msg_data.length() / 2; pos++) {
        crc ^= (uint16_t)raw_msg_data_byte[pos]; // XOR byte into least sig. byte of crc
        for (int i = 8; i != 0; i--) { // Loop over each bit
            if ((crc & 0x0001) != 0) { // If the LSB is set
                crc >>= 1; // Shift right and XOR 0xA001
                crc ^= 0xA001;
            } else // Else LSB is not set
                crc >>= 1; // Just shift right
        }
    }
    // Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes)
    // Become crc byte to a capital letter String
    crc_string = String(crc, HEX);
    crc_string.toUpperCase();
    // The crc should be like XXYY. Add zeros if need it
    if (crc_string.length() == 1) {
        crc_string = "000" + crc_string;
    } else if (crc_string.length() == 2) {
        crc_string = "00" + crc_string;
    } else if (crc_string.length() == 3) {
        crc_string = "0" + crc_string;
    } else {
        // OK
    }
    // Invert the byte positions
    crc_string = crc_string.substring(2, 4) + crc_string.substring(0, 2);
    return crc_string;
}

long responseToLong(int _index)
{
    char _rawChar[4];
    sprintf(_rawChar, "%.2X%.2X", response[_index], response[_index + 1]);
    String _rawString(_rawChar);
    int _rawInt = strtol(_rawString.c_str(), 0, 16);
    return _rawInt;
}

long responseToCurrent(int _index)
{
    char _rawChar[2];
    sprintf(_rawChar, "%.2X", response[_index + 1]);
    String _rawString(_rawChar);
    int _rawInt = strtol(_rawString.c_str(), 0, 16);
    byte b = response[_index] + 2;
    int base = ((int)(b)-2) * (-255);
    int current = base - _rawInt;

    MessageOutput.println("Current values:");
    MessageOutput.println(response[_index + 0], DEC);
    MessageOutput.println(response[_index + 1], DEC);
    return current;
}

void calculateCRCs(int _clientAddress)
{
    for (int i = 0; i < 7; i++) {
        char rawChar[16];
        sprintf(rawChar, "%.2X%.2X%.2X%.2X%.2X%.2X", _clientAddress, 3, commandByte1[i], commandByte2[i], commandByte3[i], commandByte4[i]);
        String rawString(rawChar);
        String crc = ModRTU_CRC(rawChar);
        crcByte1[i] = StrtoByte(crc.substring(0, 2));
        crcByte2[i] = StrtoByte(crc.substring(2, 4));
    }
}

void sendCommand(int _commandIndex)
{
    if (DEBUG) {
        Serial.println("Sending command:");
        Serial.println(clientAddress, HEX);
        Serial.println(3, HEX);
        Serial.println(commandByte1[_commandIndex], HEX);
        Serial.println(commandByte2[_commandIndex], HEX);
        Serial.println(commandByte3[_commandIndex], HEX);
        Serial.println(commandByte4[_commandIndex], HEX);
        Serial.println(crcByte1[_commandIndex], HEX);
        Serial.println(crcByte2[_commandIndex], HEX);
        Serial.println();
        Serial.println();
    }

    Serial1.write(clientAddress);
    Serial1.write(3);
    Serial1.write(commandByte1[_commandIndex]);
    Serial1.write(commandByte2[_commandIndex]);
    Serial1.write(commandByte3[_commandIndex]);
    Serial1.write(commandByte4[_commandIndex]);
    Serial1.write(crcByte1[_commandIndex]);
    Serial1.write(crcByte2[_commandIndex]);
}

void parseResponse(int _commandIndex)
{
    if (DEBUG) {
        Serial.println("Parsing response for command with index " + (String)_commandIndex);
    }
    int8_t SOC_temp;
    float batteryVoltage_temp;
    float batteryCurrent_temp;
    int8_t bmsTemperature_temp;
    int8_t cycles_temp;

    switch (_commandIndex) {
    case 1:
        Serial.println("CELL VOLTAGES");
        for (int i = 0; i < 16; i++) {
            cellVoltage[i] = responseToLong(i * 2 + 3);
            if (DEBUG)
                Serial.println("Cell #" + (String)(i + 1) + ": " + cellVoltage[i] + " mV");
        }
        Serial.println("Cell temperatures");
        for (int i = 0; i < 4; i++) {
            cellTemperature[i] = responseToLong(35 + i * 2);
            if (DEBUG)
                Serial.println("Cell #" + (String)(i + 1) + ": " + cellTemperature[i] + " " + (char)176 + "C");
        }
        break;
    case 3:

        batteryVoltage_temp = 0.01 * (float)responseToLong(11);
        batteryCurrent = 0.1 * ((float)responseToCurrent(13));
        bmsTemperature = responseToLong(19);
        SOC_temp = responseToLong(21);

        if (abs(batteryVoltage - batteryVoltage_temp) < 10 && batteryVoltage_temp != 0) {
            batteryVoltage = batteryVoltage_temp;
        } else {
        }

        if (abs(SOC - SOC_temp) < 5 || counter >= 10) {
            SOC = SOC_temp;
            counter = 0;
        } else {
            counter++;
        }

        if (DEBUG) {
            Serial.println("BATTERY STATS");
            Serial.println("Battery voltage: " + (String)batteryVoltage + " V");
            Serial.println("Battery current: " + (String)batteryCurrent + " A");
            Serial.println("BMS temperature: " + (String)bmsTemperature + " " + (char)176 + "C");
            Serial.println("State of charge (SoC): " + (String)SOC + "%");
        }
        break;

    case 4:
        // MessageOutput.println("Command #5");
        cycles_temp = responseToLong(29);
        if ((abs(cycles_temp - cycles) <= 1) || cycles_temp != 0) {
            cycles = cycles_temp;
        }
        // MessageOutput.println(cycles);
        // for (int i = 0; i < 80; i++) {
        //     MessageOutput.println(responseToLong(i));
        // }
        break;

    case 5:
        // MessageOutput.println("Command #6");
        // for (int i = 0; i < 80; i++) {
        //     MessageOutput.println(responseToLong(i));
        // }
        break;

    case 6:
        // MessageOutput.println("Command #7");
        // for (int i = 0; i < 80; i++) {
        //     MessageOutput.println(responseToLong(i));
        // }
        break;
    default:
        break;
    }
}

bool scanClients()
{
    for (int i = 0; i < 16; i++) {
        clientAddress = i + 1;
        calculateCRCs(clientAddress);
        sendCommand(1);
        delay(200);
        if (Serial1.available())
            break;
        if (i == 15)
            return false;
    }
    return true;
}

void setup()
{
    Serial1.begin(9600U, SERIAL_8N1, 16, 17); // interface to BMS
    Serial1.setTimeout(200);

    if (scanClients()) {
        foundClientAddress = true;
        if (DEBUG) {
            Serial.println("Client found with address " + (String)clientAddress);
        }
    } else {
        if (DEBUG) {
            Serial.println("No client found. Idle...");
        }
        while (1) { }
    }

    sendCommand(lastCommandIndex);
    millisSinceLastRequest = millis();
    delay(500);
    while (Serial1.available()) {
        response[bytesReadSinceLastRequest++] = Serial1.read();
        if (DEBUG)
            Serial.println(response[bytesReadSinceLastRequest - 1], HEX);
        millisSinceLastResponse = millis();
    }
    parseResponse(lastCommandIndex);

    // Move all dynamic allocations >512byte to psram (if available)
    heap_caps_malloc_extmem_enable(512);

    // Initialize serial output
    Serial.begin(SERIAL_BAUDRATE);
#if !ARDUINO_USB_CDC_ON_BOOT
    // Only wait for serial interface to be set up when not using CDC
    while (!Serial)
        yield();
#endif
    MessageOutput.init(scheduler);

    // For now, the log levels are just hard coded
    esp_log_level_set("*", ESP_LOG_VERBOSE);
    esp_log_level_set("CORE", ESP_LOG_ERROR);

    ESP_LOGI(TAG, "Starting OpenDTU");

    // Initialize file system
    ESP_LOGI(TAG, "Mounting FS...");
    if (!LittleFS.begin(false)) { // Do not format if mount failed
        ESP_LOGW(TAG, "Failed mounting FS... Trying to format...");
        const bool success = LittleFS.begin(true);
        ESP_LOG_LEVEL_LOCAL((success ? ESP_LOG_INFO : ESP_LOG_ERROR), TAG, "FS reformat %s", success ? "successful" : "failed");
    }

    // Read configuration values
    ESP_LOGI(TAG, "Reading configuration...");
    Configuration.init(scheduler);
    if (!Configuration.read()) {
        bool success = Configuration.write();
        ESP_LOG_LEVEL_LOCAL((success ? ESP_LOG_INFO : ESP_LOG_WARN), TAG, "Failed to read configuration. New default configuration written %s",
            success ? "successful" : "failed");
    }
    if (Configuration.get().Cfg.Version != CONFIG_VERSION) {
        ESP_LOGI(TAG, "Performing configuration migration from %" PRIX32 " to %" PRIX32 "",
            Configuration.get().Cfg.Version, CONFIG_VERSION);
        Configuration.migrate();
    }

    // Set configured log levels
    Logging.applyLogLevels();
    esp_log_level_set(TAG, ESP_LOG_VERBOSE);

    // Read languate pack
    ESP_LOGI(TAG, "Reading language pack...");
    I18n.init(scheduler);

    // Load PinMapping
    ESP_LOGI(TAG, "Reading PinMapping...");
    if (PinMapping.init(Configuration.get().Dev_PinMapping)) {
        ESP_LOGI(TAG, "Found valid mapping");
    } else {
        ESP_LOGW(TAG, "Didn't found valid mapping. Using default.");
    }

    // Initialize Network
    ESP_LOGI(TAG, "Initializing Network...");
    NetworkSettings.init(scheduler);
    NetworkSettings.applyConfig();

    // Initialize NTP
    ESP_LOGI(TAG, "Initializing NTP...");
    NtpSettings.init();

    // Initialize SunPosition
    ESP_LOGI(TAG, "Initializing SunPosition...");
    SunPosition.init(scheduler);

    // Initialize MqTT
    ESP_LOGI(TAG, "Initializing MQTT...");
    MqttSettings.init();
    MqttHandleDtu.init(scheduler);
    MqttHandleInverter.init(scheduler);
    MqttHandleInverterTotal.init(scheduler);
    MqttHandleHass.init(scheduler);

    // Initialize WebApi
    ESP_LOGI(TAG, "Initializing WebApi...");
    WebApi.init(scheduler);

    // Initialize Display
    ESP_LOGI(TAG, "Initializing Display...");
    Display.init(scheduler);

    // Initialize Single LEDs
    ESP_LOGI(TAG, "Initializing LEDs...");
    LedSingle.init(scheduler);

    InverterSettings.init(scheduler);

    Datastore.init(scheduler);
    RestartHelper.init(scheduler);

    ESP_LOGI(TAG, "Startup complete");
}

void loop()
{
    while (Serial1.available()) {
        response[bytesReadSinceLastRequest++] = Serial1.read();
        if (DEBUG)
            Serial.println(response[bytesReadSinceLastRequest - 1], HEX);
        millisSinceLastResponse = millis();
    }

    if (millis() - millisSinceLastRequest >= pollingIntv) {
        parseResponse(lastCommandIndex);
        if (++lastCommandIndex >= 7) {
            lastCommandIndex = 0;
        }
        if (lastCommandIndex == 0) {
            ++lastCommandIndex;
        }
        if (lastCommandIndex == 2) {
            ++lastCommandIndex;
        }
        // if (lastCommandIndex == 4) lastCommandIndex++;
        if (lastCommandIndex == 5) {
            ++lastCommandIndex;
        }
        if (lastCommandIndex == 6) {
            ++lastCommandIndex;
        }
        if (lastCommandIndex >= 7) {
            lastCommandIndex = 0;
        }
        if (lastCommandIndex == 0) {
            ++lastCommandIndex;
        }

        if (DEBUG) {
            Serial.println("Bytes read since last request: " + (String)bytesReadSinceLastRequest);
            Serial.println("Command index: " + (String)lastCommandIndex);
        }
        sendCommand(lastCommandIndex);
        for (int i = 0; i < 80; i++)
            response[i] = 0;
        bytesReadSinceLastRequest = 0;
        millisSinceLastRequest = millis();
    }

    scheduler.execute();

    //   if (Serial.available()) {
    //     while (Serial.available()) Serial.read();
    //   }

    if (millis() - millisSinceLastResponse > 30000) {
        if (DEBUG) {
            Serial.println("No response since 30 seconds, starting address scan...");
        }
        if (scanClients()) {
            foundClientAddress = true;
            if (DEBUG)
                Serial.println("Client found with address " + (String)clientAddress);
        } else {
            if (DEBUG)
                Serial.println("No client found. Idle...");
            while (1) { }
        }
    }
}
