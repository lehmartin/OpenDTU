// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2022-2024 Thomas Basler and others
 */
#include "Configuration.h"
#include "Datastore.h"
#include "Display_Graphic.h"
#include "InverterSettings.h"
#include "Led_Single.h"
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

#include <driver/uart.h>

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

    // Initialize SpiManager
    SpiManagerInst.register_bus(SPI2_HOST);
#if SOC_SPI_PERIPH_NUM > 2
    SpiManagerInst.register_bus(SPI3_HOST);
#endif

    // Initialize serial output
    Serial.begin(SERIAL_BAUDRATE);
#if ARDUINO_USB_CDC_ON_BOOT
    Serial.setTxTimeoutMs(0);
    delay(100);
#else
    while (!Serial)
        yield();
#endif
    MessageOutput.init(scheduler);
    MessageOutput.println();
    MessageOutput.println("Starting OpenDTU");

    // Initialize file system
    MessageOutput.print("Initialize FS... ");
    if (!LittleFS.begin(false)) { // Do not format if mount failed
        MessageOutput.print("failed... trying to format...");
        if (!LittleFS.begin(true)) {
            MessageOutput.print("success");
        } else {
            MessageOutput.print("failed");
        }
    } else {
        MessageOutput.println("done");
    }

    // Read configuration values
    MessageOutput.print("Reading configuration... ");
    if (!Configuration.read()) {
        MessageOutput.print("initializing... ");
        Configuration.init();
        if (Configuration.write()) {
            MessageOutput.print("written... ");
        } else {
            MessageOutput.print("failed... ");
        }
    }
    if (Configuration.get().Cfg.Version != CONFIG_VERSION) {
        MessageOutput.print("migrated... ");
        Configuration.migrate();
    }
    auto& config = Configuration.get();
    MessageOutput.println("done");

    // Load PinMapping
    MessageOutput.print("Reading PinMapping... ");
    if (PinMapping.init(String(Configuration.get().Dev_PinMapping))) {
        MessageOutput.print("found valid mapping ");
    } else {
        MessageOutput.print("using default config ");
    }
    const auto& pin = PinMapping.get();
    MessageOutput.println("done");

    // Initialize WiFi
    MessageOutput.print("Initialize Network... ");
    NetworkSettings.init(scheduler);
    MessageOutput.println("done");
    NetworkSettings.applyConfig();

    // Initialize NTP
    MessageOutput.print("Initialize NTP... ");
    NtpSettings.init();
    MessageOutput.println("done");

    // Initialize SunPosition
    MessageOutput.print("Initialize SunPosition... ");
    SunPosition.init(scheduler);
    MessageOutput.println("done");

    // Initialize MqTT
    MessageOutput.print("Initialize MqTT... ");
    MqttSettings.init();
    MqttHandleDtu.init(scheduler);
    MqttHandleInverter.init(scheduler);
    MqttHandleInverterTotal.init(scheduler);
    MqttHandleHass.init(scheduler);
    MessageOutput.println("done");

    // Initialize WebApi
    MessageOutput.print("Initialize WebApi... ");
    WebApi.init(scheduler);
    MessageOutput.println("done");

    // Initialize Display
    MessageOutput.print("Initialize Display... ");
    Display.init(
        scheduler,
        static_cast<DisplayType_t>(pin.display_type),
        pin.display_data,
        pin.display_clk,
        pin.display_cs,
        pin.display_reset);
    Display.setDiagramMode(static_cast<DiagramMode_t>(config.Display.Diagram.Mode));
    Display.setOrientation(config.Display.Rotation);
    Display.enablePowerSafe = config.Display.PowerSafe;
    Display.enableScreensaver = config.Display.ScreenSaver;
    Display.setContrast(config.Display.Contrast);
    Display.setLanguage(config.Display.Language);
    Display.setStartupDisplay();
    MessageOutput.println("done");

    // Initialize Single LEDs
    MessageOutput.print("Initialize LEDs... ");
    LedSingle.init(scheduler);
    MessageOutput.println("done");

    // Check for default DTU serial
    MessageOutput.print("Check for default DTU serial... ");
    if (config.Dtu.Serial == DTU_SERIAL) {
        MessageOutput.print("generate serial based on ESP chip id: ");
        const uint64_t dtuId = Utils::generateDtuSerial();
        MessageOutput.printf("%0" PRIx32 "%08" PRIx32 "... ",
            ((uint32_t)((dtuId >> 32) & 0xFFFFFFFF)),
            ((uint32_t)(dtuId & 0xFFFFFFFF)));
        config.Dtu.Serial = dtuId;
        Configuration.write();
    }
    MessageOutput.println("done");

    InverterSettings.init(scheduler);

    Datastore.init(scheduler);
    RestartHelper.init(scheduler);
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
