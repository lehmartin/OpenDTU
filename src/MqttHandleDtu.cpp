// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2022-2024 Thomas Basler and others
 */
#include "MqttHandleDtu.h"
#include "Configuration.h"
#include "MqttSettings.h"
#include "NetworkSettings.h"
#include <Hoymiles.h>
#include <CpuTemperature.h>

extern float batteryVoltage;
extern float batteryCurrent;
extern long cellVoltage[16];
extern long cellTemperature[4];
extern int8_t bmsTemperature;
extern int8_t SOC;
extern int8_t cycles;

MqttHandleDtuClass MqttHandleDtu;

MqttHandleDtuClass::MqttHandleDtuClass()
    : _loopTask(TASK_IMMEDIATE, TASK_FOREVER, std::bind(&MqttHandleDtuClass::loop, this))
{
}

void MqttHandleDtuClass::init(Scheduler& scheduler)
{
    scheduler.addTask(_loopTask);
    _loopTask.setInterval(Configuration.get().Mqtt.PublishInterval * TASK_SECOND);
    _loopTask.enable();
}

void MqttHandleDtuClass::loop()
{
    _loopTask.setInterval(Configuration.get().Mqtt.PublishInterval * TASK_SECOND);

    if (!MqttSettings.getConnected() || !Hoymiles.isAllRadioIdle()) {
        _loopTask.forceNextIteration();
        return;
    }

    MqttSettings.publish("dtu/uptime", String(millis() / 1000));
    MqttSettings.publish("dtu/ip", NetworkSettings.localIP().toString());
    MqttSettings.publish("dtu/hostname", NetworkSettings.getHostname());
    MqttSettings.publish("dtu/heap/size", String(ESP.getHeapSize()));
    MqttSettings.publish("dtu/heap/free", String(ESP.getFreeHeap()));
    MqttSettings.publish("dtu/heap/minfree", String(ESP.getMinFreeHeap()));
    MqttSettings.publish("dtu/heap/maxalloc", String(ESP.getMaxAllocHeap()));
    MqttSettings.publish("dtu/battery/voltage", String(batteryVoltage));
    MqttSettings.publish("dtu/battery/current", String(batteryCurrent));
    MqttSettings.publish("dtu/battery/soc",     String(SOC));
    MqttSettings.publish("dtu/battery/cycles",   String(cycles));

    if (NetworkSettings.NetworkMode() == network_mode::WiFi) {
        MqttSettings.publish("dtu/rssi", String(WiFi.RSSI()));
        MqttSettings.publish("dtu/bssid", WiFi.BSSIDstr());
    }

    float temperature = CpuTemperature.read();
    if (!std::isnan(temperature)) {
        MqttSettings.publish("dtu/temperature", String(temperature));
    }
}
