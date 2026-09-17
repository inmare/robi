#ifndef SENSOR_H
#define SENSOR_H

#include <Arduino.h>

// 센서 전원·I2C 시작, 실패하면 false
bool sensorInit(uint16_t timeoutMs = 500);

// 거리(mm). 실패·타임아웃이면 0 또는 별도 처리
uint16_t sensorReadDistanceMm();

// 직전 읽기가 타임아웃이었는지
bool sensorTimeoutOccurred();

// 새 거리값이 준비됐는지. 모터 run()을 막지 않으려면 이게 true일 때만 읽을 것
bool sensorRangeReady();

#endif
