// #include "sensor.h"
// #include <Wire.h>
// #include <VL53L0X.h>

// // 이 파일 안에서만 보이는 센서 객체
// static VL53L0X sensor;

// bool sensorInit(uint16_t timeoutMs) {
//   Wire.begin();
//   sensor.setTimeout(timeoutMs);
//   if (!sensor.init()) {
//     return false;
//   }
//   sensor.startContinuous();
//   return true;
// }

// uint16_t sensorReadDistanceMm() {
//   return sensor.readRangeContinuousMillimeters();
// }

// bool sensorTimeoutOccurred() {
//   return sensor.timeoutOccurred();
// }

#include "sensor.h"
#include <Wire.h>
#include <VL53L0X.h>


static VL53L0X sensor;

bool sensorInit(uint16_t timeoutMs) {
    // 아두이노 I2C 통신 시작
    Wire.begin();
    sensor.setTimeout(timeoutMs);
    if (!sensor.init()) {
        return false;
    }
    // 센서를 계속 거리를 재는 모드로 변경
    sensor.startContinuous();
    return true;
}

uint16_t sensorReadDistanceMm() {
    return sensor.readRangeContinuousMillimeters();
}

bool sensorTimeoutOccurred() {
    return sensor.timeoutOccurred();
}

bool sensorRangeReady() {
    // VL53L0X RESULT_INTERRUPT_STATUS. 값이 없으면 read()가 수십 ms를 기다림.
    return (sensor.readReg(0x13) & 0x07) != 0;
}
