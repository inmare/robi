#include "common/lift_motor.h"
#include "common/limit_switch.h"
#include "common/protocol.h"
#include "common/pusher_motor.h"
#include "common/sensor.h"
#include <AccelStepper.h>
#include <Arduino.h>
#include <SoftwareSerial.h>
#include <string.h>

// 핀맵: 리프트=X D2/D5, 푸셔=Z D4/D7.
// test_lift_motor / test_pusher_motor 에서 X·Z를 바꿔 꽂았으면 아래 두 줄을 맞바꾼다.
AccelStepper lift(AccelStepper::DRIVER, 2, 5);
AccelStepper pusher(AccelStepper::DRIVER, 4, 7);

// CNC 쉴드 Resume=A2(RX), Hold=A1(TX). ESP 9600.
SoftwareSerial esp(A2, A1);

const int EN_PIN = 8;
const float LIFT_MAX_SPEED = 650;
const float LIFT_ACCEL = 320;
const float PUSHER_MAX_SPEED = 550;
const float PUSHER_ACCEL = 220;
const int MAX_LIFT_STEPS = 12000;
const int MAX_PUSHER_STEPS = 20000;
const unsigned long LIFT_TIMEOUT_MS = 25000;
const unsigned long PUSHER_TIMEOUT_MS = 20000;

enum State { ST_IDLE, ST_LIFT_UP, ST_LIFT_DOWN, ST_PUSH_FWD, ST_PUSH_BACK };
State state = ST_IDLE;

char activeId[12] = "0";
char activeCmd[24] = "";
unsigned long deadlineMs = 0;
unsigned long lastProgMs = 0;
uint16_t lastMm = 0;
bool sensorOk = false;

static char usbBuf[96];
static uint8_t usbN = 0;
static char espBuf[96];
static uint8_t espN = 0;
static char outBuf[128];

static bool distanceOk(uint16_t mm) {
  return !sensorTimeoutOccurred() && mm > 0 && mm < 8000;
}

static const char *stateName() {
  switch (state) {
  case ST_LIFT_UP:
    return "LIFT_UP";
  case ST_LIFT_DOWN:
    return "LIFT_DOWN";
  case ST_PUSH_FWD:
    return "PUSH_FWD";
  case ST_PUSH_BACK:
    return "PUSH_BACK";
  default:
    return "IDLE";
  }
}

static void emit(const char *line) {
  Serial.println(line);
  esp.println(line);
}

static void haltMotors() {
  lift.setCurrentPosition(lift.currentPosition());
  pusher.setCurrentPosition(pusher.currentPosition());
}

static void remember(const ProtoMsg *msg) {
  strncpy(activeId, msg->id, sizeof(activeId) - 1);
  activeId[sizeof(activeId) - 1] = 0;
  strncpy(activeCmd, msg->name, sizeof(activeCmd) - 1);
  activeCmd[sizeof(activeCmd) - 1] = 0;
}

static void replyAck() {
  protoFormatAck(outBuf, sizeof(outBuf), activeId, activeCmd);
  emit(outBuf);
}

static void finishDone(const char *reason, int mm) {
  haltMotors();
  protoFormatDone(outBuf, sizeof(outBuf), activeId, activeCmd, reason, mm);
  emit(outBuf);
  state = ST_IDLE;
  activeCmd[0] = 0;
}

static void finishFail(const char *reason) {
  haltMotors();
  protoFormatFail(outBuf, sizeof(outBuf), activeId, activeCmd, reason);
  emit(outBuf);
  state = ST_IDLE;
  activeCmd[0] = 0;
}

static int currentMm() {
  if (!sensorOk) {
    return -1;
  }
  return (int)lastMm;
}

static void sendStatus(const char *id) {
  int mm = currentMm();
  if (mm < 0) {
    mm = 0;
  }
  protoFormatStatus(outBuf, sizeof(outBuf), id, stateName(), mm, sensorOk ? 1 : 0,
                    limitLiftBottomPressed() ? 1 : 0, limitLiftTopPressed() ? 1 : 0,
                    limitPusherBackPressed() ? 1 : 0, limitPusherFrontPressed() ? 1 : 0,
                    state != ST_IDLE ? 1 : 0);
  emit(outBuf);
}

static void failNow(const ProtoMsg *msg, const char *reason) {
  protoFormatFail(outBuf, sizeof(outBuf), msg->id, msg->name, reason);
  emit(outBuf);
}

static void doneNow(const ProtoMsg *msg, const char *reason, int mm) {
  protoFormatAck(outBuf, sizeof(outBuf), msg->id, msg->name);
  emit(outBuf);
  protoFormatDone(outBuf, sizeof(outBuf), msg->id, msg->name, reason, mm);
  emit(outBuf);
}

static void startLiftUp(const ProtoMsg *msg) {
  if (!sensorOk) {
    failNow(msg, "sensor");
    return;
  }
  uint16_t mm = sensorReadDistanceMm();
  lastMm = mm;
  if (distanceOk(mm) && mm <= TARGET_DISTANCE_MM) {
    doneNow(msg, "already", (int)mm);
    return;
  }
  remember(msg);
  replyAck();
  lift.move(MAX_LIFT_STEPS * LIFT_UP);
  state = ST_LIFT_UP;
  deadlineMs = millis() + LIFT_TIMEOUT_MS;
  lastProgMs = 0;
}

static void startLiftDown(const ProtoMsg *msg) {
  if (limitLiftBottomPressed()) {
    doneNow(msg, "already", currentMm());
    return;
  }
  remember(msg);
  replyAck();
  lift.move(MAX_LIFT_STEPS * LIFT_DOWN);
  state = ST_LIFT_DOWN;
  deadlineMs = millis() + LIFT_TIMEOUT_MS;
}

static void startPushFwd(const ProtoMsg *msg) {
  if (limitPusherFrontPressed()) {
    doneNow(msg, "already", currentMm());
    return;
  }
  remember(msg);
  replyAck();
  pusher.move(MAX_PUSHER_STEPS * PUSHER_FORWARD);
  state = ST_PUSH_FWD;
  deadlineMs = millis() + PUSHER_TIMEOUT_MS;
}

static void startPushBack(const ProtoMsg *msg) {
  if (limitPusherBackPressed()) {
    doneNow(msg, "already", currentMm());
    return;
  }
  remember(msg);
  replyAck();
  pusher.move(MAX_PUSHER_STEPS * PUSHER_BACK);
  state = ST_PUSH_BACK;
  deadlineMs = millis() + PUSHER_TIMEOUT_MS;
}

static void doHalt(const ProtoMsg *msg) {
  if (state != ST_IDLE && activeCmd[0]) {
    protoFormatFail(outBuf, sizeof(outBuf), activeId, activeCmd, "halted");
    emit(outBuf);
  }
  haltMotors();
  state = ST_IDLE;
  strncpy(activeId, msg->id, sizeof(activeId) - 1);
  activeId[sizeof(activeId) - 1] = 0;
  strcpy(activeCmd, "halt");
  replyAck();
  finishDone("stopped", currentMm());
}

static void handleMsg(const ProtoMsg *msg) {
  if (msg->kind == 'Q' || strcmp(msg->name, "status") == 0) {
    sendStatus(msg->id);
    return;
  }
  if (strcmp(msg->name, "halt") == 0) {
    doHalt(msg);
    return;
  }
  if (state != ST_IDLE) {
    failNow(msg, "busy");
    return;
  }
  if (strcmp(msg->name, "lift.up") == 0) {
    startLiftUp(msg);
  } else if (strcmp(msg->name, "lift.down") == 0) {
    startLiftDown(msg);
  } else if (strcmp(msg->name, "pusher.forward") == 0) {
    startPushFwd(msg);
  } else if (strcmp(msg->name, "pusher.back") == 0) {
    startPushBack(msg);
  } else {
    failNow(msg, "unknown");
  }
}

static bool feedStream(Stream &s, char *buf, uint8_t *n, ProtoMsg *out) {
  while (s.available()) {
    char c = (char)s.read();
    if (c == '\r') {
      continue;
    }
    if (c == '\n') {
      buf[*n] = 0;
      *n = 0;
      if (buf[0] == 0) {
        return false;
      }
      return protoParseLine(buf, out);
    }
    if (*n + 1 < 96) {
      buf[(*n)++] = c;
    } else {
      *n = 0;
    }
  }
  return false;
}

static void pollMotion() {
  if (state == ST_IDLE) {
    return;
  }

  if ((long)(millis() - deadlineMs) >= 0) {
    finishFail("timeout");
    return;
  }

  if (state == ST_LIFT_DOWN && limitLiftBottomPressed()) {
    finishDone("limit_bottom", currentMm());
    return;
  }
  if (state == ST_LIFT_UP && limitLiftTopPressed()) {
    finishDone("limit_top", currentMm());
    return;
  }
  if (state == ST_PUSH_FWD && limitPusherFrontPressed()) {
    finishDone("limit_front", currentMm());
    return;
  }
  if (state == ST_PUSH_BACK && limitPusherBackPressed()) {
    finishDone("limit_back", currentMm());
    return;
  }

  if (state == ST_LIFT_UP && sensorOk && sensorRangeReady()) {
    uint16_t mm = sensorReadDistanceMm();
    lastMm = mm;
    if (distanceOk(mm) && mm <= TARGET_DISTANCE_MM) {
      finishDone("tof", (int)mm);
      return;
    }
    if (millis() - lastProgMs > 300) {
      lastProgMs = millis();
      protoFormatProg(outBuf, sizeof(outBuf), activeId, activeCmd, (int)mm);
      emit(outBuf);
    }
  }

  bool movingLift = (state == ST_LIFT_UP || state == ST_LIFT_DOWN);
  if (movingLift && lift.distanceToGo() == 0) {
    finishFail("max_steps");
    return;
  }
  bool movingPush = (state == ST_PUSH_FWD || state == ST_PUSH_BACK);
  if (movingPush && pusher.distanceToGo() == 0) {
    finishFail("max_steps");
    return;
  }
}

void setup() {
  Serial.begin(115200);
  esp.begin(9600);
  delay(200);
  limitSwitchInit();

  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);

  lift.setMaxSpeed(LIFT_MAX_SPEED);
  lift.setAcceleration(LIFT_ACCEL);
  pusher.setMaxSpeed(PUSHER_MAX_SPEED);
  pusher.setAcceleration(PUSHER_ACCEL);

  sensorOk = sensorInit(500);
  if (sensorOk) {
    lastMm = sensorReadDistanceMm();
  }

  emit("H box");
}

void loop() {
  ProtoMsg msg;
  if (feedStream(Serial, usbBuf, &usbN, &msg)) {
    handleMsg(&msg);
  }
  if (feedStream(esp, espBuf, &espN, &msg)) {
    handleMsg(&msg);
  }

  pollMotion();
  lift.run();
  pusher.run();
}
