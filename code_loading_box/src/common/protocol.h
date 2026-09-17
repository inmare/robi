#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <Arduino.h>
#include <stddef.h>

// 중앙과 상자 공통 한 줄 프로토콜. JSON 없음.
// C <id> lift.up
// Q <id> status
// A/D/F/P/S ... 응답은 protocol.cpp 가 만든다.

struct ProtoMsg {
  char kind;  // C 명령, Q 조회, 0 이면 무시
  char id[12];
  char name[24];
};

bool protoParseLine(const char *line, ProtoMsg *out);

int protoFormatAck(char *buf, size_t n, const char *id, const char *name);
int protoFormatDone(char *buf, size_t n, const char *id, const char *name,
                    const char *reason, int mm);
int protoFormatFail(char *buf, size_t n, const char *id, const char *name,
                    const char *reason);
int protoFormatProg(char *buf, size_t n, const char *id, const char *name, int mm);
int protoFormatStatus(char *buf, size_t n, const char *id, const char *state,
                      int mm, int sensorOk, int liftBottom, int liftTop,
                      int pusherBack, int pusherFront, int busy);

#endif
