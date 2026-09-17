#include "protocol.h"
#include <stdio.h>
#include <string.h>

static void copyTok(char *dst, size_t n, const char *src, size_t len) {
  if (len >= n) {
    len = n - 1;
  }
  memcpy(dst, src, len);
  dst[len] = 0;
}

static bool parseAlias(char c, ProtoMsg *out) {
  out->kind = 'C';
  strcpy(out->id, "usb");
  switch (c) {
  case 'u':
  case 'U':
    strcpy(out->name, "lift.up");
    return true;
  case 'd':
  case 'D':
    strcpy(out->name, "lift.down");
    return true;
  case 'f':
  case 'F':
    strcpy(out->name, "pusher.forward");
    return true;
  case 'b':
  case 'B':
    strcpy(out->name, "pusher.back");
    return true;
  case 's':
  case 'S':
    strcpy(out->name, "halt");
    return true;
  case '?':
    out->kind = 'Q';
    strcpy(out->name, "status");
    return true;
  default:
    return false;
  }
}

bool protoParseLine(const char *line, ProtoMsg *out) {
  while (*line == ' ' || *line == '\t') {
    line++;
  }
  if (*line == 0 || *line == '#') {
    return false;
  }

  size_t len = strlen(line);
  if (len == 1 && parseAlias(line[0], out)) {
    return true;
  }

  if ((line[0] != 'C' && line[0] != 'Q') || line[1] != ' ') {
    return false;
  }
  out->kind = line[0];
  line += 2;

  const char *id0 = line;
  while (*line && *line != ' ') {
    line++;
  }
  if (line == id0) {
    return false;
  }
  copyTok(out->id, sizeof(out->id), id0, (size_t)(line - id0));
  while (*line == ' ') {
    line++;
  }

  const char *n0 = line;
  while (*line && *line != ' ') {
    line++;
  }
  if (line == n0) {
    return false;
  }
  copyTok(out->name, sizeof(out->name), n0, (size_t)(line - n0));
  return true;
}

int protoFormatAck(char *buf, size_t n, const char *id, const char *name) {
  return snprintf(buf, n, "A %s ACK %s", id, name);
}

int protoFormatDone(char *buf, size_t n, const char *id, const char *name,
                    const char *reason, int mm) {
  if (mm >= 0) {
    return snprintf(buf, n, "D %s DONE %s reason=%s mm=%d", id, name, reason, mm);
  }
  return snprintf(buf, n, "D %s DONE %s reason=%s", id, name, reason);
}

int protoFormatFail(char *buf, size_t n, const char *id, const char *name,
                    const char *reason) {
  return snprintf(buf, n, "F %s FAIL %s reason=%s", id, name, reason);
}

int protoFormatProg(char *buf, size_t n, const char *id, const char *name, int mm) {
  return snprintf(buf, n, "P %s PROG %s mm=%d", id, name, mm);
}

int protoFormatStatus(char *buf, size_t n, const char *id, const char *state,
                      int mm, int sensorOk, int liftBottom, int liftTop,
                      int pusherBack, int pusherFront, int busy) {
  return snprintf(buf, n,
                  "S %s STATUS state=%s mm=%d sensor_ok=%d lift_bottom=%d "
                  "lift_top=%d pusher_back=%d pusher_front=%d busy=%d",
                  id, state, mm, sensorOk, liftBottom, liftTop, pusherBack,
                  pusherFront, busy);
}
