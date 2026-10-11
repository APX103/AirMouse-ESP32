#pragma once
#include <Arduino.h>
#include "config.h"

class Imu;
class Pointer;
class Link;

// 调参 shell（与有线版同款，stat 增加 airlink 统计）
class Shell {
public:
  void begin(Params *p, Imu *i, Pointer *pointer, Link *l);
  void poll();
  void loadFromNvs();
  bool logEnabled = false;

private:
  void handle(char *line);
  void printStat();
  void i2cScan();
  Params *par = nullptr;
  Imu *imu = nullptr;
  Pointer *ptr = nullptr;
  Link *lnk = nullptr;
  char buf[96];
  uint8_t blen = 0;
};
