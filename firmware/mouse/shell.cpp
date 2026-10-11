#include "shell.h"
#include <Preferences.h>
#include <Wire.h>
#include "imu.h"
#include "pointer.h"
#include "link.h"

static const char *MODE_NAMES[] = {"idle", "log", "rate", "abs"};
static const char *AXIS_NAMES[] = {"px", "nx", "py", "ny", "pz", "nz"};

void Shell::begin(Params *p, Imu *i, Pointer *pointer, Link *l) {
  par = p; imu = i; ptr = pointer; lnk = l;
  Serial.println();
  Serial.println("[AirMouse-WL v0.1 鼠标端] 输入 help 查看命令。");
}

void Shell::poll() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (blen > 0) { buf[blen] = 0; handle(buf); blen = 0; }
    } else if (blen < sizeof(buf) - 1) {
      buf[blen++] = c;
    }
  }
}

static bool parseOnOff(const char *s, bool &out) {
  if (!strcasecmp(s, "1") || !strcasecmp(s, "on"))  { out = true;  return true; }
  if (!strcasecmp(s, "0") || !strcasecmp(s, "off")) { out = false; return true; }
  return false;
}

void Shell::handle(char *line) {
  char *cmd = strtok(line, " \t");
  char *a1 = strtok(nullptr, " \t");
  char *a2 = strtok(nullptr, " \t");
  if (!cmd) return;

  if (!strcasecmp(cmd, "help")) {
    Serial.println(F(
      "命令: mode idle|log|rate|abs      fov H [V]      scr W H\n"
      "      mount px|nx|py|ny|pz|nz     invx 0/1       invy 0/1\n"
      "      sensor icm|bno(重启生效)    rate 25..250\n"
      "      addr 0x68|0x69|0x4A(重启生效)  i2c 100000|400000\n"
      "      euro fc beta   dead px      freeze ms deg\n"
      "      driftcomp 0/1  rategain px/deg\n"
      "      recenter | scan | stat | log 0/1\n"
      "      save | load | defaults"));
  } else if (!strcasecmp(cmd, "stat")) {
    printStat();
  } else if (!strcasecmp(cmd, "scan")) {
    i2cScan();
  } else if (!strcasecmp(cmd, "recenter")) {
    ptr->recenter();
    Serial.println(F("recentered"));
  } else if (!strcasecmp(cmd, "log")) {
    if (a1 && parseOnOff(a1, logEnabled)) Serial.printf("log=%d\n", logEnabled);
  } else if (!strcasecmp(cmd, "mode")) {
    if (!a1) { Serial.printf("mode=%s\n", MODE_NAMES[par->mode]); return; }
    for (uint8_t i = 0; i < 4; i++)
      if (!strcasecmp(a1, MODE_NAMES[i])) {
        par->mode = i; ptr->resyncFilter();
        Serial.printf("mode=%s\n", MODE_NAMES[i]); return;
      }
    Serial.println(F("? idle|log|rate|abs"));
  } else if (!strcasecmp(cmd, "fov")) {
    if (!a1) { Serial.printf("fov %.1f/%.1f\n", par->fovH, par->fovV); return; }
    float fh = atof(a1), fv = a2 ? atof(a2) : fh * 0.75f;
    ptr->setFov(fh, fv);
    Serial.printf("fov %.1f/%.1f\n", par->fovH, par->fovV);
  } else if (!strcasecmp(cmd, "scr")) {
    if (a1 && a2) { ptr->setScreen(atoi(a1), atoi(a2)); }
    Serial.printf("scr %ux%u\n", par->scrW, par->scrH);
  } else if (!strcasecmp(cmd, "mount")) {
    if (!a1) { Serial.printf("mount=%s\n", AXIS_NAMES[par->fwd]); return; }
    for (uint8_t i = 0; i < 6; i++)
      if (!strcasecmp(a1, AXIS_NAMES[i])) {
        par->fwd = i; ptr->resyncFilter();
        Serial.printf("mount=%s\n", AXIS_NAMES[i]); return;
      }
    Serial.println(F("? px|nx|py|ny|pz|nz"));
  } else if (!strcasecmp(cmd, "invx") && a1) { parseOnOff(a1, par->invX); Serial.printf("invX=%d\n", par->invX); }
  else if (!strcasecmp(cmd, "invy") && a1) { parseOnOff(a1, par->invY); Serial.printf("invY=%d\n", par->invY); }
  else if (!strcasecmp(cmd, "rate") && a1) {
    par->reportHz = (uint32_t)atol(a1);       // 必须写入参数，save 才能持久化（v0.2.1修）
    imu->setReportHz(par->reportHz);
    Serial.printf("rate %luHz\n", (unsigned long)imu->reportHz());
  } else if (!strcasecmp(cmd, "addr") && a1) {
    par->i2cAddr = (uint8_t)strtol(a1, nullptr, 0);
    Serial.printf("addr=0x%02X (save + 断电重启生效)\n", par->i2cAddr);
  } else if (!strcasecmp(cmd, "i2c") && a1) {
    par->i2cHz = (uint32_t)atol(a1);
    Wire.setClock(par->i2cHz);
    Serial.printf("i2c=%luHz\n", (unsigned long)par->i2cHz);
  } else if (!strcasecmp(cmd, "euro") && a1) {
    par->euroFc = atof(a1);
    if (a2) par->euroBeta = atof(a2);
    ptr->resyncFilter();
    Serial.printf("euro fc=%.2f beta=%.4f\n", par->euroFc, par->euroBeta);
  } else if (!strcasecmp(cmd, "dead") && a1) { par->deadPx = atoi(a1); Serial.printf("dead=%u\n", par->deadPx); }
  else if (!strcasecmp(cmd, "freeze") && a1) {
    par->freezeMs = atoi(a1);
    if (a2) par->freezeDeg = atof(a2);
    Serial.printf("freeze %ums %.1fdeg\n", par->freezeMs, par->freezeDeg);
  } else if (!strcasecmp(cmd, "driftcomp") && a1) { parseOnOff(a1, par->driftComp); Serial.printf("driftcomp=%d\n", par->driftComp); }
  else if (!strcasecmp(cmd, "soft") && a1) { parseOnOff(a1, par->softEdge); ptr->resyncFilter(); Serial.printf("softEdge=%d\n", par->softEdge); }
  else if (!strcasecmp(cmd, "rategain") && a1) { par->rateGain = atof(a1); Serial.printf("rategain=%.1f\n", par->rateGain); }
  else if (!strcasecmp(cmd, "sensor")) {
    if (!a1) { Serial.printf("sensor=%s\n", par->sensor == SENSOR_BNO085 ? "bno085" : "icm20948"); return; }
    if (!strcasecmp(a1, "icm") || !strcasecmp(a1, "icm20948")) {
      par->sensor = SENSOR_ICM20948; par->i2cAddr = 0x68;
    } else if (!strcasecmp(a1, "bno") || !strcasecmp(a1, "bno085")) {
      par->sensor = SENSOR_BNO085; par->i2cAddr = 0x4A;
    } else { Serial.println(F("? icm|bno")); return; }
    Serial.println(F("已设置（save + 断电重启生效）"));
  } else if (!strcasecmp(cmd, "save")) {
    saveToNvs();
    Serial.println(F("saved"));
  } else if (!strcasecmp(cmd, "load")) {
    loadFromNvs();
    Serial.println(F("loaded"));
  } else if (!strcasecmp(cmd, "defaults")) {
    *par = Params{};
    Serial.println(F("defaults restored (未保存；save 持久化)"));
  } else {
    Serial.printf("? %s (help 查看)\n", cmd);
  }
}

void Shell::saveToNvs() {
  Preferences prefs;
  prefs.begin("airmouse", false);
  prefs.putFloat("fovH", par->fovH);      prefs.putFloat("fovV", par->fovV);
  prefs.putUShort("scrW", par->scrW);     prefs.putUShort("scrH", par->scrH);
  prefs.putUChar("fwd", par->fwd);        prefs.putBool("invX", par->invX);
  prefs.putBool("invY", par->invY);       prefs.putUChar("addr", par->i2cAddr);
  prefs.putUInt("i2cHz", par->i2cHz);     prefs.putUInt("hz", par->reportHz);
  prefs.putFloat("euroFc", par->euroFc);  prefs.putFloat("euroBeta", par->euroBeta);
  prefs.putUShort("deadPx", par->deadPx); prefs.putUShort("freezeMs", par->freezeMs);
  prefs.putFloat("freezeDeg", par->freezeDeg); prefs.putBool("driftComp", par->driftComp);
  prefs.putBool("softEdge", par->softEdge);
  prefs.putFloat("rateGain", par->rateGain);   prefs.putUChar("mode", par->mode);
  prefs.putUChar("sensor", par->sensor);
  prefs.end();
}

void Shell::loadFromNvs() {
  Preferences prefs;
  if (!prefs.begin("airmouse", true)) return;
  par->fovH = prefs.getFloat("fovH", par->fovH);
  par->fovV = prefs.getFloat("fovV", par->fovV);
  par->scrW = prefs.getUShort("scrW", par->scrW);
  par->scrH = prefs.getUShort("scrH", par->scrH);
  par->fwd = prefs.getUChar("fwd", par->fwd);
  par->invX = prefs.getBool("invX", par->invX);
  par->invY = prefs.getBool("invY", par->invY);
  par->i2cAddr = prefs.getUChar("addr", par->i2cAddr);
  par->i2cHz = prefs.getUInt("i2cHz", par->i2cHz);
  par->reportHz = prefs.getUInt("hz", par->reportHz);
  par->euroFc = prefs.getFloat("euroFc", par->euroFc);
  par->euroBeta = prefs.getFloat("euroBeta", par->euroBeta);
  par->deadPx = prefs.getUShort("deadPx", par->deadPx);
  par->freezeMs = prefs.getUShort("freezeMs", par->freezeMs);
  par->freezeDeg = prefs.getFloat("freezeDeg", par->freezeDeg);
  par->driftComp = prefs.getBool("driftComp", par->driftComp);
  par->softEdge = prefs.getBool("softEdge", par->softEdge);
  par->rateGain = prefs.getFloat("rateGain", par->rateGain);
  par->mode = prefs.getUChar("mode", par->mode);
  par->sensor = prefs.getUChar("sensor", par->sensor);
  prefs.end();
}

void Shell::printStat() {
  int t = temperatureRead();
  Serial.printf("== AirMouse-WL 鼠标端 v0.2 | up %lus | %s ==\n",
                millis() / 1000UL,
                t > 0 ? (String("芯片温度 ") + t + "°C").c_str() : "温度N/A");
  Serial.printf("mode=%s sensor=%s imu=%s addr=0x%02X rate=%luHz cal=%u/3\n",
                MODE_NAMES[par->mode],
                par->sensor == SENSOR_BNO085 ? "bno085" : "icm20948",
                imu->ok() ? "OK" : "FAIL",
                imu->addr(), (unsigned long)imu->reportHz(), imu->lastCal);
  Serial.printf("link: txOk=%lu txFail=%lu 心跳=%lu %s\n",
                (unsigned long)lnk->txOk, (unsigned long)lnk->txFail,
                (unsigned long)lnk->hbCount,
                lnk->healthy() ? "(健康)" : "(!!链路异常)");
  Serial.printf("speed=%.2f°/s frozen=%d click=%d anchored=%d model=(%d,%d)\n",
                ptr->speedEma, ptr->frozen, ptr->clickFrozen(), ptr->anchored,
                ptr->modelX(), ptr->modelY());
  Serial.printf("fov=%.1f/%.1f scr=%ux%u mount=%s invX=%d invY=%d driftcomp=%d soft=%d\n",
                par->fovH, par->fovV, par->scrW, par->scrH,
                AXIS_NAMES[par->fwd], par->invX, par->invY, par->driftComp, par->softEdge);
}

void Shell::i2cScan() {
  Serial.print("I2C scan:");
  for (uint8_t a = 1; a < 0x7F; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.printf(" 0x%02X", a);
      delay(1);
    }
  }
  Serial.println("  (期望 0x68=ICM20948, 0x4A/0x4B=BNO085)");
}
