#include <Arduino.h>
#include <Bluepad32.h>
#include <esp_system.h>

#define FL_IN1  25
#define FL_IN2  26
#define FR_IN1  27
#define FR_IN2  14
#define RL_IN1  32
#define RL_IN2  33
#define RR_IN1  18
#define RR_IN2  19

#define W_RPWM  16
#define W_LPWM  17

#define W_REN      21   // bts7960 r_en
#define W_LEN      22   // bts7960 l_en
#define ENC_A      13   // CLK
#define ENC_B      4    // DT
#define LIMIT_PIN  34   // input only no internal pullup

#define LED     2

const int WHEEL_PWM_HZ    = 1000;
const int DEADZONE        = 40;
const int CAL_MAX_DUTY    = 120;
const int WHEEL_MAX       = 150;
const int DRIVE_MAX       = 220;
const int SHOVEL_DUTY     = 100;  // shovel power 0 to 120
const uint32_t RAMP_MS    = 50;   // soft start time in ms
const int FLIP_UP_DUTY    = 120;  // flip up power 0 to 120
const int FLIP_DOWN_DUTY  = 90;   // flip down power 0 to 120
const uint32_t FLIP_UP_MS   = 250; // flip up time counted from the moment the arm leaves the bottom switch
const uint32_t FLIP_UP_TIMEOUT_MS = 1200; // give up going up after this long
const uint32_t FLIP_UP_RAMP_MS = 60; // soft start of the up move
const long FLIP_TOP_TICKS = 0;    // optional top in encoder ticks, 0 means use time only
const uint32_t FLIP_GAP_MS  = 60;  // coast between up and down so reverse does not spike current
const uint32_t FLIP_DOWN_RAMP_MS = 60; // soft start of the down move
const int FLIP_LAND_DUTY  = 60;   // push down after the limit switch to reach the real bottom
const uint32_t FLIP_LAND_MS = 100; // how long to push after the limit switch
const uint32_t FLIP_DOWN_MS = 1500; // max time to find the bottom switch
int shovelDir             = 0;
uint32_t shovelT          = 0;
enum FlipState { F_NONE, F_UP, F_GAP, F_DOWN, F_LAND };
FlipState flipState       = F_NONE;
uint32_t flipT            = 0;
uint32_t flipMoveT        = 0;   // time the arm left the bottom switch during the flip
const int POWER_MIN       = 20;
const int POWER_STEP      = 5;
const int RAMP_STEP       = 8;
const long GUARD_TICKS    = 200;
const long LAND_TICKS     = 12;
const uint32_t NUDGE_MS   = 150;
const uint32_t UP_TIMEOUT = 2500;
const uint32_t HOME_TIMEOUT = 4000;
const uint32_t PAUSE_MS   = 150;
const uint32_t STALL_MS   = 400;
const uint32_t DATA_TIMEOUT = 500;
const uint32_t DEAD_DISCONNECT_MS = 3000;

volatile long encCount = 0;
volatile uint8_t encPrev = 0;
volatile bool encInv = true;   // up must count positive

static const DRAM_ATTR int8_t ENC_TABLE[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

void IRAM_ATTR encISR() {
  uint8_t cur = (digitalRead(ENC_A) << 1) | digitalRead(ENC_B);
  int8_t step = ENC_TABLE[(encPrev << 2) | cur];
  encPrev = cur;
  if (encInv) step = -step;
  encCount += step;
}

bool limitOn = false;
uint32_t limitT = 0;
bool homed = false;

void limitUpdate() {
  bool raw = digitalRead(LIMIT_PIN) == LOW;
  uint32_t now = millis();
  if (raw != limitOn && now - limitT > 15) {
    limitOn = raw;
    limitT = now;
    if (limitOn) {
      encCount = 0;
      homed = true;
    }
  }
}

const int WIN[4][2] = {{FL_IN1, FL_IN2}, {FR_IN1, FR_IN2}, {RL_IN1, RL_IN2}, {RR_IN1, RR_IN2}};
const char* WNAME[4] = {"FL", "FR", "RL", "RR"};
bool invWheel[4] = {true, true, true, true};

void setWheel(int i, int v) {
  if (invWheel[i]) v = -v;
  v = constrain(v, -255, 255);
  if (v >= 0) {
    ledcWrite(WIN[i][0], v);
    ledcWrite(WIN[i][1], 0);
  } else {
    ledcWrite(WIN[i][0], 0);
    ledcWrite(WIN[i][1], -v);
  }
}

// current wheel commands after ramping, left and right side
int curL = 0, curR = 0;
// max change of wheel command per loop pass, softens reversals so the drivers do not see current spikes
const int DRIVE_RAMP = 8;

// move cur toward target by at most DRIVE_RAMP
static int rampTo(int cur, int target) {
  if (target > cur + DRIVE_RAMP) return cur + DRIVE_RAMP;
  if (target < cur - DRIVE_RAMP) return cur - DRIVE_RAMP;
  return target;
}

void stopWheels() {
  // reset ramp state so the next start begins from zero
  curL = curR = 0;
  for (int i = 0; i < 4; i++) setWheel(i, 0);
}

void drive(int thr, int turn) {
  int l = constrain(thr + turn, -512, 512) * DRIVE_MAX / 512;
  int r = constrain(thr - turn, -512, 512) * DRIVE_MAX / 512;
  // ramp toward the requested speed instead of jumping straight to it
  curL = rampTo(curL, l);
  curR = rampTo(curR, r);
  setWheel(0, curL);
  setWheel(2, curL);
  setWheel(1, curR);
  setWheel(3, curR);
}

int weaponCmd = 0;
int weaponPower = 30;
bool invWeapon = false;
long topTicks = -1;

void applyWeapon(int v) {
  if (invWeapon) v = -v;
  v = constrain(v, -CAL_MAX_DUTY, CAL_MAX_DUTY);
  if (v >= 0) {
    ledcWrite(W_RPWM, v);
    ledcWrite(W_LPWM, 0);
  } else {
    ledcWrite(W_RPWM, 0);
    ledcWrite(W_LPWM, -v);
  }
}

int weaponGuard(int v) {
  long pos = encCount;
  if (v < 0 && limitOn) return 0;
  if (v > 0 && homed) {
    long top = topTicks >= 0 ? topTicks : GUARD_TICKS;
    if (pos >= top) return 0;
  }
  return v;
}

void setWeapon(int target) {
  target = constrain(target, -CAL_MAX_DUTY, CAL_MAX_DUTY);
  target = weaponGuard(target);
  if (target == 0 || target * weaponCmd < 0) weaponCmd = 0;
  if (target > weaponCmd) {
    weaponCmd = min(weaponCmd + RAMP_STEP, target);
  } else if (target < weaponCmd) {
    weaponCmd = max(weaponCmd - RAMP_STEP, target);
  }
  applyWeapon(weaponCmd);
}

uint32_t moveT = 0;
long moveEnc = 0;

bool weaponStalled(uint32_t now) {
  long pos = encCount;
  if (weaponCmd == 0 || pos != moveEnc) {
    moveEnc = pos;
    moveT = now;
    return false;
  }
  return now - moveT > STALL_MS;
}

enum AutoState { S_IDLE, S_UP, S_PAUSE, S_HOME };
AutoState autoState = S_IDLE;
uint32_t stateT = 0;
bool stallLock = false;
int nudgeDir = 0;
uint32_t nudgeUntil = 0;

enum Mode { M_WEAPON, M_WHEELS };
Mode mode = M_WEAPON;

void stopAll() {
  shovelDir = 0;
  flipState = F_NONE;
  autoState = S_IDLE;
  nudgeUntil = 0;
  setWeapon(0);
  stopWheels();
}

void stallEvent() {
  setWeapon(0);
  Serial.println("STALL stop");
  if (autoState == S_UP) {
    autoState = S_PAUSE;
    stateT = millis();
  } else if (autoState == S_HOME) {
    homed = false;
    autoState = S_IDLE;
  } else {
    stallLock = true;
  }
}

void printConfig() {
  Serial.println("config to paste");
  for (int i = 0; i < 4; i++) {
    Serial.printf("INVERT_%s = %s\n", WNAME[i], invWheel[i] ? "true" : "false");
  }
  Serial.printf("INVERT_WEAPON = %s\n", invWeapon ? "true" : "false");
  Serial.printf("ENC_INVERT = %s\n", encInv ? "true" : "false");
  Serial.printf("FLIP_UP_TICKS = %ld\n", topTicks);
  Serial.printf("SAFE_POWER = %d\n", weaponPower);
}

ControllerPtr gamepad = nullptr;
bool rumbleDone = false;
uint32_t lastDataMs = 0;
uint32_t lastHash = 0;
uint32_t lastPrintMs = 0;

bool pA, pB, pX, pY, pR1, pSel, pStart, pUp, pDn, pLeft, pRight;
bool eA, eB, eX, eY, eR1, eSel, eStart, eUp, eDn, eLeft, eRight;

bool edge(bool cur, bool &prev) {
  bool e = cur && !prev;
  prev = cur;
  return e;
}

uint32_t stateHash(ControllerPtr c) {
  uint32_t h = 2166136261u;
  auto mix = [&](int32_t v) { h = (h ^ (uint32_t)v) * 16777619u; };
  mix(c->axisX());  mix(c->axisY());
  mix(c->axisRX()); mix(c->axisRY());
  mix(c->brake());  mix(c->throttle());
  mix(c->buttons()); mix(c->dpad()); mix(c->miscButtons());
  mix(c->accelX()); mix(c->accelY()); mix(c->accelZ());
  mix(c->gyroX());  mix(c->gyroY());  mix(c->gyroZ());
  return h;
}

int applyDeadzone(int v) { return abs(v) < DEADZONE ? 0 : v; }

// true if any button, dpad, stick or trigger is active
bool anyInput() {
  return gamepad->buttons() || gamepad->dpad() || gamepad->miscButtons() ||
         abs(gamepad->axisX()) > DEADZONE || abs(gamepad->axisY()) > DEADZONE ||
         abs(gamepad->axisRX()) > DEADZONE || abs(gamepad->axisRY()) > DEADZONE ||
         gamepad->throttle() > 60 || gamepad->brake() > 60;
}

void dropGamepad() {
  if (gamepad) gamepad->disconnect();
  gamepad = nullptr;
  stopAll();
}

void onConnectedController(ControllerPtr ctl) {
  if (gamepad == nullptr) {
    Serial.println("Gamepad connected");
    gamepad = ctl;
    rumbleDone = false;
    lastDataMs = millis();
    lastHash = 0;
    pA = pB = pX = pY = pR1 = pSel = pStart = pUp = pDn = pLeft = pRight = false;
  } else {
    ctl->disconnect();
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  if (gamepad == ctl) {
    Serial.println("Gamepad disconnected");
    gamepad = nullptr;
    stopAll();
  }
}

void weaponMode(uint32_t now) {
  bool l1 = gamepad->l1();
  int analog = (int)gamepad->throttle() - (int)gamepad->brake();
  bool analogOn = abs(analog) > 60;
  if (!analogOn) stallLock = false;

  if (eUp) {
    weaponPower = min(weaponPower + POWER_STEP, CAL_MAX_DUTY);
    Serial.printf("power %d\n", weaponPower);
  }
  if (eDn) {
    weaponPower = max(weaponPower - POWER_STEP, POWER_MIN);
    Serial.printf("power %d\n", weaponPower);
  }
  if (eLeft) {
    invWeapon = !invWeapon;
    setWeapon(0);
    Serial.printf("invWeapon %d\n", invWeapon);
  }
  if (eRight) {
    encInv = !encInv;
    Serial.printf("encInv %d\n", encInv);
  }

  if (autoState != S_IDLE) {
    if (!l1 || eA || eB || abs(analog) > 100) {
      autoState = S_IDLE;
      setWeapon(0);
      Serial.println("abort");
    }
  } else {
    if (l1 && eR1) {
      autoState = S_HOME;
      stateT = now;
      Serial.println("homing");
    } else if (l1 && eX) {
      if (homed && topTicks > 0) {
        autoState = S_UP;
        stateT = now;
        Serial.println("cycle");
      } else {
        Serial.println("need home and top first");
      }
    } else if (l1 && eY) {
      if (homed && encCount > 0) {
        topTicks = encCount;
        Serial.printf("top set %ld\n", topTicks);
      } else {
        Serial.println("need home and arm above bottom");
      }
    } else if (!l1 && eA) {
      nudgeDir = 1;
      nudgeUntil = now + NUDGE_MS;
    } else if (!l1 && eB) {
      nudgeDir = -1;
      nudgeUntil = now + NUDGE_MS;
    }
  }

  int target = 0;
  switch (autoState) {
    case S_IDLE:
      if (now < nudgeUntil) {
        target = nudgeDir * weaponPower;
      } else if (l1 && !stallLock && analogOn) {
        target = analog * weaponPower / 1023;
      }
      break;
    case S_UP:
      target = weaponPower;
      if (encCount >= topTicks || now - stateT > UP_TIMEOUT) {
        target = 0;
        autoState = S_PAUSE;
        stateT = now;
      }
      break;
    case S_PAUSE:
      if (now - stateT > PAUSE_MS) {
        autoState = S_HOME;
        stateT = now;
      }
      break;
    case S_HOME:
      if (limitOn) {
        autoState = S_IDLE;
        Serial.println("at bottom");
      } else if (now - stateT > HOME_TIMEOUT) {
        homed = false;
        autoState = S_IDLE;
        Serial.println("home timeout");
      } else {
        target = -weaponPower;
        if (homed && encCount < LAND_TICKS) target = -max(weaponPower * 2 / 3, POWER_MIN);
      }
      break;
  }
  setWeapon(target);
  if (weaponStalled(now)) stallEvent();
}

void wheelMode() {
  int y = -applyDeadzone(gamepad->axisY());
  int x = applyDeadzone(gamepad->axisX());
  drive(y, x);
}

// cross is a flip, up then down to the start position (bottom switch)
// circle held is down with soft start then stop on release
// right stick up is up and down is down with power from stick deflection
// circle or stick cancels the flip
void weaponTest(uint32_t now) {
  bool bHeld = gamepad->b();

  // stick up gives negative axisRY so flip it, up is plus and down is minus
  int ry = -gamepad->axisRY();
  int absRy = abs(ry);

  // cross starts the flip
  if (eA && flipState == F_NONE) {
    flipState = F_UP;
    flipT = now;
    flipMoveT = 0;
  }
  // circle or stick cancels it
  if (flipState != F_NONE && (bHeld || absRy > DEADZONE)) flipState = F_NONE;

  int v = 0;
  if (flipState != F_NONE) {
    shovelDir = 0;
    uint32_t dt = now - flipT;
    if (flipState == F_UP) {
      // soft start, then hold power
      v = dt >= FLIP_UP_RAMP_MS ? FLIP_UP_DUTY : (int)(FLIP_UP_DUTY * dt / FLIP_UP_RAMP_MS);
      // count the up time from the moment the arm leaves the bottom switch
      if (flipMoveT == 0 && !limitOn) flipMoveT = now;
      long top = topTicks > 0 ? topTicks : FLIP_TOP_TICKS;
      bool atTop = homed && top > 0 && encCount >= top;
      if (atTop || (flipMoveT != 0 && now - flipMoveT >= FLIP_UP_MS) || dt >= FLIP_UP_TIMEOUT_MS) {
        flipState = F_GAP;
        flipT = now;
        v = 0;
        Serial.printf("flip top enc %ld\n", encCount);
      }
    } else if (flipState == F_GAP) {
      if (dt >= FLIP_GAP_MS) {
        flipState = F_DOWN;
        flipT = now;
        Serial.printf("flip down enc %ld\n", encCount);
      }
    } else if (flipState == F_DOWN) {
      // short soft start so the reverse does not slam the driver
      int dd = dt >= FLIP_DOWN_RAMP_MS ? FLIP_DOWN_DUTY : (int)(FLIP_DOWN_DUTY * dt / FLIP_DOWN_RAMP_MS);
      v = -dd;
      // ignore the limit switch for the first 100 ms in case it has not released yet
      if (limitOn && dt > 100) {
        flipState = F_LAND;  // switch is hit but the arm is not fully down yet, keep pushing
        flipT = now;
        Serial.printf("flip land enc %ld\n", encCount);
      } else if (dt >= FLIP_DOWN_MS) {
        flipState = F_NONE;
        v = 0;
        Serial.printf("flip end time enc %ld\n", encCount);
      }
    } else {
      v = -FLIP_LAND_DUTY;
      if (dt >= FLIP_LAND_MS) {
        flipState = F_NONE;
        v = 0;
        Serial.printf("flip end enc %ld limit %d\n", encCount, limitOn);
      }
    }
  } else if (bHeld) {
    if (shovelDir != -1) {
      shovelDir = -1;
      shovelT = now;
    }
    uint32_t dt = now - shovelT;
    int d = dt >= RAMP_MS ? SHOVEL_DUTY : (int)(SHOVEL_DUTY * dt / RAMP_MS);
    v = -d;
  } else if (absRy > DEADZONE) {
    // stick power scales from deadzone edge up to SHOVEL_DUTY
    shovelDir = 0;
    int mag = (absRy - DEADZONE) * SHOVEL_DUTY / (512 - DEADZONE);
    mag = constrain(mag, 0, SHOVEL_DUTY);
    int target = (ry > 0) ? mag : -mag;

    // go through zero on direction change then ramp
    if (weaponCmd * target < 0) weaponCmd = 0;
    if (target > weaponCmd)      weaponCmd = min(weaponCmd + RAMP_STEP, target);
    else if (target < weaponCmd) weaponCmd = max(weaponCmd - RAMP_STEP, target);
    v = weaponCmd;
  } else {
    // nothing pressed so stop at once
    shovelDir = 0;
    v = 0;
  }

  weaponCmd = v;
  applyWeapon(v);
}

void setup() {
  Serial.begin(115200);
  pinMode(LED, OUTPUT);

  // why the esp restarted, brownout means the supply dropped
  esp_reset_reason_t rr = esp_reset_reason();
  Serial.printf("reset reason %d%s\n", (int)rr, rr == ESP_RST_BROWNOUT ? " BROWNOUT" : "");

  pinMode(W_REN, OUTPUT); digitalWrite(W_REN, HIGH);
  pinMode(W_LEN, OUTPUT); digitalWrite(W_LEN, HIGH);
  const int allPins[] = {FL_IN1, FL_IN2, FR_IN1, FR_IN2, RL_IN1, RL_IN2, RR_IN1, RR_IN2, W_RPWM, W_LPWM};
  for (int p : allPins) { pinMode(p, OUTPUT); digitalWrite(p, LOW); }

  const int pwmPins[] = {FL_IN1, FL_IN2, FR_IN1, FR_IN2,
                         RL_IN1, RL_IN2, RR_IN1, RR_IN2,
                         W_RPWM, W_LPWM};
  // wheels 1 khz because drv8833 dislikes 20 khz and shovel stays at 20 khz
  for (int i = 0; i < 8; i++) ledcAttach(pwmPins[i], WHEEL_PWM_HZ, 8);
  for (int i = 8; i < 10; i++) ledcAttach(pwmPins[i], 20000, 8);
  stopAll();

  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  encPrev = (digitalRead(ENC_A) << 1) | digitalRead(ENC_B);
  attachInterrupt(digitalPinToInterrupt(ENC_A), encISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), encISR, CHANGE);

  pinMode(LIMIT_PIN, INPUT);  // gpio34 needs external 10k pullup to 3v3
  limitOn = digitalRead(LIMIT_PIN) == LOW;
  limitT = millis();
  if (limitOn) {
    encCount = 0;
    homed = true;
  }

  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.enableVirtualDevice(false);

  Serial.println("Calibration. Waiting for gamepad (SHARE + PS)");
}

void loop() {
  BP32.update();
  limitUpdate();

  bool alive = gamepad && gamepad->isConnected();
  uint32_t now = millis();

  if (alive && gamepad->hasData()) {
    uint32_t h = stateHash(gamepad);
    if (h != lastHash) { lastHash = h; lastDataMs = now; }
  }

  if (!alive) {
    stopAll();
    digitalWrite(LED, LOW);
    if (gamepad && now - lastDataMs > DEAD_DISCONNECT_MS) {
      Serial.println("Gamepad silent, dropping");
      dropGamepad();
    }
    delay(5);
    return;
  }

  if (!rumbleDone) {
    gamepad->playDualRumble(0, 250, 0xC0, 0xC0);
    rumbleDone = true;
  }

  uint8_t dp = gamepad->dpad();
  eA = edge(gamepad->a(), pA);
  eB = edge(gamepad->b(), pB);
  eX = edge(gamepad->x(), pX);
  eY = edge(gamepad->y(), pY);
  eR1 = edge(gamepad->r1(), pR1);
  eStart = edge(gamepad->miscStart(), pStart);
  eUp = edge(dp & 0x01, pUp);
  eDn = edge(dp & 0x02, pDn);
  eRight = edge(dp & 0x04, pRight);
  eLeft = edge(dp & 0x08, pLeft);

  if (eStart) printConfig();

  wheelMode();
  weaponTest(now);

  // print sticks every 200 ms while any stick is moved, to see what the code gets
  static uint32_t lastStickMs = 0;
  if ((abs(gamepad->axisY()) > DEADZONE || abs(gamepad->axisX()) > DEADZONE || abs(gamepad->axisRY()) > DEADZONE) && now - lastStickMs >= 200) {
    lastStickMs = now;
    // L and R are the real wheel commands, if they are nonzero while the wheels stand still it is hardware
    Serial.printf("sticks lx %d ly %d ry %d weaponCmd %d L %d R %d\n", gamepad->axisX(), gamepad->axisY(), gamepad->axisRY(), weaponCmd, curL, curR);
  }

  // print encoder every 50 ms while it moves or the shovel is driven
  static long lastEnc = 0;
  long e = encCount;
  if ((e != lastEnc || weaponCmd != 0) && now - lastPrintMs >= 50) {
    lastPrintMs = now;
    lastEnc = e;
    Serial.printf("enc %ld cmd %d limit %d flip %d\n", e, weaponCmd, limitOn, (int)flipState);
  }

  // blue led on any gamepad input or while the shovel moves
  digitalWrite(LED, (anyInput() || weaponCmd != 0) ? HIGH : LOW);

  delay(5);
}
