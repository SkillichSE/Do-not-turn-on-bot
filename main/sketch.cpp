#include <Arduino.h>
#include <Bluepad32.h>

// drive driver 1
#define FL_IN1  25
#define FL_IN2  26
#define FR_IN1  27
#define FR_IN2  14
// drive driver 2
#define RL_IN1  32
#define RL_IN2  33
#define RR_IN1  18
#define RR_IN2  19

// weapon driver
#define W_RPWM  4
#define W_LPWM  16

// encoder and limit switch at bottom position
#define ENC_A      21
#define ENC_B      22
#define LIMIT_PIN  13

#define BUZZER  23
#define LED     2

const int  DEADZONE         = 40;

// set true if motor spins the wrong way
const bool INVERT_FL        = false;
const bool INVERT_FR        = false;
const bool INVERT_RL        = false;
const bool INVERT_RR        = false;
const bool INVERT_WEAPON    = false;
const bool ENC_INVERT       = false;

// pwm cap for 12v motor on 22v pack
const int  WEAPON_MAX_PWM   = 140;
// encoder ticks from bottom to top 80 ticks per turn
const long FLIP_UP_TICKS    = 80;
const int  FLIP_SPEED       = 255;
const int  HOME_SPEED       = 140;
const uint32_t FLIP_TIMEOUT = 2000;
const uint32_t HOME_TIMEOUT = 4000;
// no encoder change for this long means weapon stopped
const uint32_t STALL_MS     = 200;
const uint32_t DATA_TIMEOUT = 500;
const uint32_t DEAD_DISCONNECT_MS = 3000;
// 0 disables idle disconnect
const uint32_t IDLE_DISCONNECT_MS = 5UL * 60UL * 1000UL;

// zero is weapon down
volatile long encCount = 0;
volatile uint8_t encPrev = 0;

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
  if (ENC_INVERT) step = -step;
  encCount += step;
}

long getEnc() { return encCount; }

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

// v is -255 to 255
void setMotor(int in1, int in2, int v, bool invert) {
  if (invert) v = -v;
  v = constrain(v, -255, 255);
  if (v >= 0) {
    ledcWrite(in1, v);
    ledcWrite(in2, 0);
  } else {
    ledcWrite(in1, 0);
    ledcWrite(in2, -v);
  }
}

void setWeapon(int v) {
  if (INVERT_WEAPON) v = -v;
  v = constrain(v, -255, 255);
  int duty = abs(v) * WEAPON_MAX_PWM / 255;
  if (v >= 0) {
    ledcWrite(W_RPWM, duty);
    ledcWrite(W_LPWM, 0);
  } else {
    ledcWrite(W_RPWM, 0);
    ledcWrite(W_LPWM, duty);
  }
}

// thr and turn are -512 to 512
void drive(int thr, int turn) {
  int l = constrain(thr + turn, -512, 512) * 255 / 512;
  int r = constrain(thr - turn, -512, 512) * 255 / 512;
  setMotor(FL_IN1, FL_IN2, l, INVERT_FL);
  setMotor(RL_IN1, RL_IN2, l, INVERT_RL);
  setMotor(FR_IN1, FR_IN2, r, INVERT_FR);
  setMotor(RR_IN1, RR_IN2, r, INVERT_RR);
}

int applyDeadzone(int v) { return abs(v) < DEADZONE ? 0 : v; }

// home lowers to limit switch when zero is unknown
enum FlipState { F_IDLE, F_HOME, F_RAISE, F_LOWER };
FlipState fstate = F_IDLE;
uint32_t fStart = 0, lastEncMs = 0;
long lastEnc = 0;

void flipperStart() {
  if (fstate != F_IDLE) return;
  fstate = homed ? F_RAISE : F_HOME;
  fStart = millis();
  lastEnc = getEnc();
  lastEncMs = fStart;
}

void flipperStop() {
  setWeapon(0);
  fstate = F_IDLE;
}

void flipperUpdate() {
  uint32_t now = millis();
  long pos = getEnc();
  if (pos != lastEnc) { lastEnc = pos; lastEncMs = now; }

  bool stalled = (now - fStart > 150) && (now - lastEncMs > STALL_MS);
  bool timeout = (now - fStart > FLIP_TIMEOUT);

  switch (fstate) {
    case F_IDLE:
      setWeapon(0);
      break;

    case F_HOME:
      setWeapon(-HOME_SPEED);
      if (limitOn || now - fStart > HOME_TIMEOUT) flipperStop();
      break;

    case F_RAISE:
      setWeapon(FLIP_SPEED);
      if (pos >= FLIP_UP_TICKS || stalled || timeout) {
        fstate = F_LOWER;
        fStart = now;
        lastEnc = pos;
        lastEncMs = now;
      }
      break;

    case F_LOWER:
      setWeapon(-FLIP_SPEED);
      if (limitOn) {
        flipperStop();
      } else if (stalled || timeout) {
        homed = false;
        flipperStop();
      }
      break;
  }
}

bool fbActive = false;
uint32_t fbStart = 0;

void feedbackStart() { fbActive = true; fbStart = millis(); }

void feedbackUpdate() {
  bool on = false;
  if (fbActive) {
    uint32_t t = millis() - fbStart;
    on = (t < 250) || (t >= 500 && t < 750);
    if (t >= 750) fbActive = false;
  }
  digitalWrite(BUZZER, on ? HIGH : LOW);
  digitalWrite(LED, on ? HIGH : LOW);
}

ControllerPtr gamepad = nullptr;
bool rumbleDone = false;
bool prevX = false;
uint32_t lastDataMs = 0;
uint32_t lastInputMs = 0;
uint32_t lastHash = 0;

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

bool userActive(ControllerPtr c) {
  return abs(c->axisX()) >= DEADZONE || abs(c->axisY()) >= DEADZONE ||
         abs(c->axisRX()) >= DEADZONE || abs(c->axisRY()) >= DEADZONE ||
         c->buttons() || c->dpad() || c->brake() > 20 || c->throttle() > 20;
}

void stopAll() {
  drive(0, 0);
  flipperStop();
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
    prevX = false;
    lastDataMs = millis();
    lastInputMs = lastDataMs;
    lastHash = 0;
  } else {
    ctl->disconnect();
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  if (gamepad == ctl) {
    Serial.println("Gamepad disconnected, stopping");
    gamepad = nullptr;
    stopAll();
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(BUZZER, OUTPUT);
  pinMode(LED, OUTPUT);

  const int pwmPins[] = {FL_IN1, FL_IN2, FR_IN1, FR_IN2,
                         RL_IN1, RL_IN2, RR_IN1, RR_IN2,
                         W_RPWM, W_LPWM};
  for (int p : pwmPins) ledcAttach(p, 20000, 8);
  stopAll();

  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  encPrev = (digitalRead(ENC_A) << 1) | digitalRead(ENC_B);
  attachInterrupt(digitalPinToInterrupt(ENC_A), encISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), encISR, CHANGE);

  pinMode(LIMIT_PIN, INPUT_PULLUP);
  limitOn = digitalRead(LIMIT_PIN) == LOW;
  limitT = millis();
  if (limitOn) {
    encCount = 0;
    homed = true;
  }

  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.enableVirtualDevice(false);

  Serial.println("Waiting for gamepad (SHARE + PS)...");
}

void loop() {
  BP32.update();
  feedbackUpdate();
  limitUpdate();

  bool alive = gamepad && gamepad->isConnected();
  uint32_t now = millis();

  if (alive && gamepad->hasData()) {
    uint32_t h = stateHash(gamepad);
    if (h != lastHash) { lastHash = h; lastDataMs = now; }
    if (userActive(gamepad)) lastInputMs = now;
  }

  if (!alive || now - lastDataMs > DATA_TIMEOUT) {
    stopAll();
    if (gamepad && now - lastDataMs > DEAD_DISCONNECT_MS) {
      Serial.println("Gamepad silent, dropping");
      dropGamepad();
    }
    delay(5);
    return;
  }

  if (IDLE_DISCONNECT_MS && now - lastInputMs > IDLE_DISCONNECT_MS) {
    Serial.println("Idle timeout, disconnecting gamepad");
    dropGamepad();
    delay(5);
    return;
  }

  if (!rumbleDone) {
    gamepad->playDualRumble(0,   250, 0xC0, 0xC0);
    gamepad->playDualRumble(500, 250, 0xC0, 0xC0);
    feedbackStart();
    rumbleDone = true;
  }

  // up is negative y
  int thr  = -applyDeadzone(gamepad->axisY());
  int turn =  applyDeadzone(gamepad->axisX());
  drive(thr, turn);

  // a is the cross button on dualshock 4
  bool x = gamepad->a();
  if (x && !prevX) flipperStart();
  prevX = x;
  flipperUpdate();

  delay(5);
}
