// ============================================================
//  Hexapod + Arm Transmitter v3.0  —  Arduino Uno
//  Original by Emre Kalem, 04.2025
//  Updated by Divyanshu Kumar, 2026
//
//  WIRING:
//    NRF24L01  CE=9, CSN=10, SCK=13, MOSI=11, MISO=12
//    Joy1      VCC=5V, GND=GND, VRx=A0, VRy=A1
//    Joy2      VCC=5V, GND=GND, VRx=A2, VRy=A3
//    Pot1=A4   Pot2=A5
//    tg1=2     tg2=3   (INPUT_PULLUP) ← toggle switches
//    tg3=6              (INPUT_PULLUP) ← mode toggle
//    tg4=7   (INPUT_PULLUP) ← Pot2 mode toggle: UP=stepHeight DOWN=stepLength
//    sw1=4     sw2=5   (INPUT_PULLUP) ← push buttons
//
//  PACKET MAP — int message[12]:
//  ┌───────┬──────────────┬──────────────────────────────────────────────┐
//  │ Index │ Source       │ Hexapod mode          / Arm mode             │
//  ├───────┼──────────────┼──────────────────────────────────────────────┤
//  │  [0]  │ Joy1-X (A0)  │ Forward / Backward    / Shoulder up/down     │
//  │  [1]  │ Joy1-Y (A1)  │ Rotate L/R            / Wrist rotation L/R   │
//  │  [2]  │ Joy2-X (A3)  │ Strafe L/R            / Elbow bend/extend    │
//  │  [3]  │ Joy2-Y (A2)  │ Body height           / Base rotation (pan)  │
//  │  [4]  │ Pot1   (A4)  │ Walk speed            / Wrist tilt absolute  │
//  │  [5]  │ Pot2   (A5)  │ Step height           / Gripper open/close   │
//  │  [6]  │ TG1    (D2)  │ Auto-patrol ON/OFF    / Arm wave ON/OFF      │
//  │  [7]  │ TG2    (D3)  │ Demo gait ON/OFF      / Pick-place macro     │
//  │  [8]  │ SW1    (D4)  │ E-STOP (press=stop)   / Arm home (press)     │
//  │  [9]  │ SW2    (D5)  │ Sit/stand toggle      / Gripper full open    │
//  │ [10]  │ TG3    (D6)  │ Mode: 1=Hexapod, 0=Arm (same both modes)     │
//  │ [11]  │ TG4    (D7)  │ 1=Pot2 is stepHeight, 0=Pot2 is stepLength   │
//  └───────┴──────────────┴──────────────────────────────────────────────┘
//
//  NOTE: Mega v4.0 interprets all axes relative to TG3 mode.
//        The transmitter sends raw values always — Mega decides.
// ============================================================

#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

// ── Analog pins ──────────────────────────────────────────────
const int x_pin   = A0;   // Joy1 X  [0]
const int y_pin   = A1;   // Joy1 Y  [1]
const int a_pin   = A3;   // Joy2 X  [2]
const int b_pin   = A2;   // Joy2 Y  [3]
const int pt1_pin = A4;   // Pot1    [4]
const int pt2_pin = A5;   // Pot2    [5]


// ── Digital pins ─────────────────────────────────────────────
const int tg1_pin = 2;    // Toggle1  [6]
const int tg2_pin = 3;    // Toggle2  [7]
const int sw1_pin = 4;    // SW1      [8]
const int sw2_pin = 5;    // SW2      [9]
const int tg3_pin = 6;    // Toggle3  [10] — Mode select
const int tg4_pin = 7;    // Toggle4  [11] — Pot2 mode: 0=stepHeight 1=stepLength

// ── NRF24L01 ─────────────────────────────────────────────────
RF24 transmitter(9, 10);
const uint64_t PIPE = 0xE8E8F0F0E1LL;

int message[12];

// ── Joystick center calibration ──────────────────────────────
int joy1_x_center = 512;
int joy1_y_center = 512;
int joy2_x_center = 512;
int joy2_y_center = 512;

float smooth_x = 512;
float smooth_y = 512;
float smooth_a = 512;
float smooth_b = 512;

// ─────────────────────────────────────────────────────────────
void setup() {
  pinMode(tg1_pin, INPUT_PULLUP);
  pinMode(tg2_pin, INPUT_PULLUP);
  pinMode(tg3_pin, INPUT_PULLUP);
  pinMode(tg4_pin, INPUT_PULLUP);
  pinMode(sw1_pin, INPUT_PULLUP);
  pinMode(sw2_pin, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);
  Serial.println(F("Hexapod+Arm TX v3.0 starting..."));

  transmitter.begin();
  transmitter.setPALevel(RF24_PA_MIN);
  transmitter.setDataRate(RF24_250KBPS);
  transmitter.setRetries(5, 15);
  transmitter.openWritingPipe(PIPE);
  transmitter.stopListening();

  // ── Calibrate joystick centers at startup ────────────────
  long x0=0, y0=0, a0=0, b0=0;
  for (int i = 0; i < 10; i++) {
    x0 += analogRead(x_pin);
    y0 += analogRead(y_pin);
    a0 += analogRead(a_pin);
    b0 += analogRead(b_pin);
    delay(10);
  }
  joy1_x_center = x0 / 10;
  joy1_y_center = y0 / 10;
  joy2_x_center = a0 / 10;
  joy2_y_center = b0 / 10;

  Serial.print(F("Centers: Joy1X=")); Serial.print(joy1_x_center);
  Serial.print(F(" Joy1Y="));         Serial.print(joy1_y_center);
  Serial.print(F(" Joy2X="));         Serial.print(joy2_x_center);
  Serial.print(F(" Joy2Y="));         Serial.println(joy2_y_center);
  // FIX: corrected comment — PULLUP means LOW=switch closed=Arm, HIGH=open=Hexapod
  Serial.println(F("Ready. TG3 down=Arm, TG3 up=Hexapod"));
}

// ─────────────────────────────────────────────────────────────
void loop() {

  // ── Read mode first (TG3) ────────────────────────────────
  int modeVal = digitalRead(tg3_pin);  // 0=Arm, 1=Hexapod (INPUT_PULLUP)
  message[10] = modeVal;
  message[11] = digitalRead(tg4_pin);  // 0=Pot2→stepLength, 1=Pot2→stepHeight

  // ── Read & smooth all 4 joystick axes ────────────────────
  smooth_x += (analogRead(x_pin) - smooth_x) * 0.2f;
  smooth_y += (analogRead(y_pin) - smooth_y) * 0.2f;
  smooth_a += (analogRead(a_pin) - smooth_a) * 0.2f;
  smooth_b += (analogRead(b_pin) - smooth_b) * 0.2f;

  // ── Re-center & deadzone ─────────────────────────────────

  // [0] Joy1-X
  int val_x = (int)smooth_x - joy1_x_center;
  if (abs(val_x) < 20) val_x = 0;
 
  message[0] = constrain(val_x + 512, 0, 1023);

  // [1] Joy1-Y
  int val_y = (int)smooth_y - joy1_y_center;
  if (abs(val_y) < 20) val_y = 0;
  message[1] = constrain(val_y + 512, 0, 1023);

  // [2] Joy2-X
  int val_a = (int)smooth_a - joy2_x_center;
  if (abs(val_a) < 20) val_a = 0;
  message[2] = constrain(val_a + 512, 0, 1023);

  // [3] Joy2-Y
  int val_b = (int)smooth_b - joy2_y_center;
  if (abs(val_b) < 20) val_b = 0;
  message[3] = constrain(val_b + 512, 0, 1023);

  // [4] Pot1 — walk speed (hex) / wrist tilt (arm)
  message[4] = analogRead(pt1_pin);

  // [5] Pot2 — step height (hex) / gripper (arm)
  message[5] = analogRead(pt2_pin);

  // [6] TG1 — auto-patrol (hex) / arm wave (arm)
  message[6] = digitalRead(tg1_pin);

  // [7] TG2 — demo gait (hex) / pick-place macro (arm)
  message[7] = digitalRead(tg2_pin);

  // [8] SW1 — E-STOP (hex) / arm home (arm)
  message[8] = digitalRead(sw1_pin);

  // [9] SW2 — sit/stand (hex) / gripper full open (arm)
  message[9] = digitalRead(sw2_pin);

  // ── E-STOP: freeze motion axes if SW1 pressed in hex mode
  // In arm mode SW1 = arm home, so Mega handles it — don't zero here
  if (message[8] == 0 && message[10] == 1) {  // pressed + hexapod mode
    message[0] = 512;  // fwd/back  → center
    message[1] = 512;  // rotate    → center
    message[2] = 512;  // strafe    → center
    message[3] = 512;  // height    → center
    message[4] = 512;  // speed     → center
    // FIX: was 0 (minimum step height) — changed to 512 (safe center)
    // to avoid IK singularity if this packet slips through RX E-STOP check
    message[5] = 512;  // step ht   → center / safe
  }

  // ── Transmit ─────────────────────────────────────────────
  bool ok = transmitter.write(message, sizeof(message));
  digitalWrite(LED_BUILTIN, ok ? HIGH : LOW);

  // ── Serial debug ─────────────────────────────────────────
  const char* modeStr = (message[10] == 0) ? "ARM" : "HEX";
  Serial.print(F("["));    Serial.print(modeStr);
  Serial.print(F("] J1X:")); Serial.print(message[0]);
  Serial.print(F(" J1Y:")); Serial.print(message[1]);
  Serial.print(F(" J2X:")); Serial.print(message[2]);
  Serial.print(F(" J2Y:")); Serial.print(message[3]);
  Serial.print(F(" P1:"));  Serial.print(message[4]);
  Serial.print(F(" P2:"));  Serial.print(message[5]);
  Serial.print(F(" T1:"));  Serial.print(message[6]);
  Serial.print(F(" T2:"));  Serial.print(message[7]);
  Serial.print(F(" S1:"));  Serial.print(message[8]);
  Serial.print(F(" S2:"));  Serial.print(message[9]);
  Serial.print(F(" TX:"));  Serial.println(ok ? F("OK") : F("FAIL"));

  delay(5);
}
