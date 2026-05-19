// ============================================================
//  Hexapod + Robotic Arm Receiver v4.0  —  Arduino Mega 2560
//  Divyanshu Kumar, 2026
//
//  v4.0 KEY CHANGES vs v3.2:
//  ✦ Packet expanded to int[12] — matches Uno v3.0 transmitter
//  ✦ TG3 (message[10]) → mode select: 1=Hexapod, 0=Arm
//  ✦ HEXAPOD MODE new features:
//      Joy2-X  → strafe left/right
//      Joy2-Y  → body height up/down  (re-enables BODY_HEIGHT)
//      Pot1    → walk speed (was wrist tilt — now mode-split)
//      Pot2    → step height
//      SW2     → sit / stand toggle
//      TG2     → demo gait (wave sequence)
//  ✦ ARM MODE new features:
//      Joy1-Y  → wrist rotation rate control
//      Joy2-Y  → base rotation rate control (new servo, pin 44)
//      SW1     → arm home (was E-STOP — now mode-split)
//      SW2     → gripper full open (instant)
//      TG2     → pick-and-place macro
//  ✦ E-STOP (SW1 in hex mode) still halts everything regardless
//
//  PACKET MAP (from Uno v3.0):
//    message[0]  Joy1-X  → hex:fwd/back     arm:shoulder     0–1023
//    message[1]  Joy1-Y  → hex:rotate       arm:wristRot     0–1023
//    message[2]  Joy2-X  → hex:strafe       arm:elbow        0–1023
//    message[3]  Joy2-Y  → hex:bodyHeight   arm:baseRot      0–1023
//    message[4]  Pot1    → hex:speed        arm:wristTilt    0–1023
//    message[5]  Pot2    → hex:stepHeight   arm:gripper      0–1023
//    message[6]  TG1     → hex:autoPatrol   arm:armWave      0=ON 1=OFF
//    message[7]  TG2     → hex:demoGait     arm:pickPlace    0=ON 1=OFF
//    message[8]  SW1     → hex:E-STOP       arm:armHome      0=pressed
//    message[9]  SW2     → hex:sit/stand    arm:gripOpen     0=pressed
//    message[10] TG3     → mode             1=Hexapod 0=Arm
//    message[11] Pot3    → hex:stepLength   arm:(unused)   0–1023
//
//  PIN SUMMARY:
//    NRF24L01  CE=2,  CS=3   (SPI: 50,51,52,53)
//    Leg RF    28,29,30  | Leg RM  25,26,27  | Leg RR  22,23,24
//    Leg LR    31,32,33  | Leg LM  34,35,36  | Leg LF  37,38,39
//    Shoulder  L=41 R=42 | Elbow=43
//    BaseRot=44  (NEW)
//    WristTilt=7  WristRot=8  Gripper=12
// ============================================================

#include <Servo.h>
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include <math.h>

#define RAD_TO_DEG(x) ((x) * 180.0f / M_PI)
#define DEG_TO_RAD(x) ((x) * M_PI  / 180.0f)
#define POW2(x)       ((x) * (x))

// ============================================================
//  FEATURE GATES
// ============================================================
#define ENABLE_ESTOP       1
#define ENABLE_ARM         1
#define ENABLE_BODY_HEIGHT 1   // ← re-enabled, routed to Joy2-Y in hex mode

// ============================================================
//  AUTO-PATROL SETTINGS
// ============================================================
#define PATROL_FORWARD_CYCLES  80
#define PATROL_TURN_CYCLES     10
#define PATROL_WALK_SPEED      0.7f
#define PATROL_TURN_SPEED      0.6f
#define PATROL_STEP_HEIGHT     80.0f
#define PATROL_STEP_LENGTH     50.0f 

enum PatrolState { PATROL_FORWARD, PATROL_TURNING };

// ============================================================
//  DEMO GAIT SETTINGS
// ============================================================
#define DEMO_CYCLE_STEPS  120   // steps per full demo loop

// ============================================================
//  ARM WAVE SETTINGS
// ============================================================
#define WAVE_SHOULDER_CENTER  90.0f
#define WAVE_SHOULDER_AMP     35.0f
#define WAVE_ELBOW_CENTER     80.0f
#define WAVE_ELBOW_AMP        30.0f
#define WAVE_PERIOD_MS       2500.0f
#define WAVE_ELBOW_PHASE      0.4f
#define WAVE_GRIPPER_OPEN     30
#define WAVE_GRIPPER_CLOSE    90

// ============================================================
//  PICK-AND-PLACE MACRO SETTINGS
// ============================================================
// Sequence: lower → close gripper → raise → swing → open
#define PP_LOWER_SHOULDER   130.0f   // reach down
#define PP_LOWER_ELBOW      140.0f
#define PP_RAISE_SHOULDER    50.0f   // lift
#define PP_RAISE_ELBOW       60.0f
#define PP_SWING_BASE       140.0f   // pan to drop zone
#define PP_STEP_MS          1500      // ms per macro step

// ============================================================
//  SIT / STAND SETTINGS
// ============================================================
#define SIT_Z    -10.0f   // mm — low stance
#define STAND_Z  -50.0f   // mm — normal stance (DEFAULT_Z)

// ============================================================
//  ARM PINS
// ============================================================
#define PIN_SHOULDER_L   41
#define PIN_SHOULDER_R   42
#define PIN_ELBOW        43
#define PIN_BASE_ROT     44   // NEW — base rotation servo
#define PIN_WRIST_TILT    7
#define PIN_WRIST_ROT     8
#define PIN_GRIPPER      12

#define MG90S_MIN_US   700
#define MG90S_MAX_US  2300

#define SHOULDER_MIN   30
#define SHOULDER_MAX  150
#define ELBOW_MIN      10
#define ELBOW_MAX     170
#define WRIST_TILT_MIN  0
#define WRIST_TILT_MAX 180
#define WRIST_ROT_MIN   0
#define WRIST_ROT_MAX  180
#define WRIST_ROT_HOME 90
#define BASE_ROT_MIN    0
#define BASE_ROT_MAX  180
#define BASE_ROT_HOME  90
#define GRIPPER_MIN    30
#define GRIPPER_MAX   150

#define SHOULDER_RATE  0.4f
#define ELBOW_RATE     0.5f
#define WRIST_ROT_RATE 0.6f
#define BASE_ROT_RATE  0.5f
#define ARM_SMOOTH     0.06f 

// ============================================================
//  ROBOT GEOMETRY (mm)
// ============================================================
static const float COXA_LENGTH  = 38.0f;
static const float FEMUR_LENGTH = 86.25f;
static const float TIBIA_LENGTH = 160.5f;

// ============================================================
//  GAIT PARAMETERS
// ============================================================
static const float STEP_LENGTH_MIN   = 10.0f;
static const float STEP_LENGTH_MAX   = 90.0f;
static const float STEP_LENGTH_BASE  = 45.0f;  // default when no pot
// static const float STEP_LENGTH_BASE  = 45.0f;
static const float STEP_HEIGHT_MIN   = 50.0f;
static const float STEP_HEIGHT_MAX   = 120.0f;
static const float DEFAULT_Z         = -40.0f;
static const float BODY_HEIGHT_RANGE = 30.0f;
static const float ROTATION_STEP_RAD = DEG_TO_RAD(12.0f);

static const float T_INC_MIN  = 0.008f;
static const float T_INC_MAX  = 0.040f;
// static const float WALK_SMOOTH_XY    = 0.28f;   // slower XY tracking
// static const float WALK_SMOOTH_Z     = 0.75f;   // CRITICAL — faster Z recovery
// static const float STOP_SMOOTH_XY    = 0.08f;   // gentler stop
// static const float STOP_SMOOTH_Z     = 0.18f;
static const float WALK_SMOOTH_XY = 0.55f;
static const float WALK_SMOOTH_Z  = 0.92f;
static const float STOP_SMOOTH_XY = 0.12f;
static const float STOP_SMOOTH_Z  = 0.25f;

static const int   JOY_DEADBAND         = 100;
static const unsigned long PACKET_TIMEOUT_MS = 250;

static const bool GROUP_B[6] = { false, true, false, true, false, true };

static float TRIM[6][3] = {
  {0.f,0.f,0.f},{0.f,0.f,0.f},{0.f,0.f,0.f},
  {0.f,0.f,0.f},{0.f,0.f,0.f},{0.f,0.f,0.f},
};
static const bool INVERT[6][3] = {
  {false,false,false},{false,false,false},{false,false,false},
  {false,false,false},{false,false,false},{false,false,false},
};

static const float COXA_MIN  =  35.0f;
static const float COXA_MAX  = 135.0f;
static const float FEMUR_MIN =   0.0f;
static const float FEMUR_MAX = 180.0f;
static const float TIBIA_MIN =   0.0f;
static const float TIBIA_MAX = 180.0f;

// ============================================================
//  DATA STRUCTURES
// ============================================================
struct Vector {
  float x, y, z;
  Vector(float _x=0.f,float _y=0.f,float _z=0.f):x(_x),y(_y),z(_z){}
};

struct Leg {
  Vector home, origin, target;
  int    servoPins[3];
  Servo  servos[3];
  Leg(int p0,int p1,int p2,float ox,float oy)
    :servoPins{p0,p1,p2},origin(ox,oy,0.f){}
  void init(){
    origin.z=atan2(origin.y,origin.x);
    for(int i=0;i<3;i++){servos[i].attach(servoPins[i]);servos[i].write(90);}
  }
};

// ── Leg table ─────────────────────────────────────────────────
Leg legs[6]={
  Leg(28,29,30, 100.94f,-57.11f),    // 0 RF
  Leg(25,26,27,   0.00f,-105.72f),   // 1 RM
  Leg(22,23,24,-100.94f,-57.11f),    // 2 RR
  Leg(31,32,33,-100.94f, 57.11f),    // 3 LR 
  Leg(34,35,36,   0.00f, 105.72f),   // 4 LM
  Leg(37,38,39, 100.94f, 57.11f),    // 5 LF
};

// ── NRF24L01 ──────────────────────────────────────────────────
RF24 radio(2, 3);
const uint64_t PIPE = 0xE8E8F0F0E1LL;

// ── Radio data — int[12] matches Uno v3.0 EXACTLY ────────────
int radioData[12] = {512,512,512,512,512,512,1,1,1,1,1,0};
//                   j1x j1y j2x j2y pt1 pt2 t1 t2 s1 s2 t3 rsv

// ── Arm servos ────────────────────────────────────────────────
Servo shoulderL, shoulderR, elbowServo;
Servo baseRot, wristTilt, wristRot, gripperServo;

// ── Arm state ─────────────────────────────────────────────────
float shoulderAngle  = 90.0f;
float elbowAngle     = 90.0f;
float wristRotAngle  = (float)WRIST_ROT_HOME;
float baseRotAngle   = (float)BASE_ROT_HOME;

// ── Hexapod state ─────────────────────────────────────────────
float t                = 0.0f;
float bodyHeightOffset = 0.0f;
float stepLengthScale  = STEP_LENGTH_BASE; 
int   smoothRampFrames = 0;
unsigned long lastPacketTime = 0;

// ── Sit/stand state ───────────────────────────────────────────
bool  isSitting      = false;
bool  lastSW2hex     = false;   // edge detect for sit/stand toggle

// ── Auto-Patrol state ─────────────────────────────────────────
// PatrolState patrolState     = PATROL_FORWARD;
// int         patrolStepCount = 0;
// bool        lastPatrolOn    = false;

PatrolState patrolState     = PATROL_FORWARD;
float       patrolTAccum    = 0.f;
bool        lastPatrolOn    = false;

// ── Demo Gait state ───────────────────────────────────────────
int  demoStepCount = 0;
bool lastDemoOn    = false;

// ── Arm Wave state ────────────────────────────────────────────
bool          lastWaveOn       = false;
unsigned long waveStartTime    = 0;
float         waveExitShoulder = 90.0f;
float         waveExitElbow    = 90.0f;
// FIX: also save wrist/base angles on wave entry to restore on exit
float         waveExitWristRot = (float)WRIST_ROT_HOME;
float         waveExitBaseRot  = (float)BASE_ROT_HOME;

// ── Pick-and-Place macro state ────────────────────────────────
enum PPState { PP_IDLE, PP_LOWER, PP_GRIP, PP_RAISE, PP_SWING, PP_DROP, PP_RETURN };
PPState       ppState        = PP_IDLE;
unsigned long ppStepStart    = 0;
bool          lastPPOn       = false;

// ── SW2 edge detect for arm mode (gripper open) ───────────────
// FIX: dedicated edge-detect variable for SW2 in arm mode
bool          lastSW2arm     = false;

// ── Serial override (Web Visualizer) ─────────────────────────
float serialFwd=0.f, serialStrafe=0.f, serialRot=0.f;
unsigned long lastSerialTime=0;
const unsigned long SERIAL_ACTIVE_TIMEOUT=500;

// ── Telemetry ─────────────────────────────────────────────────
unsigned long lastTelemetryTime=0;
const unsigned long TELEMETRY_INTERVAL=100;
const int   VOLTAGE_PIN=A0, CURRENT_PIN=A1, TEMP_PIN=A2;
const float V_REF=5.f, VOLTAGE_DIVIDER_RATIO=3.f;
const float CURRENT_SENSOR_SENSITIVITY=0.066f, TEMP_SENSOR_BETA=3950.f;

// ── Forward declarations ──────────────────────────────────────
void   calculate_fk(const Vector&,const Vector&,Vector[4]);
void   calculate_ik(const Leg&,Vector&);
Vector get_foot_target(float,int,float,float,float,float);
void   set_foot_positions(float,float,float,float,float);
void   go_home();
void   read_radio();
void   read_serial();
float  map_joy(int raw);
void   writeMG90S(Servo&,int);
void   arm_apply(int wristTiltRaw, int gripperRaw);
void   arm_home_position();
void   run_auto_patrol();
void   run_demo_gait();
void   run_arm_wave(int wristTiltRaw);
void   run_pick_place(int gripperRaw);
void   apply_sit_stand(bool sit);
void   sendTelemetry();

// ============================================================
//  SETUP
// ============================================================
// void setup(){
//   Serial.begin(115200);

//   radio.begin();
//   radio.setPALevel(RF24_PA_MIN);
//   radio.setDataRate(RF24_250KBPS);
//   radio.setRetries(5, 15);
//   radio.openReadingPipe(1, PIPE);
//   radio.startListening();

//   // ── Hexapod legs ──────────────────────────────────────────
//   Vector np[4];
//   for(int i=0;i<6;i++){
//     legs[i].init();
//     calculate_fk(Vector(0.f,M_PI/4.f,-M_PI/2.f),legs[i].origin,np);
//     legs[i].home=np[3];
//     legs[i].home.z=DEFAULT_Z;
//     legs[i].target=legs[i].home;
//   }

//   // ── Robotic arm ───────────────────────────────────────────
//   shoulderL.attach(PIN_SHOULDER_L);
//   shoulderR.attach(PIN_SHOULDER_R);
//   elbowServo.attach(PIN_ELBOW);
//   baseRot.attach(PIN_BASE_ROT);
//   wristTilt.attach(PIN_WRIST_TILT);
//   wristRot.attach(PIN_WRIST_ROT);
//   gripperServo.attach(PIN_GRIPPER);
//   arm_home_position();

//   lastPacketTime=millis();
//   delay(1000);
//   Serial.println(F("Hexapod+Arm v4.0 ready — TG3: 1=HEX 0=ARM"));
// }
void setup(){
  Serial.begin(115200);
  Serial.println(F("1 - Serial OK"));

  radio.begin();
  Serial.println(F("2 - Radio begin OK"));
  
  radio.setPALevel(RF24_PA_MIN);
  radio.setDataRate(RF24_250KBPS);
  radio.setRetries(5, 15);
  radio.openReadingPipe(1, PIPE);
  radio.startListening();
  Serial.println(F("3 - Radio config OK"));

  Vector np[4];
  for(int i=0;i<6;i++){
    legs[i].init();
    Serial.print(F("4 - Leg init OK: ")); Serial.println(i);
    calculate_fk(Vector(0.f,M_PI/4.f,-M_PI/2.f),legs[i].origin,np);
    legs[i].home=np[3];
    legs[i].home.z=DEFAULT_Z;
    legs[i].target=legs[i].home;
  }
  Serial.println(F("5 - All legs OK"));

  shoulderL.attach(PIN_SHOULDER_L);
  shoulderR.attach(PIN_SHOULDER_R);
  elbowServo.attach(PIN_ELBOW);
  baseRot.attach(PIN_BASE_ROT);
  wristTilt.attach(PIN_WRIST_TILT);
  wristRot.attach(PIN_WRIST_ROT);
  gripperServo.attach(PIN_GRIPPER);
  Serial.println(F("6 - Servos attached OK"));

  arm_home_position();
  Serial.println(F("7 - Arm home OK"));

  lastPacketTime=millis();
  delay(1000);
  Serial.println(F("Hexapod+Arm v4.0 ready"));
}

// ============================================================
//  LOOP
// ============================================================
void loop(){
  read_radio();
  read_serial();

  unsigned long now=millis();
  if(now-lastTelemetryTime>=TELEMETRY_INTERVAL){
    lastTelemetryTime=now;
    sendTelemetry();
  }

  bool radioAlive  = (millis()-lastPacketTime  <= PACKET_TIMEOUT_MS);
  bool serialAlive = (millis()-lastSerialTime  <= SERIAL_ACTIVE_TIMEOUT);

  if(!radioAlive && !serialAlive){ go_home(); delay(10); return; }

  // ── Decode packet fields ──────────────────────────────────
  bool armMode    = radioAlive && (radioData[10] == 0);  // TG3 low = arm mode
  bool tg1On      = radioAlive && (radioData[6]  == 0);  // TG1 low = on
  bool tg2On      = radioAlive && (radioData[7]  == 0);  // TG2 low = on
  bool sw1Pressed = radioAlive && (radioData[8]  == 0);  // SW1 pressed
  bool sw2Pressed = radioAlive && (radioData[9]  == 0);  // SW2 pressed

  // ── GLOBAL E-STOP — only in hexapod mode ─────────────────
  #if ENABLE_ESTOP
  if(!armMode && sw1Pressed){ go_home(); delay(10); return; }
  #endif

  // FIX: reset arm-mode SW2 edge state when not in arm mode
  // so re-entering arm mode with SW2 held doesn't misfires
  if(!armMode) lastSW2arm = sw2Pressed;

  // ══════════════════════════════════════════════════════════
  //  HEXAPOD MODE
  // ══════════════════════════════════════════════════════════
  if(!armMode){

    // ── SW2 — Sit / Stand toggle (edge detect) ─────────────
    if(sw2Pressed && !lastSW2hex){
      isSitting = !isSitting;
      apply_sit_stand(isSitting);
      Serial.println(isSitting ? F("[HEX] Sitting") : F("[HEX] Standing"));
    }
    lastSW2hex = sw2Pressed;

    // ── TG1 — Auto-patrol ──────────────────────────────────
    // if(tg1On){
    //   if(!lastPatrolOn){
    //     patrolState=PATROL_FORWARD; patrolStepCount=0;
    //     smoothRampFrames=0; lastPatrolOn=true;
    //     Serial.println(F("[PATROL] Started"));
    //   }
    //   run_auto_patrol();
    //   goto ARM_UPDATE;
    // } else { lastPatrolOn=false; }

    if(tg1On){
  if(!lastPatrolOn){
    patrolState      = PATROL_FORWARD;
    patrolTAccum     = 0.f;
    smoothRampFrames = 0;
    t                = 0.f;
    lastPatrolOn     = true;
    Serial.println(F("[PATROL] Started"));
    goto ARM_UPDATE;   // skip first frame, let ramp build
  }
  run_auto_patrol();
  goto ARM_UPDATE;
} else { lastPatrolOn=false; }

    // ── TG2 — Demo gait ────────────────────────────────────
    if(tg2On){
      if(!lastDemoOn){ demoStepCount=0; lastDemoOn=true;
        Serial.println(F("[DEMO] Started")); }
      run_demo_gait();
      goto ARM_UPDATE;
    } else { lastDemoOn=false; }

    // ── NORMAL GAIT ────────────────────────────────────────
    {
      float fwd=0.f, strafe=0.f, rot=0.f;
      if(serialAlive){
        fwd=serialFwd; strafe=serialStrafe; rot=serialRot;
      } else if(radioAlive){
        fwd    = map_joy(radioData[0]);   // Joy1-X → forward/back
        rot    = map_joy(radioData[1]);   // Joy1-Y → rotate
        strafe = map_joy(radioData[2]);   // Joy2-X → strafe
      }

      // ── Joy2-Y → body height ─────────────────────────────
      #if ENABLE_BODY_HEIGHT
      if(radioAlive){
        float heightJoy = map_joy(radioData[3]);  // Joy2-Y
        bodyHeightOffset += heightJoy * BODY_HEIGHT_RANGE * 0.1f;
        // FIX: clamp offset itself so it can't drift unboundedly,
        // then apply relative to current home.z base
        bodyHeightOffset = constrain(bodyHeightOffset,
                                     -BODY_HEIGHT_RANGE, BODY_HEIGHT_RANGE);
        float targetZ = (isSitting ? SIT_Z : STAND_Z) + bodyHeightOffset;
        targetZ = constrain(targetZ,
                            (isSitting ? SIT_Z : STAND_Z) - BODY_HEIGHT_RANGE,
                            (isSitting ? SIT_Z : STAND_Z) + BODY_HEIGHT_RANGE);
        for(int i=0;i<6;i++) legs[i].home.z = targetZ;
      }
      #endif

      // // Pot1 → speed, Pot2 → step height
      // float speedScale = radioAlive ? max(radioData[4]/1023.0f, 0.3f) : 0.6f;
      // float stepHeight = radioAlive
      //   ? map(radioData[5], 0, 1023, (int)STEP_HEIGHT_MIN, (int)STEP_HEIGHT_MAX)
      //   : 80.0f;

      // set_foot_positions(fwd, strafe, rot, speedScale, stepHeight);
      // Pot1 → speed, Pot2 → step height, Pot3 → step length
      float speedScale = radioAlive ? max(radioData[4]/1023.0f, 0.3f) : 0.6f;
      float stepHeight;
     if (radioAlive && radioData[11] == 0) {
        // TG4 DOWN — Pot2 controls step length, step height locked to default
        stepHeight      = 80.0f;
        stepLengthScale = STEP_LENGTH_MIN + (radioData[5] / 1023.0f)
                          * (STEP_LENGTH_MAX - STEP_LENGTH_MIN);
      } else {
        // TG4 UP — Pot2 controls step height (normal mode), step length locked
        stepHeight      = radioAlive
          ? map(radioData[5], 0, 1023, (int)STEP_HEIGHT_MIN, (int)STEP_HEIGHT_MAX)
          : 80.0f;
        stepLengthScale = STEP_LENGTH_BASE;
      }
      set_foot_positions(fwd, strafe, rot, speedScale, stepHeight);
    }

  // ══════════════════════════════════════════════════════════
  //  ARM MODE
  // ══════════════════════════════════════════════════════════
  } else {
    // Hold hexapod still while in arm mode
    go_home();
  }

  // ── ARM UPDATE (runs in both modes — arm always live) ────
  ARM_UPDATE:
  #if ENABLE_ARM
  if(radioAlive){
    int wristTiltRaw = radioData[4];   // Pot1 → wrist tilt  (arm mode)
    int gripperRaw   = radioData[5];   // Pot2 → gripper

    if(armMode){

      // ── SW1 → Arm home ───────────────────────────────────
      if(sw1Pressed){
        arm_home_position();
        goto END_ARM;
      }

      // ── SW2 → Gripper full open (edge detect) ────────────
      // FIX: was continuous fire with no edge detect — servos froze
      if(sw2Pressed && !lastSW2arm){
        writeMG90S(gripperServo, GRIPPER_MIN);
      }
      lastSW2arm = sw2Pressed;
      if(sw2Pressed){
        goto END_ARM;
      }

      // ── TG1 → Arm wave ───────────────────────────────────
      if(tg1On){
        if(!lastWaveOn){
          waveStartTime    = millis();
          waveExitShoulder = shoulderAngle;
          waveExitElbow    = elbowAngle;
          // FIX: save wrist/base angles so they restore on wave exit
          waveExitWristRot = wristRotAngle;
          waveExitBaseRot  = baseRotAngle;
          lastWaveOn       = true;
        }
        run_arm_wave(wristTiltRaw);
        goto END_ARM;
      } else {
        if(lastWaveOn){
          shoulderAngle  = waveExitShoulder;
          elbowAngle     = waveExitElbow;
          // FIX: restore wrist/base on wave exit (were not restored before)
          wristRotAngle  = waveExitWristRot;
          baseRotAngle   = waveExitBaseRot;
          lastWaveOn     = false;
        }
      }

      // ── TG2 → Pick-and-place macro ───────────────────────
      if(tg2On){
        if(!lastPPOn){ ppState=PP_LOWER; ppStepStart=millis(); lastPPOn=true;
          Serial.println(F("[PP] Started")); }
        run_pick_place(gripperRaw);
        goto END_ARM;
      } else {
        if(lastPPOn){ ppState=PP_IDLE; lastPPOn=false; }
      }

      // ── MANUAL ARM CONTROL ───────────────────────────────

      // Joy1-X → shoulder
      shoulderAngle += map_joy(radioData[0]) * SHOULDER_RATE;
      shoulderAngle  = constrain(shoulderAngle, SHOULDER_MIN, SHOULDER_MAX);

      // Joy1-Y → wrist rotation
      wristRotAngle += map_joy(radioData[1]) * WRIST_ROT_RATE;
      wristRotAngle  = constrain(wristRotAngle, WRIST_ROT_MIN, WRIST_ROT_MAX);

      // Joy2-X → elbow
      elbowAngle += map_joy(radioData[2]) * ELBOW_RATE;
      elbowAngle  = constrain(elbowAngle, ELBOW_MIN, ELBOW_MAX);

      // Joy2-Y → base rotation
      baseRotAngle += map_joy(radioData[3]) * BASE_ROT_RATE;
      baseRotAngle  = constrain(baseRotAngle, BASE_ROT_MIN, BASE_ROT_MAX);

      arm_apply(wristTiltRaw, gripperRaw);

    }
    // FIX: removed arm_apply() call from hex-mode else branch —
    // Pot1 was simultaneously controlling walk speed AND wrist tilt
    // in hex mode. Arm stays fully frozen in hex mode now.
  }
  END_ARM:;
  #endif

  delay(10);
}

// ============================================================
//  APPLY SIT / STAND
// ============================================================
void apply_sit_stand(bool sit){
  float targetZ = sit ? SIT_Z : STAND_Z;
  for(int i=0;i<6;i++) legs[i].home.z = targetZ;
  bodyHeightOffset = 0.f;  // reset height offset on mode change
}

// ============================================================
//  AUTO-PATROL
// ============================================================
// void run_auto_patrol(){
//   switch(patrolState){
//     case PATROL_FORWARD:
//       set_foot_positions(PATROL_WALK_SPEED,0.f,0.f,
//                          PATROL_WALK_SPEED,PATROL_STEP_HEIGHT);
//       if(++patrolStepCount >= PATROL_FORWARD_STEPS){
//         patrolStepCount=0; patrolState=PATROL_TURNING;
//         smoothRampFrames=0;
//         Serial.println(F("[PATROL] Turning 180°"));
//       }
//       break;
//     case PATROL_TURNING:
//       set_foot_positions(0.f,0.f,PATROL_TURN_SPEED,
//                          PATROL_TURN_SPEED,PATROL_STEP_HEIGHT);
//       if(++patrolStepCount >= PATROL_TURN_STEPS){
//         patrolStepCount=0; patrolState=PATROL_FORWARD;
//         smoothRampFrames=0;
//         Serial.println(F("[PATROL] Walking forward"));
//       }
//       break;
//   }
// }
// void run_auto_patrol() {
//   switch (patrolState) {
//     case PATROL_FORWARD:
//       // was: set_foot_positions(PATROL_WALK_SPEED, 0, 0, ...)
//       // FIX: also reset bodyHeightOffset so height doesn't drift in patrol
//       set_foot_positions(PATROL_WALK_SPEED, 0.f, 0.f,
//                          PATROL_WALK_SPEED, PATROL_STEP_HEIGHT);
//       if (++patrolStepCount >= PATROL_FORWARD_STEPS) {
//         patrolStepCount  = 0;
//         patrolState      = PATROL_TURNING;
//         smoothRampFrames = 0;
//         t = 0.f;   // ← ADD THIS — reset gait phase on direction change
//         Serial.println(F("[PATROL] Turning 180°"));
//       }
//       break;
//     case PATROL_TURNING:
//       set_foot_positions(0.f, 0.f, PATROL_TURN_SPEED,
//                          PATROL_TURN_SPEED, PATROL_STEP_HEIGHT);
//       if (++patrolStepCount >= PATROL_TURN_STEPS) {
//         patrolStepCount  = 0;
//         patrolState      = PATROL_FORWARD;
//         smoothRampFrames = 0;
//         t = 0.f;   // ← ADD THIS
//         Serial.println(F("[PATROL] Walking forward"));
//       }
//       break;
//   }
// }
// FIND AND DELETE the entire run_auto_patrol() function, REPLACE WITH:
void run_auto_patrol() {
  stepLengthScale = PATROL_STEP_LENGTH; 
  switch (patrolState) {
    case PATROL_FORWARD:
      set_foot_positions(PATROL_WALK_SPEED, 0.f, 0.f,
                         PATROL_WALK_SPEED, PATROL_STEP_HEIGHT);
      patrolTAccum += T_INC_MIN + PATROL_WALK_SPEED * (T_INC_MAX - T_INC_MIN);
      if (patrolTAccum >= PATROL_FORWARD_CYCLES) {
        patrolTAccum     = 0.f;
        patrolState      = PATROL_TURNING;
        smoothRampFrames = 0;
        t                = 0.f;
        Serial.println(F("[PATROL] Turning 180"));
      }
      break;

    case PATROL_TURNING:
      set_foot_positions(0.f, 0.f, PATROL_TURN_SPEED,
                         PATROL_TURN_SPEED, PATROL_STEP_HEIGHT);
      patrolTAccum += T_INC_MIN + PATROL_TURN_SPEED * (T_INC_MAX - T_INC_MIN);
      if (patrolTAccum >= PATROL_TURN_CYCLES) {
        patrolTAccum     = 0.f;
        patrolState      = PATROL_FORWARD;
        smoothRampFrames = 0;
        t                = 0.f;
        Serial.println(F("[PATROL] Walking forward"));
      }
      break;
  }
}
// ============================================================
//  DEMO GAIT  — body sway + forward/back/turn sequence
// ============================================================
void run_demo_gait(){
  int phase = demoStepCount % DEMO_CYCLE_STEPS;

  float fwd=0.f, rot=0.f, strafe=0.f;
  float speed=0.6f;
  float stepH=90.0f;

  if     (phase < 30)  { fwd=0.8f;  rot=0.f;  }  // walk forward
  else if(phase < 50)  { fwd=0.f;   rot=0.7f; }  // spin right
  else if(phase < 80)  { fwd=-0.6f; rot=0.f;  }  // walk back
  else if(phase < 100) { fwd=0.f;   rot=-0.7f;}  // spin left
  else                 { fwd=0.f;   strafe=0.5f;} // strafe right

  set_foot_positions(fwd, strafe, rot, speed, stepH);
  demoStepCount++;
}

// ============================================================
//  ARM WAVE
// ============================================================
// void run_arm_wave(int wristTiltRaw){
//   float elapsed=(float)(millis()-waveStartTime);
//   float phase=(elapsed/WAVE_PERIOD_MS)*2.0f*M_PI;

//   shoulderAngle=constrain(
//     WAVE_SHOULDER_CENTER+WAVE_SHOULDER_AMP*sin(phase),
//     SHOULDER_MIN,SHOULDER_MAX);
//   elbowAngle=constrain(
//     WAVE_ELBOW_CENTER+WAVE_ELBOW_AMP*sin(phase+WAVE_ELBOW_PHASE*2.f*M_PI),
//     ELBOW_MIN,ELBOW_MAX);

//   shoulderL.write((int)shoulderAngle);
//   shoulderR.write(180-(int)shoulderAngle);
//   elbowServo.write((int)elbowAngle);

//   writeMG90S(wristTilt, map(wristTiltRaw,0,1023,WRIST_TILT_MIN,WRIST_TILT_MAX));
//   writeMG90S(wristRot,  WRIST_ROT_HOME);
//   // FIX: use writeMG90S for baseRot — consistent with all other MG-series servos
//   writeMG90S(baseRot, (int)baseRotAngle);

//   float normalized=sin(phase);
//   writeMG90S(gripperServo,(normalized>0.f)?WAVE_GRIPPER_OPEN:WAVE_GRIPPER_CLOSE);
// }

void run_arm_wave(int wristTiltRaw) {
  float elapsed = (float)(millis() - waveStartTime);
  float phase = (elapsed / WAVE_PERIOD_MS) * 2.0f * M_PI;

  // Target angles
  float targetShoulder = WAVE_SHOULDER_CENTER + WAVE_SHOULDER_AMP * sin(phase);
  float targetElbow    = WAVE_ELBOW_CENTER + WAVE_ELBOW_AMP * sin(phase + WAVE_ELBOW_PHASE * 2.f * M_PI);

  // Smooth toward target instead of snapping
  shoulderAngle += (targetShoulder - shoulderAngle) * ARM_SMOOTH;
  elbowAngle    += (targetElbow    - elbowAngle)    * ARM_SMOOTH;

  shoulderAngle = constrain(shoulderAngle, SHOULDER_MIN, SHOULDER_MAX);
  elbowAngle    = constrain(elbowAngle,    ELBOW_MIN,    ELBOW_MAX);

  shoulderL.write((int)shoulderAngle);
  shoulderR.write(180 - (int)shoulderAngle);
  elbowServo.write((int)elbowAngle);

  writeMG90S(wristTilt, map(wristTiltRaw, 0, 1023, WRIST_TILT_MIN, WRIST_TILT_MAX));
  writeMG90S(wristRot,  WRIST_ROT_HOME);
  writeMG90S(baseRot,   (int)baseRotAngle);

  float normalized = sin(phase);
  writeMG90S(gripperServo, (normalized > 0.f) ? WAVE_GRIPPER_OPEN : WAVE_GRIPPER_CLOSE);
}

// ============================================================
//  PICK-AND-PLACE MACRO
// ============================================================
// void run_pick_place(int gripperRaw){
//   unsigned long elapsed = millis() - ppStepStart;
//   if(elapsed < (unsigned long)PP_STEP_MS) return;  // still in current step
//   ppStepStart = millis();  // advance to next step

//   switch(ppState){
//     case PP_LOWER:
//       shoulderAngle = PP_LOWER_SHOULDER;
//       elbowAngle    = PP_LOWER_ELBOW;
//       shoulderL.write((int)shoulderAngle);
//       shoulderR.write(180-(int)shoulderAngle);
//       elbowServo.write((int)elbowAngle);
//       Serial.println(F("[PP] Lowering"));
//       ppState = PP_GRIP;
//       break;

//     case PP_GRIP:
//       writeMG90S(gripperServo, GRIPPER_MAX);  // close
//       Serial.println(F("[PP] Gripping"));
//       ppState = PP_RAISE;
//       break;

//     case PP_RAISE:
//       shoulderAngle = PP_RAISE_SHOULDER;
//       elbowAngle    = PP_RAISE_ELBOW;
//       shoulderL.write((int)shoulderAngle);
//       shoulderR.write(180-(int)shoulderAngle);
//       elbowServo.write((int)elbowAngle);
//       Serial.println(F("[PP] Raising"));
//       ppState = PP_SWING;
//       break;

//     case PP_SWING:
//       baseRotAngle = PP_SWING_BASE;
//       // FIX: use writeMG90S for baseRot — consistent with all other MG-series servos
//       writeMG90S(baseRot, (int)baseRotAngle);
//       Serial.println(F("[PP] Swinging to drop zone"));
//       ppState = PP_DROP;
//       break;

//     case PP_DROP:
//       writeMG90S(gripperServo, GRIPPER_MIN);  // open — drop
//       Serial.println(F("[PP] Dropping"));
//       ppState = PP_RETURN;
//       break;

//     case PP_RETURN:
//       arm_home_position();
//       Serial.println(F("[PP] Done — returning home"));
//       ppState = PP_IDLE;
//       lastPPOn = false;  // auto-exit macro
//       break;

//     default: break;
//   }
// }

void run_pick_place(int gripperRaw) {
  // Smooth targets for pick-place
  static float ppTargetShoulder = 90.f;
  static float ppTargetElbow    = 90.f;
  static float ppTargetBase     = BASE_ROT_HOME;

  // Smooth current angles toward targets every frame
  shoulderAngle += (ppTargetShoulder - shoulderAngle) * ARM_SMOOTH;
  elbowAngle    += (ppTargetElbow    - elbowAngle)    * ARM_SMOOTH;
  baseRotAngle  += (ppTargetBase     - baseRotAngle)  * ARM_SMOOTH;

  shoulderL.write((int)shoulderAngle);
  shoulderR.write(180 - (int)shoulderAngle);
  elbowServo.write((int)elbowAngle);
  writeMG90S(baseRot, (int)baseRotAngle);

  // Check if close enough to target to advance to next step
  bool atTarget = fabs(shoulderAngle - ppTargetShoulder) < 3.0f &&
                  fabs(elbowAngle    - ppTargetElbow)    < 3.0f &&
                  fabs(baseRotAngle  - ppTargetBase)     < 3.0f;

  // Wait minimum time AND must reach target before advancing
  unsigned long elapsed = millis() - ppStepStart;
  if (elapsed < (unsigned long)PP_STEP_MS || !atTarget) return;
  ppStepStart = millis();

  switch (ppState) {
    case PP_LOWER:
      ppTargetShoulder = PP_LOWER_SHOULDER;
      ppTargetElbow    = PP_LOWER_ELBOW;
      ppTargetBase     = baseRotAngle;  // hold base
      Serial.println(F("[PP] Lowering"));
      ppState = PP_GRIP;
      break;

    case PP_GRIP:
      writeMG90S(gripperServo, GRIPPER_MAX);
      Serial.println(F("[PP] Gripping"));
      ppState = PP_RAISE;
      break;

    case PP_RAISE:
      ppTargetShoulder = PP_RAISE_SHOULDER;
      ppTargetElbow    = PP_RAISE_ELBOW;
      Serial.println(F("[PP] Raising"));
      ppState = PP_SWING;
      break;

    case PP_SWING:
      ppTargetBase = PP_SWING_BASE;
      Serial.println(F("[PP] Swinging"));
      ppState = PP_DROP;
      break;

    case PP_DROP:
      writeMG90S(gripperServo, GRIPPER_MIN);
      Serial.println(F("[PP] Dropping"));
      ppState = PP_RETURN;
      break;

    case PP_RETURN:
      ppTargetShoulder = 90.f;
      ppTargetElbow    = 90.f;
      ppTargetBase     = BASE_ROT_HOME;
      Serial.println(F("[PP] Returning home"));
      ppState = PP_IDLE;
      lastPPOn = false;
      break;

    default: break;
  }
}

// ============================================================
//  ARM — WRITE MG90S IN 180° MODE
// ============================================================
void writeMG90S(Servo &sv,int angle){
  angle=constrain(angle,0,180);
  sv.writeMicroseconds(map(angle,0,180,MG90S_MIN_US,MG90S_MAX_US));
}

// ============================================================
//  ARM — APPLY
// ============================================================
void arm_apply(int wristTiltRaw, int gripperRaw){
  shoulderL.write((int)shoulderAngle);
  shoulderR.write(180-(int)shoulderAngle);
  elbowServo.write((int)elbowAngle);
  // FIX: use writeMG90S for baseRot — consistent with all other MG-series servos
  writeMG90S(baseRot, (int)baseRotAngle);
  writeMG90S(wristTilt, map(wristTiltRaw, 0,1023, WRIST_TILT_MIN,WRIST_TILT_MAX));
  writeMG90S(wristRot,  (int)wristRotAngle);
  writeMG90S(gripperServo, map(gripperRaw, 0,1023, GRIPPER_MIN,GRIPPER_MAX));
}

// ============================================================
//  ARM — HOME
// ============================================================
void arm_home_position(){
  shoulderAngle=90.f; elbowAngle=135.f;
  wristRotAngle=(float)WRIST_ROT_HOME;
  baseRotAngle =(float)BASE_ROT_HOME;
  shoulderL.write(90); shoulderR.write(90);
  elbowServo.write(90);
  // FIX: use writeMG90S for baseRot — consistent with all other MG-series servos
  writeMG90S(baseRot, BASE_ROT_HOME);
  writeMG90S(wristTilt,    90);
  writeMG90S(wristRot,     WRIST_ROT_HOME);
  writeMG90S(gripperServo, GRIPPER_MIN);
}

// ============================================================
//  GO HOME
// ============================================================
void go_home(){
  smoothRampFrames=0; 
  // t=0.f;
  Vector angles;
  for(int i=0;i<6;i++){
    legs[i].target.x+=(legs[i].home.x-legs[i].target.x)*STOP_SMOOTH_XY;
    legs[i].target.y+=(legs[i].home.y-legs[i].target.y)*STOP_SMOOTH_XY;
    legs[i].target.z+=(legs[i].home.z-legs[i].target.z)*STOP_SMOOTH_Z;
    calculate_ik(legs[i],angles);
    float ax=constrain(90.f+angles.x+TRIM[i][0],COXA_MIN, COXA_MAX);
    float ay=constrain(90.f+angles.y+TRIM[i][1],FEMUR_MIN,FEMUR_MAX);
    float az=constrain(fabs(angles.z)+TRIM[i][2],TIBIA_MIN,TIBIA_MAX);
    if(INVERT[i][0])ax=180.f-ax;
    if(INVERT[i][1])ay=180.f-ay;
    if(INVERT[i][2])az=180.f-az;
    legs[i].servos[0].write((int)ax);
    legs[i].servos[1].write((int)ay);
    legs[i].servos[2].write((int)az);
  }
}

// ============================================================
//  FK
// ============================================================
void calculate_fk(const Vector& angles,const Vector& pos,Vector res[4]){
  const float alpha=angles.x+pos.z;
  const float ca=cos(alpha),sa=sin(alpha);
  const float cb=cos(angles.y),sb=sin(angles.y);
  const float cbg=cos(angles.y+angles.z),sbg=sin(angles.y+angles.z);
  res[0]=pos; res[0].z=0.f;
  res[1].x=res[0].x+COXA_LENGTH*ca;
  res[1].y=res[0].y+COXA_LENGTH*sa;
  res[1].z=res[0].z;
  res[2].x=res[1].x+FEMUR_LENGTH*cb*ca;
  res[2].y=res[1].y+FEMUR_LENGTH*cb*sa;
  res[2].z=res[1].z+FEMUR_LENGTH*sb;
  res[3].x=res[1].x+(FEMUR_LENGTH*cb+TIBIA_LENGTH*cbg)*ca;
  res[3].y=res[1].y+(FEMUR_LENGTH*cb+TIBIA_LENGTH*cbg)*sa;
  res[3].z=res[1].z+(FEMUR_LENGTH*sb+TIBIA_LENGTH*sbg);
}

// ============================================================
//  IK
// ============================================================
void calculate_ik(const Leg& leg,Vector& res){
  Vector rel(leg.target.x-leg.origin.x,
             leg.target.y-leg.origin.y,leg.target.z);
  const float c=cos(-leg.origin.z),s=sin(-leg.origin.z);
  float rx=rel.x*c-rel.y*s, ry=rel.x*s+rel.y*c;
  rel.x=rx; rel.y=ry;
  const float l_xy=sqrt(POW2(rel.x)+POW2(rel.y));
  const float l_fwd=l_xy-COXA_LENGTH+0.01f;
  float D=sqrt(POW2(l_fwd)+POW2(rel.z));
  D=constrain(D,fabs(FEMUR_LENGTH-TIBIA_LENGTH)+0.5f,
               FEMUR_LENGTH+TIBIA_LENGTH-0.5f);
  res.z=RAD_TO_DEG(acos(constrain(
    (POW2(FEMUR_LENGTH)+POW2(TIBIA_LENGTH)-POW2(D))
    /(2.f*FEMUR_LENGTH*TIBIA_LENGTH),-1.f,1.f))-M_PI);
  const float b1=atan2(rel.z,l_fwd);
  const float b2=acos(constrain(
    (POW2(FEMUR_LENGTH)+POW2(D)-POW2(TIBIA_LENGTH))
    /(2.f*FEMUR_LENGTH*D),-1.f,1.f));
  res.y=RAD_TO_DEG(b1+b2);
  res.x=RAD_TO_DEG(atan2(rel.y,rel.x));
}

// ============================================================
//  FOOT TARGET
// ============================================================
Vector get_foot_target(float t, int idx, float fwd, float strafe,
                       float rot, float stepHeight) {
  float phase = fmod(t, 1.f);
  if (GROUP_B[idx]) phase = fmod(phase + 0.5f, 1.f);
  const Vector& home = legs[idx].home;
  if (fwd == 0.f && strafe == 0.f && rot == 0.f) return home;

  float sx, sy, sz, rr;

  if (phase < 0.5f) {
    float pn = phase * 2.f;                        // 0 → 1
    sx =  fwd    * (0.5f - pn) * stepLengthScale;
    sy =  strafe * (0.5f - pn) * stepLengthScale;
    rr =  rot    * (0.5f - pn) * ROTATION_STEP_RAD;
    sz = 0.f;
  // if (phase < 0.5f) {
  //   float pn = phase * 2.f;
  //   float eased = (1.f - cosf(pn * M_PI)) * 0.5f;  // ← ADD THIS
  //   sx =  fwd    * (0.5f - eased) * stepLengthScale; // pn → eased
  //   sy =  strafe * (0.5f - eased) * stepLengthScale; // pn → eased
  //   rr =  rot    * (0.5f - eased) * ROTATION_STEP_RAD; // pn → eased
  //   sz = 0.f;

  } else {
    // ── SWING: foot lifts, repositions BACKWARD→FORWARD ───
    // pn goes 0→1, sw goes -0.5→+0.5
    float pn = (phase - 0.5f) * 2.f;              // 0 → 1
    float sw  = pn - 0.5f;                         // -0.5 → +0.5
    // sx =  fwd    * sw * STEP_LENGTH_BASE;
    // sy =  strafe * sw * STEP_LENGTH_BASE;
    sx =  fwd    * sw * stepLengthScale;
    sy =  strafe * sw * stepLengthScale;
    rr =  rot    * sw * ROTATION_STEP_RAD;
    sz =  sin(pn * M_PI) * stepHeight;             // arc over ground
  }

  float cr = cos(rr), sr = sin(rr);
  return Vector(
    home.x * cr - home.y * sr + sx,
    home.x * sr + home.y * cr + sy,
    home.z + sz
  );
}
// Vector get_foot_target(float t,int idx,float fwd,float strafe,
//                        float rot,float stepHeight){
//   float phase=fmod(t,1.f);
//   if(GROUP_B[idx]) phase=fmod(phase+0.5f,1.f);
//   const Vector& home=legs[idx].home;
//   if(fwd==0.f&&strafe==0.f&&rot==0.f) return home;
//   float sx,sy,sz,rr;
//   if(phase<0.5f){
//     float pn=phase*2.f;
//     sx=fwd*(0.5f-pn)*STEP_LENGTH_BASE;
//     sy=strafe*(0.5f-pn)*STEP_LENGTH_BASE;
//     rr=rot*(0.5f-pn)*ROTATION_STEP_RAD;
//     sz=0.f;
//   } else {
//     float pn=(phase-0.5f)*2.f,sw=pn-0.5f;
//     sx=fwd*sw*STEP_LENGTH_BASE;
//     sy=strafe*sw*STEP_LENGTH_BASE;
//     rr=rot*sw*ROTATION_STEP_RAD;
//     sz=sin(pn*M_PI)*stepHeight;
//   }
//   float cr=cos(rr),sr=sin(rr);
//   return Vector(home.x*cr-home.y*sr+sx,
//                 home.x*sr+home.y*cr+sy,
//                 home.z+sz);
// }

// ============================================================
//  SET FOOT POSITIONS
// ============================================================
void set_foot_positions(float fwd,float strafe,float rot,
                        float speedScale,float stepHeight){
  if(fwd==0.f&&strafe==0.f&&rot==0.f){ go_home(); return; }
  t+=T_INC_MIN+speedScale*(T_INC_MAX-T_INC_MIN);
  if(smoothRampFrames<20) smoothRampFrames++;
  Vector angles;
  for(int i=0;i<6;i++){
    const Vector goal=get_foot_target(t,i,fwd,strafe,rot,stepHeight);
    float ramp=WALK_SMOOTH_XY*(smoothRampFrames/20.f);
    legs[i].target.x+=(goal.x-legs[i].target.x)*ramp;
    legs[i].target.y+=(goal.y-legs[i].target.y)*ramp;
    legs[i].target.z+=(goal.z-legs[i].target.z)*WALK_SMOOTH_Z;
    calculate_ik(legs[i],angles);
    float ax=constrain(90.f+angles.x+TRIM[i][0],COXA_MIN, COXA_MAX);
    float ay=constrain(90.f+angles.y+TRIM[i][1],FEMUR_MIN,FEMUR_MAX);
    float az=constrain(fabs(angles.z)+TRIM[i][2],TIBIA_MIN,TIBIA_MAX);
    if(INVERT[i][0])ax=180.f-ax;
    if(INVERT[i][1])ay=180.f-ay;
    if(INVERT[i][2])az=180.f-az;
    legs[i].servos[0].write((int)ax);
    legs[i].servos[1].write((int)ay);
    legs[i].servos[2].write((int)az);
  }
}

// ============================================================
//  RADIO READ — int[12] matching Uno v3.0 exactly
// ============================================================
void read_radio(){
  if(radio.available()){
    radio.read(radioData, sizeof(radioData));
    lastPacketTime=millis();
  }
}

// ============================================================
//  SERIAL OVERRIDE (Web Visualizer)
// ============================================================
void read_serial(){
  static String buf="";
  while(Serial.available()>0){
    char c=Serial.read();
    if(c=='\n'){
      int fc=buf.indexOf(','),sc=buf.indexOf(',',fc+1);
      if(fc>0&&sc>fc){
        serialFwd    =constrain(buf.substring(0,fc).toFloat(),-1.f,1.f);
        serialStrafe =constrain(buf.substring(fc+1,sc).toFloat(),-1.f,1.f);
        serialRot    =constrain(buf.substring(sc+1).toFloat(),-1.f,1.f);
        lastSerialTime=millis();
      }
      buf="";
    } else buf+=c;
  }
}

// ============================================================
//  JOYSTICK MAP — 0-1023 → -1..+1 with deadband
// ============================================================
float map_joy(int raw){
  int centred=raw-512;
  if(abs(centred)<JOY_DEADBAND) return 0.f;
  float norm=(centred>0)
    ?(float)(centred-JOY_DEADBAND)/(511.f-JOY_DEADBAND)
    :(float)(centred+JOY_DEADBAND)/(511.f-JOY_DEADBAND);
  return constrain(norm,-1.f,1.f);
}

// ============================================================
//  TELEMETRY
// ============================================================
void sendTelemetry(){
  float voltage=(analogRead(VOLTAGE_PIN)/1023.f)*V_REF*VOLTAGE_DIVIDER_RATIO;
  float vC=(analogRead(CURRENT_PIN)/1023.f)*V_REF;
  float current=max((vC-V_REF/2.f)/CURRENT_SENSOR_SENSITIVITY,0.f);
  float tempC=0.f;
  int tR=analogRead(TEMP_PIN);
  if(tR>0&&tR<1023){
    float r=10000.f/((1023.f/tR)-1.f);
    tempC=1.f/(log(r/10000.f)/TEMP_SENSOR_BETA+1.f/(25.f+273.15f))-273.15f;
  }
  Serial.print(F("TELEMETRY:"));
  Serial.print(voltage,2); Serial.print(',');
  Serial.print(current,2); Serial.print(',');
  Serial.println(tempC,2);
}
