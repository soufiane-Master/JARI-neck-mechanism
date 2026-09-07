#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include "MPU6050.h" 
#include <Adafruit_NeoPixel.h> 
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include "esp_wpa2.h"

Preferences prefs;

// ================= WIFI CONFIGURATION =================
// TODO: replace with your real WiFi name and password (case-sensitive)
const char* WIFI_SSID     = "TU_RED_WIFI_AQUI";
const char* WIFI_PASSWORD = "TU_CONTRASENA_AQUI";

WebServer server(80);

// ================= HARDWARE DEFINITIONS =================
#define PIN_TOUCH_L    32   
#define PIN_TOUCH_R    33   
#define PIN_LED        17   
#define NUM_LEDS       19   
#define LED_BRIGHTNESS 100

Adafruit_NeoPixel strip(NUM_LEDS, PIN_LED, NEO_GRB + NEO_KHZ800);

// ================= SERVO DEFINITIONS =================
#define PIN_SERVO_Z    18   
#define PIN_SERVO_L    27   
#define PIN_SERVO_R    23   
#define PIN_SERVO_B    19   
#define PIN_YAW_FEEDBACK 13   // repurposed from unused PIN_ARM_L — confirmed ADC-capable GPIO

// ================= MOTION PARAMETERS =================
#define STOP_VAL_L     1527
#define STOP_VAL_R     1549 
#define STOP_VAL_B     1500 
#define SPD            140   // raised further so smaller geometric fractions still clear the dead zone

// Physics-based pull/release timing compensation for the conical spring asymmetry.
// PULL fights the spring (slower) — RELEASE gets help from it (faster).
// Start here, then fine-tune empirically: if the cord still winds in after
// several 'A' cycles, shorten NECK_RLSE_MS a bit more; if it winds OUT, lengthen it.
#define NECK_PULL_MS   170
#define NECK_RLSE_MS   135

// Yaw (sZ) swing timing — same drift problem as L/R/B, no spring involved here,
// just normal motor variance between the two directions. Tune YAW_L_MS vs
// YAW_R_MS with the 'YT' repeat-test command until net drift is zero.
#define YAW_L_MS       250
#define YAW_R_MS       250

// How far the yaw swings from center (90) during the YT test — closer to 90 = slower motion.
#define YAW_ANGLE_LO   40
#define YAW_ANGLE_HI   140

#define PULL_L         (STOP_VAL_L + SPD)
#define RLSE_L         (STOP_VAL_L - SPD)
#define PULL_R         (STOP_VAL_R + SPD)
#define RLSE_R         (STOP_VAL_R - SPD)
#define PULL_B         (STOP_VAL_B - SPD)  // swapped: B's motor spins opposite to L/R for the same offset sign
#define RLSE_B         (STOP_VAL_B + SPD)  // swapped

int n_ear = 90;  int n_arm = 10; 
int h_ear = 0;   int h_arm = 175; 
int a_ear = 180; int a_arm = 0;
int s_ear = 45;  int s_arm = 160; 
int q_ear = 10;  int q_arm = 90;  

Servo sL, sR, sB, sZ;
// earL, earR, armL, armR removed — ear/arm hardware not installed on this robot
MPU6050 mpu;

// ================= MPU6050 baseline =================
bool mpuReady          = false;
bool mpuBaselineReady  = false;
float baseAx = 0.0f, baseAy = 0.0f, baseAz = 1.0f;

void calibrateMpuBaseline() {
    if (!mpuReady) { mpuBaselineReady = false; return; }
    delay(250);
    long axSum = 0, aySum = 0, azSum = 0;
    const int samples = 80;
    for (int i = 0; i < samples; i++) {
        int16_t ax, ay, az, gx, gy, gz;
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        axSum += ax; aySum += ay; azSum += az;
        delay(5);
    }
    float fax = (axSum / (float)samples) / 16384.0f;
    float fay = (aySum / (float)samples) / 16384.0f;
    float faz = (azSum / (float)samples) / 16384.0f;
    float norm = sqrtf(fax*fax + fay*fay + faz*faz);
    if (norm < 0.2f) { mpuBaselineReady = false; return; }
    baseAx = fax/norm; baseAy = fay/norm; baseAz = faz/norm;
    mpuBaselineReady = true;
}

// ================= CABLE GEOMETRY (3 motors around the platform) =================
// Angular position of each cable/motor, same convention as the mechanical
// report (0° / 120° / 240°), measured looking down at the platform from above,
// 0° = "forward" (toward PIN_SERVO_B in this build), increasing counter-clockwise.
//
// >>> THESE 3 NUMBERS ARE PROBABLY WRONG FOR YOUR ROBOT — measure them with
// >>> the calibration test in testCableAngle() below, then edit them here. <<<
const float ANGLE_L_DEG = 242.4f;
const float ANGLE_R_DEG = 116.9f;
const float ANGLE_B_DEG = 45.8f;

// ================= PID STATE (tilt leveling) =================
float pidIntegral    = 0.0f;   // accumulated error, for the I term
float pidPrevTiltDeg = 0.0f;   // last tilt reading, only used as a fallback

// ================= IMU-BASED HEAD LEVELING (real PID, 3-cable mixing) =================
// Drives sL / sR / sB together, as ONE coordinated system, using MPU6050
// accel (angle) + gyro (rate) feedback until the head is back to horizontal.
//
// Why this fixes the cable-coupling problem (Problema 2 in the report):
//   The old code treated L/R as a left-right pair and B as forward-back —
//   like a joystick with 2 axes. But the 3 cables are NOT at 2 orthogonal
//   axes, they're at 0°/120°/240°. Here, the tilt is computed as ONE 2D
//   error vector (tiltX, tiltY), and EACH motor's command is the projection
//   of that same vector onto its own cable direction:
//
//       correction_i = K * (tiltX * sin(angle_i) + tiltY * cos(angle_i))
//
//   This means: when the platform tilts toward motor 0's cable, that
//   projection is positive for motor 0 (pull) and automatically NEGATIVE
//   for the motors 120°/240° away (they get commanded to release / feed
//   cable out) — proportionally, from real geometry, not guessed timing.
//   No cable is ever left slack or fighting another cable.
//
// Tuning:
//   Kp   – µs per degree of tilt error (main strength of correction)
//   Ki   – µs per (degree × second) accumulated — kills small residual
//          error that Kp alone can't remove (e.g. leftover slack, friction)
//   Kd   – µs per (degree/second) of gyro rate — damps oscillation/overshoot
//   maxCorr   – clamp on servo deviation (µs). Keep ≤ 120 to avoid jerks.
//   tolDeg    – dead-band. 3° is a good practical threshold.
//   timeoutMs – safety timeout so leveling never blocks forever.

void levelHeadWithIMU(unsigned long timeoutMs = 2600, float tolDeg = 3.0f) {
    if (!mpuReady || !mpuBaselineReady) return;

    if (!sL.attached()) sL.attach(PIN_SERVO_L);
    if (!sR.attached()) sR.attach(PIN_SERVO_R);
    if (!sB.attached()) sB.attach(PIN_SERVO_B);

    const float Kp = 10.0f;    // confirmed stable
    const float Ki = 3.0f;     // re-enabled at a moderate value to remove the ~3.3-3.5° residual
    const float Kd = 10.0f;    // confirmed stable damping
    const float maxCorr = 100.0f;
    const float integralClamp = 15.0f; // anti-windup: caps how much error I can accumulate

    pidIntegral = 0.0f; // fresh start every time we begin a leveling pass
    unsigned long start = millis();
    unsigned long lastLoopMs = millis();

    while (millis() - start < timeoutMs) {
        unsigned long nowMs = millis();
        float dt = (nowMs - lastLoopMs) / 1000.0f;
        if (dt <= 0.0f) dt = 0.001f;
        lastLoopMs = nowMs;

        int16_t ax16, ay16, az16, gx16, gy16, gz16;
        mpu.getMotion6(&ax16, &ay16, &az16, &gx16, &gy16, &gz16);

        float fax = ax16 / 16384.0f;
        float fay = ay16 / 16384.0f;
        float faz = az16 / 16384.0f;
        float norm = sqrtf(fax*fax + fay*fay + faz*faz);
        if (norm < 0.2f) break;

        float nx = fax / norm;
        float ny = fay / norm;
        float nz = faz / norm;

        float dot = nx*baseAx + ny*baseAy + nz*baseAz;
        dot = constrain(dot, -1.0f, 1.0f);
        float tiltDeg = acosf(dot) * 57.2958f;

        if (tiltDeg < tolDeg) break; // within tolerance — done

        // 2D tilt error vector (same cross-product idea as before)
        float tiltX = ny*baseAz - nz*baseAy;  // left/right component
        float tiltY = nz*baseAx - nx*baseAz;  // forward/back component

        // Gyro rate, deg/s (default MPU6050 full-scale range = ±250°/s → 131 LSB/°/s)
        // Used for the D term instead of differentiating the noisy accel signal —
        // much cleaner and is exactly what a real angular-rate D term should use.
        float gyroRateDegS = sqrtf((gx16/131.0f)*(gx16/131.0f) + (gy16/131.0f)*(gy16/131.0f));

        // ---- PID on tilt magnitude ----
        pidIntegral += tiltDeg * dt;
        pidIntegral = constrain(pidIntegral, -integralClamp, integralClamp);

        float pTerm = Kp * tiltDeg;
        float iTerm = Ki * pidIntegral;
        float dTerm = -Kd * gyroRateDegS; // opposes current rotation rate → damping
        float magCorr = constrain(pTerm + iTerm + dTerm, -maxCorr, maxCorr);

        // Normalize the direction so magCorr carries the PID "strength" and
        // (tiltX,tiltY) only carries the direction to push in.
        float dirNorm = sqrtf(tiltX*tiltX + tiltY*tiltY);
        float dirX = (dirNorm > 0.0001f) ? tiltX/dirNorm : 0.0f;
        float dirY = (dirNorm > 0.0001f) ? tiltY/dirNorm : 0.0f;

        // ---- Geometric mixing: project the correction onto each cable ----
        float corrL = magCorr * (dirX*sinf(radians(ANGLE_L_DEG)) + dirY*cosf(radians(ANGLE_L_DEG)));
        float corrR = magCorr * (dirX*sinf(radians(ANGLE_R_DEG)) + dirY*cosf(radians(ANGLE_R_DEG)));
        float corrB = magCorr * (dirX*sinf(radians(ANGLE_B_DEG)) + dirY*cosf(radians(ANGLE_B_DEG)));

        int usL = (int)(STOP_VAL_L + corrL);
        int usR = (int)(STOP_VAL_R + corrR);
        int usB = (int)(STOP_VAL_B + corrB);

        usL = constrain(usL, STOP_VAL_L - (int)maxCorr, STOP_VAL_L + (int)maxCorr);
        usR = constrain(usR, STOP_VAL_R - (int)maxCorr, STOP_VAL_R + (int)maxCorr);
        usB = constrain(usB, STOP_VAL_B - (int)maxCorr, STOP_VAL_B + (int)maxCorr);

        sL.writeMicroseconds(usL);
        sR.writeMicroseconds(usR);
        sB.writeMicroseconds(usB);

        pidPrevTiltDeg = tiltDeg;

        Serial.print("LEVEL: tilt="); Serial.print(tiltDeg, 1);
        Serial.print(" L="); Serial.print(usL);
        Serial.print(" R="); Serial.print(usR);
        Serial.print(" B="); Serial.println(usB);

        delay(18);
    }

    // Halt all neck servos cleanly
    sL.writeMicroseconds(STOP_VAL_L);
    sR.writeMicroseconds(STOP_VAL_R);
    sB.writeMicroseconds(STOP_VAL_B);
    delay(60);
    sL.writeMicroseconds(STOP_VAL_L); delay(50); sL.detach();
    sR.writeMicroseconds(STOP_VAL_R); delay(50); sR.detach();
    sB.writeMicroseconds(STOP_VAL_B); delay(50); sB.detach();
}

// ================= CALIBRATION TEST — find the real angle of each motor =================
// Run this ONCE from setup() (or trigger it with a serial command) with the
// head hanging free. It pulses ONE motor at a time and prints the resulting
// tiltX/tiltY direction. Whatever direction each motor produces IS its real
// angle — read the printed "measured angle" and copy it into ANGLE_L_DEG /
// ANGLE_R_DEG / ANGLE_B_DEG above. No math needed, just copy the numbers.
void testCableAngle(Servo &s, int pin, int pullVal, const char* label) {
    if (!mpuReady || !mpuBaselineReady) { Serial.println("MPU not ready, skip calibration"); return; }
    if (!s.attached()) s.attach(pin);

    s.writeMicroseconds(pullVal);
    delay(500); // let the platform fully settle into the tilt (longer than before)

    // Average 25 readings over ~500ms instead of one noisy snapshot
    float sumTiltX = 0, sumTiltY = 0;
    const int samples = 25;
    for (int i = 0; i < samples; i++) {
        int16_t ax16, ay16, az16, gx16, gy16, gz16;
        mpu.getMotion6(&ax16, &ay16, &az16, &gx16, &gy16, &gz16);
        float fax = ax16/16384.0f, fay = ay16/16384.0f, faz = az16/16384.0f;
        float norm = sqrtf(fax*fax+fay*fay+faz*faz);
        if (norm > 0.2f) {
            float nx = fax/norm, ny = fay/norm, nz = faz/norm;
            sumTiltX += ny*baseAz - nz*baseAy;
            sumTiltY += nz*baseAx - nx*baseAz;
        }
        delay(20);
    }
    float tiltX = sumTiltX / samples;
    float tiltY = sumTiltY / samples;
    float measuredAngle = atan2f(tiltX, tiltY) * 57.2958f;
    if (measuredAngle < 0) measuredAngle += 360.0f;

    Serial.print(label); Serial.print(" pulled -> measured angle = ");
    Serial.print(measuredAngle, 1); Serial.println(" deg  <-- copy this number");

    s.writeMicroseconds((pin==PIN_SERVO_L)?STOP_VAL_L:(pin==PIN_SERVO_R)?STOP_VAL_R:STOP_VAL_B);
    delay(400);
    s.detach();
}

void runCableAngleCalibration() {
    Serial.println(">>> CABLE ANGLE CALIBRATION START <<<");
    testCableAngle(sL, PIN_SERVO_L, PULL_L, "L");
    testCableAngle(sR, PIN_SERVO_R, PULL_R, "R");
    testCableAngle(sB, PIN_SERVO_B, PULL_B, "B");
    Serial.println(">>> DONE. Update ANGLE_L_DEG / ANGLE_R_DEG / ANGLE_B_DEG with the printed numbers. <<<");
}

// ================= COMBINED RETURN-TO-NEUTRAL =================
// Replaces the old  stopNeck() + setBodySmooth(...neutral)  pattern.
// Order:
//   1. Level the head using IMU feedback
//   2. Smooth body (ears + arms) back to neutral
//   3. Restore LED to last emotion color (caller keeps lastEmotionColor correct)
void returnToNeutral(int ear_from, int arm_from);  // forward declaration

// ================= LED GLOBAL STATE =================
bool     wakeBlinkActive   = false;
uint32_t wakeBlinkLastMs   = 0;
int      wakeBlinkPhase    = 0;
const uint16_t WAKE_BLINK_STEP_MS = 18;
uint32_t lastEmotionColor  = 0;

// ================= LED =================
void setLedColor(uint32_t color) {
    for(int i=0; i<strip.numPixels(); i++) strip.setPixelColor(i, color);
    strip.show();
}

uint32_t colorForLetter(char cmd) {
    if (cmd == 'a') cmd = 'A';
    else if (cmd == 'v') cmd = 'V';
    else if (cmd == 'd') cmd = 'D';
    else if (cmd == 'q') cmd = 'Q';
    else if (cmd == 'u') cmd = 'U';
    switch(cmd) {
        case 'A': return strip.Color(0, 255, 0);
        case 'V': return strip.Color(50, 255, 80);
        case 'O': return strip.Color(255, 0, 0);
        case 'X': return strip.Color(255, 80, 0);
        case 'D': return strip.Color(40, 220, 40);
        case 'P': return strip.Color(128, 0, 255);
        case 'T': return strip.Color(0, 40, 150);
        case 'W': return strip.Color(0, 20, 100);
        case 'Q': return strip.Color(255, 255, 255);
        case 'U': return strip.Color(200, 180, 120);
        case 'G': return strip.Color(130, 180, 0);
        case 'H': return strip.Color(255, 140, 0);
        case 'K': return strip.Color(0, 220, 180);
        case 'M': return strip.Color(255, 220, 0);
        case 'E': return strip.Color(0, 180, 255);
        case 'R': return strip.Color(255, 0, 180);
        case 'F': return strip.Color(0, 150, 200);
        case '1': return strip.Color(255, 200, 0);
        case '3': return strip.Color(100, 220, 80);
        case '4': return strip.Color(180, 120, 255);
        default:  return 0;
    }
}

void showEmotionLED(char emotion) {
    uint32_t c = colorForLetter(emotion);
    setLedColor(c);
    lastEmotionColor = c;
}

void updateWakeBlink() {
    if (!wakeBlinkActive) return;
    uint32_t now = millis();
    if (now - wakeBlinkLastMs < WAKE_BLINK_STEP_MS) return;
    wakeBlinkLastMs = now;
    wakeBlinkPhase = (wakeBlinkPhase + 1) % 200;
    int p = wakeBlinkPhase;
    int level = (p < 100) ? p * 2 : (200 - p) * 2;
    uint8_t b = (uint8_t)constrain(level, 0, 255);
    setLedColor(strip.Color(0, 0, b));
}

void startWakeBlink() {
    wakeBlinkActive = true;
    wakeBlinkPhase = 0;
    wakeBlinkLastMs = 0;
}

void stopWakeBlink() {
    wakeBlinkActive = false;
    setLedColor(lastEmotionColor);
}

// ================= HELPER FUNCTIONS =================
// ================= GEOMETRIC CABLE MIXING (open-loop, no MPU needed) =================
// Pulling one cable geometrically MUST release the other two proportionally,
// or the released side goes slack / the tensioned side fights itself — this is
// Problema 2 from the mechanical report. This function encodes that fixed
// geometry (cable angles ANGLE_L_DEG / ANGLE_R_DEG / ANGLE_B_DEG) so any
// gesture can call it instead of hand-writing pull/release combinations.
//
// theta_deg: the direction you want the head to lean toward (0/120/240 = toward
//            that cable's own position — e.g. call with ANGLE_L_DEG to "pull toward L")
// strength:  how hard to drive, in the same µs-offset units as PULL_L/RLSE_L (e.g. SPD)
//            positive strength pulls toward theta_deg; the cable(s) on the
//            opposite side automatically come out negative (= release) from the math.
void driveCablesGeometric(float theta_deg, float strength) {
    float dirX = sinf(radians(theta_deg));
    float dirY = cosf(radians(theta_deg));
    float projL = dirX * sinf(radians(ANGLE_L_DEG)) + dirY * cosf(radians(ANGLE_L_DEG));
    float projR = dirX * sinf(radians(ANGLE_R_DEG)) + dirY * cosf(radians(ANGLE_R_DEG));
    float projB = dirX * sinf(radians(ANGLE_B_DEG)) + dirY * cosf(radians(ANGLE_B_DEG));

    // Only motors clearly aligned with the movement direction actually move.
    // Anything weak gets fully stopped instead of a small/confusing partial move.
    const float THRESHOLD = 0.5f;
    int corrL = (fabsf(projL) > THRESHOLD) ? (int)(strength * projL) : 0;
    int corrR = (fabsf(projR) > THRESHOLD) ? (int)(strength * projR) : 0;
    int corrB = (fabsf(projB) > THRESHOLD) ? (int)(strength * projB) : 0;

    sL.writeMicroseconds(STOP_VAL_L + corrL);
    sR.writeMicroseconds(STOP_VAL_R + corrR);
    sB.writeMicroseconds(STOP_VAL_B + corrB);
}

// Reusable yaw swing using the drift-tuned timing/angle values —
// use this everywhere instead of hardcoding sZ.write(65)/sZ.write(115).
void yawSwing() {
    if (!sZ.attached()) { sZ.setPeriodHertz(50); sZ.attach(PIN_SERVO_Z, 900, 2100); }
    sZ.write(YAW_ANGLE_HI); delay(YAW_R_MS);
    sZ.write(YAW_ANGLE_LO); delay(YAW_L_MS);
    sZ.write(90);
}

// ================= YAW FEEDBACK (FT1117M-FB) =================
// Reads the real position of sZ from its feedback wire (GPIO4) instead of
// guessing. Raw ADC is 0-4095. These min/max values are PLACEHOLDERS —
// you must calibrate them using the 'YF' command (see below) once the
// servo is physically wired, then update these two numbers.
int YAW_FB_RAW_MIN = 579;   // measured at Z0
int YAW_FB_RAW_MAX = 3131;  // measured at Z180

float readYawFeedbackAngle() {
    int raw = analogRead(PIN_YAW_FEEDBACK);
    float angle = map(raw, YAW_FB_RAW_MIN, YAW_FB_RAW_MAX, 180, 0); // inverted: raw decreases as commanded angle increases
    return constrain(angle, -10, 190); // small slack past the ends for noise
}

// ================= WEB CONTROL PAGE =================
const char WEB_PAGE[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>JARI - Control</title>
  <style>
    body { font-family: sans-serif; background:#111; color:#eee; text-align:center; padding:20px; }
    h1 { font-size: 22px; }
    .grid { display:grid; grid-template-columns: repeat(3, 1fr); gap:10px; max-width:500px; margin:20px auto; }
    button {
      padding: 18px 8px; font-size: 15px; border:none; border-radius:10px;
      background:#2a7de1; color:white; cursor:pointer; line-height:1.3;
    }
    button:active { background:#1a5db1; }
    #status { margin-top:15px; color:#aaa; font-size:14px; }
  </style>
</head>
<body>
  <h1>JARI - Panel de Control / Control Panel</h1>
  <div class="grid" id="grid"></div>
  <div id="status">Listo / Ready</div>
  <script>
    const buttons = [
      ["A","Feliz<br>Happy"], ["O","Enfadado<br>Angry"], ["P","Asustado<br>Scared"], ["Q","Sorpresa<br>Surprise"],
      ["V","Si<br>Yes"], ["X","No<br>No"], ["Y","Timido<br>Shy"], ["S","Neutral<br>Neutral"],
      ["U","Serio<br>Serious"], ["H","Puno<br>Fist"], ["K","OK<br>OK"], ["M","Palma<br>Palm"],
      ["E","Paz<br>Peace"], ["R","Rock<br>Rock"], ["F","Surf<br>Surf"],
      ["1","Uno<br>One"], ["3","Tres<br>Three"], ["4","Cuatro<br>Four"]
    ];
    const grid = document.getElementById("grid");
    buttons.forEach(([cmd, label]) => {
      const b = document.createElement("button");
      b.innerHTML = label;
      b.onclick = () => send(cmd);
      grid.appendChild(b);
    });
    function send(cmd) {
      document.getElementById("status").textContent = "Enviando/Sending: " + cmd + "...";
      fetch("/cmd?c=" + encodeURIComponent(cmd))
        .then(r => r.text())
        .then(t => document.getElementById("status").textContent = "OK: " + cmd)
        .catch(e => document.getElementById("status").textContent = "Error: " + e);
    }
  </script>
</body>
</html>
)HTML";

void handleRoot() {
    server.send(200, "text/html", WEB_PAGE);
}

void handleCmd() {
    if (server.hasArg("c")) {
        String c = server.arg("c");
        char buf[8];
        c.toCharArray(buf, sizeof(buf));
        handleLine(buf);      // reuses the exact same logic as the Serial commands
        server.send(200, "text/plain", "OK");
    } else {
        server.send(400, "text/plain", "Missing 'c' argument");
    }
}

bool wifiEnterpriseWasEnabled = false; // tracks whether WPA2-Enterprise was ever turned on this session

void startAccessPoint() {
    WiFi.disconnect(true);
    delay(200);
    WiFi.mode(WIFI_AP);
    WiFi.softAP("JARI-Robot", "jari1234"); // network name and password the phone will connect to
    delay(500);
    Serial.println(">>> JARI Access Point started <<<");
    Serial.print(">>> Connect your phone's WiFi to network 'JARI-Robot' (password: jari1234), then open: http://");
    Serial.print(WiFi.softAPIP());
    Serial.println(" <<<");
}

void connectWifi() {
    String mode = prefs.getString("mode", "simple"); // "simple", "enterprise", or "ap"

    if (mode == "ap") {
        startAccessPoint();
        return;
    }

    String ssid = prefs.getString("ssid", WIFI_SSID);

    WiFi.disconnect(true);
    delay(200);

    if (mode == "enterprise") {
        String user = prefs.getString("euser", "");
        String pass = prefs.getString("epass", "");
        WiFi.mode(WIFI_STA);
        esp_wifi_sta_wpa2_ent_set_identity((uint8_t*)user.c_str(), user.length());
        esp_wifi_sta_wpa2_ent_set_username((uint8_t*)user.c_str(), user.length());
        esp_wifi_sta_wpa2_ent_set_password((uint8_t*)pass.c_str(), pass.length());
        esp_wifi_sta_wpa2_ent_enable();
        wifiEnterpriseWasEnabled = true;
        WiFi.begin(ssid.c_str());
        Serial.print("Connecting to WPA2-Enterprise WiFi: "); Serial.println(ssid);
    } else {
        if (wifiEnterpriseWasEnabled) {
            esp_wifi_sta_wpa2_ent_disable(); // only needed if Enterprise mode was actually turned on before
            wifiEnterpriseWasEnabled = false;
        }
        WiFi.mode(WIFI_STA);
        String pass = prefs.getString("pass", WIFI_PASSWORD);
        WiFi.begin(ssid.c_str(), pass.c_str());
        Serial.print("Connecting to WiFi: "); Serial.println(ssid);
    }

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 30) {
        delay(500);
        Serial.print(".");
        attempts++;
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println();
        Serial.print(">>> WiFi connected. Open this in your phone/laptop browser: http://");
        Serial.print(WiFi.localIP());
        Serial.println(" <<<");
    } else {
        Serial.println();
        Serial.print(">>> WiFi FAILED to connect. Status code: ");
        Serial.print(WiFi.status());
        Serial.println(" <<<");
        Serial.println("(0=IDLE 1=NO_SSID_FOUND 3=CONNECTED 4=CONNECT_FAILED 5=CONNECTION_LOST 6=DISCONNECTED)");
        Serial.println("Send: WIFI|network|password  or  WIFIENT|network|user@uni.edu|password");
    }
}

void setupWebServer() {
    prefs.begin("jari", false); // opens persistent storage
    connectWifi();
    server.on("/", handleRoot);
    server.on("/cmd", handleCmd);
    server.begin();
}

void attachNeck() {
    if (!sL.attached()) sL.attach(PIN_SERVO_L);
    if (!sR.attached()) sR.attach(PIN_SERVO_R);
    if (!sB.attached()) sB.attach(PIN_SERVO_B);
}

void stopNeck() {
    sL.writeMicroseconds(STOP_VAL_L);
    sR.writeMicroseconds(STOP_VAL_R);
    sB.writeMicroseconds(STOP_VAL_B);
    delay(50); 
    sL.detach(); sR.detach(); sB.detach();
    // sZ intentionally NOT touched here — it should stay attached and hold
    // whatever position each emotion left it at (usually centered via sZ.write(90)
    // inside the emotion itself). Only HZ/SZ test commands should move/detach it.
}

void setBody(int ear_val, int arm_val) {
    // No-op: ear/arm hardware not installed on this robot
}

void setBodySmooth(int ear_from, int ear_to, int arm_from, int arm_to, int steps = 8, int dly = 18) {
    delay(steps * dly); // preserves the original pacing/timing of each gesture without touching any hardware
}

void neckMicroShake(int leftRightDelta, int bodyDelta, int dly) {
    attachNeck();
    sZ.write(90 - leftRightDelta);
    sB.writeMicroseconds(STOP_VAL_B + bodyDelta);
    delay(dly);
    sZ.write(90 + leftRightDelta);
    sB.writeMicroseconds(STOP_VAL_B - bodyDelta);
    delay(dly);
    sZ.write(90);
    sB.writeMicroseconds(STOP_VAL_B);
}

// ================= RETURN TO NEUTRAL (IMU-leveled) =================
void returnToNeutral(int ear_from, int arm_from) {
    levelHeadWithIMU();                                               // 1. IMU-driven head leveling (skipped if MPU not ready)
    stopNeck();                                                          // 1b. ALWAYS stop L/R/B, even if leveling was skipped
    setBodySmooth(ear_from, n_ear, arm_from, n_arm, 10, 18);       // 2. smooth body
    // LED is left as-is; caller can clear it if needed
}

// ================= Setup =================
void setup() {
    Serial.begin(115200);
    randomSeed(analogRead(36) ^ analogRead(39));
    
    strip.begin();
    strip.setBrightness(LED_BRIGHTNESS);
    strip.show(); 

    pinMode(PIN_TOUCH_L, INPUT_PULLDOWN);
    pinMode(PIN_TOUCH_R, INPUT_PULLDOWN);

    ESP32PWM::allocateTimer(0); ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2); ESP32PWM::allocateTimer(3);

    sZ.setPeriodHertz(50); sZ.attach(PIN_SERVO_Z, 900, 2100); sZ.write(90);
    
    analogSetPinAttenuation(PIN_YAW_FEEDBACK, ADC_11db); 
    
    setBody(n_ear, n_arm);
    stopNeck(); 

    Wire.begin(21, 22);
    mpu.initialize();
    mpuReady = mpu.testConnection();
    calibrateMpuBaseline();

    for(int i=0; i<100; i+=5) {
        strip.setBrightness(i);
        setLedColor(strip.Color(0, 0, 255));
        delay(10);
    }
    setLedColor(0);

    if (mpuReady) Serial.println(mpuBaselineReady ? "MPU6050 OK + BASELINE" : "MPU6050 OK, BASELINE FAIL");
    else Serial.println("MPU6050 FAIL");

    Serial.println(">>> JARI V96 (PID Level + Autonomous Touch) READY <<<");

    setupWebServer();
}

void executeEmotion(char cmd);

// ================= SERIAL COMMAND PARSER =================
char rxBuf[128];
uint8_t rxLen = 0;

void handleLine(const char* line) {
    size_t n = strlen(line);
    if (n == 0) return;
    char c0 = line[0];

    // Switch to standalone mode: the ESP32 creates its own WiFi network. Send: AP
    if (n == 2 && c0 == 'A' && line[1] == 'P') {
        prefs.putString("mode", "ap");
        connectWifi();
        return;
    }

    // University-style WPA2-Enterprise network: WIFIENT|network_name|correo@uni.edu|password
    if (n > 8 && strncmp(line, "WIFIENT|", 8) == 0) {
        String rest = String(line + 8);
        int sep1 = rest.indexOf('|');
        int sep2 = rest.indexOf('|', sep1 + 1);
        if (sep1 < 0 || sep2 < 0) {
            Serial.println("Format: WIFIENT|network_name|correo@uni.edu|password");
            return;
        }
        String newSsid = rest.substring(0, sep1);
        String newUser = rest.substring(sep1 + 1, sep2);
        String newPass = rest.substring(sep2 + 1);
        prefs.putString("mode", "enterprise");
        prefs.putString("ssid", newSsid);
        prefs.putString("euser", newUser);
        prefs.putString("epass", newPass);
        Serial.print("Saved WPA2-Enterprise WiFi: "); Serial.println(newSsid);
        connectWifi();
        return;
    }

    // Change WiFi network without reflashing: WIFI|network_name|password
    if (n > 5 && strncmp(line, "WIFI|", 5) == 0) {
        String rest = String(line + 5);
        int sep = rest.indexOf('|');
        if (sep < 0) {
            Serial.println("Format: WIFI|network_name|password");
            return;
        }
        String newSsid = rest.substring(0, sep);
        String newPass = rest.substring(sep + 1);
        prefs.putString("mode", "simple");
        prefs.putString("ssid", newSsid);
        prefs.putString("pass", newPass);
        Serial.print("Saved new WiFi: "); Serial.println(newSsid);
        connectWifi();
        return;
    }

    // One-command automatic sweep for a single motor: SL / SR / SB
    // Slowly sweeps that ONE motor through a wide range, 2.5s per step, printing
    // the value each time. Just watch that one motor and tell me roughly where
    // it looked stopped / slowest, or where it switched direction.
    // Hold one motor at a fixed value indefinitely, for physical trim-pot adjustment.
    // HL / HR / HB = hold at 1500us. Add a number to hold at a different value, e.g. H L1550.
    // Yaw feedback calibration: prints raw ADC continuously for 20s.
    // Move sZ (with HZ, or by hand if unpowered) to each mechanical end,
    // note the raw number at each end, and put them into YAW_FB_RAW_MIN/MAX above.
    if (c0 == 'Y' && n > 1 && line[1] == 'F') {
        Serial.println("=== YAW FEEDBACK RAW READ: 20s ===");
        unsigned long start = millis();
        while (millis() - start < 20000) {
            int raw = analogRead(PIN_YAW_FEEDBACK);
            Serial.print("raw="); Serial.print(raw);
            Serial.print("  (angle guess="); Serial.print(readYawFeedbackAngle(), 1); Serial.println(")");
            delay(200);
        }
        Serial.println("=== DONE ===");
        return;
    }

    // Step test: turn right, STOP 2s, turn left, STOP 2s, repeat 10x — clear pauses to watch each step
    if (c0 == 'Y' && n > 1 && line[1] == 'S') {
        if (!sZ.attached()) { sZ.setPeriodHertz(50); sZ.attach(PIN_SERVO_Z, 900, 2100); }
        Serial.println("=== YAW STEP TEST: 10 cycles, right / stop 2s / left / stop 2s ===");
        sZ.write(90); delay(1000);
        for (int i = 0; i < 10; i++) {
            Serial.print("Cycle "); Serial.print(i+1); Serial.println(" -> RIGHT");
            sZ.write(YAW_ANGLE_HI); delay(400);
            sZ.write(90);
            Serial.println("   STOP 2s");
            delay(2000);

            Serial.print("Cycle "); Serial.print(i+1); Serial.println(" -> LEFT");
            sZ.write(YAW_ANGLE_LO); delay(400);
            sZ.write(90);
            Serial.println("   STOP 2s");
            delay(2000);
        }
        Serial.println("=== DONE ===");
        return;
    }

    // Yaw drift test: runs 20 left/right swing cycles using YAW_L_MS / YAW_R_MS,
    // then returns to 90. Watch where it ends up relative to where it started —
    // that gap tells you which constant to adjust and in which direction.
    if (c0 == 'Y' && n > 1 && line[1] == 'T') {
        if (!sZ.attached()) { sZ.setPeriodHertz(50); sZ.attach(PIN_SERVO_Z, 900, 2100); }
        Serial.println("=== YAW DRIFT TEST: 20 cycles, watch the final resting spot ===");
        sZ.write(90); delay(400);
        for (int i = 0; i < 20; i++) {
            sZ.write(YAW_ANGLE_LO); delay(YAW_L_MS);
            sZ.write(YAW_ANGLE_HI); delay(YAW_R_MS);
        }
        sZ.write(90);
        Serial.println("=== DONE — compare final position to where it started ===");
        Serial.println("Drifted LEFT of start -> raise YAW_R_MS (or lower YAW_L_MS)");
        Serial.println("Drifted RIGHT of start -> raise YAW_L_MS (or lower YAW_R_MS)");
        return;
    }

    if (c0 == 'H' && n > 1 && (line[1] == 'L' || line[1] == 'R' || line[1] == 'B' || line[1] == 'Z')) {
        char which = line[1];
        Servo* s = (which == 'L') ? &sL : (which == 'R') ? &sR : (which == 'B') ? &sB : &sZ;
        int pin = (which == 'L') ? PIN_SERVO_L : (which == 'R') ? PIN_SERVO_R : (which == 'B') ? PIN_SERVO_B : PIN_SERVO_Z;
        int val = (n > 2 && isdigit((unsigned char)line[2])) ? atoi(line + 2) : 1500;
        if (!s->attached()) {
            if (which == 'Z') { s->setPeriodHertz(50); s->attach(pin, 900, 2100); }
            else s->attach(pin);
        }
        s->writeMicroseconds(val);
        Serial.print("HOLDING "); Serial.print(which); Serial.print(" at "); Serial.println(val);
        Serial.println("Now turn that servo's trim screw slowly with a small screwdriver until it stops.");
        Serial.println("Send 'S' when done to stop/release all neck motors.");
        return;
    }

    if (c0 == 'S' && n > 1 && (line[1] == 'L' || line[1] == 'R' || line[1] == 'B' || line[1] == 'Z')) {
        char which = line[1];
        Servo* s = (which == 'L') ? &sL : (which == 'R') ? &sR : (which == 'B') ? &sB : &sZ;
        int center = (which == 'L') ? STOP_VAL_L : (which == 'R') ? STOP_VAL_R : (which == 'B') ? STOP_VAL_B : 1500;
        if (which == 'Z' && !sZ.attached()) { sZ.setPeriodHertz(50); sZ.attach(PIN_SERVO_Z, 900, 2100); }
        else attachNeck();
        Serial.print("=== SWEEP "); Serial.print(which); Serial.println(" START — watch THIS motor only ===");
        for (int off = -200; off <= 200; off += 40) {
            int val = center + off;
            s->writeMicroseconds(val);
            Serial.print("value="); Serial.println(val);
            delay(2500);
        }
        s->writeMicroseconds(center);
        Serial.println("=== SWEEP END, back to assumed center ===");
        return;
    }

    // Manual single-value servo test: L1520 / R1480 / B1500 / Z90
    // Type a letter followed immediately by a number, no space.
    // L/R/B expect a microsecond value (near 1500). Z expects an angle (0-180).
    if ((c0 == 'L' || c0 == 'R' || c0 == 'B' || c0 == 'Z') && n > 1 && isdigit((unsigned char)line[1])) {
        int val = atoi(line + 1);
        if (c0 == 'L') { attachNeck(); sL.writeMicroseconds(val); Serial.print("L set to "); Serial.println(val); }
        if (c0 == 'R') { attachNeck(); sR.writeMicroseconds(val); Serial.print("R set to "); Serial.println(val); }
        if (c0 == 'B') { attachNeck(); sB.writeMicroseconds(val); Serial.print("B set to "); Serial.println(val); }
        if (c0 == 'Z') {
            if (!sZ.attached()) { sZ.setPeriodHertz(50); sZ.attach(PIN_SERVO_Z, 900, 2100); }
            val = constrain(val, 0, 180); sZ.write(val); Serial.print("Z set to "); Serial.println(val);
        }
        return;
    }

    // Raw tilt monitor: prints tiltDeg continuously for 15 seconds, no correction applied.
    // Tilt the robot by hand while this runs and watch the numbers change.
    // Retry MPU init/calibration live, without a full power cycle.
    // Direct LED test — bypasses all gesture/emotion logic entirely.
    if (c0 == 'L' && n > 1 && line[1] == 'T') {
        Serial.println("=== LED TEST: setting all pixels to full white ===");
        for (int i = 0; i < NUM_LEDS; i++) strip.setPixelColor(i, strip.Color(255,255,255));
        strip.show();
        Serial.print("NUM_LEDS="); Serial.print(NUM_LEDS);
        Serial.print(" PIN_LED="); Serial.println(PIN_LED);
        return;
    }

    if (c0 == 'I' && n > 1 && line[1] == 'R') {
        Serial.println("=== RETRYING MPU INIT ===");
        mpu.initialize();
        delay(100);
        mpuReady = mpu.testConnection();
        Serial.println(mpuReady ? "MPU6050 OK" : "MPU6050 FAIL");
        if (mpuReady) {
            calibrateMpuBaseline();
            Serial.println(mpuBaselineReady ? "BASELINE OK" : "BASELINE FAIL");
        }
        return;
    }

    // Raw I2C scan — checks what's actually responding on the I2C bus, independent of the MPU6050 library.
    if (c0 == 'I' && n > 1 && line[1] == 'S') {
        Serial.println("=== I2C SCAN ===");
        int found = 0;
        for (byte addr = 1; addr < 127; addr++) {
            Wire.beginTransmission(addr);
            byte err = Wire.endTransmission();
            if (err == 0) {
                Serial.print("Found device at 0x");
                if (addr < 16) Serial.print("0");
                Serial.println(addr, HEX);
                found++;
            }
        }
        if (found == 0) Serial.println("NOTHING FOUND on the I2C bus.");
        Serial.println("=== SCAN DONE (MPU6050 should show as 0x68 or 0x69) ===");
        return;
    }

    if (c0 == 'I' && n > 1 && line[1] == 'T') {
        if (!mpuReady || !mpuBaselineReady) { Serial.println("MPU not ready"); return; }
        Serial.println("=== RAW TILT MONITOR: 15s, tilt the head by hand and watch ===");
        unsigned long start = millis();
        while (millis() - start < 15000) {
            int16_t ax16, ay16, az16, gx16, gy16, gz16;
            mpu.getMotion6(&ax16, &ay16, &az16, &gx16, &gy16, &gz16);
            float fax = ax16 / 16384.0f, fay = ay16 / 16384.0f, faz = az16 / 16384.0f;
            float norm = sqrtf(fax*fax + fay*fay + faz*faz);
            if (norm > 0.2f) {
                float nx = fax/norm, ny = fay/norm, nz = faz/norm;
                float dot = constrain(nx*baseAx + ny*baseAy + nz*baseAz, -1.0f, 1.0f);
                float tiltDeg = acosf(dot) * 57.2958f;
                Serial.print("tilt="); Serial.println(tiltDeg, 1);
            }
            delay(200);
        }
        Serial.println("=== DONE ===");
        return;
    }

    if (c0 == '@') {
        runCableAngleCalibration();
        return;
    }

    if (c0 == '~') {
        if (n >= 2 && line[1] == '1') {
            startWakeBlink();
            Serial.println("EVT:wake_blink_on");
        } else {
            stopWakeBlink();
            Serial.println("EVT:wake_blink_off");
        }
        return;
    }

    if (wakeBlinkActive) stopWakeBlink();
    executeEmotion(c0);
}

void readSerial() {
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            if (rxLen > 0) {
                rxBuf[rxLen] = '\0';
                handleLine(rxBuf);
                rxLen = 0;
            }
        } else {
            if (rxLen < sizeof(rxBuf) - 1) {
                rxBuf[rxLen++] = c;
            } else {
                rxLen = 0;
            }
        }
    }
}

// ================= TOUCH EVENT STATE MACHINE =================
unsigned long touchStartMs = 0;
bool touchHeld = false;
char touchActiveSide = ' ';

const char* kTouchEmotionPool[]   = { "happy", "very_happy", "surprise", "good" };
const char  kTouchEmotionLetter[] = { 'a',     'v',          'q',        'd'    };
const uint8_t kTouchPoolSize = 4;

unsigned long lastTouchActionAt = 0;
const unsigned long TOUCH_COOLDOWN_MS = 1200;

void readTouch() {
    bool leftNow  = digitalRead(PIN_TOUCH_L);
    bool rightNow = digitalRead(PIN_TOUCH_R);
    bool anyNow   = leftNow || rightNow;

    if (anyNow && !touchHeld) {
        touchHeld = true;
        touchStartMs = millis();
        if (leftNow && rightNow) touchActiveSide = 'B';
        else if (leftNow)        touchActiveSide = 'L';
        else                     touchActiveSide = 'R';

        unsigned long now = millis();
        if (now - lastTouchActionAt < TOUCH_COOLDOWN_MS) return;
        lastTouchActionAt = now;

        uint8_t idx = (uint8_t)(random(0, kTouchPoolSize));
        const char* emoName   = kTouchEmotionPool[idx];
        char        emoLetter = kTouchEmotionLetter[idx];

        Serial.print("EVT:touch_emotion:");
        Serial.println(emoName);

        if (wakeBlinkActive) stopWakeBlink();
        executeEmotion(emoLetter);

        delay(50);
    } else if (!anyNow && touchHeld) {
        touchHeld = false;
        touchActiveSide = ' ';
    } else if (anyNow && touchHeld) {
        if (leftNow && rightNow) touchActiveSide = 'B';
    }
}

// ================= Loop =================
void loop() {
    readSerial();
    readTouch();
    updateWakeBlink();
    server.handleClient();
}

// ================= EXECUTE ACTION =================
void executeEmotion(char cmd) {
    showEmotionLED(cmd);

    switch(cmd) {

        case 'C': { // CALIBRATION — find the real STOP point of each continuous servo
            Serial.println("=== STOP CALIBRATION START ===");
            Serial.println("Watch/listen to EACH motor. Note the value where it visibly stops.");
            attachNeck();

            const char* names[3] = {"L", "R", "B"};
            Servo* servos[3] = {&sL, &sR, &sB};
            int centers[3] = {STOP_VAL_L, STOP_VAL_R, STOP_VAL_B};

            for (int m = 0; m < 3; m++) {
                Serial.print("--- Testing servo "); Serial.print(names[m]); Serial.println(" ---");
                for (int off = -80; off <= 80; off += 20) {
                    int val = centers[m] + off;
                    servos[m]->writeMicroseconds(val);
                    Serial.print("  value="); Serial.print(val);
                    Serial.print("  (offset "); Serial.print(off); Serial.println(")");
                    delay(1200); // watch/listen during this window
                }
                servos[m]->writeMicroseconds(centers[m]); // back to assumed center before moving on
                Serial.print("--- "); Serial.print(names[m]); Serial.println(" done, back to assumed center ---");
                delay(800);
            }

            stopNeck();
            Serial.println("=== STOP CALIBRATION END ===");
            Serial.println("Tell me the offset where each motor (L/R/B) actually stopped.");
            break;
        }

        case 'N': { // DIAGNOSTIC TEST — moves each neck motor alone, with serial confirmation at each step
            Serial.println("=== N TEST START ===");
            attachNeck();
            Serial.println("attachNeck() done");

            Serial.println("-> sL PULL for 1s");
            sL.writeMicroseconds(PULL_L);
            delay(1000);
            sL.writeMicroseconds(STOP_VAL_L);
            Serial.println("-> sL STOP");
            delay(500);

            Serial.println("-> sR PULL for 1s");
            sR.writeMicroseconds(PULL_R);
            delay(1000);
            sR.writeMicroseconds(STOP_VAL_R);
            Serial.println("-> sR STOP");
            delay(500);

            Serial.println("-> sB PULL for 1s");
            sB.writeMicroseconds(PULL_B);
            delay(1000);
            sB.writeMicroseconds(STOP_VAL_B);
            Serial.println("-> sB STOP");
            delay(500);

            Serial.println("-> sZ small sweep (safe range)");
            sZ.write(80); delay(400);
            sZ.write(100); delay(400);
            sZ.write(90);
            Serial.println("-> sZ done");

            stopNeck();
            Serial.println("=== N TEST END ===");
            break;
        }

        case 'A': // Happy — playful head shake using ONLY L and R, nothing else moves
            setBodySmooth(n_ear, h_ear, n_arm, h_arm, 8, 18);
            attachNeck();
            sB.writeMicroseconds(STOP_VAL_B); // B fixed, never touched again
            for (int i = 0; i < 4; i++) {
                sL.writeMicroseconds(PULL_L);  sR.writeMicroseconds(RLSE_R); delay(150);
                sL.writeMicroseconds(RLSE_L);  sR.writeMicroseconds(PULL_R); delay(150);
            }
            sL.writeMicroseconds(STOP_VAL_L);
            sR.writeMicroseconds(STOP_VAL_R);
            delay(50);
            sL.detach(); sR.detach(); sB.detach();
            setBodySmooth(h_ear, h_ear, h_arm, 170, 4, 20);
            setBodySmooth(h_ear, n_ear, 170, n_arm, 6, 18);
            break;

        case 'O': // Angry
            setBodySmooth(n_ear, a_ear, n_arm, a_arm, 8, 18);
            attachNeck();
            for (int i = 0; i < 3; i++) {
                driveCablesGeometric(ANGLE_B_DEG, SPD); delay(90);
                driveCablesGeometric(ANGLE_B_DEG, 0); delay(90);
            }
            for (int i = 0; i < 2; i++) { yawSwing(); }
            returnToNeutral(a_ear, a_arm);
            break;

        case 'P': // Scared
            setBodySmooth(n_ear, 28, n_arm, 155, 10, 18);
            sZ.write(90);
            attachNeck();
            for (int i = 0; i < 4; i++) {
                setLedColor(strip.Color(128, 0, 255));
                setBody(24, 165);
                neckMicroShake(4, 10, 70);
                delay(35);
                setLedColor(0);
                setBody(34, 145);
                neckMicroShake(3, 8, 60);
                delay(35);
            }
            setLedColor(strip.Color(128, 0, 255));
            setBody(28, 155);
            delay(220);
            returnToNeutral(28, 155);
            setLedColor(0);
            break;

        case 'Q': // Surprise
            setBodySmooth(n_ear, 6, n_arm, 145, 8, 18);
            sZ.write(90);
            attachNeck();
            setLedColor(strip.Color(255, 255, 255));
            driveCablesGeometric(ANGLE_B_DEG, SPD); delay(120);
            driveCablesGeometric(ANGLE_B_DEG, 0); delay(1);
            setBody(2, 160);
            yawSwing();
            setBody(8, 145);
            delay(260);
            returnToNeutral(8, 145);
            break;

        case 'S': // Stop / Neutral
            stopNeck();
            sZ.write(90);
            levelHeadWithIMU();
            setBodySmooth(n_ear, n_ear, n_arm, n_arm, 6, 18);
            setLedColor(0);
            lastEmotionColor = 0;
            break;

        case 'V': case 'D': // Positive — "Yes": natural nod (down-up-down-up) using ONLY L and R together, B untouched
            attachNeck();
            sB.writeMicroseconds(STOP_VAL_B); // B stays at rest, never commanded to move
            for (int i = 0; i < 3; i++) {
                sL.writeMicroseconds(PULL_L);  sR.writeMicroseconds(PULL_R);  delay(200); // down
                sL.writeMicroseconds(RLSE_L);  sR.writeMicroseconds(RLSE_R);  delay(200); // up
            }
            sL.writeMicroseconds(STOP_VAL_L);
            sR.writeMicroseconds(STOP_VAL_R);
            delay(50);
            sL.detach(); sR.detach(); sB.detach();
            break;

        case 'X': case 'T': case 'W': case 'G': // Negative — "No": only sZ shakes left-right
            for (int i = 0; i < 3; i++) { yawSwing(); }
            break;

        // ============================================================
        // Touch-only: lowercase letters = gentle version of the motion
        // ============================================================
        case 'a': { // touch happy, gentle version
            int ear_mid = (n_ear + h_ear) / 2;
            int arm_mid = (n_arm + h_arm) / 2;
            setBodySmooth(n_ear, ear_mid, n_arm, arm_mid, 10, 26);
            attachNeck();
            driveCablesGeometric(ANGLE_B_DEG, -SPD); delay(220);
            driveCablesGeometric(ANGLE_B_DEG, 0); delay(200);
            returnToNeutral(ear_mid, arm_mid);
            break;
        }
        case 'v': { // touch very_happy, gentle version
            int ear_high = h_ear + (n_ear - h_ear) / 4;
            int arm_high = h_arm - (h_arm - n_arm) / 5;
            setBodySmooth(n_ear, ear_high, n_arm, arm_high, 9, 22);
            attachNeck();
            for (int i = 0; i < 2; i++) {
                driveCablesGeometric(ANGLE_B_DEG, -SPD); delay(160);
                driveCablesGeometric(ANGLE_B_DEG, 0); delay(120);
            }
            returnToNeutral(ear_high, arm_high);
            break;
        }
        case 'd': { // touch good, gentle version
            int ear_mid = n_ear + (h_ear - n_ear) / 3;
            int arm_mid = n_arm + (h_arm - n_arm) / 3;
            setBodySmooth(n_ear, ear_mid, n_arm, arm_mid, 8, 28);
            attachNeck();
            driveCablesGeometric(ANGLE_B_DEG, SPD); delay(280);
            driveCablesGeometric(ANGLE_B_DEG, 0); delay(180);
            returnToNeutral(ear_mid, arm_mid);
            break;
        }
        case 'q': { // touch oh, gentle version
            int ear_up = n_ear - (n_ear - q_ear) / 2;
            setBodySmooth(n_ear, ear_up, n_arm, n_arm, 8, 24);
            attachNeck();
            driveCablesGeometric(ANGLE_B_DEG, -SPD); delay(180);
            driveCablesGeometric(ANGLE_B_DEG, 0); delay(160);
            returnToNeutral(ear_up, n_arm);
            break;
        }
        case 'u': { // touch calm
            attachNeck();
            driveCablesGeometric(ANGLE_B_DEG, SPD); delay(320);
            driveCablesGeometric(ANGLE_B_DEG, 0); delay(200);
            returnToNeutral(n_ear, n_arm);
            break;
        }

        case 'U': // Serious
            attachNeck();
            sZ.write(90);
            driveCablesGeometric(ANGLE_B_DEG, -SPD); delay(160);
            driveCablesGeometric(ANGLE_B_DEG, 0);
            returnToNeutral(n_ear, n_arm);
            break;

        // ============================================================
        // Gestures
        // ============================================================
        case 'K': // OK
            setBodySmooth(n_ear, h_ear, n_arm, h_arm, 8, 18);
            sZ.write(90);
            attachNeck();
            for (int i = 0; i < 3; i++) {
                driveCablesGeometric(ANGLE_B_DEG, SPD); delay(150);
                driveCablesGeometric(ANGLE_B_DEG, -SPD); delay(150);
            }
            driveCablesGeometric(ANGLE_B_DEG, 0);
            returnToNeutral(h_ear, h_arm);
            break;

        case 'E': // Peace
            setBodySmooth(n_ear, 45, n_arm, 120, 8, 18);
            sZ.write(90);
            attachNeck();
            for (int i = 0; i < 2; i++) { yawSwing(); }
            driveCablesGeometric(ANGLE_B_DEG, SPD); delay(180);
            driveCablesGeometric(ANGLE_B_DEG, 0);
            returnToNeutral(45, 120);
            break;

        case 'Y': { // Shy — R pulls 1.5cm while sZ glides slowly to 180,
                     // then L pulls 1.5cm while sZ glides slowly back to 90,
                     // then L and R release together back to 0
            attachNeck();
            if (!sZ.attached()) { sZ.setPeriodHertz(50); sZ.attach(PIN_SERVO_Z, 900, 2100); }

            const int PULL_MS = 1560; // ~1.5cm of cable at SPD, based on the pulley-radius calculation
            const int STEPS = 30;
            const int STEP_MS = PULL_MS / STEPS;

            // Phase 1: R pulls 1.5cm, sZ glides 90 -> 180 at the same time
            sR.writeMicroseconds(PULL_R);
            for (int i = 0; i <= STEPS; i++) {
                sZ.write(90 + (i * 90) / STEPS);
                delay(STEP_MS);
            }
            sR.writeMicroseconds(STOP_VAL_R);

            // Phase 2: L pulls 1.5cm, sZ glides 180 -> 90 at the same time
            sL.writeMicroseconds(PULL_L);
            for (int i = 0; i <= STEPS; i++) {
                sZ.write(180 - (i * 90) / STEPS);
                delay(STEP_MS);
            }
            sL.writeMicroseconds(STOP_VAL_L);

            // Phase 3: L and R release together, back to 0
            sL.writeMicroseconds(RLSE_L);
            sR.writeMicroseconds(RLSE_R);
            delay(PULL_MS);
            sL.writeMicroseconds(STOP_VAL_L);
            sR.writeMicroseconds(STOP_VAL_R);
            delay(50);
            sL.detach(); sR.detach(); sB.detach();
            break;
        }

        case '1': // Uno
            setBodySmooth(n_ear, h_ear, n_arm, h_arm, 8, 18);
            sZ.write(90);
            attachNeck();
            driveCablesGeometric(ANGLE_B_DEG, SPD); delay(220);
            driveCablesGeometric(ANGLE_B_DEG, -SPD); delay(150);
            driveCablesGeometric(ANGLE_B_DEG, 0);
            returnToNeutral(h_ear, h_arm);
            break;

        case '3': // Tres
            setBodySmooth(n_ear, h_ear, n_arm, h_arm, 6, 16);
            sZ.write(90);
            attachNeck();
            for (int i = 0; i < 3; i++) {
                driveCablesGeometric(ANGLE_B_DEG, SPD); delay(170);
                driveCablesGeometric(ANGLE_B_DEG, -SPD); delay(140);
            }
            driveCablesGeometric(ANGLE_B_DEG, 0);
            returnToNeutral(h_ear, h_arm);
            break;

        case '4': // Cuatro
            setBodySmooth(n_ear, h_ear, n_arm, h_arm, 6, 14);
            sZ.write(90);
            attachNeck();
            for (int i = 0; i < 4; i++) {
                driveCablesGeometric(ANGLE_B_DEG, SPD); delay(150);
                driveCablesGeometric(ANGLE_B_DEG, -SPD); delay(120);
            }
            driveCablesGeometric(ANGLE_B_DEG, 0);
            returnToNeutral(h_ear, h_arm);
            break;

        default:
            break;
    }
}
