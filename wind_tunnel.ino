// ============================================================
//  WIND TUNNEL COMMAND CENTER — Arduino Firmware
//  Hardware:
//    Load Cell 1 (LIFT) : HX711  DOUT=4  SCK=5
//    Load Cell 2 (DRAG) : HX711  DOUT=6  SCK=7
//    AoA Servo          : Pin 9
//  Baud: 57600
//
//  Serial OUTPUT (20 Hz):
//    "Load_cell 1 output val: X    Load_cell 2 output val: Y    AOA: Z"
//
//  Serial INPUT commands:
//    "A<float>\n"  e.g. "A5.50\n"  -> set angle of attack
//    "t\n"                          -> tare both load cells
//
//  AoA Mapping:
//    AoA -90 deg -> Servo   0 deg
//    AoA   0 deg -> Servo  90 deg  (neutral)
//    AoA +90 deg -> Servo 180 deg
//    Typical use: Python sends -20 to +20
//    -> servo moves 70 to 110 deg around neutral
//
//  Required Libraries (install via Library Manager):
//    - HX711_ADC by Olav Kallhovd
//    - Servo (built-in)
// ============================================================

#include <HX711_ADC.h>
#include <Servo.h>

// ── Pin definitions ──────────────────────────────────────────
const int LIFT_DOUT = 4;
const int LIFT_SCK  = 5;
const int DRAG_DOUT = 6;
const int DRAG_SCK  = 7;
const int SERVO_PIN = 9;

// ── Calibration values ───────────────────────────────────────
// Run the HX711_ADC calibration example sketch first
// to find the correct values for your load cells
float calLift = 696.0;
float calDrag = 733.0;

// ── Objects ──────────────────────────────────────────────────
HX711_ADC liftCell(LIFT_DOUT, LIFT_SCK);
HX711_ADC dragCell(DRAG_DOUT, DRAG_SCK);
Servo     aoaServo;

// ── State ────────────────────────────────────────────────────
float         currentAoA  = 0.0;
String        inputBuffer = "";
bool          inputReady  = false;
unsigned long lastPrint   = 0;
const int     PRINT_MS    = 50;   // 20 Hz output rate

// ── AoA to Servo conversion ──────────────────────────────────
// servoPos = AoA + 90
// Clamps input to -90 ... +90 degrees
void setAoA(float aoa) {
  aoa = constrain(aoa, -90.0, 90.0);
  currentAoA = aoa;
  int pos = constrain((int)round(aoa + 90.0), 0, 180);
  aoaServo.write(pos);
}

// ── Setup ────────────────────────────────────────────────────
void setup() {
  Serial.begin(57600);
  delay(10);
  Serial.println("Wind Tunnel Command Center starting...");

  // Servo: start at 0 degrees AoA (neutral position)
  aoaServo.attach(SERVO_PIN);
  setAoA(0.0);
  Serial.println("Servo ready. AoA = 0.0 deg");

  // Load cells: start both simultaneously
  liftCell.begin();
  dragCell.begin();

  unsigned long stabilize = 2000; // ms stabilization before tare
  bool doTare = true;
  byte rdy1 = 0, rdy2 = 0;

  Serial.println("Stabilizing load cells...");
  while ((rdy1 + rdy2) < 2) {
    if (!rdy1) rdy1 = liftCell.startMultiple(stabilize, doTare);
    if (!rdy2) rdy2 = dragCell.startMultiple(stabilize, doTare);
  }

  if (liftCell.getTareTimeoutFlag()) {
    Serial.println("ERROR: LIFT cell timeout - check wiring DOUT=4, SCK=5");
  }
  if (dragCell.getTareTimeoutFlag()) {
    Serial.println("ERROR: DRAG cell timeout - check wiring DOUT=6, SCK=7");
  }

  liftCell.setCalFactor(calLift);
  dragCell.setCalFactor(calDrag);

  Serial.println("READY");
}

// ── Main loop ────────────────────────────────────────────────
void loop() {
  static bool newData = false;

  // Poll both load cells (non-blocking)
  if (liftCell.update()) newData = true;
  dragCell.update();

  // Transmit data at 20 Hz
  if (newData && (millis() - lastPrint >= PRINT_MS)) {
    float lift = liftCell.getData();
    float drag = dragCell.getData();

    Serial.print("Load_cell 1 output val: ");
    Serial.print(lift, 3);
    Serial.print("    Load_cell 2 output val: ");
    Serial.print(drag, 3);
    Serial.print("    AOA: ");
    Serial.println(currentAoA, 2);

    newData   = false;
    lastPrint = millis();
  }

  // Process incoming serial command
  if (inputReady) {
    inputBuffer.trim();

    if (inputBuffer.equalsIgnoreCase("t")) {
      // Tare both load cells
      liftCell.tareNoDelay();
      dragCell.tareNoDelay();
      Serial.println("Tare initiated");

    } else if (inputBuffer.startsWith("A") || inputBuffer.startsWith("a")) {
      // Set angle of attack: "A5.50"
      float angle = inputBuffer.substring(1).toFloat();
      setAoA(angle);
      Serial.print("AOA_SET:");
      Serial.println(currentAoA, 2);
    }

    inputBuffer = "";
    inputReady  = false;
  }

  // Tare completion feedback
  if (liftCell.getTareStatus()) {
    Serial.println("Tare load cell 1 complete");
  }
  if (dragCell.getTareStatus()) {
    Serial.println("Tare load cell 2 complete");
  }
}

// ── Non-blocking serial reader ───────────────────────────────
// Accumulates characters until newline, then sets inputReady
void serialEvent() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      inputReady = true;
    } else if (c != '\r') {
      inputBuffer += c;
    }
  }
}
