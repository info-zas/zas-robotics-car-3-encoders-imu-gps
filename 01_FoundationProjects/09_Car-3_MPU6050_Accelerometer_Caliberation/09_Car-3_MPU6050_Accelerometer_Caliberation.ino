#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

// =====================================================
// MODE SELECT (Students change only this line)
// =====================================================
// true  = PHASE-1: COLLECT min/max by rotating the robot (6 poses)
// false = PHASE-2: APPLY calibration using your recorded min/max
const bool COLLECT_MIN_MAX = true;

// =====================================================
// PHASE-2: Paste YOUR measured values here (in "g" units)
// These are used ONLY when COLLECT_MIN_MAX = false
// =====================================================
float xMax =  1.00, xMin = -1.00;
float yMax =  1.00, yMin = -1.00;
float zMax =  1.00, zMin = -1.00;

// =====================================================
// Derived calibration parameters (computed from min/max)
// offset = (max + min)/2
// scale  = 2/(max - min)
// A_cal  = scale * (A_raw - offset)
// =====================================================
float xOffset, yOffset, zOffset;
float xScale,  yScale,  zScale;

// =====================================================
// Variables for PHASE-1 (live min/max tracking)
// Start with extreme values so updates work correctly
// =====================================================
float obsXmax = -999, obsXmin =  999;
float obsYmax = -999, obsYmin =  999;
float obsZmax = -999, obsZmin =  999;

void computeCalibrationParams() {
  // Bias (center shift)
  xOffset = (xMax + xMin) / 2.0;
  yOffset = (yMax + yMin) / 2.0;
  zOffset = (zMax + zMin) / 2.0;

  // Gain (scale correction)
  xScale  = 2.0 / (xMax - xMin);
  yScale  = 2.0 / (yMax - yMin);
  zScale  = 2.0 / (zMax - zMin);
}

void setup() {
  Serial.begin(115200);
  delay(300);

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("ERROR: MPU6050 NOT FOUND!");
    while (1);
  }

  Serial.println("MPU6050 Started!");

  // Best practice: use +/-2g range for highest sensitivity
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);

  // Low bandwidth reduces noise (good for calibration)
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Compute offsets/scales (used in APPLY mode)
  computeCalibrationParams();

  // Print student instructions once
  if (COLLECT_MIN_MAX) {
    Serial.println("\n===== PHASE-1: COLLECT MIN/MAX =====");
    Serial.println("Rotate the robot in 6 poses and HOLD still 2–3 seconds each:");
    Serial.println("1) X up, 2) X down, 3) Y up, 4) Y down, 5) Z up, 6) Z down");
    Serial.println("Copy final values: Xmax, Xmin, Ymax, Ymin, Zmax, Zmin");
    Serial.println("Then paste them into this code and set COLLECT_MIN_MAX = false.\n");
  } else {
    Serial.println("\n===== PHASE-2: APPLY CALIBRATION =====");
    Serial.println("Now Ax/Ay/Az should match physics:");
    Serial.println("Axis aligned with gravity -> about +1 or -1");
    Serial.println("Other axes -> about 0");
    Serial.println("Use Serial Plotter with UL=+1 and LL=-1 lines.\n");
  }
}

void loop() {
  // Read accelerometer
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Convert from m/s^2 to g
  float Ax = a.acceleration.x / 9.81;
  float Ay = a.acceleration.y / 9.81;
  float Az = a.acceleration.z / 9.81;

  // =====================================================
  // PHASE-1: Collect min/max (rotate robot)
  // =====================================================
  if (COLLECT_MIN_MAX) {
    // Update observed min/max for each axis
    if (Ax > obsXmax) obsXmax = Ax;
    if (Ax < obsXmin) obsXmin = Ax;

    if (Ay > obsYmax) obsYmax = Ay;
    if (Ay < obsYmin) obsYmin = Ay;

    if (Az > obsZmax) obsZmax = Az;
    if (Az < obsZmin) obsZmin = Az;

    // Print current + running min/max
    Serial.print("Raw(g) Ax:"); Serial.print(Ax, 3);
    Serial.print(" Ay:");       Serial.print(Ay, 3);
    Serial.print(" Az:");       Serial.print(Az, 3);

    Serial.print(" | Xmax:");   Serial.print(obsXmax, 3);
    Serial.print(" Xmin:");     Serial.print(obsXmin, 3);

    Serial.print(" | Ymax:");   Serial.print(obsYmax, 3);
    Serial.print(" Ymin:");     Serial.print(obsYmin, 3);

    Serial.print(" | Zmax:");   Serial.print(obsZmax, 3);
    Serial.print(" Zmin:");     Serial.println(obsZmin, 3);
  }

  // =====================================================
  // PHASE-2: Apply calibration (after pasting min/max)
  // =====================================================
  else {
    // Apply offset+scale correction
    Ax = xScale * (Ax - xOffset);
    Ay = yScale * (Ay - yOffset);
    Az = zScale * (Az - zOffset);

    // Print in Serial Plotter format + reference lines
    Serial.print("Ax:"); Serial.print(Ax, 3); Serial.print(",");
    Serial.print("Ay:"); Serial.print(Ay, 3); Serial.print(",");
    Serial.print("Az:"); Serial.print(Az, 3); Serial.print(",");

    // UL/LL are reference lines for plotter (target range)
    Serial.print("UL:"); Serial.print(1);     Serial.print(",");
    Serial.print("LL:"); Serial.println(-1);
  }

  delay(50);
}
