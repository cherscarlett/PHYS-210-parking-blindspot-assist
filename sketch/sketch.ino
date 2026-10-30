#include <ArduinoBLE.h>
#include <Arduino_LSM6DS3.h>
#include <math.h>

int blueLED = 12;
int redLED = 3;
int whiteLED = 1;

int buzzer = 9;

int passengerLED = 11;
int driverLED = 15;
int frontLED = 14;
int rearLED = 2;

int driverTrig = 7;
int driverEcho = 6;

int passengerTrig = 5;
int passengerEcho = 4;

int frontTrig = 16;
int frontEcho = 17;

int rearTrig = 20;
int rearEcho = 21;

const int distanceThreshold = 30;
const int blindSpotThreshold = 20;
const int dangerThreshold = 10;

// Order: front, driver, rear, passenger. Read one sensor per interval
// to reduce ultrasonic cross-talk.
const unsigned long sensorInterval = 60;

int trigPins[4] = {
  frontTrig, driverTrig, rearTrig, passengerTrig
};

int echoPins[4] = {
  frontEcho, driverEcho, rearEcho, passengerEcho
};

int sensorLEDs[4] = {
  frontLED, driverLED, rearLED, passengerLED
};

unsigned short distances[4] = {0, 0, 0, 0};

unsigned long lastReading = 0;
int nextSensor = 0;
bool alarmActive = false;

// --- Gyro-assisted dead reckoning ---
// Calibrate while stationary. This remains an estimate, not an odometer.
// Negative linear X acceleration corresponds to forward movement.
const int calibrationSamplesNeeded = 100;
const float standardGravity = 9.80665f;      // m/s^2 per g
const float accelerationDeadbandG = 0.020f;  // Tune after examining raw readings
const float startSpeedMps = 0.04f;            // Start showing Drive/Reverse
const float stopSpeedMps = 0.015f;           // Return to Park near zero speed
const unsigned long maxSampleGapUs = 150000UL;
const unsigned long speedPublishIntervalMs = 200;
const unsigned long rawAccelerationPublishIntervalMs = 20;

// Complementary filter: gyroscope handles rapid tilt, accelerometer corrects
// long-term gyro drift. Gravity and linear acceleration are not identical.
const float gravityCorrectionPerSecond = 0.50f;
const float orientationEventDps = 15.0f;
const float stationaryGyroDps = 6.0f;
const float stationaryRawChangeG = 0.012f;
const float stationaryLinearAccelerationG = 0.035f;
const unsigned long stationaryHoldMs = 700;
const unsigned long recentBrakeWindowMs = 1800;
const unsigned long recentTiltWindowMs = 2200;

bool imuReady = false;
bool baselineReady = false;
int calibrationSamples = 0;
float calibrationSumX = 0.0f;
float calibrationSumY = 0.0f;
float calibrationSumZ = 0.0f;
float gravityX = 0.0f;
float gravityY = 0.0f;
float gravityZ = 1.0f;
float calibratedGravityMagnitude = 1.0f;
float calibrationGyroSumX = 0.0f;
float calibrationGyroSumY = 0.0f;
float calibrationGyroSumZ = 0.0f;
int calibrationGyroSamples = 0;
float gyroBiasX = 0.0f;
float gyroBiasY = 0.0f;
float gyroBiasZ = 0.0f;
float latestGyroX = 0.0f;
float latestGyroY = 0.0f;
float latestGyroZ = 0.0f;
float previousRawX = 0.0f;
float previousRawY = 0.0f;
float previousRawZ = 0.0f;
unsigned long stableSinceMs = 0;
unsigned long lastBrakingMs = 0;
unsigned long lastTiltMs = 0;

// Positive is forward; negative is backward. Position is relative to Park.
float estimatedVelocityMps = 0.0f;
float estimatedPositionM = 0.0f;
float previousAccelerationMps2 = 0.0f;
unsigned long lastIMUReadUs = 0;
unsigned long lastSpeedPublishMs = 0;
unsigned short lastPublishedSpeedCmS = 0;
unsigned long lastRawAccelerationPublishMs = 0;

// BLE codes: 0=Park, 3=Drive, 4=Reverse.
byte motionDirection = 0;
const char* directionNames[5] = {
  "Park", "", "", "Drive", "Reverse"
};

BLEService ledService("180A");

BLEByteCharacteristic switchCharacteristic(
  "2A57", BLERead | BLEWrite
);

BLEUnsignedShortCharacteristic driverDistanceCharacteristic(
  "19b10001-e8f2-537e-4f6c-d104768a1214",
  BLERead | BLENotify
);

BLEUnsignedShortCharacteristic passengerDistanceCharacteristic(
  "19b10002-e8f2-537e-4f6c-d104768a1214",
  BLERead | BLENotify
);

BLEUnsignedShortCharacteristic frontDistanceCharacteristic(
  "19b10003-e8f2-537e-4f6c-d104768a1214",
  BLERead | BLENotify
);

BLEUnsignedShortCharacteristic rearDistanceCharacteristic(
  "19b10004-e8f2-537e-4f6c-d104768a1214",
  BLERead | BLENotify
);

BLEByteCharacteristic motionCharacteristic(
  "19b10005-e8f2-537e-4f6c-d104768a1214",
  BLERead | BLENotify
);

// Speed as an unsigned 16-bit integer in cm/s (magnitude, not signed).
BLEUnsignedShortCharacteristic speedCharacteristic(
  "19b10006-e8f2-537e-4f6c-d104768a1214",
  BLERead | BLENotify
);

// Three unfiltered IMU acceleration readings (X, Y, Z), in g.
// One notification contains three consecutive little-endian 32-bit floats.
BLECharacteristic rawAccelerationCharacteristic(
  "19b10007-e8f2-537e-4f6c-d104768a1214",
  BLERead | BLENotify, 12, true
);

void publishSpeed(bool force = false) {
  float speedCmS = fabsf(estimatedVelocityMps) * 100.0f;
  if (speedCmS > 65535.0f) speedCmS = 65535.0f;
  unsigned short roundedCmS = (unsigned short)(speedCmS + 0.5f);

  unsigned long nowMs = millis();
  if (force || (roundedCmS != lastPublishedSpeedCmS &&
      nowMs - lastSpeedPublishMs >= speedPublishIntervalMs)) {
    speedCharacteristic.writeValue(roundedCmS);
    lastPublishedSpeedCmS = roundedCmS;
    lastSpeedPublishMs = nowMs;
  }
}

void setMotionDirection(byte newDirection) {
  if (newDirection != motionDirection) {
    if (newDirection == 0) {
      // Start the next journey at position 0, even if we stopped elsewhere.
      estimatedPositionM = 0.0f;
    }
    motionDirection = newDirection;
    motionCharacteristic.writeValue(motionDirection);
  }

  digitalWrite(whiteLED, motionDirection == 4 ? HIGH : LOW);
}

void resetEstimatedVelocity() {
  estimatedVelocityMps = 0.0f;
  estimatedPositionM = 0.0f;
  previousAccelerationMps2 = 0.0f;
  lastIMUReadUs = 0;
  stableSinceMs = 0;
  lastBrakingMs = 0;
  lastTiltMs = 0;
  setMotionDirection(0);
  publishSpeed(true);
}

void setup() {
  Serial.begin(9600);

  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(blueLED, OUTPUT);
  pinMode(redLED, OUTPUT);
  pinMode(whiteLED, OUTPUT);
  pinMode(buzzer, OUTPUT);

  pinMode(driverLED, OUTPUT);
  pinMode(passengerLED, OUTPUT);
  pinMode(frontLED, OUTPUT);
  pinMode(rearLED, OUTPUT);

  pinMode(frontTrig, OUTPUT);
  pinMode(frontEcho, INPUT);

  pinMode(rearTrig, OUTPUT);
  pinMode(rearEcho, INPUT);

  pinMode(driverTrig, OUTPUT);
  pinMode(driverEcho, INPUT);

  pinMode(passengerTrig, OUTPUT);
  pinMode(passengerEcho, INPUT);

  digitalWrite(frontTrig, LOW);
  digitalWrite(rearTrig, LOW);
  digitalWrite(driverTrig, LOW);
  digitalWrite(passengerTrig, LOW);

  digitalWrite(blueLED, LOW);
  digitalWrite(frontLED, LOW);
  digitalWrite(rearLED, LOW);
  digitalWrite(driverLED, LOW);
  digitalWrite(passengerLED, LOW);
  digitalWrite(redLED, LOW);
  digitalWrite(whiteLED, LOW);
  noTone(buzzer);

  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(400);
    digitalWrite(LED_BUILTIN, LOW);
    delay(400);
  }

  imuReady = IMU.begin();

  if (imuReady) {
    Serial.println("Accelerometer initialized");
    Serial.println("Keep stationary for IMU calibration...");
  } else {
    Serial.println("Accelerometer failed; distance sensors will continue");
  }

  if (!BLE.begin()) {
    Serial.println("Bluetooth failed!");
    while (1);
  }

  BLE.setLocalName("BB JEEP"); // demo name - change this
  BLE.setAdvertisedService(ledService);

  ledService.addCharacteristic(switchCharacteristic);
  ledService.addCharacteristic(driverDistanceCharacteristic);
  ledService.addCharacteristic(passengerDistanceCharacteristic);
  ledService.addCharacteristic(frontDistanceCharacteristic);
  ledService.addCharacteristic(rearDistanceCharacteristic);
  ledService.addCharacteristic(motionCharacteristic);
  ledService.addCharacteristic(speedCharacteristic);
  ledService.addCharacteristic(rawAccelerationCharacteristic);

  BLE.addService(ledService);

  switchCharacteristic.writeValue((byte)0);
  driverDistanceCharacteristic.writeValue((unsigned short)0);
  passengerDistanceCharacteristic.writeValue((unsigned short)0);
  frontDistanceCharacteristic.writeValue((unsigned short)0);
  rearDistanceCharacteristic.writeValue((unsigned short)0);
  motionCharacteristic.writeValue((byte)0);
  speedCharacteristic.writeValue((unsigned short)0);
  const float zeroAcceleration[3] = {0.0f, 0.0f, 0.0f};
  rawAccelerationCharacteristic.writeValue(
    (const uint8_t*)zeroAcceleration, sizeof(zeroAcceleration)
  );

  BLE.advertise();

  Serial.println("Waiting for Bluetooth connection...");
}

unsigned short readDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, 30000UL);

  if (duration == 0) {
    return 0;
  }

  return (unsigned short)(duration * 0.0343 / 2.0);
}

void updateDangerAlarm() {
  bool danger =
    (distances[0] > 0 && distances[0] < dangerThreshold) ||
    (distances[2] > 0 && distances[2] < dangerThreshold);

  if (danger != alarmActive) {
    alarmActive = danger;

    digitalWrite(redLED, danger ? HIGH : LOW);

    if (danger) {
      tone(buzzer, 2000);
    } else {
      noTone(buzzer);
    }
  }
}

void updateSensors() {
  if (millis() - lastReading < sensorInterval) {
    return;
  }

  lastReading = millis();
  int i = nextSensor;

  distances[i] = readDistance(trigPins[i], echoPins[i]);

  int threshold = (i == 1 || i == 3)
    ? blindSpotThreshold : distanceThreshold;

  bool closeObject = distances[i] > 0 && distances[i] <= threshold;
  digitalWrite(sensorLEDs[i], closeObject ? HIGH : LOW);

  switch (i) {
    case 0:
      frontDistanceCharacteristic.writeValue(distances[i]);
      break;
    case 1:
      driverDistanceCharacteristic.writeValue(distances[i]);
      break;
    case 2:
      rearDistanceCharacteristic.writeValue(distances[i]);
      break;
    case 3:
      passengerDistanceCharacteristic.writeValue(distances[i]);
      break;
  }

  updateDangerAlarm();
  nextSensor = (nextSensor + 1) % 4;
}

void updateMotion() {
  if (!imuReady || !IMU.accelerationAvailable()) {
    return;
  }

  float x, y, z;
  if (!IMU.readAcceleration(x, y, z)) {
    return;
  }

  // Gyroscope readings are in degrees/second
  bool newGyro = false;
  if (IMU.gyroscopeAvailable()) {
    newGyro = IMU.readGyroscope(
      latestGyroX, latestGyroY, latestGyroZ
    );
  }

  // Send direct IMU readings to the Python BLE server, including during
  // startup calibration. Do not apply a baseline or deadband to these values.
  unsigned long nowMs = millis();
  if (nowMs - lastRawAccelerationPublishMs >= rawAccelerationPublishIntervalMs) {
    lastRawAccelerationPublishMs = nowMs;
    const float rawAcceleration[3] = {x, y, z};
    rawAccelerationCharacteristic.writeValue(
      (const uint8_t*)rawAcceleration, sizeof(rawAcceleration)
    );
  }

  unsigned long nowUs = micros();

  if (!baselineReady) {
    calibrationSumX += x;
    calibrationSumY += y;
    calibrationSumZ += z;
    if (newGyro) {
      calibrationGyroSumX += latestGyroX;
      calibrationGyroSumY += latestGyroY;
      calibrationGyroSumZ += latestGyroZ;
      calibrationGyroSamples++;
    }
    calibrationSamples++;

    if (calibrationSamples >= calibrationSamplesNeeded) {
      gravityX = calibrationSumX / calibrationSamples;
      gravityY = calibrationSumY / calibrationSamples;
      gravityZ = calibrationSumZ / calibrationSamples;
      calibratedGravityMagnitude = sqrtf(
        gravityX * gravityX + gravityY * gravityY + gravityZ * gravityZ
      );
      if (calibrationGyroSamples > 0) {
        gyroBiasX = calibrationGyroSumX / calibrationGyroSamples;
        gyroBiasY = calibrationGyroSumY / calibrationGyroSamples;
        gyroBiasZ = calibrationGyroSumZ / calibrationGyroSamples;
      }
      previousRawX = x;
      previousRawY = y;
      previousRawZ = z;
      baselineReady = true;
      lastIMUReadUs = nowUs;
      Serial.println("IMU gravity/gyro calibration complete.");
    }
    return;
  }

  if (lastIMUReadUs == 0) {
    lastIMUReadUs = nowUs;
    return;
  }

  unsigned long elapsedUs = nowUs - lastIMUReadUs;
  lastIMUReadUs = nowUs;

  // Discard unusually long gaps instead of integrating stale acceleration.
  if (elapsedUs == 0 || elapsedUs > maxSampleGapUs) {
    previousAccelerationMps2 = 0.0f;
    return;
  }

  float dt = elapsedUs * 0.000001f;

  float gxDps = latestGyroX - gyroBiasX;
  float gyDps = latestGyroY - gyroBiasY;
  float gzDps = latestGyroZ - gyroBiasZ;
  const float radiansPerDegree = 0.01745329252f;
  float wx = gxDps * radiansPerDegree;
  float wy = gyDps * radiansPerDegree;
  float wz = gzDps * radiansPerDegree;

  // Gyro-predicted gravity vector in the sensor's coordinate frame:
  // dg/dt = -omega x g. Use the old vector for all three components.
  float oldGravityX = gravityX;
  float oldGravityY = gravityY;
  float oldGravityZ = gravityZ;
  gravityX += (wz * oldGravityY - wy * oldGravityZ) * dt;
  gravityY += (wx * oldGravityZ - wz * oldGravityX) * dt;
  gravityZ += (wy * oldGravityX - wx * oldGravityY) * dt;

  float rawMagnitude = sqrtf(x * x + y * y + z * z);
  // Slowly correct gyro drift when the measured vector looks gravity-like.
  if (fabsf(rawMagnitude - calibratedGravityMagnitude) < 0.06f) {
    float correction = gravityCorrectionPerSecond * dt;
    gravityX += correction * (x - gravityX);
    gravityY += correction * (y - gravityY);
    gravityZ += correction * (z - gravityZ);
  }

  // Keep the gravity estimate at the magnitude measured during calibration.
  float gravityMagnitude = sqrtf(
    gravityX * gravityX + gravityY * gravityY + gravityZ * gravityZ
  );
  if (gravityMagnitude > 0.1f) {
    float scale = calibratedGravityMagnitude / gravityMagnitude;
    gravityX *= scale;
    gravityY *= scale;
    gravityZ *= scale;
  }

  float forwardAccelerationG = x - gravityX;

  if (fabsf(forwardAccelerationG) < accelerationDeadbandG) {
    forwardAccelerationG = 0.0f;
  }

  float accelerationMps2 = forwardAccelerationG * standardGravity;
  float oldVelocity = estimatedVelocityMps;

  // Detect signed braking and zero crossings before taking the absolute value
  // for the dashboard. A single step can pass across the stop-speed window.
  float newVelocity = oldVelocity +
    0.5f * (previousAccelerationMps2 + accelerationMps2) * dt;

  bool crossedZero =
    (motionDirection == 3 && oldVelocity > 0.0f && newVelocity <= 0.0f) ||
    (motionDirection == 4 && oldVelocity < 0.0f && newVelocity >= 0.0f);
  if (crossedZero) {
    resetEstimatedVelocity();  // Also resets position and reverse lights.
    return;
  }

  estimatedVelocityMps = newVelocity;

  // Do not accumulate a nonzero position while the dashboard says Park.
  if (motionDirection != 0) {
    estimatedPositionM +=
      0.5f * (oldVelocity + estimatedVelocityMps) * dt;
  }

  previousAccelerationMps2 = accelerationMps2;

  bool braking =
    (oldVelocity > startSpeedMps && accelerationMps2 < -0.20f) ||
    (oldVelocity < -startSpeedMps && accelerationMps2 > 0.20f);
  if (braking) lastBrakingMs = nowMs;

  // A large pitch/roll movement indicates the Jeep may have been picked up
  // or placed at a new angle. Do not use yaw, which changes during steering.
  if (fabsf(gxDps) > orientationEventDps ||
      fabsf(gyDps) > orientationEventDps) {
    lastTiltMs = nowMs;
  }

  // Best-effort Park heuristic: require a recent brake or tilt event, then a
  // stable acceleration vector and nearly zero angular velocity. Constant
  // velocity can still look stationary: this is not a guaranteed detector.
  bool rawStable =
    fabsf(x - previousRawX) < stationaryRawChangeG &&
    fabsf(y - previousRawY) < stationaryRawChangeG &&
    fabsf(z - previousRawZ) < stationaryRawChangeG;
  bool gyroStable =
    fabsf(gxDps) < stationaryGyroDps &&
    fabsf(gyDps) < stationaryGyroDps &&
    fabsf(gzDps) < stationaryGyroDps;
  previousRawX = x;
  previousRawY = y;
  previousRawZ = z;

  if (rawStable && gyroStable) {
    if (stableSinceMs == 0) stableSinceMs = nowMs;
  } else {
    stableSinceMs = 0;
  }

  bool recentBraking =
    lastBrakingMs != 0 && nowMs - lastBrakingMs <= recentBrakeWindowMs;
  bool recentTilt =
    lastTiltMs != 0 && nowMs - lastTiltMs <= recentTiltWindowMs;
  // After a tilt event we can re-zero at rest even while the gravity filter
  // is still converging. After braking, require small corrected acceleration.
  bool settledEnough = recentTilt ||
    fabsf(forwardAccelerationG) <= stationaryLinearAccelerationG;
  if (motionDirection != 0 && stableSinceMs != 0 &&
      nowMs - stableSinceMs >= stationaryHoldMs &&
      settledEnough &&
      (recentBraking || recentTilt)) {
    // Recalibrate the gravity reference only after the Jeep appears settled.
    gravityX = x;
    gravityY = y;
    gravityZ = z;
    calibratedGravityMagnitude = rawMagnitude;
    resetEstimatedVelocity();
    return;
  }

  // Direction comes from estimated velocity, NOT instantaneous acceleration.
  // Hysteresis reduces flicker around 0 m/s.
  byte detectedDirection = motionDirection;

  if (motionDirection == 0) {
    if (estimatedVelocityMps >= startSpeedMps) {
      detectedDirection = 3;  // Drive
    } else if (estimatedVelocityMps <= -startSpeedMps) {
      detectedDirection = 4;  // Reverse
    }
  } else if (fabsf(estimatedVelocityMps) <= stopSpeedMps) {
    resetEstimatedVelocity();  // Park and reset all related estimator state.
    return;
  } else if (motionDirection == 3 &&
             estimatedVelocityMps <= -startSpeedMps) {
    detectedDirection = 4;
  } else if (motionDirection == 4 &&
             estimatedVelocityMps >= startSpeedMps) {
    detectedDirection = 3;
  }

  bool justStopped = (motionDirection != 0 && detectedDirection == 0);
  setMotionDirection(detectedDirection);
  publishSpeed(justStopped);
}

void handleSerialCommands() {
  while (Serial.available() > 0) {
    char command = (char)Serial.read();
    if (command == 'p' || command == 'P') {
      resetEstimatedVelocity();
    }
  }
}

void loop() {
  handleSerialCommands();
  updateMotion();
  updateSensors();

  BLEDevice central = BLE.central();

  if (central) {
    Serial.println("Bluetooth CONNECTED!");
    digitalWrite(blueLED, HIGH);

    while (central.connected()) {
      BLE.poll();
      handleSerialCommands();
      updateMotion();
      updateSensors();
    }

    digitalWrite(blueLED, LOW);
    Serial.println("Bluetooth DISCONNECTED!");
  }
}
