#include <ArduinoBLE.h>
#include <Arduino_LSM6DS3.h>
#include <math.h>

int blueLED = 12;
int redLED = 3;
int whiteLED = 8;

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
const int blindSpotThreshold = 10;
const int dangerThreshold = 7;

// Order: front, driver, rear, passenger
// Read one sensor per interval
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

const int calibrationSamplesNeeded = 100;
const float standardGravity = 9.80665f;      
const float accelerationDeadbandG = 0.045f;  
const float startSpeedMps = 0.04f;            
const float stopSpeedMps = 0.015f;           
const unsigned long maxSampleGapUs = 150000UL;
const unsigned long speedPublishIntervalMs = 200;

bool imuReady = false;
bool baselineReady = false;
int calibrationSamples = 0;
float calibrationSumX = 0.0f;
float baselineX = 0.0f;

float estimatedVelocityMps = 0.0f;
float estimatedPositionM = 0.0f;
float previousAccelerationMps2 = 0.0f;
unsigned long lastIMUReadUs = 0;
unsigned long lastSpeedPublishMs = 0;
unsigned short lastPublishedSpeedCmS = 0;

// 0=Park, 3=Drive, 4=Reverse.
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

BLEUnsignedShortCharacteristic speedCharacteristic(
  "19b10006-e8f2-537e-4f6c-d104768a1214",
  BLERead | BLENotify
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
      // Always starts at position 0 to maintain +/- fwd/bck translational relationship
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
  setMotionDirection(0);
  publishSpeed(true);
  Serial.println("Stopped: velocity and position reset to zero.");
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
    Serial.println("Keep Jeep stationary for IMU calibration...");
  } else {
    Serial.println("Accelerometer failed; distance sensors will continue");
  }

  if (!BLE.begin()) {
    Serial.println("Bluetooth failed!");
    while (1);
  }

  BLE.setLocalName("BB JEEP");
  BLE.setAdvertisedService(ledService);

  ledService.addCharacteristic(switchCharacteristic);
  ledService.addCharacteristic(driverDistanceCharacteristic);
  ledService.addCharacteristic(passengerDistanceCharacteristic);
  ledService.addCharacteristic(frontDistanceCharacteristic);
  ledService.addCharacteristic(rearDistanceCharacteristic);
  ledService.addCharacteristic(motionCharacteristic);
  ledService.addCharacteristic(speedCharacteristic);

  BLE.addService(ledService);

  switchCharacteristic.writeValue((byte)0);
  driverDistanceCharacteristic.writeValue((unsigned short)0);
  passengerDistanceCharacteristic.writeValue((unsigned short)0);
  frontDistanceCharacteristic.writeValue((unsigned short)0);
  rearDistanceCharacteristic.writeValue((unsigned short)0);
  motionCharacteristic.writeValue((byte)0);
  speedCharacteristic.writeValue((unsigned short)0);

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

  if (nextSensor == 0) {
    Serial.print("Front: ");
    Serial.print(distances[0]);
    Serial.print(" cm | Driver: ");
    Serial.print(distances[1]);
    Serial.print(" cm | Rear: ");
    Serial.print(distances[2]);
    Serial.print(" cm | Passenger: ");
    Serial.print(distances[3]);
    Serial.print(" cm | Motion: ");
    Serial.print(directionNames[motionDirection]);
    Serial.print(" | Velocity: ");
    Serial.print(estimatedVelocityMps, 3);
    Serial.print(" m/s | Speed: ");
    Serial.print(fabsf(estimatedVelocityMps) * 100.0f, 1);
    Serial.print(" cm/s | Position: ");
    Serial.print(estimatedPositionM, 3);
    Serial.println(" m");
  }
}

void updateMotion() {
  if (!imuReady || !IMU.accelerationAvailable()) {
    return;
  }

  float x, y, z;
  if (!IMU.readAcceleration(x, y, z)) {
    return;
  }

  unsigned long nowUs = micros();

  if (!baselineReady) {
    calibrationSumX += x;
    calibrationSamples++;

    if (calibrationSamples >= calibrationSamplesNeeded) {
      baselineX = calibrationSumX / calibrationSamples;
      baselineReady = true;
      lastIMUReadUs = nowUs;
      Serial.println("IMU calibration complete; ready for motion.");
    }
    return;
  }

  if (lastIMUReadUs == 0) {
    lastIMUReadUs = nowUs;
    return;
  }

  unsigned long elapsedUs = nowUs - lastIMUReadUs;
  lastIMUReadUs = nowUs;

  // Discards long gaps to avoid integrating stale acceleration.
  if (elapsedUs == 0 || elapsedUs > maxSampleGapUs) {
    previousAccelerationMps2 = 0.0f;
    return;
  }

  float dt = elapsedUs * 0.000001f;

  // Positive acceleration = forward.
  float forwardAccelerationG = -(x - baselineX);

  if (fabsf(forwardAccelerationG) < accelerationDeadbandG) {
    forwardAccelerationG = 0.0f;
  }

  float accelerationMps2 = forwardAccelerationG * standardGravity;
  float oldVelocity = estimatedVelocityMps;

  // Integrates acceleration to velocity.
  estimatedVelocityMps +=
    0.5f * (previousAccelerationMps2 + accelerationMps2) * dt;

  // Integrates velocity to position
  estimatedPositionM +=
    0.5f * (oldVelocity + estimatedVelocityMps) * dt;

  previousAccelerationMps2 = accelerationMps2;

  // Direction from estimated velocity
  byte detectedDirection = motionDirection;

  if (motionDirection == 0) {
    if (estimatedVelocityMps >= startSpeedMps) {
      detectedDirection = 3;  // Drive
    } else if (estimatedVelocityMps <= -startSpeedMps) {
      detectedDirection = 4;  // Reverse
    }
  } else if (fabsf(estimatedVelocityMps) <= stopSpeedMps) {
    detectedDirection = 0;  // Park
    estimatedVelocityMps = 0.0f;
    previousAccelerationMps2 = 0.0f;
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
