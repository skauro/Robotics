#include <Wire.h>
#include <Adafruit_VL53L1X.h>

// Sensor instances
Adafruit_VL53L1X sensorLeft = Adafruit_VL53L1X(21);
Adafruit_VL53L1X sensorMiddle = Adafruit_VL53L1X(22);
Adafruit_VL53L1X sensorRight = Adafruit_VL53L1X(23);

// XSHUT pins for each sensor
#define XSHUT_LEFT   21
#define XSHUT_MIDDLE 22
#define XSHUT_RIGHT  23

// Custom I2C addresses for sensors
#define ADDR_LEFT    0x34
#define ADDR_MIDDLE  0x31
#define ADDR_RIGHT   0x32

// Motor control pins DRV8835
#define MODE 18
#define AIN1 19
#define AIN2 20
#define BIN1 1
#define BIN2 10

// Navigation parameters
#define DEFAULT_SPEED 80
#define TURN_SPEED 60
#define OBSTACLE_THRESHOLD 250  // mm (25cm)
#define BACKUP_TIME 150         // ms
#define TURN_TIME 300           // ms
#define SAFETY_DELAY 10         // ms
#define SIDE_BLOCK_TIMEOUT 6000 // 6 seconds

// State variables
int counter = 0;
bool leftLongBlock = false;
bool rightLongBlock = false;
unsigned long leftBlockedStartTime = 0;
unsigned long rightBlockedStartTime = 0;
enum RobotState { FORWARD, BACKWARD, TURN_LEFT, TURN_RIGHT, STOPPED };
RobotState currentState = FORWARD;

void setup() {
  Serial.begin(115200);

  // Initialize motor control
  pinMode(MODE, OUTPUT);
  digitalWrite(MODE, HIGH);
  pinMode(AIN1, OUTPUT);
  pinMode(BIN1, OUTPUT);
  stopMotors();

  // Initialize I2C (SDA: 6, SCL: 7)
  Wire.begin(6, 7, 400000);

  // Initialize sensors
  initializeSensors();

  // Start ranging on all sensors
  sensorLeft.startRanging();
  sensorMiddle.startRanging();
  sensorRight.startRanging();

  // Configure sensor settings
  configureSensor(sensorLeft);
  configureSensor(sensorMiddle);
  configureSensor(sensorRight);

  delay(5000);
}

void initializeSensors() {
  // Reset all sensors
  pinMode(XSHUT_LEFT, OUTPUT);
  pinMode(XSHUT_MIDDLE, OUTPUT);
  pinMode(XSHUT_RIGHT, OUTPUT);

  digitalWrite(XSHUT_LEFT, LOW);
  digitalWrite(XSHUT_MIDDLE, LOW);
  digitalWrite(XSHUT_RIGHT, LOW);
  delay(10);

  // Initialize left sensor
  digitalWrite(XSHUT_LEFT, HIGH);
  delay(10);
  if (!sensorLeft.begin(0x29, &Wire)) {
    Serial.println("Failed to initialize left sensor");
    while (1);
  }
  sensorLeft.VL53L1X_SetI2CAddress(ADDR_LEFT);

  // Initialize middle sensor
  digitalWrite(XSHUT_MIDDLE, HIGH);
  delay(10);
  if (!sensorMiddle.begin(0x29, &Wire)) {
    Serial.println("Failed to initialize middle sensor");
    while (1);
  }
  sensorMiddle.VL53L1X_SetI2CAddress(ADDR_MIDDLE);

  // Initialize right sensor
  digitalWrite(XSHUT_RIGHT, HIGH);
  delay(10);
  if (!sensorRight.begin(0x29, &Wire)) {
    Serial.println("Failed to initialize right sensor");
    while (1);
  }
  sensorRight.VL53L1X_SetI2CAddress(ADDR_RIGHT);
}

void configureSensor(Adafruit_VL53L1X &sensor) {
  sensor.VL53L1X_SetDistanceMode(1);  // Short distance mode
  sensor.setTimingBudget(15);         // 15ms timing budget
}

void loop() {
  // Read sensor data
  int distLeft = readSensor(sensorLeft);
  int distMiddle = readSensor(sensorMiddle);
  int distRight = readSensor(sensorRight);

  // Print sensor readings
  Serial.print("L: "); Serial.print(distLeft);
  Serial.print(" M: "); Serial.print(distMiddle);
  Serial.print(" R: "); Serial.println(distRight);

  // Determine obstacle presence
  bool blockedLeft = (distLeft >= 0 && distLeft < OBSTACLE_THRESHOLD);
  bool blockedMiddle = (distMiddle >= 0 && distMiddle < OBSTACLE_THRESHOLD);
  bool blockedRight = (distRight >= 0 && distRight < OBSTACLE_THRESHOLD);

    // Check for long blocks on sides
  checkLongSideBlocks(blockedLeft, blockedRight);

    // Handle long side blocks if detected
  if (leftLongBlock || rightLongBlock) {
    handleLongSideBlocks(blockedLeft, blockedRight);
    leftLongBlock = false;
    rightLongBlock = false;
    leftBlockedStartTime = 0;
    rightBlockedStartTime = 0;
    return; // Skip normal navigation this cycle
  }


  // Navigation logic
  if (blockedMiddle) {
    currentState = BACKWARD;
    moveBackward();
    delay(BACKUP_TIME);
    
    if (!blockedLeft && !blockedRight) {
      // If both sides are clear, alternate turning directions
      if (counter % 2 == 0) {
        currentState = TURN_LEFT;
        turnLeft();
      } else {
        currentState = TURN_RIGHT;
        turnRight();
      }
    } else if (!blockedLeft) {
      currentState = TURN_LEFT;
      turnLeft();
    } else if (!blockedRight) {
      currentState = TURN_RIGHT;
      turnRight();
    } else {
      // Completely blocked - try backing up more and turning
      currentState = BACKWARD;
      moveBackward();
      delay(BACKUP_TIME * 2);
      counter++;
      if (counter >= 6) {
        currentState = TURN_RIGHT;
        turnRight();
        counter = 0;
      } else {
        currentState = TURN_LEFT;
        turnLeft();
      }
    }
  } else {
    currentState = FORWARD;
    moveForward();
  }

  delay(SAFETY_DELAY);
}

int readSensor(Adafruit_VL53L1X &sensor) {
  if (sensor.dataReady()) {
    return sensor.distance();
  }
  return -1;
}

void checkLongSideBlocks(bool blockedLeft, bool blockedRight) {
  unsigned long currentTime = millis();
  
  // Check left sensor
  if (blockedLeft) {
    if (leftBlockedStartTime == 0) {
      leftBlockedStartTime = currentTime;
    } else if (currentTime - leftBlockedStartTime >= SIDE_BLOCK_TIMEOUT) {
      leftLongBlock = true;
    }
  } else {
    leftBlockedStartTime = 0;
  }

  // Check right sensor
  if (blockedRight) {
    if (rightBlockedStartTime == 0) {
      rightBlockedStartTime = currentTime;
    } else if (currentTime - rightBlockedStartTime >= SIDE_BLOCK_TIMEOUT) {
      rightLongBlock = true;
    }
  } else {
    rightBlockedStartTime = 0;
  }
}

void handleLongSideBlocks(bool blockedLeft, bool blockedRight) {
  Serial.println("Long side block detected! Taking evasive action...");
  
  // Back up first
  currentState = BACKWARD;
  moveBackward();
  delay(BACKUP_TIME * 2); // Longer backup
  
  // Turn to the unblocked side
  if (leftLongBlock && !blockedRight) {
    currentState = TURN_RIGHT;
    turnRight();
    turnRight(); // Double turn for more significant direction change
  } 
  else if (rightLongBlock && !blockedLeft) {
    currentState = TURN_LEFT;
    turnLeft();
    turnLeft();
  }
}


void moveBackward() {
  stopMotors();
  analogWrite(BIN2, DEFAULT_SPEED);
  analogWrite(AIN2, DEFAULT_SPEED);
  digitalWrite(BIN1, HIGH);
  digitalWrite(AIN1, HIGH);
  stopMotors();
}

void moveForward() {
  stopMotors();
  analogWrite(BIN2, 100);
  analogWrite(AIN2, 100);
  digitalWrite(AIN1, LOW);
  digitalWrite(BIN1, LOW);
  stopMotors();
}

void stopMotors() {
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN2, LOW);
  delay(50);
}

void turnLeft() {
  stopMotors();
  analogWrite(AIN2, DEFAULT_SPEED);
  analogWrite(BIN2, TURN_SPEED);
  digitalWrite(AIN1, HIGH);
  digitalWrite(BIN1, LOW);
  delay(TURN_TIME);
  stopMotors();
}

void turnRight() {
  stopMotors();
  analogWrite(BIN2, DEFAULT_SPEED);
  analogWrite(AIN2, TURN_SPEED);
  digitalWrite(AIN1, LOW);
  digitalWrite(BIN1, HIGH);
  delay(TURN_TIME);
  stopMotors();
}