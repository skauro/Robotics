#include <Wire.h>
#include <Adafruit_VL53L1X.h>

// Sensor instances
Adafruit_VL53L1X sensorLeft = Adafruit_VL53L1X();
Adafruit_VL53L1X sensorMiddle = Adafruit_VL53L1X();
Adafruit_VL53L1X sensorRight = Adafruit_VL53L1X();

// XSHUT pins for each sensor
#define XSHUT_LEFT   21
#define XSHUT_MIDDLE 22
#define XSHUT_RIGHT  23

// Custom I2C addresses
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
#define MAX_SPEED 100
#define OBSTACLE_THRESHOLD 150  // mm
#define BACKUP_TIME 450         // ms
#define SAFETY_DELAY 10         // ms
#define SIDE_BLOCK_TIMEOUT 6000 // 6 seconds

// PID parameters
float Kp = 0.85;    // Proportional gain
float Ki = 0.01;   // Integral gain
float Kd = 0.03;   // Derivative gain
float I_max = 100; // Integral windup limit

// PID variables
float previousError = 0;
float integral = 0;
unsigned long lastTime = 0;

// State variables
bool leftLongBlock = false;
bool rightLongBlock = false;
unsigned long leftBlockedStartTime = 0;
unsigned long rightBlockedStartTime = 0;

void setup() {
  Serial.begin(115200);

  // Initialize motor control
  pinMode(MODE, OUTPUT);
  digitalWrite(MODE, HIGH);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  stopMotors();

  // Initialize I2C
  Wire.begin(6, 7, 400000);  // SDA, SCL

  // Initialize sensors
  initializeSensors();

  // Start ranging
  sensorLeft.startRanging();
  sensorMiddle.startRanging();
  sensorRight.startRanging();

  // Configure sensors
  configureSensor(sensorLeft);
  configureSensor(sensorMiddle);
  configureSensor(sensorRight);

  lastTime = millis();
  delay(4900);
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
    Serial.println("Left sensor fail");
    while (1);
  }
  sensorLeft.VL53L1X_SetI2CAddress(ADDR_LEFT);
  delay(10);

  // Initialize middle sensor
  digitalWrite(XSHUT_MIDDLE, HIGH);
  delay(10);
  if (!sensorMiddle.begin(0x29, &Wire)) {
    Serial.println("Middle sensor fail");
    while (1);
  }
  sensorMiddle.VL53L1X_SetI2CAddress(ADDR_MIDDLE);
  delay(10);

  // Initialize right sensor
  digitalWrite(XSHUT_RIGHT, HIGH);
  delay(10);
  if (!sensorRight.begin(0x29, &Wire)) {
    Serial.println("Right sensor fail");
    while (1);
  }
  sensorRight.VL53L1X_SetI2CAddress(ADDR_RIGHT);
  delay(10);
}

void configureSensor(Adafruit_VL53L1X &sensor) {
  sensor.VL53L1X_SetDistanceMode(1);  // Short distance mode
  sensor.setTimingBudget(50);         // Increased timing budget for better reliability
}

void loop() {
  // Read sensors
  int distLeft = readFilteredSensor(sensorLeft);
  int distMiddle = readFilteredSensor(sensorMiddle);
  int distRight = readFilteredSensor(sensorRight);

  Serial.print("L: "); Serial.print(distLeft);
  Serial.print(" M: "); Serial.print(distMiddle);
  Serial.print(" R: "); Serial.println(distRight);

  // Determine if obstacles are present
  bool blockedLeft = (distLeft >= 0 && distLeft < OBSTACLE_THRESHOLD);
  bool blockedMiddle = (distMiddle >= 0 && distMiddle < OBSTACLE_THRESHOLD);
  bool blockedRight = (distRight >= 0 && distRight < OBSTACLE_THRESHOLD);

  // Check for long-term side blocks
  checkLongSideBlocks(blockedLeft, blockedRight);

  // Handle long side blocks if detected
  if (leftLongBlock || rightLongBlock) {
    handleLongSideBlocks(blockedLeft, blockedRight);
    leftLongBlock = false;
    rightLongBlock = false;
    leftBlockedStartTime = 0;
    rightBlockedStartTime = 0;
    return;
  }

  // Handle middle obstacle
  if (blockedMiddle) {
    moveBackward();
    delay(BACKUP_TIME);
    
    // Decide turn direction based on which side has more space
    if (distLeft > distRight) {
      turnLeft();
    } else {
      turnRight();
    }
  } 
  else {
    // Calculate time since last update for PID
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - lastTime) / 1000.0; // Convert to seconds
    lastTime = currentTime;
    
    // Calculate error (difference between left and right distances)
    float error = distLeft - distRight;
    
    // PID calculations with anti-windup
    integral += error * deltaTime;
    integral = constrain(integral, -I_max, I_max); // Anti-windup
    float derivative = (error - previousError) / deltaTime;
    
    // Calculate PID output
    float output = Kp * error + Ki * integral + Kd * derivative;
    previousError = error;
    
    // Apply output to motor speeds
    int leftSpeed = constrain(DEFAULT_SPEED - output, 0, MAX_SPEED);
    int rightSpeed = constrain(DEFAULT_SPEED + output, 0, MAX_SPEED);
    
    // Move with calculated speeds
    moveWithSpeed(leftSpeed, rightSpeed);
  }

  delay(SAFETY_DELAY);
}

int readFilteredSensor(Adafruit_VL53L1X &sensor) {
  static int lastValues[3] = {0, 0, 0};
  static byte index = 0;
  
  if (sensor.dataReady()) {
    int newValue = sensor.distance();
    sensor.clearInterrupt();
    
    // Simple moving average filter
    lastValues[index] = newValue;
    index = (index + 1) % 3;
    
    return (lastValues[0] + lastValues[1] + lastValues[2]) / 3;
  }
  return -1;
}

void checkLongSideBlocks(bool blockedLeft, bool blockedRight) {
  unsigned long now = millis();

  if (blockedLeft) {
    if (leftBlockedStartTime == 0) {
      leftBlockedStartTime = now;
    } else if (now - leftBlockedStartTime >= SIDE_BLOCK_TIMEOUT) {
      leftLongBlock = true;
    }
  } else {
    leftBlockedStartTime = 0;
  }

  if (blockedRight) {
    if (rightBlockedStartTime == 0) {
      rightBlockedStartTime = now;
    } else if (now - rightBlockedStartTime >= SIDE_BLOCK_TIMEOUT) {
      rightLongBlock = true;
    }
  } else {
    rightBlockedStartTime = 0;
  }
}

void handleLongSideBlocks(bool blockedLeft, bool blockedRight) {
  Serial.println("Long side block detected! Taking evasive action...");
  moveBackward();
  delay(BACKUP_TIME * 2);

  if (leftLongBlock && !blockedRight) {
    // If left is blocked long-term and right is clear, turn right twice
    turnRight(); 
    delay(200);
    turnRight();
  } else if (rightLongBlock && !blockedLeft) {
    // If right is blocked long-term and left is clear, turn left twice
    turnLeft();
    delay(200);
    turnLeft();
  }
}

void moveBackward() {
  stopMotors();
  analogWrite(BIN2, DEFAULT_SPEED);
  analogWrite(AIN2, DEFAULT_SPEED);
  digitalWrite(BIN1, HIGH);
  digitalWrite(AIN1, HIGH);
  delay(BACKUP_TIME);
  stopMotors();
}

void moveWithSpeed(int leftSpeed, int rightSpeed) {
  // Left motor control
  digitalWrite(AIN1, LOW);
  analogWrite(AIN2, leftSpeed);
  
  // Right motor control
  digitalWrite(BIN1, LOW);
  analogWrite(BIN2, rightSpeed);
}

void stopMotors() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}

void turnLeft() {
  stopMotors();
  analogWrite(AIN2, DEFAULT_SPEED);
  analogWrite(BIN2, DEFAULT_SPEED);
  digitalWrite(AIN1, HIGH);
  digitalWrite(BIN1, LOW);
  delay(300);
  stopMotors();
}

void turnRight() {
  stopMotors();
  analogWrite(AIN2, DEFAULT_SPEED);
  analogWrite(BIN2, DEFAULT_SPEED);
  digitalWrite(AIN1, LOW);
  digitalWrite(BIN1, HIGH);
  delay(300);
  stopMotors();
}