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

int counter = 0;
int straightCounter = 0;

void setup() {

  Serial.begin(115200);

  pinMode(MODE, OUTPUT);
  digitalWrite(MODE, HIGH);
  pinMode(AIN1, OUTPUT);
  pinMode(BIN1, OUTPUT);

  //   sda 6, scl 7
  Wire.begin(6, 7, 400'000);

  pinMode(XSHUT_LEFT, OUTPUT);
  pinMode(XSHUT_MIDDLE, OUTPUT);
  pinMode(XSHUT_RIGHT, OUTPUT);

  digitalWrite(XSHUT_LEFT, LOW);
  digitalWrite(XSHUT_MIDDLE, LOW);
  digitalWrite(XSHUT_RIGHT, LOW);
  delay(10);

  digitalWrite(XSHUT_LEFT, HIGH);
  delay(10);
  if (!sensorLeft.begin(0x29, &Wire)) {
    while (1);
  }
  sensorLeft.VL53L1X_SetI2CAddress(ADDR_LEFT);

  digitalWrite(XSHUT_MIDDLE, HIGH);
  delay(10);
  if (!sensorMiddle.begin(0x29, &Wire)) {
    while (1);
  }
  sensorMiddle.VL53L1X_SetI2CAddress(ADDR_MIDDLE);

  digitalWrite(XSHUT_RIGHT, HIGH);
  delay(10);
  if (!sensorRight.begin(0x29, &Wire)) {
    while (1);
  }
  sensorRight.VL53L1X_SetI2CAddress(ADDR_RIGHT);

  // Start ranging on all sensors
  sensorLeft.startRanging();
  sensorMiddle.startRanging();
  sensorRight.startRanging();

  sensorLeft.VL53L1X_SetDistanceMode(1);
  sensorLeft.setTimingBudget(15);

  sensorMiddle.VL53L1X_SetDistanceMode(1);
  sensorMiddle.setTimingBudget(15);

  sensorRight.VL53L1X_SetDistanceMode(1);
  sensorRight.setTimingBudget(15);

}


void loop() {
  Serial.print(straightCounter);
  analogWrite(BIN2, 80);
  analogWrite(AIN2, 80);

  
  int distLeft = -1, distMiddle = -1, distRight = -1;

  if (sensorLeft.dataReady()) {
    distLeft = sensorLeft.distance();
  }

  if (sensorMiddle.dataReady()) {
    distMiddle = sensorMiddle.distance();
  }

  if (sensorRight.dataReady()) {
    distRight = sensorRight.distance();
  }

  Serial.print("L: "); Serial.print(distLeft);
  Serial.print(" M: "); Serial.print(distMiddle);
  Serial.print(" R: "); Serial.println(distRight);


  bool blockedLeft = true;
  bool blockedMiddle = true;
  bool blockedRight = true;

  int threshold = 150; // mm (15cm)
  if(distLeft >= 0 && distLeft < threshold)
  {
    blockedLeft = true;
  }
  else
  {
    blockedLeft = false;
  }

  if(distMiddle >= 0 && distMiddle < threshold)
  {
    blockedMiddle = true;
  }
  else
  {
    blockedMiddle = false;
  }
  
  if(distRight >= 0 && distRight < threshold)
  {
    blockedRight = true;
  }
  else
  {
    blockedRight = false;
  }

  if (blockedMiddle == true) 
  {
    if (blockedLeft != true) 
    {
      straightCounter = 0;
      moveBackward();
      delay(200);
      turnLeft();
      counter = 0;
    } 
    else if (blockedRight != true) 
    {
      straightCounter = 0;
      moveBackward();
      delay(200);
      turnRight();
      counter = 0;
    } 
    else 
    {
      straightCounter = 0;
      counter++;
      moveBackward();
      delay(200);
      if(counter >= 2)
      {
          if(counter >= 4)
          {
            turnRight();
            counter = 0;
          }
          else
          {
            turnLeft();
          }
      }
    }
  } 
  else 
  {
    moveForward();
    straightCounter++;
    if(straightCounter >= 100)
    {
      straightCounter = 0;
      delay(10);
      if (sensorLeft.dataReady()) 
      {
          distLeft = sensorLeft.distance();
      }
      if(distLeft >= 0 && distLeft < threshold)
      {
        blockedLeft = true;
      }
      else
      {
        blockedLeft = false;
      }
      if (sensorRight.dataReady()) 
      {
        distRight = sensorRight.distance();
      }
        if(distRight >= 0 && distRight < threshold)
      {
        blockedRight = true;
      }
      else
      {
        blockedRight = false;
      }
      if(blockedLeft == false)
      {
        moveBackward();
        delay(200);
        turnLeft();
        turnLeft();
      }
      else if(blockedRight == false)
      {
        moveBackward();
        delay(200);
        turnRight();
        turnRight();
      }
      else 
      {
        moveBackward();
        delay(150);
      }

    }
  }

  delay(10);
}

void moveBackward() {
  stopMotors();
  analogWrite(BIN2, 80);
  analogWrite(AIN2, 80);
  digitalWrite(BIN1, HIGH);
  digitalWrite(AIN1,HIGH);
}

void moveForward() {
  stopMotors();
  analogWrite(BIN2, 80);
  analogWrite(AIN2, 80);
  digitalWrite(AIN1, LOW);
  digitalWrite(BIN1, LOW);
}

void stopMotors() {
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN2, LOW);
  delay(100);
}

void turnLeft() {
  delay(20);
  analogWrite(AIN2, 90);
  analogWrite(BIN2, 60);
  digitalWrite(AIN1, HIGH);
  digitalWrite(BIN1, LOW);
  delay(400);
}

void turnRight() {
  delay(20);
  analogWrite(BIN2, 90);
  analogWrite(AIN2, 60);
  digitalWrite(AIN1, LOW);
  digitalWrite(BIN1, HIGH);
  delay(400);
}


