#include <Adafruit_MotorShield.h>

#define FORWARD_ 0
#define BACKWARD_ 2
#define LEFT_ -1
#define RIGHT_ 1


/* Ultrasonic sensors */      
const uint8_t echoPin_right = 7;  // echo signal (receives)
const uint8_t echoPin_front =  A1;
const uint8_t echoPin_left = A3;

const uint8_t trigPin_right = 4;  // trigger signal (sends)
const uint8_t trigPin_front = A0;
const uint8_t trigPin_left = A2;


/* Measurements */
const float safetyDistance = 30; // according with the speed. Expressed in cm
const float robotWidth = 20; // expressed in cm
const float marge = safetyDistance / 3; // margin of movement. It should move between the marging and the safetyDistance

/*
 * Determines where to move
 */
int objectDetected = 0; // the side where the object is detected
boolean searchingObject = false;
int tick = 0;
int randomDir = FORWARD_;
boolean stop_ = false;


/* Movement */
const int motorSpeed = 60; // from 0 (off) to 255 (max speed)

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Motor 4 -> left / Motor 2 -> right
Adafruit_DCMotor *motorLeft = AFMS.getMotor(1);
Adafruit_DCMotor *motorRight = AFMS.getMotor(2);


/*
 * Motors setup and movement
 */ 
void setupMotors() {
  // Left wheel
  motorLeft->setSpeed(motorSpeed);
  motorLeft->run(FORWARD);
  motorLeft->run(RELEASE);
  
  // Right wheel
  motorRight->setSpeed(motorSpeed);
  motorRight->run(FORWARD);
  motorRight->run(RELEASE);
}

void moveForward() {
  motorRight->run(FORWARD);
  motorLeft->run(FORWARD);
}

void moveBackward() {
  motorRight->run(BACKWARD);
  motorLeft->run(BACKWARD);

  
}

void moveLeft() {
  motorLeft->run(RELEASE);
  motorRight->run(FORWARD);

  
}

void moveRight() {
  motorLeft->run(FORWARD);
  motorRight->run(RELEASE);

}

void dontMove() {
  motorLeft->run(RELEASE);
  motorRight->run(RELEASE);
}


/*
 * Calculates the distance with the information obtained from the sensors  
 */
float calculDistance(uint8_t trigPin, uint8_t echoPin){
  uint32_t duration; // duration of the round trip
  float cm;  // distance of the obstacle

  // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(trigPin, OUTPUT);
  digitalWrite(trigPin, LOW);
  delayMicroseconds(3);

  // Start trigger signal

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
 
  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.

  pinMode(echoPin, INPUT);
  duration = pulseIn(echoPin, HIGH);
 
  // convert the time into a distance
  cm = (float)((duration<<4)+duration)/1000.0; // cm = 17 * duration/1000
  return cm;
}


void initValue() {
  objectDetected = 0;
  searchingObject = false;
  tick = 0;
}

/*
 * Determines where to move
 */
int explore(float cm_left, float cm_front, float cm_right) {  
     
 if (tick > 203) {
  initValue();
  stop_ = false;
 }
 else if (tick == 200) {
      randomDir = -objectDetected;
      stop_ = true;
      return randomDir;
  }
  
 if (searchingObject && objectDetected == LEFT_){
    // Turn until robot is parallel to the object
    if (cm_left < safetyDistance){
         searchingObject = false;
    }
    else {
         Serial.println(" Tourne à droite pour retrouver l'object ");
         return RIGHT_;
    }
 }
 else if (searchingObject && objectDetected == RIGHT_){
    // Turn until robot is parallel to the object
    if (cm_right < safetyDistance){
         searchingObject = false;
    }
    else {
        Serial.println(" Tourne à gauche pour retrouver l'object ");
         return LEFT_;
    }
 }

 if (objectDetected == LEFT_) {
      // If there is already an object detected
      if(cm_left > safetyDistance) {
        // If there is an object detected on the left and he has passed it, then he has to turn left
        return LEFT_;
      } 
      if (cm_left < safetyDistance - marge || cm_front < safetyDistance){
        return RIGHT_;
      } 
    return FORWARD_; 
  } 
  else if (objectDetected == RIGHT_) {
      // If there is already an object detected
      if(cm_right > safetyDistance) {
        // If there is an object detected on the left and he has passed it, then he has to turn right
        return RIGHT_;
      } 
      if (cm_right < safetyDistance - marge || cm_front < safetyDistance){
          return LEFT_;
      } 
    return FORWARD_; 
  }
  else {
    // There's no object detected
    if (tick < 4) {
      return randomDir; 
    }
    
    if(cm_left < safetyDistance) {
        // Object detected on the left side
         objectDetected = LEFT_;
     }
     else if(cm_right < safetyDistance) {
        // Object detected on the right side
         objectDetected = RIGHT_;
     }
     else if(cm_front < safetyDistance) {
        // Object detected in front of him
        
        if(cm_left < safetyDistance) {
            // There is an object on the left side 
            objectDetected = LEFT_;
            return RIGHT_;
        }
        else if(cm_right < safetyDistance) {
            // There is an object on the right side
            objectDetected = RIGHT_;
            return LEFT_;
        }
        else {
            // Compare both sides
            if  (cm_right > cm_left) {
                objectDetected = LEFT_;
                searchingObject = true;
                return RIGHT_;
            }
            else {
              Serial.println(" - RIGHT");
              objectDetected = RIGHT_;
              searchingObject = true;
              return LEFT_;
            }  
        }       
     }
     return FORWARD_; 
  }
} 



/*
 * Moves the wheels
 */
void navigate(){
  int resultatExplore;
  
  float cm_front;  // distance of the obstacle
  float cm_left;
  float cm_right;

  
  
  cm_front = calculDistance(trigPin_front, echoPin_front);
  cm_left = calculDistance(trigPin_left, echoPin_left);
  cm_right = calculDistance(trigPin_right, echoPin_right);

  resultatExplore = explore(cm_left,cm_front, cm_right);
 

  tick++;

  if (stop_){
    dontMove();
    return;
  }
  
  
  
  motorRight->run(RELEASE);
  motorLeft->run(RELEASE);

  switch(resultatExplore) {
    // move forward  
    case FORWARD_:  
      Serial.print("avant  ");
      moveForward();
      break;

    // move backward
    case BACKWARD_:
      Serial.print("arriere  ");
      moveBackward();
      break;

    // move left
    case LEFT_: 
      Serial.print("gauche  ");
      moveLeft();
      break;

    // move right
    case RIGHT_:
      Serial.print("droite  ");
      moveRight();
      break;
  }
}


/*
 * Initial setup
 */
void setup() {
  // initialize serial communication:
  Serial.begin(9600);
  AFMS.begin();  // create with the default frequency 1.6KHz
  
  setupMotors();
  //setupLeds();
}


/*
 * It's the function that will be called at each tick time
 */
void loop()
{
  navigate();
}
