/******************************************************
*                  MACCARINELLI CHLOE
*                  MARION PIERRE
*                  LOTFI YACINE
*                  
*             M1 IFI NICE SOPHIA ANTIPOLIS
******************************************************/

/*******        BLOC MOTEURS     *********/

#include <Adafruit_MotorShield.h>
// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Motor 1 -> gauche / Motor 2 -> droit
Adafruit_DCMotor *motorsG = AFMS.getMotor(1);
Adafruit_DCMotor *motorsD = AFMS.getMotor(2);

const int motorSpeed = 100; // from 0 (off) to 255 (max speed)

// INIT MOTEURS 

void setupMotors() {
// Left wheel
motorsG->setSpeed(motorSpeed);
motorsG->run(FORWARD);
motorsG->run(RELEASE);

// Right wheel
motorsD->setSpeed(motorSpeed);
motorsD->run(FORWARD);
motorsD->run(RELEASE);
}

/*******        BLOC CAPTEURS     *********/

#define trigPinAvant A0 // defini les broches du capteur central Avant
#define echoPinAvant A1 

#define trigPinDroit 4 // defini les broches du capteur DROIT
#define echoPinDroit 7

#define trigPinGauche A2 // defini les broches du capteur GAUCHE
#define echoPinGauche A3 


void setupCapteurs(){

pinMode(trigPinAvant, OUTPUT);// envoie les ultra-sons AVANT CENTRAL  
pinMode(echoPinAvant, INPUT);// recoit l'echo AVANT CENTRAL
pinMode(trigPinDroit, OUTPUT);// envoie les ultra-sons DROIT
pinMode(echoPinDroit, INPUT);// recoit l'echo DROIT
pinMode(trigPinGauche, OUTPUT);// envoie les ultra-sons GAUCHE
pinMode(echoPinGauche, INPUT);// recoit l'echo GAUCHE

}

/*******         SECURITE          *********/

const float safetyDistanceFront = 20;
const float safetyDistanceRight = 40;
const float safetyDistanceLeft = 40;


/*******        SETUP     *********/

void setup() {

Serial.begin(9600); // pour afficher des données à l'écran si besoin
AFMS.begin();  // create with the default frequency 1.6KHz
setupMotors(); 
setupCapteurs();

delay (2000); // POUR PAS QUE LE ROBOT DEMARRE dès la mise sous tension

}


/*******        DETECTION D'OBSTACLES     *********/

void detectObstacles(){

  long duration, distance; 
  digitalWrite(trigPinAvant, LOW); 
  delayMicroseconds(2);
  digitalWrite(trigPinAvant, HIGH); 
  delayMicroseconds(10);
  digitalWrite(trigPinAvant, LOW); 
  duration = pulseIn(echoPinAvant, HIGH); 
  distance = (float)((duration<<4)+duration)/1000.0;// converti la distance en cm 
  
  if (distance < safetyDistanceFront)/* obstacle devant */ 
  { 
    Serial.println (" " ); 
    Serial.println ("********* ATTENTION DEVANT **********" ); 
    Serial.print ("collision possible à : " ); 
    Serial.print ( distance); 
    Serial.print ( " cm"); 
    Serial.println (" correction de la trajectoire en cours"); 
    motorsD->run(RELEASE);
    motorsG->run(RELEASE);
    
    delay(5);
    
    long duration, distance; 
    digitalWrite(trigPinDroit, LOW); 
    delayMicroseconds(2);
    digitalWrite(trigPinDroit, HIGH); 
    delayMicroseconds(10); 
    digitalWrite(trigPinDroit, LOW); 
    duration = pulseIn(echoPinDroit, HIGH); 
    distance = (float)((duration<<4)+duration)/1000.0;// converti la distance en cm 
    
    if (distance < safetyDistanceRight)//obstacle à droite
    { 
      Serial.println (" " ); 
      Serial.println ("********* ATTENTION DROITE *******" ); 
      Serial.print ("collision possible à : " ); 
      Serial.print ( distance); 
      Serial.print ( " cm"); 
       
      delay(500);
      
      long duration, distance; 
      digitalWrite(trigPinGauche, LOW); 
      delayMicroseconds(2); 
      digitalWrite(trigPinGauche, HIGH); 
      delayMicroseconds(10);
      digitalWrite(trigPinGauche, LOW); 
      duration = pulseIn(echoPinGauche, HIGH); 
      distance = (float)((duration<<4)+duration)/1000.0;// converti la distance en cm 
      
      if (distance < safetyDistanceLeft) { 
        Serial.println (" " ); 
        Serial.println ("********* ATTENTION GAUCHE *******" ); 
        Serial.print ("collision possible à : " ); 
        Serial.print ( distance); 
        Serial.print ( " cm"); 
        Serial.println (" faire demi tour"); 
        
        delay(500);
       /*********** bloc pour demi tour  *************/
        motorsD->run(BACKWARD); 
        motorsG->run(BACKWARD);
        motorsG->setSpeed(motorSpeed); 
        motorsD->setSpeed(motorSpeed);
        
        delay(1500);
        
        motorsD->run(FORWARD); 
        motorsG->run (BACKWARD);
        motorsG->setSpeed(motorSpeed); 
        motorsD->setSpeed(motorSpeed);
       
        delay(1500); // AJUSTER DUREE du DELAY SELON TENSION ACCU ET TESTURE DU SOL

        /*********** fin bloc pour demi tour  *************/  
      }
      //sinon on tourne à gauche
      else { 
        motorsD->run(FORWARD); 
        motorsG->run (BACKWARD);
        motorsG->setSpeed(motorSpeed); 
        motorsD->setSpeed(motorSpeed);
        
        delay(800);
        
        motorsD->run(RELEASE);
        motorsG->run(RELEASE);
        
        delay(1500); 
      }
    }  
    // sinon on tourne à droite
    else {
      
      motorsD->run(BACKWARD); 
      motorsG->run (FORWARD);
      motorsG->setSpeed(motorSpeed); 
      motorsD->setSpeed(motorSpeed);
      
      delay(800);
      
      motorsD->run(RELEASE);
      motorsG->run(RELEASE);
      
      delay(1500); 
    }       
  }
  /**** BLOC SI PAS D'OBSTACLES DEVANT *****/ 
  else { 
   
    delay(5);
   // on regarde à droite
    
    long duration, distance; 
    digitalWrite(trigPinDroit, LOW); 
    delayMicroseconds(2);
    digitalWrite(trigPinDroit, HIGH); 
    delayMicroseconds(10);
    digitalWrite(trigPinDroit, LOW); 
    duration = pulseIn(echoPinDroit, HIGH); 
    distance = (float)((duration<<4)+duration)/1000.0;// converti la distance en cm 
    
    if (distance > 13) { 
      motorsG->run(FORWARD); 
      motorsG->setSpeed(motorSpeed);
    }
    
    else {
      float CMG = ((4+(distance-7))/10);
      float Vit = motorSpeed*CMG;
      motorsG->run(FORWARD); 
      motorsG->setSpeed(Vit);
    }

    delay(5);
    
  // on regarde à gauche
  
    digitalWrite(trigPinGauche, LOW); 
    delayMicroseconds(2); 
    digitalWrite(trigPinGauche, HIGH); 
    delayMicroseconds(10);
    digitalWrite(trigPinGauche, LOW); 
    duration = pulseIn(echoPinGauche, HIGH); 
    distance = (duration/2) / 29.1;// converti la distance en cm 
    
    if (distance > 13) { 
      motorsD->run(FORWARD); 
      motorsD->setSpeed(motorSpeed);
    }
    
    else {
      float CMD = ((4+(distance-7))/10);
      float VitCMD = motorSpeed*CMD;
      motorsD->run(FORWARD); 
      motorsD->setSpeed(VitCMD);
    }
  }   
}


void loop() {

  detectObstacles();

} 
