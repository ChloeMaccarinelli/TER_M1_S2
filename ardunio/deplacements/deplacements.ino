
/* 
 * Code permettant les déplacements du robot en évitant les obstacles
 */
#include <Wire.h>
#include <Adafruit_MotorShield.h>


#define FORWARD_ 0
#define BACKWARD_ 2
#define LEFT_ -1
#define RIGHT_ 1


/* On initialise les broches trigger et echo*/
/* capteur de devant */
const byte TRIGGER_PIN_AVANT = 2; // Broche TRIGGER
const byte ECHO_PIN_AVANT = 3;    // Broche ECHO
/* capteur de gauche */
const byte TRIGGER_PIN_GAUCHE = 4; // Broche TRIGGER
const byte ECHO_PIN_GAUCHE = 5;    // Broche ECHO
/* capteur de droite */
const byte TRIGGER_PIN_DROITE = 6; // Broche TRIGGER
const byte ECHO_PIN_DROITE = 7;    // Broche ECHO
/* fin d'initialisation des broches pour les capteurs */

/* Vitesse du son  en mm/us */
const float VITESSE_SON = 340.0 / 1000;
 
/* Constantes pour le timeout */
const unsigned long TIMEOUT = 13000UL; // 13ms = ~4m à 340m/s

/* Sécurité */
const float distanceSecurite = 20; //distance de sécurité pour que le robot puisse tourner sans toucher l'obstacle
const float taillerobot = 19;  //taille du robot
const float marge = distanceSecurite / 3; 


/* on initialise les moteurs gauche et droit */
volatile int currentState = FORWARD_; 
const int motorSpeed = 200; // de 0 (off) a 255 (max)

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Motor 4 -> left / Motor 2 -> right
Adafruit_DCMotor *moteurGauche = AFMS.getMotor(1);
Adafruit_DCMotor *moteurDroit = AFMS.getMotor(2);

void setupMoteurs(){
  
  Serial.println("init motor");
  moteurGauche->setSpeed(motorSpeed);
  moteurGauche->run(FORWARD);
  moteurGauche->run(RELEASE);
  Serial.println("OK");

  moteurDroit->setSpeed(motorSpeed + 10);
  moteurDroit->run(FORWARD);
  moteurDroit->run(RELEASE);
  Serial.println("OK");
}
/* fin initialisation des moteurs */

/* on défini les actions des moteurs en fonction de la direction du robot */
void avancer() {
  moteurGauche->run(FORWARD);
  moteurDroit->run(FORWARD);
}

void reculer() {
  moteurGauche->run(BACKWARD);
  moteurDroit->run(BACKWARD);
}

void tournerAGauche() {
  moteurGauche->run(BACKWARD);
  moteurDroit->run(FORWARD);
}

void tournerADroite() {
  moteurGauche->run(FORWARD);
  moteurDroit->run(BACKWARD);
}
void arreter() {
  moteurGauche->run(RELEASE);
  moteurDroit->run(RELEASE);
}
/* fin de définition des actions moteurs */


/* fonction de calcul de distance entre le robot et un éventuel obstacle */
float calculDistance(byte TRIGGER_PIN, byte ECHO_PIN){
    
  /* on lance le calcul de distance en envoyant une impulsion de niveau haut de 10µs sur  TRIGGER */
  pinMode(TRIGGER_PIN, OUTPUT);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);
  
  /*  on mesure le temps entre l'envoi de l'impulsion  et son renvoi si un obstacle existe */
  long mesure_envoi_reception = pulseIn(ECHO_PIN, HIGH, TIMEOUT);
   
  /* Calcul de la distance en mm*/
  float distance = mesure_envoi_reception / 2.0 * VITESSE_SON;
   
  Serial.print(F("Distance: "));
  Serial.print(distance);
  Serial.print(F("mm"));
   Serial.println(" ");
 
   
  /* Délai pour ne pas spamer l'affichage des résultats */
  delay(2000);
  }
/* fin fonction caculDistance */
  

/*
 * Determines where to move
 */
int objectDetected = 0; 
boolean searchingObject = false;
int tick = 0;
int randomDir = FORWARD_;
boolean stop_ = false;



void initValue() {
  objectDetected = 0;
  searchingObject = false;
  tick = 0;
}
/*
 * Determines where to move to avoid obstacles
 */
int explorer(float cm_front, float cm_left, float cm_right) {


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
    if (cm_left < distanceSecurite){
         searchingObject = false;
    }
    else {
        // Serial.println(" Tourne à droite pour retrouver l'object ");
         return RIGHT_;
    }
}
else if (searchingObject && objectDetected == RIGHT_){
    // Turn until robot is parallel to the object
    if (cm_right < distanceSecurite){
         searchingObject = false;
    }
    else {
        // Serial.println(" Tourne à gauche pour retrouver l'object ");
         return LEFT_;
    }
 }

 if (objectDetected == LEFT_) {
      // If there is already an object detected
      if(cm_left > distanceSecurite) {
        // If there is an object detected on the left and he has passed it, then he has to turn left
        return LEFT_;
      } 
      if (cm_left < distanceSecurite - marge || cm_front < distanceSecurite){
        return RIGHT_;
      } 
    return FORWARD_; 
  } 
  else if (objectDetected == RIGHT_) {
      // If there is already an object detected
      if(cm_right > distanceSecurite) {
        // If there is an object detected on the left and he has passed it, then he has to turn right
        return RIGHT_;
      } 
      if ( cm_right < distanceSecurite - marge || cm_front < distanceSecurite){
          return LEFT_;
      } 
    return FORWARD_; 
  }
  else {
    // There's no object detected
    if (tick < 4) {
      return randomDir; 
    }
    
    if(cm_left < distanceSecurite) {
        // Object detected on the left side
         objectDetected = LEFT_;
     }
     else if(cm_right < distanceSecurite) {
        // Object detected on the right side
         objectDetected = RIGHT_;
     }
     else if(cm_front < distanceSecurite) {
        // Object detected in front of him
        
        if(cm_left < distanceSecurite) {
            // There is an object on the left side 
            objectDetected = LEFT_;
            return RIGHT_;
        }
        else if(cm_right < distanceSecurite) {
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
              //Serial.println(" - RIGHT");
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
 * Moves the robot
 */
void navigate()
{
  int resultatexplorer;

  // distances from all sides
  float cm_front;
  float cm_right;
  float cm_left;
   
  
  noInterrupts();
  cm_front = calculDistance(TRIGGER_PIN_AVANT, ECHO_PIN_AVANT);
  cm_left = calculDistance(TRIGGER_PIN_GAUCHE, ECHO_PIN_GAUCHE);
  cm_right = calculDistance(TRIGGER_PIN_DROITE, ECHO_PIN_DROITE);
  
  resultatexplorer=explorer(cm_front, cm_right, cm_left);  
 
  interrupts();
  tick++;
   if (stop_){
    arreter();
    return;
    
}
moteurDroit->run(RELEASE);
  moteurGauche->run(RELEASE);

  switch(resultatexplorer) {
    // move forward  
    case FORWARD_:  
      //Serial.print("avant  ");
      avancer();
      break;

    // move backward
    case BACKWARD_:
      //Serial.print("arriere  ");
      reculer();
      break;

    // move left
    case LEFT_: 
      //Serial.print("gauche  ");
      tournerAGauche();
      break;

    // move right
    case RIGHT_:
      //Serial.print("droite  ");
      tournerADroite();
      break;
  }
}

 
/*
 * Initial setup
 */
void setup() {
  // initialize serial communication
  Serial.begin(115200);
 

  Serial.println("init capteur");
  /* Initialise les broches trigger et echo */
  pinMode(TRIGGER_PIN_AVANT, OUTPUT);
  digitalWrite(TRIGGER_PIN_AVANT, LOW);
  pinMode(ECHO_PIN_AVANT, INPUT);
   pinMode(TRIGGER_PIN_GAUCHE, OUTPUT);
  digitalWrite(TRIGGER_PIN_GAUCHE, LOW);
  pinMode(ECHO_PIN_GAUCHE, INPUT);
   pinMode(TRIGGER_PIN_DROITE, OUTPUT);
  digitalWrite(TRIGGER_PIN_DROITE, LOW);
  pinMode(ECHO_PIN_DROITE, INPUT);
  Serial.println("OK");
   Serial.println("Adafruit Motorshield v2 - DC Motor test!");
  // initialise le moteur
  AFMS.begin();
  setupMoteurs();

  

}


/*
 * It's the function that will be called at each tick time. It executes navigate and sends the message
 */
void loop()
{
  calculDistance(TRIGGER_PIN_AVANT, ECHO_PIN_AVANT); 
  
calculDistance(TRIGGER_PIN_GAUCHE, ECHO_PIN_GAUCHE);
 
calculDistance(TRIGGER_PIN_DROITE, ECHO_PIN_DROITE);

}
  
