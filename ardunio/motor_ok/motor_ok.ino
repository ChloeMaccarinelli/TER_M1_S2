#include <Wire.h>
#include <Adafruit_MotorShield.h>

//moteur
// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *myMotor = AFMS.getMotor(1);
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(2);
// You can also make another motor on port M2
//Adafruit_DCMotor *myOtherMotor = AFMS.getMotor(2);

int tick=0;


/* On initialise les broches trigger et echo*/
/* capteur de devant */
const byte TRIGGER_PIN_AVANT = A2; // Broche TRIGGER
const byte ECHO_PIN_AVANT = A1;    // Broche ECHO
/* capteur de gauche */
const byte TRIGGER_PIN_GAUCHE = 2; // Broche TRIGGER
const byte ECHO_PIN_GAUCHE = 8;    // Broche ECHO
/* capteur de droite */
const byte TRIGGER_PIN_DROITE = 4; // Broche TRIGGER
const byte ECHO_PIN_DROITE = 7;    // Broche ECHO
/* Vitesse du son  en mm/us */
const float VITESSE_SON = 340.0 / 1000;
 
/* Constantes pour le timeout */
const unsigned long TIMEOUT = 13000UL; // 13ms = ~4m à 340m/s

  float calculDistance(byte TRIGGER_PIN, byte ECHO_PIN){
  //on lance le calcul de distance en envoyant une impulsion de niveau haut de 10µs sur  TRIGGER
  pinMode(TRIGGER_PIN, OUTPUT);
  analogWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  analogWrite(TRIGGER_PIN, LOW);
 
 
  
  //on mesure le temps entre l'envoi de l'impulsion  et son renvoi si un obstacle existe
  long mesure_envoi_reception = pulseIn(ECHO_PIN, HIGH, TIMEOUT);
   
  //Calcul de la distance en mm
  float distance = mesure_envoi_reception / 2.0 * VITESSE_SON;
   
  Serial.print(F("Distance: "));
  Serial.print(distance);
  Serial.print(F("mm"));
   Serial.println(" ");
 
   
  //Délai pour ne pas spamer l'affichage des résultats
  delay(2000);
  
  }

void setup() {
  while(!Serial);
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Adafruit Motorshield v2 - DC Motor test!");

  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz
  
  // Set the speed to start, from 0 (off) to 255 (max speed)
  myMotor->setSpeed(150);
  myMotor->run(FORWARD);
  myMotor2->setSpeed(150);
  myMotor2->run(FORWARD);
  // turn on motor
  myMotor->run(RELEASE);
  myMotor2->run(RELEASE);

 pinMode(TRIGGER_PIN_AVANT, OUTPUT);
 analogWrite(TRIGGER_PIN_AVANT, LOW);
 pinMode(ECHO_PIN_AVANT, INPUT);
 
 pinMode(TRIGGER_PIN_GAUCHE, OUTPUT);
 digitalWrite(TRIGGER_PIN_GAUCHE, LOW);
 pinMode(ECHO_PIN_GAUCHE, INPUT);
  
 pinMode(TRIGGER_PIN_DROITE, OUTPUT);
 digitalWrite(TRIGGER_PIN_DROITE, LOW);
 pinMode(ECHO_PIN_DROITE, INPUT);
}

void loop() {

  
  uint8_t i;

  myMotor->setSpeed(50);  
  myMotor2->setSpeed(50);
  
  Serial.println("AVANT");
  while(tick<50){
    myMotor->run(FORWARD);
    myMotor2->run(FORWARD);
    tick++;
    Serial.println("CAPTEUR AVANT 1");
    calculDistance(TRIGGER_PIN_AVANT, ECHO_PIN_AVANT);
   

  }
  
  Serial.println("ARRIERE");
  while(tick>0){
    myMotor->run(BACKWARD);
    myMotor2->run(BACKWARD);
    tick--;
   Serial.println("CAPTEUR AVANT 2");
   calculDistance(TRIGGER_PIN_AVANT, ECHO_PIN_AVANT);
   
  }
   
}
