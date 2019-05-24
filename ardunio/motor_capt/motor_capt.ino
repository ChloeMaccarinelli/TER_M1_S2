#include <Wire.h>
#include <Adafruit_MotorShield.h>

//moteur
// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Select which 'port' M1, M2, M3 or M4. In this case, M1 and M2
Adafruit_DCMotor *myMotor = AFMS.getMotor(1);
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(2);
int tick=0;


// On initialise les broches trigger et echo
// capteur de devant
const byte TRIGGER_PIN_AVANT = A0; // Broche TRIGGER
const byte ECHO_PIN_AVANT = A1;    // Broche ECHO
// capteur de gauche
const byte TRIGGER_PIN_GAUCHE = A2; // Broche TRIGGER
const byte ECHO_PIN_GAUCHE = A3;    // Broche ECHO
// capteur de droite 
const byte TRIGGER_PIN_DROITE = 4; // Broche TRIGGER
const byte ECHO_PIN_DROITE = 7;    // Broche ECHO

// Vitesse du son  en mm/us 
const float VITESSE_SON = 340.0 / 1000;
 
// Constantes pour le timeout 
const unsigned long TIMEOUT = 13000UL; // 13ms = ~4m à 340m/s

float calculDistance(byte TRIGGER_PIN, byte ECHO_PIN){
  //on lance le calcul de distance en envoyant une impulsion de niveau haut de 10µs sur  TRIGGER
  pinMode(TRIGGER_PIN, OUTPUT);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);
 
 
  
  //on mesure le temps entre l'envoi de l'impulsion  et son renvoi si un obstacle existe
  long mesure_envoi_reception = pulseIn(ECHO_PIN, HIGH, TIMEOUT);
   
  //Calcul de la distance en mm
  float distance = mesure_envoi_reception / 2.0 * VITESSE_SON;
   
  Serial.print(F("Distance: "));
  Serial.print(distance);
  Serial.print(F("mm"));
   Serial.println(" ");
 
   
  //Délai pour ne pas spamer l'affichage des résultats
  delay(750);
  
  }

void setup() {
 
  Serial.begin(115200);           // set up Serial library at 9600 bps
   while(!Serial);
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

 
}

void loop() {

  
  uint8_t i;

  myMotor->setSpeed(50);  
  myMotor2->setSpeed(50);
  
  Serial.println("AVANT");
  while(tick<3){
    myMotor->run(FORWARD);
    myMotor2->run(FORWARD);
    tick++;
    Serial.println("CAPTEUR AVANT 1");
    calculDistance(TRIGGER_PIN_AVANT, ECHO_PIN_AVANT);
    Serial.println("CAPTEUR GAUCHE 1");
    calculDistance(TRIGGER_PIN_GAUCHE, ECHO_PIN_GAUCHE);
    Serial.println("CAPTEUR DROITE 1");
    calculDistance(TRIGGER_PIN_DROITE, ECHO_PIN_DROITE);
    
  }
  
  Serial.println("ARRIERE");
  while(tick>0){
    myMotor->run(BACKWARD);
    myMotor2->run(BACKWARD);
    tick--;
   Serial.println("CAPTEUR AVANT 2");
   calculDistance(TRIGGER_PIN_AVANT, ECHO_PIN_AVANT);
   Serial.println("CAPTEUR GAUCHE 2");
   calculDistance(TRIGGER_PIN_GAUCHE, ECHO_PIN_GAUCHE);
   Serial.println("CAPTEUR DROITE 2");
   calculDistance(TRIGGER_PIN_DROITE, ECHO_PIN_DROITE);
   
  }
   
}
