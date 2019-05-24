
 
/* On initialise les broches trigger et echo*/
/* capteur de devant */
const byte TRIGGER_PIN_AVANT = A0; // Broche TRIGGER
const byte ECHO_PIN_AVANT = A1;    // Broche ECHO
/* capteur de gauche */
const byte TRIGGER_PIN_GAUCHE = A2; // Broche TRIGGER
const byte ECHO_PIN_GAUCHE = A3;    // Broche ECHO
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
  delay(2000);
  }
 
 /* initialisation */

void setup() {
     /* Initialise le port série */
     while(!Serial);
  Serial.begin(115200);
  
  Serial.println(" debut ");
 
}

 /* lancement */
void loop() {
Serial.print("capteur avant");
calculDistance(TRIGGER_PIN_AVANT, ECHO_PIN_AVANT); 
  Serial.print("capteur gauche");

calculDistance(TRIGGER_PIN_GAUCHE, ECHO_PIN_GAUCHE);
 Serial.print("capteur droite");

calculDistance(TRIGGER_PIN_DROITE, ECHO_PIN_DROITE);
 


}

  
