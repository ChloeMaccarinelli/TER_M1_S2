/* 
 * Code de calcul de la distance entre le capteur et l'obstacle
 */

/* On initialise les broches trigger et echo*/
const byte TRIGGER_PIN = 2; // Broche TRIGGER
const byte ECHO_PIN = 3;    // Broche ECHO

/* Vitesse du son  en mm/us */
const float VITESSE_SON = 340.0 / 1000;
 
/* Constantes pour le timeout */
const unsigned long TIMEOUT = 13000UL; // 13ms = ~4m à 340m/s


float calculDistance(byte TRIGGER_PIN, byte ECHO_PIN){
    
  /* on lance le calcul de distance en envoyant une impulsion de niveau haut de 10µs sur  TRIGGER */
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);
  
  /*  on mesure le temps entre l'envoi de l'impulsion  et son renvoi si un obstacle existe */
  long mesure_envoi_reception = pulseIn(ECHO_PIN, HIGH, TIMEOUT);
   
  /* Calcul de la distance en mm*/
  float distance = mesure_envoi_reception / 2.0 * VITESSE_SON;
   
  /* Affiche les résultats en mm*/
  Serial.print(F("Distance: "));
  Serial.print(distance);
  Serial.println(F("mm "));
 
   
  /* Délai pour ne pas spamer l'affichage des résultats */
  delay(2000);
}

/* initialisation */

void setup() {
   
  /* Initialise le port série */
  Serial.begin(115200);
   
  /* Initialise les broches trigger et echo */
  pinMode(TRIGGER_PIN, OUTPUT);
  digitalWrite(TRIGGER_PIN, LOW);
  pinMode(ECHO_PIN, INPUT);
}

 /* lancement */
void loop() {

calculDistance(TRIGGER_PIN, ECHO_PIN);

}
