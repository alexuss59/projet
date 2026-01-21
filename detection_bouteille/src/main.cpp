#include <Arduino.h>

// --- Configuration ---
const int PIN_CAPTEUR_IR = 4;  // la broche OUT 
const int DELAI_ANTI_REBOND = 500; // ms pour éviter les doubles détections

// --- Variables Volatiles (Modifiées par l'interruption) ---
volatile bool bouteilleDetectee = false;
volatile unsigned long dernierTempsDetection = 0;

// C'est la "clé" qui permet de verrouiller la mémoire
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// --- Interruption (ISR) ---
// IRAM_ATTR pour que le code soit en RAM (rapide)
void IRAM_ATTR isr_detection_IR() {
  unsigned long tempsActuel = millis();
  
  // Anti-rebond logiciel simple
  if (tempsActuel - dernierTempsDetection > DELAI_ANTI_REBOND) {
    bouteilleDetectee = true;
    dernierTempsDetection = tempsActuel;
  }
}

void setup() {
  Serial.begin(115200);
  
  // INPUT_PULLUP
  pinMode(PIN_CAPTEUR_IR, INPUT_PULLUP);

  // Attacher l'interruption
  // FALLING = Le signal passe de HIGH (3.3V) à LOW (0V) quand un obstacle est vu
  attachInterrupt(digitalPinToInterrupt(PIN_CAPTEUR_IR), isr_detection_IR, FALLING);

  Serial.println("Système prêt. En attente de bouteille...");
}