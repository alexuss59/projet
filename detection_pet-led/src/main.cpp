#include <Arduino.h>
#include <FastLED.h>

// --- CABLAGE ---
const int PIN_CAPTEUR_IR = 4; 
const int PIN_LEDS = 2;       
const int NOMBRE_LEDS = 10;   

// --- OBJETS ---
CRGB leds[NOMBRE_LEDS];
portMUX_TYPE myMux = portMUX_INITIALIZER_UNLOCKED;

// --- VARIABLES PARTAGEES (VOLATILE) ---
volatile bool bouteille_presente = false; 
volatile bool flag_changement = false;    

// --- INTERRUPTION (ISR) ---
// Se déclenche dès que le signal change (Arrivée OU Départ)
void IRAM_ATTR isr_detection() {
  // 1. Lecture directe de l'état (LOW = Obstacle sur ce capteur)
  bool etat_lu = (digitalRead(PIN_CAPTEUR_IR) == LOW);

  // 2. Section Critique : On protège l'écriture des variables
  portENTER_CRITICAL_ISR(&myMux);
  bouteille_presente = etat_lu;
  flag_changement = true; // On signale au loop qu'il faut agir
  portEXIT_CRITICAL_ISR(&myMux);
}

// --- FONCTION LEDS ---
void changerCouleur(CRGB couleur) {
  for(int i = 0; i < NOMBRE_LEDS; i++) {
    leds[i] = couleur;
  }
  FastLED.show();
}

void setup() {
  Serial.begin(115200);
  
  // Init Capteur
  pinMode(PIN_CAPTEUR_IR, INPUT_PULLUP);

  // Init LEDs
  FastLED.addLeds<WS2812B, PIN_LEDS, GRB>(leds, NOMBRE_LEDS);
  FastLED.setBrightness(40);

  // ETAT INITIAL : "Vert fixe: borne en attente" [Source 275]
  changerCouleur(CRGB::Green);
  Serial.println("--- ETAT : ATTENTE (Vert) ---");

  // Init Interruption
  attachInterrupt(digitalPinToInterrupt(PIN_CAPTEUR_IR), isr_detection, CHANGE);
}

void loop() {
  // Variables locales
  bool mise_a_jour_requise = false;
  bool etat_actuel = false;

  // On bloque le processeur pour éviter les problèmes écriture lecture
  portENTER_CRITICAL(&myMux);
  if (flag_changement) {
    mise_a_jour_requise = true;
    etat_actuel = bouteille_presente;
    flag_changement = false; // Reset du flag
  }
  portEXIT_CRITICAL(&myMux);

  if (mise_a_jour_requise) {
    
    if (etat_actuel == true) {
      //Bouteille détectée
      //Affichage : "Bleu fixe: présence d'une bouteille"
      changerCouleur(CRGB::Blue);
      Serial.println(">>> PRESENCE BOUTEILLE (LED Bleu)");

      // code pour code barre et autre à mettre ici pour suite
    } 
    else {
      //Bouteille partie
      //Affichage : "Vert fixe: borne en attente"
      changerCouleur(CRGB::Green);
      Serial.println("<<< BORNE LIBRE (LED Vert)");
      
      // Code futur : scanner.stopReading();
    }
  }
  
  // Petite pause pour la stabilité
  delay(10);
}