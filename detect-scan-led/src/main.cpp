#include <Arduino.h>
#include <FastLED.h>

// --- 1. CONFIGURATION ---
const int PIN_IR_ENTREE = 6;
const int PIN_LEDS = 7;
const int NUM_LEDS = 10;
const int PIN_RX_SCANNER = 18; 
const int PIN_TX_SCANNER = 17; 

// --- 2. GESTION CRITIQUE (Ce que tes profs attendent) ---
// "volatile" est OBLIGATOIRE car cette variable est modifiée par l'interruption
volatile bool flag_demande_scan = false;

// Le "Verrou" (Spinlock) pour protéger l'accès à la variable
portMUX_TYPE monVerrou = portMUX_INITIALIZER_UNLOCKED;

// --- 3. COMMANDES WAVESHARE V2.1 (Page 25) ---
const byte CMD_START_SCAN[] = {0x7E, 0x00, 0x08, 0x01, 0x00, 0x02, 0x01, 0xAB, 0xCD};
const byte CMD_STOP_SCAN[]  = {0x7E, 0x00, 0x08, 0x01, 0x00, 0x02, 0x00, 0xAB, 0xCD};

// --- 4. OBJETS & VARIABLES ---
CRGB leds[NUM_LEDS];
HardwareSerial ScannerSerial(1); 

enum Etat { ATTENTE, SCAN_EN_COURS, VALIDATION, ERREUR, ATTENTE_RETRAIT };
Etat etatActuel = ATTENTE;
unsigned long debutScan = 0;
const long TIMEOUT_SCAN = 3000; 

// --- 5. INTERRUPTION (ISR) ---
// Cette fonction s'exécute en quelques microsecondes dès que le capteur change
void IRAM_ATTR isr_detection() {
  // DÉBUT ZONE CRITIQUE (ISR)
  // On bloque tout le reste pour écrire dans la variable en sécurité
  portENTER_CRITICAL_ISR(&monVerrou);
  
  flag_demande_scan = true;
  
  portEXIT_CRITICAL_ISR(&monVerrou);
  // FIN ZONE CRITIQUE
}

// --- UTILITAIRES ---
void envoyerCommande(const byte* cmd, int taille) {
  ScannerSerial.write(cmd, taille);
}

void allumerLeds(CRGB c) { for(int i=0; i<NUM_LEDS; i++) leds[i] = c; FastLED.show(); }
void clignoter(CRGB c) { 
  for(int i=0; i<3; i++) { allumerLeds(CRGB::Black); delay(100); allumerLeds(c); delay(100); } 
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("--- SYSTEME PRO (Critical & Hex) ---");

  // Init Scanner 9600 (Waveshare V2.1)
  ScannerSerial.begin(9600, SERIAL_8N1, PIN_RX_SCANNER, PIN_TX_SCANNER);

  // Init Capteur & Interruption
  pinMode(PIN_IR_ENTREE, INPUT_PULLUP);
  // On attache l'interruption sur le front descendant (FALLING) = Passage de 3.3V à 0V
  attachInterrupt(digitalPinToInterrupt(PIN_IR_ENTREE), isr_detection, FALLING);

  // Init LEDs
  FastLED.addLeds<WS2812B, PIN_LEDS, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(40);
  
  // Sécurité démarrage
  envoyerCommande(CMD_STOP_SCAN, sizeof(CMD_STOP_SCAN));
  allumerLeds(CRGB::Green);
}

void loop() {
  // Lecture sécurisée du Flag
  bool demandeScan = false;

  // DÉBUT ZONE CRITIQUE (LOOP)
  // On verrouille juste le temps de lire et remettre à zéro le flag
  portENTER_CRITICAL(&monVerrou);
  if (flag_demande_scan) {
    demandeScan = true;
    if (etatActuel == ATTENTE) {
        flag_demande_scan = false; // On consomme l'info
    }
  }
  portEXIT_CRITICAL(&monVerrou);
  // FIN ZONE CRITIQUE

  
  switch (etatActuel) {

    case ATTENTE:
      if (demandeScan) {
        Serial.println(">>> INTERRUPT : Demande de scan reçue !");
        
        while(ScannerSerial.available()) ScannerSerial.read(); 
        envoyerCommande(CMD_START_SCAN, sizeof(CMD_START_SCAN));
        
        etatActuel = SCAN_EN_COURS;
        debutScan = millis();
        allumerLeds(CRGB::Blue);
      }
      break;

    case SCAN_EN_COURS:
      // Lecture du code
      if (ScannerSerial.available()) {
        String data = ScannerSerial.readStringUntil('\r');
        data.trim();
        // Filtre les réponses courtes (acquittement commande)
        if (data.length() > 7) { 
           Serial.print(">>> CODE LU : "); Serial.println(data);
           envoyerCommande(CMD_STOP_SCAN, sizeof(CMD_STOP_SCAN)); 
           etatActuel = VALIDATION;
        }
      }
      
      // Timeout
      if (millis() - debutScan > TIMEOUT_SCAN) {
        Serial.println("!!! TIMEOUT");
        envoyerCommande(CMD_STOP_SCAN, sizeof(CMD_STOP_SCAN));
        etatActuel = ERREUR;
      }
      
      // Si la bouteille est retirée (Lecture capteur physique pour le retrait)
      if (digitalRead(PIN_IR_ENTREE) == HIGH) {
         // Petite sécurité : si on retire pendant le scan, on annule
         envoyerCommande(CMD_STOP_SCAN, sizeof(CMD_STOP_SCAN));
         etatActuel = ATTENTE;
         allumerLeds(CRGB::Green);
      }
      break;

    case VALIDATION:
      clignoter(CRGB::Blue);
      etatActuel = ATTENTE_RETRAIT;
      break;

    case ERREUR:
      clignoter(CRGB::Red);
      etatActuel = ATTENTE_RETRAIT;
      break;

    case ATTENTE_RETRAIT:
      allumerLeds(CRGB::Blue);
      // On attend que le capteur physique remonte à HIGH (plus d'obstacle)
      if (digitalRead(PIN_IR_ENTREE) == HIGH) {
        Serial.println("--- PRET ---");
        
        // On nettoie le flag au cas où une interruption aurait eu lieu pendant le retrait
        portENTER_CRITICAL(&monVerrou);
        flag_demande_scan = false;
        portEXIT_CRITICAL(&monVerrou);
        
        etatActuel = ATTENTE;
        allumerLeds(CRGB::Green);
      }
      break;
  }
}