#include <SPI.h>
#include <Ethernet.h>
#include <FastLED.h>
#include <Wire.h>
#include "esp_system.h"

// ================= CONFIGURATION MATÉRIELLE =================
#define LED_PIN 7
#define NUM_LEDS 27
#define SCANNER_RX_PIN 17
#define SCANNER_TX_PIN 18

// I2C & Capteurs
#define SRF02_ADDR 0x70
#define SDA_PIN_sfr02 8
#define SCL_PIN_sfr02 9
#define PIN_IR_ENTREE 6
#define PIN_IR_VALIDATION_1 2
#define PIN_IR_VALIDATION_2 3

// Moteur
#define MOTEUR_DIR1 20 
#define MOTEUR_DIR2 21 

const byte COMMAND_TRIGGER[] = {0x7E, 0x00, 0x08, 0x01, 0x00, 0x02, 0x01, 0xAB, 0xCD};

HardwareSerial ScannerSerial(2);
CRGB leds[NUM_LEDS];

// ================= VARIABLES SYSTÈME =================
portMUX_TYPE myMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool interruptionEntree = false;

// --- NOUVEAUX ÉTATS POUR LE NON-BLOQUANT ---
enum State
{
  BORNE_PRETE,
  BOUTEILLE_DETECTEE,
  SCAN_EN_COURS,     // (Optionnel si tu veux séparer)
  DEPOT_VALIDE,      // La bouteille avance et on vérifie les capteurs
  ATTENTE_CHUTE,     // (Nouveau) 500ms pour laisser tomber la bouteille
  ATTENTE_REJET,     // (Nouveau) 1s de pause rouge avant marche arrière
  REJET_EN_COURS,    // (Renommé) La marche arrière de 10s
  BAC_PLEIN
};

volatile State currentState = BORNE_PRETE;
unsigned long timerEtat = 0;
bool clignotementState = false;

// Variables de suivi
bool v1_ok = false, v2_ok = false, bouteille_validee = false;
bool aVuEntree = false, aVuSortie = false, fraudeDetectee = false;

// ================= FONCTIONS UTILITAIRES =================

int lireSRF02()
{
  Wire.beginTransmission(SRF02_ADDR);
  Wire.write(0x00);
  Wire.write(0x51);
  Wire.endTransmission();
  // Note : Ce delay(70) est le SEUL bloquant restant (requis par le capteur I2C)
  // On pourrait l'enlever avec une gestion complexe, mais c'est acceptable ici.
  delay(70); 

  Wire.beginTransmission(SRF02_ADDR);
  Wire.write(0x02);
  Wire.endTransmission();

  Wire.requestFrom(SRF02_ADDR, 2);
  if (Wire.available() >= 2)
  {
    byte high = Wire.read();
    byte low = Wire.read();
    return (high << 8) | low;
  }
  return -1;
}

// --- PILOTAGE MOTEUR (SIMPLIFIÉ & INSTANTANÉ) ---
// Plus aucun delay() ici ! C'est la machine à états qui gère le timing.
void piloterMoteur(int sens)
{
  // On applique simplement les tensions.
  // La sécurité "Dead Time" est maintenant gérée par l'état ATTENTE_REJET
  
  if (sens == 1) // AVANT
  {
    digitalWrite(MOTEUR_DIR1, HIGH);
    digitalWrite(MOTEUR_DIR2, LOW);
  }
  else if (sens == -1) // ARRIÈRE
  {
    digitalWrite(MOTEUR_DIR1, LOW);
    digitalWrite(MOTEUR_DIR2, HIGH);
  }
  else // STOP
  {
    digitalWrite(MOTEUR_DIR1, LOW);
    digitalWrite(MOTEUR_DIR2, LOW);
  }
}

// --- GESTION DES LEDS (ANIMATION FLUIDE) ---
void actualiserLeds()
{
  static unsigned long lastBlink = 0;
  // Clignotement rapide (200ms) pour bien voir que ça vit
  if (millis() - lastBlink > 200) 
  {
    clignotementState = !clignotementState;
    lastBlink = millis();
  }

  State etatLocal;
  portENTER_CRITICAL(&myMux);
  etatLocal = currentState;
  portEXIT_CRITICAL(&myMux);

  switch (etatLocal)
  {
  case BORNE_PRETE:
    fill_solid(leds, NUM_LEDS, CRGB::Green);
    break;
    
  case BOUTEILLE_DETECTEE:
    fill_solid(leds, NUM_LEDS, CRGB::Blue);
    break;
    
  case DEPOT_VALIDE: // Convoyage en cours
    // Chenillard ou clignotement bleu
    fill_solid(leds, NUM_LEDS, clignotementState ? CRGB::Blue : CRGB::Black);
    break;
    
  case ATTENTE_CHUTE: // Succès imminent
    fill_solid(leds, NUM_LEDS, CRGB::Green); // Vert fixe confirmant le succès
    break;

  case ATTENTE_REJET: // Pause dramatique
    fill_solid(leds, NUM_LEDS, CRGB::Red); // Rouge FIXE et intense
    break;
    
  case REJET_EN_COURS: // Marche arrière
    // Rouge clignotant alerte
    fill_solid(leds, NUM_LEDS, clignotementState ? CRGB::Red : CRGB::Black);
    break;
    
  case BAC_PLEIN:
    fill_solid(leds, NUM_LEDS, CRGB::Red); // Rouge permanent
    break;
  }
  FastLED.show();
}

void IRAM_ATTR ISR_Entree()
{
  portENTER_CRITICAL_ISR(&myMux);
  if (currentState == BORNE_PRETE)
    interruptionEntree = true;
  portEXIT_CRITICAL_ISR(&myMux);
}

String nettoyerCode(String brut)
{
  String propre = "";
  for (int i = 0; i < brut.length(); i++) {
    if (isDigit(brut[i])) propre += brut[i];
  }
  return propre;
}

// ================= SETUP =================
void setup()
{
  Serial.begin(115200);
  Wire.begin(SDA_PIN_sfr02, SCL_PIN_sfr02);

  pinMode(MOTEUR_DIR1, OUTPUT);
  pinMode(MOTEUR_DIR2, OUTPUT);
  pinMode(PIN_IR_ENTREE, INPUT_PULLUP);
  pinMode(PIN_IR_VALIDATION_1, INPUT_PULLUP);
  pinMode(PIN_IR_VALIDATION_2, INPUT_PULLUP);

  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
  ScannerSerial.begin(9600, SERIAL_8N1, SCANNER_RX_PIN, SCANNER_TX_PIN);
  attachInterrupt(digitalPinToInterrupt(PIN_IR_ENTREE), ISR_Entree, RISING);

  Serial.println("=== SYSTÈME DÉMARRÉ (NON-BLOQUANT) ===");
}

// ================= LOOP =================
void loop()
{
  // Cette fonction tourne à toute vitesse, permettant aux LEDS de s'animer
  actualiserLeds();

  // Surveillance Bac (Toutes les 2 sec)
  static unsigned long lastCheckBac = 0;
  if (millis() - lastCheckBac > 2000)
  {
    int dist = lireSRF02(); // Le seul mini-blocage (70ms) est ici
    if (dist > 0 && dist < 10)
    {
      portENTER_CRITICAL(&myMux);
      currentState = BAC_PLEIN;
      portEXIT_CRITICAL(&myMux);
    }
    lastCheckBac = millis();
  }

  // --- MACHINE À ÉTATS ---
  State etatActuel;
  portENTER_CRITICAL(&myMux);
  etatActuel = currentState;
  portEXIT_CRITICAL(&myMux);

  switch (etatActuel)
  {
  case BORNE_PRETE:
    piloterMoteur(0); // S'assure que le moteur est coupé
    portENTER_CRITICAL(&myMux);
    if (interruptionEntree)
    {
      interruptionEntree = false;
      currentState = BOUTEILLE_DETECTEE;
      timerEtat = millis();
      while (ScannerSerial.available()) ScannerSerial.read();
      Serial.println("\n>>> BOUTEILLE DÉTECTÉE");
    }
    portEXIT_CRITICAL(&myMux);
    break;

  case BOUTEILLE_DETECTEE:
    static bool commandeEnvoyee = false;

    if (!commandeEnvoyee)
    {
      while (ScannerSerial.available()) ScannerSerial.read();
      ScannerSerial.write(0x00);
      delay(10); // Petit délai négligeable pour le UART
      ScannerSerial.write(COMMAND_TRIGGER, sizeof(COMMAND_TRIGGER));
      commandeEnvoyee = true;
      timerEtat = millis();
    }

    // Timeout Scan (3s)
    if (millis() - timerEtat > 3000)
    {
      Serial.println(">>> TIMEOUT SCAN -> Rejet");
      
      // Passage en mode ATTENTE REJET (Pause rouge)
      portENTER_CRITICAL(&myMux);
      currentState = ATTENTE_REJET;
      timerEtat = millis();
      portEXIT_CRITICAL(&myMux);
      
      commandeEnvoyee = false;
      piloterMoteur(0); // Stop moteur avant la pause
    }
    else if (ScannerSerial.available())
    {
      String codeBrut = ScannerSerial.readStringUntil('\r');
      String codePropre = nettoyerCode(codeBrut);

      if (codePropre.length() > 10)
      {
        Serial.print(">>> CODE OK: "); Serial.println(codePropre);

        portENTER_CRITICAL(&myMux);
        currentState = DEPOT_VALIDE;
        timerEtat = millis(); // On lance le timer global de 10s
        portEXIT_CRITICAL(&myMux);

        // Reset Tracking
        aVuEntree = false; aVuSortie = false; fraudeDetectee = false; bouteille_validee = false;
        
        piloterMoteur(1); // En avant toute !
        commandeEnvoyee = false;
      }
    }
    break;

  case DEPOT_VALIDE:
  {
    byte ir1 = !digitalRead(PIN_IR_VALIDATION_1);
    byte ir2 = !digitalRead(PIN_IR_VALIDATION_2);
    int etatBinaire = (ir1 << 1) | ir2;

    // Tracking
    if (etatBinaire == 1 || etatBinaire == 0) { aVuEntree = true; if(aVuSortie) fraudeDetectee = true; }
    if (etatBinaire == 2) { if(aVuEntree) aVuSortie = true; }

    // --- VERDICTS ---
    
    // CAS 1 : SUCCÈS (Sortie libérée)
    if (etatBinaire == 3 && aVuEntree && aVuSortie && !fraudeDetectee && !bouteille_validee) 
    {
       bouteille_validee = true;
       Serial.println(">>> SUCCÈS ! Passage en mode chute...");
       
       // On ne bloque pas ! On change juste d'état pour attendre 500ms
       portENTER_CRITICAL(&myMux);
       currentState = ATTENTE_CHUTE;
       timerEtat = millis();
       portEXIT_CRITICAL(&myMux);
       // Le moteur continue de tourner (1) pendant la chute
    }
    
    // CAS 2 : FRAUDE
    else if (etatBinaire == 3 && aVuEntree && (!aVuSortie || fraudeDetectee))
    {
       Serial.println(">>> FRAUDE DÉTECTÉE -> Pause Rouge");
       piloterMoteur(0); // Arrêt immédiat
       
       portENTER_CRITICAL(&myMux);
       currentState = ATTENTE_REJET; // On va attendre 1s en rouge
       timerEtat = millis();
       portEXIT_CRITICAL(&myMux);
    }

    // CAS 3 : TIMEOUT GLOBAL (5s)
    if (millis() - timerEtat > 5000)
    {
       Serial.println(">>> TIMEOUT GLOBAL -> Pause Rouge");
       piloterMoteur(0); // Arrêt immédiat
       
       portENTER_CRITICAL(&myMux);
       currentState = ATTENTE_REJET;
       timerEtat = millis();
       portEXIT_CRITICAL(&myMux);
    }
  }
  break;

  // --- NOUVEAU : GESTION DE LA CHUTE (500ms) ---
  case ATTENTE_CHUTE:
    // On laisse le moteur tourner 500ms puis on coupe
    if (millis() - timerEtat > 500)
    {
      Serial.println(">>> Chute terminée. Prêt.");
      piloterMoteur(0);
      
      portENTER_CRITICAL(&myMux);
      currentState = BORNE_PRETE;
      portEXIT_CRITICAL(&myMux);
    }
    break;

  // --- NOUVEAU : LA PAUSE DRAMATIQUE (1s) ---
  // Sert aussi de sécurité mécanique (moteur à l'arrêt avant inversion)
  case ATTENTE_REJET:
    piloterMoteur(0); // S'assure que c'est stoppé
    
    // On attend 1 seconde (pendant ce temps les LEDs sont rouges grâce à actualiserLeds)
    if (millis() - timerEtat > 1000)
    {
      Serial.println(">>> Fin pause. Lancement MARCHE ARRIÈRE (10s).");
      
      portENTER_CRITICAL(&myMux);
      currentState = REJET_EN_COURS;
      timerEtat = millis();
      portEXIT_CRITICAL(&myMux);
      
      piloterMoteur(-1); // On lance la marche arrière
    }
    break;

  // --- ANCIEN "DEPOT_REFUSE" ---
  case REJET_EN_COURS:
    piloterMoteur(-1); // Maintient la marche arrière
    
    // On recule pendant 5 secondes
    if (millis() - timerEtat > 5000)
    {
      Serial.println(">>> Fin du rejet.");
      piloterMoteur(0);
      
      portENTER_CRITICAL(&myMux);
      currentState = BORNE_PRETE;
      portEXIT_CRITICAL(&myMux);
    }
    break;

  case BAC_PLEIN:
    piloterMoteur(0);
    break;
  }
}