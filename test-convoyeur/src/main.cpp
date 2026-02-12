#include <SPI.h>
#include <Ethernet.h>
#include <FastLED.h>
#include <Wire.h>
#include "esp_system.h"

// =============================================================
//                   CONFIGURATION MATÉRIELLE
// =============================================================

// --- ETHERNET (Non utilisé pour l'instant) ---
#define W5500_CS_PIN 10

// --- LED STRIP ---
#define LED_PIN 7
#define NUM_LEDS 27

// --- SCANNER BARCODE (UART2) ---
#define SCANNER_RX_PIN 17
#define SCANNER_TX_PIN 18

// --- CAPTEUR ULTRASON (I2C) ---
#define SRF02_ADDR 0x70
#define SDA_PIN_sfr02 8
#define SCL_PIN_sfr02 9

// --- CAPTEURS IR (Logique Inversée gérée dans le code) ---
#define PIN_IR_ENTREE 6
#define PIN_IR_VALIDATION_1 2
#define PIN_IR_VALIDATION_2 3

// --- MOTEUR (Pont en H) ---
#define MOTEUR_DIR1 20 // Fil Jaune
#define MOTEUR_DIR2 21 // Fil Vert

// Commande HEX pour déclencher le scan (Trigger)
const byte COMMAND_TRIGGER[] = {0x7E, 0x00, 0x08, 0x01, 0x00, 0x02, 0x01, 0xAB, 0xCD};

// Objets Globaux
HardwareSerial ScannerSerial(2);
CRGB leds[NUM_LEDS];

// =============================================================
//                   VARIABLES GLOBALES
// =============================================================

portMUX_TYPE myMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool interruptionEntree = false;

enum State
{
  BORNE_PRETE,
  BOUTEILLE_DETECTEE,
  SCAN_EN_COURS,
  DEPOT_VALIDE,
  DEPOT_REFUSE,
  BAC_PLEIN
};

volatile State currentState = BORNE_PRETE;
unsigned long timerEtat = 0;
bool clignotementState = false;

// --- VARIABLES DE SUIVI (TRACKING) ---
// Déclarées en global pour pouvoir être remises à zéro n'importe quand
bool v1_ok = false, v2_ok = false, bouteille_validee = false;
bool aVuEntree = false;
bool aVuSortie = false;
bool fraudeDetectee = false;

// =============================================================
//                   FONCTIONS UTILITAIRES
// =============================================================

// --- LECTURE ULTRASON SRF02 ---
int lireSRF02()
{
  Wire.beginTransmission(SRF02_ADDR);
  Wire.write(0x00);
  Wire.write(0x51); // Commande mesure en cm
  Wire.endTransmission();
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

// ================= FONCTIONS MOTEUR (INTELLIGENTE) =================
void piloterMoteur(int sens)
{
  // Variable "mémoire" qui reste stockée entre deux appels
  static int dernierSens = 99; // 99 = valeur impossible au démarrage

  // Si on demande la MÊME chose que la dernière fois, on ne fait RIEN.
  // C'est ça qui empêche les à-coups !
  if (sens == dernierSens) return;

  // ---------------------------------------------------------
  // Si on arrive ici, c'est que le sens A CHANGÉ.
  // On applique donc la procédure de sécurité.
  // ---------------------------------------------------------

  // 1. On mémorise le nouveau sens
  dernierSens = sens;

  // 2. On coupe tout
  digitalWrite(MOTEUR_DIR1, LOW);
  digitalWrite(MOTEUR_DIR2, LOW);

  // 3. Temps mort (seulement si on ne demande pas l'arrêt total)
  if (sens != 0) 
  {
    delay(500); // Pause de sécurité mécanique
  }

  Serial.print(">>> CHANGEMENT MOTEUR VERS : ");
  Serial.println(sens);

  // 4. On applique la nouvelle direction
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
  // Si sens == 0, on laisse tout à LOW (déjà fait à l'étape 2)
}

// --- GESTION DES LEDS ---
void actualiserLeds()
{
  static unsigned long lastBlink = 0;
  if (millis() - lastBlink > 400)
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
  case SCAN_EN_COURS:
  case DEPOT_VALIDE:
    fill_solid(leds, NUM_LEDS, clignotementState ? CRGB::Blue : CRGB::Black);
    break;
  case DEPOT_REFUSE:
    fill_solid(leds, NUM_LEDS, clignotementState ? CRGB::Red : CRGB::Black);
    break;
  case BAC_PLEIN:
    fill_solid(leds, NUM_LEDS, CRGB::Red);
    break;
  }
  FastLED.show();
}

// --- INTERRUPTION IR ENTREE ---
void IRAM_ATTR ISR_Entree()
{
  portENTER_CRITICAL_ISR(&myMux);
  if (currentState == BORNE_PRETE)
    interruptionEntree = true;
  portEXIT_CRITICAL_ISR(&myMux);
}

// --- NETTOYAGE STRING ---
String nettoyerCode(String brut)
{
  String propre = "";
  for (int i = 0; i < brut.length(); i++)
  {
    if (isDigit(brut[i]))
      propre += brut[i];
  }
  return propre;
}

// =============================================================
//                         SETUP
// =============================================================
void setup()
{
  Serial.begin(115200);
  Wire.begin(SDA_PIN_sfr02, SCL_PIN_sfr02);

  // Config Pins
  pinMode(MOTEUR_DIR1, OUTPUT);
  pinMode(MOTEUR_DIR2, OUTPUT);
  pinMode(PIN_IR_ENTREE, INPUT_PULLUP);
  pinMode(PIN_IR_VALIDATION_1, INPUT_PULLUP);
  pinMode(PIN_IR_VALIDATION_2, INPUT_PULLUP);

  // Init LEDs
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);

  // Init Scanner
  ScannerSerial.begin(9600, SERIAL_8N1, SCANNER_RX_PIN, SCANNER_TX_PIN);
  
  // Interruption Entrée (Sur front montant = coupure faisceau)
  attachInterrupt(digitalPinToInterrupt(PIN_IR_ENTREE), ISR_Entree, RISING);

  Serial.println("=== SYSTÈME OPTIMISÉ (TIMEOUT 10s + REVERSE SECURISE) ===");
}

// =============================================================
//                         LOOP
// =============================================================
void loop()
{
  actualiserLeds();

  // ---------------------------------------------
  // 1. SURVEILLANCE BAC PLEIN (Toutes les 2 sec)
  // ---------------------------------------------
  static unsigned long lastCheckBac = 0;
  if (millis() - lastCheckBac > 2000)
  {
    int dist = lireSRF02();
    if (dist > 0 && dist < 10)
    {
      portENTER_CRITICAL(&myMux);
      currentState = BAC_PLEIN;
      portEXIT_CRITICAL(&myMux);
    }
    lastCheckBac = millis();
  }

  // ---------------------------------------------
  // 2. MACHINE À ÉTATS
  // ---------------------------------------------
  State etatActuel;
  portENTER_CRITICAL(&myMux);
  etatActuel = currentState;
  portEXIT_CRITICAL(&myMux);

  switch (etatActuel)
  {
  // --- ÉTAT 1 : ATTENTE ---
  case BORNE_PRETE:
    portENTER_CRITICAL(&myMux);
    if (interruptionEntree)
    {
      interruptionEntree = false;
      currentState = BOUTEILLE_DETECTEE;
      timerEtat = millis();
      // Vidage tampon scanner
      while (ScannerSerial.available()) ScannerSerial.read();
      Serial.println("\n>>> BOUTEILLE DÉTECTÉE (Faisceau coupé)");
    }
    portEXIT_CRITICAL(&myMux);
    break;

  // --- ÉTAT 2 : SCAN ---
  case BOUTEILLE_DETECTEE:
    static bool commandeEnvoyee = false;

    // Envoi de la commande Trigger (une seule fois)
    if (!commandeEnvoyee)
    {
      while (ScannerSerial.available()) ScannerSerial.read();
      ScannerSerial.write(0x00); // Réveil
      delay(50);
      ScannerSerial.write(COMMAND_TRIGGER, sizeof(COMMAND_TRIGGER));
      commandeEnvoyee = true;
      timerEtat = millis();
    }

    // Timeout Scan (3s) -> Si échec, on rejette
    if (millis() - timerEtat > 3000)
    {
      Serial.println(">>> TIMEOUT SCAN - Aucun code lu");
      
      portENTER_CRITICAL(&myMux);
      currentState = DEPOT_REFUSE;
      timerEtat = millis();
      portEXIT_CRITICAL(&myMux);
      
      commandeEnvoyee = false;
      piloterMoteur(-1); // Rejet immédiat
    }
    // Lecture Code
    else if (ScannerSerial.available())
    {
      String codeBrut = ScannerSerial.readStringUntil('\r');
      String codePropre = nettoyerCode(codeBrut);

      if (codePropre.length() > 10)
      {
        Serial.print(">>> CODE LU : "); Serial.println(codePropre);
        Serial.println(">>> ✅ MODE TEST : CODE ACCEPTÉ");

        portENTER_CRITICAL(&myMux);
        currentState = DEPOT_VALIDE;
        timerEtat = millis();
        bouteille_validee = false;
        portEXIT_CRITICAL(&myMux);

        // RESET COMPLET DU SUIVI TRACKING AVANT DÉMARRAGE
        aVuEntree = false;
        aVuSortie = false;
        fraudeDetectee = false;

        piloterMoteur(1); // Moteur AVANT
        commandeEnvoyee = false;
      }
    }
    break;

  // --- ÉTAT 3 : CONVOYAGE & VALIDATION ---
  case DEPOT_VALIDE:
  {
    // Lecture Capteurs avec INVERSION (!)
    byte ir1 = !digitalRead(PIN_IR_VALIDATION_1);
    byte ir2 = !digitalRead(PIN_IR_VALIDATION_2);
    int etatBinaire = (ir1 << 1) | ir2;

    // --- ANALYSE SÉQUENCE ---
    
    // 1. Passage Entrée (01 ou 00)
    if (etatBinaire == 1 || etatBinaire == 0)
    {
      aVuEntree = true;
      if (aVuSortie == true) // Si on revient en arrière
      {
        fraudeDetectee = true;
        Serial.println(">>> ALERTE : Retour arrière !");
      }
    }

    // 2. Sortie imminente (10)
    if (etatBinaire == 2)
    {
      if (aVuEntree) aVuSortie = true;
    }

    // 3. Disparition (11) -> VERDICT
    if (etatBinaire == 3) 
    { 
      // CAS A : SUCCÈS
      if (aVuEntree && aVuSortie && !fraudeDetectee && !bouteille_validee) 
      {
        bouteille_validee = true;
        Serial.println(">>> SÉQUENCE VALIDE (01->00->10->11) ✅");
        
        Serial.println(">>> Chute de la bouteille...");
        delay(500);       // Temps de chute
        piloterMoteur(0); // ARRÊT PROPRE
        
        portENTER_CRITICAL(&myMux);
        currentState = BORNE_PRETE;
        portEXIT_CRITICAL(&myMux);
      }
      
      // CAS B : ÉCHEC / FRAUDE
      else if (aVuEntree && (!aVuSortie || fraudeDetectee)) 
      {
        Serial.println(">>> REFUS : Séquence incorrecte ⛔");
        piloterMoteur(0); // Stop
        Serial.println(">>> Pause avant rejet...");
        delay(1000); // Pause visuelle
        
        portENTER_CRITICAL(&myMux);
        currentState = DEPOT_REFUSE;
        timerEtat = millis();
        portEXIT_CRITICAL(&myMux);
        
        piloterMoteur(-1); // Rejet
      }
    }

    // CAS C : TIMEOUT GLOBAL (10s)
    if (millis() - timerEtat > 10000)
    {
      Serial.println(">>> TIMEOUT (10s) : Bouteille bloquée");
      piloterMoteur(0); // Stop
      delay(1000); // Pause visuelle
      
      portENTER_CRITICAL(&myMux);
      currentState = DEPOT_REFUSE;
      timerEtat = millis();
      portEXIT_CRITICAL(&myMux);
      
      piloterMoteur(-1); // Rejet
      
      // Reset Tracking
      aVuEntree = false; aVuSortie = false; fraudeDetectee = false;
    }

  }
  break;

  // --- ÉTAT 4 : REJET (Marche Arrière 10s) ---
  case DEPOT_REFUSE:
    // On s'assure que le moteur recule
    piloterMoteur(-1);

    if (millis() - timerEtat > 10000)
    {
      Serial.println(">>> Fin du rejet (10s écoulées)");
      piloterMoteur(0); // Arrêt
      
      portENTER_CRITICAL(&myMux);
      currentState = BORNE_PRETE;
      Serial.println(">>> Borne prête");
      portEXIT_CRITICAL(&myMux);
    }
    break;

  // --- ÉTAT 5 : ERREUR BAC PLEIN ---
  case BAC_PLEIN:
    piloterMoteur(0);
    break;
  }
}