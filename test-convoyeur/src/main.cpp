#include <SPI.h>
#include <Ethernet.h>
#include <FastLED.h>
#include <Wire.h>       
#include "esp_system.h"

// ================= CONFIGURATION MATÉRIELLE =================
#define W5500_CS_PIN 10
#define LED_PIN 7
#define NUM_LEDS 27
#define SCANNER_RX_PIN 17
#define SCANNER_TX_PIN 18

// I2C pour SRF02
#define SRF02_ADDR 0x70
#define SDA_PIN_sfr02 8
#define SCL_PIN_sfr02 9

// Capteurs IR
#define PIN_IR_ENTREE 6
#define PIN_IR_VALIDATION_1 2
#define PIN_IR_VALIDATION_2 3

// Pilotage Moteur
#define MOTEUR_DIR1 20 //jaune
#define MOTEUR_DIR2 21 //vert

// Commande HEX pour déclencher le scan
const byte COMMAND_TRIGGER[] = {0x7E, 0x00, 0x08, 0x01, 0x00, 0x02, 0x01, 0xAB, 0xCD};

HardwareSerial ScannerSerial(2);
CRGB leds[NUM_LEDS];

// ================= VARIABLES SYSTÈME =================
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

bool v1_ok = false, v2_ok = false, bouteille_validee = false;

// ================= FONCTION SRF02 (I2C) =================
int lireSRF02()
{
  Wire.beginTransmission(SRF02_ADDR);
  Wire.write(0x00);
  Wire.write(0x51);
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

// ================= FONCTIONS MOTEUR =================
void piloterMoteur(int sens)
{ 
  Serial.print(">>> MOTEUR: ");
  
  if (sens == 1)  // AVANT
  {
    digitalWrite(MOTEUR_DIR1, HIGH);
    digitalWrite(MOTEUR_DIR2, LOW);
    Serial.println("AVANT (DIR1=HIGH, DIR2=LOW)");
  }
  else if (sens == -1)  // ARRIÈRE
  {
    digitalWrite(MOTEUR_DIR1, LOW);
    digitalWrite(MOTEUR_DIR2, HIGH);
    Serial.println("ARRIERE (DIR1=LOW, DIR2=HIGH)");
  }
  else  // STOP
  {
    digitalWrite(MOTEUR_DIR1, LOW);
    digitalWrite(MOTEUR_DIR2, LOW);
    Serial.println("STOP (DIR1=LOW, DIR2=LOW)");
  }
}

// ================= GESTION DES LEDS =================
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

// ================= INTERRUPTION =================
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
  for (int i = 0; i < brut.length(); i++)
  {
    if (isDigit(brut[i]))
      propre += brut[i];
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
  
  Serial.println("=== SYSTÈME DÉMARRÉ (MODE TEST SANS MQTT) ===");
  Serial.println("Tous les codes seront acceptés automatiquement");
}

// ================= LOOP =================
void loop()
{
  actualiserLeds();

  // Surveillance Bac Plein
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

  // Machine à états
  State etatActuel;
  portENTER_CRITICAL(&myMux);
  etatActuel = currentState;
  portEXIT_CRITICAL(&myMux);

  switch (etatActuel)
  {
  case BORNE_PRETE:
    portENTER_CRITICAL(&myMux);
    if (interruptionEntree)
    {
      interruptionEntree = false;
      currentState = BOUTEILLE_DETECTEE;
      timerEtat = millis();
      while (ScannerSerial.available())
        ScannerSerial.read();
      Serial.println("\n>>> BOUTEILLE DÉTECTÉE");
    }
    portEXIT_CRITICAL(&myMux);
    break;

  case BOUTEILLE_DETECTEE:
    static bool commandeEnvoyee = false;

    if (!commandeEnvoyee)
    {
      while (ScannerSerial.available())
        ScannerSerial.read();

      Serial.println(">>> Envoi commande SCAN...");
      ScannerSerial.write(0x00);
      delay(50);
      ScannerSerial.write(COMMAND_TRIGGER, sizeof(COMMAND_TRIGGER));

      commandeEnvoyee = true;
      timerEtat = millis();
    }

    // Timeout 3 secondes
    if (millis() - timerEtat > 3000)
    {
      Serial.println(">>> TIMEOUT - Aucun code scanné");
      portENTER_CRITICAL(&myMux);
      currentState = DEPOT_REFUSE;
      timerEtat = millis();
      portEXIT_CRITICAL(&myMux);
      commandeEnvoyee = false;
      piloterMoteur(-1);
    }
    // Lecture du code-barre
    else if (ScannerSerial.available())
    {
      String codeBrut = ScannerSerial.readStringUntil('\r');
      String codePropre = nettoyerCode(codeBrut);

      if (codePropre.length() > 10)
      {
        Serial.print(">>> CODE SCANNÉ : ");
        Serial.println(codePropre);
        
        // ✅ ON ACCEPTE TOUS LES CODES (MODE TEST)
        Serial.println(">>> ✅ CODE ACCEPTÉ (mode test)");
        Serial.println(">>> DÉMARRAGE MOTEUR AVANT");
        
        portENTER_CRITICAL(&myMux);
        currentState = DEPOT_VALIDE;
        timerEtat = millis();
        v1_ok = false;
        v2_ok = false;
        bouteille_validee = false;
        portEXIT_CRITICAL(&myMux);
        
        piloterMoteur(1); // ← LE MOTEUR DÉMARRE ICI
        
        commandeEnvoyee = false;
      }
    }
    break;

  case DEPOT_VALIDE:
    // Validation par double capteur IR
    if (!v1_ok && digitalRead(PIN_IR_VALIDATION_1) == HIGH)
    {
      v1_ok = true;
      Serial.println(">>> Capteur IR1 activé");
    }
    if (v1_ok && !v2_ok && digitalRead(PIN_IR_VALIDATION_2) == HIGH)
    {
      v2_ok = true;
      Serial.println(">>> Capteur IR2 activé");
    }

    if (v1_ok && v2_ok && !bouteille_validee)
    {
      bouteille_validee = true;
      Serial.println(">>> ✅ BOUTEILLE VALIDÉE");
    }

    if (millis() - timerEtat > 4000)
    {
      Serial.println(">>> Fin du convoyage (4s)");
      piloterMoteur(0);
      
      portENTER_CRITICAL(&myMux);
      currentState = bouteille_validee ? BORNE_PRETE : DEPOT_REFUSE;
      if (currentState == DEPOT_REFUSE)
      {
        timerEtat = millis();
        piloterMoteur(-1);
      }
      portEXIT_CRITICAL(&myMux);
    }
    break;

  case DEPOT_REFUSE:
    if (millis() - timerEtat > 3000)
    {
      Serial.println(">>> Fin du rejet (3s)");
      piloterMoteur(0);
      portENTER_CRITICAL(&myMux);
      currentState = BORNE_PRETE;
      Serial.println(">>> Borne prête");
      portEXIT_CRITICAL(&myMux);
    }
    break;

  case BAC_PLEIN:
    piloterMoteur(0);
    break;
  }
}