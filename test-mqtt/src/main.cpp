#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h> // <--- NOUVELLE BIBLIOTHÈQUE MQTT
#include <FastLED.h>
#include <Wire.h>
#include "esp_system.h"

// ================= CONFIGURATION MATÉRIELLE =================
#define W5500_CS_PIN 10   // Pin CS du module Ethernet
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

// ================= CONFIGURATION RÉSEAU & MQTT =================
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED }; // Adresse MAC arbitraire
IPAddress brokerIP(192, 168, 1, 100); // L'IP du Raspberry Pi de ton pote

EthernetClient ethClient;
PubSubClient mqttClient(ethClient);

unsigned long lastMQTTReconnectAttempt = 0; // Pour le timer de reconnexion

// ================= VARIABLES SYSTÈME =================
portMUX_TYPE myMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool interruptionEntree = false;

enum State
{
  BORNE_PRETE,
  BOUTEILLE_DETECTEE,
  SCAN_EN_COURS,
  DEPOT_VALIDE,
  ATTENTE_CHUTE,
  ATTENTE_REJET,
  REJET_EN_COURS,
  BAC_PLEIN
};

volatile State currentState = BORNE_PRETE;
unsigned long timerEtat = 0;
bool clignotementState = false;

// Variables de suivi
bool v1_ok = false, v2_ok = false, bouteille_validee = false;
bool aVuEntree = false, aVuSortie = false, fraudeDetectee = false;

// ================= FONCTIONS UTILITAIRES =================

// --- GESTION MQTT NON-BLOQUANTE ---
void gererMQTT() {
  // Si on n'est pas connecté, on essaie toutes les 5 secondes (sans bloquer)
  if (!mqttClient.connected()) {
    if (millis() - lastMQTTReconnectAttempt > 5000) {
      lastMQTTReconnectAttempt = millis();
      Serial.println(">>> Tentative de connexion MQTT...");
      
      // On tente de se connecter avec le nom "Borne_Ecobox"
      if (mqttClient.connect("Borne_Ecobox")) {
        Serial.println(">>> ✅ Connecté au Broker MQTT !");
      } else {
        Serial.print(">>> ❌ Échec MQTT, code erreur : ");
        Serial.println(mqttClient.state());
      }
    }
  } else {
    // Si on est connecté, on maintient la connexion active
    mqttClient.loop();
  }
}

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

void piloterMoteur(int sens)
{
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

void actualiserLeds()
{
  static unsigned long lastBlink = 0;
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
  case BORNE_PRETE:      fill_solid(leds, NUM_LEDS, CRGB::Green); break;
  case BOUTEILLE_DETECTEE: fill_solid(leds, NUM_LEDS, CRGB::Blue); break;
  case DEPOT_VALIDE:     fill_solid(leds, NUM_LEDS, clignotementState ? CRGB::Blue : CRGB::Black); break;
  case ATTENTE_CHUTE:    fill_solid(leds, NUM_LEDS, CRGB::Green); break;
  case ATTENTE_REJET:    fill_solid(leds, NUM_LEDS, CRGB::Red); break;
  case REJET_EN_COURS:   fill_solid(leds, NUM_LEDS, clignotementState ? CRGB::Red : CRGB::Black); break;
  case BAC_PLEIN:        fill_solid(leds, NUM_LEDS, CRGB::Red); break;
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

  // --- INITIALISATION ETHERNET ET MQTT ---
  Serial.println(">>> Initialisation du réseau Ethernet W5500...");
  Ethernet.init(W5500_CS_PIN); // Indispensable pour l'ESP32

  // Démarrage DHCP (Attention : ça bloque un peu au démarrage si le câble n'est pas branché)
  if (Ethernet.begin(mac) == 0) {
    Serial.println(">>> ❌ Échec de la configuration DHCP");
    // On pourrait forcer une IP statique ici si besoin
  } else {
    Serial.print(">>> ✅ Connecté au réseau. IP : ");
    Serial.println(Ethernet.localIP());
  }

  // Configuration du broker MQTT
  mqttClient.setServer(brokerIP, 1883);
  
  Serial.println("=== SYSTÈME DÉMARRÉ (AVEC MQTT) ===");
}

// ================= LOOP =================
void loop()
{
  actualiserLeds();
  
  // Maintient la connexion réseau active et se reconnecte si besoin
  gererMQTT();

  // Surveillance Bac
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

  // --- MACHINE À ÉTATS ---
  State etatActuel;
  portENTER_CRITICAL(&myMux);
  etatActuel = currentState;
  portEXIT_CRITICAL(&myMux);

  switch (etatActuel)
  {
  case BORNE_PRETE:
    piloterMoteur(0); 
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
      delay(10); 
      ScannerSerial.write(COMMAND_TRIGGER, sizeof(COMMAND_TRIGGER));
      commandeEnvoyee = true;
      timerEtat = millis();
    }

    if (millis() - timerEtat > 3000)
    {
      Serial.println(">>> TIMEOUT SCAN -> Rejet");
      
      portENTER_CRITICAL(&myMux);
      currentState = ATTENTE_REJET;
      timerEtat = millis();
      portEXIT_CRITICAL(&myMux);
      
      commandeEnvoyee = false;
      piloterMoteur(0); 
    }
    else if (ScannerSerial.available())
    {
      String codeBrut = ScannerSerial.readStringUntil('\r');
      String codePropre = nettoyerCode(codeBrut);

      if (codePropre.length() > 10)
      {
        Serial.print(">>> CODE OK: "); Serial.println(codePropre);

        // =========================================================
        // PUBLICATION MQTT : ENVOI DU CODE AU RASPBERRY PI
        // =========================================================
        if (mqttClient.connected()) {
          // On publie sur le topic "ecobox/scan"
          mqttClient.publish("ecobox/print", codePropre.c_str());
          Serial.println(">>> 📡 Code envoyé au serveur (MQTT)");
        } else {
          Serial.println(">>> ⚠️ Impossible d'envoyer le code (Non connecté au broker)");
        }
        // =========================================================

        portENTER_CRITICAL(&myMux);
        currentState = DEPOT_VALIDE;
        timerEtat = millis(); 
        portEXIT_CRITICAL(&myMux);

        aVuEntree = false; aVuSortie = false; fraudeDetectee = false; bouteille_validee = false;
        
        piloterMoteur(1); 
        commandeEnvoyee = false;
      }
    }
    break;

  case DEPOT_VALIDE:
  {
    byte ir1 = !digitalRead(PIN_IR_VALIDATION_1);
    byte ir2 = !digitalRead(PIN_IR_VALIDATION_2);
    int etatBinaire = (ir1 << 1) | ir2;

    if (etatBinaire == 1 || etatBinaire == 0) { aVuEntree = true; if(aVuSortie) fraudeDetectee = true; }
    if (etatBinaire == 2) { if(aVuEntree) aVuSortie = true; }

    if (etatBinaire == 3 && aVuEntree && aVuSortie && !fraudeDetectee && !bouteille_validee) 
    {
       bouteille_validee = true;
       Serial.println(">>> SUCCÈS ! Passage en mode chute...");
       
       portENTER_CRITICAL(&myMux);
       currentState = ATTENTE_CHUTE;
       timerEtat = millis();
       portEXIT_CRITICAL(&myMux);
    }
    
    else if (etatBinaire == 3 && aVuEntree && (!aVuSortie || fraudeDetectee))
    {
       Serial.println(">>> FRAUDE DÉTECTÉE -> Pause Rouge");
       piloterMoteur(0); 
       
       portENTER_CRITICAL(&myMux);
       currentState = ATTENTE_REJET; 
       timerEtat = millis();
       portEXIT_CRITICAL(&myMux);
    }

    if (millis() - timerEtat > 5000)
    {
       Serial.println(">>> TIMEOUT GLOBAL -> Pause Rouge");
       piloterMoteur(0); 
       
       portENTER_CRITICAL(&myMux);
       currentState = ATTENTE_REJET;
       timerEtat = millis();
       portEXIT_CRITICAL(&myMux);
    }
  }
  break;

  case ATTENTE_CHUTE:
    if (millis() - timerEtat > 500)
    {
      Serial.println(">>> Chute terminée. Prêt.");
      piloterMoteur(0);
      
      portENTER_CRITICAL(&myMux);
      currentState = BORNE_PRETE;
      portEXIT_CRITICAL(&myMux);
    }
    break;

  case ATTENTE_REJET:
    piloterMoteur(0); 
    
    if (millis() - timerEtat > 1000)
    {
      Serial.println(">>> Fin pause. Lancement MARCHE ARRIÈRE (10s).");
      
      portENTER_CRITICAL(&myMux);
      currentState = REJET_EN_COURS;
      timerEtat = millis();
      portEXIT_CRITICAL(&myMux);
      
      piloterMoteur(-1); 
    }
    break;

  case REJET_EN_COURS:
    piloterMoteur(-1); 
    
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