/*
  PROJET : Bornes de collecte des bouteilles PET 2026
  CIBLE : ESP32 (Étudiant 2 - Traitement des bouteilles)
  VERSION : 100% ETHERNET W5500 (Sans WiFi)
*/

#include <SPI.h>
#include <Ethernet.h>
#include <FastLED.h>
#include <PubSubClient.h>
#include "esp_system.h" // Pour esp_read_mac

// ================= CONFIGURATION MATÉRIELLE =================
#define LED_PIN         4
#define NUM_LEDS        8  
#define SCANNER_RX_PIN  16 
#define SCANNER_TX_PIN  17 
#define TRIGGER_PIN     13
#define ECHO_PIN        12
const int PIN_CS_ETHERNET = 10; // Pin CS du module W5500

// ===== CAPTEURS IR (3 capteurs) =====
#define PIN_IR_ENTREE         14  
#define PIN_IR_VALIDATION_1   27  
#define PIN_IR_VALIDATION_2   26  

// Pilotage Moteur (Pont en H)
#define MOTEUR_PWM      25
#define MOTEUR_DIR1     33
#define MOTEUR_DIR2     32

HardwareSerial ScannerSerial(2);
CRGB leds[NUM_LEDS];

// ================= VARIABLES CRITIQUES =================
portMUX_TYPE myMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool interruptionEntree = false;

// ================= ÉTATS DU SYSTÈME =================
enum State {
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

bool validation1_detectee = false;
bool validation2_detectee = false;
bool bouteille_validee = false;
bool scanner_actif = true;

// ================= RÉSEAU & MQTT =================
IPAddress server(192, 168, 1, 20); // IP de la Raspberry Pi 5
EthernetClient ethClient;          // REMPLACÉ : WiFiClient -> EthernetClient
PubSubClient mqttClient(ethClient);

// ================= FONCTIONS MOTEUR =================
void piloterMoteur(int sens) {
  if (sens == 1) {
    digitalWrite(MOTEUR_DIR1, HIGH);
    digitalWrite(MOTEUR_DIR2, LOW);
    analogWrite(MOTEUR_PWM, 200);
  } else if (sens == -1) {
    digitalWrite(MOTEUR_DIR1, LOW);
    digitalWrite(MOTEUR_DIR2, HIGH);
    analogWrite(MOTEUR_PWM, 200);
  } else {
    digitalWrite(MOTEUR_DIR1, LOW);
    digitalWrite(MOTEUR_DIR2, LOW);
    analogWrite(MOTEUR_PWM, 0);
  }
}

// ================= GESTION DES LEDS =================
void actualiserLeds() {
  static unsigned long lastBlink = 0;
  if (millis() - lastBlink > 400) {
    clignotementState = !clignotementState;
    lastBlink = millis();
  }

  State etatLocal;
  portENTER_CRITICAL(&myMux);
  etatLocal = currentState;
  portEXIT_CRITICAL(&myMux);

  switch (etatLocal) {
    case BORNE_PRETE:        fill_solid(leds, NUM_LEDS, CRGB::Green); break;
    case BOUTEILLE_DETECTEE: fill_solid(leds, NUM_LEDS, CRGB::Blue); break;
    case SCAN_EN_COURS:      
    case DEPOT_VALIDE:       fill_solid(leds, NUM_LEDS, clignotementState ? CRGB::Blue : CRGB::Black); break;
    case DEPOT_REFUSE:       fill_solid(leds, NUM_LEDS, clignotementState ? CRGB::Red : CRGB::Black); break;
    case BAC_PLEIN:          fill_solid(leds, NUM_LEDS, CRGB::Red); break;
  }
  FastLED.show();
}

// ================= LOGIQUE MQTT =================
void callback(char* topic, byte* payload, unsigned int length) {
  String msg = "";
  for (int i = 0; i < length; i++) msg += (char)payload[i];
  
  portENTER_CRITICAL(&myMux);
  if (msg == "OK") {
    currentState = DEPOT_VALIDE;
    timerEtat = millis();
    validation1_detectee = false;
    validation2_detectee = false;
    bouteille_validee = false;
    piloterMoteur(1); 
  } else {
    currentState = DEPOT_REFUSE;
    timerEtat = millis();
    piloterMoteur(-1);
  }
  portEXIT_CRITICAL(&myMux);
}

// ================= INTERRUPTION =================
void IRAM_ATTR ISR_Entree() {
  portENTER_CRITICAL_ISR(&myMux);
  if (currentState == BORNE_PRETE) {
    interruptionEntree = true;
  }
  portEXIT_CRITICAL_ISR(&myMux);
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== ESP32 Etudiant2 - Démarrage ETHERNET ===");
  
  pinMode(MOTEUR_PWM, OUTPUT); 
  pinMode(MOTEUR_DIR1, OUTPUT); 
  pinMode(MOTEUR_DIR2, OUTPUT);
  pinMode(PIN_IR_ENTREE, INPUT_PULLUP);
  pinMode(PIN_IR_VALIDATION_1, INPUT_PULLUP);
  pinMode(PIN_IR_VALIDATION_2, INPUT_PULLUP);
  pinMode(TRIGGER_PIN, OUTPUT); 
  pinMode(ECHO_PIN, INPUT);

  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
  fill_solid(leds, NUM_LEDS, CRGB::Orange);
  FastLED.show();
  
  // 1. Initialiser Ethernet
  Ethernet.init(PIN_CS_ETHERNET);

  // 2. Récupérer la MAC unique du processeur (méthode native ESP32)
  byte mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA); 
  mac[5] += 1; // Incrémentation pour l'ID Ethernet

  Serial.print("MAC Adresse Ethernet : ");
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X%c", mac[i], (i < 5) ? ':' : ' ');
  }
  Serial.println();

  // 3. Connexion au réseau via DHCP
  Serial.println("Connexion au réseau via DHCP...");
  if (Ethernet.begin(mac) == 0) {
    Serial.println("ERREUR : Pas de DHCP (Verifie le cable)");
    // On peut tenter une IP fixe ici si le DHCP échoue
  } else {
    Serial.print("REUSSI ! IP : ");
    Serial.println(Ethernet.localIP());
  }
  
  mqttClient.setServer(server, 1883);
  mqttClient.setCallback(callback);
  ScannerSerial.begin(9600, SERIAL_8N1, SCANNER_RX_PIN, SCANNER_TX_PIN);
  attachInterrupt(digitalPinToInterrupt(PIN_IR_ENTREE), ISR_Entree, FALLING);
  
  Serial.println("=== Système PRÊT (Mode Filaire) ===");
}

// ================= BOUCLE PRINCIPALE =================
void loop() {
  // ADAPTÉ POUR ETHERNET : Pas de WiFi.status()
  // On vérifie seulement la connexion au Broker MQTT
  if (!mqttClient.connected()) {
    static unsigned long lastReconnectAttempt = 0;
    if (millis() - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = millis();
      Serial.print("Connexion MQTT... ");
      if (mqttClient.connect("ESP32_Etudiant2")) {
        mqttClient.subscribe("borne/reponse");
        Serial.println("OK");
      } else {
        Serial.print("ECHEC, rc=");
        Serial.println(mqttClient.state());
      }
    }
  }
  
  mqttClient.loop();
  actualiserLeds();

  // ===== Vérification Bac Plein =====
  static unsigned long lastCheckBac = 0;
  if (millis() - lastCheckBac > 2000) {
    digitalWrite(TRIGGER_PIN, LOW); delayMicroseconds(2);
    digitalWrite(TRIGGER_PIN, HIGH); delayMicroseconds(10);
    digitalWrite(TRIGGER_PIN, LOW);
    long dist = (pulseIn(ECHO_PIN, HIGH, 20000) * 0.034) / 2;
    
    if (dist > 0 && dist < 10) {
      portENTER_CRITICAL(&myMux);
      currentState = BAC_PLEIN;
      portEXIT_CRITICAL(&myMux);
    } 
    else if (dist >= 10 && currentState == BAC_PLEIN) {
      portENTER_CRITICAL(&myMux);
      currentState = BORNE_PRETE;
      portEXIT_CRITICAL(&myMux);
    }
    lastCheckBac = millis();
  }

  // ===== Machine à états =====
  State etatActuel;
  portENTER_CRITICAL(&myMux);
  etatActuel = currentState;
  portEXIT_CRITICAL(&myMux);
  
  switch (etatActuel) {
    
    case BORNE_PRETE:
      portENTER_CRITICAL(&myMux);
      if (interruptionEntree) {
        interruptionEntree = false;
        currentState = BOUTEILLE_DETECTEE;
        timerEtat = millis();
        scanner_actif = true;
        while(ScannerSerial.available()) ScannerSerial.read();
      }
      portEXIT_CRITICAL(&myMux);
      break;

    case BOUTEILLE_DETECTEE:
      if (millis() - timerEtat > 3000) {
        portENTER_CRITICAL(&myMux);
        currentState = DEPOT_REFUSE;
        timerEtat = millis();
        scanner_actif = false;
        portEXIT_CRITICAL(&myMux);
        piloterMoteur(-1);
      } 
      else if (scanner_actif && ScannerSerial.available()) {
        String code = ScannerSerial.readStringUntil('\r');
        code.trim();
        if (code.length() > 5) {
          mqttClient.publish("borne/scan", code.c_str());
          portENTER_CRITICAL(&myMux);
          currentState = SCAN_EN_COURS;
          timerEtat = millis();
          scanner_actif = false;
          portEXIT_CRITICAL(&myMux);
        }
      }
      break;

    case SCAN_EN_COURS:
      if (millis() - timerEtat > 5000) {
        portENTER_CRITICAL(&myMux);
        currentState = DEPOT_REFUSE;
        timerEtat = millis();
        portEXIT_CRITICAL(&myMux);
        piloterMoteur(-1);
      }
      break;

    case DEPOT_VALIDE:
      if (!validation1_detectee && digitalRead(PIN_IR_VALIDATION_1) == LOW) {
        validation1_detectee = true;
      }
      if (validation1_detectee && !validation2_detectee && digitalRead(PIN_IR_VALIDATION_2) == LOW) {
        validation2_detectee = true;
      }
      
      if (validation1_detectee && validation2_detectee && !bouteille_validee) {
        bouteille_validee = true;
        mqttClient.publish("borne/validation", "OK");
      }
      
      if (millis() - timerEtat > 3000) {
        piloterMoteur(0);
        portENTER_CRITICAL(&myMux);
        if (bouteille_validee) {
          currentState = BORNE_PRETE;
        } else {
          currentState = DEPOT_REFUSE;
          timerEtat = millis();
          piloterMoteur(-1);
        }
        validation1_detectee = false; validation2_detectee = false; bouteille_validee = false;
        portEXIT_CRITICAL(&myMux);
      }
      break;

    case DEPOT_REFUSE:
      if (millis() - timerEtat > 3000) {
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