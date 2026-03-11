#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h> 
#include <FastLED.h>
#include <Wire.h>
#include "esp_system.h"

// ================= CONFIGURATION MATÉRIELLE =================
#define W5500_CS_PIN 10   // Pin CS du module Ethernet
#define LED_PIN 7         // vert
#define NUM_LEDS 27
#define SCANNER_RX_PIN 17 // violet tx to rx
#define SCANNER_TX_PIN 18 // bleu rx to tx

// I2C & Capteurs
#define SRF02_ADDR 0x70
#define SDA_PIN_sfr02 8
#define SCL_PIN_sfr02 9

#define PIN_IR_ENTREE 6       // blanc
#define PIN_IR_VALIDATION_1 2 // bleu
#define PIN_IR_VALIDATION_2 3 // jaune

// --- CONFIGURATION DU BAC (En centimètres) ---
#define HAUTEUR_BAC_VIDE 80  // Distance mesurée quand le bac vide
#define HAUTEUR_BAC_PLEIN 20 // Distance mesurée quand les flocons touchent (mini 12cm)

// Moteur
#define MOTEUR_DIR1 20 // jaune
#define MOTEUR_DIR2 21 // vert

const byte COMMAND_TRIGGER[] = {0x7E, 0x00, 0x08, 0x01, 0x00, 0x02, 0x01, 0xAB, 0xCD};

// ================= OBJETS GLOBAUX =================
HardwareSerial ScannerSerial(2);
CRGB leds[NUM_LEDS];

byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED }; // Adresse MAC esp32
IPAddress brokerIP(192, 168, 1, 100); // L'IP du Raspberry Pi

EthernetClient ethClient;
PubSubClient mqttClient(ethClient);

portMUX_TYPE myMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool interruptionEntree = false;

// Interruption matérielle
void IRAM_ATTR ISR_Entree() {
  portENTER_CRITICAL_ISR(&myMux);
  interruptionEntree = true;
  portEXIT_CRITICAL_ISR(&myMux);
}

enum State {
  BORNE_PRETE,
  BOUTEILLE_DETECTEE,
  DEPOT_VALIDE,
  ATTENTE_CHUTE,
  ATTENTE_REJET,
  REJET_EN_COURS,
  BAC_PLEIN
};

// =========================================================
//                   CLASSES MATÉRIELLES
// =========================================================

class MoteurConvoyeur {
  private:
    int pin1, pin2;
  public:
    MoteurConvoyeur(int p1, int p2) : pin1(p1), pin2(p2) {}
    
    void init() {
      pinMode(pin1, OUTPUT);
      pinMode(pin2, OUTPUT);
      stopper();
    }
    
    void avancer() {
      Serial.println(">>> MOTEUR : AVANT");
      digitalWrite(pin1, HIGH);
      digitalWrite(pin2, LOW);
    }
    
    void reculer() {
      Serial.println(">>> MOTEUR : ARRIÈRE");
      digitalWrite(pin1, LOW);
      digitalWrite(pin2, HIGH);
    }
    
    void stopper() {
      // Pour ne pas spammer le moniteur série à chaque tour de boucle,
      // on n'affiche le "STOP" que si le moteur tournait avant (optionnel)
      digitalWrite(pin1, LOW);
      digitalWrite(pin2, LOW);
    }
};

class CapteurUltrason {
  private:
    int adresseI2C;
  public:
    CapteurUltrason(int addr) : adresseI2C(addr) {}

    int lireDistance() {
      Wire.beginTransmission(adresseI2C);
      Wire.write(0x00);
      Wire.write(0x51);
      Wire.endTransmission();
      delay(70); 

      Wire.beginTransmission(adresseI2C);
      Wire.write(0x02);
      Wire.endTransmission();

      Wire.requestFrom(adresseI2C, 2);
      if (Wire.available() >= 2) {
        byte high = Wire.read();
        byte low = Wire.read();
        return (high << 8) | low;
      }
      return -1; 
    }

    int calculerTauxRemplissage(int distance) {
      if (distance >= HAUTEUR_BAC_VIDE) return 0;
      if (distance <= HAUTEUR_BAC_PLEIN) return 100;
      return ((HAUTEUR_BAC_VIDE - distance) * 100) / (HAUTEUR_BAC_VIDE - HAUTEUR_BAC_PLEIN);
    }
};

// =========================================================
//             CLASSE PRINCIPALE : LE CONTRÔLEUR
// =========================================================

class ControleurEcoBox {
  private:
    State currentState;
    unsigned long timerEtat;
    bool clignotementState;
    unsigned long lastMQTTReconnect;
    
    // Suivi de la bouteille
    bool aVuEntree, aVuSortie, fraudeDetectee, bouteille_validee;
    bool commandeScanEnvoyee;

    // Objets matériels
    MoteurConvoyeur moteur;
    CapteurUltrason capteurBac;

    String nettoyerCode(String brut) {
      String propre = "";
      for (int i = 0; i < brut.length(); i++) {
        if (isDigit(brut[i])) propre += brut[i];
      }
      return propre;
    }

    void resetTracking() {
      aVuEntree = false;
      aVuSortie = false;
      fraudeDetectee = false;
      bouteille_validee = false;
      commandeScanEnvoyee = false;
    }

    void gererReseau() {
      if (!mqttClient.connected()) {
        if (millis() - lastMQTTReconnect > 5000) {
          lastMQTTReconnect = millis();
          Serial.println(">>> Tentative de connexion MQTT...");
          if (mqttClient.connect("Borne_Ecobox")) {
            Serial.println(">>> ✅ Connecté au Broker MQTT !");
          } else {
            Serial.print(">>> ❌ Échec MQTT, code erreur : ");
            Serial.println(mqttClient.state());
          }
        }
      } else {
        mqttClient.loop();
      }
    }

    void actualiserLeds() {
      static unsigned long lastBlink = 0;
      if (millis() - lastBlink > 300) {
        clignotementState = !clignotementState;
        lastBlink = millis();
      }

      switch (currentState) {
        case BORNE_PRETE:
        case ATTENTE_CHUTE:
          fill_solid(leds, NUM_LEDS, CRGB::Green); break;
        case BOUTEILLE_DETECTEE: 
          fill_solid(leds, NUM_LEDS, CRGB::Blue); break;
        case DEPOT_VALIDE: 
          fill_solid(leds, NUM_LEDS, clignotementState ? CRGB::Blue : CRGB::Black); break;
        case ATTENTE_REJET:
        case REJET_EN_COURS: 
          fill_solid(leds, NUM_LEDS, clignotementState ? CRGB::Red : CRGB::Black); break;
        case BAC_PLEIN: 
          fill_solid(leds, NUM_LEDS, CRGB::Red); break;
      }
      FastLED.show();
    }

  public:
    ControleurEcoBox() : 
      moteur(MOTEUR_DIR1, MOTEUR_DIR2), 
      capteurBac(SRF02_ADDR) 
    {
      currentState = BORNE_PRETE;
      clignotementState = false;
      lastMQTTReconnect = 0;
      resetTracking();
    }

    void init() {
      moteur.init();
      pinMode(PIN_IR_ENTREE, INPUT_PULLUP);
      pinMode(PIN_IR_VALIDATION_1, INPUT_PULLUP);
      pinMode(PIN_IR_VALIDATION_2, INPUT_PULLUP);
    }

    void executerCycle() {
      actualiserLeds();
      gererReseau();

      if (currentState != BORNE_PRETE) {
        portENTER_CRITICAL(&myMux);
        interruptionEntree = false; 
        portEXIT_CRITICAL(&myMux);
      }
      // =========================================================

      switch (currentState) {
        case BORNE_PRETE: {
          moteur.stopper();
          
          // Vérification veille (10s)
          static unsigned long lastIdleCheck = 0;
          if (millis() - lastIdleCheck > 10000) {
            int dist = capteurBac.lireDistance();
            if (dist > 0 && dist <= HAUTEUR_BAC_PLEIN) {
              Serial.println(">>> ⚠️ ALERTE VEILLE : Bac plein détecté avant dépôt !");
              currentState = BAC_PLEIN;
              if (mqttClient.connected()) mqttClient.publish("ecobox/alerte", "BAC_PLEIN_VEILLE");
            }
            lastIdleCheck = millis();
          }

          portENTER_CRITICAL(&myMux);
          if (interruptionEntree && currentState == BORNE_PRETE) {
            interruptionEntree = false;
            currentState = BOUTEILLE_DETECTEE;
            timerEtat = millis();
            while (ScannerSerial.available()) ScannerSerial.read();
            Serial.println("\n>>> BOUTEILLE DÉTECTÉE");
          }
          portEXIT_CRITICAL(&myMux);
          break;
        }

        case BOUTEILLE_DETECTEE:
          if (!commandeScanEnvoyee) {
            // 1. On VIDE la mémoire du scanner avant de lui parler (C'était ça le bug !)
            while (ScannerSerial.available()) ScannerSerial.read();
            
            // 2. On le réveille
            ScannerSerial.write(0x00);
            delay(50); // Marge de sécurité augmentée à 50ms pour être sûr qu'il est prêt
            
            // 3. On envoie l'ordre de scan
            ScannerSerial.write(COMMAND_TRIGGER, sizeof(COMMAND_TRIGGER));
            commandeScanEnvoyee = true;
            timerEtat = millis();
          }

          if (millis() - timerEtat > 3000) {
            Serial.println(">>> TIMEOUT SCAN -> Rejet (Aucun code lu en 3s)");
            currentState = ATTENTE_REJET;
            timerEtat = millis();
            moteur.stopper();
            commandeScanEnvoyee = false; 
          } 
          else if (ScannerSerial.available()) {
            // Lecture brute pour pouvoir débugger si jamais il lit mal
            String codeBrut = ScannerSerial.readStringUntil('\r');
            Serial.print(">>> [DEBUG SCANNER] Brut lu : "); Serial.println(codeBrut);
            
            String codePropre = nettoyerCode(codeBrut);
            
            if (codePropre.length() > 10) {
              Serial.print(">>> CODE VALIDE : "); Serial.println(codePropre);
              
              if (mqttClient.connected()) {
                mqttClient.publish("ecobox/print", codePropre.c_str());
                Serial.println(">>> 📡 Code envoyé au serveur (MQTT)");
              } else {
                Serial.println(">>> ⚠️ Impossible d'envoyer le code (Non connecté)");
              }
              
              resetTracking();
              currentState = DEPOT_VALIDE;
              timerEtat = millis();
              moteur.avancer();
            }
          }
          break;

        case DEPOT_VALIDE: {
          byte ir1 = !digitalRead(PIN_IR_VALIDATION_1);
          byte ir2 = !digitalRead(PIN_IR_VALIDATION_2);
          int etatBinaire = (ir1 << 1) | ir2;

          if (etatBinaire == 1) { aVuEntree = true; if(aVuSortie) fraudeDetectee = true; }
          if (etatBinaire == 2) { if(aVuEntree) aVuSortie = true; }

          if (etatBinaire == 3 && aVuEntree && aVuSortie && !fraudeDetectee && !bouteille_validee) {
             bouteille_validee = true;
             Serial.println(">>> SUCCÈS ! Passage en mode chute...");
             currentState = ATTENTE_CHUTE;
             timerEtat = millis();
          } 
          else if (etatBinaire == 3 && aVuEntree && (!aVuSortie || fraudeDetectee)) {
             Serial.println(">>> FRAUDE DÉTECTÉE -> Pause Rouge");
             moteur.stopper();
             currentState = ATTENTE_REJET;
             timerEtat = millis();
          }

          if (millis() - timerEtat > 11000) {
             Serial.println(">>> TIMEOUT GLOBAL -> Pause Rouge");
             moteur.stopper();
             currentState = ATTENTE_REJET;
             timerEtat = millis();
          }
          break;
        }

        case ATTENTE_CHUTE:
          if (millis() - timerEtat > 500) {
            Serial.println(">>> Chute de la bouteille terminée.");
            moteur.stopper();
            
            Serial.println(">>> Mesure du niveau du bac...");
            int dist = capteurBac.lireDistance();
            
            if (dist == -1) {
              Serial.println(">>> ❌ ERREUR CAPTEUR SRF02 (Vérifier câblage)");
              currentState = BORNE_PRETE;
            } else {
              int taux = capteurBac.calculerTauxRemplissage(dist);
              Serial.print(">>> Distance : "); Serial.print(dist); Serial.println(" cm");
              Serial.print(">>> TAUX DE REMPLISSAGE : "); Serial.print(taux); Serial.println(" %");

              if (mqttClient.connected()) {
                mqttClient.publish("ecobox/niveau", String(taux).c_str());
                Serial.println(">>> 📡 Taux envoyé au serveur BDD (MQTT)");
              }

              if (taux >= 95) {
                Serial.println(">>> ⚠️ ALERTE : BAC PLEIN ! Verrouillage de la borne.");
                currentState = BAC_PLEIN;
                if (mqttClient.connected()) mqttClient.publish("ecobox/alerte", "BAC_PLEIN");
              } else {
                Serial.println(">>> Borne prête pour la prochaine bouteille.");
                currentState = BORNE_PRETE;
              }
            }
          }
          break;

        case ATTENTE_REJET:
          moteur.stopper();
          if (millis() - timerEtat > 1000) {
            Serial.println(">>> Fin pause. Lancement MARCHE ARRIÈRE (10s).");
            currentState = REJET_EN_COURS;
            timerEtat = millis();
            moteur.reculer();
          }
          break;

        case REJET_EN_COURS:
          // Le moteur.reculer() est déjà lancé au moment où on entre dans l'état
          if (millis() - timerEtat > 11000) {
            Serial.println(">>> Fin du rejet.");
            moteur.stopper();
            currentState = BORNE_PRETE;
          }
          break;

        case BAC_PLEIN:
          moteur.stopper();
          if (millis() - timerEtat > 5000) {
            int dist = capteurBac.lireDistance();
            if (dist > HAUTEUR_BAC_PLEIN + 5) {
              Serial.println(">>> ✅ BAC VIDÉ PAR LE RESPONSABLE. Déverrouillage.");
              currentState = BORNE_PRETE;
            }
            timerEtat = millis();
          }
          break;
      }
    }
};

// =========================================================
//                   PROGRAMME PRINCIPAL
// =========================================================

ControleurEcoBox maBorne;

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN_sfr02, SCL_PIN_sfr02);
  
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
  ScannerSerial.begin(9600, SERIAL_8N1, SCANNER_RX_PIN, SCANNER_TX_PIN);
  
  maBorne.init();
  attachInterrupt(digitalPinToInterrupt(PIN_IR_ENTREE), ISR_Entree, RISING);

  Serial.println(">>> Initialisation du réseau Ethernet W5500...");
  Ethernet.init(W5500_CS_PIN); 

  if (Ethernet.begin(mac) == 0) {
    Serial.println(">>> ❌ Échec de la configuration DHCP");
  } else {
    Serial.print(">>> ✅ Connecté au réseau. IP : ");
    Serial.println(Ethernet.localIP());
  }

  mqttClient.setServer(brokerIP, 1883);
  
  Serial.println("=== SYSTÈME DÉMARRÉ (VERSION POO) ===");
}

void loop() {
  maBorne.executerCycle();
}