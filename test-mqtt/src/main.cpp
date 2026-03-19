#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h> 
#include <FastLED.h>
#include <Wire.h>
#include "esp_system.h"

// ================= CONFIGURATION MATÉRIELLE =================
#define W5500_CS_PIN 10   
#define LED_PIN 7         
#define NUM_LEDS 27
#define SCANNER_RX_PIN 17 
#define SCANNER_TX_PIN 18 

#define SRF02_ADDR 0x70
#define SDA_PIN_sfr02 8
#define SCL_PIN_sfr02 9

#define PIN_IR_ENTREE 6       
#define PIN_IR_VALIDATION_1 2 
#define PIN_IR_VALIDATION_2 3 

#define HAUTEUR_BAC_VIDE 80  
#define HAUTEUR_BAC_PLEIN 20 

#define MOTEUR_DIR1 20 
#define MOTEUR_DIR2 21 

const byte COMMAND_TRIGGER[] = {0x7E, 0x00, 0x08, 0x01, 0x00, 0x02, 0x01, 0xAB, 0xCD};

HardwareSerial ScannerSerial(2);
CRGB leds[NUM_LEDS];

byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED }; 
IPAddress brokerIP(192, 168, 1, 100); 

EthernetClient ethClient;
PubSubClient mqttClient(ethClient);

portMUX_TYPE myMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool interruptionEntree = false;

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

class MoteurConvoyeur {
  private:
    int pin1, pin2;
  public:
    MoteurConvoyeur(int p1, int p2) : pin1(p1), pin2(p2) {}
    void init() { pinMode(pin1, OUTPUT); pinMode(pin2, OUTPUT); stopper(); }
    void avancer() { Serial.println(">>> MOTEUR : AVANT"); digitalWrite(pin1, HIGH); digitalWrite(pin2, LOW); }
    void reculer() { Serial.println(">>> MOTEUR : ARRIÈRE"); digitalWrite(pin1, LOW); digitalWrite(pin2, HIGH); }
    void stopper() { digitalWrite(pin1, LOW); digitalWrite(pin2, LOW); }
};

class CapteurUltrason {
  private:
    int adresseI2C;
  public:
    CapteurUltrason(int addr) : adresseI2C(addr) {}
    int lireDistance() {
      Wire.beginTransmission(adresseI2C); Wire.write(0x00); Wire.write(0x51); Wire.endTransmission();
      delay(70); 
      Wire.beginTransmission(adresseI2C); Wire.write(0x02); Wire.endTransmission();
      Wire.requestFrom(adresseI2C, 2);
      if (Wire.available() >= 2) return (Wire.read() << 8) | Wire.read();
      return -1; 
    }
    int calculerTauxRemplissage(int distance) {
      if (distance >= HAUTEUR_BAC_VIDE) return 0;
      if (distance <= HAUTEUR_BAC_PLEIN) return 100;
      return ((HAUTEUR_BAC_VIDE - distance) * 100) / (HAUTEUR_BAC_VIDE - HAUTEUR_BAC_PLEIN);
    }
};

class ControllerCrystaRecycle {
  private:
    State currentState;
    unsigned long timerEtat;
    bool clignotementState;
    unsigned long lastMQTTReconnect;
    
    // Suivi strict anti-fraude
    int etapeMax; 
    bool fraudeDetectee;
    bool bouteille_validee;
    bool commandeScanEnvoyee;
    int nombreBouteilles;

    unsigned long timerPerteVal1;
    unsigned long timerPerteVal2;

    MoteurConvoyeur moteur;
    CapteurUltrason capteurBac;

    String nettoyerCode(String brut) {
      String propre = "";
      for (int i = 0; i < brut.length(); i++) if (isDigit(brut[i])) propre += brut[i];
      return propre;
    }

    void resetTracking() {
      etapeMax = 0;
      fraudeDetectee = false; 
      bouteille_validee = false; 
      commandeScanEnvoyee = false;
      timerPerteVal1 = 0; 
      timerPerteVal2 = 0;
    }

    // Lisse les trous optiques du plastique transparent (250ms de tolérance)
    bool capteurVoitBouteille(int pin, unsigned long &timerPerte) {
      bool voitObjet = (digitalRead(pin) == HIGH); // Logique matérielle (PULLDOWN)
      if (voitObjet) {
        timerPerte = millis(); 
        return true; 
      } else {
        if (millis() - timerPerte < 50) return true; 
        else return false; 
      }
    }

    void gererReseau() {
      if (!mqttClient.connected()) {
        if (millis() - lastMQTTReconnect > 5000) {
          lastMQTTReconnect = millis();
          if (mqttClient.connect("Borne_Ecobox")) Serial.println(">>> ✅ Connecté au Broker MQTT !");
        }
      } else mqttClient.loop();
    }

    void actualiserLeds() {
      static unsigned long lastBlink = 0;
      if (millis() - lastBlink > 300) { clignotementState = !clignotementState; lastBlink = millis(); }
      switch (currentState) {
        case BORNE_PRETE: case ATTENTE_CHUTE: fill_solid(leds, NUM_LEDS, CRGB::Green); break;
        case BOUTEILLE_DETECTEE: fill_solid(leds, NUM_LEDS, CRGB::Blue); break;
        case DEPOT_VALIDE: fill_solid(leds, NUM_LEDS, clignotementState ? CRGB::Blue : CRGB::Black); break;
        case ATTENTE_REJET: case REJET_EN_COURS: fill_solid(leds, NUM_LEDS, clignotementState ? CRGB::Red : CRGB::Black); break;
        case BAC_PLEIN: fill_solid(leds, NUM_LEDS, CRGB::Red); break;
      }
      FastLED.show();
    }

  public:
    ControllerCrystaRecycle() : moteur(MOTEUR_DIR1, MOTEUR_DIR2), capteurBac(SRF02_ADDR) {
      currentState = BORNE_PRETE;
      clignotementState = false;
      lastMQTTReconnect = 0;
      nombreBouteilles = 0; 
      resetTracking();
    }

    void init() {
      moteur.init();
      pinMode(PIN_IR_ENTREE, INPUT_PULLUP);
      pinMode(PIN_IR_VALIDATION_1, INPUT);
      pinMode(PIN_IR_VALIDATION_2, INPUT);
    }

    void executerCycle() {
      actualiserLeds();
      gererReseau();

      if (currentState != BORNE_PRETE) {
        portENTER_CRITICAL(&myMux); interruptionEntree = false; portEXIT_CRITICAL(&myMux);
      }

      switch (currentState) {
        case BORNE_PRETE: {
          moteur.stopper();
          static unsigned long lastIdleCheck = 0;
          if (millis() - lastIdleCheck > 10000) {
            int dist = capteurBac.lireDistance();
            if (dist > 0 && dist <= HAUTEUR_BAC_PLEIN) {
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
            while (ScannerSerial.available()) ScannerSerial.read();
            ScannerSerial.write(0x00); delay(50); 
            ScannerSerial.write(COMMAND_TRIGGER, sizeof(COMMAND_TRIGGER));
            commandeScanEnvoyee = true; timerEtat = millis();
          }

          if (millis() - timerEtat > 3000) {
            Serial.println(">>> TIMEOUT SCAN -> Rejet");
            if (mqttClient.connected()) mqttClient.publish("ecobox/rejet", "ERREUR_SCAN");
            currentState = ATTENTE_REJET;
            timerEtat = millis();
            moteur.stopper();
            commandeScanEnvoyee = false; 
          } 
          else if (ScannerSerial.available()) {
            String codePropre = nettoyerCode(ScannerSerial.readStringUntil('\r'));
            if (codePropre.length() > 10) {
              Serial.print(">>> CODE VALIDE : "); Serial.println(codePropre);
              if (mqttClient.connected()) mqttClient.publish("ecobox/print", codePropre.c_str());
              resetTracking();
              currentState = DEPOT_VALIDE;
              timerEtat = millis();
              moteur.avancer();
            }
          }
          break;

        case DEPOT_VALIDE: {
          bool ir1_coupe = capteurVoitBouteille(PIN_IR_VALIDATION_1, timerPerteVal1);
          bool ir2_coupe = capteurVoitBouteille(PIN_IR_VALIDATION_2, timerPerteVal2);
          
          // Conversion binaire : 0=Vide, 2=VAL1, 3=VAL1+VAL2, 1=VAL2
          int etatBinaire = (ir1_coupe ? 2 : 0) | (ir2_coupe ? 1 : 0);

          // MACHINE À ÉTATS STRICTE
          if (etatBinaire == 2) { // 01 (Seul VAL1 coupé)
              if (etapeMax == 0) etapeMax = 1; // Avancée normale
              else if (etapeMax >= 2) fraudeDetectee = true; // CAS 1 & 3 : Retrait depuis le milieu ou la fin
          }
          else if (etatBinaire == 3) { // 00 (Les deux coupés, milieu)
              if (etapeMax == 1) etapeMax = 2; // Avancée normale
              else if (etapeMax >= 3) fraudeDetectee = true; // CAS 1 : Recul depuis la sortie vers le milieu
          }
          else if (etatBinaire == 1) { // 10 (Seul VAL2 coupé, sortie)
              if (etapeMax == 1 || etapeMax == 2) etapeMax = 3; // Avancée normale
          }
          else if (etatBinaire == 0) { // 11 (Tapis vide)
              // VICTOIRE
              if (etapeMax == 3 || etapeMax == 2) { 
                 bouteille_validee = true;
                 Serial.println(">>> Bouteille a quitté le tapis. Chute en cours...");
                 currentState = ATTENTE_CHUTE;
                 timerEtat = millis();
              }
              // CAS 2 : Retrait total depuis l'entrée
              else if (etapeMax == 1) {
                 fraudeDetectee = true; 
              }
          }

          // GESTION DU REJET POUR FRAUDE
          if (fraudeDetectee) {
             Serial.println(">>> 🛑 FRAUDE DÉTECTÉE (Mouvement anormal) !");
             if (mqttClient.connected()) mqttClient.publish("ecobox/rejet", "FRAUDE");
             moteur.stopper();
             currentState = ATTENTE_REJET;
             timerEtat = millis();
          }

          if (millis() - timerEtat > 15000) {
             Serial.println(">>> TIMEOUT GLOBAL -> Bouteille coincée");
             if (mqttClient.connected()) mqttClient.publish("ecobox/rejet", "TIMEOUT_CONVOYEUR");
             moteur.stopper();
             currentState = ATTENTE_REJET;
             timerEtat = millis();
          }
          break;
        }

        case ATTENTE_CHUTE: {
          // CAS 4 : Surveillance pendant la chute de 500ms
          bool ir1_chute = capteurVoitBouteille(PIN_IR_VALIDATION_1, timerPerteVal1);
          bool ir2_chute = capteurVoitBouteille(PIN_IR_VALIDATION_2, timerPerteVal2);
          
          if (ir1_chute || ir2_chute) {
             Serial.println(">>> 🛑 FRAUDE EXTRÊME (Cas 4) : Bouteille remontée avec une ficelle !");
             if (mqttClient.connected()) mqttClient.publish("ecobox/rejet", "FRAUDE");
             moteur.stopper();
             currentState = ATTENTE_REJET;
             timerEtat = millis();
          }
          else if (millis() - timerEtat > 500) { 
            Serial.println(">>> 🏁 CHUTE TERMINÉE PROPREMENT.");
            moteur.stopper();

            if (mqttClient.connected()) {
              mqttClient.publish("ecobox/accepte", "OK");
            }
            
            nombreBouteilles++; 
            Serial.print(">>> 🍾 BOUTEILLES RECYCLÉES : "); Serial.println(nombreBouteilles);
            if (mqttClient.connected()) mqttClient.publish("ecobox/compteur", String(nombreBouteilles).c_str());
            
            int dist = capteurBac.lireDistance();
            if (dist == -1) {
              currentState = BORNE_PRETE;
            } else {
              int taux = capteurBac.calculerTauxRemplissage(dist);
              if (mqttClient.connected()) mqttClient.publish("ecobox/niveau", String(taux).c_str());

              if (taux >= 95) {
                Serial.println(">>> ⚠️ ALERTE : BAC PLEIN !");
                currentState = BAC_PLEIN;
                if (mqttClient.connected()) mqttClient.publish("ecobox/alerte", "BAC_PLEIN");
              } else {
                currentState = BORNE_PRETE;
              }
            }
          }
          break;
        }

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
              Serial.println(">>> ✅ BAC VIDÉ. Déverrouillage.");
              currentState = BORNE_PRETE;
            }
            timerEtat = millis();
          }
          break;
      }
    }
};

ControllerCrystaRecycle maBorne;

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN_sfr02, SCL_PIN_sfr02);
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
  ScannerSerial.begin(9600, SERIAL_8N1, SCANNER_RX_PIN, SCANNER_TX_PIN);
  maBorne.init();
  attachInterrupt(digitalPinToInterrupt(PIN_IR_ENTREE), ISR_Entree, FALLING);
  Ethernet.init(W5500_CS_PIN); 
  if (Ethernet.begin(mac) == 0) Serial.println(">>> ❌ Échec DHCP");
  else { Serial.print(">>> ✅ Connecté IP : "); Serial.println(Ethernet.localIP()); }
  mqttClient.setServer(brokerIP, 1883);
  Serial.println("=== SYSTÈME DÉMARRÉ (ANTI-FRAUDE ULTIME) ===");
}

void loop() {
  maBorne.executerCycle();
}