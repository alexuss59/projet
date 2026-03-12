// =========================================================
//  PROJET BTS CIEL - BORNE DE COLLECTE BOUTEILLES PET
//  VERSION DEBUG MULTI-CAPTEURS — Analyse IR transparente
// =========================================================

#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>
#include <FastLED.h>
#include <Wire.h>
#include "esp_system.h"

// =========================================================
//  ⚙️ PARAMÈTRES DEBUG — MODIFIE ICI
// =========================================================

#define MODE_DEBUG_IR true
#define DEBOUNCE_DELAY_MS 50 // Filtre anti-rebond global

// =========================================================
//  CONFIGURATION MATÉRIELLE
// =========================================================

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

volatile unsigned long dernierDeclenchement = 0; 
volatile int compteurInterruptionsTotal   = 0;    
volatile int compteurInterruptionsValides = 0;   

// =========================================================
//  INTERRUPTION MATÉRIELLE (Entrée)
// =========================================================
void IRAM_ATTR ISR_Entree() {
  portENTER_CRITICAL_ISR(&myMux);
  unsigned long maintenant = millis();
  compteurInterruptionsTotal++; 
  if (maintenant - dernierDeclenchement >= DEBOUNCE_DELAY_MS) {
    interruptionEntree = true;
    dernierDeclenchement = maintenant;
    compteurInterruptionsValides++; 
  }
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
//  CLASSES MATÉRIELLES
// =========================================================
class MoteurConvoyeur {
  private:
    int pin1, pin2;
  public:
    MoteurConvoyeur(int p1, int p2) : pin1(p1), pin2(p2) {}
    void init() { pinMode(pin1, OUTPUT); pinMode(pin2, OUTPUT); stopper(); }
    void avancer() { digitalWrite(pin1, HIGH); digitalWrite(pin2, LOW); }
    void reculer() { digitalWrite(pin1, LOW); digitalWrite(pin2, HIGH); }
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

// =========================================================
//  FONCTION DEBUG IR MULTI-CAPTEURS
//  Analyse indépendante des bounces pour Entrée, Val1 et Val2
// =========================================================
void debugIR() {
  static byte lastEtats = 255; 
  static unsigned long dernierChg[3] = {0, 0, 0}; 
  static int cptBounces[3] = {0, 0, 0}; 
  static unsigned long debutBounce[3] = {0, 0, 0}; 
  static byte lastIr[3] = {0, 0, 0}; 

  byte ir[3];
  ir[0] = !digitalRead(PIN_IR_ENTREE);
  ir[1] = !digitalRead(PIN_IR_VALIDATION_1);
  ir[2] = !digitalRead(PIN_IR_VALIDATION_2);

  byte etatsActuels = (ir[0] << 2) | (ir[1] << 1) | ir[2];

  if (etatsActuels != lastEtats) {
    unsigned long maintenant = millis();

    for (int i = 0; i < 3; i++) {
      if (ir[i] != lastIr[i]) {
        unsigned long duree = maintenant - dernierChg[i];
        String nomCapteur = (i == 0) ? "ENTRÉE" : (i == 1) ? "VAL1" : "VAL2";

        if (duree < DEBOUNCE_DELAY_MS && dernierChg[i] != 0) {
          cptBounces[i]++;
          if (cptBounces[i] == 1) debutBounce[i] = dernierChg[i];
          Serial.print(" ⚡ BOUNCE ["); Serial.print(nomCapteur); Serial.print("] #"); 
          Serial.print(cptBounces[i]); Serial.print(" (Micro-coupure: "); 
          Serial.print(duree); Serial.println("ms)");
        } else {
          if (cptBounces[i] > 0) {
            Serial.print(" ✅ STABILISÉ ["); Serial.print(nomCapteur); Serial.print("] | ");
            Serial.print(cptBounces[i]); Serial.print(" rebonds sur ");
            Serial.print(maintenant - debutBounce[i]); Serial.println("ms total.");
            cptBounces[i] = 0;
          }
          // Log normal si ce n'est pas un bounce
          Serial.print("⏱ "); Serial.print(maintenant); Serial.print("ms | ");
          Serial.print(nomCapteur); Serial.print(" -> "); 
          Serial.println(ir[i] ? "Coupé (Bouteille vue)" : "Libre (Vide)");
        }
        lastIr[i] = ir[i];
        dernierChg[i] = maintenant;
      }
    }
    lastEtats = etatsActuels;
  }
}

// =========================================================
//  CLASSE PRINCIPALE : LE CONTRÔLEUR
// =========================================================
class ControleurEcoBox {
  private:
    State currentState;
    unsigned long timerEtat;
    bool clignotementState;
    unsigned long lastMQTTReconnect;
    bool aVuEntree, aVuSortie, fraudeDetectee, bouteille_validee;
    bool commandeScanEnvoyee;
    int nombreBouteilles;
    MoteurConvoyeur moteur;
    CapteurUltrason capteurBac;

    String nettoyerCode(String brut) {
      String propre = "";
      for (int i = 0; i < brut.length(); i++) if (isDigit(brut[i])) propre += brut[i];
      return propre;
    }

    void resetTracking() {
      aVuEntree = false; aVuSortie = false; fraudeDetectee = false;
      bouteille_validee = false; commandeScanEnvoyee = false;
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
    ControleurEcoBox() : moteur(MOTEUR_DIR1, MOTEUR_DIR2), capteurBac(SRF02_ADDR) {
      currentState = BORNE_PRETE;
      nombreBouteilles = 0;
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
            }
            lastIdleCheck = millis();
          }

          portENTER_CRITICAL(&myMux);
          if (interruptionEntree && currentState == BORNE_PRETE) {
            interruptionEntree = false;
            
            #if MODE_DEBUG_IR
              // BYPASS : On saute le scan et on démarre le moteur pour le test d'oscillo
              Serial.println("\n>>> 🔧 [DEBUG] Bouteille vue à l'entrée -> Démarrage direct du tapis !");
              currentState = DEPOT_VALIDE;
              timerEtat = millis();
              moteur.avancer();
            #else
              currentState = BOUTEILLE_DETECTEE;
              timerEtat = millis();
              while (ScannerSerial.available()) ScannerSerial.read();
            #endif
          }
          portEXIT_CRITICAL(&myMux);
          break;
        }

        case BOUTEILLE_DETECTEE:
          // Le code du scanner reste ici pour la prod, ignoré en mode DEBUG
          if (!commandeScanEnvoyee) {
            while (ScannerSerial.available()) ScannerSerial.read();
            ScannerSerial.write(0x00); delay(50); 
            ScannerSerial.write(COMMAND_TRIGGER, sizeof(COMMAND_TRIGGER));
            commandeScanEnvoyee = true; timerEtat = millis();
          }
          if (millis() - timerEtat > 3000) {
            currentState = ATTENTE_REJET; timerEtat = millis();
            moteur.stopper(); commandeScanEnvoyee = false;
          } 
          else if (ScannerSerial.available()) {
            String codePropre = nettoyerCode(ScannerSerial.readStringUntil('\r'));
            if (codePropre.length() > 10) {
              if (mqttClient.connected()) mqttClient.publish("ecobox/print", codePropre.c_str());
              resetTracking();
              currentState = DEPOT_VALIDE; timerEtat = millis();
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
             nombreBouteilles++;
             Serial.println("\n>>> 🏁 FIN DE COURSE (Bouteille validée)");
             currentState = ATTENTE_CHUTE;
             timerEtat = millis();
          } 
          #if !MODE_DEBUG_IR
          // En prod : la fraude coupe tout.
          else if (etatBinaire == 3 && aVuEntree && (!aVuSortie || fraudeDetectee)) {
             moteur.stopper();
             currentState = ATTENTE_REJET;
             timerEtat = millis();
          }
          #else
          // En DEBUG : On laisse le moteur tourner même si les rebonds génèrent une "fraude" 
          // pour pouvoir analyser les deux capteurs jusqu'au bout.
          #endif

          if (millis() - timerEtat > 11000) {
             Serial.println(">>> 🛑 TIMEOUT 11s -> Fin du test, arrêt du moteur.");
             moteur.stopper();
             currentState = BORNE_PRETE; // On revient au début directement en debug
             resetTracking();
          }
          break;
        }

        case ATTENTE_CHUTE:
          // On passe de 500ms à 2500ms (2.5 secondes) pour être sûr qu'elle a le temps de tomber
          if (millis() - timerEtat > 2500) {
            Serial.println(">>> Chute de la bouteille terminée (Moteur coupé).");
            moteur.stopper();
            currentState = BORNE_PRETE;
          }
          break;

        case ATTENTE_REJET:
          moteur.stopper();
          if (millis() - timerEtat > 1000) {
            currentState = REJET_EN_COURS; timerEtat = millis(); moteur.reculer();
          }
          break;

        case REJET_EN_COURS:
          if (millis() - timerEtat > 11000) {
            moteur.stopper(); currentState = BORNE_PRETE;
          }
          break;

        case BAC_PLEIN:
          moteur.stopper();
          if (millis() - timerEtat > 5000) {
            if (capteurBac.lireDistance() > HAUTEUR_BAC_PLEIN + 5) currentState = BORNE_PRETE;
            timerEtat = millis();
          }
          break;
      }
    }
};

ControleurEcoBox maBorne;

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN_sfr02, SCL_PIN_sfr02);
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
  ScannerSerial.begin(9600, SERIAL_8N1, SCANNER_RX_PIN, SCANNER_TX_PIN);
  maBorne.init();
  
  // CORRECTION MAJEURE ICI : FALLING au lieu de RISING
  attachInterrupt(digitalPinToInterrupt(PIN_IR_ENTREE), ISR_Entree, FALLING);

  Ethernet.init(W5500_CS_PIN);
  Ethernet.begin(mac);
  mqttClient.setServer(brokerIP, 1883);

  Serial.println("===========================================");
  Serial.println("=== SYSTÈME DÉMARRÉ - VERSION DEBUG IR ===");
  Serial.println(">>> Les bouteilles bypass le scanner et déclenchent le moteur direct !");
  Serial.println(">>> Prêt pour l'analyse à l'oscilloscope.");
  Serial.println("===========================================");
}

void loop() {
  #if MODE_DEBUG_IR
    debugIR();
  #endif

  maBorne.executerCycle();
}