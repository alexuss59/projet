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

// =================================================================
// MACHINE À ÉTATS
// =================================================================
enum State {
  ATTENTE_UTILISATEUR, // Veille logicielle (Verrouillé)
  BORNE_PRETE,         // Session active (Prêt à détecter)
  BOUTEILLE_DETECTEE,
  DEPOT_VALIDE,
  ATTENTE_CHUTE,
  ATTENTE_REJET,
  REJET_EN_COURS,
  BAC_PLEIN
};

// =================================================================
// CLASSES DE SERVICES
// =================================================================

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

class MqttService {
  private:
    unsigned long lastReconnect = 0;
    void connecter() {
      if (millis() - lastReconnect > 5000) {
        lastReconnect = millis();
        if (mqttClient.connect("Borne_Ecobox")) {
          Serial.println(">>> ✅ MQTT Connecté");
          mqttClient.subscribe("ecobox/action");  // Topics d'ordres
          mqttClient.subscribe("ecobox/action"); // Topics de session
        }
      }
    }
  public:
    void publier(String topic, String message) {
      if (mqttClient.connected()) mqttClient.publish(topic.c_str(), message.c_str());
    }
    void loop() {
      if (!mqttClient.connected()) connecter();
      else mqttClient.loop();
    }
};

class LedController {
  private:
    bool clignotement = false;
  public:
    void afficherEtat(State etat) {
      static unsigned long lastBlink = 0;
      if (millis() - lastBlink > 300) { clignotement = !clignotement; lastBlink = millis(); }
      switch (etat) {
        case ATTENTE_UTILISATEUR: fill_solid(leds, NUM_LEDS, CRGB::Green); break;
        case BORNE_PRETE: case ATTENTE_CHUTE: fill_solid(leds, NUM_LEDS, CRGB::Green); break;
        case BOUTEILLE_DETECTEE: fill_solid(leds, NUM_LEDS, CRGB::Blue); break;
        case DEPOT_VALIDE: fill_solid(leds, NUM_LEDS, clignotement ? CRGB::Blue : CRGB::Black); break;
        case ATTENTE_REJET: case REJET_EN_COURS: fill_solid(leds, NUM_LEDS, clignotement ? CRGB::Red : CRGB::Black); break;
        case BAC_PLEIN: fill_solid(leds, NUM_LEDS, CRGB::Red); break;
      }
      FastLED.show();
    }
};

class ScannerService {
  private:
    bool commandeEnvoyee = false;
  public:
    void reinitialiser() { commandeEnvoyee = false; while (ScannerSerial.available()) ScannerSerial.read(); }
    void envoyerCommande() {
      if (!commandeEnvoyee) {
        while (ScannerSerial.available()) ScannerSerial.read();
        ScannerSerial.write(0x00); delay(50);
        ScannerSerial.write(COMMAND_TRIGGER, sizeof(COMMAND_TRIGGER));
        commandeEnvoyee = true;
      }
    }
    String lireCode() {
      if (ScannerSerial.available()) {
        String res = ScannerSerial.readStringUntil('\r');
        if (res.length() > 10) return res;
      }
      return "";
    }
};

// =================================================================
// ORCHESTRATEUR : ControleurCycle
// =================================================================
class ControleurCycle {
  private:
    State etatActuel;
    CapteurUltrason ultrason;
    MoteurConvoyeur moteur;
    MqttService mqtt;
    LedController ledCtrl;
    ScannerService scanner;

    unsigned long timerEtat;
    int etapeMax;
    unsigned long tempsDeRejet;
    unsigned long timerPerteVal1, timerPerteVal2;

    void resetTracking() { etapeMax = 0; tempsDeRejet = 11000; scanner.reinitialiser(); }
    bool capteurVoitBouteille(int pin, unsigned long &timer) {
      if (digitalRead(pin) == HIGH) { timer = millis(); return true; }
      return (millis() - timer < 50);
    }

  public:
    ControleurCycle() : ultrason(SRF02_ADDR), moteur(MOTEUR_DIR1, MOTEUR_DIR2), etatActuel(ATTENTE_UTILISATEUR) {}

    void init() {
      moteur.init();
      pinMode(PIN_IR_ENTREE, INPUT_PULLUP);
      pinMode(PIN_IR_VALIDATION_1, INPUT);
      pinMode(PIN_IR_VALIDATION_2, INPUT);
    }

    // Gestion des messages de session (START / STOP)
    void gererSession(String cmd) {
      if (cmd == "ACTIVER_BORNE" && etatActuel == ATTENTE_UTILISATEUR) {
        Serial.println(">>> SESSION ACTIVÉE : Prêt.");
        etatActuel = BORNE_PRETE;
        portENTER_CRITICAL(&myMux); interruptionEntree = false; portEXIT_CRITICAL(&myMux);
      } 
      else if (cmd == "DESACTIVER_BORNE") {
        Serial.println(">>> SESSION TERMINÉE : Retour en veille.");
        moteur.stopper();
        resetTracking();
        etatActuel = ATTENTE_UTILISATEUR; // Verrouillage immédiat
      }
    }

    // Arrêt d'urgence ou Retry
    void forcerActionServeur() {
      Serial.println(">>> 🛑 ACTION SERVEUR : STOP_TAPIS.");
      moteur.stopper();
      resetTracking();
      etatActuel = ATTENTE_UTILISATEUR;
    }

    void executerCycle() {
      ledCtrl.afficherEtat(etatActuel);
      mqtt.loop();

      // Si en veille, on ignore tout et on force l'arrêt moteur
      if (etatActuel == ATTENTE_UTILISATEUR) {
        moteur.stopper();
        portENTER_CRITICAL(&myMux); interruptionEntree = false; portEXIT_CRITICAL(&myMux);
        return;
      }

      switch (etatActuel) {
        case BORNE_PRETE:
          moteur.stopper();
          portENTER_CRITICAL(&myMux);
          if (interruptionEntree) {
            interruptionEntree = false; scanner.reinitialiser();
            etatActuel = BOUTEILLE_DETECTEE; timerEtat = millis();
          }
          portEXIT_CRITICAL(&myMux);
          break;

        case BOUTEILLE_DETECTEE:
          scanner.envoyerCommande();
          {
            String code = scanner.lireCode();
            if (code.length() > 0) {
              mqtt.publier("ecobox/print", code); resetTracking();
              etatActuel = DEPOT_VALIDE; moteur.avancer(); timerEtat = millis();
            }
          }
          if (millis() - timerEtat > 3000) {
            tempsDeRejet = 3000; etatActuel = ATTENTE_REJET; timerEtat = millis();
          }
          break;

        case DEPOT_VALIDE:
          {
            bool ir1 = capteurVoitBouteille(PIN_IR_VALIDATION_1, timerPerteVal1);
            bool ir2 = capteurVoitBouteille(PIN_IR_VALIDATION_2, timerPerteVal2);
            int bin = (ir1 ? 2 : 0) | (ir2 ? 1 : 0);
            if (bin == 2 && etapeMax == 0) etapeMax = 1;
            else if (bin == 3 && etapeMax == 1) etapeMax = 2;
            else if (bin == 1 && etapeMax >= 1) etapeMax = 3;
            else if (bin == 0 && etapeMax >= 2) {
              etatActuel = ATTENTE_CHUTE; timerEtat = millis();
            } else if (bin == 0 && etapeMax == 1) {
              tempsDeRejet = 11000; etatActuel = ATTENTE_REJET; timerEtat = millis();
            }
          }
          break;

        case ATTENTE_CHUTE:
          if (millis() - timerEtat > 500) {
            moteur.stopper();
            mqtt.publier("ecobox/accepte", "OK");
            etatActuel = BORNE_PRETE;
          }
          break;

        case ATTENTE_REJET:
          moteur.stopper();
          if (millis() - timerEtat > 1000) {
            moteur.reculer(); etatActuel = REJET_EN_COURS; timerEtat = millis();
          }
          break;

        case REJET_EN_COURS:
          if (millis() - timerEtat > tempsDeRejet) {
            moteur.stopper(); etatActuel = BORNE_PRETE;
          }
          break;
      }
    }
};

// =================================================================
// SYSTEM MANAGER & CALLBACKS
// =================================================================
void receptionOrdreServeur(char* topic, byte* payload, unsigned int length);
ControleurCycle controleur;

class SystemManager {
  public:
    void setup() {
      Serial.begin(115200);
      Wire.begin(SDA_PIN_sfr02, SCL_PIN_sfr02);
      FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
      ScannerSerial.begin(9600, SERIAL_8N1, SCANNER_RX_PIN, SCANNER_TX_PIN);
      controleur.init();
      attachInterrupt(digitalPinToInterrupt(PIN_IR_ENTREE), ISR_Entree, FALLING);
      Ethernet.init(W5500_CS_PIN); Ethernet.begin(mac);
      mqttClient.setServer(brokerIP, 1883); mqttClient.setCallback(receptionOrdreServeur);
    }
    void loop() { controleur.executerCycle(); }
};

SystemManager systeme;

void receptionOrdreServeur(char* topic, byte* payload, unsigned int length) {
  String msg = "";
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
  String top = String(topic);

  if (top == "ecobox/action" && msg == "STOP_TAPIS") {
    controleur.forcerActionServeur();
  } 
  else if (top == "ecobox/action") {
    controleur.gererSession(msg);
  }
}

void setup() { systeme.setup(); }
void loop() { systeme.loop(); }