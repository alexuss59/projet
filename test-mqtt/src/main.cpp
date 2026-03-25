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
// <<enumeration>> State
// =================================================================
enum State {
  BORNE_PRETE,
  BOUTEILLE_DETECTEE,
  DEPOT_VALIDE,
  ATTENTE_CHUTE,
  ATTENTE_REJET,
  REJET_EN_COURS,
  BAC_PLEIN
};

// =================================================================
// CLASSE : CapteurUltrason
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

// =================================================================
// CLASSE : DetectionBouteille
// Agrège CapteurUltrason (composition) et expose detecter()
// =================================================================
class DetectionBouteille {
  private:
    CapteurUltrason capteur; // composition

  public:
    DetectionBouteille() : capteur(SRF02_ADDR) {}

    // Détecte la présence d'une bouteille via l'interruption IR d'entrée
    bool detecter() {
      portENTER_CRITICAL(&myMux);
      bool detection = interruptionEntree;
      portEXIT_CRITICAL(&myMux);
      return detection;
    }

    // Délégation vers CapteurUltrason pour la surveillance du bac
    int lireDistanceBac() {
      return capteur.lireDistance();
    }

    int calculerTauxRemplissage(int distance) {
      return capteur.calculerTauxRemplissage(distance);
    }
};

// =================================================================
// CLASSE : StateMachine
// =================================================================
class StateMachine {
  private:
    State etatActuel;

  public:
    StateMachine() : etatActuel(BORNE_PRETE) {}

    void changerEtat(State nouvelEtat) {
      etatActuel = nouvelEtat;
    }

    State getEtat() {
      return etatActuel;
    }
};

// =================================================================
// CLASSE : MoteurConvoyeur
// =================================================================
class MoteurConvoyeur {
  private:
    int pin1, pin2;

  public:
    MoteurConvoyeur(int p1, int p2) : pin1(p1), pin2(p2) {}

    void init() { pinMode(pin1, OUTPUT); pinMode(pin2, OUTPUT); stopper(); }

    void avancer() {
      Serial.println(">>> MOTEUR : AVANT");
      digitalWrite(pin1, HIGH); digitalWrite(pin2, LOW);
    }

    void reculer() {
      Serial.println(">>> MOTEUR : ARRIÈRE");
      digitalWrite(pin1, LOW); digitalWrite(pin2, HIGH);
    }

    void stopper() { digitalWrite(pin1, LOW); digitalWrite(pin2, LOW); }
};

// =================================================================
// CLASSE : MqttService
// =================================================================
class MqttService {
  private:
    unsigned long lastReconnect;

    void connecter() {
      if (millis() - lastReconnect > 5000) {
        lastReconnect = millis();
        if (mqttClient.connect("Borne_Ecobox")) {
          Serial.println(">>> ✅ Connecté au Broker MQTT !");
          mqttClient.subscribe("ecobox/action");
        }
      }
    }

  public:
    MqttService() : lastReconnect(0) {}

    void publier(String topic, String message) {
      if (mqttClient.connected()) {
        mqttClient.publish(topic.c_str(), message.c_str());
      }
    }

    void loop() {
      if (!mqttClient.connected()) connecter();
      else mqttClient.loop();
    }

    bool estConnecte() { return mqttClient.connected(); }
};

// =================================================================
// CLASSE : LedController
// =================================================================
class LedController {
  private:
    bool clignotement;

  public:
    LedController() : clignotement(false) {}

    void afficherEtat(State etat) {
      static unsigned long lastBlink = 0;
      if (millis() - lastBlink > 300) { clignotement = !clignotement; lastBlink = millis(); }

      switch (etat) {
        case BORNE_PRETE:
        case ATTENTE_CHUTE:
          fill_solid(leds, NUM_LEDS, CRGB::Green);
          break;
        case BOUTEILLE_DETECTEE:
          fill_solid(leds, NUM_LEDS, CRGB::Blue);
          break;
        case DEPOT_VALIDE:
          fill_solid(leds, NUM_LEDS, clignotement ? CRGB::Blue : CRGB::Black);
          break;
        case ATTENTE_REJET:
        case REJET_EN_COURS:
          fill_solid(leds, NUM_LEDS, clignotement ? CRGB::Red : CRGB::Black);
          break;
        case BAC_PLEIN:
          fill_solid(leds, NUM_LEDS, CRGB::Red);
          break;
      }
      FastLED.show();
    }
};

// =================================================================
// CLASSE : ScannerService
// =================================================================
class ScannerService {
  private:
    bool commandeEnvoyee;

    String nettoyerCode(String brut) {
      String propre = "";
      for (int i = 0; i < brut.length(); i++) {
        if (isDigit(brut[i])) propre += brut[i];
      }
      return propre;
    }

  public:
    ScannerService() : commandeEnvoyee(false) {}

    void reinitialiser() {
      commandeEnvoyee = false;
      while (ScannerSerial.available()) ScannerSerial.read();
    }

    void envoyerCommande() {
      if (!commandeEnvoyee) {
        while (ScannerSerial.available()) ScannerSerial.read();
        ScannerSerial.write(0x00); delay(50);
        ScannerSerial.write(COMMAND_TRIGGER, sizeof(COMMAND_TRIGGER));
        commandeEnvoyee = true;
      }
    }

    // Retourne le code nettoyé s'il est disponible et valide, sinon ""
    String lireCode() {
      if (ScannerSerial.available()) {
        String codePropre = nettoyerCode(ScannerSerial.readStringUntil('\r'));
        if (codePropre.length() > 10) return codePropre;
      }
      return "";
    }

    bool estCommandeEnvoyee() { return commandeEnvoyee; }
};

// =================================================================
// CLASSE : ControleurCycle
// Orchestre tous les services selon la machine à états
// =================================================================
class ControleurCycle {
  private:
    StateMachine    stateMachine;
    DetectionBouteille detection;
    MoteurConvoyeur moteur;
    MqttService     mqtt;
    LedController   ledCtrl;
    ScannerService  scanner;

    unsigned long timerEtat;
    int           etapeMax;
    bool          fraudeDetectee;
    bool          bouteille_validee;
    int           nombreBouteilles;
    unsigned long tempsDeRejet;
    unsigned long timerPerteVal1;
    unsigned long timerPerteVal2;

    // ---- Helpers privés ----

    void resetTracking() {
      etapeMax        = 0;
      fraudeDetectee  = false;
      bouteille_validee = false;
      timerPerteVal1  = 0;
      timerPerteVal2  = 0;
      tempsDeRejet    = 11000;
      scanner.reinitialiser();
    }

    bool capteurVoitBouteille(int pin, unsigned long &timerPerte) {
      if (digitalRead(pin) == HIGH) { timerPerte = millis(); return true; }
      return (millis() - timerPerte < 50);
    }

    // ---- Méthodes de traitement par état ----

    void traiterEtatBornePrete() {
      moteur.stopper();

      static unsigned long lastIdleCheck = 0;
      if (millis() - lastIdleCheck > 10000) {
        int dist = detection.lireDistanceBac();
        if (dist > 0 && dist <= HAUTEUR_BAC_PLEIN) {
          stateMachine.changerEtat(BAC_PLEIN);
          mqtt.publier("ecobox/alerte", "BAC_PLEIN_VEILLE");
        }
        lastIdleCheck = millis();
      }

      portENTER_CRITICAL(&myMux);
      if (interruptionEntree) {
        interruptionEntree = false;
        scanner.reinitialiser();
        stateMachine.changerEtat(BOUTEILLE_DETECTEE);
        timerEtat = millis();
        Serial.println("\n>>> BOUTEILLE DÉTECTÉE");
      }
      portEXIT_CRITICAL(&myMux);
    }

    void traiterEtatBouteilleDetectee() {
      scanner.envoyerCommande();

      if (millis() - timerEtat > 3000) {
        Serial.println(">>> TIMEOUT SCAN -> Rejet rapide (3s)");
        mqtt.publier("ecobox/rejet", "ERREUR_SCAN");
        tempsDeRejet = 3000;
        moteur.stopper();
        scanner.reinitialiser();
        stateMachine.changerEtat(ATTENTE_REJET);
        timerEtat = millis();
      } else {
        String code = scanner.lireCode();
        if (code.length() > 0) {
          Serial.print(">>> CODE VALIDE : "); Serial.println(code);
          mqtt.publier("ecobox/print", code);
          resetTracking();
          stateMachine.changerEtat(DEPOT_VALIDE);
          timerEtat = millis();
          moteur.avancer();
        }
      }
    }

    void traiterEtatDepotValide() {
      bool ir1 = capteurVoitBouteille(PIN_IR_VALIDATION_1, timerPerteVal1);
      bool ir2 = capteurVoitBouteille(PIN_IR_VALIDATION_2, timerPerteVal2);
      int etatBinaire = (ir1 ? 2 : 0) | (ir2 ? 1 : 0);

      if      (etatBinaire == 2) { if (etapeMax == 0) etapeMax = 1; else if (etapeMax >= 2) fraudeDetectee = true; }
      else if (etatBinaire == 3) { if (etapeMax == 1) etapeMax = 2; else if (etapeMax >= 3) fraudeDetectee = true; }
      else if (etatBinaire == 1) { if (etapeMax == 1 || etapeMax == 2) etapeMax = 3; }
      else if (etatBinaire == 0) {
        if (etapeMax == 3 || etapeMax == 2) {
          bouteille_validee = true;
          Serial.println(">>> Bouteille a quitté le tapis. Chute en cours...");
          stateMachine.changerEtat(ATTENTE_CHUTE);
          timerEtat = millis();
        } else if (etapeMax == 1) {
          fraudeDetectee = true;
        }
      }

      if (fraudeDetectee) {
        Serial.println(">>> 🛑 FRAUDE DÉTECTÉE (Mouvement anormal) !");
        mqtt.publier("ecobox/rejet", "FRAUDE");
        tempsDeRejet = 11000;
        moteur.stopper();
        stateMachine.changerEtat(ATTENTE_REJET);
        timerEtat = millis();
        return;
      }

      if (millis() - timerEtat > 15000) {
        Serial.println(">>> TIMEOUT GLOBAL -> Bouteille coincée");
        mqtt.publier("ecobox/rejet", "TIMEOUT_CONVOYEUR");
        tempsDeRejet = 11000;
        moteur.stopper();
        stateMachine.changerEtat(ATTENTE_REJET);
        timerEtat = millis();
      }
    }

    void traiterEtatAttenteChute() {
      bool ir1 = capteurVoitBouteille(PIN_IR_VALIDATION_1, timerPerteVal1);
      bool ir2 = capteurVoitBouteille(PIN_IR_VALIDATION_2, timerPerteVal2);

      if (ir1 || ir2) {
        Serial.println(">>> 🛑 FRAUDE EXTRÊME : Bouteille remontée !");
        mqtt.publier("ecobox/rejet", "FRAUDE");
        tempsDeRejet = 11000;
        moteur.stopper();
        stateMachine.changerEtat(ATTENTE_REJET);
        timerEtat = millis();
        return;
      }

      if (millis() - timerEtat > 500) {
        Serial.println(">>> 🏁 CHUTE TERMINÉE PROPREMENT.");
        moteur.stopper();
        mqtt.publier("ecobox/accepte", "OK");

        nombreBouteilles++;
        Serial.print(">>> 🍾 BOUTEILLES RECYCLÉES : "); Serial.println(nombreBouteilles);
        mqtt.publier("ecobox/compteur", String(nombreBouteilles));

        int dist = detection.lireDistanceBac();
        if (dist == -1) {
          stateMachine.changerEtat(BORNE_PRETE);
        } else {
          int taux = detection.calculerTauxRemplissage(dist);
          mqtt.publier("ecobox/niveau", String(taux));
          if (taux >= 95) {
            Serial.println(">>> ⚠️ ALERTE : BAC PLEIN !");
            mqtt.publier("ecobox/alerte", "BAC_PLEIN");
            stateMachine.changerEtat(BAC_PLEIN);
          } else {
            stateMachine.changerEtat(BORNE_PRETE);
          }
        }
      }
    }

    void traiterEtatAttenteRejet() {
      moteur.stopper();
      if (millis() - timerEtat > 1000) {
        Serial.print(">>> Fin pause. Lancement MARCHE ARRIÈRE (");
        Serial.print(tempsDeRejet / 1000);
        Serial.println("s).");
        stateMachine.changerEtat(REJET_EN_COURS);
        timerEtat = millis();
        moteur.reculer();
      }
    }

    void traiterEtatRejetEnCours() {
      if (millis() - timerEtat > tempsDeRejet) {
        Serial.println(">>> Fin du rejet automatique.");
        moteur.stopper();
        stateMachine.changerEtat(BORNE_PRETE);
      }
    }

    void traiterEtatBacPlein() {
      moteur.stopper();
      if (millis() - timerEtat > 5000) {
        int dist = detection.lireDistanceBac();
        if (dist > HAUTEUR_BAC_PLEIN + 5) {
          Serial.println(">>> ✅ BAC VIDÉ. Déverrouillage.");
          stateMachine.changerEtat(BORNE_PRETE);
        }
        timerEtat = millis();
      }
    }

  public:
    ControleurCycle() : moteur(MOTEUR_DIR1, MOTEUR_DIR2) {
      timerEtat      = 0;
      nombreBouteilles = 0;
      resetTracking();
    }

    void init() {
      moteur.init();
      pinMode(PIN_IR_ENTREE,       INPUT_PULLUP);
      pinMode(PIN_IR_VALIDATION_1, INPUT);
      pinMode(PIN_IR_VALIDATION_2, INPUT);
    }

    // Appelé par le serveur via MQTT (ordre "RETRY")
    void forcerActionServeur() {
      State etat = stateMachine.getEtat();
      if (etat == ATTENTE_REJET || etat == REJET_EN_COURS) {
        Serial.println(">>> 🟢 ACTION SERVEUR : Réessai demandé. Arrêt du tapis !");
        moteur.stopper();
        resetTracking();
        stateMachine.changerEtat(BORNE_PRETE);
        timerEtat = millis();
      }
    }

    void executerCycle() {
      State etat = stateMachine.getEtat();
      ledCtrl.afficherEtat(etat);
      mqtt.loop();

      // Neutraliser l'interruption si on n'est pas en attente
      if (etat != BORNE_PRETE) {
        portENTER_CRITICAL(&myMux); interruptionEntree = false; portEXIT_CRITICAL(&myMux);
      }

      switch (etat) {
        case BORNE_PRETE:        traiterEtatBornePrete();        break;
        case BOUTEILLE_DETECTEE: traiterEtatBouteilleDetectee(); break;
        case DEPOT_VALIDE:       traiterEtatDepotValide();       break;
        case ATTENTE_CHUTE:      traiterEtatAttenteChute();      break;
        case ATTENTE_REJET:      traiterEtatAttenteRejet();      break;
        case REJET_EN_COURS:     traiterEtatRejetEnCours();      break;
        case BAC_PLEIN:          traiterEtatBacPlein();          break;
      }
    }
};

// =================================================================
// CLASSE : SystemManager
// Point d'entrée du programme, encapsule setup() et loop()
// =================================================================
class SystemManager {
  private:
    ControleurCycle controleur;

  public:
    void setup() {
      Serial.begin(115200);
      Wire.begin(SDA_PIN_sfr02, SCL_PIN_sfr02);
      FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
      ScannerSerial.begin(9600, SERIAL_8N1, SCANNER_RX_PIN, SCANNER_TX_PIN);

      controleur.init();
      attachInterrupt(digitalPinToInterrupt(PIN_IR_ENTREE), ISR_Entree, FALLING);

      Ethernet.init(W5500_CS_PIN);
      if (Ethernet.begin(mac) == 0) {
        Serial.println(">>> ❌ Échec DHCP");
      } else {
        Serial.print(">>> ✅ Connecté IP : ");
        Serial.println(Ethernet.localIP());
      }

      mqttClient.setServer(brokerIP, 1883);
      mqttClient.setCallback(receptionOrdreServeur);

      Serial.println("=== SYSTÈME DÉMARRÉ (ARCHITECTURE REFACTORISÉE) ===");
    }

    void loop() {
      controleur.executerCycle();
    }

    ControleurCycle& getControleur() { return controleur; }
};

// =================================================================
// INSTANCE GLOBALE & CALLBACKS
// =================================================================
SystemManager systeme;

void receptionOrdreServeur(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (unsigned int i = 0; i < length; i++) message += (char)payload[i];

  Serial.print(">>> Ordre serveur reçu sur [");
  Serial.print(topic);
  Serial.print("] : ");
  Serial.println(message);

  if (String(topic) == "ecobox/action" && message == "RETRY") {
    systeme.getControleur().forcerActionServeur();
  }
}

void setup() { systeme.setup(); }
void loop()  { systeme.loop();  }
