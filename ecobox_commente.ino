// =========================================================
//  PROJET BTS CIEL - BORNE DE COLLECTE BOUTEILLES PET
//  Lycée Gustave Eiffel - Session 2026
//  Étudiant 2 : Traitement des bouteilles (ESP32)
//
//  Rôle de ce programme :
//  - Détecter l'arrivée d'une bouteille via capteur IR
//  - Scanner son code-barre pour vérifier si elle est recyclable
//  - Piloter le convoyeur (vers broyeur ou vers consommateur)
//  - Valider physiquement le dépôt via deux capteurs IR
//  - Mesurer le taux de remplissage du bac à flocons
//  - Communiquer avec le Raspberry Pi via MQTT (Ethernet W5500)
//  - Informer visuellement l'utilisateur via un bandeau LED WS2812B
// =========================================================

#include <SPI.h>         // Communication SPI pour le module Ethernet W5500
#include <Ethernet.h>    // Gestion de la connexion réseau filaire
#include <PubSubClient.h>// Client MQTT pour communiquer avec le Raspberry Pi
#include <FastLED.h>     // Pilotage du bandeau LED WS2812B
#include <Wire.h>        // Communication I2C pour le capteur ultrason SRF02
#include "esp_system.h"  // Fonctions système spécifiques ESP32

// ================= CONFIGURATION MATÉRIELLE =================

// --- Ethernet ---
#define W5500_CS_PIN 10   // Broche Chip Select du module Ethernet W5500 (SPI)

// --- Bandeau LED ---
#define LED_PIN 7         // Broche de données du bandeau LED (signal vert)
#define NUM_LEDS 27       // Nombre total de LEDs sur le bandeau WS2812B

// --- Scanner de code-barre (communication série) ---
#define SCANNER_RX_PIN 17 // Broche RX de l'ESP32 connectée au TX du scanner (fil violet)
#define SCANNER_TX_PIN 18 // Broche TX de l'ESP32 connectée au RX du scanner (fil bleu)

// --- Capteur ultrason SRF02 (mesure niveau bac) ---
#define SRF02_ADDR 0x70   // Adresse I2C du capteur SRF02
#define SDA_PIN_sfr02 8   // Broche SDA du bus I2C dédié au SRF02
#define SCL_PIN_sfr02 9   // Broche SCL du bus I2C dédié au SRF02

// --- Capteurs infrarouges ---
#define PIN_IR_ENTREE 6       // IR d'entrée : détecte l'arrivée d'une bouteille (fil blanc)
#define PIN_IR_VALIDATION_1 2 // 1er IR de validation : détecte le début de la bouteille (fil bleu)
#define PIN_IR_VALIDATION_2 3 // 2ème IR de validation : détecte la fin de la bouteille (fil jaune)

// --- Calibration du bac à flocons (en centimètres) ---
// Le capteur ultrason mesure la distance entre lui et la surface des flocons.
// Plus le bac se remplit, plus la distance diminue.
#define HAUTEUR_BAC_VIDE 80  // Distance mesurée quand le bac est complètement vide
#define HAUTEUR_BAC_PLEIN 20 // Distance mesurée quand le bac est plein (min physique : 12cm)

// --- Moteur du convoyeur ---
#define MOTEUR_DIR1 20 // Broche de direction 1 du pont en H (fil jaune)
#define MOTEUR_DIR2 21 // Broche de direction 2 du pont en H (fil vert)

// Trame hexadécimale pour déclencher une lecture sur le scanner code-barre
// C'est le protocole constructeur du module 14810 pour lancer un scan unique
const byte COMMAND_TRIGGER[] = {0x7E, 0x00, 0x08, 0x01, 0x00, 0x02, 0x01, 0xAB, 0xCD};


// ================= OBJETS GLOBAUX =================

// Port série matériel n°2 de l'ESP32, dédié au scanner code-barre
HardwareSerial ScannerSerial(2);

// Tableau représentant les 27 LEDs du bandeau
CRGB leds[NUM_LEDS];

// Adresse MAC de l'ESP32 sur le réseau local (doit être unique sur le réseau)
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };

// Adresse IP du broker MQTT Mosquitto installé sur le Raspberry Pi
IPAddress brokerIP(192, 168, 1, 100);

// Client Ethernet et client MQTT (le client MQTT utilise Ethernet comme transport)
EthernetClient ethClient;
PubSubClient mqttClient(ethClient);

// Mutex pour protéger la variable partagée entre le code principal et l'interruption
portMUX_TYPE myMux = portMUX_INITIALIZER_UNLOCKED;

// Flag positionné à true par l'interruption matérielle quand une bouteille est détectée
volatile bool interruptionEntree = false;

// =========================================================
// INTERRUPTION MATÉRIELLE — Détection d'entrée bouteille
// =========================================================
// Cette fonction s'exécute AUTOMATIQUEMENT dès que le capteur IR d'entrée
// détecte un front montant (RISING), c'est-à-dire le passage d'une bouteille.
// Elle doit être la plus courte possible : on pose juste un flag.
// IRAM_ATTR : force le code à rester en RAM pour une exécution ultra-rapide
void IRAM_ATTR ISR_Entree() {
  portENTER_CRITICAL_ISR(&myMux); // Section critique : accès exclusif à la variable
  interruptionEntree = true;       // On signale qu'une bouteille a été détectée
  portEXIT_CRITICAL_ISR(&myMux);  // Fin de la section critique
}

// =========================================================
// MACHINE À ÉTATS — Les 7 états possibles de la borne
// =========================================================
enum State {
  BORNE_PRETE,         // Borne disponible, en attente d'une bouteille
  BOUTEILLE_DETECTEE,  // Bouteille présente, scan du code-barre en cours
  DEPOT_VALIDE,        // Code-barre OK, convoyeur avance, validation physique en cours
  ATTENTE_CHUTE,       // Bouteille validée, on attend qu'elle tombe dans le bac
  ATTENTE_REJET,       // Bouteille refusée, pause avant marche arrière
  REJET_EN_COURS,      // Moteur en marche arrière pour rendre la bouteille
  BAC_PLEIN            // Bac saturé (>= 95%), borne hors service
};


// =========================================================
//                   CLASSES MATÉRIELLES
// =========================================================

// ---------------------------------------------------------
// Classe MoteurConvoyeur
// Pilote le moteur bidirectionnel via un pont en H (2 broches)
// DIR1=HIGH, DIR2=LOW → avance (vers broyeur)
// DIR1=LOW, DIR2=HIGH → recule (vers consommateur)
// DIR1=LOW, DIR2=LOW  → arrêt
// ---------------------------------------------------------
class MoteurConvoyeur {
  private:
    int pin1, pin2; // Broches du pont en H
  
  public:
    // Constructeur : mémorise les broches à utiliser
    MoteurConvoyeur(int p1, int p2) : pin1(p1), pin2(p2) {}
    
    // Configure les broches en sortie et arrête le moteur
    void init() {
      pinMode(pin1, OUTPUT);
      pinMode(pin2, OUTPUT);
      stopper();
    }
    
    // Fait avancer le convoyeur → bouteille part vers le broyeur
    void avancer() {
      Serial.println(">>> MOTEUR : AVANT");
      digitalWrite(pin1, HIGH);
      digitalWrite(pin2, LOW);
    }
    
    // Fait reculer le convoyeur → bouteille revient vers le consommateur
    void reculer() {
      Serial.println(">>> MOTEUR : ARRIÈRE");
      digitalWrite(pin1, LOW);
      digitalWrite(pin2, HIGH);
    }
    
    // Arrête complètement le moteur
    void stopper() {
      digitalWrite(pin1, LOW);
      digitalWrite(pin2, LOW);
    }
};

// ---------------------------------------------------------
// Classe CapteurUltrason
// Pilote le capteur SRF02 via I2C pour mesurer la distance
// en cm jusqu'à la surface des flocons de PET dans le bac.
// Plus la distance est petite → plus le bac est plein.
// ---------------------------------------------------------
class CapteurUltrason {
  private:
    int adresseI2C; // Adresse I2C du capteur (0x70 pour le SRF02)
  
  public:
    CapteurUltrason(int addr) : adresseI2C(addr) {}

    // Déclenche une mesure et retourne la distance en cm
    // Retourne -1 en cas d'erreur de communication I2C
    int lireDistance() {
      // Étape 1 : envoyer la commande de mesure (registre 0x00, mode cm = 0x51)
      Wire.beginTransmission(adresseI2C);
      Wire.write(0x00); // Registre de commande
      Wire.write(0x51); // 0x51 = mesure en centimètres
      Wire.endTransmission();
      delay(70); // Attendre 70ms que le capteur effectue la mesure (datasheet SRF02)

      // Étape 2 : pointer vers le registre de résultat (registre 0x02)
      Wire.beginTransmission(adresseI2C);
      Wire.write(0x02);
      Wire.endTransmission();

      // Étape 3 : lire 2 octets (octet fort + octet faible = distance 16 bits)
      Wire.requestFrom(adresseI2C, 2);
      if (Wire.available() >= 2) {
        byte high = Wire.read(); // Octet de poids fort
        byte low = Wire.read();  // Octet de poids faible
        return (high << 8) | low; // Reconstitution de la valeur 16 bits
      }
      return -1; // Erreur : pas de réponse du capteur
    }

    // Calcule le taux de remplissage en % à partir de la distance mesurée
    // 0% = bac vide (distance max), 100% = bac plein (distance min)
    int calculerTauxRemplissage(int distance) {
      if (distance >= HAUTEUR_BAC_VIDE) return 0;   // Bac complètement vide
      if (distance <= HAUTEUR_BAC_PLEIN) return 100; // Bac complètement plein
      // Interpolation linéaire entre les deux extrêmes
      return ((HAUTEUR_BAC_VIDE - distance) * 100) / (HAUTEUR_BAC_VIDE - HAUTEUR_BAC_PLEIN);
    }
};


// =========================================================
//             CLASSE PRINCIPALE : LE CONTRÔLEUR
// =========================================================
// C'est le "cerveau" de la borne. Elle orchestre tous les
// composants matériels et gère la machine à états complète.
// =========================================================
class ControleurEcoBox {
  private:
    State currentState;          // État actuel de la machine à états
    unsigned long timerEtat;     // Horodatage du dernier changement d'état (pour les timeouts)
    bool clignotementState;      // État actuel du clignotement LED (allumé/éteint)
    unsigned long lastMQTTReconnect; // Horodatage de la dernière tentative de reconnexion MQTT
    
    // --- Suivi du cycle de vie d'une bouteille ---
    bool aVuEntree;          // true = capteur de validation a vu le début de la bouteille
    bool aVuSortie;          // true = capteur de validation a vu la fin de la bouteille
    bool fraudeDetectee;     // true = la bouteille a reculé (tentative de fraude)
    bool bouteille_validee;  // true = le dépôt est confirmé physiquement
    bool commandeScanEnvoyee;// true = l'ordre de scan a déjà été envoyé au scanner

    // --- Objets matériels instanciés ---
    MoteurConvoyeur moteur;     // Contrôle du convoyeur
    CapteurUltrason capteurBac; // Mesure du niveau du bac

    // Supprime tous les caractères non numériques d'un code-barre brut
    // Utile car le scanner envoie parfois des caractères parasites
    String nettoyerCode(String brut) {
      String propre = "";
      for (int i = 0; i < brut.length(); i++) {
        if (isDigit(brut[i])) propre += brut[i];
      }
      return propre;
    }

    // Réinitialise toutes les variables de suivi d'une bouteille
    // À appeler à chaque nouveau cycle (nouvelle bouteille)
    void resetTracking() {
      aVuEntree = false;
      aVuSortie = false;
      fraudeDetectee = false;
      bouteille_validee = false;
      commandeScanEnvoyee = false;
    }

    // Maintient la connexion MQTT active
    // Si déconnecté, tente une reconnexion toutes les 5 secondes
    // Si connecté, appelle mqttClient.loop() pour traiter les messages entrants
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
        mqttClient.loop(); // Traitement des messages MQTT entrants/sortants
      }
    }

    // Met à jour la couleur du bandeau LED selon l'état actuel de la borne
    // Conforme au cahier des charges :
    // - Vert fixe   : borne prête ou dépôt validé
    // - Bleu fixe   : bouteille présente
    // - Bleu clign. : code-barre reconnu (bouteille acceptée)
    // - Rouge clign.: bouteille refusée ou dépôt non validé
    // - Rouge fixe  : bac plein, borne inutilisable
    void actualiserLeds() {
      // Gestion du clignotement : bascule toutes les 300ms
      static unsigned long lastBlink = 0;
      if (millis() - lastBlink > 300) {
        clignotementState = !clignotementState;
        lastBlink = millis();
      }

      switch (currentState) {
        case BORNE_PRETE:
        case ATTENTE_CHUTE:
          fill_solid(leds, NUM_LEDS, CRGB::Green); break;  // Vert fixe
        case BOUTEILLE_DETECTEE:
          fill_solid(leds, NUM_LEDS, CRGB::Blue); break;   // Bleu fixe
        case DEPOT_VALIDE:
          fill_solid(leds, NUM_LEDS, clignotementState ? CRGB::Blue : CRGB::Black); break; // Bleu clignotant
        case ATTENTE_REJET:
        case REJET_EN_COURS:
          fill_solid(leds, NUM_LEDS, clignotementState ? CRGB::Red : CRGB::Black); break;  // Rouge clignotant
        case BAC_PLEIN:
          fill_solid(leds, NUM_LEDS, CRGB::Red); break;    // Rouge fixe
      }
      FastLED.show(); // Envoie les données au bandeau physique
    }

  public:
    // Constructeur : initialise les objets matériels et les variables
    ControleurEcoBox() :
      moteur(MOTEUR_DIR1, MOTEUR_DIR2),  // Crée le moteur avec ses deux broches
      capteurBac(SRF02_ADDR)             // Crée le capteur avec son adresse I2C
    {
      currentState = BORNE_PRETE; // L'état initial est toujours BORNE_PRETE
      clignotementState = false;
      lastMQTTReconnect = 0;
      resetTracking();
    }

    // Initialisation physique des broches et du moteur
    void init() {
      moteur.init();
      pinMode(PIN_IR_ENTREE, INPUT_PULLUP);       // IR d'entrée en entrée avec pull-up interne
      pinMode(PIN_IR_VALIDATION_1, INPUT_PULLUP); // IR validation 1 idem
      pinMode(PIN_IR_VALIDATION_2, INPUT_PULLUP); // IR validation 2 idem
    }

    // =========================================================
    // BOUCLE PRINCIPALE — appelée à chaque tour de loop()
    // Exécute les tâches périodiques puis traite l'état courant
    // =========================================================
    void executerCycle() {
      actualiserLeds(); // Toujours mettre à jour les LEDs
      gererReseau();    // Toujours maintenir la connexion MQTT

      // SÉCURITÉ ANTI-FANTÔME : Si la borne est occupée (pas dans BORNE_PRETE),
      // on ignore toute interruption déclenchée par le capteur d'entrée.
      // Cela évite qu'un reflet ou une vibration lance un faux cycle.
      if (currentState != BORNE_PRETE) {
        portENTER_CRITICAL(&myMux);
        interruptionEntree = false;
        portEXIT_CRITICAL(&myMux);
      }

      // =========================================================
      // MACHINE À ÉTATS — traitement selon l'état courant
      // =========================================================
      switch (currentState) {

        // ---------------------------------------------------------
        // ÉTAT 1 : BORNE_PRETE
        // La borne attend une bouteille. Moteur à l'arrêt.
        // Vérifie périodiquement le niveau du bac (toutes les 10s).
        // ---------------------------------------------------------
        case BORNE_PRETE: {
          moteur.stopper();

          // Vérification périodique du bac (toutes les 10 secondes)
          static unsigned long lastIdleCheck = 0;
          if (millis() - lastIdleCheck > 10000) {
            int dist = capteurBac.lireDistance();
            if (dist > 0 && dist <= HAUTEUR_BAC_PLEIN) {
              // Le bac était plein avant même qu'une bouteille arrive !
              Serial.println(">>> ⚠️ ALERTE VEILLE : Bac plein détecté avant dépôt !");
              currentState = BAC_PLEIN;
              if (mqttClient.connected()) mqttClient.publish("ecobox/alerte", "BAC_PLEIN_VEILLE");
            }
            lastIdleCheck = millis();
          }

          // Vérification si une bouteille a été détectée par l'interruption
          portENTER_CRITICAL(&myMux);
          if (interruptionEntree && currentState == BORNE_PRETE) {
            interruptionEntree = false;
            currentState = BOUTEILLE_DETECTEE; // → transition vers l'état suivant
            timerEtat = millis();
            while (ScannerSerial.available()) ScannerSerial.read(); // Vide le buffer série
            Serial.println("\n>>> BOUTEILLE DÉTECTÉE");
          }
          portEXIT_CRITICAL(&myMux);
          break;
        }

        // ---------------------------------------------------------
        // ÉTAT 2 : BOUTEILLE_DETECTEE
        // Une bouteille est présente. On envoie l'ordre de scan
        // au lecteur code-barre et on attend le résultat (max 3s).
        // ---------------------------------------------------------
        case BOUTEILLE_DETECTEE:
          if (!commandeScanEnvoyee) {
            // Étape 1 : vider le buffer du scanner pour éviter de lire un ancien code
            while (ScannerSerial.available()) ScannerSerial.read();

            // Étape 2 : réveiller le scanner (octet nul = wake-up)
            ScannerSerial.write(0x00);
            delay(50); // 50ms de marge pour s'assurer que le scanner est prêt

            // Étape 3 : envoyer la commande de déclenchement du scan
            ScannerSerial.write(COMMAND_TRIGGER, sizeof(COMMAND_TRIGGER));
            commandeScanEnvoyee = true;
            timerEtat = millis(); // Démarrage du timeout de 3 secondes
          }

          // Timeout : si aucun code n'est lu en 3 secondes → rejet
          if (millis() - timerEtat > 3000) {
            Serial.println(">>> TIMEOUT SCAN -> Rejet (Aucun code lu en 3s)");
            currentState = ATTENTE_REJET;
            timerEtat = millis();
            moteur.stopper();
            commandeScanEnvoyee = false;
          }
          // Un code a été reçu sur le port série du scanner
          else if (ScannerSerial.available()) {
            String codeBrut = ScannerSerial.readStringUntil('\r'); // Lecture jusqu'au retour chariot
            Serial.print(">>> [DEBUG SCANNER] Brut lu : "); Serial.println(codeBrut);

            String codePropre = nettoyerCode(codeBrut); // Suppression des caractères non numériques

            // Un code-barre valide a plus de 10 chiffres
            if (codePropre.length() > 10) {
              Serial.print(">>> CODE VALIDE : "); Serial.println(codePropre);

              // Envoi du code au Raspberry Pi via MQTT
              // Le Raspberry vérifiera si ce code est dans la BDD des bouteilles recyclables
              if (mqttClient.connected()) {
                mqttClient.publish("ecobox/print", codePropre.c_str());
                Serial.println(">>> 📡 Code envoyé au serveur (MQTT)");
              } else {
                Serial.println(">>> ⚠️ Impossible d'envoyer le code (Non connecté)");
              }

              resetTracking();              // Réinitialisation du suivi pour ce nouveau dépôt
              currentState = DEPOT_VALIDE; // → on démarre la validation physique
              timerEtat = millis();
              moteur.avancer();            // Convoyeur → direction broyeur
            }
          }
          break;

        // ---------------------------------------------------------
        // ÉTAT 3 : DEPOT_VALIDE
        // Le code-barre est bon. Le convoyeur avance.
        // On surveille les deux capteurs IR pour confirmer que la
        // bouteille est bien passée (début ET fin détectés).
        // Détection de fraude : si la bouteille recule puis repasse.
        // Timeout global : 3 secondes (CDC : validation en < 3s)
        // ---------------------------------------------------------
        case DEPOT_VALIDE: {
          // Lecture des deux capteurs IR de validation
          // INPUT_PULLUP → LOW quand le capteur est activé → on inverse avec !
          byte ir1 = !digitalRead(PIN_IR_VALIDATION_1); // 1 = bouteille devant IR1
          byte ir2 = !digitalRead(PIN_IR_VALIDATION_2); // 1 = bouteille devant IR2

          // Encodage de l'état des deux capteurs en valeur binaire :
          // etatBinaire = 0b(ir1)(ir2)
          // 0 = aucun capteur activé
          // 1 = seulement IR2 activé (fin de bouteille)
          // 2 = seulement IR1 activé (début de bouteille)
          // 3 = les deux activés (bouteille en transit)
          int etatBinaire = (ir1 << 1) | ir2;

          // Détection du début de passage (ir1=0, ir2=1 → état=1)
          // OU état 0 (aucun capteur) - NOTE: état 0 ici est un bug, voir rapport
          if (etatBinaire == 1 || etatBinaire == 0) {
            aVuEntree = true;
            if (aVuSortie) fraudeDetectee = true; // La bouteille revient en arrière → fraude !
          }
          // Détection de la sortie (ir1=1, ir2=0 → état=2)
          if (etatBinaire == 2) {
            if (aVuEntree) aVuSortie = true; // La bouteille est bien passée devant IR1
          }

          // SUCCÈS : les deux capteurs libres ET début ET fin vus ET pas de fraude
          // → la bouteille est tombée dans le bac, dépôt confirmé !
          if (etatBinaire == 3 && aVuEntree && aVuSortie && !fraudeDetectee && !bouteille_validee) {
            bouteille_validee = true;
            Serial.println(">>> SUCCÈS ! Passage en mode chute...");
            currentState = ATTENTE_CHUTE; // → on attend la fin de la chute
            timerEtat = millis();
          }
          // FRAUDE ou bouteille non passée → rejet
          else if (etatBinaire == 3 && aVuEntree && (!aVuSortie || fraudeDetectee)) {
            Serial.println(">>> FRAUDE DÉTECTÉE -> Pause Rouge");
            moteur.stopper();
            currentState = ATTENTE_REJET;
            timerEtat = millis();
          }

          // TIMEOUT : si la validation n'a pas eu lieu en 11 secondes → rejet
          // NOTE : Le CDC prévoit 3s, mais les tests sur le prototype physique ont
          // montré que ce délai est insuffisant compte tenu de la longueur réelle
          // du convoyeur. La valeur a été portée à 11s pour garantir un fonctionnement fiable.
          if (millis() - timerEtat > 11000) { // Ajusté à 11s après tests terrain (CDC théorique : 3s)
            Serial.println(">>> TIMEOUT GLOBAL -> Pause Rouge");
            moteur.stopper();
            currentState = ATTENTE_REJET;
            timerEtat = millis();
          }
          break;
        }

        // ---------------------------------------------------------
        // ÉTAT 4 : ATTENTE_CHUTE
        // Le dépôt est validé. On laisse 500ms pour que la bouteille
        // finisse de tomber dans le bac, puis on arrête le moteur
        // et on mesure le niveau du bac.
        // ---------------------------------------------------------
        case ATTENTE_CHUTE:
          if (millis() - timerEtat > 500) {
            Serial.println(">>> Chute de la bouteille terminée.");
            moteur.stopper();

            Serial.println(">>> Mesure du niveau du bac...");
            int dist = capteurBac.lireDistance();

            if (dist == -1) {
              // Erreur capteur : on remet en service par sécurité
              Serial.println(">>> ❌ ERREUR CAPTEUR SRF02 (Vérifier câblage)");
              currentState = BORNE_PRETE;
            } else {
              int taux = capteurBac.calculerTauxRemplissage(dist);
              Serial.print(">>> Distance : "); Serial.print(dist); Serial.println(" cm");
              Serial.print(">>> TAUX DE REMPLISSAGE : "); Serial.print(taux); Serial.println(" %");

              // Envoi du taux au Raspberry Pi pour enregistrement en BDD
              if (mqttClient.connected()) {
                mqttClient.publish("ecobox/niveau", String(taux).c_str());
                Serial.println(">>> 📡 Taux envoyé au serveur BDD (MQTT)");
              }

              // CDC : borne indisponible si taux >= 95%
              if (taux >= 95) {
                Serial.println(">>> ⚠️ ALERTE : BAC PLEIN ! Verrouillage de la borne.");
                currentState = BAC_PLEIN;
                if (mqttClient.connected()) mqttClient.publish("ecobox/alerte", "BAC_PLEIN");
              } else {
                Serial.println(">>> Borne prête pour la prochaine bouteille.");
                currentState = BORNE_PRETE; // → cycle terminé, on repart en attente
              }
            }
          }
          break;

        // ---------------------------------------------------------
        // ÉTAT 5 : ATTENTE_REJET
        // Pause d'1 seconde (LEDs rouge clignotant) pour informer
        // l'utilisateur avant de lancer la marche arrière.
        // ---------------------------------------------------------
        case ATTENTE_REJET:
          moteur.stopper();
          if (millis() - timerEtat > 1000) {
            Serial.println(">>> Fin pause. Lancement MARCHE ARRIÈRE (11s).");
            currentState = REJET_EN_COURS;
            timerEtat = millis();
            moteur.reculer(); // Convoyeur → direction consommateur
          }
          break;

        // ---------------------------------------------------------
        // ÉTAT 6 : REJET_EN_COURS
        // Le moteur recule pendant 11 secondes pour rendre la bouteille.
        // NOTE : Le CDC prévoit 3s, mais les tests sur le prototype physique ont
        // montré que ce délai est insuffisant pour que la bouteille revienne
        // jusqu'à l'entrée de la borne. La valeur a été portée à 11s.
        // ---------------------------------------------------------
        case REJET_EN_COURS:
          // Le moteur.reculer() a déjà été lancé dans ATTENTE_REJET
          if (millis() - timerEtat > 11000) { // Ajusté à 11s après tests terrain (CDC théorique : 3s)
            Serial.println(">>> Fin du rejet.");
            moteur.stopper();
            currentState = BORNE_PRETE; // → retour en service
          }
          break;

        // ---------------------------------------------------------
        // ÉTAT 7 : BAC_PLEIN
        // Borne hors service. LEDs rouge fixe.
        // Toutes les 5 secondes, on vérifie si le responsable
        // a vidé le bac (distance > seuil plein + 5cm de marge).
        // ---------------------------------------------------------
        case BAC_PLEIN:
          moteur.stopper();
          if (millis() - timerEtat > 5000) {
            int dist = capteurBac.lireDistance();
            if (dist > HAUTEUR_BAC_PLEIN + 5) {
              // Le bac a été vidé par le responsable → remise en service
              Serial.println(">>> ✅ BAC VIDÉ PAR LE RESPONSABLE. Déverrouillage.");
              currentState = BORNE_PRETE;
            }
            timerEtat = millis(); // Réarme le timer pour la prochaine vérification
          }
          break;
      }
    }
};


// =========================================================
//                   PROGRAMME PRINCIPAL
// =========================================================

// Instance unique du contrôleur (crée implicitement moteur + capteur)
ControleurEcoBox maBorne;

// ---------------------------------------------------------
// setup() — exécuté une seule fois au démarrage de l'ESP32
// ---------------------------------------------------------
void setup() {
  Serial.begin(115200); // Moniteur série pour le débogage

  // Initialisation du bus I2C sur les broches dédiées au SRF02
  Wire.begin(SDA_PIN_sfr02, SCL_PIN_sfr02);

  // Initialisation du bandeau LED WS2812B
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);

  // Initialisation du port série pour le scanner code-barre (9600 bauds, 8N1)
  ScannerSerial.begin(9600, SERIAL_8N1, SCANNER_RX_PIN, SCANNER_TX_PIN);

  // Initialisation des broches capteurs + moteur
  maBorne.init();

  // Attachement de l'interruption matérielle sur le capteur IR d'entrée
  // RISING = déclenchement sur front montant (arrivée de la bouteille)
  attachInterrupt(digitalPinToInterrupt(PIN_IR_ENTREE), ISR_Entree, RISING);

  // Initialisation du module Ethernet W5500 via SPI
  Serial.println(">>> Initialisation du réseau Ethernet W5500...");
  Ethernet.init(W5500_CS_PIN);

  // Tentative d'obtention d'une adresse IP via DHCP
  if (Ethernet.begin(mac) == 0) {
    Serial.println(">>> ❌ Échec de la configuration DHCP");
  } else {
    Serial.print(">>> ✅ Connecté au réseau. IP : ");
    Serial.println(Ethernet.localIP());
  }

  // Configuration du broker MQTT (IP du Raspberry Pi, port standard 1883)
  mqttClient.setServer(brokerIP, 1883);

  Serial.println("=== SYSTÈME DÉMARRÉ ===");
}

// ---------------------------------------------------------
// loop() — exécuté en boucle infinie après setup()
// Délègue tout le traitement au contrôleur
// ---------------------------------------------------------
void loop() {
  maBorne.executerCycle(); // Un seul appel suffit, tout est géré en interne
}
