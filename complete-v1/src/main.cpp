
#include <SPI.h>
#include <Ethernet.h>
#include <FastLED.h>
#include <PubSubClient.h>
#include <Wire.h>       // Pour le capteur SRF02 (I2C)
#include "esp_system.h" // Pour la lecture de la MAC d'usine

// ================= CONFIGURATION MATÉRIELLE =================
#define W5500_CS_PIN 10 // Pin CS Ethernet
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
#define MOTEUR_DIR1 20
#define MOTEUR_DIR2 21

// Commande HEX pour déclencher le scan (Manuel Page 25)
const byte COMMAND_TRIGGER[] = {0x7E, 0x00, 0x08, 0x01, 0x00, 0x02, 0x01, 0xAB, 0xCD};

HardwareSerial ScannerSerial(2);
CRGB leds[NUM_LEDS];

// ================= VARIABLES SYSTÈME =================
portMUX_TYPE myMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool interruptionEntree = false;

enum State
{
  BORNE_PRETE,        // Vert fixe
  BOUTEILLE_DETECTEE, // Bleu fixe
  SCAN_EN_COURS,      // Bleu clignotant
  DEPOT_VALIDE,       // Bleu clignotant
  DEPOT_REFUSE,       // Rouge clignotant
  BAC_PLEIN           // Rouge fixe
};

volatile State currentState = BORNE_PRETE;
unsigned long timerEtat = 0;
bool clignotementState = false;

bool v1_ok = false, v2_ok = false, bouteille_validee = false;

// ================= RÉSEAU & MQTT =================
IPAddress server(192, 168, 1, 100); // Raspberry Pi 5
EthernetClient ethClient;
PubSubClient mqttClient(ethClient);

// ================= FONCTION SRF02 (I2C) =================
int lireSRF02()
{
  // 1. Lancement de la mesure en cm (commande 0x51)
  Wire.beginTransmission(SRF02_ADDR);
  Wire.write(0x00);
  Wire.write(0x51);
  Wire.endTransmission();

  // 2. Le SRF02 a besoin de 70ms pour mesurer
  delay(70);

  // 3. Lecture du résultat (registres 2 et 3)
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
{ // 1:Avant, -1:Arrière, 0:Stop
  if (sens == 1)
  {
    digitalWrite(MOTEUR_DIR1, HIGH);
  }
  else if (sens == -1)
  {
    digitalWrite(MOTEUR_DIR2, HIGH);
  }
  else
  {
    digitalWrite(MOTEUR_DIR2, LOW);
    digitalWrite(MOTEUR_DIR1, LOW);
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

// ================= LOGIQUE MQTT =================
void callback(char *topic, byte *payload, unsigned int length)
{
  String msg = "";
  for (int i = 0; i < length; i++)
    msg += (char)payload[i];

  portENTER_CRITICAL(&myMux);
  if (msg == "OK")
  {
    currentState = DEPOT_VALIDE;
    timerEtat = millis();
    v1_ok = false;
    v2_ok = false;
    bouteille_validee = false;
    piloterMoteur(1); // Marche avant
  }
  else
  {
    currentState = DEPOT_REFUSE;
    timerEtat = millis();
    piloterMoteur(-1); // Marche arrière (3s CDC)
  }
  portEXIT_CRITICAL(&myMux);
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
    { // Si c'est un chiffre (0-9)
      propre += brut[i];
    }
  }
  return propre;
}

// ================= SETUP =================
void setup()
{
  Serial.begin(115200);
  Wire.begin(SDA_PIN_sfr02, SCL_PIN_sfr02); // Initialisation I2C pour SRF02

  pinMode(MOTEUR_DIR1, OUTPUT);
  pinMode(MOTEUR_DIR2, OUTPUT);
  pinMode(PIN_IR_ENTREE, INPUT_PULLUP);
  pinMode(PIN_IR_VALIDATION_1, INPUT_PULLUP);
  pinMode(PIN_IR_VALIDATION_2, INPUT_PULLUP);

  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);

  // Ethernet & MAC
  Ethernet.init(W5500_CS_PIN);
  byte mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  mac[5] += 1; // MAC Ethernet unique

  Serial.print("Connexion Ethernet... MAC: ");
  for (int i = 0; i < 6; i++)
    Serial.printf("%02X ", mac[i]);

  if (Ethernet.begin(mac) == 0)
    Serial.println("\nEchec DHCP");
  else
    Serial.println(Ethernet.localIP());

  mqttClient.setServer(server, 1883);
  mqttClient.setCallback(callback);
  ScannerSerial.begin(9600, SERIAL_8N1, SCANNER_RX_PIN, SCANNER_TX_PIN);
  attachInterrupt(digitalPinToInterrupt(PIN_IR_ENTREE), ISR_Entree, FALLING);
}

// ================= LOOP =================
void loop()
{
  if (!mqttClient.connected())
  {
    if (mqttClient.connect("ESP32_Etudiant2"))
      mqttClient.subscribe("borne/reponse");
  }
  mqttClient.loop();
  actualiserLeds();

  // 1. Surveillance périodique Bac Plein avec SRF02
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

  // 2. Machine à états
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
    }
    portEXIT_CRITICAL(&myMux);
    break;

  case BOUTEILLE_DETECTEE:
    static bool commandeEnvoyee = false; // Mémoire pour ne pas spammer la commande

    // 1. Envoi de la commande de scan (UNE SEULE FOIS)
    if (!commandeEnvoyee)
    {
      // Vidage du tampon
      while (ScannerSerial.available())
        ScannerSerial.read();

      Serial.println(">>> REVEIL");

      // 1. On envoie un octet nul pour réveiller le module
      ScannerSerial.write(0x00);
      delay(50); // On lui laisse 50ms

      // 2. On envoie la vraie commande
      ScannerSerial.write(COMMAND_TRIGGER, sizeof(COMMAND_TRIGGER));

      commandeEnvoyee = true;
      timerEtat = millis();
    }

    // 2. Gestion du Timeout (3 secondes)
    if (millis() - timerEtat > 3000)
    {
      Serial.println(">>> Timeout (3s) ! Aucune bouteille lue.");

      portENTER_CRITICAL(&myMux);
      currentState = DEPOT_REFUSE;
      timerEtat = millis();
      portEXIT_CRITICAL(&myMux);

      commandeEnvoyee = false; // Reset pour la prochaine fois
      piloterMoteur(-1);       // Rejet
    }

    // 3. Lecture du résultat
    else if (ScannerSerial.available())
    {
      String codeBrut = ScannerSerial.readStringUntil('\r');

      // On nettoie le code reçu
      String codePropre = nettoyerCode(codeBrut);

      // Un code EAN-13 fait 13 chiffres (parfois le scanner ajoute un préfixe, d'où > 10)
      if (codePropre.length() > 10)
      {
        Serial.print(">>> Code NETTOYÉ : ");
        Serial.println(codePropre);

        // Envoi au MQTT du code propre uniquement
        mqttClient.publish("borne/scan", codePropre.c_str());

        portENTER_CRITICAL(&myMux);
        currentState = SCAN_EN_COURS;
        timerEtat = millis();
        portEXIT_CRITICAL(&myMux);

        commandeEnvoyee = false;
      }
    }
    break;

  case SCAN_EN_COURS:
    if (millis() - timerEtat > 5000)
    { // Timeout RPi
      portENTER_CRITICAL(&myMux);
      currentState = DEPOT_REFUSE;
      timerEtat = millis();
      portEXIT_CRITICAL(&myMux);
      piloterMoteur(-1);
    }
    break;

  case DEPOT_VALIDE:
    // Validation par double capteur IR (CDC Page 6)
    if (!v1_ok && digitalRead(PIN_IR_VALIDATION_1) == LOW)
      v1_ok = true;
    if (v1_ok && !v2_ok && digitalRead(PIN_IR_VALIDATION_2) == LOW)
      v2_ok = true;

    if (v1_ok && v2_ok && !bouteille_validee)
    {
      bouteille_validee = true;
      mqttClient.publish("borne/validation", "OK");
    }

    if (millis() - timerEtat > 4000)
    { // Temps de convoyage
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
    { // Retour client 3s
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
