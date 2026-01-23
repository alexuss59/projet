#include <Arduino.h>
#include <FastLED.h>

// --- CONFIGURATION ---
const int PIN_IR_ENTREE = 6;
const int PIN_LEDS = 7;
const int NUM_LEDS = 10;

const int PIN_RX_SCANNER = 18; 
const int PIN_TX_SCANNER = 17; 

// --- COMMANDES WAVESHARE V2.1 (Page 25 & 81) ---
// Format : Header(2) + Type(1) + Len(1) + Address(2) + Data(1) + CRC(2)
// On utilise le CRC "Magique" AB CD pour ne pas avoir à le calculer [cite: 5705]

// Start Scan : Ecrire '1' à l'adresse 0x0002
const byte CMD_START_SCAN[] = {0x7E, 0x00, 0x08, 0x01, 0x00, 0x02, 0x01, 0xAB, 0xCD};

// Stop Scan : Ecrire '0' à l'adresse 0x0002
const byte CMD_STOP_SCAN[]  = {0x7E, 0x00, 0x08, 0x01, 0x00, 0x02, 0x00, 0xAB, 0xCD};

CRGB leds[NUM_LEDS];
HardwareSerial ScannerSerial(1); 

// États
enum Etat { ATTENTE, SCAN_EN_COURS, VALIDATION, ERREUR, ATTENTE_RETRAIT };
Etat etatActuel = ATTENTE;
unsigned long debutScan = 0;
const long TIMEOUT_SCAN = 3000; 

// --- UTILITAIRES ---
void allumerLeds(CRGB c) { for(int i=0; i<NUM_LEDS; i++) leds[i] = c; FastLED.show(); }
void clignoter(CRGB c) { for(int i=0; i<3; i++) { allumerLeds(CRGB::Black); delay(100); allumerLeds(c); delay(100); } }

// Envoi de commande brute (Hexadécimal)
void envoyerCommande(const byte* cmd, int taille) {
  ScannerSerial.write(cmd, taille);
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("--- SYSTEME V2.1 (HEXADECIMAL) ---");

  // Init Scanner 9600 (Waveshare V2.1 standard)
  ScannerSerial.begin(9600, SERIAL_8N1, PIN_RX_SCANNER, PIN_TX_SCANNER);

  pinMode(PIN_IR_ENTREE, INPUT_PULLUP);
  FastLED.addLeds<WS2812B, PIN_LEDS, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(40);
  
  // On s'assure qu'il est éteint au démarrage
  envoyerCommande(CMD_STOP_SCAN, sizeof(CMD_STOP_SCAN));
  allumerLeds(CRGB::Green);
}

void loop() {
  bool bouteilleLa = (digitalRead(PIN_IR_ENTREE) == LOW);

  switch (etatActuel) {

    case ATTENTE:
      if (bouteilleLa) {
        Serial.println(">>> DETECTÉ -> SCAN (HEX)");
        
        while(ScannerSerial.available()) ScannerSerial.read(); // Vider buffer
        
        // ENVOI DE LA COMMANDE HEX POUR ALLUMER
        envoyerCommande(CMD_START_SCAN, sizeof(CMD_START_SCAN));
        
        etatActuel = SCAN_EN_COURS;
        debutScan = millis();
        allumerLeds(CRGB::Blue);
      }
      break;

    case SCAN_EN_COURS:
      // A. Lecture du code
      if (ScannerSerial.available()) {
        // Le scanner peut répondre "02 00 00 01 00 33 31" pour dire "Ok j'ai reçu l'ordre"
        // On doit filtrer ça et chercher le vrai code-barre.
        
        // Pour faire simple ici : on lit tout. Si c'est long (>7 caractères), c'est probablement un code-barre.
        // Si c'est court (7 bytes), c'est juste l'acquittement de commande.
        String data = ScannerSerial.readStringUntil('\r');
        data.trim();
        
        // On ignore les réponses de commande (souvent commencent par un caractère non imprimable ou sont courtes)
        if (data.length() > 7) { 
           Serial.print(">>> CODE : "); Serial.println(data);
           
           envoyerCommande(CMD_STOP_SCAN, sizeof(CMD_STOP_SCAN)); // On éteint
           etatActuel = VALIDATION;
        }
      }
      
      // B. Timeout
      if (millis() - debutScan > TIMEOUT_SCAN) {
        Serial.println("!!! TIMEOUT");
        envoyerCommande(CMD_STOP_SCAN, sizeof(CMD_STOP_SCAN)); // On éteint de force
        etatActuel = ERREUR;
      }

      // C. Retrait
      if (!bouteilleLa) {
        envoyerCommande(CMD_STOP_SCAN, sizeof(CMD_STOP_SCAN));
        etatActuel = ATTENTE;
        allumerLeds(CRGB::Green);
      }
      break;

    case VALIDATION:
      clignoter(CRGB::Blue);
      etatActuel = ATTENTE_RETRAIT;
      break;

    case ERREUR:
      clignoter(CRGB::Red);
      etatActuel = ATTENTE_RETRAIT;
      break;

    case ATTENTE_RETRAIT:
      allumerLeds(CRGB::Blue);
      if (!bouteilleLa) {
        etatActuel = ATTENTE;
        allumerLeds(CRGB::Green);
      }
      break;
  }
}