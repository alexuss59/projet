#include <Arduino.h>

// --- PINS (ESP32-S3) ---
// Attention : On croise toujours TX et RX !
const int PIN_RX_ESP = 18; // Brancher le fil TX du scanner ici
const int PIN_TX_ESP = 17; // Brancher le fil RX du scanner ici

// Création de l'interface série matérielle n°1
HardwareSerial ScannerSerial(1); 

void setup() {
  // 1. Console pour le PC
  Serial.begin(115200);
  
  // 2. Console pour le Scanner (9600 bauds est le standard usine)
  ScannerSerial.begin(115200, SERIAL_8N1, PIN_RX_ESP, PIN_TX_ESP);
  
  Serial.println("--- TEST SCANNER SEUL ---");
  Serial.println("Scanne un code-barre maintenant...");
}

void loop() {
  // Si le scanner envoie des données
  if (ScannerSerial.available()) {
    
    // On lit toute la chaîne jusqu'au retour à la ligne (\r)
    String codeBarre = ScannerSerial.readStringUntil('\r');
    
    // On nettoie les espaces vides au début ou à la fin
    codeBarre.trim();
    
    if (codeBarre.length() > 0) {
      Serial.print("CODE REÇU : ");
      Serial.println(codeBarre);
    }
  }
}