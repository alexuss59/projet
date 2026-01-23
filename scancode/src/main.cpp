#include <Arduino.h>

// --- Configuration ---
const int PIN_RX_SCANNER = 18; //(TX du scanner)
const int PIN_TX_SCANNER = 17; //(RX du scanner)

// Création de l'interface pour le scanner
HardwareSerial ScannerSerial(1); 

void setup() {
  
  Serial.begin(115200);
  delay(2000); // Laisse le temps à l'USB de se connecter
  Serial.println("--- TEST SCANNER SEUL (Waveshare 9600) ---");

  // 2. Scanner (UART)
  // Vitesse 9600 imposée par ton manuel Waveshare V2.1 [Page 14]
  ScannerSerial.begin(9600, SERIAL_8N1, PIN_RX_SCANNER, PIN_TX_SCANNER);
}

void loop() {
  // Si le scanner envoie quelque chose
  if (ScannerSerial.available()) {
    
    // On lit jusqu'au retour à la ligne (\r)
    String codeBarre = ScannerSerial.readStringUntil('\r');
    
    // On nettoie les espaces inutiles
    codeBarre.trim();
    
    // On affiche
    if (codeBarre.length() > 0) {
      Serial.print("Code lu : ");
      Serial.println(codeBarre);
    }
  }
}