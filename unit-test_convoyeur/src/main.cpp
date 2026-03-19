#include <Arduino.h> // <-- LA LIGNE MAGIQUE POUR PLATFORMIO

// ==========================================
// TEST UNITAIRE : MARCHE AVANT DU CONVOYEUR
// ==========================================

// Broches de ton moteur (d'après ta configuration)
#define MOTEUR_DIR1 20 // fil jaune
#define MOTEUR_DIR2 21 // fil vert

void setup() {
  // Démarrage du moniteur série pour le diagnostic
  Serial.begin(115200);
  delay(1000); // Petite pause pour laisser le port série s'ouvrir
  Serial.println("=== TEST MOTEUR : DÉMARRAGE ===");

  // Configuration des broches en sortie
  pinMode(MOTEUR_DIR1, OUTPUT);
  pinMode(MOTEUR_DIR2, OUTPUT);

  // Commande d'avance (Basée sur ta classe MoteurConvoyeur)
  digitalWrite(MOTEUR_DIR1, HIGH);
  digitalWrite(MOTEUR_DIR2, LOW);

  Serial.println(">>> Moteur en MARCHE AVANT continue...");
}

void loop() {
  // On ne met rien ici. 
  // L'état des broches reste mémorisé, le moteur tourne à l'infini.
}