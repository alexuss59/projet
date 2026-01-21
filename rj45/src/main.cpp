#include <Arduino.h>
#include <SPI.h>
#include <Ethernet.h>
#include <WiFi.h> // On inclut ça juste pour récupérer la MAC unique de l'ESP

// Pins pour l'ESP32-S3
const int PIN_CS_ETHERNET = 10; 

void setup() {
  Serial.begin(115200);
  
  // 1. Initialiser la broche CS
  Ethernet.init(PIN_CS_ETHERNET);

  // 2. L'ASTUCE : Récupérer l'adresse MAC unique de l'ESP32
  byte mac[6];
  WiFi.macAddress(mac); 
  
  // Petite modif pour dire que c'est de l'Ethernet (optionnel mais propre)
  // On change juste le dernier chiffre pour qu'elle soit différente du WiFi
  mac[5] += 1; 

  Serial.print("Mon Adresse MAC est : ");
  for (int i = 0; i < 6; i++) {
    Serial.print(mac[i], HEX);
    if (i < 5) Serial.print(":");
  }
  Serial.println();

  // 3. Démarrer Ethernet avec cette MAC dynamique
  Serial.println("Connexion au réseau...");
  if (Ethernet.begin(mac) == 0) {
    Serial.println("ERREUR : Pas de DHCP (Verifie le cable)");
  } else {
    Serial.print("REUSSI ! Mon IP est : ");
    Serial.println(Ethernet.localIP());
  }
}

void loop() {
  // Rien
}