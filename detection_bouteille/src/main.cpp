#include <Arduino.h>
#include "esp_timer.h"

const int PIN_CAPTEUR_IR = 4; 
const uint64_t DELAI_FILTRE_US = 400 * 1000; 

esp_timer_handle_t timer_filtre_handle; 
portMUX_TYPE myMux = portMUX_INITIALIZER_UNLOCKED;

volatile bool event_bouteille_arrivee = false;
volatile bool event_bouteille_partie = false;
volatile bool bouteille_en_cours = false; 

void IRAM_ATTR on_timer_fin_bouteille(void* arg) {
  portENTER_CRITICAL_ISR(&myMux);
  event_bouteille_partie = true;
  portEXIT_CRITICAL_ISR(&myMux);
}

void IRAM_ATTR isr_capteur_change() {
  int niveau = digitalRead(PIN_CAPTEUR_IR); 
  
  if (niveau == LOW) {
    esp_timer_stop(timer_filtre_handle);
    
    if (!bouteille_en_cours) {
      bouteille_en_cours = true;
      portENTER_CRITICAL_ISR(&myMux);
      event_bouteille_arrivee = true;
      portEXIT_CRITICAL_ISR(&myMux);
    }
  } 
  else {
    esp_timer_start_once(timer_filtre_handle, DELAI_FILTRE_US);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_CAPTEUR_IR, INPUT_PULLUP);

  const esp_timer_create_args_t timer_args = {
    .callback = &on_timer_fin_bouteille,
    .name = "timer_filtre_ir"
  };
  esp_timer_create(&timer_args, &timer_filtre_handle);

  attachInterrupt(digitalPinToInterrupt(PIN_CAPTEUR_IR), isr_capteur_change, CHANGE);
}

void loop() {
  bool debut = false;
  bool fin = false;

  portENTER_CRITICAL(&myMux);
  if (event_bouteille_arrivee) {
    debut = true;
    event_bouteille_arrivee = false;
  }
  if (event_bouteille_partie) {
    fin = true;
    event_bouteille_partie = false;
    bouteille_en_cours = false; 
  }
  portEXIT_CRITICAL(&myMux);

  if (debut) {
    Serial.println(">>> DEBUT DETECTION");
  }

  if (fin) {
    Serial.println("<<< FIN DETECTION");
  }
  
  delay(1); 
}