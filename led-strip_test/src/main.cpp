#include <Arduino.h>
// Pour adapter la lib FastLED à l'ESP
#define FASTLED_ESP32_RAW_PIN_ORDER
// ajout de la lib
#include <FastLED.h>

#define NUM_LEDS 10 // nombre de LED du ruban
#define DATA_PIN 2  // D4 broche du bus de commande du ruban

int compteur = 0;
unsigned long dernierTemps = 0;
const long intervalle = 500; // 1 seconde (1000 ms)
bool estAllume = false;

// Création d'un tableau pour stocker et contrôler les LEDS
CRGB leds[NUM_LEDS];

void setup()
{
  // configuration du ruban de LEDs
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
}

void loop()
{
  unsigned long tempsActuel = millis();

    if (tempsActuel - dernierTemps >= intervalle)
    {
      dernierTemps = tempsActuel;
      estAllume = !estAllume;
      
      if (estAllume);
      
    }

  FastLED.show();
}
