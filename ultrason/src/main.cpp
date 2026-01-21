#include <Arduino.h>
//
//
#include <Wire.h>


int reading = 0;
int sda=13;
int scl=15;
int addressI2C = 0x70; //=112 en décimal

void setup()
{
  Wire.begin(sda,scl);       // join i2c bus (address optional for master)
  Serial.begin(9600); // start serial communication at 9600bps
}

void loop()
{
  // step 1: instruct sensor to read echoes
  Wire.beginTransmission(addressI2C); // transmit to device #112 (0x70)
  // the address specified in the datasheet is 224 (0xE0)
  // but i2c adressing uses the high 7 bits so it's 112
  Wire.write(byte(0x00)); // sets register pointer to the command register (0x00)
  Wire.write(byte(0x51)); // command sensor to measure in "centimeters" (0x51)
  // use 0x51 for centimeters
  // use 0x52 for ping microseconds
  Wire.endTransmission(); // stop transmitting

  // step 2: wait for readings to happen
  delay(70); // datasheet suggests at least 65 milliseconds

  // step 3: instruct sensor to return a particular echo reading
  Wire.beginTransmission(addressI2C); // transmit to device #112
  Wire.write(byte(0x02));      // sets register pointer to echo #1 register (0x02)
  Wire.endTransmission();      // stop transmitting

  // step 4: request reading from sensor
  Wire.requestFrom(addressI2C, 2); // request 2 bytes from slave device #112

  // step 5: receive reading from sensor
  if (2 <= Wire.available()) // if two bytes were received
  {
    reading = Wire.read();  // receive high byte (overwrites previous reading)
    reading = reading << 8; // shift high byte to be high 8 bits
    reading |= Wire.read(); // receive low byte as lower 8 bits
    Serial.print(reading);  // print the reading
    Serial.println("cm");
  }

  delay(250); // wait a bit since people have to read the output :)
}
