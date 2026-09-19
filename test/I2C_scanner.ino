#include <Arduino.h>
#include <Wire.h>

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 4000)
    {
        // wait for serial monitor
    }

    Wire.begin();

    Serial.println("\nI2C Scanner");
}

void loop()
{
    Serial.println("Scanning...");

    int devicesFound = 0;

    for (uint8_t address = 1; address < 127; address++)
    {
        Wire.beginTransmission(address);
        uint8_t error = Wire.endTransmission();

        if (error == 0)
        {
            Serial.print("I2C device found at address 0x");
            if (address < 16)
                Serial.print("0");
            Serial.println(address, HEX);
            devicesFound++;
        }
        else if (error == 4)
        {
            Serial.print("Unknown error at address 0x");
            if (address < 16)
                Serial.print("0");
            Serial.println(address, HEX);
        }
    }

    if (devicesFound == 0)
        Serial.println("No I2C devices found\n");
    else
        Serial.println("Scan complete\n");

    delay(3000);
}