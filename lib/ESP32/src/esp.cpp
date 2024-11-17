#include "esp32.h"

String serString1;

void ESP32::sendMessage(const String& topic, const String& message){
    Serial1.println(topic + String(':') + message);
    #ifdef __ESP32_DEBUG__
        Serial.println(topic + String(':') + message);
    #endif
}

bool ESP32::checkSerial1(){
    while(Serial1.available())
    {
        char c = Serial1.read();
        serString1 += c;

        if(c == '\n')
        {
            return true;
        }
    }

    return false;
}