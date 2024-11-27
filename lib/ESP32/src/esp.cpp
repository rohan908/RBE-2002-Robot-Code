#include "esp32.h"

String serString1;

void ESP32::sendMessage(const String& topic, const String& message){
    Serial1.println("RohanRobot/" + topic + String(':') + message);
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

#ifdef __ESP_DEBUG__
void ESP32::heartbeat(){
    static uint32_t lastSend = 0;
    uint32_t currTime = millis();
    if(currTime - lastSend >= 5000) //send every five seconds
    {
        lastSend = currTime;
        Serial.println("trying to send message");
        sendMessage("timer/time", String(currTime));
    }

    // Check to see if we've received anything
    if(checkSerial1())
    {
        Serial.print("Rec'd:\t");
        Serial.print(serString1);
        serString1 = "";
    }
}
#endif