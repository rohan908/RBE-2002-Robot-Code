#pragma once
#include <Arduino.h>

class ESP32{
    public:
        /**
         * parameters:
         * topic: topic_name/var_name -> (you can add more preceding topics as well)
         * message: message to send that is assigned to the var name
         */
        void sendMessage(const String& topic, const String& message);
        void getMessage(void);
        #ifdef __ESP_DEBUG__
        void heartbeat();
        #endif
    
    private:
        bool checkSerial1(void);
    
};
