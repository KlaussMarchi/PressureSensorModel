#include <ArduinoJson.h>
#include <HX711.h>
#include <Wire.h>

#define LOADCELL_DOUT_PIN 4
#define LOADCELL_SCK_PIN 5
HX711 scale;

unsigned long espMillis(){
    return esp_timer_get_time()/1000;
}

void setup(){
    Serial.begin(115200);
    Wire.begin(18, 19);
    scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
    
    while(!Serial.available())
        continue;
}

void loop(){
    static const unsigned long startTime = espMillis();
    StaticJsonDocument<256> data;
    String response;

    data["time"]     = float(espMillis() - startTime) / 1000.0;
    data["pressure"] = float(scale.read() / 10000.0);

    serializeJson(data, response);
    Serial.println(response);
}

