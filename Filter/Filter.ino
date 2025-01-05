#include <ArduinoJson.h>
#include <HX711.h>
#include <Wire.h>

#define LOADCELL_DOUT_PIN 4
#define LOADCELL_SCK_PIN 5
HX711 scale;

unsigned long espMillis(){
    return esp_timer_get_time()/1000;
}

float getPressure(){
    static unsigned long startTime;
    static float X_n1, Y_n1;

    if(espMillis() - startTime < 100)
        return Y_n1;

    startTime = espMillis();
    float X_n = float(scale.read() / 10000.0);
    float Y_n = X_n*(0.329680) + Y_n1*(0.670320);
    X_n1 = X_n;
    Y_n1 = Y_n;

    Serial.println(String(X_n) + "," + String(Y_n));
    return Y_n;
}

void setup(){
    Serial.begin(115200);
    Wire.begin(18, 19);
    scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
    
    while(!Serial.available())
        continue;
}

void loop(){
    getPressure();
}

