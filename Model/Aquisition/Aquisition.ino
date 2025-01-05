#include <HX711.h>
#include <Wire.h>
#include <ArduinoJson.h>

#define LOADCELL_DOUT_PIN 4
#define LOADCELL_SCK_PIN 5

HX711 scale;
unsigned long startProg;

unsigned long espMillis(){
    return esp_timer_get_time()/1000;
}

float smooth(float value){
    static const byte size = 5;
    static float array[size];
    static float sum = 0;
    static byte i = 0;

    sum = sum - array[i];
    array[i] = value;

    sum = sum + array[i];
    i = (i + 1 < size) ? (i + 1) : 0;
    return (sum / size);
}

float getPressure(){
    static unsigned long startTime;
    static float X_n1, Y_n1;

    if(espMillis() - startTime < 100)
        return Y_n1;

    startTime = espMillis();
    float X_n = float(scale.read() / 10000.0);
    float Y_n = X_n*(0.124827) + Y_n1*(0.875173);
    X_n1 = X_n;
    Y_n1 = Y_n;
    return Y_n;
}

float getPressureVariation(float newValue){
    static float previousValue;
    static float previousDerivative;

    if(newValue == previousValue)
        return previousDerivative;
    
    static const float dt = 0.100;
    const float derivative = (newValue - previousValue) / dt;

    previousDerivative = derivative;
    previousValue      = newValue;
    return smooth(derivative);
}

void setup(){
    Serial.begin(115200);
    Wire.begin(18, 19);
    scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

    while(!Serial.available())
        continue;

    startProg = espMillis();
}

void loop(){
    static unsigned long startTime;

    if(espMillis() - startTime < 100)
        return;
    
    startTime = espMillis();
    const float pressure   = getPressure();
    const float derivative = getPressureVariation(pressure);
    const float timePassed = (espMillis() - startProg) / 1000.0;

    StaticJsonDocument<256> data;
    String response;
    data["time"] = timePassed;
    data["pressure"]   = pressure;
    data["derivative"] = derivative;
    serializeJson(data, response);
    Serial.println(response);

    if(timePassed > 200.0)
        ESP.restart();
}

