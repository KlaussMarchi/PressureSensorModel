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

float getPressure(){
    static unsigned long startTime = espMillis();
    static float X_n1, Y_n1;

    if(espMillis() - startTime < 100)
        return Y_n1;

    startTime = espMillis();
    float X_n = float(scale.read() / 10000.0);
    float Y_n = 0.3558495560245919*X_n + 0.6441504439754081*Y_n1;
    X_n1 = X_n;
    Y_n1 = Y_n;
    return Y_n;
}

float getPressureDerivative(float newValue){
    static float previousValue;
    static float previousDerivative;

    if(newValue == previousValue)
        return previousDerivative;
    
    static const float dt = 0.100;
    const float derivative = (newValue - previousValue) / dt;

    previousDerivative = derivative;
    previousValue      = newValue;
    return derivative;
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
    static unsigned long startTime = espMillis();

    if(espMillis() - startTime < 100)
        return;
    
    startTime = espMillis();
    const float pressure   = getPressure();
    const float derivative = getPressureDerivative(pressure);
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

