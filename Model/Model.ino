#include <HX711.h>
#include <Wire.h>
#include <ArduinoJson.h>

#define LOADCELL_DOUT_PIN 4
#define LOADCELL_SCK_PIN  5

HX711 scale;
bool press_debug = false;

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

bool isBlowing(bool reset){
    static float Pn1, Pn2, Pn3, Pn4;
    static float Dn1, Dn2, Dn3, Dn4;
    static unsigned long startTime;
    static bool is_blowing = false;
    
    if(espMillis() - startTime < 100)
        return is_blowing;
    
    startTime = espMillis();
    const float Pn = getPressure();
    const float Dn = getPressureVariation(Pn);

    const float pBlow    = (1.0 / (1.0 + exp(-(0.359894 + Pn*(-0.137681) + Pn1*(-0.067455) + Pn2*(-0.049503) + Pn3*(-0.001147) + Pn4*(0.014043) + Dn*(1.702614) + Dn1*(0.990425) + Dn2*(0.185378) + Dn3*(-0.868179) + Dn4*(0.540314)))));
    const float pNotBlow = (1.0 / (1.0 + exp(-(1.355041 + Pn*(-0.362906) + Pn1*(0.501752) + Pn2*(0.042881) + Pn3*(0.025190) + Pn4*(-0.423810) + Dn*(-6.137248) + Dn1*(0.010673) + Dn2*(-0.055308) + Dn3*(-0.198588) + Dn4*(0.147152)))));
    
    if(pBlow > 0.5)
        is_blowing = true;

    if(pNotBlow > 0.5 || reset)
        is_blowing = false;

    Pn4 = Pn3; Pn3 = Pn2; Pn2 = Pn1; Pn1 = Pn;
    Dn4 = Dn3; Dn3 = Dn2; Dn2 = Dn1; Dn1 = Dn;

    Serial.println(is_blowing);
    return press_debug ? true : is_blowing;
}

void tarePressure(){
    const unsigned long startTime = espMillis();
    Serial.println("tentando...");

    while(espMillis() - startTime < 3000)
        isBlowing(true);

    if(!isBlowing(false))
        return;

    tarePressure();
}

void setup(){
    Serial.begin(115200);
    Wire.begin(18, 19);
    scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
    tarePressure();
}

void loop(){
    isBlowing(false);
}
