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

float getPressure(){
    static unsigned long startTime = espMillis();
    static double X_n1, Y_n1;

    if(espMillis() - startTime < 100)
        return Y_n1;

    startTime = espMillis();
    double X_n = double(scale.read() / 10000.0);
    double Y_n = 0.4665119089088967*X_n + 0.5334880910911033*Y_n1;
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

bool isBlowing(bool reset){
    static float Pn1, Pn2, Pn3, Pn4;
    static float Dn1, Dn2, Dn3, Dn4;
    static unsigned long startTime;
    static bool is_blowing = false;
    
    if(espMillis() - startTime < 100)
        return is_blowing;
    
    startTime = espMillis();
    const float Pn = getPressure();
    const float Dn = getPressureDerivative(Pn);

    const float pBlow    = (1.0 / (1.0 + exp(-(-7.831586 + Pn*(-0.208470) + Pn1*(0.250839)  + Pn2*(-0.001847) + Pn3*(-0.251983) + Pn4*(0.144647) + Dn*(2.159529)  + Dn1*(1.412934) + Dn2*(-1.313097) + Dn3*(0.817576) + Dn4*(-0.418235)))));
    const float pNotBlow = (1.0 / (1.0 + exp(-(-3.774275 + Pn*(-0.223280) + Pn1*(-0.043048) + Pn2*(0.033473) + Pn3*(-0.031903) + Pn4*(0.120884) + Dn*(-2.590689) + Dn1*(-1.163686) + Dn2*(1.051179) + Dn3*(-0.568338) + Dn4*(0.287333)))));

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

    while(espMillis() - startTime < 2000)
        isBlowing(true);
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
