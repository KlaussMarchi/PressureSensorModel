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
    static const byte size = 20;
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
    static const byte xSize=2, ySize=3;
    static float Xn[xSize] = {0};
    static float Yn[ySize] = {0};

    if(espMillis() - startTime < 100)
        return Yn[0];

    startTime = espMillis();

    for(byte n=xSize-1; n>0; n--)
        Xn[n] = Xn[n-1];

    for(byte n=ySize-1; n>0; n--) 
        Yn[n] = Yn[n-1];

    Xn[0] = float(scale.read() / 10000.0);
    Yn[0] = Xn[0]*(0.016239) + Xn[1]*(0.014858) + Yn[1]*(1.734903) + Yn[2]*(-0.766000);
    return Yn[0];
}

bool isBlowing(bool reset){
    static unsigned long startTime;
    static bool is_blowing = false;
    static const byte size = 30;
    static float Yn[size]  = {0};

    if(press_debug)
        return true;

    if(espMillis() - startTime < 100)
        return is_blowing;

    startTime = espMillis();
    const float pressure = getPressure();
    const float mean  = smooth(pressure);
    const float ratio = (pressure - mean);

    for(int n=size-1; n>0; n--) 
        Yn[n] = Yn[n-1];
    
    Yn[0] = ratio;
    const float pBlow    = (1.0 / (1.0 + exp(-(-11.466047 + mean*(-0.001163) + Yn[0]*(3.025957) + Yn[1]*(0.501925) + Yn[2]*(0.116958) + Yn[3]*(0.043823) + Yn[4]*(-0.077863) + Yn[5]*(-0.178389) + Yn[6]*(0.011499) + Yn[7]*(-0.034682) + Yn[8]*(0.081269) + Yn[9]*(0.186142) + Yn[10]*(0.096591) + Yn[11]*(0.004255) + Yn[12]*(-0.343280) + Yn[13]*(-0.082784) + Yn[14]*(-0.242662) + Yn[15]*(-0.128529) + Yn[16]*(0.097176) + Yn[17]*(0.410877) + Yn[18]*(0.080874) + Yn[19]*(0.119035) + Yn[20]*(-0.026173) + Yn[21]*(-0.017991) + Yn[22]*(-0.184531) + Yn[23]*(-0.140506) + Yn[24]*(-0.072026) + Yn[25]*(0.042958) + Yn[26]*(0.327036) + Yn[27]*(0.191725) + Yn[28]*(-0.014662) + Yn[29]*(-0.218431)))));
    const float pNotBlow = (1.0 / (1.0 + exp(-(-3.343228 + mean*(-0.002885) + Yn[0]*(-2.188255) + Yn[1]*(-0.779892) + Yn[2]*(-0.162790) + Yn[3]*(0.410243) + Yn[4]*(0.388240) + Yn[5]*(0.093102) + Yn[6]*(-0.155476) + Yn[7]*(-0.238493) + Yn[8]*(-0.204821) + Yn[9]*(0.007323) + Yn[10]*(0.132886) + Yn[11]*(0.118478) + Yn[12]*(0.102561) + Yn[13]*(0.000825) + Yn[14]*(-0.132400) + Yn[15]*(0.103011) + Yn[16]*(-0.025502) + Yn[17]*(0.096225) + Yn[18]*(-0.084072) + Yn[19]*(0.049787) + Yn[20]*(-0.244701) + Yn[21]*(0.031043) + Yn[22]*(-0.050779) + Yn[23]*(-0.009207) + Yn[24]*(-0.036644) + Yn[25]*(0.171826) + Yn[26]*(0.013527) + Yn[27]*(0.094782) + Yn[28]*(-0.002178) + Yn[29]*(-0.211547)))));
    
    if(pBlow > 0.5)
        is_blowing = true;
    
    if(pNotBlow > 0.5 || reset)
        is_blowing = false;

    Serial.println(is_blowing);
    return is_blowing;
}

void tarePressure(){
    const unsigned long startTime = espMillis();
    Serial.println("tentando...");

    while(espMillis() - startTime < 7000)
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
