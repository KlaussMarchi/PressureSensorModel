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
    const float pBlow    = (1.0 / (1.0 + exp(-(-12.723226 + mean*(-0.001074) + Yn[0]*(3.135929) + Yn[1]*(0.819931) + Yn[2]*(0.038669) + Yn[3]*(-0.116841) + Yn[4]*(-0.077378) + Yn[5]*(-0.075347) + Yn[6]*(-0.082068) + Yn[7]*(0.004154) + Yn[8]*(0.043705) + Yn[9]*(0.085579) + Yn[10]*(0.140492) + Yn[11]*(0.086938) + Yn[12]*(-0.032627) + Yn[13]*(-0.127955) + Yn[14]*(-0.240136) + Yn[15]*(-0.193426) + Yn[16]*(-0.061131) + Yn[17]*(0.045340) + Yn[18]*(0.198331) + Yn[19]*(0.169784) + Yn[20]*(0.043824) + Yn[21]*(-0.026014) + Yn[22]*(-0.004750) + Yn[23]*(0.022710) + Yn[24]*(0.049700) + Yn[25]*(0.023259) + Yn[26]*(-0.039008) + Yn[27]*(0.003634) + Yn[28]*(-0.038732) + Yn[29]*(-0.032282)))));
    const float pNotBlow = (1.0 / (1.0 + exp(-(-4.019635 + mean*(0.000839) + Yn[0]*(-1.988922) + Yn[1]*(-0.944138) + Yn[2]*(-0.289720) + Yn[3]*(0.074394) + Yn[4]*(0.186094) + Yn[5]*(0.157274) + Yn[6]*(0.073803) + Yn[7]*(0.018157) + Yn[8]*(-0.025524) + Yn[9]*(-0.039075) + Yn[10]*(-0.020316) + Yn[11]*(-0.001538) + Yn[12]*(0.016627) + Yn[13]*(0.020282) + Yn[14]*(0.017750) + Yn[15]*(0.006238) + Yn[16]*(0.005705) + Yn[17]*(0.005668) + Yn[18]*(0.013519) + Yn[19]*(-0.015652) + Yn[20]*(-0.026465) + Yn[21]*(-0.052845) + Yn[22]*(-0.075128) + Yn[23]*(-0.041471) + Yn[24]*(0.025289) + Yn[25]*(0.101555) + Yn[26]*(0.094913) + Yn[27]*(0.036083) + Yn[28]*(-0.052047) + Yn[29]*(-0.161567)))));

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
