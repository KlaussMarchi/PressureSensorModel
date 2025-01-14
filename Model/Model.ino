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
    static const byte size = 7;
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
    const float ratio = (pressure - mean) / mean * 100;   

    for(int n=size-1; n>0; n--) 
        Yn[n] = Yn[n-1];
    
    Yn[0] = ratio;
    const float pBlow    = (1.0 / (1.0 + exp(-(-8.543893 + Yn[0]*(2.848940)  + Yn[1]*(1.060645)  + Yn[2]*(-0.107047) + Yn[3]*(-0.545528) + Yn[4]*(-0.403575) + Yn[5]*(0.004101) + Yn[6]*(0.369147)  + Yn[7]*(0.474087)  + Yn[8]*(0.271210)  + Yn[9]*(-0.088831) + Yn[10]*(-0.353993) + Yn[11]*(-0.365422) + Yn[12]*(-0.167581) + Yn[13]*(0.087180) + Yn[14]*(0.258654)  + Yn[15]*(0.286384)  + Yn[16]*(0.103591)  + Yn[17]*(-0.112949) + Yn[18]*(-0.047308) + Yn[19]*(0.108799) + Yn[20]*(0.086008) + Yn[21]*(-0.021517) + Yn[22]*(-0.088323) + Yn[23]*(-0.079275) + Yn[24]*(-0.041819) + Yn[25]*(-0.024299) + Yn[26]*(-0.023152) + Yn[27]*(-0.002426) + Yn[28]*(0.070831)  + Yn[29]*(0.207190)))));
    const float pNotBlow = (1.0 / (1.0 + exp(-(-8.123264 + Yn[0]*(-4.590411) + Yn[1]*(-2.229742) + Yn[2]*(-0.645425) + Yn[3]*(0.112559)  + Yn[4]*(0.268435)  + Yn[5]*(0.096018) + Yn[6]*(-0.150878) + Yn[7]*(-0.231400) + Yn[8]*(-0.116921) + Yn[9]*(0.079551)  + Yn[10]*(0.281014)  + Yn[11]*(0.381144)  + Yn[12]*(0.293945)  + Yn[13]*(0.054571) + Yn[14]*(-0.202666) + Yn[15]*(-0.329408) + Yn[16]*(-0.272321) + Yn[17]*(-0.073560) + Yn[18]*(0.143963)  + Yn[19]*(0.202819) + Yn[20]*(0.065134) + Yn[21]*(-0.035419) + Yn[22]*(0.008357)  + Yn[23]*(0.006768)  + Yn[24]*(-0.045385) + Yn[25]*(-0.016880) + Yn[26]*(0.019709)  + Yn[27]*(-0.034382) + Yn[28]*(-0.074823) + Yn[29]*(-0.005556)))));

    if(pBlow > 0.5)
        is_blowing = true;

    if(pNotBlow > 0.5 || reset)
        is_blowing = false;

    return is_blowing;
}

void tarePressure(){
    const unsigned long startTime = espMillis();
    Serial.println("tentando...");

    while(espMillis() - startTime < 5000)
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
