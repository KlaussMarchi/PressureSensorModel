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
    const float pBlow    = (1.0 / (1.0 + exp(-(-7.451511 + mean*(0.047237) + Yn[0]*(10.632947) + Yn[1]*(0.737019) + Yn[2]*(-1.950293) + Yn[3]*(-0.531354) + Yn[4]*(1.408774) + Yn[5]*(1.928095) + Yn[6]*(1.165842) + Yn[7]*(0.134007) + Yn[8]*(-1.027697) + Yn[9]*(-1.256293) + Yn[10]*(-1.102414) + Yn[11]*(-0.625669) + Yn[12]*(0.061847) + Yn[13]*(0.543553) + Yn[14]*(1.053867) + Yn[15]*(0.836082) + Yn[16]*(-0.307825) + Yn[17]*(-0.730549) + Yn[18]*(-0.791620) + Yn[19]*(-0.413691) + Yn[20]*(0.155163) + Yn[21]*(0.810035) + Yn[22]*(0.489494) + Yn[23]*(0.532470) + Yn[24]*(-0.074235) + Yn[25]*(-0.033409) + Yn[26]*(-0.353952) + Yn[27]*(-0.520150) + Yn[28]*(-0.086801) + Yn[29]*(0.920368)))));
    const float pNotBlow = (1.0 / (1.0 + exp(-(-16.969145 + mean*(0.110913) + Yn[0]*(-19.796728) + Yn[1]*(-9.309003) + Yn[2]*(-1.474165) + Yn[3]*(1.947950) + Yn[4]*(2.570617) + Yn[5]*(1.383755) + Yn[6]*(-0.298808) + Yn[7]*(-1.100382) + Yn[8]*(-1.444671) + Yn[9]*(-0.886527) + Yn[10]*(-0.408624) + Yn[11]*(-0.673584) + Yn[12]*(-0.011276) + Yn[13]*(0.619164) + Yn[14]*(0.911600) + Yn[15]*(0.736368) + Yn[16]*(0.502654) + Yn[17]*(0.424793) + Yn[18]*(0.239187) + Yn[19]*(0.336186) + Yn[20]*(0.017092) + Yn[21]*(-0.167252) + Yn[22]*(-0.095511) + Yn[23]*(0.019943) + Yn[24]*(-0.093213) + Yn[25]*(-0.220906) + Yn[26]*(-0.261631) + Yn[27]*(-0.669120) + Yn[28]*(-0.451332) + Yn[29]*(0.666113)))));
    
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
