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
    const float pressure = getPressure();
    const float mean  = smooth(pressure);
    const float ratio = (pressure - mean) / mean * 100; 
    const float timePassed = (espMillis() - startProg) / 1000.0;

    StaticJsonDocument<256> data;
    String response;
    data["time"] = timePassed;
    data["pressure"] = pressure;
    data["ratio"]    = ratio;
    serializeJson(data, response);
    Serial.println(response);

    if(timePassed > 200.0)
        ESP.restart();
}

