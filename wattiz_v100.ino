/*********
wattiz_v100  version 100 pour IDE Arduino pour regler pb d'access point
28 fev 2023

Setting AP (Access Point)…AP IP address: 192.168.4.1
AP : WATTIZ 
ouvert, pas de password
esp8266 : 4MB / FS 1MB / OTA 1MB
*********/

/*contribution *********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-esp8266-input-data-html-form/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*********/

#include <Arduino.h>
#include <Adafruit_GFX.h> 
#include <Adafruit_ST7735.h>
#include <SPI.h>

/*

#ifdef ESP32
  #include <WiFi.h>
  #include <AsyncTCP.h>
  //#include <SPIFFS.h>
#else
*/
  #include <ESP8266WiFi.h>
  #include <ESPAsyncTCP.h>
  #include <Hash.h>
  //#include <FS.h>
/*
#endif
*/
#include <ESPAsyncWebServer.h>
#include <arduino-timer.h>
#include <LittleFS.h>


const char* LENOM = "WATTIZ";
String LaVersion = " v100_280223";

int decalageAff = 10;

// parametres de calculs récuépés par getconsignes
float p_cout;       //€ par kWh
int   p_tension;    // tension de reference EDF
float p_coef;       // coef de calibrage
int p_typeACS;      // type de ACS712 5, 20 , 30
double p_offesetcalib = 0.0; //offset de calibrage au zero

String yourStrLoc;
float yourfltKWH;
int yourintEDF;
float yourfltCoefCalb;
int yourintACS;
float yourfltOffsetCalib;

// parametrage ecran 1.44
  // For the breakout, you can use any 2 or 3 pins
  // These pins will also work for the 1.8" TFT shield
#define TFT_CS     D4 //10
#define TFT_RST    9  // you can also connect this to the Arduino reset
                      // in which case, set this #define pin to -1!
#define TFT_DC     D3   //8
// For 1.44" and 1.8" TFT with ST7735 use
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS,  TFT_DC, TFT_RST);

//parametrage ACS712
const int sensorIn = A0;      // pin where the OUT pin from sensor is connected on Arduino
int mVperAmp5A = 185;           // this the 5A version of the ACS712 -use 100 for 20A Module and 66 for 30A Module
int mVperAmp20A = 100;
int mVperAmp30A = 66;
int mVperAmp = mVperAmp30A;  // par defaut le plus fort
int Watt = 0;
double Voltage = 0;
double VRMS = 0;
double AmpsRMS = 0;
double tension5Vref = 5.0;
 

AsyncWebServer server(80);

const char* ssid = "WATTIZ";
const char* password = "";

auto timerGlobal = timer_create_default();
bool MesurePossible = false;
WiFiEventHandler probeRequestHandler;
WiFiEventHandler stationConnectedHandler;
WiFiEventHandler stationDisconnectedHandler;
int deviceconnecte;

const char* PARAM_STR_Localisation = "inputStrLocalisation";
const char* PARAM_FLOAT_Cout_kwh = "inputFloatKWH";
const char* PARAM_INT_EDF = "inputIntEDF";
const char* PARAM_FLOAT_Coef_calib = "inputFloatCoefCalib";
const char* PARAM_INT_ACS = "inputIntACS";
const char* PARAM_FLOAT_Offset_calib = "inputFloatOffsetCalib";

String macToString(const uint8 mac[6]) {
  char buf[20];
  snprintf(buf, sizeof(buf), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(buf);
}

void onProbeRequest(const WiFiEventSoftAPModeProbeRequestReceived& evt) {
      Serial.print("New device found");
}

void onStationConnected(const WiFiEventSoftAPModeStationConnected& evt) {
  String AdrConnecte;
  AdrConnecte = macToString(evt.mac);
  Serial.print(AdrConnecte);
  Serial.println(" est connecté");

  deviceconnecte++;
  MesurePossible=false;

}
void onStationDisconnected(const WiFiEventSoftAPModeStationDisconnected& evt) {
  String AdrDeconnecte;
  AdrDeconnecte = macToString(evt.mac);
  Serial.print(AdrDeconnecte);
  Serial.println("  est déconnecté");
  deviceconnecte--;
  if (deviceconnecte<1){
    MesurePossible=true;
  }

}


// HTML web page to handle 6 input fields
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
  <title>WATTIZ</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <script>
    function submitMessage() {
      alert("Valeur sauvegardee sur WATTIZ");
      setTimeout(function(){ document.location.reload(false); }, 500);   
    }
  </script></head><body>
  <h1>WATTIZ</h1>
  <h2>Saisie du parametrage</h2>
  <form action="/get" target="hidden-form">
    Localisation (valeur actuelle : %inputStrLocalisation%): <input type="text" name="inputStrLocalisation">
    <input type="submit" value="Submit" onclick="submitMessage()">
  </form><br>
  <form action="/get" target="hidden-form">
    Prix kHh (valeur actuelle : %inputFloatKWH% euro): <input type="number " name="inputFloatKWH">
    <input type="submit" value="Submit" onclick="submitMessage()">
  </form> <br> 
  <form action="/get" target="hidden-form">
    Tension EDF (valeur actuelle : %inputIntEDF% V): <input type="number " name="inputIntEDF">
    <input type="submit" value="Submit" onclick="submitMessage()">
  </form><br>
  <form action="/get" target="hidden-form">
    Coef Calib (valeur actuelle : %inputFloatCoefCalib%): <input type="number " name="inputFloatCoefCalib">
    <input type="submit" value="Submit" onclick="submitMessage()">
  </form> <br>   
  <form action="/get" target="hidden-form">
    Type ACS (valeur actuelle : %inputIntACS% A): <input type="number " name="inputIntACS">
    <input type="submit" value="Submit" onclick="submitMessage()">
  </form><br>  
  <form action="/get" target="hidden-form">
    Offset Calib (valeur actuelle : %inputFloatOffsetCalib% V): <input type="number " name="inputFloatOffsetCalib">
    <input type="submit" value="Submit" onclick="submitMessage()">
  </form> <br>  
  <iframe style="display:none" name="hidden-form"></iframe>
  <br>
  <br>
  Relancer WATTIZ apres modification pour une prise en compte des nouveaux parametres.
  <br>
  <p>propulse par Acefou - %LaVersion%</p>
</body></html>)rawliteral";

void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}

String readFile(const char * path){
  Serial.printf("Reading file: %s\r\n", path);
  File file = LittleFS.open(path, "r");
  if(!file || file.isDirectory()){
    Serial.println("- empty file or failed to open file");
    return String();
  }
  Serial.println("- read from file:");
  String fileContent;
  while(file.available()){
    fileContent+=String((char)file.read());
  }
  file.close();
  Serial.println(fileContent);
  return fileContent;
}

void writeFile( const char * path, const char * message){
  Serial.printf("Writing file: %s\r\n", path);
  File file = LittleFS.open(path, "w");
  if(!file){
    Serial.println("- failed to open file for writing");
    return;
  }
  if(file.print(message)){
    Serial.println("- file written");
  } else {
    Serial.println("- write failed");
  }
  file.close();
}

// Replaces placeholder with stored values
String processor(const String& var){
  Serial.println("processor " + var);
  if(var == "inputStrLocalisation"){
    return readFile("/inputStrLocalisation.txt");
  }
  else if(var == "inputFloatKWH"){
    return readFile("/inputFloatKWH.txt");
  }
  else if(var == "inputIntEDF"){
    return readFile("/inputIntEDF.txt");
  }
  else if(var == "inputFloatCoefCalib"){
    return readFile("/inputFloatCoefCalib.txt");
  }  
  else if(var == "inputIntACS"){
    return readFile("/inputIntACS.txt");
  }
  else if(var == "inputFloatOffsetCalib"){
    return readFile("/inputFloatOffsetCalib.txt");
  }
  else if(var == "LaVersion"){
    return LaVersion;
  }  
  
  return String();
}


void tftdrawtext(char *text, uint16_t color, int16_t px, int16_t py){
  tft.setCursor( px, py );
  tft.setTextColor(color);
  tft.setTextWrap(true);
  tft.print(text);
}

void tftPrintString(String LeStr, uint16_t color, int16_t px, int16_t py){

  char c[128];
  LeStr.toCharArray(c,sizeof(c));
  tftdrawtext(c,color,px, py);

}

void RecupConsigne(){

/*
 * //reponse: {"active_photo":"0","nb_prise":"0","active_allumage":"0","MaxiVisionNuit":"200","MiniVisionJour":"160","DelaiPrisePhoto":"5","CommandeAD":"1"}
 */
  
  Serial.println("RecupConsigne...");



      //Valeurs par défaut
        p_cout    = 0.1661; //€ par kWh  22Fev2023
        p_tension = 234;    // tension de reference EDF
        p_coef    = 1.3;    // coef de calibrage
        p_typeACS = 30;
        mVperAmp=mVperAmp30A;
        p_offesetcalib=0.0;
        

  yourStrLoc = readFile("/inputStrLocalisation.txt");
  Serial.print("*** Your yourStrLoc: ");
  Serial.println(yourStrLoc);

  yourfltKWH = readFile("/inputFloatKWH.txt").toFloat();
  if ((yourfltKWH>0.1) && (yourfltKWH<1.0)) {
      p_cout=yourfltKWH;
  }else{
      p_cout=0.1661; //€ par kWh  22Fev2023
  }
  Serial.print("*** Your yourfltKWH: ");
  Serial.println(p_cout);

  yourintEDF = readFile("/inputIntEDF.txt").toInt();
  if ((yourintEDF>190) && (yourintEDF<250)) {
      p_tension=yourintEDF;
  }else{
      p_tension=230; 
  }  
  Serial.print("*** Your yourintEDF: ");
  Serial.println(p_tension);

  yourfltCoefCalb = readFile("/inputFloatCoefCalib.txt").toFloat();
  if ((yourfltCoefCalb>1) && (yourfltCoefCalb<2)) {
      p_coef=yourfltCoefCalb;
  }else{
      p_coef=1.3; 
  }
  Serial.print("*** Your yourfltCoefCalb: ");
  Serial.println(p_coef);

  yourintACS = readFile("/inputIntACS.txt").toInt();
  if((yourintACS==5) || (yourintACS==20) || (yourintACS==30) ){
    p_typeACS = yourintACS;
  }else{
    p_typeACS=30;
  }
  switch (p_typeACS)
  {
  case 5:
    mVperAmp=mVperAmp5A;
    break;
  case 20:
    mVperAmp=mVperAmp20A;
    break;        
  case 30:
    mVperAmp=mVperAmp30A;
    break;        
  default:
    mVperAmp=mVperAmp30A;
    break;
  }
  Serial.print("*** Your yourintACS: ");
  Serial.println(mVperAmp);

  yourfltOffsetCalib = readFile("/inputFloatOffsetCalib.txt").toFloat();
   if ((yourfltOffsetCalib>-1.0) && (yourfltOffsetCalib<1.0)) {
      p_offesetcalib=yourfltOffsetCalib;
  }else{
      p_offesetcalib=0.0; 
  } 
  Serial.print("*** Your yourfltOffsetCalib: ");
  Serial.println(yourfltOffsetCalib);

  Serial.println("...Fin RecupConsigne");
}


float getVPP()
{
  float result;
  int readValue;                // value read from the sensor
  int maxValue = 0;             // store max value here
  int minValue = 1024;          // store min value here
  
   uint32_t start_time = millis();
   while((millis()-start_time) < 1000) //sample for 1 Sec
   {
       readValue = analogRead(sensorIn);
       // see if you have a new maxValue
       if (readValue > maxValue) 
       {
           /*record the maximum sensor value*/
           maxValue = readValue;
       }
       if (readValue < minValue) 
       {
           /*record the minimum sensor value*/
           minValue = readValue;
       }
   }
   
   // Subtract min from max
   result = ((maxValue - minValue) * tension5Vref)/1024.0;
      result = result + p_offesetcalib;
      if (result<0) { result=0;}
   return result;
 }


String CalculCout(int H, int J,float lesWatt){

  float cout  = (p_cout/1000) * lesWatt * J * H;  // Cout kWh/1000 * Conso en W * NbJour * 24h

  char buffer[10]; // Enough room for the digits you want and more to be safe
  dtostrf(cout, 7, 2, buffer);

  return (String)buffer;
}

bool DemarreMesure(void *){
  
  if( deviceconnecte<1) {
    Serial.println("ouverture mesure");
    MesurePossible=true;
  }
  return true;
}

void setup() {
  
  Serial.begin(115200);
  
  while (!Serial) {
    delay(100); // wait for serial port to connect. Needed for native USB
  }
  Serial.flush();
  
  
  Serial.println();
  Serial.println("-------------------------------------------------");
  Serial.println("setup :  "+ LaVersion);
  Serial.println("-------------------------------------------------");
  Serial.println();

  //**************************Initialize SPIFFS*****************************************
  // Initialize LittleFS
  #ifdef ESP32
    if(!LittleFS.begin(true)){
      Serial.println("An Error has occurred while mounting LittleFS");
      return;
    }
  #else
    if(!LittleFS.begin()){
      Serial.println("An Error has occurred while mounting LittleFS");
      return;
    }
  #endif
//*******************************************************************

//**************************Initialize Access Point*****************************************
  Serial.print("Setting AP (Access Point)…");
  WiFi.softAP(ssid, password);

  delay(1000);
  deviceconnecte=0;
  MesurePossible=false;
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  // Print ESP8266 Local IP Address
  //sSerial.println(WiFi.localIP());

    probeRequestHandler = WiFi.onSoftAPModeProbeRequestReceived(&onProbeRequest);
    stationConnectedHandler = WiFi.onSoftAPModeStationConnected(&onStationConnected);
    stationDisconnectedHandler = WiFi.onSoftAPModeStationDisconnected(&onStationDisconnected);


  
//*******************************************************************

//**************************Initialize server web*****************************************
  // Send web page with input fields to client
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html, processor);
  });

  // Send a GET request to <ESP_IP>/get?inputString=<inputMessage>
  server.on("/get", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String inputMessage;
    // GET inputString value on <ESP_IP>/get?inputString=<inputMessage>
    if (request->hasParam(PARAM_STR_Localisation)) {
      inputMessage = request->getParam(PARAM_STR_Localisation)->value();
      writeFile("/inputStrLocalisation.txt", inputMessage.c_str());
    }
    // GET inputFloat value on <ESP_IP>/get?inputFloat=<inputMessage>
    else if (request->hasParam(PARAM_FLOAT_Cout_kwh)) {
      inputMessage = request->getParam(PARAM_FLOAT_Cout_kwh)->value();
      writeFile("/inputFloatKWH.txt", inputMessage.c_str());
    }    
    // GET inputInt value on <ESP_IP>/get?inputInt=<inputMessage>
    else if (request->hasParam(PARAM_INT_EDF)) {
      inputMessage = request->getParam(PARAM_INT_EDF)->value();
      writeFile("/inputIntEDF.txt", inputMessage.c_str());
    }
    // GET inputFloat value on <ESP_IP>/get?inputFloat=<inputMessage>
    else if (request->hasParam(PARAM_FLOAT_Coef_calib)) {
      inputMessage = request->getParam(PARAM_FLOAT_Coef_calib)->value();
      writeFile("/inputFloatCoefCalib.txt", inputMessage.c_str());
    }
    else if (request->hasParam(PARAM_INT_ACS)) {
      inputMessage = request->getParam(PARAM_INT_ACS)->value();
      writeFile("/inputIntACS.txt", inputMessage.c_str());
    }
    else if (request->hasParam(PARAM_FLOAT_Offset_calib)) {
      inputMessage = request->getParam(PARAM_FLOAT_Offset_calib)->value();
      writeFile("/inputFloatOffsetCalib.txt", inputMessage.c_str());
    }      
    else {
      inputMessage = "No message sent";
    }
    Serial.println(inputMessage);
    request->send(200, "text/text", inputMessage);
  });
  server.onNotFound(notFound);
  server.begin();
//*******************************************************************



//**************************Initialize Ecran*****************************************

  tft.initR(INITR_144GREENTAB);   // initialize a ST7735S chip, black tab

  tft.setRotation(3);
  
  // large block of text
  tft.fillScreen(ST77XX_BLACK);
  //tft.setTextSize(1);
  //tftPrintString(CetESP.Version_sketch, ST77XX_BLUE,5,10);
  //tftPrintString("Calib    : " + String(p_offesetcalib) + " V", ST77XX_GREEN,5,100); 
  //tftPrintString("Sonde ACS: " + String(p_typeACS) + " A", ST77XX_GREEN,5,110); 
//*******************************************************************

//**************************RecupConsigne*****************************************

  RecupConsigne();

  Serial.println("Offset Calib    : " + String(p_offesetcalib) + " V");
  Serial.println("Sonde ACS: " + String(p_typeACS) + " A");
  Serial.println("Prix kWh: " + String(p_cout) + " E");
  Serial.println("Tension EDF: " + String(p_tension) + " V");
  Serial.println("Coef calib: " + String(p_coef) );
  
  
  
//*******************************************************************

//**************************Initialize affichage fixe*****************************************

  //affichage fixe
    tft.setTextSize(2);
    tftPrintString(LENOM, ST77XX_BLUE,31,10); 
    tft.setTextSize(1);
    tftPrintString("Cout kWh : " + String(p_cout,4) + " E", ST77XX_GREEN,5,110); 
    //tftPrintString("Calib    : " + String(p_offesetcalib) + " V", ST77XX_GREEN,5,100); 
    //tftPrintString("Sonde ACS: " + String(p_typeACS) + " A", ST77XX_GREEN,5,110); 
    tftPrintString("acefou "+LaVersion, ST77XX_BLUE,5,120);

 
//**************************Debut attente de connexion entrante*****************************************
    MesurePossible = false;
    tft.fillRect(3,25 + decalageAff ,128,62,ST77XX_WHITE);
    tftPrintString("attente connexion...", ST77XX_RED,5,60);
    timerGlobal.in(30000,DemarreMesure);

//**************************Fin de SETUP*****************************************

}

void loop() {

timerGlobal.tick();

  //Serial.println(deviceconnecte);
  
  if (MesurePossible){
      Voltage = getVPP();
      
      Serial.println(Voltage);
    
      VRMS = (Voltage/2.0) *0.707;   //root 2 is 0.707
      AmpsRMS = (VRMS * 1000)/mVperAmp;
     
      Serial.print((String)VRMS); Serial.print(" VRMS    ");
    
      Serial.print(AmpsRMS);
      Serial.print(" Amps RMS  ---  ");
      Watt = (AmpsRMS*p_tension/p_coef);      // 1.3 is an empirical calibration factor
      Serial.print(Watt);
      Serial.println(" W");
    
     // tft.fillScreen(ST77XX_BLACK); // vidage ecran
    
    
      tft.fillRect(3,25 + decalageAff ,128,62,ST77XX_WHITE);
      //tftPrintString("Tension  : " + (String)VRMS + " V", ST77XX_BLUE,5,25); 
      tftPrintString("Courant  : " + (String)AmpsRMS + " A", ST77XX_BLUE,5,27 + decalageAff);
      tftPrintString("Puissance: " + (String)Watt + " W", ST77XX_BLUE,5,37 + decalageAff); 
      tftPrintString("Cout 1 H : " + CalculCout(1,1,Watt) + " E", ST77XX_BLUE,5,47 + decalageAff);
      tftPrintString("Cout 1 J : " + CalculCout(24,1,Watt) + " E", ST77XX_BLUE,5,57 + decalageAff); 
      tftPrintString("Cout 1 M : " + CalculCout(24,30,Watt) + " E", ST77XX_BLUE,5,67 + decalageAff); 
      tftPrintString("Cout 1 A : " + CalculCout(24,365,Watt) + " E", ST77XX_BLUE,5,77 + decalageAff); 
      
  }else{
    if (deviceconnecte>0){
        tft.fillRect(3,25 + decalageAff ,128,62,ST77XX_WHITE);
        tftPrintString("connexion en cours", ST77XX_RED,5,60);
    }
  }
}
