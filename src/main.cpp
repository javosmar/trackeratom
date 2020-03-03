#define TINY_GSM_MODEM_A6

//++++++++++++++++++++++++++++++++++++++
//++++++++++++ LIBRERÍAS +++++++++++++++
//++++++++++++++++++++++++++++++++++++++

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiManager.h>
//#include "FS.h"
#include "SD.h"
#include <SPI.h>
#include <TinyGsmClient.h>
#include <TinyGPS.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <EEPROM.h>

//*********************************
//*********** CONFIG **************
//*********************************

#define WIFI_PIN 0
WiFiManager wifiManager;

//++++++++++++++++++++++++++++++++++++++
//+++++++++ VARIABLES OTA ++++++++++++++
//++++++++++++++++++++++++++++++++++++++

String Cliente = "SH_Nodemcu_v2";                                               // Nombre de producción
String NewVersion = "0.7";                                                      // Número de la próxima versión a la que se actualizará en el futuro
String url_server = "http://iotoro.000webhostapp.com/firms/" + Cliente;
String url_sketch = url_server+"/"+Cliente+"_"+NewVersion+".bin";               // Ruta de la nueva versión, a la que se añade el número de la nueva versión
bool updateAvailable = true;
WiFiClient wifiClient;

//++++++++++++++++++++++++++++++++++++++
//++++++++++ DEFINICIONES ++++++++++++++
//++++++++++++++++++++++++++++++++++++++

#define DEBUG false
#define DEBUGGPS false
#define DEBUGSD false
#define DEBUGGPRS false
#define DEBUGWIFI false
String NAME_FILE = "/GPSLOG.txt";

#define serialGprs Serial2
#define serialGps Serial1
#define Consola Serial

#define GPS_BAUD_RATE 9600
#define BAUD_RATE 115200
#define A9_BAUD_RATE 9600

#define A9G_PON     15  //ESP12 GPIO16 A9/A9G POWON
//#define A9G_POFF    2  //ESP12 GPIO15 A9/A9G POWOFF
//#define A9G_WAKE    22  //ESP12 GPIO13 A9/A9G WAKE
//#define A9G_LOWP    23  //ESP12 GPIO2 A9/A9G ENTER LOW POWER MODULE
#define LED    0
#define LED2    2

//unsigned long periodoUpdate = 604800000; // periodo de actualización semanal
unsigned long periodoUpdate = 15000;

#define AP_NAME "IoToro"
#define AP_PASS "administrador"
#define AP_TIMEOUT 120 // timeout para el portal wifimanager en segundos

//++++++++++++++++++++++++++++++++++++++
//++++++++++++++ GPRS ++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++

const char apn[]  = "igprs.claro.com.ar";
const char user[] = "";
const char pass[] = "";
const char simCode[] = "";
int inicia = 1;

const int periodoRefresh = 15000;
long tActual = 0;
char mensaje[22] = "0,0";

//++++++++++++++++++++++++++++++++++++++
//++++++++++++ INSTANCIAS ++++++++++++++
//++++++++++++++++++++++++++++++++++++++

TinyGPS gps;
TinyGsm modem(Serial2);
TinyGsmClient cliente(modem);
TaskHandle_t Task2, Task3;

//++++++++++++++++++++++++++++++++++++++
//+++++++++++++ MICRO SD +++++++++++++++
//++++++++++++++++++++++++++++++++++++++

#define SD_CS 4
bool microSD = false;
String dataMessage;
String coordenada = "-34.602232,-58.424695", velocidad;
String hora;

//++++++++++++++++++++++++++++++++++++++
//++++++++++++ VARIABLES +++++++++++++++
//++++++++++++++++++++++++++++++++++++++

bool newData = false, mensajeListo = false;

//************************************
//***** DECLARACION FUNCIONES ********
//************************************

void handleInterrupt();
int A9GPOWERON();
int A9GPOWEROFF();
int A9GENTERLOWPOWER();
void A9GMQTTCONNECT();
//void iniciarSd();
static void print_date(TinyGPS &gps);
void iniciarModem();
void iniciarAPN();
void OTA_Updates();
void sdInit();
int GPSPOWERON();
void logSDCard(const char * path);
String sendData(String command, const int timeout, boolean debug);
void appendFile(fs::FS &fs, const char * path, const char * message);
void updateCallback();
String lee(int addr);
void graba(int addr, String a);

//++++++++++++++++++++++++++++++++++++++
//++++++++++++ LOOP GPS ++++++++++++++++
//++++++++++++++++++++++++++++++++++++++

void loop2( void * pvParameters ) {
  sdInit();
  for (;;) {
    delay(100);
    /*
    if(!microSD){
      sdInit();
    }
    */
    for (unsigned long start = millis(); millis() - start < 1000;){
      while (serialGps.available()) {
        char c = serialGps.read();
        if(DEBUGGPS){
          Consola.print(c);
        }
        if (gps.encode(c)){
          newData = true;
        }
      }
    }
    if(DEBUG){
      Consola.println(newData);
    }
    if (newData){
      float flat, flon;
      unsigned long age;
      gps.f_get_position(&flat, &flon, &age);
      float vel = gps.f_speed_kmph();
      print_date(gps);
      coordenada = String(flat, 6);
      coordenada += ",";
      coordenada += String(flon, 6);
      velocidad = String(vel, 2);
      logSDCard(NAME_FILE.c_str());
    }
    coordenada.toCharArray(mensaje, 22);
    mensajeListo = true;
    newData = false;
    if(DEBUG){
      Consola.print(F("Coordenada detectada: "));
      Consola.println(coordenada);
    }
  }
  vTaskDelay(10);
}

//++++++++++++++++++++++++++++++++++++++
//++++++++++++ LOOP GPRS +++++++++++++++
//++++++++++++++++++++++++++++++++++++++

void loop3( void * pvParameters ) {
  delay(1000);
    // if(no hay conexion GPRS){
  for (char ch = ' '; ch <= 'z'; ch++) {
    serialGprs.write(ch);
  }
  serialGprs.println("");
  pinMode(A9G_PON, OUTPUT);//LOW LEVEL ACTIVE
  digitalWrite(A9G_PON, HIGH);
  if(DEBUGGPRS){
    Consola.println("El módulo se encenderá luego de 2 segundos...");
  }
  unsigned long now = millis();
  while(millis() - now < 2000);
  //delay(2000);
  if (A9GPOWERON() == 1) {
    if(DEBUGGPRS){
      Consola.println("Módulo A9G encendido.");
    }
  }
  now = millis();
  while(millis() - now < 5000);
  //delay(5000);
  iniciarModem();
  iniciarAPN();
  now = millis();
  while(millis() - now < 1000);
  //delay(1000);
  A9GMQTTCONNECT();
  now = millis();
  while(millis() - now < 5000);
  //delay(5000);
  GPSPOWERON();
  now = millis();
  while(millis() - now < 5000);
  //delay(5000);
    // }
    //**************************************
  for (;;) {
    if(modem.isNetworkConnected()){
      if(DEBUGGPRS){
        Consola.println("conectado a la red claro");
      }
      digitalWrite(LED,HIGH);
    }
    else {
      if(DEBUGGPRS){
        Consola.println("no conectado a la red claro");
      }
      iniciarModem();
      digitalWrite(LED,LOW);
    }
    if(modem.isGprsConnected()){
      if(DEBUGGPRS){
        Consola.println("APN conectada");
      }
      digitalWrite(LED2,HIGH);
    }
    else {
      if(DEBUGGPRS){
        Consola.println("APN no conectada");
      }
      iniciarAPN();
      digitalWrite(LED2,LOW);
    }
    if (millis() > tActual + periodoRefresh) {
      //A9GMQTTCONNECT();
      tActual = millis();
      String topic = "/gps/coordenadas";
      if(DEBUG){
        Consola.println(topic);
      }
      String payload = mensaje;
      if(DEBUG){
        Consola.println(payload);
      }
      String ATCMD = "AT+MQTTPUB=";
      String cammand = ATCMD + "\"" + topic + "\"" + "," + "\"" + payload + "\"" + ",0,0,0";
      if(DEBUG){
        Consola.println(cammand);
      }
      sendData(cammand, 1000, DEBUG);
    }

  }
  vTaskDelay(10);
}

//++++++++++++++++++++++++++++++++++++++
//++++++++++++ LOOP WIFI +++++++++++++++
//++++++++++++++++++++++++++++++++++++++

void loop() {

}

//++++++++++++++++++++++++++++++++++++++
//++++++++++++++ SETUP +++++++++++++++++
//++++++++++++++++++++++++++++++++++++++

void setup() {
  xTaskCreatePinnedToCore(
    loop2,   /* Task function. */
      "Task2",     /* name of task. */
      10000,       /* Stack size of task */
      NULL,        /* parameter of the task */
      1,           /* priority of the task */
      &Task2,      /* Task handle to keep track of created task */
      0);          /* pin task to core 0 */
  delay(500);

  xTaskCreatePinnedToCore(
    loop3,   /* Task function. */
      "Task3",     /* name of task. */
      10000,       /* Stack size of task */
      NULL,        /* parameter of the task */
      1,           /* priority of the task */
      &Task3,      /* Task handle to keep track of created task */
      1);          /* pin task to core 1 */
  delay(500);

  serialGps.begin(GPS_BAUD_RATE);
  Consola.begin(BAUD_RATE);
  serialGprs.begin(A9_BAUD_RATE);
  EEPROM.begin(4096);

  randomSeed(analogRead(0));
  pinMode(WIFI_PIN,INPUT_PULLUP);

  wifiManager.setConfigPortalTimeout(AP_TIMEOUT);
  wifiManager.setDebugOutput(DEBUGWIFI);
  //wifiManager.setSaveConfigCallback(updateCallback);
  if (!wifiManager.autoConnect(AP_NAME,AP_PASS)) {
    if(DEBUGWIFI){
      Consola.println("failed to connect and hit timeout");
    }
    //unsigned long now = millis();
    //while(millis() - now < 3000);
  }
  else{
    if(DEBUGWIFI){
      Consola.println("Conexión a WiFi exitosa!");
      Consola.println("es momento de guardar ssid y pass");
    }
    updateCallback();
  }
  pinMode(LED,OUTPUT);
  digitalWrite(LED,LOW);
  pinMode(LED2,OUTPUT);
  digitalWrite(LED2,LOW);
}

//++++++++++++++++++++++++++++++++++++++
//++++++++++++ FUNCIONES +++++++++++++++
//++++++++++++++++++++++++++++++++++++++

void handleInterrupt(){
  Consola.println("An interrupt has occurred.");
}

String sendData(String command, const int timeout, boolean debug){
  String response = "";
  serialGprs.println(command);
  long int time = millis();
  while ( (time + timeout) > millis()) {
    while (serialGprs.available()) {
      char c = serialGprs.read();
      response += c;
    }
    //Consola.print("respuesta: ");
    //Consola.print(response);
  }
  if (debug) {
    Consola.print("respuesta: ");
    Consola.print(response);
  }
  return response;
}

int GPSPOWERON() {
  String msg = String("");
  msg = sendData("AT+GPS=1", 1000, DEBUG);
  if ( msg.indexOf("OK") >= 0 ) {
    if(DEBUGGPS){
      Consola.println("GPS encendido");
    }
    return 1;
  }
  else {
    if(DEBUG){
      if(DEBUGGPS){
        Consola.println("No se recibió confirmación GPS");
      }
      return 0;
    }
  }
}

int A9GPOWERON() {
  digitalWrite(A9G_PON, LOW);
  unsigned long now = millis();
  while(millis() - now < 3000);
  //delay(3000);
  digitalWrite(A9G_PON, HIGH);
  now = millis();
  while(millis() - now < 5000);
  //delay(5000);
  String msg = String("");
  msg = sendData("AT", 1000, DEBUG);
  if ( msg.indexOf("OK") >= 0 ) {
    if(DEBUG){
      Consola.println("GET OK");
    }
    return 1;
  }
  else {
    if(DEBUG){
      Consola.println("NOT GET OK");
    }
    return 0;
  }
}

int A9GPOWEROFF() {
  //digitalWrite(A9G_POFF, HIGH);
  //delay(3000);
  //digitalWrite(A9G_POFF, LOW);
  //delay(5000);
  String msg = String("");
  msg = sendData("AT", 1000, DEBUG);
  if ( msg.indexOf("OK") >= 0 ) {
    Consola.println("GET OK");
    return 1;
  }
  else {
    Consola.println("NOT GET OK");       return 0;
  }
}

int A9GENTERLOWPOWER() {
  String msg = String("");
  msg = sendData("AT+SLEEP=1", 1000, DEBUG);
  if ( msg.indexOf("OK") >= 0 ) {
    //digitalWrite(A9G_LOWP, LOW);
    return 1;
  }
  else {
    return 0;
  }
}

void A9GMQTTCONNECT() {
  sendData("AT+CGATT=1", 1000, DEBUG);
  unsigned long now = millis();
  while(millis() - now < 1000);
  //delay(1000);
  sendData("AT+CGDCONT=1,\"IP\",\"igprs.claro.com.ar\"", 1000, DEBUG);
  now = millis();
  while(millis() - now < 1000);
  //delay(1000);
  sendData("AT+CGACT=1,1", 1000, DEBUG);
  now = millis();
  while(millis() - now < 1000);
  //delay(1000);
  sendData("AT+MQTTDISCONN", 1000, DEBUG);
  now = millis();
  while(millis() - now < 2000);
  //delay(2000);
  String peticion = "AT+MQTTCONN=\"190.7.57.163\",1883,\"DHT11\",120,1";
  String msg = sendData(peticion, 1000, DEBUG);
  if(DEBUG){
    Consola.println(peticion);
  }
  now = millis();
  while(millis() - now < 1000);
  //delay(1000);
  if ( msg.indexOf("OK") >= 0 ) {
    Consola.println("CONECTADO AL BROKEER");
  }
  else if(msg.indexOf("+CME ERROR:") >= 0){
    Consola.print("ERROR AL CONECTAR AL BROKER ");
    Consola.println(msg);
  }
  Consola.println(msg);
  now = millis();
  while(millis() - now < 2000);
  //delay(2000);
}

/*
void iniciarSd() {
  if(DEBUGSD){
    Consola.print(F("Inicializando SD..."));
  }
  if (!SD.begin(SD_CS)) {
    if(DEBUGSD){
      Consola.println(F("Verificar que exista una SD."));
    }
    microSD = false;
  }
  else {
    if(DEBUGSD){
      Consola.println(F("SD inicializada."));
    }
    microSD = true;
  }
}
*/

static void print_date(TinyGPS &gps) {
  int year;
  byte month, day, hour, minute, second, hundredths;
  unsigned long age;
  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
  if (age == TinyGPS::GPS_INVALID_AGE) {
    Consola.print(F("********** ******** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d %02d:%02d:%02d ",
            day, month, year, hour, minute, second);
    hora = sz;  // rescato el dato de la hora y fecha
    //Consola.print(sz);
  }
}

void iniciarModem() {
  if (inicia == 1)
  {
    if(DEBUGGPRS){
      Consola.println(F(""));
      Consola.println(F("Inicializando modem..."));
    }
    modem.restart();
    inicia++;
  }
  else
  {
    if(DEBUGGPRS){
      Consola.println(F("Reiniciando modem..."));
    }
    modem.restart();
  }

  String modemInfo = modem.getModemInfo();
  if(DEBUGGPRS){
    Consola.print(F("Modem: "));
    Consola.println(modemInfo);
  }

  //Desbloquea la sim con un pin
  modem.simUnlock(simCode);

  if(DEBUGGPRS){
    Consola.print(F("Esperando red..."));
  }
  if (!modem.waitForNetwork()){
    if(DEBUGGPRS){
      Consola.println(F(" Fallo"));
      Consola.print(F("\n"));
    }
    while (!modem.waitForNetwork()){
      if(DEBUGGPRS){
        Consola.print(F("\n"));
        Consola.print(F("Esperando por red..."));
      }
    }
  }
  //******************************
  /*
  if(DEBUGGPRS){
    Consola.println(" OK");
    Consola.print(F("Conectadose a (APN):"));
    if (strcmp(apn, "claro")){
      Consola.print(F(" Claro Argentina"));
    }
  }
  if (!modem.gprsConnect(apn, user, pass)) {
    if(DEBUGGPRS){
      Consola.println(F(" Fallo"));
    }
    while (true);
  }
  if(DEBUGGPRS){
    Consola.println(F(" OK"));
  }
  */
}

void iniciarAPN() {
  if(DEBUGGPRS){
    Consola.println(" OK");
    Consola.print(F("Conectadose a (APN):"));
    if (strcmp(apn, "claro")){
      Consola.print(F(" Claro Argentina"));
    }
  }
  if (!modem.gprsConnect(apn, user, pass)) {
    if(DEBUGGPRS){
      Consola.println(F(" Fallo"));
    }
    while (true);
  }
  if(DEBUGGPRS){
    Consola.println(F(" OK"));
  }
}

//++++++++++++++++++++++++++++++++++++++
//+++++++++++++ SD CARD ++++++++++++++++
//++++++++++++++++++++++++++++++++++++++

void logSDCard(const char * path) {
  String dato = hora;
  dato += ",";
  dato += coordenada;
  dato += ",";
  dato += velocidad;
  dato += "\r\n";
  //appendFile(SD, "/data3.txt", dato.c_str());
  appendFile(SD, path, dato.c_str());
  //Consola.print("Save data: ");
  //Consola.println(dato);
}

void writeFile(fs::FS &fs, const char * path, const char * message) {
  Consola.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if(!file) {
    Consola.println("Failed to open file for writing");
    return;
  }
  if(file.print(message)) {
    Consola.println("File written");
  } else {
    Consola.println("Write failed");
  }
  file.close();
}

void appendFile(fs::FS &fs, const char * path, const char * message) {
  Consola.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if(!file) {
    Consola.println("Failed to open file for appending");
    return;
  }
  if(file.print(message)) {
    Consola.println("Message appended");
  } else {
    Consola.println("Append failed");
  }
  file.close();
}

void sdInit(){
  SD.begin(SD_CS);
  if(!SD.begin(SD_CS)) {
    if(DEBUGSD){
      Consola.println("Card Mount Failed");
    }
    return;
  }
  uint8_t cardType = SD.cardType();
  if(cardType == CARD_NONE) {
    if(DEBUGSD){
      Consola.println("No SD card attached");
    }
    return;
  }
  if(DEBUGSD){
    Consola.println("Initializing SD card...");
  }
  if (!SD.begin(SD_CS)) {
    if(DEBUGSD){
      Consola.println("ERROR - SD card initialization failed!");
    }
    return;
  }
  //File file = SD.open("/data3.txt");
  File file = SD.open(NAME_FILE.c_str());
  if(!file) {
    if(DEBUGSD){
      Consola.println("File doens't exist");
      Consola.println("Creating file...");
    }
    //writeFile(SD, "/data3.txt", "Fecha, Hora, Coordenadas, velocidad \r\n");
    writeFile(SD, NAME_FILE.c_str(), "Fecha, Hora, Coordenadas, velocidad \r\n");
  }
  else {
    if(DEBUGSD){
      Consola.println("File already exists");
    }
  }
  file.close();
}

void OTA_Updates(){           //función que verifica la existencia de una nueva versión de firmware
  t_httpUpdate_return ret = httpUpdate.update(wifiClient, url_sketch);
  switch(ret) {
    case HTTP_UPDATE_FAILED:
    Consola.printf("HTTP_UPDATE_FAILD Error (%d): %s\n", httpUpdate.getLastError(), httpUpdate.getLastErrorString().c_str());
    //Consola.println("error");
    break;

    case HTTP_UPDATE_NO_UPDATES:
    Consola.println("HTTP_UPDATE_NO_UPDATES");
    break;

    case HTTP_UPDATE_OK:
    Consola.println("HTTP_UPDATE_OK");
    break;
  }
  Consola.println("El sistema se encuentra actualizado.");
  updateAvailable = false;
}

void graba(int addr, String a){
  int tamano = (a.length()+1);
  Consola.println(tamano);
  char inchar[tamano];
  a.toCharArray(inchar,tamano);
  EEPROM.write(addr,tamano);
  for(int i=0; i<tamano; i++){
    addr++;
    EEPROM.write(addr,inchar[i]);
  }
  EEPROM.commit();
}

String lee(int addr){
  String nuevostring;
  int valor;
  int tamano = EEPROM.read(addr);
  for(int i=0; i<tamano; i++){
    addr++;
    valor = EEPROM.read(addr);
    nuevostring += (char)(valor);
  }
  return nuevostring;
}

void updateCallback(){
  if(DEBUGWIFI){
    Consola.println("Buscando actualizaciones disponibles.");
  }
  OTA_Updates();
}
