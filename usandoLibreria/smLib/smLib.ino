#include "BluetoothSerial.h"
#include <virtuabotixRTC.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <WiFi.h>
#include <Wire.h>
#include <ArduinoJson.h>

#define CPMonitorOP  //enable or disable monitori ng on serial output
#define pushbt 26

const byte cupo = 32;
char recividos[cupo];
int ID,BS = 0,con = 0;
String dato, archivo, ssid, password;
long debouncing_time = 15; //Debouncing Time in Milliseconds
volatile unsigned long last_micros;
BluetoothSerial SerialBT;
virtuabotixRTC myRTC(15, 2, 4);
boolean newData = false, estadoPM2 = HIGH, estadoPM10 = HIGH;

//Datos thingspeak
String apiKey = "2DG3BNBX6JQAGVYI";
const char* server = "api.thingspeak.com";
WiFiClient client;

//  Width Guide       "----------------------"
#define SplashScreen "SM-PWM-01C, v1.2"
char ScreenHeader[] = "  Advanced Sensors";
char ScreenFooter[] = "  Evaluation Only";

// Asignar pines de los canales de entrada
#define PM10_IN_PIN 14   //input for PM10 signal, P2
#define PM2_IN_PIN 27  //input for PM2 signal, P1

// These bit flags are set in bUpdateFlagsShared to indicate which
// channels have new signals
#define PM10_FLAG 1
#define PM2_FLAG 2

// holds the update flags defined above
volatile uint8_t bUpdateFlagsShared;

unsigned long samplerate = 5000;
unsigned long sampletime = 0;
int SampleCount;
int NoOfSamples = 12;  //maximum 14
float PM2_Value;
float PM10_Value;
String AQIColour;
#define FILTER_WEIGHT 2

// shared variables are updated by the ISR and read by loop.
// In loop we immediatley take local copies so that the ISR can keep ownership of the
// shared ones. To access these in loop
// we first turn interrupts off with noInterrupts
// we take a copy to use in loop and the turn interrupts back on
// as quickly as possible, this ensures that we are always able to receive new signals
volatile unsigned long tiempoBajoPM10;
volatile unsigned long tiempoBajoPM2;

// These are used to record the fallng edge of a pulse in the calcInput functions
// They do not need to be volatile as they are only used in the ISR. If we wanted
// to refer to these in loop and the ISR then they would need to be declared volatile
unsigned long inicioPM10;
unsigned long inicioPM2;

//NO EDITAR, funciones de la SD
void readFile(fs::FS &fs, const char * path)
{
    Serial.printf("Reading file: %s\n", path);

    File file = fs.open(path);
    if(!file){
        Serial.println("Failed to open file for reading");
        return;
    }

    Serial.print("Read from file: ");
    while(file.available()){
        Serial.write(file.read());
    }
    file.close();
}

void writeFile(fs::FS &fs, const char * path, const char * message)
{
    #ifdef CPMonitorOP
    Serial.printf("Writing file: %s\n", path);
    #endif

    File file = fs.open(path, FILE_WRITE);
    if(!file){
        #ifdef CPMonitorOP
        Serial.println("Failed to open file for writing");
        #endif
        return;
    }
    if(file.print(message)){
        #ifdef CPMonitorOP
        Serial.println("File written");
        #endif
    } else {
        #ifdef CPMonitorOP
        Serial.println("Write failed");
        #endif
    }
    file.close();
}

void appendFile(fs::FS &fs, const char * path, const char * message)
{
    #ifdef CPMonitorOP
    Serial.printf("Appending to file: %s\n", path);
    #endif
    File file = fs.open(path, FILE_APPEND);
    if(!file){
        #ifdef CPMonitorOP
        Serial.println("Failed to open file for appending");
        #endif
        return;
    }
    if(file.print(message)){
        #ifdef CPMonitorOP
        Serial.println("Message appended");
        #endif
        if(con==2)
        {
          SerialBT.println("Message appended");
        }
    } else {
        #ifdef CPMonitorOP
        Serial.println("Append failed");
        #endif
        if(con==2)
        {
          SerialBT.println("Append failed");
        }
    }
    file.close();
}

void callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
  if(event == ESP_SPP_SRV_OPEN_EVT)
  {
    #ifdef CPMonitorOP
    Serial.println("Celular Conectado");
    #endif
    con=1;
  }
  if(event == ESP_SPP_CLOSE_EVT)
  {
    #ifdef CPMonitorOP
    Serial.println("Celular Desconectado");
    #endif
    con=0;
  }
}

void setup()
{
  Serial.begin(115200);
  Wire.begin ();
  SerialBT.begin("ESP32test");
  swmPmConfig();
  interrupciones();
  SerialBT.register_callback(callback);
  sdConfig();
  #ifdef CPMonitorOP
    Serial.println("Arduino Dust & CO2 Value Calculation");
    Serial.print("Dust Sensor sample rate of ");
    Serial.print(samplerate / 1000); Serial.print(" sec, with rolling average over ");
    Serial.print(samplerate / 1000 * NoOfSamples); Serial.println(" sec.");
    Serial.println(SplashScreen);
  #endif
}

//NO USAR DELAY EN EL LOOP
void loop()
{
    if(con==1)
    {
      #ifdef CPMonitorOP
      Serial.println("Celular Conectado...");
      #endif
      SerialBT.println("Conectado...");
      SerialBT.println("Configuracion wifi...........w ");
      SerialBT.println("Solo monitoreo...............m ");
      do
      {
        leer();
      }while(newData==false || con == 0);
      check();
      if(*recividos=='w')
      {
        SerialBT.println("WIFI_SSID:");
        do
        {
          leer();
        }while(newData==false || con == 0);
        check();
        ssid=recividos;
        #ifdef CPMonitorOP
        Serial.println(ssid);
        #endif
        SerialBT.println("PASSWORD:");
        do
        {
          leer();
        }while(newData==false);
        check();
        password = recividos;
        Serial.println(password);
        WiFi.begin( ssid.c_str(), password.c_str() );
        Serial.println();
        Serial.println();
        Serial.print("Connecting to ");
        Serial.println(ssid);
        WiFi.begin( ssid.c_str(), password.c_str() );
        while (WiFi.status() != WL_CONNECTED)
        {
          delay(500);
          Serial.print(".");
        }
        Serial.println("");
        Serial.println("WiFi connected");
        con = 2;
      }
      if(*recividos == 'm')
      {
        con = 2;
      }
    }
    if(BS == 1)
    {
      BS = 3;
      SerialBT.end();
    }
    if ( millis() >= (samplerate + sampletime) )
    {
      calcularConcentracion();
      color();
      mostrar();
      if(SampleCount == 1)
      {
        //mostrar();
      }
    }
}

void mostrar()
{

  #ifdef CPMonitorOP
    Serial.print("PM2.5: ");
    Serial.print (PM2_Value);
    Serial.print(" ");
    Serial.print("g/m3");
    Serial.print("\t");
    Serial.print("PM10:  ");
    Serial.print (PM10_Value);
    Serial.print(" ");
    Serial.print("g/m3  ");
    Serial.print("AQI Colour Code: ");
    Serial.println(AQIColour);
    //Serial.printf("PM2.5 LO: %f  PM10 LO: %f\n", tiempoBajoPM2, tiempoBajoPM);
  #endif

  myRTC.updateTime();
  dato=String(ID)+","+String(myRTC.dayofmonth)+"/"+String(myRTC.month)+"/"+String(myRTC.year)+","+String(myRTC.hours)+":"+String(myRTC.minutes)+":"+String(myRTC.seconds)+","+String(PM10_Value)+","+String(PM2_Value)+","+String(AQIColour)+ "\r\n";
  appendFile(SD, archivo.c_str(), dato.c_str());

  if(con=2)
  {
    SerialBT.println(dato);
  }
  if (client.connect(server,80))
  {
    String postStr = apiKey;
    postStr +="&field1=";
    postStr += String(PM10_Value);
    postStr +="&field2=";
    postStr += String(PM2_Value);
    postStr += "\r\n\r\n";
    client.print("POST /update HTTP/1.1\n");
    client.print("Host: api.thingspeak.com\n");
    client.print("Connection: close\n");
    client.print("X-THINGSPEAKAPIKEY: "+apiKey+"\n");
    client.print("Content-Type: application/x-www-form-urlencoded\n");
    client.print("Content-Length: ");
    client.print(postStr.length());
    client.print("\n\n");
    client.print(postStr);
    Serial.println("% send to Thingspeak");
  }
  #ifdef CPMonitorOP
  Serial.println("Waiting…");
  #endif
}

void interrupciones()
{

  attachInterrupt(digitalPinToInterrupt(pushbt), debounceInterrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(PM10_IN_PIN), interrupcionPM, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PM2_IN_PIN), interrupcionPM, CHANGE);

}

void sdConfig()
{

  if( !SD.begin() )
  {
      #ifdef CPMonitorOP
      Serial.println("Card Mount Failed");
      #endif
      return;
  }
      uint8_t cardType = SD.cardType();

  if( cardType == CARD_NONE )
  {
        #ifdef CPMonitorOP
        Serial.println("No SD card attached");
        #endif
        return;
  }

  myRTC.updateTime();
  archivo = "/" + String(myRTC.dayofmonth) + "-" + String(myRTC.month) + "-" +
  String(myRTC.year) + ".txt";
  File file = SD.open( archivo.c_str() );
  if(!file)
  {
    #ifdef CPMonitorOP
    Serial.println("File doens't exist");
    Serial.println("Creating file...");
    #endif
    writeFile(SD, archivo.c_str(), "ID, Date, Hour, PM10, PM2.5, color\r\n");
  }
  else
  {
    #ifdef CPMonitorOP
    Serial.println("File already exists");
    #endif
  }
  file.close();

}

void swmPmConfig()
{
  pinMode(PM10_IN_PIN, INPUT);
  pinMode(PM2_IN_PIN, INPUT);
  pinMode(pushbt, INPUT_PULLUP);
}

void calcularConcentracion()
{
  sampletime = millis();
  noInterrupts();
  float ratio, conc; //temporary variables for concentration calculations

  //percentage of time dust has been detected in one sampling period
  ratio = ( tiempoBajoPM10 / (samplerate * 10) );
  // polynomial approximation of the curve in the datasheet. Yields the concentration in micrograms per cubic meter
  conc = -10.747 * pow(ratio, 3) + 33.548 * pow(ratio, 2) + 78.617 * ratio;// - 0.5196;
  // some filtering
  PM10_Value = conc;
  //reseting the count
  tiempoBajoPM10 = 0;

  ratio = ( tiempoBajoPM2 / (samplerate * 10 ) );
  conc = -10.747 * pow(ratio, 3) + 33.548 * pow(ratio, 2) + 78.617 * ratio;//0.5196;
  PM2_Value = conc;
  tiempoBajoPM2 = 0;
  interrupts();
}

void interrupcionPM()
{
  unsigned long us = micros();

  if (estadoPM10 != digitalRead(PM10_IN_PIN))
  {
    if ((estadoPM10 = digitalRead(PM10_IN_PIN)) == LOW)
    {
      inicioPM10 = us;
    }
    else
    {
      tiempoBajoPM10 += (us - inicioPM10);
    }
  }

  if (estadoPM2 != digitalRead(PM2_IN_PIN))
  {
    if ( (estadoPM2 = digitalRead(PM2_IN_PIN) ) == LOW)
    {
      inicioPM2 = us;
    }
    else
    {
      tiempoBajoPM2 += (us - inicioPM2);
    }
  }

}

void leer()
{
    static boolean reciviendo = false;
    static byte ndx = 0;
    char inicio = '<';
    char fin = '>';
    char rc;

    while (SerialBT.available() > 0 && newData == false) {
        rc = SerialBT.read();

        if (reciviendo == true)
        {
            if (rc != fin)
            {
                recividos[ndx] = rc;
                ndx++;
                if (ndx >= cupo)
                {
                    ndx = cupo - 1;
                }
            }
            else
            {
                recividos[ndx] = '\0'; // terminate the string
                reciviendo = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == inicio) {
            reciviendo = true;
        }
    }
}

void check()
{
    if (newData == true) {
        Serial.print("se recivio: ");
        Serial.println(recividos);
        newData = false;
    }
}

void blueswitch ()
{
  if(BS==0)
  {
    BS=1;
  }
}

void debounceInterrupt()
{
 if((long)(micros() - last_micros) >= debouncing_time * 1000) {
   blueswitch();
   last_micros = micros();
 }
}

void color()
{
    // Colour Values based on US EPA Air Quality Index for PM 2.5 and PM 10
  if (PM2_Value <= 12 && PM10_Value <= 54)
  {
    AQIColour = "Green ";
  }
  else if (PM2_Value <= 35 && PM10_Value <= 154)
  {
    AQIColour = "Yellow";
  }
  else if (PM2_Value <= 55 && PM10_Value <= 254)
  {
    AQIColour = "Orange";
  }
  else if (PM2_Value <= 150 && PM10_Value <= 354)
  {
    AQIColour = " Red  ";
  }
  else if (PM2_Value <= 250 && PM10_Value <= 424)
  {
    AQIColour = "Purple";
  }
  else {
    AQIColour = "Maroon";
  }
}
