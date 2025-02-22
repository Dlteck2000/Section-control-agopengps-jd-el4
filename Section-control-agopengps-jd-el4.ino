#include <EEPROM.h> 
#include <Wire.h>
#include "EtherCard_AOG.h"
#include <IPAddress.h>
    
// ========== PARAMETRES UTILSATEUR - USER SETTINGS ==========
#define VERSION 1.1 //Version de l'ino
#define Main_Switch A0 // Broche entrés vanne générale
#define Auto_Switch A1 // Broche entrés switch Auto/Manu
#define nb_Sec 6 // Nombre de sections
#define EEP_Ident 0x5425
#define CS_Pin 10 //Broche CS Arduino Nano pour carte Ethernet ENC28J60 - Arduino Nano CS Pin for ENC28J60 Ethernet Board

// ========== CONFIGURATION CARTE RESEAU - NETWORK CARD CONFIGURATION ==========
//the default network address
struct ConfigIP {
  uint8_t ipOne = 192;
  uint8_t ipTwo = 168;
  uint8_t ipThree = 1;
};  ConfigIP networkAddress;   //3 bytes

static uint8_t myip[] = { 0,0,0,123 };/// Adresse IP de la carte machine - Machine board IP address
static uint8_t gwip[] = { 0,0,0,1 };// Adresse de la passerelle - Gateway address
static uint8_t myDNS[] = { 8,8,8,8 };// DNS
static uint8_t mask[] = { 255,255,255,0 };// Masque de sous réseau - subnet mask
uint16_t portMy = 5123;// Port de la carte machine - Machine board port

//Adresse Ip et port d'AOG - Ip address and port of AOG
static uint8_t ipDestination[] = { 0,0,0,255 };//sending back to where and which port
uint16_t portDestination = 9999; //AOG port that listens

// Adresse mac de la carte, doit être unique sur votre réseau - Mac address of the card, must be unique on your network
static uint8_t mymac[] = { 0x00,0x00,0x56,0x00,0x00,0x7B };

uint8_t Ethernet::buffer[200]; // Tampon d'envoi et de réception udp - Udp send and receive buffer

//// ========== CONFIGURATION UTILISATEUR AOG ==========
struct Config {
  uint8_t raiseTime = 2;
  uint8_t lowerTime = 4;
  uint8_t enableToolLift = 0;
  uint8_t isRelayActiveHigh = 0; //if zero, active low (default)

  uint8_t user1 = 0; //user defined values set in machine tab
  uint8_t user2 = 0;
  uint8_t user3 = 0;
  uint8_t user4 = 0;

};  Config aogConfig;   //4 bytes

//Broches des sorties relays
const uint8_t Relay_PinArray[] = {2, 3, 4, 5, 6, 7}; // 6 sections du pulvé

//Broches entrées optocoupleurs sections
const uint8_t Opto_PinArray[] = {A2, A3, A4, A5, 8, 9}; // 6 optocoupleurs pour vérifier l'état des sections
bool Main_Section = 0;
bool Auto_Section = 0;

/* Functions as below assigned to pins
- 1 à 16 : Sections,
- 17,18 : Hyd Up, Hyd Down,
- 19 : Tramline,
- 20 : Geo Stop
- 21,22,23 - unused so far*/
uint8_t pin[] = { 1,2,3,4,5,6,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };

//read value from Machine data and set 1 or zero according to list
uint8_t relayState[] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };

// Gestion du temps en millisecondes - Time management in milliseconds
const unsigned long LOOP_TIME = 200; //en msec : 5Hz - 200ms
unsigned long lastTime = LOOP_TIME;
unsigned long currentTime = LOOP_TIME;

// Gestion du temps de communication entre AOG et la carte machine
uint8_t watchdogTimer = 20; // Max 20*200ms = 4s (LOOP TIME) si plus de 20 déclaration de perte de communication avec le module
uint8_t serialResetTimer = 0; // Vidange de la mémoire tampon

//Parsing PGN
bool isPGNFound = false, isHeaderFound = false;
uint8_t pgn = 0, dataLength = 0, idx = 0;
int16_t tempHeader = 0;

//hello from AgIO
uint8_t helloFromMachine[] = { 128, 129, 123, 123, 5, 0, 0, 0, 0, 0, 71 };

/* PGN234 Switch Control
Byte 0 : 0x80 (128)
Byte 1 : 0x81 (129)
Byte 2 : 0x7B (123)
Byte 3 : 0xEA (234)
Byte 4 : Len (8)
Byte 5 : Main (1 ou 2)
Byte 6 : (Auto group 0)
Byte 7 : (Auto group 1)
Byte 8 : Sections
Byte 9 : (On group 0)
Byte 10 : (Off group 0)
Byte 11 : (On group 1)
Byte 12 : (Off group 1)
*/
uint8_t PGN_234[] = { 128, 129, 123, 234, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0xCC }; //Equivalent Hexa { 0x80, 0x81, 0x7B, 0xEA, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0xCC }
int8_t PGN_234_Size = sizeof(PGN_234) - 1;

//The variables used for storage
uint8_t relayHi = 0, relayLo = 0, tramline = 0, uTurn = 0, hydLift = 0, geoStop = 0;
float gpsSpeed;
uint8_t onLo = 0, offLo = 0, onHi = 0, offHi = 0, mainByte = 0;
uint8_t count = 0;
bool isRaise = false, isLower = false;

//Program counter reset
void(*resetFunc) (void) = 0;

//Communication with AgOpenGPS
int16_t temp, EEread = 0;



void setup() {

  Serial.begin(38400);
  while (!Serial) {}// wait for serial port to connect. Needed for native USB

  EEPROM.get(0, EEread);// read identifier

  delay(100);

  if (EEread != EEP_Ident)   // check on first start and write EEPROM
  {
    EEPROM.put(0, EEP_Ident);
    EEPROM.put(6, aogConfig);
    EEPROM.put(20, pin);
    EEPROM.put(50, networkAddress);
  }
  else
  {
    EEPROM.get(6, aogConfig);
    EEPROM.get(20, pin);
    EEPROM.get(50, networkAddress);
  }

  if (ether.begin(sizeof Ethernet::buffer, mymac, CS_Pin) == 0) {Serial.println(F("Failed to access Ethernet controller"));}

  //grab the ip from EEPROM
  myip[0] = networkAddress.ipOne;
  myip[1] = networkAddress.ipTwo;
  myip[2] = networkAddress.ipThree;

  gwip[0] = networkAddress.ipOne;
  gwip[1] = networkAddress.ipTwo;
  gwip[2] = networkAddress.ipThree;

  ipDestination[0] = networkAddress.ipOne;
  ipDestination[1] = networkAddress.ipTwo;
  ipDestination[2] = networkAddress.ipThree;

  //set up connection
  ether.staticSetup(myip, gwip, myDNS, mask);
  ether.printIp("_IP_: ", ether.myip);
  ether.printIp("GWay: ", ether.gwip);
  ether.printIp("AgIO: ", ipDestination);

  //register to port 8888
  ether.udpServerListenOnPort(&udpSteerRecv, 8888);

  //Affectation des relays aux broches digitales
  for (count = 0; count < nb_Sec; count++) {
    pinMode(Relay_PinArray[count], OUTPUT);
  }

  //Affectation des optocoupleurs aux broches analogiques
  for (count = 0; count < nb_Sec; count++) {
    pinMode(Opto_PinArray[count], INPUT_PULLUP);
  }

  //Affectation de la coupure générale sur la broches A0
  pinMode(Main_Switch, INPUT_PULLUP);
  pinMode(Auto_Switch, INPUT_PULLUP);
  
  Serial.println("Setup complete, waiting for AgOpenGPS");

}

void loop() {

  currentTime = millis();

  if (currentTime - lastTime >= LOOP_TIME)
  {
    lastTime = currentTime;

    //If connection lost to AgOpenGPS, the watchdog will count up 
    if (watchdogTimer++ > 250) watchdogTimer = 20;

    //clean out serial buffer to prevent buffer overflow
    if (serialResetTimer++ > 20)
    {
      while (Serial.available() > 0) Serial.read();
      serialResetTimer = 0;
    }

    if (watchdogTimer > 20)
    {
      //Serial.println("Perte de communication avec AgOpenGPS ! Désactivation de sécurité.");
      mainByte = 2;
      RelaysOff();
    }
    Serial.print("Watch : ");
    Serial.println(watchdogTimer);

    //Si changement d'état du switch Main_Switch en A0, acctualistation de la varaible Main_Section (0 ou 1)
    if (Main_Section != digitalRead(Main_Switch)){ Main_Section = digitalRead(Main_Switch);}

    //Si changement d'état du switch Auto_Switch en A1, acctualistation de la varaible Auto_Section (0 ou 1)
    if (Auto_Section != digitalRead(Auto_Switch)){ Auto_Section = digitalRead(Auto_Switch);}

    //Si le switch Auto est actionné
    if (Auto_Section == 1)
      {
        onLo = 0, offLo = 0, onHi = 0, offHi = 0;
        if (Main_Section == 1) // On récupere l'état de la vanne générale
        {
          //Serial.println("Auto - vanne générale fermée -> 0V");
          mainByte = 2;
          RelaysOff();
        }
        else
        {
          //Serial.println("Auto - vanne générale ouverte -> 12V");
          mainByte = 1;
        }
        SetRelays();
      }
      else //Sinon mode manuel
      {
        mainByte = 2;
        RelaysOff();
        onLo = 0, offLo = 0, onHi = 0, offHi = 0;

        if (Main_Section == 1) // On récupere l'état de la vanne générale
        {
          //Serial.println("Manuel - vanne générale fermée -> 0V");
          for (count = 0; count < nb_Sec; count++)
          {
              bitSet(offLo, count);
              bitClear(onLo, count);
            }
          
          //mainByte = 2;
        }
        else
        {
          //Serial.println("Manuel - vanne générale ouverte -> 12V");
          for (count = 0; count < nb_Sec; count++)
          {
            if (digitalRead(Opto_PinArray[count]))
            {
                bitSet(offLo, count);
                bitClear(onLo, count);
            }
            else 
            {
                bitClear(offLo, count);
                bitSet(onLo, count);
            }
          }
        }
      }
            
    //Send to AOG
    PGN_234[5] = (uint8_t)mainByte;
    PGN_234[9] = (uint8_t)onLo;
    PGN_234[10] = (uint8_t)offLo;
      
    //add the checksum
    int16_t CK_A = 0;
    for (uint8_t i = 2; i < PGN_234_Size; i++)
    {
      CK_A = (CK_A + PGN_234[i]);
      //Serial.print(PGN_234[i]);
      //Serial.print("-");
    }
    PGN_234[PGN_234_Size] = CK_A;
    //Serial.println();

    ether.sendUdp(PGN_234, sizeof(PGN_234), portMy, ipDestination, portDestination);

  }//end of timed loop

  delay (1);

  ether.packetLoop(ether.packetReceive());
}

//callback when received packets
void udpSteerRecv(uint16_t dest_port, uint8_t src_ip[IP_LEN], uint16_t src_port, uint8_t* udpData, uint16_t len)
{
  /*IPAddress src(src_ip[0],src_ip[1],src_ip[2],src_ip[3]);
  Serial.print("dPort:");  Serial.print(dest_port);
  Serial.print("  sPort: ");  Serial.print(src_port);
  Serial.print("  sIP: ");  ether.printIp(src_ip);  Serial.println("  end");

  for (int16_t i = 0; i < len; i++) {
    Serial.print(udpData[i],HEX); Serial.print("\t"); } 
  Serial.println(len);*/
  
  if (udpData[0] == 0x80 && udpData[1] == 0x81 && udpData[2] == 0x7F) //Data
  {
    if (udpData[3] == 239)  //Machine data
    {
      //Serial.println(" ----- PGN 239 -----");
      uTurn = udpData[5];
      gpsSpeed = (float)udpData[6];//actual speed times 4, single uint8_t

      hydLift = udpData[7];
      tramline = udpData[8]; //bit 0 is right bit 1 is left

      relayLo = udpData[11]; // read relay control from AgOpenGPS SC1to8
      relayHi = udpData[12]; // read relay control from AgOpenGPS SC9to16

      if (aogConfig.isRelayActiveHigh)
      {
        tramline = 255 - tramline;
        relayLo = 255 - relayLo;
        relayHi = 255 - relayHi;
      }

      //Bit 13 CRC

      //reset watchdog
      watchdogTimer = 0;
    }

    //Hello de AOG vers la carte Machine
    else if (udpData[3] == 200) // Hello from AgIO
    {
      //Serial.println(" ----- PGN 200 Hello de AOG vers la carte Machine -----");
      if (udpData[7] == 1)
      {
        relayLo -= 255;
        relayHi -= 255;
        watchdogTimer = 0;
      }

      //byte section = 0;
      helloFromMachine[5] = relayLo;//section;
      helloFromMachine[6] = relayHi;//relay;

      ether.sendUdp(helloFromMachine, sizeof(helloFromMachine), portMy, ipDestination, portDestination);
    }
    
    //Configuration carte Machine
    else if (udpData[3] == 238) 
    {
      //Serial.println(" ----- PGN 238 Configuration carte Machine -----");
      aogConfig.raiseTime = udpData[5];
      aogConfig.lowerTime = udpData[6];
      aogConfig.enableToolLift = udpData[7];

      //set1 
      uint8_t sett = udpData[8];  //setting0     
      if (bitRead(sett, 0)){
        aogConfig.isRelayActiveHigh = 1;
        Serial.print("Relay en mode inversé - Byte 8 - set0 = ");
        Serial.println(aogConfig.isRelayActiveHigh);
      }
      else {
        aogConfig.isRelayActiveHigh = 0;
        Serial.print("Relay en mode normal - Byte 8 - set0 = ");
        Serial.println(aogConfig.isRelayActiveHigh);
      }

      aogConfig.user1 = udpData[9];
      aogConfig.user2 = udpData[10];
      aogConfig.user3 = udpData[11];
      aogConfig.user4 = udpData[12];

      //crc

      //save in EEPROM and restart
      EEPROM.put(6, aogConfig);
      //resetFunc();
    }

    //PGN 201 Reception configuration adresse IP
    else if (udpData[3] == 201)
    {
      //Serial.println(" ----- PGN 201 Reception configuration adresse IP -----");
      //make really sure this is the subnet pgn
      if (udpData[4] == 5 && udpData[5] == 201 && udpData[6] == 201)
      {
        networkAddress.ipOne = udpData[7];
        networkAddress.ipTwo = udpData[8];
        networkAddress.ipThree = udpData[9];

        //save in EEPROM and restart
        EEPROM.put(50, networkAddress);
        resetFunc();
      }
    }

    // Scan request provenant de AgIO quand on ouvre la config réseau
    else if (udpData[3] == 202)
    {
      //make really sure this is the subnet pgn
      if (udpData[4] == 3 && udpData[5] == 202 && udpData[6] == 202)
      {
        Serial.println(" ----- PGN 202 Scan request provenant de AgIO -----");
        uint8_t scanReply[] = { 128, 129, 123, 203, 7, 
        networkAddress.ipOne, networkAddress.ipTwo, networkAddress.ipThree, 123,
        src_ip[0], src_ip[1], src_ip[2], 23   };

        //checksum
        int16_t CK_A = 0;
        for (uint8_t i = 2; i < sizeof(scanReply) - 1; i++)
        {
          CK_A = (CK_A + scanReply[i]);
        }
        scanReply[sizeof(scanReply)-1] = CK_A;

        static uint8_t ipDest[] = { 255,255,255,255 };
        uint16_t portDest = 9999; //AOG port that listens

        //off to AOG
        ether.sendUdp(scanReply, sizeof(scanReply), portMy, ipDest, portDest);
      }
    }

    else if (udpData[3] == 236) //Configuration carte Machine Pin / Relays
    {
      //Serial.println(" ----- PGN 236 Configuration carte Machine Pin / Relays -----");
      for (uint8_t i = 0; i < 24; i++)
      {
        pin[i] = udpData[i + 5];
          Serial.print("Pin ");
          Serial.print(i+1);
          Serial.print(" = ");
          Serial.print(pin[i]);
          Serial.print(" -- ");
      }
      Serial.println();

      //save in EEPROM and restart
      EEPROM.put(20, pin);
    }
  }
}

void RelaysOff()
{
  if (aogConfig.isRelayActiveHigh)
  {
    relayLo = 255;
    relayHi = 255;
  }
  else
  {
    relayLo = 0;
    relayHi = 0;
  }
  SetRelays();
}


void SetRelays(void)
{
  //pin, rate, duration  130 pp meter, 3.6 kmh = 1 m/sec or gpsSpeed * 130/3.6 or gpsSpeed * 36.1111
  //gpsSpeed is 10x actual speed so 3.61111
  gpsSpeed *= 3.61111;
  //tone(13, gpsSpeed);

  //Load the current pgn relay state - Sections
  for (uint8_t i = 0; i < 8; i++)
  {
    relayState[i] = bitRead(relayLo, i);
  }

  for (uint8_t i = 0; i < 8; i++)
  {
    relayState[i + 8] = bitRead(relayHi, i);
  }

  // Hydraulics
  relayState[16] = isLower;
  relayState[17] = isRaise;

  //Tram
  relayState[18] = bitRead(tramline, 0); //right
  relayState[19] = bitRead(tramline, 1); //left

  //GeoStop
  relayState[20] = (geoStop == 0) ? 0 : 1;

  if (pin[0]) digitalWrite(2, relayState[pin[0] - 1]);
  if (pin[1]) digitalWrite(3, relayState[pin[1] - 1]);
  if (pin[2]) digitalWrite(4, relayState[pin[2] - 1]);
  if (pin[3]) digitalWrite(5, relayState[pin[3] - 1]);
  if (pin[4]) digitalWrite(6, relayState[pin[4] - 1]);
  if (pin[5]) digitalWrite(7, relayState[pin[5] - 1]);

  //if (pin[6]) digitalWrite(10, relayState[pin[6]-1]);
  //if (pin[7]) digitalWrite(11, relayState[pin[7]-1]);
  //if (pin[8]) digitalWrite(12, relayState[pin[8]-1]);
  //if (pin[9]) digitalWrite(4, relayState[pin[9]-1]);
  //if (pin[10]) digitalWrite(IO#Here, relayState[pin[10]-1]);
  //if (pin[11]) digitalWrite(IO#Here, relayState[pin[11]-1]);
  //if (pin[12]) digitalWrite(IO#Here, relayState[pin[12]-1]);
  //if (pin[13]) digitalWrite(IO#Here, relayState[pin[13]-1]);
  //if (pin[14]) digitalWrite(IO#Here, relayState[pin[14]-1]);
  //if (pin[15]) digitalWrite(IO#Here, relayState[pin[15]-1]);
  //if (pin[16]) digitalWrite(IO#Here, relayState[pin[16]-1]);
  //if (pin[17]) digitalWrite(IO#Here, relayState[pin[17]-1]);
  //if (pin[18]) digitalWrite(IO#Here, relayState[pin[18]-1]);
  //if (pin[19]) digitalWrite(IO#Here, relayState[pin[19]-1]);
}
