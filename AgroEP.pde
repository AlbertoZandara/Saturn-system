#include <WaspXBee802.h>
#include <WaspFrame.h>
#include <WString.h>
#include <Wire.h>  

//////////////////////////////////////
////////// Sensors data //////////////
//////////////////////////////////////

//GY30
uint8_t gy30ICAdress = 0x23; //I2C address for the luminosity sensor
int luxIndex = 0;

//Capacitive 1.2
const int AirValue = 800;   //you need to replace this value with the value produced by the sensor in air
const int WaterValue = 480;  //you need to replace this value with the value produced by the sensor in water
int moistureIndex = 0;

//DS1820
float tempIndex = 0;

//BMP280
#define BMP280_ADDRESS 0x76
unsigned long int temp_raw,pres_raw;
signed long int t_fine;
double BMPIndex[2];

uint16_t dig_T1;
int16_t dig_T2;
int16_t dig_T3;
uint16_t dig_P1;
int16_t dig_P2;
int16_t dig_P3;
int16_t dig_P4;
int16_t dig_P5;
int16_t dig_P6;
int16_t dig_P7;
int16_t dig_P8;
int16_t dig_P9;

uint8_t osrs_t = 1;             //Temperature oversampling x 1
uint8_t osrs_p = 1;             //Pressure oversampling x 1
uint8_t mode = 3;               //Normal mode
uint8_t t_sb = 5;               //Tstandby 1000ms
uint8_t filter = 0;             //Filter off 
uint8_t spi3w_en = 0;           //3-wire SPI Disable

uint8_t ctrl_meas_reg = (osrs_t << 5) | (osrs_p << 2) | mode;
uint8_t config_reg    = (t_sb << 5) | (filter << 2) | spi3w_en;

//Raindrop module
const int RainMax = 1024;  
const int RainMin = 512;  
int rainIndex;

//UVM30A
uint16_t uvIndexValue [11] = { 227, 318, 408, 503, 606, 696, 795, 881, 976, 1079, 1170};
int uvIndex = 0;

//////////////////////////////////////
///////// 802.15.4 data //////////////
//////////////////////////////////////

// PAN (Personal Area Network) Identifier
uint8_t panID[2] = {0x11,0x14};         //User-defined PANID
uint8_t std_panID[2] = {0x07,0x09};     //Standard PANID for broadcast pairing

// Define Freq Channel to be set: 
// Center Frequency = 2.405 + (CH - 11d) * 5 MHz
//   Range: 0x0B - 0x1A (XBee)
//   Range: 0x0C - 0x17 (XBee-PRO)
uint8_t  std_channel = 0x11;            //User-defined CHANNEL
uint8_t  channel = 0x11;                //Standard CHANNEL for broadcast pairing

// Define the Encryption mode: 1 (enabled) or 0 (disabled)
uint8_t encryptionMode;
uint8_t std_encryptionMode = 0;

// Define the AES 16-byte Encryption Key
char  encryptionKey[16]; 

// Destination MAC address
//////////////////////////////////////////
// char RX_ADDRESS[] = "0013A200416B4CBC";
//////////////////////////////////////////
char BROADCAST_ADDRESS[] = "000000000000FFFF";
uint8_t C_ADDRESS [8];       //Coordinator address

// Define the Waspmote ID
char WASPMOTE_ID[] = "Agro_EP";

// define variable
String Xbee_settings;
int msg_length;
uint8_t error;

//////////////////////////////////////
//////////   RTC data   //////////////
//////////////////////////////////////

char Year[3], Month[3], Day[3], Dow[2], Hour[3], Minute[3], Second[3];
int order;
long startTime, elapsedTime;

void Xbee_default()
{

  USB.println(F("-------------------------------"));
  USB.println(F("----- Reset XBee 802.15.4 -----"));
  USB.println(F("-------------------------------"));

  
  // 1. set channel 
  
  xbee802.setChannel( std_channel );

  // check at commmand execution flag
  if( xbee802.error_AT == 0 ) 
  {
    USB.print(F("1. Channel set OK to: 0x"));
    USB.printHex( xbee802.channel );
    USB.println();
  }
  else 
  {
    USB.println(F("1. Error calling 'setChannel()'"));
  }


  // 2. set PANID
  
  xbee802.setPAN( std_panID );

  // check the AT commmand execution flag
  if( xbee802.error_AT == 0 ) 
  {
    USB.print(F("2. PAN ID set OK to: 0x"));
    USB.printHex( xbee802.PAN_ID[0] ); 
    USB.printHex( xbee802.PAN_ID[1] ); 
    USB.println();
  }
  else 
  {
    USB.println(F("2. Error calling 'setPAN()'"));  
  }


  // 3. set encryption mode (1:enable; 0:disable)
  xbee802.setEncryptionMode( std_encryptionMode );

  // check the AT commmand execution flag
  if( xbee802.error_AT == 0 ) 
  {
    USB.print(F("3. AES encryption configured:"));
    if(xbee802.encryptMode == 1)
      USB.println("ENABLED");
    else if (xbee802.encryptMode == 0)
      USB.println("DISABLED");
    else
      USB.println(F("3. Error calling 'setEncryptionMode()'"));
  }
  else 
  {
    USB.println(F("3. Error calling 'setEncryptionMode()'"));
  }


  // 4. write values to XBee module memory

  xbee802.writeValues();
  
  // check the AT commmand execution flag
  if( xbee802.error_AT == 0 ) 
  {
    USB.println(F("4. Changes stored OK"));
  }
  else 
  {
    USB.println(F("4. Error calling 'writeValues()'"));   
  }

  USB.println(F("----------------------------------------------"));
  USB.println(F("----- Standard 802.15.4 values adjusted. -----"));
  USB.println(F("----------------------------------------------"));
  
}


void Xbee_update(String Set, int len)
{
  // Example of received string:  <=>�#6A443F057C10542C#Agro_C#0#PAN_ID:7;9#CH:17#ENCRY:1#PW:Saturn-encrypted#       IN NODERED: #PAN_ID:7;9#CH:17#ENCRY:1#PW:Saturn-encrypted#
  int j = 0, k = 0, l = 0;
  String PAN, CH, EN, PW;
  for(int i = 0; i < Set.lastIndexOf('#'); i = Set.indexOf('#', i+1))
  {  
    switch (j) {
      
      case 0:             //After 4 fields separated by # there is the PANID *****BUT NOT IN NODERED THERE IS AFTER 1 #*****
      
        for(k = 0; Set[i+8+k] != ';'; k++)
          PAN += Set[i+8+k];
        panID[0] = PAN.toInt();

        PAN = 0;
        
        for(l = 0; Set[i+9+k+l] != '#'; l++)
          PAN += Set[i+9+k+l];
        panID[1] = PAN.toInt();

        USB.print(F("PANID detected:"));
        USB.printHex(panID[0]); 
        USB.printHex(panID[1]); 
        USB.println();
        break;
        
      case 1:             //After 5 fields separated by # there is the CHANNEL
        
        for(k = 0; Set[i+4+k] != '#'; k++)
          CH += Set[i+4+k];
        channel = CH.toInt();

        USB.print(F("CHANNEL detected:"));
        USB.printHex(channel);  
        USB.println();
        break;
        
      case 2:             //After 6 fields separated by # there is the ENCRYPTION MODE
        
        EN = Set[i+7];
        encryptionMode = EN.toInt();
        
        USB.print(F("Encryption:"));
        if(encryptionMode == 1)
          USB.println("ENABLED");
        else if (encryptionMode == 0)
          USB.println("DISABLED");
        else
          USB.println(F("Encryption value error!"));  
        break;

      case 3:             //After 7 fields separated by # there is the PASSWORD
        
        for(k = 0; Set[i+4+k] != '#'; k++)
          PW += Set[i+4+k];
        PW.toCharArray(encryptionKey, 17);

        USB.print(F("PASSWORD detected:"));
        USB.println(encryptionKey);
        if(k == 16)  
          USB.println("The password has the correct length.");
        break;
        
    }
    
    j++;
    
  }
}



void Xbee_config()
{
  
  USB.println("Waiting for a Coordinator:");
  
  //Waiting broadcast packet
  error = xbee802.receivePacketTimeout( 100000 );

  // check answer  
  if( error == 0 ) 
  {
    Xbee_settings = String((char *)xbee802._payload);
    msg_length = xbee802._length;
    
    USB.print(F("Data: "));  
    USB.println( (const char*)&Xbee_settings[0] );

    // Show data stored in '_payload' buffer indicated by '_length'
    USB.print(F("Length: "));  
    USB.println( msg_length );

    C_ADDRESS[0] = xbee802._srcMAC[0];
    C_ADDRESS[1] = xbee802._srcMAC[1];
    C_ADDRESS[2] = xbee802._srcMAC[2];
    C_ADDRESS[3] = xbee802._srcMAC[3];
    C_ADDRESS[4] = xbee802._srcMAC[4];
    C_ADDRESS[5] = xbee802._srcMAC[5];
    C_ADDRESS[6] = xbee802._srcMAC[6];
    C_ADDRESS[7] = xbee802._srcMAC[7];
    
    // Show data stored in '_payload' buffer indicated by '_length'
    USB.print(F("Source MAC Address: "));  
    USB.printHex( C_ADDRESS[0] );
    USB.printHex( C_ADDRESS[1] );
    USB.printHex( C_ADDRESS[2] );
    USB.printHex( C_ADDRESS[3] );
    USB.printHex( C_ADDRESS[4] );
    USB.printHex( C_ADDRESS[5] );
    USB.printHex( C_ADDRESS[6] );
    USB.printHex( C_ADDRESS[7] );
    USB.println();    
    USB.println(F("--------------------------------"));
  }
  else
  {
    // Print error message:
    /*
     * '7' : Buffer full. Not enough memory space
     * '6' : Error escaping character within payload bytes
     * '5' : Error escaping character in checksum byte
     * '4' : Checksum is not correct    
     * '3' : Checksum byte is not available 
     * '2' : Frame Type is not valid
     * '1' : Timeout when receiving answer   
    */
    USB.print(F("Error receiving a packet:"));
    USB.println(error,DEC);     
    USB.println(F("--------------------------------"));
  }

  // The end point need to updated the settengs according to broadcast packet received
  Xbee_update(Xbee_settings, msg_length);

  
}


void Xbee_setup ()
{
  
  USB.println(F("--------------------------------------------"));
  USB.println(F("----- Setting 802.15.4 user parameters -----"));
  USB.println(F("--------------------------------------------"));

  
  // 1. set channel 
  
  xbee802.setChannel( channel );

  // check at commmand execution flag
  if( xbee802.error_AT == 0 ) 
  {
    USB.print(F("1. Channel set OK to: 0x"));
    USB.printHex( xbee802.channel );
    USB.println();
  }
  else 
  {
    USB.println(F("1. Error calling 'setChannel()'"));
  }


  // 2. set PANID
  
  xbee802.setPAN( panID );

  // check the AT commmand execution flag
  if( xbee802.error_AT == 0 ) 
  {
    USB.print(F("2. PAN ID set OK to: 0x"));
    USB.printHex( xbee802.PAN_ID[0] ); 
    USB.printHex( xbee802.PAN_ID[1] ); 
    USB.println();
  }
  else 
  {
    USB.println(F("2. Error calling 'setPAN()'"));  
  }


  // 3. set encryption mode (1:enable; 0:disable)

  xbee802.setEncryptionMode( encryptionMode );

  // check the AT commmand execution flag
  if( xbee802.error_AT == 0 ) 
  {
    USB.print(F("3. AES encryption configured:"));
    USB.print( xbee802.encryptMode);
    if(xbee802.encryptMode == 1)
      USB.println("ENABLED");
    else if (xbee802.encryptMode == 0)
      USB.println("DISABLED");
    else
      USB.println(F("3. Error calling 'setEncryptionMode()'"));
  }
  else 
  {
    USB.println(F("3. Error calling 'setEncryptionMode()'"));
  }


  // 4. set encryption key

  xbee802.setLinkKey( encryptionKey );

  // check the AT commmand execution flag
  if( xbee802.error_AT == 0 ) 
  {
    USB.println(F("4. AES encryption key set OK"));
  }
  else 
  {
    USB.println(F("4. Error calling 'setLinkKey()'")); 
  }

  // 5. write values to XBee module memory

  xbee802.writeValues();
  
  // check the AT commmand execution flag
  if( xbee802.error_AT == 0 ) 
  {
    USB.println(F("5. Changes stored OK"));
  }
  else 
  {
    USB.println(F("5. Error calling 'writeValues()'"));   
  }

  USB.println(F("----------------------------------------------------"));
  USB.println(F("-----  User-defined 802.15.4 values adjusted.  -----"));
  USB.println(F("----------------------------------------------------"));
  
  
}
  //Creating network
  // Sending broadcast message with PAN CH ENCRY PW 
  // Wait for hello message
  // Sending ORDER RTC
  // When TIMEOUT send start message

void dominusAdvertisement ()
{
  frame.createFrame(ASCII); 

  // set frame fields (String - char*)
  frame.addSensor(SENSOR_STR, "Agro PAIRED");

  error = xbee802.send( C_ADDRESS, frame.buffer, frame.length );   
    
  // check TX flag
  if( error == 0 )
  {
    //USB.println(F("send ok"));
    
    // blink green LED
    Utils.blinkGreenLED();    
  }
  else 
  {
    USB.println(F("send error"));
    
    // blink red LED
    Utils.blinkRedLED();  
  }

  USB.println(F("#####################################"));
  USB.println(F("#####   DOMINUS ADVERTISED!!!   #####"));
  USB.println(F("#####################################"));
}

int timeFix(String Time)
{
  // Example of received string:  #DATE:2021-03-02-2#TIME:15:42:28.166#ORDER:1#       IN NODERED: #PAN_ID:7;9#CH:17#ENCRY:1#PW:Saturn-encrypted#

  String YYYY, MM, DD, D, HH, MMs, SS, NN;
  char clk[20];
  int ord, k;
  
  for(k = 2; Time[6+k] != '-'; k++)
    YYYY += Time[6+k];
  YYYY.toCharArray(Year, sizeof(Year));
//  itoa(YYYY.toInt(), Year, 10);
//  Year = YYYY.toInt();
  
  for( ; Time[7+k] != '-'; k++)
    MM += Time[7+k];
  MM.toCharArray(Month, sizeof(Month));
//  itoa(MM.toInt(), Month, 10);
//  Month = MM.toInt();

  for( ; Time[8+k] != '-'; k++)
    DD += Time[8+k];
  DD.toCharArray(Day, sizeof(Day));
//  itoa(DD.toInt(), Day, 10);
//  Day = DD.toInt();

  for( ; Time[9+k] != '#'; k++)
    D += Time[9+k];
  itoa(D.toInt()+1, Dow, 10);
//  Dow = D.toInt();

  for( ; Time[15+k] != ':'; k++)
    HH += Time[15+k];
  HH.toCharArray(Hour, sizeof(Hour));
//  itoa(HH.toInt(), Hour, 10);
//  Hour = HH.toInt();

  for( ; Time[16+k] != ':'; k++)
    MMs += Time[16+k];
  MMs.toCharArray(Minute, sizeof(Minute));
//  itoa(MMs.toInt(), Minute, 10);
//  Minute = MMs.toInt();

  for( ; Time[17+k] != '.'; k++)
    SS += Time[17+k];
  SS.toCharArray(Second, sizeof(Second));
//  itoa(SS.toInt(), Second, 10);
//  Second = SS.toInt();

  for( ; Time[28+k] != '#'; k++)
    NN += Time[17+k];
  ord = NN.toInt();


//  USB.print(F("Date: "));
//  USB.print(Day); 
//  USB.print(F("-"));
//  USB.print(Month); 
//  USB.print(F("-"));
//  USB.print(Year); 
//  USB.print(F("-"));
//  USB.print(Dow); 
//  USB.print(F("   Time: "));
//  USB.print(Hour); 
//  USB.print(F(":"));
//  USB.print(Minute); 
//  USB.print(F(":"));
//  USB.print(Second); 
//  USB.println();

  strcpy(clk, Year);
  strcat(clk, ":");
  strcat(clk, Month);
  strcat(clk, ":");
  strcat(clk, Day);
  strcat(clk, ":");
  strcat(clk, "0\0");
  strcat(clk, Dow);
  strcat(clk, ":");
  strcat(clk, Hour);
  strcat(clk, ":");
  strcat(clk, Minute);
  strcat(clk, ":");
  strcat(clk, Second);
  
//  USB.println(clk);

  RTC.setTime(clk);
  USB.println(RTC.getTime());
  
  return ord;
  
}

int dominusStart ()
{
  int ord;
  
  USB.println(F("-------------------------------------------------"));
  USB.println(F("-----   Waiting for dominus start command   -----"));
  USB.println(F("-------------------------------------------------"));
  error = xbee802.receivePacketTimeout( 100000 );

  // check answer  
  if( error == 0 ) 
  {
    // Show data stored in '_payload' buffer indicated by '_length'
    USB.print(F("--> Data: "));  
    USB.println( xbee802._payload, xbee802._length);

    
    USB.println(F("#####################################"));
    USB.println(F("#####     START DETECTED!!!     #####"));
    USB.println(F("#####################################"));
  }
  else
  {
    // Print error message:
    /*
     * '7' : Buffer full. Not enough memory space
     * '6' : Error escaping character within payload bytes
     * '5' : Error escaping character in checksum byte
     * '4' : Checksum is not correct    
     * '3' : Checksum byte is not available 
     * '2' : Frame Type is not valid
     * '1' : Timeout when receiving answer   
    */
    USB.print(F("Error receiving a packet:"));
    USB.println(error,DEC);     
  }

  ord = timeFix(String((char *)xbee802._payload));
  
  return ord;
}

void TDM (int ord)
{
  String Clk = RTC.getTime(), SS;                // Fri, 13/01/11, 12:33:00
  int i = Clk.lastIndexOf(':') + 1, Sec;

  do {
    Clk = RTC.getTime();
    i = Clk.lastIndexOf(':') + 1;
    for(SS = ""; i < Clk.lastIndexOf(':') + 3; i++)
      SS += Clk[i];
    Sec = SS.toInt();
//    USB.println(Sec);   
  } while (Sec != ord);

  USB.print(F("Time-slot identified! Time:   "));   
  USB.println(RTC.getTime());
   
}

void timeUpdate(int ord)
{
  error = xbee802.receivePacketTimeout( 60000 );

  // check answer  
  if( error == 0 ) 
  {
    // Show data stored in '_payload' buffer indicated by '_length'
    USB.print(F("--> Data: "));  
    USB.println( xbee802._payload, xbee802._length);
  }
  else
  {
    // Print error message:
    /*
     * '7' : Buffer full. Not enough memory space
     * '6' : Error escaping character within payload bytes
     * '5' : Error escaping character in checksum byte
     * '4' : Checksum is not correct    
     * '3' : Checksum byte is not available 
     * '2' : Frame Type is not valid
     * '1' : Timeout when receiving answer   
    */
    USB.print(F("Error receiving a packet:"));
    USB.println(error,DEC);     
  }

  timeFix(String((char *)xbee802._payload));
}

void isMidnight(char* Time, int ord)
{
  String Clk = Time, HH = "", MM = "";
  int k = 15;
  
  for(; Clk[k] != ':'; k++)
    HH += Clk[k];
  
  k++;
  
  for(; Clk[k] != ':'; k++)
    MM += Clk[k];

//  USB.println(HH.toInt());
//  USB.println(MM.toInt());

  if (HH.toInt() == 23 && MM.toInt() == 59)       //Hour has to be 23 and minute has to be 59
    timeUpdate(ord);
}

//////////////////////////////////////
///////// Sensors functions //////////
//////////////////////////////////////

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//UVM30A
int readUVSensor()
{
  int i = 0;
  int uvValue = 0;
  int UVIndex = 0;
  
  uvValue = analogRead(ANALOG4);                        //connect UV sensor to Analog 4
  //USB.println(uvValue);

  int voltage = (uvValue * (5.0 / 1023.0))*1000;  //Voltage in miliVolts
  //USB.println(voltage);


  for (i = 0; voltage > uvIndexValue[i] && voltage < uvIndexValue[10]; i++)
  {}
  if (voltage > uvIndexValue[10]) 
    UVIndex = 11;
  else
    UVIndex = i;
  
  return UVIndex;
}

// Raindrop module
int readRainSensor ()
{
  int RainValue = 0;
  int RainPercent = 0;
  int rslt;
  RainValue = analogRead(ANALOG3);  //put Sensor insert into soil
  //USB.println(RainValue);
  RainPercent = map(RainValue, RainMax, RainMin, 0, 100);
  
  if(RainPercent >= 100)
  {
    rslt = 100;
  }
  else if(RainPercent <=0)
  {
    rslt = 0;
  }
  else if(RainPercent >0 && RainPercent < 100)
  {
    rslt = RainPercent;
  }

  return rslt;
}

// BMP280
void receiveTrim()
{
    uint8_t data[32],i=0;                      
    Wire.beginTransmission(BMP280_ADDRESS);
    Wire.send(0x88);
    Wire.endTransmission();
    Wire.requestFrom(BMP280_ADDRESS,24);       
    while(Wire.available()){
        data[i] = Wire.receive();
        i++;
    }

    Wire.beginTransmission(BMP280_ADDRESS);    
    Wire.send(0xA1);                          
    Wire.endTransmission();                   
    Wire.requestFrom(BMP280_ADDRESS,1);       
    data[i] = Wire.receive();                  
    i++;                                     
    Wire.beginTransmission(BMP280_ADDRESS);
    Wire.send(0xE1);
    Wire.endTransmission();
    Wire.requestFrom(BMP280_ADDRESS,7);        
    while(Wire.available()){
        data[i] = Wire.receive();
        i++;    
    }
    dig_T1 = (data[1] << 8) | data[0];
    dig_T2 = (data[3] << 8) | data[2];
    dig_T3 = (data[5] << 8) | data[4];
    dig_P1 = (data[7] << 8) | data[6];
    dig_P2 = (data[9] << 8) | data[8];
    dig_P3 = (data[11]<< 8) | data[10];
    dig_P4 = (data[13]<< 8) | data[12];
    dig_P5 = (data[15]<< 8) | data[14];
    dig_P6 = (data[17]<< 8) | data[16];
    dig_P7 = (data[19]<< 8) | data[18];
    dig_P8 = (data[21]<< 8) | data[20];
    dig_P9 = (data[23]<< 8) | data[22];
}

void sendReg(uint8_t reg_address, uint8_t data)
{
    Wire.beginTransmission(BMP280_ADDRESS);
    Wire.send(reg_address);
    Wire.send(data);
    Wire.endTransmission();    
}

void receiveData()
{
    int i = 0;
    uint32_t data[8];
    Wire.beginTransmission(BMP280_ADDRESS);
    Wire.send(0xF7);
    Wire.endTransmission();
    Wire.requestFrom(BMP280_ADDRESS,8);
    while(Wire.available()){
        data[i] = Wire.receive();
        i++;
    }
    pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
    temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);
}

signed long int calibration_T(signed long int adc_T)
{

    signed long int var1, var2, T;
    var1 = ((((adc_T >> 3) - ((signed long int)dig_T1<<1))) * ((signed long int)dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((signed long int)dig_T1)) * ((adc_T>>4) - ((signed long int)dig_T1))) >> 12) * ((signed long int)dig_T3)) >> 14;

    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T; 
}

unsigned long int calibration_P(signed long int adc_P)
{
    signed long int var1, var2;
    unsigned long int P;
    var1 = (((signed long int)t_fine)>>1) - (signed long int)64000;
    var2 = (((var1>>2) * (var1>>2)) >> 11) * ((signed long int)dig_P6);
    var2 = var2 + ((var1*((signed long int)dig_P5))<<1);
    var2 = (var2>>2)+(((signed long int)dig_P4)<<16);
    var1 = (((dig_P3 * (((var1>>2)*(var1>>2)) >> 13)) >>3) + ((((signed long int)dig_P2) * var1)>>1))>>18;
    var1 = ((((32768+var1))*((signed long int)dig_P1))>>15);
    if (var1 == 0)
    {
        return 0;
    }    
    P = (((unsigned long int)(((signed long int)1048576)-adc_P)-(var2>>12)))*3125;
    if(P<0x80000000)
    {
       P = (P << 1) / ((unsigned long int) var1);   
    }
    else
    {
        P = (P / (unsigned long int)var1) * 2;    
    }
    var1 = (((signed long int)dig_P9) * ((signed long int)(((P>>3) * (P>>3))>>13)))>>12;
    var2 = (((signed long int)(P>>2)) * ((signed long int)dig_P8))>>13;
    P = (unsigned long int)((signed long int)P + ((var1 + var2 + dig_P7) >> 4));
    return P;
}

void readBMPSensor(double &temp, double &press)
{
  double temp_act = 0.0, press_act = 0.0;
  signed long int temp_cal;
  unsigned long int press_cal;

  receiveData();

  temp_cal = calibration_T(temp_raw);
  press_cal = calibration_P(pres_raw);
  temp_act = (double)temp_cal / 100.0;
  press_act = (double)press_cal / 100.0;
  temp = temp_act;
  press = press_act;
}

//GY30
void initSensor(){
  Wire.beginTransmission(gy30ICAdress);
  Wire.send(0x10); //Sensor accuracy
  Wire.endTransmission();
}

int readLUXSensor(){
  byte buffer[2]; //Array to store luminosity data
  int rslt = 0;
  byte value = 0;
  Wire.beginTransmission(gy30ICAdress);
  Wire.requestFrom(gy30ICAdress, 2);
  
  while(Wire.available()){
    buffer[value] = Wire.receive();
    value++;
  }
  Wire.endTransmission();

  rslt = ((buffer[0] << 8) | buffer[1]);
  
  return rslt;
}

//Capacitive 1.2
int readMoisture ()
{
  int soilMoistureValue = 0;
  int soilmoisturepercent = 0;
  int rslt = 0;
  soilMoistureValue = analogRead(ANALOG2);  //put Sensor insert into soil
  soilmoisturepercent = map(soilMoistureValue, AirValue, WaterValue, 0, 100);
    
  if(soilmoisturepercent >= 100)
  {
    rslt = 100;
  }
  else if(soilmoisturepercent <=0)
  {
    rslt = 0;
  }
  else if(soilmoisturepercent >0 && soilmoisturepercent < 100)
  {
    rslt = soilmoisturepercent;
  }

  return rslt;
}


// Hub initialization
void sensInitialization ()
{
  // GY30
  initSensor();

  // BMP280
  sendReg(0xF4,ctrl_meas_reg);
  sendReg(0xF5,config_reg);
  receiveTrim();
}


void setup()
{
  startTime = millis();

  PWR.ifHibernate();
    
  if( intFlag & HIB_INT )
  {
    USB.println(RTC.getTime());


    PWR.setSensorPower(SENS_5V, SENS_ON);
    PWR.setSensorPower(SENS_3V3, SENS_ON);
    
    // Activating RTC for synchronization
    RTC.ON();
  
    //I2C initiualization
    Wire.begin();
    
    // Open USB port
    USB.ON();

    // Init Xbee
    xbee802.ON();

    // Sensor initialization
    sensInitialization();

    C_ADDRESS[0] = Utils.readEEPROM(1024);
    C_ADDRESS[1] = Utils.readEEPROM(1025);
    C_ADDRESS[2] = Utils.readEEPROM(1026);
    C_ADDRESS[3] = Utils.readEEPROM(1027);
    C_ADDRESS[4] = Utils.readEEPROM(1028);
    C_ADDRESS[5] = Utils.readEEPROM(1029);
    C_ADDRESS[6] = Utils.readEEPROM(1030);
    C_ADDRESS[7] = Utils.readEEPROM(1031);
    order = Utils.readEEPROM(1032);

    intFlag &= ~(HIB_INT); 
    delay(853);
  }
  else
  {
    //Power to the hub
    PWR.setSensorPower(SENS_5V, SENS_ON);
    PWR.setSensorPower(SENS_3V3, SENS_ON);
    
    // Activating RTC for synchronization
    RTC.ON();
  
    //I2C initiualization
    Wire.begin();
    
    // Open USB port
    USB.ON();
  
    // Sensor initialization
    sensInitialization();
    
    USB.println(F("Starting 802.15 End-point..."));
  
    // Store Waspmote identifier in EEPROM memory
    frame.setID( WASPMOTE_ID );
  
    // Init Xbee
    xbee802.ON();
  
    // Setting standard parameters to receive the broadcast initialization packet
    Xbee_default();
  
    // Handshake 1st phase --- Starting cofiguration receiving settings broadcast message and extracting parameters
    Xbee_config();
  
    Utils.writeEEPROM(1024,C_ADDRESS[0]);
    Utils.writeEEPROM(1025,C_ADDRESS[1]);
    Utils.writeEEPROM(1026,C_ADDRESS[2]);
    Utils.writeEEPROM(1027,C_ADDRESS[3]);
    Utils.writeEEPROM(1028,C_ADDRESS[4]);
    Utils.writeEEPROM(1029,C_ADDRESS[5]);
    Utils.writeEEPROM(1030,C_ADDRESS[6]);
    Utils.writeEEPROM(1031,C_ADDRESS[7]);
    
    // Applying user-defined parameters
    Xbee_setup();
  
    delay(2000);
  
    // Handshake 2nd phase --- Advertise the dominus that the agro is ready
    dominusAdvertisement();
    
    // Handshake 3rd phase --- Waiting Coordinator start
    order = dominusStart();

    Utils.writeEEPROM(1032,order);

    TDM(order);
    
    delay (1004);

  }
}



void loop()
{
  //UVM30A
  uvIndex = readUVSensor(); 
//  USB.print("UV index: ");
//  USB.println(uvIndex);

  delay(50);

  //Raindrop module
  rainIndex = readRainSensor();
//  USB.print("Rain index: ");
//  USB.print(rainIndex);
//  USB.println(" %");

  delay(50);

  //BMP280 temp and pressure
  readBMPSensor(BMPIndex[0], BMPIndex[1]);
//  USB.print(F("External temperature : "));
//  USB.print(BMPIndex[0]);
//  USB.print(F(" degrees  ---  PRESS : "));
//  USB.print(BMPIndex[1]);
//  USB.println(F(" bar"));

  delay(50);

  //DS1820 temp
  tempIndex = Utils.readTempDS1820(DIGITAL8);
//  USB.print(F("Soil temperature: "));
//  USB.print(tempIndex);
//  USB.println(F(" degrees"));
  
  delay(50);

  //GY30 Lux
  luxIndex = readLUXSensor();
//  USB.print("Luminosity: ");
//  USB.print(luxIndex);
//  USB.println(" Lux");

  delay(50);

  //Capacitive 1.2 Moisture
  moistureIndex = readMoisture();
//  USB.print("Soil moisture: ");
//  USB.print(moistureIndex);
//  USB.println(" %");

  delay(50);

  
  frame.createFrame(ASCII); 
  
  // Set frame fields (String - char*)
  //frame.addSensor(SENSOR_STR, "Agro data");
  frame.addSensor(BMP_TC, BMPIndex[0]);
  frame.addSensor(BMP_PRES, BMPIndex[1]);
  frame.addSensor(GY30, luxIndex);
  frame.addSensor(UVM30A, uvIndex);
  frame.addSensor(RAIN, rainIndex);
  frame.addSensor(DS1820, tempIndex);
  frame.addSensor(MOISTURE, moistureIndex);
  frame.addSensor(SENSOR_BAT, PWR.getBatteryLevel()); 
  
  error = xbee802.send( C_ADDRESS, frame.buffer, frame.length );   
    
  // check TX flag
  if( error == 0 )
  {
//    USB.println(F("send ok"));
    
    // blink green LED
    Utils.blinkGreenLED();    
  }
  else 
  {
    USB.println(F("send error"));
    
    // blink red LED
    Utils.blinkRedLED();  
  }

  isMidnight(RTC.getTime(), order);
  
  elapsedTime =   millis();
  
//  elapsedTime = elapsedTime - startTime;
//  USB.println(elapsedTime);
  
  PWR.hibernate("00:00:00:06", RTC_OFFSET, RTC_ALM1_MODE2); 
}


