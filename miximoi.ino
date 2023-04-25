#include "LSM6DS3.h"
#include "Wire.h"
#include <math.h>
#include <bluefruit.h>
#include <SimpleKalmanFilter.h>
const int analog_ip = A0;
const int analog_out=A1;
int inputVal = 0;
int out=0;
float mv_per_lsb = 3600.0F / 4096.0F; // 10-bit ADC with 3.3V input range
const int ledRed = 11 ; // pin to use for the LED

BLEService        positionService = BLEService(UUID16_SVC_INDOOR_POSITIONING);
BLECharacteristic ntcCharacter = BLECharacteristic(UUID16_UNIT_ACCELERATION_METRES_PER_SECOND_SQUARED);
BLECharacteristic gyroscopeCharacter = BLECharacteristic(UUID16_UNIT_ANGULAR_ACCELERATION_RADIAN_PER_SECOND_SQUARED);


BLEDis bledis;    // DIS (Device Information Service) helper class instance
BLEBas blebas;    // BAS (Battery Service) helper class instance


float ntc;
int ntcNew;
int absNtc;
uint8_t  bps = 0;

SimpleKalmanFilter bo_loc(1, 1, 0.1);
SimpleKalmanFilter bo_loc1(1, 1, 0.1);
SimpleKalmanFilter bo_loc2(1, 1, 0.1);
SimpleKalmanFilter bo_loc3(1, 1, 0.1);

void setup() {
    Serial.begin(9600);
    analogReadResolution(12);
//    while (!Serial);
    Bluefruit.begin();
    Bluefruit.setName("App BLE");
    pinMode (ledRed , OUTPUT);

    Bluefruit.Periph.setConnectCallback(connect_callback);
    Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

    bledis.setManufacturer("Adafruit Industries");
    bledis.setModel("Bluefruit Feather52");
    bledis.begin();

    blebas.begin();
    blebas.write(100);

    setupNTC();

    startAdv();
}

void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include HRM Service UUID
  Bluefruit.Advertising.addService(positionService);

  // Include Name
  Bluefruit.Advertising.addName();

  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds  
}

void setupNTC(void)
{
 
  positionService.begin();

  ntcCharacter.setProperties(CHR_PROPS_NOTIFY+CHR_PROPS_READ+CHR_PROPS_WRITE );
  ntcCharacter.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  ntcCharacter.setFixedLen(3);
  ntcCharacter.setCccdWriteCallback(cccd_callback);  // Optionally capture CCCD updates
  ntcCharacter.begin();
  uint8_t ntcData[3] = {0b00000000,0b00000000,0b00000000}; // Set the characteristic to use 8-bit values, with the sensor connected and detected
  ntcCharacter.write(ntcData, 3);


  gyroscopeCharacter.setProperties(CHR_PROPS_READ);
  gyroscopeCharacter.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  gyroscopeCharacter.setFixedLen(1);
  gyroscopeCharacter.begin();
  gyroscopeCharacter.write8(2);    // Set the characteristic to 'Wrist' (2)
}

void connect_callback(uint16_t conn_handle)
{

  digitalWrite(ledRed, LOW);
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));

//  Serial.print("Connected to ");
//  Serial.println(central_name);
}
void cccd_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint16_t cccd_value)
{
 

   
    if (chr->uuid == ntcCharacter.uuid) {
        if (chr->notifyEnabled(conn_hdl)) {
        } else {
        }
    }
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
   digitalWrite(ledRed, HIGH);
  (void) conn_handle;
  (void) reason;
}

uint8_t convertData(int data){
  uint8_t res = 0;
  res = res + abs(data);
  return res;
}

uint8_t check0(int data){
  
  uint8_t res = 1;
  if(data<0) res = 0;
  
  return res;
}


//-36.04  * 1000 = -3604 

float X, Y, Z;
int xNew,yNew,zNew,trunggian;
float absX;
void loop() {
  float xFilter,yFilter,zFilter;
    float u_kalman;
    float ntcFilter;
    float nhietdo,Vout,Rth,Vb;
   inputVal = analogRead(analog_ip);
    Serial.print("ADC Vb reading :");
    Serial.println(inputVal); // in giá trị adc đọc được 
   // inputVal = analogRead(analog_ip) * 0.2 + inputVal *0.8; // Áp dụng bộ lọc thông thấp với mức độ thấp

    out = analogRead(analog_out);
    Serial.print("ADC Vout reading :");
    Serial.println(out); // in giá trị adc đọc được 
  //  out = analogRead(analog_out) * 0.2 + out *0.8; // Áp dụng bộ lọc thông thấp với mức độ thấp

    Serial.print("Voltage Vb --:");
    Serial.println((float)inputVal * mv_per_lsb); // giá trị voltage Vb
    Serial.print("Voltage Vout --:");
    Serial.println((float)out * mv_per_lsb); // giá trị voltage Vout
    Vb  =(float)inputVal * mv_per_lsb;

    Vout =(float)out * mv_per_lsb;
    Rth = ((Vb-Vout)*10000 ) / (3300-Vb);
    Serial.print("Giá trị NTC :");
    Serial.println(Rth); // giá trị NTC
    nhietdo = 1 / ( (log(Rth/10000)/3950) + (1/298.15) ) - 273.15;// tính trị nhiệt độ
    X = (float)nhietdo;
    //xFilter = bo_loc1.updateEstimate(X);
    Serial.print("Nhiệt độ tính toán :");
    Serial.print(X); // in giá nhiệt độ tính được 
     
     Serial.print(" ----- ");
    xFilter = bo_loc1.updateEstimate(X);// bộ lọc kalman
   //  Serial.print("Nhiệt độ tính toán :");
    Serial.println(xFilter); // in giá nhiệt độ tính được 
    xNew=(int)(xFilter*100);
   // xNew=(int)(X*100);
    absX=(int)xNew/100;
    
    if ( Bluefruit.connected() ) { 
    uint8_t  ntcData[3] = {
      check0(xNew),
      convertData(xNew/100),
      convertData(xNew- (xNew / 100)*100)
    };           // Sensor connected, increment BPS value
   Serial.println(X);
   Serial.println(xNew);
   Serial.println(absX);
   
   Serial.println(check0(xNew));
   Serial.println(convertData(xNew/100));
   Serial.println(convertData(xNew- (xNew / 100)*100));
    
    if ( ntcCharacter.notify(ntcData, sizeof(ntcData)) ){
    }else{
    }
  }
 
    delay(1000);
}