#include "Sensirion_GadgetBle_Lib.h"
#include "DFRobot_OxygenSensor.h"
#include <Omron_D6FPH.h>
#include <Wire.h>
Omron_D6FPH mySensor;
float area_1 = 0.000531; //these are the areas taken carefully from the 3D printed venturi 2M before constriction
float area_2 = 0.000201;// this is area within the venturi
float rho = 1.225; //Demsity of air in kg/m3;
float massFlow = 0;
float volFlow = 0;
float maxFlow = 0.0;
float volumeTotal = 0;
float TimerNow = 0.0;
float timerThen = 0.0;
float oneMinute = 0.0;
DFRobot_OxygenSensor Oxygen;
#define COLLECT_NUMBER    10             // collect number, the collection range is 1-100.
#define Oxygen_IICAddress ADDRESS_3
GadgetBle gadgetBle = GadgetBle(GadgetBle::DataType::T_RH_CO2);
const int16_t SCD_ADDRESS = 0x62;
float co2, temperature, humidity;
  
uint8_t data[12], counter;

void setup()
{
  
Wire.begin();
Serial.begin(115200);
while(!Serial);
gadgetBle.begin();
 delay(1000);
  Serial.print("Sensirion GadgetBle Lib initialized with deviceId = ");
  Serial.println(gadgetBle.getDeviceIdString());
Wire.beginTransmission(0x25); //Start communication over i2c on channel 25 (page 6 in cut sheet)
Wire.write(0x36); Wire.write(0x03); //Start sensor in mode "Averate till read, Mass flow", use 3615 for DP (see page 7 in cutsheet)
Wire.endTransmission();
Wire.beginTransmission(SCD_ADDRESS);
  Wire.write(0x21);
  Wire.write(0xb1);
  Wire.endTransmission();
   while(!Oxygen.begin(Oxygen_IICAddress)) {
    Serial.println("I2c device number error !");
    delay(1000);
  }
  Serial.println("I2c connect success !");
mySensor.begin(MODEL_0025AD1);
timerThen = millis();
oneMinute = millis();
  // wait for first measurement to be finished
  delay(5000);
delay(8); //First measurement is available after 8ms, page 7 in cut sheet
}
void loop()
{
  //if((millis() - timerThen) > 10000)seeO();

//Read pressure from SDP810
float pressure = mySensor.getPressure();
    if(isnan(pressure)){
    Serial.println("Error: Could not read pressure data");
    }else{
        //Serial.print("Differential Pressure: ");
        //Serial.println(pressure);
    }
if(pressure > 1){
massFlow = 1000*sqrt((abs(pressure)*2*rho)/((1/(pow(area_2,2)))-(1/(pow(area_1,2))))); //Bernoulli equation
volFlow = massFlow/rho; //volumetric flow of air
if(volFlow > maxFlow) maxFlow = volFlow;

volumeTotal = volFlow * (millis() - TimerNow) + volumeTotal;//int
}//else if(pressure < 3) volumeTotal = 0;
TimerNow = millis();
//Serial.println(volumeTotal);
  //gadgetBle.writeCO2(volumeTotal);
  //gadgetBle.writeTemperature(0);
  //gadgetBle.writeHumidity(0);

  //gadgetBle.commit();
delay(20);
if((millis() - oneMinute) > 60000){
  oneMinute = millis();
  //Serial.print( "one minute volume = ");
  //Serial.println(volumeTotal);
  seeO();
  volumeTotal = 0;
}
}
void seeO(){
  
  // send read data command
  Wire.beginTransmission(SCD_ADDRESS);
  Wire.write(0xec);
  Wire.write(0x05);
  Wire.endTransmission();
  
  // read measurement data: 2 bytes co2, 1 byte CRC,
  // 2 bytes T, 1 byte CRC, 2 bytes RH, 1 byte CRC,
  // 2 bytes sensor status, 1 byte CRC
  // stop reading after 12 bytes (not used)
  // other data like  ASC not included
  Wire.requestFrom(SCD_ADDRESS, 12);
  counter = 0;
  while (Wire.available()) {
    data[counter++] = Wire.read();
  }
  
  // floating point conversion according to datasheet
  co2 = (float)((uint16_t)data[0] << 8 | data[1]);
  // convert T in degC
  temperature = -45 + 175 * (float)((uint16_t)data[3] << 8 | data[4]) / 65536;
  // convert RH in %
  humidity = 100 * (float)((uint16_t)data[6] << 8 | data[7]) / 65536;
  Serial.print("Real CO2=");
  Serial.println(co2);
  //Serial.print("\t");
  //Serial.print(temperature);
  //Serial.print("\t");
  //Serial.print(humidity);
  //Serial.println();
  float oxygenData = Oxygen.ReadOxygenData(COLLECT_NUMBER);
  Serial.print(" Oxygen concentration is ");
  Serial.print(oxygenData);
  Serial.println(" %vol");
  delay(1000);
  float projectedO2 = (20.5 - (co2/10000));
  //Serial.print( "projected o2=");
  //Serial.println(projectedO2);
  float projectedCO2 = 20.5 - oxygenData;
  //Serial.print( "projected Co2 = ");
  //Serial.println( projectedCO2 );
 gadgetBle.writeCO2(co2);
 gadgetBle.writeTemperature(oxygenData);
 gadgetBle.writeHumidity(0);

  gadgetBle.commit();
timerThen = millis();

  
}
