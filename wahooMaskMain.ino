#include "Sensirion_GadgetBle_Lib.h"
#include "DFRobot_OxygenSensor.h"
#include <Omron_D6FPH.h>
#include <Wire.h>
#include <TFT_eSPI.h> // Graphics and font library for ST7735 driver chip
#include <SPI.h>
TFT_eSPI tft = TFT_eSPI();  // Invoke library, pins defined in User_Setup.h
const int  buttonPin1 = 0; 
const int  buttonPin2 = 35; 
#define COLLECT_NUMBER    10             // collect number, the collection range is 1-100.
Omron_D6FPH mySensor;
int wtTotal = 0;
int buttonPushCounter1 = 0;   // counter for the number of button presses
int buttonState1 = 1;         // current state of the button
int lastButtonState1 = 0;  
int buttonPushCounter2 = 0;   // counter for the number of button presses
int buttonState2 = 1;         // current state of the button
int lastButtonState2 = 0;  
   
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
float correction = 0;
const int numReadings = 5;
float oReadings[numReadings];
float minuteTotal;
float lastOtwo = 0;
int oreadIndex = 0;
float oTotal = 0;
float oAverage = 0;
float vo2Max= 0;
 float vo2MaxMax= 0;
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
   tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.fillScreen(TFT_RED);
  pinMode(buttonPin1, INPUT_PULLUP);
  pinMode(buttonPin2, INPUT_PULLUP);
  tft.setTextColor(TFT_GREEN, TFT_RED);
  tft.drawCentreString("Enter Weight in LBS",120,12,4);
  tft.setCursor(180, 115, 4);
  tft.setTextColor(TFT_GREEN, TFT_RED);
  tft.println("UP");
   for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    oReadings[thisReading] = 0;
  }
  while(buttonPushCounter2 < 3)wtRead();
  tft.fillScreen(TFT_RED);
  tft.setTextColor(TFT_GREEN, TFT_RED);   
  tft.drawCentreString("VO2  MAX",120,60,4);
  tft.setCursor(160, 115, 4);
  tft.setTextColor(TFT_GREEN, TFT_RED);
  tft.println("GO!");
  minuteTotal = millis();
  correction = 12.50/19.00;
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
//if(volFlow > maxFlow) maxFlow = volFlow;

volumeTotal = volFlow * (millis() - TimerNow) + volumeTotal;//int
}else if((millis() - timerThen) > 5000) seeO();
TimerNow = millis();
//Serial.println(volumeTotal);
  //gadgetBle.writeCO2(volumeTotal);
  //gadgetBle.writeTemperature(0);
  //gadgetBle.writeHumidity(0);
  
  float corrvolumeTotal = volumeTotal * correction; 
  //Serial.print(correction);
Serial.print(volumeTotal);
Serial.print("     ");
Serial.println(corrvolumeTotal);
  //gadgetBle.commit();
delay(20);

if((millis() - oneMinute) > 30000){
  oneMinute = millis();
  Serial.print( "one minute volume = ");
  Serial.print(volumeTotal);
  Serial.print("    ");
  Serial.println(corrvolumeTotal);
  //seeO();
  volumeTotal = 0;
}
}
void seeO(){
 
  float oxygenData = Oxygen.ReadOxygenData(COLLECT_NUMBER);
  Serial.print(" Oxygen concentration is ");
  Serial.print(oxygenData);
  Serial.println(" %vol");
  delay(1000);
  
  //float projectedCO2 = 21.8 - oxygenData;
  //Serial.print( "projected Co2 = ");
  //Serial.println( projectedCO2 );
 //gadgetBle.writeCO2(co2);
 //gadgetBle.writeTemperature(oxygenData);
 //gadgetBle.writeHumidity(volumeTotal/1000);

 //gadgetBle.commit();
  oTotal = oTotal - oReadings[oreadIndex];
  // read from the sensor:
  oReadings[oreadIndex] = oxygenData;
  // add the reading to the total:
  oTotal = oTotal + oReadings[oreadIndex];
  // advance to the next position in the array:
  oreadIndex = oreadIndex + 1;

  // if we're at the end of the array...
  if (oreadIndex >= numReadings) {
    // ...wrap around to the beginning:
    oreadIndex = 0;
  }

  // calculate the average:
  oAverage = oTotal / numReadings;
  // send it to the computer as ASCII digits
  
  lastOtwo = oAverage;
  timerThen = millis();

  
}
void goFigure(){
  float percentN2exp;
  float co2 = 21.8 - lastOtwo;
  float volumeMinute = volumeTotal * 2.0;
  volumeMinute = volumeMinute/1000.0; //gives liters of air VE
  //Serial.print("liters/min uncorrected");
  //Serial.print(volumeMinute);
  //co2 = lastCotwo/10000.0;
  //lastOtwo = 17.5;
  //Serial.print("CO2  ");
  //Serial.print(co2);
  //Serial.print("02  ");
  //Serial.println(lastOtwo);
  percentN2exp = (100.0 - (co2 + lastOtwo));
  //Serial.print("%N  ");    
  //Serial.println(percentN2exp);
  //volumeMinute = volumeMinute * (273/(273 + lastTemp)) * ((760.0 - 25.2)/760);
  //Serial.print("liters/min corrected  ");
  //Serial.print(volumeMinute);
  volumeMinute = volumeMinute * 0.852;
  vo2Max = volumeMinute * (((percentN2exp/100.0) * 0.265) - (lastOtwo/100.0));
  //Serial.print(lastOtwo/100.0);
  //Serial.print("This: ");
  //Serial.print((percentN2exp/100.0 * 0.265) - (lastOtwo/100.0));
  //Serial.print("VO2Max!  ");
  //Serial.print(vo2Max);
  vo2Max = (vo2Max * 1000.0)/(float(wtTotal)/2.2);
  //Serial.print("VO2Max!!!ml/kg:  ");
  //Serial.print(vo2Max);
  tft.fillScreen(TFT_RED);
  tft.setTextColor(TFT_GREEN, TFT_RED);
  tft.drawCentreString("VO2Max= ",60,10,4);
  tft.setTextColor(TFT_RED, TFT_RED);
  tft.drawString("888888",70,40,7);
  tft.setTextColor(TFT_WHITE, TFT_RED); // Orange
  //tft.drawNumber(vo2Max,100,40,7);
  tft.setCursor(70, 40, 7);
  
  if(vo2Max > vo2MaxMax) vo2MaxMax = vo2Max;
   tft.println(vo2MaxMax);
   tft.setCursor(160, 115, 4);
   tft.setTextColor(TFT_GREEN, TFT_RED);
   tft.println("RESET"); 
   
   
  
  
  
}
void wtRead(){
  buttonState1 = digitalRead(buttonPin1);
  if (buttonState1 != lastButtonState1) {
    // if the state has changed, increment the counter
    if (buttonState1 == LOW) {
      // if the current state is HIGH then the button went from off to on:
      buttonPushCounter1++;
      //Serial.println("on");
      //Serial.print("number of button pushes: ");
      //Serial.println(buttonPushCounter1);
      if(buttonPushCounter1 == 10)buttonPushCounter1 = 0;
    } else {
      // if the current state is LOW then the button went from on to off:
     // Serial.println("off");
    }
    // Delay a little bit to avoid bouncing
    delay(50);
  }
  lastButtonState1 = buttonState1;
  buttonState2 = digitalRead(buttonPin2);
  if (buttonState2 != lastButtonState2) {
    // if the state has changed, increment the counter
    if (buttonState2 == LOW) {
      // if the current state is HIGH then the button went from off to on:
      buttonPushCounter2++;
      wtTotal = wtTotal + 1000/pow(10, buttonPushCounter2) * buttonPushCounter1;
      buttonPushCounter1  = 0;
      
      if(buttonPushCounter2 == 3){
        //Serial.print("total wt = ");
        //Serial.print(wtTotal);
      }
    } else {
      // if the current state is LOW then the button went from on to off:
     // Serial.println("off");
    }
    // Delay a little bit to avoid bouncing
    delay(50);
  }
  lastButtonState2 = buttonState2;
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  if(buttonPushCounter2 < 3){
  int counter = 30 + (40 * buttonPushCounter2);
  tft.drawNumber(buttonPushCounter1,counter,40,7);
  }
}
