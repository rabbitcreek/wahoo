#include "Sensirion_GadgetBle_Lib.h"  //This is library to connect to Sensirion App
#include "DFRobot_OxygenSensor.h"    //Library for Oxygen sensor
#include <Omron_D6FPH.h>      //Library for pressure sensor
#include <Wire.h>
#include <TFT_eSPI.h> // Graphics and font library for ST7735 driver chip
#include <SPI.h>
//These files are for connecting to Apple TV or Zwift Bluetooth

byte flags = 0b00111110;
byte bpm;
byte heart[8] = { 0b00001110, 60, 0, 0, 0 , 0, 0, 0};
byte hrmPos[1] = {2};

//This defines the Bluetooth Heartrate Service to Advertize to Zwift


//Starts Screen for TTGO device
TFT_eSPI tft = TFT_eSPI();  // Invoke library, pins defined in User_Setup.h
const int  buttonPin1 = 0; 
const int  buttonPin2 = 35; 
#define COLLECT_NUMBER    10             // collect number, the collection range is 1-100.
//Labels the pressure sensor: mySensor
Omron_D6FPH mySensor;
//Defines button state for adding wt
int wtTotal = 0;
int buttonPushCounter1 = 0;   // counter for the number of button presses
int buttonState1 = 1;         // current state of the button
int lastButtonState1 = 0;  
int buttonPushCounter2 = 0;   // counter for the number of button presses
int buttonState2 = 1;         // current state of the button
int lastButtonState2 = 0;  
//Defines the size of the Venturi openings for the  calculations of AirFlow   
float area_1 = 0.000531; //these are the areas taken carefully from the 3D printed venturi 2M before constriction
float area_2 = 0.000201;// this is area within the venturi
float rho = 1.225; //Demsity of air in kg/m3;
float massFlow = 0;
float volFlow = 0;
float volumeTotal = 0; //variable for holding total volume of breath
float corrvolumeTotal = 0.0;  // correcion of volumeTotal based on actual measurement(currently set equal)
float TimerNow = 0.0;
float timerThen = 0.0;
float oneMinute = 0.0;
float correction = 0;
const int numReadings = 3; //number of rolling average readings of O2 
float oReadings[numReadings];
float minuteTotal;
float lastOtwo = 0;
int oreadIndex = 0;
float oTotal = 0;
float oAverage = 0;
float vo2Max= 0; //value of vo2Max every 30 seconds
 float vo2MaxMax= 0;  //Best value of vo2 max for whole time machine is on
DFRobot_OxygenSensor Oxygen;  //Label of oxygen sensor
#define COLLECT_NUMBER    10             // collect number, the collection range is 1-100.
#define Oxygen_IICAddress ADDRESS_3  //I2C  label for o2 address
GadgetBle gadgetBle = GadgetBle(GadgetBle::DataType::T_RH_CO2); //uncomment to activate Sensirion App
const int16_t SCD_ADDRESS = 0x62;

  
uint8_t data[12], counter;

void setup()
{
  
Wire.begin();
Serial.begin(115200);
while(!Serial);

  bpm = 1;
gadgetBle.begin();  //uncomment to activate Sensirion App
 delay(1000);
  Serial.print("Sensirion GadgetBle Lib initialized with deviceId = ");
  //Serial.println(gadgetBle.getDeviceIdString()); //uncomment for Sensirion App
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
  
  //correction = 12.50/19.00;
  correction = 1.0;
  while(digitalRead(buttonPin1));
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.drawCentreString("TotalVol",50,10,4);
 
}
void loop()
{
   

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
volumeTotal = volFlow * (millis() - TimerNow) + volumeTotal;//int
}else if((millis() - timerThen) > 5000) seeO(); //If pressure is <1 and timer is over 5 seconds check oxygen level
TimerNow = millis(); ///This is part of the integral function to keep calculation volume over time..resets amount of time between calcs
 
corrvolumeTotal = volumeTotal * correction; //This is where a correcion factor changes volume to corrvolumeTotal
  //Serial.print(correction);
Serial.print(volumeTotal);
Serial.print("     ");
Serial.println(corrvolumeTotal);
delay(20);

if((millis() - oneMinute) > 30000){ // This calls goFigue calculation to figure Vo2Max every 30 seconds
  
  oneMinute = millis();  //resets the timer
  Serial.print( "one minute volume = ");
  Serial.print(volumeTotal);
  Serial.print("    ");
  Serial.println(corrvolumeTotal);
  goFigure();  //vo2 max function call
  volumeTotal = 0;  //resets 30 breath volume to 0
}

  
}
//This function is called every 5 seconds to save oxygen data and publish VO2 Max to TV
void seeO(){
 
  float oxygenData = Oxygen.ReadOxygenData(COLLECT_NUMBER);
  Serial.print(" Oxygen concentration is ");
  Serial.print(oxygenData);
  Serial.println(" %vol");
  delay(1000);
 
//This is the averaging array--rolling with three input (average O2 over 15 seconds)
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
  //Publishes vo2Max to TV bluetooth
  timerThen = millis();
//Pushes the current volume to the screen
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK); // Orange
  tft.setCursor(100, 40, 7);
  int puff = corrvolumeTotal; 
  tft.println(puff); 

  
}
void goFigure(){
  float percentN2exp;
  float co2 = 20.9 - lastOtwo;  //this is the calculated level of Co2 based on Oxygen level loss
  float volumeMinute = corrvolumeTotal * 2.0;  //doubles the air volume to represent 1 minute of breathing
  volumeMinute = volumeMinute/1000.0; //gives liters of air VE
  //Serial.print("liters/min uncorrected");
  //Serial.print(volumeMinute);
  //co2 = lastCotwo/10000.0;
  //lastOtwo = 17.5;
  //Serial.print("CO2  ");
  //Serial.print(co2);
  //Serial.print("02  ");
  //Serial.println(lastOtwo);
  percentN2exp = (100.0 - (co2 + lastOtwo));  //calculation expresses vol of expired nitrogen
  volumeMinute = volumeMinute * 0.852;  //Calculation for dry air at room temp
  vo2Max = volumeMinute * (((percentN2exp/100.0) * 0.265) - (lastOtwo/100.0)); //Vo2 max calculation
  //Serial.print("VO2Max!  ");
  //Serial.print(vo2Max);
  vo2Max = (vo2Max * 1000.0)/(float(wtTotal)/2.2); //correcion for wt and changed to CC Final vo2 max 
  //This will broadcast the data to Sensirion App:  Uncomment all to activate!
  gadgetBle.writeCO2(vo2Max);
  gadgetBle.writeTemperature(lastOtwo);
  gadgetBle.writeHumidity(volumeMinute);
  gadgetBle.commit();
  if(vo2Max > vo2MaxMax) vo2MaxMax = vo2Max;
   
  //Uncomment all to broadcast datat to Sensirion App
   gadgetBle.handleEvents();
   delay(3);
  
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
//Function for initiating bluetooth for TV transmission
