/*
  Interrup Based Encoder LED Blinking
 
 Turns on and off a light emitting diode(LED) connected to digital  
 pin 13, when an encoder sends a rising edge to digital input pin 2.
 
 
 The circuit:
 * LED attached from pin 13 to ground 
 * encoder attached to pin 2
 * 10K resistor attached to pin 2 from 5v+
 
 * Note: on most Arduinos there is already an LED on the board
 attached to pin 13.
 
 
 created 2015
 Jordan Berke
 */
// constants won't change. They're used here to 
// set pin numbers:
#include <SPI.h>
#include <Wire.h>
#include <Arduino.h>
#include <MAX1704.h>
//#include <LiquidCrystal_I2C.h>  //Nate comments
#include <Adafruit_GFX.h>         //Nate addition
#include <Adafruit_SSD1306ms.h>     //Nate addition
#include <RFduinoBLE.h>  // Elliot addition

#define OLED_RESET 9
Adafruit_SSD1306 display(OLED_RESET);

#define NUMFLAKES 10
#define XPOS 0
#define YPOS 1
#define DELTAY 2
#define LOGO16_GLCD_HEIGHT 16 
#define LOGO16_GLCD_WIDTH  16 
#define UNIT63 2689
#define UNIT93 2680
#define UNIT56 2641

float CODE_VERSION = 1.02;

boolean bluetoothOn = false;  // Elliot addition
boolean bluetoothStartNextLoop = false; // Elliot addition
float repPerformance[] = {0.0, 1.0,2.0,3.0,4.0,5.0};  // Elliot addition
int repPerformanceI[] = {0,1,2,3,4,5};  // Elliot addition
long loopCount = 0; // Elliot addition
int looploopCount = 0; // Elliot addition

static const unsigned char PROGMEM logo16_glcd_bmp[] =
{ B00000000, B11000000,
  B00000001, B11000000,
  B00000001, B11000000,
  B00000011, B11100000,
  B11110011, B11100000,
  B11111110, B11111000,
  B01111110, B11111111,
  B00110011, B10011111,
  B00011111, B11111100,
  B00001101, B01110000,
  B00011011, B10100000,
  B00111111, B11100000,
  B00111111, B11110000,
  B01111100, B11110000,
  B01110000, B01110000,
  B00000000, B00110000 };
const int pin_buttonRight =  0;      // the number of the LED pin
const int pin_buttonLeft = 1;
const int pin_led =  2;      // the number of the LED pin
const int pin_encoder_dir = 3;
const int pin_encoder_tach = 4;

const unsigned long batteryCheckTime = 10000000;     //The amount of time between battery checks. 10 sec
const long ticLength = UNIT63; //in micrometers
volatile int state = LOW;
volatile int goingUpward = HIGH;
int isGoingUpwardLast = 0;
int currentStateTemp = 0;
long startheight = 0;
long sumVelocities = 0;
long avgVelocity = 0;
unsigned long starttime = 0;
int tics = 0;
int rep = 0;
int repDone = 0;
int repDoneLast = 0;
int repDisplay = 0;
int repDisplayLast = 0;
long displacement = 0;
long total_displacement = 0;
long lastDisplacement = 0;
unsigned long tic_time = 0;
unsigned long tic_timestamp = 0;
unsigned long tic_timestampLast = 0;
unsigned long tic_timestampLast2 = 0;
unsigned long tic_timestamp_last = 0;
unsigned long minDT = 1000000;
unsigned long total_time = 0;
unsigned long tic_time2 = 0;
unsigned long displayTime = 0;
unsigned long batteryTime = 0;
const unsigned long backlightTime = 10000000;
int i = 0;
int myVelocities[500] = {0};
int initialized = 0;
const int repArrayCount=100;
float repArray[repArrayCount] = {0};
int buttonStateRight = 0; // variable for reading the pushbutton status
int buttonStateLeft = 0; // variable for reading the pushbutton status
int buttonstateRtemp = 0;
int buttonstateLtemp = 0;
unsigned long rightHold = 0;
unsigned long leftHold = 0;
int rightHoldActionTime = 1500;
int leftHoldActionTime = 1500;
int bothHoldActionTime = 5000;
int replast = 0;
boolean backlightFlag = 1;
boolean RbuttonDepressed = 0;
boolean LbuttonDepressed = 0;
float testVelocity[repArrayCount] = {0};
float peakVelocity[repArrayCount] = {0};
float dispArray[repArrayCount] = {0};
float timeArray[repArrayCount] = {0};
//unsigned int instVelTimestamps[1000] = {0};
// Pin 13: Arduino has an LED connected on pin 13
// Pin 11: Teensy 2.0 has the LED on pin 11
// Pin 6: Teensy++ 2.0 has the LED on pin 6
int buttonstate = 0;        
static unsigned long last_interrupt_time = 0;
static unsigned long last_interrupt_time2 = 0;
static unsigned long last_tic_time = 0;
long instvel = 0;
int flipLED = 0;
int charge = 50;
int battUpdates = 0;
//LiquidCrystal_I2C lcd(0x27,20,4); //Addr: 0x3F, 20 chars & 4 lines    Nate comment
const int threshold_buttonhold=100; //cycles of buttonholdtimer to cross threshold
const int buttonholdtimer=10;  //delay time
int counter_buttonRighthold=0;
int counter_buttonLefthold=0;
bool buttonRightLongPress=0;
bool buttonLeftLongPress=0;
bool bothbuttonlong=0;
  
int counter_simplelengthbytic=0;
int counter_lengthbyticinfunction=0;

int encoderState(uint32_t ulPin)
{
  //unsigned long interrupt_time = micros();
  //max tach pulse width is 20 micros. Double that to be safe. Min is 3. Info starts to deteriorate at less than 12 micros
  //We could just use "RISING" for interrupt handler, since a tach pulse happens every time the encoder state changes.
  //if (interrupt_time - last_interrupt_time >40)
  //{
  state = !state;
  counter_simplelengthbytic++;
    //Serial.println(state);
    //}
  //last_interrupt_time = interrupt_time;
  return 0;
}

MAX1704 fuelGauge;






// ********** Primary setup. Only to be run once on startup. ********** \\

void setup() {  
  Wire.begin();
  delay(200); //display needs about 100ms to initialize the IC
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // initialize with the I2C addr 0x3C (for the 128x64)  //Nate addition

  pinMode(pin_led, OUTPUT);
  pinMode(pin_buttonRight, INPUT_PULLDOWN); 
  pinMode(pin_buttonLeft, INPUT_PULLDOWN); 
  pinMode(pin_encoder_tach, INPUT); 
  pinMode(pin_encoder_dir, INPUT); 

  digitalWrite(pin_led,HIGH);

  RFduino_pinWakeCallback(pin_encoder_tach, HIGH, encoderState);

  /* Elliots test messages for Bluetooth
    Serial.begin(9600);
    Serial.println("starting my task ...");
    randomSeed(analogRead(0));  // can be removed at any time since its only for generating random test message
  */
  
  fuelGauge.reset();
  fuelGauge.quickStart();
  fuelGauge.showConfig();

  //Welcome Screen
  display.clearDisplay();
  display.setTextSize(3);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.println("Open");
  display.setCursor(0,25);
  display.println("Barbell");
  display.setTextSize(1);
  display.setCursor(0,50);
  display.print("Rev: ");
  display.print(CODE_VERSION);
  display.display();
  

  delay(1500);

  charge = fuelGauge.stateOfCharge();
  if(charge>100){
    charge=100;
  } else if (charge<=0){
    charge=1;
  }
  
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0,15);
  display.println("Begin Set!");
  
  checkBatteryandDisplay();
  display.display();
  initializeBluetooth();
  RFduinoBLE.deviceName = "OpenBarbell";
  RFduinoBLE.advertisementData = "OpenBarbell";
}

// ******************************************************************** \\





// ********** Primary loop. This loop should only run function calls or poll pins. ********** \\

void loop() {
  
  //Elliot addition
  //initializeBluetooth();
  
  goingUpward = !digitalRead(pin_encoder_dir);

  buttonStateLeft = digitalRead(pin_buttonLeft);
  buttonStateRight = digitalRead(pin_buttonRight);

  calcRep(goingUpward, state);
  
  buttonStateCalc(buttonStateRight, buttonStateLeft);

}

// **************************************************************************************************************** \\





// ********** Function to initialize Bluetooth if given the command by user ********** \\

void initializeBluetooth(){
  //if (!bluetoothOn && bluetoothStartNextLoop) {
    bluetoothOn = true;
    RFduinoBLE.begin();
    RFduino_ULPDelay(0);
  //}
}

// *********************************************************************************** \\






// ********** Function that turns off Bluetooth if requested by phone ********** \\

void RFduinoBLE_onReceive(char *data, int len)
{
// if the first byte is 0x01 / on / true
  if (data[0]){
    Serial.println("new message received: " + charToString(data, len));  // buffer reused, so need to check length
    String msg3 = charToString(data, min(len,13));
    if (msg3.equalsIgnoreCase("bluetooth off")) {
      bluetoothOn = false;
      bluetoothStartNextLoop= false;
      Serial.println("bluetooth terminated");
      RFduinoBLE.end();
    }
  }
}

String charToString(char *text, int len)
{
 String s = "";
 for (int i =0; i < len; i++) s += text[i];
 return s;
}

// **************************************************************************** \\





// ********** Function to send message over Bluetooth ********** \\

void send_intList_charString(int *intList, int len) {
  String intString = "";
  for (int i=0; i < len; i++) {
    if (i > 0) intString += ",";
    intString += String(intList[i]);
  }
  int nbytes = intString.length()+1;
  char bytes[nbytes];
  intString.toCharArray(bytes, nbytes);
  RFduinoBLE.send(bytes, nbytes);
  RFduino_ULPDelay(5);  
} // END send_intList_charString

void send_intList(int *intList, int len) {
  for (int i=0; i < len; i++) {
    RFduinoBLE.sendInt(intList[i]);
    RFduino_ULPDelay(1);
  }
} // END send_intList

void send_floatList(float *floatList, int len) {
  for (int i=0; i < len; i++) {
    RFduinoBLE.sendFloat(floatList[i]);
    RFduino_ULPDelay(1); 
  }
} // END send_floatList

// ************************************************************************** \\






// ********** Function to check battery charge and add it to the screen. Will be displayed to the screen outside of this function ********** \\

void checkBatteryandDisplay(){
  //Only updates the battery percentage when requested by other functions. Does not run on a loop.
    charge = fuelGauge.stateOfCharge();
  if(charge>100){
    charge=100;
  } else if (charge<=0){
    charge=1;
  }
  
  display.setTextSize(1);
  display.setCursor(104,0);
  display.print("    ");
  display.setCursor(104,0);
  display.print(charge);
  display.print("%");
}

// ***************************************************************************************************************************************** \\






// ********** Function that deals with incoming tics. It puts them in different categories based on certain criteria. ********** \\

void calcRep(int isGoingUpward, int currentState){
  if (currentState != currentStateTemp) { //First thing we do is make sure that you are not accessing the function unless the state has changed since the last iteration
    long denom = 0; 
    //Since you just found a rising edge, take down the time
    tic_time = micros();
    //increment or decrement the distance by one tic length, depending on direction
    if (isGoingUpward){
      displacement += ticLength;
      counter_lengthbyticinfunction++;
      tic_timestampLast2 = tic_timestampLast;
      tic_timestampLast = tic_timestamp;
      tic_timestamp = micros();
      //keeping instantaneous velocities for our peak velocity reading
      //instVelTimestamps[counter_lengthbyticinfunction] = (unsigned int)(tic_timestamp-tic_timestamp_last);
      if((tic_timestamp - tic_timestamp_last) < minDT){
        minDT = tic_timestamp - tic_timestamp_last;
      }
      tic_timestamp_last = tic_timestamp;
      // If you're going upward but you were just going downward, clear your array so you can start a fresh rep
      if (!isGoingUpwardLast){
        memset(myVelocities,0,sizeof(myVelocities));
        //memset(instVelTimestamps,0,sizeof(instVelTimestamps));
        startheight = displacement;
        counter_lengthbyticinfunction=0;
        counter_simplelengthbytic=0;
        starttime = tic_timestamp;
        rep += 1;
        sumVelocities = 0;
        lastDisplacement = startheight;
        tic_time2 = 0;
        minDT = 1000000;
        i = 0;
      }
      // This records a value every 20ms
      if (tic_time - tic_time2 > 20000){
        denom = (long)(tic_time - tic_time2);
        //displacement is in micrometers, denom is in microseconds, so instvel is in m/s.
        instvel = ((displacement - lastDisplacement)*1000)/denom;
        myVelocities[i] = (int)instvel;
        tic_time2 = tic_time;
        lastDisplacement = displacement;
        i += 1;
      } 
    } else {
      // If you're going downward, and you were just going upward, you potentially just finished a rep. 
      // Do your math, check if it fits the rep criteria, and store it in an array.
      if (isGoingUpwardLast && rep<=repArrayCount){
        if ((displacement - startheight) > 150000){ //JDL TEST - REMOVED 0
          
          for (int count = 0; count <= i; count++){
            sumVelocities = sumVelocities + (long)myVelocities[count];
          }
          
          avgVelocity = sumVelocities/(long)i; 
          total_displacement = displacement - startheight;
          total_time = (tic_timestampLast - starttime) + .5*(tic_timestampLast - tic_timestampLast2);
          dispArray[rep] = (float)total_displacement/1000;
          timeArray[rep] = (float)total_time/1000000;
          testVelocity[rep] = (float)((total_displacement)*1000/((long)(total_time)))/1000;
          peakVelocity[rep] = (float)(ticLength*1000/((long)(minDT)))/1000;
          repArray[rep] = (float)avgVelocity/1000;
          repDone = rep;
          
          //bluetooth broadcast Elliot addition
          if (bluetoothOn) {

            repPerformance[0] = (float) rep;
            repPerformance[1] = (float) i;
            repPerformance[2] = (float) avgVelocity; 
            repPerformance[3] = (float) repArray[rep]; 
            repPerformance[4] = (float) total_displacement; 
            repPerformance[5] = (float) peakVelocity[rep]; 
            send_floatList(repPerformance, 6); 
 /*          
            repPerformanceI[0] = rep;
            repPerformanceI[1] = i;
            repPerformanceI[2] = (int) avgVelocity; 
            repPerformanceI[3] = (int) total_time / 1000; 
            repPerformanceI[4] = (int) total_displacement / 100; // overflows sometimes, so sign bit turned on
            repPerformanceI[5] = (int) (peakVelocity[rep] * 1000); 
            send_intList(repPerformanceI, 6); 
*/
//            send_intList_charString(repPerformanceI, 6);
          }
          
        } else { 
          rep -= 1;
        }
      }
      displacement -= ticLength;
    }
      
    isGoingUpwardLast = isGoingUpward;
    //Serial.println(instvel);
    currentStateTemp = currentState;
  }
}

// ***************************************************************************************************************************** \\






// ********** Function that deals with and implements button presses. ********** \\

void buttonStateCalc(int buttonstateR, int buttonstateL){
  
  // since hooking up to battery power, we want to turn off backlight after a certain time.
  // if the displayed rep hasn't changed in a while, we don't need the backlight
  if ((micros() - displayTime) > backlightTime){
    display.ssd1306_command(SSD1306_DISPLAYOFF);
    backlightFlag = 0; // might not be necessary anymore -- Nate
  }
  
  //update rep to be displayed to the screen if you recorded a new rep
  if (repDone != repDoneLast){
    repDisplay = repDone; 
    repDoneLast = repDone;
    display.ssd1306_command(SSD1306_DISPLAYON);
  }
  
  //register a button press on the release of the right button
  if (buttonstateRtemp && !buttonstateR){
    if ((backlightFlag)&&(repDisplay < (repDone + 2))){
      repDisplay += 1;
    }else {
      display.ssd1306_command(SSD1306_DISPLAYON); 
    backlightFlag = 1;
    rightHold = 0;
    RbuttonDepressed = 0;
    displayTime = micros();
    bluetoothStartNextLoop = true;
    }
  }
  //register a button press on the release of the left button 
  if (buttonstateLtemp && !buttonstateL){
    if ((backlightFlag)&&(repDisplay > 1)){
      repDisplay -= 1;
    } else {
      display.ssd1306_command(SSD1306_DISPLAYON);  
      backlightFlag = 1;
      leftHold = 0;
      LbuttonDepressed = 0;
      displayTime = micros();
    }
  }
  
  //set a flag if you just pressed the right button, and look at when that happened
  if (!buttonstateRtemp && buttonstateR){
    rightHold = millis();
    RbuttonDepressed = 1;
  }

  //set a flag if you just pressed the left button, and look at when that happened  
  if (!buttonstateLtemp && buttonstateL){
    leftHold = millis();
    LbuttonDepressed = 1;
  }
  
  //if both buttons are depressed, enable bluetooth if over 5 seconds
  if (LbuttonDepressed && RbuttonDepressed){
    if (((millis() - rightHold) > bothHoldActionTime)&&((millis() - leftHold) > bothHoldActionTime)){
      if (!bluetoothOn && !bluetoothStartNextLoop){
        bluetoothStartNextLoop = true;
        //rightHold = 
        display.setTextSize(1);
        display.setCursor(64,0);
        display.print("BT ON");
        display.display();
      }
    }
  }
  /* Button and string combination code. Not finished - does not work. Might need to separate Rbutton and Lbutton, as millis() - XHold for the non-depressed button will almost always be greater than the action times
  if (RbuttonDepressed ^ LbuttonDepressed){
    if (((millis() - rightHold) > rightHoldActionTime)||((millis() - leftHold) > leftHoldActionTime)){
      if ((repDisplay < (repDone + 1))&&(repDisplay > 0)){
        repDisplay += (int)instvel;
      }
    }
  }
  */
  if (repDisplay != repDisplayLast){
  
    // if the displayed rep changes, keep the time so we know when to dim the backlight
    displayTime = micros();
    // make sure we can see the new rep
    display.ssd1306_command(SSD1306_DISPLAYON);
    backlightFlag = 1;
    
    if (repDisplay == (repDone + 1)){
    
      display.clearDisplay(); //nate add
      display.setTextSize(2);
      display.setTextColor(WHITE);
      display.setCursor(0,0);
      display.println("Delete");
      display.println("Past Set?");
      display.setTextSize(1);
      display.setCursor(0,40);
      display.println("R Button-Delete Set");
      display.println("L Button-Go Back");
      checkBatteryandDisplay();
      display.display();         //end nate add
    } else if (repDisplay > (repDone + 1)){
      display.clearDisplay(); //nate add
      display.setTextSize(2);
      display.setTextColor(WHITE);
      display.setCursor(0,15);
      display.print("Begin Set!");
      checkBatteryandDisplay();
      display.display();         //end nate add
      startheight = displacement;
      counter_simplelengthbytic=0; //JDLTEST
      counter_lengthbyticinfunction=0;  //JDLTest
      
      rep = 1;
      repDone = 0;
      repDoneLast = 0;
      sumVelocities = 0;
      memset(repArray,0,sizeof(repArray));
      memset(testVelocity,0,sizeof(testVelocity));
      memset(myVelocities,0,sizeof(myVelocities));
      memset(dispArray,0,sizeof(dispArray));
      memset(timeArray,0,sizeof(timeArray));
      //memset(instVelTimestamps,0,sizeof(instVelTimestamps));
      i = 0;
    } else {
      display.clearDisplay(); //nate add
      display.setTextSize(1);
      display.setTextColor(WHITE);
      display.setCursor(0,9);
      display.print("Avg Vel");
      display.setTextSize(2);
      display.setTextColor(WHITE);
      display.setCursor(0,19);
      display.print(repArray[repDisplay]);
      display.setTextSize(1);
      display.print("m/s");
      display.setCursor(0,36);
      display.print("Peak Vel");
      display.setCursor(0,45);
      display.setTextSize(2);
      display.print(peakVelocity[repDisplay]);
      display.setTextSize(1);
      display.print("m/s");
      display.setCursor(80,9);
      display.print(dispArray[repDisplay]);
      display.setCursor(80,19);
      display.print(timeArray[repDisplay]);
      display.setCursor(80,29);
      display.print(testVelocity[repDisplay]);
      //display.setTextSize(2);
      /*  display.setCursor(80,6);  //JDLTEST
        display.print(counter_simplelengthbytic); //JDLTEST
        display.print(counter_lengthbyticinfunction);
        counter_simplelengthbytic=0; //JDLTEST
        counter_lengthbyticinfunction=0;
        */
      display.setTextSize(1);
      display.setCursor(0,0);
      display.print("Rep#:");
      display.print(repDisplay);
      //display.setTextSize(2);
      //display.setTextColor(WHITE);
      //display.setCursor(0,40);
      //display.print("-");
      //display.print(total_displacement/1000);
      //display.print("-");
      //display.print(((long)total_time)/1000);
        
      checkBatteryandDisplay();
      display.display();         //end nate add
    }
    
    repDisplayLast = repDisplay; 
  }
  
  buttonstateRtemp = buttonstateR;
  buttonstateLtemp = buttonstateL;
  
}

// ***************************************************************************** \\
