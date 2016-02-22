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
#define UNIT63 2644
#define UNIT93 2680
#define UNIT56 2641

float CODE_VERSION = 1.04;

//START TestBed Section - Do not modify
const bool testbed_readouts = 0;
//END TestBed Section

boolean bluetoothOn = false;  // Elliot addition
boolean bluetoothStartNextLoop = false; // Elliot addition
float repPerformance[] = {0.0, 1.0,2.0,3.0,4.0,5.0};  // Elliot addition

const int pin_buttonRight =  0;
const int pin_buttonLeft = 1;
const int pin_led =  2;      // the number of the LED pin
const int pin_encoder_dir = 3;
const int pin_encoder_tach = 4;

const unsigned long batteryCheckTime = 10000000;     //The amount of time between battery checks. 10 sec
const long ticLength = UNIT63; //in micrometers
volatile int state = LOW;
//state flips on startup, so we need to put the temp value at HIGH so we don't run through the 
//code one time at startup before getting an actual tic reading
volatile int currentStateTemp = HIGH;
volatile bool goingUpward = 0;
volatile bool isGoingUpwardLast = 0;
long startheight = 0;
long sumVelocities = 0;
long avgVelocity = 0;
unsigned long starttime = 0;
uint16_t tics = 0;
uint16_t rep = 0;
uint16_t repDone = 0;
uint16_t repDoneLast = 0;
uint16_t repDisplay = 2;
uint16_t repDisplayLast = 0;
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
unsigned long minTimer = 0;
//unsigned long fiveSecTimer = 0; //not being used?
unsigned long minTimer2 = 0;
unsigned long twoSecTimer2 = 0;
unsigned long oneMinute = 60000;
unsigned long twoSec = 2000;
unsigned long ticDiff = 0;
const unsigned long backlightTime = 10000;
int global_counter_i = 0;
uint16_t myDTs[500] = {0};
const int repArrayCount=90;
float repArray[repArrayCount] = {0.0};
uint16_t buttonStateRight = 0; // variable for reading the pushbutton status
uint16_t buttonStateLeft = 0; // variable for reading the pushbutton status
uint16_t buttonstateRtemp = 0;
uint16_t buttonstateLtemp = 0;
unsigned long rightHold = 0;
unsigned long leftHold = 0;
uint16_t rightHoldActionTime = 1500;
uint16_t leftHoldActionTime = 1500;
uint16_t bothHoldActionTime = 3000;
uint16_t singleHoldActionTime = 3000;
uint16_t replast = 0;
boolean backlightFlag = 1;
boolean RbuttonDepressed = 0;
boolean LbuttonDepressed = 0;
float testVelocity[repArrayCount] = {0.0};
float peakVelocity[repArrayCount] = {0.0};
float peakAcceleration[repArrayCount] = {0.0};
uint16_t dispArray[repArrayCount] = {0};
float timeArray[repArrayCount] = {0.0};
float peakAcc = 0.0;
float peakAccFinal = 0.0;
float currentInstVel = 0.0;
float lastInstVel = 0.0;
//unsigned int instVelTimestamps[1000] = {0};
// Pin 13: Arduino has an LED connected on pin 13
// Pin 11: Teensy 2.0 has the LED on pin 11
// Pin 6: Teensy++ 2.0 has the LED on pin 6
int buttonstate = 0;        
static unsigned long last_interrupt_time = 0;
static unsigned long last_interrupt_time2 = 0;
static unsigned long last_tic_time = 0;
long instvel = 0;
uint16_t flipLED = 0;
uint16_t charge = 50;
uint16_t restTime = 0;
uint16_t myDTCounter = 0;
float startMessage[1] = {-1234.0};
//int battUpdates = 0;
//LiquidCrystal_I2C lcd(0x27,20,4); //Addr: 0x3F, 20 chars & 4 lines    Nate comment
const int threshold_buttonhold=100; //cycles of buttonholdtimer to cross threshold
const int buttonholdtimer=10;  //delay time
int counter_buttonRighthold=0;
int counter_buttonLefthold=0;
bool buttonRightLongPress=0;
bool buttonLeftLongPress=0;
bool bothbuttonlong=0;
bool isFlipped = false;
bool LEDInit = true;
bool accomplishedDoubleHold = false;
bool accomplishedSingleHold = false;
bool flipPowerOlyScreen = false; //0 = Power screen, 1 = Oly screen
bool sendData = false;
bool initialized = false;
bool flippedDirection = false;
bool normalDirection = false;
  
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
  RFduinoBLE.deviceName = "OpenBarbell";
  RFduinoBLE.advertisementData = "OBBT";
  initializeBluetooth();
  Wire.begin();
  delay(200); //display needs about 100ms to initialize the IC
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // initialize with the I2C addr 0x3C (for the 128x64)  //Nate addition

  pinMode(pin_led, OUTPUT);
  pinMode(pin_buttonRight, INPUT_PULLDOWN); 
  pinMode(pin_buttonLeft, INPUT_PULLDOWN); 
  pinMode(pin_encoder_tach, INPUT); 
  pinMode(pin_encoder_dir, INPUT); 

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
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(68,5);
  display.println("Open");
  display.setTextSize(1);
  display.setCursor(68,21);
  display.println("Barbell");
  display.setTextSize(1);
  display.setCursor(68,32);
  display.print("Rev:");
  display.print(CODE_VERSION);
  display.display();
  
  RFduino_ULPDelay(SECONDS(2));

  charge = fuelGauge.stateOfCharge();
	if(charge>100){
		charge=100;
	} else if (charge<=0){
		charge=1;
	}
}

// ******************************************************************** \\





// ****************************************************************************************** \\
// ********** Primary loop. This loop should only run function calls or poll pins. ********** \\
// ****************************************************************************************** \\

void loop() {
  directionCalc();
  calcRep(goingUpward, state);
  buttonStateCalc(buttonStateRight, buttonStateLeft);
  minuteTimer();
  displayOffTimer();
  LEDBlink();
  
}

// ****************************************************************************************** \\
// ****************************************************************************************** \\






// ********** Function to calculate direction ********** \\

void directionCalc(){
//define flipped direction (direction when you use your device upside-down)
	flippedDirection = digitalRead(pin_encoder_dir);
//normal direction is usually directly read from the encoder, but the encoder initializes giving us the 'up'
//direction (pin value zero). We need the intial reading to tell us 'down', because that's what it always is
//when the string is retracted into the device. So if we haven't read any tics yet, set normalDirection to zero
	normalDirection = (!initialized)?(0):(!flippedDirection);
	goingUpward = (isFlipped)?(flippedDirection):(normalDirection);
}

// ***************************************************** \\






// ********** Function to update rest timer and battery fuel gauge every minute ********** \\

void minuteTimer(){
  if(((millis()-minTimer)%oneMinute) < 20){
	if((millis()-minTimer2)>30){
	  minTimer2 = millis();
	  restTime++;
	  
	  charge = fuelGauge.stateOfCharge();
	  if(charge>100){
		charge=100;
	  } else if (charge<=0){
		charge=1;
		}
  
	  display.setTextColor(WHITE,BLACK);
	  display.setTextSize(1);
	  display.setCursor(55,0);
	  display.print(restTime);
	  display.print(" min");
	  display.display();
	}
  }
}

// *************************************************************************************** \\






// ********** Function to blink LED every 5 seconds, if there's nothing going on ********** \\

void LEDBlink(){
	//if it's been 5 seconds, enter the statement. If the second timer is true, it won't be during the next loop so the first timer can't trip more than once.
  if(((millis()%twoSec) < 20)&&((!goingUpward)||(LEDInit))){
	if((millis()-twoSecTimer2)>30){
	  twoSecTimer2 = millis();
	  digitalWrite(pin_led,HIGH);
	  RFduino_ULPDelay(20);
	  digitalWrite(pin_led,LOW);
	}
  }
}

// *************************************************************************************** \\






// ********** Function to check if we should shut off the display ********** \\

void displayOffTimer(){
  // since hooking up to battery power, we want to turn off backlight after a certain time.
  // if the displayed rep hasn't changed in a while, we don't need the backlight
  if ((millis() - displayTime) > backlightTime){
    display.ssd1306_command(SSD1306_DISPLAYOFF);
    backlightFlag = 0;
  }
}

// ************************************************************************* \\






// ********** Function to initialize Bluetooth if given the command by user ********** \\

void initializeBluetooth(){
  //if (!bluetoothOn && bluetoothStartNextLoop) {
    bluetoothOn = true;
    RFduinoBLE.begin();
    RFduino_ULPDelay(0);
	
	//initializing bluetooth seems to mess with the millis() function. So we have to initialize timers after BT
	displayTime = millis();
	minTimer = millis();
	minTimer2 = millis();
	//fiveSecTimer = millis(); //not being used?
	twoSecTimer2 = millis();
  //}
}

// *********************************************************************************** \\






// ********** Function that turns off Bluetooth if requested by phone ********** \\

void RFduinoBLE_onReceive(char *data, int len)
{
// if the first byte is 0x01 / on / true
  if (data[0]){
    //Serial.println("new message received: " + charToString(data, len));  // buffer reused, so need to check length
    String msg3 = charToString(data, min(len,13));
    if (msg3.equalsIgnoreCase("bluetooth off")) {
      bluetoothOn = false;
      bluetoothStartNextLoop= false;
      //Serial.println("bluetooth terminated");
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






// ********** Function to send bulk velocity data over Bluetooth ********** \\

void send_float_from_intList(uint16_t *intList, uint16_t len) {
  for (int i=0; i < len; i++) {
    RFduinoBLE.sendFloat((float) intList[i]);
	RFduino_ULPDelay(1);  // appears to be a number > 1 otherwise the end of long arrays are truncated in the transmission. 17ms seems to be good for 600 floats
  }
  
} // END send_float_from_intList

void send_float_from_intListTest(uint16_t *intList, uint16_t len) {
  for(int i=0; i<len; i++){
	while (!RFduinoBLE.sendFloat((float)intList[i]));
  }
} // END send_float_from_intList

void send_single_float(float singleFloat) {
    RFduinoBLE.sendFloat(singleFloat);
    RFduino_ULPDelay(1);
} // END send_single_float


void send_all_data() {
	if (sendData) {//only send data if you just registered a new rep
	  repPerformance[0] = (float) rep;
	  repPerformance[1] = (float) global_counter_i;
	  repPerformance[2] = (float) avgVelocity; 
	  repPerformance[3] = (float) repArray[rep]; 
	  repPerformance[4] = (float) total_displacement; 
	  repPerformance[5] = (float) peakVelocity[rep]; 
	  send_floatList(repPerformance, 6);
	  send_single_float(-9876.0);
	  send_float_from_intListTest(myDTs, (myDTCounter/2));
	  send_single_float(-6789.0);
	  sendData = false;
	  }
} //END send_all_data

// ************************************************************************ \\






// ********** Function to add rep number, battery and rest timer to the screen buffer. Will be displayed to the screen outside of this function ********** \\

void systemTrayDisplay(){

	display.setTextColor(WHITE,BLACK);
	display.setTextSize(1);
	display.setCursor(0,0);
	display.print("Rep#:");
	display.print(repDisplay);
	display.print("  ");
	display.setCursor(55,0);
	display.print(restTime);
	display.print(" min");
	display.setCursor(104,0);
	display.print(charge);
	display.print("%");
}

// ***************************************************************************************************************************************** \\






// ********** Function that deals with incoming tics. It puts them in different categories based on certain criteria. ********** \\

void calcRep(bool isGoingUpward, int currentState){
  if (currentState != currentStateTemp) { //First thing we do is make sure that you are not accessing the function unless the state has changed since the last iteration
    long denom = 0; 
	//we consider initialization after we record our first tic.
	initialized = 1;
    //Since you just found a rising edge, take down the time
    tic_time = micros();
    //increment or decrement the distance by one tic length, depending on direction
    if (isGoingUpward){
      displacement += ticLength;
      counter_lengthbyticinfunction++;
      tic_timestampLast2 = tic_timestampLast;
      tic_timestampLast = tic_timestamp;
      tic_timestamp = micros();

      
      // If you're going upward but you were just going downward, clear your array so you can start a fresh rep
      if (!isGoingUpwardLast){
        memset(myDTs,0,sizeof(myDTs));
        //memset(instVelTimestamps,0,sizeof(instVelTimestamps));
        startheight = displacement;
        counter_lengthbyticinfunction=0;
        counter_simplelengthbytic=0;
        starttime = tic_timestamp;
		send_floatList(startMessage, 1);
        rep += 1;
        sumVelocities = 0;
        lastDisplacement = startheight;
        tic_time2 = micros();
        minDT = 1000000;
        global_counter_i = 0;
		myDTCounter = 0;
      }
	  //keeping instantaneous velocities for our peak velocity reading
      //instVelTimestamps[counter_lengthbyticinfunction] = (unsigned int)(tic_timestamp-tic_timestamp_last);
      ticDiff = tic_timestamp - tic_timestamp_last;
	  if(!(myDTCounter%2)){
		myDTs[(myDTCounter/2)] = (uint16_t)(ticDiff/100);
	  }
	  myDTCounter++;
	  if(ticDiff < minDT){
        minDT = ticDiff;
      }
	  currentInstVel = (float)(ticLength*1000/((long)(minDT)))/1000;
	  peakAcc = (currentInstVel - lastInstVel)/(((float)(ticDiff))/1000000);
	  if(peakAcc > peakAccFinal){
		 peakAccFinal = peakAcc;
	  }
	  lastInstVel = currentInstVel;
      tic_timestamp_last = tic_timestamp;
      // This records a value every 20ms
      if (tic_time - tic_time2 > 20000){
        denom = (long)(tic_time - tic_time2);
        //displacement is in micrometers, denom is in microseconds, so instvel is in m/s.
        instvel = ((displacement - lastDisplacement)*1000)/denom;
        sumVelocities += (long)instvel;
        tic_time2 = tic_time;
        lastDisplacement = displacement;
        global_counter_i += 1;
      } 
    } else {
      // If you're going downward, and you were just going upward, you potentially just finished a rep. 
      // Do your math, check if it fits the rep criteria, and store it in an array.
	  if (isGoingUpwardLast && rep<=repArrayCount){
        if ((displacement - startheight) > 150000){ //JDL TEST - REMOVED 0
          
          avgVelocity = sumVelocities/(long)global_counter_i; 
          total_displacement = displacement - startheight;
          total_time = (tic_timestampLast - starttime) + .5*(tic_timestampLast - tic_timestampLast2);
          dispArray[rep] = total_displacement/1000;
          timeArray[rep] = (float)total_time/1000000;
          testVelocity[rep] = (float)((total_displacement)*1000/((long)(total_time)))/1000;
          peakVelocity[rep] = (float)(ticLength*1000/((long)(minDT)))/1000;
          peakAcceleration[rep] = peakAccFinal;
          repArray[rep] = (float)avgVelocity/1000;
          repDone = rep;
		  //resets 60 second rest time counter
          minTimer = millis();
          minTimer2 = millis();
		  restTime = 0;

        } else { 
          rep -= 1;
        }
      }
	  //Fixes the LED disabled on startup bug
	  LEDInit = false;
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
  
  //Read button state once per loop
  buttonStateLeft = digitalRead(pin_buttonLeft);
  buttonStateRight = digitalRead(pin_buttonRight);
  
  //update rep to be displayed to the screen if you recorded a new rep
  if (repDone != repDoneLast){
    repDisplay = repDone; 
	//repDisplayLast and repDoneLast are reset below
	if(!backlightFlag){
    display.ssd1306_command(SSD1306_DISPLAYON);
	systemTrayDisplay();
	display.display();
	}
	sendData = true;
  }
  
  //register a button press on the release of the right button
  if (buttonstateRtemp && !buttonstateR){
    if ((backlightFlag)&&(repDisplay < (repDone + 2))){
      repDisplay += 1;
    }else {
      display.ssd1306_command(SSD1306_DISPLAYON); 
	  backlightFlag = 1;
	  //systemTrayDisplay();
	  display.display();
	  }
    rightHold = 0;
    RbuttonDepressed = 0;
	//this flag forces the double hold to execute its code for only one loop
	accomplishedDoubleHold = false;
	accomplishedSingleHold = false;
    displayTime = millis();
    //bluetoothStartNextLoop = true;
    
  }
  
  //register a button press on the release of the left button 
  if (buttonstateLtemp && !buttonstateL){
    if ((backlightFlag)&&(repDisplay > 1)&&(repDisplay < repDone + 2)){
      repDisplay -= 1;
    } else {
      display.ssd1306_command(SSD1306_DISPLAYON); 	  
      backlightFlag = 1;
	  //systemTrayDisplay();
	  display.display();
	  }
    leftHold = 0;
    LbuttonDepressed = 0;
	//this flag forces the double hold to execute its code for only one loop
	accomplishedDoubleHold = false;
	accomplishedSingleHold = false;
    displayTime = millis();
    
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
      if (!accomplishedDoubleHold){
        isFlipped = !isFlipped;
		accomplishedDoubleHold = true;
        //rightHold = 
		display.ssd1306_command(SSD1306_DISPLAYON); 
		display.clearDisplay();
		display.invertDisplay(isFlipped);
        display.setTextSize(3);
        display.setCursor(0,0);
		if(isFlipped){
			display.print("INVERT");
			display.setCursor(0,32);
			display.setTextSize(2);
			display.print("MODE ON");
		}else {
			display.print("INVERT");
			display.setCursor(0,32);
			display.setTextSize(2);
			display.print("MODE OFF");
		}
        display.display();
		//display.startscrollleft(0x00, 0x0F);
		RFduino_ULPDelay(SECONDS(2));
		//display.stopscroll();
		repDisplay = repDone;
      }
    }
  } else if (LbuttonDepressed || RbuttonDepressed){
		if (((millis() - rightHold) > singleHoldActionTime)&&((millis() - leftHold) > singleHoldActionTime)){
			if(!accomplishedSingleHold){
			flipPowerOlyScreen = !flipPowerOlyScreen;
			accomplishedSingleHold = true;
			
			display.ssd1306_command(SSD1306_DISPLAYON); 
			display.clearDisplay();
			display.setTextSize(3);
			display.setCursor(0,0);
			if(flipPowerOlyScreen){
				display.print("OLY");
				display.setCursor(0,32);
				display.print("MODE");
			}else{
				display.print("POWER");
				display.setCursor(0,32);
				display.print("MODE");
			}
			display.display();
			//display.startscrollleft(0x00, 0x0F);
			RFduino_ULPDelay(SECONDS(2));
			//display.stopscroll();
			repDisplay = repDone;
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
  if ((repDisplay != repDisplayLast)||(repDone != repDoneLast)){  
    // if the displayed rep changes, keep the time so we know when to dim the backlight
    displayTime = millis();
    // make sure we can see the new rep
    display.ssd1306_command(SSD1306_DISPLAYON);
    backlightFlag = 1;
    
    if (repDisplay == (repDone + 1)){
		//This 'if' statement keeps you from going from "Begin Set" back to "Delete Past Set?"
		if(repDisplayLast < (repDone + 1)){
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
		  display.display();         //end nate add
		}
    } else if (repDisplay > (repDone + 1)){
      startheight = displacement;
      counter_simplelengthbytic=0; //JDLTEST
      counter_lengthbyticinfunction=0;  //JDLTest
	  //This line keeps the repDisplay value from getting too big, and causing a bug to miss the first rep (edit: might not be necessary)
	  //repDisplay = repDone + 2;
      
	  //Yes, you can just say rep = goingUpward. But goingUpward can only be 0 or 1 and rep can be any integer. It just doesn't feel right.
      rep = (goingUpward)?(1):(0);
      repDone = 0;
      repDoneLast = 0;
      sumVelocities = 0;
	  
	  display.clearDisplay(); //nate add
      display.setTextSize(2);
      display.setTextColor(WHITE,BLACK);
      display.setCursor(0,15);
      display.print("Begin Set!");
	  systemTrayDisplay();
	  display.setTextSize(1);
	  display.setCursor(0,0);
	  display.print("Rep#:1 ");
      display.display();         //end nate add
	  RFduino_ULPDelay(1);
	  
      memset(repArray,0,sizeof(repArray));
      memset(testVelocity,0,sizeof(testVelocity));
      memset(myDTs,0,sizeof(myDTs));
      memset(dispArray,0,sizeof(dispArray));
      memset(timeArray,0,sizeof(timeArray));
      memset(peakVelocity,0,sizeof(peakVelocity));
      //memset(instVelTimestamps,0,sizeof(instVelTimestamps));
      global_counter_i = 0;
      myDTCounter = 0;
    } else {
		if(!flipPowerOlyScreen){
		  display.clearDisplay(); //nate add
		  display.setTextSize(1);
		  display.setTextColor(WHITE);
		  display.setCursor(0,9);
		  display.print("Avg Vel:");
		  display.setTextSize(3);
		  display.setTextColor(WHITE);
		  display.setCursor(0,19);
		  display.print(repArray[repDisplay]);
		  display.setTextSize(1);
		  display.print("m/s");
		  display.setCursor(0,42);
		  display.print("Peak Vel:");
		  display.setCursor(0,51);
		  display.print(peakVelocity[repDisplay]);
		  //display.print(myDTs[10]);
		  //display.print(myDTCounter/2);
		  display.print("m/s");
		  display.setCursor(82,42);
		  display.print("ROM:");
		  display.setCursor(82,51);
		  display.print(dispArray[repDisplay]);
		  display.print("mm");

			if(testbed_readouts){
				display.setTextSize(1);
				display.setCursor(100,22);
				display.print(counter_simplelengthbytic);
				display.setCursor(80,32);
				display.print(total_time/1000);
			}

		  systemTrayDisplay();
		  display.display();         //end nate add
		  
          send_all_data(); //moved send_all_data here so it doesn't lag the display
		  
	    } else {
		  display.clearDisplay(); //nate add
		  display.setTextSize(1);
		  display.setTextColor(WHITE);
		  display.setCursor(0,9);
		  display.print("Peak Vel:");
		  display.setTextSize(3);
		  display.setTextColor(WHITE);
		  display.setCursor(0,19);
		  display.print(peakVelocity[repDisplay]);
		  display.setTextSize(1);
		  display.print("m/s");
		  display.setCursor(0,42);
		  display.print("Time:");
		  display.setCursor(0,51);
		  display.print(timeArray[repDisplay]);
		  display.print("sec");
		  display.setCursor(77,42);
		  display.print("Peak Acc:");
		  display.setCursor(77,51);
		  display.print(peakAcceleration[repDisplay]);
		  systemTrayDisplay();
		  display.display();         //end nate add
		  
		  send_all_data(); //moved send_all_data here so it doesn't lag the display
	    }
    }
    repDoneLast = repDone;
    repDisplayLast = repDisplay; 
  }
  
  buttonstateRtemp = buttonstateR;
  buttonstateLtemp = buttonstateL;
  
}

// ***************************************************************************** \\