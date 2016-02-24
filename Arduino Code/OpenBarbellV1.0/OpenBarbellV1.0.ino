/*
 OpenBarbell V1.0 - the worlds first open source velocity measuring device
 
 This code utilizes an RFduino and an HLC2705 quatrature encoder chip to
 read the position of a retractable string attached to a barbell. 
 You can see squatsandscience.com/openbarbell for more information.
 
 Copyright (c) 2015 squatsandscience.com.  All right reserved.

 This code is free software; you can redistribute it and/or
 modify it under the terms of the Creative Commons 
 Attribution-NonCommercial-ShareAlike 4.0 International Public License.

 This code is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 See the Creative Commons Attribution-NonCommercial-ShareAlike 
 4.0 International Public License for more details.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 
 Created 2015
 Jordan Berke
 Jonathan Lenoff
 Elliot Noma
 Squats & Science Labs, LLC
 */

#include <SPI.h>
#include <Wire.h>
#include <Arduino.h>
#include <MAX1704.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306ms.h>
#include <RFduinoBLE.h>

#define OLED_RESET 9
Adafruit_SSD1306 display(OLED_RESET);

#define NUMFLAKES 10
#define XPOS 0
#define YPOS 1
#define DELTAY 2
#define LOGO16_GLCD_HEIGHT 16 
#define LOGO16_GLCD_WIDTH  16 


/***********START DEVICE SPECIFIC INFO ***************/

//Scheme "OB XXX" where XXX is the serial number
const char *device_name = "OB 20";
const long ticLength = 2684;

/***********END DEVICE SPECIFIC INFO ***************/


float CODE_VERSION = 1.04;

//START TestBed Section - Do not modify
const bool testbed_readouts = 0;
int peak_vel_at = 0;
//END TestBed Section

boolean bluetoothOn = false;
boolean bluetoothStartNextLoop = false;
float repPerformance[] = {0.0, 1.0,2.0,3.0,4.0,5.0};

const int pin_buttonRight =  0;
const int pin_buttonLeft = 1;
const int pin_led =  2;      // the number of the LED pin
const int pin_encoder_dir = 3;
const int pin_encoder_tach = 4;

const unsigned long batteryCheckTime = 10000000;     //The amount of time between battery checks. 10 sec
volatile int state = LOW;
//state flips on startup, so we need to put the temp value at HIGH so we don't run through the 
//code one time at startup before getting an actual tic reading
volatile int currentStateTemp = HIGH;
volatile bool goingUpward = 0;
volatile bool isGoingUpwardLast = 0;
long avgVelocity = 0;
unsigned long starttime = 0;

uint16_t rep = 0;
uint16_t repDone = 0;
uint16_t repDoneLast = 0;
uint16_t repDisplay = 2;
uint16_t repDisplayLast = 0;
long displacement = 0;

unsigned long tic_time = 0;
unsigned long tic_timestamp = 0;
unsigned long tic_timestampLast = 0;
unsigned long tic_timestampLast2 = 0;
unsigned long tic_timestamp_last = 0;
unsigned long minDT = 1000000;
unsigned long total_time = 0;

unsigned long displayTime = 0;
unsigned long batteryTime = 0;
unsigned long minTimer = 0;
unsigned long minTimer2 = 0;
unsigned long twoSecTimer2 = 0;
unsigned long oneMinute = 60000;
unsigned long twoSec = 2000;
unsigned long ticDiff = 0;
const unsigned long backlightTime = 10000;

//1200 Dts will give 1200*~2.68 mm = 3.2 m = 10.5 ft
const int myDTCounter_size = 1200;
uint16_t myDTs[myDTCounter_size] = {0};
const int repArrayCount=100;
float repArray[repArrayCount] = {0.0};
float peakVelocity[repArrayCount] = {0.0};
uint16_t dispArray[repArrayCount] = {0};
float timeArray[repArrayCount] = {0.0};


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


float currentInstVel = 0.0;
float lastInstVel = 0.0;
int buttonstate = 0;        
static unsigned long last_interrupt_time = 0;
static unsigned long last_interrupt_time2 = 0;
static unsigned long last_tic_time = 0;

uint16_t flipLED = 0;
uint16_t charge = 50;
uint16_t restTime = 0;
uint16_t myDTCounter = 0;
float startMessage[1] = {-1234.0};
const int threshold_buttonhold=100; //cycles of buttonholdtimer to cross threshold
const int buttonholdtimer=10;  //delay time
int counter_buttonRighthold=0;
int counter_buttonLefthold=0;
int minRepThreshold=150000;		//in micrometers - 150000 micrometers = 150 mm = ~5.9 inches
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
  
unsigned int counter_simplelengthbytic=0;		//This is a simple counter that is called to count how many times we enter the interrupt

unsigned long micros_holder=0;	//This is a temporary holder used so we don't have to keep calling micros()

// 		"Min" detectable speed = .01 m/s = 10 mm/s
// 		"Min" tick length = ~2.6 mm
// 		"Min" ticks/second = (min detectable speed)/(min tick length) = (10 mm/s)/(2.6 mm/tick) = ~3.8462 ticks/second
// 		"Max" time between ticks = (1 tick)/(min ticks/second)=(1)/(~3.8462 ticks/second) = .26 sec = 260000 microseconds = max_tick_time_allowable
const unsigned long  max_tick_time_allowable = 260000;		// max_tick_time_allowable is a variable that is used to determine if the the rep "started" but really it's just pausing - see above for math used to derive number

unsigned long time_waiting = 0; // Once we determine that the user is pausing during a rep we start to increment a waiting timer to subtract from the overall time

//		Starting threshold = first ~6 inches
//		starting threshold in ticks = (starting threshold)*(25.4 mm/in)/(min tick length) = (6)*(25.4)/(2.6)
const int wait_time_distance_min_in_ticks = 58;

int encoderState(uint32_t ulPin)
{
  state = !state;
  
  if(goingUpward){	//This if statement keeps the counter from counting on downward movements
	counter_simplelengthbytic++;	//This is the counter that counts the number of encoder wheel transitions in the interrupt
  }

  return 0;
}

MAX1704 fuelGauge;






// ********** Primary setup. Only to be run once on startup. ********** \\

void setup() {  


  RFduinoBLE.txPowerLevel = +4;
  RFduinoBLE.deviceName = device_name;
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
  
  fuelGauge.reset();
  fuelGauge.quickStart();
  fuelGauge.showConfig();

  //Welcome Screen - Custom Screen requires custom libraries
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
  RFduino_ULPDelay(2000);

  //initial check of the battery charge
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


/*



// ********** Test function to make sure we're setting BT power correctly ********** \\
//This doesn't ever exit...

void RFduinoBLE_onRSSI(int rssi){
	display.setTextColor(WHITE,BLACK);
	  display.setTextSize(1);
	  display.setCursor(55,0);
	  display.print(rssi);
	  display.print("db");
	  display.display();
}

// ********************************************************************************************************** \\

*/




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
  //this timer allows you in if you just reached the timer amount
	if((millis()-minTimer2)>30){
	//this timer resets after a longer period than the first timer, so it only enters once
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






// ********** Function to blink LED every XX seconds, if there's nothing going on ********** \\

void LEDBlink(){
	//if it's been 2 seconds, enter the statement. If the second timer is true, it won't be during the next loop so the first timer can't trip more than once.
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






// ********** Function to initialize Bluetooth if given the command by setup ********** \\

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

// ************************************************************************************ \\






// ********** Function to throw up a splash screen on BT connect ********** \\

void RFduinoBLE_onConnect(){
	display.clearDisplay();
	display.setTextSize(2);
    display.setCursor(0,0);
	display.print("BLUETOOTH");
	display.setCursor(0,32);
	display.print("CONNECTED");
	display.display();
	//display.startscrollleft(0x00, 0x0F);
	RFduino_ULPDelay(SECONDS(2));
	repDisplay = repDone+1;
}

// ************************************************************************ \\





/*
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

*/



// ********** Function that changes variables when requested by phone ********** \\

void RFduinoBLE_onReceive(char *data, int len)
{
// if the first byte is 0x01 / on / true
  if (data[0]){
	if(bitRead(data[1],1)){
		minRepThreshold = data[2]*1000;
		}
	if(bitRead(data[1],2)){
		flipPowerOlyScreen = !flipPowerOlyScreen;
		}
	if(bitRead(data[1],3)){
		isFlipped = !isFlipped;
		}
	if(bitRead(data[1],4)){
		//anything else?
		}
    }
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
  while (!RFduinoBLE.send(bytes, nbytes));
} // END send_intList_charString

void send_intList(int *intList, int len) {
  for (int i=0; i < len; i++) {
    while (!RFduinoBLE.sendInt(intList[i]));
  }
} // END send_intList

void send_floatList(float *floatList, int len) {
  for (int i=0; i < len; i++) {
    while (!RFduinoBLE.sendFloat(floatList[i]));
  }
} // END send_floatList

// ************************************************************************** \\






// ********** Function to send bulk velocity data over Bluetooth ********** \\


// In order to save on transmission time the uint16s will be combined together (when possible).
// The uint16_t are being combined like so... eg if we had an array foo where foo[0] = 16 and foo[1] = 23
// the numbers would be sent over as foo[0].(foo[1]/10000) or 16.0023.
// HOWEVER - The floats that are being sent can display up to 8 characters eg. 1234.5678 or 123456.78
// it is therefore necessary to make sure that the values being sent over are not over 10,000 or
// they will be truncated - Thus we check to make sure that the values aren't over 10,000 before combining
// since we are combining every two values, if the list has an odd number of values we need to send the last one

void send_float_from_intList(uint16_t *intList, uint16_t len) {
 
 //analogWrite(pin_led,10);
 
 for(int i=0; i<(len-1); i=i+2){
	if((intList[i]-10000)<0 && (intList[i+1]-10000)<0){	//Check to see if there will be a value that may be truncated
		while (!RFduinoBLE.sendFloat((float)intList[i]+(float)intList[i+1]/10000));
	} else {	//If there is a potential for truncation then each number will be sent separate
		while (!RFduinoBLE.sendFloat((float)intList[i]));
		while (!RFduinoBLE.sendFloat((float)intList[i+1]));
	}
  }
  
  if(len%2==1){	//Check if there was an odd number of values
	while(!RFduinoBLE.sendFloat((float)intList[len]));
  }
  
  //digitalWrite(pin_led,LOW);
  
} // END send_float_from_intList


// ******************************************** \\


void send_single_float(float singleFloat) {
    while (!RFduinoBLE.sendFloat(singleFloat));
    RFduino_ULPDelay(1);
} // END send_single_float


void send_all_data() {
	if (sendData) {//only send data if you just registered a new rep
	  repPerformance[0] = (float) rep;
	  repPerformance[1] = (float) avgVelocity; 
	  repPerformance[2] = (float) repArray[rep];
	  repPerformance[3] = (float) displacement; 
	  repPerformance[4] = (float) peakVelocity[rep]; 
	  send_floatList(repPerformance, 5);
	  send_single_float(-9876.0);
	  send_float_from_intList(myDTs, (myDTCounter/2));
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
      
	  //displacement = counter_simplelengthbytic*ticLength;
	  
	  micros_holder = micros();
	  
	  
	  // There was a bug found where it was possible to start going up but then hold a position without going down...this caused the total_time to
	  // continually increase and throw off the average velocity for the rep - to compensate for this we see how much time someone is waiting and
	  // subtract that from the total_time
	  
	  if(counter_simplelengthbytic<wait_time_distance_min_in_ticks && (micros_holder-tic_timestamp)>max_tick_time_allowable){
		time_waiting = time_waiting + micros_holder-tic_timestamp;
	  }
	  
      tic_timestampLast2 = tic_timestampLast;
      tic_timestampLast = tic_timestamp;
      tic_timestamp = micros_holder;

      
      // If you're going upward but you were just going downward, clear your array so you can start a fresh rep
      if (!isGoingUpwardLast){
        memset(myDTs,0,sizeof(myDTs));
        //memset(instVelTimestamps,0,sizeof(instVelTimestamps));
		
		time_waiting=0;
        counter_simplelengthbytic=0;
		
        starttime = tic_timestamp;
		send_floatList(startMessage, 1);
        rep += 1;

		currentInstVel = 0;
		lastInstVel = 0;
		
        minDT = 1000000;
		myDTCounter = 0;
      }
	  //keeping instantaneous velocities for our peak velocity reading
      //instVelTimestamps[counter_lengthbyticinfunction] = (unsigned int)(tic_timestamp-tic_timestamp_last);
      ticDiff = tic_timestamp - tic_timestamp_last;
	  tic_timestamp_last = tic_timestamp;
	  
		if (ticDiff < minDT){
			minDT=ticDiff;
			peak_vel_at=myDTCounter;
		}

	  
	  if(myDTCounter<myDTCounter_size){
		myDTs[myDTCounter] = (uint16_t)(ticDiff/10);
	  }
	  
	  myDTCounter++;
	  
	  
	  
      
    } else {
      // If you're going downward, and you were just going upward, you potentially just finished a rep. 
      // Do your math, check if it fits the rep criteria, and store it in an array.
	  if (isGoingUpwardLast && rep<=repArrayCount){
	  displacement = counter_simplelengthbytic*ticLength;
        if (displacement > minRepThreshold){ 
		  total_time = (tic_timestampLast - starttime) + .5*(tic_timestampLast - tic_timestampLast2) - time_waiting;
          dispArray[rep] = displacement/1000;
          timeArray[rep] = (float)total_time/1000000;
          peakVelocity[rep] = float(ticLength)/float(minDT);
         
		 repArray[rep] = ((float)(counter_simplelengthbytic*ticLength)/(float)(total_time/1000))/1000;
          
		  repDone = rep;
		  //resets 60 second rest time counter
          minTimer = millis();
          minTimer2 = millis();
		  restTime = 0;
		  counter_simplelengthbytic=0;

        } else { 
          rep -= 1;
        }
      }
	  //Fixes the LED disabled on startup bug
	  LEDInit = false;
      displacement -= ticLength;
    }
      
    isGoingUpwardLast = isGoingUpward;
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
		RFduino_ULPDelay(SECONDS(2));
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
				display.print("OLYMPIC");
				display.setCursor(0,32);
				display.print("MODE");
			}else{
				display.print("POWER");
				display.setCursor(0,32);
				display.print("MODE");
			}
			display.display();
			RFduino_ULPDelay(SECONDS(2));
			repDisplay = repDone;
			}
		}
  }

  if ((repDisplay != repDisplayLast)||(repDone != repDoneLast)){  
    // if the displayed rep changes, keep the time so we know when to dim the backlight
    displayTime = millis();
    // make sure we can see the new rep
    display.ssd1306_command(SSD1306_DISPLAYON);
    backlightFlag = 1;
    
    if (repDisplay == (repDone + 1)){
		//This 'if' statement keeps you from going from "Begin Set" back to "Delete Past Set?"
		if(repDisplayLast < (repDone + 1)){
		  display.clearDisplay(); 
		  display.setTextSize(2);
		  display.setTextColor(WHITE);
		  display.setCursor(0,0);
		  display.println("Delete");
		  display.println("Past Set?");
		  display.setTextSize(1);
		  display.setCursor(0,40);
		  display.println("R Button-Delete Set");
		  display.println("L Button-Go Back");
		  display.display();        
		}
    } else if (repDisplay > (repDone + 1)){
      counter_simplelengthbytic=0; //JDLTEST
	  //This line keeps the repDisplay value from getting too big, and causing a bug to miss the first rep (edit: might not be necessary)
	  //repDisplay = repDone + 2;
      
	  //Yes, you can just say rep = goingUpward. But goingUpward can only be 0 or 1 and rep can be any integer. It just doesn't feel right.
      rep = (goingUpward)?(1):(0);
      repDone = 0;
      repDoneLast = 0;
	  
	  display.clearDisplay(); 
      display.setTextSize(2);
      display.setTextColor(WHITE,BLACK);
      display.setCursor(0,15);
      display.print("Begin Set!");
	  systemTrayDisplay();
	  display.setTextSize(1);
	  display.setCursor(0,0);
	  display.print("Rep#:1  ");
      display.display(); 
	  RFduino_ULPDelay(1);
	  
      memset(repArray,0,sizeof(repArray));
      memset(myDTs,0,sizeof(myDTs));
      memset(dispArray,0,sizeof(dispArray));
      memset(timeArray,0,sizeof(timeArray));
      memset(peakVelocity,0,sizeof(peakVelocity));
      //memset(instVelTimestamps,0,sizeof(instVelTimestamps));
      myDTCounter = 0;
    } else {
		if(!flipPowerOlyScreen){
		  display.clearDisplay();
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
				display.setCursor(100,12);
				display.print(peak_vel_at);

				display.setCursor(100,22);
				display.print(myDTCounter);
			}

		  systemTrayDisplay();
		  display.display();      
		  
          send_all_data(); //moved send_all_data here so it doesn't lag the display
		  
	    } else {
		  display.clearDisplay();
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
		  systemTrayDisplay();
		  
		  	if(testbed_readouts){
				display.setTextSize(1);
				display.setCursor(100,12);
				display.print(peak_vel_at);

				display.setCursor(100,22);
				display.print(myDTCounter);
			}
		  
		  
		  display.display();      
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