#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h> //OLED display
#include <Adafruit_ADS1015.h> //AD converter

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */

  // The ADC input range (or gain) can be changed via the following
  // functions, but be careful never to exceed VDD +0.3V max, or to
  // exceed the upper and lower limits if you adjust the input range!
  // Setting these values incorrectly may destroy your ADC!
  //                                                                ADS1015  ADS1115
  //                                                                -------  -------
  // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  //ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV

float multiplier = 0.03125F; //multiply output of AD converter (in bits) by this number to obtain mV (corresponding to chosen gain)



//moving average variables
const int numReadings = 10;
int16_t readings[numReadings];

int readIndex = 0;                // the index of the current reading
float total = 0;                  // the running total
float average = 0;                // the average

float zero = 0; //set offset in mV for flow = 0 during initial calibration



//variables for alarm and buttons
const int soundAlarm = D6;
const int ledAlarm = D5;

const int stopAlarmButton = D7;
const int selectionButton = D8;

int selectionValue = 0;
int selectionLast = 0;

const int debounce = 20; // ms debounce period to prevent flickering when pressing or releasing the button
const int holdTime = 2000; // ms hold period: how long to wait for press+hold event
const int waitTime = 3000; // how long after release to wait for next press, before requiring to hold again

float btnDnTime; // time the button was pressed down
float btnUpTime; // time the button was released
boolean ignoreUp = true; // whether to ignore the button release because of the need to click+hold (true allows execution, because button was held longer than holdTime)

int volumeOptions[] = {0, 300, 350, 400, 450, 500, 550, 600}; //possible reference volumes for the patient, chosen by pressing selection button
int j = 0; // iterate over volumeOptions

int stopAlarmValue = 0;
int stopAlarmLast = 0;

const int stopAlarmHoldTime = 2000; // ms hold period: how long to wait for press+hold event
long alarmBtnDnTime; // time the button was pressed down
long alarmBtnUpTime; // time the button was released

bool alarm = false; // alarm activated = true



//volume and flow variables
int referenceVolume = 0; // target volume in ml (set by doctor as an input, chosen from volumeOptions). Minimum to achieve in each inspiratory cycle
float receivedVolume = 0; // received so far on this inspiratory cycle
float receivedVolPreviousCycle = 0; //received during the cycle before current peak was detected. (this is the volume sent to display)

float flow = 0; // calculated flow from pressure differential
float flowBefore = 0; //flow in previous iteration
float offset = 0;



//variables for flow peak detection and volume integration
float flowDifferential = 0;
bool integrate = false;
float threshold = 5;
int startTimeLastCycle = 0;
int endTimeLastCycle = 0;

int timeNow = millis(); // start counting time
int timeBefore = timeNow;
float timeInterval; //will be converted to seconds



void setup() {
  Serial.begin(115200);
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.println(F("calibrating"));
  display.print(F("zero flow..."));
  display.display();
  delay(1000);
  ads.begin();
  ads.setGain(GAIN_FOUR);
  // calculate offset (in bits):
  for (int i = 0; i < 300; i++) {
    zero = ads.readADC_Differential_0_1();
    total += zero;
  }
  zero = total / 300;
  total = 0;
  // set first values for moving average as the calculated zero
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = zero;
    total += readings[thisReading];
  }
  offset = (0.0825 * pow(zero*multiplier,2)) - 42.002 * zero*multiplier + 5156.3;//zero flow offset
  updateScreen(000, 000, false);
  pinMode(soundAlarm, OUTPUT);
  pinMode(ledAlarm, OUTPUT);
  pinMode(stopAlarmButton, INPUT);
  pinMode(selectionButton, INPUT);
}


// function for updating display
void updateScreen(float lastVT, float targetVT, bool alarm){//, int referenceVolume) {
  display.clearDisplay();
  display.setTextSize(2); // Draw 2X-scale text
  display.setTextColor(WHITE);
  if(alarm) {
    display.setCursor(32, 0);
    display.println(F("ALARM"));
  }
  display.setCursor(0, 26);
  display.print(F("VT: "));
  display.print(lastVT);
  display.setCursor(0, 45);
  display.print(F("Ref: "));
  display.print(targetVT);
  display.display();
}


void loop() {

  ///--------------------------------- moving average filter--------------------------///
  total = total - readings[readIndex]; // remove oldest value from average
  
  readings[readIndex] = ads.readADC_Differential_0_1(); // update array and total with this reading
  timeNow = millis();
  
  //---------Read the state of the buttons to use later------------////
  stopAlarmValue = digitalRead(stopAlarmButton);
  selectionValue = digitalRead(selectionButton);
  //---------------------------------------------------------------////

  total += readings[readIndex];
  readIndex = (readIndex + 1) % numReadings;

  // calculate the average:
  average = total * multiplier / numReadings;
  flow = (0.0825 * pow(average,2)) - 42.002 * average + 5156.3 - offset;//calibrated curve for flow vs voltage
  
  timeInterval = (timeNow - timeBefore); //in milliseconds

  flowDifferential = flow * ((flow - flowBefore)/timeInterval);


  ///-------------------------------- peak detection---------------------------------//

  if (integrate == false && flowDifferential >= 100 && (timeNow - endTimeLastCycle) > 200){
    integrate = true;
    startTimeLastCycle = timeNow;
  }
  else if (flow < flowBefore && flow <= threshold && integrate == true){
    integrate = false;
    endTimeLastCycle = timeNow;
    receivedVolPreviousCycle = receivedVolume; //store value of volume before the new cycle begins (does this before integration of current flow)
    receivedVolume = 0; //reset for upcoming cycle

    //check for alarm activation, based on target VT (referenceVolume)
    if (receivedVolPreviousCycle < referenceVolume){
      alarm = true;
      digitalWrite(ledAlarm, HIGH);
      tone(soundAlarm,1000);
    }
  }
  
  //display volume achieved on the previous cycle, and alarm, if activated
  updateScreen(receivedVolPreviousCycle, referenceVolume, alarm);
  
  if (integrate == true){
    receivedVolume += (timeInterval * (flow + flowBefore)/2000);
  }
  
  timeBefore = timeNow;
  flowBefore = flow;


  ///------------------------------------- stop alarm button -------------------------------------////

    // Test for button pressed and store the down time
    if (stopAlarmValue == HIGH && stopAlarmLast == LOW && (timeNow - alarmBtnUpTime) > long(debounce)){
      alarmBtnDnTime = timeNow;
    }
    // Test for button release and store the up time
    if (stopAlarmValue == LOW && stopAlarmLast == HIGH && (timeNow - alarmBtnDnTime) > long(debounce)){
      alarmBtnUpTime = timeNow;
    }
    // Test for button held down for longer than the hold time, and deactivate alarm
    if (stopAlarmValue == HIGH && (timeNow - alarmBtnDnTime) > long(stopAlarmHoldTime)){
      noTone(soundAlarm);
      digitalWrite(ledAlarm, LOW);
      alarm = false;
      //update screen to cancel ALARM
      updateScreen(receivedVolPreviousCycle, referenceVolume, alarm);
    }
    stopAlarmLast = stopAlarmValue;



  ///-------------------------------------- selection button ---------------------------------------////

    // Test for button pressed and store the down time
    if (selectionValue == HIGH && selectionLast == LOW && (timeNow - btnUpTime) > long(debounce)){
      btnDnTime = timeNow;
      // reset ignoreUp if button has been up for longer than waitTime (ignoreUp deactivates selection)
      if ((btnDnTime - btnUpTime) > long(waitTime)) ignoreUp = true; 
    }
    // Test for button release and store the up time
    if (selectionValue == LOW && selectionLast == HIGH && (timeNow - btnDnTime) > long(debounce)){
      btnUpTime = timeNow;
      //if selection is active, iterate over volumeOptions and update display with new referenceVolume
      if (ignoreUp == false) {
        j = (j + 1) % 8;
        referenceVolume = volumeOptions[j];
        updateScreen(receivedVolPreviousCycle, referenceVolume, alarm);
      }
    }
    // Test for button held down for longer than the hold time and activate selection by setting ignoreUp to false
    if (selectionValue == HIGH && (timeNow - btnDnTime) > long(holdTime)){
      ignoreUp = false;
    }
    selectionLast = selectionValue;



}
