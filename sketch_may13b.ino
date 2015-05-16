/*
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */

#define USE_USBCON
#include "ros.h"
#include "std_msgs/Empty.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "arvp_main/Depth.h"
#include "arvp_main/Touch.h"
#include "arvp_main/IMU.h"
#include <UTFT.h>


// Set the pins to the correct ones for your development shield
// ------------------------------------------------------------
// Standard Arduino Mega/Due shield            : <display model>,38,39,40,41
// CTE TFT LCD/SD Shield for Arduino Due       : <display model>,25,26,27,28
// Teensy 3.x TFT Test Board                   : <display model>,23,22, 3, 4
// ElecHouse TFT LCD/SD Shield for Arduino Due : <display model>,22,23,31,33
//
// Remember to change the model parameter to suit your display module!
UTFT myGLCD(SSD1289,25,26,27,28);

ros::NodeHandle nh;

// Declare which fonts we will be using
extern uint8_t SmallFont[];
extern uint8_t BigFont[];
extern uint8_t SevenSegNumFont[];

boolean mainDataPage = true;
boolean errorPage = true;

struct data {
  char * title;
  float value;
  int data_x;
  int data_y;
  int title_x;
  int title_y;
  int decimal_places;  
};  


data currentHeading;
data currentRoll;
data currentPitch;
data currentTouch;
data currentDepth;
data all_values[5];


void depthMessage( const arvp_main::Depth& DepthTopic){
  nh.loginfo("displaying stuff");
      myGLCD.setColor(255, 255, 255);
  myGLCD.print("D: ", 0, 0);
  myGLCD.printNumF(DepthTopic.depth, 2, 50, 0);
//  delay(900);
  
}

void setupData() {
 currentHeading.title = "H: ";
currentRoll.title = "R: ";
currentPitch.title = "P: ";
currentTouch.title = "T: ";
currentDepth.title = "D: ";


currentHeading.value = 0.0;
currentRoll.value = 0.0;
currentPitch.value = 0.0;
currentTouch.value = 0.0;
currentDepth.value = 0.0;



currentHeading.title_x = 0;
currentHeading.title_y = 60;

currentDepth.title_x = 0;
currentDepth.title_y = 0;

currentRoll.title_x = 0;
currentRoll.title_y = 120;

currentPitch.title_x = 0;
currentPitch.title_y = 180;

currentTouch.title_x = 150;
currentTouch.title_y = 0;





currentHeading.data_x = 50;
currentHeading.data_y = 60;

currentDepth.data_x = 50;
currentDepth.data_y = 0;

currentRoll.data_x = 50;
currentRoll.data_y = 120;

currentPitch.data_x = 50;
currentPitch.data_y = 180;

currentTouch.data_x = 200;
currentTouch.data_y = 0;


currentHeading.decimal_places = 2;
currentDepth.decimal_places = 2;
currentRoll.decimal_places = 2;
currentPitch.decimal_places = 2;
currentTouch.decimal_places = 2;

data all_values[5] = {currentHeading, currentRoll, currentPitch, currentTouch, currentDepth};

 
}

void updateAllValuesWithCurrentValues() {
    // print (decimal places, x ,y )
  
    myGLCD.clrScr();
    
    /*
    
    for (int i = 0; i < 5; i++) {
      nh.loginfo((const char*)all_values[i].title);
      nh.loginfo((const char*)all_values[i].title_x);
      nh.loginfo((const char*)all_values[i].title_y);
      nh.loginfo((const char*)(int)all_values[i].value);
      nh.loginfo((const char*)all_values[i].data_x);
      nh.loginfo((const char*)all_values[i].data_y);
      nh.loginfo((const char*)all_values[i].decimal_places);
      
      myGLCD.print(all_values[i].title, all_values[i].title_x, all_values[i].title_y);
      myGLCD.printNumF(all_values[i].value, all_values[i].decimal_places, all_values[i].data_x, all_values[i].data_y);
       
    }
    
    */
    
    myGLCD.print(currentDepth.title, currentDepth.title_x, currentDepth.title_y);
    myGLCD.printNumF(currentDepth.value, currentDepth.decimal_places, currentDepth.data_x, currentDepth.data_y);

    
    myGLCD.print("H: ", 0, 60);
    myGLCD.printNumF(currentHeading.value, 2, 50, 60);


    myGLCD.print("R: ", 0, 120);
    myGLCD.printNumF(currentRoll.value, 2, 50, 120);


    myGLCD.print("P: ", 0, 180);
    myGLCD.printNumF(currentPitch.value, 2, 50, 180);

    myGLCD.print("T: ", 200, 0);
    myGLCD.printNumF(currentTouch.value, 2, 250, 0);

}

void IMUMessage( const arvp_main::IMU& IMUTopic){
//  nh.loginfo(Depth.data);
//  nh.loginfo("displaying stuff");
  if(IMUTopic.heading != currentHeading.value) {
    currentHeading.value = IMUTopic.heading;
      updateAllValuesWithCurrentValues();
//    myGLCD.setColor(255, 255, 255);
//    myGLCD.print(currentHeading.title, currentHeading.title_x , currentHeading.title_y);
//    myGLCD.printNumF(currentHeading.value, currentHeading.decimal_places, currentHeading.data_x, currentHeading.data_y);
     
   }
   
   else if (IMUTopic.pitch != currentPitch.value) {
    currentPitch.value = IMUTopic.pitch;
      updateAllValuesWithCurrentValues();
//    myGLCD.setColor(255, 255, 255);
//   myGLCD.print(currentPitch.title, currentPitch.title_x , currentPitch.title_y);
//    myGLCD.printNumF(currentPitch.value, currentPitch.decimal_places, currentPitch.data_x, currentPitch.data_y);
     
   }

   else if (IMUTopic.roll != currentRoll.value) {
    currentRoll.value = IMUTopic.roll;
      updateAllValuesWithCurrentValues();
//    myGLCD.setColor(255, 255, 255);
//    myGLCD.print(currentRoll.title, currentRoll.title_x , currentRoll.title_y);
//    myGLCD.printNumF(currentRoll.value, currentRoll.decimal_places, currentRoll.data_x, currentRoll.data_y);
     
   }
 
//  delay(900);
  
  
}


void RollMessage( const std_msgs::String& Roll){
//  nh.loginfo(Depth.data);
  char * message;
  sprintf(message, "Roll: %s",Roll.data);
  myGLCD.print(message, LEFT, 200);
//  delay(900);
  
}


void pitchMessage( const std_msgs::String& Pitch){
//  nh.loginfo(Depth.data);
  char * message;
  sprintf(message, "Pitch: %s",Pitch.data);
  myGLCD.print(message, CENTER, 1);
//  delay(900);
  
  
}

void PIDMessage( const std_msgs::String& PID){
//  nh.loginfo(Depth.data);
  char * message;
  sprintf(message, "PID: %s",PID.data);
  myGLCD.print(message, CENTER, 100);
//  delay(900);
  
  
}


void touchMessage( const arvp_main::Touch& TouchTopic){
  if(TouchTopic.touch != currentTouch.value) {
    currentTouch.value = TouchTopic.touch;
      updateAllValuesWithCurrentValues();
    
//    myGLCD.setColor(255, 255, 255);
//    myGLCD.print(currentTouch.title, currentTouch.title_x , currentTouch.title_y);
//    myGLCD.printNumF(currentTouch.value, currentTouch.decimal_places, currentTouch.data_x, currentTouch.data_y);
     
   }
  
  
}



void errorMessage( const std_msgs::String& ErrorTopic) {
//  nh.loginfo(Depth.data);
  displayErrorLines("Error Reported!!!", ErrorTopic.data, "");
  delay(1000);
  
  
}



void batteryMessage( const std_msgs::String& Battery) {
//  nh.loginfo(Depth.data);
  char * message;
  sprintf(message, "Touch: %s", Battery.data);
  myGLCD.print(message, RIGHT, 1);
//  delay(900);
  
  
}


void displayErrorLines(String line1, String line2, String line3) {

      myGLCD.clrScr();
      myGLCD.setFont(SmallFont);
      myGLCD.fillScr(0, 0, 255);
      myGLCD.setColor(255, 0, 0);
      myGLCD.fillRoundRect(80, 70, 239, 169);
    
      myGLCD.setColor(255, 255, 255);
      myGLCD.setBackColor(255, 0, 0);
      myGLCD.print(line1, CENTER, 93);
      myGLCD.print(line2, CENTER, 119);
      myGLCD.print(line3, CENTER, 132);
  
}




ros::Subscriber<arvp_main::Depth> depthSub("sensors/Depth", &depthMessage );
ros::Subscriber<arvp_main::IMU> IMUSub("sensors/imu", &IMUMessage );

ros::Subscriber<arvp_main::Touch> touchSub("sensors/touch", &touchMessage );

ros::Subscriber<std_msgs::String> errorSub("display/errors", &errorMessage );


ros::Subscriber<std_msgs::String> RollSub("Roll", &RollMessage );
ros::Subscriber<std_msgs::String> pitchSub("Pitch", &pitchMessage );
ros::Subscriber<std_msgs::String> PIDSub("PID", &PIDMessage );
ros::Subscriber<std_msgs::String> batterySub("Battery", &batteryMessage );


void setup()
{
  
  
  setupData();
    
  // Setup the LCD
  myGLCD.InitLCD();
  myGLCD.setFont(BigFont);


  // subscribe to required topics  
  nh.initNode();
  nh.subscribe(depthSub);
  nh.subscribe(IMUSub);
  nh.subscribe(touchSub);
  nh.subscribe(errorSub);
  //  nh.subscribe(headingSub);
 // nh.subscribe(RollSub);
  //nh.subscribe(pitchSub);
//  nh.subscribe(PIDSub);
//  nh.subscribe(touchSub);
//  nh.subscribe(batterySub);
}

void loop()
{
  
  // for event checking
  nh.spinOnce();
  
  if (nh.connected() && errorPage) {
     // this is where we wait for messages  
      myGLCD.setFont(BigFont);
      nh.loginfo("displaying mainDataPage");    
      myGLCD.clrScr();
      myGLCD.fillScr(0, 0, 0);
      myGLCD.setColor(255, 255, 255);
      myGLCD.setBackColor(0, 0, 0);
      
      updateAllValuesWithCurrentValues();
         
      errorPage = false;
      mainDataPage = true;
    
  }

  else if (mainDataPage && !nh.connected()) {

    
      displayErrorLines("We Aren't Connected", "Checking connection", "in a few seconds...");
      
      
      delay(300); 
      errorPage = true;
      mainDataPage = false;
    
  }
}

