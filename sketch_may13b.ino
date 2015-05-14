/*
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */

#define USE_USBCON
#include "ros.h"
#include "std_msgs/Empty.h"
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


void messageCb( const std_msgs::Empty& toggle_msg){
//  nh.loginfo("got something");
    myGLCD.clrScr();

   myGLCD.clrScr();
  myGLCD.print("* CONNECTED!!!!!!!  and recieved something*", CENTER, 1);
    delay(900);

}

ros::Subscriber<std_msgs::Empty> sub("toggle_led", &messageCb );

void setup()
{
  
  // Setup the LCD
  myGLCD.InitLCD();
  myGLCD.setFont(SmallFont);

  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{
  nh.spinOnce();
    
  if (nh.connected()) {
   myGLCD.clrScr();
   myGLCD.print("* CONNECTED!!!!!!! *", CENTER, 1);
    delay(300);
  }
  else {
  myGLCD.fillScr(0, 0, 255);
  myGLCD.setColor(255, 0, 0);
  myGLCD.fillRoundRect(80, 70, 239, 169);
  
  myGLCD.setColor(255, 255, 255);
  myGLCD.setBackColor(255, 0, 0);
  myGLCD.print("We Aren't Connected", CENTER, 93);
  myGLCD.print("Checking connection", CENTER, 119);
  myGLCD.print("in a few seconds...", CENTER, 132);
    delay(300);
 }
}
