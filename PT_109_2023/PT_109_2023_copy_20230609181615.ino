/*  PT-109 Payload Sketch
    Code to control the top deck functions of the PT-109:
      - Deck gun (LED, servo, speaker)
      - Searchlight (LED, servo)
      - Console (temp sensor, LEDs)
    By: Micah Schwichtenberg and Randy Knox
    Version history:
      pt109_payload_v06: 13 Aug 2016: used for Lighthouse Night
      pt109_payload_v11b: 29 Aug 2016: modified 'm' and 'b' in linear equation in MonitorTemp routine
      pt109_payload-v11c: 01 Sep 2016: updated random seed to use micros time; cleaned up some 
                                        dormant code fragments
      pt109-payload_v11d: 14 Sep 2016: modified the switch limit checks for the EVEL channel
                                        see p31 of engineering notebook 
      pt109_payload_v11e: 14 Sep 2016: put in a statement to drive elev pulse width to 1599 if it came back
                                        zero; did not seem to be effective in all cases
      pt109_payload_v11f: 14 Sep 2016: took the timeout of 25000 from elev pulsein function, letting
                                        the default of 1 sec; this seems to be working  
      pt109_payload_v11g: 18 Sep 2016: modified the MonitorTemp routine to output temp in C; then 
                                        convert to F in mainline code; also added statements to 
                                        drive gun and SL servos to zero when switch is turned off 
      pt109_payload_v12: 18 Sep 2016: a) modified C to F equation with all floating point numbers
                                      b) took the 25000 timeout out of gun pulseIn function
                                      c) lowered gun slew rate interval to 80 msec from 120
      pt109 payload v13: 9 Jun 2023:  a) will put this in to Github for version control
                                      b) remove the console lights and temp monitoring as we will not do that
                                      c) corrected the variable 'signal' as it is now a keyword                                                                
*/ 
/* 
+-+-+-+-+-+-+-+-+-+-+-+-+ 
    Summary of Pin Assignments
      Pin  0:  unassigned
      Pin  1:  unassigned
      Pin  2:  console led 0
      Pin  3:  console led 1
      Pin  4:  console led 2
      Pin  5:  console led 3
      Pin  6:  searchlight LED
      Pin  7:  searchlight servo
      Pin  8:  gun speaker
      Pin  9:  gun muzzle LED
      Pin 10:  gun servo
      Pin 11:  signal from evel channel (searchlight)
      Pin 12:  signal from gear channel (gun)
      Pin 13:  unassigned
      Pin A0:  unassigned
      Pin A1:  input voltage from temputure sensor
      Pin A3:  unassigned
      Pin A4:  unassigned
      Pin A5:  unassigned
      Pin A6:  unassigned
+-+-+-+-+-+-+-+-+-+-+-+-+ 
*/
#include <Servo.h>
Servo slServo;  // searchlight servo object
Servo gunServo;  // gun servo object
int pos = 0;

// variables for searchlight routine
boolean sloffFlag = true;
int searchlightPin = 6;
int searchlightsignal = 11;  // signal from elev channel (searchlight)
int slServoPin = 7;
int angle = 0;
int quad = 1;
unsigned long slMark = 0;
unsigned long slInterval = 120; //  search light slew rate (Msec)

// variables for gun fire routine
int gearpulseWidth = 1300;
boolean fireFlag = true;
boolean burstFlag = true;
int muzzleLedPin = 9;
int gunSignal = 12;  //signal from Gear switch/channel (gun)
int gunServoPin = 10;
int gunSpeaker = 8;
int gunPitch = 75;
int gunAngle = 0;
int gunQuad = 1;
unsigned long gunMark = 0;
unsigned long gunInterval = 80;  // gun slew rate (msec)
unsigned long fireRateTimeMark = 0;
unsigned long fireRateTimeInterval = 40;
unsigned long burstTimeMark = 0;
unsigned long burstInterval = 000;
int minInterval = 700;  //gunfire burst & silence interval limits (msec)
int maxInterval = 2700;

// vareables for console display
int consoleLED0 = 2;
int consoleLED1 = 3;
int consoleLED2 = 4;
int consoleLED3 = 5;
//=====================================================================
//                        CONSOLE
//=====================================================================
void console(int number){
  if(number < 0) number = 0;
  if (number > 15) number = 15;
  switch(number){
  case 0:
  digitalWrite(consoleLED0, LOW);
  digitalWrite(consoleLED1, LOW);
  digitalWrite(consoleLED2, LOW);
  digitalWrite(consoleLED3, LOW);
  break;
  
  case 1:
  digitalWrite(consoleLED0, HIGH);
  digitalWrite(consoleLED1, LOW);
  digitalWrite(consoleLED2, LOW);
  digitalWrite(consoleLED3, LOW);
  break;
  
  case 2:
  digitalWrite(consoleLED0, LOW);
  digitalWrite(consoleLED1, HIGH);
  digitalWrite(consoleLED2, LOW);
  digitalWrite(consoleLED3, LOW);
  break;
  
  case 3:
  digitalWrite(consoleLED0, HIGH);
  digitalWrite(consoleLED1, HIGH);
  digitalWrite(consoleLED2, LOW);
  digitalWrite(consoleLED3, LOW);
  break;
  
  case 4:
  digitalWrite(consoleLED0, LOW);
  digitalWrite(consoleLED1, LOW);
  digitalWrite(consoleLED2, HIGH);
  digitalWrite(consoleLED3, LOW);
  break;
  
  case 5:
  digitalWrite(consoleLED0, HIGH);
  digitalWrite(consoleLED1, LOW);
  digitalWrite(consoleLED2, HIGH);
  digitalWrite(consoleLED3, LOW);
  break;
  
  case 6:
  digitalWrite(consoleLED0, LOW);
  digitalWrite(consoleLED1, HIGH);
  digitalWrite(consoleLED2, HIGH);
  digitalWrite(consoleLED3, LOW);
  break;
  
  case 7:
  digitalWrite(consoleLED0, HIGH);
  digitalWrite(consoleLED1, HIGH);
  digitalWrite(consoleLED2, HIGH);
  digitalWrite(consoleLED3, LOW);
  break;
  
  case 8:
  digitalWrite(consoleLED0, LOW);
  digitalWrite(consoleLED1, LOW);
  digitalWrite(consoleLED2, LOW);
  digitalWrite(consoleLED3, HIGH);
  break;
  
  case 9:
  digitalWrite(consoleLED0, HIGH);
  digitalWrite(consoleLED1, LOW);
  digitalWrite(consoleLED2, LOW);
  digitalWrite(consoleLED3, HIGH);
  break;
  
  case 10:
  digitalWrite(consoleLED0, LOW);
  digitalWrite(consoleLED1, HIGH);
  digitalWrite(consoleLED2, LOW);
  digitalWrite(consoleLED3, HIGH);
  break;
  
  case 11:
  digitalWrite(consoleLED0, HIGH);
  digitalWrite(consoleLED1, HIGH);
  digitalWrite(consoleLED2, LOW);
  digitalWrite(consoleLED3, HIGH);
  break;
  
  case 12:
  digitalWrite(consoleLED0, LOW);
  digitalWrite(consoleLED1, LOW);
  digitalWrite(consoleLED2, HIGH);
  digitalWrite(consoleLED3, HIGH);
  break;
  
  case 13:
  digitalWrite(consoleLED0, HIGH);
  digitalWrite(consoleLED1, LOW);
  digitalWrite(consoleLED2, HIGH);
  digitalWrite(consoleLED3, HIGH);
  break;
  
  case 14:
  digitalWrite(consoleLED0, LOW);
  digitalWrite(consoleLED1, HIGH);
  digitalWrite(consoleLED2, HIGH);
  digitalWrite(consoleLED3, HIGH);
  break;
  
  case 15:
  digitalWrite(consoleLED0, HIGH);
  digitalWrite(consoleLED1, HIGH);
  digitalWrite(consoleLED2, HIGH);
  digitalWrite(consoleLED3, HIGH);
  break;
  }
 return;
}
//=====================================================================
//                        ISTIME
//=====================================================================
int IsTime(unsigned long *timeMark, unsigned long timeInterval){
  unsigned long timeCurrent;
  unsigned long timeElapsed;
  int result = false;
  
  timeCurrent = millis();
  timeElapsed= timeCurrent - *timeMark;
  
  if (timeElapsed >= timeInterval){
    *timeMark = timeCurrent;
    result = true;
  }
  return(result);
}
//=====================================================================
//                        SEARCHLIGHT
//=====================================================================
void searchLight(){
  digitalWrite(searchlightPin, HIGH);
  //Serial.println("Searchlight on");
  if(IsTime(&slMark, slInterval)){  //if time to slew, ascertain which quad we are in
  //Serial.print("++++++++++++Time ");
  //Serial.println(quad);
  //delay(200);
  switch(quad){
    case 1:  // between 0 and +90 degrees
    angle = angle + 1; 
    //Serial.print("quad 1");
    //Serial.print("  angle  ");
    //Serial.println(angle);
    //delay(200);
    if(angle < 179){
      slServo.write(angle);
    }
    else{
      quad = 2;
    }
    break;
    
    case 2:  // between +90 and -90 degrees
    angle = angle - 1;
    //Serial.print("quad 2  ");
    //Serial.print("  angle  ");
    //Serial.println(angle);
    //delay(200);
    if(angle > 0){
      slServo.write(angle);
    }
    else{
      quad = 3;
    }
    break;
    
    case 3:  // between -90 and 0 degrees
    angle = angle + 1;
    //Serial.print("quad 3  ");
    //Serial.print("  angle  ");
    //Serial.println(angle);
    //delay(200);
    if(angle < 0){
      slServo.write(angle);
      //Serial.println("quad 3");
    }
      else{
        quad = 1;
      }
    }
  }// if not time to slew, just drop through
}  
//=====================================================================
//                        GUNFIRE
//=====================================================================
void gunFire(){
  //Serial.println("Gunfire function");
  if(IsTime(&gunMark, gunInterval)){  //if time to slew, ascertain which quad we are in
  //Serial.print("++++++++++++Time ");          
  //Serial.println(gunQuad);
  //delay(200); 
    switch(gunQuad){
    case 1:  // between 0 and +90 degrees
    gunAngle = gunAngle + 1; 
//    Serial.print("quad 1");
//    Serial.print("  angle  ");
//    Serial.println(gunAngle);
//    delay(200);
    if(gunAngle < 179){
      gunServo.write(gunAngle);     
    }
    else{
      gunQuad = 2;
    }
    break;
    
    case 2:  // between +90 and -90 degrees
    gunAngle = gunAngle - 1;
//    Serial.print("quad 2  ");
//    Serial.print("  angle  ");
//    Serial.println(gunAngle);
//    delay(200);
    if(gunAngle > 0){
      gunServo.write(gunAngle);
    }
    else{
      gunQuad = 3;
    }
    break;
    
    case 3:  // between -90 and 0 degrees
    gunAngle = gunAngle + 1;
    //Serial.print("quad 3  ");
    //Serial.print("  angle  ");
    //Serial.println(angle);
    //delay(200);
    if(gunAngle < 0){
      gunServo.write(gunAngle);
      //Serial.println("quad 3");
    }
      else{
        gunQuad = 1;
      }
    }
  }// if not time to slew, just drop through
  //Serial.println("end of slew rtn");
                                                                                
  if (IsTime(&fireRateTimeMark, fireRateTimeInterval)){ //has the fire rate timer expired,
                // if yes, then determine if we are in burst or silence mode
    if(burstFlag){
      //Serial.println("burst flag on");
      //delay(300);
      if(fireFlag){
        //Serial.println("fire flag on");
        //delay(600);
        digitalWrite(muzzleLedPin, HIGH);
        tone(gunSpeaker, gunPitch);
        fireFlag = !fireFlag;
      }
      else{
       digitalWrite (muzzleLedPin, LOW);
       noTone(gunSpeaker);
       fireFlag = !fireFlag;
      } 
  }
  else{
      digitalWrite (muzzleLedPin, LOW);
      noTone(gunSpeaker);
      fireFlag = !fireFlag;
   }
  }
  else{
    digitalWrite (muzzleLedPin, LOW);
  }
  
  if(IsTime(&burstTimeMark, burstInterval)){
    
    burstInterval =random(minInterval, maxInterval);
    burstFlag = !burstFlag;
  }
  return;
}
//=====================================================================
//                          MonitorTemp
//=====================================================================
float monitorTemp(){
 int tempReading;
 float tempC;
 tempReading = analogRead(1);
 //Serial.print("mV  ");
 //Serial.println(tempReading);
 //delay(1000);
 //tempC = tempReading;
 //tempC = 60 * tempReading -32;
 //float tempF =  ((tempC + 40)* (9 / 5) - 40);
 //float mTemp = 1.8 * (tempReading) - 32;   // conversion factor based on expirements with a cup of hot water
                                           // see Note on p30 of engineering note book
 /*   convert to C using equation from:
      playground.arduino.cc/Main/LM35HigherResolution
 */
 tempC = (5.0 * tempReading * 100.0)/1024;                                          
 //Serial.print("mTemp  ");
 //Serial.println(mTemp);
 return tempC;
}
//= + = + = + = + = + = + = + = + = + = + = + = + = + = + = + = + = + =
//                          setup
//= + = + = + = + = + = + = + = + = + = + = + = + = + = + = + = + = + =
void setup() { 
  Serial.begin(9600); 
  pinMode(searchlightsignal, INPUT);
  pinMode(gunSignal, INPUT);
  pinMode(searchlightPin, OUTPUT);
  pinMode(muzzleLedPin, OUTPUT);
  pinMode(gunSpeaker, OUTPUT);
  pinMode(consoleLED0, OUTPUT);
  pinMode(consoleLED1, OUTPUT);
  pinMode(consoleLED2, OUTPUT);
  pinMode(consoleLED3, OUTPUT);
  slServo.attach(slServoPin);  // attaches the servo on pin 9 to the servo object 
  slServo.write(0);  // drive searchlight to home position
  gunServo.attach(gunServoPin);  // attaches the servo on pin 10 to the servo object 
  gunServo.write(0);  // drive gun to home position 
  for(int i = 1; i < 16; i++){ //self test of console lights
  console(i);
  delay(200);
 }
 console(0);
 randomSeed(millis());  //set random nr generator for gun interval
}
//= + = + = + = + = + = + = + = + = + = + = + = + = + = + = + = + = + =
//                          loop
//= + = + = + = + = + = + = + = + = + = + = + = + = + = + = + = + = + =
void loop() {
// Serial.println("top of loop doop");
// delay(100);
 float tCent = monitorTemp();
 float temputure = (tCent + 40.0) * (9.0 / 5.0) - 40.0;  // C to F conversion
 if(temputure > 100.0){
 float scaleTemp = round((temputure - 100.0) / 10.0);
 int intScaleTemp = scaleTemp;  // on 28 Aug 2016, the console display was showing temps that were
 //                                20-30 degrees higher than what I think they should have been.
 //                                Perhaps the problem is caused by feeding a FP number to the
 //                                Console function. Added this integer conversion on 28 Aug; 
 //                                if temps still seem high after this, subtract 25 from the 
 //                                float temputure statement above.
 //                                
 console(intScaleTemp);
 }
 //Serial.print("Temp:  ");
 //Serial.println(temputure); 
  int elevpulseWidth = pulseIn(searchlightsignal, HIGH); // reading the pulse width from ELEV channel
  if(elevpulseWidth == 0) elevpulseWidth = 1599;  // set pulsewidth to a neutral switch if zero
//Serial.print("Channel ELEV:     ");  
//Serial.println(elevpulseWidth);
  if(elevpulseWidth > 1800){ // Modified limits based on tests conducted 14 Sep 2016
    sloffFlag = false;
//    Serial.println("EV CP GT 1800");
//    delay(600);
  }
  if(elevpulseWidth < 1125){  // Modified limits based on tests conducted 14 Sep 2016
    sloffFlag = true;
//    Serial.println("EV CP LT 1125");
//    delay(600);
  }
//  int aux1pulseWidth = 1600;
    if(sloffFlag){
//  if(aux1pulseWidth > 1625 || aux1pulseWidth < 1200){
    digitalWrite(searchlightPin, LOW);  //if aux1 switch is off, turn off of LED
    angle = 0;  // reset parameters of slew routine
    slServo.write(angle);
    quad = 1;
//    Serial.println("EV CP 1  ");
//    delay(400);
    //Serial.println("sw off");
  }
    else{  // if switch is on, call searchlight routine
    //Serial.println("sw on");
        searchLight();
//        Serial.println("EV CP 2 ");
//        delay(400);
      }  
      
  gearpulseWidth = pulseIn(gunSignal, HIGH); // reading the pulse width from gear channel
//  gearpulseWidth = 1600;
//Serial.println(gearpulseWidth);
//delay(100);
  if(gearpulseWidth < 1600){  //if gear switch is off, ensure muzzle LED and speaker are off
    digitalWrite(muzzleLedPin, LOW);
    noTone(gunSpeaker);
    gunAngle = 0;  // reset parameters of gun slew routine
//    gunServo.write(gunAngle);
    gunQuad = 1;
//    Serial.println("gear sw off");
  }
  else{
    gunFire();
  }
}