#include <Servo.h>

/*-----------------ULTRASONIC SENSOR-----------------------------------------------------------------------------*/
const int pwPin = 7;    //sensor connects to digital pin 7
long pulse, inches, cm; //variables needed to store values

/*-----------------Phototransmitter Sensor-----------------------------------------------------------------------*/
const int sensorPin = 0; //analog pin 0

/*-----------------FSRs------------------------------------------------------------------------------------------*/
const int fsrPin1 = 1, fsrPin2 = 2, fsrPin3 = 3, fsrPin4 = 4; //analog pin for each FSR(index, middle, ring, thumb)
int fsrReading1, fsrReading2, fsrReading3, fsrReading4;       // the analog reading from the FSR resistor divider

/*-----------------Servos----------------------------------------------------------------------------------------*/
Servo thumb_servo, finger_servo, extend_servo;
int thumb_position = 0, finger_position = 0, forearm_position = 10;
boolean thumb = true, finger = true, stopped = false;
int thumb_stop, finger_stop;
int indexMiddle;
int indexRing;
int middleRing;

/*-----------------Buttons---------------------------------------------------------------------------------------*/
const int letGoButton = 2;    // reset button pin 2
const int outButton = 3;      // out button pin 3
const int inButton = 4;       // in button pin 4
int letGoButtonState = LOW, outButtonState = LOW, inButtonState = LOW;
boolean letGo = false;

/*-----------------DC Motor--------------------------------------------------------------------------------------
int enablePin = 11;
int in1Pin = 6;
int in2Pin = 5;               //digital pin 5
int potPin = 5;               //analog pin A5
int speed;
boolean reverse = false;*/


void setup()
{
  thumb_servo.attach(9);    //thumb servo to digital pin 9
  thumb_servo.write(10);     //set angle to 10
  finger_servo.attach(10);  //finger servo to digital pin 10
  finger_servo.write(10);
  extend_servo.attach(11);
  extend_servo.write(10);
  
  pinMode(letGoButton, INPUT_PULLUP);  
  pinMode(outButton, INPUT_PULLUP);  
  pinMode(inButton, INPUT_PULLUP);
  
  //pinMode(in1Pin, OUTPUT);
  //pinMode(in2Pin, OUTPUT);
  //pinMode(enablePin, OUTPUT);
  //analogWrite(enablePin, 0);
  
  //This opens up a serial connection to shoot the results back to the PC console
  Serial.begin(9600);
}

void loop()
{
/*-----------------SERVO-RUN ARM EXTENSION-----------------------------------------------------------------------*/
  outButtonState = digitalRead(outButton);
  inButtonState = digitalRead(inButton);
  
  if(outButtonState == LOW && forearm_position <= 170){
    Serial.println("Red");
    extend_servo.write(forearm_position);
    Serial.print("Arm in");
    Serial.println(forearm_position);
    forearm_position--;
    delay(50);
  }
    
  else if(inButtonState == LOW && forearm_position >= 0){
    Serial.println("Green");
    extend_servo.write(forearm_position);
    Serial.print("Arm out");
    Serial.println(forearm_position);
    forearm_position++;
    delay(50);
  }
  
/*-----------------ULTRASONIC SENSOR-----------------------------------------------------------------------------

  pinMode(pwPin, INPUT);
  //Used to read in the pulse that is being sent by the MaxSonar device.
  //Pulse Width representation with a scale factor of 147 uS per Inch.

  pulse = pulseIn(pwPin, HIGH);

  //147uS per inch
  inches = pulse / 147;

  //change inches to centimetres
  cm = inches * 2.54;
  
  Serial.print("Ultrasonic Sensor Distance: ");
  Serial.print(cm);
  Serial.println("cm");
  */

/*-----------------Phototransmitter Sensor-----------------------------------------------------------------------*/

  int val = analogRead(sensorPin);
  float proximityV = (float)val * 5.0 / 1023.0;
  Serial.print("Phototransmitter Sensor Voltage: ");
  Serial.println(proximityV);
  
/*-----------------Servos and FSRs-------------------------------------------------------------------------------*/
  
  if(proximityV >= 4.8){ //if the phototransmitter is detecting a close object
    fsrReading1 = analogRead(fsrPin1);
    fsrReading2 = analogRead(fsrPin2);
    fsrReading3 = analogRead(fsrPin3);
    fsrReading4 = analogRead(fsrPin4);
    //some checks on FSR values
    Serial.print("FSR 1= ");
    //Serial.print(fsrReading1);     // the raw analog reading
    // We'll have a few threshholds, qualitatively determined
    if (fsrReading1 < 10) {
      Serial.println(" - No pressure");
    } 
    else if (fsrReading1 < 100) {
      Serial.println(" - Light touch");
    } 
    else if (fsrReading1 < 200) {
      Serial.println(" - Light squeeze");
    } 
    else if (fsrReading1 < 500) {
      Serial.println(" - Medium squeeze");
    } 
    else {
      Serial.println(" - Big squeeze");
    }
    
    Serial.print("FSR 2= ");
    //Serial.print(fsrReading2);     // the raw analog reading
   
    if (fsrReading2 < 10) {
      Serial.println(" - No pressure");
    } 
    else if (fsrReading2 < 100) {
      Serial.println(" - Light touch");
    } 
    else if (fsrReading2 < 200) {
      Serial.println(" - Light squeeze");
    } 
    else if (fsrReading2 < 500) {
      Serial.println(" - Medium squeeze");
    } 
    else {
      Serial.println(" - Big squeeze");
    }
    
    Serial.print("FSR 3= ");
    //Serial.print(fsrReading3);     // the raw analog reading
   
    if (fsrReading3 < 10) {
      Serial.println(" - No pressure");
    } 
    else if (fsrReading3 < 100) {
      Serial.println(" - Light touch");
    } 
    else if (fsrReading3 < 200) {
      Serial.println(" - Light squeeze");
    } 
    else if (fsrReading3 < 500) {
      Serial.println(" - Medium squeeze");
    } 
    else {
      Serial.println(" - Big squeeze");
    }
    
    Serial.print("FSR 4= ");
    //Serial.print(fsrReading4);     // the raw analog reading
  
    if (fsrReading4 < 10) {
      Serial.println(" - No pressure");
    } 
    else if (fsrReading4 < 100) {
      Serial.println(" - Light touch");
    } 
    else if (fsrReading4 < 200) {
      Serial.println(" - Light squeeze");
    } 
    else if (fsrReading4 < 500) {
      Serial.println(" - Medium squeeze");
    } 
    else {
      Serial.println(" - Big squeeze");
    }
    
    indexMiddle = (fsrReading1+fsrReading2)/2; //average reading between two fingers
    indexRing = (fsrReading1+fsrReading3)/2;
    middleRing = (fsrReading2+fsrReading3)/2;
    
    if(!stopped){ //if the servos have not both stopped
      for(int angle = 10; angle <= 170; angle++){
        fsrReading1 = analogRead(fsrPin1);
        fsrReading2 = analogRead(fsrPin2);
        fsrReading3 = analogRead(fsrPin3);
        fsrReading4 = analogRead(fsrPin4);
        
        indexMiddle = (fsrReading1+fsrReading2)/2;
        indexRing = (fsrReading1+fsrReading3)/2;
        middleRing = (fsrReading2+fsrReading3)/2;
        
        if(thumb){  //allow the process of turning the servo to occur if the FSR doesnt sense anything
          if(fsrReading4<400){ //FSR detects nothing]
            thumb_position = angle;
            thumb_servo.write(thumb_position);
            Serial.print("Thumb move");
            Serial.println(thumb_position);
            delay(50);
          }
          else{
            thumb_position = angle;
            thumb_stop = thumb_position;
            thumb = false; //if detects something, servo turning loop stops where it is completely
          }
        }
        
        if(finger){ //same logic repeated for finger
          if(indexMiddle<400 && indexRing<400 && middleRing<400){ //if not sufficient pressure is detected
            finger_position = angle;
            finger_servo.write(finger_position);
            Serial.print("Finger move");
            Serial.println(finger_position);
            delay(50);
          }
          else{
            finger_position = angle;
            finger_stop = finger_position;
            finger = false;
            
            Serial.print("IndexMiddle: "); //print final values as a check if my math is right
            Serial.println(indexMiddle);
            Serial.print("IndexRing: ");
            Serial.println(indexRing);
            Serial.print("MiddleRing: ");
            Serial.println(middleRing);
            //stop the for loop in charge of turning the servo
          }
        }
        
        if(!thumb && !finger){  //if both thumb and finger are stopped
          
          stopped = true;       //tell controller to not run the servos again
          break;                //and leave the for loop
        }
        
        else if(angle == 170){
          thumb_stop = thumb_position;
          finger_stop = finger_position;
          stopped = true;       //prevents hand from repeatedly opening and closing (need to press let go button)
          break;
        }
        
        delay(50);
      }
    }
    
    // read the state of the pushbutton value:
    letGoButtonState = digitalRead(letGoButton);
    
    if(letGoButtonState == LOW){ //if being pressed
      letGo = true;
      Serial.println("Orange");
    }
    
    // check if the pushbutton is pressed. If it is, the letGoButtonState is LOW:
    while(letGo){
      if(thumb_stop >= 10){
        thumb_servo.write(thumb_stop);
        thumb_stop--;
        Serial.print("Thumb reopen");
        Serial.println(thumb_stop);
        delay(50);
      }
       
      if(finger_stop >= 10){
        finger_servo.write(finger_stop);
        finger_stop--;
        Serial.print("Finger reopen");
        Serial.println(finger_stop);
        delay(50);
      }
      
      if(thumb_stop < 10 && finger_stop < 10){
        letGo = false;          //resets logic variables and allows hand to grab again
        stopped = false;
      }
    }  
  }
  
/*-----------------DC Motor--------------------------------------------------------------------------------------

  speed = analogRead(potPin) / 4;
  
  outButtonState = digitalRead(outButton);
  inButtonState = digitalRead(inButton);
    
  if(inButtonState == LOW){
    digitalWrite(in1Pin, !reverse);
    digitalWrite(in2Pin, reverse);
    analogWrite(enablePin, speed);
  }
    
  else if(outButtonState == LOW){
    digitalWrite(in1Pin, reverse);
    digitalWrite(in2Pin, !reverse);
    analogWrite(enablePin, speed);
  }*/
  
  delay(10);
}
