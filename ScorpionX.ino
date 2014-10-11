/**************************************************************************
Copyright 2014  Juan M. Falquez (University of Colorado - Boulder)
                Based on Trossen Robotics' ArbotiX Library

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
***************************************************************************/


#include <ax12.h>
#include <avr/pgmspace.h>

#include <BioloidController.h>

// Main controller instance.
BioloidController bioloid = BioloidController(1000000);

const int   kServoCount = 2;

const int   kCenter   = 2048;
const int   kMinPan   = 512;
const int   kMaxPan   = 3584;
const int   kMinTilt  = 1024;
const int   kMaxTilt  = 3072;

// Define servos center pose.
PROGMEM prog_uint16_t center_pose[] = {2, kCenter, kCenter};
PROGMEM prog_uint16_t pan_min_pose[] = {2, kMinPan, kCenter};
PROGMEM prog_uint16_t pan_max_pose[] = {2, kMaxPan, kCenter};

int         servo_id;
int         servo_position;
String      input_string;
char        input_buffer[10];


////////////////////////////////////////////////////////////////////////////////
void setup()
{
   pinMode(0, OUTPUT);  
   
   // Initialize variables.
   servo_id = 1;
   servo_position = 0;
   
  // Open serial port.
  Serial.begin(9600);
  delay(500);   
  Serial.println("==================================="); 
  Serial.println("Serial Communication Established.");    

  // Check Lipo Battery Voltage.
  CheckVoltage();

  // Center servos.
  CenterServos();
    
  // Show menu options.  
  MenuOptions();
}


////////////////////////////////////////////////////////////////////////////////
void loop()
{
  char c = Serial.read();

  if (c != -1) {
    
    input_string += c;
        
    if (c == '.') {
      input_string.toCharArray(input_buffer, 10);
      int input = atoi(input_buffer);

      if (input == 1) {
        CheckVoltage();
      }
    
      if (input == 2) {
        RelaxServos();
      }
    
      if (input == 3) {
        CenterServos();
      }
    
      if (input == 4) {
        ScanServos();
      }
    
      if (input == 5) {
        CenterServos();
        MoveTestBasic();
      }
    
      if (input > 10) {
        MoveTestInterp(input);
      }
    
      // Show menu.
      if (input != -1) {
        MenuOptions();  
      }
      
      // Reset input string.
      input_string = "";
    }
  }
}


////////////////////////////////////////////////////////////////////////////////
void MenuOptions()
{  
    Serial.println(""); 
    Serial.println("==================================="); 
    Serial.println("MENU OPTIONS");     
    Serial.println("-----------------------------------"); 
    Serial.println("1) Check System Voltage");   
    Serial.println("2) Relax Servos");            
    Serial.println("3) Center Servos");    
    Serial.println("4) Scan Servos Position");        
    Serial.println("5) Perform Movement Test");                
    Serial.println("===================================");
    Serial.println(""); 
}


////////////////////////////////////////////////////////////////////////////////
void CheckVoltage()
{  
  // Wait, then check the voltage (LiPO safety)
  float voltage = (ax12GetRegister (1, AX_PRESENT_VOLTAGE, 1)) / 10.0;
  
  Serial.println("===================================");
  Serial.print ("System Voltage: ");
  Serial.print (voltage);
  Serial.println (" volts.");
  
  if (voltage < 10.0) {
    Serial.println("Voltage levels below 10v, please charge battery.");
    while (1) { }
  }
  
  if (voltage > 10.0) {
    Serial.println("Voltage levels nominal.");
  }
  
  Serial.println("===================================");
}


////////////////////////////////////////////////////////////////////////////////
void RelaxServos()
{
  servo_id = 1;
  Serial.println("===================================");
  Serial.println("Relaxing Servos.");
  Serial.println("===================================");
  while (servo_id <= kServoCount) {
    Relax(servo_id);
    servo_id = (servo_id++) % kServoCount;  // Overflow guard.
    delay(50);
  }
}


////////////////////////////////////////////////////////////////////////////////
void CenterServos()
{
  // Recommended pause.
  delay(100);                    

  // Load the pose from FLASH, into the nextPose buffer.
  bioloid.loadPose(center_pose);
 
  // Read in current servo positions to the curPose buffer. 
  bioloid.readPose();            

  Serial.println("===================================");
  Serial.println("Centering Servos.");
  Serial.println("===================================");
  delay(1000);
  
  // Setup for interpolation from current->next over 1/2 a second.
  bioloid.interpolateSetup(1000);      
  while (bioloid.interpolating > 0) {  // do this while we have not reached our new pose
    bioloid.interpolateStep();         // move servos, if necessary. 
    delay(1);
  }
}


////////////////////////////////////////////////////////////////////////////////
void ScanServos()
{
  boolean     IDCheck = true;
  
  servo_id = 1;  
  Serial.println("===================================");
  Serial.println("Scanning Servo Positions");
  Serial.println("===================================");
      
  while (servo_id <= kServoCount) {
    servo_position =  ax12GetRegister(servo_id, 36, 2);
    Serial.print("Servo ID: ");
    Serial.println(servo_id);
    Serial.print("Servo Position: ");
    Serial.println(servo_position);
    
    if (servo_position <= 0) {
      Serial.print("ERROR! Servo ID: ");
      Serial.print(servo_id);
      Serial.println(" not found. Please check connection and verify correct ID is set.");
      IDCheck = 0;
    }
  
    servo_id = (servo_id++) % kServoCount;  // Overflow guard.
    delay(1000);
  }
  
  if (IDCheck == 0) {
    Serial.println("ERROR! Servo ID(s) are missing from Scan. Please check connection and verify correct ID is set.");
  } else {
    Serial.println("All servo IDs present.");
  }

  Serial.println("===================================");
}


////////////////////////////////////////////////////////////////////////////////
void MoveTestBasic()
{
  Serial.println("===================================");
  Serial.println("Initializing Movement Test");  
  Serial.println("===================================");
  delay(500);  
  
  ///----- Servo ID #1 (Pan)
  Serial.print("Moving Servo ID: 1");

  // Initialize.
  servo_id = 1;
  servo_position = 2048;

  while (servo_position >= kMinPan) {  
    SetPosition(servo_id, servo_position);
    servo_position = servo_position--;
    delay(1);
  }

  while (servo_position <= kMaxPan) {  
    SetPosition(servo_id, servo_position);
    servo_position = servo_position++;
    delay(1);
  }


  ///----- Servo ID #2 (Tilt)
  Serial.print("Moving Servo ID: 2");

  // Initialize.
  servo_id = 2;
  servo_position = 2048;

  while (servo_position >= kMinTilt) {  
    SetPosition(servo_id, servo_position);
    servo_position = servo_position--;
    delay(1);
  }

  while (servo_position <= kMaxTilt) {  
    SetPosition(servo_id, servo_position);
    servo_position = servo_position++;
    delay(1);
  }

  Serial.println("===================================");
}


////////////////////////////////////////////////////////////////////////////////
void MoveTestInterp(int time)
{
  Serial.println("===================================");
  Serial.println("Initializing Pan");  
  Serial.println("===================================");
  delay(500);  
  
  ///----- Set initial pose.
  // Load the pose from FLASH, into the nextPose buffer.
  bioloid.loadPose(pan_min_pose);
 
  // Read in current servo positions to the curPose buffer. 
  bioloid.readPose();            
  
  // Setup for interpolation from current->next over 1/2 a second.
  bioloid.interpolateSetup(3000);      
  while (bioloid.interpolating > 0) {  // do this while we have not reached our new pose
    bioloid.interpolateStep();         // move servos, if necessary. 
    delay(1);
  }

    
  ///----- Reset servo position to min pan.
  // Load the pose from FLASH, into the nextPose buffer.
  bioloid.loadPose(pan_max_pose);
 
  // Read in current servo positions to the curPose buffer. 
  bioloid.readPose();            
  
  // Setup for interpolation from current->next over 1/2 a second.
  bioloid.interpolateSetup(time);      
  while (bioloid.interpolating > 0) {  // do this while we have not reached our new pose
    bioloid.interpolateStep();         // move servos, if necessary. 
    delay(1);
  }


  Serial.println("===================================");
}
