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


// Define servos center pose.
PROGMEM prog_uint16_t center_pose[] = {2, 2048, 2048};

BioloidController bioloid = BioloidController(1000000);

const int   kServoCount = 2;

int         servo_id;
int         servo_position;
boolean     IDCheck;
boolean     RunCheck;


////////////////////////////////////////////////////////////////////////////////
void setup()
{
   pinMode(0, OUTPUT);  
   
   // Initialize variables.
   servo_id = 1;
   servo_position = 0;
   IDCheck = 1;
   RunCheck = 0;
   
  // Open serial port.
  Serial.begin(9600);
  delay(500);   
  Serial.println("###########################");    
  Serial.println("Serial Communication Established.");    

  // Check Lipo Battery Voltage.
  CheckVoltage();

  CenterServos();
  
  //Scan Servos, return position.
  ScanServos();
  
  MoveTest();
  
  
  MenuOptions();
 
  RunCheck = 1;
}


////////////////////////////////////////////////////////////////////////////////
void loop()
{
  int input = Serial.read();

  switch (input) {

  case '1':    
    CheckVoltage();
    break;

  case '2':    
    RelaxServos();
    break;

  case '3':    
    CenterServos();
    break; 

  case '4':    
    ScanServos();
    break;     

  case '5':
    CenterServos();
    MoveTest();
    break;
 
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

  if (RunCheck == 1) {
    MenuOptions();
  }
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

  if (RunCheck == 1) {
    MenuOptions();
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

  if (RunCheck == 1) {
    MenuOptions();
  }
}


////////////////////////////////////////////////////////////////////////////////
void ScanServos()
{
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
  
  if (RunCheck == 1) {
    MenuOptions();
  }
}


////////////////////////////////////////////////////////////////////////////////
void MoveTest()
{
  Serial.println("===================================");
  Serial.println("Initializing Movement Test");  
  Serial.println("===================================");
  delay(500);  
  
  servo_id = 1;
  servo_position = 2048;
  while (servo_id <= kServoCount) {
    Serial.print("Moving Servo ID: ");
    Serial.println(servo_id);  

    while (servo_position >= 1024) {  
      SetPosition(servo_id, servo_position);
      servo_position = servo_position--;
      delay(2);
    }

    while (servo_position <= 2048) {  
      SetPosition(servo_id, servo_position);
      servo_position = servo_position++;
      delay(2);
    }

    // Iterate to next servo ID.
    servo_id = (servo_id++) % kServoCount;  // Overflow guard.
  }

  Serial.println("===================================");

  if (RunCheck == 1) {
    MenuOptions();
  }
}


