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


#include <Bioloid/ax12.h>
#include <Bioloid/BioloidController.h>

#include <avr/pgmspace.h>
#include "scorpx.h"

/*
typedef struct
{
  
} servo_cmd_t;
*/

typedef struct
{
  uint8_t id;
  uint8_t led;
  uint8_t enabled;
  uint8_t moving;
  uint16_t position;
  uint16_t speed;
  uint16_t load;
} servo_state_t __attribute__ ((__packed__));

typedef struct
{
  uint8_t servoID;
  uint8_t cmdID;
  uint16_t value;
} scorpion_cmd_t __attribute__ ((__packed__));

typedef struct
{
  uint8_t cmdCount;
  scorpion_cmd_t* commands;
} scorpion_envelope_t __attribute__ ((__packed__));

//void* __dso_handle;

// Main controller instance.
BioloidController bioloid = BioloidController(115200);

const int   kServoCount = 2;
const uint8_t maxServos = 32;
uint8_t servoMap[maxServos];
uint8_t servoCount;
servo_state_t* servo_states;

//Square mount
/*
const int   kCenter   = 2048;
const int   kMinPan   = 1024;
const int   kMaxPan   = 3072;
const int   kMinTilt  = 1792;
const int   kMaxTilt  = 2304;
*/

//Diagonal mount
const int   kCenterPan   = 1560;
const int   kCenterTilt  = 2000;
const int   kMinPan   = 500;
const int   kMaxPan   = 2600;
const int   kMinTilt  = 2530;
const int   kMaxTilt  = 1650;

// Define servos center pose.
//Square
//const PROGMEM prog_uint16_t center_pose[] = {2, kCenter, kCenter};

//Diagonal
const PROGMEM uint16_t center_pose[] = {2, kCenterPan, kCenterTilt};

// Define max pan rotation.
const PROGMEM uint16_t pan_min_pose[] = {2, kMinPan, kCenterTilt};
const PROGMEM uint16_t pan_max_pose[] = {2, kMaxPan, kCenterTilt};


bool        moving[kServoCount];
int         servo_positions[kServoCount];
char 	    input_buffer[32];
uint8_t     input_count = 0;
void CheckVoltage();
void SetSpeed(int rpm);
void CenterServos();
void MenuOptions();
void RelaxServos();
void ScanServos();
void MoveTestBasic();
void GotoPose2(int pan, int tilt);
bool humanInterface;
void ListServos();
void AssignServoID(uint8_t src, uint8_t dest);
void updateServos();

////////////////////////////////////////////////////////////////////////////////
void setup()
{
  humanInterface = true; 
   pinMode(0, OUTPUT);  

   
  // Open terminal serial port.
  Serial.begin(115200);
  delay(500);   
  Serial.println("BOARD RESET\n"); 
  Serial.println("Serial Communication Established.");    

  //Populate the dynamic servo finder thing
  ListServos();

  //Set up the status structs
  servo_states = (servo_state_t*)malloc(sizeof(servo_state_t)*servoCount);
  
  // Check Lipo Battery Voltage.
  CheckVoltage();
  
  SetSpeed(20); 
  // Center servos.
  CenterServos();


  // Show menu options.  
  MenuOptions();
}

void printServoState(uint8_t index)
{
  Serial.print(servo_states[index].id, DEC);
  Serial.print(":");
  Serial.print(servo_states[index].led, DEC);
  Serial.print(" ");
  Serial.print(servo_states[index].position, DEC);
  Serial.print(" ");
  Serial.print(servo_states[index].load, DEC);
  Serial.println("");
}

void doMachineCommand(scorpion_cmd_t *cmd)
{
  switch (cmd->cmdID)
    {
    case SX_NOOP:
      break;
    case SX_SET_LED:
      ax12SetRegister(cmd->servoID, AX_LED, cmd->value);
      break;
    case SX_SET_ENABLED:
      ax12SetRegister(cmd->servoID, AX_TORQUE_ENABLE, cmd->value);
      break;
    case SX_SET_POS:
      ax12SetRegister2(cmd->servoID, AX_GOAL_POSITION_L, cmd->value);
      break;
    case SX_SET_SPEED:
      ax12SetRegister2(cmd->servoID, AX_GOAL_SPEED_L, cmd->value);
      break;
    case SX_EXIT_MACHINE:
    default:
      humanInterface = true;
      MenuOptions();
      break;
    }
}

void processMachine(char* input_buffer)
{
  //Decode a specific command set based on a sequence of space-separated integers
  scorpion_envelope_t thisCmd;
  thisCmd.cmdCount = atoi(strtok(input_buffer, " ")); //skip over set command
  thisCmd.commands = new scorpion_cmd_t[thisCmd.cmdCount];
  for (int i=0; i < thisCmd.cmdCount; i++)
    {
      thisCmd.commands[i].servoID = atoi(strtok(0, " "));
      thisCmd.commands[i].cmdID = atoi(strtok(0, " "));
      thisCmd.commands[i].value = atoi(strtok(0, " "));
      doMachineCommand(&thisCmd.commands[i]);
    }
 
  updateServos();
  
  //Send the number of servos first
  Serial.write((uint8_t*)&servoCount, sizeof(servoCount));

  //And a record for each servo
  //printServoState(i);
  Serial.write((uint8_t*)servo_states, sizeof(servo_state_t)*servoCount);
  
  delete[] thisCmd.commands;
}




////////////////////////////////////////////////////////////////////////////////
void loop()
{
  char c = Serial.read();
  if (c != -1) {

   input_buffer[input_count] = c;
   input_buffer[++input_count] = 0; //null terminate
    
    if (c == '\r') { //replace with \r to use 0xD as a terminator

      if (!humanInterface)
	{
	  processMachine(input_buffer);
	  // Reset input string.
	  input_buffer[0] = 0;
	  input_count = 0;
	  return;
	}
      
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
          Serial.println("===================================");
	  Serial.println("Scanning Servo Positions");
	  Serial.println("===================================");
	  ScanServos();
	  for (int i=1; i<=kServoCount; i++)
	  {
	    Serial.print("Servo ID: ");
	    Serial.println(i);
	    Serial.print("Servo Position: ");
	    Serial.println(servo_positions[i-1]);
	  }


      }
    
      if (input == 5) {
        CenterServos();
        MoveTestBasic();
      }
      
      if (input == 6) {
        //From http://arduino.stackexchange.com/questions/1013/how-do-i-split-an-incoming-string
        // Read each command pair 
	int pan = atoi(strtok(&input_buffer[1], " ")); //skip over set command
	int tilt = atoi(strtok(0, " ")); 
	Serial.print("Moving to pan:");
	Serial.print(pan, DEC);
	Serial.print(", tilt:");
	Serial.println(tilt,DEC);
	SetSpeed(20);  
        GotoPose2(pan, tilt);
      }

      if (input == 7)
	{
	  //Move to a specific pan/tilt value over the specified time interval
	  //From http://arduino.stackexchange.com/questions/1013/how-do-i-split-an-incoming-string
        // Read each command pair 
	int pan = atoi(strtok(&input_buffer[1], " ")); //skip over set command
	int tilt = atoi(strtok(0, " "));
	int speed = atoi(strtok(0, " "));
	
	Serial.print("Moving to pan:");
	Serial.print(pan, DEC);
	Serial.print(", tilt:");
	Serial.print(tilt,DEC);
	Serial.print(",speed:");
	Serial.println(speed, DEC);
	SetSpeed(speed);  
        GotoPose2(pan, tilt);
	}

      if (input == 8)
	{
	  ListServos();
	}

      if (input == 9)
	{
	  int src = atoi(strtok(&input_buffer[1], " ")); //skip over set command
	  int dest = atoi(strtok(0, " "));
	  AssignServoID(src,dest);
	}
      
      if (input == 10)
	{
	  Serial.println("Setting machine interface");
	  humanInterface = false;
	}
      
      if (input > 10) {
	Serial.println("No commands defined!");
      }
    
      // Show menu.
      if ((input != -1) && (humanInterface == true)) {
        MenuOptions();  
      }
      
      // Reset input string.
      input_buffer[0] = 0;
      input_count = 0;
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
    Serial.println("5) Basic Movement Test");   
    Serial.println("6) Goto <pan> <tilt>");
    Serial.println("7) Goto <pan> <tilt> <speed>");
    Serial.println("8) List servos");
    Serial.println("9) Assign servo id <src> to <dest>");
    Serial.println(""); 
    Serial.println("10) Switch to machine interface");                
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
  uint8_t servo_id = 1;
  Serial.println("===================================");
  Serial.println("Relaxing Servos.");
  Serial.println("===================================");
  while (servo_id <= kServoCount) {
    Relax(servo_id);
    servo_id++;
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

  
  // Setup for interpolation from current->next over 1/2 a second.
  bioloid.interpolateSetup(1000);      
  while (bioloid.interpolating > 0) {  // do this while we have not reached our new pose
    bioloid.interpolateStep();         // move servos, if necessary. 
    delay(1);
  }
}

void AssignServoID(uint8_t src, uint8_t dest)
{
  Serial.println("Not implemented yet: Change servo ID <src> to <dest>");
}

void ListServos()
{
  servoCount = 0;
  uint8_t servo_id = 1; 
  int curPos;
  
  memset(servoMap, 0, maxServos*sizeof(uint8_t));
  
  while (servo_id <= maxServos) {
    curPos =  ax12GetRegister(servo_id, 36, 2);
    //Returns -1 on not found -> needs to have a signed type
    if (curPos > 0)
      {
	
	servoMap[servoCount] = servo_id;
	servoCount++;
      }
    
    servo_id++;
  }
  Serial.print("Found ");
  Serial.print(servoCount, DEC);
  Serial.print(" servos:\r\n");
  Serial.println("Number: ID");
  for (uint8_t i=0; i<servoCount; i++)
    {
      Serial.print(i, DEC);
      Serial.print(":");
      Serial.println(servoMap[i], DEC);
    }
  Serial.println("");
}

uint16_t getPosition(uint8_t servoID)
{
  return ax12GetRegister(servoID, AX_PRESENT_POSITION_L, 2);
}

uint16_t getSpeed(uint8_t servoID)
{
  return ax12GetRegister(servoID, AX_PRESENT_SPEED_L, 2);
}

uint16_t getLoad(uint8_t servoID)
{
  return ax12GetRegister(servoID, AX_PRESENT_LOAD_L, 2);
}

uint8_t getMoving(uint8_t servoID)
{
  return ax12GetRegister(servoID, AX_MOVING, 1);
}

uint8_t getEnabled(uint8_t servoID)
{
  return ax12GetRegister(servoID, AX_TORQUE_ENABLE, 1);
}

uint8_t getLED(uint8_t servoID)
{
  return ax12GetRegister(servoID, AX_LED, 1);
}

void getState(uint8_t index)
{
  uint8_t servoID = servoMap[index];
  servo_states[index].id = servoID;
  servo_states[index].led = getLED(servoID);
  servo_states[index].enabled = getEnabled(servoID);
  servo_states[index].moving = getMoving(servoID);
  servo_states[index].position = getPosition(servoID);
  servo_states[index].speed = getSpeed(servoID);
  servo_states[index].load = getLoad(servoID);
}

void updateServos()
{
  for (uint8_t i = 0; i<servoCount; i++)
    {
      getState(i);
    }
}

////////////////////////////////////////////////////////////////////////////////
void ScanServos()
{
  uint8_t servo_id = 1; 
  int curPos = -1;
  while (servo_id <= kServoCount) {
    curPos =  ax12GetRegister(servo_id, 36, 2);

    if (curPos <= 0) {
      Serial.print("ERROR! Servo ID: ");
      Serial.print(servo_id);
      Serial.println(" not found. Please check connection and verify correct ID is set.");
    }
    else
    {
    	servo_positions[servo_id-1] = curPos;
    }
    servo_id++;
  }
}


////////////////////////////////////////////////////////////////////////////////
void MoveTestBasic()
{
	uint8_t servo_id;
	uint16_t cmd_position;

  Serial.println("===================================");
  Serial.println("Initializing Movement Test");  
  Serial.println("===================================");

  
  ///----- Servo ID #1 (Pan)
  Serial.print("Moving Servo ID: 1");

  // Initialize.
  ScanServos();
  servo_id = 1;

  while (servo_positions[servo_id-1] >= kMinPan) {  
    SetPosition(servo_id, --servo_positions[servo_id-1]);
    ScanServos();
    delay(1);
  }

  Serial.print("At minimum pan");
  while (servo_positions[servo_id-1] <= kMaxPan) {  
    SetPosition(servo_id, --servo_positions[servo_id-1]);
    ScanServos();
    delay(1);
  }
 Serial.print("At max pan");

  ///----- Servo ID #2 (Tilt)
  Serial.print("Moving Servo ID: 2");

  // Initialize.
  servo_id = 2;

  while (servo_positions[servo_id-1] >= kMinTilt) {  
    SetPosition(servo_id, --servo_positions[servo_id-1]);
    ScanServos();
    delay(1);
  }

  while (servo_positions[servo_id-1] <= kMaxTilt) {  
    SetPosition(servo_id, --servo_positions[servo_id-1]);
    ScanServos();
    delay(1);
  }

  Serial.println("===================================");
}


////////////////////////////////////////////////////////////////////////////////
void MoveSwing(int time)
{
  Serial.println("===================================");
  Serial.println("Initializing Pan Swing");  
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

    
  ///----- Set goal pose as max_pan.
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


////////////////////////////////////////////////////////////////////////////////
void SetSpeed(int rpm)
{
  int temp;
  int length = 4 + (kServoCount * 3);   // 3 = id + pos(2byte)
  int checksum = 254 + length + AX_SYNC_WRITE + 2 + AX_GOAL_SPEED_L;
  setTXall();
  ax12write(0xFF);
  ax12write(0xFF);
  ax12write(0xFE);
  ax12write(length);
  ax12write(AX_SYNC_WRITE);
  ax12write(AX_GOAL_SPEED_L);
  ax12write(2);
  for (int ii=0; ii < kServoCount; ii++) {
    if (ii == 0) {
      temp = rpm;
    } else {
      temp = rpm/2;
    }
    checksum += (temp&0xff) + (temp>>8) + (ii+1);
    ax12write(ii+1);
    ax12write(temp&0xff);
    ax12write(temp>>8);
  } 
  ax12write(0xff - (checksum % 256));
  setRX(0);
}


////////////////////////////////////////////////////////////////////////////////
void GotoPose(const unsigned int* pose)
{
  // Parse pose into individual servo positions.
  for (int ii=0; ii < kServoCount; ii++) {
    servo_positions[ii] = pgm_read_word_near(pose+1+ii) << BIOLOID_SHIFT;   
  }
  Serial.println("Servo position1: ");
  Serial.println(servo_positions[0]);
  Serial.println(servo_positions[0] >> BIOLOID_SHIFT);
  Serial.println("Servo position2: ");
  Serial.println(servo_positions[1]);
  Serial.println(servo_positions[1] >> BIOLOID_SHIFT);
  
  int temp;
  int length = 4 + (kServoCount * 3);   // 3 = id + pos(2byte)
  int checksum = 254 + length + AX_SYNC_WRITE + 2 + AX_GOAL_POSITION_L;
  setTXall();
  ax12write(0xFF);
  ax12write(0xFF);
  ax12write(0xFE);
  ax12write(length);
  ax12write(AX_SYNC_WRITE);
  ax12write(AX_GOAL_POSITION_L);
  ax12write(2);
  for (int ii=0; ii < kServoCount; ii++) {
    temp = servo_positions[ii] >> BIOLOID_SHIFT;
    checksum += (temp&0xff) + (temp>>8) + (ii+1);
    ax12write(ii+1);
    ax12write(temp&0xff);
    ax12write(temp>>8);
  } 
  ax12write(0xff - (checksum % 256));
  setRX(0);
}

////////////////////////////////////////////////////////////////////////////////
void GotoPose2(int pan, int tilt)
{
  int temp;
  int length = 4 + (kServoCount * 3);   // 3 = id + pos(2byte)
  int checksum = 254 + length + AX_SYNC_WRITE + 2 + AX_GOAL_POSITION_L;
  setTXall();
  ax12write(0xFF);
  ax12write(0xFF);
  ax12write(0xFE);
  ax12write(length);
  ax12write(AX_SYNC_WRITE);
  ax12write(AX_GOAL_POSITION_L);
  ax12write(2);
  for (int ii=0; ii < kServoCount; ii++) {
    if (ii==0) {
      temp = pan;
    } else {
      temp = tilt;
    }
    checksum += (temp&0xff) + (temp>>8) + (ii+1);
    ax12write(ii+1);
    ax12write(temp&0xff);
    ax12write(temp>>8);
  } 
  ax12write(0xff - (checksum % 256));
  setRX(0);
}


////////////////////////////////////////////////////////////////////////////////
void MoveSwingSmooth(int rpm)
{
  Serial.println("===================================");
  Serial.println("Initializing Pan Swing");  
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

    
  ///----- Set speed and final pose.
  SetSpeed(rpm);    
  GotoPose(pan_max_pose);

  Serial.println("===================================");
}

