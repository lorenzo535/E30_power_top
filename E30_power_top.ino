

#if ARDUINO >= 100 
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include <Streaming.h>
#include "Switch.h"
#include "mystructs.h"

#define  LidMotorSwitch1 6  //S3
#define  LidMotorSwitch2 9  //S4

#define  TopMotorSwitch1 4  //S1
#define  TopMotorSwitch2 5  //S2

#define ButtonRoofOpen 8
#define ButtonRoofClose 7

#define CLOCKWISE  1
#define STOP	0
#define COUNTERCLOCKWISE 2

#define CURRENT_AVERAGING_STEPS  8
#define CURRENT_RANGE 20
#define MAX_CURRENT 20

#define MV_TO_AMP (CURRENT_RANGE  )/ 2500
#define MAX_MOTION_TIME_MS 	25000	//25 seconds
// ROOF OPENING SEQUENCE STATES
#define OP_TOP_UNLOCKED_AND_LIFTED	2
#define OP_TENSION_BOW_RAISING  	6
#define OP_TENSION_BOW_ALL_UP       	14
#define OP_COVER_UNLOCKED		12
#define OP_COVER_OPEN    		13
#define OP_TENSION_BOW_LOWERING	       5
#define OP_TOP_IN_COMPARTMENT	      1
#define OP_COVER_CLOSING		0
#define OP_COVER_LOCKED               2


//ROOF CLOSING SEQUENCE STATES
#define CL_COVER_LOCKED_STOWED  2
#define CL_COVER_UNLOCKED	0
#define CL_COVER_OPENED		1
#define CL_RAISING_TOP		5
#define CL_TENSION_BOW_ALL_UP	13
#define CL_COVER_CLOSING	12
#define CL_COVER_LOCKED	        14
#define CL_TENSION_BOW_LOWERING	6
#define CL_TENSION_BOW_DOWN	2


#define COMMAND_OPEN 1
#define COMMAND_IDLE 0
#define COMMAND_CLOSE 2
#define COMMAND_AUTO_OPEN 3
#define COMMAND_AUTO_CLOSE 4
#define COMMAND_AUTO COMMAND_AUTO_OPEN
Switch InRoofOpen= Switch (ButtonRoofOpen, INPUT,LOW);
Switch InRoofClose= Switch(ButtonRoofClose, INPUT,LOW);
Switch InputSW1= Switch (TopMotorSwitch1, INPUT,LOW);
Switch InputSW2= Switch (TopMotorSwitch2, INPUT,LOW);
Switch InputSW3= Switch (LidMotorSwitch1, INPUT,HIGH);
Switch InputSW4= Switch (LidMotorSwitch2, INPUT,HIGH);
int SW1 = 0;
int SW2 = 0;
int SW3 = 0;
int SW4 = 0;
int display_switches = 0;

int pin[6];
int _pin[6];
int i;
int current_av_steps;
OutputCmd opening_state_cmds[16];
OutputCmd closing_state_cmds[16];
int current_state, old_state;
int  display_motor_command = 0;
int current_command, old_command;
float raw_current[CURRENT_AVERAGING_STEPS];
unsigned long motion_start_time;
int manual_commands = 0;
int manual_counter = 1;
int OldRoofClose, OldRoofOpen;

void setup() {
//Initialise board
   pinMode(14, OUTPUT);
   pinMode(15, OUTPUT);
   pinMode(16, OUTPUT);
   pinMode(10, OUTPUT);

  /* pinMode(4, INPUT);
   pinMode(5, INPUT);
   pinMode(6, INPUT);
   pinMode(7, INPUT);
   pinMode(8, INPUT);
   pinMode(9, INPUT);
*/
    Serial.begin(9600);
    current_command = COMMAND_IDLE;
    old_command = current_command;
     //Initialise states and commands
    int i;
    for (i = 0; i<=15; i++)
    {
            opening_state_cmds[i].motor_top_cmd = STOP;
            opening_state_cmds[i].motor_lid_cmd = STOP;
            closing_state_cmds[i].motor_top_cmd = STOP;
            closing_state_cmds[i].motor_lid_cmd = STOP;
    }
    closing_state_cmds[CL_COVER_LOCKED].motor_top_cmd = STOP;
    closing_state_cmds[CL_COVER_LOCKED].motor_lid_cmd = COUNTERCLOCKWISE;
    closing_state_cmds[CL_COVER_UNLOCKED].motor_top_cmd = STOP;
    closing_state_cmds[CL_COVER_UNLOCKED].motor_lid_cmd = COUNTERCLOCKWISE;
    closing_state_cmds[CL_COVER_OPENED].motor_top_cmd = CLOCKWISE;
    closing_state_cmds[CL_COVER_OPENED].motor_lid_cmd = STOP;
    closing_state_cmds[CL_RAISING_TOP].motor_top_cmd = CLOCKWISE;
    closing_state_cmds[CL_RAISING_TOP].motor_lid_cmd = STOP;
    closing_state_cmds[CL_TENSION_BOW_ALL_UP].motor_top_cmd = STOP;
    closing_state_cmds[CL_TENSION_BOW_ALL_UP].motor_lid_cmd =CLOCKWISE;
    closing_state_cmds[CL_COVER_CLOSING].motor_top_cmd = STOP;
    closing_state_cmds[CL_COVER_CLOSING].motor_lid_cmd = CLOCKWISE;
    closing_state_cmds[CL_COVER_LOCKED].motor_top_cmd = COUNTERCLOCKWISE;
    closing_state_cmds[CL_COVER_LOCKED].motor_lid_cmd = STOP;
    closing_state_cmds[CL_TENSION_BOW_LOWERING].motor_top_cmd = COUNTERCLOCKWISE;
    closing_state_cmds[CL_TENSION_BOW_LOWERING].motor_lid_cmd = STOP;
    closing_state_cmds[CL_TENSION_BOW_DOWN].motor_top_cmd = STOP;
    closing_state_cmds[CL_TENSION_BOW_DOWN].motor_lid_cmd = STOP;

    opening_state_cmds[OP_TOP_UNLOCKED_AND_LIFTED].motor_top_cmd = CLOCKWISE;
    opening_state_cmds[OP_TOP_UNLOCKED_AND_LIFTED].motor_lid_cmd = STOP;
    opening_state_cmds[OP_TENSION_BOW_RAISING].motor_top_cmd = CLOCKWISE;
    opening_state_cmds[OP_TENSION_BOW_RAISING].motor_lid_cmd = STOP;
    opening_state_cmds[OP_TENSION_BOW_ALL_UP].motor_top_cmd = STOP;
    opening_state_cmds[OP_TENSION_BOW_ALL_UP].motor_lid_cmd = COUNTERCLOCKWISE;
    opening_state_cmds[OP_COVER_UNLOCKED].motor_top_cmd = STOP;
    opening_state_cmds[OP_COVER_UNLOCKED].motor_lid_cmd = COUNTERCLOCKWISE;
    opening_state_cmds[OP_COVER_OPEN].motor_top_cmd = COUNTERCLOCKWISE;
    opening_state_cmds[OP_COVER_OPEN].motor_lid_cmd = STOP;
    opening_state_cmds[OP_TENSION_BOW_LOWERING].motor_top_cmd = COUNTERCLOCKWISE;
    opening_state_cmds[OP_TENSION_BOW_LOWERING].motor_lid_cmd = STOP;
    opening_state_cmds[OP_TOP_IN_COMPARTMENT].motor_top_cmd = STOP;
    opening_state_cmds[OP_TOP_IN_COMPARTMENT].motor_lid_cmd = CLOCKWISE;
    opening_state_cmds[OP_COVER_CLOSING].motor_top_cmd = STOP;
    opening_state_cmds[OP_COVER_CLOSING].motor_lid_cmd = CLOCKWISE;
    opening_state_cmds[OP_COVER_LOCKED].motor_top_cmd = STOP;
    opening_state_cmds[OP_COVER_LOCKED].motor_lid_cmd = STOP;

    current_av_steps = 0;
    int j;
    for (j = 0; j < CURRENT_AVERAGING_STEPS; j++)
    {
        raw_current [j] = 0;
    }

}

void loop ()
{
  
  //Read inputs
  PollInputs();
  
    current_command = ReadUserCommand();
    if (old_command != current_command)
    {
      Serial << "current command " << current_command << "\n";
      display_motor_command = 1;
    }
    old_command = current_command;

  current_state = ReadSwitchState();
  
  if (current_state != old_state)
  {  
    Serial << " ******  new state is --> " << current_state<< "\n";
    display_motor_command = 1;
  }
  old_state = current_state;
  
  ReadKeyboardCmds()  ;
  
   //Execute the actual power top control
  if (!manual_commands)
  {   
    ExecuteLogic();
    CurrentProtection();
    CheckTimeout();
  }
  display_motor_command = 0;

}


void ExecuteLogic()
{      

    if (current_command == COMMAND_IDLE)
    {
        MotorTopStop();
        MotorLidStop();
        return;
    }


if ((current_command == COMMAND_OPEN)||(current_command == COMMAND_AUTO_OPEN))
    {
      ///Serial << "OPENING " << current_state << " Top " <<     opening_state_cmds[current_state].motor_top_cmd << " Lid: " << opening_state_cmds[current_state].motor_lid_cmd  <<"\n";
      switch (opening_state_cmds[current_state].motor_top_cmd)
        {
                case STOP : MotorTopStop(); break;
                case CLOCKWISE : MotorTopClockwise(); break;
                case COUNTERCLOCKWISE : MotorTopCounterClockwise(); break;
                default : MotorTopStop();
        }
        switch (opening_state_cmds[current_state].motor_lid_cmd )
        {
                case STOP : MotorLidStop(); break;
                case CLOCKWISE : MotorLidClockwise(); break;
                case COUNTERCLOCKWISE : MotorLidCounterClockwise(); break;
                default : MotorLidStop();
        }
    }

    if ((current_command == COMMAND_CLOSE)||(current_command == COMMAND_AUTO_CLOSE))
    {


        //Serial << "CLOSING " << current_state << " Top " <<     closing_state_cmds[current_state].motor_top_cmd << " Lid: " << closing_state_cmds[current_state].motor_lid_cmd  <<"\n";      
        switch (closing_state_cmds[current_state].motor_top_cmd )
        {

                case STOP : MotorTopStop();break;
                case CLOCKWISE : MotorTopClockwise();  break;
                case COUNTERCLOCKWISE : MotorTopCounterClockwise();  break;
               // default : MotorTopStop();
        }
        switch (closing_state_cmds[current_state].motor_lid_cmd )
        {

                case STOP : MotorLidStop();  break;
                case CLOCKWISE : MotorLidClockwise();  break;
                case COUNTERCLOCKWISE : MotorLidCounterClockwise();  break;
                ///default : MotorLidStop();
        }
    }


}


void CurrentProtection()
{
    int j;
    raw_current [current_av_steps] = ADCValueToCurrent(analogRead(A1));
    current_av_steps++;

    if (current_av_steps == CURRENT_AVERAGING_STEPS)
        current_av_steps = 0;

    //compute average
    float average = 0;
    for (j = 0; j < CURRENT_AVERAGING_STEPS; j++)
    {
    average = average + raw_current [j];
    }
    average = average / CURRENT_AVERAGING_STEPS;

    if (fabs(average) >= MAX_CURRENT)
    {
            current_command = COMMAND_IDLE;
    }

}



void TestMotors()
{
    MotorTopStop();
    delay(1000);
    MotorTopClockwise();
    delay(2000);
    MotorTopStop();
    delay(1000);
    MotorTopCounterClockwise();
    delay(2000);
    MotorTopStop();


    MotorLidStop();
    delay(1000);
    MotorLidClockwise();
    delay(2000);
    MotorLidStop();
    delay(1000);
    MotorLidCounterClockwise();
    delay(2000);
    MotorLidStop();

}

void MotorLidClockwise()
{
   digitalWrite(14, HIGH);
   digitalWrite(15, LOW);
   if ((manual_commands)||(display_motor_command)) Serial << "MotorLid: clockwise \n";
}

void MotorLidCounterClockwise()
{
   digitalWrite(15, HIGH);
   digitalWrite(14, LOW);
   if ((manual_commands)||(display_motor_command)) Serial << "MotorLid: counter-clockwise \n";
}

void MotorLidStop()
{
   digitalWrite(14, LOW);
   digitalWrite(15, LOW);
   if ((manual_commands)||(display_motor_command))Serial << "MotorLid: stop \n";
}

void MotorTopClockwise()
{
   digitalWrite(16, HIGH);
   digitalWrite(10, LOW);
   if ((manual_commands)||(display_motor_command)) Serial << "MotorTop: clockwise \n";
}

void MotorTopCounterClockwise()
{
   digitalWrite(10, HIGH);
   digitalWrite(16, LOW);
   if ((manual_commands)||(display_motor_command)) Serial << "MotorTop: counter-clockwise \n";   
}

void MotorTopStop()
{
   digitalWrite(10, LOW);
   digitalWrite(16, LOW);
   if ((manual_commands)||(display_motor_command)) Serial << "MotorTop: stop \n";   
}

void ReadAndDisplayInputs()
{
  
  for (i= 0; i< 6; i++)
        pin[i] = digitalRead(i+4);

    for (i= 0; i< 6; i++)
    {
      if (_pin[i] != pin[i])
      {
      Serial.print("Pin ");
      Serial.print(i+4);
      Serial.print("is: ");
      Serial.println(pin[i]);
      }
      _pin[i]=pin[i];
    }
    
}


float ADCValueToCurrent (long int adc_in)
{
  float mv = (adc_in / 1023.0) * 5000;

  return (mv - 2500) * MV_TO_AMP;

}

void PollInputs()
{
   InRoofOpen.poll();  
   InRoofClose.poll();  
   InputSW1.poll();
   InputSW2.poll();
   InputSW3.poll();
   InputSW4.poll();
 
}

int ReadSwitchState()
{
    
    int sw1 = SW1;
    int sw2 = SW2;
    int sw3 = SW3;
    int sw4 = SW4;
    if (InputSW1.pushed()) sw1 = 1;
    if (InputSW1.released()) sw1 = 0;
    if (InputSW2.pushed()) sw2 = 1;
    if (InputSW2.released()) sw2 = 0;
    if (InputSW3.pushed()) sw3 = 1;
    if (InputSW3.released()) sw3 = 0;
    if (InputSW4.pushed()) sw4 = 1;
    if (InputSW4.released()) sw4 = 0;
    SW1 = sw1;
    SW2 = sw2;
    SW3 = sw3;
    SW4 = sw4;

if (display_switches )
{
  
  Serial << "SW1: " << SW1 <<  " SW2: " << SW2<< " SW3: " << SW3 << " SW4: " << SW4 <<"\n";
  display_switches = 0;
  /*
  int i;
    for (i = 0; i<=15; i++)    
        Serial << "OPENING " << i << " Top " <<     opening_state_cmds[i].motor_top_cmd << " Lid: " << opening_state_cmds[i].motor_lid_cmd  <<"\n";
  for (i = 0; i<=15; i++)
        Serial << "CLOSING " << i << " Top " <<     closing_state_cmds[i].motor_top_cmd << " Lid: " << closing_state_cmds[i].motor_lid_cmd  <<"\n";
    */

}
  
    return (8*SW1 + 4*SW2 + 2*SW3 + SW4);
}

void CheckTimeout()
{
    unsigned long motion_time = millis() -motion_start_time;
    if ( (current_command >= COMMAND_AUTO_OPEN) && (motion_time > MAX_MOTION_TIME_MS  ) )
            current_command = COMMAND_IDLE;
}


int ReadUserCommand()
{
    
   int output_command = current_command;

  if( (InRoofOpen.pushed() || InRoofClose.pushed()) && (current_command >= COMMAND_AUTO))
  {

                      Serial << "STOP Auto!!\n";
                      return COMMAND_IDLE;
  }

  if(InRoofOpen.pushed()) {
                    motion_start_time = millis();
                    output_command = COMMAND_OPEN;
                    Serial << "command_open\n";
  }
  if(InRoofOpen.released()) 
  {
    if ((current_command == COMMAND_OPEN) &&  ((millis()-motion_start_time) <= 200))
    {
                      motion_start_time = millis();
                      output_command= COMMAND_AUTO_OPEN;
                                          Serial << "command_auto_open\n";
                    }
                     else {
                     output_command = COMMAND_IDLE; 
                   Serial << "command_idle\n" ;
                 }
  }                
  
  if(InRoofClose.pushed()) {
                    motion_start_time = millis();
                    output_command = COMMAND_CLOSE;
                    Serial << "command_close\n";
  }
  if(InRoofClose.released()) 
  {
    if ((current_command == COMMAND_CLOSE) &&  ((millis()-motion_start_time) <= 200))
    {
                      motion_start_time = millis();
                      output_command= COMMAND_AUTO_CLOSE;
                                          Serial << "command_auto_close\n";
                    }
                     else {
                     output_command = COMMAND_IDLE; 
                   Serial << "command_idle\n" ;
                 }
  }
    return output_command;
}


void ReadKeyboardCmds()
{
  
    //Check for manual commands
   if (Serial.available() > 0) 
   {  
      char rx_byte = Serial.read();       // get the character
    
      // check if manual command
      if ((rx_byte == 'x') || (rx_byte == 'X')) 
      {
        manual_commands =!manual_commands;
        Serial << "switched to " << (manual_commands ? "manual" : "programmed") << "controls \n";
      }
      
      // Display state
      if ((rx_byte == 's') || (rx_byte == 'S')) 
      {
        Serial << " ######  current state is --> " << current_state<< "\n";
          current_state = ReadSwitchState();
      }
      
      // ?
      if (rx_byte == '?')  
      {
        display_switches = 1;
      }
   
      
     if (manual_commands)
     {
      switch (rx_byte) 
      {
       case 'A':
       case 'a': MotorTopClockwise(); MotorLidStop(); manual_counter = 1000; break;
       case 'Z':
       case 'z': MotorTopCounterClockwise(); MotorLidStop();  manual_counter = 1000; break;
       case 'E':
       case 'e': MotorTopStop(); MotorLidClockwise();  manual_counter = 1000; break;
       case 'R':
       case 'r': MotorTopStop(); MotorLidCounterClockwise();  manual_counter = 1000; break;
       default :  MotorTopStop(); MotorLidStop(); manual_counter = 10; break; 
      }
        
     }
   }
   
   else 
   { 
      manual_counter --;
      if (manual_counter < 0)
        manual_counter= 0;
     //if no keypressed and manual control, stop motion
     if ((manual_commands)&&(manual_counter == 1))
     {
       MotorTopStop();
       MotorLidStop();
       manual_counter = 0;
     }
   }
   
}











