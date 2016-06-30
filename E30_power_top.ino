#if ARDUINO >= 100 
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include <Streaming.h>
#include <Switch.h>
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
// CLOSING SEQUENCE STATES
#define TOP_UNLOCKED_AND_LIFTED	2
#define TENSION_BOW_RAISING	6
#define TENSION_BOW_UP   	14
#define COVER_UNLOCKED		15
#define COVER_OPEN		13
#define TENSION_BOW_LOWERING	5
#define TOP_IN_COMPARTMENT	1
#define COVER_CLOSING		11
#define COVER_LOCKED		10


//OPENING SEQUENCE STATES
#define COVER_LOCKED		10
#define COVER_UNLOCKED		11
#define COVER_OPENED		9
#define RAISING_TOP		5
#define TENSION_BOW_ALL_UP	13
#define COVER_CLOSED		15
#define COVER_LOCKED		14
#define TENSION_BOW_LOWERING	6
#define TENSION_BOW_DOWN	2


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
Switch InputSW3= Switch (LidMotorSwitch1, INPUT,LOW);
Switch InputSW4= Switch (LidMotorSwitch2, INPUT,LOW);
int SW1 = 0;
int SW2 = 0;
int SW3 = 0;
int SW4 = 0;

int pin[6];
int _pin[6];
int i;
int current_av_steps;
OutputCmd opening_state_cmds[16];
OutputCmd closing_state_cmds[16];
int current_state, old_state;
int current_command, old_command;
float raw_current[CURRENT_AVERAGING_STEPS];
unsigned long motion_start_time;

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
            opening_state_cmds[i].motor_1_cmd = STOP;
            opening_state_cmds[i].motor_2_cmd = STOP;
            closing_state_cmds[i].motor_1_cmd = STOP;
            closing_state_cmds[i].motor_2_cmd = STOP;
    }
    opening_state_cmds[COVER_LOCKED].motor_1_cmd = STOP;
    opening_state_cmds[COVER_LOCKED].motor_2_cmd = COUNTERCLOCKWISE;
    opening_state_cmds[COVER_UNLOCKED].motor_1_cmd = STOP;
    opening_state_cmds[COVER_UNLOCKED].motor_2_cmd = COUNTERCLOCKWISE;
    opening_state_cmds[COVER_OPENED].motor_1_cmd = CLOCKWISE;
    opening_state_cmds[COVER_OPENED].motor_2_cmd = STOP;
    opening_state_cmds[RAISING_TOP].motor_1_cmd = CLOCKWISE;
    opening_state_cmds[RAISING_TOP].motor_2_cmd = STOP;
    opening_state_cmds[TENSION_BOW_UP].motor_1_cmd = STOP;
    opening_state_cmds[TENSION_BOW_UP].motor_2_cmd = CLOCKWISE;
    opening_state_cmds[COVER_CLOSED].motor_1_cmd = STOP;
    opening_state_cmds[COVER_CLOSED].motor_2_cmd = CLOCKWISE;
    opening_state_cmds[COVER_LOCKED].motor_1_cmd = COUNTERCLOCKWISE;
    opening_state_cmds[COVER_LOCKED].motor_2_cmd = STOP;
    opening_state_cmds[TENSION_BOW_LOWERING].motor_1_cmd = COUNTERCLOCKWISE;
    opening_state_cmds[TENSION_BOW_LOWERING].motor_2_cmd = STOP;
    opening_state_cmds[TENSION_BOW_DOWN].motor_1_cmd = STOP;
    opening_state_cmds[TENSION_BOW_DOWN].motor_2_cmd = STOP;

    closing_state_cmds[TOP_UNLOCKED_AND_LIFTED].motor_1_cmd = CLOCKWISE;
    closing_state_cmds[TOP_UNLOCKED_AND_LIFTED].motor_2_cmd = STOP;
    closing_state_cmds[TENSION_BOW_RAISING].motor_1_cmd = CLOCKWISE;
    closing_state_cmds[TENSION_BOW_RAISING].motor_2_cmd = STOP;
    closing_state_cmds[TENSION_BOW_ALL_UP].motor_1_cmd = STOP;
    closing_state_cmds[TENSION_BOW_ALL_UP].motor_2_cmd = COUNTERCLOCKWISE;
    closing_state_cmds[COVER_UNLOCKED].motor_1_cmd = STOP;
    closing_state_cmds[COVER_UNLOCKED].motor_2_cmd = COUNTERCLOCKWISE;
    closing_state_cmds[COVER_OPEN].motor_1_cmd = COUNTERCLOCKWISE;
    closing_state_cmds[COVER_OPEN].motor_2_cmd = STOP;
    closing_state_cmds[TENSION_BOW_LOWERING].motor_1_cmd = COUNTERCLOCKWISE;
    closing_state_cmds[TENSION_BOW_LOWERING].motor_2_cmd = STOP;
    closing_state_cmds[TOP_IN_COMPARTMENT].motor_1_cmd = STOP;
    closing_state_cmds[TOP_IN_COMPARTMENT].motor_2_cmd = CLOCKWISE;
    closing_state_cmds[COVER_CLOSING].motor_1_cmd = STOP;
    closing_state_cmds[COVER_CLOSING].motor_2_cmd = CLOCKWISE;
    closing_state_cmds[COVER_LOCKED].motor_1_cmd = STOP;
    closing_state_cmds[COVER_LOCKED].motor_2_cmd = STOP;

    current_av_steps = 0;
    int j;
    for (j = 0; j < CURRENT_AVERAGING_STEPS; j++)
    {
        raw_current [j] = 0;
    }

}

void loop ()
{
  PollInputs();
  
    current_command = ReadUserCommand();
    if (old_command != current_command)
    {
      Serial << "current command " << current_command << "\n";
    }
    old_command = current_command;

  current_state = ReadSwitchState();
  if (current_state != old_state)
    Serial << " new state is" << current_state<< "\n";
  old_state = current_state;
  
}


      
/*}
    if (current_command == COMMAND_IDLE)
    {
        Motor1Stop();
        Motor2Stop();
        return;
    }

    current_state = ReadSwitchState();

    if ((current_command == COMMAND_OPEN)||(current_command == COMMAND_AUTO_OPEN))
    {
        switch (opening_state_cmds[current_state].motor_1_cmd)
        {
                case STOP : Motor1Stop(); break;
                case CLOCKWISE : Motor1Clockwise(); break;
                case COUNTERCLOCKWISE : Motor1CounterClockwise(); break;
                default : Motor1Stop();
        }
        switch (opening_state_cmds[current_state].motor_2_cmd )
        {
                case STOP : Motor2Stop(); break;
                case CLOCKWISE : Motor2Clockwise(); break;
                case COUNTERCLOCKWISE : Motor2CounterClockwise(); break;
                default : Motor2Stop();
        }
    }

    if ((current_command == COMMAND_CLOSE)||(current_command == COMMAND_AUTO_CLOSE))
    {
        switch (closing_state_cmds[current_state].motor_1_cmd )
        {
                case STOP : Motor1Stop(); break;
                case CLOCKWISE : Motor1Clockwise(); break;
                case COUNTERCLOCKWISE : Motor1CounterClockwise(); break;
                default : Motor1Stop();
        }
        switch (closing_state_cmds[current_state].motor_2_cmd )
        {
                case STOP : Motor2Stop(); break;
                case CLOCKWISE : Motor2Clockwise(); break;
                case COUNTERCLOCKWISE : Motor2CounterClockwise(); break;
                default : Motor2Stop();
        }
    }


    CurrentProtection();
    CheckTimeout();
}
*/

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
    Motor1Stop();
    delay(1000);
    Motor1Clockwise();
    delay(2000);
    Motor1Stop();
    delay(1000);
    Motor1CounterClockwise();
    delay(2000);
    Motor1Stop();


    Motor2Stop();
    delay(1000);
    Motor2Clockwise();
    delay(2000);
    Motor2Stop();
    delay(1000);
    Motor2CounterClockwise();
    delay(2000);
    Motor2Stop();

}

void Motor2Clockwise()
{
   digitalWrite(14, HIGH);
   digitalWrite(15, LOW);
}

void Motor2CounterClockwise()
{
   digitalWrite(15, HIGH);
   digitalWrite(14, LOW);
}

void Motor2Stop()
{
   digitalWrite(14, LOW);
   digitalWrite(15, LOW);
}

void Motor1Clockwise()
{
   digitalWrite(16, HIGH);
   digitalWrite(10, LOW);
}

void Motor1CounterClockwise()
{
   digitalWrite(10, HIGH);
   digitalWrite(16, LOW);
}

void Motor1Stop()
{
   digitalWrite(10, LOW);
   digitalWrite(16, LOW);
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
    // Switches are inverted logic: high: open , low closed
    // converted to 0 = open , 1 = close
    
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
        
    return (8*sw1 + 4*sw2 + 2*sw3 + sw4);
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














