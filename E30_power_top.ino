//ACTUAL BOARD:
//ARDUINO NANO

#if ARDUINO >= 100 
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include <Streaming.h>
#include "Switch.h"
#include "mystructs.h"
#include <SoftwareSerial.h>

#define  LidMotorSwitch1 6  //S3
#define  LidMotorSwitch2 9  //S4

#define  TopMotorSwitch1 4  //S1
#define  TopMotorSwitch2 5  //S2

#define MOTOR_LID_1 11
#define MOTOR_LID_2 12
#define MOTOR_TOP_1 3  //was 13
#define MOTOR_TOP_2 10

#define ButtonRoofOpen 8
#define ButtonRoofClose 7

#define CLOCKWISE  1
#define STOP	0
#define COUNTERCLOCKWISE 2

#define CURRENT_AVERAGING_STEPS  10
#define CURRENT_RANGE 20
#define MAX_CURRENT 12
#define CURRENT_LIMIT_ROOF_LIFTING 9
#define MAX_CURRENT_MANUAL 20 
#define MAX_CURRENT_EXTRA_LIMIT 2
#define MAX_CURRENT_EXTRA_LIMIT_2 5
#define STATE_CHANGE_CURRNET_CHECK_INHIBIT_STEPS 10
#define MV_PER_AMP 100
#define MAX_PHASE_MOTION_TIME_MS 	7000	//7 seconds
#define MONITOR_TIME_PERIOD_MS     200 // ms ; period for sending message

// ROOF OPENING SEQUENCE STATES
#define OP_TOP_UNLOCKED_AND_LIFTED	2
#define OP_TENSION_BOW_RAISING  	6
#define OP_TENSION_BOW_ALL_UP       	14
#define OP_COVER_UNLOCKED		12
#define OP_COVER_OPEN    		13
#define OP_TENSION_BOW_LOWERING	       5
#define OP_TOP_GOING_DOWN	      1
#define OP_TOP_IN_COMPARTMENT	      9
#define OP_COVER_CLOSING		8
#define OP_COVER_LOCKED               10


//ROOF CLOSING SEQUENCE STATES
#define CL_COVER_LOCKED_STOWED  10
#define CL_COVER_UNLOCKED	8
#define CL_COVER_OPENED		9
#define CL_RAISING_TOP		1  
#define CL_TOP_UP               5
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
int state_just_changed = 1;
int allow_lid_repeat_closing = 1;

int pin[6];
int _pin[6];
int i;
int current_av_steps;
OutputCmd opening_state_cmds[16];
OutputCmd closing_state_cmds[16];
int current_state, old_state, monitor_old_state;
float monitor_current;
int monitor_current_limit, monitor_last_current_exceed, monitor_last_current_exceed_state;
int  display_motor_command = 0;
int current_command, old_command;
float raw_current[CURRENT_AVERAGING_STEPS];
unsigned long motion_start_time;
unsigned long phase_start_time;
unsigned long monitor_time;
int manual_commands = 0;
int manual_counter = 1;
int OldRoofClose, OldRoofOpen;
int show_current_measure;

unsigned short known_states[10];
char state_labels_open[10][12];
char state_labels_close[10][12];

SoftwareSerial mySerial(A6, A7);

void setup() {
//Initialise board
   pinMode(MOTOR_LID_1, OUTPUT);  //was pin 14
   pinMode(MOTOR_LID_2, OUTPUT);  //was pin 15
   pinMode(MOTOR_TOP_1, OUTPUT);  //was pin 16
   pinMode(MOTOR_TOP_2, OUTPUT);
   MotorTopStop();
   MotorLidStop();
   InitLabels();

  /* pinMode(4, INPUT);
   pinMode(5, INPUT);
   pinMode(6, INPUT);
   pinMode(7, INPUT);
   pinMode(8, INPUT);
   pinMode(9, INPUT);
*/
    Serial.begin(9600);
    mySerial.begin(9600);
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
    closing_state_cmds[CL_COVER_LOCKED_STOWED].motor_top_cmd = STOP;
    closing_state_cmds[CL_COVER_LOCKED_STOWED].motor_lid_cmd = COUNTERCLOCKWISE;
    closing_state_cmds[CL_COVER_UNLOCKED].motor_top_cmd = STOP;
    closing_state_cmds[CL_COVER_UNLOCKED].motor_lid_cmd = COUNTERCLOCKWISE;
    closing_state_cmds[CL_COVER_OPENED].motor_top_cmd = CLOCKWISE;
    closing_state_cmds[CL_COVER_OPENED].motor_lid_cmd = STOP;
    closing_state_cmds[CL_RAISING_TOP].motor_top_cmd = CLOCKWISE;
    closing_state_cmds[CL_RAISING_TOP].motor_lid_cmd = STOP;
    closing_state_cmds[CL_TOP_UP].motor_top_cmd = CLOCKWISE;
    closing_state_cmds[CL_TOP_UP].motor_lid_cmd = STOP;
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
    opening_state_cmds[OP_TOP_GOING_DOWN].motor_top_cmd = COUNTERCLOCKWISE;
    opening_state_cmds[OP_TOP_GOING_DOWN].motor_lid_cmd = STOP;

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
    
    PollInputs();
    display_switches = 1;
    show_current_measure = 0;
    ReadSwitchState();
    monitor_old_state = -99;
    monitor_current = 0;
    monitor_current_limit= 0;
    monitor_last_current_exceed = 0;
    monitor_last_current_exceed_state = 0;
    monitor_time = millis();

}
void resetCurrentAverage()
{
  int j;
  for (j = 0; j < CURRENT_AVERAGING_STEPS; j++)
    {
        raw_current [j] = 0;
    }
    current_av_steps = 0;
}

void loop ()
{

  
  //Read inputs
  PollInputs();
  //ShowCurrent();
  
    current_command = ReadUserCommand();
    if (old_command != current_command)
    {
      Serial << "current command " << current_command << ";  current state" << current_state << "\n";
      display_motor_command = 1;
    }
    old_command = current_command;

  current_state = ReadSwitchState();
  
  if (current_state != old_state)
  {  
    Serial << " ******  new state is --> " << current_state<< "\n";
    display_motor_command = 1;
    state_just_changed = STATE_CHANGE_CURRNET_CHECK_INHIBIT_STEPS;
    //StopALittle();
    phase_start_time = millis();
    resetCurrentAverage();
    allow_lid_repeat_closing = 1;
    monitor_old_state = old_state;
  }
  //old_state = current_state;
  
  ReadKeyboardCmds()  ;
  
   //Execute the actual power top control
  if (!manual_commands)
  {   
    ExecuteLogic();
   
    if (state_just_changed == 0)
      CurrentProtection();
    if (state_just_changed > 0)  
    { 
      Serial << state_just_changed <<"\n"; 
      state_just_changed--;
    }
      
      CheckTimeout();       
  }

  SendToMonitor();
  old_state = current_state;
  display_motor_command = 0;
 
}

void SendToMonitor()
{
    unsigned long _monitor_time = millis() - monitor_time;
    if (_monitor_time < MONITOR_TIME_PERIOD_MS)
        return;

    char buffer[50];
    // current state (12), old state (12), current_limit (2), monitor_last_current_exceed (4), monitor_last_current_exceed_state (12)
    if ((current_command == COMMAND_OPEN) || (current_command == COMMAND_AUTO_OPEN))
        sprintf(buffer, "$,%s,%s,%i,%2.1f,%s", (IsStateValid(current_state) ? state_labels_open[OpeningStateLabelIndex(current_state)] : "ERROR"),
                                                state_labels_open[OpeningStateLabelIndex(monitor_old_state)],
                                                monitor_current_limit,
                                                monitor_last_current_exceed,
                                                state_labels_open[OpeningStateLabelIndex(monitor_last_current_exceed_state)] );


    else
        sprintf(buffer, "$,%s,%s,%i,%2.1f,%s", (IsStateValid(current_state) ? state_labels_close[ClosingStateLabelIndex(current_state)] : "ERROR"),
                                                state_labels_close[ClosingStateLabelIndex(monitor_old_state)],
                                                monitor_current_limit,
                                                monitor_last_current_exceed,
                                                state_labels_close[ClosingStateLabelIndex(monitor_last_current_exceed_state)] );


    mySerial.println(buffer);
}

void StopALittle()
{
  MotorTopStop();
  MotorLidStop();
  delay (500);
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
  
    static int skip = 0;
    int j;
    int anain = analogRead(7);
    raw_current [current_av_steps] =ADCValueToCurrent(anain) ;
    current_av_steps++;

    if (current_av_steps >= CURRENT_AVERAGING_STEPS)
        current_av_steps = 0;

    //compute average
    float average = 0;
    for (j = 0; j < CURRENT_AVERAGING_STEPS; j++)
    {
    average = average + raw_current [j];
    }
    average = average / CURRENT_AVERAGING_STEPS;

  //  Serial <<"ana in " << anain << " current  " << ADCValueToCurrent(anain) << " (A) \n";  

    
   float current_limit = MAX_CURRENT;

  if ((current_command == COMMAND_OPEN ) || (current_command == COMMAND_CLOSE ))
    current_limit = MAX_CURRENT_MANUAL;

    
    if (current_command == COMMAND_AUTO_OPEN)
    {
      if((current_state==OP_COVER_CLOSING)||(current_state==CL_COVER_CLOSING))
	      current_limit = MAX_CURRENT + MAX_CURRENT_EXTRA_LIMIT;
      else if(current_state ==OP_TENSION_BOW_RAISING)
        current_limit = MAX_CURRENT_MANUAL;
       else
       current_limit = MAX_CURRENT;
    }

  if (current_command == COMMAND_AUTO_CLOSE)
    if(current_state==CL_RAISING_TOP    )
       current_limit  = CURRENT_LIMIT_ROOF_LIFTING;
     else if(current_state == CL_TOP_UP)
       current_limit = MAX_CURRENT + MAX_CURRENT_EXTRA_LIMIT_2;
       else
        current_limit = MAX_CURRENT;
   
 if (show_current_measure)
 {
   if (skip >= 3)
   {
       Serial << " current measure " << fabs(average) << " (A)  LIMIT : " << current_limit << "\n";
       skip = 0;
   }
   skip = skip+1;
 }

 monitor_current = fabs(average);
 monitor_current_limit = current_limit;

    if (fabs(average) >= current_limit)
    {
            if (current_state == OP_COVER_CLOSING)
                if (allow_lid_repeat_closing)
                   TryAndCloseLidAgain();           
                else 
                   current_command = COMMAND_IDLE;
            Serial << "##### current limit reached " << fabs(average) << " (A) \n";
            monitor_last_current_exceed = fabs(average);
            monitor_last_current_exceed_state = current_state;
    }
  
}


void TryAndCloseLidAgain()
{
    allow_lid_repeat_closing = 0;
    MotorLidStop();
    delay (1000);
    MotorLidCounterClockwise();
    delay(1500);
    MotorLidStop();
    delay(1000);
    MotorLidClockwise();
    current_state = OP_COVER_CLOSING;
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
   digitalWrite(MOTOR_LID_1, HIGH);  //14  #define MOTOR_LID_1 14
   digitalWrite(MOTOR_LID_2, LOW); 
   if ((manual_commands)||(display_motor_command)) Serial << "MotorLid: clockwise \n";
}

void MotorLidCounterClockwise()
{
   digitalWrite(MOTOR_LID_2, HIGH);
   digitalWrite(MOTOR_LID_1, LOW);
   if ((manual_commands)||(display_motor_command)) Serial << "MotorLid: counter-clockwise \n";
}

void MotorLidStop()
{
   digitalWrite(MOTOR_LID_1, LOW);
   digitalWrite(MOTOR_LID_2, LOW);
   if ((manual_commands)||(display_motor_command))Serial << "MotorLid: stop \n";
}

void MotorTopClockwise()
{
   digitalWrite(MOTOR_TOP_1, HIGH);
   digitalWrite(MOTOR_TOP_2, LOW);
   if ((manual_commands)||(display_motor_command)) Serial << "MotorTop: clockwise \n";
}

void MotorTopCounterClockwise()
{
   digitalWrite(MOTOR_TOP_2, HIGH);
   digitalWrite(MOTOR_TOP_1, LOW);
   if ((manual_commands)||(display_motor_command)) Serial << "MotorTop: counter-clockwise \n";   
}

void MotorTopStop()
{
   digitalWrite(MOTOR_TOP_2, LOW);
   digitalWrite(MOTOR_TOP_1, LOW);
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
 
 float temp = adc_in;
 float mv = (temp / 1023.0) * 5000.0;

  return ((mv - 2500) / MV_PER_AMP);

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
    
    SW1 = !digitalRead(TopMotorSwitch1);
    SW2 = !digitalRead(TopMotorSwitch2); 
    SW3 = digitalRead(LidMotorSwitch1); 
    SW4 =digitalRead(LidMotorSwitch2);

if (display_switches )
{
  
  Serial << "SW1: " << SW1 <<  " SW2: " << SW2<< " SW3: " << SW3 << " SW4: " << SW4 <<"\n";
  Serial << "in1: " <<!digitalRead(TopMotorSwitch1) <<" in2: " <<!digitalRead(TopMotorSwitch2) <<" in3: " <<digitalRead(LidMotorSwitch1) <<" in4: " <<digitalRead(LidMotorSwitch2)<<"\n";
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
    unsigned long _motion_time = millis() -phase_start_time;
    unsigned long _max_phase_time = MAX_PHASE_MOTION_TIME_MS + (current_state == (CL_RAISING_TOP||OP_TOP_GOING_DOWN) ? 6000: 0);
    if ( (current_command >= COMMAND_AUTO_OPEN) && (_motion_time > _max_phase_time ) )
    {
            current_command = COMMAND_IDLE;
            Serial << " ####  Phase timeout ###\n";
    }
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
                      phase_start_time = motion_start_time;
                      output_command= COMMAND_AUTO_OPEN;
                      state_just_changed = STATE_CHANGE_CURRNET_CHECK_INHIBIT_STEPS;
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
                      phase_start_time = motion_start_time;
                      output_command= COMMAND_AUTO_CLOSE;
                      state_just_changed = STATE_CHANGE_CURRNET_CHECK_INHIBIT_STEPS;
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
        if (manual_commands)
        {
        Serial << "MANUAL e,E : lid close; r,R lid open; a,A roof close; z,Z roof open ·\n";
        Serial << "AUTO   >  roof close; <  roof open ·\n";
        MotorLidStop();
        MotorTopStop();
        current_command = COMMAND_IDLE;
        
        }
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

   // Display current
      if ((rx_byte == 'c') || (rx_byte == 'C')) 
      {
        show_current_measure = ! show_current_measure;
          
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
       case '<': Serial << ">>> Auto OPEN\n"; manual_commands = false; current_command = COMMAND_AUTO_OPEN; phase_start_time = millis();state_just_changed = STATE_CHANGE_CURRNET_CHECK_INHIBIT_STEPS;break;
       case '>': Serial << "<<< Auto CLOSE\n"; manual_commands = false; current_command = COMMAND_AUTO_CLOSE; phase_start_time = millis();state_just_changed = STATE_CHANGE_CURRNET_CHECK_INHIBIT_STEPS;break;
       
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



void ShowCurrent()
{
  static int tt = 0;
  int anain = analogRead(7);
  if (tt >=1000)
  {
  tt=0;
  Serial <<"ana in " << anain << " current  " << ADCValueToCurrent(anain) << " (A) \n";  
 
  }
  tt++;

}

void InitLabels()
{


strncpy(state_labels_open[0],"UNLOCK_LIFT",12);
strncpy(state_labels_open[1],"BOW_RAISING",12);
strncpy(state_labels_open[2],"BOW_ALL_UP",12);
strncpy(state_labels_open[3],"COVER_UNLOCK",12);
strncpy(state_labels_open[4],"COVER_OPEN",12);
strncpy(state_labels_open[5],"BOW_LOWERING",12);
strncpy(state_labels_open[6],"TOP_GO_DOWN",12);
strncpy(state_labels_open[7],"TOP_STOWED",12);
strncpy(state_labels_open[8],"COVER_CLOSING",12);
strncpy(state_labels_open[9],"COVER_LOCKED",12);

strncpy(state_labels_open[9],"COVER_LOCK_IN",12);
strncpy(state_labels_open[8],"COVER_UNLCKD",12);
strncpy(state_labels_open[7],"COVER_OPENED",12);
strncpy(state_labels_open[6],"RAISING_TOP",12);
strncpy(state_labels_open[5],"TOP_UP",12);
strncpy(state_labels_open[4],"BOW_ALL_UP",12);
strncpy(state_labels_open[3],"COVER_CLOSING",12);
strncpy(state_labels_open[2],"COVER_LOCKED",12);
strncpy(state_labels_open[1],"BOW_LOWERING",12);
strncpy(state_labels_open[0],"BOW_DOWN",12);

//sequence of state follows opening sequence
known_states[0] = 2;
known_states[1] = 6;
known_states[2] = 14;
known_states[3] = 12;
known_states[4] = 13;
known_states[5] = 5;
known_states[6] = 1;
known_states[7] = 9;
known_states[8] = 8;
known_states[9] = 10;

}

bool IsStateValid(int _state)
{
    int i;
    for (i=0; i<= 9; i++)
        if (known_states[i] = _state)
            return true;
    return false;
}

bool OpeningStateLabelIndex(int _state)
{
    int i;
    for (i=0; i<= 9; i++)
        if (known_states[i] = _state)
            return i;

}

bool ClosingStateLabelIndex(int _state)
{
    return 9 - OpeningStateLabelIndex(_state);

}



