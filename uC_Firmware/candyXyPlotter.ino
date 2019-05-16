#include "ServoTimer2.h" // https://github.com/nabontra/ServoTimer2
#include "TimerOne.h" // https://playground.arduino.cc/Code/Timer1/
#include "candyXyPlotter.h"

// ###################################
// general definitions

#define FALSE              0
#define TRUE               1

// left / down / close (L/D/C) must be 0 and right / up / open (R/U/O) must be 1 for the for the bitshifting in the "do_step" function
#define MOVEMENT_DIR_L_D_C 0
#define MOVEMENT_DIR_R_U_O 1
#define MOVEMENT_DISABLED  2

#define LIMIT_NO_ERROR     0
#define LIMIT_ERROR_MAX    1
#define LIMIT_ERROR_MIN    2

#define REACHED_NOTHING    0
#define REACHED_TARGET     1
#define REACHED_MAX        2
#define REACHED_MIN        3
#define REACHED_MIN_SW     4


// ###################################
// Settings

// CAUTION!! YOU CAN RUN INTO MAX IF YOU DONT INIT THE STEPPERS, BECAUSE THE MACHINE ZERO POSITION IS THEN WHERE YOU START
#define INIT_STEPPERS     TRUE
#define ENABLE_Z_AXIS     TRUE
#define ENABLE_Z_SERVO    TRUE
#define DEBUG_OUTPUTS     FALSE

// if you choose to litte values (under 90) it can happen, that the serial interface could not be processed completely in the main-loop 
// and probably the stop command can't reach the controller and you can't control it from the joystick/RPi anymore (connected via serial)
#define DEFAULT_STEP_SPEED_IN_MICROSEC   90
#define SLOW_STEP_SPEED_IN_MICROSEC      500
#define FAST_STEP_SPEED_IN_MICROSEC      30

#define SERVO_STEP_SIZE   1
#define SERVO_MIN_POS     1300
#define SERVO_MAX_POS     2250
#define SERVO_START_POS   (int)(SERVO_MIN_POS + ((SERVO_MAX_POS - SERVO_MIN_POS)/2))

// A7 is probably not working
#define X_STEP_PIN        10
#define X_DIR_PIN         11
//#define X_ENABLE_PIN      4
#define X_MIN_PIN         12
                          
#define Y_STEP_PIN        9
#define Y_DIR_PIN         3
//#define Y_ENABLE_PIN      4
#define Y_MIN_PIN         13
                          
#define Z_STEP_PIN        2
#define Z_DIR_PIN         8
//#define Z_ENABLE_PIN      4
#define Z_MIN_PIN         A2
#define Z_SERVO_PIN       A3

#define X_MOTOR_STEPS     16L
#define X_STEPS_PER_MM    21.168 * X_MOTOR_STEPS
//#define X_MAX_POS_MM      360L
//#define X_MAX_STEPS       X_MAX_POS_MM * X_STEPS_PER_MM
// => X_MAX_STEPS = 121927
// => X_MAX_COUNTER_3 = floor(X_MAX_STEPS / 256 / 256)
// => X_MAX_COUNTER_2 = floor((X_MAX_STEPS - (X_MAX_COUNTER_3 * 256 * 256)) / 256 )
// => X_MAX_COUNTER_1 = X_MAX_STEPS - (X_MAX_COUNTER_3 * 256 * 256) - (X_MAX_COUNTER_2 * 256)
#define X_MAX_COUNTER_3   1
#define X_MAX_COUNTER_2   220
                          
#define Y_MOTOR_STEPS     16L
#define Y_STEPS_PER_MM    21.168 * Y_MOTOR_STEPS
// without Z_END_SWITCH attached: 395L
//#define Y_MAX_POS_MM      370L
//#define Y_MAX_STEPS       Y_MAX_POS_MM * Y_STEPS_PER_MM
// => Y_MAX_STEPS = 121927
// => Y_MAX_COUNTER_3 = floor(Y_MAX_STEPS / 256 / 256)
// => Y_MAX_COUNTER_2 = floor((Y_MAX_STEPS - (Y_MAX_COUNTER_3 * 256 * 256)) / 256 )
// => Y_MAX_COUNTER_1 = Y_MAX_STEPS - (Y_MAX_COUNTER_3 * 256 * 256) - (Y_MAX_COUNTER_2 * 256)
#define Y_MAX_COUNTER_3   1
#define Y_MAX_COUNTER_2   235
                          
#define Z_MOTOR_STEPS     16L
#define Z_STEPS_PER_MM    13.5 * Z_MOTOR_STEPS
//#define Z_MAX_POS_MM      600L
//#define Z_MAX_STEPS       Z_MAX_POS_MM * Z_STEPS_PER_MM
// => Z_MAX_STEPS = 121927
// => Z_MAX_COUNTER_3 = floor(Z_MAX_STEPS / 256 / 256)
// => Z_MAX_COUNTER_2 = floor((Z_MAX_STEPS - (Z_MAX_COUNTER_3 * 256 * 256)) / 256 )
// => Z_MAX_COUNTER_1 = Z_MAX_STEPS - (Z_MAX_COUNTER_3 * 256 * 256) - (Z_MAX_COUNTER_2 * 256)
#define Z_MAX_COUNTER_3   1
#define Z_MAX_COUNTER_2   250

// ToDo
#define Z_MIN_POS_MM      150L
#define Z_MIN_STEPS       Z_MIN_POS_MM * Z_STEPS_PER_MM

#define COMMAND_SIZE      32
#define MAX_COUNTER_VAL   255

// ###################################
// automatic definitions

#if ENABLE_Z_AXIS
#define STEPMOTOR_AMOUNT  3
#else
#define STEPMOTOR_AMOUNT  2
#endif

#define STEPARRAY_EL_X    0
#define STEPARRAY_EL_Y    1
#define STEPARRAY_EL_Z    2

// ###################################

struct stepperAttr {
  char coordinateName;
  short move_and_dir;
  short counted_steps_1;
  short counted_steps_2;
  short counted_steps_3;
  float currentPosInMm;
  float targetPosInMm;
  short maxSteps_counter_3;
  short maxSteps_counter_2;
  double stepsPerMm;
  short step_pin;
  short dir_pin;
  //short enable_pin;
  short switch_min_pin;
};

volatile stepperAttr stepperAttrArray[STEPMOTOR_AMOUNT];

char commands[COMMAND_SIZE];
byte serial_count;
long no_data = 0;
byte limit_error = LIMIT_NO_ERROR;
char writeSerialBuffer[50];
int slowDownCounter = 0;

#if ENABLE_Z_SERVO
ServoTimer2 servo;
int servoCurrentPos = SERVO_START_POS;
int servoStatus = MOVEMENT_DISABLED;
#endif


// ##########################

void setup()
{
  //Do startup stuff here
  Serial.begin(115200);
  
  init_steppers();

  // init Timer interrupt for stepper control
  Timer1.initialize(SLOW_STEP_SPEED_IN_MICROSEC); // interrupt will go off every x microseconds
  Timer1.attachInterrupt(do_step);
  
#if INIT_STEPPERS
  goto_machine_zero();
#else
  Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
  Serial.println("!! CAUTION - INITIALIZATION IS DISABLED!");
  Serial.println("  YOU CAN RUN INTO MAX IF YOU DON'T INIT THE STEPPERS!  ");
  Serial.println("  (BECAUSE THE MACHINE ZERO POSITION IS NOW WHERE YOU STARTED)  ");
  Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
#endif

#if ENABLE_Z_SERVO
  servo.attach(Z_SERVO_PIN);
  servo.write(servoCurrentPos);
#endif
}

void loop()
{
  char c;
  // read in characters if we got them.
  if (Serial.available() > 0)
  {
    c = Serial.read();
    no_data = 0;
    // newlines are ends of commands.
    if (c != '\n')
    {
      commands[serial_count] = c;
      serial_count++;
    }
  }
  else
  {
    no_data++;

    // if theres a pause or we got a real command, do it
    if (serial_count && (c == '\n' || no_data > 100))
    {
      // process our command!
      process_string(commands, serial_count);
      // clear command
      init_process_string();
    }

    // no data?  turn off all movements after awhile
    if (no_data > 500000L) {
      stop_all_movements();
      no_data = 0;
#if DEBUG_OUTPUTS
      Serial.println("No movements for a long time: stop steppers and servo!");
#endif
    }
    
    if (limit_error != LIMIT_NO_ERROR) {
      if (limit_error == LIMIT_ERROR_MAX) {
        Serial.println("!! counter limit error - above 256**3 !!");
      } else if (limit_error == LIMIT_ERROR_MIN) {
        Serial.println("!! counter limit error - underneath 0 !!");
      }
      limit_error = LIMIT_NO_ERROR;
    }
  }
  
  // check if we reached something and stop  
  // also send the current position to the RPi for tingg.io
  for (short el = 0; el < STEPMOTOR_AMOUNT; el++) {
    short tmp = reached_target_or_end(el);
    // ToDo: add some more operations like set to zero when reaching End-switch etc
    if (tmp == REACHED_MAX || tmp == REACHED_MIN_SW) {
      stepperAttrArray[el].move_and_dir = MOVEMENT_DISABLED;
#if DEBUG_OUTPUTS
      if (tmp == REACHED_MAX) {
        sprintf(writeSerialBuffer,"!! reached max for %c !!", stepperAttrArray[el].coordinateName);
        Serial.println(writeSerialBuffer);
      } else if (tmp == REACHED_MIN_SW) {
        sprintf(writeSerialBuffer,"!! reached min switch for %c !!", stepperAttrArray[el].coordinateName);
        Serial.println(writeSerialBuffer);
      }
#endif
    }
  }
  
  // send current positions (without tiny changes in counter_1) to RPi
#if ENABLE_Z_AXIS
  if (slowDownCounter % 15000 == 0) {
    sprintf(writeSerialBuffer, "x%d;y%d;z%d", (stepperAttrArray[STEPARRAY_EL_X].counted_steps_3 * 256) + stepperAttrArray[STEPARRAY_EL_X].counted_steps_2, (stepperAttrArray[STEPARRAY_EL_Y].counted_steps_3 * 256) + stepperAttrArray[STEPARRAY_EL_Y].counted_steps_2, (stepperAttrArray[STEPARRAY_EL_Z].counted_steps_3 * 256) + stepperAttrArray[STEPARRAY_EL_Z].counted_steps_2);
    Serial.println(writeSerialBuffer);
    slowDownCounter = 0;
    delayMicroseconds(1);
  }
  slowDownCounter++;
#endif

#if ENABLE_Z_SERVO  
  // check if the servo position should change
  if (servoStatus != MOVEMENT_DISABLED)
  {
    if (!servo.attached())
      servo.attach(Z_SERVO_PIN);
    
    if (servoStatus == MOVEMENT_DIR_L_D_C)
    {
      if (servoCurrentPos < SERVO_MAX_POS) {
        servoCurrentPos = servoCurrentPos + SERVO_STEP_SIZE;
        servo.write(servoCurrentPos);
        delayMicroseconds(500); // waits for the servo to get there
      }
      else
        servoStatus = MOVEMENT_DISABLED;
    }
    else if (servoStatus == MOVEMENT_DIR_R_U_O)
    {
      if (servoCurrentPos > SERVO_MIN_POS) {
        servoCurrentPos = servoCurrentPos - SERVO_STEP_SIZE;
        servo.write(servoCurrentPos);
        delayMicroseconds(500); // waits for the servo to get there
      }
      else
        servoStatus = MOVEMENT_DISABLED;
    }
  }
#endif
}

void init_steppers()
{
  // X
  stepperAttrArray[STEPARRAY_EL_X].coordinateName = 'X';
  stepperAttrArray[STEPARRAY_EL_X].move_and_dir = MOVEMENT_DISABLED;
  stepperAttrArray[STEPARRAY_EL_X].currentPosInMm = 0.0;
  stepperAttrArray[STEPARRAY_EL_X].targetPosInMm = 0.0;
  stepperAttrArray[STEPARRAY_EL_X].counted_steps_1 = 0;
  stepperAttrArray[STEPARRAY_EL_X].counted_steps_2 = 0;
  stepperAttrArray[STEPARRAY_EL_X].counted_steps_3 = 0;
  stepperAttrArray[STEPARRAY_EL_X].maxSteps_counter_3 = X_MAX_COUNTER_3;
  stepperAttrArray[STEPARRAY_EL_X].maxSteps_counter_2 = X_MAX_COUNTER_2;
  stepperAttrArray[STEPARRAY_EL_X].stepsPerMm = X_STEPS_PER_MM;
  stepperAttrArray[STEPARRAY_EL_X].step_pin = X_STEP_PIN;
  stepperAttrArray[STEPARRAY_EL_X].dir_pin = X_DIR_PIN;
  //stepperAttrArray[STEPARRAY_EL_X].enable_pin = X_ENABLE_PIN;
  stepperAttrArray[STEPARRAY_EL_X].switch_min_pin = X_MIN_PIN;

  // Y
  stepperAttrArray[STEPARRAY_EL_Y].coordinateName = 'Y';
  stepperAttrArray[STEPARRAY_EL_Y].move_and_dir = MOVEMENT_DISABLED;
  stepperAttrArray[STEPARRAY_EL_Y].currentPosInMm = 0.0;
  stepperAttrArray[STEPARRAY_EL_Y].targetPosInMm = 0.0;
  stepperAttrArray[STEPARRAY_EL_Y].counted_steps_1 = 0;
  stepperAttrArray[STEPARRAY_EL_Y].counted_steps_2 = 0;
  stepperAttrArray[STEPARRAY_EL_Y].counted_steps_3 = 0;
  stepperAttrArray[STEPARRAY_EL_Y].maxSteps_counter_3 = Y_MAX_COUNTER_3;
  stepperAttrArray[STEPARRAY_EL_Y].maxSteps_counter_2 = Y_MAX_COUNTER_2;
  stepperAttrArray[STEPARRAY_EL_Y].stepsPerMm = Y_STEPS_PER_MM;
  stepperAttrArray[STEPARRAY_EL_Y].step_pin = Y_STEP_PIN;
  stepperAttrArray[STEPARRAY_EL_Y].dir_pin = Y_DIR_PIN;
  //stepperAttrArray[STEPARRAY_EL_Y].enable_pin = Y_ENABLE_PIN;
  stepperAttrArray[STEPARRAY_EL_Y].switch_min_pin = Y_MIN_PIN;

  // Z
#if ENABLE_Z_AXIS
  stepperAttrArray[STEPARRAY_EL_Z].coordinateName = 'Z';
  stepperAttrArray[STEPARRAY_EL_Z].move_and_dir = MOVEMENT_DISABLED;
  stepperAttrArray[STEPARRAY_EL_Z].currentPosInMm = 0.0;
  stepperAttrArray[STEPARRAY_EL_Z].targetPosInMm = 0.0;
  stepperAttrArray[STEPARRAY_EL_Z].counted_steps_1 = 0;
  stepperAttrArray[STEPARRAY_EL_Z].counted_steps_2 = 0;
  stepperAttrArray[STEPARRAY_EL_Z].counted_steps_3 = 0;
  stepperAttrArray[STEPARRAY_EL_Z].maxSteps_counter_3 = Z_MAX_COUNTER_3;
  stepperAttrArray[STEPARRAY_EL_Z].maxSteps_counter_2 = Z_MAX_COUNTER_2;
  stepperAttrArray[STEPARRAY_EL_Z].stepsPerMm = Z_STEPS_PER_MM;
  stepperAttrArray[STEPARRAY_EL_Z].step_pin = Z_STEP_PIN;
  stepperAttrArray[STEPARRAY_EL_Z].dir_pin = Z_DIR_PIN;
  //stepperAttrArray[STEPARRAY_EL_Z].enable_pin = Z_ENABLE_PIN;
  stepperAttrArray[STEPARRAY_EL_Z].switch_min_pin = Z_MIN_PIN;
#endif

  // turn them off to start
  //disable_all_steppers();
  
  for (short i = 0; i < STEPMOTOR_AMOUNT; i++) {
    pinMode(stepperAttrArray[i].step_pin, OUTPUT); // Step-Pin
    pinMode(stepperAttrArray[i].dir_pin, OUTPUT); // Dir-Pin
    //pinMode(stepperAttrArray[i].enable_pin, OUTPUT); // Enable-Pin
    pinMode(stepperAttrArray[i].switch_min_pin, INPUT_PULLUP); // Min-Limiter-Switch-Pin
  }
}

/*
void enable_stepper(short stepperAttrArrayEl)
{
  digitalWrite(stepperAttrArray[stepperAttrArrayEl].enable_pin, HIGH); // ToDo: enable pin is not attached to the motor driver thru the telefon cable
}

void disable_stepper(short stepperAttrArrayEl)
{
  digitalWrite(stepperAttrArray[stepperAttrArrayEl].enable_pin, LOW); // ToDo: enable pin is not attached to the motor driver thru the telefon cable
  stepperAttrArray[stepperAttrArrayEl].move_and_dir = MOVEMENT_DISABLED;
}

void disable_all_steppers()
{
  for (short el = 0; el < STEPMOTOR_AMOUNT; el++) {
    digitalWrite(stepperAttrArray[el].enable_pin, LOW); // ToDo: enable pin is not attached to the motor driver thru the telefon cable
  }
}
*/

void stop_all_movements()
{
  for (short el = 0; el < STEPMOTOR_AMOUNT; el++) {
    stepperAttrArray[el].move_and_dir = MOVEMENT_DISABLED;
  }
  //disable_all_steppers();
#if ENABLE_Z_SERVO
  servo.detach();
#endif
}

void goto_machine_zero()
{
  Serial.println("GoToHome");
  
  stop_all_movements();
  
  // first move Z-axis to zero, so start with maximum array element
  for (short el = (STEPMOTOR_AMOUNT - 1); el >= 0; el--) {
    move_to_min(el);
  }
  
  limit_error = LIMIT_NO_ERROR;
  
  Serial.println("GoToHome done");
}

void move_to_min(short stepperAttrArrayEl)
{
  // increase Speed to max while moving to min
  Timer1.initialize(FAST_STEP_SPEED_IN_MICROSEC);
  
  //enable_stepper(stepperAttrArrayEl);
  stepperAttrArray[stepperAttrArrayEl].move_and_dir = MOVEMENT_DIR_L_D_C;
  
  while(!(read_switch(stepperAttrArray[stepperAttrArrayEl].switch_min_pin))){ }
    
  // slower movement when min-point received
  Timer1.initialize(SLOW_STEP_SPEED_IN_MICROSEC);
  
  // drive back until pin is released
  stepperAttrArray[stepperAttrArrayEl].move_and_dir = MOVEMENT_DIR_R_U_O;
  
  while(read_switch(stepperAttrArray[stepperAttrArrayEl].switch_min_pin)){ }
  
  stepperAttrArray[stepperAttrArrayEl].move_and_dir = MOVEMENT_DISABLED;
  //disable_stepper(stepperAttrArrayEl);
      
  // set Step-Counter to zero (or here a little higher, that the counter does not fall underneath 0 when you drive al little further
  stepperAttrArray[stepperAttrArrayEl].counted_steps_3 = 0;
  stepperAttrArray[stepperAttrArrayEl].counted_steps_2 = 0;
  stepperAttrArray[stepperAttrArrayEl].counted_steps_1 = 250;
  
  // set positions to zero
  stepperAttrArray[stepperAttrArrayEl].currentPosInMm = 0.0;
  stepperAttrArray[stepperAttrArrayEl].targetPosInMm = 0.0;
  
  // default Speed for further operations
  Timer1.initialize(DEFAULT_STEP_SPEED_IN_MICROSEC);

  sprintf(writeSerialBuffer, "Reached min of Axis: %c", stepperAttrArray[stepperAttrArrayEl].coordinateName);
  Serial.println(writeSerialBuffer);
}

void do_step()
{
  for (short el = 0; el < STEPMOTOR_AMOUNT; el++)
  {
    if (stepperAttrArray[el].move_and_dir != MOVEMENT_DISABLED)
    {
      switch (stepperAttrArray[el].move_and_dir << 2 | digitalRead(stepperAttrArray[el].step_pin) << 1 | digitalRead(stepperAttrArray[el].dir_pin))
      {
        case 0: /* 0 00 -> 10 */
        case 5: /* 1 01 -> 11 */
          digitalWrite(stepperAttrArray[el].step_pin, HIGH);
          break;
        case 1: /* 0 01 -> 00 */
        case 7: /* 1 11 -> 10 */
          digitalWrite(stepperAttrArray[el].dir_pin, LOW);
          break;
        case 2: /* 0 10 -> 11 */
        case 4: /* 1 00 -> 01 */   
          digitalWrite(stepperAttrArray[el].dir_pin, HIGH);
          break;
        case 3: /* 0 11 -> 01 */
        case 6: /* 1 10 -> 00 */
          digitalWrite(stepperAttrArray[el].step_pin, LOW);
          break;
      }
      
      // MOVEMENT_DIR_R_U_O
      if (stepperAttrArray[el].move_and_dir == MOVEMENT_DIR_R_U_O) {
        if (stepperAttrArray[el].counted_steps_1 < MAX_COUNTER_VAL) {
          stepperAttrArray[el].counted_steps_1++;
        } else {
          if (stepperAttrArray[el].counted_steps_2 < MAX_COUNTER_VAL) {
            stepperAttrArray[el].counted_steps_2++;
          } else {
            if (stepperAttrArray[el].counted_steps_3 < MAX_COUNTER_VAL) {
              stepperAttrArray[el].counted_steps_3++;
            } else {
              limit_error = LIMIT_ERROR_MAX;
              return;
            }
            stepperAttrArray[el].counted_steps_2 = 0;
          }
          stepperAttrArray[el].counted_steps_1 = 0;
        }
      }
      // MOVEMENT_DIR_L_D_C
      else {
        if (stepperAttrArray[el].counted_steps_1 > 0) {
          stepperAttrArray[el].counted_steps_1--;
        } else {
          if (stepperAttrArray[el].counted_steps_2 > 0) {
            stepperAttrArray[el].counted_steps_2--;
          } else {
            if (stepperAttrArray[el].counted_steps_3 > 0) {
              stepperAttrArray[el].counted_steps_3--;
            } else {
              limit_error = LIMIT_ERROR_MIN;
              return;
            }
            stepperAttrArray[el].counted_steps_2 = MAX_COUNTER_VAL;
          }
          stepperAttrArray[el].counted_steps_1 = MAX_COUNTER_VAL;
        }
      }
    }
  }
}

short reached_target_or_end(short stepperAttrArrayEl)
{  
  // stop if we reached end-switch
  if (read_switch(stepperAttrArray[stepperAttrArrayEl].switch_min_pin) && stepperAttrArray[stepperAttrArrayEl].move_and_dir == MOVEMENT_DIR_L_D_C) {
		return REACHED_MIN_SW;
  }
  // ToDo
  /*
  // stop if we're on target
  else if ((stepperAttrArray[stepperAttrArrayEl].move_and_dir == MOVEMENT_DIR_L_D_C && stepperAttrArray[stepperAttrArrayEl].currentPosInMm >= stepperAttrArray[stepperAttrArrayEl].targetPosInMm) || 
      (stepperAttrArray[stepperAttrArrayEl].move_and_dir == MOVEMENT_DIR_L_D_C && stepperAttrArray[stepperAttrArrayEl].currentPosInMm <= stepperAttrArray[stepperAttrArrayEl].targetPosInMm)) {
    return REACHED_TARGET;
  }
  */
  else {
    // stop if we're on maximum or minimum
    if (stepperAttrArray[stepperAttrArrayEl].move_and_dir == MOVEMENT_DIR_R_U_O && stepperAttrArray[stepperAttrArrayEl].counted_steps_3 >= stepperAttrArray[stepperAttrArrayEl].maxSteps_counter_3 && stepperAttrArray[stepperAttrArrayEl].counted_steps_2 >= stepperAttrArray[stepperAttrArrayEl].maxSteps_counter_2)
      return REACHED_MAX;
    else if (stepperAttrArray[stepperAttrArrayEl].move_and_dir == MOVEMENT_DIR_L_D_C && (stepperAttrArray[stepperAttrArrayEl].counted_steps_3 < 0 || stepperAttrArray[stepperAttrArrayEl].counted_steps_2 < 0 || stepperAttrArray[stepperAttrArrayEl].counted_steps_1 < 0))
      return REACHED_MIN;
  }
  
	// being able to step
	return REACHED_NOTHING;
}

bool read_switch(byte pin)
{
	// dual read as crude debounce
	return !digitalRead(pin) && !digitalRead(pin);
}

void calcCurrentPositionInMm(short stepperAttrArrayEl)
{
  long steps = stepperAttrArray[stepperAttrArrayEl].counted_steps_1 + (stepperAttrArray[stepperAttrArrayEl].counted_steps_2 * 256L) + (stepperAttrArray[stepperAttrArrayEl].counted_steps_3 * 256L * 256L);
  stepperAttrArray[stepperAttrArrayEl].currentPosInMm = (steps * stepperAttrArray[stepperAttrArrayEl].stepsPerMm);
}

short determineMovementForNewTargetPos(short stepperAttrArrayEl)
{
  calcCurrentPositionInMm(stepperAttrArrayEl);
  if (stepperAttrArray[stepperAttrArrayEl].targetPosInMm < stepperAttrArray[stepperAttrArrayEl].currentPosInMm)
    return MOVEMENT_DIR_L_D_C;
  else if (stepperAttrArray[stepperAttrArrayEl].targetPosInMm > stepperAttrArray[stepperAttrArrayEl].currentPosInMm)
    return MOVEMENT_DIR_R_U_O;
  else
    return MOVEMENT_DISABLED;
}

// look for the command if it exists.
bool has_command(char key, char instruction[], int string_size)
{
	for (byte i=0; i<string_size; i++)
	{
		if (instruction[i] == key)
			return true;
	}
	
	return false;
}

// look for the number that appears after the char key and return it
double search_string(char key, char instruction[], int string_size)
{
	char temp[10] = "";

	for (byte i=0; i<string_size; i++)
	{
		if (instruction[i] == key)
		{
			i++;      
			int k = 0;
			while (i < string_size && k < 10)
			{
				if (instruction[i] == 0 || instruction[i] == ' ')
					break;

				temp[k] = instruction[i];
				i++;
				k++;
			}
			return strtod(temp, NULL);
		}
	}
	
	return 0;
}

// Read the string and execute instructions
void process_string(char instruction[], int size)
{
	// the character / means delete block... used for comments and stuff.
	if (instruction[0] == '/')
	{
		return;
	}
  
  if (has_command('S', instruction, size))
  {
    stop_all_movements();
  }
  else if (has_command('H', instruction, size))
  {
    goto_machine_zero();
  }
  else if (has_command('D', instruction, size))
	{
    Timer1.initialize(DEFAULT_STEP_SPEED_IN_MICROSEC);
    
    for (short el = 0; el < STEPMOTOR_AMOUNT; el++) {
      if (has_command(stepperAttrArray[el].coordinateName, &instruction[1], size-1)) {
        int temp = search_string(stepperAttrArray[el].coordinateName, &instruction[1], size-1);
        if (temp == 1) {
          stepperAttrArray[el].move_and_dir = MOVEMENT_DIR_R_U_O;
        } else if (temp == -1) {
          stepperAttrArray[el].move_and_dir = MOVEMENT_DIR_L_D_C;
        } else {
          stepperAttrArray[el].move_and_dir = MOVEMENT_DISABLED;
        }
      }
    }
  }
  else if (has_command('C', instruction, size))
  {
#if ENABLE_Z_SERVO
    int temp = search_string('C', instruction, size);

    if (temp > 0) {
      if (servoCurrentPos < SERVO_MAX_POS)
        servoStatus = MOVEMENT_DIR_L_D_C;
    }
    else if (temp < 0) {
      if (servoCurrentPos > SERVO_MIN_POS)
        servoStatus = MOVEMENT_DIR_R_U_O;
    }
    else
      servoStatus = MOVEMENT_DISABLED;
#endif
  }
	else if (has_command('X', instruction, size) ||
		has_command('Y', instruction, size) ||
		has_command('Z', instruction, size))
	{
    for (short el = 0; el < STEPMOTOR_AMOUNT; el++) {
      if (has_command(stepperAttrArray[el].coordinateName, instruction, size)) {
        stepperAttrArray[el].targetPosInMm = search_string(stepperAttrArray[el].coordinateName, instruction, size);
        stepperAttrArray[el].move_and_dir = determineMovementForNewTargetPos(el);
      }
    }
  }
  
	// tell our host we're done.
  //Serial.println("ok");
}

// init our string processing
void init_process_string()
{
	// init our command
	for (byte i=0; i<  COMMAND_SIZE; i++)
		commands[i] = 0;
	serial_count = 0;
}