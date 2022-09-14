

/*





To do:

  Essentials for implementation

  1. Test wifi signal strength
    -> Implement opening on webserver message
    -> Implement sending log message to server somehow
    -> Implement powering off 24v on webseerver message
    -> Implement send server message on outer button press
    -> Implement send server message on getting stuck
    -> Implement watchdog to check it isn't stuck open
    
    
  Optional:
    Implement "closed" sensor
    Implement state storage and recovery after reset/power-out
    Implement persistent logging maybe
    Implement angle sensor
    Implement remote control
    Implement fall back GSM control


  Parameters to adjust for final deployment
      MOTOR_CURRENT_LIMIT
      IGNORE_ENDSTOP_DURATION
      WAIT_LOOP_DELAY_MS
      DELAY_BEFORE_TRYING_TO_CLEAR_STUCK
      DELAY_BEFORE_CLOSING_GATE_MS
      FULL_PWM_RATE
      



*/







#include <Arduino.h>

#define PIN_END_TRIP 19
#define END_TRIP_HIT false

#define PIN_DRIVER_OVERLOAD_L 23
#define PIN_DRIVER_OVERLOAD_R 18
#define DRIVER_OVERLOADED true

#define PIN_PWM_OPEN 17
#define PIN_PWM_CLOSE 16
#define PWM_LOCK_SIDE 0

#define PIN_MOTOR_CURRENT 36

#define DEFAULT_MAX_MOTOR_DURATION_MS 10000

#define MOTOR_CURRENT_LIMIT 4.000

#define IGNORE_ENDSTOP_DURATION_MS 1500

#define WAIT_LOOP_DELAY_MS 500

#define PWM_CHANNEL_OPEN 0
#define PWM_CHANNEL_CLOSE 1

#define PIN_TWENTY_FOUR_VOLT_RELAY 13
#define TWENTY_FOUR_VOLT_RELAY_NORMAL true
#define TWENTY_FOUR_VOLT_RELAY_DISCONNECTED false

#define DELAY_BEFORE_TRYING_TO_CLEAR_STUCK_MS 10000

#define PIN_BEAM 14
#define BEAM_BROKEN false

#define PIN_NEAR_BUTTON 27
#define PIN_INSIDE_BUTTON 26
#define PIN_OUTSIDE_BUTTON 25
#define BUTTON_PRESSED false

#define DELAY_BEFORE_CLOSING_GATE_MS 30000

#define FULL_PWM_RATE 200
#define PWM_FREQUENCY 10000

enum GateStates 
{
  CLOSED='C',
  OPENING='O',
  STUCK_WHILE_OPENING='s',
  WAITING_TO_CLOSE='W',
  CLOSING='V',
  STUCK_WHILE_CLOSING='x'
};

struct GateState
{
  GateStates state;
  uint32_t state_change_time;
  uint8_t pwm_speed;
  uint32_t timeout_time;
  uint16_t current_zero;
  uint32_t time_to_ignore_endstops_ms;
  float amps_per_bit=5.0/1402;
};



GateState gate;


char temp_buff[255];


void change_gate_state(GateStates new_state)
{
  Serial.printf("OOOO Changing State from : %c -> %c\n",(char)gate.state,(char)new_state);
  gate.state=new_state;
  gate.state_change_time=millis();
}


void send_message(char * mess)
{
  Serial.printf("*** %s\n",mess);
}


bool get_beam_broken()
{
  return (digitalRead(PIN_BEAM)==BEAM_BROKEN);
}

bool get_inner_button_pressed()
{
  return (digitalRead(PIN_INSIDE_BUTTON)==BUTTON_PRESSED);
}

bool get_outer_button_pressed()
{
  return (digitalRead(PIN_OUTSIDE_BUTTON)==BUTTON_PRESSED);
}

bool get_near_button_pressed()
{
  return (digitalRead(PIN_NEAR_BUTTON)==BUTTON_PRESSED);
}

void send_gate_open_request_message()
{
  send_message((char *)"Outer gate button pressed");
}



void calibrate_motor_current()
{
  /* CALL ONLY WHEN MOTOR HAS BEEN OFF FOR LONG ENOUGH FOR CURRENT TO ZERO!!!
  */
  uint16_t raw=analogRead(PIN_MOTOR_CURRENT);
  gate.current_zero=raw;
  Serial.printf("New motor zero current reading set to: %d\n",raw);
}

void set_up_motor()
{
  pinMode(PIN_END_TRIP,INPUT_PULLUP);
  pinMode(PIN_DRIVER_OVERLOAD_L,INPUT);
  pinMode(PIN_DRIVER_OVERLOAD_R,INPUT);
  pinMode(PIN_PWM_OPEN,OUTPUT);
  digitalWrite(PIN_PWM_OPEN,PWM_LOCK_SIDE);//analogWrite(PIN_PWM_LEFT,PWM_LOCK_SIDE);
  pinMode(PIN_PWM_CLOSE,OUTPUT);
  digitalWrite(PIN_PWM_CLOSE,PWM_LOCK_SIDE);//analogWrite(PIN_PWM_RIGHT,PWM_LOCK_SIDE);
  
  pinMode(PIN_MOTOR_CURRENT,INPUT);
  analogSetWidth(12);

  delay(100); // Allow time for current to zero
  calibrate_motor_current();

  ledcSetup(PWM_CHANNEL_OPEN,PWM_FREQUENCY,8);
  ledcSetup(PWM_CHANNEL_CLOSE,PWM_FREQUENCY,8);
  ledcAttachPin(PIN_PWM_OPEN,PWM_CHANNEL_OPEN);
  ledcAttachPin(PIN_PWM_CLOSE,PWM_CHANNEL_CLOSE);

  // Immediately stop the motor!
  ledcWrite(PWM_CHANNEL_OPEN,0);
  ledcWrite(PWM_CHANNEL_CLOSE,0);


}

float get_motor_current()
{
  /*

      Full scale 4096 is approximately 3.3v on the analog input
      Sensor goes  2.4 +/- 2.4v for +/- 5 Amps of motor current
      But this is divided by 2 before being read
      So zero amps, will be 2.4v - actually 2.26v measured
      or 1.130 v at the analog input
      A change of 1.13v = 5 amps
      So assuming 1.13v will read around 1402

  */


  int raw=analogRead(PIN_MOTOR_CURRENT);
  float current=-gate.amps_per_bit*(raw-gate.current_zero);
  //Serial.printf("\t\tRaw motor current measurement: %d = %f Amps\n",raw,current);
  return current;

}





bool motor_drivers_ok()
{
  return true;

  bool left_overload=digitalRead(PIN_DRIVER_OVERLOAD_L);
  bool right_overload=digitalRead(PIN_DRIVER_OVERLOAD_R);
  Serial.printf("\t\tMotor driver overload states: %d, %d\n",left_overload,right_overload);
  if (left_overload || right_overload)
  {
    Serial.println("DRIVER OVERLOADED!!!!");
    return false;
  }
  return true;
}


void move_gate(bool open,uint8_t pwm,uint32_t timeout_ms=DEFAULT_MAX_MOTOR_DURATION_MS)
{
  Serial.printf("Starting motor in direction %s, with pwm of: %d for a max time of: %f seconds\n",open?"open":"close",pwm,timeout_ms/1000.0);
  set_up_motor(); // Also stops the motor if it was running
  if (!motor_drivers_ok())
  {
    Serial.println("Not starting motor due to driver issue");
    return;
  }

  // Start the motor and return immediately
  if (open)
  {
    ledcWrite(PWM_CHANNEL_CLOSE,0);//digitalWrite(PIN_PWM_RIGHT,PWM_LOCK_SIDE);//analogWrite(PIN_PWM_RIGHT,PWM_LOCK_SIDE);
    delayMicroseconds(100);
    ledcWrite(PWM_CHANNEL_OPEN,pwm);//digitalWrite(PIN_PWM_LEFT,1);//analogWrite(PIN_PWM_LEFT,pwm);
    change_gate_state(OPENING);
  } else {
    ledcWrite(PWM_CHANNEL_OPEN,0);//digitalWrite(PIN_PWM_LEFT,PWM_LOCK_SIDE);//analogWrite(PIN_PWM_LEFT,PWM_LOCK_SIDE);
    delayMicroseconds(100);
    ledcWrite(PWM_CHANNEL_CLOSE,pwm);//digitalWrite(PIN_PWM_RIGHT,1);//analogWrite(PIN_PWM_RIGHT,pwm);
    change_gate_state(CLOSING);
  }
  gate.timeout_time=millis()+timeout_ms;
  Serial.printf("\t\tMotor>>>>Now>>>>Running>>> %s",open?"OPEN":"CLOSE");

}

void stop_motor(bool success)
{
  set_up_motor();
  if (success)
  {
    switch (gate.state)
    {
      case OPENING:
        change_gate_state(WAITING_TO_CLOSE);
        send_message((char *)"Gate now open");
        break;
      case CLOSING:
        change_gate_state(CLOSED);
        send_message((char *)"Gate now closed");
        break;
      default:
        sprintf(temp_buff,"Stop motor called when gate was neither opening nor closing (state=%d)",gate.state);
        send_message(temp_buff);
    }
  } else {
    send_message((char*)"Motor stopped with error condition");
    switch (gate.state)
    {
      case OPENING:
        change_gate_state(STUCK_WHILE_OPENING);
        send_message((char *)"Gate stuck during opening");
        break;

      case CLOSING:
        change_gate_state(STUCK_WHILE_CLOSING);
        send_message((char *)"Gate stuck during closing");
        break;

      default:
        sprintf(temp_buff,"Stop motor called when gate was neither opening nor closing (state=%d)",gate.state);
        send_message(temp_buff);
    } 
  }
}

void start_opening_gate(uint32_t time_to_ignore_endstops=IGNORE_ENDSTOP_DURATION_MS)
{
  gate.time_to_ignore_endstops_ms=time_to_ignore_endstops;
  move_gate(true,FULL_PWM_RATE,40000);
  change_gate_state(OPENING);
}

void start_closing_gate()
{
  move_gate(false,FULL_PWM_RATE,40000);
  change_gate_state(CLOSING);
}

bool end_trip_reached()
{
  bool raw=digitalRead(PIN_END_TRIP);
  Serial.printf("\t\tEndstop hit?: %s\n",raw?"No":"Yes");
  return (raw==END_TRIP_HIT);
}

bool run_motor()
{
  /* Non-blocking
    Checks state of motor:
        arrived at endstop?
        driver overloaded?
        current too-high?
        timed out?

          - if so stops motor and returns false

    returns true otherwise
  */

  // Just return straight away if the motor isn't meant to be running at the moment!
  if (gate.state!=OPENING && gate.state!=CLOSING)
  {
    return false; // Like reached the end already! 
  }

  // Check too much current
  float motor_current=get_motor_current(); 
  Serial.printf("\tFound motor current: %f",motor_current);
  if (motor_current>MOTOR_CURRENT_LIMIT)
  {
    Serial.printf("Motor taking too much current, stopping with fault!");
    stop_motor(false);
    return false;
  }

  // Check driver overload condition
  if (!motor_drivers_ok())
  {
    Serial.println("Motor driver overloaded, so stopping");
    stop_motor(true);
    return false;
  }

  // Check endstop reached
  if (end_trip_reached() && millis()>(gate.state_change_time+gate.time_to_ignore_endstops_ms))
  {
    Serial.println("Reached endstop, stopping motor!");
    stop_motor(true);
    return false;
  }

  // Check for timeout
  if (millis()>gate.timeout_time)
  {
    Serial.println("Stopping motor due to timeout");
    stop_motor(false);
    return false;
  }


  return true; // still moving!
}

void run_motor_until_done()
{
  while (run_motor())
  {
    delay(WAIT_LOOP_DELAY_MS);
    Serial.print(".");
  }
}

void set_24v_relay(bool on)
{

}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(100);
  Serial.println("Started....");
  pinMode(PIN_TWENTY_FOUR_VOLT_RELAY,OUTPUT);
  digitalWrite(PIN_TWENTY_FOUR_VOLT_RELAY,TWENTY_FOUR_VOLT_RELAY_NORMAL);
  pinMode(PIN_BEAM,INPUT);
  pinMode(PIN_NEAR_BUTTON,INPUT);
  pinMode(PIN_INSIDE_BUTTON,INPUT);
  pinMode(PIN_OUTSIDE_BUTTON,INPUT);
  change_gate_state(CLOSED);

}

void loop() {

  // Main loop

  uint32_t time_since_state_change=millis()-gate.state_change_time;

  bool mot_done=run_motor();
  if (get_beam_broken())
  {
    // Send open command
    uint32_t time_to_ignore_endstops=IGNORE_ENDSTOP_DURATION_MS;
    if (gate.state==CLOSING)
    {
      time_to_ignore_endstops=500;
    }
    start_opening_gate(time_to_ignore_endstops);
  }
  if (get_outer_button_pressed())
  {
    send_gate_open_request_message();
    start_opening_gate();
  }
  if (get_inner_button_pressed() || get_near_button_pressed())
  {
    start_opening_gate();
  }

  // Timeout stuck state
  if (time_since_state_change>DELAY_BEFORE_TRYING_TO_CLEAR_STUCK_MS)
  {
    if (gate.state==STUCK_WHILE_CLOSING || gate.state==STUCK_WHILE_OPENING)
    {
      Serial.println("!!!!!Waited to long enough for stuck gate to clear; now trying open...");
      start_opening_gate(); // Try to open the gate to clear the error
    }
  }
  // Close gate after delay
  if ((time_since_state_change>DELAY_BEFORE_CLOSING_GATE_MS) && (gate.state==WAITING_TO_CLOSE))
  {
    start_closing_gate();
  }

  delay(WAIT_LOOP_DELAY_MS);


}