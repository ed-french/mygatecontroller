

/*





To do:



    
    
  Optional:
    Implement "closed" sensor
    Implement state storage and recovery after reset/power-out
    Implement persistent logging maybe
    Implement angle sensor
    Implement remote control
    Implement fall back GSM control


  Parameters to adjust for final deployment
      WAIT_LOOP_DELAY_MS
      DELAY_BEFORE_TRYING_TO_CLEAR_STUCK
      DELAY_BEFORE_CLOSING_GATE_MS
      HEARTBEAT_INTERVAL
      ASYNC_HTTP_LOGLEVEL_
      HTTP_REQUEST_INTERVAL





*/



#if !( defined(ESP8266) ||  defined(ESP32) )
  #error This code is intended to run on the ESP8266 or ESP32 platform! Please check your Tools->Board setting.
#endif


#define ASYNC_HTTP_REQUEST_GENERIC_VERSION_MIN_TARGET      "AsyncHTTPRequest_Generic v1.9.1"
#define ASYNC_HTTP_REQUEST_GENERIC_VERSION_MIN             1009001



#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
#include <AsyncHTTPRequest_Generic.h>
#include <Preferences.h>



#include "credentials.h" // Just includes ssid and password declarations and ota_username and ota_password

#define PIN_POSITION_SERIAL_RX 19
#define PIN_POSITION_SERIAL_DUMMY_TX 15
#define POSITION_SERIAL_BAUD_RATE 115200




#define PIN_DRIVER_OVERLOAD_L 23
#define PIN_DRIVER_OVERLOAD_R 18
#define DRIVER_OVERLOADED true

#define PIN_PWM_OPEN 17
#define PIN_PWM_CLOSE 16
#define PWM_LOCK_SIDE 0

#define PIN_MOTOR_CURRENT 36






#define WAIT_LOOP_DELAY_MS 50

#define PWM_CHANNEL_OPEN 0
#define PWM_CHANNEL_CLOSE 1

#define PIN_TWENTY_FOUR_VOLT_RELAY 13
#define TWENTY_FOUR_VOLT_RELAY_NORMAL true
#define TWENTY_FOUR_VOLT_RELAY_DISCONNECTED false

#define TWENTY_FOUR_VOLT_RELAY_INTERRUPT_TIME_MS 10000

#define DELAY_BEFORE_TRYING_TO_CLEAR_STUCK_MS 10000

#define PIN_BEAM 14
#define BEAM_BROKEN false

#define PIN_NEAR_BUTTON 27
#define PIN_INSIDE_BUTTON 26
#define PIN_OUTSIDE_BUTTON 25
#define BUTTON_PRESSED false
#define BUTTON_PRESS_CERTAINTY_TIME_MS 75
#define BUTTON_UNUSED_TIMEOUT_MS 2000

#define DELAY_BEFORE_CLOSING_GATE_MS 70000

#define PWM_FREQUENCY 10000
#define HEARTBEAT_INTERVAL_MS 600000 // 5 minutes=300000

#define WIFI_AT_BOOT_TIMEOUT 50000 // 50 seconds


#define RAM_PARAM_MAX_SPEED 255
#define RAM_PARAM_ACCEL 5
#define RAM_PARAM_MAX_START_CURRENT 3.0
#define RAM_PARAM_MAX_START_PWM 100
#define RAM_PARAM_END_STALL_CURRENT 3.0
#define RAM_PARAM_DEFAULT_BLOCK_CURRENT_LIMIT 5.0
#define RAM_PARAM_DESTINATION_TOLERANCE 0.3
#define RAM_PARAM_DECCELERATION_TRIGGER_ZONE 0.9
#define RAM_PARAM_DECCELERATION_PER_LOOP 20
#define RAM_PARAM_MIN_RUNNING_PWM 100 // Approx 10v
#define RAM_PARAM_MINIMUM_COMMAND_MOVE 1.0
#define RAM_PARAM_START_PWM 50
#define RAM_PARAM_MIN_MOVEMENT_TO_BE_STARTED 0.3
#define RAM_PARAM_START_TIMEOUT 8000
#define RAM_PARAM_ACCEL_TIMEOUT 8000
#define RAM_PARAM_RUNNING_TIMEOUT 40000
#define RAM_PARAM_DECCELERATION_TIMEOUT 10000
#define RAM_PARAM_ERROR_COOLDOWN 10000
#define RAM_PARAM_SAFE_STALL_PWM 40 // About 4V should be OK stalled for a  while like that
#define RAM_PARAM_POSITION_AT_OPEN -0.2
#define RAM_PARAM_POSITION_AT_CLOSE 6.2




// Async http lib defines...
  // Level from 0-4
  #define ASYNC_HTTP_DEBUG_PORT     Serial
  #define _ASYNC_HTTP_LOGLEVEL_     1

  // 300s = 5 minutes to not flooding
  #define HTTP_REQUEST_INTERVAL     1  //300

char hostname[]="nunshallpass";

Preferences preferences;


struct PositionState
{
  float last_good_reading;
  uint32_t time_last_good_reading;
  char last_rx_line[256];
  uint16_t count_sequential_bad_readings;
  bool last_reading_good;
};

PositionState position;

enum RamStates
{
  STOPPED='S',
  TRYING_TO_MOVE='T',
  ACCELERATING='A',
  FULL_SPEED='F',
  DECCELERATING='D',
  ERROR_COULD_NOT_START='J',
  ERROR_BLOCKED='B',
  ERROR_GLOBAL_CURRENT_LIMIT='I',
  ERROR_TIMEOUT='T'

};

struct RamState
{
  RamStates state;
  uint8_t pwm_now;
  float target_position;
  float starting_position;
  uint32_t start_time;
  uint32_t last_state_change_time;
  bool error_state;
  bool reached_position;
  uint8_t max_allowable_pwm;

};

RamState ram;

struct RelayState
{
  bool connected;
  uint32_t restore_time;
};



RelayState twentyfourvoltrelay;


char previous_restart_story[32];

uint32_t next_current_sent_time=0;
uint32_t time_since_gate_state_change;



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
  uint32_t timeout_time;
  uint16_t current_zero;
  float amps_per_bit=5.0/1402;
};

enum ButtonStates
{
  QUIET_UNPRESSED='Q',
  TENTATIVELY_PRESSED='T',
  PRESSED_WAITING_USE='P',
  USED_WAITING_RELEASE='W'
};

struct ButtonState
{
  uint8_t pin_number;
  ButtonStates state;
  uint32_t pressed_time;
};

ButtonState outer_button;

AsyncHTTPRequest request; 

void update_button(ButtonState &button)
{
  /*
      Update the passed button based on reading it
      returns quickly
  */
 bool now_pressed=(digitalRead(button.pin_number)==BUTTON_PRESSED);

  switch (button.state)
  {
    case QUIET_UNPRESSED:
      if (now_pressed)
      {
        button.state=TENTATIVELY_PRESSED;
        button.pressed_time=millis();
      }
     return;
    case TENTATIVELY_PRESSED:
      if (!now_pressed)
      {
        // button released before being confirmed- probably noise
        button.state=QUIET_UNPRESSED;
        return;
      }
      if (millis()>(button.pressed_time+BUTTON_PRESS_CERTAINTY_TIME_MS))
      {
        button.state=PRESSED_WAITING_USE;
        return;
      }
      return; // Just wait for tentative to be resolved either way
    case PRESSED_WAITING_USE:
      if (millis()>(button.pressed_time+BUTTON_UNUSED_TIMEOUT_MS))
      {
        if (now_pressed)
        {
          button.state=USED_WAITING_RELEASE;
          return;
        }
        return;
      }
    case USED_WAITING_RELEASE:
      if (!now_pressed)
      {
        button.state=QUIET_UNPRESSED;
      }
      return;
  }
}

void use_button_press(ButtonState &button)
{
  if (button.state==PRESSED_WAITING_USE)
  {
    if (digitalRead(button.pin_number)==BUTTON_PRESSED)
    {
      button.state=USED_WAITING_RELEASE;
    } else {
      button.state=QUIET_UNPRESSED;
    }
    return;
  }
}


GateState gate;


char temp_buff[511];

char message_buffer[1023]; // NOTE MAXIMUM MESSAGE SIZE!

AsyncWebServer server(80);

uint32_t next_heartbeat_time;

//HardwareSerial Serial2(2);


void requestCB(void *optParm, AsyncHTTPRequest *request, int readyState)
{
  (void) optParm;

  if (readyState == readyStateDone)
  {
    AHTTP_LOGDEBUG(F("\n**************************************"));
    AHTTP_LOGDEBUG1(F("Response Code = "), request->responseHTTPString());

    if (request->responseHTTPcode() == 200)
    {
      Serial.println(F("\n**************************************"));
      Serial.println(request->responseText());
      Serial.println(F("**************************************"));
    }
  }
}





void send_message(const char * message)
{
  Serial.printf("*SENDING* %s\n",message);
  
  static bool requestOpenResult;

  char url[1400];

  char next_to_add[5];

  strcpy(url,"http://192.168.1.125/rx_gate_message?message=");
  for (uint16_t i=0;i<strlen(message);i++)
  {
    char c=message[i];
    if (c==32)
    {
      strcpy(next_to_add,"%20");
    } else {
      next_to_add[0]=c;
      next_to_add[1]=0;
    }
    strcat(url,next_to_add);
  }

 
  
  if (request.readyState() == readyStateUnsent || request.readyState() == readyStateDone)
  {
    //requestOpenResult = request.open("GET", "http://worldtimeapi.org/api/timezone/Europe/London.txt");
    requestOpenResult = request.open("GET", url);
    Serial.printf("Sent to url: %s\n",url);
    if (requestOpenResult)
    {
      // Only send() if open() returns true, or crash
      request.send();
    }
    else
    {
      Serial.println(F("Can't send bad request"));
    }
  }
  else
  {
    Serial.println(F("Can't send request now as one is still pending!"));
  }
}



void restart_esp(const char * reason)
{
  preferences.putString("reason",reason);
  if (WiFi.status()==WL_CONNECTED)
  {
    sprintf(temp_buff,"Rebooting in 10 seconds due to %s",reason);
    send_message(temp_buff);
    delay(10000);
    ESP.restart();
  }
  ESP.restart(); // No point waiting if wifi is disconnected anyway!
  

}


void change_gate_state(GateStates new_state)
{
  Serial.printf("$$$$ Changing Gate State from : %c -> %c\n",(char)gate.state,(char)new_state);
  gate.state=new_state;
  gate.state_change_time=millis();
}




// void send_message(char * mess)
// {
//   Serial.printf("*** %s\n",mess);
// }


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
  update_button(outer_button);

  return (outer_button.state==PRESSED_WAITING_USE);
}

bool get_near_button_pressed()
{
  return (digitalRead(PIN_NEAR_BUTTON)==BUTTON_PRESSED);
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
  //pinMode(PIN_END_TRIP,INPUT_PULLUP);
  pinMode(PIN_DRIVER_OVERLOAD_L,INPUT);
  pinMode(PIN_DRIVER_OVERLOAD_R,INPUT);
  pinMode(PIN_PWM_OPEN,OUTPUT);
  digitalWrite(PIN_PWM_OPEN,PWM_LOCK_SIDE);//analogWrite(PIN_PWM_LEFT,PWM_LOCK_SIDE);
  pinMode(PIN_PWM_CLOSE,OUTPUT);
  digitalWrite(PIN_PWM_CLOSE,PWM_LOCK_SIDE);//analogWrite(PIN_PWM_RIGHT,PWM_LOCK_SIDE);
  
  pinMode(PIN_MOTOR_CURRENT,INPUT);
  analogSetWidth(12);

   // Allow time for current to zero
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

void set_motor_speed_and_direction(uint8_t new_pwm_rate,bool opening_needed)
{
  // Sends the new level to the motor
  if (opening_needed)
  {
    ledcWrite(PWM_CHANNEL_CLOSE,0);//digitalWrite(PIN_PWM_RIGHT,PWM_LOCK_SIDE);//analogWrite(PIN_PWM_RIGHT,PWM_LOCK_SIDE);
    delayMicroseconds(100);
    ledcWrite(PWM_CHANNEL_OPEN,new_pwm_rate);//digitalWrite(PIN_PWM_LEFT,1);//analogWrite(PIN_PWM_LEFT,pwm);
  } else {
    ledcWrite(PWM_CHANNEL_OPEN,0);//digitalWrite(PIN_PWM_LEFT,PWM_LOCK_SIDE);//analogWrite(PIN_PWM_LEFT,PWM_LOCK_SIDE);
    delayMicroseconds(100);
    ledcWrite(PWM_CHANNEL_CLOSE,new_pwm_rate);//digitalWrite(PIN_PWM_RIGHT,1);//analogWrite(PIN_PWM_RIGHT,pwm);
  }
  Serial.printf("~~~~~~~~~~~~~Motor speed set to %d\n",new_pwm_rate);
}

void stop_ram(const char * reason)
{
  Serial.printf("!K!K!K! Ram stop triggered by : %s\n\n",reason);
  set_motor_speed_and_direction(0,true);
}

void change_ram_state(RamStates new_state,const char * reason)
{
  Serial.printf(">>> Ram state changing from : %c to %c due to %s\n",ram.state,new_state,reason);
  ram.state=new_state;
  ram.last_state_change_time=millis();
  ram.error_state= (new_state==ERROR_BLOCKED || new_state==ERROR_COULD_NOT_START || new_state==ERROR_GLOBAL_CURRENT_LIMIT);

}


void start_ram(float new_target_destination,uint8_t max_pwm_allowed=RAM_PARAM_MAX_SPEED)
{
      ram.reached_position=false;
      Serial.printf("################\nRequest to move ram to : %f\n",new_target_destination);
      //delay(300);
      set_up_motor(); // Set the pins, zero the current measurement
      Serial.println("\t\t\t**&& motor now set up and ready to start");
      ram.starting_position=position.last_good_reading;
      ram.target_position=new_target_destination;
      ram.pwm_now=20;
      ram.max_allowable_pwm=max_pwm_allowed;
      Serial.printf("\t\t\tram destination set to: %f\n",ram.target_position);
      change_ram_state(TRYING_TO_MOVE,"ram start demanded");
      Serial.println("\t\t\tRam state now changed :->");

}

void update_ram()
{
  /*
        Runs state machine for the ram movement

        Parent loop must call the get_position to keep the global position up-to-date for this routine
        Does its own call to get the motor current

        updates the global ram struct 

        should return quickly having changed its own state and ajusted the PWM & direction signal as required

        will stop at end and in error condition for parent routine to deal with clearing, re-opening and messaging server




  */
  float motor_current=get_motor_current();
  if (abs(motor_current)>0.2)
  {
    Serial.printf(" i=%f, ",motor_current);
  }
  bool open_needed=(ram.target_position<position.last_good_reading);
  uint32_t time_since_last_state_change=millis()-ram.last_state_change_time;

  // Special case regardless of the ram state machine state- if the global current limit is exceeded stop right away!
  if (abs(motor_current)>max(RAM_PARAM_DEFAULT_BLOCK_CURRENT_LIMIT,RAM_PARAM_END_STALL_CURRENT))
  {
    sprintf(temp_buff,"detected global overcurrent of %f",motor_current);
    stop_ram(temp_buff); // Block
    change_ram_state(ERROR_GLOBAL_CURRENT_LIMIT,temp_buff);
    return;
  }
  float move_so_far=(position.last_good_reading-ram.starting_position);
  switch (ram.state)
  {
    case STOPPED:
      /* Ram will stay stopped unless it receives a new destination sufficiently different from the last requested */
      // if (abs(position.last_good_reading-ram.target_position)>RAM_PARAM_MINIMUM_COMMAND_MOVE)
      // {
      //   sprintf(temp_buff,"inferring move req as position is %f and target is %f",position.last_good_reading,ram.target_position);
      //   change_ram_state(TRYING_TO_MOVE,temp_buff);
      //   ram.start_time=millis();
      //   ram.pwm_now=RAM_PARAM_START_PWM;
      //   ram.starting_position=position.last_good_reading;
      //   set_motor_speed_and_direction(ram.pwm_now,open_needed);
      //   return;

      // } else {
      //   // We carry on in a stopped state doing nought!
      // }
      break;



    case TRYING_TO_MOVE:
      /* INCREASING CURRENT LOOKING FOR START OF MOTION */

      // Are we moving?
      
      if (abs(move_so_far)>RAM_PARAM_MIN_MOVEMENT_TO_BE_STARTED)
      {
        // Hey we're in motion now!
        change_ram_state(ACCELERATING,"first movement");
        sprintf(temp_buff,"Gate in motion to : %f",ram.target_position);
        send_message(temp_buff);
      } else {
        if (time_since_last_state_change>RAM_PARAM_START_TIMEOUT)
        {
          /* It's been too long since trying to start moving without any noticeable movement 
          */
          sprintf(temp_buff,"Ram couldn't detect movement when started after %f secs",time_since_last_state_change/1000.0);
          stop_ram(temp_buff);
          Serial.println(temp_buff);
          send_message(temp_buff);
          change_ram_state(ERROR_COULD_NOT_START,"no movement started");
          return;
        }
      }
      if (abs(motor_current)<RAM_PARAM_MAX_START_CURRENT)
      {
          // Increase the motor pwm but keep inside the max stalled pwm level
          uint8_t oldspeed=ram.pwm_now;
          ram.pwm_now=min(ram.pwm_now+RAM_PARAM_ACCEL,RAM_PARAM_MAX_START_PWM);
          Serial.printf("Accelerating from %d to %d\n",oldspeed,ram.pwm_now);
          set_motor_speed_and_direction(ram.pwm_now,open_needed);
        
      }
      break;

    case ACCELERATING:
      /* 
          CHECK IF WE REACHED THE TARGET
          CHECK FOR OVER_CURRENT
          CHECK FOR REACHED TOP PWM, ACCELERATE IF NOT
          CHECK FOR TIMEOUT
      */

      // Reduce voltage if over start current
      if (abs(motor_current)>RAM_PARAM_MAX_START_CURRENT)
      {
        // Reduce PWM by 6 acceleration steps or to safe stall level
        ram.pwm_now=max(ram.pwm_now-RAM_PARAM_ACCEL*6,RAM_PARAM_SAFE_STALL_PWM);
        set_motor_speed_and_direction(ram.pwm_now,open_needed);
        sprintf(temp_buff,"PWM to %d as motor current: %f",ram.pwm_now,motor_current);
        Serial.println(temp_buff);
        send_message(temp_buff);
      } else {
        // Maybe accelerate some more
        if (ram.pwm_now<ram.max_allowable_pwm)
        {
          uint8_t old_speed=ram.pwm_now;
          uint8_t ram_param_accel=RAM_PARAM_ACCEL;
          ram.pwm_now+=ram_param_accel;
          if (ram.pwm_now>ram.max_allowable_pwm)
          {
            ram.pwm_now=ram.max_allowable_pwm;
          }
          
          set_motor_speed_and_direction(ram.pwm_now,open_needed);
          Serial.printf("(we have just accelerated a bit more from %d to %d !)\n",old_speed,ram.pwm_now);
        } else {
          // We have reached full speed, so accelerating is done!
          Serial.println("Finished accelerating");
          change_ram_state(FULL_SPEED,"fully accelerated");
        }
      }
      
      // Check if we reached the target
      if (abs(position.last_good_reading-ram.target_position)<RAM_PARAM_DESTINATION_TOLERANCE)
      {
        // We arrived!
        
        sprintf(temp_buff,"arrived (pos=%f, target=%f) during acceleration",position.last_good_reading,ram.target_position);
        stop_ram(temp_buff);
        change_ram_state(STOPPED,temp_buff);
        ram.reached_position=true;
        sprintf(temp_buff,"Ram reached dest in accel: target=%f actual=%f",ram.target_position,position.last_good_reading);
        Serial.println(temp_buff);
        send_message(temp_buff);
        return;
      }

      // Check for timeout
      if (time_since_last_state_change>RAM_PARAM_ACCEL_TIMEOUT)
      {
        stop_ram("acceleration timeout");
        change_ram_state(ERROR_TIMEOUT,"acceleration timedout");
        strcpy(temp_buff,"Timeout accelerating the ram");
        Serial.println(temp_buff);
        send_message(temp_buff);
        return;
      }    


     break;

    case FULL_SPEED:
      /* 
          CHECK FOR OVER_CURRENT
          CHECK IF WE REACHED THE END
          CHECK IF WE ARE APPROACHING THE END AND SHOULD DECCELERATE
          CHECK FOR TIMEOUT
      */
      // Reduce voltage if over start current
      if (abs(motor_current)>RAM_PARAM_END_STALL_CURRENT)
      {
        // Reduce PWM to safe level
        ram.pwm_now=max(ram.pwm_now/2,RAM_PARAM_SAFE_STALL_PWM);
        set_motor_speed_and_direction(ram.pwm_now,open_needed);
        sprintf(temp_buff,"PWM to %d as motor current: %f",ram.pwm_now,motor_current);
        Serial.println(temp_buff);
        send_message(temp_buff);
      } 

      /* Check for arrival */
      if (abs(position.last_good_reading-ram.target_position)<RAM_PARAM_DESTINATION_TOLERANCE)
      {
        // We arrived!
        stop_ram("arrived during accel");
        change_ram_state(STOPPED,"arrived during full speed");
        ram.reached_position=true;
        sprintf(temp_buff,"Ram reached dest suddenly: target=%f actual=%f",ram.target_position,position.last_good_reading);
        Serial.println(temp_buff);
        send_message(temp_buff);
        return;
      }
      
      /* Have we nearly arrived and should decelerate */
      if (abs(position.last_good_reading-ram.target_position)<RAM_PARAM_DECCELERATION_TRIGGER_ZONE)
      {
        change_ram_state(DECCELERATING,"nearly there...");
        return;
      }

      if (time_since_last_state_change>RAM_PARAM_RUNNING_TIMEOUT)
      {
        stop_ram("timeout during full speed");
        change_ram_state(ERROR_TIMEOUT,"timed out during full speed");
        sprintf(temp_buff,"Timeout running the ram");
        Serial.println(temp_buff);
        send_message(temp_buff);
        return;
      }


      break;

    case DECCELERATING:
      /*
          CHECK FOR OVER_CURRENT
          CHECK IF WE REACHED THE END
          DECREASE SPEED DOWN TO MINIMUM
          CHECK FOR TIMEOUT
ram.pwm_now=max(RAM_PARAM_MIN_RUNNING_PWM,ram.pwm_now-RAM_PARAM_DECCELERATION_PER_LOOP);
      */
      if (abs(motor_current)>RAM_PARAM_END_STALL_CURRENT)
      {
        // Assume that we reached the end before we hit the destination
        stop_ram("endstop current limit");
        sprintf(temp_buff,"Increased current (%f) during decel. assuming finished.",motor_current);
        Serial.println(temp_buff);
        send_message(temp_buff);
        change_ram_state(STOPPED,"over-current whilst decelerating");
        return;
      } 

      // Check if we arrived in an orderly fashion!
      if (abs(position.last_good_reading-ram.target_position)<RAM_PARAM_DESTINATION_TOLERANCE)
      {
        // We arrived!
        stop_ram("orderly arrival");
        change_ram_state(STOPPED,"arrived orderly");
        ram.reached_position=true;
        sprintf(temp_buff,"Ram reached dest OK: target=%f actual=%f",ram.target_position,position.last_good_reading);
        Serial.println(temp_buff);
        send_message(temp_buff);
        return;
      }

      // Deccelerate towards minimum speed
      if (ram.pwm_now>RAM_PARAM_MIN_RUNNING_PWM)
      {
        ram.pwm_now=max(RAM_PARAM_MIN_RUNNING_PWM,ram.pwm_now-RAM_PARAM_DECCELERATION_PER_LOOP);
        set_motor_speed_and_direction(ram.pwm_now,open_needed);
      }

      // Check for decceleration timeout
      if (time_since_last_state_change>RAM_PARAM_DECCELERATION_TIMEOUT)
      {
        stop_ram("deccel tiemout");
        change_ram_state(ERROR_TIMEOUT,"timeout during decceleration");
        sprintf(temp_buff,"Timeout deccelerating and approaching the position");
        Serial.println(temp_buff);
        send_message(temp_buff);
        return;
      }

      break;

    case ERROR_COULD_NOT_START:
      /*
          Parent routine will clear the error down eventually!
      */

      if (time_since_last_state_change>3000)
      {
        strcpy(temp_buff,"Still stuck after error during start");
        Serial.println(temp_buff);
        send_message(temp_buff);
        ram.last_state_change_time=millis(); 

      }


      break;

     case ERROR_BLOCKED:
      /*
          Parent routine will clear the error down eventually!
      */

      if (time_since_last_state_change>3000)
      {
        strcpy(temp_buff,"Still stuck after error during start");
        Serial.println(temp_buff);
        send_message(temp_buff);
        ram.last_state_change_time=millis(); 
      }
      break;

     case ERROR_GLOBAL_CURRENT_LIMIT:
        /*
            Parent routine will clear the error down eventually!
        */

        if (time_since_last_state_change>3000)
        {
          strcpy(temp_buff,"Still stuck after error during start");
          Serial.println(temp_buff);
          send_message(temp_buff);
          ram.last_state_change_time=millis(); 
        }
        break;

      default:

        /* REPORT THE ERROR*/
          
        if (time_since_last_state_change>3000)
        {
          sprintf(temp_buff,"Still stuck in unknown state: %c",ram.state);
          Serial.println(temp_buff);
          send_message(temp_buff);
          ram.last_state_change_time=millis(); 
        }
        break;



  }


}



void set_up_position_sensor()
{

  //
  Serial2.begin(POSITION_SERIAL_BAUD_RATE,SERIAL_8N1,PIN_POSITION_SERIAL_RX,PIN_POSITION_SERIAL_DUMMY_TX);
  position.count_sequential_bad_readings=0;
  position.last_good_reading=-1.0;
  position.last_reading_good=false;
  strcpy(position.last_rx_line,"");
  position.time_last_good_reading=millis()-100000;
}

float get_position()
{
  /*


    
    Sets the global position variable with the detail, but returns the last good reading too
    
    N.B. If there is nothing new on the serial port, and it's less than 1000ms since a good reading, it will just return the same
    immediately to minimise waiting  
  */ 

  if (!Serial2.available() && millis()<(position.time_last_good_reading+1000))
  {
    //Serial.println("We don't have new data, but we've had a good recent reading, so repeating that...");
    return position.last_good_reading;
  }

  char last_line[256]; // We will pull lines until we have none left to make sure it's the most recent reading
  uint16_t length=0;
  while (true)
  {
    length=Serial2.readBytesUntil('\n',last_line,256);
    if(!Serial2.available() && (length>10))
    {
      last_line[length]='\0';
      strcpy(position.last_rx_line,last_line);
      break;
    }
  }
  // At this point we have the line as a char array
  // for now we are just going to look at the final parameter and the OK flag
  //Serial.printf("Rx'd from position sensor: %s\n",last_line);
  // find the last comma
  uint16_t comma_pos=-1;
  for (uint16_t i=strlen(last_line)-1;i>0;i--)
  {
    if (last_line[i]==',')
    {
      comma_pos=i;
      break;
    }
  }
  if (comma_pos==-1)
  {
    Serial.println("ERROR, FAILED TO FIND A COMMA!!!!! in the rx'd char array");
    position.last_reading_good=false;
    position.count_sequential_bad_readings++;
    return position.last_good_reading;
  }
  // Try and parse the position from the string...
  float reading=atof(last_line+comma_pos+1);
  //Serial.printf("\t\t\t->Reading: %f\t",reading);
  
  if (strncmp(last_line,"OK",2)==0 && reading>0.0 && reading<10.0)
  {
    // We have a good reading, so update stuff
    //Serial.println("=good");
    position.last_good_reading=reading;
    position.last_reading_good=true;
    position.time_last_good_reading=millis();
    position.count_sequential_bad_readings=0;
    Serial.printf(" %f, ",reading);
    return reading;
  }
  // Somehow a bad reading
  //Serial.println("=bad");
  position.last_reading_good=false;
  position.count_sequential_bad_readings++;

  // If we happen to start with a bad reading, then we
  // must use it anyway!!!
  if (position.last_good_reading<0.0)
  {
    position.last_good_reading=reading;
  }
  
  return position.last_good_reading;

}

void test_echo_position_sensor()
{
  while(true)
  {
    Serial.println(get_position());
    Serial.println();
    delay(100);
  }
}



void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(100);
  Serial.println("Started....");

  set_up_position_sensor();
  



  pinMode(PIN_TWENTY_FOUR_VOLT_RELAY,OUTPUT);
  digitalWrite(PIN_TWENTY_FOUR_VOLT_RELAY,TWENTY_FOUR_VOLT_RELAY_NORMAL);
  pinMode(PIN_BEAM,INPUT);
  pinMode(PIN_NEAR_BUTTON,INPUT);
  pinMode(PIN_INSIDE_BUTTON,INPUT);
  pinMode(PIN_OUTSIDE_BUTTON,INPUT);
  

  twentyfourvoltrelay.connected=true;

  preferences.begin("gatectrl", false);

  preferences.getString("reason",previous_restart_story,32);
  preferences.putString("reason","unknown");



  
  WiFi.setHostname(hostname);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  delay(1000);
  WiFi.begin(ssid, password);
  delay(1000);
  Serial.printf("Connecting to: %s\n",ssid);
  Serial.println("");

  uint32_t wifi_timeout=millis()+WIFI_AT_BOOT_TIMEOUT;

  // Wait for connection
  while (true)
  {
    if (WiFi.status() == WL_CONNECTED)
    {
      Serial.println("\nWifi now connected");
      break;
    }
    if (millis()>wifi_timeout)
    {
      Serial.println("Pointless message saying we are restarting to have another go at connecting");
      restart_esp("wifi boot timeout");
    }
    
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "Hi. NunShallPass is your gate slave.\n"
                        "\t* Use /open to open the gate\n"
                        "\tuse /reboot24v to switch on and off the 24v auxilliary output\n"
                        "\tUse /update to install new firmware remotely\n"
                        "\tUse /restart to reboot the ESP\n"
                        "\tUse /getfreeheap to see how much is free\n"
                        "\tUse /getpinstates to see what state and raw inputs are now\n"
                        "\tUse /move_to?position=1.244 to move the ram to a specific position"
                        "CLOSED'C'\tOPENING 'O'\tSTUCK_WHILE_OPENING 's'\tWAITING_TO_CLOSE='W'"
                        "CLOSING 'V'\tSTUCK_WHILE_CLOSING 'x'");
  });

  server.on("/open",HTTP_GET,[](AsyncWebServerRequest *request)
  {
    if (gate.state!=OPENING)
    {
      Serial.println("\n\nWeb request received to open gate");
      
      ram.state=TRYING_TO_MOVE;
      change_gate_state(OPENING);
      start_ram(RAM_PARAM_POSITION_AT_OPEN);
      strcpy(temp_buff,"OK");
    } else {
      Serial.println("Ignoring request to open gate that's already opening");
      strcpy(temp_buff,"Ignoring gate open request as it's already opening");
    }
    request->send(200,"text/plain",temp_buff);
    
  });

  server.on("/reboot24v",HTTP_GET,[](AsyncWebServerRequest *request) {
    send_message("about to interrupt twenty four volt system");
    digitalWrite(PIN_TWENTY_FOUR_VOLT_RELAY,TWENTY_FOUR_VOLT_RELAY_DISCONNECTED); // Cut relay
    Serial.println("Cutting 24v");
    twentyfourvoltrelay.restore_time=millis()+TWENTY_FOUR_VOLT_RELAY_INTERRUPT_TIME_MS;
    twentyfourvoltrelay.connected=false;
    request->send(200,"text/plain","Done");
    
  });

  server.on("/restart",HTTP_GET,[](AsyncWebServerRequest *request) {
    restart_esp("web request");
    request->send(200,"text/plain","Done");
    
  });

  server.on("/getfreeheap",HTTP_GET,[](AsyncWebServerRequest *request) {
    uint32_t free=ESP.getFreeHeap();
    sprintf(temp_buff,"Free heap=%d",free);
    request->send(200,"text/plain",temp_buff);
    
  });

  server.on("/move_to",HTTP_GET,[](AsyncWebServerRequest * request) {
    /* Parameter is move_to?position=1.2443 */
    Serial.println("move_to request received...");
    if (!request->hasParam("position"))
    {
      Serial.println("Missing position parameter from move_to request");
      request->send(400,"text/plain","move_to requires a position parameter");
      return;
    }

    uint16_t param_count=request->params();
    if (param_count!=1)
    {
      Serial.println("Too many arguments in request to move_to");
      request->send(400,"text/plain","too many arguments");
      return;
    }

    AsyncWebParameter* p = request->getParam("position");
    float new_position=atof(p->value().c_str());
    Serial.printf("New target position requested is %f\n",new_position);

    if (new_position<-0.6 || new_position>7.0)
    {
      sprintf(temp_buff,"Requested param (%f) is outside allowable range -0.6 -> 7.0",new_position);
      Serial.println(temp_buff);
      request->send(400,"text/plain",temp_buff);
      return;
    }



    Serial.printf("\n\nWeb request received to move gate to %f\n",new_position);
    
    ram.state=TRYING_TO_MOVE;
    //change_gate_state(OPENING);
    start_ram(new_position,100);
    strcpy(temp_buff,"OK");
    request->send(400,"text/plain",temp_buff);
    return;


  });

  server.on("/getpinstates",HTTP_GET,[](AsyncWebServerRequest *request) {
    char extra[80];
    //bool endtrip=end_trip_reached();
    bool beambroke=get_beam_broken();
    // sprintf(temp_buff,"Current state=%d\n"
    //                     "Endtrip=%s\n"
    //                     "Beam broken=%s\n"
    //                     "Outer button pressed=%s\n",
    //                     "Inner button pressed=%s\n",
    //                     "Nearside button pressed=%s\n",
    //                       gate.state,
    //                       endtrip?"At End":"Not at end",
    //                       beambroke?"Broken":"Not broken",
    //                       get_outer_button_pressed()?"Pressed":"Not pressed",
    //                       get_inner_button_pressed()?"Pressed":"Not pressed",
    //                       get_near_button_pressed()?"Pressed":"Not pressed"
    //                       );
    strcpy(temp_buff,"Current state\n=============\n");
    sprintf(extra,"Current state - %c\n",gate.state);
    strcat(temp_buff,extra);

    // sprintf(extra,"Endtrip - %s\n",endtrip?"At End":"Not at end");
    // strcat(temp_buff,extra);

    sprintf(extra,"Beam broken - %s\n",beambroke?"Broken":"Not broken");
    strcat(temp_buff,extra);

    sprintf(extra,"Outer button - %s\n",get_outer_button_pressed()?"Pressed":"Not pressed");
    strcat(temp_buff,extra);

    request->send(200,"text/plain",temp_buff);
    
  });


  AsyncElegantOTA.begin(&server,ota_username,ota_password);    // Start AsyncElegantOTA
  server.begin();
  Serial.println("HTTP server started");



  request.setDebug(false);
  
  request.onReadyStateChange(requestCB);



  next_heartbeat_time=millis()+HEARTBEAT_INTERVAL_MS;

  sprintf(temp_buff,"gate booted after %s",previous_restart_story);
  send_message(temp_buff);
  outer_button.state=QUIET_UNPRESSED;
  outer_button.pin_number=PIN_OUTSIDE_BUTTON;


  
  // Starting state
  position.last_good_reading=-100.0; // Set to a bad value, to force a reading to be taken even if inaccurate
  get_position();
  ram.target_position=position.last_good_reading;
  Serial.println("Target position set to current measurement");
  ram.error_state=false;
  ram.last_state_change_time=millis();
  ram.pwm_now=0;
  ram.reached_position=false;
  ram.start_time=millis();
  change_ram_state(STOPPED,"set initial state");
  set_up_motor();
  if (position.last_good_reading<2)
  {
    change_gate_state(WAITING_TO_CLOSE);
  } else {
    if (position.last_good_reading>4)
    {
      change_gate_state(CLOSED);
    } else {
      change_gate_state(STUCK_WHILE_CLOSING); // not really, but near enough for now!
    }
  }


  Serial.println("#######################\n\nSetup complete.... starting loop....\n\n");


}

void ram_control_test_loop()
{
  get_position();
  while(true)
  {
    float target_position;
    while(true)
    {
      target_position=random(1,14)/2.0;
      if (abs(target_position-position.last_good_reading)>1.2)
      {
        break;
      }
    }

    Serial.printf("\n>>>>>ABOUT TO TRAVEL TO >>>>>>>>> %f\n",target_position);
    delay(1500);
    start_ram(target_position);
    while(true)
    {
      get_position();
      update_ram();
      if (ram.error_state)
      {
        Serial.println("............. giving up to try another move.....!");
        break;
      }
      if (ram.state==STOPPED)
      {
        Serial.println("............... ARRIVED OK .........................\n\n\n");
        break;
      }
      delay(200);
    }

  }
}



void loop()
{


  

  // Main loop
  while (true)
  {
    time_since_gate_state_change=millis()-gate.state_change_time;
    
    get_position();// Update on the current position of the ram
    update_ram();

    // Current ram status
    Serial.printf("Ram state:%c,Ram target: %f, Position now: %f, Gate state: %c \n",ram.state,ram.target_position,position.last_good_reading,gate.state);

    // Check if movement has finished
    switch(gate.state)
    {
      case OPENING:
        // Check for finished opening
        if (ram.reached_position)
        {
          if (position.last_good_reading>1.5)
          {
            sprintf(temp_buff,"Unexpected problem,\nram.state is STOPPED RAM thinks it's finished opening but position is %f",position.last_good_reading);
            Serial.println(temp_buff);
            send_message(temp_buff);
          }
          change_gate_state(WAITING_TO_CLOSE);
          time_since_gate_state_change=millis()-gate.state_change_time;
        }
        break;
      case CLOSING:
        // check for finished closing
        if (ram.reached_position)
        {
          if (position.last_good_reading<4.5)
          {
            sprintf(temp_buff,"Unexpected problem, RAM thinks it's finished closing but position is %f",position.last_good_reading);
            Serial.println(temp_buff);
            send_message(temp_buff);
          }
          change_gate_state(CLOSED);
          time_since_gate_state_change=millis()-gate.state_change_time;
        }
        break;
    }
    
    if (get_beam_broken() && gate.state==CLOSING)
    {
      // Send open command
      Serial.println("B E A M    B R O K E N");
      send_message("Beam Broke during close- reopening");
      change_gate_state(OPENING);
      start_ram(RAM_PARAM_POSITION_AT_OPEN);
      
    }

    if (get_outer_button_pressed() && gate.state!=OPENING) // true only when button state is PRESSED_WAITING_FOR_USE
    {
        use_button_press(outer_button);// Mark that button press as used
        send_message("Outer gate button pressed");
        change_gate_state(OPENING);
        start_ram(RAM_PARAM_POSITION_AT_OPEN);
    }

    if (get_inner_button_pressed() && gate.state!=OPENING)
    {
      send_message("Opening for inner button");
      change_gate_state(OPENING);
      start_ram(RAM_PARAM_POSITION_AT_OPEN);
    }

    if (get_near_button_pressed() && gate.state!=OPENING) 
    {
      send_message("Opening for nearside button");
      change_gate_state(OPENING);
      start_ram(RAM_PARAM_POSITION_AT_OPEN);
    }

    // Timeout stuck state
    if (time_since_gate_state_change>DELAY_BEFORE_TRYING_TO_CLEAR_STUCK_MS)
    {
      if (gate.state==STUCK_WHILE_CLOSING || gate.state==STUCK_WHILE_OPENING)
      {
        Serial.println("!!!!!Waited to long enough for stuck gate to clear; now trying open...");
        send_message("Too long in stuck state - now opening");
        change_gate_state(OPENING);
        start_ram(RAM_PARAM_POSITION_AT_OPEN);
      }
    }

    // Close gate after delay
    if ((gate.state==WAITING_TO_CLOSE) && (time_since_gate_state_change>DELAY_BEFORE_CLOSING_GATE_MS))
    {
      Serial.printf("Closing as it's been open for %f seconds",time_since_gate_state_change/1000.0);
      change_gate_state(CLOSING);
      start_ram(RAM_PARAM_POSITION_AT_CLOSE);
    }







    if (millis()>next_heartbeat_time)
    {
      sprintf(temp_buff,"heartbeat+free+%d",ESP.getFreeHeap());
      send_message(temp_buff);
      next_heartbeat_time=millis()+HEARTBEAT_INTERVAL_MS;
    }

    // Check if relay should be reconnected
    if (!twentyfourvoltrelay.connected && millis()>twentyfourvoltrelay.restore_time)
    {
      digitalWrite(PIN_TWENTY_FOUR_VOLT_RELAY,TWENTY_FOUR_VOLT_RELAY_NORMAL);
      twentyfourvoltrelay.connected=true;
      Serial.println("Reconnected 24v");
      send_message("Reconnected 24v");

    }

    // Reset errors after a period of time
    if (ram.error_state && time_since_gate_state_change>DELAY_BEFORE_TRYING_TO_CLEAR_STUCK_MS)
    {
      send_message("Trying to re-open gate after error");
      change_gate_state(OPENING);
      start_ram(RAM_PARAM_POSITION_AT_OPEN);
    }

    // Every day it resets automonously, but may be at a bad time of day
    // However, the server sends a reset command every 4am, so it should sync to that
    if (millis()>1000*60*60*24*1)
    {
      restart_esp("daily reboot");
    }
    if (WiFi.status() != WL_CONNECTED) 
    {
      restart_esp("wifi lost");
    }

    


    // Checking current during loop delay
    delay(WAIT_LOOP_DELAY_MS);
  }


}