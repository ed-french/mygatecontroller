

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
      MOTOR_CURRENT_LIMIT
      IGNORE_ENDSTOP_DURATION
      WAIT_LOOP_DELAY_MS
      DELAY_BEFORE_TRYING_TO_CLEAR_STUCK
      DELAY_BEFORE_CLOSING_GATE_MS
      FULL_PWM_RATE
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



#include "credentials.h" // Just includes ssid and password declarations and ota_username and ota_password

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

#define MOTOR_CURRENT_LIMIT 3.000

#define IGNORE_ENDSTOP_DURATION_MS 1500

#define WAIT_LOOP_DELAY_MS 100

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

#define DELAY_BEFORE_CLOSING_GATE_MS 80000

#define FULL_PWM_RATE 200
#define PWM_FREQUENCY 10000
#define HEARTBEAT_INTERVAL_MS 600000 // 5 minutes=300000


// Async http lib defines...
  // Level from 0-4
  #define ASYNC_HTTP_DEBUG_PORT     Serial
  #define _ASYNC_HTTP_LOGLEVEL_     1

  // 300s = 5 minutes to not flooding
  #define HTTP_REQUEST_INTERVAL     1  //300

char hostname[]="nunshallpass";


struct RelayState
{
  bool connected;
  uint32_t restore_time;
};

RelayState twentyfourvoltrelay;


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

void update_button(ButtonState button)
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

void use_button_press(ButtonState button)
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



void restart_esp()
{
  send_message("Rebooting in 10 seconds");
  delay(10000);
  ESP.restart();
}


void change_gate_state(GateStates new_state)
{
  Serial.printf("OOOO Changing State from : %c -> %c\n",(char)gate.state,(char)new_state);
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
/*
      success is a bool that indicates if we think we reached the end of travel
      either through and endstop or high current measured

*/
{
  set_up_motor();
  if (success)
  {
    switch (gate.state)
    {
      case OPENING:
        change_gate_state(WAITING_TO_CLOSE);
        send_message("Gate now open");
        break;
      case CLOSING:
        change_gate_state(CLOSED);
        send_message("Gate now closed");
        break;
      default:
        sprintf(temp_buff,"Stop motor called when gate was neither opening nor closing -state %d",gate.state);
        send_message(temp_buff);
    }
  } else {
    send_message("Motor stopped with error condition");
    switch (gate.state)
    {
      case OPENING:
        change_gate_state(STUCK_WHILE_OPENING);
        send_message("Gate stuck during opening");
        break;

      case CLOSING:
        change_gate_state(STUCK_WHILE_CLOSING);
        send_message("Gate stuck during closing");
        break;

      default:
        sprintf(temp_buff,"Stopmotor called when gatewas neither opening nor closing - state=%d",gate.state);
        send_message(temp_buff);
    } 
  }
}

void start_opening_gate(const char * reason,uint32_t time_to_ignore_endstops=IGNORE_ENDSTOP_DURATION_MS)
{
  Serial.printf("Opening gate due to: %s\n",reason);
  sprintf(temp_buff,"Gate opening due to %s",reason);
  send_message(temp_buff);
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

void run_motor()
{
  /* Non-blocking
    Checks state of motor:
        arrived at endstop?
        driver overloaded?
        current too-high?
        timed out?
      Uses stop_motor to record the new state - passing true if we think we finished!
  */

  // Just return straight away if the motor isn't meant to be running at the moment!
  if (gate.state!=OPENING && gate.state!=CLOSING)
  {
    return; // Like reached the end already! 
  }

  // Check too much current
  float motor_current=get_motor_current(); 
  Serial.printf("\tFound motor current: %f",motor_current);
  if (motor_current>MOTOR_CURRENT_LIMIT)
  {

    stop_motor(true); // assume we reached the end
    Serial.printf("Motor taking too much current, stopping with fault!");
    sprintf(temp_buff,"Motor current over limit- %f",motor_current);
    send_message(temp_buff);
    return;
  }

  // Check driver overload condition
  if (!motor_drivers_ok())
  {
    
    stop_motor(false);
    Serial.println("Motor driver overloaded, so stopping");
    strcpy(temp_buff,"Motor drivers were in error state");
    send_message(temp_buff);
    return;
  }

  // Check endstop reached
  if (end_trip_reached() && millis()>(gate.state_change_time+gate.time_to_ignore_endstops_ms))
  {
    Serial.println("Reached endstop, stopping motor!");
    stop_motor(true);
    strcpy(temp_buff,"endstop was reached");
    send_message(temp_buff);


    return;
  }

  // Check for timeout
  if (millis()>gate.timeout_time)
  {
    Serial.println("Stopping motor due to timeout");
    stop_motor(true);
    send_message("motor stopped after timeout");
    return;
  }


  return; // still moving!
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

  twentyfourvoltrelay.connected=true;

  
  WiFi.setHostname(hostname);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  delay(1000);
  WiFi.begin(ssid, password);
  delay(1000);
  Serial.printf("Connecting to: %s\n",ssid);
  Serial.println("");

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
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
                        "\tUse /getpinstates to see what state and raw inputs are now"
                        "CLOSED'C'\tOPENING 'O'\tSTUCK_WHILE_OPENING 's'\tWAITING_TO_CLOSE='W'"
                        "CLOSING 'V'\tSTUCK_WHILE_CLOSING 'x'");
  });

  server.on("/open",HTTP_GET,[](AsyncWebServerRequest *request)
  {
    if (gate.state!=OPENING)
    {
      Serial.println("Web request received to open gate");
      strcpy(temp_buff,"Gate is now opening!");
      start_opening_gate("request from web");
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
    restart_esp();
    request->send(200,"text/plain","Done");
    
  });

  server.on("/getfreeheap",HTTP_GET,[](AsyncWebServerRequest *request) {
    uint32_t free=ESP.getFreeHeap();
    sprintf(temp_buff,"Free heap=%d",free);
    request->send(200,"text/plain",temp_buff);
    
  });

  server.on("/getpinstates",HTTP_GET,[](AsyncWebServerRequest *request) {
    char extra[80];
    bool endtrip=end_trip_reached();
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

    sprintf(extra,"Endtrip - %s\n",endtrip?"At End":"Not at end");
    strcat(temp_buff,extra);

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
  send_message("gate booted");
  outer_button.state=QUIET_UNPRESSED;
  outer_button.pin_number=PIN_OUTSIDE_BUTTON;
}

void loop()
{

  // Main loop

  uint32_t time_since_state_change=millis()-gate.state_change_time;

  run_motor();
  if (get_beam_broken() && gate.state==CLOSING)
  {
    // Send open command
    send_message("Beam Broke during close- reopening");
    start_opening_gate("beam broken",500);
  }
  if (get_outer_button_pressed() && gate.state!=OPENING)
  {
      use_button_press(outer_button);// Mark that button press as used
      send_message("Outer gate button pressed");
      start_opening_gate("outer button pressed");




  }
  if (get_inner_button_pressed() && gate.state!=OPENING)
  {
    send_message("Opening for inner button");
    start_opening_gate("inner button pressed");
  }
  if (get_near_button_pressed() && gate.state!=OPENING)
  {
    send_message("Opening for nearside button");
    start_opening_gate("nearside button pressed");
  }

  // Timeout stuck state
  if (time_since_state_change>DELAY_BEFORE_TRYING_TO_CLEAR_STUCK_MS)
  {
    if (gate.state==STUCK_WHILE_CLOSING || gate.state==STUCK_WHILE_OPENING)
    {
      Serial.println("!!!!!Waited to long enough for stuck gate to clear; now trying open...");
      send_message("Too long in stuck state - now opening");
      start_opening_gate("retry after getting stuck"); // Try to open the gate to clear the error
    }
  }
  // Close gate after delay
  if ((time_since_state_change>DELAY_BEFORE_CLOSING_GATE_MS) && (gate.state==WAITING_TO_CLOSE))
  {
    start_closing_gate();
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

  // Every day it resets automonously, but may be at a bad time of day
  // However, the server sends a reset command every 4am, so it should sync to that
  if (millis()>1000*60*60*24*1) 
  {
    restart_esp();
  }
  delay(WAIT_LOOP_DELAY_MS);


}