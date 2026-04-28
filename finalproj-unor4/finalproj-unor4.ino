// Coulton Manning - Final project
// Code is to be used on the UNO R4 WIFI platform

//Library definitions
#include <IRremote.hpp> // IRremote 4.5.0
#include <WiFiS3.h>
#include "arduino_secrets.h"

//define our pins
#define IR_RECV_PIN 11
#define IR_TRANS_PIN 3
#define LIGHT_PIN 0

//sensor value storage
int light_val = -1;
int humFromMega;
int tempFromMega;
bool ir_success = false;

//FSM variables (not in any particular order)
int state = 1;
#define LOCAL_SENSE 1
#define IR_COMM 2
#define IR_COMM2 3
#define SUMMARY 4
#define GET_REQUEST 5
#define PARSE_REQUEST 6
#define SEND_RESPONSE 7
#define ACT 9
#define THINK 10


// IR Helper functions and protocol definitions
// These will be used across both of the arduino platforms.

// definitions of commands
#define HEAD_NODE_ADDR 0x01 // Thats me!
#define MEGA_NODE_ADDR 0x02
#define CMD_REQUEST 0x10
#define CMD_TEMP 0x20
#define CMD_HUMID 0x21
#define CMD_LIGHT 0x30
#define CMD_ACK 0x13

//variables to store commands
uint8_t last_addr;
uint8_t last_cmd;

// if a ir message is ready at execution, return true and update parameter variables
bool getIrMessage(uint8_t &addr, uint8_t &cmd) {
  if (!IrReceiver.decode()) return false;
  // if we continue here, there is a message to be read

  addr = IrReceiver.decodedIRData.address;
  cmd = IrReceiver.decodedIRData.command;

  IrReceiver.resume(); // Resume the ability to sense
  return true;
}

// Wait a certain number of milliseconds for an IR command directed at the specified address.
// Return true if we get a command before timeout. Return false if the time expires.
bool waitForIrAddress(uint8_t expectedAddr, uint8_t &addr ,uint8_t &cmd, unsigned long timeoutMs) {
  unsigned long start = millis();

  while (millis() - start < timeoutMs){
    if (IrReceiver.decode()) {
        addr = IrReceiver.decodedIRData.address;
        cmd  = IrReceiver.decodedIRData.command;

        // Allow the receiver to continue
        IrReceiver.resume();

        // Filter IR noise: ignore frames not intended for the expected address
        if (addr == expectedAddr) {
            return true;        // Valid frame for us
        }

        // otherwise ignore and continue waiting
    }
  }

  //If we reach here no message was found
  return false;
}

// Wait for an IR command but not directed to any specific address
bool waitForIr(uint8_t &addr, uint8_t &cmd, long timeoutMs) {
  unsigned long start = millis();

  while (millis() - start < timeoutMs){
    if (getIrMessage(addr,cmd)) return true;
    // Message is loaded and ready, sent back to parameter variables
  }

  //If we reach here no message was found
  return false;
}

// Serial print helper functions
void timeoutReached(){
  Serial.println("Timeout reached");
}

// print expected command vs the received command
void badCommand(uint8_t expected_cmd, uint8_t &cmd){
  Serial.print("Err: Unexpected command. Got ");
  Serial.print(cmd, HEX);
  Serial.print(" Expected ");
  Serial.print(expected_cmd);
  Serial.println();
}

#define PRT_HEX 0
#define PRT_DEC 1
// mode defines whether we print as a decimal or hexadecimal value (useful for values vs commands)
void printRecvCmd(uint8_t &cmd, int mode=PRT_HEX) {
  Serial.print("GOT-CMD");
  if (mode == PRT_HEX){
    Serial.print("-HEX: 0x");
    Serial.print(cmd, HEX);
  }

  if (mode == PRT_DEC) {
    Serial.print("-DEC: ");
    Serial.print(cmd, DEC);
  }
  
  Serial.print("\n");
}


// End of IR Helper Functions




///// Start of Wi-Fi related code
// Used from dataPushServerScheduled

#define DEBUG 1

/***
 * Client request types
 */
#define MSG_NONE 1
#define MSG_REGISTER 2

#define BAUD_RATE 9600
#define CONNECT_ATTEMPTS  5
#define CONNECT_RETRY_DELAY 5000

/***
 *  Token count for client request
 */
#define NUM_TOKENS 8


/***
 * event types for trigger
 */
#define NONE 0
#define RISING 1
#define FALLING 2
#define ANY 3

/***
 *  Measurement types for event trigger
 */
#define MEAS_TEMPERATURE 1
#define MEAS_HUMIDITY 2

// DEFINE NEW SENSOR HERE
#define MEAS_LIGHT 3
#define MEAS_IR 4

#define MSG_DELIMITERS " ,\n"

/***
 *  Time units in mS
 */
#define TICKS_PER_SEC 1000
#define TICKS_PER_MIN  TICKS_PER_SEC * 60
#define TICKS_PER_HOUR TICKS_PER_MIN * 60
#define TICKS_PER_DAY  TICKS_PER_HOUR * 24


#define MSG_BUFFLEN 1026

char ssid[] = SECRET_SSID;     // the name of your network
char passwd[] = SECRET_PASS;
int status = WL_IDLE_STATUS;     // the Wifi radio's status
int attempts= 0;
byte mac[6];                     // the MAC address of your Wifi shield
IPAddress ip;
char messageBuffer[MSG_BUFFLEN];

/***
 *  Registration information
 *
 *  Contact IP addresses
 *  Contact port numbers
 *  Contact measurement
 *  Contact event
 *  Contact start
 *  Contact delta
 *  Contact duration
 */
IPAddress contactIP;
int contactPort;
int contactMeasurement;
int contactEvent;
unsigned long contactStart;
unsigned long contactDelta;
unsigned long contactDuration;

float sensorVal= 0.0;

unsigned long eventStartTime= 0;
unsigned long eventStopTime= 0;
unsigned long lastFireTime= 0;

/***
 *  is registration active
 *  how many notifications were sent
 */
bool activeRegistration= false;
bool readyToFire= false;
int notificationCount= 0;

/***
 *  Time Management Information
 */
 unsigned long beginOfTime= 0;
 unsigned long currentTime= 0;
 unsigned long oldTime= 0;


int evt= 0;
String valStr= "";


WiFiServer server(80);            //server socket
WiFiClient connectionSocket = server.available();


WiFiClient notifier;     //for reaching out with notifications

/***
 *  Holds message line from client
 *  holds tokens parsed from message line
 */
String msgFromClient= "";
char *tokens[NUM_TOKENS];

bool clientRequest= false;

/*******************
 * Print status of WiFi connection to the
 * Serial terminal
 **/
void printWifiStatus() {
  // print the SSID of the network you're attached to:

  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  ip = WiFi.localIP();

  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();

  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
  Serial.print("The IP Address is:  ");
  Serial.println(ip);
}


/*******
 *  fetchLine
 *
 *  Retrieve a line of text from the client
 *  Given connection socket, retrief from the client a
 *  command.  By design, a command is terminated by a newline.
 *
 *  connectionSocket- active socket conneting you to the client
 */
void fetchLine(WiFiClient connectionSocket) {
    Serial.println("fetchLine");
    /***
     * As long as connection to client maintained
     * fetch line of text from client.
     */
    Serial.print("recieved from client:  ");

    bool lineDone= false;
    msgFromClient= "";
    
    while( connectionSocket.connected() && (!lineDone) ) {
      if (connectionSocket.available() && (!lineDone) ) {
         char c = connectionSocket.read();

         Serial.write(c);

         if (c != '\n') {
           msgFromClient += c;
         }  else {
           lineDone= true;
           break;
         }
      }
    }
}


/***
 *  A message is defined as
 *
 *  register IP_address,port,measurement,transition,start,delta,duration
 *  
 *   register:  keword register
 *   IP_address:  string IP_Address
 *   port:   string port number
 *   transition:  rising/falling
 *   duration:  integer
 */
void parseLine(String msg)  {
  int index= 0;
  char *ptr= NULL;

  if (DEBUG) {
    Serial.println("parseLine");
  }

  /*****
   * Note, the assumption is that the message is never longer than
   * MSG_BUFFLEN
   */
  msg.toCharArray(messageBuffer,MSG_BUFFLEN);

  /*****
   * Intantiate a string tokenizer to parse line of client request
   */

  ptr= strtok(messageBuffer,MSG_DELIMITERS);

  while(ptr != NULL)  {
     if (DEBUG) {
       Serial.print("token: ");
       Serial.println(ptr);
     }

     tokens[index]= ptr;
     index++;
     ptr= strtok(NULL, MSG_DELIMITERS);
  }
}


/***
 *  processMessage
 *
 *  Use parsed tokens to set registration information.
 *  Tokens are assumed populated by the parseLine routine, 
 *  that is an array of char * (strings) in order 
 * 
 *  1. registration command:  { register }
 *  2. contact/delivery IP:   validIP
 *  3. contact/delivery port:  integer
 *  4. measurement type:   { temperature, humidity }
 *  5. transition:  { rising,falling, any}
 *  6. start time:  double (millis)
 *  7. delta time:  double (millis)
 *  8. duration:  double (millis)
 */
int processMessage(char** toks) {
   int msgType= MSG_NONE;
   int index= 0;

   String command(toks[index]);    //registration commend
   index++;

   String ipStr(toks[index]);      //IP address string
   index++;

   String port(toks[index]);       //port number string
   index++;

   String measurement(toks[index]); //measurement type
   index++;

   String eventStr(toks[index]);    //rising or falling or any
   index++;

   String start(toks[index]);       //start time 
   index++;

   String delta(toks[index]);       //delta time
   index++;

   String durationStr(toks[index]); //duration



   /***
    * Implementation of client request semantics
    *
    * At the moment, it only implements event registrations
    */
   if (command.equals("register")) {
      if (DEBUG)  {
        Serial.print("processMessage: command= ");
        Serial.println(command);
      }

      if (!contactIP.fromString(ipStr.c_str())) {   //get the IP address
         return MSG_NONE;
      }    

      contactPort= port.toInt();                    //get the port number

      if (measurement.equals("temperature")) {      //set the measurement sensor
        contactMeasurement= MEAS_TEMPERATURE;
      } else if (measurement.equals("humidity")) {
        contactMeasurement= MEAS_HUMIDITY;
      } else if (measurement.equals("light")){
        contactMeasurement= MEAS_LIGHT;
      } else if (measurement.equals("infrared")){
        contactMeasurement= MEAS_IR;
      }
      
      if (eventStr.equals("rising")) {          //set the event transition type
        contactEvent= RISING;                  
      } else if (eventStr.equals("falling")) {  //or falling based on message
        contactEvent= FALLING;
      } else if (eventStr.equals("any")) {      //or every sensor measurement
        contactEvent= ANY;
      } else {
        contactEvent= NONE;
      }

      contactStart= (unsigned long) start.toDouble();
      contactDelta= (unsigned long) delta.toDouble();
      contactDuration= (unsigned long) durationStr.toDouble();

      eventStartTime = beginOfTime + contactStart;
      eventStopTime = beginOfTime + contactStart + contactDuration;

      msgType= MSG_REGISTER;
   }

   if (DEBUG)  {
     Serial.print("processMessage: msgType= ");
     Serial.println(msgType);
   }

   return msgType;
}

/***
 *  Triggering of randomized fake event
 */
int checkEvent() {
  int theEvent= 0;   
  float oldVal= sensorVal;
  
  if (contactMeasurement == MEAS_TEMPERATURE) {
     sensorVal= tempFromMega;
  } else if (contactMeasurement== MEAS_HUMIDITY) {
     sensorVal= humFromMega;
  } else if (contactMeasurement == MEAS_LIGHT) {
     sensorVal = light_val;
  } else if (contactMeasurement == MEAS_IR) {
     sensorVal = ir_success;
  }
 
  if (DEBUG) {
     Serial.print("checkEvent:  oldVal= ");
     Serial.println(oldVal);
     Serial.print("checkEvent:  sensorVal= ");
     Serial.println(sensorVal);
  }

  if (sensorVal > oldVal) {
    theEvent= RISING;
    if (DEBUG)
      Serial.println("checkEvent:  set event to RISING");

  } else if (sensorVal < oldVal) {
    theEvent= FALLING;

    if (DEBUG)
       Serial.println("checkEvent:  set event to FALLING");
  }  else if ((sensorVal == oldVal) || (contactEvent == ANY) ) {
    theEvent= ANY;
    if (DEBUG)
       Serial.println("checkEvent:  set event to ANY");
  } else {
    theEvent= NONE;
      if (DEBUG)
       Serial.println("checkEvent:  set event to NONE");
  }

  return theEvent;
}

void printStateVars() {
  if (DEBUG)  {
    Serial.print("clientRequest= ");
    Serial.println(clientRequest);

    Serial.print("activeRegistration= ");
    Serial.println(activeRegistration);

    Serial.print("readyToFire= ");
    Serial.println(readyToFire);
  }
}





/////// End of Wi-Fi related code





void setup() {
  // put your setup code here, to run once:
  // begin serial connection
  Serial.begin(115200);

  while (!Serial) {;}
  
  IrReceiver.begin(IR_RECV_PIN, ENABLE_LED_FEEDBACK);
  IrSender.begin(IR_TRANS_PIN, ENABLE_LED_FEEDBACK);

  // Wi-Fi Pubsub code

    /***
   * try to connect to network identified by SSID using
   * specified password
   */
  while( (status != WL_CONNECTED) && (attempts < CONNECT_ATTEMPTS) ) {
    Serial.print("Attempting to connect on WPA2 personal net, SSID: ");
    Serial.println(ssid);
    /*********
     * Join the WiFi network
     **/
    status= WiFi.begin(ssid,passwd);
    attempts++;
    delay(CONNECT_RETRY_DELAY);

    if (status == WL_CONNECTED) {
      Serial.println("Connected");
    } else {
      Serial.println("Couldn't get a wifi connection");
    }
  }

  server.begin();
  /******
   * Signal attached to WiFi network by printing status info
   **/
  printWifiStatus();
 /***
  *  mark the beginning of time
  */
  beginOfTime= millis();

}

void loop() {
  // put your main code here, to run repeatedly:  
  // BEGINNING OF FSM
  switch(state){
    case LOCAL_SENSE:      
      Serial.println("LOCAL_SENSE");
      //Read from our local sensor (Ambient Light)
      Serial.println("Reading light sensor.");
      light_val = analogRead(LIGHT_PIN);
      Serial.println(light_val, DEC);

      Serial.println();
      state = IR_COMM;
      break;

    case IR_COMM:
      // STAGE 1 of protocol from slideshow

      Serial.println("IR_COMM");
      ir_success=false;
      
      // Send the request command
      Serial.println("Sending CMD_REQUEST..");
      IrSender.sendNEC(MEGA_NODE_ADDR, CMD_REQUEST, 0);

      // Wait for a response, 5 second timeout
      Serial.println("Waiting for response. (5sec)");
      if (waitForIrAddress(HEAD_NODE_ADDR, last_addr, last_cmd, 5000)) {
        //If we reach here then a command was sent for us

        // print the command
        printRecvCmd(last_cmd);

        if(last_cmd == CMD_ACK) {
          Serial.println("\nGot ACK. Sending CMD_TEMP");

          // Request for temperature
          IrSender.sendNEC(MEGA_NODE_ADDR, CMD_TEMP, 0);

          Serial.println("Waiting for response. (5sec)");
          if (waitForIrAddress(HEAD_NODE_ADDR, last_addr, last_cmd, 5000)) {
            //If we reach here then the command is now the 2-byte temperature value.
            Serial.print("Temperature:"); printRecvCmd(last_cmd, PRT_DEC); Serial.println();
            tempFromMega = last_cmd;

            // Request the humidity value
            Serial.println("Sending CMD_HUMID..");
            IrSender.sendNEC(MEGA_NODE_ADDR, CMD_HUMID, 0);


            Serial.println("Waiting for response. (5sec)");
            if ( waitForIrAddress(HEAD_NODE_ADDR, last_addr, last_cmd, 5000) ) {
              //If we reach here then the command is now the 2-byte humidity value.
              Serial.print("Humidity:"); printRecvCmd(last_cmd, PRT_DEC); Serial.println();
              humFromMega = last_cmd;

              Serial.println("Sending CMD_ACK.");
              IrSender.sendNEC(MEGA_NODE_ADDR, CMD_ACK, 0);
              Serial.println("Client-role complete.");


              // The following else blocks print relevant info to serial
              // if any failures occour.
            } else {timeoutReached();}
          } else {timeoutReached();}
        } else {badCommand(CMD_ACK, last_cmd);}
      } else {
        // If we cant grab a connection skip IR_COMM2
        timeoutReached();
        state = SUMMARY;
        break;
      }

      Serial.println();
      state = IR_COMM2;
      break;

    case IR_COMM2:
      // STAGE 2 of protocol from slideshow

      Serial.println("IR_COMM2");
      // Server role

      Serial.println("Waiting for command (5secs)...");
      if (waitForIrAddress(HEAD_NODE_ADDR, last_addr, last_cmd, 5000)){
        // if we reach here, then a command was sent to us

        // print the command
        printRecvCmd(last_cmd);

        // Expecting start of request command
        if (last_cmd == CMD_REQUEST) {
          Serial.println("Got CMD_REQUEST.");
          Serial.println();

          // send acknowledgement
          Serial.println("Sending CMD_ACK..");
          IrSender.sendNEC(MEGA_NODE_ADDR, CMD_ACK, 0);

          Serial.println("Waiting for response. (5secs)");
          if (waitForIrAddress(HEAD_NODE_ADDR, last_addr, last_cmd, 5000)) {
            // Command is for us
            printRecvCmd(last_cmd);
            // Expecting light request
            if(last_cmd == CMD_LIGHT) {
              Serial.println("Got CMD_LIGHT.");
              // Send the light value
              Serial.println("Sending higher 2 bits of light_val");

              // Send first two bits of light_val (max is 1024)
              IrSender.sendNEC(MEGA_NODE_ADDR, (light_val >> 8) & 0x03, 0);

              Serial.println("Waiting for response. (5secs)");

              // Expecting acknowledgement
              if (waitForIrAddress(HEAD_NODE_ADDR, last_addr, last_cmd, 5000)) {
                printRecvCmd(last_cmd);
                if (last_cmd == CMD_ACK) {
                  Serial.println("Got CMD_ACK.");
                  
                  Serial.println("Sending lower 8 bits of light_val");
                  //send lower 8 bits of light value
                  IrSender.sendNEC(MEGA_NODE_ADDR, light_val & 0xFF, 0);

                  Serial.println("Waiting for response. (5secs)");
                  if (waitForIrAddress(HEAD_NODE_ADDR, last_addr, last_cmd, 5000)) {
                      // looking for ACK to finish communication
                      if (last_cmd == CMD_ACK) {
                        Serial.println("Got ACK.");
                        Serial.println("Server role complete");
                        ir_success=true;

                        // The following else blocks print out relevant information to serial
                        // for each step in the protocol if a failure occours.
                      } else {badCommand(CMD_ACK, last_cmd);}
                  } else {timeoutReached();}
                } else {badCommand(CMD_ACK, last_cmd);}
              } else {timeoutReached();}
            } else {badCommand(CMD_LIGHT, last_cmd);}
          } else {timeoutReached();}
        } else {badCommand(CMD_REQUEST, last_cmd);}
      } else {timeoutReached();}

      Serial.println();
      state=SUMMARY;
      break;
    case SUMMARY:
      // print a summary which includes all sensors and IR success.

      Serial.println();
      Serial.println("------Summary------");
      Serial.println("Remote sensors:");
      Serial.print("Temperature: "); Serial.print(tempFromMega, DEC); Serial.println();
      Serial.print("Humidity: "); Serial.println(humFromMega, DEC); Serial.println();

      Serial.println("Local sensors:");
      Serial.print("Light sensor:"); Serial.println(light_val, DEC); Serial.println();

      Serial.print("IR_SUCCESS: "); Serial.print(ir_success); Serial.println();

      Serial.println("-------------------");

      Serial.println();
      state=THINK;
      break;

    // FSM States from dataPushServerScheduled
    case THINK:
        if (DEBUG) {
          Serial.println();
          Serial.println("THINK");
        }

        printStateVars();

       /***
        * If there is a registration, go ahead
        * and trigger send of data to registered 
        * listener
        */
        evt= checkEvent();

        oldTime= currentTime;
        currentTime= millis();

        /*****
         * current time needed for repeated runs of
         * Python process implementing registration Client
         * Do not allow this message to be turned off via DEBUG
         **/
        Serial.print("currentTime= ");
        Serial.println(currentTime);


        if (DEBUG)  {
          Serial.print("eventStartTime= ");
          Serial.println(eventStartTime);
          Serial.print("eventStopTime= ");
          Serial.println(eventStopTime);
          Serial.print("lastFireTime + contactDelta= ");
          Serial.println(lastFireTime + contactDelta);
        }

        Serial.print("clientRequest= ");
        Serial.println(clientRequest);

        if (clientRequest)  {
          /***
           * Only send event if during active period
           */
          if ( (currentTime > eventStartTime) && (currentTime < eventStopTime) ) {    //if within active period
            /***
             * Only fire event once within each delta-t period
             * this is the same as checking if timer has exceeded
             * delta-t since the last event trigger
             */
            Serial.println("event fire within active period");
            activeRegistration= true;

            if (currentTime >= (lastFireTime + contactDelta)) {
              Serial.println("signaling event fire currentTime >= lastFireTime + contactDelta");
              readyToFire= true;
            } else {
              Serial.println("signaling hold event fire");
              readyToFire= false;
            } 
          } else if ( currentTime < eventStartTime ) { 
            /****
             * event not active but still valid
             **/
            activeRegistration= true;
            readyToFire= false;
          } else {  //(currentTime > eventStopTime) {
            clientRequest= false;
            activeRegistration= false;
            readyToFire= false;
            notificationCount= 0;
          } 
          
          state= ACT;
        } else {
          state= GET_REQUEST;
        }

        Serial.println();
        break;

    case ACT:
        if (DEBUG) {
          Serial.println();
          Serial.println("ACT");
        }

        printStateVars();

       /***
        * Depending on what is known about the event
        * and registration, compose the appropriate 
        * response
        */
        if (clientRequest && activeRegistration && readyToFire)  {
          String measStr= "";

          if (contactMeasurement== MEAS_TEMPERATURE) {
            measStr.concat("temperature ");
            sensorVal= tempFromMega;
          } else if (contactMeasurement== MEAS_HUMIDITY) {
            measStr.concat("humidity ");
            sensorVal= humFromMega;
          } else if (contactMeasurement == MEAS_LIGHT){
            measStr.concat("light ");
            sensorVal = light_val;
          } else if (contactMeasurement == MEAS_IR){
            measStr.concat("infrared ");
            sensorVal = ir_success;
          }

          valStr= "";

          if (DEBUG) {
            Serial.print("evt= ");
            Serial.println(evt);
            Serial.print("contactEvent= ");
            Serial.println(contactEvent);
          }

          /***
           * check the observed event against the registration
           * of interest
           */
          if ( (evt == contactEvent) || (contactEvent== ANY) )  {
            switch(evt)  {
              case RISING:
                if (DEBUG)  {
                  Serial.println("evt is RISING");
                }

                valStr.concat(measStr);
                valStr.concat("msg(");
                valStr.concat(notificationCount);
                valStr.concat("): ");

                valStr.concat("rising,");
                valStr.concat(sensorVal);
                break;

              case FALLING:
                if (DEBUG)  {
                  Serial.println("evt is FALLING");
                }

                valStr.concat(measStr);
                valStr.concat("msg(");
                valStr.concat(notificationCount);
                valStr.concat("): ");

                valStr.concat("falling,");
                valStr.concat(sensorVal);
                break;

              case ANY:
                if (DEBUG)  {
                  Serial.println("evt is ANY");
                }

                valStr.concat(measStr);
                valStr.concat("msg(");
                valStr.concat(notificationCount);
                valStr.concat("): ");

                valStr.concat("any,");
                valStr.concat(sensorVal);
                break;

              case NONE:
                if (DEBUG)  {
                  Serial.println("evt is NONE");
                }
                break;
            }

            state= SEND_RESPONSE;
          } else {
            state= LOCAL_SENSE;
          } 
        } else {
            state= LOCAL_SENSE;
        }

        Serial.println();
        break;
    
    case GET_REQUEST:
        if (DEBUG) {
          Serial.println();
          Serial.println("GET_REQUEST");
        }
        printStateVars();

        /*****
         * Note:  it is important to print out the IP
         *        address repetedly because it is difficult
         *        to see in the Serial terminal if only printed
         *        during setup.   IP address is needed to know
         *        where to send request from Python process's registration
         *        Client
         **/
        Serial.print("The IP Address is:  ");
        Serial.println(ip);

       /***
        *  Accept registration message from client
        *  If the registration is received then
        *  transition to parsing the request,
        *  otherwise go back to sensor query.
        *
        *  1.  Check if connectionSocket is valid.
        *  2.  Check if client on other end of 
        *      connection socket has sent data.
        *      This is done by checking if receive buffer
        *      has data (i.e. nonzero byte count).  
        *      Don't wait if the client has nothing to say
        *      at the moment.  The buffer will hang around
        *      and eventually fill until connectionSocket 
        *      is torn down or closed.
        *
        *  Note:  the registration must be parsed
        *         before sending response to client
        */
        connectionSocket= server.available();
        
    
        if (connectionSocket && (!clientRequest) ) {
          if (connectionSocket.available()) {
            fetchLine(connectionSocket);  //retrieve message from client  
            clientRequest= true;
            state= PARSE_REQUEST;
          } else {
             clientRequest= false;
             state= LOCAL_SENSE;
          }

        } else {
          state= LOCAL_SENSE;
        }

        Serial.println();
        break;

    case PARSE_REQUEST:
        if (DEBUG) {
          Serial.println();
          Serial.println("PARSE_REQUEST");
        }

        printStateVars();
       /***
        *  First check the incoming connection from
        *  the client to receive and process it's registration
        *  request
        *
        *  connectionSocket must still be valid because the
        *  response is sent after processing the message (i.e. semantics)
        *
        *  We don't check receive buffer underlying connectionSocket here
        *  because we already have a command and only need to parse it. 
        *  At this juncture we don't care about new commands from the client.
        *  We only care about sending a response to the message we currently
        *  staged.
        */
        if (connectionSocket) {
          parseLine(msgFromClient);                        //parse it  

          if (processMessage(tokens)== MSG_REGISTER) {    //set registration from tokens
            notificationCount= 0;
            activeRegistration= true;
            connectionSocket.println("registered");
          } else {
            connectionSocket.println("unknown");
          }

          /****
           * Close the connectionSocket.  We only allow for a single
           * registration request per connection.
           **/
          connectionSocket.stop();
        }

        Serial.println();
        state= LOCAL_SENSE;
        break;

    case SEND_RESPONSE:
        if (DEBUG) {
          Serial.println();
          Serial.println("SEND_RESPONSE");
          Serial.print("activeRegistration= ");
          Serial.println(activeRegistration);
        }

        printStateVars();

        if (activeRegistration && readyToFire && clientRequest)  {
          Serial.println("connecting to notificaiton server...");
          if (notifier.connect(contactIP, contactPort)) {
            Serial.println("connected");
            notifier.println(valStr.c_str());
            Serial.print("sent message:  ");
            Serial.println(valStr.c_str());
            notifier.stop();
            lastFireTime= millis();
            notificationCount++;
          } else {
            Serial.print("unable to connect to ");
            Serial.print(contactIP);
            Serial.print(":");
            Serial.println(contactPort);
            notifier.stop();
          }
        } 

        Serial.println();
        state= LOCAL_SENSE;
        break;

  }


  // END OF CURRENT FSM STATE
  delay(1000);
}
