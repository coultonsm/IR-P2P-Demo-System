// Coulton Manning - Final project
// Code is to be used on Arduino MEGA platform. 
// (but should work on the UNO too if DHT is on a non-PWM pin).

#include "DHT.h" // Adafruit DHT library
#include <IRremote.hpp> // IRremote 4.5.0

// dht sensor information
#define DHT_11_PIN 22
#define DHTTYPE DHT11
DHT dht(DHT_11_PIN, DHTTYPE);

// other sensor pins
#define IR_RECV_PIN 11
#define IR_TRANS_PIN 6

// variables to store sensor information
int dhtHumidity=0;
int dhtTemperature=0;
uint32_t realValue;
int light_val;

// FSM variables
int state = 1;
#define LOCAL_SENSE 1
#define IR_COMM 2
#define IR_COMM2 3
#define SUMMARY 4

// IR Helper functions and protocol definitions
// These will be used across both of the arduino platforms.

// definitions of commands
#define HEAD_NODE_ADDR 0x01
#define MEGA_NODE_ADDR 0x02 // Thats me!
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

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  IrReceiver.begin(IR_RECV_PIN, ENABLE_LED_FEEDBACK);
  IrSender.begin(IR_TRANS_PIN, ENABLE_LED_FEEDBACK);

  dht.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  switch(state){
    case LOCAL_SENSE:
      Serial.println("LOCAL_SENSE");

      // read from our local sensor (DHT sensor)
      dhtTemperature = dht.readTemperature();
      dhtHumidity = dht.readHumidity();


      //DHT related code, adafruit gives NaN but conversion to int results in 0
      //so as long as both dont happen then we got a good reading
      if ( (dhtTemperature!=0) || (dhtHumidity!=0) ) {
        Serial.println("DHT reading success");
        Serial.print("Temp:"); Serial.print(dhtTemperature); Serial.println();
        Serial.print("Humidity:"); Serial.print(dhtHumidity); Serial.println();
      } else {
        Serial.println("Error reading from DHT");
      }

      Serial.println();
      state=IR_COMM;
      break;


    case IR_COMM:
      // STAGE 1 of protocol from slideshow

      Serial.println("IR_COMM");
      // Head node will reach out to us first
      // 5sec timeout
      Serial.println("Waiting for command (10secs)...");
      if ( waitForIrAddress(MEGA_NODE_ADDR, last_addr, last_cmd, 10000) ) {
        
        printRecvCmd(last_cmd);
        
        // expecting start of request
        if (last_cmd == CMD_REQUEST) {
          Serial.println("Got CMD_REQUEST.");

          // Respond with ACK
          Serial.println("\nReplying with CMD_ACK.");
          IrSender.sendNEC(HEAD_NODE_ADDR, CMD_ACK, 0);;

          Serial.println("Waiting for response (5secs)...");
          if ( waitForIrAddress(MEGA_NODE_ADDR, last_addr, last_cmd, 5000) ){
            printRecvCmd(last_cmd);

            // expecting temperature command
            if (last_cmd == CMD_TEMP) {
              Serial.println("Got request for temperature");

              Serial.println("\nSending temperature");
              IrSender.sendNEC(HEAD_NODE_ADDR, constrain(dhtTemperature, 0, 255), 0);
      
              Serial.println("Waiting for response. (5sec)");
              if (waitForIrAddress(MEGA_NODE_ADDR, last_addr, last_cmd, 5000)) {
                
                // expecting humidity command
                if (last_cmd == CMD_HUMID) {
                  Serial.println("Got request for humidity.");
                  
                  Serial.println("\nSending humidity");
                  IrSender.sendNEC(HEAD_NODE_ADDR, constrain(dhtHumidity, 0, 255), 0);

                  if (waitForIrAddress(MEGA_NODE_ADDR, last_addr, last_cmd, 5000)){
                    //expecting ACK FROM Uno R4
                    if (last_cmd == CMD_ACK) {
                      Serial.println("Received ACK"); 
                      Serial.println("Server-role complete.");
                    }

                    // the following else blocks provide feedback in the event
                    // a failure occours
                   
                  } else {badCommand(CMD_ACK, last_cmd);}
                } else {badCommand(CMD_HUMID, last_cmd);}
              } else {timeoutReached();}
            } else {badCommand(CMD_TEMP, last_cmd);}
          } else {timeoutReached();}
        } else {badCommand(CMD_REQUEST, last_cmd);}
      } else {
        timeoutReached();
        state = LOCAL_SENSE;
        break;
      }

      Serial.println();
      state = IR_COMM2;
      break;
  
    case IR_COMM2:
      // STAGE 2 of protocol from slideshow
    
      Serial.println("IR_COMM2");
      // Client role
      delay(200);

      // Send CMD_REQ
      IrSender.sendNEC(HEAD_NODE_ADDR, CMD_REQUEST, 0);

      if (waitForIrAddress(MEGA_NODE_ADDR, last_addr, last_cmd, 5000)) {
        // IR command is for us
        printRecvCmd(last_cmd);

        if (last_cmd == CMD_ACK) {
          // ACK received
          Serial.println("CMD_ACK recieved");

          Serial.println("Sending CMD_LIGHT");
          IrSender.sendNEC(HEAD_NODE_ADDR, CMD_LIGHT, 0);

          Serial.println("Waiting for response. (5secs)");
          if (waitForIrAddress(MEGA_NODE_ADDR, last_addr, last_cmd, 5000)) {
            // This command value is first 2 bits of the light sensor value
            printRecvCmd(last_cmd);
            light_val = last_cmd;
            
            Serial.println("Sending ACK");
            IrSender.sendNEC(HEAD_NODE_ADDR, CMD_ACK, 0);

            Serial.println("Waiting for response. (5secs)");
            if (waitForIrAddress(MEGA_NODE_ADDR, last_addr, last_cmd, 5000)) {
              printRecvCmd(last_cmd);
              // This command value is the lower 8 bits
              // reconstruct the full value using the higher 2 bits and lower 8 bits
              light_val = (light_val << 8) | last_cmd;
              Serial.print("Light sensor:"); printRecvCmd(last_cmd, PRT_DEC); Serial.println();

              Serial.println("Sending ACK.");

              IrSender.sendNEC(HEAD_NODE_ADDR, CMD_ACK, 0);
              Serial.println("Client role complete.");

              // the following else blocks provide feedback in the event
              // of a failure

            } else {timeoutReached();}
          } else {timeoutReached();}
        } else {badCommand(CMD_ACK, last_cmd);}
      } else {timeoutReached();}

      Serial.println();
      state = SUMMARY;
      break;

    case SUMMARY:
      // print a summary of all sensors

      Serial.println();
      Serial.println("------Summary------");
      Serial.println("Remote sensors:");
      Serial.print("Light sensor:"); Serial.print(light_val, DEC); Serial.println();

      Serial.println();
      Serial.println("Local sensors:");
      Serial.print("Temperature: "); Serial.print(dhtTemperature, DEC); Serial.println();
      Serial.print("Humidity: "); Serial.println(dhtHumidity, DEC); Serial.println();
      Serial.println("-------------------");

      Serial.println();
      state = LOCAL_SENSE;
      break;
  }

  // end of current fsm state
  delay(1000);
}
