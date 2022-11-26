/* 
 * Written By: Cory J. Fowler
 * Date: November 5, 2015
 * 
 * I sniffed a PLC trainer that I had access to in order to figure out the handshake needed to control an Allen Bradley CompactBlock LDX (1790D-T8BV8B)
 * The notes are what I was able to figure out about the DeviceNET protocol available from the Internet at the time.  It is most certainly object oriented.
 * 
 */


#include <mcp_can.h>
#include <SPI.h>

char msgString[80];

unsigned long rxId;
byte rxBuf[8];
byte len = 0;

byte DIO_FLAGS = 0x80;

byte rxFlag = 0xFF;
byte flag = 0x00;
byte i = 0;
byte x = 0;
byte y = 4;
#define TXEN 1  // Enable transmiting
// Duplicate MACID Sequence (Data is from School's PLC)
byte transBuf0[8] = {0x00,0x01,0x00,0x92,0x39,0x60,0x60,0xFF};  
// UCMM Open Request
byte transBuf1[8]   =  {0x02,0x4B,0x02,0x34,0xFF,0xFF,0xFF,0xFF};
// Send Unconnected Explicit Request
byte transBuf2[8]   =  {0x00,0x4B,0x03,0x01,0x01,0x00,0xFF,0xFF};  // Allocate with MACID 0x00 (Master)
// Send Explicit Requests
byte transBuf3[][8] = {{0x00,0x4B,0x03,0x01,0x02,0x00,0xFF,0xFF},  // Allocate with BAUD Rate to 0 (0: 125kbps, 1: 250kbps, 2: 500kbps, 3: Software) (Changing this breaks handshake?)
                       {0x00,0x0E,0x01,0x01,0x01,0xFF,0xFF,0xFF},  // Get Vendor ID (0x0001 should be returned)
                       {0x00,0x0E,0x01,0x01,0x02,0xFF,0xFF,0xFF},  // Get Device Type (0x0007 should be returned)
                       {0x00,0x0E,0x01,0x01,0x03,0xFF,0xFF,0xFF},  // Get Product Code (0x008D should be returned)
                       {0x00,0x0E,0x01,0x01,0x04,0xFF,0xFF,0xFF},  // Get Revision (0x0202 should be returned)
                       {0x00,0x10,0x05,0x01,0x0C,0x03,0xFF,0xFF},  // Set Watchdog/Timeout Action to 0x03? (Changing seems to have no effect)
                       {0x00,0x10,0x05,0x02,0x09,0x00,0x01,0xFF},  // Set Expected Packet Rate to 0x004B (0x4B00 is about 1 minute 17 seconds.) (0x0100 = 1 second)  (value/256 equals seconds.)
                       {0x00,0x0E,0x05,0x02,0x07,0xFF,0xFF,0xFF},  // Get Produced Data Size (One Byte)
                       {0x00,0x0E,0x05,0x02,0x08,0xFF,0xFF,0xFF}}; // Get Consumed Data Size (One Byte)
byte transBuf3index[] = {6,5,5,5,5,6,7,5,5};
// Send Poll Request
byte transBuf4[8] = {0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

byte cls[] = {0x1B,0x5B,0x32,0x4A,0x1B,0x5B,0x3B,0x48,0x0A,0x0D};  // Terminal control commands to erase screen and return home: <Esc>[2J<Esc>[;H

// Timing variables: CAN transmit interval
unsigned long previousMillis[] = {0,0,0,0,0};
unsigned long interval[] = {1000,100,25,1000,5000};                // Change the first value to increase or decrease PGN request rate.

byte pgn = 0;

MCP_CAN CAN(10);

/******************************************************************************
 * 
 * Initialization
 * 
 ******************************************************************************/
void setup(){
  Serial.begin(115200);
  Serial.println("Working!");
  
  pinMode(2, INPUT);

  if(CAN.begin(MCP_STDEXT, CAN_250KBPS, MCP_20MHZ) == CAN_OK) Serial.println("CAN Initialized Successfully!");  // MCP_STDEXT or MCP_ANY - MCP_STD and MCP_EXT do not function correctly, bug discovered in die.
  else Serial.println("Error Initializing CAN...");

  CAN.init_Mask(0,0x00000000);                     // Init first mask...    //Filter out extended IDs
  CAN.init_Filt(0,0x00000000);                     // Init first filter...
  CAN.init_Filt(1,0x00000000);                     // Init second filter...
  
  CAN.init_Mask(1,0x00000000);                     // Init second mask...   //Filter out extended IDs
  CAN.init_Filt(2,0x00000000);                     // Init third filter...
  CAN.init_Filt(3,0x00000000);                     // Init fouth filter...
  CAN.init_Filt(4,0x00000000);                     // Init fifth filter...
  CAN.init_Filt(5,0x00000000);                     // Init sixth filter...
  
  CAN.setMode(MCP_NORMAL);
}

/******************************************************************************
 * 
 * Main Loop
 * 
 ******************************************************************************/
void loop(){
  
/******************************************************************************
 * Read CAN then Use or Discard Received Data
 ******************************************************************************/
  if(!digitalRead(2)){                                                         // CAN Interrupt
    receiveCAN_task();
  } // if CAN interrupt  set

  if(serialEvent() == 'a'){
    digitalIO_task(0x02);
  }
  
/******************************************************************************
 * CAN Output
 ******************************************************************************/
#if TXEN
  switch(flag){
    case 0x00:
      if((millis() - previousMillis[0]) >= interval[0]) {        // Transmit Duplicate MAC Check Messages
        previousMillis[0] = millis();
        CAN.sendMsgBuf(0x407, 7, transBuf0);
        i++;
        if(i==4){
          flag++;
          i = 0;
        }
      }
      break;
      
    case 0x01:
      if((millis() - previousMillis[0]) >= interval[0]) {        // Transmit Unknown Group 3 messages (UCMM Open Message perhaps?)
        previousMillis[0] = millis();
        CAN.sendMsgBuf(0x780, 4, transBuf1);
        i++;
        if(i==2){
          flag++;
          i = 0;
        }
      }
      break;
    
    case 0x02:
      if((millis() - previousMillis[0]) >= interval[1]) {        // Transmit Unconnected Explicit Request
        previousMillis[0] = millis();
        CAN.sendMsgBuf(0x416, 6, transBuf2);
        i++;
        if(i==1){
          flag++;
          i = 0;
        }
      }
      break;
    
    case 0x03:
      if(!rxFlag) {                                              // Transmit Explicit Requests to slave
        previousMillis[0] = millis();
        CAN.sendMsgBuf(0x414, transBuf3index[i], transBuf3[i]);
        i++;
        rxFlag = 0xFF;
        if(i==9){
          flag++;
          i = 0;
        }
      }
      break;
  
    case 0x04:
      if((millis() - previousMillis[0]) >= interval[2]) {        // Transmit Poll requests to slave
        previousMillis[0] = millis();
        if(i==5){
          i = 0;
          x ^= 0x01;
        }
        transBuf4[0] = (transBuf4[0] & 0x03) | y;
        CAN.sendMsgBuf(0x415, 1, transBuf4);
        i++;
        if(x == 0x00) y = y << 1;
        else y = y >> 1;
      }
      break;
      
    default:
      Serial.println("There is a problem if this is being printed");
  }


#endif  //TXEN

  if(((millis() - previousMillis[3]) >= interval[3]) && ((DIO_FLAGS & 0x01) == 0x00)) {        // Flash button LED
    previousMillis[3] = millis();
    digitalIO_task(0x30);
  }

  if(((millis() - previousMillis[4]) >= interval[4]) && ((DIO_FLAGS & 0x01) == 0x01)) {        // Output 2 off delay
    digitalIO_task(0x20);
  }

        
}

/*********************************************************************************************************
  Extra Stuff
*********************************************************************************************************/
/*
int freeRAM () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}
*/

char serialEvent(void) {
  char inChar = '\n';
  while (Serial.available()) {
    // get the new byte:
    inChar = (char)Serial.read();
    // add it to the inputString:
//    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
//    if (inChar == '\n') {
//      stringComplete = true;
//    }
  }
  return inChar;
}

void receiveCAN_task(void){
  CAN.readMsgBuf(&rxId, &len, rxBuf);                                        // Get CAN Message
//  sprintf(msgString,"Millis: %.10u ", millis());
//  Serial.print(msgString);
  if(rxId < 0x7F0) {
    if(rxId == 0x3C2) {
      digitalIO_task(0x01);
      return;
    }
    else if((rxId & 0x400) == 0x000) {
      // Group 1 Message Recieved
      Serial.print("Group 1 Message: ");
      char macID = 0x03F & rxId;
      char msgID = (rxId >> 6) & 0x00F;
      sprintf(msgString, "MAC: %.2u Message: %.2u Data:", macID, msgID);
      Serial.print(msgString);
    }
    else if((rxId & 0x600) == 0x400) {
      // Group 2 Message Recieved
      Serial.print("Group 2 Message: ");
      char msgID = 0x007 & rxId;
      char macID = (rxId >> 3) & 0x03F;
      sprintf(msgString, "MAC: %.2u Message: %.2u Data:", macID, msgID);
      Serial.print(msgString);
      if((msgID == 3) && (macID == 2)) rxFlag = 0x00;
    }
    else if((rxId & 0x7C0) == 0x7C0) {
      // Group 4 Message Recieved
      Serial.print("Group 4 Message: ");
      char msgID = rxId & 0x03F;
      sprintf(msgString, "        Message: %.2u Data:", msgID);
      Serial.print(msgString);
    }
    else if(((rxId & 0x600) == 0x600)) {                                          // && ((rxId & 0x1C0) != 0x1C0)
      // Group 3 Message Recieved
      Serial.print("Group 3 Message: ");
      char macID = 0x03F & rxId;
      char msgID = (rxId >> 6) & 0x007;
      sprintf(msgString, "MAC: %.2u Message: %.2u Data:", macID, msgID);
      Serial.print(msgString);
    }
//    else if((rxId & 0x7F0) == 0x7F0) {
//      // Invalid ID on DeviceNet.
//    }
  }
  else{
    
    rxId &= 0x1FFFFFFF; // Remove extended ID (and remote request) identifier bits used by CAN Library.
  
    // Generic CAN serial output
    sprintf(msgString, " ID: 0x%.8lX  Data:", rxId);
    Serial.print(msgString);
    
  } // else 

  for(byte i = 0; i<len; i++){
    sprintf(msgString, " 0x%.2X", rxBuf[i]);
    Serial.print(msgString);
  } // for
    
  Serial.println();
  
}

void digitalIO_task(byte task){
  switch(task){
    case 0x01:      // We received the response to our poll request
      Serial.println(DIO_FLAGS, BIN);
      if(((rxBuf[0] & 0x01) == 0x00) & ((DIO_FLAGS & 0x02) == 0x02)){  // Button was released
        transBuf4[0] |= 0x03;                                          // Turn on Output 0 and 1
        previousMillis[4] = millis();                                  // Prepare output timer
        DIO_FLAGS &= 0xFD;
      }
      else if((rxBuf[0] & 0x01) == 0x01){                              // Button is pressed
        transBuf4[0] |= 0x01;                                          // Turn on Output 1 (Button LED)
        previousMillis[4] = millis();                                  // Prepare output timer
        if((DIO_FLAGS & 0x03) == 0x00){
          DIO_FLAGS |= 0x03;                                           // Set latch flag
        }
      }
      break;

    case 0x02:      // We received a force request...
      Serial.println(DIO_FLAGS, BIN);
      transBuf4[0] |= 0x03;                                            // Turn on Output 1 (Button LED)
      previousMillis[4] = millis();                                    // Prepare output timer
      if((DIO_FLAGS & 0x03) == 0x00){
        DIO_FLAGS |= 0x01;                                             // Set latch flag
      }
      break;
      
    case 0x20:      // Output 2 scheduler actions
      transBuf4[0] &= 0xFC;
      previousMillis[3] = millis();                       // Reset button LED toggle timer                             
      DIO_FLAGS &= 0xFE;
      break;
      
    case 0x30:      // Toggle the button LED
      transBuf4[0] ^= 0x01;
      break;
      
    default:
      break;
  }
}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
