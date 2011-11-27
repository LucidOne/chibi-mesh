/*
 * Copyright (c) 2011, I Heart Engineering
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of I Heart Engineering nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

//  ================================================================
//  Robot Mesh Node
//  ================================================================

//  ----------------------------------------------------------------
//  Includes
//  ----------------------------------------------------------------
#include <chibi.h>

//  ----------------------------------------------------------------
//  Definitions
//  ----------------------------------------------------------------
#define COMMANDER  0                        // Command Line interface
#define HELO_INTERVAL 5000                  // Milliseconds between HELO broadcasts
#define SREQ_INTERVAL 500
#define QUEUE_INTERVAL 1000                  // Time between resending commands
#define MAX_NODES 10
#define DUMP_FULL 0
#define DUMP_FILTER 1
#define SEQ_WINDOW 2                        // Sequence window
#define DEBUG 0                             // Enable Serial Debug Output

#define QUEUE_IDLE 0                        // Command queue empty
#define QUEUE_SEND 1                        // Sending queued command
#define QUEUE_WAIT 2                        // Waiting for command to be ACKed

#define ROBO_INIT 0x00
#define ROBO_STOP 0x01
#define ROBO_FWD 0x02
#define ROBO_LEFT 0x03
#define ROBO_RIGHT 0x04
#define ROBO_ACK 0xFF
//  ----------------------------------------------------------------
//  Global Variables
//  ----------------------------------------------------------------
unsigned int my_addr;                       // Short Address for this node
unsigned int my_seq = 1;                    // Sequence number for this node
unsigned long helo_time;                    // Time of last HELO message
unsigned long sreq_time;                    // Time of last SREQ message
int sreq_index = 1;                         // 0xF000 + sreq_index = node to SREQ
unsigned long beep_timer;                   // Beep Timer

byte status_count = 0;                      // Number of neighbor nodes found
unsigned int status_id[MAX_NODES];          // Short address of neighbor node
unsigned int status_seq[MAX_NODES];         // Last known sequence number
byte status_rssi[MAX_NODES];                // RSSI of latest message
unsigned long status_time[MAX_NODES];       // Time of last update

unsigned int net_id[MAX_NODES][MAX_NODES];  // ID of Neighbors
unsigned int net_index[MAX_NODES];          // Index
int net_rssi[MAX_NODES][MAX_NODES];         // RSSI
int net_nodes[MAX_NODES];                   // Number of Neighbors
int net_count = 0;

//  ----------------------------------------------------------------
//  setup()  -  Initialize microcontroller
//  ----------------------------------------------------------------
void setup()
{
  Serial.begin(57600);                      // Init serial port to 57600 baud
  pinMode(9, OUTPUT);                       // Configure Pin 9 as an output
  digitalWrite(9, HIGH);                    // Set Pin 9 HIGH to turn off buzzer
  helo_time = millis();                     // Init HELO timer
  my_addr = chibiGetShortAddr();            // Get node address from EEPROM
  chibiInit();                              // Init radio
  debugBeep(50);
  debugBeep(50);
  delay(100);
  debugBeep(100);  
}

//  ----------------------------------------------------------------
//  loop()  -  Main program loop
//  ----------------------------------------------------------------
void loop()
{ 
  if (chibiDataRcvd() == true)              // Process received messages
  {
    byte *p;                                // Byte pointer
    int len;                                // Packet length
    byte rssi;                              // Receive Signal Strength Indicator
    unsigned int org_addr;                  // Origin Address
    unsigned int src_addr;                  // Source Address
    unsigned int dst_addr;                  // Destination Address
    unsigned int cmd_seq;                   // Command Sequence #
    byte ttl;
    byte buf[100];                          // Receive buffer

    len = chibiGetData(buf);                // get packet length
    rssi = chibiGetRSSI();                  // get RSSI
    src_addr = chibiGetSrcAddr();           // get source address
    
    statusNodeUpdate(src_addr,rssi,millis());  // update status 
    if (rssi > 0x10) {                      // Only respond to nearby nodes
      switch (buf[0])                       // Check command byte
      {
        case 0x00:                          // DO NOT ECHO HELO messages
          break;
        case 0x01:                          // SREQ - Status Request
          // WARNING: NETWORK ORDER IS BIG ENDIAN!!
          ttl = buf[1];                     // Unpack TTL from buffer
          p= (byte*)&cmd_seq;               // Set byte pointer
          p[0] = buf[3]; p[1] = buf[2];     // Unpack 2 bytes from buf into cmd_seq
          p= (byte*)&org_addr;              // Set byte pointer
          p[0] = buf[5]; p[1] = buf[4];     // Unpack 2 bytes from buf into org_addr
          p= (byte*)&dst_addr;              // Set byte pointer
          p[0] = buf[7]; p[1] = buf[6];     // Unpack 2 bytes from buf into dst_addr
          
          if (DEBUG) {
            Serial.print("PING Received: ");
            Serial.print(src_addr, HEX);
            Serial.print(" TTL:");
            Serial.print(ttl, HEX);
            Serial.print(" SEQ:");
            Serial.print(cmd_seq, HEX);
            Serial.print(" ORG:");
            Serial.print(org_addr, HEX);
            Serial.print(" DST:");
            Serial.print(dst_addr, HEX);
            Serial.println("");
          }
          
          if (dst_addr == my_addr) {        // If this node is the destination
            if (statusSeqUpdate(org_addr,cmd_seq)) {  // If the sequence number is valid
              for (int i=0; i < 256-ttl; i++) {          // For each hop
                debugBeep(50);                           // Beep for 25 ms
              }
              msgStatusResponse(org_addr);         // Send Response to originating node
            }
            break;                          // Do not echo packets addressed to this node
          }
        case 0x07:                          // ROBO - Robot Control Message
          // WARNING: NETWORK ORDER IS BIG ENDIAN!!
          ttl = buf[1];                     // Unpack TTL from buffer
          p= (byte*)&cmd_seq;               // Set byte pointer
          p[0] = buf[3]; p[1] = buf[2];     // Unpack 2 bytes from buf into cmd_seq
          p= (byte*)&org_addr;              // Set byte pointer
          p[0] = buf[5]; p[1] = buf[4];     // Unpack 2 bytes from buf into org_addr
          p= (byte*)&dst_addr;              // Set byte pointer
          p[0] = buf[7]; p[1] = buf[6];     // Unpack 2 bytes from buf into dst_addr

          if (dst_addr == chibiGetShortAddr()) {
            if (statusSeqUpdate(org_addr,cmd_seq)) {
              if (org_addr != src_addr) {
                debugBeep(50);              // Beep for 50ms
              }
              roboCmd(buf[8]);              // Command robot to perform requested action
            }
            msgRobotControl(org_addr,ROBO_ACK);
            break;                          // Do not echo packets addressed to this node
          }
        default:                            // Check for rebroadcasting
          if (dst_addr != my_addr) {        // If this node is not the destination
            if (statusSeqUpdate(org_addr,cmd_seq)) {  // If sequence number is valid then update
              buf[1]--;                                 // Decrement TTL
              chibiTx(BROADCAST_ADDR, buf,len);                 // Rebroadcast
            }
          }
          break;
      }
    }
  }

  if (millis() - helo_time > HELO_INTERVAL) {    // If time since last HELo is greater than interval
    byte msg[1]= { 0x00 };                       // HELO 0x00 Command Byte
    chibiTx(BROADCAST_ADDR, msg, 1);             // Broadcast HELO
    helo_time = millis();                        // Update HELO timer
    /*
    byte foo[5];
    foo[0] = 0x09;
    Serial.print("HELO TIME: ");
    Serial.print(helo_time, HEX);
    Serial.println("");
    byte *p;
    p= (byte*)&helo_time;                        // Set byte pointer
    foo[1] = p[3]; foo[2] = p[2];                // Pack 2 bytes from sequence into buffer
    foo[3] = p[1]; foo[4] = p[0];                // Pack 2 bytes from sequence into buffer
    chibiTx(BROADCAST_ADDR, foo, 5);             // Broadcast HELO
    */
  }
}


//  ================================================================
//  Message Functions
//  ================================================================

//  ----------------------------------------------------------------
//  msgStatusResponse()  -  Send a status response message
//    Description     msgStatusResponse sends a status response (SRES)
//                    to the destination dst_addr
//  ----------------------------------------------------------------
void msgStatusResponse(unsigned int dst_addr)
{
  byte data[100];
  byte *p;
  unsigned long now = millis();
  
  debugDumpState();

  data[0] = 0x02;                           // SREQ 0x01 Status Request Command
  data[1] = 0xFF;                           // Set initial TTL to 255

  // WARNING: NETWORK ORDER IS BIG ENDIAN!!
  p= (byte*)&my_seq;                        // Set byte pointer
  data[2] = p[1]; data[3] = p[0];           // Pack 2 bytes from sequence into buffer
  p= (byte*)&my_addr;                       // Set byte pointer
  data[4] = p[1]; data[5] = p[0];           // Pack 2 bytes from source address into buffer
  p= (byte*)&dst_addr;                      // Set byte pointer
  data[6] = p[1]; data[7] = p[0];           // Pack 2 bytes from desitination address buffer
  data[8] = status_count;                   // Node count
  data[9] = 0x00;                           // Reserved byte 1
  data[10] = 0x00;                          // Reserved byte 2
  data[11] = 0x00;                          // Reserved byte 3

  my_seq++;

  for (int i=0; i < status_count; i++) {
    p= (byte*)&status_id[i];
    data[12+(3*i)] = p[1];
    data[12+(3*i)+1] = p[0];
    data[12+(3*i)+2] = status_rssi[i];
  }

  chibiTx(BROADCAST_ADDR, data,12 + (status_count*3));
}

//  ----------------------------------------------------------------
//  msgRobotControl()  -  Send a robot control message
//    Description     msgRobotControl sends robot control message to command the robot
//                    to perform the required action
//  ----------------------------------------------------------------
void msgRobotControl(unsigned int dst_addr,byte action)
{
  byte data[12];
  byte *p;
  
  data[0] = 0x07;                           // SREQ 0x01 Status Request Command
  data[1] = 0xFF;                           // Set initial TTL to 255
  // WARNING: NETWORK ORDER IS BIG ENDIAN!!
  p= (byte*)&my_seq;                        // Set byte pointer
  data[2] = p[1]; data[3] = p[0];           // Pack 2 bytes from sequence into buffer
  p= (byte*)&my_addr;                       // Set byte pointer
  data[4] = p[1]; data[5] = p[0];           // Pack 2 bytes from source address into buffer
  p= (byte*)&dst_addr;                      // Set byte pointer
  data[6] = p[1]; data[7] = p[0];           // Pack 2 bytes from desitination address buffer
  data[8] = action;                         // Pack robot action
  data[9] = 0x00;                           // Reserved byte 1
  data[10] = 0x00;                          // Reserved byte 2
  data[11] = 0x00;                          // Reserved byte 3

  my_seq++;

  chibiTx(BROADCAST_ADDR, data,12);
}

//  ================================================================
//  Robot Functions
//  ================================================================

//  ----------------------------------------------------------------
//  roboCmd()  -  Command the robot to perform the requested action
//    Description     roboCmd sends the required bytes to the iRobot create to perform
//                    the requested action. WARNING: This won't work with DEBUG enabled
//  ----------------------------------------------------------------
void roboCmd(int action)
{
  for (int i=0; i < 5; i++) {               // For five iterations
    debugBeep(10);                          // Beep for 25 ms
  }
  
  switch (action)
  {
    case 0x00:    // START
      Serial.print(128, BYTE);
      Serial.print(132, BYTE);
      break;
    case 0x01:    // STOP
      Serial.print(137, BYTE);  //  Movement
      Serial.print(0x00, BYTE);
      Serial.print(0x00, BYTE);  // 0 mm/s
      Serial.print(0x80, BYTE);  // No turn
      Serial.print(0x00, BYTE);
      break;
    case 0x02:    // FWD
      Serial.print(137, BYTE);  //  Movement
      Serial.print(0x00, BYTE);
      Serial.print(0x80, BYTE);  // 0 mm/s
      Serial.print(0x80, BYTE);  // No turn
      Serial.print(0x00, BYTE);
      break;
    case 0x03:    // Left
      Serial.print(137, BYTE);  //  Movement
      Serial.print(0x00, BYTE);
      Serial.print(0x80, BYTE);  // 0 mm/s
      Serial.print(0x00, BYTE);  // No turn
      Serial.print(0x80, BYTE);
      break;
    case 0x04:    // Right
      Serial.print(137, BYTE);  //  Movement
      Serial.print(0x00, BYTE);
      Serial.print(0x80, BYTE);  // 0 mm/s
      Serial.print(0xff, BYTE);  // No turn
      Serial.print(0x80, BYTE);
      break;
  }
}

//  ================================================================
//  Status Functions
//  ================================================================

//  ----------------------------------------------------------------
//  statusSeqUpdate()  -  Check and update status table with latest sequence number
//    Description     statusSeqUpdate looks up the node by id in the status table
//                    and updates the sequence number to seq
//    return    true   if update successful
//              false  if update fails or is the sequence number is too old
//  ----------------------------------------------------------------
boolean statusSeqUpdate(unsigned int id, unsigned int seq)
{
  if (DEBUG) {
    Serial.println("statusSeqUpdate()");
  }
  
  if (!statusNodeFound(id)) {                    // If node id is not found
    if (!statusNodeAdd(id,0,millis())) {         // add node with an RSSI of 0
      return false;                                 // Return false if node cannot be added
    }
  }
  for (int i=0; i < status_count; i++) {    // Iterate across all known nodes
    if (status_id[i] == id) {               // If id matches ...
      if (seq > status_seq[i] ||                // If sequence number is newer or
          seq + SEQ_WINDOW < status_seq[i]) {   // If outside the sequence window
        status_seq[i] = seq;                // Update sequence number in status table
        return true;
      }
    }
  }
  return false;                             // Error updating node
}

//  ----------------------------------------------------------------
//  statusNodeUpdate()  -  Check and update status table with node's RSSI
//    Description     statusNodeUpdate looks up the node by id in the status table
//                    and updates the rssi and time
//    return    true   if update successful
//              false  if update fails 
//  ----------------------------------------------------------------
boolean statusNodeUpdate(unsigned int id,byte rssi,unsigned long time)
{
  if (DEBUG) {
    Serial.println("netNodeUpdate()");
  }
  if (id < 0xF000 || id > 0xF0FF) {
    return false;
  }
  if (!statusNodeFound(id)) {                    // If node id is not found
    if (statusNodeAdd(id,rssi,time)) {           // Add node to status table
      return true;                          // Success adding node
    }
    return false;                           // Error adding node
  } else {                                  // If node is found
    for (int i=0; i < status_count; i++) {  // Iterate across all known nodes
      if (status_id[i] == id) {             // If id matches
        status_rssi[i] = rssi;              // Update RSSI
        status_time[i] = time;              // Update time
        return true;                        // Success updating node
      }
    }
  }
  return false;                             // Error updating node
}

//  ----------------------------------------------------------------
//  statusNodeAdd()  -  Add node to the status table
//    Description     statusAddNode add the node by id and updates
//                    the rssi and time and increments the status_count
//    return    true   if update successful
//              false  if update fails 
//  ----------------------------------------------------------------
boolean statusNodeAdd(unsigned int id,byte rssi,unsigned long time)
{
  if (DEBUG) {
    Serial.print("statusNodeAdd(");
    Serial.print(id, HEX);
    Serial.print(", ");
    Serial.print(rssi, HEX);
    Serial.print(", ");
    Serial.print(time, HEX);
    Serial.println(")");
  }
  if (status_count < MAX_NODES) {           // If the table is not full
    status_id[status_count] = id;           // Add new node id
    status_rssi[status_count] = rssi;       // Add new node RSSI
    status_time[status_count] = time;       // Add new node time
    status_count++;                         // update node status counter
    return true;
  }
  return false;
}

//  ----------------------------------------------------------------
//  statusNodeFound()  -  search for node in status table
//    Description     statusNodeFound search for the node by id
//                    by iterating through the status table
//    return    true   if node found
//              false  if node not found
//  ----------------------------------------------------------------
boolean statusNodeFound(unsigned int id)
{
  if (DEBUG) {
    Serial.println("statusNodeFound(");
    Serial.print(id, HEX);
    Serial.println(")");
  }
  for (int i=0; i < status_count; i++) {    // Iterate across all known nodes
    if (DEBUG) {
      Serial.print("node_id[i] ");
      Serial.print(status_id[i],HEX);
      Serial.print("id ");
      Serial.print(id,HEX);
      Serial.println("");
    }
    if (status_id[i] == id) {
      return true;
    }
  }
  return false;
}

//  ================================================================
//  Debug Functions
//  ================================================================

//  ----------------------------------------------------------------
//  debugDumpState()  -  Dump status table
//    Description     debugDumpState dumps the status table to the serial port
//  ----------------------------------------------------------------
void debugDumpState()
{
  unsigned long now;
  now = millis();

  if (DEBUG || COMMANDER) {
    Serial.println("debugDumpState()");
    for (int i=0; i < status_count; i++) {
      Serial.print("ID: ");
      Serial.print(status_id[i], HEX);
      Serial.print(" RSSI: ");
      Serial.print(status_rssi[i], HEX);
      Serial.print(" Time: ");
      Serial.print((int)(now-status_time[i]),DEC);
      Serial.println("");
    }
  }
}

//  ----------------------------------------------------------------
//  debugDumpNet()  -  Dump network status table
//    Description     debugDumpNet dumps the network status table to the serial port
//  ----------------------------------------------------------------
void debugDumpNet(boolean filter)
{
  unsigned int to_addr, from_addr;
  int from_rssi;
  Serial.println("debugDumpNet()");
  Serial.println("----------------------------------------------------------------");
  for (int i=0; i < net_count; i++) {
    to_addr = net_index[i];
    for (int j=0; j < net_nodes[i]; j++) {
      from_addr = net_id[i][j];
      from_rssi = net_rssi[i][j];
      if (!filter || (net_rssi[i][j] >= 0x10)) {
        Serial.print("\t\"");
        Serial.print(from_addr,HEX);
        Serial.print("\" -> \"");
        Serial.print(to_addr,HEX);
        Serial.print("\" [ label = \"");
        Serial.print(from_rssi,HEX);
        Serial.println("\" ];");
      }
    }
  }
  Serial.println("----------------------------------------------------------------");    
}

//  ----------------------------------------------------------------
//  debugBeep()  -  Beep!
//    Description     debugBeep activates the buzzer for time milliseconds
//  ----------------------------------------------------------------
void debugBeep(int time)
{
  digitalWrite(9, LOW);
  delay(time);
  digitalWrite(9, HIGH);
  delay(time);
}




