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
//  Robot Mesh Controller
//  ================================================================

//  ----------------------------------------------------------------
//  Includes
//  ----------------------------------------------------------------
#include <chibi.h>

//  ----------------------------------------------------------------
//  Definitions
//  ----------------------------------------------------------------

#define COMMANDER  1                        // Command Line interface
#define HELO_INTERVAL 5000                  // Milliseconds between HELO broadcasts
#define SREQ_INTERVAL 1000
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
unsigned long queue_time;                   // Time queued command was last sent
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

byte queue_cmd = 0x00;
int queue_state = QUEUE_IDLE;

/**************************************************************************/
// Initialize
/**************************************************************************/
void setup()
{  
  chibiCmdInit(57600);                      // Init cmd line and set serial port to 57600 bps
  pinMode(9, OUTPUT);                       // Configure Pin 9 as an output
  digitalWrite(9, HIGH);                    // Set Pin 9 HIGH to turn off buzzer
  my_addr = chibiGetShortAddr();            // Get node address from EEPROM
  chibiInit();                              // Initialize the chibi wireless stack

  chibiCmdAdd("s", cmdRoboInit);
  chibiCmdAdd("x", cmdRoboStop);
  chibiCmdAdd("w", cmdRoboFwd);
  chibiCmdAdd("d", cmdRoboRight);
  chibiCmdAdd("a", cmdRoboLeft);
  chibiCmdAdd("q", cmdRoboIdle);
  chibiCmdAdd("p", cmdNetPing);
  chibiCmdAdd("r", netStatReset);
  chibiCmdAdd("i", cmdDumpState);
  chibiCmdAdd("net", cmdDumpNet);
  
  memset(net_nodes,0x00,MAX_NODES);
  debugBeep(100);  
  delay(100);
  debugBeep(50);
  debugBeep(50);
  debugBeep(50);
  debugBeep(50);
}

/**************************************************************************/
// Loop
/**************************************************************************/
void loop()
{
  // This function checks the command line to see if anything new was typed.
  chibiCmdPoll();
  
  // Check if any data was received from the radio. If so, then handle it.
  if (chibiDataRcvd() == true)
  { 
    byte *p;
    int len, rssi, node_rssi, count;
    byte ttl;
    unsigned int org_addr, src_addr, dst_addr, net_addr, cmd_seq;
    unsigned long tx_time;
    byte buf[100];  // this is where we store the received data
    
    // retrieve the data and the signal strength
    len = chibiGetData(buf);
    rssi = chibiGetRSSI();
    src_addr = chibiGetSrcAddr();

    statusNodeUpdate(src_addr,rssi,millis());
    /*
    if (buf[0] == 0x09) {
      p= (byte*)&tx_time;               // Set byte pointer
      p[0] = buf[4]; p[1] = buf[3];     // Unpack 2 bytes from buf into cmd_seq
      p[2] = buf[2]; p[3] = buf[1];     // Unpack 2 bytes from buf into cmd_seq
      Serial.print("TEST: ");
      Serial.print(tx_time,HEX);
      Serial.println("");
    }
    */
    if (buf[0] == 0x07) {
      if (DEBUG) {
        Serial.println("Robot Control Packet Received");
      }
      // WARNING: NETWORK ORDER IS BIG ENDIAN!!
      ttl = buf[1];                     // Unpack TTL from buffer
      p= (byte*)&cmd_seq;               // Set byte pointer
      p[0] = buf[3]; p[1] = buf[2];     // Unpack 2 bytes from buf into cmd_seq
      p= (byte*)&org_addr;              // Set byte pointer
      p[0] = buf[5]; p[1] = buf[4];     // Unpack 2 bytes from buf into org_addr
      p= (byte*)&dst_addr;              // Set byte pointer
      p[0] = buf[7]; p[1] = buf[6];     // Unpack 2 bytes from buf into dst_addr

      net_update(org_addr);
      if (statusSeqUpdate(org_addr,cmd_seq)) {
        if (buf[8] == ROBO_ACK) {
          queue_state = QUEUE_IDLE;
          Serial.println("Robo Command ACKed");
        }
      }      
    }
    if (buf[0] == 0x02) {
      if (DEBUG) {
        Serial.println("Status Response Packet Received");
      }
      // WARNING: NETWORK ORDER IS BIG ENDIAN!!
      ttl = buf[1];                     // Unpack TTL from buffer
      p= (byte*)&cmd_seq;               // Set byte pointer
      p[0] = buf[3]; p[1] = buf[2];     // Unpack 2 bytes from buf into cmd_seq
      p= (byte*)&org_addr;              // Set byte pointer
      p[0] = buf[5]; p[1] = buf[4];     // Unpack 2 bytes from buf into org_addr
      p= (byte*)&dst_addr;              // Set byte pointer
      p[0] = buf[7]; p[1] = buf[6];     // Unpack 2 bytes from buf into dst_addr

      net_update(org_addr);
      if (statusSeqUpdate(org_addr,cmd_seq)) {
        count = buf[8];
        if (DEBUG) {
          Serial.print("SRC: ");
          Serial.print(src_addr,HEX);
          Serial.print(" TTL: ");
          Serial.print(ttl,HEX);
          Serial.print(" ORG: ");
          Serial.print(org_addr,HEX);
          Serial.print(" DST: ");
          Serial.print(dst_addr,HEX);
          Serial.print(" SEQ: ");
          Serial.print(cmd_seq,HEX);
          Serial.print(" LEN: ");
          Serial.print(count,DEC);
          Serial.println("");
        }
        for (int i=0; i < count; i++) {
          p= (byte*)&net_addr;
          p[0] = buf[12+(i*3)+1];
          p[1] = buf[12+(i*3)];
          node_rssi = buf[12+(i*3)+2];
          if (DEBUG) {
            Serial.print("NODE: ");
            Serial.print(net_addr,HEX);
            Serial.print(" RSSI: ");
            if (node_rssi < 0x10) {
              Serial.print("0");
            }
            Serial.print(node_rssi,HEX);
            Serial.println("");
          }
          net_update_node(org_addr,net_addr,node_rssi);
        }
        // debugDumpNet(DUMP_FULL);
      }
    }
  }
  if (millis() - sreq_time > SREQ_INTERVAL) {
    sreq_time = millis();
    msgStatusRequest(my_addr,0xf000 + sreq_index);
    sreq_index++;
    if (sreq_index>4) {    // Iterate from 0xF001 to 0xF004
      sreq_index = 1;
    }
  }
  if (queue_state == QUEUE_SEND ||
      (queue_state == QUEUE_WAIT && (millis() - queue_time > QUEUE_INTERVAL))) {
      queue_time = millis();
      if (queue_state == QUEUE_SEND) {
        Serial.println("Sending Initial Robot Command");
        queue_state = QUEUE_WAIT;
      } else {
        Serial.println("Repeating Robot Command");
      }
      msgRobotControl(0xf001,queue_cmd);
  }
}

//  ================================================================
//  Command Functions
//  ================================================================

void cmdDumpNet(int arg_cnt, char **args)
{
  debugDumpNet(DUMP_FILTER);
}

void cmdDumpState(int arg_cnt, char **args)
{
  debugDumpState();
}

void cmdNetPing(int arg_cnt, char **args)
{
  unsigned int src_addr, dst_addr;
  dst_addr = chibiCmdStr2Num(args[1], 16);  
  if (arg_cnt != 2) {
    dst_addr = BROADCAST_ADDR;
  }
  src_addr = chibiGetShortAddr();  
  msgStatusRequest(src_addr,dst_addr);
}

void cmdRoboInit(int arg_cnt, char **args)
{
  queue_state = QUEUE_SEND;
  queue_cmd = ROBO_INIT;
}

void cmdRoboStop(int arg_cnt, char **args)
{
  queue_state = QUEUE_SEND;
  queue_cmd = ROBO_STOP;
}

void cmdRoboFwd(int arg_cnt, char **args)
{
  queue_state = QUEUE_SEND;
  queue_cmd = ROBO_FWD;
}

void cmdRoboLeft(int arg_cnt, char **args)
{
  queue_state = QUEUE_SEND;
  queue_cmd = ROBO_LEFT;
}

void cmdRoboRight(int arg_cnt, char **args)
{
  queue_state = QUEUE_SEND;
  queue_cmd = ROBO_RIGHT;
}

void cmdRoboIdle(int arg_cnt, char **args)
{
  queue_state = QUEUE_IDLE;
}

int strCat(char *buf, unsigned char index, char arg_cnt, char **args)
{
    uint8_t i, len;
    char *data_ptr;

    data_ptr = buf;
    for (i=0; i<arg_cnt - index; i++)
    {
        len = strlen(args[i+index]);
        strcpy((char *)data_ptr, (char *)args[i+index]);
        data_ptr += len;
        *data_ptr++ = ' ';
    }
    *data_ptr++ = '\0';

    return data_ptr - buf;
}

void netStatReset(int arg_cnt, char **args)
{
  status_count = 0;

  memset(status_id,0x00,MAX_NODES*2);
  memset(status_seq,0x00,MAX_NODES*2);
  memset(status_rssi,0x00,MAX_NODES);
  memset(status_time,0x00,MAX_NODES*4);

  memset(net_id,0x00,MAX_NODES*MAX_NODES*2);
  memset(net_index,0x00,MAX_NODES*2);
  memset(net_rssi,0x00,MAX_NODES*MAX_NODES);
  memset(net_nodes,0x00,MAX_NODES);
  net_count=0;
  Serial.println("Clearing Network State Database");
}



//  ================================================================
//  Message Functions
//  ================================================================

//  ----------------------------------------------------------------
//  msgStatusRequest()  -  Send a status request message
//    Description     msgStatusRequest sends a status request (SREQ) from
//                    the origin address org_addr to the destination dst_addr
//  ----------------------------------------------------------------
void msgStatusRequest(unsigned int org_addr,unsigned int dst_addr)
{
  byte data[8];
  byte *p;
  if (DEBUG) {
    Serial.print("msgStatusRequest(");
    Serial.print(org_addr,HEX);
    Serial.print(", ");
    Serial.print(dst_addr,HEX);
    Serial.println(")");
  }
  data[0] = 0x01;                           // SREQ 0x01 Status Request Command
  data[1] = 0xFF;                           // Set initial TTL to 255

  // WARNING: NETWORK ORDER IS BIG ENDIAN!!
  p= (byte*)&my_seq;                        // Set byte pointer
  data[2] = p[1]; data[3] = p[0];           // Pack 2 bytes from sequence into buffer
  p= (byte*)&org_addr;                      // Set byte pointer
  data[4] = p[1]; data[5] = p[0];           // Pack 2 bytes from source address into buffer
  p= (byte*)&dst_addr;                      // Set byte pointer
  data[6] = p[1]; data[7] = p[0];           // Pack 2 bytes from desitination address buffer
  
  my_seq++;

  chibiTx(BROADCAST_ADDR, data,8);
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
//  Network Functions
//  ================================================================


boolean net_found_node(unsigned int net, unsigned int id)
{
  int index;
  index = net_getindex(net);
  for (int i=0; i < net_count; i++) {
    if (net_id[index][i] == id) {
      if (DEBUG) {
        Serial.print(net_id[index][i],HEX);
        Serial.print(" ");
        Serial.print(id,HEX);
        Serial.println("");
      }
      return true;
    }
    else if (DEBUG)
    {
      Serial.print("NO MATCH?!~ ");
      Serial.print(net_id[index][i],HEX);
      Serial.print(" ");
      Serial.print(id,HEX);
      Serial.println("");
    }
  }
  if (DEBUG) {
    Serial.println("NOT FOUND?!");
  }
  return false;
}

int net_getindex(unsigned int id)
{
  for (int i=0; i < net_count; i++) {
    if (net_index[i] == id) {
      return i;
    }
  }
  return -1;
}

boolean net_update(unsigned int id)
{
  if (!net_found(id)) {
    net_index[net_count] = id;
    net_count++;
  }
}

boolean net_update_node(unsigned int org_addr,unsigned int net_addr,int rssi)
{  
  int index, nodes;
  net_update(net_addr);
  net_update(org_addr);
  index = net_getindex(org_addr);
  if (index == -1) {
    return false;
  }
  if (!net_found_node(org_addr,net_addr)) {
    if (DEBUG) {
      Serial.print("NET ADD: ");
      Serial.print(org_addr,HEX);
      Serial.print(" ");
      Serial.print(net_addr,HEX);
      Serial.println("");
    }
    nodes = net_nodes[index];
    net_id[index][nodes] = net_addr;
    net_rssi[index][nodes] = rssi;
    net_nodes[index]++;
  } else {
    for (int i=0; i < net_nodes[index]; i++) {
      if (net_id[index][i] == net_addr) {
        net_rssi[index][i] = rssi;
        return i;
      }
    }
  }
  
}

boolean net_found(unsigned int id)
{
  for (int i=0; i < net_count; i++) {
    if (net_index[i] == id) {
      return true;
    }
  }
  return false;
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
  net_update_node(my_addr,id,rssi);
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




