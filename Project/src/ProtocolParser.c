#include "ProtocolParser.h"
#include "_global.h"
#include "MyMessage.h"
#include "xliNodeConfig.h"
#include "rf24l01.h"

uint16_t delaySendTick = 0;
bool bDelaySend = FALSE;

uint8_t ParseProtocol(){
  if( rcvMsg.header.destination != gConfig.nodeID && rcvMsg.header.destination != BROADCAST_ADDRESS ) return 0;
  
  uint8_t ret = ParseCommonProtocol();
  if(ret) return 1;
  
  uint8_t _cmd = miGetCommand();
  uint8_t _sender = rcvMsg.header.sender;  // The original sender
  uint8_t _type = rcvMsg.header.type;
  uint8_t _sensor = rcvMsg.header.sensor;
  uint8_t _lenPayl = miGetLength();
  bool _needAck = (bool)miGetRequestAck();
  bool _isAck = (bool)miGetAck();
  
  switch( _cmd ) {
  case C_INTERNAL:
      if( _type == I_CONFIG ) {
      // Node Config
      switch( _sensor ) {
        case NCF_CFG_PIRTIMEOUT:
          {
            uint8_t  timeouttick = (rcvMsg.payload.data[0]<<8 | rcvMsg.payload.data[1]);
            if(timeouttick !=0 && timeouttick<=60000)
            {
              gConfig.timeout = timeouttick;
            }
          }
         break; 
      }
    }
    break;
      
  case C_PRESENTATION:
    break;
  
  case C_REQ:
    if( _needAck ) {
      if( IS_MINE_SUBID(_sensor) ) {
        if( _type == V_STATUS ) {
/*
//typedef struct
//{
//    //uint8_t devNum;
//    uint8_t devType1;
//    uint8_t devType2;
//    uint8_t devType3;
      ...
//    uint8_t devType5;
//}MyMsgPayload_t  
*/ 
          MsgScanner_ProbeAck(_sender);      
///////////////////not common config////////////////
          uint8_t len = moGetLength()-1;
          sndMsg.payload.data[len++] = gConfig.timeout>>8;
          sndMsg.payload.data[len++] = (gConfig.timeout&0xFF);
          moSetLength(len);
///////////////////not common config////////////////       
          return 1;
        }
      }
    }    
    break;
  case C_SET:
    if( _isAck ) {
    }    
    break;
  }
  
  return 0;
}

void Msg_SendPIR(uint8_t _value) {
  build(NODEID_GATEWAY, S_MOTION, C_PRESENTATION, V_STATUS, 0, 0);
  moSetPayloadType(P_BYTE);
  moSetLength(1);
  sndMsg.payload.data[0] = _value;
  bMsgReady = 1;
}

void Msg_RequestNodeID() {
  // Request NodeID for remote
  build(BASESERVICE_ADDRESS, NODE_TYP_REMOTE, C_INTERNAL, I_ID_REQUEST, 1, 0);
  moSetPayloadType(P_ULONG32);
  moSetLength(UNIQUE_ID_LEN);
  memcpy(sndMsg.payload.data, _uniqueID, UNIQUE_ID_LEN);
  bMsgReady = 1;
}

// Prepare device presentation message
void Msg_Presentation() {
  build(NODEID_GATEWAY, S_ZENSENSOR, C_PRESENTATION, gConfig.type, 1, 0);
  moSetPayloadType(P_ULONG32);
  moSetLength(UNIQUE_ID_LEN);
  memcpy(sndMsg.payload.data, _uniqueID, UNIQUE_ID_LEN);
  bMsgReady = 1;
}