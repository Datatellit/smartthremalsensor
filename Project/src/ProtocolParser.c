#include "ProtocolParser.h"
#include "_global.h"
#include "MyMessage.h"
#include "xliNodeConfig.h"
#include "rf24l01.h"

uint16_t delaySendTick = 0;

bool NeedProcess(const uint8_t _targetNID, uint8_t *arrType, uint8_t num)
{
  // If BROADCAST_ADDRESS msg, using NodeType; 
  // If group, using DevType;
  // Otherwise, return true directly
  uint8_t lv_type;
  if( _targetNID == BROADCAST_ADDRESS ) lv_type = NodeID2Type(_targetNID);
  else if( IS_GROUP_NODEID(_targetNID) ) lv_type = gConfig.type;
  else return TRUE;
  for( uint8_t tidx = 0; tidx < num; tidx++ ) {
    // Return true if matches any one in the list
    if(*(arrType+tidx) == lv_type) return TRUE;
  }
  return FALSE;
}

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
  uint16_t timeouttick;
  
  switch( _cmd ) {
  case C_INTERNAL:
    if( _type == I_CONFIG ) {
      // Node Config
      switch( _sensor ) {
        case NCF_CFG_PIRTIMEOUT:
          if(_lenPayl > 0 ) {
            timeouttick = rcvMsg.payload.data[0];
            if(_lenPayl > 1 ) timeouttick += (rcvMsg.payload.data[1]<<8);
            if( timeouttick > 0 ) {
              gConfig.timeout = timeouttick;
              gIsConfigChanged = TRUE;
            }
          }
        break;
        case NCF_CFG_PIR_MSG_MAX_INTERVAL:
          timeouttick = 0;
          if(_lenPayl > 0 ) {
            timeouttick = rcvMsg.payload.data[0];
            if(_lenPayl > 1 ) timeouttick += (rcvMsg.payload.data[1]<<8);
          }
          gConfig.keepalive = timeouttick;
          gIsConfigChanged = TRUE;
        break;
      }
    }
    break;
      
  case C_PRESENTATION:
    if( _sensor == S_ZENSENSOR ) {
      if( _isAck ) {
        // Device/client got Response to Presentation message, ready to work
        gConfig.token = rcvMsg.payload.uiValue;
        gConfig.present = (gConfig.token >  0);
        //GotPresented();
        // Need update subID & devType?
        if(_lenPayl > 2) {
          if( gConfig.subID != rcvMsg.payload.data[2] ) {
              gConfig.subID = rcvMsg.payload.data[2];
              gIsStatusChanged = TRUE;
          }
          if( gConfig.type != _type ) {
              gConfig.type = _type;
              gIsStatusChanged = TRUE;
          }
        }
      }
    }    
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
          //MsgScanner_ProbeAck(_sender);
///////////////////not common config////////////////
          //uint8_t len = moGetLength()-1;
          //sndMsg.payload.data[len++] = gConfig.timeout>>8;
          //sndMsg.payload.data[len++] = (gConfig.timeout&0xFF);
          //moSetLength(len);
///////////////////not common config////////////////       
          bool bNeedProcess = TRUE;
          uint8_t devTypeNum = _lenPayl;
          if(devTypeNum > 0) {
            bNeedProcess = NeedProcess(rcvMsg.header.destination, &rcvMsg.payload.data[0],devTypeNum);
          }
          if(bNeedProcess) {
            delaySendTick = GetDelayTick(rcvMsg.header.destination);
            Msg_DevState(mSysStatus, 1);
          }          
          return 0;
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

void Msg_SendPIR(const uint8_t _value) {
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
  copyBuffer(sndMsg.payload.data, _uniqueID, UNIQUE_ID_LEN);
  bMsgReady = 1;
}

// Prepare device presentation message
void Msg_Presentation(const uint8_t _rack) {
  build(NODEID_GATEWAY, S_ZENSENSOR, C_PRESENTATION, gConfig.type, _rack, 0);
  moSetPayloadType(P_ULONG32);
  moSetLength(UNIQUE_ID_LEN);
  copyBuffer(sndMsg.payload.data, _uniqueID, UNIQUE_ID_LEN);
  if( gSubIDChanged ) {
    sndMsg.payload.data[UNIQUE_ID_LEN] = gConfig.subID;
    moSetLength(UNIQUE_ID_LEN + 1);
    gSubIDChanged = FALSE;
  }
  bMsgReady = 1;
}

// Device Status Notification
void Msg_DevState(const uint8_t _state, const uint8_t _hasid) {
  uint8_t payl_len = 0;
  build(NODEID_GATEWAY, gConfig.subID, C_REQ, V_STATUS, 0, 1);
  if(_hasid) {
    copyBuffer(sndMsg.payload.data, _uniqueID, UNIQUE_ID_LEN);
    payl_len += UNIQUE_ID_LEN;
    moSetVersion(2);
    moSetPayloadType(P_CUSTOM);
  } else {
    moSetPayloadType(P_BYTE);
  }
  sndMsg.payload.data[payl_len++] = _state;
  moSetLength(payl_len); 
  bMsgReady = 1;
}
