#ifndef __PROTOCOL_PARSER_H
#define __PROTOCOL_PARSER_H

#include "_global.h"

extern uint8_t bMsgReady;

uint8_t ParseProtocol();
void build(uint8_t _destination, uint8_t _sensor, uint8_t _command, uint8_t _type, bool _enableAck, bool _isAck);
void Msg_NodeConfigAck(uint8_t _to, uint8_t _ncf);
void Msg_NodeConfigData(uint8_t _to);
void Msg_RequestNodeID();
void Msg_Presentation();
void Msg_SendPIR(uint8_t _value);

bool ProcessOutputCfgMsg();

#endif /* __PROTOCOL_PARSER_H */