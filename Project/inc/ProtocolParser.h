#ifndef __PROTOCOL_PARSER_H
#define __PROTOCOL_PARSER_H

#include "_global.h"
#include "ProtocolBus.h"

extern bool bDelaySend;
extern uint16_t delaySendTick;


uint8_t ParseProtocol();

void Msg_RequestNodeID();
void Msg_Presentation();
void Msg_SendPIR(uint8_t _value);

bool ProcessOutputCfgMsg();

#endif /* __PROTOCOL_PARSER_H */