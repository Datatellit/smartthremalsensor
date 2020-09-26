#ifndef __PROTOCOL_PARSER_H
#define __PROTOCOL_PARSER_H

#include "_global.h"
#include "ProtocolBus.h"

extern uint16_t delaySendTick;

uint8_t ParseProtocol();

void Msg_RequestNodeID();
void Msg_Presentation();
void Msg_DevState(const uint8_t _state, const uint8_t _hasid);
void Msg_SendPIR(const uint8_t _value);

#endif /* __PROTOCOL_PARSER_H */