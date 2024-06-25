////////////////  1   ////////////////

#ifndef _CAN_PROTO_INTERFACE_H_
#define _CAN_PROTO_INTERFACE_H_

#include <cstdio>
#include <stdint.h>

class ICanDevInferface
{
public:
    ICanDevInferface(){};
    virtual ~ICanDevInferface(){};

    virtual bool init() = 0;

    virtual bool send(uint8_t *data, uint32_t len) = 0;
    virtual bool read(uint8_t *data) = 0;
    virtual bool can_is_ok() = 0;
    virtual bool parseCanValue(uint8_t *data, uint32_t &id, uint8_t *value) = 0;
    virtual void encapValue(uint8_t *data, uint32_t &send_len, uint32_t &id, uint8_t *value, uint8_t data_len) = 0;
};
#endif
