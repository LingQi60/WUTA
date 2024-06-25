
#ifndef _CAN_SOCKET_H_
#define _CAN_SOCKET_H_

#include <iostream>
#include "common/can_box_interface.hpp"
#include <iostream>
#include <string>
#include <cstdio>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <unistd.h>
#include <pthread.h>
#include <arpa/inet.h>
#include <cstring>

// 该类需要给别人使用,需要创建hpp文件

class CanSocket : public ICanDevInferface
{
public:
    CanSocket(int port);
    ~CanSocket();

    // 初始化 socket连接,并监听
    bool init();
    // socket 读
    bool send(uint8_t *data, uint32_t len);
    // socket 写
    bool read(uint8_t *data);
    // 校验是否有can client连接
    bool can_is_ok();
    // 将socket can读取的数据进行解析出id与8byte数据
    bool parseCanValue(uint8_t *data, uint32_t &id, uint8_t *value);
    // 通过id和8type数据进行封装后
    void encapValue(uint8_t *data, uint32_t &send_len, uint32_t &id, uint8_t *value,uint8_t data_len);

private:
    int _sock;
    int _listen_port;
    int _client_sock;
    static void *run(void *arg);       // 注明 状态static
};

#endif
