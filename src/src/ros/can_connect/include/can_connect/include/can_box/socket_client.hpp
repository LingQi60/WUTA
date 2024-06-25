
#ifndef _V2X_SOCKET_H_
#define _V2X_SOCKET_H_

#include <iostream>
#include "common/can_box_interface.hpp"
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

class ClientSocket : public ICanDevInferface
{
public:
    ClientSocket(int port,std::string ip);
    ~ClientSocket();

    // 初始化 socket连接,并监听
    bool init();
    // socket 读
    bool send(uint8_t *data, uint32_t len);
    // socket 写
    bool read(uint8_t *data);
    // 校验是否有client连接
    bool can_is_ok();
    // 将socket client读取的数据进行解析出id与8byte数据
    bool parseCanValue(uint8_t *data, uint32_t &id, uint8_t *value);
    // 通过id和8type数据进行封装后
    void encapValue(uint8_t *data, uint32_t &send_len, uint32_t &id, uint8_t *value,uint8_t data_len);

private:
    int _sock;
    int _service_port;
    std::string _service_ip;
    int _client_sock;
    static void *run(void *arg);       // 注明 状态static
};

#endif
