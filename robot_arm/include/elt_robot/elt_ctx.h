#ifndef ELT_CTX_H
#define ELT_CTX_H

#ifdef __cplusplus
extern "C" {
#endif

#include "elt_transport.h"

/* 登陆及操作时的上下文 */
struct elt_ctx {

    struct elt_transport* transport;
};

typedef struct elt_ctx* ELT_CTX;

/**
 * 初始化上下文
 * @ctx 上下文的指针
 * @addr 目标机械臂控制系统的IP地址
 * @port 服务端口号，目前默认8055
 * @return 成功返回ctx, 错误返回NULL
 */
ELT_SDK_PUBLIC(ELT_CTX) elt_create_ctx(const char* addr, int port);


/**
 * 退出时，销毁上下文
 * @ctx 上下文的指针
 * @return 成功或错误
 */
ELT_SDK_PUBLIC(int) elt_destroy_ctx(ELT_CTX ctx);

#ifdef __cplusplus
}
#endif

#endif // ELT_CTX_H
