#ifndef ELT_TRANSPORT_H
#define ELT_TRANSPORT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "elt_define.h"
#include "pthread.h"

#ifdef _WIN32
#include <winsock2.h>
typedef SOCKET elt_socket;
#endif

#ifdef __linux__
#include <sys/socket.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <unistd.h>

typedef int elt_socket;
#endif

#define ELT_TRANSPORT_SUCCESS 0
#define ELT_TRANSPORT_ERROR -1
#define ELT_TRANSPORT_WRITE_FAILURE -2
#define ELT_TRANSPORT_READ_FAILURE -3

#define ELT_TRANSPORT_CMD_BUFFER_SIZE 1024

typedef struct elt_transport {
    char* addr;
    int port;
    //int    timeout;
    //char*  token;
    pthread_mutex_t mutex;
    elt_socket sock;
} elt_transport;

int elt_transport_init(elt_transport* transport, const char *addr, int port);

int elt_transport_connect(elt_transport* transport);

int elt_transport_call(elt_transport* transport, char* cmd, char* receiveMessage);

void elt_transport_close(elt_transport* transport);

void elt_transport_finalize(elt_transport* transport);

#ifdef __cplusplus
}
#endif

#endif // ELT_TRANSPORT_H
