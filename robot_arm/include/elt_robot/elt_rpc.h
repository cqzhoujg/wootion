#ifndef JSON_RPC_H
#define JSON_RPC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "elt_ctx.h"

/* JSON-rpc 返回结构 */
typedef struct jsonrpc_result_t {
    int success;                        // 标记是否成功
    cJSON *data;                        // 返回结果
    int err_code;                       // 错误码
    char *err_msg;                      // 错误信息
}jsonrpc_result;

// JSON-rpc error code
#define JSON_RPC_ERR_NONE               0
#define JSON_RPC_ERR_UNKNOWN            -32000
#define JSON_RPC_ERR_CMD_FAILURE        -32001
#define JSON_RPC_ERR_NEW_REQ_FAILURE    -32002
#define JSON_RPC_ERR_PARAM_INVALID      -32003
#define JSON_RPC_ERR_RES_DATA_INVALID   -32004

// CMD
#define CMD_MAX_LEN                     1024

cJSON *jsonrpc_new_request_by_null_params(const char *fun, int id);
cJSON *jsonrpc_new_request(const char *fun, cJSON *params, int id);
jsonrpc_result json_rpc_call(ELT_CTX ctx, cJSON* req);
void jsonrpc_result_release(jsonrpc_result *result);

#ifdef __cplusplus
}
#endif

#endif // JSON_RPC_H
