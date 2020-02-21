/**
 * @file
 * @brief initialization file read and write API
 *	-size of the ini file must less than 16K
 *	-after '=' in key value pair, can not support empty char. this would not like WIN32 API
 *	-support comment using ';' prefix
 *	-can not support multi line
 * @author Lai Han
 * @date 2019-3-22
 * @version 0.1
 */

#ifndef INI_FILE_H
#define INI_FILE_H

#define MAX_FILE_SIZE 1024*16
#define LEFT_BRACE '['
#define RIGHT_BRACE ']'
#define MAX_VALUE_SIZE 32

#ifdef __cplusplus
extern "C"
{
#endif

int read_profile_string(const char *section, const char *key, char *value, int size, const char *default_value, const char *file);
int read_profile_int(const char *section, const char *key, int default_value, const char *file);
int read_profile_double(const char *section, const char *key, double *value, const char *file);

int write_profile_string(const char *section, const char *key, const char *value, const char *file);
int write_profile_int(const char *section, const char *key, int value, const char *file);
int write_profile_double(const char *section, const char *key, double value, const char *file);

#ifdef __cplusplus
}; //end of extern "C" {
#endif

#endif //end of INI_FILE_H

