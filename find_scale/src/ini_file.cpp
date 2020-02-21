/**
 * @file
 * @brief initialization file read and write API implementation
 * @author Lai Han
 * @date 2019-3-22
 * @version 0.1
 */

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>
#include <ctype.h>
#include "ini_file.h"

#ifdef __cplusplus
extern "C"
{
#endif

static int load_ini_file(const char *file, char *buf, unsigned long *file_size)
{
    FILE *in = nullptr;
    unsigned long i = 0;
    *file_size = 0;

    assert(file != nullptr);
    assert(buf != nullptr);

    in = fopen(file, "r");
    if (nullptr == in)
    {
        return 0;
    }

    buf[i] = (char)fgetc(in);

    //load initialization file
    while (buf[i] != (char)EOF)
    {
        i++;
        assert(i < MAX_FILE_SIZE); //file too big, you can redefine MAX_FILE_SIZE to fit the big file
        buf[i] = (char)fgetc(in);
    }

    buf[i] = '\0';
    *file_size = i;

    fclose(in);
    return 1;
}

static int newline(char c)
{
    return ('\n' == c || '\r' == c) ? 1 : 0;
}

static int end_of_string(char c)
{
    return '\0' == c ? 1 : 0;
}

static int left_barce(char c)
{
    return LEFT_BRACE == c ? 1 : 0;
}

static int isright_brace(char c)
{
    return RIGHT_BRACE == c ? 1 : 0;
}

static int parse_file(const char *section, const char *key, const char *buf, unsigned long *sec_s, unsigned long *sec_e,
                      unsigned long *key_s, unsigned long *key_e, unsigned long *value_s, unsigned long *value_e)
{
    const char *p = buf;
    unsigned long i = 0;

    assert(buf != nullptr);
    assert(section != nullptr && strlen(section));
    assert(key != nullptr && strlen(key));

    *sec_e = *sec_s = *key_e = *key_s = *value_s = *value_e = 0;

    while (!end_of_string(p[i]))
    {
        //find the section
        if ((0 == i || newline(p[i - 1])) && left_barce(p[i]))
        {
            unsigned long section_start = i + 1;

            //find the ']'
            do
            {
                i++;
            } while (!isright_brace(p[i]) && !end_of_string(p[i]));

            if (0 == strncmp(p + section_start, section, i - section_start))
            {
                unsigned long newline_start = 0;

                i++;

                //Skip over space char after ']'
                while (isspace(p[i]))
                {
                    i++;
                }

                //find the section
                *sec_s = section_start;
                *sec_e = i;

                while (!(newline(p[i - 1]) && left_barce(p[i])) && !end_of_string(p[i]))
                {
                    unsigned long j = 0;
                    //get a new line
                    newline_start = i;

                    while (!newline(p[i]) && !end_of_string(p[i]))
                    {
                        i++;
                    }

                    //now i  is equal to end of the line
                    j = newline_start;

                    if (';' != p[j]) //skip over comment
                    {
                        while (j < i && p[j] != '=')
                        {
                            j++;
                            if ('=' == p[j])
                            {
                                if (strncmp(key, p + newline_start, j - newline_start) == 0)
                                {
                                    //find the key ok
                                    *key_s = newline_start;
                                    *key_e = j - 1;

                                    *value_s = j + 1;
                                    *value_e = i;

                                    return 1;
                                }
                            }
                        }
                    }

                    i++;
                }
            }
        }
        else
        {
            i++;
        }
    }
    return 0;
}

/**
*@brief read string in initialization file\n
* retrieves a string from the specified section in an initialization file
*@param section [in] name of the section containing the key name
*@param key [in] name of the key pairs to value
*@param value [in] pointer to the buffer that receives the retrieved string
*@param size [in] size of result's buffer
*@param default_value [in] default value of result
*@param file [in] path of the initialization file
*@return 1 : read success; \n 0 : read fail
*/
int read_profile_string(const char *section, const char *key, char *value, unsigned long size, const char *file)
{
    char buf[MAX_FILE_SIZE] = {0};
    unsigned long file_size;
    unsigned long sec_s, sec_e, key_s, key_e, value_s, value_e;

    //check parameters
    assert(section != nullptr && strlen(section));
    assert(key != nullptr && strlen(key));
    assert(value != nullptr);
    assert(size > 0);
    assert(file != nullptr && strlen(key));

    if (!load_ini_file(file, buf, &file_size))
    {
        return 0;
    }

    if (!parse_file(section, key, buf, &sec_s, &sec_e, &key_s, &key_e, &value_s, &value_e))
    {
        return 0; //not find the key
    }
    else
    {
        unsigned long cpcount = value_e - value_s;

        if (size - 1 < cpcount)
        {
            cpcount = size - 1;
        }

        memset(value, 0, size);
        memcpy(value, buf + value_s, cpcount);
        value[cpcount] = '\0';

        return 1;
    }
}

/**
*@brief read int value in initialization file\n
* retrieves int value from the specified section in an initialization file
*@param section [in] name of the section containing the key name
*@param key [in] name of the key pairs to value
*@param default_value [in] default value of result
*@param file [in] path of the initialization file
*@return profile int value,if read fail, return default value
*/
int read_profile_int(const char *section, const char *key, int *value, const char *file)
{
    char szValue[MAX_VALUE_SIZE] = {0};
    char *pValueEnd;

    if (!read_profile_string(section, key, szValue, sizeof(szValue), file))
    {
        return 0;
    }
    else
    {
        *value = (int)strtol(szValue, &pValueEnd, 10);
        if (pValueEnd >= szValue + MAX_VALUE_SIZE)
        {
            return 0;
        }

        return 1;
    }
}

int read_profile_int_vector(const char *section, const char *key, const char seperator, int *value, unsigned long *size, const char *file)
{
    char szArray[MAX_VALUE_VECTOR_SIZE] = {0};
    char szTempValue[MAX_VALUE_SIZE] = {0};
    unsigned long ulArraySize;
    unsigned long ulMaxSize = *size;
    char *pChar;
    char *pValue;
    char *pValueEnd;

    if (!read_profile_string(section, key, szArray, sizeof(szArray), file))
    {
        return 0;
    }

    ulArraySize = strlen(szArray);
    if (ulArraySize == 0)
    {
        return 0;
    }

    *size = 0;
    pChar = szArray;
    pValue = szTempValue;

    for (; pChar < szArray + ulArraySize; pChar++)
    {
        if (*pChar == ' ' || *pChar == '(' || *pChar == ')')
        {
            continue;
        }

        if (*pChar != seperator)
        {
            *pValue = *pChar;

            if (pChar != szArray + ulArraySize - 1)
            {
                pValue++;
                continue;
            }
        }

        //pChar is seperator or array end
        *value = (int)strtol(szTempValue, &pValueEnd, 10);
        if (pValueEnd >= szTempValue + MAX_VALUE_SIZE)
        {
            return 0;
        }

        value++;
        (*size)++;
        pValue = szTempValue;
        memset(szTempValue, 0, MAX_VALUE_SIZE);

        if (*size == ulMaxSize)
        {
            break;
        }
    }

    return 1;
}

int read_profile_double(const char *section, const char *key, double *value, const char *file)
{
    char szValue[MAX_VALUE_SIZE] = {0};
    char *pValueEnd;

    if (!read_profile_string(section, key, szValue, sizeof(szValue), file))
    {
        return 0;
    }
    else
    {
        *value = strtod(szValue, &pValueEnd);
        if (pValueEnd >= szValue + MAX_VALUE_SIZE)
        {
            return 0;
        }

        return 1;
    }
}

/**
 * @brief write a profile string to a ini file
 * @param section [in] name of the section,can't be nullptr and empty string
 * @param key [in] name of the key pairs to value, can't be nullptr and empty string
 * @param value [in] profile string value
 * @param file [in] path of ini file
 * @return 1 : success\n 0 : failure
 */
int write_profile_string(const char *section, const char *key, const char *value, const char *file)
{
    char buf[MAX_FILE_SIZE] = {0};
    char w_buf[MAX_FILE_SIZE] = {0};
    unsigned long sec_s, sec_e, key_s, key_e, value_s, value_e;
    unsigned long value_len = strlen(value);
    unsigned long file_size;
    FILE *out;

    //check parameters
    assert(section != nullptr && strlen(section));
    assert(key != nullptr && strlen(key));
    assert(value != nullptr);
    assert(file != nullptr && strlen(key));

    if (!load_ini_file(file, buf, &file_size))
    {
        sec_s = 0;
    }
    else
    {
        parse_file(section, key, buf, &sec_s, &sec_e, &key_s, &key_e, &value_s, &value_e);
    }

    if (0 == sec_s)
    {
        if (0 == file_size)
        {
            sprintf(w_buf + file_size, "[%s]\n%s=%s\n", section, key, value);
        }
        else
        {
            //not find the section, then add the new section at end of the file
            memcpy(w_buf, buf, file_size);
            sprintf(w_buf + file_size, "\n[%s]\n%s=%s\n", section, key, value);
        }
    }
    else if (0 == key_s)
    {
        //not find the key, then add the new key=value at end of the section
        memcpy(w_buf, buf, sec_e);
        sprintf(w_buf + sec_e, "%s=%s\n", key, value);
        sprintf(w_buf + sec_e + strlen(key) + strlen(value) + 2, buf + sec_e, file_size - sec_e);
    }
    else
    {
        //update value with new value
        memcpy(w_buf, buf, value_s);
        memcpy(w_buf + value_s, value, value_len);
        memcpy(w_buf + value_s + value_len, buf + value_e, file_size - value_e);
    }

    out = fopen(file, "w");
    if (nullptr == out)
    {
        return 0;
    }

    if (-1 == fputs(w_buf, out))
    {
        fclose(out);
        return 0;
    }

    fclose(out);
    return 1;
}

int write_profile_int(const char *section, const char *key, int value, const char *file)
{
    char arrValue[MAX_VALUE_SIZE] = {0};
    sprintf(arrValue, "%d", value);
    if (!write_profile_string(section, key, arrValue, file))
    {
        return 0;
    }
    else
    {
        return 1;
    }
}

int write_profile_double(const char *section, const char *key, double value, const char *file)
{
    char arrValue[MAX_VALUE_SIZE] = {0};
    sprintf(arrValue, "%f", value);
    if (!write_profile_string(section, key, arrValue, file))
    {
        return 0;
    }
    else
    {
        return 1;
    }
}

#ifdef __cplusplus
}; //end of extern "C" {
#endif
