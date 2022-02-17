#ifndef _FSERVO_LOG_H_
#define _FSERVO_LOG_H_

#include <assert.h>
#include <cstdio>
#include <errno.h>
#include <stdio.h>
#include <string.h>

#define OPT_LOG 2

#ifdef _UNIX //linux log

#if OPT_LOG < 2
#define LOG_DEBUG(format, ...)                                                                  \
    do {                                                                                        \
        static const char *file_last_level = strrchr(__FILE__, '/');                            \
        static const char *file_name = file_last_level ? file_last_level + 1 : file_last_level; \
        fprintf(stderr, "[%s:%d][\033[1;36mdebug\033[0m] " format "\n",                         \
                file_name, __LINE__, ##__VA_ARGS__);                                            \
        \                                    
                                                       \
    } while (0)
#else
#define LOG_DEBUG(format, ...)
#endif

#if OPT_LOG < 3
#define LOG_INFO(format, ...)                                                                   \
    do {                                                                                        \
        static const char *file_last_level = strrchr(__FILE__, '/');                            \
        static const char *file_name = file_last_level ? file_last_level + 1 : file_last_level; \
        fprintf(stderr, "[%s:%d][\033[1;32minfo\033[0m] " format "\n",                          \
                file_name, __LINE__, ##__VA_ARGS__);                                            \
        \                                    
                                                       \
    } while (0)
#else
#define LOG_INFO(format, ...)
#endif

#if OPT_LOG < 4
#define LOG_WARN(format, ...)                                                                   \
    do {                                                                                        \
        static const char *file_last_level = strrchr(__FILE__, '/');                            \
        static const char *file_name = file_last_level ? file_last_level + 1 : file_last_level; \
        fprintf(stderr, "[%s:%d][\033[1;33mwarn\033[0m] " format "\n",                          \
                file_name, __LINE__, ##__VA_ARGS__);                                            \
        \                                    
                                                       \
    } while (0)
#else
#define LOG_WARN(format, ...)
#endif

#if OPT_LOG < 5
#define LOG_ERR(format, ...)                                                                    \
    do {                                                                                        \
        static const char *file_last_level = strrchr(__FILE__, '/');                            \
        static const char *file_name = file_last_level ? file_last_level + 1 : file_last_level; \
        fprintf(stderr, "[%s:%d][\033[1;31merror\033[0m] " format "\n",                         \
                file_name, __LINE__, ##__VA_ARGS__);                                            \
        \                                    
                                                       \
    } while (0)
#else
#define LOG_ERR(format, ...)
#endif

#else //windows log

#if OPT_LOG < 2
#define LOG_DEBUG(format, ...)                                                                \
    do {                                                                                      \
        static const char *file_last_level = strrchr(__FILE__, '\\');                         \
        static const char *file_name = file_last_level ? file_last_level + 1 : __FILE__;      \
        fprintf(stderr, "[%s:%d][debug] " format " \n ", file_name, __LINE__, ##__VA_ARGS__); \
    } while (0)
#else
#define LOG_DEBUG(format, ...)
#endif

#if OPT_LOG < 3
#define LOG_INFO(format, ...)                                                                \
    do {                                                                                     \
        static const char *file_last_level = strrchr(__FILE__, '\\');                        \
        static const char *file_name = file_last_level ? file_last_level + 1 : __FILE__;     \
        fprintf(stderr, "[%s:%d][info] " format " \n ", file_name, __LINE__, ##__VA_ARGS__); \
    } while (0)
#else
#define LOG_INFO(format, ...)
#endif

#if OPT_LOG < 4
#define LOG_WARN(format, ...)                                                              \
    do {                                                                                   \
        static const char *file_last_level = strrchr(__FILE__, '\\');                      \
        static const char *file_name = file_last_level ? file_last_level + 1 : __FILE__;   \
        fprintf(stderr, "[%s:%d][warn] " format "\n", file_name, __LINE__, ##__VA_ARGS__); \
    } while (0)
#else
#define LOG_WARN(format, ...)
#endif

#if OPT_LOG < 5
#define LOG_ERR(format, ...)                                                                \
    do {                                                                                    \
        static const char *file_last_level = strrchr(__FILE__, '\\');                       \
        static const char *file_name = file_last_level ? file_last_level + 1 : __FILE__;    \
        fprintf(stderr, "[%s:%d][error] " format "\n", file_name, __LINE__, ##__VA_ARGS__); \
    } while (0)
#else
#define LOG_ERR(format, ...)
#endif

#endif

#define FSERVO_ASSERT(expression) assert((expression))

#define FSERVO_CHECK(exp)                                      \
    do {                                                       \
        if (false == (exp)) {                                  \
            LOG_ERR("Check failed, invalid expression:" #exp); \
            FSERVO_ASSERT(exp);                                \
        }                                                      \
    } while (0)

#endif
