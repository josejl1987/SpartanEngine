#pragma once

#include <cstdio>
#include <cstring>
#include <ctime>
#include <cstddef>
#include <cwchar>
#include <cstdarg>
#include <cstdlib>  // for getenv, setenv, strdup
#include <cerrno>   // for errno, EINVAL

// Types
typedef int errno_t;

// Macro constants
#ifndef _TRUNCATE
#define _TRUNCATE ((size_t)-1)
#endif

// Safe string functions replacements
inline int strncpy_s(char* dest, size_t destsz, const char* src, size_t count) {
    if (count == _TRUNCATE) {
        if (destsz == 0) return 0;
        strncpy(dest, src, destsz - 1);
        dest[destsz - 1] = '\0';
        return 0;
    }
    size_t copy_count = (count < destsz) ? count : destsz;
    strncpy(dest, src, copy_count);
    if (copy_count < destsz) {
        dest[copy_count] = '\0';
    } else if (destsz > 0) {
        dest[destsz - 1] = '\0';
    }
    return 0;
}

inline int strncpy_s(char* dest, size_t destsz, const char* src) {
    return strncpy_s(dest, destsz, src, _TRUNCATE);
}

inline int strcpy_s(char* dest, size_t destsz, const char* src) {
    if (destsz == 0) return 0;
    strncpy(dest, src, destsz - 1);
    dest[destsz - 1] = '\0';
    return 0;
}

inline int strncat_s(char* dest, size_t destsz, const char* src, size_t count) {
    size_t len = strlen(dest);
    if (len >= destsz) return 1;
    size_t remaining = destsz - len;
    strncat(dest, src, (count < remaining) ? count : remaining - 1);
    return 0;
}

inline int sprintf_s(char* buffer, size_t sizeOfBuffer, const char* format, ...) {
    va_list args;
    va_start(args, format);
    int res = vsnprintf(buffer, sizeOfBuffer, format, args);
    va_end(args);
    return res;
}

inline int vsprintf_s(char* buffer, size_t sizeOfBuffer, const char* format, va_list args) {
    return vsnprintf(buffer, sizeOfBuffer, format, args);
}

inline char* strtok_s(char* str, const char* delimiters, char** context) {
    return strtok_r(str, delimiters, context);
}

inline errno_t localtime_s(struct tm* _tm, const time_t* time) {
    return localtime_r(time, _tm) ? 0 : 1;
}

inline errno_t fopen_s(FILE** pFile, const char* filename, const char* mode) {
    if (!pFile || !filename || !mode) {
        if (pFile) *pFile = nullptr;
        return EINVAL;
    }
    *pFile = fopen(filename, mode);
    return (*pFile != nullptr) ? 0 : errno;
}

inline errno_t _dupenv_s(char** buffer, size_t* numberOfElements, const char* varname) {
    if (!buffer || !varname) {
        if (buffer) *buffer = nullptr;
        if (numberOfElements) *numberOfElements = 0;
        return EINVAL;
    }

    const char* env = getenv(varname);
    if (env) {
        *buffer = strdup(env);  // Allocate copy so caller can free()
        if (numberOfElements) *numberOfElements = strlen(env) + 1;
        return 0;
    }

    *buffer = nullptr;
    if (numberOfElements) *numberOfElements = 0;
    return 0;
}

inline errno_t _putenv_s(const char* varname, const char* value_string) {
    return setenv(varname, value_string, 1) ? errno : 0;
}

// Template overloads for fixed-size arrays (MSVC compatibility)
template <size_t N>
inline int strncpy_s(char (&dest)[N], const char* src, size_t count) {
    return strncpy_s(dest, N, src, count);
}

template <size_t N>
inline int strcpy_s(char (&dest)[N], const char* src) {
    return strcpy_s(dest, N, src);
}

template <size_t N>
inline int strncat_s(char (&dest)[N], const char* src, size_t count) {
    return strncat_s(dest, N, src, count);
}

template <size_t N>
inline int sprintf_s(char (&dest)[N], const char* format, ...) {
    va_list args;
    va_start(args, format);
    int res = vsnprintf(dest, N, format, args);
    va_end(args);
    return res;
}

#include <algorithm>

// Dummy specific Windows functions if needed
#ifdef _WIN32
// nothing
#else
// Ensure size_t and other types are available
struct FIBITMAP;
extern "C" {
    unsigned       FreeImage_GetBPP(FIBITMAP* dib);
    unsigned char* FreeImage_GetBits(FIBITMAP* dib);
    unsigned       FreeImage_GetWidth(FIBITMAP* dib);
    unsigned       FreeImage_GetHeight(FIBITMAP* dib);
    unsigned       FreeImage_GetPitch(FIBITMAP* dib);
}

inline int SwapRedBlue32(FIBITMAP* dib)
{
    if (FreeImage_GetBPP(dib) != 32)
        return 0;

    unsigned char* bits = FreeImage_GetBits(dib);
    unsigned w          = FreeImage_GetWidth(dib);
    unsigned h          = FreeImage_GetHeight(dib);
    unsigned pitch      = FreeImage_GetPitch(dib);

    for (unsigned y = 0; y < h; ++y)
    {
        unsigned char* row = bits + y * pitch;
        for (unsigned x = 0; x < w; ++x)
        {
            unsigned char tmp = row[x * 4 + 0];
            row[x * 4 + 0]    = row[x * 4 + 2];
            row[x * 4 + 2]    = tmp;
        }
    }
    return 1;
}
#endif
