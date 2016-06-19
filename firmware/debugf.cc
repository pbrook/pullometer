#include <stdarg.h>
#include <stdint.h>
#include <string.h>
#include "debugf.h"

static void
print_base(unsigned long val, uint8_t base, int size, char pad, char sign)
{
    char buf[12];
    int n;
    char c;

    for (n = 0; val; n++) {
        c = val % base;
        val /= base;
        if (c < 10)
          buf[n] = c + '0';
        else
          buf[n] = c  - 10 + 'a';
    }
    if (n == 0)
        buf[n++] = '0';
    if (sign)
        buf[n++] = sign;
    while (n < size)
        buf[n++] = pad;
    while (n--)
        debugf_putc(buf[n]);
}

static void
print_string(const char *s, int size)
{
    int n;

    n = strlen(s);
    while (n < size) {
        debugf_putc(' ');
        size--;
    }
    while (*s) {
        debugf_putc(*s);
        s++;
    }
}

void
debugf(const char * msg, ...)
{
    va_list va;
    char c;
    char pad;
    int size;
    bool is_unsigned;
    bool is_long;
    unsigned long val;
    char sign;

    va_start(va, msg);
    while (true) {
        c = *(msg++);
        switch (c) {
        case 0:
            va_end(va);
            return;
        case '%':
            pad = ' ';
            is_unsigned = false;
            is_long = false;
            size = 0;
        more_format:
            c = *(msg++);
            switch (c) {
            case '0':
                if (size == 0) {
                    pad = '0';
                    goto more_format;
                }
                /* Fall through...  */
            case '1':
            case '2':
            case '3':
            case '4':
            case '5':
            case '6':
            case '7':
            case '8':
            case '9':
                size = (size * 10) + (c - '0');
                goto more_format;
            case 'l':
                is_long = true;
                goto more_format;
            case 'u':
                is_unsigned = true;
                goto more_format;
            case 'x':
                if (is_long)
                    val = va_arg(va, unsigned long);
                else
                    val = va_arg(va, unsigned int);
                print_base(val, 16, size, pad, 0);
                break;
            case 'd':
                sign = 0;
                if (is_unsigned) {
                    if (is_long)
                        val = va_arg(va, unsigned long);
                    else
                        val = va_arg(va, unsigned int);
                } else {
                    long signedval;
                    if (is_long)
                        signedval = va_arg(va, long);
                    else
                        signedval = va_arg(va, int);
                    val = signedval;
                    if (signedval < 0) {
                        val = -val;
                        sign = '-';
                    }
                }
                print_base(val, 10, size, pad, sign);
                break;
            case 'c':
                debugf_putc(va_arg(va, int));
                break;
            case 's':
                print_string(va_arg(va, const char *), size);
            default:
                msg--;
                break;
            }
            break;
        default:
            debugf_putc(c);
            break;
        }
    }
}

