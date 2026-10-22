#include <stdarg.h>
#include <stdio.h>

int showmsg(const char *format, ...)
{
    va_list args;

    if (!format||!*format) return 0;
    va_start(args,format);
    vfprintf(stderr,format,args);
    va_end(args);
    fputc('\n',stderr);
    return 0;
}
