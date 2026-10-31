/* Small string helpers shared by the simulator modules. */
#ifndef SIM_UTIL_H
#define SIM_UTIL_H

#include <ctype.h>
#include <stdbool.h>
#include <stdlib.h>

static inline bool sim_streq(const char *a, const char *b)
{
    while (*a && *b) {
        if (tolower((unsigned char)*a) != tolower((unsigned char)*b)) return false;
        a++;
        b++;
    }
    return *a == *b;
}

/* Accepts decimal, negative, and 0x hex. */
static inline bool sim_parse_double(const char *s, double *out)
{
    char *end;
    if (s == NULL || *s == '\0') return false;
    if (s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) *out = (double)strtoul(s, &end, 16);
    else *out = strtod(s, &end);
    return *end == '\0';
}

static inline bool sim_parse_long(const char *s, long *out)
{
    char *end;
    if (s == NULL || *s == '\0') return false;
    *out = strtol(s, &end, 0);
    return *end == '\0';
}

#endif
