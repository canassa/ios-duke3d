// converted from asm to c by Jonof

#include <stdio.h>
#include <math.h>
#include "platform.h"
#include "fixedPoint_math.h"


//CC: Returns 1/denominator shifted 30
int32_t krecip(int32_t denominator)
{
    return (int32_t)(1073741824.0/denominator);
}

int32_t fixedPointSqrt(uint32_t radicand)
{
    return (int32_t)floor(sqrt(radicand));
}


double_t toRadiains(int32_t degree)
{
    return ((double_t)degree * M_PI) / 1024.0;
}

// fixedPointSin(ang)     = sin(ang * (3.141592/1024)) * 16383
// fixedPointSin(ang+512) = cos(ang * (3.141592/1024)) * 16383
int32_t fixedPointSin(int32_t degree)
{
    return (int32_t)(sin(toRadiains(degree)) * 16383);
}


int32_t fixedPointCos(int32_t degree)
{
    return (int32_t)(cos(toRadiains(degree)) * 16383);
}


void clearbuf(void *d, int32_t c, int32_t a)
{
    int32_t *p = (int32_t *)d;
    while ((c--) > 0) {
        *(p++) = a;
    }
}

void clearbufbyte(void *D, int32_t c, int32_t a)
{
    // Cringe City
    uint8_t  *p = (uint8_t *)D;
    int32_t m[4] = { 0xffl,0xff00l,0xff0000l,0xff000000l };
    int32_t n[4] = { 0,8,16,24 };
    int32_t z=0;
    while ((c--) > 0) {
        *(p++) = (uint8_t )((a & m[z])>>n[z]);
        z=(z+1)&3;
    }
}

void copybuf(void *s, void *d, int32_t c)
{
    int32_t *p = (int32_t *)s, *q = (int32_t *)d;
    while ((c--) > 0) {
        *(q++) = *(p++);
    }
}

void copybufbyte(void *S, void *D, int32_t c)
{
    uint8_t  *p = (uint8_t *)S, *q = (uint8_t *)D;
    while ((c--) > 0) {
        *(q++) = *(p++);
    }
}

void copybufreverse(void *S, void *D, int32_t c)
{
    uint8_t  *p = (uint8_t *)S, *q = (uint8_t *)D;
    while ((c--) > 0) {
        *(q++) = *(p--);
    }
}

void qinterpolatedown16(int32_t *bufptr, int32_t num, int32_t val, int32_t add)
{
    // gee, I wonder who could have provided this...
    int32_t i, *lptr = bufptr;
    for (i=0; i<num; i++) {
        lptr[i] = (val>>16);
        val += add;
    }
}

void qinterpolatedown16short(int32_t *bufptr, int32_t num, int32_t val, int32_t add)
{
    // ...maybe the same person who provided this too?
    int32_t i;
    short *sptr = (short *)bufptr;
    for (i=0; i<num; i++) {
        sptr[i] = (short)(val>>16);
        val += add;
    }
}

