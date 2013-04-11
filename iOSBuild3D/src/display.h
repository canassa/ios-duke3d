//
//  display.h
//  iOSBuild3D
//
//  Created by Cesar Canassa on 4/7/13.
//  Copyright (c) 2013 Cesar Canassa. All rights reserved.
//

#ifndef iOSBuild3D_display_h
#define iOSBuild3D_display_h

#include "inttypes.h"

#define WIDTH 480
#define HEIGHT 300

typedef struct {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} Palette;

extern uint8_t *frameplace;
extern uint8_t *frameoffset;
extern uint8_t lastPalette[768];
extern uint8_t temp_frame[WIDTH * HEIGHT];
extern Palette gPalette[256];

void setvmode(int mode);
void clear2dscreen(void);
void _nextpage(void);
void readmousexy(short *x, short *y);
void drawpixel(uint8_t   *location, uint8_t pixel);
int32_t _setgamemode(uint8_t  davidoption, int32_t daxdim, int32_t daydim);
void _uninitengine(void);
int setupmouse(void);
void readmousebstatus(short *bstatus);
void _updateScreenRect(int32_t x, int32_t y, int32_t w, int32_t h);
int VBE_setPalette(uint8_t  *palettebuffer);
uint8_t  readpixel(uint8_t   *location);

#endif
