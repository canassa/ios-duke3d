//
//  display.c
//  iOSBuild3D
//
//  Created by Cesar Canassa on 4/7/13.
//  Copyright (c) 2013 Cesar Canassa. All rights reserved.
//

#include "display.h"
#include "build.h"
#include "engine.h"
#include "draw.h"

void faketimerhandler () {

}

void Error (int errorType, char  *error, ...) {
    //exit(1);
}

uint8_t *frameplace;
uint8_t *frameoffset;
uint8_t lastPalette[768];

uint8_t temp_frame[WIDTH * HEIGHT];


Palette gPalette[256];

void setvmode(int mode) {
}

void clear2dscreen(void) {
}

void _nextpage(void) {
}

void readmousexy(short *x, short *y) {
}

void drawpixel(uint8_t   *location, uint8_t pixel) {
}

int32_t _setgamemode(uint8_t  davidoption, int32_t daxdim, int32_t daydim) {
    int j = 0, i = 0;
    
    xdim = WIDTH;
    ydim = HEIGHT;

    game_mode.bytesperline = WIDTH;
    game_mode.qsetmode = HEIGHT;

    frameoffset = frameplace = (uint8_t *)temp_frame;


    j = ydim * 4 * sizeof(int32_t);  /* Leave room for horizlookup&horizlookup2 */

    if (horizlookup) {
        free(horizlookup);
    }

    if (horizlookup2) {
        free(horizlookup2);
    }

    horizlookup = (int32_t *)malloc(j);
    horizlookup2 = (int32_t *)malloc(j);

    j = 0;

    //Build lookup table (X screespace -> frambuffer offset.
    for (i = 0; i <= ydim; i++) {
        ylookup[i] = j;
        j += game_mode.bytesperline;
    }

    horizycent = ((ydim*4)>>1);

    /* Force drawrooms to call dosetaspect & recalculate stuff */
    oxyaspect = oxdimen = oviewingrange = -1;

    //Let the Assembly module how many pixels to skip when drawing a column
    setBytesPerLine(game_mode.bytesperline);


    setview(0L,0L,xdim-1,ydim-1);

    setbrightness(curbrightness, palette);

    if (searchx < 0) {
        searchx = halfxdimen;
        searchy = (ydimen>>1);
    }

    return 0;
}

void _uninitengine(void) {
}

int setupmouse(void) {
    return 0;
}

void readmousebstatus(short *bstatus) {
}

void _updateScreenRect(int32_t x, int32_t y, int32_t w, int32_t h) {
}

int VBE_setPalette(uint8_t  *palettebuffer) {
    uint8_t  *p = palettebuffer;
    
    for (int i = 0; i < 256; i++) {
        gPalette[i].blue = (uint8_t) ((((float) *p++) / 63.0) * 255.0);
        gPalette[i].green = (uint8_t) ((((float) *p++) / 63.0) * 255.0);
        gPalette[i].red = (uint8_t) ((((float) *p++) / 63.0) * 255.0);
        p++;   /* This byte is unused in BUILD, too. */
    }

    return 0;
}

uint8_t  readpixel(uint8_t   *location) {
    return 0;
}