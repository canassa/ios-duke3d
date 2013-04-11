// "Build Engine & Tools" Copyright (c) 1993-1997 Ken Silverman
// Ken Silverman's official web site: "http://www.advsys.net/ken"
// See the included license file "BUILDLIC.TXT" for license info.
// This file has been modified from Ken Silverman's original release

/* DDOI - This file is an attempt to reimplement a_nasm.asm in C */
/* FCS: However did that work: This is far from perfect but you have my eternal respect !!! */

#include "platform.h"
#include "build.h"
#include "tiles.h"
#include "draw.h"
#include "fixedPoint_math.h"

int32_t pixelsAllowed = 10000000000;

uint8_t  *transluc = NULL;

static int transrev = 0;


#define shrd(a,b,c) (((b)<<(32-(c))) | ((a)>>(c)))
#define shld(a,b,c) (((b)>>(32-(c))) | ((a)<<(c)))















/* ---------------  WALLS RENDERING METHOD (USED TO BE HIGHLY OPTIMIZED ASSEMBLY) ----------------------------*/
static uint8_t  machmv;

static uint8_t machxbits_al;
static uint8_t bitsSetup;
static uint8_t *textureSetup;
void sethlinesizes(int32_t i1, int32_t _bits, uint8_t *textureAddress)
{
    machxbits_al = i1;
    bitsSetup = _bits;
    textureSetup = textureAddress;
}



//FCS:   Draw ceiling/floors
//Draw a line from destination in the framebuffer to framebuffer-numPixels
void hlineasm4(int32_t numPixels, int32_t shade, uint32_t i4, uint32_t i5, uint8_t *dest, int32_t asm1, int32_t asm2, uint8_t *pallete)
{

    int32_t shifter = ((256-machxbits_al) & 0x1f);
    uint32_t source;

    uint8_t *texture = textureSetup;
    uint8_t bits = bitsSetup;

    shade = shade & 0xffffff00;
    numPixels++;

    if (!RENDER_DRAW_CEILING_AND_FLOOR) {
        return;
    }

    while (numPixels) {

        source = i5 >> shifter;
        source = shld(source,i4,bits);
        source = texture[source];

        if (pixelsAllowed-- > 0) {
            *dest = pallete[shade|source];
        }

        dest--;

        i5 -= asm1;
        i4 -= asm2;

        numPixels--;

    }
}

static int32_t rmach_eax;
static int32_t rmach_ebx;
static int32_t rmach_ecx;
static int32_t rmach_edx;
static int32_t rmach_esi;
void setuprhlineasm4(int32_t i1, int32_t i2, int32_t i3, int32_t i4, int32_t i5)
{
    rmach_eax = i1;
    rmach_ebx = i2;
    rmach_ecx = i3;
    rmach_edx = i4;
    rmach_esi = i5;
}


void rhlineasm4(int32_t i1, uint8_t *texture, int32_t i3, uint32_t i4, uint32_t i5, int32_t dest)
{
    uint32_t ebp = dest - i1;
    uint32_t rmach6b = ebp-1;
    int32_t numPixels;

    if (i1 <= 0) {
        return;
    }

    numPixels = i1;
    do {



        i3 = ((i3&0xffffff00)|(*texture));
        i4 -= rmach_eax;
        ebp = (((i4+rmach_eax) < i4) ? -1 : 0);
        i5 -= rmach_ebx;

        if ((i5 + rmach_ebx) < i5) {
            texture -= (rmach_ecx+1);
        } else {
            texture -= rmach_ecx;
        }

        ebp &= rmach_esi;
        i1 = ((i1&0xffffff00)|(((uint8_t *)i3)[rmach_edx]));

        if (pixelsAllowed-- > 0) {
            ((uint8_t *)rmach6b)[numPixels] = (i1&0xff);
        }

        texture -= ebp;
        numPixels--;
    } while (numPixels);
}

static int32_t rmmach_eax;
static int32_t rmmach_ebx;
static int32_t rmmach_ecx;
static int32_t rmmach_edx;
static int32_t setupTileHeight;
void setuprmhlineasm4(int32_t i1, int32_t i2, int32_t i3, int32_t i4, int32_t tileHeight)
{
    rmmach_eax = i1;
    rmmach_ebx = i2;
    rmmach_ecx = i3;
    rmmach_edx = i4;
    setupTileHeight = tileHeight;
}


//FCS: ????
void rmhlineasm4(int32_t i1, int32_t shade, int32_t colorIndex, int32_t i4, int32_t i5, int32_t dest)
{
    uint32_t ebp = dest - i1;
    uint32_t rmach6b = ebp-1;
    int32_t numPixels;

    if (i1 <= 0) {
        return;
    }

    numPixels = i1;
    do {



        colorIndex = ((colorIndex&0xffffff00)|(*((uint8_t *)shade)));
        i4 -= rmmach_eax;
        ebp = (((i4+rmmach_eax) < i4) ? -1 : 0);
        i5 -= rmmach_ebx;

        if ((i5 + rmmach_ebx) < i5) {
            shade -= (rmmach_ecx+1);
        } else {
            shade -= rmmach_ecx;
        }

        ebp &= setupTileHeight;

        //Check if this colorIndex is the transparent color (255).
        if ((colorIndex&0xff) != 255) {
            if (pixelsAllowed-- > 0) {
                i1 = ((i1&0xffffff00)|(((uint8_t *)colorIndex)[rmmach_edx]));
                ((uint8_t *)rmach6b)[numPixels] = (i1&0xff);
            }
        }

        shade -= ebp;
        numPixels--;

    } while (numPixels);
}


//Variable used to draw column.
//This is how much you have to skip in the framebuffer in order to be one pixel below.
static int32_t bytesperline;
void setBytesPerLine(int32_t _bytesperline)
{
    bytesperline = _bytesperline;
}


// Internal, only used by DrawVerticalLine
int32_t DrawTopBottomLines(int32_t vince, uint8_t *palette, int32_t vplce, uint8_t  *texture, uint8_t  *dest)
{

    if (!RENDER_DRAW_TOP_AND_BOTTOM_COLUMN) {
        return 0;
    }

    vince += vplce;
    vplce = ((uint32_t)vplce) >> machmv;
    vplce = (vplce&0xffffff00) | texture[vplce];

    if (pixelsAllowed-- > 0) {
        *dest = palette[vplce];
    }

    return vince;
}


int32_t DrawVerticalLine(int32_t vince, uint8_t *palette, int32_t numPixels, int32_t vplce, uint8_t *texture, uint8_t *dest)
{
    uint32_t temp;

    if (!RENDER_DRAW_WALL_BORDERS) {
        return vplce;
    }

    if (numPixels == 0) {
        return DrawTopBottomLines(vince, palette, vplce, texture, dest);
    }

    numPixels++;
    while (numPixels) {
        temp = ((uint32_t)vplce) >> machmv;
        temp = texture[temp];

        // 255 is the index for transparent color index. Skip drawing this pixel.
        if (temp != 255) {
            if (pixelsAllowed-- > 0) {
                *dest = palette[temp];
            }
        }

        vplce += vince;
        dest += bytesperline;
        numPixels--;
    }
    return vplce;
}


int32_t tvlineasm1(int32_t i1, uint8_t   *texture, int32_t numPixels, int32_t i4, uint8_t  *source, int16_t shiftval, uint8_t  *dest)
{
    uint8_t shiftValue = (shiftval & 0x1f);

    numPixels++;
    while (numPixels) {
        uint32_t temp = i4;
        temp >>= shiftValue;
        temp = source[temp];

        //255 is the index for transparent color index. Skip drawing this pixel.
        if (temp != 255) {
            uint16_t colorIndex;

            colorIndex = texture[temp];
            colorIndex |= ((*dest)<<8);

            if (transrev) {
                colorIndex = ((colorIndex>>8)|(colorIndex<<8));
            }

            if (pixelsAllowed-- > 0) {
                *dest = transluc[colorIndex];
            }
        }

        i4 += i1;

        //We are drawing a column ?!
        dest += bytesperline;
        numPixels--;
    }
    return i4;
}


void SetupVerticalLine(int32_t i1)
{
    //Only keep 5 first bits
    machmv = (i1&0x1f);
}

/* END ---------------  WALLS RENDERING METHOD (USED TO BE HIGHLY OPTIMIZED ASSEMBLY) ----------------------------*/





























/* ---------------  SPRITE RENDERING METHOD (USED TO BE HIGHLY OPTIMIZED ASSEMBLY) ----------------------------*/
static int32_t spal_eax;
static int32_t smach_eax;
static int32_t smach2_eax;
static int32_t smach5_eax;
static int32_t smach_ecx;
void setupspritevline(int32_t i1, int32_t i2, int32_t i3, int32_t i4, int32_t i5)
{
    spal_eax = i1;
    smach_eax = (i5<<16);
    smach2_eax = (i5>>16)+i2;
    smach5_eax = smach2_eax + i4;
    smach_ecx = i3;
}


void spritevline(int32_t i1, uint32_t i2, uint32_t i4, uint8_t *source, uint8_t *dest)
{


setup:

    i2 += smach_eax;
    i1 = (i1&0xffffff00) | (*source&0xff);
    if ((i2 - smach_eax) > i2) {
        source += smach2_eax + 1;
    } else {
        source += smach2_eax;
    }

    while (1) {

        i1 = (i1&0xffffff00) | (((uint8_t *)spal_eax)[i1]&0xff);

        if (pixelsAllowed-- > 0) {
            *dest = i1;
        }

        dest += bytesperline;

        i4 += smach_ecx;
        i4--;
        if (!((i4 - smach_ecx) > i4) && i4 != 0) {
            goto setup;
        }

        if (i4 == 0) {
            return;
        }

        i2 += smach_eax;

        i1 = (i1&0xffffff00) | (*source&0xff);

        if ((i2 - smach_eax) > i2) {
            source += smach5_eax + 1;
        } else {
            source += smach5_eax;
        }
    }
}


static int32_t mspal_eax;
static int32_t msmach_eax;
static int32_t msmach2_eax;
static int32_t msmach5_eax;
static int32_t msmach_ecx;
void msetupspritevline(int32_t i1, int32_t i2, int32_t i3, int32_t i4, int32_t i5)
{
    mspal_eax = i1;
    msmach_eax = (i5<<16);
    msmach2_eax = (i5>>16)+i2;
    msmach5_eax = smach2_eax + i4;
    msmach_ecx = i3;
}


void mspritevline(int32_t colorIndex, int32_t i2, int32_t i4, uint8_t   *source, uint8_t   *dest)
{

setup:
    i2 += smach_eax;

    colorIndex = (colorIndex&0xffffff00) | (*source&0xff);

    if ((i2 - smach_eax) > i2) {
        source += smach2_eax + 1;
    } else {
        source += smach2_eax;
    }

    while (1) {

        //Skip transparent pixels (index=255)
        if ((colorIndex&0xff) != 255) {
            colorIndex = (colorIndex&0xffffff00) | (((uint8_t *)spal_eax)[colorIndex]&0xff);

            if (pixelsAllowed-- > 0) {
                *dest = colorIndex;
            }
        }

        dest += bytesperline;
        i4 += smach_ecx;
        i4--;

        if (!((i4 - smach_ecx) > i4) && i4 != 0) {
            goto setup;
        }

        if (i4 == 0) {
            return;
        }

        i2 += smach_eax;

        colorIndex = (colorIndex&0xffffff00) | (*source&0xff);

        if ((i2 - smach_eax) > i2) {
            source += smach5_eax + 1;
        } else {
            source += smach5_eax;
        }
    }
}


uint8_t *tspal;
uint32_t tsmach_eax1;
uint32_t adder;
uint32_t tsmach_eax3;
uint32_t tsmach_ecx;
void tsetupspritevline(uint8_t *palette, int32_t i2, int32_t i3, int32_t i4, int32_t i5)
{
    tspal = palette;
    tsmach_eax1 = i5 << 16;
    adder = (i5 >> 16) + i2;
    tsmach_eax3 = adder + i4;
    tsmach_ecx = i3;
}


/*
 FCS: Draw a sprite vertical line of pixels.
 */
void DrawSpriteVerticalLine(int32_t i2, int32_t numPixels, uint32_t i4, uint8_t   *texture, uint8_t   *dest)
{
    uint8_t colorIndex;

    while (numPixels) {
        numPixels--;

        if (numPixels != 0) {

            i4 += tsmach_ecx;

            if (i4 < (i4 - tsmach_ecx)) {
                adder = tsmach_eax3;
            }

            colorIndex = *texture;

            i2 += tsmach_eax1;
            if (i2 < (i2 - tsmach_eax1)) {
                texture++;
            }

            texture += adder;

            //255 is the index of the transparent color: Do not draw it.
            if (colorIndex != 255) {
                uint16_t val;
                val = tspal[colorIndex];
                val |= (*dest)<<8;

                if (transrev) {
                    val = ((val>>8)|(val<<8));
                }

                colorIndex = transluc[val];

                if (pixelsAllowed-- > 0) {
                    *dest = colorIndex;
                }
            }

            //Move down one pixel on the framebuffer
            dest += bytesperline;
        }


    }
}
/* END---------------  SPRITE RENDERING METHOD (USED TO BE HIGHLY OPTIMIZED ASSEMBLY) ----------------------------*/
























/* ---------------  FLOOR/CEILING RENDERING METHOD (USED TO BE HIGHLY OPTIMIZED ASSEMBLY) ----------------------------*/

void settrans(int32_t type)
{
    transrev = type;
}

static uint8_t   *textureData;
static uint8_t   *mmach_asm3;
static int32_t mmach_asm1;
static int32_t mmach_asm2;

void mhline(uint8_t   *texture, int32_t i2, int32_t numPixels, int32_t i5, uint8_t *dest, int32_t asm1, int32_t asm2, int32_t asm3)
{
    textureData = texture;
    mmach_asm3 = asm3;
    mmach_asm1 = asm1;
    mmach_asm2 = asm2;
    mhlineskipmodify(i2,numPixels>>16,i5,dest);
}


static uint8_t  mshift_al = 26;
static uint8_t  mshift_bl = 6;
void mhlineskipmodify( uint32_t i2, int32_t numPixels, int32_t i5, uint8_t *dest)
{
    uint32_t ebx;
    int32_t colorIndex;

    while (numPixels >= 0) {
        ebx = i2 >> mshift_al;
        ebx = shld (ebx, (uint32_t)i5, mshift_bl);
        colorIndex = textureData[ebx];

        //Skip transparent color.
        if ((colorIndex&0xff) != 0xff) {
            if (pixelsAllowed-- > 0) {
                *dest = mmach_asm3[colorIndex];
            }
        }
        i2 += mmach_asm1;
        i5 += mmach_asm2;
        dest++;
        numPixels--;


    }
}


void msethlineshift(int32_t i1, int32_t i2)
{
    i1 = 256-i1;
    mshift_al = (i1&0x1f);
    mshift_bl = (i2&0x1f);
} /* msethlineshift */


static uint8_t *tmach_eax;
static uint8_t *tmach_asm3;
static int32_t tmach_asm1;
static int32_t tmach_asm2;

void thline(uint8_t   *i1, int32_t i2, int32_t i3, int32_t i5, uint8_t *i6, int32_t asm1, int32_t asm2, int32_t asm3)
{
    tmach_eax = i1;
    tmach_asm3 = asm3;
    tmach_asm1 = asm1;
    tmach_asm2 = asm2;
    thlineskipmodify(asm2,i2,i3,i5,i6);
}

static uint8_t  tshift_al = 26;
static uint8_t  tshift_bl = 6;
void thlineskipmodify(int32_t i1, uint32_t i2, uint32_t i3, int32_t i5, uint8_t *i6)
{
    uint32_t ebx;
    int counter = (i3>>16);
    while (counter >= 0) {
        ebx = i2 >> tshift_al;
        ebx = shld (ebx, (uint32_t)i5, tshift_bl);
        i1 = tmach_eax[ebx];
        if ((i1&0xff) != 0xff) {
            uint16_t val = tmach_asm3[i1];
            val |= (*i6)<<8;

            if (transrev) {
                val = ((val>>8)|(val<<8));
            }

            if (pixelsAllowed-- > 0) {
                *i6 = transluc[val];
            }
        }

        i2 += tmach_asm1;
        i5 += tmach_asm2;
        i6++;
        counter--;


    }
}


void tsethlineshift(int32_t i1, int32_t i2)
{
    i1 = 256-i1;
    tshift_al = (i1&0x1f);
    tshift_bl = (i2&0x1f);
}


//FCS: Render RENDER_SLOPPED_CEILING_AND_FLOOR
void slopevlin(uint8_t *framebuffer, int32_t *slopaloffs, int32_t cnt,
               int32_t bx, int32_t by, int32_t asm3,
               int32_t x3, int32_t y3, int32_t asm1,
               int32_t byter_per_line, tile_t *tile)
{
    int32_t bz, bzinc;
    uint32_t i, u, v, frame_index = 0;
    
    uint32_t glogx = tile->dim_power_2.width;
    uint32_t glogy = tile->dim_power_2.height;
    
    bz = asm3;
    bzinc = (asm1>>3);

    while (cnt--) {
        i = krecip(bz>>6);
        bz += bzinc;
        
        u = bx + x3 * i;
        v = by + y3 * i;
        
        int32_t index = ((u >> (32-glogx)) << glogy) + (v >> (32-glogy));
        framebuffer[frame_index] = *(char *)(slopaloffs[0] + tile->data[index]);
        
        slopaloffs--;
        frame_index -= byter_per_line;
    }
}


/* END ---------------  FLOOR/CEILING RENDERING METHOD (USED TO BE HIGHLY OPTIMIZED ASSEMBLY) ----------------------------*/
