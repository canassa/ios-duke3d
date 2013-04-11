/*
 * "Build Engine & Tools" Copyright (c) 1993-1997 Ken Silverman
 * Ken Silverman's official web site: "http://www.advsys.net/ken"
 * See the included license file "BUILDLIC.TXT" for license info.
 * This file has been modified from Ken Silverman's original release
 */

#ifndef _INCLUDE_BUILD_H_
#define _INCLUDE_BUILD_H_
#ifdef _WIN32
#include "windows/inttypes.h"
#else
#include <inttypes.h>
#endif

#define MAXSECTORS 1024
#define MAXWALLS 8192
#define MAXSPRITES 4096
#define MAXTILES 9216

#define MAXSTATUS 1024
#define MAXPLAYERS 16
#define MAXXDIM 1600
#define MAXYDIM 1200
#define MAXPALOOKUPS 256
#define MAXPSKYTILES 256
#define MAXSPRITESONSCREEN 1024

#define CLIPMASK0 (((1L)<<16)+1L)
#define CLIPMASK1 (((256L)<<16)+64L)


/*
 * Make all variables in BUILD.H defined in the ENGINE,
 *  and externed in GAME
 * (dear lord.  --ryan.)
 */
#ifdef ENGINE
#define EXTERN
#else
#define EXTERN extern
#endif

#pragma pack(1)

/*
 * ceilingstat/floorstat:
 *   bit 0: 1 = parallaxing, 0 = not                                 "P"
 *   bit 1: 1 = groudraw, 0 = not
 *   bit 2: 1 = swap x&y, 0 = not                                    "F"
 *   bit 3: 1 = double smooshiness                                   "E"
 *   bit 4: 1 = x-flip                                               "F"
 *   bit 5: 1 = y-flip                                               "F"
 *   bit 6: 1 = Align texture to first wall of sector                "R"
 *   bits 7-8:                                                       "T"
 *          00 = normal floors
 *          01 = masked floors
 *          10 = transluscent masked floors
 *          11 = reverse transluscent masked floors
 *   bits 9-15: reserved
 */


typedef struct {
    unsigned parallaxing                 :1;
    unsigned groudraw                    :1;
    unsigned swap_xy                     :1;
    unsigned double_smooshiness          :1;
    unsigned x_flip                      :1;
    unsigned y_flip                      :1;
    unsigned align_texture_to_first_wall :1;
    // I checked all levels in Duke Nukem Atomic edition but none the 'type' flag
    // I guess this is used for room over room in Shadow Warrior, but I am not sure
    unsigned type                        :2;
    unsigned reserved_1                  :1;
    unsigned reserved                    :6;
} SectorFlags;

// Defines for the 'type' flag
//#define SECTOR_NORMAL               0
//#define SECTOR_MASKED               1
//#define SECTOR_TRANSLUSCENT         2
//#define SECTOR_REVERSE_TRANSLUSCENT 3

/* 40 bytes */
typedef struct {
    short wallptr, wallnum;
    int32_t ceilingz, floorz;
    short ceilingstat, floorstat;
    short ceilingpicnum, ceilingheinum;
    int8_t ceilingshade;
    uint8_t  ceilingpal, ceilingxpanning, ceilingypanning;
    short floorpicnum, floorheinum;
    int8_t floorshade;
    uint8_t  floorpal, floorxpanning, floorypanning;
    uint8_t  visibility, filler;
    short lotag, hitag, extra;
} SectorOnGRP;

struct _Sector;

struct _InnerSector {
    int32_t z;
    SectorFlags flags;
    short picnum, heinum;
    int8_t shade;
    uint8_t  pal, xpanning, ypanning;
    struct _Sector *sector;
} _InnerSector;

struct _Sector {
    short wallptr, wallnum;
    uint8_t  visibility, filler;
    short lotag, hitag, extra;
    struct _InnerSector floor;
    struct _InnerSector ceiling;
};

typedef struct _InnerSector InnerSector;
typedef struct _Sector Sector;

/*
 * cstat:
 *   bit 0: 1 = Blocking wall (use with clipmove, getzrange)         "B"
 *   bit 1: 1 = bottoms of invisible walls swapped, 0 = not          "2"
 *   bit 2: 1 = align picture on bottom (for doors), 0 = top         "O"
 *   bit 3: 1 = x-flipped, 0 = normal                                "F"
 *   bit 4: 1 = masking wall, 0 = not                                "M"
 *   bit 5: 1 = 1-way wall, 0 = not                                  "1"
 *   bit 6: 1 = Blocking wall (use with hitscan / cliptype 1)        "H"
 *   bit 7: 1 = Transluscence, 0 = not                               "T"
 *   bit 8: 1 = y-flipped, 0 = normal                                "F"
 *   bit 9: 1 = Transluscence reversing, 0 = normal                  "T"
 *   bits 10-15: reserved
 */

typedef struct {
    unsigned blocking                 :1;
    unsigned bottom_texture_swap      :1;
    unsigned align_bottom             :1;
    unsigned x_flip                   :1;
    unsigned masking                  :1;
    unsigned one_way                  :1;
    unsigned hitscan                  :1;
    unsigned y_flip                   :1;
    unsigned transluscence_reversing  :1;
    unsigned reserved                 :7;
} WallFlags;

/* 32 bytes */
typedef struct {
    int32_t x, y;               // Coordinate of left side of wall, get right side from next wall's left side
    short point2;               // Index to next wall on the right (always in the same sector)
    short nextwall;             // Index to wall on other side of wall (-1 if there is no sector)
    short nextsector;           // Index to sector on other side of wall (-1 if there is no sector)
    WallFlags flags;
    short picnum;               // texture index into art file
    short overpicnum;           // texture index into art file for masked walls / 1-way walls
    int8_t shade;               // shade offset of wall
    uint8_t pal;                // palette lookup table number (0 - use standard colors)
    uint8_t xrepeat, yrepeat;   // used to change the size of pixels (stretch textures)
    uint8_t xpanning, ypanning; // used to align textures or to do texture panning
    short lotag, hitag, extra;  // Used by the game only
} walltype;


/*
 * WallFlags:
 *   bit 0: 1 = Blocking sprite (use with clipmove, getzrange)       "B"
 *   bit 1: 1 = transluscence, 0 = normal                            "T"
 *   bit 2: 1 = x-flipped, 0 = normal                                "F"
 *   bit 3: 1 = y-flipped, 0 = normal                                "F"
 *   bits 5-4: 00 = FACE sprite (default)                            "R"
 *             01 = WALL sprite (like masked walls)
 *             10 = FLOOR sprite (parallel to ceilings&floors)
 *   bit 6: 1 = 1-sided sprite, 0 = normal                           "1"
 *   bit 7: 1 = Real centered centering, 0 = foot center             "C"
 *   bit 8: 1 = Blocking sprite (use with hitscan / cliptype 1)      "H"
 *   bit 9: 1 = Transluscence reversing, 0 = normal                  "T"
 *   bits 10-14: reserved
 *   bit 15: 1 = Invisible sprite, 0 = not invisible
 */

typedef struct {
    unsigned blocking                 :1;
    unsigned transluscence            :1;
    unsigned x_flip                   :1;
    unsigned y_flip                   :1;
    unsigned type                     :2;
    unsigned one_sided                :1;
    unsigned real_centered            :1;
    unsigned hitscan                  :1;
    unsigned transluscence_reversing  :1;
    unsigned reserved                 :5;
    unsigned invisible                :1;
} SpriteFlags;

#define FACE_SPRITE          0
#define WALL_SPRITE          1
#define FLOOR_SPRITE         2
#define UNKNOW_SPRITE        3  // TODO: WTF is this type?

/* 44 bytes */
typedef struct {
    int32_t x, y, z;
    SpriteFlags flags;
    short picnum;
    int8_t shade;
    uint8_t  pal, clipdist, filler;
    uint8_t  xrepeat, yrepeat;
    int8_t xoffset, yoffset;
    short sectnum, statnum;
    short ang, owner, xvel, yvel, zvel;
    short lotag, hitag, extra;
} Sprite;

#pragma pack()

EXTERN SectorOnGRP sector_on_grp[MAXSECTORS];
EXTERN Sector sector[MAXSECTORS];
EXTERN walltype wall[MAXWALLS];
EXTERN Sprite sprite[MAXSPRITES];

EXTERN uint16_t mapCRC;

EXTERN Sprite tsprite[MAXSPRITESONSCREEN];

EXTERN int32_t xdim, ydim, numpages;

// Fast way to retrive the start of a column in the framebuffer, given a screenspace X coordinate.
EXTERN int32_t ylookup[MAXYDIM+1];

EXTERN int32_t yxaspect, viewingrange;

EXTERN int32_t validmodecnt;
EXTERN short validmode[256];
EXTERN int32_t validmodexdim[256], validmodeydim[256];

EXTERN short numsectors, numwalls;
EXTERN volatile int32_t totalclock;
EXTERN int32_t numframes, randomseed;
EXTERN uint8_t  palette[768];
EXTERN short numpalookups;
EXTERN uint8_t  *palookup[MAXPALOOKUPS];
EXTERN uint8_t  parallaxtype, showinvisibility;
EXTERN int32_t parallaxyoffs, parallaxyscale;
EXTERN int32_t visibility, parallaxvisibility;

EXTERN int32_t windowx1, windowy1, windowx2, windowy2;
EXTERN short startumost[MAXXDIM], startdmost[MAXXDIM];

EXTERN short pskyoff[MAXPSKYTILES], pskybits;

EXTERN short headspritesect[MAXSECTORS+1], headspritestat[MAXSTATUS+1];
EXTERN short prevspritesect[MAXSPRITES], prevspritestat[MAXSPRITES];
EXTERN short nextspritesect[MAXSPRITES], nextspritestat[MAXSPRITES];



//This is the bit vector that marks visited sector during portal flooding. Size is hence (MAXSECTORS / 8)
EXTERN uint8_t  visitedSectors[(MAXSECTORS+7)>>3];

/*************************************************************************
POSITION VARIABLES:

        POSX is your x - position ranging from 0 to 65535
        POSY is your y - position ranging from 0 to 65535
            (the length of a side of the grid in EDITBORD would be 1024)
        POSZ is your z - position (height) ranging from 0 to 65535, 0 highest.
        ANG is your angle ranging from 0 to 2047.  Instead of 360 degrees, or
             2 * PI radians, I use 2048 different angles, so 90 degrees would
             be 512 in my system.

SPRITE VARIABLES:

    EXTERN short headspritesect[MAXSECTORS+1], headspritestat[MAXSTATUS+1];
    EXTERN short prevspritesect[MAXSPRITES], prevspritestat[MAXSPRITES];
    EXTERN short nextspritesect[MAXSPRITES], nextspritestat[MAXSPRITES];

    Example: if the linked lists look like the following:
         ��������������������������������������������������������������Ŀ
         �      Sector lists:               Status lists:               �
         ��������������������������������������������������������������Ĵ
         �  Sector0:  4, 5, 8             Status0:  2, 0, 8             �
         �  Sector1:  16, 2, 0, 7         Status1:  4, 5, 16, 7, 3, 9   �
         �  Sector2:  3, 9                                              �
         ����������������������������������������������������������������
    Notice that each number listed above is shown exactly once on both the
        left and right side.  This is because any sprite that exists must
        be in some sector, and must have some kind of status that you define.


Coding example #1:
    To go through all the sprites in sector 1, the code can look like this:

        sectnum = 1;
        i = headspritesect[sectnum];
        while (i != -1)
        {
            nexti = nextspritesect[i];

            //your code goes here
            //ex: printf("Sprite %d is in sector %d\n",i,sectnum);

            i = nexti;
        }

Coding example #2:
    To go through all sprites with status = 1, the code can look like this:

        statnum = 1;        //status 1
        i = headspritestat[statnum];
        while (i != -1)
        {
            nexti = nextspritestat[i];

            //your code goes here
            //ex: printf("Sprite %d has a status of 1 (active)\n",i,statnum);

            i = nexti;
        }

             insertsprite(short sectnum, short statnum);
             deletesprite(short spritenum);
             changespritesect(short spritenum, short newsectnum);
             changespritestat(short spritenum, short newstatnum);

TILE VARIABLES:
        NUMTILES - the number of tiles found TILES.DAT.
        TILESIZX[MAXTILES] - simply the x-dimension of the tile number.
        TILESIZY[MAXTILES] - simply the y-dimension of the tile number.
        WALOFF[MAXTILES] - the actual 32-bit offset pointing to the top-left
                                 corner of the tile.
        PICANM[MAXTILES] - flags for animating the tile.

TIMING VARIABLES:
        TOTALCLOCK - When the engine is initialized, TOTALCLOCK is set to zero.
            From then on, it is incremented 120 times a second by 1.  That
            means that the number of seconds elapsed is totalclock / 120.
        NUMFRAMES - The number of times the draw3dscreen function was called
            since the engine was initialized.  This helps to determine frame
            rate.  (Frame rate = numframes * 120 / totalclock.)

OTHER VARIABLES:

        STARTUMOST[320] is an array of the highest y-coordinates on each column
                that my engine is allowed to write to.  You need to set it only
                once.
        STARTDMOST[320] is an array of the lowest y-coordinates on each column
                that my engine is allowed to write to.  You need to set it only
                once.
        NUMSECTORS - the total number of existing sectors.  Modified every time
            you call the loadboard function.
***************************************************************************/

#define PORTSIG  "Port by Ryan C. Gordon, Andrew Henderson, Dan Olson, Fabien Sanglard and a cast of thousands."

//Global.c
void Error (int errorType, char  *error, ...);
int FindDistance2D(int ix, int iy);

#define main SDL_main

#endif  /* defined _INCLUDE_BUILD_H_ */

/* end of build.h ... */


