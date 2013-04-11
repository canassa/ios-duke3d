/*
 * "Build Engine & Tools" Copyright (c) 1993-1997 Ken Silverman
 * Ken Silverman's official web site: "http://www.advsys.net/ken"
 * See the included license file "BUILDLIC.TXT" for license info.
 * This file has been modified from Ken Silverman's original release
 */

/* SUPERBUILD define is in engine.h ... */

#define ENGINE

#include <string.h>


#include <stdbool.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include "platform.h"

#if !PLATFORM_MACOSX
#include <malloc.h>
#endif

#include "build.h"

#include "engine.h"
#include "tiles.h"

//uint8_t  oa1, o3c2, ortca, ortcb, overtbits, laststereoint;

#include "display.h"

#define MAXCLIPNUM 512
#define MAXPERMS 512
#define MAXTILEFILES 256
#define MAXYSAVES ((MAXXDIM*MAXSPRITES)>>7)
#define MAXNODESPERLINE 42   /* Warning: This depends on MAXYSAVES & MAXYDIM! */
#define MAXWALLSB 2048
#define MAXCLIPDIST 1024

/* used to be static. --ryan. */
uint8_t  moustat = 0;

int32_t beforedrawrooms = 1;

/* used to be static. --ryan. */
int32_t oxdimen = -1, oviewingrange = -1, oxyaspect = -1;

/* used to be static. --ryan. */
int32_t curbrightness = 0;

int32_t ebpbak, espbak;
int32_t slopalookup[16384];

/*
 * !!! used to be static. If we ever put the original setgamemode() back, this
 * !!! can be made static again.  --ryan.
 */
uint8_t  permanentlock = 255;
int32_t  mapversion;

uint8_t tilefilenum[MAXTILES];
int32_t lastageclock;
int32_t tilefileoffs[MAXTILES];

static short radarang[1280], radarang2[MAXXDIM+1];
uint8_t  pow2char[8] = {1,2,4,8,16,32,64,-128};
int32_t pow2long[32] = {
    1L,2L,4L,8L,
    16L,32L,64L,128L,
    256L,512L,1024L,2048L,
    4096L,8192L,16384L,32768L,
    65536L,131072L,262144L,524288L,
    1048576L,2097152L,4194304L,8388608L,
    16777216L,33554432L,67108864L,134217728L,
    268435456L,536870912L,1073741824L,2147483647L,
};

uint8_t  britable[16][64];
uint8_t  textfont[1024], smalltextfont[1024];


GameMode game_mode;

enum vector_index_e {VEC_X=0,VEC_Y=1};
enum screenSpaceCoo_index_e {VEC_COL=0,VEC_DIST=1};
typedef int32_t vector_t[2];
typedef int32_t coo2D_t[2];
// This is the structure emitted for each wall that is potentially visible.
// A stack of those is populated when the sectors are scanned.
typedef struct pvWall_s {
    vector_t cameraSpaceCoo[2]; //Camera space coordinates of the wall endpoints. Access with vector_index_e.
    int16_t sectorId;        //The index of the sector this wall belongs to in the map database.
    int16_t worldWallId;     //The index of the wall in the map database.
    coo2D_t screenSpaceCoo[2]; //Screen space coordinate of the wall endpoints. Access with screenSpaceCoo_index_e.
} pvWall_t;

// Potentially Visible walls are stored in this stack.
pvWall_t pvWalls[MAXWALLSB];

// bunchWallsList contains the list of walls in a bunch.
static short bunchWallsList[MAXWALLSB];

static short bunchfirst[MAXWALLSB], bunchlast[MAXWALLSB];



static short smoststart[MAXWALLSB];
static uint8_t  smostwalltype[MAXWALLSB];
static int32_t smostwall[MAXWALLSB], smostwallcnt = -1L;

static Sprite *tspriteptr[MAXSPRITESONSCREEN];

//FCS: (up-most pixel on column x that can still be drawn to)
short umost[MAXXDIM+1];

//FCS: (down-most pixel +1 on column x that can still be drawn to)
short dmost[MAXXDIM+1];


short uplc[MAXXDIM+1], dplc[MAXXDIM+1];
static int16_t uwall[MAXXDIM+1], dwall[MAXXDIM+1];
static int32_t swplc[MAXXDIM+1], lplc[MAXXDIM+1];
static int32_t swall[MAXXDIM+1], lwall[MAXXDIM+4];
int32_t xdimen = -1, xdimenrecip, halfxdimen, xdimenscale, xdimscale;
int32_t wx1, wy1, wx2, wy2, ydimen;

// Updated by setview
int32_t viewoffset;

static int32_t rxi[8], ryi[8], rzi[8], rxi2[8], ryi2[8], rzi2[8];
static int32_t xsi[8], ysi[8];

/* used to be static. --ryan. */
int32_t *horizlookup=0, *horizlookup2=0, horizycent;

int32_t cosviewingrangeglobalang, sinviewingrangeglobalang;

int32_t xyaspect, viewingrangerecip;

uint8_t  *palookupoffse[4];


//FCS:
// Those two variables are using during portal flooding:
// sectorBorder is the stack and sectorbordercnt is the stack counter.
// There is no really point to have this on the heap. That would have been better on the stack.

//static short sectorborder[256], sectorbordercnt;
//FCS: Moved this on the stack

int32_t pageoffset, ydim16;
int32_t startposx, startposy, startposz;
int16_t startang, startsectnum;
int16_t pointhighlight, linehighlight, highlightcnt;
static int32_t lastx[MAXYDIM];
uint8_t  paletteloaded = 0;

#define FASTPALGRIDSIZ 8
static int32_t rdist[129], gdist[129], bdist[129];
static uint8_t  colhere[((FASTPALGRIDSIZ+2)*(FASTPALGRIDSIZ+2)*(FASTPALGRIDSIZ+2))>>3];
static uint8_t  colhead[(FASTPALGRIDSIZ+2)*(FASTPALGRIDSIZ+2)*(FASTPALGRIDSIZ+2)];
static int32_t colnext[256];
static uint8_t  coldist[8] = {0,1,2,3,4,3,2,1};
static int32_t colscan[27];

static int16_t clipnum, hitwalls[4];
int32_t hitscangoalx = (1<<29)-1, hitscangoaly = (1<<29)-1;

typedef struct {
    int32_t x1, y1, x2, y2;
} linetype;
static linetype clipit[MAXCLIPNUM];
static short clipsectorlist[MAXCLIPNUM], clipsectnum;
static short clipobjectval[MAXCLIPNUM];

typedef struct {
    int32_t sx, sy, z;
    short a, picnum;
    int8_t dashade;
    uint8_t  dapalnum, dastat, pagesleft;
    int32_t cx1, cy1, cx2, cy2;
} permfifotype;
static permfifotype permfifo[MAXPERMS];
static int32_t permhead = 0, permtail = 0;

short searchit;
int32_t searchx = -1, searchy;                     /* search input  */
short searchsector, searchwall, searchstat;     /* search output */

int32_t numtilefiles, artfil = -1, artfilnum, artfilplc;

static int32_t mirrorsx1, mirrorsy1, mirrorsx2, mirrorsy2;

int32_t totalclocklock;

uint16_t mapCRC;

#include "draw.h"

static __inline int32_t getclipmask(int32_t a, int32_t b, int32_t c, int32_t d)
{
    // Ken did this
    d = ((a<0)*8) + ((b<0)*4) + ((c<0)*2) + (d<0);
    return (((d<<4)^0xf0)|d);
}

uint16_t _swap16(uint16_t D)
{
    return ((D<<8)|(D>>8));
}

unsigned int _swap32(unsigned int D)
{
    return ((D<<24)|((D<<8)&0x00FF0000)|((D>>8)&0x0000FF00)|(D>>24));
}

/*
 FCS:
 Scan through sectors using portals (a portal is wall with a nextsector attribute >= 0).
 Flood is prevented if a portal does not face the POV.
 */
static void scansector (short sectnum, short *numscans, short *numbunches, EngineState *engine_state)
{
    walltype *wal, *wal2;
    Sprite *spr;
    int32_t xs, ys, x1, y1, x2, y2, xp1, yp1, xp2=0, yp2=0, tempint;
    short z, zz, startwall, endwall, numscansbefore, scanfirst, bunchfrst;
    short nextsectnum;

    //The stack storing sectors to visit.
    short sectorsToVisit[256], numSectorsToVisit;


    if (sectnum < 0) {
        return;
    }

    sectorsToVisit[0] = sectnum;
    numSectorsToVisit = 1;
    do {
        sectnum = sectorsToVisit[--numSectorsToVisit];

        //Add every script in the current sector as potentially visible.
        //TODO: this seens to be running even when there is no sprites in the map
        // I need to double check this
        for (z=headspritesect[sectnum]; z>=0; z=nextspritesect[z]) {
            spr = &sprite[z];
            if ((!spr->flags.invisible || showinvisibility) &&
                (spr->xrepeat > 0) && (spr->yrepeat > 0) &&
                (engine_state->spritesortcnt < MAXSPRITESONSCREEN)) {
                xs = spr->x - engine_state->posx;
                ys = spr->y - engine_state->posy;
                if ((spr->flags.type == WALL_SPRITE || spr->flags.type == FLOOR_SPRITE) ||
                    (xs * fixedPointCos(engine_state->ang) + ys * fixedPointSin(engine_state->ang) > 0)) {
                    copybufbyte(spr,&tsprite[engine_state->spritesortcnt],sizeof(Sprite));
                    tsprite[engine_state->spritesortcnt++].owner = z;
                }
            }
        }

        //Mark the current sector bit as "visited" in the bitvector
        //TODO: There is no need to save memory like this anymore, we can increase this
        // array size
        visitedSectors[sectnum>>3] |= pow2char[sectnum&7];

        bunchfrst = *numbunches;
        numscansbefore = *numscans;

        startwall = sector[sectnum].wallptr;
        endwall = startwall + sector[sectnum].wallnum;
        scanfirst = *numscans;
        for (z=startwall,wal=&wall[z]; z<endwall; z++,wal++) {
            nextsectnum = wal->nextsector;

            wal2 = &wall[wal->point2];

            // In camera space the center is the player.
            // Tranform the 2 Wall endpoints (x,y) from worldspace to camera space.
            // After that we have two vectors starting from the camera and going to the endpoints (x1,y1) and (x2,y2).
            x1 = wal->x - engine_state->posx;
            y1 = wal->y - engine_state->posy;

            x2 = wal2->x - engine_state->posx;
            y2 = wal2->y - engine_state->posy;

            // If this is a portal...
            if ((nextsectnum >= 0) && !wal->flags.one_way) {
                //If this portal has not been visited yet.
                if ((visitedSectors[nextsectnum>>3]&pow2char[nextsectnum&7]) == 0) {
                    //Cross product -> Z component
                    tempint = x1*y2-x2*y1;

                    // Using cross product, determine if the portal is facing us or not.
                    // If it is: Add it to the stack and bump the stack counter.
                    // This line is equivalent to tempint < 0x40000
                    if (((uint32_t)tempint+262144) < 524288) { // ??? What is this test ?? How acute the angle is ?
                        //(x2-x1)*(x2-x1)+(y2-y1)*(y2-y1) is the squared length of the wall
                        // ??? What is this test ?? How acute the angle is ?
                        if (mulscale5(tempint,tempint) <= (x2-x1)*(x2-x1)+(y2-y1)*(y2-y1)) {
                            sectorsToVisit[numSectorsToVisit++] = nextsectnum;
                        }
                    }
                }
            }

            // Rotate the wall endpoints vectors according to the player orientation.
            // This is a regular rotation matrix using [29.3] fixed point.
            if ((z == startwall) || (wall[z-1].point2 != z)) {
                //If this is the first endpoint of the bunch, rotate: This is a standard cos sin 2D rotation matrix projection
                xp1 = dmulscale6(y1, fixedPointCos(engine_state->ang), -x1, fixedPointSin(engine_state->ang));
                yp1 = dmulscale6(x1, cosviewingrangeglobalang, y1, sinviewingrangeglobalang);
            } else {
                //If this is NOT the first endpoint, Save the coordinate for next loop.
                xp1 = xp2;
                yp1 = yp2;
            }

            // Rotate: This is a standard cos sin 2D rotation matrix projection
            xp2 = dmulscale6(y2, fixedPointCos(engine_state->ang), -x2, fixedPointSin(engine_state->ang));
            yp2 = dmulscale6(x2, cosviewingrangeglobalang, y2, sinviewingrangeglobalang);

            // Equivalent of a near plane clipping ?
            if ((yp1 < 256) && (yp2 < 256)) {
                goto skipitaddwall;
            }

            // If wall's NOT facing you
            if (dmulscale32(xp1,yp2,-xp2,yp1) >= 0) {
                goto skipitaddwall;
            }

            // The wall is still not eligible for rendition: Let's do some more Frustrum culling !!
            if (xp1 >= -yp1) {

                if ((xp1 > yp1) || (yp1 == 0)) {
                    goto skipitaddwall;
                }

                //Project the point onto screen and see in which column it belongs.
                pvWalls[*numscans].screenSpaceCoo[0][VEC_COL] = halfxdimen + scale(xp1,halfxdimen,yp1);
                if (xp1 >= 0) {
                    pvWalls[*numscans].screenSpaceCoo[0][VEC_COL]++;    // Fix for SIGNED divide
                }

                if (pvWalls[*numscans].screenSpaceCoo[0][VEC_COL] >= xdimen) {
                    pvWalls[*numscans].screenSpaceCoo[0][VEC_COL] = xdimen-1;
                }

                pvWalls[*numscans].screenSpaceCoo[0][VEC_DIST] = yp1;
            } else {

                if (xp2 < -yp2) {
                    goto skipitaddwall;
                }

                pvWalls[*numscans].screenSpaceCoo[0][VEC_COL] = 0;
                tempint = yp1-yp2+xp1-xp2;

                if (tempint == 0) {
                    goto skipitaddwall;
                }

                pvWalls[*numscans].screenSpaceCoo[0][VEC_DIST] = yp1 + scale(yp2-yp1,xp1+yp1,tempint);
            }

            if (pvWalls[*numscans].screenSpaceCoo[0][VEC_DIST] < 256) {
                goto skipitaddwall;
            }

            if (xp2 <= yp2) {

                if ((xp2 < -yp2) || (yp2 == 0)) {
                    goto skipitaddwall;
                }
                pvWalls[*numscans].screenSpaceCoo[1][VEC_COL] = halfxdimen + scale(xp2,halfxdimen,yp2) - 1;
                if (xp2 >= 0) {
                    pvWalls[*numscans].screenSpaceCoo[1][VEC_COL]++;    /* Fix for SIGNED divide */
                }
                if (pvWalls[*numscans].screenSpaceCoo[1][VEC_COL] >= xdimen) {
                    pvWalls[*numscans].screenSpaceCoo[1][VEC_COL] = xdimen-1;
                }
                pvWalls[*numscans].screenSpaceCoo[1][VEC_DIST] = yp2;
            } else {

                if (xp1 > yp1) {
                    goto skipitaddwall;
                }
                pvWalls[*numscans].screenSpaceCoo[1][VEC_COL] = xdimen-1;
                tempint = xp2-xp1+yp1-yp2;
                if (tempint == 0) {
                    goto skipitaddwall;
                }
                pvWalls[*numscans].screenSpaceCoo[1][VEC_DIST] = yp1 + scale(yp2-yp1,yp1-xp1,tempint);
            }
            if ((pvWalls[*numscans].screenSpaceCoo[1][VEC_DIST] < 256) || (pvWalls[*numscans].screenSpaceCoo[0][VEC_COL] > pvWalls[*numscans].screenSpaceCoo[1][VEC_COL])) {
                goto skipitaddwall;
            }

            // Made it all the way!
            // Time to add this wall information to the stack of wall potentially visible.
            pvWalls[*numscans].sectorId = sectnum;
            pvWalls[*numscans].worldWallId = z;

            //Save the camera space wall endpoints coordinate (camera origin at player location + rotated according to player orientation).
            pvWalls[*numscans].cameraSpaceCoo[0][VEC_X] = xp1;
            pvWalls[*numscans].cameraSpaceCoo[0][VEC_Y] = yp1;
            pvWalls[*numscans].cameraSpaceCoo[1][VEC_X] = xp2;
            pvWalls[*numscans].cameraSpaceCoo[1][VEC_Y] = yp2;


            bunchWallsList[*numscans] = (*numscans)+1;
            (*numscans)++;

skipitaddwall:

            if ((wall[z].point2 < z) && (scanfirst < *numscans)) {
                bunchWallsList[(*numscans) - 1] = scanfirst;
                scanfirst = *numscans;
            }
        }

        //FCS: TODO rename this p2[] to bunchList[] or something like that. This name is an abomination
        //     DONE, p2 is now called "bunchWallsList".

        //Break down the list of walls for this sector into bunchs. Since a bunch is a
        // continuously visible list of wall: A sector can generate many bunches.
        for (z=numscansbefore; z < *numscans; z++) {
            if ((wall[pvWalls[z].worldWallId].point2 !=
                 pvWalls[bunchWallsList[z]].worldWallId) || (pvWalls[z].screenSpaceCoo[1][VEC_COL] >= pvWalls[bunchWallsList[z]].screenSpaceCoo[0][VEC_COL])) {
                // Create an entry in the bunch list
                bunchfirst[(*numbunches)++] = bunchWallsList[z];

                //Mark the end of the bunch wall list.
                bunchWallsList[z] = -1;
            }
        }

        //For each bunch, find the last wall and cache it in bunchlast.
        for (z=bunchfrst; z < *numbunches; z++) {
            for (zz=bunchfirst[z]; bunchWallsList[zz]>=0; zz=bunchWallsList[zz]);
            bunchlast[z] = zz;
        }

    } while (numSectorsToVisit > 0);
    // do this until the stack of sectors to visit if empty.
}

/*
 FCS:

 Goal : calculate texture coordinates and scales along a wall
 param 1: Z is the wallID in the list of potentially visible walls.
 param 2: Only used to lookup the xrepeat attribute of the wall.

*/
static void prepwall(int32_t wall_id, walltype *wal)
{
    int32_t i, l=0, ol=0, splc, sinc, x, topinc, top, botinc, bot, walxrepeat;
    vector_t *wallCoo = pvWalls[wall_id].cameraSpaceCoo;

    walxrepeat = wal->xrepeat << 3;

    // lwall calculation
    i = pvWalls[wall_id].screenSpaceCoo[0][VEC_COL]-halfxdimen;

    //Let's use some of the camera space wall coordinate now.
    topinc = -(wallCoo[0][VEC_Y]>>2);
    botinc = ((wallCoo[1][VEC_Y]-wallCoo[0][VEC_Y])>>8);

    top = mulscale5(wallCoo[0][VEC_X],xdimen)+mulscale2(topinc,i);
    bot = mulscale11(wallCoo[0][VEC_X]-wallCoo[1][VEC_X],xdimen)+mulscale2(botinc,i);

    splc = mulscale19(wallCoo[0][VEC_Y],xdimscale);
    sinc = mulscale16(wallCoo[1][VEC_Y]-wallCoo[0][VEC_Y],xdimscale);

    //X screenspce column of point Z.
    x = pvWalls[wall_id].screenSpaceCoo[0][VEC_COL];

    if (bot != 0) {
        l = divscale12(top,bot);
        swall[x] = mulscale21(l,sinc)+splc;
        l *= walxrepeat;
        lwall[x] = (l>>18);
    }

    //If the wall is less than 4 column wide.
    while (x+4 <= pvWalls[wall_id].screenSpaceCoo[1][VEC_COL]) {
        top += topinc;
        bot += botinc;
        if (bot != 0) {
            ol = l;
            l = divscale12(top,bot);
            swall[x+4] = mulscale21(l,sinc)+splc;
            l *= walxrepeat;
            lwall[x+4] = (l>>18);
        }
        i = ((ol+l)>>1);
        lwall[x+2] = (i>>18);
        lwall[x+1] = ((ol+i)>>19);
        lwall[x+3] = ((l+i)>>19);
        swall[x+2] = ((swall[x]+swall[x+4])>>1);
        swall[x+1] = ((swall[x]+swall[x+2])>>1);
        swall[x+3] = ((swall[x+4]+swall[x+2])>>1);
        x += 4;
    }

    //If the wall is less than 2 columns wide.
    if (x+2 <= pvWalls[wall_id].screenSpaceCoo[1][VEC_COL]) {
        top += (topinc>>1);
        bot += (botinc>>1);
        if (bot != 0) {
            ol = l;
            l = divscale12(top,bot);
            swall[x+2] = mulscale21(l,sinc)+splc;
            l *= walxrepeat;
            lwall[x+2] = (l>>18);
        }
        lwall[x+1] = ((l+ol)>>19);
        swall[x+1] = ((swall[x]+swall[x+2])>>1);
        x += 2;
    }

    //The wall is 1 column wide.
    if (x+1 <= pvWalls[wall_id].screenSpaceCoo[1][VEC_COL]) {
        bot += (botinc>>2);
        if (bot != 0) {
            l = divscale12(top+(topinc>>2),bot);
            swall[x+1] = mulscale21(l,sinc)+splc;
            lwall[x+1] = mulscale18(l,walxrepeat);
        }
    }

    if (lwall[pvWalls[wall_id].screenSpaceCoo[0][VEC_COL]] < 0) {
        lwall[pvWalls[wall_id].screenSpaceCoo[0][VEC_COL]] = 0;
    }

    if ((lwall[pvWalls[wall_id].screenSpaceCoo[1][VEC_COL]] >= walxrepeat) && (walxrepeat)) {
        lwall[pvWalls[wall_id].screenSpaceCoo[1][VEC_COL]] = walxrepeat-1;
    }

    if (wal->flags.x_flip) {
        walxrepeat--;
        for (x=pvWalls[wall_id].screenSpaceCoo[0][VEC_COL]; x<=pvWalls[wall_id].screenSpaceCoo[1][VEC_COL]; x++) {
            lwall[x] = walxrepeat-lwall[x];
        }
    }
}


static int32_t getpalookup(int32_t davis, int32_t dashade)
{
    return(min(max(dashade+(davis>>8),0),numpalookups-1));
}


static void hline (int32_t xr, int32_t yp,
                   int32_t xpanning, int32_t ypanning,
                   int32_t g_x1, int32_t g_y1,
                   int32_t g_x2, int32_t g_y2, int32_t shade, int32_t vis, uint8_t *pallete,
                   EngineState *engine_state)
{
    int32_t xl, r, s;

    xl = lastx[yp];

    if (xl > xr) {
        return;
    }

    r = horizlookup2[yp - engine_state->horiz + horizycent];
    s = (getpalookup(mulscale16(r,vis),shade)<<8);

    hlineasm4(xr-xl,s,
              g_x2 * r + ypanning,
              g_y1 * r + xpanning,
              ylookup[yp]+xr+frameoffset,
              g_x1 * r,
              g_y2 * r, pallete);
}

int32_t GetDistanceFromFloorOrCeiling (InnerSector floor_or_ceiling, int32_t z_position, bool is_floor) {
    return is_floor ? z_position - floor_or_ceiling.z : floor_or_ceiling.z - z_position;
}


// Renders non-parallaxed ceilings or floors
static void FloorCeilingScan (int32_t x1, int32_t x2, InnerSector floor_or_ceiling, bool is_floor, EngineState *engine_state)
{
    int8_t xshift, yshift;
    int32_t xpanning, ypanning;
    int32_t i, j, ox, oy, x, y1, y2, twall, bwall, zd;
    int32_t g_x1, g_y1, g_x2, g_y2;
    int16_t picnum;
    int32_t shade;
    int32_t vis;
    uint8_t *pallete = (uint8_t *)palookup[floor_or_ceiling.pal];

    
    zd = GetDistanceFromFloorOrCeiling(floor_or_ceiling, engine_state->posz, is_floor);
    
    //We are UNDER the floor: Do NOT render anything.
    if (zd > 0) {
        return;
    }
    
    //Retrive the floor texture.
    picnum = floor_or_ceiling.picnum;
    if ((uint32_t)picnum >= (uint32_t)MAXTILES) {
        picnum = 0;
    }
    
    //Lock the floor texture
    setgotpic(picnum);
    
    //This tile has unvalid dimensions (negative)
    if ((tiles[picnum].dim.width <= 0) ||
        (tiles[picnum].dim.height <= 0)) {
        return;
    }
    
    //If this is an animated texture: Animate it.
    //TODO: Not sure, this is executing for non-animated textures
    if (tiles[picnum].animFlags&192) {
        picnum += animateoffs(picnum);
    }
    
    //If the texture is not in RAM: Load it !!
    TILE_MakeAvailable(picnum);
    
    //Retrieve the shade of the sector (illumination level).
    shade = (int32_t)floor_or_ceiling.shade;
    
    vis = engine_state->cisibility;
    if (floor_or_ceiling.sector->visibility != 0) {
        vis = mulscale4(vis,(int32_t)((uint8_t )(floor_or_ceiling.sector->visibility+16)));
    }

    if (!floor_or_ceiling.flags.align_texture_to_first_wall) {
        g_x1 = g_x2 = fixedPointSin(engine_state->ang);
        g_y1 = g_y2 = fixedPointCos(engine_state->ang);
        xpanning = (engine_state->posx<<20);
        ypanning = -(engine_state->posy<<20);
    } else {
        j = floor_or_ceiling.sector->wallptr;
        ox = wall[wall[j].point2].x - wall[j].x;
        oy = wall[wall[j].point2].y - wall[j].y;
        i = fixedPointSqrt(ox*ox+oy*oy);
        
        if (i == 0) {
            i = 1024;
        } else {
            i = 1048576/i;
        }
        
        g_x1 = mulscale10(dmulscale10(ox, fixedPointSin(engine_state->ang), -oy, fixedPointCos(engine_state->ang)), i);
        g_y1 = mulscale10(dmulscale10(ox, fixedPointCos(engine_state->ang),  oy, fixedPointSin(engine_state->ang)), i);
        g_x2 = -g_x1;
        g_y2 = -g_y1;
        
        ox = ((wall[j].x - engine_state->posx)<<6);
        oy = ((wall[j].y - engine_state->posy)<<6);
        i = dmulscale14(oy, fixedPointCos(engine_state->ang), -ox, fixedPointSin(engine_state->ang));
        j = dmulscale14(ox, fixedPointCos(engine_state->ang),  oy, fixedPointSin(engine_state->ang));
        ox = i;
        oy = j;
        xpanning = g_x1*ox - g_y1*oy;
        ypanning = g_y2*ox + g_x2*oy;
    }
    
    g_x2 = mulscale16(g_x2,viewingrangerecip);
    g_y1 = mulscale16(g_y1,viewingrangerecip);
    xshift = 8 - tiles[picnum].dim_power_2.width;
    yshift = 8 - tiles[picnum].dim_power_2.height;
    
    if (floor_or_ceiling.flags.double_smooshiness) {
        xshift++;
        yshift++;
    }
    
    if (floor_or_ceiling.flags.swap_xy) {
        i = xpanning;
        xpanning = ypanning;
        ypanning = i;
        i = g_x2;
        g_x2 = -g_y1;
        g_y1 = -i;
        i = g_x1;
        g_x1 = g_y2;
        g_y2 = i;
    }
    
    if (floor_or_ceiling.flags.x_flip) {
        g_x1 = -g_x1;
        g_y1 = -g_y1;
        xpanning = -xpanning;
    }
    
    if (floor_or_ceiling.flags.y_flip) {
        g_x2 = -g_x2;
        g_y2 = -g_y2;
        ypanning = -ypanning;
    }
    
    g_x1 <<= xshift;
    g_y1 <<= xshift;
    g_x2 <<= yshift;
    g_y2 <<= yshift;
    xpanning <<= xshift;
    ypanning <<= yshift;
    xpanning += (((int32_t)floor_or_ceiling.xpanning) << 24);
    ypanning += (((int32_t)floor_or_ceiling.ypanning) << 24);
    g_y1 = (-g_x1-g_y1)*halfxdimen;
    g_x2 = (g_x2-g_y2)*halfxdimen;
    
    //Setup the drawing routine paramters
    sethlinesizes(tiles[picnum].dim_power_2.width, tiles[picnum].dim_power_2.height, tiles[picnum].data);
    
    g_x2 += g_y2*(x1-1);
    g_y1 += g_x1*(x1-1);
    g_x1 = mulscale16(g_x1, zd);
    g_x2 = mulscale16(g_x2, zd);
    g_y1 = mulscale16(g_y1, zd);
    g_y2 = mulscale16(g_y2, zd);
    vis = klabs(mulscale10(vis, zd));

    y1 = is_floor ? max(dplc[x1], umost[x1]) : umost[x1];
    y2 = y1;
    for (x=x1; x<=x2; x++) {
        twall = is_floor ? max(dplc[x], umost[x]) - 1 : umost[x]-1;
        bwall = is_floor ? dmost[x] : min(uplc[x],dmost[x]);
        if (twall < bwall-1) {
            if (twall >= y2) {
                while (y1 < y2-1) {
                    hline(x-1, ++y1, xpanning, ypanning, g_x1, g_y1, g_x2, g_y2, shade, vis, pallete, engine_state);
                }
                y1 = twall;
            } else {
                while (y1 < twall) {
                    hline(x-1, ++y1, xpanning, ypanning, g_x1, g_y1, g_x2, g_y2, shade, vis, pallete, engine_state);
                }
                while (y1 > twall) {
                    lastx[y1--] = x;
                }
            }
            while (y2 > bwall) {
                hline(x-1, --y2, xpanning, ypanning, g_x1, g_y1, g_x2, g_y2, shade, vis, pallete, engine_state);
            }
            while (y2 < bwall) {
                lastx[y2++] = x;
            }
        } else {
            while (y1 < y2-1) {
                hline(x-1,++y1, xpanning, ypanning, g_x1, g_y1, g_x2, g_y2, shade, vis, pallete, engine_state);
            }
            if (x == x2) {
                g_x2 += g_y2;
                g_y1 += g_x1;
                break;
            }
            y1 = is_floor ? max(dplc[x+1], umost[x+1]) : umost[x+1];
            y2 = y1;
        }
        g_x2 += g_y2;
        g_y1 += g_x1;
    }
    while (y1 < y2-1) {
        hline(x2, ++y1, xpanning, ypanning, g_x1, g_y1, g_x2, g_y2, shade, vis, pallete, engine_state);
    }
    
    faketimerhandler();
    return;
}


/* renders non-parallaxed ceilings. --ryan. */
static void CeilingScan (int32_t x1, int32_t x2, int32_t sectnum, EngineState *engine_state)
{
    FloorCeilingScan(x1, x2, sector[sectnum].ceiling, false, engine_state);
}


/* renders non-parallaxed floors. --ryan. */
static void FloorScan (int32_t x1, int32_t x2, int32_t sectnum, EngineState *engine_state)
{
    FloorCeilingScan(x1, x2, sector[sectnum].floor, true, engine_state);
}


/*
 * renders walls and parallaxed skies/floors. Look at parascan() for the
 *  higher level of parallaxing.
 *
 *    x1 == offset of leftmost pixel of wall. 0 is left of surface.
 *    x2 == offset of rightmost pixel of wall. 0 is left of surface.
 *
 *  apparently, walls are always vertical; there are sloping functions
 *   (!!!) er...elsewhere. Only the sides need be vertical, as the top and
 *   bottom of the polygon will need to be angled as the camera perspective
 *   shifts (user spins in a circle, etc.)
 *
 *  uwal is an array of the upper most pixels, and dwal are the lower most.
 *   This must be a list, as the top and bottom of the polygon are not
 *   necessarily horizontal lines.
 *
 *   So, the screen coordinate of the top left of a wall is specified by
 *   uwal[x1], the bottom left by dwal[x1], the top right by uwal[x2], and
 *   the bottom right by dwal[x2]. Every physical point on the edge of the
 *   wall in between is specified by traversing those arrays, one pixel per
 *   element.
 *
 *  --ryan.
 */
static void wallscan(int32_t x1, int32_t x2,
                     int16_t *uwal, int16_t *dwal,
                     int32_t *swal, int32_t *lwal,
                     int32_t zd,
                     int32_t xpanning,
                     int16_t picnum,
                     int32_t shade,
                     int16_t shiftval,
                     int32_t yscale, int32_t vis,
                     int32_t pallete,
                     EngineState *engine_state)
{
    int32_t bufplce[4], vplce[4], vince[4];
    int32_t x, xnice, ynice;
    int32_t y1ve[4], y2ve[4], tileWidth, tileHeight;
    uint8_t *fpalookup;

    tileWidth = tiles[picnum].dim.width;
    tileHeight = tiles[picnum].dim.height;

    setgotpic(picnum);

    if ((tileWidth <= 0) || (tileHeight <= 0)) return;
    if ((uwal[x1] > ydimen) && (uwal[x2] > ydimen)) return;
    if ((dwal[x1] < 0) && (dwal[x2] < 0)) return;

    TILE_MakeAvailable(picnum);

    xnice = (pow2long[tiles[picnum].dim_power_2.width] == tileWidth);
    if (xnice) {
        tileWidth--;
    }

    ynice = (pow2long[tiles[picnum].dim_power_2.height] == tileHeight);
    if (ynice) {
        tileHeight = tiles[picnum].dim_power_2.height;
    }

    fpalookup = palookup[pallete];

    SetupVerticalLine(shiftval);

    //Starting on the left column of the wall, check the occlusion arrays.
    x = x1;
    while ((umost[x] > dmost[x]) && (x <= x2)) {
        x++;
    }

    for (; x<=x2; x++) {
        y1ve[0] = max(uwal[x],umost[x]);
        y2ve[0] = min(dwal[x],dmost[x]);

        if (y2ve[0] <= y1ve[0]) continue;

        palookupoffse[0] = fpalookup+(getpalookup((int32_t)mulscale16(swal[x],vis),shade)<<8);

        bufplce[0] = lwal[x] + xpanning;
        
        if (bufplce[0] >= tileWidth) {
            if (xnice == 0) {
                bufplce[0] %= tileWidth;
            } else {
                bufplce[0] &= tileWidth;
            }
        }

        if (ynice == 0) {
            bufplce[0] *= tileHeight;
        } else {
            bufplce[0] <<= tileHeight;
        }

        vince[0] = swal[x]*yscale;
        vplce[0] = zd + vince[0] * (y1ve[0] - engine_state->horiz + 1);
        
        DrawVerticalLine(vince[0],palookupoffse[0],y2ve[0]-y1ve[0]-1,vplce[0],bufplce[0]+tiles[picnum].data,x+frameoffset+ylookup[y1ve[0]]);
    }
    faketimerhandler();
}


/* this renders masking sprites. See wallscan(). --ryan. */
static void maskwallscan(int32_t x1, int32_t x2,
                         short *uwal, short *dwal,
                         int32_t *swal, int32_t *lwal,
                         int32_t zd,
                         int32_t xpanning,
                         int16_t picnum,
                         int32_t shade,
                         int16_t shiftval,
                         int32_t yscale, int32_t vis,
                         int32_t pallete,
                         EngineState *engine_state)
{
    int32_t x, xnice, ynice;
    uint8_t *fpalookup;
    int32_t y1ve[4], y2ve[4], tileWidth, tileHeight;
    int32_t bufplce[4], vplce[4], vince[4];

    tileWidth = tiles[picnum].dim.width;
    tileHeight = tiles[picnum].dim.height;

    setgotpic(picnum);

    if ((tileWidth <= 0) || (tileHeight <= 0)) return;
    if ((uwal[x1] > ydimen) && (uwal[x2] > ydimen)) return;
    if ((dwal[x1] < 0) && (dwal[x2] < 0)) return;

    TILE_MakeAvailable(picnum);

    xnice = (pow2long[tiles[picnum].dim_power_2.width] == tileWidth);
    if (xnice) {
        tileWidth--;
    }

    ynice = (pow2long[tiles[picnum].dim_power_2.height] == tileHeight);
    if (ynice) {
        tileHeight = tiles[picnum].dim_power_2.height;
    }

    fpalookup = palookup[pallete];

    SetupVerticalLine(shiftval);

    //Starting on the left column of the wall, check the occlusion arrays.
    x = x1;
    while ((startumost[x+windowx1] > startdmost[x+windowx1]) && (x <= x2)) {
        x++;
    }

    for (; x<=x2; x++) {
        y1ve[0] = max(uwal[x],startumost[x+windowx1]-windowy1);
        y2ve[0] = min(dwal[x],startdmost[x+windowx1]-windowy1);

        if (y2ve[0] <= y1ve[0]) {
            continue;
        }

        palookupoffse[0] = fpalookup+(getpalookup((int32_t)mulscale16(swal[x],vis),shade)<<8);

        bufplce[0] = lwal[x] + xpanning;
        if (bufplce[0] >= tileWidth) {
            if (xnice == 0) {
                bufplce[0] %= tileWidth;
            } else {
                bufplce[0] &= tileWidth;
            }
        }

        if (ynice == 0) {
            bufplce[0] *= tileHeight;
        } else {
            bufplce[0] <<= tileHeight;
        }

        vince[0] = swal[x]*yscale;
        vplce[0] = zd + vince[0] * (y1ve[0] - engine_state->horiz + 1);
        
        DrawVerticalLine(vince[0],palookupoffse[0],y2ve[0]-y1ve[0]-1,vplce[0],bufplce[0]+tiles[picnum].data,x+frameoffset+ylookup[y1ve[0]]);
    }
    faketimerhandler();
}


/* renders parallaxed skies/floors  --ryan. */
static void parascan(bool is_floor, int32_t bunch, EngineState *engine_state)
{
    Sector *sec;
    InnerSector floor_or_ceiling, next_sector;
    int32_t a, k, l, m, n, x, z, wallnum, nextsectnum, horizbak, zd;
    int32_t xpanning, ypanning;
    short *topptr, *botptr;
    int16_t picnum;
    int32_t shade;
    int16_t shiftval;
    int32_t yscale;
    int32_t vis;
    int32_t pallete;

    sec = &sector[pvWalls[bunchfirst[bunch]].sectorId];
    
    floor_or_ceiling = is_floor ? sec->floor : sec->ceiling;

    horizbak = engine_state->horiz;
    if (parallaxyscale != 65536) {
        engine_state->horiz = mulscale16(engine_state->horiz-(ydimen>>1),parallaxyscale) + (ydimen>>1);
    }
    vis = engine_state->pisibility;
    /* globalorientation = 0L; */
    if (sec->visibility != 0) {
        vis = mulscale4(vis,(int32_t)((uint8_t )(sec->visibility+16)));
    }

    pallete = floor_or_ceiling.pal;
    picnum = floor_or_ceiling.picnum;
    shade = (int32_t)floor_or_ceiling.shade;
    xpanning = (int32_t)floor_or_ceiling.xpanning;
    ypanning = (int32_t)floor_or_ceiling.ypanning;
    topptr = is_floor ? dplc : umost;
    botptr = is_floor ? dmost : uplc;

    if ((uint32_t)picnum >= (uint32_t)MAXTILES) {
        picnum = 0;
    }

    if (tiles[picnum].animFlags&192) {
        picnum += animateoffs(picnum);
    }

    shiftval = tiles[picnum].dim_power_2.height;

    if (pow2long[shiftval] != tiles[picnum].dim.height) {
        shiftval++;
    }
    shiftval = 32-shiftval;
    zd = (((tiles[picnum].dim.height >> 1) + parallaxyoffs) << shiftval) + (ypanning << 24);
    yscale = (8<<(shiftval-19));

    k = 11 - tiles[picnum].dim_power_2.width - pskybits;
    x = -1;

    for (z=bunchfirst[bunch]; z>=0; z=bunchWallsList[z]) {
        wallnum = pvWalls[z].worldWallId;
        nextsectnum = wall[wallnum].nextsector;
        
        next_sector = is_floor ? sector[nextsectnum].floor : sector[nextsectnum].ceiling;

        if ((nextsectnum < 0) || (wall[wallnum].flags.one_way) || !next_sector.flags.parallaxing) {
            if (x == -1) {
                x = pvWalls[z].screenSpaceCoo[0][VEC_COL];
            }

            if (parallaxtype == 0) {
                n = mulscale16(xdimenrecip,viewingrange);
                for (a=pvWalls[z].screenSpaceCoo[0][VEC_COL]; a<=pvWalls[z].screenSpaceCoo[1][VEC_COL]; a++) {
                    lplc[a] = (((mulscale23(a-halfxdimen, n) + engine_state->ang) & 2047)>>k);
                }
            } else {
                for (a=pvWalls[z].screenSpaceCoo[0][VEC_COL]; a<=pvWalls[z].screenSpaceCoo[1][VEC_COL]; a++) {
                    lplc[a] = ((((int32_t)radarang2[a] + engine_state->ang) & 2047)>>k);
                }
            }
            
            if (parallaxtype == 2) {
                n = mulscale16(xdimscale,viewingrange);
                for (a=pvWalls[z].screenSpaceCoo[0][VEC_COL]; a<=pvWalls[z].screenSpaceCoo[1][VEC_COL]; a++) {
                    swplc[a] = mulscale14(fixedPointSin(((int32_t)radarang2[a]+512)),n);
                }
            } else {
                clearbuf(&swplc[pvWalls[z].screenSpaceCoo[0][VEC_COL]],pvWalls[z].screenSpaceCoo[1][VEC_COL]-pvWalls[z].screenSpaceCoo[0][VEC_COL]+1,mulscale16(xdimscale,viewingrange));
            }
        } else if (x >= 0) {
            l = picnum;
            m = tiles[picnum].dim_power_2.width;
            picnum = l+pskyoff[lplc[x]>>m];

            if (((lplc[x]^lplc[pvWalls[z].screenSpaceCoo[0][VEC_COL]-1])>>m) == 0) {
                wallscan(x,
                         pvWalls[z].screenSpaceCoo[0][VEC_COL] - 1,
                         topptr, botptr, swplc, lplc,
                         zd,
                         xpanning, picnum, shade, shiftval, yscale, vis, pallete,
                         engine_state);
            } else {
                a = x;
                while (x < pvWalls[z].screenSpaceCoo[0][VEC_COL]) {
                    n = l+pskyoff[lplc[x]>>m];
                    if (n != picnum) {
                        wallscan(a, x-1, topptr ,botptr, swplc, lplc, zd,
                                 xpanning, picnum, shade, shiftval, yscale, vis, pallete,
                                 engine_state);
                        a = x;
                        picnum = n;
                    }
                    x++;
                }
                if (a < x) {
                    wallscan(a, x-1, topptr, botptr, swplc, lplc, zd,
                             xpanning, picnum, shade, shiftval,  yscale, vis, pallete,
                             engine_state);
                }
            }

            picnum = l;
            x = -1;
        }
    }

    if (x >= 0) {
        l = picnum;
        m = tiles[picnum].dim_power_2.width;
        picnum = l+pskyoff[lplc[x]>>m];

        if (((lplc[x]^lplc[pvWalls[bunchlast[bunch]].screenSpaceCoo[1][VEC_COL]])>>m) == 0) {
            wallscan(x,
                     pvWalls[bunchlast[bunch]].screenSpaceCoo[1][VEC_COL],
                     topptr, botptr, swplc, lplc,
                     zd,
                     xpanning, picnum, shade, shiftval, yscale, vis, pallete,
                     engine_state);
        } else {
            a = x;
            while (x <= pvWalls[bunchlast[bunch]].screenSpaceCoo[1][VEC_COL]) {
                n = l+pskyoff[lplc[x]>>m];
                if (n != picnum) {
                    wallscan(a, x-1,
                             topptr, botptr, swplc, lplc, zd,
                             xpanning, picnum, shade, shiftval, yscale, vis, pallete,
                             engine_state);
                    a = x;
                    picnum = n;
                }
                x++;
            }
            if (a <= x) {
                wallscan(a, x, topptr, botptr, swplc, lplc, zd,
                         xpanning, picnum, shade, shiftval, yscale, vis, pallete,
                         engine_state);
            }
        }
        picnum = l;
    }
    engine_state->horiz = horizbak;
}


#define BITSOFPRECISION 3  /* Don't forget to change this in A.ASM also! */
static void grouscan (int32_t dax1, int32_t dax2, InnerSector surface, bool is_floor, EngineState *engine_state)
{
    int32_t i, j, l, x, y, dx, dy, wx, wy, y1, y2, zd;
    int32_t dasqr;
    int32_t shoffs, shinc, m1, m2, *mptr1, *mptr2, *nptr1, *nptr2;
    int32_t g_x1, g_y1, g_x2, g_y2, g_x3, g_y3, g_zx;
    int32_t lx, ly, lz;
    walltype *wal;

    if (is_floor) {
        if (engine_state->posz >= GetZOfSlope(surface, engine_state->posx, engine_state->posy)) return;
    } else {
        if (engine_state->posz <= GetZOfSlope(surface, engine_state->posx, engine_state->posy)) return;
    }
    
    if ((tiles[surface.picnum].animFlags&192) != 0) {
        surface.picnum += animateoffs(surface.picnum);
    }

    setgotpic(surface.picnum);

    if ((tiles[surface.picnum].dim.width <= 0) ||
        (tiles[surface.picnum].dim.height <= 0)) {
        return;
    }

    TILE_MakeAvailable(surface.picnum);

    wal = &wall[surface.sector->wallptr];
    wx = wall[wal->point2].x - wal->x;
    wy = wall[wal->point2].y - wal->y;
    dasqr = krecip(fixedPointSqrt(wx*wx+wy*wy));
    i = mulscale21(surface.heinum,dasqr);
    wx *= i;
    wy *= i;

    lx = -mulscale19(fixedPointSin(engine_state->ang), xdimenrecip);
    ly = mulscale19(fixedPointCos(engine_state->ang), xdimenrecip);
    g_x1 = (engine_state->posx << 8);
    g_y1 = -(engine_state->posy << 8);
    i = (dax1-halfxdimen)*xdimenrecip;
    g_x2 = mulscale16(fixedPointCos(engine_state->ang)<<4,viewingrangerecip) - mulscale27(fixedPointSin(engine_state->ang),i);
    g_y2 = mulscale16(fixedPointSin(engine_state->ang)<<4,viewingrangerecip) + mulscale27(fixedPointCos(engine_state->ang),i);
    zd = (xdimscale<<9);
    g_zx = -dmulscale17(wx, g_y2, -wy, g_x2) + mulscale10(1 - engine_state->horiz, zd);
    lz = -dmulscale25(wx, ly, -wy, lx);

    if (surface.flags.align_texture_to_first_wall) { /* Relative alignment */
        dx = mulscale14(wall[wal->point2].x-wal->x,dasqr);
        dy = mulscale14(wall[wal->point2].y-wal->y,dasqr);

        i = fixedPointSqrt(surface.heinum*surface.heinum+16777216);

        x = lx;
        y = ly;
        lx = dmulscale16(x,dx,y,dy);
        ly = mulscale12(dmulscale16(-y,dx,x,dy),i);

        x = ((wal->x - engine_state->posx)<<8);
        y = ((wal->y - engine_state->posy)<<8);
        g_x1 = dmulscale16(-x,dx,-y,dy);
        g_y1 = mulscale12(dmulscale16(-y,dx,x,dy),i);

        x = g_x2;
        y = g_y2;
        g_x2 = dmulscale16(x,dx,y,dy);
        g_y2 = mulscale12(dmulscale16(-y,dx,x,dy),i);
    }
    
    if (surface.flags.swap_xy) {
        i = lx;
        lx = -ly;
        ly = -i;
        i = g_x1;
        g_x1 = g_y1;
        g_y1 = i;
        i = g_x2;
        g_x2 = -g_y2;
        g_y2 = -i;
    }
    
    if (surface.flags.x_flip) {
        g_x1 = -g_x1;
        g_x2 = -g_x2;
        lx = -lx;
    }
    
    if (surface.flags.y_flip) {
        g_y1 = -g_y1;
        g_y2 = -g_y2;
        ly = -ly;
    }

    surface.z = dmulscale9(wx, engine_state->posy - wal->y, -wy, engine_state->posx - wal->x) + ((surface.z - engine_state->posz)<<8);
    g_x2 = mulscale20(g_x2,surface.z);
    lx = mulscale28(lx, surface.z);
    g_y2 = mulscale20(g_y2,-surface.z);
    ly = mulscale28(ly, -surface.z);

    i = 8 - tiles[surface.picnum].dim_power_2.width;
    j = 8 - tiles[surface.picnum].dim_power_2.height;
    if (surface.flags.double_smooshiness) {
        i++;
        j++;
    }
    g_x1 <<= (i+12);
    g_x2 <<= i;
    lx <<= i;
    g_y1 <<= (j+12);
    g_y2 <<= j;
    ly <<= j;

    if (is_floor == 0) {
        g_x1 += (((int32_t)surface.xpanning)<<24);
        g_y1 += (((int32_t)surface.ypanning)<<24);
    } else {
        g_x1 += (((int32_t)surface.xpanning)<<24);
        g_y1 += (((int32_t)surface.ypanning)<<24);
    }

    int32_t visibility = engine_state->visibility;
    if (surface.sector->visibility != 0) {
        visibility = mulscale4(visibility, (int32_t)((uint8_t )(surface.sector->visibility + 16)));
    }
    visibility = mulscale13(visibility, surface.z);
    visibility = mulscale16(visibility, xdimscale);
    
    j =(int32_t) FP_OFF(palookup[surface.pal]);
    l = (zd >> 16);

    shinc = mulscale16(lx, xdimenscale);
    if (shinc > 0) {
        shoffs = (4<<15);
    } else {
        shoffs = ((16380-ydimen)<<15);    // JBF: was 2044     16380
    }
    
    if (is_floor) {
        y1 = max(umost[dax1],dplc[dax1]);
    } else {
        y1 = umost[dax1];
    }
    
    m1 = mulscale16(y1, zd) + (g_zx>>6);
    // Avoid visibility overflow by crossing horizon
    if (zd > 0) {
        m1 += (zd >> 16);
    } else {
        m1 -= (zd >> 16);
    }
    m2 = m1+l;
    mptr1 = (int32_t *)&slopalookup[y1+(shoffs>>15)];
    mptr2 = mptr1+1;

    for (x=dax1; x<=dax2; x++) {
        if (is_floor == 0) {
            y1 = umost[x];
            y2 = min(dmost[x],uplc[x])-1;
        } else {
            y1 = max(umost[x],dplc[x]);
            y2 = dmost[x]-1;
        }
        if (y1 <= y2) {
            nptr1 = (int32_t *)&slopalookup[y1+(shoffs>>15)];
            nptr2 = (int32_t *)&slopalookup[y2+(shoffs>>15)];
            while (nptr1 <= mptr1) {
                *mptr1-- = j + (getpalookup((int32_t)mulscale24(krecip(m1), visibility), surface.shade) << 8);
                m1 -= l;
            }
            while (nptr2 >= mptr2) {
                *mptr2++ = j + (getpalookup((int32_t)mulscale24(krecip(m2), visibility), surface.shade) << 8);
                m2 += l;
            }

            g_x3 = (g_x2>>10);
            g_y3 = (g_y2>>10);
            slopevlin(&frameoffset[ylookup[y2] + x],
                      nptr2,
                      y2 - y1 + 1,
                      g_x1, g_y1,
                      mulscale16(y2, zd) + (g_zx >> 6), // asm3
                      g_x3, g_y3,
                      -(zd >> (16-BITSOFPRECISION)), //asm1
                      game_mode.bytesperline,
                      &tiles[surface.picnum]);

            if ((x&15) == 0) {
                faketimerhandler();
            }
        }
        g_x2 += lx;
        g_y2 += ly;
        g_zx += lz;
        shoffs += shinc;
    }
}


static int owallmost(short *mostbuf, int32_t w, int32_t z, EngineState *engine_state)
{
    int32_t bad, inty, xcross, y, yinc;
    int32_t s1, s2, s3, s4, ix1, ix2, iy1, iy2, t;

    z <<= 7;
    s1 = mulscale20(engine_state->uclip,pvWalls[w].screenSpaceCoo[0][VEC_DIST]);
    s2 = mulscale20(engine_state->uclip,pvWalls[w].screenSpaceCoo[1][VEC_DIST]);
    s3 = mulscale20(engine_state->dclip,pvWalls[w].screenSpaceCoo[0][VEC_DIST]);
    s4 = mulscale20(engine_state->dclip,pvWalls[w].screenSpaceCoo[1][VEC_DIST]);
    bad = (z<s1)+((z<s2)<<1)+((z>s3)<<2)+((z>s4)<<3);

    ix1 = pvWalls[w].screenSpaceCoo[0][VEC_COL];
    iy1 = pvWalls[w].screenSpaceCoo[0][VEC_DIST];
    ix2 = pvWalls[w].screenSpaceCoo[1][VEC_COL];
    iy2 = pvWalls[w].screenSpaceCoo[1][VEC_DIST];

    if ((bad&3) == 3) {
        clearbufbyte(&mostbuf[ix1],(ix2-ix1+1)*sizeof(mostbuf[0]),0L);
        return(bad);
    }

    if ((bad&12) == 12) {
        clearbufbyte(&mostbuf[ix1],(ix2-ix1+1)*sizeof(mostbuf[0]),ydimen+(ydimen<<16));
        return(bad);
    }

    if (bad&3) {
        t = divscale30(z-s1,s2-s1);
        inty = pvWalls[w].screenSpaceCoo[0][VEC_DIST] + mulscale30(pvWalls[w].screenSpaceCoo[1][VEC_DIST]-pvWalls[w].screenSpaceCoo[0][VEC_DIST],t);
        xcross = pvWalls[w].screenSpaceCoo[0][VEC_COL] + scale(mulscale30(pvWalls[w].screenSpaceCoo[1][VEC_DIST],t),pvWalls[w].screenSpaceCoo[1][VEC_COL]-pvWalls[w].screenSpaceCoo[0][VEC_COL],inty);

        if ((bad&3) == 2) {
            if (pvWalls[w].screenSpaceCoo[0][VEC_COL] <= xcross) {
                iy2 = inty;
                ix2 = xcross;
            }
            clearbufbyte(&mostbuf[xcross+1],(pvWalls[w].screenSpaceCoo[1][VEC_COL]-xcross)*sizeof(mostbuf[0]),0L);
        } else {
            if (xcross <= pvWalls[w].screenSpaceCoo[1][VEC_COL]) {
                iy1 = inty;
                ix1 = xcross;
            }
            clearbufbyte(&mostbuf[pvWalls[w].screenSpaceCoo[0][VEC_COL]],(xcross-pvWalls[w].screenSpaceCoo[0][VEC_COL]+1)*sizeof(mostbuf[0]),0L);
        }
    }

    if (bad&12) {
        t = divscale30(z-s3,s4-s3);
        inty = pvWalls[w].screenSpaceCoo[0][VEC_DIST] + mulscale30(pvWalls[w].screenSpaceCoo[1][VEC_DIST]-pvWalls[w].screenSpaceCoo[0][VEC_DIST],t);
        xcross = pvWalls[w].screenSpaceCoo[0][VEC_COL] + scale(mulscale30(pvWalls[w].screenSpaceCoo[1][VEC_DIST],t),pvWalls[w].screenSpaceCoo[1][VEC_COL]-pvWalls[w].screenSpaceCoo[0][VEC_COL],inty);

        if ((bad&12) == 8) {
            if (pvWalls[w].screenSpaceCoo[0][VEC_COL] <= xcross) {
                iy2 = inty;
                ix2 = xcross;
            }
            clearbufbyte(&mostbuf[xcross+1],(pvWalls[w].screenSpaceCoo[1][VEC_COL]-xcross)*sizeof(mostbuf[0]),ydimen+(ydimen<<16));
        } else {
            if (xcross <= pvWalls[w].screenSpaceCoo[1][VEC_COL]) {
                iy1 = inty;
                ix1 = xcross;
            }
            clearbufbyte(&mostbuf[pvWalls[w].screenSpaceCoo[0][VEC_COL]],(xcross-pvWalls[w].screenSpaceCoo[0][VEC_COL]+1)*sizeof(mostbuf[0]),ydimen+(ydimen<<16));
        }
    }

    y = (scale(z,xdimenscale,iy1)<<4);
    yinc = ((scale(z,xdimenscale,iy2)<<4)-y) / (ix2-ix1+1);
    qinterpolatedown16short((int32_t *)&mostbuf[ix1],ix2-ix1+1,y+(engine_state->horiz<<16),yinc);

    if (mostbuf[ix1] < 0) {
        mostbuf[ix1] = 0;
    }
    if (mostbuf[ix1] > ydimen) {
        mostbuf[ix1] = ydimen;
    }
    if (mostbuf[ix2] < 0) {
        mostbuf[ix2] = 0;
    }
    if (mostbuf[ix2] > ydimen) {
        mostbuf[ix2] = ydimen;
    }

    return(bad);
}


// calculate top and bottom edges of walls
static int wallmost(short *mostbuf, int32_t w, InnerSector floor_or_ceiling, EngineState *engine_state)
{
    int32_t bad, i, j, t, y, z, inty, intz, xcross, yinc, fw;
    int32_t x1, y1, z1, x2, y2, z2, xv, yv, dx, dy, dasqr, oz1, oz2;
    int32_t s1, s2, s3, s4, ix1, ix2, iy1, iy2;

    z = floor_or_ceiling.z - engine_state->posz;
    if (!floor_or_ceiling.flags.groudraw) {
        return(owallmost(mostbuf, w, z, engine_state));
    }

    i = pvWalls[w].worldWallId;
    if (i == floor_or_ceiling.sector->wallptr) {
        return(owallmost(mostbuf, w, z, engine_state));
    }

    x1 = wall[i].x;
    x2 = wall[wall[i].point2].x-x1;
    y1 = wall[i].y;
    y2 = wall[wall[i].point2].y-y1;

    fw = floor_or_ceiling.sector->wallptr;
    i = wall[fw].point2;
    dx = wall[i].x-wall[fw].x;
    dy = wall[i].y-wall[fw].y;
    dasqr = krecip(fixedPointSqrt(dx*dx+dy*dy));

    if (pvWalls[w].screenSpaceCoo[0][VEC_COL] == 0) {
        xv = fixedPointCos(engine_state->ang) + sinviewingrangeglobalang;
        yv = fixedPointSin(engine_state->ang) - cosviewingrangeglobalang;
    } else {
        xv = x1 - engine_state->posx;
        yv = y1 - engine_state->posy;
    }
    i = xv*(y1 - engine_state->posy) - yv*(x1 - engine_state->posx);
    j = yv*x2 - xv*y2;

    if (klabs(j) > klabs(i>>3)) {
        i = divscale28(i,j);
    }

    t = mulscale15(floor_or_ceiling.heinum,dasqr);
    z1 = floor_or_ceiling.z;

    z1 = dmulscale24(dx*t, mulscale20(y2,i)+((y1-wall[fw].y)<<8), -dy*t,mulscale20(x2, i)+((x1-wall[fw].x)<<8))+((z1-engine_state->posz)<<7);


    if (pvWalls[w].screenSpaceCoo[1][VEC_COL] == xdimen-1) {
        xv = fixedPointCos(engine_state->ang) - sinviewingrangeglobalang;
        yv = fixedPointSin(engine_state->ang) + cosviewingrangeglobalang;
    } else {
        xv = (x2+x1) - engine_state->posx;
        yv = (y2+y1) - engine_state->posy;
    }

    i = xv*(y1 - engine_state->posy) - yv*(x1-engine_state->posx);
    j = yv*x2-xv*y2;

    if (klabs(j) > klabs(i>>3)) {
        i = divscale28(i,j);
    }

    t = mulscale15(floor_or_ceiling.heinum,dasqr);
    z2 = floor_or_ceiling.z;

    z2 = dmulscale24(dx*t,mulscale20(y2,i)+((y1-wall[fw].y)<<8),-dy*t,mulscale20(x2,i)+((x1-wall[fw].x)<<8))+((z2-engine_state->posz)<<7);


    s1 = mulscale20(engine_state->uclip,pvWalls[w].screenSpaceCoo[0][VEC_DIST]);
    s2 = mulscale20(engine_state->uclip,pvWalls[w].screenSpaceCoo[1][VEC_DIST]);
    s3 = mulscale20(engine_state->dclip,pvWalls[w].screenSpaceCoo[0][VEC_DIST]);
    s4 = mulscale20(engine_state->dclip,pvWalls[w].screenSpaceCoo[1][VEC_DIST]);
    bad = (z1<s1)+((z2<s2)<<1)+((z1>s3)<<2)+((z2>s4)<<3);

    ix1 = pvWalls[w].screenSpaceCoo[0][VEC_COL];
    ix2 = pvWalls[w].screenSpaceCoo[1][VEC_COL];
    iy1 = pvWalls[w].screenSpaceCoo[0][VEC_DIST];
    iy2 = pvWalls[w].screenSpaceCoo[1][VEC_DIST];
    oz1 = z1;
    oz2 = z2;

    if ((bad&3) == 3) {
        clearbufbyte(&mostbuf[ix1],(ix2-ix1+1)*sizeof(mostbuf[0]),0L);
        return(bad);
    }

    if ((bad&12) == 12) {
        clearbufbyte(&mostbuf[ix1],(ix2-ix1+1)*sizeof(mostbuf[0]),ydimen+(ydimen<<16));
        return(bad);
    }

    if (bad&3) {
        /* inty = intz / (globaluclip>>16) */
        t = divscale30(oz1-s1,s2-s1+oz1-oz2);
        inty = pvWalls[w].screenSpaceCoo[0][VEC_DIST] + mulscale30(pvWalls[w].screenSpaceCoo[1][VEC_DIST]-pvWalls[w].screenSpaceCoo[0][VEC_DIST],t);
        intz = oz1 + mulscale30(oz2-oz1,t);
        xcross = pvWalls[w].screenSpaceCoo[0][VEC_COL] + scale(mulscale30(pvWalls[w].screenSpaceCoo[1][VEC_DIST],t),pvWalls[w].screenSpaceCoo[1][VEC_COL]-pvWalls[w].screenSpaceCoo[0][VEC_COL],inty);

        if ((bad&3) == 2) {
            if (pvWalls[w].screenSpaceCoo[0][VEC_COL] <= xcross) {
                z2 = intz;
                iy2 = inty;
                ix2 = xcross;
            }
            clearbufbyte(&mostbuf[xcross+1],(pvWalls[w].screenSpaceCoo[1][VEC_COL]-xcross)*sizeof(mostbuf[0]),0L);
        } else {
            if (xcross <= pvWalls[w].screenSpaceCoo[1][VEC_COL]) {
                z1 = intz;
                iy1 = inty;
                ix1 = xcross;
            }
            clearbufbyte(&mostbuf[pvWalls[w].screenSpaceCoo[0][VEC_COL]],(xcross-pvWalls[w].screenSpaceCoo[0][VEC_COL]+1)*sizeof(mostbuf[0]),0L);
        }
    }

    if (bad&12) {
        /* inty = intz / (globaldclip>>16) */
        t = divscale30(oz1-s3,s4-s3+oz1-oz2);
        inty = pvWalls[w].screenSpaceCoo[0][VEC_DIST] + mulscale30(pvWalls[w].screenSpaceCoo[1][VEC_DIST]-pvWalls[w].screenSpaceCoo[0][VEC_DIST],t);
        intz = oz1 + mulscale30(oz2-oz1,t);
        xcross = pvWalls[w].screenSpaceCoo[0][VEC_COL] + scale(mulscale30(pvWalls[w].screenSpaceCoo[1][VEC_DIST],t),pvWalls[w].screenSpaceCoo[1][VEC_COL]-pvWalls[w].screenSpaceCoo[0][VEC_COL],inty);

        if ((bad&12) == 8) {
            if (pvWalls[w].screenSpaceCoo[0][VEC_COL] <= xcross) {
                z2 = intz;
                iy2 = inty;
                ix2 = xcross;
            }
            clearbufbyte(&mostbuf[xcross+1],(pvWalls[w].screenSpaceCoo[1][VEC_COL]-xcross)*sizeof(mostbuf[0]),ydimen+(ydimen<<16));
        } else {
            if (xcross <= pvWalls[w].screenSpaceCoo[1][VEC_COL]) {
                z1 = intz;
                iy1 = inty;
                ix1 = xcross;
            }
            clearbufbyte(&mostbuf[pvWalls[w].screenSpaceCoo[0][VEC_COL]],(xcross-pvWalls[w].screenSpaceCoo[0][VEC_COL]+1)*sizeof(mostbuf[0]),ydimen+(ydimen<<16));
        }
    }

    y = (scale(z1,xdimenscale,iy1)<<4);
    yinc = ((scale(z2,xdimenscale,iy2)<<4)-y) / (ix2-ix1+1);
    qinterpolatedown16short((int32_t *)&mostbuf[ix1],ix2-ix1+1,y+(engine_state->horiz<<16),yinc);

    if (mostbuf[ix1] < 0) {
        mostbuf[ix1] = 0;
    }
    if (mostbuf[ix1] > ydimen) {
        mostbuf[ix1] = ydimen;
    }
    if (mostbuf[ix2] < 0) {
        mostbuf[ix2] = 0;
    }
    if (mostbuf[ix2] > ydimen) {
        mostbuf[ix2] = ydimen;
    }

    return(bad);
}


static void drawalls(int32_t bunch, short *numscans, short *numhits, short *numbunches, EngineState *engine_state)
{
    Sector *sec, *nextsec;
    walltype *wal;
    int32_t i, x, x1, x2, cz[5], fz[5];
    int32_t z, wallnum, sectnum, nextsectnum;
    int32_t startsmostwallcnt, startsmostcnt, gotswall;
    int32_t zd;
    int32_t xpanning, ypanning;
    uint8_t  andwstat1, andwstat2;
    int16_t picnum;
    int32_t shade;
    int16_t shiftval;
    int32_t yscale;
    int32_t vis;
    
    z = bunchfirst[bunch];
    sectnum = pvWalls[z].sectorId;
    sec = &sector[sectnum];

    andwstat1 = 0xff;
    andwstat2 = 0xff;
    for (; z>=0; z=bunchWallsList[z]) { /* uplc/dplc calculation */
        andwstat1 &= wallmost(uplc, z, sec->ceiling, engine_state);
        andwstat2 &= wallmost(dplc, z, sec->floor, engine_state);
    }

    /* draw ceilings */
    if ((andwstat1&3) != 3) {
        if (sec->ceiling.flags.groudraw && !sec->ceiling.flags.parallaxing) {
            grouscan(pvWalls[bunchfirst[bunch]].screenSpaceCoo[0][VEC_COL],
                     pvWalls[bunchlast[bunch]].screenSpaceCoo[1][VEC_COL],
                     sector[sectnum].ceiling, false, engine_state);
        } else if (!sec->ceiling.flags.parallaxing) {
            CeilingScan(pvWalls[bunchfirst[bunch]].screenSpaceCoo[0][VEC_COL],
                        pvWalls[bunchlast[bunch]].screenSpaceCoo[1][VEC_COL],
                        sectnum,
                        engine_state);
        } else {
            parascan(false, bunch, engine_state);
        }
    }

    /* draw floors */
    if ((andwstat2&12) != 12) {
        if (sec->floor.flags.groudraw && !sec->floor.flags.parallaxing) {
            grouscan(pvWalls[bunchfirst[bunch]].screenSpaceCoo[0][VEC_COL],
                     pvWalls[bunchlast[bunch]].screenSpaceCoo[1][VEC_COL],
                     sector[sectnum].floor, true, engine_state);
        } else if (!sec->floor.flags.parallaxing) {
            FloorScan(pvWalls[bunchfirst[bunch]].screenSpaceCoo[0][VEC_COL],
                      pvWalls[bunchlast[bunch]].screenSpaceCoo[1][VEC_COL],
                      sectnum,
                      engine_state);
        } else {
            parascan(true, bunch, engine_state);
        }
    }

    /* DRAW WALLS SECTION! */
    for (z=bunchfirst[bunch]; z>=0; z=bunchWallsList[z]) {

        x1 = pvWalls[z].screenSpaceCoo[0][VEC_COL];
        x2 = pvWalls[z].screenSpaceCoo[1][VEC_COL];
        if (umost[x2] >= dmost[x2]) {

            for (x=x1; x<x2; x++)
                if (umost[x] < dmost[x]) {
                    break;
                }

            if (x >= x2) {
                smostwall[smostwallcnt] = z;
                smostwalltype[smostwallcnt] = 0;
                smostwallcnt++;
                continue;
            }
        }

        wallnum = pvWalls[z].worldWallId;
        wal = &wall[wallnum];
        nextsectnum = wal->nextsector;
        nextsec = &sector[nextsectnum];

        gotswall = 0;

        startsmostwallcnt = smostwallcnt;
        startsmostcnt = engine_state->smostcnt;

        if ((searchit == 2) && (searchx >= x1) && (searchx <= x2)) {
            if (searchy <= uplc[searchx]) { /* ceiling */
                searchsector = sectnum;
                searchwall = wallnum;
                searchstat = 1;
                searchit = 1;
            } else if (searchy >= dplc[searchx]) { /* floor */
                searchsector = sectnum;
                searchwall = wallnum;
                searchstat = 2;
                searchit = 1;
            }
        }

        if (nextsectnum >= 0) {
            getzsofslope((short)sectnum,wal->x,wal->y,&cz[0],&fz[0]);
            getzsofslope((short)sectnum,wall[wal->point2].x,wall[wal->point2].y,&cz[1],&fz[1]);
            getzsofslope((short)nextsectnum,wal->x,wal->y,&cz[2],&fz[2]);
            getzsofslope((short)nextsectnum,wall[wal->point2].x,wall[wal->point2].y,&cz[3],&fz[3]);
            getzsofslope((short)nextsectnum,engine_state->posx,engine_state->posy,&cz[4],&fz[4]);

            if (wal->flags.masking && !wal->flags.one_way) {
                engine_state->maskwall[engine_state->maskwallcnt++] = z;
            }

            if (!sec->ceiling.flags.parallaxing || !nextsec->ceiling.flags.parallaxing) {
                if ((cz[2] <= cz[0]) && (cz[3] <= cz[1])) {
                    if (engine_state->ceiling_clip)
                        for (x=x1; x<=x2; x++)
                            if (uplc[x] > umost[x])
                                if (umost[x] <= dmost[x]) {
                                    umost[x] = uplc[x];
                                    if (umost[x] > dmost[x]) {
                                        (*numhits)--;
                                    }
                                }
                } else {
                    wallmost(dwall, z, sector[nextsectnum].ceiling, engine_state);
                    if ((cz[2] > fz[0]) || (cz[3] > fz[1]))
                        for (i=x1; i<=x2; i++) if (dwall[i] > dplc[i]) {
                                dwall[i] = dplc[i];
                            }

                    if ((searchit == 2) && (searchx >= x1) && (searchx <= x2))
                        if (searchy <= dwall[searchx]) { /* wall */
                            searchsector = sectnum;
                            searchwall = wallnum;
                            searchstat = 0;
                            searchit = 1;
                        }

                    picnum = wal->picnum;
                    if ((uint32_t)picnum >= (uint32_t)MAXTILES) {
                        picnum = 0;
                    }
                    xpanning = (int32_t)wal->xpanning;
                    ypanning = (int32_t)wal->ypanning;
                    shiftval = tiles[picnum].dim_power_2.height;
                    if (pow2long[shiftval] != tiles[picnum].dim.height) {
                        shiftval++;
                    }
                    shiftval = 32-shiftval;

                    //Animated
                    if (tiles[picnum].animFlags&192) {
                        picnum += animateoffs(picnum);
                    }

                    shade = (int32_t)wal->shade;
                    vis = engine_state->visibility;
                    if (sec->visibility != 0) {
                        vis = mulscale4(vis,(int32_t)((uint8_t )(sec->visibility+16)));
                    }

                    yscale = (wal->yrepeat<<(shiftval-19));
                    if (!wal->flags.align_bottom) {
                        zd = (((engine_state->posz-nextsec->ceiling.z)*yscale)<<8);
                    } else {
                        zd = (((engine_state->posz-sec->ceiling.z)*yscale)<<8);
                    }
                    zd += (ypanning<<24);
                    if (wal->flags.transluscence_reversing) {
                        yscale = -yscale;
                        zd = -zd;
                    }

                    if (gotswall == 0) {
                        gotswall = 1;
                        prepwall(z,wal);
                    }
                    wallscan(x1, x2, uplc, dwall, swall, lwall, zd,
                             xpanning, picnum, shade, shiftval, yscale, vis, wal->pal,
                             engine_state);

                    if ((cz[2] >= cz[0]) && (cz[3] >= cz[1])) {
                        for (x=x1; x<=x2; x++)
                            if (dwall[x] > umost[x])
                                if (umost[x] <= dmost[x]) {
                                    umost[x] = dwall[x];
                                    if (umost[x] > dmost[x]) {
                                        (*numhits)--;
                                    }
                                }
                    } else {
                        for (x=x1; x<=x2; x++)
                            if (umost[x] <= dmost[x]) {
                                i = max(uplc[x],dwall[x]);
                                if (i > umost[x]) {
                                    umost[x] = i;
                                    if (umost[x] > dmost[x]) {
                                        (*numhits)--;
                                    }
                                }
                            }
                    }
                }
                if ((cz[2] < cz[0]) || (cz[3] < cz[1]) || (engine_state->posz < cz[4])) {
                    i = x2-x1+1;
                    if (engine_state->smostcnt+i < MAXYSAVES) {
                        smoststart[smostwallcnt] = engine_state->smostcnt;
                        smostwall[smostwallcnt] = z;
                        smostwalltype[smostwallcnt] = 1;   /* 1 for umost */
                        smostwallcnt++;
                        copybufbyte((int32_t *)&umost[x1],
                                    (int32_t *)&(engine_state->smost[engine_state->smostcnt]),
                                    i*sizeof(engine_state->smost[0]));
                        engine_state->smostcnt += i;
                    }
                }
            }
            if (!sec->floor.flags.parallaxing || !nextsec->floor.flags.parallaxing) {
                if ((fz[2] >= fz[0]) && (fz[3] >= fz[1])) {
                    if (engine_state->floor_clip)
                        for (x=x1; x<=x2; x++)
                            if (dplc[x] < dmost[x])
                                if (umost[x] <= dmost[x]) {
                                    dmost[x] = dplc[x];
                                    if (umost[x] > dmost[x]) {
                                        (*numhits)--;
                                    }
                                }
                } else {
                    wallmost(uwall, z, sector[nextsectnum].floor, engine_state);
                    if ((fz[2] < cz[0]) || (fz[3] < cz[1]))
                        for (i=x1; i<=x2; i++) if (uwall[i] < uplc[i]) {
                                uwall[i] = uplc[i];
                            }

                    if ((searchit == 2) && (searchx >= x1) && (searchx <= x2))
                        if (searchy >= uwall[searchx]) { /* wall */
                            searchsector = sectnum;
                            searchwall = wallnum;
                            if (wal->flags.bottom_texture_swap) {
                                searchwall = wal->nextwall;
                            }
                            searchstat = 0;
                            searchit = 1;
                        }

                    if (wal->flags.bottom_texture_swap) {
                        wallnum = wal->nextwall;
                        wal = &wall[wallnum];
                        picnum = wal->picnum;
                        if ((uint32_t)picnum >= (uint32_t)MAXTILES) {
                            picnum = 0;
                        }
                        xpanning = (int32_t)wal->xpanning;
                        ypanning = (int32_t)wal->ypanning;

                        if (tiles[picnum].animFlags&192) {
                            picnum += animateoffs(picnum);
                        }

                        shade = (int32_t)wal->shade;
                        wallnum = pvWalls[z].worldWallId;
                        wal = &wall[wallnum];
                    } else {
                        picnum = wal->picnum;

                        if ((uint32_t)picnum >= (uint32_t)MAXTILES) {
                            picnum = 0;
                        }

                        xpanning = (int32_t)wal->xpanning;
                        ypanning = (int32_t)wal->ypanning;

                        if (tiles[picnum].animFlags&192) {
                            picnum += animateoffs(picnum);
                        }
                        shade = (int32_t)wal->shade;
                    }
                    vis = engine_state->visibility;
                    if (sec->visibility != 0) {
                        vis = mulscale4(vis,(int32_t)((uint8_t )(sec->visibility+16)));
                    }
                    shiftval = tiles[picnum].dim_power_2.height;

                    if (pow2long[shiftval] != tiles[picnum].dim.height) {
                        shiftval++;
                    }

                    shiftval = 32-shiftval;
                    yscale = (wal->yrepeat<<(shiftval-19));

                    if (!wal->flags.align_bottom) {
                        zd = (((engine_state->posz-nextsec->floor.z)*yscale)<<8);
                    } else {
                        zd = (((engine_state->posz-sec->ceiling.z)*yscale)<<8);
                    }

                    zd += (ypanning<<24);
                    if (wal->flags.transluscence_reversing) {
                        yscale = -yscale;
                        zd = -zd;
                    }

                    if (gotswall == 0) {
                        gotswall = 1;
                        prepwall(z,wal);
                    }
                    wallscan(x1, x2, uwall, dplc, swall, lwall, zd,
                             xpanning, picnum, shade, shiftval, yscale, vis, wal->pal,
                             engine_state);

                    if ((fz[2] <= fz[0]) && (fz[3] <= fz[1])) {
                        for (x=x1; x<=x2; x++)
                            if (uwall[x] < dmost[x])
                                if (umost[x] <= dmost[x]) {
                                    dmost[x] = uwall[x];
                                    if (umost[x] > dmost[x]) {
                                        (*numhits)--;
                                    }
                                }
                    } else {
                        for (x=x1; x<=x2; x++)
                            if (umost[x] <= dmost[x]) {
                                i = min(dplc[x],uwall[x]);
                                if (i < dmost[x]) {
                                    dmost[x] = i;
                                    if (umost[x] > dmost[x]) {
                                        (*numhits)--;
                                    }
                                }
                            }
                    }
                }
                if ((fz[2] > fz[0]) || (fz[3] > fz[1]) || (engine_state->posz > fz[4])) {
                    i = x2-x1+1;
                    if (engine_state->smostcnt+i < MAXYSAVES) {
                        smoststart[smostwallcnt] = engine_state->smostcnt;
                        smostwall[smostwallcnt] = z;
                        smostwalltype[smostwallcnt] = 2;   /* 2 for dmost */
                        smostwallcnt++;
                        copybufbyte((int32_t *)&dmost[x1],
                                    (int32_t *)&(engine_state->smost[engine_state->smostcnt]),
                                    i*sizeof(engine_state->smost[0]));
                        engine_state->smostcnt += i;
                    }
                }
            }
            if (*numhits < 0) {
                return;
            }
            if (!wal->flags.one_way && ((visitedSectors[nextsectnum>>3]&pow2char[nextsectnum&7]) == 0)) {
                if (umost[x2] < dmost[x2]) {
                    scansector((short) nextsectnum, numscans, numbunches, engine_state);
                } else {
                    for (x=x1; x<x2; x++)
                        if (umost[x] < dmost[x]) {
                            scansector((short) nextsectnum, numscans, numbunches, engine_state);
                            break;
                        }

                    /*
                     * If can't see sector beyond, then cancel smost array and just
                     *  store wall!
                     */
                    if (x == x2) {
                        smostwallcnt = startsmostwallcnt;
                        engine_state->smostcnt = startsmostcnt;
                        smostwall[smostwallcnt] = z;
                        smostwalltype[smostwallcnt] = 0;
                        smostwallcnt++;
                    }
                }
            }
        }
        if ((nextsectnum < 0) || wal->flags.one_way) { /* White/1-way wall */
            if (nextsectnum < 0) {
                picnum = wal->picnum;
            } else {
                picnum = wal->overpicnum;
            }

            if ((uint32_t)picnum >= (uint32_t)MAXTILES) {
                picnum = 0;
            }

            xpanning = (int32_t)wal->xpanning;
            ypanning = (int32_t)wal->ypanning;

            if (tiles[picnum].animFlags&192) {
                picnum += animateoffs(picnum);
            }

            shade = (int32_t)wal->shade;
            vis = engine_state->visibility;
            if (sec->visibility != 0) {
                vis = mulscale4(vis,(int32_t)((uint8_t )(sec->visibility+16)));
            }

            shiftval = tiles[picnum].dim_power_2.height;
            if (pow2long[shiftval] != tiles[picnum].dim.height) {
                shiftval++;
            }

            shiftval = 32-shiftval;
            yscale = (wal->yrepeat<<(shiftval-19));
            if (nextsectnum >= 0) {
                if (!wal->flags.align_bottom) {
                    zd = engine_state->posz - nextsec->ceiling.z;
                } else {
                    zd = engine_state->posz - sec->ceiling.z;
                }
            } else {
                if (!wal->flags.align_bottom) {
                    zd = engine_state->posz - sec->ceiling.z;
                } else {
                    zd = engine_state->posz - sec->floor.z;
                }
            }
            zd = ((zd * yscale) << 8) + (ypanning << 24);

            if (wal->flags.transluscence_reversing) {
                yscale = -yscale;
                zd = -zd;
            }

            if (gotswall == 0) {
                gotswall = 1;
                prepwall(z,wal);
            }

            wallscan(x1, x2, uplc, dplc, swall, lwall, zd,
                     xpanning, picnum, shade, shiftval, yscale, vis, wal->pal,
                     engine_state);

            for (x=x1; x<=x2; x++)
                if (umost[x] <= dmost[x]) {
                    umost[x] = 1;
                    dmost[x] = 0;
                    (*numhits)--;
                }
            smostwall[smostwallcnt] = z;
            smostwalltype[smostwallcnt] = 0;
            smostwallcnt++;

            if ((searchit == 2) && (searchx >= x1) && (searchx <= x2)) {
                searchit = 1;
                searchsector = sectnum;
                searchwall = wallnum;
                if (nextsectnum < 0) {
                    searchstat = 0;
                } else {
                    searchstat = 4;
                }
            }
        }
    }
}


static void dosetaspect(void)
{
    int32_t i, j, k, x, xinc;

    if (xyaspect != oxyaspect) {
        oxyaspect = xyaspect;
        j = xyaspect*320;
        horizlookup2[horizycent-1] = divscale26(131072,j);
        for (i=ydim*4-1; i>=0; i--)
            if (i != (horizycent-1)) {
                horizlookup[i] = divscale28(1,i-(horizycent-1));
                horizlookup2[i] = divscale14(klabs(horizlookup[i]),j);
            }
    }


    if ((xdimen != oxdimen) || (viewingrange != oviewingrange)) {
        oxdimen = xdimen;
        oviewingrange = viewingrange;
        xinc = mulscale32(viewingrange*320,xdimenrecip);
        x = (640<<16)-mulscale1(xinc,xdimen);
        for (i=0; i<xdimen; i++) {
            j = (x&65535);
            k = (x>>16);
            x += xinc;
            if (j != 0) {
                j = mulscale16((int32_t)radarang[k+1]-(int32_t)radarang[k],j);
            }
            radarang2[i] = (short)(((int32_t)radarang[k]+j)>>6);
        }
    }
}


/*
  FCS: Geez one more horrible algorithm to decipher :| :/ :( cry smiley.....
  Algorithm:

  1.
  Take wall 1 vector [point1,point2] and using two cross products determine if the two endpoints of wall 2 are on the same side of Wall 1 plan.
  If they are then we can determine according to globalposx and globalposy if  wall2 is before or after wall1's plan.

  2. Do the same thing again but this time with wall2's plan. Try to find if wall1 is in front of behind wall2's plan.

  Key concept: If a cross-product is equal to 0 this mean they are parallel.

  Return: pvWallID1 in the potentially visible wall list is in front of pvWallID2 (in the same potentially visible list)
*/
int wallfront(int32_t pvWallID1, int32_t pvWallID2, EngineState *engine_state)
{
    walltype *wal;
    int32_t x11, y11, x21, y21, x12, y12, x22, y22, dx, dy, t1, t2;

    //It seems we are going to work in Worldspace coordinates.
    wal = &wall[pvWalls[pvWallID1].worldWallId];
    x11 = wal->x;
    y11 = wal->y;
    wal = &wall[wal->point2];
    x21 = wal->x;
    y21 = wal->y;
    wal = &wall[pvWalls[pvWallID2].worldWallId];
    x12 = wal->x;
    y12 = wal->y;
    wal = &wall[wal->point2];
    x22 = wal->x;
    y22 = wal->y;


    //This is part 1

    //Wall 1's vector
    dx = x21-x11;
    dy = y21-y11;

    //This is a cross-product between Wall 1 vector and the [Wall 1 Point 1-> Wall 2 Point 1] vector
    t1 = dmulscale2(x12-x11,dy,-dx,y12-y11); /* p1(l2) vs. l1 */
    //This is a cross-product between Wall 1 vector and the [Wall 1 Point 1-> Wall 2 Point 2] vector
    t2 = dmulscale2(x22-x11,dy,-dx,y22-y11); /* p2(l2) vs. l1 */

    //If the vectors a parallel, then the cross-product is zero.
    if (t1 == 0) {
        //wall2's point1 is on wall1's plan.
        t1 = t2;
        if (t1 == 0) { // Those two walls are on the same plan.
            //Wall 2's point 2 is on wall1's plan.
            return(-1);
        }
    }
    if (t2 == 0) {
        t2 = t1;
    }


    //This XOR just determine if the cross-product have the same sign and hence if both points are on the same side of wall 1 plan.
    //Test if both points of wall2 are on the same side of wall 1 (in front or behind).
    if ((t1^t2) >= 0) {
        //cross-product have the same sign: Both points of wall2 are on the same side of wall1 : An answer is possible !!

        //Now is time to take into account the camera position and determine which of wall1 or wall2 is seen first.
        t2 = dmulscale2(engine_state->posx - x11,dy,-dx,engine_state->posy-y11); /* pos vs. l1 */

        //Test the cross product sign difference.
        //If (t2^t1) >= 0 then  both cross product had different sign so wall1 is in front of wall2
        //otherwise wall2 is in front of wall1
        return((t2^t1) >= 0);
    }


    //This is part 2
    //Do it again but this time will wall2's plan.

    //Wall 2's vector
    dx = x22-x12;
    dy = y22-y12;

    t1 = dmulscale2(x11-x12,dy,-dx,y11-y12); /* p1(l1) vs. l2 */
    t2 = dmulscale2(x21-x12,dy,-dx,y21-y12); /* p2(l1) vs. l2 */
    if (t1 == 0) {
        t1 = t2;
        if (t1 == 0) {
            return(-1);
        }
    }
    if (t2 == 0) {
        t2 = t1;
    }
    if ((t1^t2) >= 0) {
        t2 = dmulscale2(engine_state->posx - x12,dy,-dx,engine_state->posy-y12); /* pos vs. l2 */
        return((t2^t1) < 0);
    }

    //FCS: No wall is in front of the other's plan: This means they are crossing.
    return(-2);
}


//Return 1 if bunch firstBunchID is in from of bunch secondBunchID.
static int bunchfront(int32_t firstBunchID, int32_t secondBunchID, EngineState *engine_state)
{
    int32_t x1b1, x2b1, x1b2, x2b2;


    x1b1 = pvWalls[bunchfirst[firstBunchID]].screenSpaceCoo[0][VEC_COL];
    x2b2 = pvWalls[bunchlast[secondBunchID]].screenSpaceCoo[1][VEC_COL]+1;
    if (x1b1 >= x2b2) {
        //Bunch 1 left side is completely on the right of bunch2's right in screenspace: They do not overlap.
        return(-1);
    }


    x1b2 = pvWalls[bunchfirst[secondBunchID]].screenSpaceCoo[0][VEC_COL];
    x2b1 = pvWalls[bunchlast[firstBunchID]].screenSpaceCoo[1][VEC_COL]+1;
    if (x1b2 >= x2b1) {
        //Bunch 2 left side is completely on the right of bunch 1 right side: They do not overlap.
        return(-1);
    }


    if (x1b1 >= x1b2) {
        //Get the last wall in the bunch2.
        int lastWallID;
        for (lastWallID=bunchfirst[secondBunchID];
             pvWalls[lastWallID].screenSpaceCoo[1][VEC_COL]<x1b1;
             lastWallID=bunchWallsList[lastWallID]);

        return(wallfront(bunchfirst[firstBunchID],lastWallID,engine_state));
    } else {
        //Get the last wall in the bunch.
        int lastWallID;
        for (lastWallID=bunchfirst[firstBunchID];
             pvWalls[lastWallID].screenSpaceCoo[1][VEC_COL]<x1b2;
             lastWallID=bunchWallsList[lastWallID]);

        return(wallfront(lastWallID,bunchfirst[secondBunchID],engine_state));
    }
}



//#include "keyboard.h"
//void WriteLastPaletteToFile(void);
//void WriteTranslucToFile(void);
/*
      FCS: Draw every walls in Front to Back Order.
*/
EngineState *drawrooms(int32_t daposx, int32_t daposy, int32_t daposz,
                       short daang, int32_t dahoriz, short currentSectorNumber, bool draw_mirror)
{
    int32_t i, j, closest;
    //Ceiling and Floor height at the player position.
    int32_t cz, fz;
    short *shortptr1, *shortptr2;
    static int pixelRenderable = 0;

    //FCS: Num walls to potentially render.
    short numscans;

    //FCS: Number of colums to draw. ALWAYS set to the screen dimension width.
    short numhits;

    short numbunches;

    char  buffer[MAXWALLS];

    static EngineState engine_state;

    // When visualizing the rendering process, part of the screen
    // are not updated: In order to avoid the "ghost effect", we
    // clear the framebuffer to black.
    if (CLEAR_FRAMEBUFFER) {
        clear2dscreen();
    }


    //CODE EXPLORATION
    /*
    if( KB_KeyDown[0x39]){ // 0x39 = SPACE
        //CODE EXPLORATION
        WriteLastPaletteToFile();
        WriteTranslucToFile();
    }
    */

    pixelRenderable+=100;
    if (pixelRenderable >= MAX_PIXEL_RENDERERED) {
        pixelRenderable =  0 ;
    }

    //pixelsAllowed = pixelRenderable;
    pixelsAllowed = 100000000;
    //printf("%d\n",pixelsAllowed);

    beforedrawrooms = 0;

    // FCS: What was the point of having those values as parameters of this function....if it is to overwrite the
    // values with the gloval variables ?!?!?
    engine_state.posx = daposx;
    engine_state.posy = daposy;
    engine_state.posz = daposz;
    engine_state.ang = (daang&2047); //FCS: Mask and keep only 11 bits of angle value.

    engine_state.horiz = mulscale16(dahoriz-100,xdimenscale)+(ydimen>>1);
    engine_state.uclip = (0 - engine_state.horiz) * xdimscale;
    engine_state.dclip = (ydimen - engine_state.horiz) * xdimscale;

    i = mulscale16(xdimenscale,viewingrangerecip);
    engine_state.pisibility = mulscale16(parallaxvisibility, i);
    engine_state.visibility = mulscale16(visibility, i);
    engine_state.hisibility = mulscale16(engine_state.visibility, xyaspect);
    engine_state.cisibility = mulscale8(engine_state.hisibility, 320);

    totalclocklock = totalclock;

    cosviewingrangeglobalang = mulscale16(fixedPointCos(engine_state.ang), viewingrange);
    sinviewingrangeglobalang = mulscale16(fixedPointSin(engine_state.ang), viewingrange);

    if ((xyaspect != oxyaspect) || (xdimen != oxdimen) || (viewingrange != oviewingrange)) {
        dosetaspect();
    }

    frameoffset = frameplace+viewoffset;

    //Clear the bit vector that keep track of what sector has been flooded in.
    clearbufbyte(visitedSectors,(int32_t)((numsectors+7)>>3),0L);

    //Clear the occlusion array.
    shortptr1 = (short *)&startumost[windowx1];
    shortptr2 = (short *)&startdmost[windowx1];

    for (i=0; i<xdimen; i++) {
        umost[i] = shortptr1[i]-windowy1;
        dmost[i] = shortptr2[i]-windowy1;
    }

    //NumHits is the number of column to draw.
    numhits = xdimen;
    //Num walls to potentially render.
    numscans = 0;

    numbunches = 0;
    engine_state.maskwallcnt = 0;
    smostwallcnt = 0;
    engine_state.smostcnt = 0;
    engine_state.spritesortcnt = 0;

    if (currentSectorNumber >= MAXSECTORS) {
        currentSectorNumber -= MAXSECTORS;
    } else {
        // Even if the player leaves the map, the engine will keep on rendering from the last visited sector.
        // Save it.
        i = currentSectorNumber;
        updatesector(engine_state.posx, engine_state.posy, &currentSectorNumber);
        //Seem the player has left the map since updatesector cannot locate him -> Restore to the last known sector.
        if (currentSectorNumber < 0) {
            currentSectorNumber = i;
        }
    }

    engine_state.ceiling_clip = 1;
    engine_state.floor_clip = 1;

    //Update the ceiling and floor Z coordinate for the player's 2D position.
    getzsofslope(currentSectorNumber, engine_state.posx, engine_state.posy, &cz, &fz);

    if (engine_state.posz < cz) {
        engine_state.ceiling_clip = 0;
    }
    if (engine_state.posz > fz) {
        engine_state.floor_clip = 0;
    }

    //Build the list of potentially visible wall in to "bunches".
    scansector(currentSectorNumber, &numscans, &numbunches, &engine_state);

    // Are we drawing a mirror?
    if (draw_mirror) {
        mirrorsx1 = xdimen-1;
        mirrorsx2 = 0;
        for (i=numscans-1; i>=0; i--) {
            if (wall[pvWalls[i].worldWallId].nextsector < 0) {
                continue;
            }
            if (pvWalls[i].screenSpaceCoo[0][VEC_COL] < mirrorsx1) {
                mirrorsx1 = pvWalls[i].screenSpaceCoo[0][VEC_COL];
            }
            if (pvWalls[i].screenSpaceCoo[1][VEC_COL] > mirrorsx2) {
                mirrorsx2 = pvWalls[i].screenSpaceCoo[1][VEC_COL];
            }
        }

        for (i=0; i<mirrorsx1; i++)
            if (umost[i] <= dmost[i]) {
                umost[i] = 1;
                dmost[i] = 0;
                numhits--;
            }
        for (i=mirrorsx2+1; i<xdimen; i++)
            if (umost[i] <= dmost[i]) {
                umost[i] = 1;
                dmost[i] = 0;
                numhits--;
            }

        drawalls(0L, &numscans, &numhits, &numbunches, &engine_state);
        numbunches--;
        bunchfirst[0] = bunchfirst[numbunches];
        bunchlast[0] = bunchlast[numbunches];

        mirrorsy1 = min(umost[mirrorsx1],umost[mirrorsx2]);
        mirrorsy2 = max(dmost[mirrorsx1],dmost[mirrorsx2]);
    }

    // scansector has generated the bunches, it is now time to see which ones to render.
    // numhits is the number of column of pixels to draw: (if the screen is 320x200 then numhits starts at 200).
    // Due to rounding error, not all columns may be drawn so an additional stop condition is here:
    // When every bunches have been tested for rendition.
    while ((numbunches > 0) && (numhits > 0)) {
        // buffer is used to mark which bunches have been elected as "closest".
        // if tempbug[x] == 1 then it should be skipped.
        clearbuf(&buffer[0],(int32_t)((numbunches+3)>>2),0L);

        /* Almost works, but not quite :( */
        closest = 0;
        buffer[closest] = 1;
        for (i=1; i<numbunches; i++) {
            if ((j = bunchfront(i,closest,&engine_state)) < 0) {
                continue;
            }
            buffer[i] = 1;
            if (j == 0) {
                buffer[closest] = 1;
                closest = i;
            }
        }

        /* Double-check */
        for (i=0; i<numbunches; i++) {
            if (buffer[i]) {
                continue;
            }
            if ((j = bunchfront(i,closest,&engine_state)) < 0) {
                continue;
            }
            buffer[i] = 1;
            if (j == 0) {
                buffer[closest] = 1;
                closest = i, i = 0;
            }
        }

        //Draw every solid walls with ceiling/floor in the bunch "closest"
        drawalls(closest, &numscans, &numhits, &numbunches, &engine_state);

        //Since we just rendered a bunch, lower the current stack element so we can treat the next item
        numbunches--;
        //...and move the bunch at the top of the stack so we won't iterate on it again...
        bunchfirst[closest] = bunchfirst[numbunches];
        bunchlast[closest] = bunchlast[numbunches];
    }

    return &engine_state;
}


static int spritewallfront (Sprite *s, int32_t w)
{
    walltype *wal;
    int32_t x1, y1;

    wal = &wall[w];
    x1 = wal->x;
    y1 = wal->y;
    wal = &wall[wal->point2];
    return (dmulscale32(wal->x-x1,s->y-y1,-(s->x-x1),wal->y-y1) >= 0);
}


static void transmaskvline(int32_t x, int32_t zd,
                           int32_t xpanning, int16_t picnum, int32_t shade,
                           int16_t shiftval, int32_t yscale, int32_t vis, int32_t pallete,
                           EngineState *engine_state)
{
    int32_t vplc, vinc, i;
    short y1v, y2v;

    if ((x < 0) || (x >= xdimen)) {
        return;
    }

    y1v = max(uwall[x],startumost[x+windowx1]-windowy1);
    y2v = min(dwall[x],startdmost[x+windowx1]-windowy1);
    y2v--;
    if (y2v < y1v) {
        return;
    }

    vinc = swall[x]*yscale;
    vplc = zd + vinc*(y1v - engine_state->horiz + 1);

    i = lwall[x] + xpanning;

    if (i >= tiles[picnum].dim.width) {
        i %= tiles[picnum].dim.width;
    }

    tvlineasm1(vinc,
               FP_OFF(palookup[pallete]) + (getpalookup((int32_t)mulscale16(swall[x],vis),shade)<<8),  // palookupoffs
               y2v-y1v,
               vplc,
               tiles[picnum].data + i * tiles[picnum].dim.height,
               shiftval,
               ylookup[y1v] + x + frameoffset);
}


// transmaskwallscan is like maskwallscan, but it can also blend to the background
static void transmaskwallscan(int32_t x1, int32_t x2, int32_t zd,
                              int32_t xpanning, int16_t picnum, int32_t shade,
                              int16_t shiftval, int32_t yscale, int32_t vis, int32_t pallete,
                              EngineState *engine_state)
{
    int32_t x;

    setgotpic(picnum);

    //Tile dimensions are invalid
    if ((tiles[picnum].dim.width <= 0) ||
        (tiles[picnum].dim.height <= 0)) {
        return;
    }

    TILE_MakeAvailable(picnum);

    x = x1;
    while ((startumost[x+windowx1] > startdmost[x+windowx1]) && (x <= x2)) {
        x++;
    }

    while (x <= x2) {
        transmaskvline(x, zd, xpanning, picnum, shade, shiftval, yscale, vis, pallete, engine_state), x++;
    }
    
    faketimerhandler();
}

int loadboard(char  *filename, int32_t *daposx, int32_t *daposy,
              int32_t *daposz, short *daang, short *dacursectnum)
{
    int x;
    short fil, i, numsprites;
    Sector *sect;
    Sprite *s;
    walltype *w;

    // FIX_00058: Save/load game crash in both single and multiplayer
    // We have to reset those arrays since the same
    // arrays are used as temporary space in the
    // compilecons() function like "label = (uint8_t  *)&sprite[0];"
    // to save memory space I guess.
    // Not reseting the array will leave dumps fooling
    // the function saveplayer(), eg at if(actorscrptr[PN] == 0)
    // where PN is sprite[i].picnum was beyong actorscrptr[] size)
    memset(sprite, 0, sizeof(sprite));
    memset(sector, 0, sizeof(sector));
    memset(wall, 0, sizeof(wall));

    if ((fil = kopen4load(filename, 0)) == -1) {
        mapversion = 7L;
        return(-1);
    }

    kread32(fil,&mapversion);
    if (mapversion != 7L) {
        return(-1);
    }

    initspritelists();

    kread32(fil,daposx);
    kread32(fil,daposy);
    kread32(fil,daposz);
    kread16(fil,daang);
    kread16(fil,dacursectnum);
    kread16(fil,&numsectors);

    for (x = 0, sect = &sector[0]; x < numsectors; x++, sect++) {
        kread16(fil,&sect->wallptr);
        kread16(fil,&sect->wallnum);
        kread32(fil,&sect->ceiling.z);
        kread32(fil,&sect->floor.z);
        kread16(fil,(int16_t *)&sect->ceiling.flags);
        kread16(fil,(int16_t *)&sect->floor.flags);
        kread16(fil,&sect->ceiling.picnum);
        kread16(fil,&sect->ceiling.heinum);
        kread8(fil,(uint8_t *)&sect->ceiling.shade);
        kread8(fil,(uint8_t *)&sect->ceiling.pal);
        kread8(fil,(uint8_t *)&sect->ceiling.xpanning);
        kread8(fil,(uint8_t *)&sect->ceiling.ypanning);
        kread16(fil,&sect->floor.picnum);
        kread16(fil,&sect->floor.heinum);
        kread8(fil,(uint8_t *)&sect->floor.shade);
        kread8(fil,(uint8_t *)&sect->floor.pal);
        kread8(fil,(uint8_t *)&sect->floor.xpanning);
        kread8(fil,(uint8_t *)&sect->floor.ypanning);
        kread8(fil,(uint8_t *)&sect->visibility);
        kread8(fil,(uint8_t *)&sect->filler);
        kread16(fil,&sect->lotag);
        kread16(fil,&sect->hitag);
        kread16(fil,&sect->extra);

        sect->ceiling.sector = sect->floor.sector = sect;
    }

    kread16(fil,&numwalls);
    for (x = 0, w = &wall[0]; x < numwalls; x++, w++) {
        kread32(fil,&w->x);
        kread32(fil,&w->y);
        kread16(fil,&w->point2);
        kread16(fil,&w->nextwall);
        kread16(fil,&w->nextsector);
        kread16(fil,(int16_t *)&w->flags);
        kread16(fil,&w->picnum);
        kread16(fil,&w->overpicnum);
        kread8(fil,(uint8_t *)&w->shade);
        kread8(fil,&w->pal);
        kread8(fil,&w->xrepeat);
        kread8(fil,&w->yrepeat);
        kread8(fil,&w->xpanning);
        kread8(fil,&w->ypanning);
        kread16(fil,&w->lotag);
        kread16(fil,&w->hitag);
        kread16(fil,&w->extra);
    }

    kread16(fil,&numsprites);
    for (x = 0, s = &sprite[0]; x < numsprites; x++, s++) {
        kread32(fil,&s->x);
        kread32(fil,&s->y);
        kread32(fil,&s->z);
        kread16(fil,(int16_t *)&s->flags);
        kread16(fil,&s->picnum);
        kread8(fil,(uint8_t *)&s->shade);
        kread8(fil,(uint8_t *)&s->pal);
        kread8(fil,(uint8_t *)&s->clipdist);
        kread8(fil,(uint8_t *)&s->filler);
        kread8(fil,(uint8_t *)&s->xrepeat);
        kread8(fil,(uint8_t *)&s->yrepeat);
        kread8(fil,(uint8_t *)&s->xoffset);
        kread8(fil,(uint8_t *)&s->yoffset);
        kread16(fil,&s->sectnum);
        kread16(fil,&s->statnum);
        kread16(fil,&s->ang);
        kread16(fil,&s->owner);
        kread16(fil,&s->xvel);
        kread16(fil,&s->yvel);
        kread16(fil,&s->zvel);
        kread16(fil,&s->lotag);
        kread16(fil,&s->hitag);
        kread16(fil,&s->extra);
    }


    for (i=0; i<numsprites; i++) {
        insertsprite(sprite[i].sectnum,sprite[i].statnum);
    }

    /* Must be after loading sectors, etc! */
    updatesector(*daposx,*daposy,dacursectnum);

    kclose(fil);

    // FIX_00009: Show map CRC and GRP file version of each player in case of Out Of Synch

    mapCRC = crc16((uint8_t *)sector, numsectors*sizeof(Sector));
    mapCRC += crc16((uint8_t *)wall, numwalls*sizeof(walltype));
    mapCRC += crc16((uint8_t *)sprite, numsprites*sizeof(Sprite));

    return(0);
}


static void write32(int f, int32_t val)
{
    val = BUILDSWAP_INTEL32(val);
    write(f, &val, 4);
}

static void write16(int f, short val)
{
    val = BUILDSWAP_INTEL16(val);
    write(f, &val, 2);
}

static void write8(int f, uint8_t  val)
{
    write(f, &val, 1);
}


int saveboard(char  *filename, int32_t *daposx, int32_t *daposy,
              int32_t *daposz, short *daang, short *dacursectnum)
{
    int fil;
    int x;
    short i, j, numsprites;
    int permissions = 0;
    walltype *w;
    Sector *sect;

#if ((defined PLATFORM_DOS) || (defined PLATFORM_WIN32))
    permissions = S_IWRITE;
#elif (defined PLATFORM_UNIX)
    permissions = S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH;
#endif

    if ((fil = open(filename,
                    O_BINARY|O_TRUNC|O_CREAT|O_WRONLY,
                    permissions)) == -1) {
        return(-1);
    }

    write32(fil,mapversion);

    write32(fil,*daposx);
    write32(fil,*daposy);
    write32(fil,*daposz);
    write16(fil,*daang);
    write16(fil,*dacursectnum);

    write16(fil,numsectors);
    for (x = 0, sect = &sector[0]; x < numsectors; x++, sect++) {
        write16(fil,sect->wallptr);
        write16(fil,sect->wallnum);
        write32(fil,sect->ceiling.z);
        write32(fil,sect->floor.z);
        write16(fil,*(uint16_t *)&sect->ceiling.flags);
        write16(fil,*(uint16_t *)&sect->floor.flags);
        write16(fil,sect->ceiling.picnum);
        write16(fil,sect->ceiling.heinum);
        write8(fil,sect->ceiling.shade);
        write8(fil,sect->ceiling.pal);
        write8(fil,sect->ceiling.xpanning);
        write8(fil,sect->ceiling.ypanning);
        write16(fil,sect->floor.picnum);
        write16(fil,sect->floor.heinum);
        write8(fil,sect->floor.shade);
        write8(fil,sect->floor.pal);
        write8(fil,sect->floor.xpanning);
        write8(fil,sect->floor.ypanning);
        write8(fil,sect->visibility);
        write8(fil,sect->filler);
        write16(fil,sect->lotag);
        write16(fil,sect->hitag);
        write16(fil,sect->extra);
    }

    write16(fil,numwalls);
    for (x = 0, w = &wall[0]; x < numwalls; x++, w++) {
        write32(fil,w->x);
        write32(fil,w->y);
        write16(fil,w->point2);
        write16(fil,w->nextwall);
        write16(fil,w->nextsector);
        write16(fil,*(short *)&w->flags);
        write16(fil,w->picnum);
        write16(fil,w->overpicnum);
        write8(fil,w->shade);
        write8(fil,w->pal);
        write8(fil,w->xrepeat);
        write8(fil,w->yrepeat);
        write8(fil,w->xpanning);
        write8(fil,w->ypanning);
        write16(fil,w->lotag);
        write16(fil,w->hitag);
        write16(fil,w->extra);
    }

    numsprites = 0;
    for (j=0; j<MAXSTATUS; j++) {
        i = headspritestat[j];
        while (i != -1) {
            numsprites++;
            i = nextspritestat[i];
        }
    }
    write16(fil,numsprites);

    for (j=0; j<MAXSTATUS; j++) {
        i = headspritestat[j];
        while (i != -1) {
            Sprite *s = &sprite[i];
            write32(fil,s->x);
            write32(fil,s->y);
            write32(fil,s->z);
            write16(fil,*(int16_t *)&s->flags);
            write16(fil,s->picnum);
            write8(fil,s->shade);
            write8(fil,s->pal);
            write8(fil,s->clipdist);
            write8(fil,s->filler);
            write8(fil,s->xrepeat);
            write8(fil,s->yrepeat);
            write8(fil,s->xoffset);
            write8(fil,s->yoffset);
            write16(fil,s->sectnum);
            write16(fil,s->statnum);
            write16(fil,s->ang);
            write16(fil,s->owner);
            write16(fil,s->xvel);
            write16(fil,s->yvel);
            write16(fil,s->zvel);
            write16(fil,s->lotag);
            write16(fil,s->hitag);
            write16(fil,s->extra);

            i = nextspritestat[i];
        }
    }

    close(fil);
    return(0);
}


static void loadtables(void)
{
    int32_t i, fil;
    static uint8_t  tablesloaded = 0;

    if (tablesloaded == 0) {

        if ((fil = TCkopen4load("tables.dat",0)) != -1) {
            // Skips the first 2048*2 bytes
            for (i = 0; i < 2048; i++) {
                kread16(fil,&radarang[0]);
            }

            for (i = 0; i < 640; i++) {
                kread16(fil,&radarang[i]);
            }

            for (i=0; i<640; i++) {
                radarang[1279-i] = -radarang[i];
            }
            kread(fil,textfont,1024);
            kread(fil,smalltextfont,1024);
            kread(fil,britable,1024);
            kclose(fil);
        }
        tablesloaded = 1;
    }
}


static void initfastcolorlookup(int32_t rscale, int32_t gscale, int32_t bscale)
{
    int32_t i, j, x, y, z;
    uint8_t  *pal1;

    j = 0;
    for (i=64; i>=0; i--) {
        /*j = (i-64)*(i-64);*/
        rdist[i] = rdist[128-i] = j*rscale;
        gdist[i] = gdist[128-i] = j*gscale;
        bdist[i] = bdist[128-i] = j*bscale;
        j += 129-(i<<1);
    }

    clearbufbyte((void *)FP_OFF(colhere),sizeof(colhere),0L);
    clearbufbyte((void *)FP_OFF(colhead),sizeof(colhead),0L);

    pal1 = &palette[768-3];
    for (i=255; i>=0; i--,pal1-=3) {
        j = (pal1[0]>>3)*FASTPALGRIDSIZ*FASTPALGRIDSIZ+(pal1[1]>>3)*FASTPALGRIDSIZ+(pal1[2]>>3)+FASTPALGRIDSIZ*FASTPALGRIDSIZ+FASTPALGRIDSIZ+1;
        if (colhere[j>>3]&pow2char[j&7]) {
            colnext[i] = colhead[j];
        } else {
            colnext[i] = -1;
        }
        colhead[j] = i;
        colhere[j>>3] |= pow2char[j&7];
    }

    i = 0;
    for (x=-FASTPALGRIDSIZ*FASTPALGRIDSIZ; x<=FASTPALGRIDSIZ*FASTPALGRIDSIZ; x+=FASTPALGRIDSIZ*FASTPALGRIDSIZ)
        for (y=-FASTPALGRIDSIZ; y<=FASTPALGRIDSIZ; y+=FASTPALGRIDSIZ)
            for (z=-1; z<=1; z++) {
                colscan[i++] = x+y+z;
            }
    i = colscan[13];
    colscan[13] = colscan[26];
    colscan[26] = i;
}

extern uint8_t lastPalette[768];
static void loadpalette(void)
{
    int32_t k, fil;


    if (paletteloaded != 0) return;

    if ((fil = TCkopen4load("palette.dat",0)) == -1) return;

    kread(fil,palette,768);

    //CODE EXPLORATION
    //WritePaletteToFile(palette,"palette.tga",16, 16);
    memcpy(lastPalette, palette, 768);


    kread16(fil,&numpalookups);

    //CODE EXPLORATION
    //printf("Num palettes lookup: %d.\n",numpalookups);

    if ((palookup[0] = (uint8_t *)kkmalloc(numpalookups<<8)) == NULL) {
        allocache(&palookup[0],numpalookups<<8,&permanentlock);
    }

    //Transluctent pallete is 65KB.
    if ((transluc = (uint8_t *)kkmalloc(65536)) == NULL) {
        allocache(&transluc,65536,&permanentlock);
    }
    
    kread(fil,palookup[0],numpalookups<<8);

    /*kread(fil,transluc,65536);*/
    for (k = 0; k < (65536 / 4); k++) {
        kread32(fil, ((int32_t *) transluc) + k);
    }

    kclose(fil);

    initfastcolorlookup(30L,59L,11L);

    paletteloaded = 1;

}



int setgamemode(uint8_t  davidoption, int32_t daxdim, int32_t daydim)
{
    return(_setgamemode(davidoption, daxdim, daydim));
}


void initengine(void)
{
    int32_t i;

    loadtables();

    xyaspect = -1;

    pskyoff[0] = 0;
    pskybits = 0;

    parallaxtype = 2;
    parallaxyoffs = 0L;
    parallaxyscale = 65536;
    showinvisibility = 0;

    paletteloaded = 0;

    searchit = 0;
    searchstat = -1;

    for (i=0; i<MAXPALOOKUPS; i++) {
        palookup[i] = NULL;
    }

    for (i=0 ; i < MAXTILES ; i++) {
        tiles[i].data = NULL;
    }

    validmodecnt = 0;

    pointhighlight = -1;
    linehighlight = -1;
    highlightcnt = 0;

    totalclock = 0;
    visibility = 512;
    parallaxvisibility = 512;

    loadpalette();
}


void uninitengine(void)
{
    if (transluc != NULL) {
        kkfree(transluc);
        transluc = NULL;
    }
    if (pic != NULL) {
        kkfree(pic);
        pic = NULL;
    }
    if (artfil != -1) {
        kclose(artfil);
    }
    _uninitengine(); /* video driver specific. */
}


/* Assume npoints=4 with polygon on &rx1,&ry1 */
//FCS This is horrible to read: I hate you.
static int clippoly4(int32_t cx1, int32_t cy1, int32_t cx2, int32_t cy2)
{
    int32_t n, nn, z, zz, x, x1, x2, y, y1, y2, t;

    nn = 0;
    z = 0;
    do {
        zz = ((z+1)&3);


        x1 = pvWalls[z] .cameraSpaceCoo[0][VEC_X];
        x2 = pvWalls[zz].cameraSpaceCoo[0][VEC_X]-x1;

        if ((cx1 <= x1) && (x1 <= cx2)) {
            pvWalls[nn] .cameraSpaceCoo[1][VEC_X] = x1;
            pvWalls[nn] .cameraSpaceCoo[1][VEC_Y] = pvWalls[z] .cameraSpaceCoo[0][VEC_Y];
            nn++;
        }

        if (x2 <= 0) {
            x = cx2;
        } else {
            x = cx1;
        }

        t = x-x1;

        if (((t-x2)^t) < 0) {
            pvWalls[nn] .cameraSpaceCoo[1][VEC_X] = x;
            pvWalls[nn] .cameraSpaceCoo[1][VEC_Y] = pvWalls[z].cameraSpaceCoo[0][VEC_Y] +
                                                    scale(t,pvWalls[zz].cameraSpaceCoo[0][VEC_Y]-pvWalls[z].cameraSpaceCoo[0][VEC_Y],x2);
            nn++;
        }

        if (x2 <= 0) {
            x = cx1;
        } else {
            x = cx2;
        }

        t = x-x1;

        if (((t-x2)^t) < 0) {
            pvWalls[nn] .cameraSpaceCoo[1][VEC_X] = x;
            pvWalls[nn] .cameraSpaceCoo[1][VEC_Y] = pvWalls[z] .cameraSpaceCoo[0][VEC_Y]+
                                                    scale(t,pvWalls[zz].cameraSpaceCoo[0][VEC_Y]-pvWalls[z].cameraSpaceCoo[0][VEC_Y],x2);
            nn++;
        }
        z = zz;
    } while (z != 0);
    if (nn < 3) {
        return(0);
    }

    n = 0;
    z = 0;
    do {
        zz = z+1;
        if (zz == nn) {
            zz = 0;
        }

        y1 = pvWalls[z] .cameraSpaceCoo[1][VEC_Y];
        y2 = pvWalls[zz].cameraSpaceCoo[1][VEC_Y]-y1;

        if ((cy1 <= y1) && (y1 <= cy2)) {
            pvWalls[n] .cameraSpaceCoo[0][VEC_Y] = y1;
            pvWalls[n] .cameraSpaceCoo[0][VEC_X] = pvWalls[z] .cameraSpaceCoo[1][VEC_X];
            n++;
        }
        if (y2 <= 0) {
            y = cy2;
        } else {
            y = cy1;
        }
        t = y-y1;
        if (((t-y2)^t) < 0) {
            pvWalls[n] .cameraSpaceCoo[0][VEC_Y] = y;
            pvWalls[n] .cameraSpaceCoo[0][VEC_X] =
                pvWalls[z] .cameraSpaceCoo[1][VEC_X]+scale(t,
                        pvWalls[zz].cameraSpaceCoo[1][VEC_X]-
                        pvWalls[z] .cameraSpaceCoo[1][VEC_X],y2);
            n++;
        }

        if (y2 <= 0) {
            y = cy1;
        } else {
            y = cy2;
        }
        t = y-y1;
        if (((t-y2)^t) < 0) {
            pvWalls[n] .cameraSpaceCoo[0][VEC_Y] = y;
            pvWalls[n] .cameraSpaceCoo[0][VEC_X] =
                pvWalls[z] .cameraSpaceCoo[1][VEC_X]+scale(t,
                        pvWalls[zz].cameraSpaceCoo[1][VEC_X]-
                        pvWalls[z ].cameraSpaceCoo[1][VEC_X],y2);
            n++;
        }
        z = zz;
    } while (z != 0);
    return(n);
}



static void dorotatesprite (int32_t sx, int32_t sy, int32_t z, short a, short picnum,
                            int8_t dashade, uint8_t  dapalnum, uint8_t  dastat, int32_t cx1,
                            int32_t cy1, int32_t cx2, int32_t cy2)
{
    int32_t cosang, sinang, v, nextv, dax1, dax2, oy, bx, by, ny1, ny2;
    int32_t i, x, y, x1, y1, x2, y2, gx1, gy1;
    uint8_t *bufplc;
    uint8_t *palookupoffs;
    uint8_t *p;
    int32_t xoff, yoff, npoints, yplc, yinc, lx, rx, xend;
    int32_t xv, yv, xv2, yv2, qlinemode=0, y1ve[4], y2ve[4];
    uint8_t  bad;
    int32_t bufplce[4], vplce[4], vince[4];

    short tileWidht, tileHeight;

    tileWidht = tiles[picnum].dim.width;
    tileHeight = tiles[picnum].dim.height;

    if (dastat&16) {
        xoff = 0;
        yoff = 0;
    } else {
        xoff = (int32_t)((int8_t )((tiles[picnum].animFlags>>8)&255))+(tileWidht>>1);
        yoff = (int32_t)((int8_t )((tiles[picnum].animFlags>>16)&255))+(tileHeight>>1);
    }

    if (dastat&4) {
        yoff = tileHeight-yoff;
    }

    cosang = fixedPointCos(a);
    sinang = fixedPointSin(a);

    if ((dastat&2) != 0) { /* Auto window size scaling */
        if ((dastat&8) == 0) {
            x = xdimenscale;   /* = scale(xdimen,yxaspect,320); */
            sx = ((cx1+cx2+2)<<15)+scale(sx-(320<<15),xdimen,320);
            sy = ((cy1+cy2+2)<<15)+mulscale16(sy-(200<<15),x);
        } else {
            /*
             * If not clipping to startmosts, & auto-scaling on, as a
             *  hard-coded bonus, scale to full screen instead
             */
            x = scale(xdim,yxaspect,320);
            sx = (xdim<<15)+32768+scale(sx-(320<<15),xdim,320);
            sy = (ydim<<15)+32768+mulscale16(sy-(200<<15),x);
        }
        z = mulscale16(z,x);
    }

    xv = mulscale14(cosang,z);
    yv = mulscale14(sinang,z);
    if (((dastat&2) != 0) || ((dastat&8) == 0)) { /* Don't aspect unscaled perms */
        xv2 = mulscale16(xv,xyaspect);
        yv2 = mulscale16(yv,xyaspect);
    } else {
        xv2 = xv;
        yv2 = yv;
    }


    //Taking care of the Y coordinates.
    pvWalls[0].cameraSpaceCoo[0][VEC_Y] = sy - (yv*xoff + xv*yoff);
    pvWalls[1].cameraSpaceCoo[0][VEC_Y] = pvWalls[0].cameraSpaceCoo[0][VEC_Y] + yv * tileWidht;
    pvWalls[3].cameraSpaceCoo[0][VEC_Y] = pvWalls[0].cameraSpaceCoo[0][VEC_Y] + xv * tileHeight;

    pvWalls[2].cameraSpaceCoo[0][VEC_Y] = pvWalls[1].cameraSpaceCoo[0][VEC_Y] +
                                          pvWalls[3].cameraSpaceCoo[0][VEC_Y] -
                                          pvWalls[0].cameraSpaceCoo[0][VEC_Y] ;

    i = (cy1<<16);

    if ((pvWalls[0].cameraSpaceCoo[0][VEC_Y]<i) &&
        (pvWalls[1].cameraSpaceCoo[0][VEC_Y]<i) &&
        (pvWalls[2].cameraSpaceCoo[0][VEC_Y]<i) &&
        (pvWalls[3].cameraSpaceCoo[0][VEC_Y]<i)) {
        return;
    }

    i = (cy2<<16);

    if ((pvWalls[0].cameraSpaceCoo[0][VEC_Y]>i) &&
        (pvWalls[1].cameraSpaceCoo[0][VEC_Y]>i) &&
        (pvWalls[2].cameraSpaceCoo[0][VEC_Y]>i) &&
        (pvWalls[3].cameraSpaceCoo[0][VEC_Y]>i)) {
        return;
    }

    //Taking care of the X coordinates.
    pvWalls[0].cameraSpaceCoo[0][VEC_X] = sx - (xv2*xoff - yv2*yoff);
    pvWalls[1].cameraSpaceCoo[0][VEC_X] = pvWalls[0].cameraSpaceCoo[0][VEC_X] + xv2 * tileWidht;
    pvWalls[3].cameraSpaceCoo[0][VEC_X] = pvWalls[0].cameraSpaceCoo[0][VEC_X] - yv2 * tileHeight;
    pvWalls[2].cameraSpaceCoo[0][VEC_X] = pvWalls[1].cameraSpaceCoo[0][VEC_X] +
                                          pvWalls[3].cameraSpaceCoo[0][VEC_X] -
                                          pvWalls[0].cameraSpaceCoo[0][VEC_X] ;

    i = (cx1<<16);
    if ((pvWalls[0].cameraSpaceCoo[0][VEC_X]<i) &&
        (pvWalls[1].cameraSpaceCoo[0][VEC_X]<i) &&
        (pvWalls[2].cameraSpaceCoo[0][VEC_X]<i) &&
        (pvWalls[3].cameraSpaceCoo[0][VEC_X]<i)) {
        return;
    }

    i = (cx2<<16);
    if ((pvWalls[0].cameraSpaceCoo[0][VEC_X]>i) &&
        (pvWalls[1].cameraSpaceCoo[0][VEC_X]>i) &&
        (pvWalls[2].cameraSpaceCoo[0][VEC_X]>i) &&
        (pvWalls[3].cameraSpaceCoo[0][VEC_X]>i)) {
        return;
    }

    gx1 = pvWalls[0].cameraSpaceCoo[0][VEC_X];
    gy1 = pvWalls[0].cameraSpaceCoo[0][VEC_Y];   /* back up these before clipping */

    if ((npoints = clippoly4(cx1<<16,cy1<<16,(cx2+1)<<16,(cy2+1)<<16)) < 3) {
        return;
    }

    lx = pvWalls[0].cameraSpaceCoo[0][VEC_X];
    rx = pvWalls[0].cameraSpaceCoo[0][VEC_X];

    nextv = 0;
    for (v=npoints-1; v>=0; v--) {
        x1 = pvWalls[    v].cameraSpaceCoo[0][VEC_X];
        x2 = pvWalls[nextv].cameraSpaceCoo[0][VEC_X];
        dax1 = (x1>>16);
        if (x1 < lx) {
            lx = x1;
        }
        dax2 = (x2>>16);
        if (x1 > rx) {
            rx = x1;
        }
        if (dax1 != dax2) {
            y1 = pvWalls[    v].cameraSpaceCoo[0][VEC_Y];
            y2 = pvWalls[nextv].cameraSpaceCoo[0][VEC_Y];
            yinc = divscale16(y2-y1,x2-x1);
            if (dax2 > dax1) {
                yplc = y1 + mulscale16((dax1<<16)+65535-x1,yinc);
                qinterpolatedown16short((int32_t *)(&uplc[dax1]),dax2-dax1,yplc,yinc);
            } else {
                yplc = y2 + mulscale16((dax2<<16)+65535-x2,yinc);
                qinterpolatedown16short((int32_t *)(&dplc[dax2]),dax1-dax2,yplc,yinc);
            }
        }
        nextv = v;
    }

    TILE_MakeAvailable(picnum);

    setgotpic(picnum);
    bufplc = tiles[picnum].data;

    palookupoffs = palookup[dapalnum] + (getpalookup(0L,(int32_t)dashade)<<8);

    i = divscale32(1L,z);
    xv = mulscale14(sinang,i);
    yv = mulscale14(cosang,i);
    if (((dastat&2) != 0) || ((dastat&8) == 0)) { /* Don't aspect unscaled perms */
        yv2 = mulscale16(-xv,yxaspect);
        xv2 = mulscale16(yv,yxaspect);
    } else {
        yv2 = -xv;
        xv2 = yv;
    }

    x1 = (lx>>16);
    x2 = (rx>>16);

    oy = 0;
    x = (x1<<16)-1-gx1;
    y = (oy<<16)+65535-gy1;
    bx = dmulscale16(x,xv2,y,xv);
    by = dmulscale16(x,yv2,y,yv);

    if (dastat&4) {
        yv = -yv;
        yv2 = -yv2;
        by = (tileHeight<<16)-1-by;
    }

    if ((dastat&1) == 0) {
        if (((a&1023) == 0) && (tileHeight <= 256)) { /* vlineasm4 has 256 high limit! */

            SetupVerticalLine(24L);

            by <<= 8;
            yv <<= 8;

            palookupoffse[0] = palookupoffse[1] = palookupoffse[2] = palookupoffse[3] = palookupoffs;
            vince[0] = vince[1] = vince[2] = vince[3] = yv;

            for (x=x1; x<x2; x++) {
                bad = 15;
                xend = min(x2-x,4);

                bx += xv2;

                y1 = uplc[x];
                y2 = dplc[x];
                
                if ((dastat&8) == 0) {
                    if (startumost[x] > y1) {
                        y1 = startumost[x];
                    }
                    if (startdmost[x] < y2) {
                        y2 = startdmost[x];
                    }
                }

                if (y2 <= y1) continue;

                by += yv*(y1-oy);
                oy = y1;

                bufplce[0] = (bx>>16)*tileHeight+bufplc;
                vplce[0] = by;
                y1ve[0] = y1;
                y2ve[0] = y2-1;
                bad &= ~pow2char[x];

                DrawVerticalLine(vince[0], palookupoffse[0], y2ve[0] - y1ve[0] - 1, vplce[0], bufplce[0], x + frameoffset + ylookup[y1ve[0]]);

                faketimerhandler();
            }
        } else {
            if (dastat&64) {
                if ((xv2&0x0000ffff) == 0) {
                    qlinemode = 1;
                    setuprhlineasm4(0L,yv2<<16,(xv2>>16)*tileHeight+(yv2>>16),palookupoffs,0L);
                } else {
                    qlinemode = 0;
                    setuprhlineasm4(xv2<<16,yv2<<16,(xv2>>16)*tileHeight+(yv2>>16),palookupoffs,tileHeight);
                }
            } else {
                setuprmhlineasm4(xv2<<16,yv2<<16,(xv2>>16)*tileHeight+(yv2>>16),palookupoffs,tileHeight);
            }

            y1 = uplc[x1];
            if (((dastat&8) == 0) && (startumost[x1] > y1)) {
                y1 = startumost[x1];
            }
            y2 = y1;
            for (x=x1; x<x2; x++) {
                ny1 = uplc[x]-1;
                ny2 = dplc[x];
                if ((dastat&8) == 0) {
                    if (startumost[x]-1 > ny1) {
                        ny1 = startumost[x]-1;
                    }
                    if (startdmost[x] < ny2) {
                        ny2 = startdmost[x];
                    }
                }

                if (ny1 < ny2-1) {
                    if (ny1 >= y2) {
                        while (y1 < y2-1) {
                            y1++;
                            if ((y1&31) == 0) {
                                faketimerhandler();
                            }

                            /* x,y1 */
                            bx += xv*(y1-oy);
                            by += yv*(y1-oy);
                            oy = y1;
                            if (dastat&64) {
                                if (qlinemode) {
                                    rhlineasm4(x-lastx[y1],(bx>>16)*tileHeight+(by>>16)+bufplc,0L,0L    ,by<<16,ylookup[y1]+x+frameplace);
                                } else {
                                    rhlineasm4(x-lastx[y1],(bx>>16)*tileHeight+(by>>16)+bufplc,0L,bx<<16,by<<16,ylookup[y1]+x+frameplace);
                                }
                            } else {
                                rmhlineasm4(x-lastx[y1],(bx>>16)*tileHeight+(by>>16)+bufplc,0L,bx<<16,by<<16,ylookup[y1]+x+frameplace);
                            }
                        }
                        y1 = ny1;
                    } else {
                        while (y1 < ny1) {
                            y1++;
                            if ((y1&31) == 0) {
                                faketimerhandler();
                            }

                            /* x,y1 */
                            bx += xv*(y1-oy);
                            by += yv*(y1-oy);
                            oy = y1;
                            if (dastat&64) {
                                if (qlinemode) {
                                    rhlineasm4(x-lastx[y1],(bx>>16)*tileHeight+(by>>16)+bufplc,0L,0L,by<<16,ylookup[y1]+x+frameplace);
                                } else {
                                    rhlineasm4(x-lastx[y1],(bx>>16)*tileHeight+(by>>16)+bufplc,0L,bx<<16,by<<16,ylookup[y1]+x+frameplace);
                                }
                            } else {
                                rmhlineasm4(x-lastx[y1],(bx>>16)*tileHeight+(by>>16)+bufplc,0L,bx<<16,by<<16,ylookup[y1]+x+frameplace);
                            }
                        }
                        while (y1 > ny1) {
                            lastx[y1--] = x;
                        }
                    }
                    while (y2 > ny2) {
                        y2--;
                        if ((y2&31) == 0) {
                            faketimerhandler();
                        }

                        /* x,y2 */
                        bx += xv*(y2-oy);
                        by += yv*(y2-oy);
                        oy = y2;
                        if (dastat&64) {
                            if (qlinemode) {
                                rhlineasm4(x-lastx[y2],(bx>>16)*tileHeight+(by>>16)+bufplc,0L,0L    ,by<<16,ylookup[y2]+x+frameplace);
                            } else {
                                rhlineasm4(x-lastx[y2],(bx>>16)*tileHeight+(by>>16)+bufplc,0L,bx<<16,by<<16,ylookup[y2]+x+frameplace);
                            }
                        } else {
                            rmhlineasm4(x-lastx[y2],(bx>>16)*tileHeight+(by>>16)+bufplc,0L,bx<<16,by<<16,ylookup[y2]+x+frameplace);
                        }
                    }
                    while (y2 < ny2) {
                        lastx[y2++] = x;
                    }
                } else {
                    while (y1 < y2-1) {
                        y1++;
                        if ((y1&31) == 0) {
                            faketimerhandler();
                        }

                        /* x,y1 */
                        bx += xv*(y1-oy);
                        by += yv*(y1-oy);
                        oy = y1;
                        if (dastat&64) {
                            if (qlinemode) {
                                rhlineasm4(x-lastx[y1],(bx>>16)*tileHeight+(by>>16)+bufplc,0L,0L    ,by<<16,ylookup[y1]+x+frameplace);
                            } else {
                                rhlineasm4(x-lastx[y1],(bx>>16)*tileHeight+(by>>16)+bufplc,0L,bx<<16,by<<16,ylookup[y1]+x+frameplace);
                            }
                        } else {
                            rmhlineasm4(x-lastx[y1],(bx>>16)*tileHeight+(by>>16)+bufplc,0L,bx<<16,by<<16,ylookup[y1]+x+frameplace);
                        }
                    }
                    if (x == x2-1) {
                        bx += xv2;
                        by += yv2;
                        break;
                    }

                    y1 = uplc[x+1];

                    if (((dastat&8) == 0) && (startumost[x+1] > y1)) {
                        y1 = startumost[x+1];
                    }

                    y2 = y1;
                }
                bx += xv2;
                by += yv2;
            }
            while (y1 < y2-1) {
                y1++;
                if ((y1&31) == 0) {
                    faketimerhandler();
                }

                /* x2,y1 */
                bx += xv*(y1-oy);
                by += yv*(y1-oy);
                oy = y1;
                if (dastat&64) {
                    if (qlinemode) {
                        rhlineasm4(x2-lastx[y1],(bx>>16)*tileHeight+(by>>16)+bufplc,0L,0L    ,by<<16,ylookup[y1]+x2+frameplace);
                    } else {
                        rhlineasm4(x2-lastx[y1],(bx>>16)*tileHeight+(by>>16)+bufplc,0L,bx<<16,by<<16,ylookup[y1]+x2+frameplace);
                    }
                } else {
                    rmhlineasm4(x2-lastx[y1],(bx>>16)*tileHeight+(by>>16)+bufplc,0L,bx<<16,by<<16,ylookup[y1]+x2+frameplace);
                }
            }
        }
    } else {
        if ((dastat&1) == 0) {
            if (dastat&64) {
                setupspritevline(palookupoffs,(xv>>16)*tileHeight,xv<<16,tileHeight,yv);
            } else {
                msetupspritevline(palookupoffs,(xv>>16)*tileHeight,xv<<16,tileHeight,yv);
            }
        } else {
            tsetupspritevline(palookupoffs,(xv>>16)*tileHeight,xv<<16,tileHeight,yv);

            if (dastat&32) {
                settrans(TRANS_REVERSE);
            } else {
                settrans(TRANS_NORMAL);
            }
        }

        for (x=x1; x<x2; x++) {
            bx += xv2;
            by += yv2;

            y1 = uplc[x];
            y2 = dplc[x];
            if ((dastat&8) == 0) {
                if (startumost[x] > y1) {
                    y1 = startumost[x];
                }
                if (startdmost[x] < y2) {
                    y2 = startdmost[x];
                }
            }
            if (y2 <= y1) {
                continue;
            }

            switch (y1-oy) {
                case -1:
                    bx -= xv;
                    by -= yv;
                    oy = y1;
                    break;
                case 0:
                    break;
                case 1:
                    bx += xv;
                    by += yv;
                    oy = y1;
                    break;
                default:
                    bx += xv*(y1-oy);
                    by += yv*(y1-oy);
                    oy = y1;
                    break;
            }

            p = ylookup[y1]+x+frameplace;

            if ((dastat&1) == 0) {
                if (dastat&64) {
                    spritevline(0L,by<<16,bx<<16,(bx>>16)*tileHeight+(by>>16)+bufplc,p);
                } else {
                    mspritevline(0L,by<<16,bx<<16,(bx>>16)*tileHeight+(by>>16)+bufplc,p);
                }
            } else {
                DrawSpriteVerticalLine(by<<16,y2-y1+1,bx<<16,(bx>>16)*tileHeight+(by>>16)+bufplc,p);
            }
            faketimerhandler();
        }
    }
}


void nextpage(void)
{
    int32_t i;
    permfifotype *per;

    if (game_mode.qsetmode == 200) {
        for (i=permtail; i!=permhead; i=((i+1)&(MAXPERMS-1))) {
            per = &permfifo[i];
            if ((per->pagesleft > 0) && (per->pagesleft <= numpages)) {
                dorotatesprite(per->sx,per->sy,per->z,per->a,per->picnum,per->dashade,per->dapalnum,per->dastat,per->cx1,per->cy1,per->cx2,per->cy2);
            }
        }
    } /* if */

    _nextpage();  /* video driver specific. */


    if (game_mode.qsetmode == 200) {
        for (i=permtail; i!=permhead; i=((i+1)&(MAXPERMS-1))) {
            per = &permfifo[i];
            if (per->pagesleft >= 130)
                dorotatesprite(per->sx,per->sy,per->z,per->a,per->picnum,
                               per->dashade,per->dapalnum,per->dastat,
                               per->cx1,per->cy1,per->cx2,per->cy2);
            if (per->pagesleft&127) {
                per->pagesleft--;
            }
            if (((per->pagesleft&127) == 0) && (i == permtail)) {
                permtail = ((permtail+1)&(MAXPERMS-1));
            }
        }
    } /* if */

    faketimerhandler();

    if ((totalclock >= lastageclock+8) || (totalclock < lastageclock)) {
        lastageclock = totalclock;
        agecache();
    }

    beforedrawrooms = 1;
    numframes++;
}



int clipinsidebox(int32_t x, int32_t y, short wallnum, int32_t walldist)
{
    walltype *wal;
    int32_t x1, y1, x2, y2, r;

    r = (walldist<<1);
    wal = &wall[wallnum];
    x1 = wal->x+walldist-x;
    y1 = wal->y+walldist-y;
    wal = &wall[wal->point2];
    x2 = wal->x+walldist-x;
    y2 = wal->y+walldist-y;

    if ((x1 < 0) && (x2 < 0)) return 0;
    if ((y1 < 0) && (y2 < 0)) return 0;
    if ((x1 >= r) && (x2 >= r)) return 0;
    if ((y1 >= r) && (y2 >= r)) return 0;

    x2 -= x1;
    y2 -= y1;
    if (x2*(walldist-y1) >= y2*(walldist-x1)) { /* Front */
        if (x2 > 0) {
            x2 *= (0-y1);
        } else {
            x2 *= (r-y1);
        }
        if (y2 > 0) {
            y2 *= (r-x1);
        } else {
            y2 *= (0-x1);
        }
        return(x2 < y2);
    }
    if (x2 > 0) {
        x2 *= (r-y1);
    } else {
        x2 *= (0-y1);
    }
    if (y2 > 0) {
        y2 *= (0-x1);
    } else {
        y2 *= (r-x1);
    }
    return((x2 >= y2)<<1);
}

static int clipinsideboxline(int32_t x, int32_t y, int32_t x1, int32_t y1, int32_t x2, int32_t y2, int32_t walldist)
{
    int32_t r;

    r = (walldist<<1);

    x1 += walldist-x;
    x2 += walldist-x;
    if ((x1 < 0) && (x2 < 0)) {
        return(0);
    }
    if ((x1 >= r) && (x2 >= r)) {
        return(0);
    }

    y1 += walldist-y;
    y2 += walldist-y;
    if ((y1 < 0) && (y2 < 0)) {
        return(0);
    }
    if ((y1 >= r) && (y2 >= r)) {
        return(0);
    }

    x2 -= x1;
    y2 -= y1;
    if (x2*(walldist-y1) >= y2*(walldist-x1)) { /* Front */
        if (x2 > 0) {
            x2 *= (0-y1);
        } else {
            x2 *= (r-y1);
        }
        if (y2 > 0) {
            y2 *= (r-x1);
        } else {
            y2 *= (0-x1);
        }
        return(x2 < y2);
    }
    if (x2 > 0) {
        x2 *= (r-y1);
    } else {
        x2 *= (0-y1);
    }
    if (y2 > 0) {
        y2 *= (0-x1);
    } else {
        y2 *= (r-x1);
    }
    return((x2 >= y2)<<1);
}


void drawline256 (int32_t x1, int32_t y1, int32_t x2, int32_t y2, uint8_t  col)
{
    int32_t dx, dy, i, j, inc, plc, daend;
    uint8_t *p;
    col = palookup[0][col];

    dx = x2-x1;
    dy = y2-y1;
    if (dx >= 0) {
        if ((x1 >= wx2) || (x2 < wx1)) {
            return;
        }
        if (x1 < wx1) {
            y1 += scale(wx1-x1,dy,dx), x1 = wx1;
        }
        if (x2 > wx2) {
            y2 += scale(wx2-x2,dy,dx), x2 = wx2;
        }
    } else {
        if ((x2 >= wx2) || (x1 < wx1)) {
            return;
        }
        if (x2 < wx1) {
            y2 += scale(wx1-x2,dy,dx), x2 = wx1;
        }
        if (x1 > wx2) {
            y1 += scale(wx2-x1,dy,dx), x1 = wx2;
        }
    }
    if (dy >= 0) {
        if ((y1 >= wy2) || (y2 < wy1)) {
            return;
        }
        if (y1 < wy1) {
            x1 += scale(wy1-y1,dx,dy), y1 = wy1;
        }
        if (y2 > wy2) {
            x2 += scale(wy2-y2,dx,dy), y2 = wy2;
        }
    } else {
        if ((y2 >= wy2) || (y1 < wy1)) {
            return;
        }
        if (y2 < wy1) {
            x2 += scale(wy1-y2,dx,dy), y2 = wy1;
        }
        if (y1 > wy2) {
            x1 += scale(wy2-y1,dx,dy), y1 = wy2;
        }
    }

    if (klabs(dx) >= klabs(dy)) {
        if (dx == 0) {
            return;
        }
        if (dx < 0) {
            i = x1;
            x1 = x2;
            x2 = i;
            y1 = y2;
        }

        inc = divscale12(dy,dx);
        plc = y1+mulscale12((2047-x1)&4095,inc);
        i = ((x1+2048)>>12);
        daend = ((x2+2048)>>12);
        for (; i<daend; i++) {
            j = (plc>>12);
            if ((j >= startumost[i]) && (j < startdmost[i])) {
                drawpixel(ylookup[j]+i+frameplace,col);
            }
            plc += inc;
        }
    } else {
        if (dy < 0) {
            int32_t swap;

            x1 = x2;

            swap = y1;
            y1 = y2;
            y2 = swap;
        }

        inc = divscale12(dx,dy);
        plc = x1+mulscale12((2047-y1)&4095,inc);
        i = ((y1+2048)>>12);
        daend = ((y2+2048)>>12);
        p = ylookup[i]+frameplace;
        for (; i<daend; i++) {
            j = (plc>>12);
            if ((i >= startumost[j]) && (i < startdmost[j])) {
                drawpixel(j+p,col);
            }
            plc += inc;
            p += ylookup[1];
        }
    }
}

/*
 FCS: Return true if the point (x,Y) is inside the sector sectnum.
 Note that a sector is closed (but can be concave) so the answer is always 0 or 1.

 Algorithm: This is an optimized raycasting inside polygon test:
 http://en.wikipedia.org/wiki/Point_in_polygon#Ray_casting_algorithm
 The goal is to follow an ***horizontal*** ray passing by (x,y) and count how many
 wall are being crossed.
 If it is an odd number of time: (x,y) is inside the sector.
 If it is an even nymber of time:(x,y) is outside the sector.
 */

int inside(int32_t x, int32_t y, short sectnum)
{
    walltype *wal;
    int32_t i, x1, y1, x2, y2;
    uint32_t  wallCrossed;

    //Quick check if the sector ID is valid.
    if ((sectnum < 0) || (sectnum >= numsectors)) {
        return(-1);
    }

    wallCrossed = 0;
    wal = &wall[sector[sectnum].wallptr];
    i = sector[sectnum].wallnum;
    do {
        y1 = wal->y-y;
        y2 = wall[wal->point2].y-y;

        // Compare the sign of y1 and y2.
        // If (y1^y2) < 0 : y1 and y2 have different sign bit:  y is between wal->y and wall[wal->point2].y.
        // The goal is to not take into consideration any wall that is totally above or totally under the point [x,y].
        if ((y1^y2) < 0) {
            x1 = wal->x-x;
            x2 = wall[wal->point2].x-x;

            //If (x1^x2) >= 0 x1 and x2 have identic sign bit: x is on the left or the right of both wal->x and wall[wal->point2].x.
            if ((x1^x2) >= 0) {
                // If (x,y) is totally on the left or on the right, just count x1 (which indicate if we are on
                // on the left or on the right.
                wallCrossed ^= x1;
            } else {
                // This is the most complicated case: X is between x1 and x2, we need a fine grained test.
                // We need to know exactly if it is on the left or on the right in order to know if the ray
                // is crossing the wall or not,
                // The sign of the Cross-Product can answer this case :) !
                wallCrossed ^= (x1*y2-x2*y1)^y2;
            }
        }

        wal++;
        i--;

    } while (i);

    //Just return the sign. If the position vector cut the sector walls an odd number of time
    //it is inside. Otherwise (even) it is outside.
    return(wallCrossed>>31);
}


int getangle(int32_t xvect, int32_t yvect)
{
    if ((xvect|yvect) == 0) {
        return(0);
    }
    if (xvect == 0) {
        return(512+((yvect<0)<<10));
    }
    if (yvect == 0) {
        return(((xvect<0)<<10));
    }
    if (xvect == yvect) {
        return(256+((xvect<0)<<10));
    }
    if (xvect == -yvect) {
        return(768+((xvect>0)<<10));
    }
    if (klabs(xvect) > klabs(yvect)) {
        return(((radarang[640+scale(160,yvect,xvect)]>>6)+((xvect<0)<<10))&2047);
    }

    return(((radarang[640-scale(160,xvect,yvect)]>>6)+512+((yvect<0)<<10))&2047);
}


int ksqrt(int32_t num)
{
    return(fixedPointSqrt(num));
}





static void drawmaskwall(EngineState *engine_state)
{
    int32_t i, j, k, x, z, sectnum, z1, z2, lx, rx, zd;
    int32_t xpanning, ypanning;
    Sector *sec, *nsec;
    walltype *wal;
    int16_t picnum;
    int32_t shade;
    int16_t shiftval;
    int32_t yscale;
    int32_t vis;
    int32_t pallete;
    
    engine_state->maskwallcnt--;

    //Retrive pvWall ID.
    z = engine_state->maskwall[engine_state->maskwallcnt];

    //Retrive world wall ID.
    wal = &wall[pvWalls[z].worldWallId];

    //Retrive sector ID
    sectnum = pvWalls[z].sectorId;

    //Retrive sector.
    sec = &sector[sectnum];

    //Retrive next sector.
    nsec = &sector[wal->nextsector];

    z1 = max(nsec->ceiling.z, sec->ceiling.z);
    z2 = min(nsec->floor.z, sec->floor.z);

    wallmost(uwall, z, sec->ceiling, engine_state);
    wallmost(uplc, z, nsec->ceiling, engine_state);
    for (x=pvWalls[z].screenSpaceCoo[0][VEC_COL]; x<=pvWalls[z].screenSpaceCoo[1][VEC_COL]; x++)
        if (uplc[x] > uwall[x]) {
            uwall[x] = uplc[x];
        }

    wallmost(dwall, z, sec->floor, engine_state);
    wallmost(dplc, z, nsec->floor, engine_state);
    for (x=pvWalls[z].screenSpaceCoo[0][VEC_COL]; x<=pvWalls[z].screenSpaceCoo[1][VEC_COL]; x++)
        if (dplc[x] < dwall[x]) {
            dwall[x] = dplc[x];
        }


    prepwall(z,wal);

    picnum = wal->overpicnum;
    if ((uint32_t)picnum >= (uint32_t)MAXTILES) {
        picnum = 0;
    }
    xpanning = (int32_t)wal->xpanning;
    ypanning = (int32_t)wal->ypanning;

    if (tiles[picnum].animFlags&192) {
        picnum += animateoffs(picnum);
    }

    shade = (int32_t)wal->shade;
    vis = engine_state->visibility;
    if (sec->visibility != 0) {
        vis = mulscale4(vis,(int32_t)((uint8_t )(sec->visibility+16)));
    }
    pallete = (int32_t)wal->pal;
    shiftval = tiles[picnum].dim_power_2.height;
    if (pow2long[shiftval] != tiles[picnum].dim.height) {
        shiftval++;
    }

    shiftval = 32-shiftval;
    yscale = (wal->yrepeat<<(shiftval-19));
    if (!wal->flags.align_bottom) {
        zd = (((engine_state->posz-z1)*yscale)<<8);
    } else {
        zd = (((engine_state->posz-z2)*yscale)<<8);
    }
    zd += (ypanning<<24);
    if (wal->flags.transluscence_reversing) {
        yscale = -yscale;
        zd = -zd;
    }

    for (i=smostwallcnt-1; i>=0; i--) {
        j = smostwall[i];
        if ((pvWalls[j].screenSpaceCoo[0][VEC_COL] > pvWalls[z].screenSpaceCoo[1][VEC_COL]) || (pvWalls[j].screenSpaceCoo[1][VEC_COL] < pvWalls[z].screenSpaceCoo[0][VEC_COL])) {
            continue;
        }
        if (wallfront(j,z,engine_state)) {
            continue;
        }

        lx = max(pvWalls[j].screenSpaceCoo[0][VEC_COL],pvWalls[z].screenSpaceCoo[0][VEC_COL]);
        rx = min(pvWalls[j].screenSpaceCoo[1][VEC_COL],pvWalls[z].screenSpaceCoo[1][VEC_COL]);

        switch (smostwalltype[i]) {
            case 0:
                if (lx <= rx) {
                    if ((lx == pvWalls[z].screenSpaceCoo[0][VEC_COL]) && (rx == pvWalls[z].screenSpaceCoo[1][VEC_COL])) {
                        return;
                    }
                    clearbufbyte(&dwall[lx],(rx-lx+1)*sizeof(dwall[0]),0L);
                }
                break;
            case 1:
                k = smoststart[i] - pvWalls[j].screenSpaceCoo[0][VEC_COL];
                for (x=lx; x<=rx; x++)
                    if (engine_state->smost[k+x] > uwall[x]) {
                        uwall[x] = engine_state->smost[k+x];
                    }
                break;
            case 2:
                k = smoststart[i] - pvWalls[j].screenSpaceCoo[0][VEC_COL];
                for (x=lx; x<=rx; x++)
                    if (engine_state->smost[k+x] < dwall[x]) {
                        dwall[x] = engine_state->smost[k+x];
                    }
                break;
        }
    }

    /* maskwall */
    if ((searchit >= 1) && (searchx >= pvWalls[z].screenSpaceCoo[0][VEC_COL]) && (searchx <= pvWalls[z].screenSpaceCoo[1][VEC_COL]))
        if ((searchy >= uwall[searchx]) && (searchy <= dwall[searchx])) {
            searchsector = sectnum;
            searchwall = pvWalls[z].worldWallId;
            searchstat = 4;
            searchit = 1;
        }

    if (!wal->flags.transluscence_reversing) {
        maskwallscan(pvWalls[z].screenSpaceCoo[0][VEC_COL],
                     pvWalls[z].screenSpaceCoo[1][VEC_COL],
                     uwall, dwall, swall, lwall,
                     zd,
                     xpanning, picnum, shade, shiftval, yscale, vis, pallete,
                     engine_state);
    } else {
        if (wal->flags.transluscence_reversing) {
            if (wal->flags.reserved) {
                settrans(TRANS_REVERSE);
            } else {
                settrans(TRANS_NORMAL);
            }
        }
        transmaskwallscan(pvWalls[z].screenSpaceCoo[0][VEC_COL],
                          pvWalls[z].screenSpaceCoo[1][VEC_COL],
                          zd, xpanning, picnum, shade, shiftval, yscale, vis, pallete,
                          engine_state);
    }
}





static void ceilspritehline (int32_t x2, int32_t y, int32_t zd,
                             int32_t xpanning, int32_t ypanning,
                             int32_t g_x1, int32_t g_y1, int32_t g_x2, int32_t g_y2,
                             SectorFlags flags,
                             int32_t shade, int32_t vis,
                             int32_t pallete,
                             uint8_t *tile_data,
                             EngineState *engine_state)
{
    int32_t x1, v, bx, by, a1, a2, a3;

    /*
     * x = x1 + (x2-x1)t + (y1-y2)u  ³  x = 160v
     * y = y1 + (y2-y1)t + (x2-x1)u  ³  y = (scrx-160)v
     * z = z1 = z2                   ³  z = posz + (scry-horiz)v
     */

    x1 = lastx[y];
    if (x2 < x1) {
        return;
    }

    v = mulscale20(zd, horizlookup[y - engine_state->horiz + horizycent]);

    bx = mulscale14(g_x2 * x1 + g_x1, v) + xpanning;
    by = mulscale14(g_y2 * x1 + g_y1, v) + ypanning;

    a1 = mulscale14(g_x2,v);
    a2 = mulscale14(g_y2,v);
    a3 = (int32_t)FP_OFF(palookup[pallete]) + (getpalookup((int32_t)mulscale28(klabs(v),vis),shade)<<8);

    if (!flags.groudraw) {
        mhline(tile_data, bx, (x2-x1)<<16, by, ylookup[y]+x1+frameoffset, a1, a2, a3);
    } else {
        thline(tile_data, bx, (x2-x1)<<16, by, ylookup[y]+x1+frameoffset, a1, a2, a3);
    }
}


static void ceilspritescan (int32_t x1, int32_t x2, int32_t zd,
                            int32_t xpanning, int32_t ypanning,
                            int32_t g_x1, int32_t g_y1, int32_t g_x2, int32_t g_y2,
                            SectorFlags flags, int32_t shade, int32_t vis,
                            int32_t pallete,
                            uint8_t *tile_data,
                            EngineState *engine_state)
{
    int32_t x, y1, y2, twall, bwall;

    y1 = uwall[x1];
    y2 = y1;
    for (x=x1; x<=x2; x++) {
        twall = uwall[x]-1;
        bwall = dwall[x];
        if (twall < bwall-1) {
            if (twall >= y2) {
                while (y1 < y2-1) {
                    ceilspritehline(x-1, ++y1, zd, xpanning, ypanning, g_x1, g_y1, g_x2, g_y2, flags, shade, vis, pallete, tile_data, engine_state);
                }
                y1 = twall;
            } else {
                while (y1 < twall) {
                    ceilspritehline(x-1, ++y1, zd, xpanning, ypanning, g_x1, g_y1, g_x2, g_y2, flags, shade, vis, pallete, tile_data, engine_state);
                }
                while (y1 > twall) {
                    lastx[y1--] = x;
                }
            }
            while (y2 > bwall) {
                ceilspritehline(x-1, --y2, zd, xpanning, ypanning, g_x1, g_y1, g_x2, g_y2, flags, shade, vis, pallete, tile_data, engine_state);
            }
            while (y2 < bwall) {
                lastx[y2++] = x;
            }
        } else {
            while (y1 < y2-1) {
                ceilspritehline(x-1, ++y1, zd, xpanning, ypanning, g_x1, g_y1, g_x2, g_y2, flags, shade, vis, pallete, tile_data, engine_state);
            }
            if (x == x2) {
                break;
            }
            y1 = uwall[x+1];
            y2 = y1;
        }
    }
    while (y1 < y2-1) {
        ceilspritehline(x2, ++y1, zd, xpanning, ypanning, g_x1, g_y1, g_x2, g_y2, flags, shade, vis, pallete, tile_data, engine_state);
    }
    faketimerhandler();
}

static void drawsprite (EngineState *engine_state, int32_t *spritesx, int32_t *spritesy)
{
    Sprite *tspr;
    Sector *sec;
    int32_t startum, startdm, sectnum, xb, yp;
    SpriteFlags cstat;
    int32_t siz, xsiz, ysiz, xoff, yoff;
    dimensions_t spriteDim;
    int32_t x1, y1, x2, y2, lx, rx, dalx2, darx2, i, j, k, x, linum, linuminc;
    int32_t yinc, z, z1, z2, xp1, yp1, xp2, yp2;
    int32_t xv, yv, top, topinc, bot, botinc, hplc, hinc;
    int32_t cosang, sinang, dax, day, lpoint, lmax, rpoint, rmax, dax1, dax2, y;
    int32_t npoints, npoints2, zz, t, zsgn, zzsgn;
    int32_t zd;
    int32_t xpanning, ypanning;
    int32_t g_x1, g_y1, g_x2, g_y2;
    short tilenum, spritenum;
    uint8_t  swapped, daclip;
    int16_t picnum;
    int32_t shade;
    int16_t shiftval;
    int32_t yscale;
    int32_t vis;
    int32_t pallete;
    
    engine_state->spritesortcnt--;

    tspr = tspriteptr[engine_state->spritesortcnt];

    xb = spritesx[engine_state->spritesortcnt];
    yp = spritesy[engine_state->spritesortcnt];
    tilenum = tspr->picnum;
    spritenum = tspr->owner;
    cstat = tspr->flags;

    if (cstat.type != UNKNOW_SPRITE) {
        if (tiles[tilenum].animFlags&192) {
            tilenum += animateoffs(tilenum);
        }

        if ((tiles[tilenum].dim.width <= 0) || (tiles[tilenum].dim.height <= 0) || (spritenum < 0)) {
            return;
        }
    }
    if ((tspr->xrepeat <= 0) || (tspr->yrepeat <= 0)) {
        return;
    }

    sectnum = tspr->sectnum;
    sec = &sector[sectnum];
    pallete = tspr->pal;
    // FIX_00088: crash on maps using a bad palette index (like the end of roch3.map)
    if (!palookup[pallete]) {
        pallete = 0;    // seem to crash when globalpal > 25
    }
    shade = tspr->shade;
    if (cstat.transluscence) {

        if (cstat.transluscence_reversing) {
            settrans(TRANS_REVERSE);
        } else {
            settrans(TRANS_NORMAL);
        }
    }

    xoff = (int32_t)((int8_t )((tiles[tilenum].animFlags>>8)&255))+((int32_t)tspr->xoffset);
    yoff = (int32_t)((int8_t )((tiles[tilenum].animFlags>>16)&255))+((int32_t)tspr->yoffset);

    if (cstat.type == FACE_SPRITE) {
        if (yp <= (4<<8)) {
            return;
        }

        siz = divscale19(xdimenscale,yp);

        xv = mulscale16(((int32_t)tspr->xrepeat)<<16,xyaspect);

        spriteDim.width = tiles[tilenum].dim.width;
        spriteDim.height = tiles[tilenum].dim.height;

        xsiz = mulscale30(siz,xv * spriteDim.width);
        ysiz = mulscale14(siz,tspr->yrepeat * spriteDim.height);

        if (((tiles[tilenum].dim.width>>11) >= xsiz) || (spriteDim.height >= (ysiz>>1))) {
            return;    /* Watch out for divscale overflow */
        }

        x1 = xb-(xsiz>>1);
        if (spriteDim.width & 1) {
            x1 += mulscale31(siz,xv);    /* Odd xspans */
        }
        i = mulscale30(siz,xv*xoff);
        if (!cstat.x_flip) {
            x1 -= i;
        } else {
            x1 += i;
        }

        y1 = mulscale16(tspr->z-engine_state->posz,siz);
        y1 -= mulscale14(siz,tspr->yrepeat*yoff);
        y1 += (engine_state->horiz<<8)-ysiz;
        if (cstat.real_centered) {
            y1 += (ysiz>>1);
            if (spriteDim.height&1) {
                y1 += mulscale15(siz,tspr->yrepeat);    /* Odd yspans */
            }
        }

        x2 = x1+xsiz-1;
        y2 = y1+ysiz-1;
        if ((y1|255) >= (y2|255)) {
            return;
        }

        lx = (x1>>8)+1;
        if (lx < 0) {
            lx = 0;
        }
        rx = (x2>>8);
        if (rx >= xdimen) {
            rx = xdimen-1;
        }
        if (lx > rx) {
            return;
        }

        if (!sec->ceiling.flags.groudraw && !sec->ceiling.flags.parallaxing) {
            startum = engine_state->horiz+mulscale24(siz,sec->ceiling.z - engine_state->posz)-1;
        } else {
            startum = 0;
        }

        if (!sec->floor.flags.groudraw && !sec->floor.flags.parallaxing) {
            startdm = engine_state->horiz+mulscale24(siz,sec->floor.z - engine_state->posz)+1;
        } else {
            startdm = 0x7fffffff;
        }

        if ((y1>>8) > startum) {
            startum = (y1>>8);
        }
        if ((y2>>8) < startdm) startdm = (y2>>8);

        if (startum < -32768) {
            startum = -32768;
        }
        if (startdm > 32767) {
            startdm = 32767;
        }
        if (startum >= startdm) {
            return;
        }

        if (!cstat.x_flip) {
            linuminc = divscale24(spriteDim.width,xsiz);
            linum = mulscale8((lx<<8)-x1,linuminc);
        } else {
            linuminc = -divscale24(spriteDim.width,xsiz);
            linum = mulscale8((lx<<8)-x2,linuminc);
        }


        for (x=lx; x<=rx; x++) {
            uwall[x] = max(startumost[x+windowx1]-windowy1,(short)startum);
            dwall[x] = min(startdmost[x+windowx1]-windowy1,(short)startdm);
        }
        daclip = 0;
        for (i=smostwallcnt-1; i>=0; i--) {
            if (smostwalltype[i]&daclip) {
                continue;
            }

            j = smostwall[i];
            if ((pvWalls[j].screenSpaceCoo[0][VEC_COL] > rx) || (pvWalls[j].screenSpaceCoo[1][VEC_COL] < lx)) {
                continue;
            }

            if ((yp <= pvWalls[j].screenSpaceCoo[0][VEC_DIST]) && (yp <= pvWalls[j].screenSpaceCoo[1][VEC_DIST])) {
                continue;
            }

            if (spritewallfront(tspr,pvWalls[j].worldWallId) && ((yp <= pvWalls[j].screenSpaceCoo[0][VEC_DIST]) || (yp <= pvWalls[j].screenSpaceCoo[1][VEC_DIST]))) {
                continue;
            }

            dalx2 = max(pvWalls[j].screenSpaceCoo[0][VEC_COL],lx);
            darx2 = min(pvWalls[j].screenSpaceCoo[1][VEC_COL],rx);

            switch (smostwalltype[i]) {
                case 0:
                    if (dalx2 <= darx2) {
                        if ((dalx2 == lx) && (darx2 == rx)) {
                            return;
                        }
                        clearbufbyte(&dwall[dalx2],(darx2-dalx2+1)*sizeof(dwall[0]),0L);
                    }
                    break;
                case 1:
                    k = smoststart[i] - pvWalls[j].screenSpaceCoo[0][VEC_COL];
                    for (x=dalx2; x<=darx2; x++)
                        if (engine_state->smost[k+x] > uwall[x]) {
                            uwall[x] = engine_state->smost[k+x];
                        }
                    if ((dalx2 == lx) && (darx2 == rx)) {
                        daclip |= 1;
                    }
                    break;
                case 2:
                    k = smoststart[i] - pvWalls[j].screenSpaceCoo[0][VEC_COL];
                    for (x=dalx2; x<=darx2; x++)
                        if (engine_state->smost[k+x] < dwall[x]) {
                            dwall[x] = engine_state->smost[k+x];
                        }
                    if ((dalx2 == lx) && (darx2 == rx)) {
                        daclip |= 2;
                    }
                    break;
            }
        }

        if (uwall[rx] >= dwall[rx]) {
            for (x=lx; x<rx; x++)
                if (uwall[x] < dwall[x]) {
                    break;
                }
            if (x == rx) {
                return;
            }
        }

        /* sprite */
        if ((searchit >= 1) && (searchx >= lx) && (searchx <= rx))
            if ((searchy >= uwall[searchx]) && (searchy < dwall[searchx])) {
                searchsector = sectnum;
                searchwall = spritenum;
                searchstat = 3;
                searchit = 1;
            }

        z2 = tspr->z - ((yoff*tspr->yrepeat)<<2);
        if (cstat.real_centered) {
            z2 += ((spriteDim.height*tspr->yrepeat)<<1);
            if (spriteDim.height&1) {
                z2 += (tspr->yrepeat<<1);    /* Odd yspans */
            }
        }
        z1 = z2 - ((spriteDim.height*tspr->yrepeat)<<2);

        //*(uint16_t *)&globalorientation = 0;
        picnum = tilenum;
        if ((uint32_t)picnum >= (uint32_t)MAXTILES) {
            picnum = 0;
        }
        xpanning = 0L;
        ypanning = 0L;
        vis = engine_state->visibility;
        if (sec->visibility != 0) {
            vis = mulscale4(vis,(int32_t)((uint8_t )(sec->visibility+16)));
        }
        shiftval = tiles[picnum].dim_power_2.height;
        if (pow2long[shiftval] != tiles[picnum].dim.height) {
            shiftval++;
        }

        shiftval = 32-shiftval;
        yscale = divscale(512,tspr->yrepeat,shiftval-19);
        zd = (((engine_state->posz-z1)*yscale)<<8);
        if (cstat.y_flip) {
            yscale = -yscale;
            zd = (((engine_state->posz-z2)*yscale)<<8);
        }

        qinterpolatedown16((int32_t *)&lwall[lx],rx-lx+1,linum,linuminc);
        clearbuf(&swall[lx],rx-lx+1,mulscale19(yp,xdimscale));

        if (!cstat.transluscence) {
            maskwallscan(lx,rx,uwall,dwall,swall,lwall, zd,
                         xpanning, picnum, shade, shiftval, yscale, vis, pallete,
                         engine_state);
        } else {
            transmaskwallscan(lx, rx, zd,
                              xpanning, picnum, shade, shiftval, yscale, vis, pallete,
                              engine_state);
        }
    } else if (cstat.type == WALL_SPRITE) {
        if (cstat.x_flip) {
            xoff = -xoff;
        }
        if (cstat.y_flip) {
            yoff = -yoff;
        }

        spriteDim.width = tiles[tilenum].dim.width;
        spriteDim.height = tiles[tilenum].dim.height;

        xv = tspr->xrepeat*fixedPointCos((tspr->ang+2048+1536));
        yv = tspr->xrepeat*fixedPointSin((tspr->ang+2048+1536));
        i = (spriteDim.width >>1)+xoff;
        x1 = tspr->x-engine_state->posx-mulscale16(xv,i);
        x2 = x1+mulscale16(xv,spriteDim.width );
        y1 = tspr->y-engine_state->posy-mulscale16(yv,i);
        y2 = y1+mulscale16(yv,spriteDim.width );

        // Rotate: This is a standard cos sin 2D rotation matrix projection
        yp1 = dmulscale6(x1,cosviewingrangeglobalang,y1,sinviewingrangeglobalang);
        yp2 = dmulscale6(x2,cosviewingrangeglobalang,y2,sinviewingrangeglobalang);
        if ((yp1 <= 0) && (yp2 <= 0)) {
            return;
        }
        xp1 = dmulscale6(y1, fixedPointCos(engine_state->ang), -x1, fixedPointSin(engine_state->ang));
        xp2 = dmulscale6(y2, fixedPointCos(engine_state->ang), -x2, fixedPointSin(engine_state->ang));

        x1 += engine_state->posx;
        y1 += engine_state->posy;
        x2 += engine_state->posx;
        y2 += engine_state->posy;

        swapped = 0;
        if (dmulscale32(xp1,yp2,-xp2,yp1) >= 0) { /* If wall's NOT facing you */
            if (cstat.one_sided) return;
            
            i = xp1, xp1 = xp2, xp2 = i;
            i = yp1, yp1 = yp2, yp2 = i;
            i = x1, x1 = x2, x2 = i;
            i = y1, y1 = y2, y2 = i;
            swapped = 1;
        }

        if (xp1 >= -yp1) {
            if (xp1 > yp1) {
                return;
            }

            if (yp1 == 0) {
                return;
            }
            pvWalls[MAXWALLSB-1].screenSpaceCoo[0][VEC_COL] = halfxdimen + scale(xp1,halfxdimen,yp1);
            if (xp1 >= 0) {
                pvWalls[MAXWALLSB-1].screenSpaceCoo[0][VEC_COL]++;    /* Fix for SIGNED divide */
            }
            if (pvWalls[MAXWALLSB-1].screenSpaceCoo[0][VEC_COL] >= xdimen) {
                pvWalls[MAXWALLSB-1].screenSpaceCoo[0][VEC_COL] = xdimen-1;
            }
            pvWalls[MAXWALLSB-1].screenSpaceCoo[0][VEC_DIST] = yp1;
        } else {
            if (xp2 < -yp2) {
                return;
            }
            pvWalls[MAXWALLSB-1].screenSpaceCoo[0][VEC_COL] = 0;
            i = yp1-yp2+xp1-xp2;
            if (i == 0) {
                return;
            }
            pvWalls[MAXWALLSB-1].screenSpaceCoo[0][VEC_DIST] = yp1 + scale(yp2-yp1,xp1+yp1,i);
        }
        if (xp2 <= yp2) {
            if (xp2 < -yp2) {
                return;
            }

            if (yp2 == 0) {
                return;
            }
            pvWalls[MAXWALLSB-1].screenSpaceCoo[1][VEC_COL] = halfxdimen + scale(xp2,halfxdimen,yp2) - 1;
            if (xp2 >= 0) {
                pvWalls[MAXWALLSB-1].screenSpaceCoo[1][VEC_COL]++;    /* Fix for SIGNED divide */
            }
            if (pvWalls[MAXWALLSB-1].screenSpaceCoo[1][VEC_COL] >= xdimen) {
                pvWalls[MAXWALLSB-1].screenSpaceCoo[1][VEC_COL] = xdimen-1;
            }
            pvWalls[MAXWALLSB-1].screenSpaceCoo[1][VEC_DIST] = yp2;
        } else {
            if (xp1 > yp1) {
                return;
            }

            pvWalls[MAXWALLSB-1].screenSpaceCoo[1][VEC_COL] = xdimen-1;
            i = xp2-xp1+yp1-yp2;
            if (i == 0) {
                return;
            }
            pvWalls[MAXWALLSB-1].screenSpaceCoo[1][VEC_DIST] = yp1 + scale(yp2-yp1,yp1-xp1,i);
        }

        if ((pvWalls[MAXWALLSB-1].screenSpaceCoo[0][VEC_DIST] < 256) || (pvWalls[MAXWALLSB-1].screenSpaceCoo[1][VEC_DIST] < 256) || (pvWalls[MAXWALLSB-1].screenSpaceCoo[0][VEC_COL] > pvWalls[MAXWALLSB-1].screenSpaceCoo[1][VEC_COL])) {
            return;
        }

        topinc = -mulscale10(yp1,spriteDim.width);
        top = (((mulscale10(xp1,xdimen) - mulscale9(pvWalls[MAXWALLSB-1].screenSpaceCoo[0][VEC_COL]-halfxdimen,yp1))*spriteDim.width)>>3);
        botinc = ((yp2-yp1)>>8);
        bot = mulscale11(xp1-xp2,xdimen) + mulscale2(pvWalls[MAXWALLSB-1].screenSpaceCoo[0][VEC_COL]-halfxdimen,botinc);

        j = pvWalls[MAXWALLSB-1].screenSpaceCoo[1][VEC_COL]+3;
        z = mulscale20(top,krecip(bot));
        lwall[pvWalls[MAXWALLSB-1].screenSpaceCoo[0][VEC_COL]] = (z>>8);
        for (x=pvWalls[MAXWALLSB-1].screenSpaceCoo[0][VEC_COL]+4; x<=j; x+=4) {
            top += topinc;
            bot += botinc;
            zz = z;
            z = mulscale20(top,krecip(bot));
            lwall[x] = (z>>8);
            i = ((z+zz)>>1);
            lwall[x-2] = (i>>8);
            lwall[x-3] = ((i+zz)>>9);
            lwall[x-1] = ((i+z)>>9);
        }

        if (lwall[pvWalls[MAXWALLSB-1].screenSpaceCoo[0][VEC_COL]] < 0) {
            lwall[pvWalls[MAXWALLSB-1].screenSpaceCoo[0][VEC_COL]] = 0;
        }
        if (lwall[pvWalls[MAXWALLSB-1].screenSpaceCoo[1][VEC_COL]] >= spriteDim.width) {
            lwall[pvWalls[MAXWALLSB-1].screenSpaceCoo[1][VEC_COL]] = spriteDim.width-1;
        }

        if ((swapped^(cstat.x_flip)) > 0) {
            j = spriteDim.width-1;
            for (x=pvWalls[MAXWALLSB-1].screenSpaceCoo[0][VEC_COL]; x<=pvWalls[MAXWALLSB-1].screenSpaceCoo[1][VEC_COL]; x++) {
                lwall[x] = j-lwall[x];
            }
        }

        pvWalls[MAXWALLSB-1].cameraSpaceCoo[0][VEC_X] = xp1 ;
        pvWalls[MAXWALLSB-1].cameraSpaceCoo[0][VEC_Y] = yp1 ;
        pvWalls[MAXWALLSB-1].cameraSpaceCoo[1][VEC_X] = xp2 ;
        pvWalls[MAXWALLSB-1].cameraSpaceCoo[1][VEC_Y] = yp2 ;


        hplc = divscale19(xdimenscale,pvWalls[MAXWALLSB-1].screenSpaceCoo[0][VEC_DIST]);
        hinc = divscale19(xdimenscale,pvWalls[MAXWALLSB-1].screenSpaceCoo[1][VEC_DIST]);
        hinc = (hinc-hplc)/(pvWalls[MAXWALLSB-1].screenSpaceCoo[1][VEC_COL]-pvWalls[MAXWALLSB-1].screenSpaceCoo[0][VEC_COL]+1);

        z2 = tspr->z - ((yoff*tspr->yrepeat)<<2);
        if (cstat.real_centered) {
            z2 += ((spriteDim.height*tspr->yrepeat)<<1);
            if (spriteDim.height&1) {
                z2 += (tspr->yrepeat<<1);    /* Odd yspans */
            }
        }
        z1 = z2 - ((spriteDim.height*tspr->yrepeat)<<2);

        //*(uint16_t *)&globalorientation = 0;
        picnum = tilenum;
        if ((uint32_t)picnum >= (uint32_t)MAXTILES) {
            picnum = 0;
        }
        xpanning = 0L;
        ypanning = 0L;
        vis = engine_state->visibility;
        if (sec->visibility != 0) {
            vis = mulscale4(vis,(int32_t)((uint8_t )(sec->visibility+16)));
        }
        shiftval = tiles[picnum].dim_power_2.height;
        if (pow2long[shiftval] != tiles[picnum].dim.height) {
            shiftval++;
        }
        shiftval = 32-shiftval;
        yscale = divscale(512,tspr->yrepeat,shiftval-19);
        zd = (((engine_state->posz-z1)*yscale)<<8);
        if (cstat.y_flip) {
            yscale = -yscale;
            zd = (((engine_state->posz-z2)*yscale)<<8);
        }

        if (!sec->ceiling.flags.parallaxing && (z1 < sec->ceiling.z)) {
            z1 = sec->ceiling.z;
        }
        if (!sec->floor.flags.parallaxing && (z2 > sec->floor.z)) {
            z2 = sec->floor.z;
        }

        owallmost(uwall, (int32_t)(MAXWALLSB-1), z1 - engine_state->posz, engine_state);
        owallmost(dwall, (int32_t)(MAXWALLSB-1), z2 - engine_state->posz, engine_state);
        for (i=pvWalls[MAXWALLSB-1].screenSpaceCoo[0][VEC_COL]; i<=pvWalls[MAXWALLSB-1].screenSpaceCoo[1][VEC_COL]; i++) {
            swall[i] = (krecip(hplc)<<2);
            hplc += hinc;
        }

        for (i=smostwallcnt-1; i>=0; i--) {
            j = smostwall[i];

            if ((pvWalls[j].screenSpaceCoo[0][VEC_COL] > pvWalls[MAXWALLSB-1].screenSpaceCoo[1][VEC_COL]) || (pvWalls[j].screenSpaceCoo[1][VEC_COL] < pvWalls[MAXWALLSB-1].screenSpaceCoo[0][VEC_COL])) {
                continue;
            }

            dalx2 = pvWalls[j].screenSpaceCoo[0][VEC_COL];
            darx2 = pvWalls[j].screenSpaceCoo[1][VEC_COL];
            if (max(pvWalls[MAXWALLSB-1].screenSpaceCoo[0][VEC_DIST],pvWalls[MAXWALLSB-1].screenSpaceCoo[1][VEC_DIST]) > min(pvWalls[j].screenSpaceCoo[0][VEC_DIST],pvWalls[j].screenSpaceCoo[1][VEC_DIST])) {
                if (min(pvWalls[MAXWALLSB-1].screenSpaceCoo[0][VEC_DIST],pvWalls[MAXWALLSB-1].screenSpaceCoo[1][VEC_DIST]) > max(pvWalls[j].screenSpaceCoo[0][VEC_DIST],pvWalls[j].screenSpaceCoo[1][VEC_DIST])) {
                    x = 0x80000000;
                } else {
                    x = pvWalls[j].worldWallId;
                    xp1 = wall[x].x;
                    yp1 = wall[x].y;
                    x = wall[x].point2;
                    xp2 = wall[x].x;
                    yp2 = wall[x].y;

                    z1 = (xp2-xp1)*(y1-yp1) - (yp2-yp1)*(x1-xp1);
                    z2 = (xp2-xp1)*(y2-yp1) - (yp2-yp1)*(x2-xp1);
                    if ((z1^z2) >= 0) {
                        x = (z1+z2);
                    } else {
                        z1 = (x2-x1)*(yp1-y1) - (y2-y1)*(xp1-x1);
                        z2 = (x2-x1)*(yp2-y1) - (y2-y1)*(xp2-x1);

                        if ((z1^z2) >= 0) {
                            x = -(z1+z2);
                        } else {
                            if ((xp2-xp1)*(tspr->y-yp1) == (tspr->x-xp1)*(yp2-yp1)) {
                                if (wall[pvWalls[j].worldWallId].nextsector == tspr->sectnum) {
                                    x = 0x80000000;
                                } else {
                                    x = 0x7fffffff;
                                }
                            } else {
                                /* INTERSECTION! */
                                x = (xp1-engine_state->posx) + scale(xp2-xp1,z1,z1-z2);
                                y = (yp1-engine_state->posy) + scale(yp2-yp1,z1,z1-z2);

                                yp1 = dmulscale14(x, fixedPointCos(engine_state->ang), y, fixedPointSin(engine_state->ang));
                                if (yp1 > 0) {
                                    xp1 = dmulscale14(y, fixedPointCos(engine_state->ang), -x, fixedPointSin(engine_state->ang));

                                    x = halfxdimen + scale(xp1,halfxdimen,yp1);
                                    if (xp1 >= 0) {
                                        x++;    /* Fix for SIGNED divide */
                                    }

                                    if (z1 < 0) {
                                        if (dalx2 < x) {
                                            dalx2 = x;
                                        }
                                    } else {
                                        if (darx2 > x) {
                                            darx2 = x;
                                        }
                                    }
                                    x = 0x80000001;
                                } else {
                                    x = 0x7fffffff;
                                }
                            }
                        }
                    }
                }
                if (x < 0) {
                    if (dalx2 < pvWalls[MAXWALLSB-1].screenSpaceCoo[0][VEC_COL]) {
                        dalx2 = pvWalls[MAXWALLSB-1].screenSpaceCoo[0][VEC_COL];
                    }
                    if (darx2 > pvWalls[MAXWALLSB-1].screenSpaceCoo[1][VEC_COL]) {
                        darx2 = pvWalls[MAXWALLSB-1].screenSpaceCoo[1][VEC_COL];
                    }
                    switch (smostwalltype[i]) {
                        case 0:
                            if (dalx2 <= darx2) {
                                if ((dalx2 == pvWalls[MAXWALLSB-1].screenSpaceCoo[0][VEC_COL]) && (darx2 == pvWalls[MAXWALLSB-1].screenSpaceCoo[1][VEC_COL])) {
                                    return;
                                }
                                clearbufbyte(&dwall[dalx2],(darx2-dalx2+1)*sizeof(dwall[0]),0L);
                            }
                            break;
                        case 1:
                            k = smoststart[i] - pvWalls[j].screenSpaceCoo[0][VEC_COL];
                            for (x=dalx2; x<=darx2; x++)
                                if (engine_state->smost[k+x] > uwall[x]) {
                                    uwall[x] = engine_state->smost[k+x];
                                }
                            break;
                        case 2:
                            k = smoststart[i] - pvWalls[j].screenSpaceCoo[0][VEC_COL];
                            for (x=dalx2; x<=darx2; x++)
                                if (engine_state->smost[k+x] < dwall[x]) {
                                    dwall[x] = engine_state->smost[k+x];
                                }
                            break;
                    }
                }
            }
        }

        /* sprite */
        if ((searchit >= 1) && (searchx >= pvWalls[MAXWALLSB-1].screenSpaceCoo[0][VEC_COL]) && (searchx <= pvWalls[MAXWALLSB-1].screenSpaceCoo[1][VEC_COL]))
            if ((searchy >= uwall[searchx]) && (searchy <= dwall[searchx])) {
                searchsector = sectnum;
                searchwall = spritenum;
                searchstat = 3;
                searchit = 1;
            }

        if (!cstat.transluscence) {
            maskwallscan(pvWalls[MAXWALLSB-1].screenSpaceCoo[0][VEC_COL],
                         pvWalls[MAXWALLSB-1].screenSpaceCoo[1][VEC_COL],
                         uwall,dwall,swall,lwall, zd,
                         xpanning, picnum, shade, shiftval, yscale, vis, pallete,
                         engine_state);
        } else {
            transmaskwallscan(pvWalls[MAXWALLSB-1].screenSpaceCoo[0][VEC_COL],
                              pvWalls[MAXWALLSB-1].screenSpaceCoo[1][VEC_COL],
                              zd, xpanning, picnum, shade, shiftval, yscale, vis, pallete,
                              engine_state);
        }
    } else if (cstat.type == FLOOR_SPRITE) {
        if (cstat.one_sided)
            if ((engine_state->posz > tspr->z) == (!cstat.y_flip)) return;

        if (cstat.x_flip) {
            xoff = -xoff;
        }
        if (cstat.y_flip) {
            yoff = -yoff;
        }
        spriteDim.width = tiles[tilenum].dim.width;
        spriteDim.height = tiles[tilenum].dim.height;

        /* Rotate center point */
        dax = tspr->x - engine_state->posx;
        day = tspr->y - engine_state->posy;
        rzi[0] = dmulscale10(fixedPointCos(engine_state->ang), dax, fixedPointSin(engine_state->ang), day);
        rxi[0] = dmulscale10(fixedPointCos(engine_state->ang), day, -fixedPointSin(engine_state->ang), dax);

        /* Get top-left corner */
        i = ((tspr->ang + 2048 - engine_state->ang) & 2047);
        cosang = fixedPointCos(i);
        sinang = fixedPointSin(i);
        dax = ((spriteDim.width>>1)+xoff)*tspr->xrepeat;
        day = ((spriteDim.height>>1)+yoff)*tspr->yrepeat;
        rzi[0] += dmulscale12(sinang,dax,cosang,day);
        rxi[0] += dmulscale12(sinang,day,-cosang,dax);

        /* Get other 3 corners */
        dax = spriteDim.width*tspr->xrepeat;
        day = spriteDim.height*tspr->yrepeat;
        rzi[1] = rzi[0]-mulscale12(sinang,dax);
        rxi[1] = rxi[0]+mulscale12(cosang,dax);
        dax = -mulscale12(cosang,day);
        day = -mulscale12(sinang,day);
        rzi[2] = rzi[1]+dax;
        rxi[2] = rxi[1]+day;
        rzi[3] = rzi[0]+dax;
        rxi[3] = rxi[0]+day;

        /* Put all points on same z */
        ryi[0] = scale((tspr->z - engine_state->posz),yxaspect,320<<8);
        if (ryi[0] == 0) {
            return;
        }
        ryi[1] = ryi[2] = ryi[3] = ryi[0];

        if (!cstat.x_flip) {
            z = 0;
            z1 = 1;
            z2 = 3;
        } else {
            z = 1;
            z1 = 0;
            z2 = 2;
        }

        dax = rzi[z1]-rzi[z];
        day = rxi[z1]-rxi[z];
        bot = dmulscale8(dax,dax,day,day);
        if (((klabs(dax)>>13) >= bot) || ((klabs(day)>>13) >= bot)) {
            return;
        }
        g_x1 = divscale18(dax,bot);
        g_x2 = divscale18(day,bot);

        dax = rzi[z2]-rzi[z];
        day = rxi[z2]-rxi[z];
        bot = dmulscale8(dax,dax,day,day);
        if (((klabs(dax)>>13) >= bot) || ((klabs(day)>>13) >= bot)) {
            return;
        }
        g_y1 = divscale18(dax,bot);
        g_y2 = divscale18(day,bot);

        /* Calculate globals for hline texture mapping function */
        xpanning = (rxi[z]<<12);
        ypanning = (rzi[z]<<12);
        zd = (ryi[z]<<12);

        rzi[0] = mulscale16(rzi[0],viewingrange);
        rzi[1] = mulscale16(rzi[1],viewingrange);
        rzi[2] = mulscale16(rzi[2],viewingrange);
        rzi[3] = mulscale16(rzi[3],viewingrange);

        if (ryi[0] < 0) { /* If ceilsprite is above you, reverse order of points */
            i = rxi[1];
            rxi[1] = rxi[3];
            rxi[3] = i;
            i = rzi[1];
            rzi[1] = rzi[3];
            rzi[3] = i;
        }


        /* Clip polygon in 3-space */
        npoints = 4;

        /* Clip edge 1 */
        npoints2 = 0;
        zzsgn = rxi[0]+rzi[0];
        for (z=0; z<npoints; z++) {
            zz = z+1;
            if (zz == npoints) {
                zz = 0;
            }
            zsgn = zzsgn;
            zzsgn = rxi[zz]+rzi[zz];
            if (zsgn >= 0) {
                rxi2[npoints2] = rxi[z];
                ryi2[npoints2] = ryi[z];
                rzi2[npoints2] = rzi[z];
                npoints2++;
            }
            if ((zsgn^zzsgn) < 0) {
                t = divscale30(zsgn,zsgn-zzsgn);
                rxi2[npoints2] = rxi[z] + mulscale30(t,rxi[zz]-rxi[z]);
                ryi2[npoints2] = ryi[z] + mulscale30(t,ryi[zz]-ryi[z]);
                rzi2[npoints2] = rzi[z] + mulscale30(t,rzi[zz]-rzi[z]);
                npoints2++;
            }
        }
        if (npoints2 <= 2) {
            return;
        }

        /* Clip edge 2 */
        npoints = 0;
        zzsgn = rxi2[0]-rzi2[0];
        for (z=0; z<npoints2; z++) {
            zz = z+1;
            if (zz == npoints2) {
                zz = 0;
            }
            zsgn = zzsgn;
            zzsgn = rxi2[zz]-rzi2[zz];
            if (zsgn <= 0) {
                rxi[npoints] = rxi2[z];
                ryi[npoints] = ryi2[z];
                rzi[npoints] = rzi2[z];
                npoints++;
            }
            if ((zsgn^zzsgn) < 0) {
                t = divscale30(zsgn,zsgn-zzsgn);
                rxi[npoints] = rxi2[z] + mulscale30(t,rxi2[zz]-rxi2[z]);
                ryi[npoints] = ryi2[z] + mulscale30(t,ryi2[zz]-ryi2[z]);
                rzi[npoints] = rzi2[z] + mulscale30(t,rzi2[zz]-rzi2[z]);
                npoints++;
            }
        }
        if (npoints <= 2) {
            return;
        }

        /* Clip edge 3 */
        npoints2 = 0;
        zzsgn = ryi[0]*halfxdimen + (rzi[0]*(engine_state->horiz-0));
        for (z=0; z<npoints; z++) {
            zz = z+1;
            if (zz == npoints) {
                zz = 0;
            }
            zsgn = zzsgn;
            zzsgn = ryi[zz]*halfxdimen + (rzi[zz]*(engine_state->horiz-0));
            if (zsgn >= 0) {
                rxi2[npoints2] = rxi[z];
                ryi2[npoints2] = ryi[z];
                rzi2[npoints2] = rzi[z];
                npoints2++;
            }
            if ((zsgn^zzsgn) < 0) {
                t = divscale30(zsgn,zsgn-zzsgn);
                rxi2[npoints2] = rxi[z] + mulscale30(t,rxi[zz]-rxi[z]);
                ryi2[npoints2] = ryi[z] + mulscale30(t,ryi[zz]-ryi[z]);
                rzi2[npoints2] = rzi[z] + mulscale30(t,rzi[zz]-rzi[z]);
                npoints2++;
            }
        }
        if (npoints2 <= 2) {
            return;
        }

        /* Clip edge 4 */
        npoints = 0;
        zzsgn = ryi2[0]*halfxdimen + (rzi2[0]*(engine_state->horiz-ydimen));
        for (z=0; z<npoints2; z++) {
            zz = z+1;
            if (zz == npoints2) {
                zz = 0;
            }
            zsgn = zzsgn;
            zzsgn = ryi2[zz]*halfxdimen + (rzi2[zz]*(engine_state->horiz-ydimen));
            if (zsgn <= 0) {
                rxi[npoints] = rxi2[z];
                ryi[npoints] = ryi2[z];
                rzi[npoints] = rzi2[z];
                npoints++;
            }
            if ((zsgn^zzsgn) < 0) {
                t = divscale30(zsgn,zsgn-zzsgn);
                rxi[npoints] = rxi2[z] + mulscale30(t,rxi2[zz]-rxi2[z]);
                ryi[npoints] = ryi2[z] + mulscale30(t,ryi2[zz]-ryi2[z]);
                rzi[npoints] = rzi2[z] + mulscale30(t,rzi2[zz]-rzi2[z]);
                npoints++;
            }
        }
        if (npoints <= 2) {
            return;
        }

        /* Project onto screen */
        lpoint = -1;
        lmax = 0x7fffffff;
        rpoint = -1;
        rmax = 0x80000000;
        for (z=0; z<npoints; z++) {
            xsi[z] = scale(rxi[z],xdimen<<15,rzi[z]) + (xdimen<<15);
            ysi[z] = scale(ryi[z],xdimen<<15,rzi[z]) + (engine_state->horiz<<16);
            if (xsi[z] < 0) {
                xsi[z] = 0;
            }
            if (xsi[z] > (xdimen<<16)) {
                xsi[z] = (xdimen<<16);
            }
            if (ysi[z] < ((int32_t)0<<16)) {
                ysi[z] = ((int32_t)0<<16);
            }
            if (ysi[z] > ((int32_t)ydimen<<16)) {
                ysi[z] = ((int32_t)ydimen<<16);
            }
            if (xsi[z] < lmax) {
                lmax = xsi[z], lpoint = z;
            }
            if (xsi[z] > rmax) {
                rmax = xsi[z], rpoint = z;
            }
        }

        /* Get uwall arrays */
        for (z=lpoint; z!=rpoint; z=zz) {
            zz = z+1;
            if (zz == npoints) {
                zz = 0;
            }

            dax1 = ((xsi[z]+65535)>>16);
            dax2 = ((xsi[zz]+65535)>>16);
            if (dax2 > dax1) {
                yinc = divscale16(ysi[zz]-ysi[z],xsi[zz]-xsi[z]);
                y = ysi[z] + mulscale16((dax1<<16)-xsi[z],yinc);
                qinterpolatedown16short((int32_t *)(&uwall[dax1]),dax2-dax1,y,yinc);
            }
        }

        /* Get dwall arrays */
        for (; z!=lpoint; z=zz) {
            zz = z+1;
            if (zz == npoints) {
                zz = 0;
            }

            dax1 = ((xsi[zz]+65535)>>16);
            dax2 = ((xsi[z]+65535)>>16);
            if (dax2 > dax1) {
                yinc = divscale16(ysi[zz]-ysi[z],xsi[zz]-xsi[z]);
                y = ysi[zz] + mulscale16((dax1<<16)-xsi[zz],yinc);
                qinterpolatedown16short((int32_t *)(&dwall[dax1]),dax2-dax1,y,yinc);
            }
        }


        lx = ((lmax+65535)>>16);
        rx = ((rmax+65535)>>16);
        for (x=lx; x<=rx; x++) {
            uwall[x] = max(uwall[x],startumost[x+windowx1]-windowy1);
            dwall[x] = min(dwall[x],startdmost[x+windowx1]-windowy1);
        }

        /* Additional uwall/dwall clipping goes here */
        for (i=smostwallcnt-1; i>=0; i--) {
            j = smostwall[i];
            if ((pvWalls[j].screenSpaceCoo[0][VEC_COL] > rx) || (pvWalls[j].screenSpaceCoo[1][VEC_COL] < lx)) {
                continue;
            }
            if ((yp <= pvWalls[j].screenSpaceCoo[0][VEC_DIST]) && (yp <= pvWalls[j].screenSpaceCoo[1][VEC_DIST])) {
                continue;
            }

            /* if (spritewallfront(tspr,thewall[j]) == 0) */
            x = pvWalls[j].worldWallId;
            xp1 = wall[x].x;
            yp1 = wall[x].y;
            x = wall[x].point2;
            xp2 = wall[x].x;
            yp2 = wall[x].y;
            x = (xp2-xp1)*(tspr->y-yp1)-(tspr->x-xp1)*(yp2-yp1);
            if ((yp > pvWalls[j].screenSpaceCoo[0][VEC_DIST]) && (yp > pvWalls[j].screenSpaceCoo[1][VEC_DIST])) {
                x = -1;
            }
            if ((x >= 0) && ((x != 0) || (wall[pvWalls[j].worldWallId].nextsector != tspr->sectnum))) {
                continue;
            }

            dalx2 = max(pvWalls[j].screenSpaceCoo[0][VEC_COL],lx);
            darx2 = min(pvWalls[j].screenSpaceCoo[1][VEC_COL],rx);

            switch (smostwalltype[i]) {
                case 0:
                    if (dalx2 <= darx2) {
                        if ((dalx2 == lx) && (darx2 == rx)) {
                            return;
                        }
                        clearbufbyte(&dwall[dalx2],(darx2-dalx2+1)*sizeof(dwall[0]),0L);
                    }
                    break;
                case 1:
                    k = smoststart[i] - pvWalls[j].screenSpaceCoo[0][VEC_COL];
                    for (x=dalx2; x<=darx2; x++)
                        if (engine_state->smost[k+x] > uwall[x]) {
                            uwall[x] = engine_state->smost[k+x];
                        }
                    break;
                case 2:
                    k = smoststart[i] - pvWalls[j].screenSpaceCoo[0][VEC_COL];
                    for (x=dalx2; x<=darx2; x++)
                        if (engine_state->smost[k+x] < dwall[x]) {
                            dwall[x] = engine_state->smost[k+x];
                        }
                    break;
            }
        }

        /* sprite */
        if ((searchit >= 1) && (searchx >= lx) && (searchx <= rx))
            if ((searchy >= uwall[searchx]) && (searchy <= dwall[searchx])) {
                searchsector = sectnum;
                searchwall = spritenum;
                searchstat = 3;
                searchit = 1;
            }

        picnum = tilenum;
        if ((uint32_t)picnum >= (uint32_t)MAXTILES) {
            picnum = 0;
        }

        TILE_MakeAvailable(picnum);

        setgotpic(picnum);

        vis = mulscale16(engine_state->hisibility,viewingrange);
        if (sec->visibility != 0) {
            vis = mulscale4(vis,(int32_t)((uint8_t )(sec->visibility+16)));
        }

        x = tiles[picnum].dim_power_2.width;
        y = tiles[picnum].dim_power_2.height;

        if (pow2long[x] != spriteDim.width) {
            x++;
            g_x1 = mulscale(g_x1,spriteDim.width,x);
            g_x2 = mulscale(g_x2,spriteDim.width,x);
        }

        dax = xpanning;
        day = ypanning;
        xpanning = -dmulscale6(g_x1,day,g_x2,dax);
        ypanning = -dmulscale6(g_y1,day,g_y2,dax);

        g_x2 = mulscale16(g_x2,viewingrange);
        g_y2 = mulscale16(g_y2,viewingrange);
        zd = mulscale16(zd, viewingrangerecip);

        g_x1 = (g_x1-g_x2)*halfxdimen;
        g_y1 = (g_y1-g_y2)*halfxdimen;

        if (!cstat.transluscence) {
            msethlineshift(x,y);
        } else {
            tsethlineshift(x,y);
        }

        /* Draw it! */
        // TODO: remove SectorFlags casting
        ceilspritescan(lx, rx-1, zd, xpanning, ypanning, g_x1, g_y1, g_x2, g_y2, *(SectorFlags *)&cstat, shade, vis, pallete, tiles[picnum].data, engine_state);
    }
}


void DrawMirror(void)
{
    int32_t i, dy, p;
    char buffer[MAXWALLS];
    
    // This seems to be bleeding the mirror by 1 pixel when
    // the left or right border are on screen. I am not sure
    // why this is done. I didn't notice no difference when
    // this as was removed.
    if (mirrorsx1 > 0) mirrorsx1--;
    if (mirrorsx2 < windowx2-windowx1-1) mirrorsx2++;
    
    // Wall is not visible, skip drawing
    if (mirrorsx2 < mirrorsx1) return;
    
    p = frameplace+ylookup[windowy1+mirrorsy1]+windowx1+mirrorsx1;
    i = windowx2-windowx1-mirrorsx2-mirrorsx1;
    mirrorsx2 -= mirrorsx1;
    // FIX_00085: Optimized Video driver. FPS increases by +20%.
    for (dy=mirrorsy2-mirrorsy1-1; dy>=0; dy--) {
        copybufbyte((void *)(p), buffer, mirrorsx2 + 1);
        buffer[mirrorsx2] = buffer[mirrorsx2 - 1];
        copybufreverse(&buffer[mirrorsx2], (void *)(p+i), mirrorsx2 + 1);
        p += ylookup[1];
        faketimerhandler();
    }
}


/*
     FCS: Draw every transparent sprites in Back To Front Order. Also draw decals on the walls...
 */
void drawmasks(EngineState *engine_state, bool draw_mirror)
{
    int32_t i, j, k, l, gap, xs, ys, xp, yp, yoff, yspan;
    /* int32_t zs, zp; */

    static int32_t spritesx[MAXSPRITESONSCREEN];
    static int32_t spritesy[MAXSPRITESONSCREEN+1];
    static int32_t spritesz[MAXSPRITESONSCREEN];

    //Copy sprite address in a sprite proxy structure (pointers are easier to re-arrange than structs).
    for (i=(engine_state->spritesortcnt)-1; i>=0; i--) {
        tspriteptr[i] = &tsprite[i];
    }


    //Generate screenspace coordinate (X column and Y distance).
    for (i=(engine_state->spritesortcnt)-1; i>=0; i--) {
        //Translate and rotate the sprite in Camera space coordinate.
        xs = tspriteptr[i]->x - engine_state->posx;
        ys = tspriteptr[i]->y - engine_state->posy;
        yp = dmulscale6(xs,cosviewingrangeglobalang,ys,sinviewingrangeglobalang);

        if (yp > (4<<8)) {
            xp = dmulscale6(ys, fixedPointCos(engine_state->ang), -xs, fixedPointSin(engine_state->ang));
            spritesx[i] = scale(xp+yp,xdimen<<7,yp);
        } else if (tspriteptr[i]->flags.type == FACE_SPRITE) {
            engine_state->spritesortcnt--;  /* Delete face sprite if on wrong side! */
            //Move the sprite at the end of the array and decrease array length.
            if (i != engine_state->spritesortcnt) {
                tspriteptr[i] = tspriteptr[engine_state->spritesortcnt];
                spritesx[i] = spritesx[engine_state->spritesortcnt];
                spritesy[i] = spritesy[engine_state->spritesortcnt];
            }
            continue;
        }
        spritesy[i] = yp;
    }

    //FCS: Bubble sort ?! REally ?!?!?
    gap = 1;
    while (gap < engine_state->spritesortcnt) {
        gap = (gap<<1)+1;
    }
    for (gap>>=1; gap>0; gap>>=1)   /* Sort sprite list */
        for (i=0; i<engine_state->spritesortcnt-gap; i++)
            for (l=i; l>=0; l-=gap) {
                if (spritesy[l] <= spritesy[l+gap]) {
                    break;
                }

                swaplong((int32_t *)&tspriteptr[l],(int32_t *)&tspriteptr[l+gap]);
                swaplong(&spritesx[l],&spritesx[l+gap]);
                swaplong(&spritesy[l],&spritesy[l+gap]);
            }

    if (engine_state->spritesortcnt > 0) {
        spritesy[engine_state->spritesortcnt] = (spritesy[engine_state->spritesortcnt-1]^1);
    }

    ys = spritesy[0];
    i = 0;
    for (j=1; j<=engine_state->spritesortcnt; j++) {
        if (spritesy[j] == ys) {
            continue;
        }

        ys = spritesy[j];
        if (j > i+1) {
            for (k=i; k<j; k++) {
                spritesz[k] = tspriteptr[k]->z;
                if (tspriteptr[k]->flags.type != FLOOR_SPRITE) {
                    yoff = (int32_t)((int8_t )((tiles[tspriteptr[k]->picnum].animFlags>>16)&255))+((int32_t)tspriteptr[k]->yoffset);
                    spritesz[k] -= ((yoff*tspriteptr[k]->yrepeat)<<2);
                    yspan = (tiles[tspriteptr[k]->picnum].dim.height*tspriteptr[k]->yrepeat<<2);
                    if (!tspriteptr[k]->flags.real_centered) {
                        spritesz[k] -= (yspan>>1);
                    }
                    if (klabs(spritesz[k] - engine_state->posz) < (yspan>>1)) {
                        spritesz[k] = engine_state->posz;
                    }
                }
            }
            for (k=i+1; k<j; k++)
                for (l=i; l<k; l++)
                    if (klabs(spritesz[k]-engine_state->posz) < klabs(spritesz[l]-engine_state->posz)) {
                        swaplong((int32_t *)&tspriteptr[k],(int32_t *)&tspriteptr[l]);
                        swaplong(&spritesx[k],&spritesx[l]);
                        swaplong(&spritesy[k],&spritesy[l]);
                        swaplong(&spritesz[k],&spritesz[l]);
                    }
            for (k=i+1; k<j; k++)
                for (l=i; l<k; l++)
                    if (tspriteptr[k]->statnum < tspriteptr[l]->statnum) {
                        swaplong((int32_t *)&tspriteptr[k],(int32_t *)&tspriteptr[l]);
                        swaplong(&spritesx[k],&spritesx[l]);
                        swaplong(&spritesy[k],&spritesy[l]);
                    }
        }
        i = j;
    }

    while ((engine_state->spritesortcnt > 0) && (engine_state->maskwallcnt > 0)) { /* While BOTH > 0 */
        j = engine_state->maskwall[engine_state->maskwallcnt-1];
        if (spritewallfront(tspriteptr[engine_state->spritesortcnt-1],pvWalls[j].worldWallId) == 0) {
            drawsprite(engine_state, spritesx, spritesy);
        } else {
            /*
            // Check to see if any sprites behind the masked wall...
            k = -1;
            gap = 0;
            for(i=engine_state->spritesortcnt-2; i>=0; i--) {
                if ((pvWalls[j].screenSpaceCoo[0][VEC_COL] <= (spritesx[i]>>8)) && ((spritesx[i]>>8) <= pvWalls[j].screenSpaceCoo[1][VEC_COL])) {
                    if (spritewallfront(tspriteptr[i],pvWalls[j].worldWallId) == 0) {
                        drawsprite(i, spritesx, spritesy);
                        tspriteptr[i]->owner = -1;
                        k = i;
                        gap++;
                    }
                }
            }

            if (k >= 0)  // remove holes in sprite list
            {
                for(i=k; i<engine_state->spritesortcnt; i++)
                    if (tspriteptr[i]->owner >= 0)
                    {
                        if (i > k)
                        {
                            tspriteptr[k] = tspriteptr[i];
                            spritesx[k] = spritesx[i];
                            spritesy[k] = spritesy[i];
                        }
                        k++;
                    }
                engine_state->spritesortcnt -= gap;
            }
            */

            /* finally safe to draw the masked wall */
            drawmaskwall(engine_state);
        }
    }
    while (engine_state->spritesortcnt > 0) {
        drawsprite(engine_state, spritesx, spritesy);
    }
    while (engine_state->maskwallcnt > 0) {
        drawmaskwall(engine_state);
    }
    
    if (draw_mirror) DrawMirror();
}


int setsprite(short spritenum, int32_t newx, int32_t newy, int32_t newz)
{
    short tempsectnum;

    sprite[spritenum].x = newx;
    sprite[spritenum].y = newy;
    sprite[spritenum].z = newz;

    tempsectnum = sprite[spritenum].sectnum;
    updatesector(newx,newy,&tempsectnum);
    if (tempsectnum < 0) {
        return(-1);
    }
    if (tempsectnum != sprite[spritenum].sectnum) {
        changespritesect(spritenum,tempsectnum);
    }

    return(0);
}


void initspritelists(void)
{
    int32_t i;

    for (i=0; i<MAXSECTORS; i++) { /* Init doubly-linked sprite sector lists */
        headspritesect[i] = -1;
    }

    headspritesect[MAXSECTORS] = 0;

    for (i=0; i<MAXSPRITES; i++) {
        prevspritesect[i] = i-1;
        nextspritesect[i] = i+1;
        sprite[i].sectnum = MAXSECTORS;
    }
    prevspritesect[0] = -1;
    nextspritesect[MAXSPRITES-1] = -1;


    for (i=0; i<MAXSTATUS; i++) { /* Init doubly-linked sprite status lists */
        headspritestat[i] = -1;
    }
    headspritestat[MAXSTATUS] = 0;
    for (i=0; i<MAXSPRITES; i++) {
        prevspritestat[i] = i-1;
        nextspritestat[i] = i+1;
        sprite[i].statnum = MAXSTATUS;
    }
    prevspritestat[0] = -1;
    nextspritestat[MAXSPRITES-1] = -1;
}


int insertsprite(short sectnum, short statnum)
{
    insertspritestat(statnum);
    return(insertspritesect(sectnum));
}


int insertspritesect(short sectnum)
{
    short blanktouse;

    if ((sectnum >= MAXSECTORS) || (headspritesect[MAXSECTORS] == -1)) {
        return(-1);    /* list full */
    }

    blanktouse = headspritesect[MAXSECTORS];

    headspritesect[MAXSECTORS] = nextspritesect[blanktouse];
    if (headspritesect[MAXSECTORS] >= 0) {
        prevspritesect[headspritesect[MAXSECTORS]] = -1;
    }

    prevspritesect[blanktouse] = -1;
    nextspritesect[blanktouse] = headspritesect[sectnum];
    if (headspritesect[sectnum] >= 0) {
        prevspritesect[headspritesect[sectnum]] = blanktouse;
    }
    headspritesect[sectnum] = blanktouse;

    sprite[blanktouse].sectnum = sectnum;

    return(blanktouse);
}


int insertspritestat(short statnum)
{
    short blanktouse;

    if ((statnum >= MAXSTATUS) || (headspritestat[MAXSTATUS] == -1)) {
        return(-1);    /* list full */
    }

    blanktouse = headspritestat[MAXSTATUS];

    headspritestat[MAXSTATUS] = nextspritestat[blanktouse];
    if (headspritestat[MAXSTATUS] >= 0) {
        prevspritestat[headspritestat[MAXSTATUS]] = -1;
    }

    prevspritestat[blanktouse] = -1;
    nextspritestat[blanktouse] = headspritestat[statnum];
    if (headspritestat[statnum] >= 0) {
        prevspritestat[headspritestat[statnum]] = blanktouse;
    }
    headspritestat[statnum] = blanktouse;

    sprite[blanktouse].statnum = statnum;

    return(blanktouse);
}


int deletesprite(short spritenum)
{
    deletespritestat(spritenum);
    return(deletespritesect(spritenum));
}


int deletespritesect(short deleteme)
{
    if (sprite[deleteme].sectnum == MAXSECTORS) {
        return(-1);
    }

    if (headspritesect[sprite[deleteme].sectnum] == deleteme) {
        headspritesect[sprite[deleteme].sectnum] = nextspritesect[deleteme];
    }

    if (prevspritesect[deleteme] >= 0) {
        nextspritesect[prevspritesect[deleteme]] = nextspritesect[deleteme];
    }
    if (nextspritesect[deleteme] >= 0) {
        prevspritesect[nextspritesect[deleteme]] = prevspritesect[deleteme];
    }

    if (headspritesect[MAXSECTORS] >= 0) {
        prevspritesect[headspritesect[MAXSECTORS]] = deleteme;
    }
    prevspritesect[deleteme] = -1;
    nextspritesect[deleteme] = headspritesect[MAXSECTORS];
    headspritesect[MAXSECTORS] = deleteme;

    sprite[deleteme].sectnum = MAXSECTORS;
    return(0);
}


int deletespritestat(short deleteme)
{
    if (sprite[deleteme].statnum == MAXSTATUS) {
        return(-1);
    }

    if (headspritestat[sprite[deleteme].statnum] == deleteme) {
        headspritestat[sprite[deleteme].statnum] = nextspritestat[deleteme];
    }

    if (prevspritestat[deleteme] >= 0) {
        nextspritestat[prevspritestat[deleteme]] = nextspritestat[deleteme];
    }
    if (nextspritestat[deleteme] >= 0) {
        prevspritestat[nextspritestat[deleteme]] = prevspritestat[deleteme];
    }

    if (headspritestat[MAXSTATUS] >= 0) {
        prevspritestat[headspritestat[MAXSTATUS]] = deleteme;
    }
    prevspritestat[deleteme] = -1;
    nextspritestat[deleteme] = headspritestat[MAXSTATUS];
    headspritestat[MAXSTATUS] = deleteme;

    sprite[deleteme].statnum = MAXSTATUS;
    return(0);
}


int changespritesect(short spritenum, short newsectnum)
{
    if ((newsectnum < 0) || (newsectnum > MAXSECTORS)) {
        return(-1);
    }
    if (sprite[spritenum].sectnum == newsectnum) {
        return(0);
    }
    if (sprite[spritenum].sectnum == MAXSECTORS) {
        return(-1);
    }
    if (deletespritesect(spritenum) < 0) {
        return(-1);
    }
    insertspritesect(newsectnum);
    return(0);
}


int changespritestat(short spritenum, short newstatnum)
{
    if ((newstatnum < 0) || (newstatnum > MAXSTATUS)) {
        return(-1);
    }
    if (sprite[spritenum].statnum == newstatnum) {
        return(0);
    }
    if (sprite[spritenum].statnum == MAXSTATUS) {
        return(-1);
    }
    if (deletespritestat(spritenum) < 0) {
        return(-1);
    }
    insertspritestat(newstatnum);
    return(0);
}


int nextsectorneighborz(short sectnum, int32_t thez,
                        short topbottom, short direction)
{
    walltype *wal;
    int32_t i, testz, nextz;
    short sectortouse;

    if (direction == 1) {
        nextz = 0x7fffffff;
    } else {
        nextz = 0x80000000;
    }

    sectortouse = -1;

    wal = &wall[sector[sectnum].wallptr];
    i = sector[sectnum].wallnum;
    do {
        if (wal->nextsector >= 0) {
            if (topbottom == 1) {
                testz = sector[wal->nextsector].floor.z;
                if (direction == 1) {
                    if ((testz > thez) && (testz < nextz)) {
                        nextz = testz;
                        sectortouse = wal->nextsector;
                    }
                } else {
                    if ((testz < thez) && (testz > nextz)) {
                        nextz = testz;
                        sectortouse = wal->nextsector;
                    }
                }
            } else {
                testz = sector[wal->nextsector].ceiling.z;
                if (direction == 1) {
                    if ((testz > thez) && (testz < nextz)) {
                        nextz = testz;
                        sectortouse = wal->nextsector;
                    }
                } else {
                    if ((testz < thez) && (testz > nextz)) {
                        nextz = testz;
                        sectortouse = wal->nextsector;
                    }
                }
            }
        }
        wal++;
        i--;
    } while (i != 0);

    return(sectortouse);
}


int cansee(int32_t x1, int32_t y1, int32_t z1, short sect1,
           int32_t x2, int32_t y2, int32_t z2, short sect2)
{
    Sector *sec;
    walltype *wal, *wal2;
    int32_t i, cnt, nexts, x, y, z, cz, fz, dasectnum, dacnt, danum;
    int32_t x21, y21, z21, x31, y31, x34, y34, bot, t;

    if ((x1 == x2) && (y1 == y2)) {
        return(sect1 == sect2);
    }

    x21 = x2-x1;
    y21 = y2-y1;
    z21 = z2-z1;

    clipsectorlist[0] = sect1;
    danum = 1;
    for (dacnt=0; dacnt<danum; dacnt++) {
        dasectnum = clipsectorlist[dacnt];
        sec = &sector[dasectnum];
        for (cnt=sec->wallnum,wal=&wall[sec->wallptr]; cnt>0; cnt--,wal++) {
            wal2 = &wall[wal->point2];
            x31 = wal->x-x1;
            x34 = wal->x-wal2->x;
            y31 = wal->y-y1;
            y34 = wal->y-wal2->y;

            bot = y21*x34-x21*y34;
            if (bot <= 0) {
                continue;
            }

            t = y21*x31-x21*y31;
            if ((uint32_t)t >= (uint32_t)bot) {
                continue;
            }
            t = y31*x34-x31*y34;
            if ((uint32_t)t >= (uint32_t)bot) {
                continue;
            }

            nexts = wal->nextsector;
            if ((nexts < 0) || wal->flags.one_way) {
                return(0);
            }

            t = divscale24(t,bot);
            x = x1 + mulscale24(x21,t);
            y = y1 + mulscale24(y21,t);
            z = z1 + mulscale24(z21,t);

            getzsofslope((short)dasectnum,x,y,&cz,&fz);
            if ((z <= cz) || (z >= fz)) {
                return(0);
            }
            getzsofslope((short)nexts,x,y,&cz,&fz);
            if ((z <= cz) || (z >= fz)) {
                return(0);
            }

            for (i=danum-1; i>=0; i--)
                if (clipsectorlist[i] == nexts) {
                    break;
                }
            if (i < 0) {
                clipsectorlist[danum++] = nexts;
            }
        }
    }
    for (i=danum-1; i>=0; i--)
        if (clipsectorlist[i] == sect2) {
            return(1);
        }
    return(0);
}


int lintersect(int32_t x1, int32_t y1, int32_t z1, int32_t x2, int32_t y2, int32_t z2,
               int32_t x3, int32_t y3, int32_t x4, int32_t y4, int32_t *intx,
               int32_t *inty, int32_t *intz)
{
    /* p1 to p2 is a line segment */
    int32_t x21, y21, x34, y34, x31, y31, bot, topt, topu, t;

    x21 = x2-x1;
    x34 = x3-x4;
    y21 = y2-y1;
    y34 = y3-y4;
    bot = x21*y34 - y21*x34;
    if (bot >= 0) {
        if (bot == 0) {
            return(0);
        }
        x31 = x3-x1;
        y31 = y3-y1;
        topt = x31*y34 - y31*x34;
        if ((topt < 0) || (topt >= bot)) {
            return(0);
        }
        topu = x21*y31 - y21*x31;
        if ((topu < 0) || (topu >= bot)) {
            return(0);
        }
    } else {
        x31 = x3-x1;
        y31 = y3-y1;
        topt = x31*y34 - y31*x34;
        if ((topt > 0) || (topt <= bot)) {
            return(0);
        }
        topu = x21*y31 - y21*x31;
        if ((topu > 0) || (topu <= bot)) {
            return(0);
        }
    }
    t = divscale24(topt,bot);
    *intx = x1 + mulscale24(x21,t);
    *inty = y1 + mulscale24(y21,t);
    *intz = z1 + mulscale24(z2-z1,t);
    return(1);
}


int rintersect(int32_t x1, int32_t y1, int32_t z1, int32_t vx, int32_t vy, int32_t vz,
               int32_t x3, int32_t y3, int32_t x4, int32_t y4, int32_t *intx,
               int32_t *inty, int32_t *intz)
{
    /* p1 towards p2 is a ray */
    int32_t x34, y34, x31, y31, bot, topt, topu, t;

    x34 = x3-x4;
    y34 = y3-y4;
    bot = vx*y34 - vy*x34;
    if (bot >= 0) {
        if (bot == 0) {
            return(0);
        }
        x31 = x3-x1;
        y31 = y3-y1;
        topt = x31*y34 - y31*x34;
        if (topt < 0) {
            return(0);
        }
        topu = vx*y31 - vy*x31;
        if ((topu < 0) || (topu >= bot)) {
            return(0);
        }
    } else {
        x31 = x3-x1;
        y31 = y3-y1;
        topt = x31*y34 - y31*x34;
        if (topt > 0) {
            return(0);
        }
        topu = vx*y31 - vy*x31;
        if ((topu > 0) || (topu <= bot)) {
            return(0);
        }
    }
    t = divscale16(topt,bot);
    *intx = x1 + mulscale16(vx,t);
    *inty = y1 + mulscale16(vy,t);
    *intz = z1 + mulscale16(vz,t);
    return(1);
}


int hitscan(int32_t xs, int32_t ys, int32_t zs, short sectnum,
            int32_t vx, int32_t vy, int32_t vz,
            short *hitsect, short *hitwall, short *hitsprite,
            int32_t *hitx, int32_t *hity, int32_t *hitz, uint32_t  cliptype)
{
    Sector *sec;
    walltype *wal, *wal2;
    Sprite *spr;
    int32_t z, zz, x1, y1=0, z1=0, x2, y2, x3, y3, x4, y4, intx, inty, intz;
    int32_t topt, topu, bot, dist, offx, offy;
    SpriteFlags cstat;
    int32_t i, j, k, l, tilenum, xoff, yoff, dax, day, daz, daz2;
    int32_t ang, cosang, sinang, xspan, yspan, xrepeat, yrepeat;
    int32_t dawalclipmask, dasprclipmask;
    short tempshortcnt, tempshortnum, dasector, startwall, endwall;
    short nextsector;
    uint8_t  clipyou;

    *hitsect = -1;
    *hitwall = -1;
    *hitsprite = -1;
    if (sectnum < 0) {
        return(-1);
    }

    *hitx = hitscangoalx;
    *hity = hitscangoaly;

    //TODO: Refactor this clip masks thing
    dawalclipmask = (cliptype&65535);
    dasprclipmask = (cliptype>>16);

    clipsectorlist[0] = sectnum;
    tempshortcnt = 0;
    tempshortnum = 1;
    do {
        dasector = clipsectorlist[tempshortcnt];
        sec = &sector[dasector];

        x1 = 0x7fffffff;
        if (sec->ceiling.flags.groudraw) {
            wal = &wall[sec->wallptr];
            wal2 = &wall[wal->point2];
            dax = wal2->x-wal->x;
            day = wal2->y-wal->y;
            i = fixedPointSqrt(dax*dax+day*day);
            if (i == 0) {
                continue;
            }
            i = divscale15(sec->ceiling.heinum,i);
            dax *= i;
            day *= i;

            j = (vz<<8)-dmulscale15(dax,vy,-day,vx);
            if (j != 0) {
                i = ((sec->ceiling.z-zs)<<8)+dmulscale15(dax,ys-wal->y,-day,xs-wal->x);
                if (((i^j) >= 0) && ((klabs(i)>>1) < klabs(j))) {
                    i = divscale30(i,j);
                    x1 = xs + mulscale30(vx,i);
                    y1 = ys + mulscale30(vy,i);
                    z1 = zs + mulscale30(vz,i);
                }
            }
        } else if ((vz < 0) && (zs >= sec->ceiling.z)) {
            z1 = sec->ceiling.z;
            i = z1-zs;
            if ((klabs(i)>>1) < -vz) {
                i = divscale30(i,vz);
                x1 = xs + mulscale30(vx,i);
                y1 = ys + mulscale30(vy,i);
            }
        }
        if ((x1 != 0x7fffffff) && (klabs(x1-xs)+klabs(y1-ys) < klabs((*hitx)-xs)+klabs((*hity)-ys)))
            if (inside(x1,y1,dasector) != 0) {
                *hitsect = dasector;
                *hitwall = -1;
                *hitsprite = -1;
                *hitx = x1;
                *hity = y1;
                *hitz = z1;
            }

        x1 = 0x7fffffff;
        if (sec->floor.flags.groudraw) {
            wal = &wall[sec->wallptr];
            wal2 = &wall[wal->point2];
            dax = wal2->x-wal->x;
            day = wal2->y-wal->y;
            i = fixedPointSqrt(dax*dax+day*day);
            if (i == 0) {
                continue;
            }
            i = divscale15(sec->floor.heinum,i);
            dax *= i;
            day *= i;

            j = (vz<<8)-dmulscale15(dax,vy,-day,vx);
            if (j != 0) {
                i = ((sec->floor.z-zs)<<8)+dmulscale15(dax,ys-wal->y,-day,xs-wal->x);
                if (((i^j) >= 0) && ((klabs(i)>>1) < klabs(j))) {
                    i = divscale30(i,j);
                    x1 = xs + mulscale30(vx,i);
                    y1 = ys + mulscale30(vy,i);
                    z1 = zs + mulscale30(vz,i);
                }
            }
        } else if ((vz > 0) && (zs <= sec->floor.z)) {
            z1 = sec->floor.z;
            i = z1-zs;
            if ((klabs(i)>>1) < vz) {
                i = divscale30(i,vz);
                x1 = xs + mulscale30(vx,i);
                y1 = ys + mulscale30(vy,i);
            }
        }
        if ((x1 != 0x7fffffff) && (klabs(x1-xs)+klabs(y1-ys) < klabs((*hitx)-xs)+klabs((*hity)-ys)))
            if (inside(x1,y1,dasector) != 0) {
                *hitsect = dasector;
                *hitwall = -1;
                *hitsprite = -1;
                *hitx = x1;
                *hity = y1;
                *hitz = z1;
            }

        startwall = sec->wallptr;
        endwall = startwall + sec->wallnum;
        for (z=startwall,wal=&wall[startwall]; z<endwall; z++,wal++) {
            wal2 = &wall[wal->point2];
            x1 = wal->x;
            y1 = wal->y;
            x2 = wal2->x;
            y2 = wal2->y;

            if ((x1-xs)*(y2-ys) < (x2-xs)*(y1-ys)) {
                continue;
            }
            if (rintersect(xs,ys,zs,vx,vy,vz,x1,y1,x2,y2,&intx,&inty,&intz) == 0) {
                continue;
            }

            if (klabs(intx-xs)+klabs(inty-ys) >= klabs((*hitx)-xs)+klabs((*hity)-ys)) {
                continue;
            }

            nextsector = wal->nextsector;
            if ((nextsector < 0) || ((*(int32_t *)&wal->flags) & dawalclipmask)) {
                *hitsect = dasector;
                *hitwall = z;
                *hitsprite = -1;
                *hitx = intx;
                *hity = inty;
                *hitz = intz;
                continue;
            }
            getzsofslope(nextsector,intx,inty,&daz,&daz2);
            if ((intz <= daz) || (intz >= daz2)) {
                *hitsect = dasector;
                *hitwall = z;
                *hitsprite = -1;
                *hitx = intx;
                *hity = inty;
                *hitz = intz;
                continue;
            }

            for (zz=tempshortnum-1; zz>=0; zz--)
                if (clipsectorlist[zz] == nextsector) {
                    break;
                }
            if (zz < 0) {
                clipsectorlist[tempshortnum++] = nextsector;
            }
        }

        for (z=headspritesect[dasector]; z>=0; z=nextspritesect[z]) {
            spr = &sprite[z];
            cstat = spr->flags;
            if (((*(int16_t *)&cstat) & dasprclipmask) == 0) {
                continue;
            }

            x1 = spr->x;
            y1 = spr->y;
            z1 = spr->z;
            switch (cstat.type) {
                case FACE_SPRITE:
                    topt = vx*(x1-xs) + vy*(y1-ys);
                    if (topt <= 0) {
                        continue;
                    }
                    bot = vx*vx + vy*vy;
                    if (bot == 0) {
                        continue;
                    }

                    intz = zs+scale(vz,topt,bot);

                    i = (tiles[spr->picnum].dim.height*spr->yrepeat<<2);

                    if (cstat.real_centered) {
                        z1 += (i>>1);
                    }

                    if (tiles[spr->picnum].animFlags&0x00ff0000) {
                        z1 -= ((int32_t)((int8_t )((tiles[spr->picnum].animFlags>>16)&255))*spr->yrepeat<<2);
                    }

                    if ((intz > z1) || (intz < z1-i)) {
                        continue;
                    }
                    topu = vx*(y1-ys) - vy*(x1-xs);

                    offx = scale(vx,topu,bot);
                    offy = scale(vy,topu,bot);
                    dist = offx*offx + offy*offy;
                    i = tiles[spr->picnum].dim.width*spr->xrepeat;
                    i *= i;
                    if (dist > (i>>7)) {
                        continue;
                    }
                    intx = xs + scale(vx,topt,bot);
                    inty = ys + scale(vy,topt,bot);

                    if (klabs(intx-xs)+klabs(inty-ys) > klabs((*hitx)-xs)+klabs((*hity)-ys)) {
                        continue;
                    }

                    *hitsect = dasector;
                    *hitwall = -1;
                    *hitsprite = z;
                    *hitx = intx;
                    *hity = inty;
                    *hitz = intz;
                    break;
                case WALL_SPRITE:
                    /*
                     * These lines get the 2 points of the rotated sprite
                     * Given: (x1, y1) starts out as the center point
                     */
                    tilenum = spr->picnum;
                    xoff = (int32_t)((int8_t )((tiles[tilenum].animFlags>>8)&255))+((int32_t)spr->xoffset);
                    if (cstat.x_flip) {
                        xoff = -xoff;
                    }
                    k = spr->ang;
                    l = spr->xrepeat;
                    dax = fixedPointSin(k)*l;
                    day = fixedPointSin((k+1536))*l;
                    l = tiles[tilenum].dim.width;
                    k = (l>>1)+xoff;
                    x1 -= mulscale16(dax,k);
                    x2 = x1+mulscale16(dax,l);
                    y1 -= mulscale16(day,k);
                    y2 = y1+mulscale16(day,l);

                    if (cstat.one_sided)   /* back side of 1-way sprite */
                        if ((x1-xs)*(y2-ys) < (x2-xs)*(y1-ys)) {
                            continue;
                        }

                    if (rintersect(xs,ys,zs,vx,vy,vz,x1,y1,x2,y2,&intx,&inty,&intz) == 0) {
                        continue;
                    }

                    if (klabs(intx-xs)+klabs(inty-ys) > klabs((*hitx)-xs)+klabs((*hity)-ys)) {
                        continue;
                    }

                    k = ((tiles[spr->picnum].dim.height*spr->yrepeat)<<2);
                    if (cstat.real_centered) {
                        daz = spr->z+(k>>1);
                    } else {
                        daz = spr->z;
                    }

                    if (tiles[spr->picnum].animFlags&0x00ff0000) {
                        daz -= ((int32_t)((int8_t  )((tiles[spr->picnum].animFlags>>16)&255))*spr->yrepeat<<2);
                    }

                    if ((intz < daz) && (intz > daz-k)) {
                        *hitsect = dasector;
                        *hitwall = -1;
                        *hitsprite = z;
                        *hitx = intx;
                        *hity = inty;
                        *hitz = intz;
                    }
                    break;
                case FLOOR_SPRITE:
                    if (vz == 0) {
                        continue;
                    }
                    intz = z1;
                    if (((intz-zs)^vz) < 0) {
                        continue;
                    }
                    if (cstat.one_sided)
                        if ((zs > intz) == (cstat.y_flip == 0)) {
                            continue;
                        }

                    intx = xs+scale(intz-zs,vx,vz);
                    inty = ys+scale(intz-zs,vy,vz);

                    if (klabs(intx-xs)+klabs(inty-ys) > klabs((*hitx)-xs)+klabs((*hity)-ys)) {
                        continue;
                    }

                    tilenum = spr->picnum;
                    xoff = (int32_t)((int8_t )((tiles[tilenum].animFlags>>8)&255))+((int32_t)spr->xoffset);
                    yoff = (int32_t)((int8_t )((tiles[tilenum].animFlags>>16)&255))+((int32_t)spr->yoffset);
                    if (cstat.x_flip) {
                        xoff = -xoff;
                    }
                    if (cstat.y_flip) {
                        yoff = -yoff;
                    }

                    ang = spr->ang;
                    cosang = fixedPointCos(ang);
                    sinang = fixedPointSin(ang);
                    xspan = tiles[tilenum].dim.width;
                    xrepeat = spr->xrepeat;
                    yspan = tiles[tilenum].dim.height;
                    yrepeat = spr->yrepeat;

                    dax = ((xspan>>1)+xoff)*xrepeat;
                    day = ((yspan>>1)+yoff)*yrepeat;
                    x1 += dmulscale16(sinang,dax,cosang,day)-intx;
                    y1 += dmulscale16(sinang,day,-cosang,dax)-inty;
                    l = xspan*xrepeat;
                    x2 = x1 - mulscale16(sinang,l);
                    y2 = y1 + mulscale16(cosang,l);
                    l = yspan*yrepeat;
                    k = -mulscale16(cosang,l);
                    x3 = x2+k;
                    x4 = x1+k;
                    k = -mulscale16(sinang,l);
                    y3 = y2+k;
                    y4 = y1+k;

                    clipyou = 0;
                    if ((y1^y2) < 0) {
                        if ((x1^x2) < 0) {
                            clipyou ^= (x1*y2<x2*y1)^(y1<y2);
                        } else if (x1 >= 0) {
                            clipyou ^= 1;
                        }
                    }
                    if ((y2^y3) < 0) {
                        if ((x2^x3) < 0) {
                            clipyou ^= (x2*y3<x3*y2)^(y2<y3);
                        } else if (x2 >= 0) {
                            clipyou ^= 1;
                        }
                    }
                    if ((y3^y4) < 0) {
                        if ((x3^x4) < 0) {
                            clipyou ^= (x3*y4<x4*y3)^(y3<y4);
                        } else if (x3 >= 0) {
                            clipyou ^= 1;
                        }
                    }
                    if ((y4^y1) < 0) {
                        if ((x4^x1) < 0) {
                            clipyou ^= (x4*y1<x1*y4)^(y4<y1);
                        } else if (x4 >= 0) {
                            clipyou ^= 1;
                        }
                    }

                    if (clipyou != 0) {
                        *hitsect = dasector;
                        *hitwall = -1;
                        *hitsprite = z;
                        *hitx = intx;
                        *hity = inty;
                        *hitz = intz;
                    }
                    break;
            }
        }
        tempshortcnt++;
    } while (tempshortcnt < tempshortnum);
    return(0);
}


int neartag(int32_t xs, int32_t ys, int32_t zs, short sectnum, short ange,
            short *neartagsector, short *neartagwall, short *neartagsprite,
            int32_t *neartaghitdist, int32_t neartagrange, uint8_t  tagsearch)
{
    walltype *wal, *wal2;
    Sprite *spr;
    int32_t i, z, zz, xe, ye, ze, x1, y1, z1, x2, y2, intx, inty, intz;
    int32_t topt, topu, bot, dist, offx, offy, vx, vy, vz;
    short tempshortcnt, tempshortnum, dasector, startwall, endwall;
    short nextsector, good;

    *neartagsector = -1;
    *neartagwall = -1;
    *neartagsprite = -1;
    *neartaghitdist = 0;

    if (sectnum < 0) {
        return(0);
    }
    if ((tagsearch < 1) || (tagsearch > 3)) {
        return(0);
    }

    vx = mulscale14(fixedPointCos((ange+2048)),neartagrange);
    xe = xs+vx;
    vy = mulscale14(fixedPointSin((ange+2048)),neartagrange);
    ye = ys+vy;
    vz = 0;
    ze = 0;

    clipsectorlist[0] = sectnum;
    tempshortcnt = 0;
    tempshortnum = 1;

    do {
        dasector = clipsectorlist[tempshortcnt];

        startwall = sector[dasector].wallptr;
        endwall = startwall + sector[dasector].wallnum - 1;
        for (z=startwall,wal=&wall[startwall]; z<=endwall; z++,wal++) {
            wal2 = &wall[wal->point2];
            x1 = wal->x;
            y1 = wal->y;
            x2 = wal2->x;
            y2 = wal2->y;

            nextsector = wal->nextsector;

            good = 0;
            if (nextsector >= 0) {
                if ((tagsearch&1) && sector[nextsector].lotag) {
                    good |= 1;
                }
                if ((tagsearch&2) && sector[nextsector].hitag) {
                    good |= 1;
                }
            }
            if ((tagsearch&1) && wal->lotag) {
                good |= 2;
            }
            if ((tagsearch&2) && wal->hitag) {
                good |= 2;
            }

            if ((good == 0) && (nextsector < 0)) {
                continue;
            }
            if ((x1-xs)*(y2-ys) < (x2-xs)*(y1-ys)) {
                continue;
            }

            if (lintersect(xs,ys,zs,xe,ye,ze,x1,y1,x2,y2,&intx,&inty,&intz) == 1) {
                if (good != 0) {
                    if (good&1) {
                        *neartagsector = nextsector;
                    }
                    if (good&2) {
                        *neartagwall = z;
                    }
                    *neartaghitdist = dmulscale14(intx-xs,fixedPointCos((ange+2048)),inty-ys,fixedPointSin((ange+2048)));
                    xe = intx;
                    ye = inty;
                    ze = intz;
                }
                if (nextsector >= 0) {
                    for (zz=tempshortnum-1; zz>=0; zz--)
                        if (clipsectorlist[zz] == nextsector) {
                            break;
                        }
                    if (zz < 0) {
                        clipsectorlist[tempshortnum++] = nextsector;
                    }
                }
            }
        }

        for (z=headspritesect[dasector]; z>=0; z=nextspritesect[z]) {
            spr = &sprite[z];

            good = 0;
            if ((tagsearch&1) && spr->lotag) {
                good |= 1;
            }
            if ((tagsearch&2) && spr->hitag) {
                good |= 1;
            }
            if (good != 0) {
                x1 = spr->x;
                y1 = spr->y;
                z1 = spr->z;

                topt = vx*(x1-xs) + vy*(y1-ys);
                if (topt > 0) {
                    bot = vx*vx + vy*vy;
                    if (bot != 0) {
                        intz = zs+scale(vz,topt,bot);
                        i = tiles[spr->picnum].dim.height*spr->yrepeat;
                        if (spr->flags.real_centered) {
                            z1 += (i<<1);
                        }
                        if (tiles[spr->picnum].animFlags&0x00ff0000) {
                            z1 -= ((int32_t)((int8_t  )((tiles[spr->picnum].animFlags>>16)&255))*spr->yrepeat<<2);
                        }
                        if ((intz <= z1) && (intz >= z1-(i<<2))) {
                            topu = vx*(y1-ys) - vy*(x1-xs);

                            offx = scale(vx,topu,bot);
                            offy = scale(vy,topu,bot);
                            dist = offx*offx + offy*offy;
                            i = (tiles[spr->picnum].dim.width*spr->xrepeat);
                            i *= i;
                            if (dist <= (i>>7)) {
                                intx = xs + scale(vx,topt,bot);
                                inty = ys + scale(vy,topt,bot);
                                if (klabs(intx-xs)+klabs(inty-ys) < klabs(xe-xs)+klabs(ye-ys)) {
                                    *neartagsprite = z;
                                    *neartaghitdist = dmulscale14(intx-xs,fixedPointSin((ange+2560)),inty-ys,fixedPointSin((ange+2048)));
                                    xe = intx;
                                    ye = inty;
                                    ze = intz;
                                }
                            }
                        }
                    }
                }
            }
        }

        tempshortcnt++;
    } while (tempshortcnt < tempshortnum);
    return(0);
}


void dragpoint(short pointhighlight, int32_t dax, int32_t day)
{
    short cnt, tempshort;

    wall[pointhighlight].x = dax;
    wall[pointhighlight].y = day;

    cnt = MAXWALLS;
    tempshort = pointhighlight;    /* search points CCW */
    do {
        if (wall[tempshort].nextwall >= 0) {
            tempshort = wall[wall[tempshort].nextwall].point2;
            wall[tempshort].x = dax;
            wall[tempshort].y = day;
        } else {
            tempshort = pointhighlight;    /* search points CW if not searched all the way around */
            do {
                if (wall[lastwall(tempshort)].nextwall >= 0) {
                    tempshort = wall[lastwall(tempshort)].nextwall;
                    wall[tempshort].x = dax;
                    wall[tempshort].y = day;
                } else {
                    break;
                }
                cnt--;
            } while ((tempshort != pointhighlight) && (cnt > 0));
            break;
        }
        cnt--;
    } while ((tempshort != pointhighlight) && (cnt > 0));
}


int lastwall(short point)
{
    int32_t i, j, cnt;

    if ((point > 0) && (wall[point-1].point2 == point)) {
        return(point-1);
    }
    i = point;
    cnt = MAXWALLS;
    do {
        j = wall[i].point2;
        if (j == point) {
            return(i);
        }
        i = j;
        cnt--;
    } while (cnt > 0);
    return(point);
}

#define addclipline(dax1, day1, dax2, day2, daoval)      \
{                                                        \
clipit[clipnum].x1 = dax1; clipit[clipnum].y1 = day1; \
clipit[clipnum].x2 = dax2; clipit[clipnum].y2 = day2; \
clipobjectval[clipnum] = daoval;                      \
clipnum++;                                            \
}                                                        \
 

static void keepaway (int32_t *x, int32_t *y, int32_t w)
{
    int32_t dx, dy, ox, oy, x1, y1;
    uint8_t  first;

    x1 = clipit[w].x1;
    dx = clipit[w].x2-x1;
    y1 = clipit[w].y1;
    dy = clipit[w].y2-y1;
    ox = ksgn(-dy);
    oy = ksgn(dx);
    first = (klabs(dx) <= klabs(dy));
    while (1) {
        if (dx*(*y-y1) > (*x-x1)*dy) {
            return;
        }
        if (first == 0) {
            *x += ox;
        } else {
            *y += oy;
        }
        first ^= 1;
    }
}


static int raytrace(int32_t x3, int32_t y3, int32_t *x4, int32_t *y4)
{
    int32_t x1, y1, x2, y2, bot, topu, nintx, ninty, cnt, z, hitwall;
    int32_t x21, y21, x43, y43;

    hitwall = -1;
    for (z=clipnum-1; z>=0; z--) {
        x1 = clipit[z].x1;
        x2 = clipit[z].x2;
        x21 = x2-x1;
        y1 = clipit[z].y1;
        y2 = clipit[z].y2;
        y21 = y2-y1;

        topu = x21*(y3-y1) - (x3-x1)*y21;
        
        if (topu <= 0) continue;
        if (x21*(*y4-y1) > (*x4-x1)*y21) continue;
        
        x43 = *x4-x3;
        y43 = *y4-y3;
        
        if (x43*(y1-y3) > (x1-x3)*y43) continue;
        if (x43*(y2-y3) <= (x2-x3)*y43) continue;
        
        bot = x43*y21 - x21*y43;

        if (bot == 0) continue;

        cnt = 256;
        do {
            cnt--;
            if (cnt < 0) {
                *x4 = x3;
                *y4 = y3;
                return(z);
            }
            nintx = x3 + scale(x43,topu,bot);
            ninty = y3 + scale(y43,topu,bot);
            topu--;
        } while (x21*(ninty-y1) <= (nintx-x1)*y21);

        if (klabs(x3-nintx)+klabs(y3-ninty) < klabs(x3-*x4)+klabs(y3-*y4)) {
            *x4 = nintx;
            *y4 = ninty;
            hitwall = z;
        }
    }
    return(hitwall);
}


/* !!! ugh...move this var into clipmove as a parameter, and update build2.txt! */
int32_t clipmoveboxtracenum = 3;
int clipmove (int32_t *x, int32_t *y, int32_t *z, short *sectnum,
              int32_t xvect, int32_t yvect, int32_t walldist, int32_t ceildist,
              int32_t flordist, uint32_t  cliptype)
{
    walltype *wal, *wal2;
    Sprite *spr;
    Sector *sec, *sec2;
    int32_t i, j, templong1, templong2;
    int32_t oxvect, oyvect, goalx, goaly, intx, inty, lx, ly, retval;
    int32_t k, l, clipsectcnt, startwall, endwall, dasect;
    SpriteFlags cstat;
    int32_t x1, y1, x2, y2, cx, cy, rad, xmin, ymin, xmax, ymax, daz, daz2;
    int32_t bsz, dax, day, xoff, yoff, xspan, yspan, cosang, sinang, tilenum;
    int32_t xrepeat, yrepeat, gx, gy, dx, dy, dasprclipmask, dawalclipmask;
    int32_t hitwall, cnt, clipyou;

    if (((xvect|yvect) == 0) || (*sectnum < 0)) {
        return(0);
    }
    retval = 0;

    oxvect = xvect;
    oyvect = yvect;

    goalx = (*x) + (xvect>>14);
    goaly = (*y) + (yvect>>14);


    clipnum = 0;

    cx = (((*x)+goalx)>>1);
    cy = (((*y)+goaly)>>1);
    /* Extra walldist for sprites on sector lines */
    gx = goalx-(*x);
    gy = goaly-(*y);
    rad = fixedPointSqrt(gx*gx + gy*gy) + MAXCLIPDIST+walldist + 8;
    xmin = cx-rad;
    ymin = cy-rad;
    xmax = cx+rad;
    ymax = cy+rad;

    dawalclipmask = (cliptype&65535);        /* CLIPMASK0 = 0x00010001 */
    dasprclipmask = (cliptype>>16);          /* CLIPMASK1 = 0x01000040 */

    clipsectorlist[0] = (*sectnum);
    clipsectcnt = 0;
    clipsectnum = 1;
    do {
        dasect = clipsectorlist[clipsectcnt++];
        sec = &sector[dasect];
        startwall = sec->wallptr;
        endwall = startwall + sec->wallnum;
        for (j=startwall,wal=&wall[startwall]; j<endwall; j++,wal++) {
            wal2 = &wall[wal->point2];
            if ((wal->x < xmin) && (wal2->x < xmin)) continue;
            if ((wal->x > xmax) && (wal2->x > xmax)) continue;
            if ((wal->y < ymin) && (wal2->y < ymin)) continue;
            if ((wal->y > ymax) && (wal2->y > ymax)) continue;

            x1 = wal->x;
            y1 = wal->y;
            x2 = wal2->x;
            y2 = wal2->y;

            dx = x2-x1;
            dy = y2-y1;
            if (dx*((*y)-y1) < ((*x)-x1)*dy) {
                continue;    /* If wall's not facing you */
            }

            if (dx > 0) {
                dax = dx*(ymin-y1);
            } else {
                dax = dx*(ymax-y1);
            }
            if (dy > 0) {
                day = dy*(xmax-x1);
            } else {
                day = dy*(xmin-x1);
            }
            if (dax >= day) continue;

            clipyou = 0;
            if ((wal->nextsector < 0) || ((*(int32_t *)&wal->flags) & dawalclipmask)) {
                clipyou = 1;
            } else {
                if (rintersect(*x,*y,0,gx,gy,0,x1,y1,x2,y2,&dax,&day,&daz) == 0) {
                    dax = *x, day = *y;
                }
                daz = GetZOfSlope(sector[dasect].floor, dax, day);
                daz2 = GetZOfSlope(sector[wal->nextsector].floor, dax, day);

                sec2 = &sector[wal->nextsector];
                if (daz2 < daz-(1<<8))
                    if (!sec2->floor.flags.parallaxing)
                        if ((*z) >= daz2-(flordist-1)) {
                            clipyou = 1;
                        }
                if (clipyou == 0) {
                    daz = GetZOfSlope(sector[dasect].ceiling, dax, day);
                    daz2 = GetZOfSlope(sector[wal->nextsector].ceiling, dax, day);
                    if (daz2 > daz+(1<<8))
                        if (!sec2->ceiling.flags.parallaxing)
                            if ((*z) <= daz2+(ceildist-1)) {
                                clipyou = 1;
                            }
                }
            }

            if (clipyou) {
                /* Add 2 boxes at endpoints */
                bsz = walldist;
                if (gx < 0) {
                    bsz = -bsz;
                }
                addclipline(x1-bsz,y1-bsz,x1-bsz,y1+bsz,(short)j+32768);
                addclipline(x2-bsz,y2-bsz,x2-bsz,y2+bsz,(short)j+32768);
                bsz = walldist;
                if (gy < 0) {
                    bsz = -bsz;
                }
                addclipline(x1+bsz,y1-bsz,x1-bsz,y1-bsz,(short)j+32768);
                addclipline(x2+bsz,y2-bsz,x2-bsz,y2-bsz,(short)j+32768);

                dax = walldist;
                if (dy > 0) {
                    dax = -dax;
                }
                day = walldist;
                if (dx < 0) {
                    day = -day;
                }
                addclipline(x1+dax,y1+day,x2+dax,y2+day,(short)j+32768);
            } else {
                for (i=clipsectnum-1; i>=0; i--)
                    if (wal->nextsector == clipsectorlist[i]) {
                        break;
                    }
                if (i < 0) {
                    clipsectorlist[clipsectnum++] = wal->nextsector;
                }
            }
        }

        for (j=headspritesect[dasect]; j>=0; j=nextspritesect[j]) {
            spr = &sprite[j];
            cstat = spr->flags;
            if (( (*(int16_t *)&cstat) & dasprclipmask) == 0) {
                continue;
            }
            x1 = spr->x;
            y1 = spr->y;
            switch (cstat.type) {
                case FACE_SPRITE:
                    if ((x1 >= xmin) && (x1 <= xmax) && (y1 >= ymin) && (y1 <= ymax)) {
                        k = ((tiles[spr->picnum].dim.height*spr->yrepeat)<<2);
                        if (cstat.real_centered) {
                            daz = spr->z+(k>>1);
                        } else {
                            daz = spr->z;
                        }

                        if (tiles[spr->picnum].animFlags&0x00ff0000) {
                            daz -= ((int32_t)((int8_t )((tiles[spr->picnum].animFlags>>16)&255))*spr->yrepeat<<2);
                        }

                        if (((*z) < daz+ceildist) && ((*z) > daz-k-flordist)) {
                            bsz = (spr->clipdist<<2)+walldist;
                            if (gx < 0) {
                                bsz = -bsz;
                            }
                            addclipline(x1-bsz,y1-bsz,x1-bsz,y1+bsz,(short)j+49152);
                            bsz = (spr->clipdist<<2)+walldist;
                            if (gy < 0) {
                                bsz = -bsz;
                            }
                            addclipline(x1+bsz,y1-bsz,x1-bsz,y1-bsz,(short)j+49152);
                        }
                    }
                    break;
                case 16:
                    k = ((tiles[spr->picnum].dim.height*spr->yrepeat)<<2);

                    if (cstat.real_centered) {
                        daz = spr->z+(k>>1);
                    } else {
                        daz = spr->z;
                    }

                    if (tiles[spr->picnum].animFlags&0x00ff0000) {
                        daz -= ((int32_t)((int8_t  )((tiles[spr->picnum].animFlags>>16)&255))*spr->yrepeat<<2);
                    }
                    daz2 = daz-k;
                    daz += ceildist;
                    daz2 -= flordist;
                    if (((*z) < daz) && ((*z) > daz2)) {
                        /*
                         * These lines get the 2 points of the rotated sprite
                         * Given: (x1, y1) starts out as the center point
                         */
                        tilenum = spr->picnum;
                        xoff = (int32_t)((int8_t  )((tiles[tilenum].animFlags>>8)&255))+((int32_t)spr->xoffset);
                        if (cstat.x_flip) {
                            xoff = -xoff;
                        }
                        k = spr->ang;
                        l = spr->xrepeat;
                        dax = fixedPointSin(k)*l;
                        day = fixedPointSin((k+1536))*l;
                        l = tiles[tilenum].dim.width;
                        k = (l>>1)+xoff;
                        x1 -= mulscale16(dax,k);
                        x2 = x1+mulscale16(dax,l);
                        y1 -= mulscale16(day,k);
                        y2 = y1+mulscale16(day,l);
                        if (clipinsideboxline(cx,cy,x1,y1,x2,y2,rad) != 0) {
                            dax = mulscale14(fixedPointCos((spr->ang+256)),walldist);
                            day = mulscale14(fixedPointSin((spr->ang+256)),walldist);

                            if ((x1-(*x))*(y2-(*y)) >= (x2-(*x))*(y1-(*y))) { /* Front */
                                addclipline(x1+dax,y1+day,x2+day,y2-dax,(short)j+49152);
                            } else {
                                if (cstat.one_sided) continue;
                                
                                addclipline(x2-dax,y2-day,x1-day,y1+dax,(short)j+49152);
                            }

                            /* Side blocker */
                            if ((x2-x1)*((*x)-x1) + (y2-y1)*((*y)-y1) < 0) {
                                addclipline(x1-day,y1+dax,x1+dax,y1+day,(short)j+49152);
                            } else if ((x1-x2)*((*x)-x2) + (y1-y2)*((*y)-y2) < 0) {
                                addclipline(x2+day,y2-dax,x2-dax,y2-day,(short)j+49152);
                            }
                        }
                    }
                    break;
                case 32:
                    daz = spr->z+ceildist;
                    daz2 = spr->z-flordist;
                    if (((*z) < daz) && ((*z) > daz2)) {
                        if (cstat.one_sided)
                            if (((*z) > spr->z) == (cstat.y_flip == 0)) {
                                continue;
                            }

                        tilenum = spr->picnum;
                        xoff = (int32_t)((int8_t  )((tiles[tilenum].animFlags>>8)&255))+((int32_t)spr->xoffset);
                        yoff = (int32_t)((int8_t  )((tiles[tilenum].animFlags>>16)&255))+((int32_t)spr->yoffset);
                        if (cstat.x_flip) {
                            xoff = -xoff;
                        }
                        if (cstat.y_flip) {
                            yoff = -yoff;
                        }

                        k = spr->ang;
                        cosang = fixedPointSin((k+512));
                        sinang = fixedPointSin(k);
                        xspan = tiles[tilenum].dim.width;
                        xrepeat = spr->xrepeat;
                        yspan = tiles[tilenum].dim.height;
                        yrepeat = spr->yrepeat;

                        dax = ((xspan>>1)+xoff)*xrepeat;
                        day = ((yspan>>1)+yoff)*yrepeat;
                        rxi[0] = x1 + dmulscale16(sinang,dax,cosang,day);
                        ryi[0] = y1 + dmulscale16(sinang,day,-cosang,dax);
                        l = xspan*xrepeat;
                        rxi[1] = rxi[0] - mulscale16(sinang,l);
                        ryi[1] = ryi[0] + mulscale16(cosang,l);
                        l = yspan*yrepeat;
                        k = -mulscale16(cosang,l);
                        rxi[2] = rxi[1]+k;
                        rxi[3] = rxi[0]+k;
                        k = -mulscale16(sinang,l);
                        ryi[2] = ryi[1]+k;
                        ryi[3] = ryi[0]+k;

                        dax = mulscale14(fixedPointCos((spr->ang-256)),walldist);
                        day = mulscale14(fixedPointSin((spr->ang-256)),walldist);

                        if ((rxi[0]-(*x))*(ryi[1]-(*y)) < (rxi[1]-(*x))*(ryi[0]-(*y))) {
                            if (clipinsideboxline(cx,cy,rxi[1],ryi[1],rxi[0],ryi[0],rad) != 0) {
                                addclipline(rxi[1]-day,ryi[1]+dax,rxi[0]+dax,ryi[0]+day,(short)j+49152);
                            }
                        } else if ((rxi[2]-(*x))*(ryi[3]-(*y)) < (rxi[3]-(*x))*(ryi[2]-(*y))) {
                            if (clipinsideboxline(cx,cy,rxi[3],ryi[3],rxi[2],ryi[2],rad) != 0) {
                                addclipline(rxi[3]+day,ryi[3]-dax,rxi[2]-dax,ryi[2]-day,(short)j+49152);
                            }
                        }

                        if ((rxi[1]-(*x))*(ryi[2]-(*y)) < (rxi[2]-(*x))*(ryi[1]-(*y))) {
                            if (clipinsideboxline(cx,cy,rxi[2],ryi[2],rxi[1],ryi[1],rad) != 0) {
                                addclipline(rxi[2]-dax,ryi[2]-day,rxi[1]-day,ryi[1]+dax,(short)j+49152);
                            }
                        } else if ((rxi[3]-(*x))*(ryi[0]-(*y)) < (rxi[0]-(*x))*(ryi[3]-(*y))) {
                            if (clipinsideboxline(cx,cy,rxi[0],ryi[0],rxi[3],ryi[3],rad) != 0) {
                                addclipline(rxi[0]+dax,ryi[0]+day,rxi[3]+day,ryi[3]-dax,(short)j+49152);
                            }
                        }
                    }
                    break;
            }
        }
    } while (clipsectcnt < clipsectnum);


    cnt = clipmoveboxtracenum;
    do {
        intx = goalx;
        inty = goaly;
        if ((hitwall = raytrace(*x, *y, &intx, &inty)) >= 0) {
            lx = clipit[hitwall].x2-clipit[hitwall].x1;
            ly = clipit[hitwall].y2-clipit[hitwall].y1;
            templong2 = lx*lx + ly*ly;
            if (templong2 > 0) {
                templong1 = (goalx-intx)*lx + (goaly-inty)*ly;

                if ((klabs(templong1)>>11) < templong2) {
                    i = divscale20(templong1,templong2);
                } else {
                    i = 0;
                }
                goalx = mulscale20(lx,i)+intx;
                goaly = mulscale20(ly,i)+inty;
            }

            templong1 = dmulscale6(lx,oxvect,ly,oyvect);
            for (i=cnt+1; i<=clipmoveboxtracenum; i++) {
                j = hitwalls[i];
                templong2 = dmulscale6(clipit[j].x2-clipit[j].x1,oxvect,clipit[j].y2-clipit[j].y1,oyvect);
                if ((templong1^templong2) < 0) {
                    updatesector(*x,*y,sectnum);
                    return(retval);
                }
            }

            keepaway(&goalx, &goaly, hitwall);
            xvect = ((goalx-intx)<<14);
            yvect = ((goaly-inty)<<14);

            if (cnt == clipmoveboxtracenum) {
                retval = clipobjectval[hitwall];
            }
            hitwalls[cnt] = hitwall;
        }
        cnt--;

        *x = intx;
        *y = inty;
    } while (((xvect|yvect) != 0) && (hitwall >= 0) && (cnt > 0));

    for (j=0; j<clipsectnum; j++)
        if (inside(*x,*y,clipsectorlist[j]) == 1) {
            *sectnum = clipsectorlist[j];
            return(retval);
        }

    *sectnum = -1;
    templong1 = 0x7fffffff;
    for (j=numsectors-1; j>=0; j--)
        if (inside(*x,*y,j) == 1) {
            if (sector[j].ceiling.flags.groudraw) {
                templong2 = (GetZOfSlope(sector[j].ceiling, *x, *y) - (*z));
            } else {
                templong2 = (sector[j].ceiling.z-(*z));
            }

            if (templong2 > 0) {
                if (templong2 < templong1) {
                    *sectnum = j;
                    templong1 = templong2;
                }
            } else {
                if (sector[j].floor.flags.groudraw) {
                    templong2 = ((*z)-GetZOfSlope(sector[j].floor, *x, *y));
                } else {
                    templong2 = ((*z)-sector[j].floor.z);
                }

                if (templong2 <= 0) {
                    *sectnum = j;
                    return(retval);
                }
                if (templong2 < templong1) {
                    *sectnum = j;
                    templong1 = templong2;
                }
            }
        }

    return(retval);
}


int pushmove(int32_t *x, int32_t *y, int32_t *z, short *sectnum,
             int32_t walldist, int32_t ceildist, int32_t flordist,
             uint32_t  cliptype)
{
    Sector *sec, *sec2;
    walltype *wal;
    int32_t i, j, k, t, dx, dy, dax, day, daz, daz2, bad, dir;
    int32_t dawalclipmask;
    short startwall, endwall, clipsectcnt;
    uint8_t  bad2;

    if ((*sectnum) < 0) {
        return(-1);
    }

    dawalclipmask = (cliptype&65535);

    k = 32;
    dir = 1;
    do {
        bad = 0;

        clipsectorlist[0] = *sectnum;
        clipsectcnt = 0;
        clipsectnum = 1;
        do {
            sec = &sector[clipsectorlist[clipsectcnt]];
            if (dir > 0) {
                startwall = sec->wallptr, endwall = startwall + sec->wallnum;
            } else {
                endwall = sec->wallptr, startwall = endwall + sec->wallnum;
            }

            for (i=startwall,wal=&wall[startwall]; i!=endwall; i+=dir,wal+=dir)
                if (clipinsidebox(*x,*y,i,walldist-4) == 1) {
                    j = 0;
                    if (wal->nextsector < 0) {
                        j = 1;
                    }
                    if ((*(int32_t *)&wal->flags) & dawalclipmask) {
                        j = 1;
                    }
                    if (j == 0) {
                        sec2 = &sector[wal->nextsector];


                        /* Find closest point on wall (dax, day) to (*x, *y) */
                        dax = wall[wal->point2].x-wal->x;
                        day = wall[wal->point2].y-wal->y;
                        daz = dax*((*x)-wal->x) + day*((*y)-wal->y);
                        if (daz <= 0) {
                            t = 0;
                        } else {
                            daz2 = dax*dax+day*day;
                            if (daz >= daz2) {
                                t = (1<<30);
                            } else {
                                t = divscale30(daz,daz2);
                            }
                        }
                        dax = wal->x + mulscale30(dax,t);
                        day = wal->y + mulscale30(day,t);


                        daz = GetZOfSlope(sector[clipsectorlist[clipsectcnt]].floor, dax, day);
                        daz2 = GetZOfSlope(sector[wal->nextsector].floor, dax, day);
                        if ((daz2 < daz-(1<<8)) && !sec2->floor.flags.parallaxing)
                            if (*z >= daz2-(flordist-1)) {
                                j = 1;
                            }

                        daz = GetZOfSlope(sector[clipsectorlist[clipsectcnt]].ceiling, dax, day);
                        daz2 = GetZOfSlope(sector[wal->nextsector].ceiling, dax, day);
                        if ((daz2 > daz+(1<<8)) && !sec2->ceiling.flags.parallaxing)
                            if (*z <= daz2+(ceildist-1)) {
                                j = 1;
                            }
                    }
                    if (j != 0) {
                        j = getangle(wall[wal->point2].x-wal->x,wall[wal->point2].y-wal->y);
                        dx = (fixedPointSin(j+1024)>>11);
                        dy = (fixedPointCos(j)>>11);
                        bad2 = 16;
                        do {
                            *x = (*x) + dx;
                            *y = (*y) + dy;
                            bad2--;
                            if (bad2 == 0) {
                                break;
                            }
                        } while (clipinsidebox(*x,*y,i,walldist-4) != 0);
                        bad = -1;
                        k--;
                        if (k <= 0) {
                            return(bad);
                        }
                        updatesector(*x,*y,sectnum);
                    } else {
                        for (j=clipsectnum-1; j>=0; j--)
                            if (wal->nextsector == clipsectorlist[j]) {
                                break;
                            }
                        if (j < 0) {
                            clipsectorlist[clipsectnum++] = wal->nextsector;
                        }
                    }
                }

            clipsectcnt++;
        } while (clipsectcnt < clipsectnum);
        dir = -dir;
    } while (bad != 0);

    return(bad);
}

/*
 FCS:  x and y are the new position of the entity that has just moved:
 lastKnownSector is an hint (the last known sectorID of the entity).

 Thanks to the "hint", the algorithm check:
 1. Is (x,y) inside sectors[sectnum].
 2. Flood in sectnum portal and check again if (x,y) is inside.
 3. Do a linear search on sectors[sectnum] from 0 to numSectors.

 Note: Inside uses cross_product and return as soon as the point switch
 from one side to the other.
 */
void updatesector(int32_t x, int32_t y, short *lastKnownSector)
{
    walltype *wal;
    int32_t i, j;

    //First check the last sector where (old_x,old_y) was before being updated to (x,y)
    if (inside(x,y,*lastKnownSector) == 1) {
        //We found it and (x,y) is still in the same sector: nothing to update !
        return;
    }

    // Seems (x,y) moved into an other sector....hopefully one connected via a portal. Let's flood in each portal.
    if ((*lastKnownSector >= 0) && (*lastKnownSector < numsectors)) {
        wal = &wall[sector[*lastKnownSector].wallptr];
        j = sector[*lastKnownSector].wallnum;
        do {
            i = wal->nextsector;
            if (i >= 0)
                if (inside(x,y,(short)i) == 1) {
                    *lastKnownSector = i;
                    return;
                }
            wal++;
            j--;
        } while (j != 0);
    }

    //Damn that is a BIG move, still cannot find which sector (x,y) belongs to. Let's search via linear search.
    for (i=numsectors-1; i>=0; i--) {
        if (inside(x,y,(short)i) == 1) {
            *lastKnownSector = i;
            return;
        }
    }
    // (x,y) is contained in NO sector. (x,y) is likely out of the map.
    *lastKnownSector = -1;
}


void rotatepoint(int32_t xpivot, int32_t ypivot, int32_t x, int32_t y, short daang, int32_t *x2, int32_t *y2)
{
    int32_t dacos, dasin;

    dacos = fixedPointCos((daang+2048));
    dasin = fixedPointSin((daang+2048));
    x -= xpivot;
    y -= ypivot;
    *x2 = dmulscale14(x,dacos,-y,dasin) + xpivot;
    *y2 = dmulscale14(y,dacos,x,dasin) + ypivot;
}


int initmouse(void)
{
    return(moustat = setupmouse());
}


void getmousevalues(short *mousx, short *mousy, short *bstatus)
{
    if (moustat == 0) {
        *mousx = 0;
        *mousy = 0;
        *bstatus = 0;
        return;
    }
    readmousexy(mousx,mousy);
    readmousebstatus(bstatus);
}


/*
 * This is ryan's change. SDL requires me to call SDL_UpdateRect() to force
 *  vid updates without a SDL_Flip() call, but there's no such thing in the
 *  DOS version of this program, so text won't show up sometimes without
 *  my update call in Linux.  However, to get a nice shadow effect on some
 *  text, Ken draws a string at an offset and darker, and then on top of it
 *  draws the actual string. Two SDL_UpdateRect() calls in over top of each
 *  other cause flicker, so I have this function here so the shadow can
 *  be drawn with _noupdate, and the actual string is draw with an update.
 */
void printext256(int32_t xpos, int32_t ypos, short col, short backcol, char  *name, uint8_t  fontsize)
{
    int32_t stx, i, x, y, charxsiz;
    uint8_t  *fontptr, *letptr, *ptr;



    stx = xpos;

    if (fontsize) {
        fontptr = smalltextfont;
        charxsiz = 4;
    } else {
        fontptr = textfont;
        charxsiz = 8;
    }

    //For each character in the string.
    for (i=0; name[i]; i++) {
        letptr = &fontptr[name[i]<<3];
        ptr = ylookup[ypos+7]+(stx-fontsize)+frameplace;
        for (y=7; y>=0; y--) {
            for (x=charxsiz-1; x>=0; x--) {
                if (letptr[y]&pow2char[7-fontsize-x]) {
                    ptr[x] = (uint8_t )col;
                } else if (backcol >= 0) {
                    ptr[x] = (uint8_t )backcol;
                }
            }
            ptr -= ylookup[1];
        }
        stx += charxsiz;
    }

    _updateScreenRect(xpos, ypos, charxsiz * i, 8);
}

int krand()
{
    randomseed = (randomseed*27584621)+1;
    return(((uint32_t )randomseed)>>16);
}


void getzrange(int32_t x, int32_t y, int32_t z, short sectnum,
               int32_t *ceilz, int32_t *ceilhit, int32_t *florz, int32_t *florhit,
               int32_t walldist, uint32_t  cliptype)
{
    Sector *sec;
    walltype *wal, *wal2;
    Sprite *spr;
    int32_t clipsectcnt, startwall, endwall, tilenum, xoff, yoff, dax, day;
    int32_t xmin, ymin, xmax, ymax, i, j, k, l, daz, daz2, dx, dy;
    int32_t x1, y1, x2, y2, x3, y3, x4, y4, ang, cosang, sinang;
    int32_t xspan, yspan, xrepeat, yrepeat, dasprclipmask, dawalclipmask;
    SpriteFlags cstat;
    uint8_t  clipyou;

    if (sectnum < 0) {
        *ceilz = 0x80000000;
        *ceilhit = -1;
        *florz = 0x7fffffff;
        *florhit = -1;
        return;
    }

    /* Extra walldist for sprites on sector lines */
    i = walldist+MAXCLIPDIST+1;
    xmin = x-i;
    ymin = y-i;
    xmax = x+i;
    ymax = y+i;

    getzsofslope(sectnum,x,y,ceilz,florz);
    *ceilhit = sectnum+16384;
    *florhit = sectnum+16384;

    dawalclipmask = (cliptype&65535);
    dasprclipmask = (cliptype>>16);

    clipsectorlist[0] = sectnum;
    clipsectcnt = 0;
    clipsectnum = 1;

    do { /* Collect sectors inside your square first */
        sec = &sector[clipsectorlist[clipsectcnt]];
        startwall = sec->wallptr;
        endwall = startwall + sec->wallnum;
        for (j=startwall,wal=&wall[startwall]; j<endwall; j++,wal++) {
            k = wal->nextsector;
            if (k >= 0) {
                wal2 = &wall[wal->point2];
                x1 = wal->x;
                x2 = wal2->x;
                if ((x1 < xmin) && (x2 < xmin)) continue;
                if ((x1 > xmax) && (x2 > xmax)) continue;
                y1 = wal->y;
                y2 = wal2->y;
                if ((y1 < ymin) && (y2 < ymin)) continue;
                if ((y1 > ymax) && (y2 > ymax)) continue;

                dx = x2-x1;
                dy = y2-y1;
                if (dx*(y-y1) < (x-x1)*dy) {
                    continue;    /* back */
                }
                if (dx > 0) {
                    dax = dx*(ymin-y1);
                } else {
                    dax = dx*(ymax-y1);
                }
                if (dy > 0) {
                    day = dy*(xmax-x1);
                } else {
                    day = dy*(xmin-x1);
                }
                if (dax >= day) continue;

                if ((*(int32_t *)&wal->flags) & dawalclipmask) continue;
                sec = &sector[k];

                if (!sec->ceiling.flags.parallaxing && (z <= sec->ceiling.z+(3<<8))) continue;
                if (!sec->floor.flags.parallaxing && (z >= sec->floor.z-(3<<8))) continue;

                for (i=clipsectnum-1; i>=0; i--) {
                    if (clipsectorlist[i] == k) break;
                }
                if (i < 0) {
                    clipsectorlist[clipsectnum++] = k;
                }

                if ((x1 < xmin+MAXCLIPDIST) && (x2 < xmin+MAXCLIPDIST)) continue;
                if ((x1 > xmax-MAXCLIPDIST) && (x2 > xmax-MAXCLIPDIST)) continue;
                if ((y1 < ymin+MAXCLIPDIST) && (y2 < ymin+MAXCLIPDIST)) continue;
                if ((y1 > ymax-MAXCLIPDIST) && (y2 > ymax-MAXCLIPDIST)) continue;
                
                if (dx > 0) {
                    dax += dx*MAXCLIPDIST;
                } else {
                    dax -= dx*MAXCLIPDIST;
                }
                if (dy > 0) {
                    day -= dy*MAXCLIPDIST;
                } else {
                    day += dy*MAXCLIPDIST;
                }
                if (dax >= day) continue;

                /* It actually got here, through all the continue's! */
                getzsofslope((short)k,x,y,&daz,&daz2);
                if (daz > *ceilz) {
                    *ceilz = daz;
                    *ceilhit = k+16384;
                }
                if (daz2 < *florz) {
                    *florz = daz2;
                    *florhit = k+16384;
                }
            }
        }
        clipsectcnt++;
    } while (clipsectcnt < clipsectnum);

    for (i=0; i<clipsectnum; i++) {
        for (j=headspritesect[clipsectorlist[i]]; j>=0; j=nextspritesect[j]) {
            spr = &sprite[j];
            cstat = spr->flags;
            if ((*(uint16_t *)&cstat) & dasprclipmask) {
                x1 = spr->x;
                y1 = spr->y;

                clipyou = 0;
                switch (cstat.type) {
                    case FACE_SPRITE:
                        k = walldist+(spr->clipdist<<2)+1;
                        if ((klabs(x1-x) <= k) && (klabs(y1-y) <= k)) {
                            daz = spr->z;
                            k = ((tiles[spr->picnum].dim.height*spr->yrepeat)<<1);
                            if (cstat.real_centered) {
                                daz += k;
                            }
                            if (tiles[spr->picnum].animFlags&0x00ff0000) {
                                daz -= ((int32_t)((int8_t  )((tiles[spr->picnum].animFlags>>16)&255))*spr->yrepeat<<2);
                            }
                            daz2 = daz - (k<<1);
                            clipyou = 1;
                        }
                        break;
                    case WALL_SPRITE:
                        tilenum = spr->picnum;
                        xoff = (int32_t)((int8_t  )((tiles[tilenum].animFlags>>8)&255))+((int32_t)spr->xoffset);
                        if (cstat.x_flip) {
                            xoff = -xoff;
                        }
                        k = spr->ang;
                        l = spr->xrepeat;
                        dax = fixedPointSin(k)*l;
                        day = fixedPointSin((k+1536))*l;
                        l = tiles[tilenum].dim.width;
                        k = (l>>1)+xoff;
                        x1 -= mulscale16(dax,k);
                        x2 = x1+mulscale16(dax,l);
                        y1 -= mulscale16(day,k);
                        y2 = y1+mulscale16(day,l);
                        if (clipinsideboxline(x,y,x1,y1,x2,y2,walldist+1) != 0) {
                            daz = spr->z;
                            k = ((tiles[spr->picnum].dim.height*spr->yrepeat)<<1);
                            if (cstat.real_centered) {
                                daz += k;
                            }

                            if (tiles[spr->picnum].animFlags&0x00ff0000) {
                                daz -= ((int32_t)((int8_t  )((tiles[spr->picnum].animFlags>>16)&255))*spr->yrepeat<<2);
                            }

                            daz2 = daz-(k<<1);
                            clipyou = 1;
                        }
                        break;
                    case FLOOR_SPRITE:
                        daz = spr->z;
                        daz2 = daz;

                        if (cstat.one_sided)
                            if ((z > daz) == ((cstat.y_flip)==0)) {
                                continue;
                            }

                        tilenum = spr->picnum;
                        xoff = (int32_t)((int8_t  )((tiles[tilenum].animFlags>>8)&255))+((int32_t)spr->xoffset);
                        yoff = (int32_t)((int8_t  )((tiles[tilenum].animFlags>>16)&255))+((int32_t)spr->yoffset);
                        if (cstat.x_flip) {
                            xoff = -xoff;
                        }
                        if (cstat.y_flip) {
                            yoff = -yoff;
                        }

                        ang = spr->ang;
                        cosang = fixedPointCos(ang);
                        sinang = fixedPointSin(ang);
                        xspan = tiles[tilenum].dim.width;
                        xrepeat = spr->xrepeat;
                        yspan = tiles[tilenum].dim.height;
                        yrepeat = spr->yrepeat;

                        dax = ((xspan>>1)+xoff)*xrepeat;
                        day = ((yspan>>1)+yoff)*yrepeat;
                        x1 += dmulscale16(sinang,dax,cosang,day)-x;
                        y1 += dmulscale16(sinang,day,-cosang,dax)-y;
                        l = xspan*xrepeat;
                        x2 = x1 - mulscale16(sinang,l);
                        y2 = y1 + mulscale16(cosang,l);
                        l = yspan*yrepeat;
                        k = -mulscale16(cosang,l);
                        x3 = x2+k;
                        x4 = x1+k;
                        k = -mulscale16(sinang,l);
                        y3 = y2+k;
                        y4 = y1+k;

                        dax = mulscale14(fixedPointCos(spr->ang-256),walldist+4);
                        day = mulscale14(fixedPointSin(spr->ang-256),walldist+4);
                        x1 += dax;
                        x2 -= day;
                        x3 -= dax;
                        x4 += day;
                        y1 += day;
                        y2 += dax;
                        y3 -= day;
                        y4 -= dax;

                        if ((y1^y2) < 0) {
                            if ((x1^x2) < 0) {
                                clipyou ^= (x1*y2<x2*y1)^(y1<y2);
                            } else if (x1 >= 0) {
                                clipyou ^= 1;
                            }
                        }
                        if ((y2^y3) < 0) {
                            if ((x2^x3) < 0) {
                                clipyou ^= (x2*y3<x3*y2)^(y2<y3);
                            } else if (x2 >= 0) {
                                clipyou ^= 1;
                            }
                        }
                        if ((y3^y4) < 0) {
                            if ((x3^x4) < 0) {
                                clipyou ^= (x3*y4<x4*y3)^(y3<y4);
                            } else if (x3 >= 0) {
                                clipyou ^= 1;
                            }
                        }
                        if ((y4^y1) < 0) {
                            if ((x4^x1) < 0) {
                                clipyou ^= (x4*y1<x1*y4)^(y4<y1);
                            } else if (x4 >= 0) {
                                clipyou ^= 1;
                            }
                        }
                        break;
                }

                if (clipyou != 0) {
                    if ((z > daz) && (daz > *ceilz)) {
                        *ceilz = daz;
                        *ceilhit = j+49152;
                    }
                    if ((z < daz2) && (daz2 < *florz)) {
                        *florz = daz2;
                        *florhit = j+49152;
                    }
                }
            }
        }
    }
}

/*
 * Sets the viewing window to a given rectangle of the screen.
 * Example: For full screen 320*200, call like this: setview(0L,0L,319L,199L);
 */
void setview(int32_t x1, int32_t y1, int32_t x2, int32_t y2)
{
    int32_t i;

    windowx1 = x1;
    wx1 = (x1<<12);
    windowy1 = y1;
    wy1 = (y1<<12);
    windowx2 = x2;
    wx2 = ((x2+1)<<12);
    windowy2 = y2;
    wy2 = ((y2+1)<<12);

    xdimen = (x2-x1)+1;
    halfxdimen = (xdimen>>1);
    xdimenrecip = divscale32(1L,xdimen);
    ydimen = (y2-y1)+1;

    setaspect(65536L,(int32_t)divscale16(ydim*320L,xdim*200L));

    for (i=0; i<windowx1; i++) {
        startumost[i] = 1, startdmost[i] = 0;
    }
    for (i=windowx1; i<=windowx2; i++) {
        startumost[i] = windowy1, startdmost[i] = windowy2+1;
    }
    for (i=windowx2+1; i<xdim; i++) {
        startumost[i] = 1, startdmost[i] = 0;
    }

    viewoffset = windowy1 * game_mode.bytesperline + windowx1;
}


void setaspect(int32_t daxrange, int32_t daaspect)
{
    viewingrange = daxrange;
    viewingrangerecip = divscale32(1L,daxrange);

    yxaspect = daaspect;
    xyaspect = divscale32(1,yxaspect);
    xdimenscale = scale(xdimen,yxaspect,320);
    xdimscale = scale(320,xyaspect,xdimen);
}


void flushperms(void)
{
    permhead = permtail = 0;
}

// Render a sprite on screen. This is used by the Engine but also the Game module
// when drawing the HUD or the Weapon held by the player !!!
void rotatesprite(int32_t sx, int32_t sy, int32_t z, short a, short picnum,
                  int8_t dashade, uint8_t  dapalnum, uint8_t  dastat,
                  int32_t cx1, int32_t cy1, int32_t cx2, int32_t cy2)
{
    int32_t i;
    permfifotype *per, *per2;

    //If 2D target coordinate do not make sense (left > right)..
    if ((cx1 > cx2) || (cy1 > cy2)) {
        return;
    }

    if (z <= 16) {
        return;
    }

    if (tiles[picnum].animFlags&192) {
        picnum += animateoffs(picnum);
    }

    //Does the tile has negative dimensions ?
    if ((tiles[picnum].dim.width <= 0) || (tiles[picnum].dim.height <= 0)) {
        return;
    }

    if (((dastat&128) == 0) || (numpages < 2) || (beforedrawrooms != 0)) {
        dorotatesprite(sx,sy,z,a,picnum,dashade,dapalnum,dastat,cx1,cy1,cx2,cy2);
    }

    if ((dastat&64) && (cx1 <= 0) && (cy1 <= 0) && (cx2 >= xdim-1) && (cy2 >= ydim-1) &&
        (sx == (160<<16)) && (sy == (100<<16)) && (z == 65536L) && (a == 0) && ((dastat&1) == 0)) {
        permhead = permtail = 0;
    }

    if ((dastat&128) == 0) {
        return;
    }
    if (numpages >= 2) {
        per = &permfifo[permhead];
        per->sx = sx;
        per->sy = sy;
        per->z = z;
        per->a = a;
        per->picnum = picnum;
        per->dashade = dashade;
        per->dapalnum = dapalnum;
        per->dastat = dastat;
        per->pagesleft = numpages+((beforedrawrooms&1)<<7);
        per->cx1 = cx1;
        per->cy1 = cy1;
        per->cx2 = cx2;
        per->cy2 = cy2;

        /* Would be better to optimize out true bounding boxes */
        if (dastat&64) { /* If non-masking write, checking for overlapping cases */
            for (i=permtail; i!=permhead; i=((i+1)&(MAXPERMS-1))) {
                per2 = &permfifo[i];
                if ((per2->pagesleft&127) == 0) continue;
                if (per2->sx != per->sx) continue;
                if (per2->sy != per->sy) continue;
                if (per2->z != per->z) continue;
                if (per2->a != per->a) continue;
                if (tiles[per2->picnum].dim.width > tiles[per->picnum].dim.width) continue;

                if (tiles[per2->picnum].dim.height > tiles[per->picnum].dim.height) continue;
                if (per2->cx1 < per->cx1) continue;
                if (per2->cy1 < per->cy1) continue;
                if (per2->cx2 > per->cx2) continue;
                if (per2->cy2 > per->cy2) continue;
                per2->pagesleft = 0;
            }
            if ((per->z == 65536) && (per->a == 0))
                for (i=permtail; i!=permhead; i=((i+1)&(MAXPERMS-1))) {
                    per2 = &permfifo[i];
                    if ((per2->pagesleft&127) == 0) continue;
                    if (per2->z != 65536) continue;
                    if (per2->a != 0) continue;
                    if (per2->cx1 < per->cx1) continue;
                    if (per2->cy1 < per->cy1) continue;
                    if (per2->cx2 > per->cx2) continue;
                    if (per2->cy2 > per->cy2) continue;
                    if ((per2->sx>>16) < (per->sx>>16)) continue;
                    if ((per2->sy>>16) < (per->sy>>16)) continue;
                    if ((per2->sx>>16)+tiles[per2->picnum].dim.width > (per->sx>>16)+tiles[per->picnum].dim.width) continue;
                    if ((per2->sy>>16)+tiles[per2->picnum].dim.height > (per->sy>>16)+tiles[per->picnum].dim.height) continue;
                    per2->pagesleft = 0;
                }
        }

        permhead = ((permhead+1)&(MAXPERMS-1));
    }
}


static int getclosestcol(int32_t r, int32_t g, int32_t b)
{
    int32_t i, j, k, dist, mindist, retcol;
    uint8_t  *pal1;

    j = (r>>3)*FASTPALGRIDSIZ*FASTPALGRIDSIZ+(g>>3)*FASTPALGRIDSIZ+(b>>3)+FASTPALGRIDSIZ*FASTPALGRIDSIZ+FASTPALGRIDSIZ+1;
    mindist = min(rdist[coldist[r&7]+64+8],gdist[coldist[g&7]+64+8]);
    mindist = min(mindist,bdist[coldist[b&7]+64+8]);
    mindist++;

    r = 64-r;
    g = 64-g;
    b = 64-b;

    retcol = -1;
    for (k=26; k>=0; k--) {
        i = colscan[k]+j;
        if ((colhere[i>>3]&pow2char[i&7]) == 0) {
            continue;
        }
        i = colhead[i];
        do {
            pal1 = (uint8_t *)&palette[i*3];
            dist = gdist[pal1[1]+g];
            if (dist < mindist) {
                dist += rdist[pal1[0]+r];
                if (dist < mindist) {
                    dist += bdist[pal1[2]+b];
                    if (dist < mindist) {
                        mindist = dist;
                        retcol = i;
                    }
                }
            }
            i = colnext[i];
        } while (i >= 0);
    }
    if (retcol >= 0) {
        return(retcol);
    }

    mindist = 0x7fffffff;
    pal1 = (uint8_t *)&palette[768-3];
    for (i=255; i>=0; i--,pal1-=3) {
        dist = gdist[pal1[1]+g];
        if (dist >= mindist) continue;
        dist += rdist[pal1[0]+r];
        if (dist >= mindist) continue;
        dist += bdist[pal1[2]+b];
        if (dist >= mindist) continue;
        mindist = dist;
        retcol = i;
    }
    return(retcol);
}


void makepalookup(int32_t palnum, uint8_t  *remapbuf, int8_t r,
                  int8_t g, int8_t b, uint8_t  dastat)
{
    int32_t i, j, palscale;
    uint8_t  *ptr, *ptr2;

    if (paletteloaded == 0) {
        return;
    }

    if (palookup[palnum] == NULL) {
        /* Allocate palookup buffer */
        if ((palookup[palnum] = (uint8_t *)kkmalloc(numpalookups<<8)) == NULL) {
            allocache(&palookup[palnum],numpalookups<<8,&permanentlock);
        }
    }

    if (dastat == 0) {
        return;
    }
    if ((r|g|b|63) != 63) {
        return;
    }

    if ((r|g|b) == 0) {
        for (i=0; i<256; i++) {
            ptr = (uint8_t *)(FP_OFF(palookup[0])+remapbuf[i]);
            ptr2 = (uint8_t *)(FP_OFF(palookup[palnum])+i);
            for (j=0; j<numpalookups; j++) {
                *ptr2 = *ptr;
                ptr += 256;
                ptr2 += 256;
            }
        }
    } else {
        ptr2 = (uint8_t *)FP_OFF(palookup[palnum]);
        for (i=0; i<numpalookups; i++) {
            palscale = divscale16(i,numpalookups);
            for (j=0; j<256; j++) {
                ptr = (uint8_t *)&palette[remapbuf[j]*3];
                *ptr2++ = getclosestcol((int32_t)ptr[0]+mulscale16(r-ptr[0],palscale),
                                        (int32_t)ptr[1]+mulscale16(g-ptr[1],palscale),
                                        (int32_t)ptr[2]+mulscale16(b-ptr[2],palscale));
            }
        }
    }
}


void setbrightness(uint8_t  dabrightness, uint8_t  *dapal)
{
    int32_t i, k;
    uint8_t newPalette[256*4];

    //Clamp bightness to [0-15]
    curbrightness = min(max(dabrightness,0),15);

    k = 0;

    for (i=0; i<256; i++) {
        newPalette[k++] = britable[curbrightness][dapal[i*3+2]];
        newPalette[k++] = britable[curbrightness][dapal[i*3+1]];
        newPalette[k++] = britable[curbrightness][dapal[i*3+0]];
        newPalette[k++] = 0;
    }


    VBE_setPalette(newPalette);
}

//This is only used by drawmapview.
static void fillpolygon(int32_t npoints, uint8_t polyType,
                        int32_t a1, int32_t a2, int32_t asm3,
                        int32_t g_x1, int32_t g_y2, int32_t shade,
                        uint8_t *pallete, uint8_t *tile_data,
                        EngineState *engine_state)
{
    int32_t z, zz, x1, y1, x2, y2, miny, maxy, y, xinc, cnt;
    int32_t ox, oy, bx, by, p, day1, day2;
    short *ptr, *ptr2;
    short *dotp1[MAXYDIM], *dotp2[MAXYDIM];
    static short smost[MAXYSAVES];

    miny = 0x7fffffff;
    maxy = 0x80000000;
    for (z=npoints-1; z>=0; z--) {
        y = pvWalls[z].cameraSpaceCoo[0][VEC_Y];
        miny = min(miny,y);
        maxy = max(maxy,y);
    }
    miny = (miny>>12);
    maxy = (maxy>>12);
    if (miny < 0) {
        miny = 0;
    }
    if (maxy >= ydim) {
        maxy = ydim-1;
    }
    ptr = smost;    /* They're pointers! - watch how you optimize this thing */
    for (y=miny; y<=maxy; y++) {
        dotp1[y] = ptr;
        dotp2[y] = ptr+(MAXNODESPERLINE>>1);
        ptr += MAXNODESPERLINE;
    }

    for (z=npoints-1; z>=0; z--) {
        zz = pvWalls[z].screenSpaceCoo[0][VEC_COL];
        y1 = pvWalls[z] .cameraSpaceCoo[0][VEC_Y];
        day1 = (y1>>12);
        y2 = pvWalls[zz].cameraSpaceCoo[0][VEC_Y];
        day2 = (y2>>12);
        if (day1 != day2) {
            x1 = pvWalls[z ].cameraSpaceCoo[0][VEC_X];
            x2 = pvWalls[zz].cameraSpaceCoo[0][VEC_X];
            xinc = divscale12(x2-x1,y2-y1);
            if (day2 > day1) {
                x1 += mulscale12((day1<<12)+4095-y1,xinc);
                for (y=day1; y<day2; y++) {
                    *dotp2[y]++ = (x1>>12);
                    x1 += xinc;
                }
            } else {
                x2 += mulscale12((day2<<12)+4095-y2,xinc);
                for (y=day2; y<day1; y++) {
                    *dotp1[y]++ = (x2>>12);
                    x2 += xinc;
                }
            }
        }
    }

    g_x1 = mulscale16(g_x1,xyaspect);
    g_y2 = mulscale16(g_y2,xyaspect);

    oy = miny+1-(ydim>>1);
    engine_state->posx += oy*g_x1;
    engine_state->posy += oy*g_y2;



    ptr = smost;
    for (y=miny; y<=maxy; y++) {
        cnt = dotp1[y]-ptr;
        ptr2 = ptr+(MAXNODESPERLINE>>1);
        for (z=cnt-1; z>=0; z--) {
            day1 = 0;
            day2 = 0;
            for (zz=z; zz>0; zz--) {
                if (ptr[zz] < ptr[day1]) {
                    day1 = zz;
                }
                if (ptr2[zz] < ptr2[day2]) {
                    day2 = zz;
                }
            }
            x1 = ptr[day1];
            ptr[day1] = ptr[z];
            x2 = ptr2[day2]-1;
            ptr2[day2] = ptr2[z];
            if (x1 > x2) {
                continue;
            }

            if (polyType < 1) {
                /* maphline */
                ox = x2+1-(xdim>>1);
                bx = ox*a1 + engine_state->posx;
                by = ox*a2 - engine_state->posy;

                p = ylookup[y]+x2+frameplace;
                hlineasm4(x2-x1,shade<<8,by,bx,p,a1,a2,pallete);
            } else {
                /* maphline */
                ox = x1+1-(xdim>>1);
                bx = ox*a1 + engine_state->posx;
                by = ox*a2 - engine_state->posy;

                p = ylookup[y]+x1+frameplace;
                if (polyType == 1) {
                    mhline(tile_data, bx, (x2-x1)<<16, by, p, a1, a2, asm3);
                } else {
                    thline(tile_data, bx, (x2-x1)<<16, by, p, a1, a2, asm3);
                }
            }
        }
        engine_state->posx += g_x1;
        engine_state->posy += g_y2;
        ptr += MAXNODESPERLINE;
    }
    faketimerhandler();
}


static int clippoly (int32_t npoints, int32_t clipstat)
{
    int32_t z, zz, s1, s2, t, npoints2, start2, z1, z2, z3, z4, splitcnt;
    int32_t cx1, cy1, cx2, cy2;

    cx1 = windowx1;
    cy1 = windowy1;
    cx2 = windowx2+1;
    cy2 = windowy2+1;
    cx1 <<= 12;
    cy1 <<= 12;
    cx2 <<= 12;
    cy2 <<= 12;

    if (clipstat&0xa) { /* Need to clip top or left */
        npoints2 = 0;
        start2 = 0;
        z = 0;
        splitcnt = 0;
        do {
            s2 = cx1-pvWalls[z].cameraSpaceCoo[0][VEC_X];
            do {
                zz = pvWalls[z].screenSpaceCoo[0][VEC_COL];
                pvWalls[z].screenSpaceCoo[0][VEC_COL] = -1;
                s1 = s2;
                s2 = cx1-pvWalls[zz].cameraSpaceCoo[0][VEC_X];
                if (s1 < 0) {
                    pvWalls[npoints2].cameraSpaceCoo[1][VEC_X] = pvWalls[zz].cameraSpaceCoo[0][VEC_X];
                    pvWalls[npoints2].cameraSpaceCoo[1][VEC_Y] = pvWalls[zz].cameraSpaceCoo[0][VEC_Y];
                    pvWalls[npoints2].screenSpaceCoo[1][VEC_COL] = npoints2+1;
                    npoints2++;
                }

                if ((s1^s2) < 0) {
                    pvWalls[npoints2].cameraSpaceCoo[1][VEC_X] =
                        pvWalls[z].cameraSpaceCoo[0][VEC_X]+scale(pvWalls[zz].cameraSpaceCoo[0][VEC_X]-pvWalls[z].cameraSpaceCoo[0][VEC_X],s1,s1-s2);
                    pvWalls[npoints2].cameraSpaceCoo[1][VEC_Y] =
                        pvWalls[z].cameraSpaceCoo[0][VEC_Y]+scale(pvWalls[zz].cameraSpaceCoo[0][VEC_Y]-pvWalls[z].cameraSpaceCoo[0][VEC_Y],s1,s1-s2);

                    if (s1 < 0) {
                        bunchWallsList[splitcnt++] = npoints2;
                    }

                    pvWalls[npoints2].screenSpaceCoo[1][VEC_COL] = npoints2+1;
                    npoints2++;
                }
                z = zz;
            } while (pvWalls[z].screenSpaceCoo[0][VEC_COL] >= 0);

            if (npoints2 >= start2+3) {
                pvWalls[npoints2-1].screenSpaceCoo[1][VEC_COL] = start2, start2 = npoints2;
            } else {
                npoints2 = start2;
            }

            z = 1;
            while ((z < npoints) && (pvWalls[z].screenSpaceCoo[0][VEC_COL] < 0)) {
                z++;
            }
        } while (z < npoints);
        if (npoints2 <= 2) {
            return(0);
        }

        for (z=1; z<splitcnt; z++)
            for (zz=0; zz<z; zz++) {
                z1 = bunchWallsList[z];
                z2 = pvWalls[z1].screenSpaceCoo[1][VEC_COL];
                z3 = bunchWallsList[zz];
                z4 = pvWalls[z3].screenSpaceCoo[1][VEC_COL];
                s1  = klabs(pvWalls[z1].cameraSpaceCoo[1][VEC_X]-pvWalls[z2].cameraSpaceCoo[1][VEC_X])+klabs(pvWalls[z1].cameraSpaceCoo[1][VEC_Y]-pvWalls[z2].cameraSpaceCoo[1][VEC_Y]);
                s1 += klabs(pvWalls[z3].cameraSpaceCoo[1][VEC_X]-pvWalls[z4].cameraSpaceCoo[1][VEC_X])+klabs(pvWalls[z3].cameraSpaceCoo[1][VEC_Y]-pvWalls[z4].cameraSpaceCoo[1][VEC_Y]);
                s2  = klabs(pvWalls[z1].cameraSpaceCoo[1][VEC_X]-pvWalls[z4].cameraSpaceCoo[1][VEC_X])+klabs(pvWalls[z1].cameraSpaceCoo[1][VEC_Y]-pvWalls[z4].cameraSpaceCoo[1][VEC_Y]);
                s2 += klabs(pvWalls[z3].cameraSpaceCoo[1][VEC_X]-pvWalls[z2].cameraSpaceCoo[1][VEC_X])+klabs(pvWalls[z3].cameraSpaceCoo[1][VEC_Y]-pvWalls[z2].cameraSpaceCoo[1][VEC_Y]);
                if (s2 < s1) {
                    t = pvWalls[bunchWallsList[z]].screenSpaceCoo[1][VEC_COL];
                    pvWalls[bunchWallsList[z]].screenSpaceCoo[1][VEC_COL] = pvWalls[bunchWallsList[zz]].screenSpaceCoo[1][VEC_COL];
                    pvWalls[bunchWallsList[zz]].screenSpaceCoo[1][VEC_COL] = t;
                }
            }


        npoints = 0;
        start2 = 0;
        z = 0;
        splitcnt = 0;
        do {
            s2 = cy1-pvWalls[z].cameraSpaceCoo[1][VEC_Y];
            do {
                zz = pvWalls[z].screenSpaceCoo[1][VEC_COL];
                pvWalls[z].screenSpaceCoo[1][VEC_COL] = -1;
                s1 = s2;
                s2 = cy1-pvWalls[zz].cameraSpaceCoo[1][VEC_Y];
                if (s1 < 0) {
                    pvWalls[npoints].cameraSpaceCoo[0][VEC_X] = pvWalls[z].cameraSpaceCoo[1][VEC_X];
                    pvWalls[npoints].cameraSpaceCoo[0][VEC_Y] = pvWalls[z].cameraSpaceCoo[1][VEC_Y];
                    pvWalls[npoints].screenSpaceCoo[0][VEC_COL] = npoints+1;
                    npoints++;
                }
                if ((s1^s2) < 0) {
                    pvWalls[npoints].cameraSpaceCoo[0][VEC_X] = pvWalls[z].cameraSpaceCoo[1][VEC_X]+scale(pvWalls[zz].cameraSpaceCoo[1][VEC_X]-pvWalls[z].cameraSpaceCoo[1][VEC_X],s1,s1-s2);
                    pvWalls[npoints].cameraSpaceCoo[0][VEC_Y] = pvWalls[z].cameraSpaceCoo[1][VEC_Y]+scale(pvWalls[zz].cameraSpaceCoo[1][VEC_Y]-pvWalls[z].cameraSpaceCoo[1][VEC_Y],s1,s1-s2);
                    if (s1 < 0) {
                        bunchWallsList[splitcnt++] = npoints;
                    }
                    pvWalls[npoints].screenSpaceCoo[0][VEC_COL] = npoints+1;
                    npoints++;
                }
                z = zz;
            } while (pvWalls[z].screenSpaceCoo[1][VEC_COL] >= 0);

            if (npoints >= start2+3) {
                pvWalls[npoints-1].screenSpaceCoo[0][VEC_COL] = start2, start2 = npoints;
            } else {
                npoints = start2;
            }

            z = 1;
            while ((z < npoints2) && (pvWalls[z].screenSpaceCoo[1][VEC_COL] < 0)) {
                z++;
            }
        } while (z < npoints2);
        if (npoints <= 2) {
            return(0);
        }

        for (z=1; z<splitcnt; z++)
            for (zz=0; zz<z; zz++) {
                z1 = bunchWallsList[z];
                z2 = pvWalls[z1].screenSpaceCoo[0][VEC_COL];
                z3 = bunchWallsList[zz];
                z4 = pvWalls[z3].screenSpaceCoo[0][VEC_COL];
                s1  = klabs(pvWalls[z1].cameraSpaceCoo[0][VEC_X]-pvWalls[z2].cameraSpaceCoo[0][VEC_X])+klabs(pvWalls[z1].cameraSpaceCoo[0][VEC_Y]-pvWalls[z2].cameraSpaceCoo[0][VEC_Y]);
                s1 += klabs(pvWalls[z3].cameraSpaceCoo[0][VEC_X]-pvWalls[z4].cameraSpaceCoo[0][VEC_X])+klabs(pvWalls[z3].cameraSpaceCoo[0][VEC_Y]-pvWalls[z4].cameraSpaceCoo[0][VEC_Y]);
                s2  = klabs(pvWalls[z1].cameraSpaceCoo[0][VEC_X]-pvWalls[z4].cameraSpaceCoo[0][VEC_X])+klabs(pvWalls[z1].cameraSpaceCoo[0][VEC_Y]-pvWalls[z4].cameraSpaceCoo[0][VEC_Y]);
                s2 += klabs(pvWalls[z3].cameraSpaceCoo[0][VEC_X]-pvWalls[z2].cameraSpaceCoo[0][VEC_X])+klabs(pvWalls[z3].cameraSpaceCoo[0][VEC_Y]-pvWalls[z2].cameraSpaceCoo[0][VEC_Y]);
                if (s2 < s1) {
                    t = pvWalls[bunchWallsList[z]].screenSpaceCoo[0][VEC_COL];
                    pvWalls[bunchWallsList[z]].screenSpaceCoo[0][VEC_COL] = pvWalls[bunchWallsList[zz]].screenSpaceCoo[0][VEC_COL];
                    pvWalls[bunchWallsList[zz]].screenSpaceCoo[0][VEC_COL] = t;
                }
            }
    }
    if (clipstat&0x5) { /* Need to clip bottom or right */
        npoints2 = 0;
        start2 = 0;
        z = 0;
        splitcnt = 0;
        do {
            s2 = pvWalls[z].cameraSpaceCoo[0][VEC_X]-cx2;
            do {
                zz = pvWalls[z].screenSpaceCoo[0][VEC_COL];
                pvWalls[z].screenSpaceCoo[0][VEC_COL] = -1;
                s1 = s2;
                s2 = pvWalls[zz].cameraSpaceCoo[0][VEC_X]-cx2;
                if (s1 < 0) {
                    pvWalls[npoints2].cameraSpaceCoo[1][VEC_X] = pvWalls[z].cameraSpaceCoo[0][VEC_X];
                    pvWalls[npoints2].cameraSpaceCoo[1][VEC_Y] = pvWalls[z].cameraSpaceCoo[0][VEC_Y];
                    pvWalls[npoints2].screenSpaceCoo[1][VEC_COL] = npoints2+1;
                    npoints2++;
                }
                if ((s1^s2) < 0) {
                    pvWalls[npoints2].cameraSpaceCoo[1][VEC_X] = pvWalls[z].cameraSpaceCoo[0][VEC_X]+scale(pvWalls[zz].cameraSpaceCoo[0][VEC_X]-pvWalls[z].cameraSpaceCoo[0][VEC_X],s1,s1-s2);
                    pvWalls[npoints2].cameraSpaceCoo[1][VEC_Y] = pvWalls[z].cameraSpaceCoo[0][VEC_Y]+scale(pvWalls[zz].cameraSpaceCoo[0][VEC_Y]-pvWalls[z].cameraSpaceCoo[0][VEC_Y],s1,s1-s2);
                    if (s1 < 0) {
                        bunchWallsList[splitcnt++] = npoints2;
                    }
                    pvWalls[npoints2].screenSpaceCoo[1][VEC_COL] = npoints2+1;
                    npoints2++;
                }
                z = zz;
            } while (pvWalls[z].screenSpaceCoo[0][VEC_COL] >= 0);

            if (npoints2 >= start2+3) {
                pvWalls[npoints2-1].screenSpaceCoo[1][VEC_COL] = start2, start2 = npoints2;
            } else {
                npoints2 = start2;
            }

            z = 1;
            while ((z < npoints) && (pvWalls[z].screenSpaceCoo[0][VEC_COL] < 0)) {
                z++;
            }
        } while (z < npoints);
        if (npoints2 <= 2) {
            return(0);
        }

        for (z=1; z<splitcnt; z++)
            for (zz=0; zz<z; zz++) {
                z1 = bunchWallsList[z];
                z2 = pvWalls[z1].screenSpaceCoo[1][VEC_COL];
                z3 = bunchWallsList[zz];
                z4 = pvWalls[z3].screenSpaceCoo[1][VEC_COL];
                s1  = klabs(pvWalls[z1].cameraSpaceCoo[1][VEC_X]-pvWalls[z2].cameraSpaceCoo[1][VEC_X])+klabs(pvWalls[z1].cameraSpaceCoo[1][VEC_Y]-pvWalls[z2].cameraSpaceCoo[1][VEC_Y]);
                s1 += klabs(pvWalls[z3].cameraSpaceCoo[1][VEC_X]-pvWalls[z4].cameraSpaceCoo[1][VEC_X])+klabs(pvWalls[z3].cameraSpaceCoo[1][VEC_Y]-pvWalls[z4].cameraSpaceCoo[1][VEC_Y]);
                s2  = klabs(pvWalls[z1].cameraSpaceCoo[1][VEC_X]-pvWalls[z4].cameraSpaceCoo[1][VEC_X])+klabs(pvWalls[z1].cameraSpaceCoo[1][VEC_Y]-pvWalls[z4].cameraSpaceCoo[1][VEC_Y]);
                s2 += klabs(pvWalls[z3].cameraSpaceCoo[1][VEC_X]-pvWalls[z2].cameraSpaceCoo[1][VEC_X])+klabs(pvWalls[z3].cameraSpaceCoo[1][VEC_Y]-pvWalls[z2].cameraSpaceCoo[1][VEC_Y]);
                if (s2 < s1) {
                    t = pvWalls[bunchWallsList[z]].screenSpaceCoo[1][VEC_COL];
                    pvWalls[bunchWallsList[z]].screenSpaceCoo[1][VEC_COL] = pvWalls[bunchWallsList[zz]].screenSpaceCoo[1][VEC_COL];
                    pvWalls[bunchWallsList[zz]].screenSpaceCoo[1][VEC_COL] = t;
                }
            }


        npoints = 0;
        start2 = 0;
        z = 0;
        splitcnt = 0;
        do {
            s2 = pvWalls[z].cameraSpaceCoo[1][VEC_Y]-cy2;
            do {
                zz = pvWalls[z].screenSpaceCoo[1][VEC_COL];
                pvWalls[z].screenSpaceCoo[1][VEC_COL] = -1;
                s1 = s2;
                s2 = pvWalls[zz].cameraSpaceCoo[1][VEC_Y]-cy2;
                if (s1 < 0) {
                    pvWalls[npoints].cameraSpaceCoo[0][VEC_X] = pvWalls[z].cameraSpaceCoo[1][VEC_X];
                    pvWalls[npoints].cameraSpaceCoo[0][VEC_Y] = pvWalls[z].cameraSpaceCoo[1][VEC_Y];
                    pvWalls[npoints].screenSpaceCoo[0][VEC_COL] = npoints+1;
                    npoints++;
                }
                if ((s1^s2) < 0) {
                    pvWalls[npoints].cameraSpaceCoo[0][VEC_X] = pvWalls[z].cameraSpaceCoo[1][VEC_X]+scale(pvWalls[zz].cameraSpaceCoo[1][VEC_X]-pvWalls[z].cameraSpaceCoo[1][VEC_X],s1,s1-s2);
                    pvWalls[npoints].cameraSpaceCoo[0][VEC_Y] = pvWalls[z].cameraSpaceCoo[1][VEC_Y]+scale(pvWalls[zz].cameraSpaceCoo[1][VEC_Y]-pvWalls[z].cameraSpaceCoo[1][VEC_Y],s1,s1-s2);
                    if (s1 < 0) {
                        bunchWallsList[splitcnt++] = npoints;
                    }
                    pvWalls[npoints].screenSpaceCoo[0][VEC_COL] = npoints+1;
                    npoints++;
                }
                z = zz;
            } while (pvWalls[z].screenSpaceCoo[1][VEC_COL] >= 0);

            if (npoints >= start2+3) {
                pvWalls[npoints-1].screenSpaceCoo[0][VEC_COL] = start2, start2 = npoints;
            } else {
                npoints = start2;
            }

            z = 1;
            while ((z < npoints2) && (pvWalls[z].screenSpaceCoo[1][VEC_COL] < 0)) {
                z++;
            }
        } while (z < npoints2);
        if (npoints <= 2) {
            return(0);
        }

        for (z=1; z<splitcnt; z++)
            for (zz=0; zz<z; zz++) {
                z1 = bunchWallsList[z];
                z2 = pvWalls[z1].screenSpaceCoo[0][VEC_COL];
                z3 = bunchWallsList[zz];
                z4 = pvWalls[z3].screenSpaceCoo[0][VEC_COL];
                s1  = klabs(pvWalls[z1].cameraSpaceCoo[0][VEC_X]-pvWalls[z2].cameraSpaceCoo[0][VEC_X])+klabs(pvWalls[z1].cameraSpaceCoo[0][VEC_Y]-pvWalls[z2].cameraSpaceCoo[0][VEC_Y]);
                s1 += klabs(pvWalls[z3].cameraSpaceCoo[0][VEC_X]-pvWalls[z4].cameraSpaceCoo[0][VEC_X])+klabs(pvWalls[z3].cameraSpaceCoo[0][VEC_Y]-pvWalls[z4].cameraSpaceCoo[0][VEC_Y]);
                s2  = klabs(pvWalls[z1].cameraSpaceCoo[0][VEC_X]-pvWalls[z4].cameraSpaceCoo[0][VEC_X])+klabs(pvWalls[z1].cameraSpaceCoo[0][VEC_Y]-pvWalls[z4].cameraSpaceCoo[0][VEC_Y]);
                s2 += klabs(pvWalls[z3].cameraSpaceCoo[0][VEC_X]-pvWalls[z2].cameraSpaceCoo[0][VEC_X])+klabs(pvWalls[z3].cameraSpaceCoo[0][VEC_Y]-pvWalls[z2].cameraSpaceCoo[0][VEC_Y]);
                if (s2 < s1) {
                    t = pvWalls[bunchWallsList[z]].screenSpaceCoo[0][VEC_COL];
                    pvWalls[bunchWallsList[z]].screenSpaceCoo[0][VEC_COL] = pvWalls[bunchWallsList[zz]].screenSpaceCoo[0][VEC_COL];
                    pvWalls[bunchWallsList[zz]].screenSpaceCoo[0][VEC_COL] = t;
                }
            }
    }
    return(npoints);
}


void drawmapview(int32_t dax, int32_t day, int32_t zoome, short ang, EngineState *engine_state)
{
    Sector *sec;
    Sprite *spr;
    int32_t tilenum, xoff, yoff, i, j, k, l, cosang, sinang, xspan, yspan;
    int32_t xrepeat, yrepeat, x, y, x1, y1, x2, y2, x3, y3, x4, y4, bakx1, baky1;
    int32_t s, ox, oy, cx1, cy1, cx2, cy2;
    int32_t bakgxvect, bakgyvect, sortnum, gap, npoints;
    int32_t xvect, yvect, xvect2, yvect2;
    int32_t g_x1, g_y1, g_x2, g_y2;
    int32_t a3;
    int16_t picnum;
    int32_t shade;
    int32_t vis;

    static uint8_t  polyType;

    beforedrawrooms = 0;

    //This seems to be dead code.
    //clearbuf(visitedSectors,(int32_t)((numsectors+31)>>5),0L);

    cx1 = (windowx1<<12);
    cy1 = (windowy1<<12);
    cx2 = ((windowx2+1)<<12)-1;
    cy2 = ((windowy2+1)<<12)-1;
    zoome <<= 8;
    bakgxvect = divscale28(fixedPointSin((1536-ang)),zoome);
    bakgyvect = divscale28(fixedPointCos((1536-ang)),zoome);
    xvect = mulscale8(fixedPointCos((1536-ang)),zoome);
    yvect = mulscale8(fixedPointSin((1536-ang)),zoome);
    xvect2 = mulscale16(xvect,yxaspect);
    yvect2 = mulscale16(yvect,yxaspect);

    sortnum = 0;
    for (s=0,sec=&sector[s]; s<numsectors; s++,sec++) { }

    /* Sort sprite list */
    gap = 1;
    while (gap < sortnum) {
        gap = (gap<<1)+1;
    }
    for (gap>>=1; gap>0; gap>>=1)
        for (i=0; i<sortnum-gap; i++)
            for (j=i; j>=0; j-=gap) {
                if (sprite[tsprite[j].owner].z <= sprite[tsprite[j+gap].owner].z) {
                    break;
                }
                swapshort(&tsprite[j].owner,&tsprite[j+gap].owner);
            }

    for (s=sortnum-1; s>=0; s--) {
        spr = &sprite[tsprite[s].owner];
        if (spr->flags.type == FLOOR_SPRITE) {
            tilenum = spr->picnum;
            xoff = (int32_t)((int8_t  )((tiles[tilenum].animFlags>>8)&255))+((int32_t)spr->xoffset);
            yoff = (int32_t)((int8_t  )((tiles[tilenum].animFlags>>16)&255))+((int32_t)spr->yoffset);

            if (spr->flags.x_flip) {
                xoff = -xoff;
            }
            if (spr->flags.y_flip) {
                yoff = -yoff;
            }

            k = spr->ang;
            cosang = fixedPointCos(k);
            sinang = fixedPointSin(k);
            xspan = tiles[tilenum].dim.width;
            xrepeat = spr->xrepeat;
            yspan = tiles[tilenum].dim.height;
            yrepeat = spr->yrepeat;

            ox = ((xspan>>1)+xoff)*xrepeat;
            oy = ((yspan>>1)+yoff)*yrepeat;
            x1 = spr->x + mulscale(sinang,ox,16) + mulscale(cosang,oy,16);
            y1 = spr->y + mulscale(sinang,oy,16) - mulscale(cosang,ox,16);
            l = xspan*xrepeat;
            x2 = x1 - mulscale(sinang,l,16);
            y2 = y1 + mulscale(cosang,l,16);
            l = yspan*yrepeat;
            k = -mulscale(cosang,l,16);
            x3 = x2+k;
            x4 = x1+k;
            k = -mulscale(sinang,l,16);
            y3 = y2+k;
            y4 = y1+k;

            pvWalls[0].screenSpaceCoo[0][VEC_COL] = 1;
            pvWalls[1].screenSpaceCoo[0][VEC_COL] = 2;
            pvWalls[2].screenSpaceCoo[0][VEC_COL] = 3;
            pvWalls[3].screenSpaceCoo[0][VEC_COL] = 0;
            npoints = 4;

            i = 0;

            ox = x1 - dax;
            oy = y1 - day;
            x = dmulscale16(ox,xvect,-oy,yvect) + (xdim<<11);
            y = dmulscale16(oy,xvect2,ox,yvect2) + (ydim<<11);
            i |= getclipmask(x-cx1,cx2-x,y-cy1,cy2-y);
            pvWalls[0].cameraSpaceCoo[0][VEC_X] = x;
            pvWalls[0].cameraSpaceCoo[0][VEC_Y] = y;

            ox = x2 - dax;
            oy = y2 - day;
            x = dmulscale16(ox,xvect,-oy,yvect) + (xdim<<11);
            y = dmulscale16(oy,xvect2,ox,yvect2) + (ydim<<11);
            i |= getclipmask(x-cx1,cx2-x,y-cy1,cy2-y);
            pvWalls[1].cameraSpaceCoo[0][VEC_X] = x;
            pvWalls[1].cameraSpaceCoo[0][VEC_Y] = y;

            ox = x3 - dax;
            oy = y3 - day;
            x = dmulscale16(ox,xvect,-oy,yvect) + (xdim<<11);
            y = dmulscale16(oy,xvect2,ox,yvect2) + (ydim<<11);
            i |= getclipmask(x-cx1,cx2-x,y-cy1,cy2-y);
            pvWalls[2].cameraSpaceCoo[0][VEC_X] = x;
            pvWalls[2].cameraSpaceCoo[0][VEC_Y] = y;

            x = pvWalls[0].cameraSpaceCoo[0][VEC_X]+pvWalls[2].cameraSpaceCoo[0][VEC_X]-pvWalls[1].cameraSpaceCoo[0][VEC_X];
            y = pvWalls[3].cameraSpaceCoo[0][VEC_Y]+pvWalls[2].cameraSpaceCoo[0][VEC_Y]-pvWalls[1].cameraSpaceCoo[0][VEC_Y];
            i |= getclipmask(x-cx1,cx2-x,y-cy1,cy2-y);
            pvWalls[3].cameraSpaceCoo[0][VEC_X] = x;
            pvWalls[3].cameraSpaceCoo[0][VEC_Y] = y;

            if ((i&0xf0) != 0xf0) {
                continue;
            }
            bakx1 = pvWalls[0].cameraSpaceCoo[0][VEC_X];
            baky1 = mulscale16(pvWalls[0].cameraSpaceCoo[0][VEC_Y]-(ydim<<11),xyaspect)+(ydim<<11);
            if (i&0x0f) {
                npoints = clippoly(npoints,i);
                if (npoints < 3) {
                    continue;
                }
            }

            picnum = spr->picnum;
            if ((uint32_t)picnum >= (uint32_t)MAXTILES) {
                picnum = 0;
            }
            setgotpic(picnum);

            if ((tiles[picnum].dim.width <= 0) ||
                (tiles[picnum].dim.height <= 0)) {
                continue;
            }

            if ((tiles[picnum].animFlags&192) != 0) {
                picnum += animateoffs(picnum);
            }

            TILE_MakeAvailable(picnum);

            if (sector[spr->sectnum].ceiling.flags.parallaxing) {
                shade = ((int32_t)sector[spr->sectnum].ceiling.shade);
            } else {
                shade = ((int32_t)sector[spr->sectnum].floor.shade);
            }
            shade = max(min(shade+spr->shade+6,numpalookups-1),0);
            a3 = (int32_t) FP_OFF(palookup[spr->pal]+(shade<<8));
            vis = engine_state->hisibility;
            if (sec->visibility != 0) {
                vis = mulscale4(vis,(int32_t)((uint8_t )(sec->visibility+16)));
            }
            polyType = spr->flags.transluscence + 1;

            /* relative alignment stuff */
            ox = x2-x1;
            oy = y2-y1;
            i = ox*ox+oy*oy;
            if (i == 0) {
                continue;
            }
            i = (65536*16384)/i;
            g_x1 = mulscale10(dmulscale10(ox,bakgxvect,oy,bakgyvect),i);
            g_y1 = mulscale10(dmulscale10(ox,bakgyvect,-oy,bakgxvect),i);
            ox = y1-y4;
            oy = x4-x1;
            i = ox*ox+oy*oy;
            if (i == 0) {
                continue;
            }
            i = (65536*16384)/i;
            g_x2 = mulscale10(dmulscale10(ox,bakgxvect,oy,bakgyvect),i);
            g_y2 = mulscale10(dmulscale10(ox,bakgyvect,-oy,bakgxvect),i);

            ox = tiles[picnum].dim_power_2.width;
            oy = tiles[picnum].dim_power_2.height;
            if (pow2long[ox] != xspan) {
                ox++;
                g_x1 = mulscale(g_x1,xspan,ox);
                g_y1 = mulscale(g_y1,xspan,ox);
            }

            bakx1 = (bakx1>>4)-(xdim<<7);
            baky1 = (baky1>>4)-(ydim<<7);
            engine_state->posx = dmulscale28(-baky1,g_x1,-bakx1,g_y1);
            engine_state->posy = dmulscale28(bakx1,g_x2,-baky1,g_y2);

            if (!spr->flags.transluscence) {
                msethlineshift(ox,oy);
            } else {
                tsethlineshift(ox,oy);
            }

            if (spr->flags.x_flip) {
                g_x1 = -g_x1, g_y1 = -g_y1, engine_state->posx = -engine_state->posx;
            }
            g_x1 <<= 2;
            engine_state->posx <<= (20+2);
            g_y2 <<= 2;
            engine_state->posy <<= (20+2);

            fillpolygon(npoints, polyType, (g_y1<<2), (g_x2<<2), a3, g_x1, g_y2, shade, (uint8_t *)palookup, tiles[picnum].data, engine_state);
        }
    }
}


void clearview(int32_t dacol)
{
    int32_t p, y, dx;

    if (game_mode.qsetmode != 200) {
        return;
    }

    dx = windowx2-windowx1+1;
    dacol += (dacol<<8);
    dacol += (dacol<<16);

    p = frameplace+ylookup[windowy1]+windowx1;
    for (y=windowy1; y<=windowy2; y++) {
        clearbufbyte((void *)p,dx,dacol);
        p += ylookup[1];
    }
    faketimerhandler();
}


void plotpixel(int32_t x, int32_t y, uint8_t  col)
{
    drawpixel(ylookup[y]+x+frameplace,(int32_t)col);
}


uint8_t  getpixel(int32_t x, int32_t y)
{
    return(readpixel(ylookup[y]+x+frameplace));
}

/* MUST USE RESTOREFORDRAWROOMS AFTER DRAWING */
int32_t setviewcnt = 0;
uint8_t *bakframeplace[4];
int32_t bakxsiz[4], bakysiz[4];
int32_t bakwindowx1[4], bakwindowy1[4];
int32_t bakwindowx2[4], bakwindowy2[4];

void setviewback(void)
{
    int32_t i, j, k;
    int16_t bakumost[MAXXDIM+1], bakdmost[MAXXDIM+1];

    if (setviewcnt <= 0) {
        return;
    }
    setviewcnt--;

    setview(bakwindowx1[setviewcnt],bakwindowy1[setviewcnt],
            bakwindowx2[setviewcnt],bakwindowy2[setviewcnt]);
    copybufbyte(&bakumost[windowx1],&startumost[windowx1],(windowx2-windowx1+1)*sizeof(startumost[0]));
    copybufbyte(&bakdmost[windowx1],&startdmost[windowx1],(windowx2-windowx1+1)*sizeof(startdmost[0]));
    frameplace = bakframeplace[setviewcnt];
    if (setviewcnt == 0) {
        k = bakxsiz[0];
    } else {
        k = max(bakxsiz[setviewcnt-1],bakxsiz[setviewcnt]);
    }
    j = 0;
    for (i=0; i<=k; i++) {
        ylookup[i] = j, j += game_mode.bytesperline;
    }
    setBytesPerLine(game_mode.bytesperline);
}




// Reverse x-wise the input coordinates, used for drawing mirrors
void ReverseCoordinatesInX(int32_t camera_x, int32_t camera_y, short camera_ang, walltype *mirror_wall,
                   int32_t *tposx, int32_t *tposy, short *tang)
{
    int32_t i, j, dx, dy;

    dx = wall[mirror_wall->point2].x - mirror_wall->x;
    dy = wall[mirror_wall->point2].y - mirror_wall->y;

    j = dx * dx + dy * dy;
    
    if (j == 0) return;
    
    i = (((camera_x - mirror_wall->x) * dx + (camera_y - mirror_wall->y) * dy)<<1);
    
    *tposx = (mirror_wall->x << 1) + scale(dx, i, j) - camera_x;
    *tposy = (mirror_wall->y << 1) + scale(dy, i, j) - camera_y;
    *tang = (((getangle(dx, dy) << 1) - camera_ang) & 2047);
}


int sectorofwall(short theline)
{
    int32_t i, gap;

    if ((theline < 0) || (theline >= numwalls)) {
        return(-1);
    }
    i = wall[theline].nextwall;
    if (i >= 0) {
        return(wall[i].nextsector);
    }

    gap = (numsectors>>1);
    i = gap;
    while (gap > 1) {
        gap >>= 1;
        if (sector[i].wallptr < theline) {
            i += gap;
        } else {
            i -= gap;
        }
    }
    while (sector[i].wallptr > theline) {
        i--;
    }
    while (sector[i].wallptr+sector[i].wallnum <= theline) {
        i++;
    }

    return(i);
}

int32_t GetZOfSlope(InnerSector floor_or_ceiling, int32_t x, int32_t y)
{
    int32_t distance_x, distance_y, i, j;
    walltype *wal;

    // If the sector is flat, just return the Z coordinate
    if (!floor_or_ceiling.flags.groudraw) {
        return floor_or_ceiling.z;
    }
    
    wal = &wall[floor_or_ceiling.sector->wallptr];
    distance_x = wall[wal->point2].x - wal->x;
    distance_y = wall[wal->point2].y - wal->y;
    i = (fixedPointSqrt(distance_x * distance_x + distance_y * distance_y) << 5);
    
    if (i == 0) {
        return floor_or_ceiling.z;
    }
    
    j = dmulscale3(distance_x, y - wal->y, -distance_y, x - wal->x);
    
    return floor_or_ceiling.z + scale(floor_or_ceiling.heinum, j, i);
}


/*
 FCS:

 Output the ceiling and floor Z coordinate in the two last parameters for given:
 sectorNumber and worldspace (coordinate X,Y).

 If the sector is flat, this is jsut a lookup. But if either the floor/ceiling have
 a slope it requires more calculation

 */
void getzsofslope(short sectnum, int32_t x, int32_t y, int32_t *ceiling_z, int32_t *floor_z)
{
    *ceiling_z = GetZOfSlope(sector[sectnum].ceiling, x, y);
    *floor_z = GetZOfSlope(sector[sectnum].floor, x, y);
}


void alignceilslope(short dasect, int32_t x, int32_t y, int32_t z)
{
    int32_t i, dax, day;
    walltype *wal;

    wal = &wall[sector[dasect].wallptr];
    dax = wall[wal->point2].x-wal->x;
    day = wall[wal->point2].y-wal->y;

    i = (y-wal->y)*dax - (x-wal->x)*day;
    if (i == 0) {
        return;
    }
    sector[dasect].ceiling.heinum = scale((z-sector[dasect].ceiling.z)<<8,
                                         fixedPointSqrt(dax*dax+day*day),i);

    if (sector[dasect].ceiling.heinum == 0) {
        sector[dasect].ceiling.flags.groudraw = 0;
    } else {
        sector[dasect].ceiling.flags.groudraw = 1;
    }
}


void alignflorslope(short dasect, int32_t x, int32_t y, int32_t z)
{
    int32_t i, dax, day;
    walltype *wal;

    wal = &wall[sector[dasect].wallptr];
    dax = wall[wal->point2].x-wal->x;
    day = wall[wal->point2].y-wal->y;

    i = (y-wal->y)*dax - (x-wal->x)*day;
    if (i == 0) {
        return;
    }
    sector[dasect].floor.heinum = scale((z-sector[dasect].floor.z)<<8,
                                       fixedPointSqrt(dax*dax+day*day),i);

    if (sector[dasect].floor.heinum == 0) {
        sector[dasect].floor.flags.groudraw = 0;
    } else {
        sector[dasect].floor.flags.groudraw = 1;
    }
}

/*
 FCS:
 Search for ???
*/
int loopnumofsector(short sectnum, short wallnum)
{
    int32_t i, numloops, startwall, endwall;

    numloops = 0;

    startwall = sector[sectnum].wallptr;
    endwall = startwall + sector[sectnum].wallnum;

    for (i=startwall; i<endwall; i++) {
        if (i == wallnum) {
            return(numloops);
        }

        if (wall[i].point2 < i) {
            numloops++;
        }
    }
    return(-1);
}


void setfirstwall(short sectnum, short newfirstwall)
{
    int32_t i, j, k, numwallsofloop;
    int32_t startwall, endwall, danumwalls, dagoalloop;

    startwall = sector[sectnum].wallptr;
    danumwalls = sector[sectnum].wallnum;
    endwall = startwall+danumwalls;
    if ((newfirstwall < startwall) || (newfirstwall >= startwall+danumwalls)) {
        return;
    }
    for (i=0; i<danumwalls; i++) {
        memcpy(&wall[i+numwalls],&wall[i+startwall],sizeof(walltype));
    }

    numwallsofloop = 0;
    i = newfirstwall;
    do {
        numwallsofloop++;
        i = wall[i].point2;
    } while (i != newfirstwall);

    /* Put correct loop at beginning */
    dagoalloop = loopnumofsector(sectnum,newfirstwall);
    if (dagoalloop > 0) {
        j = 0;
        while (loopnumofsector(sectnum,j+startwall) != dagoalloop) {
            j++;
        }
        for (i=0; i<danumwalls; i++) {
            k = i+j;
            if (k >= danumwalls) {
                k -= danumwalls;
            }
            memcpy(&wall[startwall+i],&wall[numwalls+k],sizeof(walltype));

            wall[startwall+i].point2 += danumwalls-startwall-j;
            if (wall[startwall+i].point2 >= danumwalls) {
                wall[startwall+i].point2 -= danumwalls;
            }
            wall[startwall+i].point2 += startwall;
        }
        newfirstwall += danumwalls-j;
        if (newfirstwall >= startwall+danumwalls) {
            newfirstwall -= danumwalls;
        }
    }

    for (i=0; i<numwallsofloop; i++) {
        memcpy(&wall[i+numwalls],&wall[i+startwall],sizeof(walltype));
    }
    for (i=0; i<numwallsofloop; i++) {
        k = i+newfirstwall-startwall;
        if (k >= numwallsofloop) {
            k -= numwallsofloop;
        }
        memcpy(&wall[startwall+i],&wall[numwalls+k],sizeof(walltype));

        wall[startwall+i].point2 += numwallsofloop-newfirstwall;
        if (wall[startwall+i].point2 >= numwallsofloop) {
            wall[startwall+i].point2 -= numwallsofloop;
        }
        wall[startwall+i].point2 += startwall;
    }

    for (i=startwall; i<endwall; i++)
        if (wall[i].nextwall >= 0) {
            wall[wall[i].nextwall].nextwall = i;
        }
}

/* end of engine.c ... */


