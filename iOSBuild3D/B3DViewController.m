//
//  B3DViewController.m
//  iOSBuild3D
//
//  Created by Cesar Canassa on 4/6/13.
//  Copyright (c) 2013 Cesar Canassa. All rights reserved.
//

#import "B3DViewController.h"

#import "inttypes.h"

#import "build.h"
#import "engine.h"
#import "display.h"
#import "fixedPoint_math.h"

int32_t pos_x, pos_y, pos_z;
int32_t speed_x, speed_y, speed_ang;
short ang, sector_num;

@interface B3DViewController ()

@property (weak, nonatomic) IBOutlet UIButton *leftButton;
@property (weak, nonatomic) IBOutlet UIButton *backButton;
@property (weak, nonatomic) IBOutlet UIButton *rightButton;
@property (weak, nonatomic) IBOutlet UIButton *frontButton;
@property (weak, nonatomic) IBOutlet UIButton *upButton;
@property (weak, nonatomic) IBOutlet UIButton *downButton;

@end

@implementation B3DViewController

- (IBAction)downPressed:(id)sender {
    pos_z += 6000;
}

- (IBAction)upPressed:(id)sender {
    pos_z -= 6000;
}

- (IBAction)leftTouchDown:(id)sender {
    speed_ang = -20;
}

- (IBAction)rightTouchDown:(id)sender {
    speed_ang = 20;
}

- (IBAction)leftOrRightTouchUp:(id)sender {
    speed_ang = 0;
}

- (IBAction)frontTouchDown:(id)sender {
    speed_x = fixedPointCos(ang) * 100;
    speed_y = fixedPointSin(ang) * 100;
}

- (IBAction)backOrFrontTouchUp:(id)sender {
    speed_x = 0;
    speed_y = 0;
}

- (IBAction)backTouchDown:(id)sender {
    speed_x = -fixedPointCos(ang) * 100;
    speed_y = -fixedPointSin(ang) * 100;
}



- (void)gameLoop
{
    [self.view setNeedsDisplay];
}

- (void)generateSpriteMaps
{
    int8_t look_pos;
    uint8_t  numl;
    uint8_t  buffer[2048];

    
    int32_t fp = TCkopen4load("lookup.dat", 0);
    kread(fp, (uint8_t *)&numl, 1);

    for (int j=0; j < numl; j++) {
        kread(fp, (int8_t *)&look_pos, 1);
        kread(fp, buffer, 256);
    
        makepalookup((int32_t)look_pos, (uint8_t *)buffer, 0, 0, 0, 1);
    }
    /*
    kread(fp,&waterpal[0],768);
    kread(fp,&slimepal[0],768);
    kread(fp,&titlepal[0],768);
    kread(fp,&drealms[0],768);
    kread(fp,&endingpal[0],768);

    palette[765] = palette[766] = palette[767] = 0;
    slimepal[765] = slimepal[766] = slimepal[767] = 0;
    waterpal[765] = waterpal[766] = waterpal[767] = 0;
    */
    
    kclose(fp);
}


- (void)initEngine
{
    NSURL* GRPUrl = [[NSBundle mainBundle] URLForResource:@"DUKE3D" withExtension:@"GRP"];

    initgroupfile(GRPUrl.path.UTF8String);

    initengine();

    [self generateSpriteMaps];

    loadpics("tiles000.art", "\0");

    //NSURL* Map = [[NSBundle mainBundle] URLForResource:@"simple" withExtension:@"map"];
    //loadboard((char *)Map.path.UTF8String, &pos_x, &pos_y, &pos_z, &ang, &sector_num);

    loadboard("E1L1.map", &pos_x, &pos_y, &pos_z, &ang, &sector_num);
    pos_x = 8155;
    pos_y = 42501;
    pos_z = -1791;
    ang = 913;
    sector_num = 255;
    NSLog(@"Pos_z: %d\n", pos_z);

    setgamemode(1, 1024, 640);

    [NSTimer scheduledTimerWithTimeInterval:1.0 / 25.0
                                     target:self
                                   selector:@selector(gameLoop)
                                   userInfo:nil
                                    repeats:YES];
}

+ (void)draw
{
    int32_t ceiling_z, floor_z;
    int r1, r2;

    ang += speed_ang;
    
    getzsofslope(sector_num, pos_x, pos_y, &ceiling_z, &floor_z);
    
    if (pos_z < ceiling_z + (4<<8)) {
        pos_z = ceiling_z + (4<<8);
    }
    if (pos_z > floor_z - (4<<8)) {
        pos_z = floor_z - (4<<8);
    }

    r1 = clipmove(&pos_x, &pos_y, &pos_z, &sector_num, speed_x, speed_y, 164L, 4L<<8, 4L<<8, CLIPMASK0);
    r2 = pushmove(&pos_x, &pos_y, &pos_z, &sector_num, 164L, 4L<<8, 4L<<8, CLIPMASK0);

    //NSLog(@"%d %d", r1 , r2);
    
    drawrooms(pos_x, pos_y, pos_z, ang, 140, sector_num, false);
}

- (void)viewDidLoad
{
    [super viewDidLoad];

    // Set the direction buttons title using unicode arrows
    [self.leftButton  setTitle:@"\U000025C0\U0000FE0E" forState:UIControlStateNormal];
    [self.backButton  setTitle:@"\U000025BC\U0000FE0E" forState:UIControlStateNormal];
    [self.rightButton setTitle:@"\U000025B6\U0000FE0E" forState:UIControlStateNormal];
    [self.frontButton setTitle:@"\U000025B2\U0000FE0E" forState:UIControlStateNormal];
    [self.upButton    setTitle:@"\U000025B2\U0000FE0E" forState:UIControlStateNormal];
    [self.downButton  setTitle:@"\U000025BC\U0000FE0E" forState:UIControlStateNormal];

    // Initialize the engine
    [self initEngine];
}

- (void)didReceiveMemoryWarning
{
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}

@end
