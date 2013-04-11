//
//  MainView.m
//  iOSBuild3D
//
//  Created by Cesar Canassa on 4/6/13.
//  Copyright (c) 2013 Cesar Canassa. All rights reserved.
//

#import "MainView.h"
#import "B3DViewController.h"

#import "display.h"

#define WIDTH 480
#define HEIGHT 300

@implementation MainView


- (id)initWithFrame:(CGRect)frame
{
    self = [super initWithFrame:frame];
    if (self) {
    }
    return self;
}


- (void)drawRect:(CGRect)rect
{
    uint8_t bitmapData[WIDTH * HEIGHT * 4];

    [B3DViewController draw];

    // Converts the palette frambuffer to RGB
    for (int i=0; i<sizeof(bitmapData);i+=4) {
        bitmapData[i] = gPalette[temp_frame[i/4]].red;
        bitmapData[i+1] = gPalette[temp_frame[i/4]].green;
        bitmapData[i+2] = gPalette[temp_frame[i/4]].blue;
        //bitmapData[i+3] = 0;
    }

    // Gets an imageRef from the bitmapData
    CGColorSpaceRef colorSpace = CGColorSpaceCreateDeviceRGB();
    CGContextRef bitmapContext = CGBitmapContextCreate(bitmapData, WIDTH, HEIGHT, 8, WIDTH * 4, colorSpace, kCGImageAlphaNoneSkipLast);
    CGImageRef imageRef = CGBitmapContextCreateImage(bitmapContext);

    // Render the imageRef
    [[UIImage imageWithCGImage:imageRef] drawInRect:self.bounds];

    // Releases stuff from memory
    CGContextRelease(bitmapContext);
    CGColorSpaceRelease(colorSpace);
    CGImageRelease(imageRef);
}

@end
