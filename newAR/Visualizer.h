//
//  Visualizer.h
//  PanoSynth
//
//  Created by wangnan on 14-9-24.
//  Copyright (c) 2014年 wangnan. All rights reserved.
//

#import <Foundation/Foundation.h>
#import <GLKit/GLKit.h>
#import <OpenGLES/ES1/gl.h>
#import <vector>
using namespace cv;
#define BUFFER_OFFSET(i) ((char *)NULL + (i))

@interface Visualizer : NSObject
{
    int mWidth;
    int mHeight;
}


-(id)initWithSize:(int) w andHeight:(int) h andFocal:(float) f;
-(int)generateTexture:(GLubyte*) rgba_ptr;
-(void)renderKeyFrames;
-(void)renderTexture:(GLuint) tex_id withImagePtr:(GLubyte*) rgba_ptr;
-(void)setupFrustum : (float) zNear;
-(void)setupOrtho;
-(void)setupVideoOrtho;
-(void)renderPlane;
-(void)renderCube:(Point3f) center;
-(void)renderGrid;
-(void)renderPoints:(const vector<Point3d> &)points;

-(void)onMove:(float) x :(float)y;
-(void)onScale:(float)s;

@end
