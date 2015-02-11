
//  Visualizer.m
//  PanoSynth
//
//  Created by wangnan on 14-9-24.
//  Copyright (c) 2014年 wangnan. All rights reserved.
//

#import "Visualizer.h"
#import <OpenGLES/ES1/gl.h>
#import <OpenGLES/ES1/glext.h>


@interface Visualizer ()
{
    //for panorama
    int nSlice ;

    int mPanoramaTex;
    int bgImageTex;
    void* PanoPtr;
    //
    GLfloat xRot ,yRot;
    GLfloat scale;
}
@end

@implementation Visualizer

static GLfloat texcoord[]=
{
    0.0f , 1.0f,
    0.0f , 0.0f,
    1.0f , 1.0f,
    1.0f , 0.0f
};

static GLfloat aspect = 1.333f;
static GLfloat zNear = 1.0f;
static GLfloat focal = 2.0f;
static float squareCoord[4][3]={

-aspect , 1.0f , -zNear,
-aspect , -1.0f , -zNear,
aspect , 1.0f , -zNear,
aspect , -1.0f , -zNear
};

GLfloat gCubeVertexData[216] =
{
    // Data layout for each line below is:
    // positionX, positionY, positionZ,     normalX, normalY, normalZ,
    0.5f, -0.5f, -0.5f,        1.0f, 0.0f, 0.0f,
    0.5f, 0.5f, -0.5f,         1.0f, 0.0f, 0.0f,
    0.5f, -0.5f, 0.5f,         1.0f, 0.0f, 0.0f,
    0.5f, -0.5f, 0.5f,         1.0f, 0.0f, 0.0f,
    0.5f, 0.5f, -0.5f,          1.0f, 0.0f, 0.0f,
    0.5f, 0.5f, 0.5f,         1.0f, 0.0f, 0.0f,
    
    0.5f, 0.5f, -0.5f,         0.0f, 1.0f, 0.0f,
    -0.5f, 0.5f, -0.5f,        0.0f, 1.0f, 0.0f,
    0.5f, 0.5f, 0.5f,          0.0f, 1.0f, 0.0f,
    0.5f, 0.5f, 0.5f,          0.0f, 1.0f, 0.0f,
    -0.5f, 0.5f, -0.5f,        0.0f, 1.0f, 0.0f,
    -0.5f, 0.5f, 0.5f,         0.0f, 1.0f, 0.0f,
    
    -0.5f, 0.5f, -0.5f,        -1.0f, 0.0f, 0.0f,
    -0.5f, -0.5f, -0.5f,       -1.0f, 0.0f, 0.0f,
    -0.5f, 0.5f, 0.5f,         -1.0f, 0.0f, 0.0f,
    -0.5f, 0.5f, 0.5f,         -1.0f, 0.0f, 0.0f,
    -0.5f, -0.5f, -0.5f,       -1.0f, 0.0f, 0.0f,
    -0.5f, -0.5f, 0.5f,        -1.0f, 0.0f, 0.0f,
    
    -0.5f, -0.5f, -0.5f,       0.0f, -1.0f, 0.0f,
    0.5f, -0.5f, -0.5f,        0.0f, -1.0f, 0.0f,
    -0.5f, -0.5f, 0.5f,        0.0f, -1.0f, 0.0f,
    -0.5f, -0.5f, 0.5f,        0.0f, -1.0f, 0.0f,
    0.5f, -0.5f, -0.5f,        0.0f, -1.0f, 0.0f,
    0.5f, -0.5f, 0.5f,         0.0f, -1.0f, 0.0f,
    
    0.5f, 0.5f, 0.5f,          0.0f, 0.0f, 1.0f,
    -0.5f, 0.5f, 0.5f,         0.0f, 0.0f, 1.0f,
    0.5f, -0.5f, 0.5f,         0.0f, 0.0f, 1.0f,
    0.5f, -0.5f, 0.5f,         0.0f, 0.0f, 1.0f,
    -0.5f, 0.5f, 0.5f,         0.0f, 0.0f, 1.0f,
    -0.5f, -0.5f, 0.5f,        0.0f, 0.0f, 1.0f,
    
    0.5f, -0.5f, -0.5f,        0.0f, 0.0f, -1.0f,
    -0.5f, -0.5f, -0.5f,       0.0f, 0.0f, -1.0f,
    0.5f, 0.5f, -0.5f,         0.0f, 0.0f, -1.0f,
    0.5f, 0.5f, -0.5f,         0.0f, 0.0f, -1.0f,
    -0.5f, -0.5f, -0.5f,       0.0f, 0.0f, -1.0f,
    -0.5f, 0.5f, -0.5f,        0.0f, 0.0f, -1.0f
};

-(id)initWithSize:(int) w andHeight:(int) h andFocal:(float) f
{
    self = [super init];
    if ( self )
    {
        mWidth = w;
        mHeight = h;
        
        mPanoramaTex = -1;
        bgImageTex = -1;
    }
    return self;
}




-(int)generateTexture:(GLubyte*) image_ptr
{
    GLuint tex_id;
    glEnable(GL_TEXTURE_2D);
    glGenTextures(1,&tex_id);
    assert(tex_id>=0);
    glBindTexture(GL_TEXTURE_2D , tex_id);
    glTexImage2D(GL_TEXTURE_2D , 0 , GL_RGBA , mWidth , mHeight ,
                 0, GL_RGBA , GL_UNSIGNED_BYTE , image_ptr);
    glTexParameteri(GL_TEXTURE_2D , GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D , GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glDisable(GL_TEXTURE_2D);
    return tex_id;
    
}

-(void)renderTexture:(GLuint) tex_id withImagePtr:(GLubyte*) dataPtr
{
    
    glEnable(GL_TEXTURE_2D);
    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
    glBindTexture(GL_TEXTURE_2D, tex_id);
    
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, mWidth, mHeight, GL_RGBA, GL_UNSIGNED_BYTE, dataPtr);
    
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
    
    glEnableClientState(GL_TEXTURE_COORD_ARRAY);
    glEnableClientState(GL_VERTEX_ARRAY);
    
    glTexCoordPointer(2, GL_FLOAT, 0 , texcoord);
    glVertexPointer(3, GL_FLOAT, 0, squareCoord);
    
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    
    glDisableClientState(GL_TEXTURE_COORD_ARRAY);
    glDisableClientState(GL_VERTEX_ARRAY);
    
    glDisable(GL_TEXTURE_2D);
    
}

-(void)renderGrid
{
    const int nHalfCells = 5;
    const int nTot = nHalfCells * 2 + 1;
    const float step = 0.3;
    
    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glColor4f(0, 0, 0, 1);
    glLineWidth(1.5);
    
    glEnableClientState(GL_VERTEX_ARRAY);
    
    float *vertexArray = new float[nTot * 3 + 6];
    
    for(int i = 0; i < nTot; i++)
    {
        for (int j = -1, k = 0; j < nTot + 1; j++) {
            vertexArray[k++] = (float) (j - nHalfCells) * step;
            vertexArray[k++] = (float) (i - nHalfCells) * step;
            vertexArray[k++] = 0;
        }
        glVertexPointer(3, GL_FLOAT, 0, vertexArray);
        glDrawArrays(GL_LINE_STRIP, 0, nTot + 2);
    }
    
    for(int i = 0; i < nTot; i++)
    {
        for (int j = 0, k = 0; j < nTot; j++) {
            vertexArray[k++] = (float) (i - nHalfCells) * step;
            vertexArray[k++] = (float) (j - nHalfCells) * step;
            vertexArray[k++] = 0;
        }
        glVertexPointer(3, GL_FLOAT, 0, vertexArray);
        glDrawArrays(GL_LINE_STRIP, 0, nTot);
    }
    
    glDisableClientState(GL_VERTEX_ARRAY);
    delete [] vertexArray;
    
    glDisable(GL_LINE_SMOOTH);
    glDisable(GL_BLEND);
}

-(void)renderPoints:(const vector<Point3d> &)points
{
    
    
    glEnableClientState(GL_VERTEX_ARRAY);
    
    float *vertexArray = new float[points.size() * 3];
    for(int i = 0, k = 0; i < points.size(); i++)
    {
        //printf("%lf %lf %lf\n",points[i].x, points[i].y, points[i].z);
        vertexArray[k++] = (float) points[i].x ;//* 10;
        vertexArray[k++] = (float) points[i].y ;//* 10;
        vertexArray[k++] = 0;
    }
    glColor4f(1, 0, 0, 1);
    glPointSize(10);
    glVertexPointer(3, GL_FLOAT, 0, vertexArray);
    glDrawArrays(GL_POINTS, 0, (int)points.size());
    
    glDisableClientState(GL_VERTEX_ARRAY);
    delete [] vertexArray;
}

-(void)setupFrustum : (float) zNear
{
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    //float aspect = (float)PanoSynth::Config::ImageSize.x / (float)PanoSynth::Config::ImageSize.y;
    //glFrustumf(-zNear, zNear, zNear/aspect,-zNear/aspect,zNear,50);
    glFrustumf(-aspect, aspect, 1.0f, -1.0f, zNear , 1000.0f);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);

}

-(void)setupOrtho
{
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrthof(-aspect, aspect, 1.0f, -1.0f, -10.0f, 10.0f);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

-(void)setupVideoOrtho
{
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrthof(-0.5f, mWidth - 0.5f, mHeight-0.5f, -0.5f, -1.0f, 1.0f);
    glMatrixMode(GL_MODELVIEW);
}



-(void)onMove:(float) x :(float)y
{
    xRot += x*0.1f;
    yRot += y*0.1f;
}

-(void)onScale:(float)s
{
    scale*=s;
}

-(void)renderCube:(Point3f)center
{
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_NORMAL_ARRAY);
    glEnableClientState(GL_COLOR_ARRAY);
    
    GLfloat nowCubeVertexData[216];
    for(int i = 0; i < 216; i++)
        nowCubeVertexData[i] = gCubeVertexData[i];
    for(int i = 0; i < 36; i++)
    {
        nowCubeVertexData[i * 6] = gCubeVertexData[i * 6] + center.x;
        nowCubeVertexData[i * 6 + 1] = gCubeVertexData[i * 6 + 1] + center.y;
    }
    
    glVertexPointer(3,GL_FLOAT ,24 , nowCubeVertexData);
    glNormalPointer(GL_FLOAT, 24, (GLubyte*)nowCubeVertexData+12);

    glDrawArrays(GL_TRIANGLE_STRIP, 0, 36);
    
    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_NORMAL_ARRAY);
}

-(void)renderPlane
{
    glEnableClientState(GL_VERTEX_ARRAY);
    
    glVertexPointer(3, GL_FLOAT, 0, squareCoord);
    
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

    glDisableClientState(GL_VERTEX_ARRAY);
}

@end