//
//  ViewController.h
//  newAR
//
//  Created by Shangjin Zhai on 14-11-17.
//  Copyright (c) 2014年 Shangjin Zhai. All rights reserved.
//

#import <AVFoundation/AVFoundation.h>
#import <UIKit/UIKit.h>
#import <CoreMotion/CoreMotion.h>
#import <opencv2/highgui/highgui_c.h>
#import <opencv2/features2d/features2d.hpp>
#import <opencv2/nonfree/features2d.hpp>
#include "IOS_related/OpenCVController.h"
#include "IOS_related/DrawImage.h"
#import <mach/mach_time.h>
#include <thread>
#include <future>
#include <chrono>
#include "Solve3D.h"
#include "Visualizer.h"
#import <GLKit/GLKit.h>
using namespace cv;
using namespace std;

extern double focus;
extern double zNear;

@interface ViewController : UIViewController <GLKViewDelegate,AVCaptureVideoDataOutputSampleBufferDelegate>
{
    vector<u_int64_t> timeStamp;
    Feature_Match featureMatch;
    bool isMatch;
    UIImage *photoImage;
    Mat imageObject;
    int markerNum;
    vector<int> markerIdx;
    int searchNum;
    bool isTrain;
    future<Mat> backThread;
    bool isMatched;
    Feature_Track featureTrack;
    bool backgroundIsRun;
    GLKView* mGLView;
    Mat CameraPose;
    bool isCapture;
    Visualizer* mVisualizer;
    int keyPointSize;
    double detectfps,matchfps;
    int saveId;
    double yaw, pitch, roll;

    float mFocal;
    
    GLubyte* image;
}

@property (nonatomic, strong) AVCaptureSession *captureSession;
@property (nonatomic, strong) AVCaptureVideoDataOutput *dataOutput;
@property (nonatomic, strong) CALayer *customPreviewerLayer;
@property (nonatomic, strong) AVCaptureStillImageOutput *stillImageOutput;
@property (nonatomic, strong) CMMotionManager *motionManager;
@property (nonatomic, strong) UILabel *numberLabel;
@property (nonatomic, strong) UILabel *fpsLabel;
@property (nonatomic, strong) UILabel *markerCount;
@property (nonatomic, strong) UILabel *markerList;
@property (nonatomic, strong) UIButton *matchButton;
@property (nonatomic, strong) UIButton *refsButton;
@property (nonatomic, strong) UIButton *saveButton;
@end


