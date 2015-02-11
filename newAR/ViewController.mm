//
//  ViewController.m
//  newAR
//
//  Created by Shangjin Zhai on 14-11-17.
//  Copyright (c) 2014年 Shangjin Zhai. All rights reserved.
//

#import "ViewController.h"

double focus;
double zNear;

static double machTimeToSecs(uint64_t time)
{
    mach_timebase_info_data_t timebase;
    mach_timebase_info(&timebase);
    return (double)time * (double)timebase.numer /
    (double)timebase.denom / 1e9;
}


@interface ViewController ()

@end

@implementation ViewController

@synthesize captureSession = _captureSession;
@synthesize dataOutput = _dataOutput;
@synthesize customPreviewerLayer = _customPreviewerLayer;
@synthesize stillImageOutput = _stillImageOutput;
@synthesize numberLabel = _numberLabel;
@synthesize fpsLabel = _fpsLabel;
@synthesize markerCount = _markerCount;
@synthesize markerList = _markerList;
@synthesize matchButton = _matchButton;
@synthesize refsButton = _refsButton;

- (double)getFPS
{
    double timeInSeconds;
    uint64_t currentTime = mach_absolute_time();
    if(timeStamp.size() < 11)
    {
        timeStamp.push_back(currentTime);
    }
    else
    {
        for(int i = 0 ; i < 10 ; i++)
            timeStamp[i] = timeStamp[i + 1];
        timeStamp[10] = currentTime;
    }
    timeInSeconds = machTimeToSecs(currentTime - timeStamp[0]) / (timeStamp.size() - 1);
    return 1.0/timeInSeconds;
}


- (void)glkView:(GLKView *)view drawInRect:(CGRect)rect
{
    glClear(GL_COLOR_BUFFER_BIT);
    
    [mVisualizer setupOrtho];
    
    static int texID = [mVisualizer generateTexture:NULL];
    [mVisualizer renderTexture:texID withImagePtr:image];
    
    
    [mVisualizer setupFrustum:zNear];
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    
    static GLfloat m[4][4];
    if(CameraPose.rows > 0)
    {
        Mat tmp;
        
        
        //cout<<"t: "<<nowRotation.col(3)<<endl;
        //cout<<"fdfd "<<nowRotation<<endl;
        
        
        //invert(nowRotation, tmp, CV_SVD);
        //        cout<<"4x3 ="<<tmp<<endl;
        
        //        Mat aa = tmp.clone();
        //
        //        tmp.row(0)= aa.row(1);
        //        tmp.row(1)= -aa.row(0);
        
        tmp = CameraPose.t();
        
        for(int i = 0 ; i < 4 ;i++)
        {
            for(int j = 0; j < 4; j++)
                m[i][j] = (float)tmp.at<float>(i, j);
        }
        //        m[3][0] =   m[3][1] =     m[3][2] = 0;
        //        m[3][3] = 1;
        
        glMatrixMode(GL_MODELVIEW);
        glLoadMatrixf((GLfloat *)m);
        
        
        cout<<"camera: "<<tmp<<endl;
    }
    
    // glTranslatef(0, 0, -9);
    if(isCapture)
    {
        for(int i = 0; i < featureTrack.markers.size(); i++)
            if(featureTrack.markers[i].registed)
                [mVisualizer renderCube:featureTrack.markers[i].center];
        //[mVisualizer renderGrid];
        
        [mVisualizer setupVideoOrtho];
        
    }
}



- (void) refsButton: (UIButton *) button
{
    featureMatch.addRef(imageObject.clone());
    CMRotationMatrix CR = [self getRotationMatrix];
    Mat R;
    R = Solve3D::setRotationMatrix(CR);
    featureTrack.setKMatrix(480, 360, focus);
    featureTrack.addMarker(imageObject, R);
    markerNum++;
}

- (void) matchButton: (UIButton *) button
{
    AVCaptureConnection *videoConnection = nil;
    for (AVCaptureConnection *connection in _stillImageOutput.connections) {
        for (AVCaptureInputPort *port in [connection inputPorts]) {
            if ([[port mediaType] isEqual:AVMediaTypeVideo] ) {
                videoConnection = connection;
                break;
            }
        }
        if (videoConnection) { break; }
    }
    
    if(videoConnection == nil)
        return;
    
    [_stillImageOutput captureStillImageAsynchronouslyFromConnection:videoConnection completionHandler:^(CMSampleBufferRef imageDataSampleBuffer, NSError *error) {
        
        isTrain = YES;
        NSData *imageData = [AVCaptureStillImageOutput jpegStillImageNSDataRepresentation:imageDataSampleBuffer];
        photoImage= [[UIImage alloc]initWithData:imageData];
        isMatch = YES;
        
        featureMatch.train();
        isTrain = NO;
    }];
}



- (void)initUI
{
    _numberLabel = [[UILabel alloc]initWithFrame:CGRectMake(50, 50, 150, 50)];
    [self.view addSubview: _numberLabel];
    _numberLabel.backgroundColor = [UIColor yellowColor];
    
    _fpsLabel = [[UILabel alloc]initWithFrame:CGRectMake(250, 50, 150, 50)];
    [self.view addSubview: _fpsLabel];
    _fpsLabel.backgroundColor = [UIColor yellowColor];
    
    _markerCount = [[UILabel alloc]initWithFrame:CGRectMake(50, 150, 150, 50)];
    [self.view addSubview: _markerCount];
    _markerCount.backgroundColor = [UIColor yellowColor];
    
    _markerList = [[UILabel alloc]initWithFrame:CGRectMake(250, 150, 150, 50)];
    [self.view addSubview: _markerList];
    _markerList.backgroundColor = [UIColor yellowColor];
    
    _matchButton = [[UIButton alloc]initWithFrame:CGRectMake(750, 600, 100, 50)];
    [_matchButton setTitle:@"Match" forState:UIControlStateNormal];
    [_matchButton setTitleColor:[UIColor redColor] forState:UIControlStateNormal];
    [_matchButton addTarget:self action:@selector(matchButton:) forControlEvents:UIControlEventTouchUpInside];
    [self.view addSubview:_matchButton];
    
    _refsButton = [[UIButton alloc]initWithFrame:CGRectMake(50, 600, 100, 50)];
    [_refsButton setTitle:@"SetRefs" forState:UIControlStateNormal];
    [_refsButton setTitleColor:[UIColor redColor] forState:UIControlStateNormal];
    [_refsButton addTarget:self action:@selector(refsButton:) forControlEvents:UIControlEventTouchUpInside];
    [self.view addSubview:_refsButton];
}

- (void)show
{
    _numberLabel.text = [NSString stringWithFormat:@"匹配点个数:%d", keyPointSize];
    _fpsLabel.text = [NSString stringWithFormat:@"FPS = %3.2f",[self getFPS]];
    _markerCount.text = [NSString stringWithFormat:@"%3.2f / %3.2f",detectfps, matchfps];
    _markerList.text = [NSString stringWithFormat:@"marker:%d",markerNum];
}

- (void)setupCameraSession
{
    //Session
    _captureSession = [AVCaptureSession new];
    [_captureSession beginConfiguration];
    [_captureSession setSessionPreset:AVCaptureSessionPresetMedium];
    
    //Capture device
    AVCaptureDevice *inputDevice = [AVCaptureDevice defaultDeviceWithMediaType:AVMediaTypeVideo];
    
    NSError *error;
    
    if([inputDevice lockForConfiguration:&error])
    {
        //inputDevice.whiteBalanceMode = AVCaptureWhiteBalanceModeLocked;
        inputDevice.focusMode = AVCaptureFocusModeLocked;
        //inputDevice.exposureMode = AVCaptureExposureModeLocked;
        [inputDevice unlockForConfiguration];
    }
    
    double fov = inputDevice.activeFormat.videoFieldOfView;
    int height = 360;
    int halfHeight = height/2;
    focus = halfHeight/tan(fov * 3.14159265 / 180 * 0.5);
    
    zNear = focus/halfHeight;
    
    //Device input
    AVCaptureDeviceInput *deviceInput = [AVCaptureDeviceInput deviceInputWithDevice:inputDevice error:&error];
    if([_captureSession canAddInput:deviceInput])
        [_captureSession addInput:deviceInput];
    
    //Preview
    _customPreviewerLayer = [CALayer layer];
    _customPreviewerLayer.bounds = CGRectMake(0, 0, self.view.frame.size.width, self.view.frame.size.height);
    _customPreviewerLayer.position = CGPointMake(self.view.frame.size.width/2, self.view.frame.size.height/2);
   // _customPreviewerLayer.affineTransform = CGAffineTransformMakeRotation(M_PI/2.0);
    [self.view.layer addSublayer:_customPreviewerLayer];
    
    //Video output
    _dataOutput = [AVCaptureVideoDataOutput new];
    _dataOutput.videoSettings = [NSDictionary dictionaryWithObject:[NSNumber numberWithUnsignedInt:kCVPixelFormatType_32BGRA] forKey:(NSString *)kCVPixelBufferPixelFormatTypeKey];
    [_dataOutput setAlwaysDiscardsLateVideoFrames:YES];
    
    if([_captureSession canAddOutput:_dataOutput])
        [_captureSession addOutput:_dataOutput];
    
    //Image output
    _stillImageOutput = [[AVCaptureStillImageOutput alloc] init];
    NSDictionary *outputSettings = [[NSDictionary alloc] initWithObjectsAndKeys:AVVideoCodecJPEG,AVVideoCodecKey, nil];
    [_stillImageOutput setOutputSettings:outputSettings];
    if([_captureSession canAddOutput:_stillImageOutput])
        [_captureSession addOutput:_stillImageOutput];
    
    [_captureSession commitConfiguration];
    
    dispatch_queue_t queue = dispatch_queue_create("VideoQueue", DISPATCH_QUEUE_SERIAL);
    [_dataOutput setSampleBufferDelegate:self queue:queue];
    
}


-(void)initGL
{
    EAGLContext* context = [[EAGLContext alloc] initWithAPI:kEAGLRenderingAPIOpenGLES1];
    mGLView = [[GLKView alloc]initWithFrame:[UIScreen mainScreen].bounds];
    mGLView.context = context;
    mGLView.delegate = self;
    
    [self.view addSubview:mGLView];
    [EAGLContext setCurrentContext:mGLView.context];
    glClearColor(1, 0, 0, 0);
    
}

- (void)viewDidLoad {
    [super viewDidLoad];
    
    [self initGL];
    mFocal = 560;
    mVisualizer = [[Visualizer alloc] initWithSize:480 andHeight:360 andFocal:mFocal];
    image = new GLubyte[360*480*4];
    _motionManager = [[CMMotionManager alloc]init];
    [_motionManager startDeviceMotionUpdates];
    initModule_nonfree();
    featureMatch = Feature_Match("SURF", 7,2);
    featureTrack = Feature_Track("CV_BRISK");
    markerNum = 0;
    searchNum = 2;
    isTrain = NO;
    isMatched = NO;
    backgroundIsRun = NO;
    
    [self setupCameraSession];
    
    [self initUI];
    
    [_captureSession startRunning];
}

- (void)addContent:(int) keypointSize
{
    _numberLabel.text = [NSString stringWithFormat:@"特征点：%d", keypointSize];
    _fpsLabel.text = [NSString stringWithFormat:@"FPS = %3.2f",[self getFPS]];
    _markerCount.text = [NSString stringWithFormat:@"marker个数: %d",markerNum];
}

- (void)addContent:(int) keypointSize with: (int) matchpointSize with: (double) detectfps with: (double) matchfps
{
    _numberLabel.text = [NSString stringWithFormat:@"匹配点：%d/%d", keypointSize,matchpointSize];
    _fpsLabel.text = [NSString stringWithFormat:@"FPS=%3.2f/%3.2f",detectfps,matchfps];
    _markerCount.text = [NSString stringWithFormat:@"marker个数: %d",markerNum];
    NSString *list = [NSString stringWithFormat:@"匹配列表："];
    if(markerIdx.size() > 0)
    {
        for(int i = 0; i < searchNum; i++)
            list = [list stringByAppendingString:[NSString stringWithFormat:@" %d",markerIdx[i]]];
    }
    _markerList.text = list;
}

- (CMRotationMatrix)getRotationMatrix{
    CMDeviceMotion *motion = _motionManager.deviceMotion;
    CMAttitude *attitude = motion.attitude;
    return attitude.rotationMatrix;
}


- (void)captureOutput:(AVCaptureOutput *)captureOutput didOutputSampleBuffer:(CMSampleBufferRef)sampleBuffer       fromConnection:(AVCaptureConnection *)connection
{
    if(isTrain == YES)
        return;
    //convert to cv::Mat
    UIImage *imageOutput = [OpenCVController imageFromSampleBuffer:sampleBuffer];
    CVImageBufferRef imageBuffer = CMSampleBufferGetImageBuffer(sampleBuffer);
    CVPixelBufferLockBaseAddress(imageBuffer,0);
    memcpy( image , (GLubyte *)CVPixelBufferGetBaseAddress(imageBuffer) , 360*480*4);
    CVPixelBufferUnlockBaseAddress(imageBuffer, 0);
    Mat matOutput = [OpenCVController cvMatFromUIImage:imageOutput];
    if(isMatch == NO)
    {
        imageObject = matOutput;
    }

    int matchpointSize = 0;
    CMRotationMatrix CR = [self getRotationMatrix];
    Mat R;
    R = Solve3D::setRotationMatrix(CR);
    if(isMatch == NO)
    {
//        drawFeature(keyPointSize, matOutput, featureMatch);
    }
    else
        isCapture = drawMatcher(keyPointSize, matchpointSize, matOutput, detectfps, matchfps, featureMatch, markerNum, backThread, isMatched, featureTrack, backgroundIsRun,R,CameraPose);
    
    //convert to CGImageRef
    imageOutput = [OpenCVController UIImageFromCVMat:matOutput];
    CGImageRef dstImageFilter = imageOutput.CGImage;
    
    //show image
    dispatch_sync(dispatch_get_main_queue(), ^{
        [self show];
        [mGLView display];
        return ;
        //_customPreviewerLayer.contents = (__bridge id)dstImageFilter;
        if(isMatch == NO)
            [self addContent:keyPointSize];
        else
            [self addContent:keyPointSize with:matchpointSize with:detectfps with:matchfps];
    });
}

- (void)didReceiveMemoryWarning {
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}

@end
