//
//  OpenCVController.h
//  avfoundation
//
//  Created by Shangjin Zhai on 14-10-30.
//  Copyright (c) 2014年 Shangjin Zhai. All rights reserved.
//

#ifndef __avfoundation__OpenCVController__
#define __avfoundation__OpenCVController__

#import <Foundation/Foundation.h>
#import <AVFoundation/AVFoundation.h>
#import <UIKit/UIKit.h>
#import <opencv2/opencv.hpp>
#import "opencv2/nonfree/features2d.hpp"
#import "opencv2/nonfree/nonfree.hpp"
#import "opencv2/legacy/legacy.hpp"

@interface OpenCVController : NSObject
+(cv::Mat)cvMatFromUIImage:(UIImage *)image;
+(UIImage *)UIImageFromCVMat:(cv::Mat)cvMat;
+(UIImage *)imageFromSampleBuffer:(CMSampleBufferRef) sampleBuffer;
@end

#endif /* defined(__avfoundation__OpenCVController__) */
