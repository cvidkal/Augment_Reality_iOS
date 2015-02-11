//
//  DrawImage.h
//  newAR
//
//  Created by Shangjin Zhai on 14-11-22.
//  Copyright (c) 2014年 Shangjin Zhai. All rights reserved.
//

#ifndef __newAR__DrawImage__
#define __newAR__DrawImage__

#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <future>
#include <thread>
#include <chrono>
#include "feature_match.h"
#include "Feature_Track.h"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "ViewController.h"

using namespace cv;
using namespace std;

void drawFeature(int &keypointSize, Mat &matOutput, Feature_Match &featureMatch);

bool drawMatcher(int &keypointSize, int &matchpointSize, Mat &matOutput, double &detectfps, double &matchfps,Feature_Match &featureMatch, int &markerIdx, future<Mat> &backThread, bool &isMatched, Feature_Track &featureTrack, bool &backgroundIsRun,Mat IMUR,Mat &CameraPose);
#endif /* defined(__newAR__DrawImage__) */
