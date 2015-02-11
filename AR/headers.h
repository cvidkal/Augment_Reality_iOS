#ifndef __HEADERS_H__
#define __HEADERS_H__

#define _CRT_SECURE_NO_WARNINGS

#include <opencv2\features2d\features2d.hpp>
#include <opencv2\core\core.hpp>
#include <iostream>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\nonfree\features2d.hpp>
#include <opencv2\nonfree\nonfree.hpp>
#include <time.h>
#include <ANN\ANN.h>
#include <opencv2\calib3d\calib3d.hpp>
#include <fstream>

#include <cstdio>
#include <cstring>

#include "cascade hash\Share.h"
#include "cascade hash\Timer.h"
#include "cascade hash\DataPreProcessor.h"
#include "cascade hash\HashConvertor.h"
#include "cascade hash\BucketBuilder.h"
#include "cascade hash\MatchPairLoader.h"
#include "cascade hash\CasHashMatcher.h"
#include <opencv2\brisk\brisk.h>
#include <opencv2\brisk\hammingsse.hpp>
#include <opencv2\contrib\contrib.hpp>




using namespace cv;
using namespace std;


struct Matches
{
	int count;//the number of matched keypoints
	vector<Point2f> trainPts;//The coordinate of keypoints in train image.
	vector<Point2f> queryPts;//The coordinate of keypoints in query image.
	vector<int> trainIdxs;// The indices of matched training keypoints in the original keypoints array.
	int refIdx;//This is the index of reference frame which matches best for the current frame. 
	float *data;  //data包含count组数据，每组4个float，(x_1,y_1,x_2,y_2)，如果第一帧那么x_1=x_2,y_1=y_2
};

typedef std::vector<std::pair<int, int> > MatchList; // SIFT point match list between two images


#endif
