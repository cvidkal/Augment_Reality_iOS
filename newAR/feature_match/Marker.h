//
//  Marker.h
//  newAR
//
//  Created by ZJU_CVG on 15/1/29.
//  Copyright (c) 2015年 Shangjin Zhai. All rights reserved.
//

#ifndef newAR_Marker_h
#define newAR_Marker_h

#include <opencv2/core/core.hpp>


using namespace std;
using namespace cv;

class Marker
{
public:
    Marker();
    ~Marker();
    Marker(const Marker&);
    void reset();
    bool registed;
    
    
    vector<Point3f> points3D1;
    vector<Point3f> points3D2;
    Mat descriptor;
    Mat img;
    vector<KeyPoint> keypoints;
    vector<Point2f> pts2d;
    vector<vector<pair<int, int>>> invertIdx;
    Point3f center;
    
private:

};



#endif
