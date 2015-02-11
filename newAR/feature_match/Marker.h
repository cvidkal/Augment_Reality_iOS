//
//  Marker.h
//  newAR
//
//  Created by ZJU_CVG on 15/1/29.
//  Copyright (c) 2015年 Shangjin Zhai. All rights reserved.
//

#ifndef newAR_Marker_h
#define newAR_Marker_h

#include "headers.h"

class Marker
{
public:
    Marker();
    Marker(vector<Point3f> _points3D);
    Point3f getPoint3D(int i);
    ~Marker();
    vector<Point3f> points3D1;
    vector<Point3f> points3D2;
    bool registed;
    Mat descriptor;
    vector<KeyPoint> pts2d;
    Point3f center;
private:
};



#endif
