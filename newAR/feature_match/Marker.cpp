//
//  Marker.cpp
//  newAR
//
//  Created by ZJU_CVG on 15/1/29.
//  Copyright (c) 2015年 Shangjin Zhai. All rights reserved.
//

#include "Marker.h"


Marker::Marker()
{
    registed = false;
}

Marker::~Marker()
{
}

Point3f Marker::getPoint3D(int i)
{
    return points3D1[i];
}