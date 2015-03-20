#ifndef __KEYFRAME_H__
#define __KEYFRAME_H__

#include "headers.h"




class Keyframe
{
public:
	Keyframe();
	~Keyframe();
	Mat img;
	Mat R;
	Mat t;
	Mat descriptor;
//	vector<Point3d> pointSet3D;
	vector<pair<int, int>> indices;//(-1,-1) means a invalid index
    vector<int> markerIdx;
	vector<KeyPoint> keyPoints;
    vector<Point2f> pts2D;
    vector<Point3f> pts3D;
	
private:

};



#endif
