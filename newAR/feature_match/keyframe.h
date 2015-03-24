#ifndef __KEYFRAME_H__
#define __KEYFRAME_H__
#include <opencv2/core/core.hpp>


using namespace cv;
using namespace std; 

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
//    bool a;

//    vector<Point3f> points3D1;
//    vector<Point3f> points3D2;
//    bool registed;
//    vector<KeyPoint> keypoints;
//    vector<Point2f> pts2d;
//    vector<vector<pair<int, int>>> invertIdx;
//    Point3f center;
    
	
private:

};



#endif
