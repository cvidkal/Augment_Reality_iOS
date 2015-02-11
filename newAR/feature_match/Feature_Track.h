#ifndef __FEATURE_TRACK_H__
#define __FEATURE_TRACK_H__

#include "headers.h"
#include <opencv2/flann/flann.hpp>
#include "keyframe.h"
#include "Solve3D.h"
#include "Marker.h"

class Feature_Track
{
public:
	Feature_Track(string feature);
    Feature_Track(){};
	void setKMatrix(double imageWidth, double imageHeight, double focus);//init the K
	void match(Mat query, Matches &matches, double &detectfps, double &matchfps);
    bool track(Mat &frame,Mat & C_GL,Mat RIMU,double &detectfps,double &matchfps,int &keypointSize,int &keyframes);
	void setRef(Mat ref,int markerNum,Mat R);
    void addMarker(Mat frame,Mat R);
    ~Feature_Track(){};
    vector<Marker> markers;
    

private:
    bool isKeyFrame(Mat R,Mat t);
    int searchKeyFrame(int currentNumber,Mat R,Mat t);
	Mat KMatrix;
    bool isSet = false;
	vector<Keyframe> keyframes;

	Ptr<FeatureDetector> create_detector(string feature);
	Ptr<DescriptorExtractor> create_descriptor(string feature);

	Ptr<FeatureDetector> detector;
	Ptr<DescriptorExtractor> extractor;
	Ptr<DescriptorMatcher> matcher;
	string _featureType;
	Mat _ref;
	Mat desp;
	vector<KeyPoint> trainPts;
	vector<Point3f> trainPts3D;
    int keyFrameNumber;
    bool isTracked;
};

#endif 