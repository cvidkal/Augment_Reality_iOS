#ifndef __FEATURE_TRACK_H__
#define __FEATURE_TRACK_H__

#include "headers.h"
#include <opencv2\flann\flann.hpp>
#include <opencv2\flann\lsh_index.h>
#include <opencv2\flann\miniflann.hpp>
#include "keyframe.h"
#include "Solve3D.h"

class Feature_Track
{
public:
	Feature_Track(string feature);
	void setKMatrix(double imageWidth, double imageHeight, double focus);//init the K
	void match(Mat query, Matches &matches, double &detectfps, double &matchfps);
	Keyframe Feature_Track::track(Mat frame);
	void setRef(Mat ref);
	~Feature_Track(){};

private:

	Mat KMatrix;
	vector<Keyframe> keyframes;
	vector<Marker> markers;

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
};

#endif 