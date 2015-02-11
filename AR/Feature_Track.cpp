#include "Feature_Track.h"


Ptr<FeatureDetector> Feature_Track::create_detector(string feature)
{
	Ptr<FeatureDetector> ret;
	if (feature == "SIFT")
	{
		int nFeature = 500;
		int noctaves = 3;
		double contrastThreshold = 0.04;
		double edgeThreshold = 10;
		double sigma = 1.6;
		ret = new SiftFeatureDetector(nFeature, noctaves, contrastThreshold, edgeThreshold, sigma);
	}
	else if (feature == "SURF")
	{
		double hessianThreshold = 300;
		int nOctaves = 4;
		int nOctavesLayers = 2;
		ret = new SurfFeatureDetector(hessianThreshold, nOctaves, nOctavesLayers);
	}
	else if (feature == "BRIEF")
	{
		int threshold = 40;
		ret = new FastFeatureDetector(threshold);
	}
	else if (feature == "ORB")
	{
		int nFeatures = 500;
		float scaleFactor = 1.2f;
		int nlevels = 8;
		int edgeThreshold = 31;
		ret = new OrbFeatureDetector(nFeatures, scaleFactor, nlevels, edgeThreshold);
	}
	else if (feature == "BRISK")
	{
		int thresh = 60;
		int octaves = 4;
		ret = new BriskFeatureDetector(thresh, octaves);
	}
	else if (feature == "CV_BRISK")
	{
		int nFeatures = 500;
		float scaleFactor = 1.2f;
		int nlevels = 8;
		int edgeThreshold = 31;
		ret = new OrbFeatureDetector(nFeatures, scaleFactor, nlevels, edgeThreshold);

	}
	return ret;
}
Ptr<DescriptorExtractor> Feature_Track::create_descriptor(string feature)
{
	Ptr<DescriptorExtractor> ret;
	if (feature == "SIFT")
	{
		int nFeature = 500;
		int noctaves = 3;
		double contrastThreshold = 0.04;
		double edgeThreshold = 10;
		double sigma = 1.6;
		ret = new SiftDescriptorExtractor(nFeature, noctaves, contrastThreshold, edgeThreshold, sigma);
	}
	else if (feature == "SURF")
	{
		double hessianThreshold = 300;
		int nOctaves = 4;
		int nOctavesLayers = 2;
		ret = new SurfDescriptorExtractor(hessianThreshold, nOctaves, nOctavesLayers);
	}
	else if (feature == "BRIEF")
	{
		ret = new BriefDescriptorExtractor();
	}
	else if (feature == "ORB")
	{
		int nFeatures = 500;
		float scaleFactor = 1.2f;
		int nlevels = 8;
		int edgeThreshold = 31;
		ret = new OrbDescriptorExtractor(nFeatures, scaleFactor, nlevels, edgeThreshold);
	}
	else if (feature == "BRISK")
	{
		ret = new BriskDescriptorExtractor();
	}
	else if (feature == "CV_BRISK")
	{
		ret = DescriptorExtractor::create("BRISK");
	}
	return ret;
}



Feature_Track::Feature_Track(string feature)
{
	_featureType = feature;
	detector = create_detector(feature);
	extractor = create_descriptor(feature);
	matcher = DescriptorMatcher::create("BruteForce-Hamming");
	Ptr<flann::IndexParams> indexParams = new flann::LshIndexParams(20,20,2);
}

void Feature_Track::setKMatrix(double imageWidth, double imageHeight, double focus)
{
	KMatrix = Mat::zeros(3, 3, CV_32F);
	KMatrix.at<double>(0, 0) = focus;
	KMatrix.at<double>(1, 1) = focus;
	KMatrix.at<double>(2, 2) = 1;
	KMatrix.at<double>(0, 2) = (imageHeight - 1) / 2;
	KMatrix.at<double>(1, 2) = (imageWidth - 1) / 2;
}


void Feature_Track::setRef(Mat ref)
{
	Keyframe kf;
	kf.img = ref.clone();
	detector->detect(kf.img, kf.keyPoints);
	extractor->compute(kf.img, kf.keyPoints, kf.descriptor);
	Marker marker;
	Solve3D::get3DPoints(KMatrix, kf.R, kf.keyPoints, marker.points3D, kf.t);
	for (int i = 0; i < kf.keyPoints.size(); i++)
	{
		kf.indices.push_back(pair<int, int>(0, i));
	}
	markers.push_back(marker);
	keyframes.push_back(kf);

}

void Feature_Track::match(Mat query, Matches &matches, double &detectfps, double &matchfps)
{
	vector<KeyPoint> queryPts;
	vector<vector<DMatch>> dmatches;
	Mat desp2;
	detector->detect(query, queryPts);
	extractor->compute(query, queryPts, desp2);
	matcher->knnMatch(desp2, desp, dmatches, 2);

	matches.count = 0;
	for (int i = 0; i < dmatches.size(); i++)
	{
		if ((dmatches[i][0].distance < dmatches[i][1].distance*0.6) && dmatches[i][0].distance < 90)
		{
			matches.trainPts.push_back(trainPts[dmatches[i][0].trainIdx].pt);
			matches.queryPts.push_back(queryPts[dmatches[i][0].queryIdx].pt);
			matches.count++;
		}
	}
}

Keyframe Feature_Track::track(Mat frame)
{
	Keyframe tmp;
	int keyFrameNumber = 0;
	detector->detect(frame, tmp.keyPoints);
	extractor->compute(frame, tmp.keyPoints, tmp.descriptor);
	vector<vector<DMatch>> dmatches;
	matcher->knnMatch(tmp.descriptor, keyframes[keyFrameNumber].descriptor, dmatches, 2);
	Mat pts3D, pts2D;
	for (int i = 0; i < dmatches.size(); i++)
	{
		if ((dmatches[i][0].distance < dmatches[i][1].distance*0.6) && dmatches[i][0].distance < 90)
		{
			if (keyframes[keyFrameNumber].indices[dmatches[i][0].trainIdx].first != -1)
			{
				pts3D.push_back(Mat(markers[keyframes[keyFrameNumber].indices[dmatches[i][0].trainIdx].first].getPoint3D(keyframes[keyFrameNumber].indices[dmatches[i][0].trainIdx].second)));
				pts2D.push_back(Mat(tmp.keyPoints[dmatches[i][0].queryIdx].pt));
			}
		}
	}
	Solve3D::getCameraRT(KMatrix, pts3D, pts2D, tmp.R, tmp.t);
	return tmp;
}