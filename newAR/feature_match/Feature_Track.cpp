#include "Feature_Track.h"
#include <fstream>

ofstream fout("t.txt");


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
//		ret = new BriskFeatureDetector(thresh, octaves);
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
//		ret = new BriskDescriptorExtractor();
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
}

void Feature_Track::setKMatrix(double imageWidth, double imageHeight, double focus)
{
    if(isSet)
        return;
	KMatrix = Mat::zeros(3, 3, CV_32F);
	KMatrix.at<float>(0, 0) = focus;
	KMatrix.at<float>(1, 1) = focus;
	KMatrix.at<float>(2, 2) = 1;
	KMatrix.at<float>(0, 2) = (imageHeight - 1) / 2;
	KMatrix.at<float>(1, 2) = (imageWidth - 1) / 2;
}


void Feature_Track::setRef(Mat ref,int markerNum,Mat R)
{
    
	Keyframe kf;
	kf.img = ref.clone();
    kf.R  = R.clone();
	detector->detect(kf.img, kf.keyPoints);
	extractor->compute(kf.img, kf.keyPoints, kf.descriptor);
    desp = kf.descriptor;
    trainPts = kf.keyPoints;
	Marker marker;
	Solve3D::get3DPoints(KMatrix, kf.R, kf.keyPoints, kf.pts3D, kf.t);
    
    
    vector<vector<DMatch>> dmatches;
    matcher->knnMatch(kf.descriptor, markers[markerNum].descriptor, dmatches, 2);
    vector<Point3f> pts3d1,pts3d2;
    for (int i = 0; i < dmatches.size(); i++)
    {
        if ((dmatches[i][0].distance < dmatches[i][1].distance*0.6) && dmatches[i][0].distance < 90)
        {
            pts3d1.push_back(kf.pts3D[dmatches[i][0].queryIdx]);
            pts3d2.push_back(markers[markerNum].points3D2[dmatches[i][0].trainIdx]);
 //           mask.push_back(dmatches[i][0].trainIdx);
        }
 //           mask.push_back(-1);
    }
    double s;
    Mat _R,_t;
    Mat _pts3d1(pts3d1),_pts3d2(pts3d2);
    Solve3D::solveSRT(_pts3d1, _pts3d2, s, _R, _t);
    Solve3D::transformSRT(markers[markerNum].points3D2, markers[markerNum].points3D1, s, _R, _t);
    markers[markerNum].registed = true;

	keyframes.push_back(kf);
    keyFrameNumber = 0;
    
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

bool Feature_Track::isKeyFrame(Mat R,Mat t)
{
    double threshold = 0.6*0.6;
    double minDist = 10000000;
    vector<double> dists;
    for(int i =0;i<keyframes.size();i++)
    {
        double dist = norm(t-keyframes[i].t,NORM_L2SQR);
        if(dist<minDist)
        {
            minDist = dist;
        }
        dists.push_back(dist);
        if(dist<threshold)
        {
            return false;
        }
    }
    cout<<minDist<<endl;
    cout<<t-keyframes[keyframes.size()-1].t<<endl;
    return true;
}

int Feature_Track::searchKeyFrame(int currentNumber,Mat R,Mat t)
{
    /*
    double minDist1 = -1,minDist2=-1;
    int mini1,mini2;
    
    for(int i=0;i<keyframes.size();i++)
    {
        double dist = norm(t-keyframes[i].t,NORM_L2SQR);
        if(dist<minDist2||minDist2 == -1)
        {
            minDist2 = dist;
            mini2 = i;
        }
        if(minDist2<minDist1||minDist1 == -1)
        {
            std::swap(minDist1, minDist2);
            std::swap(mini1, mini2);
        }
    }
    cout<< mini1<<" "<<mini2<<endl;
    cout<<minDist1<<" "<<minDist2<<endl;

    if(minDist1<minDist2*0.6)
    {
        return mini1;
    }
    else
    {
        return -1;
    }
     */
    double minDist = -1;
    int mini;
    vector<double> dists;
    for(int i=0;i<keyframes.size();i++)
    {
        double dist = norm(t-keyframes[i].t,NORM_L2SQR);
        dists.push_back(dist);
        if(dist<minDist||minDist == -1)
        {
            minDist = dist;
            mini = i;
        }
    }
    if(mini != currentNumber)
    {
        cout<<minDist<<"--"<<dists[currentNumber]<<endl;
        if(minDist<dists[currentNumber]*0.6)
        {
            return mini;
        }
    }
    return currentNumber;
}

void Feature_Track::addMarker(Mat frame,Mat R)
{
    Marker a;
    detector->detect(frame, a.pts2d);
    extractor->compute(frame, a.pts2d, a.descriptor);
    Mat t;
    Solve3D::get3DPoints(KMatrix, R, a.pts2d, a.points3D2, t);
    markers.push_back(a);
}


bool Feature_Track::track(Mat &frame,Mat & C_GL,Mat RIMU,double &detectfps,double &matchfps,int &keypointSize,int & keyframeCnt)
{
    if(!isTracked)
    {
 //       keyFrameNumber = searchKeyFrame(RIMU, t)
        
    }
    
    keyframeCnt = keyFrameNumber;
    TickMeter tm;
	Keyframe tmp;
    tm.start();
	detector->detect(frame, tmp.keyPoints);
	extractor->compute(frame, tmp.keyPoints, tmp.descriptor);
    tm.stop();
    detectfps = 1/tm.getTimeSec();
    tm.reset();
    tm.start();
    vector<vector<DMatch>> dmatches;
	matcher->knnMatch(tmp.descriptor, keyframes[keyFrameNumber].descriptor, dmatches, 2);
    tm.stop();
    matchfps = 1/tm.getTimeSec();
    Mat pts3D, pts2D;
    vector<Point3f> pts3d;vector<Point2f> pts2d;
    vector<Point2f> pts2dt;
    for (int i = 0; i < dmatches.size(); i++)
	{
		if ((dmatches[i][0].distance < dmatches[i][1].distance*0.6) && dmatches[i][0].distance < 90)
		{
            pts3d.push_back(keyframes[keyFrameNumber].pts3D[dmatches[i][0].trainIdx]);
            pts2d.push_back(tmp.keyPoints[dmatches[i][0].queryIdx].pt);
            pts2dt.push_back(keyframes[keyFrameNumber].keyPoints[dmatches[i][0].trainIdx].pt);

		}
	}
    Mat inliers;
    findFundamentalMat(pts2d, pts2dt,inliers);
    for(int i=0;i<pts2d.size();i++)
    {
        if(inliers.at<char>(i,0))
        {
            Mat temp =Mat(keyframes[keyFrameNumber].pts3D[dmatches[i][0].trainIdx]);
            temp = temp.t();
            pts3D.push_back(temp);
            temp =Mat(tmp.keyPoints[dmatches[i][0].queryIdx].pt);
            temp = temp.t();
            pts2D.push_back(temp);
        }
    }
    
    keypointSize = pts3D.rows;
    cout<<keyFrameNumber<<endl;
    if(pts3D.rows<20)
    {
        isTracked = false;
        return false;
    }
    
    Solve3D::getCameraRT(KMatrix, pts3D, pts2D, tmp.R, tmp.t);
    
    cout<<tmp.t <<endl;

    if (isKeyFrame(tmp.R, tmp.t))
    {
        Solve3D::get3DPoints(KMatrix, tmp.R, tmp.t, tmp.keyPoints,tmp.pts3D);
        cout<<tmp.t<<endl;
        keyframes.push_back(tmp);
//        cout<<"keyframes numbers"<<keyframes.size()<<endl;
    }
 
//    double lowx, lowy, highx, highy;
//    Solve3D::getHighAndLow(pts3d, lowx, lowy, highx, highy);
//    Solve3D::printPlane(KMatrix, tmp.R, tmp.t, frame, lowx, lowy, highx, highy);
    
    C_GL = Solve3D::getP(tmp.R, tmp.t);
    keyFrameNumber = searchKeyFrame(keyFrameNumber, tmp.R, tmp.t);
//    if(q>=0)
//    {
 //       keyFrameNumber = q;
 //   }
    isTracked = true;
    return true;
}