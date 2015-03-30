#include "Feature_Track.h"
#include <fstream>
#include <opencv2/core/core.hpp>
#include <algorithm>


inline bool RoiPredicate(const float minX, const float minY, const float maxX, const float maxY, const KeyPoint& keyPt)
{
    const Point2f& pt = keyPt.pt;
    return (pt.x < minX) || (pt.x >= maxX) || (pt.y < minY) || (pt.y >= maxY);
}

static inline float getScale(int level, int firstLevel, double scaleFactor)
{
    return (float)std::pow(scaleFactor, (double)(level - firstLevel));
}

static float IC_Angle(const Mat& image, const int half_k, Point2f pt,
                      const vector<int> & u_max)
{
    int m_01 = 0, m_10 = 0;
    
    const uchar* center = &image.at<uchar>(cvRound(pt.y), cvRound(pt.x));
    
    // Treat the center line differently, v=0
    for (int u = -half_k; u <= half_k; ++u)
        m_10 += u * center[u];
    
    // Go line by line in the circular patch
    int step = (int)image.step1();
    for (int v = 1; v <= half_k; ++v)
    {
        // Proceed over the two lines
        int v_sum = 0;
        int d = u_max[v];
        for (int u = -d; u <= d; ++u)
        {
            int val_plus = center[u + v*step], val_minus = center[u - v*step];
            v_sum += (val_plus - val_minus);
            m_10 += u * (val_plus + val_minus);
        }
        m_01 += v * v_sum;
    }
    
    return fastAtan2((float)m_01, (float)m_10);
}


static void computeOrientation(const Mat& image, vector<KeyPoint>& keypoints,
                               int halfPatchSize, const vector<int>& umax)
{
    // Process each keypoint
    for (vector<KeyPoint>::iterator keypoint = keypoints.begin(),
         keypointEnd = keypoints.end(); keypoint != keypointEnd; ++keypoint)
    {
        keypoint->angle = IC_Angle(image, halfPatchSize, keypoint->pt, umax);
    }
}

void foo(Mat _image, vector<KeyPoint> &keypoints)
{
    Mat image;
    cvtColor(_image, image, CV_BGR2GRAY);
    vector<vector<KeyPoint>> allKeypoints(8);
    vector<pair<int, int>> idxs;
    double scaleFactor = 1.2000000476837158;
    int firstLevel = 0;
    
    for (size_t i = 0; i < keypoints.size(); i++)
    {
        int octave = keypoints[i].octave;
        float scale = getScale(octave, firstLevel, scaleFactor);
        keypoints[i].pt *= 1 / scale;
        allKeypoints[octave].push_back(keypoints[i]);
        idxs.push_back(pair<int, int>(octave, allKeypoints[octave].size() - 1));
    }
    const int HARRIS_BLOCK_SIZE = 9;
    int patchSize = 31;
    int edgeThreshold = 31;
    int halfPatchSize = patchSize / 2;
    int border = std::max(edgeThreshold, std::max(halfPatchSize, HARRIS_BLOCK_SIZE / 2)) + 1;
    int levelsNum = 8;
    
    vector<Mat> imagePyramid(levelsNum), maskPyramid(levelsNum);
    for (int level = 0; level < levelsNum; ++level)
    {
        float scale = 1 / getScale(level, firstLevel, scaleFactor);
        cv::Size sz(cvRound(image.cols*scale), cvRound(image.rows*scale));
        cv::Size wholeSize(sz.width + border * 2, sz.height + border * 2);
        Mat temp(wholeSize, image.type()), masktemp;
        imagePyramid[level] = temp(cv::Rect(border, border, sz.width, sz.height));
        
        // Compute the resized image
        if (level != firstLevel)
        {
            if (level < firstLevel)
            {
                resize(image, imagePyramid[level], sz, 0, 0, INTER_LINEAR);
            }
            else
            {
                resize(imagePyramid[level - 1], imagePyramid[level], sz, 0, 0, INTER_LINEAR);
            }
            copyMakeBorder(imagePyramid[level], temp, border, border, border, border,
                           BORDER_REFLECT_101 + BORDER_ISOLATED);
        }
        else
        {
            copyMakeBorder(image, temp, border, border, border, border,
                           BORDER_REFLECT_101);
        }
    }
    
    // pre-compute the end of a row in a circular patch
    vector<int> umax(halfPatchSize + 2);
    
    int v, v0, vmax = cvFloor(halfPatchSize * sqrt(2.f) / 2 + 1);
    int vmin = cvCeil(halfPatchSize * sqrt(2.f) / 2);
    for (v = 0; v <= vmax; ++v)
        umax[v] = cvRound(sqrt((double)halfPatchSize * halfPatchSize - v * v));
    
    // Make sure we are symmetric
    for (v = halfPatchSize, v0 = 0; v >= vmin; --v)
    {
        while (umax[v0] == umax[v0 + 1])
            ++v0;
        umax[v] = v0;
        ++v0;
    }
    
    keypoints.clear();
    
    for (size_t level = 0; level < levelsNum; level++)
    {
        vector<KeyPoint> & kpts = allKeypoints[level];
        computeOrientation(imagePyramid[level], kpts, halfPatchSize, umax);
        if (level != firstLevel)
        {
            float scale = getScale(level, firstLevel, scaleFactor);
            for (vector<KeyPoint>::iterator keypoint = kpts.begin(),
                 keypointEnd = kpts.end(); keypoint != keypointEnd; ++keypoint)
                keypoint->pt *= scale;
        }
        keypoints.insert(keypoints.end(), kpts.begin(), kpts.end());
    }
    
}




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
    keyframes.resize(20);
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
	KMatrix.at<float>(0, 2) = (imageWidth - 1) / 2;
	KMatrix.at<float>(1, 2) = (imageHeight - 1) / 2;
}



bool Feature_Track::setCS(cv::Mat current, cv::Mat marker, vector<int> markerNum ,cv::Mat R)
{
    /*
    //match correpondence of current frame and warpped marker
    vector<KeyPoint> markerPoints;
    Mat markerDesp;
    Keyframe kf;
    kf.img = current.clone();
    kf.R  = R.clone();
    detector->detect(kf.img, kf.keyPoints);
    extractor->compute(kf.img, kf.keyPoints, kf.descriptor);
    detector->detect(marker, markerPoints);
    extractor->compute(marker, markerPoints, markerDesp);
    vector<vector<DMatch>> dmatches;
    matcher->knnMatch(kf.descriptor, markerDesp, dmatches, 2);
    */
    
    keyframes.clear();
    //The first KeyFrame
    if (markerNum.empty()) {
        return false;
    }
    Keyframe kf;
    int nowKeyFrameNum = 0;
    auto & nowMarker = markers[markerNum[0]];
 //   Keyframe mk;
 //   mk.markerIdx.push_back(markerNum[0]);
 //   mk.img = markers[markerNum[0]].img.clone();
//    mk.descriptor = markers[markerNum[0]].descriptor.clone();
//
 //   for (int i=0;i<markers[markerNum[0]].pts2d.size(); i++)
 //   {
 //       mk.indices.push_back(pair<int,int>(markerNum[0],i));
 //       mk.keyPoints.push_back(markers[markerNum[0]].pts2d[i]);
 //       mk.pts2D.push_back(markers[markerNum[0]].pts2d[i].pt);
 //   }

    kf.img = current.clone();
    kf.R = R.clone();
    detector->detect(kf.img, kf.keyPoints);
    extractor->compute(kf.img, kf.keyPoints, kf.descriptor);
    
    //Use the initial Camera Pose to estimate all 3D points, suppose all keyPoints are on a plane(Height = 5).
    Solve3D::get3DPoints(KMatrix, kf.R, kf.keyPoints, kf.pts3D, kf.t);
    
    kf.markerIdx.push_back(markerNum[0]);
    vector<vector<DMatch>> dmatches;
    for (int i = 0; i < kf.keyPoints.size(); i++)
    {
        kf.indices.push_back(pair<int, int>(-1, -1));
    }
    //for (int j = 0; j < markerNum.size(); j++)
    //{
    matcher->knnMatch(kf.descriptor, nowMarker.descriptor, dmatches, 2);
    
    vector<Point3f> pts3d1, pts3d1_t,pts3d2,pts3d2_t;
    vector<Point2f> pts2d1,pts2d1_t,pts2d2,pts2d2_t;
    //	vector<Point2f> pts2d1, pts2d2;
    vector<int> indices;
    for (int i = 0; i < dmatches.size(); i++)
    {
        if ((dmatches[i][0].distance < dmatches[i][1].distance*0.6) && dmatches[i][0].distance < 90)
        {
            CV_Assert(dmatches[i][0].distance < (0.6*dmatches[i][1].distance));
            indices.push_back(i);
            /*
            bool flag = false;
            int tmp,tmpi;
            kf.indices[i] = pair<int, int>(markerNum[0], dmatches[i][0].trainIdx);
            for(int k = 0; k<markers[markerNum[0]].invertIdx[dmatches[i][0].trainIdx].size();k++)
            {
                auto j =markers[markerNum[0]].invertIdx[dmatches[i][0].trainIdx][k];
                
                if(j.first == markerNum[0])
                {
                    tmp = j.first;
                    tmpi = k;
                    flag = true;
                    break;
                }
            }
            if(flag)
            {
                if(dmatches[i][0].distance<dmatches[tmp][0].distance)
                {
                    markers[markerNum[0]].invertIdx[dmatches[i][0].trainIdx][tmpi].second = i;
                }
            }
            else
                markers[markerNum[0]].invertIdx[dmatches[i][0].trainIdx].push_back(pair<int, int>(markerNum[0], i));
  */
            pts3d1_t.push_back(kf.pts3D[dmatches[i][0].queryIdx]);
            pts3d2_t.push_back(nowMarker.points3D2[dmatches[i][0].trainIdx]);
            pts2d1_t.push_back(kf.keyPoints[dmatches[i][0].queryIdx].pt);
            pts2d2_t.push_back(nowMarker.keypoints[dmatches[i][0].trainIdx].pt);
        }
    }
    Keyframe warpMarker;
    Mat mask;
    if(pts2d1_t.size()<8)
    {
        return false;
    }
    findFundamentalMat(pts2d1_t, pts2d2_t,mask);
    for (int i=0;i<mask.rows; i++) {
        if(mask.at<char>(i,0))
        {
            pts2d1.push_back(pts2d1_t[i]);
            pts2d2.push_back(pts2d2_t[i]);
        }
    }
    if(pts2d1.size()<8)
    {
        return false;
    }
    Mat H = findHomography(pts2d2, pts2d1);
    warpPerspective(nowMarker.img, warpMarker.img, H, nowMarker.img.size());
    for (int i=0; i<nowMarker.keypoints.size(); i++) {
        nowMarker.pts2d.push_back(nowMarker.keypoints[i].pt);
    }
    vector<Point3f> pts2d_H;
    vector<Point3f> pts2d_W;
    convertPointsToHomogeneous(nowMarker.pts2d, pts2d_H);
    for (int i=0; i<nowMarker.pts2d.size(); i++) {
        Point3d a;
        a.x = pts2d_H[i].x, a.y = pts2d_H[i].y,a.z = pts2d_H[i].z;
        Mat aa = H*(Mat(a));
        double *aaa = (double*)aa.data;
        a.x = aaa[0],a.y = aaa[1],a.z = aaa[2];
        pts2d_W.push_back(a);
    }
    convertPointsFromHomogeneous(pts2d_W, warpMarker.pts2D);
    for (int i=0; i<nowMarker.keypoints.size(); i++) {
        KeyPoint kp;
        auto &pt = warpMarker.pts2D[i];
        if(pt.x<0||pt.y<0||pt.x>kf.img.cols-10||pt.y>kf.img.rows-10)
        {
            pt.x = 0;
            pt.y = 0;
        }
        kp.pt = warpMarker.pts2D[i];
        kp.octave = nowMarker.keypoints[i].octave;
        kp.response = nowMarker.keypoints[i].response;
        kp.size = nowMarker.keypoints[i].size;
        warpMarker.keyPoints.push_back(kp);
    }
    foo(warpMarker.img, warpMarker.keyPoints);
    vector<int> idx;
    for(int i=0;i<warpMarker.keyPoints.size();i++)
    {
        unsigned int scale = warpMarker.keyPoints[i].octave;
        const int sizeList_[8] = {48,59,69,85,99,122,143,168};
        // saturate
        
        const int border = sizeList_[scale];
        const int border_x = warpMarker.img.cols - border;
        const int border_y = warpMarker.img.rows - border;
        if(!RoiPredicate((float)border,(float)border,(float)border_x,(float)border_y,warpMarker.keyPoints[i]))
        {
            idx.push_back(i);
        }
    }
    extractor->compute(warpMarker.img, warpMarker.keyPoints, warpMarker.descriptor);
    matcher->knnMatch(kf.descriptor, warpMarker.descriptor, dmatches,2);
    pts2d1_t.clear();
    pts2d2_t.clear();
    indices.clear();
    for (int i = 0; i < dmatches.size(); i++)
    {
        if ((dmatches[i][0].distance < dmatches[i][1].distance*0.6) && dmatches[i][0].distance < 90)
        {
            indices.push_back(i);
            auto trainIdx =idx[dmatches[i][0].trainIdx];
            CV_Assert(dmatches[i][0].distance < (0.6*dmatches[i][1].distance));
            pts2d1_t.push_back(kf.keyPoints[dmatches[i][0].queryIdx].pt);
            pts2d2_t.push_back(nowMarker.keypoints[trainIdx].pt);
            pts3d1_t.push_back(kf.pts3D[dmatches[i][0].queryIdx]);
            pts3d2_t.push_back(nowMarker.points3D2[trainIdx]);
        }
    }
    if(pts2d1_t.size()<8)
    {
        return false;
    }
    findFundamentalMat(pts2d1_t, pts2d2_t,mask);
    pts2d1.clear();
    pts2d2.clear();
    pts3d1.clear();
    pts3d2.clear();
    for(int i=0;i<mask.rows;i++)
    {
        if(mask.at<char>(i,0))
        {
            pts2d1.push_back(pts2d1_t[i]);
            pts2d2.push_back(pts2d2_t[i]);
            pts3d1.push_back(pts3d1_t[i]);
            pts3d2.push_back(pts3d2_t[i]);
            bool flag = false;
            auto trainIdx =idx[dmatches[i][0].trainIdx];
            kf.indices[dmatches[i][0].queryIdx] = pair<int,int>(markerNum[0],trainIdx);
            for(int k = 0; k<markers[markerNum[0]].invertIdx[trainIdx].size();k++)
            {
                auto &f =markers[markerNum[0]].invertIdx[trainIdx][k];
                
                if(f.first == markerNum[0])
                {
                    if(dmatches[i][0].distance < dmatches[f.second][0].distance)
                    {
                        f.second = i;
                        flag = true;
                        break;
                    }
                }
            }
            if(!flag)
            {
                markers[markerNum[0]].invertIdx[trainIdx].push_back(pair<int, int>(nowKeyFrameNum, i));
            }
        }
    }
    /*
    
    vector<int> inliers;
    if(pts3d1_t.size()<15)
        return false;
    Solve3D::getCameraRT(KMatrix, pts3d1_t, pts2d1_t, kf.R, kf.t,inliers, false);
    cout<<kf.R<<kf.t<<endl;
    for(auto j:inliers)
    {
        int i=indices[j];
        pts3d1.push_back(pts3d1_t[j]);
        pts2d1.push_back(pts2d1_t[j]);
        pts3d2.push_back(pts3d2_t[j]);
        bool flag = false;
        int tmp,tmpi;
        kf.indices[i] = pair<int, int>(markerNum[0], dmatches[i][0].trainIdx);
        for(int k = 0; k<markers[markerNum[0]].invertIdx[dmatches[i][0].trainIdx].size();k++)
        {
            auto &f =markers[markerNum[0]].invertIdx[dmatches[i][0].trainIdx][k];
            
            if(f.first == markerNum[0])
            {
                if(dmatches[i][0].distance < dmatches[f.second][0].distance)
                {
                    f.second = i;
                    flag = true;
                    break;
                }
            }
        }
        if(!flag)
        {
            markers[markerNum[0]].invertIdx[dmatches[i][0].trainIdx].push_back(pair<int, int>(1, i));
        }
    }
    */
    
    if (pts3d1.size()<20) {
        return false;
    }
    //Mat i1 = kf.img.clone(), i2 = markers[markerNum[0]].img;
    //for (size_t i = 0; i < pts2d1.size(); i++)
    //{
    //	circle(i1, pts2d1[i], 3, CV_RGB(255, 255, 0));
    //	circle(i2, pts2d2[i], 3, CV_RGB(255, 255, 0));
    //}
    //imshow("img1", i1);
    //imshow("img2", i2);
    //	cvWaitKey(0);
    double s;
    Mat _R, _t;
    Mat _pts3d1(pts3d1), _pts3d2(pts3d2);
    vector<Point3f> pts3d3;
    Solve3D::solveSRT(_pts3d2, _pts3d1, s, _R, _t);
    Solve3D::transformSRT(pts3d2, pts3d3, s, _R, _t);
    
    
    Solve3D::transformSRT(markers[markerNum[0]].points3D2, markers[markerNum[0]].points3D1, s, _R, _t);
    
//    Solve3D::getCameraRT(KMatrix, markers[markerNum[0]].points3D1, mk.pts2D, mk.R, mk.t, inliers, false);
    
    
    vector<int> inliers;
    pts3d1.clear();
    for(int i=0;i<indices.size();i++)
    {
        auto trainIdx =idx[dmatches[i][0].trainIdx];
        if(mask.at<char>(i,0))
            pts3d1.push_back(markers[markerNum[0]].points3D1[trainIdx]);
    }
    Solve3D::getCameraRT(KMatrix, pts3d1, pts2d1, kf.R, kf.t, inliers, false);
    cout<<kf.R<<kf.t<<endl;
    markers[markerNum[0]].registed = true;
    markers[markerNum[0]].center.x = 0;
    markers[markerNum[0]].center.y = 0;
    markers[markerNum[0]].center.z = 0;
    

    int num = markers[markerNum[0]].points3D1.size();
    markers[markerNum[0]].center.x /= num;
    markers[markerNum[0]].center.y /= num;
    markers[markerNum[0]].center.z /= num;
    
    /*}*/
 //   images.push_back(mk.img.clone());
 //   keyframes.push_back(mk);
    keyframes.push_back(kf);
    images.push_back(current.clone());
    keyFrameNumber = 0;
//    Solve3D::sba(KMatrix, keyframes, markers);
    isTracked = false;
    return true;
}
/*
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
*/
void Feature_Track::match(Mat query, Mat train,Matches &matches, double &detectfps, double &matchfps)
{
    vector<KeyPoint> queryPts, trainPts;
    vector<vector<DMatch>> dmatches;
    Mat desp2,desp;
    detector->detect(train, trainPts);
    detector->detect(query, queryPts);
    extractor->compute(query, queryPts, desp2);
    extractor->compute(train, trainPts, desp);
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

bool Feature_Track::isKeyFrame(Mat R,Mat t,int &keyFrameNumber)
{
    //double threshold = 0.6*0.6;
    //double minDist = 10000000;
    //vector<double> dists;
    //for(int i =0;i<keyframes.size();i++)
    //{
    //    double dist = norm(t-keyframes[i].t,NORM_L2SQR);
    //    if(dist<minDist)
    //    {
    //        minDist = dist;
    //    }
    //    dists.push_back(dist);
    //    if(dist<threshold)
    //    {
    //        return false;
    //    }
    //}
    //cout<<minDist<<endl;
    //cout<<t-keyframes[keyframes.size()-1].t<<endl;
    //return true;
    
    
    double minDist = -1;
    double threshold = 0.5*0.5;
    int mini;
    vector<double> dists;
    for (int i = 0; i<keyframes.size(); i++)
    {
        double dist = norm(t - keyframes[i].t, NORM_L2SQR);
        dists.push_back(dist);
        if (dist<minDist || minDist == -1)
        {
            minDist = dist;
            mini = i;
        }
    }
    if (minDist > threshold)
    {
        return true;
    }
    if (mini != keyFrameNumber)
    {
//        cout << minDist << "--" << dists[keyFrameNumber] << endl;
        if (minDist<dists[keyFrameNumber] * 0.6)
        {
            keyFrameNumber = mini;
        }
    }
    return false;

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
    static int idx = 0;
//    markers.resize(markers.size()+1);
    Marker a;
//    Marker &a = markers[idx];

    detector->detect(frame, a.keypoints);
    extractor->compute(frame, a.keypoints, a.descriptor);
    Mat t;
    Solve3D::get3DPoints(KMatrix, R, a.keypoints, a.points3D2, t);
    a.invertIdx.resize(a.keypoints.size());
    a.img = frame.clone();
    idx++;
 /*
    cout<<"#camera track project file"<<endl;
    cout<<"<Image Sequence>"<<endl;
    cout<<"Sequence:.\\0.jpg"<<endl;
    cout<<"start:0"<<endl;
    cout<<"step:1"<<endl;
    cout<<"end:"<<0<<endl;
    cout<<"</Image Sequence>"<<endl;
    
    cout<<"<intrinsic parameter>"<<endl;
    cout<<KMatrix.at<float>(0,0)<<" "<<KMatrix.at<float>(1,1)<<" "<<KMatrix.at<float>(0,2)<<" "<<KMatrix.at<float>(1,2)<<" 0.0"<<" 1.0"<<endl;
    cout<<"</intrinsic parameter>"<<endl;
    cout<<"<Feature Tracks>"<<endl;
    cout<<a.pts2d.size()<<endl;
    for(int i=0;i<a.points3D2.size();i++)
    {
        cout<<"1 0 1 ";
        cout<<a.points3D2[i].x<<" "<<a.points3D2[i].y<<" "<<0<<endl;
        cout<<"0 "<<a.pts2d[i].pt.x<<" "<<a.pts2d[i].pt.y<<endl;
    }
    cout<<"</Feature Tracks>"<<endl;
    cout<<"<Camera Track>"<<endl;
        cout<<"<FRAME"<<0<<">"<<endl;
        cout<<KMatrix.at<float>(0,0)<<endl;
        for (int k=0; k<3; k++) {
            for (int v=0; v<3; v++) {
                cout<<R.at<float>(k,v)<<" ";
            }
            cout<<t.at<float>(k,0)<<endl;
        }
        cout<<"0.0 0.0 0.0 1.0"<<endl;
        cout<<"</FRAME"<<0<<">"<<endl;
    cout<<"</Camera Track>"<<endl;
    images.push_back(frame.clone());
*/
//    frame.copyTo(a.img);
    markers.push_back(a);
}


bool Feature_Track::track(Mat &frame, Mat & C_GL, double &detectfps, double &matchfps, int &keypointSize, int & keyframeCnt,Feature_Match &featureMatch)
{
    static bool is100 = true;
    static bool isNewMarker = false;
    static int newMarker = -1;
    static Mat R,t;
    Keyframe tmp;
    tmp.R = R.clone();
    tmp.t = t.clone();
    tmp.keyPoints.clear();
    tmp.indices.clear();
    tmp.descriptor.release();
    tmp.img.release();
    tmp.markerIdx.clear();
    tmp.pts2D.clear();
    tmp.pts3D.clear();
    if (!isTracked)
    {
        //       keyFrameNumber = searchKeyFrame(RIMU, t)
    }
    
    keyframeCnt = keyFrameNumber;
    TickMeter tm;
    tmp.img = frame.clone();
    vector<vector<DMatch>> dmatches;
    tm.start();
    detector->detect(frame, tmp.keyPoints);
    extractor->compute(frame, tmp.keyPoints, tmp.descriptor);
    tm.stop();
    detectfps = 1 / tm.getTimeSec();

    
 //   Mat pts3D, pts2D;
    vector<Point3f> pts3d1, pts3d1_t,pts3d2,pts3d2_t;
    vector<Point2f> pts2d1,pts2d1_t,pts2d2,pts2d2_t;
    vector<Point3f> pts3;vector<Point2f> pts2;
    // ◊œ»ªÒ»°µ±«∞πÿº¸÷°µƒmarker–≈œ¢£¨∂‘√ø∏ˆmarker∂ºΩ¯––∆•≈‰£¨ªÒµ√»˝Œ¨µ„∫Õ∂˛Œ¨µ„µƒ¡™œµ
    vector<pair<int,int>> indices;
    vector<vector<int>> idx(keyframes[keyFrameNumber].markerIdx.size());

    tm.reset();
    tm.start();
    for (int j = 0; j < keyframes[keyFrameNumber].markerIdx.size(); j++)
    {
        int markerNum = keyframes[keyFrameNumber].markerIdx[j];
        auto &nowMarker = markers[markerNum];
        if (!nowMarker.registed)
            continue;
        matcher->knnMatch(tmp.descriptor, nowMarker.descriptor, dmatches, 2);
        matchfps = 1 / tm.getTimeSec();
        for (int i = 0; i < dmatches.size(); i++)
        {
            tmp.indices.push_back(pair<int, int>(-1, -1));
            if ((dmatches[i][0].distance < dmatches[i][1].distance*0.6) && dmatches[i][0].distance < 90)
            {
//                indices.push_back(pair<int,int>(markerNum,i));
                pts3d1_t.push_back(nowMarker.points3D1[dmatches[i][0].trainIdx]);
                pts2d1_t.push_back(tmp.keyPoints[dmatches[i][0].queryIdx].pt);
                pts2d2_t.push_back(nowMarker.keypoints[dmatches[i][0].trainIdx].pt);
//                tmp.indices[i] = pair<int, int>(markerNum, dmatches[i][0].trainIdx);
            }
        }
//add for homography
        Keyframe warpMarker;
        Mat mask;
        if(pts2d1_t.size()<8)
        {
            isTracked = false;
            return false;
        }
        findFundamentalMat(pts2d1_t, pts2d2_t,mask);
        for (int i=0;i<mask.rows; i++) {
            if(mask.at<char>(i,0))
            {
                pts2d1.push_back(pts2d1_t[i]);
                pts2d2.push_back(pts2d2_t[i]);
            }
        }
        if(pts2d2.size()<8)
        {
            isTracked = false;
            return false;
        }
        Mat H = findHomography(pts2d2, pts2d1);
        nowMarker.pts2d.clear();
        warpPerspective(nowMarker.img, warpMarker.img, H, nowMarker.img.size());
        for (int i=0; i<nowMarker.keypoints.size(); i++) {
            nowMarker.pts2d.push_back(nowMarker.keypoints[i].pt);
        }
        vector<Point3f> pts2d_H;
        vector<Point3f> pts2d_W;

        convertPointsToHomogeneous(nowMarker.pts2d, pts2d_H);

        for (int i=0; i<nowMarker.pts2d.size(); i++) {
            auto &a = pts2d_H[i];
            double *aa = (double*)H.data;
            Point3f aaa;
            aaa.x = aa[0]*a.x + aa[1]*a.y+aa[2]*a.z;
            aaa.y = aa[3]*a.x + aa[4]*a.y+aa[5]*a.z;
            aaa.z = aa[6]*a.x + aa[7]*a.y+aa[8]*a.z;
            pts2d_W.push_back(aaa);
        }

        convertPointsFromHomogeneous(pts2d_W, warpMarker.pts2D);
        for (int i=0; i<nowMarker.keypoints.size(); i++) {
            KeyPoint kp;
            auto &pt = warpMarker.pts2D[i];
            if(pt.x<0||pt.y<0||pt.x>tmp.img.cols-10||pt.y>tmp.img.rows-10)
            {
                pt.x = 0;
                pt.y = 0;
            }
            kp.pt = warpMarker.pts2D[i];
            kp.octave = nowMarker.keypoints[i].octave;
            kp.response = nowMarker.keypoints[i].response;
            kp.size = nowMarker.keypoints[i].size;
            warpMarker.keyPoints.push_back(kp);
        }


        foo(warpMarker.img, warpMarker.keyPoints);
        for(int i=0;i<warpMarker.keyPoints.size();i++)
        {
            unsigned int scale = warpMarker.keyPoints[i].octave;
            const int sizeList_[8] = {48,59,69,85,99,122,143,168};
            // saturate
            
            const int border = sizeList_[scale];
            const int border_x = warpMarker.img.cols - border;
            const int border_y = warpMarker.img.rows - border;
            if(!RoiPredicate((float)border,(float)border,(float)border_x,(float)border_y,warpMarker.keyPoints[i]))
            {
                idx[j].push_back(i);
            }
        }
        extractor->compute(warpMarker.img, warpMarker.keyPoints, warpMarker.descriptor);
        matcher->knnMatch(tmp.descriptor, warpMarker.descriptor, dmatches,2);

        pts2d1_t.clear();
        pts3d2_t.clear();
        pts2d2_t.clear();
        vector<pair<int,int>> indices2;
        indices2.clear();
        for (int i = 0; i < dmatches.size(); i++)
        {
            if ((dmatches[i][0].distance < dmatches[i][1].distance*0.6) && dmatches[i][0].distance < 90)
            {
                auto trainIdx =idx[j][dmatches[i][0].trainIdx];
                indices2.push_back(pair<int,int>(j,i));
                CV_Assert(dmatches[i][0].distance < (0.6*dmatches[i][1].distance));
                pts2d1_t.push_back(tmp.keyPoints[dmatches[i][0].queryIdx].pt);
                pts2d2_t.push_back(nowMarker.keypoints[trainIdx].pt);
                pts3d2_t.push_back(nowMarker.points3D1[trainIdx]);
            }
        }
        pts2d2.clear();
        pts3d2.clear();
        if(pts2d1_t.size()<8)
        {
            continue;
        }
        findFundamentalMat(pts2d1_t, pts2d2_t,mask);
        for(int i=0;i<mask.rows;i++)
        {
            if(mask.at<char>(i,0))
            {
                indices.push_back(indices2[i]);
                pts2.push_back(pts2d1_t[i]);
                pts3.push_back(pts3d2_t[i]);
 //               bool flag = false;
//                tmp.indices[dmatches[i][0].queryIdx] = pair<int,int>(markerNum,dmatches[i][0].trainIdx);
            }
        }
        if(pts3.size()>8)
        {
            tmp.markerIdx.push_back(markerNum);
        }
        //Mat inliers;
        //findFundamentalMat(pts2d, pts2dt, inliers);
        //for (int i = 0; i<pts2d.size(); i++)
        //{
        //	if (inliers.at<char>(i, 0))
        //	{
        //		Mat temp = Mat(keyframes[keyFrameNumber].pts3D[dmatches[i][0].trainIdx]);
        //		temp = temp.t();
        //		pts3D.push_back(temp);
        //		temp = Mat(tmp.keyPoints[dmatches[i][0].queryIdx].pt);
        //		temp = temp.t();
        //		pts2D.push_back(temp);
        //	}
        //}
    }
    tm.stop();
    cout<<"=-=-="<<tm.getTimeSec()<<endl;
    tm.reset();
    tm.start();

    keypointSize = pts3.size();
    if(keypointSize<15)
    {
        isTracked = false;
        return false;
    }
    vector<int> inliers;
    bool status=Solve3D::getCameraRT(KMatrix, pts3, pts2, tmp.R, tmp.t,inliers,isTracked);

    if(!status)
    {
        isTracked = false;
        return false;
    }
    R = tmp.R.clone();
    t = tmp.t.clone();
    cout<<tmp.t<<endl;
    pts3d1.clear();
    pts2d2.clear();
    for(auto j:inliers)
    {
        auto k = indices[j];
        auto markerNum = tmp.markerIdx[k.first];
        int i = k.second;
        auto trainIdx =idx[k.first][dmatches[i][0].trainIdx];
        tmp.indices[i] = pair<int, int>(markerNum, trainIdx);
        pts3d1.push_back(pts3[j]);
        pts2d2.push_back(pts2[j]);
    }
    

    //¿˚”√»˝Œ¨µ„∫Õ∂˛Œ¨µ„«ÛΩ‚µ±«∞÷°µƒCamera Pose

    
    Mat rvec;
    Rodrigues(tmp.R, rvec);
    projectPoints(pts3d1, rvec, tmp.t, KMatrix, Mat(), pts2d1);
    double err=0;
    
    for(int i=0;i<pts2d2.size();i++)
    {
        err += sqrtf(pow(pts2d2[i].x-pts2d1[i].x,2)+pow(pts2d2[i].y-pts2d1[i].y,2));
    }
    err /= pts2d1.size();
    cout<<err<<endl;
    if(err>3)
    {
        isTracked = false;
//        return false;
    }
    else{
        isTracked = true;
    }
    
    //cout<<tmp.t <<endl;
    //»Áπ˚ «¥¥Ω®–¬Markerµƒµ⁄∂˛÷°
    if (isNewMarker)
    {
        // π”√¡Ω’≈ÕºœÒµƒæ…markerµƒCorrespondence«ÛΩ‚R∫Õt
        // π”√¡Ω’≈ÕºœÒµƒR∫Õt£¨∫Õ–¬markerµƒ∂˛Œ¨µ„»˝Ω«∂®±Í«ÛΩ‚–¬markerµ„‘⁄÷˜◊¯±Íœµ÷–µƒ◊¯±Í
        //¿˚”√»˝Œ¨µ„◊¯±Í«ÛΩ‚œ‡À∆±‰ªª£¨»ª∫ÛΩ´–¬markerµƒ◊¯±Í◊¢≤·µΩ÷˜◊¯±Íœµ÷–
        int size = keyframes[keyFrameNumber].markerIdx.size();
        int oldMarkerNum = keyframes[keyFrameNumber].markerIdx[size-2];
        int newMarkerNum = keyframes[keyFrameNumber].markerIdx[size-1];
        assert(markers[oldMarkerNum].registed&&!markers[newMarkerNum].registed);
        matcher->knnMatch(tmp.descriptor, markers[newMarkerNum].descriptor, dmatches, 2);
        for (int i = 0; i < dmatches.size(); i++)
        {
            if ((dmatches[i][0].distance < dmatches[i][1].distance*0.6) && dmatches[i][0].distance < 90)
            {
                if (tmp.indices[i].first == -1 && tmp.indices[i].second == -1)
                {
                    tmp.indices[i] = pair<int, int>(newMarkerNum, dmatches[i][0].trainIdx);
                }
            }
        }
        matcher->knnMatch(tmp.descriptor, keyframes[keyFrameNumber].descriptor, dmatches, 2);
        vector<int> marker3DIdx;
        //”√”⁄«ÛΩ‚»˝Œ¨µ„µƒ∂˛Œ¨Correspondence
        vector<Point2f> pts2D1, pts2D2;
        vector<Point2f> pts2D1_t,pts2D2_t;
        vector<int> indices;
        for (int i = 0; i < dmatches.size(); i++)
        {
            int train = dmatches[i][0].trainIdx;
            int query = dmatches[i][0].queryIdx;
            if ((dmatches[i][0].distance < dmatches[i][1].distance*0.6) && dmatches[i][0].distance < 90)
            {
                if (keyframes[keyFrameNumber].indices[train].first == tmp.indices[query].first&&keyframes[keyFrameNumber].indices[train].second == tmp.indices[query].second&&tmp.indices[query].first == newMarkerNum)
                {
                    indices.push_back(tmp.indices[query].second);
 //                   marker3DIdx.push_back(tmp.indices[query].second);
                    pts2D1_t.push_back(keyframes[keyFrameNumber].keyPoints[train].pt);
                    pts2D2_t.push_back(tmp.keyPoints[query].pt);
                }
            }
        }
        Mat inliers;
        findFundamentalMat(pts2D1_t, pts2D2_t,inliers);
        for (int i=0; i<inliers.rows; i++) {
            if(inliers.at<char>(i,0))
            {
                pts2D1.push_back(pts2D1_t[i]);
                pts2D2.push_back(pts2D2_t[i]);
                marker3DIdx.push_back(indices[i]);
            }
        }
        
        Mat P1, P2;
        Mat C1(3, 4, CV_32F), C2(3, 4, CV_32F);
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                C1.at<float>(i, j) = keyframes[keyFrameNumber].R.at<float>(i, j);
                C2.at<float>(i, j) = tmp.R.at<float>(i, j);
            }
            C1.at<float>(i, 3) = keyframes[keyFrameNumber].t.at<float>(i, 0);
            C2.at<float>(i, 3) = tmp.t.at<float>(i, 0);
        }
        P1 = KMatrix*C1;
        P2 = KMatrix*C2;
        Mat Points4D;
        //«ÛΩ‚marker‘⁄µ±«∞◊¯±Íœµœ¬µƒ3Œ¨◊¯±Í
        triangulatePoints(P1, P2, pts2D1, pts2D2, Points4D);
        Mat dst;
        //		convertPointsFromHomogeneous(Points4D, dst);
        //marker‘⁄◊‘…Ì◊¯±Íœµœ¬µƒ3Œ¨◊¯±Í
        vector<Point3f> pts3ds,pts3d;
        for (int i = 0; i < Points4D.cols; i++)
        {
            Point3f t;
            double s = Points4D.at<float>(3, i);
            t.x = Points4D.at<float>(0, i) / s;
            t.y = Points4D.at<float>(1, i) / s;
            t.z = Points4D.at<float>(2, i) / s;
            pts3d.push_back(t);
        }
        for (auto i : marker3DIdx)
        {
            pts3ds.push_back(markers[newMarkerNum].points3D2[i]);
        }
        //◊™±‰ ˝æ›Ω·ππ”√”⁄«ÛΩ‚srt
        Mat src(pts3ds);
        dst = Mat(pts3d);
        double s;
        Mat R, t;
        Solve3D::solveSRT(src, dst, s, R, t);
        Solve3D::transformSRT(markers[newMarkerNum].points3D2, markers[newMarkerNum].points3D1, s, R, t);
        markers[newMarkerNum].registed = true;
        markers[newMarkerNum].center.x = 0;
        markers[newMarkerNum].center.y = 0;
        markers[newMarkerNum].center.z = 0;
        double minx = 1000,miny=10000,minz=1000,maxx=-1,maxy=-1,maxz=-1;
        
        for (auto i:markers[newMarkerNum].points3D1)
        {
            maxx = std::max<double>(maxx,i.x);
            maxy = std::max<double>(maxy,i.y);
            maxz = std::max<double>(maxz,i.z);
            minx = std::min<double>(minx,i.x);
            miny = std::min<double>(miny,i.y);
            minz = std::min<double>(minz,i.z);

        }
        
        int num = markers[newMarkerNum].points3D1.size();
        markers[newMarkerNum].center.x = (minx+maxx)/2;
        markers[newMarkerNum].center.y = (miny+maxy)/2;;
        markers[newMarkerNum].center.z =(minz+maxz)/2;;
        isNewMarker = false;
    }
    
    //»Áπ˚µ±«∞÷°∑˚∫œπÿº¸÷°Ãıº˛
    if (isKeyFrame(tmp.R, tmp.t,keyFrameNumber))
    {
        //≈–∂œ «∑Òµ±«∞÷°∫¨”––¬µƒmarker–≈œ¢
        vector<Matches> matches;
        featureMatch.matchNimages(frame, 2, matches);
        if (matches[0].count > 100 && matches[1].count > 100)
        {
            //»Áπ˚”––¬µƒmarker£¨Œ™–¬µƒmarker«ÛΩ‚µ±«∞◊¯±Íœµµƒ◊¯±Í£¨»ª∫Û«Ûœ‡À∆±‰ªª
            if (!markers[matches[0].refIdx].registed || !markers[matches[1].refIdx].registed)
            {
                int markerIdx = markers[matches[0].refIdx].registed ? matches[1].refIdx : matches[0].refIdx;
                newMarker = markerIdx;
                matcher->knnMatch(tmp.descriptor, markers[markerIdx].descriptor, dmatches, 2);
                int matchCount = 0;
                pts2d1_t.clear();
                pts2d2_t.clear();
                vector<int> indices;
                for (int i = 0; i < dmatches.size(); i++)
                {
                    if ((dmatches[i][0].distance < dmatches[i][1].distance*0.6) && dmatches[i][0].distance < 90)
                    {
                        indices.push_back(i);
                        pts2d1_t.push_back(tmp.keyPoints[dmatches[i][0].trainIdx].pt);
                        pts2d2_t.push_back(markers[markerIdx].keypoints[i].pt);
                    }
                }
                Mat inliers;
                if(indices.size()>8)
                {
                    findFundamentalMat(pts2d1_t, pts2d2_t,inliers);
                    for (int j=0; j<inliers.rows; j++)
                    {
                        if(inliers.at<char>(j,0))
                        {
                            pts2d1.push_back(pts2d1_t[j]);
                            pts2d2.push_back(pts2d2_t[j]);
                        //                        int i=indices[j];
//                        matchCount++;
                        /*
                        if (tmp.indices[i].first == -1 && tmp.indices[i].second == -1)
                        {
                            tmp.indices[i] = pair<int, int>(markerIdx, dmatches[i][0].trainIdx);
                        }
                        */
                        
                        }
                    }
                    auto & nowMarker = markers[markerIdx];
                    Mat H = findHomography(pts2d2, pts2d1);
                    Keyframe warpMarker;
                    warpPerspective(nowMarker.img, warpMarker.img, H, nowMarker.img.size());
                    for (int i=0; i<nowMarker.keypoints.size(); i++) {
                        nowMarker.pts2d.push_back(nowMarker.keypoints[i].pt);
                    }
                    vector<Point3f> pts2d_H;
                    vector<Point3f> pts2d_W;
                    convertPointsToHomogeneous(nowMarker.pts2d, pts2d_H);
                    for (int i=0; i<nowMarker.pts2d.size(); i++) {
                        auto &a = pts2d_H[i];
                        double *aa = (double*)H.data;
                        Point3f aaa;
                        aaa.x = aa[0]*a.x + aa[1]*a.y+aa[2]*a.z;
                        aaa.y = aa[3]*a.x + aa[4]*a.y+aa[5]*a.z;
                        aaa.z = aa[6]*a.x + aa[7]*a.y+aa[8]*a.z;
                        pts2d_W.push_back(aaa);
                    }
                    convertPointsFromHomogeneous(pts2d_W, warpMarker.pts2D);
                    for (int i=0; i<nowMarker.keypoints.size(); i++) {
                        KeyPoint kp;
                        auto &pt = warpMarker.pts2D[i];
                        if(pt.x<0||pt.y<0||pt.x>tmp.img.cols-10||pt.y>tmp.img.rows-10)
                        {
                            pt.x = 0;
                            pt.y = 0;
                        }
                        kp.pt = warpMarker.pts2D[i];
                        kp.octave = nowMarker.keypoints[i].octave;
                        kp.response = nowMarker.keypoints[i].response;
                        kp.size = nowMarker.keypoints[i].size;
                        warpMarker.keyPoints.push_back(kp);
                    }
                    foo(warpMarker.img, warpMarker.keyPoints);
                    vector<int> idx;
                    for(int i=0;i<warpMarker.keyPoints.size();i++)
                    {
                        unsigned int scale = warpMarker.keyPoints[i].octave;
                        const int sizeList_[8] = {48,59,69,85,99,122,143,168};
                        // saturate
                        
                        const int border = sizeList_[scale];
                        const int border_x = warpMarker.img.cols - border;
                        const int border_y = warpMarker.img.rows - border;
                        if(!RoiPredicate((float)border,(float)border,(float)border_x,(float)border_y,warpMarker.keyPoints[i]))
                        {
                            idx.push_back(i);
                        }
                    }
                    extractor->compute(warpMarker.img, warpMarker.keyPoints, warpMarker.descriptor);
                    matcher->knnMatch(tmp.descriptor, warpMarker.descriptor, dmatches,2);
                    for (int i = 0; i < dmatches.size(); i++)
                    {
                        if ((dmatches[i][0].distance < dmatches[i][1].distance*0.6) && dmatches[i][0].distance < 90)
                        {
//                        int i=indices[j];
                            matchCount++;
                            auto trainIdx = idx[dmatches[i][0].trainIdx];
                            if (tmp.indices[i].first == -1 && tmp.indices[i].second == -1)
                            {
                                tmp.indices[i] = pair<int, int>(markerIdx, trainIdx);
                            }
                        }
                    }
                    if (matchCount > 20)
                    {
                        tmp.markerIdx.push_back(newMarker);
                        isNewMarker = true;

                    }
                    

                }
            }
        }
        
        for (size_t i = 0; i < tmp.indices.size(); i++)
        {
            bool flag = false;
            if (tmp.indices[i].first!=-1)
            {
                for(auto &j:markers[tmp.indices[i].first].invertIdx[tmp.indices[i].second])
                {
                    if(j.first == keyframes.size())
                    {
                        if(dmatches[i][0].distance < dmatches[j.second][0].distance)
                        {
                            j.second = i;
                            flag = true;
                            break;
                        }
                    }
                }
                if(!flag)
                    markers[tmp.indices[i].first].invertIdx[tmp.indices[i].second].push_back(pair<int,int>(keyframes.size(), i));
            }
        }

        //       Solve3D::get3DPoints(KMatrix, tmp.R, tmp.t, tmp.keyPoints,tmp.pts3D);
        //		cout<<tmp.t<<endl;
        keyframes.push_back(tmp);

        keyFrameNumber = keyframes.size() - 1;
        images.push_back(frame.clone());
        assert(images.size() == keyframes.size());
        if(images.size() == 50)
            is100 = false;
        
        if(images.size()%10)
           Solve3D::sba(KMatrix, keyframes, markers);
       else
           Solve3D::sba(KMatrix,keyframes,markers,false);
        
        //        cout<<"keyframes numbers"<<keyframes.size()<<endl;
    }
    
    //    double lowx, lowy, highx, highy;
    //    Solve3D::getHighAndLow(pts3d, lowx, lowy, highx, highy);
    //    Solve3D::printPlane(KMatrix, tmp.R, tmp.t, frame, lowx, lowy, highx, highy);
    tm.stop();
    cout<<"---"<<tm.getTimeSec()<<endl;

    C_GL = Solve3D::getP(tmp.R, tmp.t);
    keyFrameNumber = searchKeyFrame(keyFrameNumber, tmp.R, tmp.t);
    //    if(q>=0)
    //    {
    //       keyFrameNumber = q;
    //   }
    return true;
}