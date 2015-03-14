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
    
    
    //The first KeyFrame
    Keyframe kf;
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
    matcher->knnMatch(kf.descriptor, markers[markerNum[0]].descriptor, dmatches, 2);
    
    vector<Point3f> pts3d1, pts3d2;
    //	vector<Point2f> pts2d1, pts2d2;
    for (int i = 0; i < dmatches.size(); i++)
    {
        if ((dmatches[i][0].distance < dmatches[i][1].distance*0.6) && dmatches[i][0].distance < 90)
        {
            kf.indices[i] = pair<int, int>(markerNum[0], dmatches[i][0].trainIdx);
            //			markers[markerNum[0]].invertIdx[dmatches[i][0].trainIdx].push_back(pair<int, int>(0, i));
            pts3d1.push_back(kf.pts3D[dmatches[i][0].queryIdx]);
            pts3d2.push_back(markers[markerNum[0]].points3D2[dmatches[i][0].trainIdx]);
            
        }
    }
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
    
    Solve3D::transformSRT(markers[markerNum[0]].points3D2, markers[markerNum[0]].points3D1, s, _R, _t);
    markers[markerNum[0]].registed = true;
    markers[markerNum[0]].center.x = 0;
    markers[markerNum[0]].center.y = 0;
    markers[markerNum[0]].center.z = 0;

    for (auto i:markers[markerNum[0]].points3D1)
    {
        markers[markerNum[0]].center.x += i.x;
        markers[markerNum[0]].center.y += i.y;
        markers[markerNum[0]].center.z += i.z;
    }
    int num = markers[markerNum[0]].points3D1.size();
    markers[markerNum[0]].center.x /= num;
    markers[markerNum[0]].center.y /= num;
    markers[markerNum[0]].center.z /= num;
    
    /*}*/
    keyframes.push_back(kf);
    keyFrameNumber = 0;
    return true;
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
    double threshold = 0.6*0.6;
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
        cout << minDist << "--" << dists[keyFrameNumber] << endl;
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
    markers.resize(markers.size()+1);
    Marker &a = markers[markers.size()-1];
    detector->detect(frame, a.pts2d);
    extractor->compute(frame, a.pts2d, a.descriptor);
    Mat t;
    Solve3D::get3DPoints(KMatrix, R, a.pts2d, a.points3D2, t);
    a.invertIdx.resize(a.pts2d.size());
    a.img = frame.clone();
//    frame.copyTo(a.img);
//    markers.push_back(a);
}


bool Feature_Track::track(Mat &frame, Mat & C_GL, double &detectfps, double &matchfps, int &keypointSize, int & keyframeCnt,Feature_Match &featureMatch)
{
    static bool isNewMarker = false;
    static int newMarker = -1;
    
    if (!isTracked)
    {
        //       keyFrameNumber = searchKeyFrame(RIMU, t)
    }
    
    keyframeCnt = keyFrameNumber;
    TickMeter tm;
    Keyframe tmp;
    vector<vector<DMatch>> dmatches;
    tm.start();
    detector->detect(frame, tmp.keyPoints);
    extractor->compute(frame, tmp.keyPoints, tmp.descriptor);
    tm.stop();
    detectfps = 1 / tm.getTimeSec();
    
    
    Mat pts3D, pts2D;
    vector<Point3f> pts3d; vector<Point2f> pts2d;
    vector<Point2f> pts2dt;
    // ◊œ»ªÒ»°µ±«∞πÿº¸÷°µƒmarker–≈œ¢£¨∂‘√ø∏ˆmarker∂ºΩ¯––∆•≈‰£¨ªÒµ√»˝Œ¨µ„∫Õ∂˛Œ¨µ„µƒ¡™œµ
    for (int j = 0; j < keyframes[keyFrameNumber].markerIdx.size(); j++)
    {
        int markerNum = keyframes[keyFrameNumber].markerIdx[j];
        if (!markers[markerNum].registed)
            continue;
        tmp.markerIdx.push_back(markerNum);
        tm.reset();
        tm.start();
        matcher->knnMatch(tmp.descriptor, markers[markerNum].descriptor, dmatches, 2);
        tm.stop();
        matchfps = 1 / tm.getTimeSec();
        for (int i = 0; i < dmatches.size(); i++)
        {
            tmp.indices.push_back(pair<int, int>(-1, -1));
            if ((dmatches[i][0].distance < dmatches[i][1].distance*0.6) && dmatches[i][0].distance < 90)
            {
                pts3d.push_back(markers[markerNum].points3D1[dmatches[i][0].trainIdx]);
                pts2d.push_back(tmp.keyPoints[dmatches[i][0].queryIdx].pt);
                pts2dt.push_back(markers[markerNum].pts2d[dmatches[i][0].trainIdx].pt);
                tmp.indices[i] = pair<int, int>(markerNum, dmatches[i][0].trainIdx);
                Mat temp = Mat(markers[markerNum].points3D1[dmatches[i][0].trainIdx]);
                temp = temp.t();
                pts3D.push_back(temp);
                temp = Mat(tmp.keyPoints[dmatches[i][0].queryIdx].pt);
                temp = temp.t();
                pts2D.push_back(temp);
            }
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
    
    keypointSize = pts3D.rows;
    if(pts3D.rows<15)
    {
        isTracked = false;
        return false;
    }
    //¿˚”√»˝Œ¨µ„∫Õ∂˛Œ¨µ„«ÛΩ‚µ±«∞÷°µƒCamera Pose
    Solve3D::getCameraRT(KMatrix, pts3D, pts2D, tmp.R, tmp.t);
    
    cout<<tmp.t <<endl;
    //»Áπ˚ «¥¥Ω®–¬Markerµƒµ⁄∂˛÷°
    if (isNewMarker)
    {
        // π”√¡Ω’≈ÕºœÒµƒæ…markerµƒCorrespondence«ÛΩ‚R∫Õt
        // π”√¡Ω’≈ÕºœÒµƒR∫Õt£¨∫Õ–¬markerµƒ∂˛Œ¨µ„»˝Ω«∂®±Í«ÛΩ‚–¬markerµ„‘⁄÷˜◊¯±Íœµ÷–µƒ◊¯±Í
        //¿˚”√»˝Œ¨µ„◊¯±Í«ÛΩ‚œ‡À∆±‰ªª£¨»ª∫ÛΩ´–¬markerµƒ◊¯±Í◊¢≤·µΩ÷˜◊¯±Íœµ÷–
        int oldMarkerNum = keyframes[keyFrameNumber].markerIdx[0];
        int newMarkerNum = keyframes[keyFrameNumber].markerIdx[1];
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
        for (int i = 0; i < dmatches.size(); i++)
        {
            int train = dmatches[i][0].trainIdx;
            int query = dmatches[i][0].queryIdx;
            if ((dmatches[i][0].distance < dmatches[i][1].distance*0.6) && dmatches[i][0].distance < 90)
            {
                if (keyframes[keyFrameNumber].indices[train].first == tmp.indices[query].first&&keyframes[keyFrameNumber].indices[train].second == tmp.indices[query].second&&tmp.indices[query].first == newMarkerNum)
                {
                    marker3DIdx.push_back(tmp.indices[query].second);
                    pts2D1.push_back(keyframes[keyFrameNumber].keyPoints[train].pt);
                    pts2D2.push_back(tmp.keyPoints[query].pt);
                }
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
        
        for (auto i:markers[newMarkerNum].points3D1)
        {
            markers[newMarkerNum].center.x += i.x;
            markers[newMarkerNum].center.y += i.y;
            markers[newMarkerNum].center.z += i.z;
        }
        int num = markers[newMarkerNum].points3D1.size();
        markers[newMarkerNum].center.x /= num;
        markers[newMarkerNum].center.y /= num;
        markers[newMarkerNum].center.z /= num;
        isNewMarker = false;
    }
    
    //»Áπ˚µ±«∞÷°∑˚∫œπÿº¸÷°Ãıº˛
    if (isKeyFrame(tmp.R, tmp.t,keyFrameNumber))
    {
        //≈–∂œ «∑Òµ±«∞÷°∫¨”––¬µƒmarker–≈œ¢
        vector<Matches> matches;
        featureMatch.matchNimages(frame, 2, matches);
        if (matches[0].count > 30 && matches[1].count > 30)
        {
            //»Áπ˚”––¬µƒmarker£¨Œ™–¬µƒmarker«ÛΩ‚µ±«∞◊¯±Íœµµƒ◊¯±Í£¨»ª∫Û«Ûœ‡À∆±‰ªª
            if (!markers[matches[0].refIdx].registed || !markers[matches[1].refIdx].registed)
            {
                int markerIdx = markers[matches[0].refIdx].registed ? matches[1].refIdx : matches[0].refIdx;
                newMarker = markerIdx;
                matcher->knnMatch(tmp.descriptor, markers[markerIdx].descriptor, dmatches, 2);
                int matchCount = 0;
                for (int i = 0; i < dmatches.size(); i++)
                {
                    if ((dmatches[i][0].distance < dmatches[i][1].distance*0.6) && dmatches[i][0].distance < 90)
                    {
                        matchCount++;
                        if (tmp.indices[i].first == -1 && tmp.indices[i].second == -1)
                        {
                            tmp.indices[i] = pair<int, int>(markerIdx, dmatches[i][0].trainIdx);
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
        for (size_t i = 0; i < tmp.indices.size(); i++)
        {
            if (tmp.indices[i].first!=-1)
                markers[tmp.indices[i].first].invertIdx[tmp.indices[i].second].push_back(pair<int,int>(keyframes.size(), i));
        }
        //       Solve3D::get3DPoints(KMatrix, tmp.R, tmp.t, tmp.keyPoints,tmp.pts3D);
        //		cout<<tmp.t<<endl;
        keyframes.push_back(tmp);
        keyFrameNumber = keyframes.size() - 1;
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