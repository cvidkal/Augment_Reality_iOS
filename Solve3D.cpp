//
//  Solve3D.cpp
//  newAR
//
//  Created by Shangjin Zhai on 14-12-6.
//  Copyright (c) 2014Äê Shangjin Zhai. All rights reserved.
//

#include "Solve3D.h"
#include "headers.h"

Solve3D::Solve3D()
{
    KMatrix = Mat::zeros(3, 3, CV_64F);
    rotationMatrix = Mat::zeros(3, 3, CV_64F);
    invertMatrix = Mat::zeros(3, 3, CV_64F);
    distCoeffs = Mat::zeros(4, 1, CV_64F);
    TVector = Mat::zeros(3, 1, CV_64F);
}

void Solve3D::setKMatrix(double imageWidth, double imageHeight, double focus)
{
    KMatrix.at<double>(0, 0) =  focus;
    KMatrix.at<double>(1, 1) =  focus;
    KMatrix.at<double>(2, 2) = 1;
    
    offset.y = (imageHeight - 1)/2;
    offset.x = (imageWidth - 1)/2;
    KMatrix.at<double>(0, 2) = offset.x;
    KMatrix.at<double>(1, 2) = offset.y;
 }

//void Solve3D::setRotationMatrix(const CMRotationMatrix &rotation)
//{
//    /*
//    rotationMatrix.at<double>(0, 0) = -rotation.m11;
//    rotationMatrix.at<double>(0, 1) = -rotation.m21;
//    rotationMatrix.at<double>(0, 2) = -rotation.m31;
//    
//    rotationMatrix.at<double>(1, 0) = rotation.m12;
//    rotationMatrix.at<double>(1, 1) = rotation.m22;
//    rotationMatrix.at<double>(1, 2) = rotation.m32;
//    
//    rotationMatrix.at<double>(2, 0) = rotation.m13;
//    rotationMatrix.at<double>(2, 1) = rotation.m23;
//    rotationMatrix.at<double>(2, 2) = rotation.m33;*/
//    rotationMatrix.at<double>(0, 0) = rotation.m12;
//    rotationMatrix.at<double>(0, 1) = rotation.m22;
//    rotationMatrix.at<double>(0, 2) = rotation.m32;
//    
//    rotationMatrix.at<double>(1, 0) = rotation.m11;
//    rotationMatrix.at<double>(1, 1) = rotation.m21;
//    rotationMatrix.at<double>(1, 2) = rotation.m31;
//    
//    rotationMatrix.at<double>(2, 0) = -rotation.m13;
//    rotationMatrix.at<double>(2, 1) = -rotation.m23;
//    rotationMatrix.at<double>(2, 2) = -rotation.m33;
//    
//    Mat rr = Mat(3, 3, CV_64F);
//    rr.at<double>(0, 1) = -1;
//    rr.at<double>(1, 0) = 1;
//    rr.at<double>(2, 2) = 1;
//    //rotationMatrix = rr * rotationMatrix;
//    
//    //rotationMatrix = rotationMatrix.t();
//    
//    rotationMatrix.row(2) = rotationMatrix.row(2);
//    
//    cout<<"rotation:   "<<rotationMatrix<<endl;
//}


void Solve3D::InitRotationMatrix()
{
	rotationMatrix.at<double>(0, 0) = 1;
	rotationMatrix.at<double>(0, 1) = 0;
	rotationMatrix.at<double>(0, 2) =0;

	rotationMatrix.at<double>(1, 0) = 0;
	rotationMatrix.at<double>(1, 1) = 1;
	rotationMatrix.at<double>(1, 2) = 0;

	rotationMatrix.at<double>(2, 0) = 0;
	rotationMatrix.at<double>(2, 1) = 0;
	rotationMatrix.at<double>(2, 2) = 1;
}

void Solve3D::setTVector(const cv::Mat &_TVector)
{
    TVector = _TVector;
}

void Solve3D::InitTVector()
{
	TVector = Mat::zeros(3, 1, CV_32F);
}


void Solve3D::get3DPoints(Mat KMatrix,Mat rotationMatrix,vector<KeyPoint> &pts2D,vector<Point3f> &pts3D,Mat &TVector)
{
#define REAL_HEIGHT 5.0

	pts3D.clear();
    cv::Point3d point3D;
    Mat imageCoordinate = Mat::Mat(3, 1, CV_64F);
    Mat realCoordinate;
    Mat minusK, minusR;
    invert(KMatrix, minusK, CV_SVD);
    invert(rotationMatrix, minusR, CV_SVD);
    
	for (int i = 0; i < pts2D.size(); i++)
	{
		imageCoordinate.at<double>(2, 0) = 1;
		double realHeight = REAL_HEIGHT, realScale = 1;

		imageCoordinate.at<double>(0, 0) = pts2D[i].pt.x;
		imageCoordinate.at<double>(1, 0) = pts2D[i].pt.y;

		Mat right = minusR * minusK * imageCoordinate;
		realScale = -realHeight / right.at<double>(2, 0);

		point3D.x = right.at<double>(0, 0) * realScale;//+ offset.x;
		point3D.y = right.at<double>(1, 0) * realScale;//+ offset.y;
		point3D.z = right.at<double>(2, 0) * realScale + REAL_HEIGHT;
		pts3D.push_back(point3D);
	}

    
    Mat tmp = Mat(3, 1, CV_64F);
    tmp.at<double>(0) = 0;
    tmp.at<double>(1) = 0;
    tmp.at<double>(2) = REAL_HEIGHT;
    tmp = -rotationMatrix * tmp;
    
    for(int i=0; i< 3; i++)
        TVector.at<float>(i) = tmp.at<double>(i);
 
}

void Solve3D::getinvertMatrix()
{
    P = Mat(3, 4, CV_64F);
    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            P.at<double>(i, j) = rotationMatrix.at<double>(i, j);
        }
        P.at<double>(i, 3) = (double)TVector.at<float>(i);
    }
    converseMatrix = KMatrix * P;
    invert(converseMatrix, invertMatrix, CV_SVD);
}

void Solve3D::getNextInvertMatrix()
{
    P = Mat(3, 4, CV_64F);
    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            P.at<double>(i, j) = RVector.at<double>(i, j);
        }
        P.at<double>(i, 3) = (double)TVector.at<float>(i);
    }
    converseMatrix = KMatrix * P;
    invert(converseMatrix, invertMatrix, CV_SVD);
    
	 
}

void Solve3D::get2DPoint(Mat &drawMatch)
{
    for(int i= 0; i<points3DVector.size();i++)
    {
        Mat realCoordinate = Mat(4, 1, CV_64F);
        realCoordinate.at<double>(3, 0) = 1;
        Mat imageCoordinate;
        double realScale;
        realCoordinate.at<double>(0,0) = points3DVector[i].x;//all3DPoints[i].x;
        realCoordinate.at<double>(1,0) = points3DVector[i].y;
        realCoordinate.at<double>(2,0) = points3DVector[i].z;
    
        imageCoordinate = converseMatrix * realCoordinate;
    
        realScale = imageCoordinate.at<double>(2, 0);
    
        //printf("%d: %lf %lf haha %lf %lf %lf\n", i, imageCoordinate.at<double>(0, 0)/realScale , points2DVector[i].x, imageCoordinate.at<double>(1, 0)/realScale, points2DVector[i].y, realScale);
        circle(drawMatch, Point2d(imageCoordinate.at<double>(0, 0)/realScale , imageCoordinate.at<double>(1,0)/realScale), 1, Scalar(255, 0, 0));
        circle(drawMatch, points2DVector[i], 1, Scalar(0, 0, 255));
    }
}

Point3d Solve3D::getNext3DPoint(int i)
{
    Point3d point3D;
    Mat imageCoordinate = Mat(3, 1, CV_64F);
    Mat realCoordinate;
    imageCoordinate.at<double>(2, 0) = 1;
    double realScale = 1;
    
    imageCoordinate.at<double>(0, 0) = all2DPoints[i].x;
    imageCoordinate.at<double>(1, 0) = all2DPoints[i].y;
    realCoordinate = invertMatrix * imageCoordinate;
    
    realScale = realCoordinate.at<double>(3, 0);
    
    point3D.x = realCoordinate.at<double>(0, 0) / realScale;
    point3D.y = realCoordinate.at<double>(1, 0) / realScale;
    point3D.z = realCoordinate.at<double>(2, 0) / realScale;
    
    all3DPoints[i] = point3D;
    return point3D;
}

void Solve3D::getCameraRT(Mat KMatrix,Mat P3D,Mat P2D,Mat &R,Mat &t)
{

	Mat dtmp = Mat::zeros(4, 1, CV_32F);
    solvePnPRansac(P3D, P2D, KMatrix, dtmp, R, t , false);
   
}

void Solve3D::printCameraInfo()
{
    printf("rotationMatrix\n");
    for(int i = 0 ; i < rotationMatrix.rows ; i++)
    {
        for(int j = 0 ; j < rotationMatrix.cols ; j++)
            printf("%lf ",rotationMatrix.at<double>(i,j));
        printf("\n");
    }
    
    printf("REVC\n");
    for(int i = 0 ; i < RVector.rows ; i++)
    {
        for(int j = 0 ; j < RVector.cols ; j++)
            printf("%lf ",RVector.at<double>(i,j));
        printf("\n");
    }
    
    printf("TEVC\n");
    for(int i = 0 ; i < TVector.rows ; i++)
    {
        for(int j = 0 ; j < TVector.cols ; j++)
            printf("%f ",TVector.at<float>(i,j));
        printf("\n");
    }
}

void Solve3D::printPoints()
{
    int pointSize = (int)keyPoints.size();
    
    for(int i = 0 ; i < pointSize ; i++)
    {
        cv::Point3d p = get3DPoints(i); 
        //printf("v %.2lf %.2lf %.2lf\n",p.x,p.y,p.z);
        pointsIndex[i] = i;
    }
}

void Solve3D::detectFeature(const cv::Mat &imageObject, const Ptr<cv::FeatureDetector> &detector, const Ptr<cv::DescriptorExtractor> &extractor)
{
    detector->detect(imageObject, keyPoints);
    extractor->compute(imageObject, keyPoints, descriptors);
    all3DPoints.resize(keyPoints.size());
    all2DPoints.resize(keyPoints.size());
    pointsIndex.resize(keyPoints.size(), -1);
    for(int i = 0; i < keyPoints.size(); i++)
    {
        all2DPoints[i] = Point2d(keyPoints[i].pt.x, keyPoints[i].pt.y);
        //printf("%d: %lf %lf\n",i,all2DPoints[i].x,all2DPoints[i].y);
    }
}

//void Solve3D::Match(const Solve3D &refsSolve3D, int &keyPointSize, Mat &drawMatch)
//{
//    vector<Point2d> pts1, pts2;
//    vector<int> pstIndex;
//    vector<DMatch> matches;
//    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
//    matcher->match(descriptors, refsSolve3D.descriptors, matches);
//    for(int i = 0; i < matches.size(); i++)
//    {
//        if ( matches[i].distance < 90)
//        {
//            pts1.push_back(all2DPoints[matches[i].queryIdx]);
//            pts2.push_back(refsSolve3D.all2DPoints[matches[i].trainIdx]);
//            pstIndex.push_back(matches[i].trainIdx);
//        }
//        pointsIndex[matches[i].queryIdx] = refsSolve3D.pointsIndex[matches[i].trainIdx];
//        //printf("%lf %lf %lf %lf\n",p1.x,p1.y,p2.x,p2.y);
//    }
//    Mat mask;
//    if(pts1.size() >=8)
//        cv::findHomography(pts1, pts2, mask, CV_RANSAC);
//    for (int i = 0; i < pts1.size(); i++)
//    {
//        if (mask.at<char>(i, 0))
//        {
//            points2DVector.push_back(pts1[i]);
//            points3DVector.push_back(refsSolve3D.all3DPoints[pstIndex[i]]);
//            line(drawMatch, pts1[i], pts2[i], Scalar(0, 255, 0), 1);
//        }
//    }
//    keyPointSize = points2DVector.size();
//    //printf("Match numbers:%d\n",points2DVector.size());
//}

void Solve3D::getOtherPoints()
{
    for(int i = 0 ; i < pointsIndex.size(); i++)
    {
        if(pointsIndex[i] < 0)
        {
            //cv::Point3d p = getNext3DPoint(i);
            //printf("V %lf %lf %lf\n",p.x,p.y,p.z);
        }
    }
}

void Solve3D::getRotation(cv::Mat &getRotation)
{
    getRotation = rotationMatrix.clone();
}

void Solve3D::getP(cv::Mat &PP)
{
    Mat tmp ;
    Rodrigues(rrvec, tmp);
    PP = Mat(4, 4, CV_64F);
    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            PP.at<double>(i, j) = tmp.at<double>(i, j);
            if(i != 0)
                PP.at<double>(i, j) = -tmp.at<double>(i, j);
        }
        PP.at<double>(i, 3) = TVector.at<float>(i);
        if(i != 0)
            PP.at<double>(i, 3) = -PP.at<double>(i, 3);
    }
    PP.at<double>(3, 0) =PP.at<double>(3, 1) =PP.at<double>(3, 2) =0;
    PP.at<double>(3, 3) =1;
}

void Solve3D::getLocalPoints(vector<Point2d> &LocalPoints)
{
    LocalPoints.clear();
    Mat tmp ;
    invert(KMatrix, tmp, CV_SVD);
    Mat haha = Mat(3, 1, CV_64F);
    for(int i = 0; i < points3DVector.size(); i++)
    {
        haha.at<double>(0) = points3DVector[i].x;
        haha.at<double>(1) = points3DVector[i].y;
        haha.at<double>(2) = points3DVector[i].z;
        haha = converseMatrix * haha;
        //printf("%d %d\n",points2DVector[i].x, points2DVector[i].y);
        //printf("%d %d %d\n",haha.at<double>(0), haha.at<double>(1), haha.at<double>(2));
        LocalPoints.push_back(Point2d(haha.at<double>(0)/haha.at<double>(2), haha.at<double>(1)/haha.at<double>(2)));
    }
}

void Solve3D::get3DPoints(vector<Point3d> &realPoints)
{
    realPoints.clear();
    realPoints = points3DVector;
}

void Solve3D::getHighAndLow(double &lowX, double &lowY, double &highX, double &highY)
{
    lowX = 100, lowY = 100, highX = -100, highY = -100;
    for(int i = 0; i < points3DVector.size() ; i++)
    {
        if(points3DVector[i].x < lowX)
            lowX = points3DVector[i].x;
        if(points3DVector[i].y < lowY)
            lowY = points3DVector[i].y;
        if(points3DVector[i].x > highX)
            highX = points3DVector[i].x;
        if (points3DVector[i].y > highY) {
            highY = points3DVector[i].y;
        }
    }
}

void Solve3D::printPlane(cv::Mat &drawPlane, double lowX, double lowY, double highX, double highY)
{
    vector<Point2d> drawLine;
    double stepX, stepY;
    double meshCnt = 11;
    
    stepX = (highX - lowX)/meshCnt;
    stepY = (highY - lowY)/meshCnt;
    //double step = MIN(stepX, stepY);
    
    for(double i = lowX; i <= highX; i += stepX)
    {
        drawLine.clear();
        for(double j = lowY; j <= highY; j += stepY)
        {
            Mat tmp = Mat(4, 1, CV_64F);
            tmp.at<double>(3) = 1;
            tmp.at<double>(0) = i;
            tmp.at<double>(1) = j;
            tmp.at<double>(2) = 0;
            Mat result = converseMatrix * tmp;
            drawLine.push_back(Point2d(result.at<double>(0)/result.at<double>(2), result.at<double>(1)/result.at<double>(2)));
            //printf("%lf %lf\n",result.at<double>(0)/result.at<double>(2), result.at<double>(1)/result.at<double>(2));
        }
        for(int j = 0; j < drawLine.size() - 1; j++)
        {
            line(drawPlane, drawLine[j], drawLine[j+1], Scalar(255, 0, 0));
        }
    }
    
    for(double i = lowY; i <= highY; i += stepY)
    {
        drawLine.clear();
        for(double j = lowX; j <= highX; j += stepX)
        {
            Mat tmp = Mat(4, 1, CV_64F);
            tmp.at<double>(3) = 1;
            tmp.at<double>(0) = j;
            tmp.at<double>(1) = i;
            tmp.at<double>(2) = 0;
            Mat result = converseMatrix * tmp;
            drawLine.push_back(Point2d(result.at<double>(0)/result.at<double>(2), result.at<double>(1)/result.at<double>(2)));
        }
        for(int j = 0; j < drawLine.size() - 1; j++)
        {
            line(drawPlane, drawLine[j], drawLine[j+1], Scalar(255, 0, 0));
        }
    }
}


