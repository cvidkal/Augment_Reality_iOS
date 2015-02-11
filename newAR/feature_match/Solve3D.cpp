//
//  Solve3D.cpp
//  newAR
//
//  Created by Shangjin Zhai on 14-12-6.
//  Copyright (c) 2014�� Shangjin Zhai. All rights reserved.
//

#include "Solve3D.h"

//#include "5point.h"
#include "levmar.h"


Solve3D::~Solve3D()
{
    
}

Solve3D::Solve3D()
{
  //  KMatrix = Mat::zeros(3, 3, CV_64F);
//    rotationMatrix = Mat::zeros(3, 3, CV_64F);
    invertMatrix = Mat::zeros(3, 3, CV_64F);
    distCoeffs = Mat::zeros(4, 1, CV_64F);
    TVector = Mat::zeros(3, 1, CV_64F);
}

void Solve3D::Calibrate_Points(Point2f pp,double focal,vector<Point2f> &origin,vector<Point2f> &out)
{
    for(int i=0;i<origin.size();i++)
    {
        out.push_back(Point2f((origin[i].x-pp.x)/focal,(origin[i].y-pp.y)/focal));
    }
}
/*
bool Solve3D::Solve5Point(vector<Point2f> &pts1, vector<Point2f> &pts2, int num_pts, Mat &E, Mat &P, std::vector<int> &ret_inliers)
{
    double * pts11 = new double[pts1.size()*2];
    double * pts22 = new double[pts2.size()*2];
    std::vector<cv::Mat> ret_E;
    std::vector<cv::Mat> ret_P;
    for(int i=0;i<pts1.size();i++)
    {
        pts11[i*2] = pts1[i].x  ;
        pts11[i*2+1] = pts1[i].y;
    }
    for(int i=0;i<pts2.size();i++)
    {
        pts22[i*2] = pts2[i].x  ;
        pts22[i*2+1] = pts2[i].y;
    }
    bool ret = Solve5PointEssential(pts11,pts22,num_pts,ret_E,ret_P,ret_inliers);
    if(ret)
    {
        E=ret_E[0];
            if(cv::determinant(ret_P[0](cv::Range(0,3), cv::Range(0,3))) < 0)
            {
                P=ret_P[0] * -1;
            }
            else
            {
                P=ret_P[0];
            }
        
    }
    delete[] pts11;
    delete[] pts22;
    return ret;
}
*/


void func2(double *p, double *hx, int m, int n, void *adata)
{
    int pointsnum = (int)(((double*)adata)[0]);
    double * pts3d1 = ((double*)adata)+1;
    double * pts3d2 = &((double*)adata)[pointsnum * 3 + 1];
    double s = p[0];
    Mat _r(3, 1, CV_32F);
    _r.at<float>(0, 0) = p[1];
    _r.at<float>(1, 0) = p[2];
    _r.at<float>(2, 0) = p[3];
    Mat R;
    Rodrigues(_r, R);
    
    float *a = (float*)R.data;
    
    double r11 = a[0], r12 = a[1], r13 = a[2], r21 = a[3], r22 = a[4], r23 = a[5], r31 = a[6], r32 = a[7], r33 = a[8];
    double t1 = p[4], t2 = p[5], t3 = p[6];
    
    
    
    for (int i = 0; i < pointsnum; i++)
    {
        double x, y, z;
        double x1 = pts3d1[i * 3];
        double x2 = pts3d1[i * 3 + 1];
        double x3 = pts3d1[i * 3 + 2];
        double y1 = pts3d2[i * 3];
        double y2 = pts3d2[i * 3 + 1];
        double y3 = pts3d2[i * 3 + 2];
        x = t1 + r11*s*x1 + r12*s*x2 + r13*s*x3;
        y = t2 + r21*s*x1 + r22*s*x2 + r23*s*x3;
        z = t3 + r31*s*x1 + r32*s*x2 + r33*s*x3;
        hx[3*i] = fabs(x - y1);
        hx[3 * i + 1] = fabs(y - y2);
        hx[3 * i + 2] = fabs(z - y3);
        x = -(r11*(t1 - y1) + r21*(t2 - y2) + r31*(t3 - y3)) / s;
        y = -(r12*(t1 - y1) + r22*(t2 - y2) + r32*(t3 - y3)) / s;
        z = -(r13*(t1 - y1) + r23*(t2 - y2) + r33*(t3 - y3)) / s;
        hx[3 * i] += fabs(x - x1);
        hx[3 * i + 1] += fabs(y - x2);
        hx[3 * i + 2] += fabs(z - x3);
    }
}

void jacf(double *p, double *j, int m, int n, void *adata) /* function to evaluate the Jacobian \part x / \part p */
{
    int pointsnum = (int)(((double*)adata)[0]);
    double * pts3d1 = (double*)adata + 1;
    double * pts3d2 = &((double*)adata)[pointsnum * 3 + 1];
    double s = p[0];
    Mat _r(3,1,CV_32F), R, jac;
    _r.at<float>(0, 0) = p[1];
    _r.at<float>(1, 0) = p[2];
    _r.at<float>(2, 0) = p[3];
    Rodrigues(_r, R, jac);
    float *r = (float*)R.data;
    float r11 = r[0], r12 = r[1], r13 = r[2], r21 = r[3], r22 = r[4], r23 = r[5], r31 = r[6], r32 = r[7], r33 = r[8];
    float t1 = p[4], t2 = p[5], t3 = p[6];
    jac = jac.t();
    float * jt = (float*)jac.data;
    float j111 = jt[0], j112 = jt[1], j113 = jt[2],
    j121 = jt[3], j122 = jt[4], j123 = jt[5],
    j131 = jt[6], j132 = jt[7], j133 = jt[8],
    j211 = jt[9], j212 = jt[10], j213 = jt[11],
    j221 = jt[12], j222 = jt[13], j223 = jt[14],
    j231 = jt[15], j232 = jt[16], j233 = jt[17],
    j311 = jt[18], j312 = jt[19], j313 = jt[20],
    j321 = jt[21], j322 = jt[22], j323 = jt[23],
    j331 = jt[24], j332 = jt[25], j333 = jt[26];
    for (int i = 0; i < pointsnum; i++)
    {
        float x1 = pts3d1[i * 3];
        float x2 = pts3d1[i * 3 + 1];
        float x3 = pts3d1[i * 3 + 2];
        float y1 = pts3d2[i * 3];
        float y2 = pts3d2[i * 3 + 1];
        float y3 = pts3d2[i * 3 + 2];
        j[i * 21] = r11*x1 - (r11*(t1 - y1) + r21*(t2 - y2) + r31*(t3 - y3)) / (s*s) + r12*x2 + r13*x3;
        j[i * 21 + 1] = r21*x1 - (r12*(t1 - y1) + r22*(t2 - y2) + r32*(t3 - y3)) / (s*s) + r22*x2 + r23*x3;
        j[i * 21 + 2] = r31*x1 - (r13*(t1 - y1) + r23*(t2 - y2) + r33*(t3 - y3)) / (s*s) + r32*x2 + r33*x3;
        j[i * 21 + 3] = t1 + (j111*(t1 - y1) + j211*(t2 - y2) + j311*(t3 - y3)) / s + j111*s*x1 + j121*s*x2 + j131*s*x3;
        j[i * 21 + 4] = t2 + (j121*(t1 - y1) + j221*(t2 - y2) + j321*(t3 - y3)) / s + j211*s*x1 + j221*s*x2 + r23*s*x3;
        j[i * 21 + 5] = t3 + (j131*(t1 - y1) + j231*(t2 - y2) + j331*(t3 - y3)) / s + j311*s*x1 + j321*s*x2 + j331*s*x3;
        j[i * 21 + 6] = t1 + (j112*(t1 - y1) + j212*(t2 - y2) + j312*(t3 - y3)) / s + j112*s*x1 + j122*s*x2 + j132*s*x3;
        j[i * 21 + 7] = t2 + (j122*(t1 - y1) + j222*(t2 - y2) + j322*(t3 - y3)) / s + j212*s*x1 + j222*s*x2 + j232*s*x3;
        j[i * 21 + 8] = t3 + (j132*(t1 - y1) + j232*(t2 - y2) + j332*(t3 - y3)) / s + j312*s*x1 + j322*s*x2 + j332*s*x3;
        j[i * 21 + 9] = t1 + (j113*(t1 - y1) + j213*(t2 - y2) + j313*(t3 - y3)) / s + j113*s*x1 + j123*s*x2 + j133*s*x3;
        j[i * 21 + 10] = t2 + (j123*(t1 - y1) + j223*(t2 - y2) + j323*(t3 - y3)) / s + j213*s*x1 + j223*s*x2 + j233*s*x3;
        j[i * 21 + 11] = t3 + (j133*(t1 - y1) + j233*(t2 - y2) + j333*(t3 - y3)) / s + j313*s*x1 + j323*s*x2 + j333*s*x3;
        j[i * 21 + 12] = r11 / s + 1;
        j[i * 21 + 13] = r12 / s;
        j[i * 21 + 14] = r13 / s;
        j[i * 21 + 15] = r21 / s;
        j[i * 21 + 16] = r22 / s + 1;
        j[i * 21 + 17] = r23 / s;
        j[i * 21 + 18] = r31 / s;
        j[i * 21 + 19] = r32 / s;
        j[i * 21 + 20] = r33 / s + 1;
    }
}

void LM_SRT2(Mat pts3d1, Mat pts3d2, double&s, Mat&R, Mat &t)
{
    double p[13];
    int n = pts3d1.rows;
    p[0] = s;
    Mat _rv;
    Rodrigues(R, _rv);
    for (int i = 0; i < 3; i++)
    {
        p[i + 1] = ((float*)_rv.data)[i];
    }
    p[4] = t.at<float>(0, 0);
    p[5] = t.at<float>(1, 0);
    p[6] = t.at<float>(2, 0);
    double * adata = new double[n * 3 * 2 + 1];
    adata[0] = n;
    for (int i = 0; i < n; i++)
    {
        adata[i * 3 + 1] = pts3d1.at<float>(i, 0);
        adata[i * 3 + 2] = pts3d1.at<float>(i, 1);
        adata[i * 3 + 3] = pts3d1.at<float>(i, 2);
        adata[i * 3 + n * 3 + 1] = pts3d2.at<float>(i, 0);
        adata[i * 3 + n * 3 + 2] = pts3d2.at<float>(i, 1);
        adata[i * 3 + n * 3 + 3] = pts3d2.at<float>(i, 2);
    }
    int q = dlevmar_der(func2,jacf,p, NULL, 7, n*3, 100, NULL, NULL, NULL, NULL, adata);
    s = p[0];
    _rv.at<float>(0, 0) = p[1];
    _rv.at<float>(1, 0) = p[2];
    _rv.at<float>(2, 0) = p[3];
    Rodrigues(_rv, R);
    
    t.at<float>(0, 0) = p[4];
    t.at<float>(1, 0) = p[5];
    t.at<float>(2, 0) = p[6];
}




void Solve3D::solveSRT(Mat &pts3d1, Mat &pts3d2, double &s, Mat &R, Mat &t)
{
    Scalar_<float> a = mean(pts3d1);
    Scalar_<float> b = mean(pts3d2);
    Mat ux(3, 1, CV_32F);
    Mat uy(3, 1, CV_32F);
    for (int i = 0; i < 3; i++)
    {
        ux.at<float>(i, 0) = a[i];
        uy.at<float>(i, 0) = b[i];
    }
    Mat cov = Mat::zeros(3, 3, CV_32F);
    pts3d1 = pts3d1.reshape(1);//N*3
    pts3d2 = pts3d2.reshape(1);
    for (int i = 0; i < pts3d1.rows; i++)
    {
        cov +=(pts3d2.row(i).t() - uy)*(pts3d1.row(i).t() - ux).t();
    }
    cov = cov / pts3d1.rows;
    Mat S = Mat::eye(3, 3, CV_32F);
    
    SVD svd = SVD(cov, SVD::FULL_UV);
    if (determinant(svd.u)*determinant(svd.vt) < 0)
    {
        S.at<float>(2, 2) = -1;
    }
    R = svd.u*S*svd.vt;
    double vx = 0;
    Scalar c = sum(svd.w);
    double tr = 0;
    for (int i = 0; i < 3; i++)
    {
        tr += c[i] * S.at<float>(i, i);
    }
    for (int i = 0; i < pts3d1.rows; i++)
    {
        vx += norm(pts3d1.row(i).t() - ux, NORM_L2SQR);
    }
    s = tr / (vx / pts3d1.rows);
    t = uy - (s*R*ux);
    LM_SRT2(pts3d1, pts3d2, s, R, t);

}

void Solve3D::transformSRT(const vector<Point3f> &pts3d1,vector<Point3f> &pts3d2,double s,Mat R,Mat t)
{
    pts3d2.resize(pts3d1.size());
    float * r = (float*)R.data;
    float * tt = (float*)t.data;
    for(int i=0;i<pts3d1.size();i++)
    {
        pts3d2[i].x =s*(r[0]*pts3d1[i].x+r[1]*pts3d1[i].y+r[2]*pts3d1[i].z)+tt[0];
        pts3d2[i].y =s*(r[3]*pts3d1[i].x+r[4]*pts3d1[i].y+r[5]*pts3d1[i].z)+tt[1];
        pts3d2[i].z =s*(r[6]*pts3d1[i].x+r[7]*pts3d1[i].y+r[8]*pts3d1[i].z)+tt[2];
    }
}




void Solve3D::setKMatrix(Mat &KMatrix,double imageWidth, double imageHeight, double focus)
{
    KMatrix = Mat::zeros(3, 3, CV_32F);
    KMatrix.at<double>(0, 0) =  focus;
    KMatrix.at<double>(1, 1) =  focus;
    KMatrix.at<double>(2, 2) = 1;
    KMatrix.at<double>(0, 2) = (imageWidth - 1)/2;
    KMatrix.at<double>(1, 2) = (imageHeight - 1)/2;
 }

Mat Solve3D::setRotationMatrix(const CMRotationMatrix &rotation)
{

    Mat rotationMatrix(3,3,CV_32F);
    rotationMatrix.at<float>(0, 0) = -rotation.m12;
    rotationMatrix.at<float>(0, 1) = -rotation.m22;
    rotationMatrix.at<float>(0, 2) = -rotation.m32;
    
    rotationMatrix.at<float>(1, 0) = rotation.m11;
    rotationMatrix.at<float>(1, 1) = rotation.m21;
    rotationMatrix.at<float>(1, 2) = rotation.m31;
    
    rotationMatrix.at<float>(2, 0) = -rotation.m13;
    rotationMatrix.at<float>(2, 1) = -rotation.m23;
    rotationMatrix.at<float>(2, 2) = -rotation.m33;
    return rotationMatrix;
    

}




void Solve3D::setTVector(const cv::Mat &_TVector)
{
    TVector = _TVector;
}



void Solve3D::get3DPoints(Mat KMatrix,Mat rotationMatrix,vector<KeyPoint> &pts2D,vector<Point3f> &pts3D,Mat &TVector)
{
#define REAL_HEIGHT 5.0

	pts3D.clear();
    TVector.release();
    TVector = Mat(3,1,CV_32F);
    cv::Point3d point3D;
    Mat imageCoordinate = Mat::Mat(3, 1, CV_32F);
    Mat realCoordinate;
    Mat minusK, minusR;
    invert(KMatrix, minusK, CV_SVD);
    invert(rotationMatrix, minusR, CV_SVD);
    
	for (int i = 0; i < pts2D.size(); i++)
	{
		imageCoordinate.at<float>(2, 0) = 1;
		float realHeight = REAL_HEIGHT, realScale = 1;

		imageCoordinate.at<float>(0, 0) = pts2D[i].pt.x;
		imageCoordinate.at<float>(1, 0) = pts2D[i].pt.y;

		Mat right = minusR * minusK * imageCoordinate;
		realScale = -realHeight / right.at<float>(2, 0);

		point3D.x = right.at<float>(0, 0) * realScale;//+ offset.x;
		point3D.y = right.at<float>(1, 0) * realScale;//+ offset.y;
		point3D.z = right.at<float>(2, 0) * realScale + REAL_HEIGHT;
		pts3D.push_back(point3D);
	}

    
    Mat tmp = Mat(3, 1, CV_32F);
    tmp.at<float>(0) = 0;
    tmp.at<float>(1) = 0;
    tmp.at<float>(2) = REAL_HEIGHT;
    tmp = -rotationMatrix * tmp;
    
    for(int i=0; i< 3; i++)
        TVector.at<float>(i) = tmp.at<float>(i);
 
}


void Solve3D::get3DPoints(Mat KMatrix,const Mat rotationMatrix,const Mat TVector,const vector<KeyPoint> &pts2D,vector<Point3f> &pts3D)
{
    double height = TVector.at<float>(2,0);

    pts3D.clear();
    cv::Point3d point3D;
    Mat imageCoordinate = Mat::Mat(3, 1, CV_32F);
    Mat realCoordinate;
    Mat minusK, minusR;
    invert(KMatrix, minusK, CV_SVD);
    invert(rotationMatrix, minusR, CV_SVD);
    
    for (int i = 0; i < pts2D.size(); i++)
    {
        imageCoordinate.at<float>(2, 0) = 1;
        float realHeight = height, realScale = 1;
        
        imageCoordinate.at<float>(0, 0) = pts2D[i].pt.x;
        imageCoordinate.at<float>(1, 0) = pts2D[i].pt.y;
        
        Mat right = minusR * minusK * imageCoordinate;
        realScale = -realHeight / right.at<float>(2, 0);
        
        point3D.x = right.at<float>(0, 0) * realScale;//+ offset.x;
        point3D.y = right.at<float>(1, 0) * realScale;//+ offset.y;
        point3D.z = right.at<float>(2, 0) * realScale + height;
        pts3D.push_back(point3D);
    }
    
}


Mat Solve3D::getNextInvertMatrix(Mat KMatrix,Mat R,Mat t)
{
    Mat converseMatrix;
    Mat P = Mat(3, 4, CV_32F);
    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            P.at<float>(i, j) = R.at<float>(i, j);
        }
        P.at<float>(i, 3) = (float)t.at<float>(i);
    }
    return converseMatrix = KMatrix * P;
//    invert(converseMatrix, invertMatrix, CV_SVD);
    
	 
}


void Solve3D::get2DPoint(Mat KMatrix,Mat R,Mat t,vector<Point3f> &points3DVector,vector<Point2f> &points2DVector,Mat &drawMatch)
{
    Mat converseMatrix = getNextInvertMatrix(KMatrix, R, t);

    for(int i= 0; i<points3DVector.size();i++)
    {
        Mat realCoordinate = Mat(4, 1, CV_32F);
        realCoordinate.at<float>(3, 0) = 1;
        Mat imageCoordinate;
        double realScale;
        realCoordinate.at<float>(0,0) = points3DVector[i].x;//all3DPoints[i].x;
        realCoordinate.at<float>(1,0) = points3DVector[i].y;
        realCoordinate.at<float>(2,0) = points3DVector[i].z;
    
        imageCoordinate = converseMatrix * realCoordinate;
    
        realScale = imageCoordinate.at<float>(2, 0);
    
        //printf("%d: %lf %lf haha %lf %lf %lf\n", i, imageCoordinate.at<double>(0, 0)/realScale , points2DVector[i].x, imageCoordinate.at<double>(1, 0)/realScale, points2DVector[i].y, realScale);
        circle(drawMatch, Point2d(imageCoordinate.at<float>(0, 0)/realScale , imageCoordinate.at<float>(1,0)/realScale), 1, Scalar(255, 0, 0));
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
    Mat rvec,TVector;
    Mat temp;
    TickMeter tm;
    tm.start();
    solvePnPRansac(P3D, P2D, KMatrix, dtmp, R, temp , false);
    tm.stop();
    R.convertTo(rvec,CV_32F);
    temp.convertTo(t, CV_32F);
    Rodrigues(rvec,R);
}


Mat Solve3D::getP(Mat R,Mat t)
{
    Mat PP = Mat(4, 4, CV_32F);
    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            PP.at<float>(i, j) = R.at<float>(i, j);
            if(i == 2)
                PP.at<float>(i, j) = -R.at<float>(i, j);
        }
        PP.at<float>(i, 3) = t.at<float>(i);
        if(i == 2)
            PP.at<float>(i, 3) = -PP.at<float>(i, 3);
    }
    PP.at<float>(3, 0) =PP.at<float>(3, 1) =PP.at<float>(3, 2) =0;
    PP.at<float>(3, 3) =1;
    return PP;
}
/*
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
*/
void Solve3D::get3DPoints(vector<Point3d> &realPoints)
{
    realPoints.clear();
    realPoints = points3DVector;
}

void Solve3D::getHighAndLow(vector<Point3f> &points3DVector,double &lowX, double &lowY, double &highX, double &highY)
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

void Solve3D::printPlane(Mat KMatrix,Mat R,Mat t,cv::Mat &drawPlane, double lowX, double lowY, double highX, double highY)
{
    Mat converseMatrix = getNextInvertMatrix(KMatrix, R, t);
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
            Mat tmp = Mat(4, 1, CV_32F);
            tmp.at<float>(3) = 1;
            tmp.at<float>(0) = i;
            tmp.at<float>(1) = j;
            tmp.at<float>(2) = 0;
            Mat result = converseMatrix * tmp;
            drawLine.push_back(Point2d(result.at<float>(0)/result.at<float>(2), result.at<float>(1)/result.at<float>(2)));
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
            Mat tmp = Mat(4, 1, CV_32F);
            tmp.at<float>(3) = 1;
            tmp.at<float>(0) = j;
            tmp.at<float>(1) = i;
            tmp.at<float>(2) = 0;
            Mat result = converseMatrix * tmp;
            drawLine.push_back(Point2d(result.at<float>(0)/result.at<float>(2), result.at<float>(1)/result.at<float>(2)));
        }
        for(int j = 0; j < drawLine.size() - 1; j++)
        {
            line(drawPlane, drawLine[j], drawLine[j+1], Scalar(255, 0, 0));
        }
    }
}


