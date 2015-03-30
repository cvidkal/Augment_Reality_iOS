//
//  Solve3D.cpp
//  newAR
//
//  Created by Shangjin Zhai on 14-12-6.
//  Copyright (c) 2014年 Shangjin Zhai. All rights reserved.
//

#include "Solve3D.h"

//#include "5point.h"
#include "levmar.h"
#include "sba.h"

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
    double p[7];
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

void func(double *p, double *hx, int m, int n, void *adata)
{
    int pointsnum = (int)(((double*)adata)[0]);
    double * pts3d1 = (double*)adata + 1;
    double * pts3d2 = &((double*)adata)[pointsnum * 3 + 1];
    double s = p[0];
    double r11 = p[1], r12 = p[2], r13 = p[3], r21 = p[4], r22 = p[5], r23 = p[6], r31 = p[7], r32 = p[8], r33 = p[9];
    double t1 = p[10], t2 = p[11], t3 = p[12];
    
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
        hx[3 * i] = fabs(x - y1);
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

void LM_SRT(Mat pts3d1, Mat pts3d2, double &s, Mat &R, Mat &t)
{
    double p[7];
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
    int a = dlevmar_dif(func2, p, NULL, 7, n * 3, 100, NULL, NULL, NULL, NULL, adata);
    s = p[0];
    _rv.at<float>(0, 0) = p[1];
    _rv.at<float>(1, 0) = p[2];
    _rv.at<float>(2, 0) = p[3];
    Rodrigues(_rv, R);
    
    t.at<float>(0, 0) = p[4];
    t.at<float>(1, 0) = p[5];
    t.at<float>(2, 0) = p[6];
}

void Solve3D::solveSRT(Mat &src, Mat &dst, double &s, Mat &R, Mat &t)
{
    Scalar_<float> a = mean(src);
    Scalar_<float> b = mean(dst);
    Mat ux(3, 1, CV_32F);
    Mat uy(3, 1, CV_32F);
    for (int i = 0; i < 3; i++)
    {
        ux.at<float>(i, 0) = a[i];
        uy.at<float>(i, 0) = b[i];
    }
    Mat cov = Mat::zeros(3, 3, CV_32F);
    src = src.reshape(1);//N*3
    dst = dst.reshape(1);
    for (int i = 0; i < src.rows; i++)
    {
        cov += (dst.row(i).t() - uy)*(src.row(i).t() - ux).t();
    }
    cov = cov / src.rows;
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
    for (int i = 0; i < src.rows; i++)
    {
        vx += norm(src.row(i).t() - ux, NORM_L2SQR);
    }
    s = tr / (vx / src.rows);
    t = uy - (s*R*ux);
    
    LM_SRT2(src, dst, s, R, t);
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
    KMatrix.at<double>(0, 2) = (imageHeight - 1)/2;
    KMatrix.at<double>(1, 2) = (imageWidth - 1)/2;
 }

Mat Solve3D::setRotationMatrix(const CMRotationMatrix &rotation)
{

    Mat rotationMatrix(3,3,CV_32F);

    rotationMatrix.at<float>(0, 0) = -rotation.m12;
    rotationMatrix.at<float>(0, 1) = -rotation.m22;
    rotationMatrix.at<float>(0, 2) = -rotation.m32;
    
    rotationMatrix.at<float>(1, 0) = -rotation.m11;
    rotationMatrix.at<float>(1, 1) = -rotation.m21;
    rotationMatrix.at<float>(1, 2) = -rotation.m31;
    
    rotationMatrix.at<float>(2, 0) = -rotation.m13;
    rotationMatrix.at<float>(2, 1) = -rotation.m23;
    rotationMatrix.at<float>(2, 2) = -rotation.m33;
 

    return rotationMatrix;
}

Mat Solve3D::setRotationMatrix(double yaw,double pitch,double roll)
{
    Mat_<float> rotationMatrix(3,3);
    const double cx = cos(pitch), sx = sin(pitch), cy = cos(roll), sy = sin(roll), cz = cos(yaw), sz = sin(yaw);
    const double cycz = cy * cz, sxsy = sx * sy, cysz = cy * sz;
    rotationMatrix.at<float>(0, 0) = cycz - sxsy * sz;
    rotationMatrix.at<float>(0, 1) = cysz + sxsy * cz;
    rotationMatrix.at<float>(0, 2) = -cx * sy;
    
    rotationMatrix.at<float>(1, 0) = -cx * sz;
    rotationMatrix.at<float>(1, 1) = cx * cz;
    rotationMatrix.at<float>(1, 2) = sx;
    
    rotationMatrix.at<float>(2, 0) = sy * cz + sx * cysz;
    rotationMatrix.at<float>(2, 1) = sy * sz - sx * cycz;
    rotationMatrix.at<float>(2, 2) = cx * cy;
    Mat_<float> Rk(3,3);
    Rk<<0,-1,0,-1,0,0,0,0,-1;
    rotationMatrix = Rk*rotationMatrix;
//    cout<<rotationMatrix<<endl;
    return rotationMatrix;
}




void Solve3D::setTVector(const cv::Mat &_TVector)
{
    TVector = _TVector;
}



void Solve3D::get3DPoints(Mat KMatrix,Mat rotationMatrix,vector<KeyPoint> &pts2D,vector<Point3f> &pts3D,Mat &TVector)
{
#define REAL_HEIGHT 5.0
    TVector = Mat_<float>(3,1);
    for (int i=0; i<pts2D.size(); i++)
    {

    cv::Point3f point3D;
    Mat imageCoordinate = Mat::Mat(3, 1, CV_32F);
    Mat realCoordinate;
    Mat minusK, minusR;
    invert(KMatrix, minusK, CV_SVD);
    invert(rotationMatrix, minusR, CV_SVD);
    
    imageCoordinate.at<float>(2, 0) = 1;
    double realHeight = REAL_HEIGHT, realScale = 1;
    
    imageCoordinate.at<float>(0, 0) = pts2D[i].pt.x;
    imageCoordinate.at<float>(1, 0) = pts2D[i].pt.y;
    
    Mat right = minusR * minusK * imageCoordinate;
    realScale = -realHeight/right.at<float>(2, 0);
    
    point3D.x = right.at<float>(0, 0) * realScale ;
    point3D.y = right.at<float>(1, 0) * realScale ;
    point3D.z = right.at<float>(2, 0) * realScale + REAL_HEIGHT;
    
    Mat tmp = Mat(3, 1, CV_32F);
    tmp.at<float>(0) = 0;
    tmp.at<float>(1) = 0;
    tmp.at<float>(2) = REAL_HEIGHT;
    tmp = -rotationMatrix * tmp;
    
    for(int i=0; i< 3; i++)
        TVector.at<float>(i) = tmp.at<float>(i);
    //cout<<"TVector:"<<TVector<<endl;
    
    pts3D.push_back(point3D);
    }
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

bool Solve3D::getCameraRT(Mat KMatrix,vector<Point3f> &P3D,vector<Point2f> &P2D,Mat &R,Mat &t,vector<int> &inliers,bool isTracked)
{
    Mat dtmp = Mat::zeros(4, 1, CV_32F);
    Mat rvec,TVector;
    Mat temp;
    Scalar origin;
    double err1=0, err2=0;
    //    Mat p3d,p2d;
    //   Mat(P3D).convertTo(p3d, CV_64F);
    //    Mat(P2D).convertTo(p2d, CV_64F);
    solvePnPRansac(P3D, P2D, KMatrix, Mat(), rvec, temp,false,100,3,20,inliers,CV_EPNP);
    vector<Point3f> P3D_in;
    vector<Point2f> P2D_in;
    if(inliers.size() <8)
    {
        return false;
    }
    for(auto i:inliers)
    {
        P3D_in.push_back(P3D[i]);
        P2D_in.push_back(P2D[i]);
    }
    
    if(!isTracked)
    {
        //    cout<<rvec<<temp<<endl;
        //   solvePnP(Mat(P3D_in), Mat(P2D_in), KMatrix, Mat(), rvec, temp,CV_ITERATIVE);
        solvePnPRansac(P3D_in, P2D_in, KMatrix, Mat(), rvec, temp,false,100,3,20);
    }
    else
    {
        Mat rr;
        Rodrigues(R,rr);
        FromRotationMatrix(R, origin);
        vector<Point2f> pts2d;
        projectPoints(P3D_in, rr, t, KMatrix, Mat(), pts2d);
        for (int i = 0; i<pts2d.size(); i++)
        {
            err1 += sqrtf(pow(pts2d[i].x - P2D_in[i].x, 2) + pow(pts2d[i].y - P2D_in[i].y, 2));
        }
        err1 /= pts2d.size();
        rr.convertTo(rvec, CV_64F);
        t.convertTo(temp, CV_64F);
        solvePnPRansac(P3D, P2D, KMatrix, Mat(), rvec, temp,true,100,3,20);
        temp.convertTo(TVector, CV_32F);
        rvec.convertTo(rr, CV_32F);
        Mat RR;
        Rodrigues(rr, RR);
        
        Scalar optimised;
        FromRotationMatrix(RR, optimised);
        projectPoints(P3D_in, rr, TVector, KMatrix, Mat(), pts2d);
        for (int i = 0; i<pts2d.size(); i++)
        {
            err2 += sqrtf(pow(pts2d[i].x - P2D_in[i].x, 2) + pow(pts2d[i].y - P2D_in[i].y, 2));
        }
        err2 /= pts2d.size();
        //		CV_Assert(err2 <= err1);
        double threshold = err2 + (err1 - err2)*0.6;
        double w = 1;
        Scalar output;
        for (int i = 0; i < 10; i++)
        {
            w -= 0.1;
            double err3 = 0;
            Slerp(w, optimised, origin, output);
            ToRotationMatrix(output, rr);
            Rodrigues(rr, RR);
            Mat tt;
            Tlerp(w, t, TVector, tt);
            projectPoints(P3D_in, RR, tt, KMatrix, Mat(), pts2d);
            for (int j = 0; j<pts2d.size(); j++)
            {
                err3 += (pow(pts2d[j].x - P2D_in[j].x, 2) + pow(pts2d[j].y - P2D_in[j].y, 2));
            }
            err3 /= pts2d.size();
            if (err3>threshold)
            {
                RR.convertTo(rvec, CV_64F);
                temp = tt;
                break;
            }
        }
        
    }
    //    cout<<rvec<<temp<<endl;
    //    isTracked = false;
    /*
     if(isTracked)
     {
     vector<Point3d> P3D_in;
     vector<Point2d> P2D_in;
     for(auto i:inliers)
     {
     P3D_in.push_back(P3D[i]);
     P2D_in.push_back(P2D[i]);
     }
     Mat rr;
     Rodrigues(R,rvec);
     
     t.convertTo(temp, CV_64F);
     cout<<rvec<<temp<<endl;
     solvePnP(Mat(P3D_in),Mat(P2D_in),KMatrix,Mat(),rvec,temp,true,CV_ITERATIVE);
     cout<<rvec<<endl;
     CV_Assert(rvec.at<double>(0,0)<1000);
     //        solvePnPRansac(P3D, P2D, KMatrix, dtmp, rvec, temp , true);
     cout<<rvec<<temp<<endl;
     }
     */
    /*
     else
     {
     solvePnPRansac(P3D, P2D, KMatrix, dtmp, rvec, temp , false);
     }
     */
    rvec.convertTo(TVector, CV_32F);
    Rodrigues(TVector, R);
    temp.convertTo(t, CV_32F);
    
    return true;
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





__inline bool sba_getIdx(struct sba_crsm * idxij, int i, int j, int &idx)
{
    int rowIdx = idxij->rowptr[i];
    int rowEnd = idxij->rowptr[i + 1];
    for (int i = rowIdx; i < rowEnd; i++)
    {
        if (idxij->colidx[i] == j)
        {
            idx = idxij->val[i];
            return true;
        }
        else if (idxij->colidx[i]>j)
        {
            return false;
        }
    }
    return false;
    
}

void func(double *p, struct sba_crsm *idxij, int *rcidxs, int *rcsubs, double *hx, void *adata)
{
    int m = idxij->nc;//images
    int n = idxij->nr;//points
    double * data = (double*)adata;
    double fx = data[0], fy = data[1], cx = data[2], cy = data[3];
    double r11, r12, r13, r21, r22, r23, r31, r32, r33;
    double t1, t2, t3;
    Mat rvec(3, 1, CV_32F);
    Mat R;
    
    const int offset = 4;
    for (size_t j = 0; j < m; j++)
    {
        float* rd = (float*)rvec.data;
        rd[0] = p[j * 6];
        rd[1] = p[j * 6 + 1];
        rd[2] = p[j * 6 + 2];
        Rodrigues(rvec, R);
        rd = (float*)R.data;
        r11 = rd[0]; r12 = rd[1]; r13 = rd[2];
        r21 = rd[3]; r22 = rd[4]; r23 = rd[5];
        r31 = rd[6]; r32 = rd[7]; r33 = rd[8];
        t1 = p[j * 6 + 3];
        t2 = p[j * 6 + 4];
        t3 = p[j * 6 + 5];
        
        for (size_t i = 0; i < n; i++)
        {
            int idx;
            if (sba_getIdx(idxij, i, j, idx))
            {
                double X = data[offset + 3 * i], Y = data[offset + 3 * i + 1], Z = data[offset + 3 * i + 2];
                double t = t3 + X*r31 + Y*r32 + Z*r33;
                hx[2 * idx] = (cx*t3 + fx*t1 + X*(cx*r31 + fx*r11) + Y*(cx*r32 + fx*r12) + Z*(cx*r33 + fx*r13)) / t;
                hx[2 * idx + 1] = (cy*t3 + fy*t2 + X*(cy*r31 + fy*r21) + Y*(cy*r32 + fy*r22) + Z*(cy*r33 + fy*r23)) / t;
            }
        }
    }
    
}




void fjac(double *p, struct sba_crsm *idxij, int *rcidxs, int *rcsubs, double *jac, void *adata)
{
    int m = idxij->nc;//images
    int n = idxij->nr;//points
    double * data = (double*)adata;
    double fx = data[0], fy = data[1], cx = data[2], cy = data[3];
    double r11, r12, r13, r21, r22, r23, r31, r32, r33;
    double t1, t2, t3;
    
    Mat rvec(3, 1, CV_32F);
    Mat R;
    Mat jacM;
    const int offset = 4;
    for (size_t j = 0; j < m; j++)
    {
        float* rd = (float*)rvec.data;
        rd[0] = p[j * 6];
        rd[1] = p[j * 6 + 1];
        rd[2] = p[j * 6 + 2];
        t1 = p[j * 6 + 3];
        t2 = p[j * 6 + 4];
        t3 = p[j * 6 + 5];
        Rodrigues(rvec, R, jacM);
        rd = (float*)R.data;
        r11 = rd[0]; r12 = rd[1]; r13 = rd[2];
        r21 = rd[3]; r22 = rd[4]; r23 = rd[5];
        r31 = rd[6]; r32 = rd[7]; r33 = rd[8];
        jacM = jacM.t();
        float * jt = (float*)jacM.data;
        float j111 = jt[0], j112 = jt[1], j113 = jt[2],
        j121 = jt[3], j122 = jt[4], j123 = jt[5],
        j131 = jt[6], j132 = jt[7], j133 = jt[8],
        j211 = jt[9], j212 = jt[10], j213 = jt[11],
        j221 = jt[12], j222 = jt[13], j223 = jt[14],
        j231 = jt[15], j232 = jt[16], j233 = jt[17],
        j311 = jt[18], j312 = jt[19], j313 = jt[20],
        j321 = jt[21], j322 = jt[22], j323 = jt[23],
        j331 = jt[24], j332 = jt[25], j333 = jt[26];
        for (size_t i = 0; i < n; i++)
        {
            int idx;
            if (sba_getIdx(idxij, i, j, idx))
            {
                double X = data[offset + 3 * i], Y = data[offset + 3 * i + 1], Z = data[offset + 3 * i + 2];
                double t = t3 + X*r31 + Y*r32 + Z*r33;
                jac[idx * 12] = (X*fx*j111) / t - j321*((Y*(cx*t3 + fx*t1 + X*(cx*r31 + fx*r11) + Y*(cx*r32 + fx*r12) + Z*(cx*r33 + fx*r13))) / pow(t, 2) - (Y*cx) / t) - j331*((Z*(cx*t3 + fx*t1 + X*(cx*r31 + fx*r11) + Y*(cx*r32 + fx*r12) + Z*(cx*r33 + fx*r13))) / pow(t, 2) - (Z*cx) / t) - j311*((X*(cx*t3 + fx*t1 + X*(cx*r31 + fx*r11) + Y*(cx*r32 + fx*r12) + Z*(cx*r33 + fx*r13))) / pow(t, 2) - (X*cx) / t) + (Y*fx*j121) / t + (Z*fx*j131) / t;
                jac[idx * 12 + 1] = (X*fy*j211) / t - j321*((Y*(cy*t3 + fy*t2 + X*(cy*r31 + fy*r21) + Y*(cy*r32 + fy*r22) + Z*(cy*r33 + fy*r23))) / t / t - (Y*cy) / t) - j331*((Z*(cy*t3 + fy*t2 + X*(cy*r31 + fy*r21) + Y*(cy*r32 + fy*r22) + Z*(cy*r33 + fy*r23))) / t / t - (Z*cy) / t) - j311*((X*(cy*t3 + fy*t2 + X*(cy*r31 + fy*r21) + Y*(cy*r32 + fy*r22) + Z*(cy*r33 + fy*r23))) / t / t - (X*cy) / t) + (Y*fy*j221) / t + (Z*fy*j231) / t;
                jac[idx * 12 + 2] = (X*fx*j112) / t - j322*((Y*(cx*t3 + fx*t1 + X*(cx*r31 + fx*r11) + Y*(cx*r32 + fx*r12) + Z*(cx*r33 + fx*r13))) / t / t - (Y*cx) / t) - j332*((Z*(cx*t3 + fx*t1 + X*(cx*r31 + fx*r11) + Y*(cx*r32 + fx*r12) + Z*(cx*r33 + fx*r13))) / t / t - (Z*cx) / t) - j312*((X*(cx*t3 + fx*t1 + X*(cx*r31 + fx*r11) + Y*(cx*r32 + fx*r12) + Z*(cx*r33 + fx*r13))) / t / t - (X*cx) / t) + (Y*fx*j122) / t + (Z*fx*j132) / t;
                jac[idx * 12 + 3] = (X*fy*j212) / t - j322*((Y*(cy*t3 + fy*t2 + X*(cy*r31 + fy*r21) + Y*(cy*r32 + fy*r22) + Z*(cy*r33 + fy*r23))) / t / t - (Y*cy) / t) - j332*((Z*(cy*t3 + fy*t2 + X*(cy*r31 + fy*r21) + Y*(cy*r32 + fy*r22) + Z*(cy*r33 + fy*r23))) / t / t - (Z*cy) / t) - j312*((X*(cy*t3 + fy*t2 + X*(cy*r31 + fy*r21) + Y*(cy*r32 + fy*r22) + Z*(cy*r33 + fy*r23))) / t / t - (X*cy) / t) + (Y*fy*j222) / t + (Z*fy*j232) / t;
                jac[idx * 12 + 4] = (X*fx*j113) / t - j323*((Y*(cx*t3 + fx*t1 + X*(cx*r31 + fx*r11) + Y*(cx*r32 + fx*r12) + Z*(cx*r33 + fx*r13))) / t / t - (Y*cx) / t) - j333*((Z*(cx*t3 + fx*t1 + X*(cx*r31 + fx*r11) + Y*(cx*r32 + fx*r12) + Z*(cx*r33 + fx*r13))) / t / t - (Z*cx) / t) - j313*((X*(cx*t3 + fx*t1 + X*(cx*r31 + fx*r11) + Y*(cx*r32 + fx*r12) + Z*(cx*r33 + fx*r13))) / t / t - (X*cx) / t) + (Y*fx*j123) / t + (Z*fx*j133) / t;
                jac[idx * 12 + 5] = (X*fy*j213) / t - j323*((Y*(cy*t3 + fy*t2 + X*(cy*r31 + fy*r21) + Y*(cy*r32 + fy*r22) + Z*(cy*r33 + fy*r23))) / t / t - (Y*cy) / t) - j333*((Z*(cy*t3 + fy*t2 + X*(cy*r31 + fy*r21) + Y*(cy*r32 + fy*r22) + Z*(cy*r33 + fy*r23))) / t / t - (Z*cy) / t) - j313*((X*(cy*t3 + fy*t2 + X*(cy*r31 + fy*r21) + Y*(cy*r32 + fy*r22) + Z*(cy*r33 + fy*r23))) / t / t - (X*cy) / t) + (Y*fy*j223) / t + (Z*fy*j233) / t;
                jac[idx * 12 + 6] = fx / t;
                jac[idx * 12 + 7] = 0;
                jac[idx * 12 + 8] = 0;
                jac[idx * 12 + 9] = fy / t;
                jac[idx * 12 + 10] = cx / t - (cx*t3 + fx*t1 + X*(cx*r31 + fx*r11) + Y*(cx*r32 + fx*r12) + Z*(cx*r33 + fx*r13)) / t / t;
                jac[idx * 12 + 11] = cy / t - (cy*t3 + fy*t2 + X*(cy*r31 + fy*r21) + Y*(cy*r32 + fy*r22) + Z*(cy*r33 + fy*r23)) / t / t;
                
                
            }
        }
    }
}

void Solve3D::sba(Mat KMatrix, vector<Keyframe> &image, const vector<Marker> &pts3Ds,bool flag)
{
    vector<pair<int, int>> pts3D_Idx;
    int n = 0;
    //将所有的3D点排成一列
    for (size_t i = 0; i < pts3Ds.size(); i++)
    {
        if (pts3Ds[i].registed)
        {
            for (size_t j = 0; j < pts3Ds[i].points3D1.size(); j++)
            {
                if (pts3Ds[i].invertIdx[j].size() > 0)
                {
                    pts3D_Idx.push_back(pair<int, int>(i, j));
                }
            }
        }
    }
    n = pts3D_Idx.size();
    int ncon = n;
    int m = image.size();
    int mcon = 1;
    //处理vmask数组
    char * vmask = new char[n*m];
    memset(vmask, 0, sizeof(char)*m*n);
    int nonzero = 0;
    for (size_t i = 0; i < pts3D_Idx.size(); i++)
    {
        for (size_t j = 0; j < pts3Ds[pts3D_Idx[i].first].invertIdx[pts3D_Idx[i].second].size(); j++)
        {
            if (!vmask[i*m + pts3Ds[pts3D_Idx[i].first].invertIdx[pts3D_Idx[i].second][j].first])
            {
                vmask[i*m + pts3Ds[pts3D_Idx[i].first].invertIdx[pts3D_Idx[i].second][j].first] = 1;
                nonzero++;
            }
        }
    }
    
    int cnp = 6;
    int pnp = 3;
    //处理p
    double *p = new double[cnp*m];
    for (size_t i = 0; i < m; i++)
    {
        Mat rvec;
        Rodrigues(image[i].R, rvec);
        p[i*cnp] = rvec.at<float>(0, 0);
        p[i*cnp + 1] = rvec.at<float>(1, 0);
        p[i*cnp + 2] = rvec.at<float>(2, 0);
        p[i*cnp + 3] = image[i].t.at<float>(0, 0);
        p[i*cnp + 4] = image[i].t.at<float>(1, 0);
        p[i*cnp + 5] = image[i].t.at<float>(2, 0);
    }
    
    
    //处理x
    int mnp = 2;
    
    //	double *x = new double[mnp*nonzero];
    vector<double> x(mnp*nonzero);
    int z = 0;
    for (size_t i = 0; i < pts3D_Idx.size(); i++)
    {
        auto t = pts3D_Idx[i];
        int last = -1;
        for (size_t j = 0; j < pts3Ds[t.first].invertIdx[t.second].size(); j++)
        {
            auto u = pts3Ds[t.first].invertIdx[t.second][j];
            if (last == u.first)
                continue;
            CV_Assert(last < u.first);
            last = u.first;
            x[z*mnp] = image[u.first].keyPoints[u.second].pt.x;
            x[z*mnp + 1] = image[u.first].keyPoints[u.second].pt.y;
            CV_Assert(x[z*mnp]>0&&x[z*mnp]<4000);
            CV_Assert(x[z*mnp+1]>0&&x[z*mnp+1]<4000);
            z++;
        }
    }
    
    vector<double> k(pnp*n + 4);
    k[0] = KMatrix.at<float>(0, 0);
    k[1] = KMatrix.at<float>(1, 1);
    k[2] = KMatrix.at<float>(0, 2);
    k[3] = KMatrix.at<float>(1, 2);
    for (size_t i = 0; i < pts3D_Idx.size(); i++)
    {
        k[4 + pnp*i] = pts3Ds[pts3D_Idx[i].first].points3D1[pts3D_Idx[i].second].x;
        k[4 + pnp*i + 1] = pts3Ds[pts3D_Idx[i].first].points3D1[pts3D_Idx[i].second].y;
        k[4 + pnp*i + 2] = pts3Ds[pts3D_Idx[i].first].points3D1[pts3D_Idx[i].second].z;
    }
    double opts[SBA_OPTSSZ], info[SBA_INFOSZ];
    opts[0] = SBA_INIT_MU; opts[1] = SBA_STOP_THRESH; opts[2] = SBA_STOP_THRESH;
    opts[3] = SBA_STOP_THRESH;
    opts[4] = 0.0;
    if(flag)
    {
        cout<<sba_mot_levmar_x(n, m, mcon, vmask, p, cnp, &x[0], NULL, mnp, func, fjac, &k[0], 20, 2, opts, info)<<endl;
        for (size_t i = 0; i < m; i++)
        {
            Mat rvec(3,1,CV_32F);
            rvec.at<float>(0, 0) = p[i*cnp];
            rvec.at<float>(1, 0) = p[i*cnp+1];
            rvec.at<float>(2, 0) = p[i*cnp+2];
            Rodrigues(rvec, image[i].R);
            image[i].t.at<float>(0, 0) = p[i*cnp + 3];
            image[i].t.at<float>(1, 0) = p[i*cnp + 4];
            image[i].t.at<float>(2, 0) = p[i*cnp + 5];
        }
    }
    
    cout<<"#camera track project file"<<endl;
    cout<<"<Image Sequence>"<<endl;
    cout<<"Sequence:.\\0.jpg"<<endl;
    cout<<"start:0"<<endl;
    cout<<"step:1"<<endl;
    cout<<"end:"<<image.size()-1<<endl;
    cout<<"</Image Sequence>"<<endl;
    
     cout<<"<intrinsic parameter>"<<endl;
     cout<<KMatrix.at<float>(0,0)<<" "<<KMatrix.at<float>(1,1)<<" "<<KMatrix.at<float>(0,2)<<" "<<KMatrix.at<float>(1,2)<<" 0.0"<<" 1.0"<<endl;
     cout<<"</intrinsic parameter>"<<endl;
     cout<<"<Feature Tracks>"<<endl;
     cout<<n<<endl;
     for(auto i:pts3D_Idx)
     {
     cout<<pts3Ds[i.first].invertIdx[i.second].size()<<" 0 1 ";
     cout<<pts3Ds[i.first].points3D1[i.second].x<<" "<<pts3Ds[i.first].points3D1[i.second].y<<" "<<0<<endl;
     auto &t = pts3Ds[i.first].invertIdx[i.second];
     for(int j=0;j<t.size();j++)
     {
     if(!j)
     {
     cout<<t[j].first<<" "<<image[t[j].first].keyPoints[t[j].second].pt.x<<" "<<image[t[j].first].keyPoints[t[j].second].pt.y;
     }
     else
     {
     cout<<" "<<t[j].first<<" "<<image[t[j].first].keyPoints[t[j].second].pt.x<<" "<<image[t[j].first].keyPoints[t[j].second].pt.y;
     
     }
     }
     cout<<endl;
     }
     cout<<"</Feature Tracks>"<<endl;
     cout<<"<Camera Track>"<<endl;
    int j =0;
    for (auto i:image)
     {
        cout<<"<FRAME"<<j<<">"<<endl;
        cout<<KMatrix.at<float>(0,0)<<endl;
         for (int k=0; k<3; k++) {
             for (int v=0; v<3; v++) {
                 cout<<i.R.at<float>(k,v)<<" ";
                }
             cout<<i.t.at<float>(k,0)<<endl;
            }
         cout<<"0.0 0.0 0.0 1.0"<<endl;
         cout<<"</FRAME"<<j<<">"<<endl;
         j++;
     }
    cout<<"</Camera Track>"<<endl;
     
}


void Solve3D::Tlerp(const double &w1, const Mat &origin, const Mat &optimised, Mat &output)
{
    float* a = (float*)origin.data;
    float* b = (float*)optimised.data;
    output = Mat(3, 1, CV_32F);
    float *c = (float*)output.data;
    for (size_t i = 0; i < 3; i++)
    {
        c[i] = (1 - w1)*a[i] + b[i]*w1;
    }
}

void Solve3D::Slerp(const double &w1, const Scalar &q1, const Scalar &q2, Scalar &output)
{
    output[0] = q1.dot(q2);
    if (output[0] > 0.0)
    {
        if (output[0] > 1.0)
            output[0] = 0.0;
        //else if(output[0] < -1.0)
        //	output[0] = PI;
        else
            output[0] = acos(output[0]);
        if (fabs(output[0]) < DBL_EPSILON)
        {
            output = q1;
            return;
        }
        output[1] = 1 / sin(output[0]);
        const double s1 = sin(w1 * output[0]) * output[1];
        const double s2 = sin((1 - w1) * output[0]) * output[1];
        output[0] = s1 * q1[0] + s2 * q2[0];
        output[1] = s1 * q1[1] + s2 * q2[1];
        output[2] = s1 * q1[2] + s2 * q2[2];
        output[3] = s1 * q1[3] + s2 * q2[3];
    }
    else
    {
        output[0] = -output[0];
        if (output[0] > 1.0)
            output[0] = 0.0;
        //else if(output[0] < -1.0)
        //	output[0] = PI;
        else
            output[0] = acos(output[0]);
        if (fabs(output[0]) < DBL_EPSILON)
        {
            output = q1;
            return;
        }
        output[1] = 1 / sin(output[0]);
        const double s1 = sin(w1 * output[0]) * output[1];
        const double s2 = sin((1 - w1) * output[0]) * output[1];
        output[0] = s1 * q1[0] - s2 * q2[0];
        output[1] = s1 * q1[1] - s2 * q2[1];
        output[2] = s1 * q1[2] - s2 * q2[2];
        output[3] = s1 * q1[3] - s2 * q2[3];
    }
}


void Solve3D::ToRotationMatrix(const Scalar q, Mat &R)
{
    R = Mat(3, 3, CV_32F);
    const double q00 = q[0] * q[0], q01 = q[0] * q[1], q02 = q[0] * q[2], q03 = q[0] * q[3];
    const double q11 = q[1] * q[1], q12 = q[1] * q[2], q13 = q[1] * q[3];
    const double q22 = q[2] * q[2], q23 = q[2] * q[3];
    float *p = (float*)R.data;
    p[0] = q11 + q22;				p[1] = q01 + q23;				p[2] = q02 - q13;
    p[3] = q01 - q23;				p[4] = q00 + q22;				p[5] = q12 + q03;
    p[6] = q02 + q13;			p[7] = q12 - q03;		p[8] = q00 + q11;
    p[0] = 1 - p[0] - p[0];	p[1] = p[1] + p[1];		p[2] = p[2] + p[2];
    p[3] = p[3] + p[3];		p[4] = 1 - p[4] - p[4];	p[5] = p[5] + p[5];
    p[6] = p[6] + p[6];		p[7] = p[7] + p[7];		p[8] = 1 - p[8] - p[8];
}

void Solve3D::FromRotationMatrix(const Mat &R, Scalar &q)
{
    
    float * p = (float*)R.data;
    q[3] = p[0] + p[4] + p[8];
    if (q[3] > p[0] && q[3] > p[4] && q[3] > p[8])
    {
        q[3] = sqrt(q[3] + 1) * 0.5;
        q[2] = 0.25 / q[3];
        q[0] = (p[5] - p[7]) * q[2];
        q[1] = (p[6] - p[2]) * q[2];
        q[2] = (p[1] - p[3]) * q[2];
    }
    else if (p[0] > p[4] && p[0] > p[8])
    {
        q[0] = sqrt(p[0] + p[0] - q[3] + 1) * 0.5;
        q[3] = 0.25 / q[0];
        q[1] = (p[1] + p[3]) * q[3];
        q[2] = (p[2] + p[6]) * q[3];
        q[3] = (p[5] - p[7]) * q[3];
    }
    else if (p[4] > p[8])
    {
        q[1] = sqrt(p[4] + p[4] - q[3] + 1) * 0.5;
        q[3] = 0.25 / q[1];
        q[0] = (p[1] + p[3]) * q[3];
        q[2] = (p[5] + p[7]) * q[3];
        q[3] = (p[6] - p[2]) * q[3];
    }
    else
    {
        q[2] = sqrt(p[8] + p[8] - q[3] + 1) * 0.5;
        q[3] = 0.25 / q[2];
        q[0] = (p[2] + p[6]) * q[3];
        q[1] = (p[5] + p[7]) * q[3];
        q[3] = (p[1] - p[3]) * q[3];
    }
}






