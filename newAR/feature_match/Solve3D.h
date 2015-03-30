#ifndef __SOLVE3D_H
#define __SOLVE3D_H
#include "headers.h"
#include "keyframe.h"
#include "Marker.h"
#include <CoreMotion/CoreMotion.h>

static class Solve3D
{
public:
	Solve3D();
	~Solve3D();
    static void setKMatrix(Mat &KMatrix,double imageWidth, double imageHeight, double focus);
    static void solveSRT(Mat &pts3d1, Mat &pts3d2, double &s, Mat &R, Mat &t);
    static void transformSRT(const vector<Point3f> &pts3d1,vector<Point3f> &pts3d2,double s,Mat R,Mat t);
    static void Calibrate_Points(Point2f pp,double focal,vector<Point2f> &origin,vector<Point2f> &out);
    static bool Solve5Point(vector<Point2f> &pts1, vector<Point2f> &pts2, int num_pts, Mat &E, Mat &P, std::vector<int> &ret_inliers);
    static Mat setRotationMatrix(const CMRotationMatrix &rotation);//init the R
    static Mat setRotationMatrix(double yaw,double pitch,double roll);
    void setTVector(const Mat& _TVector);
    static void sba(Mat KMatrix, vector<Keyframe> &image, const vector<Marker> &pts3Ds,bool flag = true);
    static void get3DPoints(Mat KMatrix, Mat rotationMatrix, vector<KeyPoint> &pts2D, vector<Point3f> &pts3D, Mat &TVector);
    static void get3DPoints(Mat KMatrix,const Mat rotationMatrix,const Mat TVector,const vector<KeyPoint> &pts2D,vector<Point3f> &pts3D);
	void getinvertMatrix();//get (KR)^-1
    static Mat getNextInvertMatrix(Mat KMatrix,Mat R,Mat t);
	Point3d getNext3DPoint(int i);
    static bool getCameraRT(Mat KMatrix,vector<Point3f> &P3D,vector<Point2f> &P2D,Mat &R,Mat &t,vector<int> &inliers,bool isTracked);
	void getOtherPoints();
	Mat_<float> TVector;
	void getRotation(Mat &getRotation);
    static void get2DPoint(Mat KMatrix,Mat R,Mat t,vector<Point3f> &points3DVector,vector<Point2f> &points2DVector,Mat &drawMatch);
	static Mat getP(Mat R,Mat t);
	void getLocalPoints(vector<Point2d> &LocalPoints);
	void get3DPoints(vector<Point3d> &realPoints);
    static void printPlane(Mat KMatrix,Mat R,Mat t,cv::Mat &drawPlane, double lowX, double lowY, double highX, double highY);
    static void getHighAndLow(vector<Point3f> &points3DVector,double &lowX, double &lowY, double &highX, double &highY);
    static void Slerp(const double &w1, const Scalar &q1, const Scalar &q2, Scalar &output);
    static void ToRotationMatrix(const Scalar q, Mat &R);
    static void FromRotationMatrix(const Mat &R, Scalar &q);
    static void Tlerp(const double &w1, const Mat &origin, const Mat &optimised, Mat &output);
private:
	Mat descriptors;
	Mat P;
	Mat rrvec;
	vector<KeyPoint> keyPoints;//keypoint of Marker
	vector<Point3d> points3DVector;//real points
	vector<Point2d> points2DVector;//image points
	vector<Point3d> all3DPoints;
	vector<Point2d> all2DPoints;
//	Mat rotationMatrix;//R
	Mat invertMatrix;//(KR)^-1
	Mat distCoeffs;//distortion coefficient
	Mat RVector;
//	Mat KMatrix;//K
	Mat converseMatrix;
	Point2d offset;//offset of x and y
	vector<int> pointsIndex;//the index of SBA
	int markerIndex;

};




#endif