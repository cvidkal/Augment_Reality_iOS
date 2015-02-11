#ifndef __SOLVE3D_H
#define __SOLVE3D_H
#include "headers.h"
#include "keyframe.h"

static class Solve3D
{
public:
	Solve3D();
	~Solve3D();
	void setKMatrix( double imageWidth, double imageHeight,double focus);//init the K
	//    void setRotationMatrix(const CMRotationMatrix &rotation);//init the R
	void InitRotationMatrix();
	void setTVector(const Mat& _TVector);
	void InitTVector();
    static void get3DPoints(Mat KMatrix, Mat rotationMatrix, vector<KeyPoint> &pts2D, vector<Point3f> &pts3D, Mat &TVector);
	void getinvertMatrix();//get (KR)^-1
	void getNextInvertMatrix();//get next (KR)^-1
	Point3d getNext3DPoint(int i);
	static void Solve3D::getCameraRT(Mat KMatrix, Mat P3D, Mat P2D, Mat &R, Mat &t);
		void printCameraInfo();//print R and T
	void printPoints();
	void detectFeature(const Mat& imageObject, const Ptr<FeatureDetector>& detector, const Ptr<cv::DescriptorExtractor> &extractor);
	//void Match(const Marker &refsMarker, int &keyPointSize, Mat &drawMatch);
	void getOtherPoints();
	Mat_<float> TVector;
	void getRotation(Mat &getRotation);
	void get2DPoint(Mat &drawMatch);
	void getP(Mat &PP);
	void getLocalPoints(vector<Point2d> &LocalPoints);
	void get3DPoints(vector<Point3d> &realPoints);
	void printPlane(Mat &drawPlane, double lowX, double lowY, double highX, double highY);
	void getHighAndLow(double &lowX, double &lowY, double &highX, double &highY);
private:
	Mat descriptors;
	Mat P;
	Mat rrvec;
	vector<KeyPoint> keyPoints;//keypoint of Marker
	vector<Point3d> points3DVector;//real points
	vector<Point2d> points2DVector;//image points
	vector<Point3d> all3DPoints;
	vector<Point2d> all2DPoints;
	Mat rotationMatrix;//R
	Mat invertMatrix;//(KR)^-1
	Mat distCoeffs;//distortion coefficient
	Mat RVector;
	Mat KMatrix;//K
	Mat converseMatrix;
	Point2d offset;//offset of x and y
	vector<int> pointsIndex;//the index of SBA
	int markerIndex;

};

Solve3D::Solve3D()
{
}

Solve3D::~Solve3D()
{
}











#endif