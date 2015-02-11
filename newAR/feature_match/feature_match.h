#ifndef __FEATUREMATCH_H__
#define __FEATUREMATCH_H__

#include "headers.h"
#include <opencv2/imgproc/imgproc.hpp>
#include "vocabulary tree.h"



Ptr<FeatureDetector> create_detector(string feature);
Ptr<DescriptorExtractor> create_descriptor(string feature);


class Feature_Match
{
public:
    Feature_Match(){};
	Feature_Match(string featureType,int clusterCount);
	Feature_Match(string featureType, int depth, int branch);
	void setRefs(vector<Mat> &images);
	void addRef(Mat image);
	~Feature_Match();
	void train();
	

	void matchNimages(Mat image, int n,vector<Matches> &matches,int &keyPointSize, vector<int> &matchPointSize, double &detectfps, double &matchfps);
	vector<Matches> matchNimages(Mat image, int n)
	{
        vector<Matches> matches;
		int a;
		vector<int> b;
		double c, d;
		matchNimages(image, n, matches, a, b, c, d);
        return matches;
	}

	Ptr<FeatureDetector> getDetector(){ return detector;}
	Ptr<DescriptorExtractor> getExtractor(){ return extractor; }
	string getFeatureType(){return _featureType;}
	Ptr<vector<KeyPoint>> getKeypoints(int refnum);
	Mat getRefImages(int refnum){ return refImages[refnum]; }

private:

	void refs2json(ofstream &out);
	void cur2json(int n,vector<int> indices, vector<KeyPoint> points, vector<MatchList> &list,ofstream &out);
	Ptr<FeatureDetector> create_detector(string feature);
	Ptr<DescriptorExtractor> create_descriptor(string feature);

	string _featureType;
	int _clusterCount;
	int _method;


	vector<vector<KeyPoint>> refKeypoints; 
	vector<Mat> refImages;
    
	Ptr<FeatureDetector> detector;
	Ptr<DescriptorExtractor> extractor;
	Ptr<VTreeMatcher> vtMatcher;
	Ptr<DescriptorMatcher> flann;

	
};







#define ANN 0
#define CASHASH 1


#endif 