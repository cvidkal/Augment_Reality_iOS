#ifndef __BOW_H__
#define __BOW_H__

#include "headers.h"
#include "ANN.h"
#include "CasHash.h"

class BoWMatcher
{
public:
	BoWMatcher(int _clusterCount,string _feature);
	void addDescriptor(Mat &descriptor);
	int getDescriptorCount();
	Mat getTrainDescriptor(int n);
	Mat getImgDescriptor(int n);
	void train();
	void clear();
	int match(Mat Descriptor,int n,vector<int> &matches);
	~BoWMatcher();

private:
	Mat computeImgDescriptor(Mat descriptor);
	void trainIdfs_imgDesp();
	int matchImgDescritpors(Mat imgDescriptor,int n,vector<int> &matches);
	vector<Mat> trainDescriptors;
	vector<Mat> imgDescriptors;
	Mat vocabulary;
	int clusterCount;
	vector<float> idfs;
	string feature;
	Ptr<BOWKMeansTrainer> trainer;
	Ptr<ANN_Matcher> ann;

};








#endif