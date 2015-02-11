#ifndef __VOCABULARY_TREE_H__
#define __VOCABULARY_TREE_H__

#include "headers.h"
#include "hierarchical k-means.h"

class VTreeMatcher
{
public:
	VTreeMatcher(int _depth,int _branch);
	~VTreeMatcher();
	void addDescriptor(Mat train);
	void train();
	void clear();
	int match(Mat Descriptor, int n, vector<int> &matches);
	Mat getTrainDescriptor(int n);
private:
	Mat computeImgDesp(Mat desp);
	void feature2word(Mat desp, vector<int> &words);
	int matchImgDescritpors(Mat imgDescriptor, int n, vector<int> &matches);
	Ptr<hierarchical_kmeans> hkmeans;
	vector<Mat> trainDesps;
	vector<Mat> vocabulary;
	vector<float> idfs;
	vector<Mat> trainImgDesps;
	int depth;
	int branch;
	int clusterCount;
};

















#endif