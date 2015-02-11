#ifndef __HIERARCHICAL_KMEANS_H__
#define __HIERARCHICAL_KMEANS_H__


#include "headers.h"
class hierarchical_kmeans
{
public:
	hierarchical_kmeans(int _depth,int _branch);
	~hierarchical_kmeans();
	void add(Mat dat);
	int cluster(vector<Mat> &centers);
	

private:
	void clear();
	vector<Mat> data;
	int depth;
	int branch;
};


#endif