#ifndef __ANN_H__
#define __ANN_H__

#include "headers.h"


struct ANNparams
{
	ANNpointArray pa;//point array pointer
	ANNkd_tree *kd;//kd-tree pointer
	int npoints;//the number of points
	int d;//point dimension
};


class ANN_Matcher
{
public:
	ANN_Matcher(string featureType);
	~ANN_Matcher();
	void setRefs(vector<Mat> &descriptor);
	void addRef(Mat &descriptor);
	vector<MatchList> ANN_Matcher::match(Mat descriptor, bool distCheck);
	void clear();

private:
	vector<ANNparams> trees;
	string _featureType;
};


#endif

