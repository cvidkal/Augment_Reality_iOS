#include "ANN.h"


void ANN_Matcher::addRef(Mat &descriptor)
{
	ANNparams a;
	a.npoints = descriptor.size().height;
	a.d = descriptor.size().width;
	a.pa = annAllocPts(a.npoints, a.d);
	if (_featureType == "SIFT")
	{

		for (int i = 0; i < a.npoints; i++)
		{
			for (int j = 0; j < a.d; j++)
			{
				a.pa[i][j] = descriptor.at<float>(i, j);
			}
		}
	}
	else if (_featureType == "SURF")
	{
		for (int i = 0; i < a.npoints; i++)
		{
			for (int j = 0; j < a.d; j++)
			{
				a.pa[i][j] = descriptor.at<float>(i, j) * 256;
			}
		}
	}
	a.kd = new ANNkd_tree(a.pa, a.npoints, a.d);
	trees.push_back(a);
}

void ANN_Matcher::setRefs(vector<Mat> & descriptors)
{
	clear();
	for (auto i : descriptors)
	{
		addRef(i);
	}
}

void ANN_Matcher::clear()
{
	for (auto i : trees)
	{
		annDeallocPts(i.pa);
		i.kd->~ANNkd_tree();
	}
	trees.clear();
}

vector<MatchList> ANN_Matcher::match(Mat descriptor,bool distCheck)
{
	vector<MatchList> ret;
	ret.resize(trees.size());

	ANNidx nn_idx[2];
	ANNdist dd[2];
	int npoints = descriptor.size().height;
	int d = descriptor.size().width;

	for (int i = 0; i < npoints; i++)
	{
		ANNpoint a = new int[d];
		if (_featureType == "SIFT")
		{
			for (int j = 0; j < d; j++)
			{
				a[j] = descriptor.at<float>(i, j);
			}
		}
		else
		{
			for (int j = 0; j < d; j++)
			{
				a[j] = descriptor.at<float>(i, j) * 256;
			}
		}
		//ANN search for the 2 closest points
		for (int j = 0; j < trees.size(); j++)
		{
			trees[j].kd->annkSearch(a, 2, nn_idx, dd);
			if (distCheck)
			{
		//		make sure if it is a good feature			
				if (dd[0] < 0.7*dd[1])
				{
					ret[j].push_back(pair<int, int>(nn_idx[0], i));
				}
			}
			else
			{
				ret[j].push_back(pair<int, int>(nn_idx[0], i));
			}
				
		}
		delete[] a;
	}
	return ret;
}



ANN_Matcher::ANN_Matcher(string featureType) :_featureType(featureType)
{}

ANN_Matcher::~ANN_Matcher()
{
}