#include "vocabulary tree.h"





void VTreeMatcher::addDescriptor(Mat train)
{
	trainDesps.push_back(train.clone());
	hkmeans->add(train);
}

void VTreeMatcher::train()
{
	int imgCount = trainDesps.size();
	hkmeans->cluster(vocabulary);

	vector<int> words;
	vector<int> tmpidfs(clusterCount);
	for (int i = 0; i < imgCount; i++)
	{
		feature2word(trainDesps[i], words);
		Mat imgDesp(1, clusterCount, CV_32F, Scalar::all(0.0));
		float *a = (float*)imgDesp.data;
		for (auto j : words)
		{
			a[j]++;
		}
		for (int j = 0; j < clusterCount; j++)
		{
			if (a[j]>0.5)
			{
				tmpidfs[j]++;
			}
		}
		trainImgDesps.push_back(imgDesp.clone());
	}
	idfs.resize(clusterCount);
	for (int i = 0; i < clusterCount; i++)
	{
//		idfs[i] = log(imgCount / tmpidfs[i]);
		idfs[i] = 1;
	}
	for (int i = 0; i < imgCount; i++)
	{
		float *a = (float*)trainImgDesps[i].data;
		for (int j = 0; j < clusterCount; j++)
		{
			a[j] *= idfs[j];
		}
		trainImgDesps[i] /= trainDesps[i].rows;
	}

}

__inline double dist(Mat A, Mat B)
{
	int dim = A.cols;
	float *a = (float*)A.data;
	float *b = (float*)B.data;
	double ret = 0, tmp;
	for (int i = 0; i < dim; i++)
	{
		tmp = (a[i] - b[i])*256;
		ret += tmp*tmp;
	}
	return ret;
}

void VTreeMatcher::feature2word(Mat desp, vector<int> &words)
{
	words.clear();
	for (int k = 0; k < desp.rows; k++)
	{
		Mat feature = desp.row(k);
		int word = 0;
		for (int i = 0, now = 0; i < depth; i++)
		{
			int minj;
			double dmin = INFINITY;
			for (int j = 0; j < branch; j++)
			{
				double d;
				if ((d = dist(feature, vocabulary[now].row(j))) < dmin)
				{
					dmin = d;
					minj = j;
				}
			}
			word = word*branch + minj;
			now = now*branch + minj + 1;
		}
		words.push_back(word);
	}
}

Mat VTreeMatcher::computeImgDesp(Mat desp)
{
	vector<int> words;
	feature2word(desp, words);
	
	Mat ret(1, clusterCount, CV_32F,Scalar::all(0.0));
	float *a = (float*)ret.data;
	for (auto i:words)
	{
		a[i]++;
	}
	for (int i = 0; i < clusterCount; i++)
	{
		a[i] *= idfs[i];
	}
	ret /= desp.rows;
	return ret;
}

int VTreeMatcher::matchImgDescritpors(Mat imgDescriptor, int n, vector<int> &matches)
{
	matches.clear();
	vector<pair<float, int>> thetas;
	int trainSize = trainImgDesps.size();
	for (int i = 0; i < trainSize; i++)
	{
		Mat result = (imgDescriptor - trainImgDesps[i])*256;
		thetas.push_back(pair<float, int>(norm(result,NORM_L2SQR), i));
	}
	sort(thetas.begin(), thetas.end(), [](pair<float, int>a, pair<float, int>b){return a.first<b.first; });
	int max = n > thetas.size() ? thetas.size() : n;
	for (int i = 0; i < max; i++)
	{
		matches.push_back(thetas[i].second);
	}
	return matches[0];
}

int VTreeMatcher::match(Mat Descriptor, int n, vector<int> &matches)
{
	Mat imgDesp = computeImgDesp(Descriptor);
	return matchImgDescritpors(imgDesp, n, matches);
}

void VTreeMatcher::clear()
{
	trainDesps.clear();
	vocabulary.clear();
	trainImgDesps.clear();
	idfs.clear();
}

Mat VTreeMatcher::getTrainDescriptor(int n)
{
	return trainDesps[n];
}




VTreeMatcher::VTreeMatcher(int _depth, int _branch)
:depth(_depth), branch(_branch)
{
	hkmeans = new hierarchical_kmeans(depth, branch);
	clusterCount = pow(branch, depth);
}

VTreeMatcher::~VTreeMatcher()
{
}