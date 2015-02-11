#include "vocabulary tree.h"





void VTreeMatcher::addDescriptor(Mat train)
{
	trainDesps.push_back(train.clone());
	hkmeans->add(train);
}

void VTreeMatcher::train()
{
	int imgCount = trainDesps.size();
	depth = hkmeans->cluster(vocabulary);
	clusterCount = pow(branch, depth);
	bucketNums.resize(imgCount);
	for (auto &i : bucketNums)
	{
		i.resize(clusterCount);
	}
	vector<int> words;
	vector<int> tmpidfs(clusterCount);
	for (int i = 0; i < imgCount; i++)
	{
		feature2word(trainDesps[i], words);
		Mat imgDesp(1, clusterCount, CV_32F, Scalar::all(0.0));
		float *a = (float*)imgDesp.data;
		for (int j = 0; j < words.size(); j++)
		{
			bucketNums[i][words[j]].push_back(j);
			a[words[j]]++;
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

Mat VTreeMatcher::computeImgDesp(Mat desp,vector<vector<int>> &bucketNum)
{
	bucketNum.clear();
	bucketNum.resize(clusterCount);
	vector<int> words;
	feature2word(desp, words);

	Mat ret(1, clusterCount, CV_32F,Scalar::all(0.0));
	float *a = (float*)ret.data;
	for (int j = 0; j < words.size(); j++)
	{
		bucketNum[words[j]].push_back(j);
		a[words[j]]++;
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

void VTreeMatcher::match(Mat Descriptor, int n, vector<int> &candidates,vector<MatchList> &matchLists)
{
	vector<vector<int>> bucketNum;
	Mat imgDesp = computeImgDesp(Descriptor,bucketNum);
	int ret = matchImgDescritpors(imgDesp, n, candidates);
	computeMatchList(Descriptor, bucketNum, candidates, matchLists);
}

void VTreeMatcher::computeMatchList(Mat desp, vector<vector<int>> &bucketNum, vector<int> &candidates, vector<MatchList> &matchLists)
{
	matchLists.clear();
	matchLists.resize(candidates.size());
	for (int i = 0; i < candidates.size(); i++)
	{
		for (int j = 0; j < clusterCount; j++)
		{
			if (bucketNum[j].size() && bucketNums[candidates[i]][j].size())
			{
				int n = bucketNum[j].size(), m = bucketNums[candidates[i]][j].size();
				vector<double> dists;
				dists.resize(m);
				if (n <= m)
				{
					for (int k = 0; k < n; k++)
					{
						//find the top-2 candidates with minimal Euclidean Distance 
						double minVal1 = -1, minVal2 = -1;
						int minVal_Idx1, minVal_Idx2;
						for (int l = 0; l < m; l++)
						{
							Mat a = desp.row(bucketNum[j][k]);
							dists[l] = dist(desp.row(bucketNum[j][k]), trainDesps[candidates[i]].row(bucketNums[candidates[i]][j][l]));
							if (minVal2 == -1 || minVal2>dists[l])
							{
								minVal2 = dists[l];
								minVal_Idx2 = l;
							}
							if (minVal1 == -1 || minVal1 > minVal2)
							{
								swap(minVal_Idx1, minVal_Idx2);
								swap(minVal1, minVal2);
							}
						}
						//apply the threhold for the matching rejection
						if (minVal1 < minVal2*0.5 || minVal2 == -1)
						{
							matchLists[i].push_back(pair<int, int>(bucketNums[candidates[i]][j][minVal_Idx1],bucketNum[j][k]));
						}
					}
				}

				else
				{
					for (int l = 0; l < m; l++)
					{
						//find the top-2 candidates with minimal Euclidean Distance 
						double minVal1 = -1, minVal2 = -1;
						int minVal_Idx1, minVal_Idx2;
						for (int k = 0; k < n; k++)
						{
							Mat a = desp.row(bucketNum[j][k]);
							dists[l] = dist(desp.row(bucketNum[j][k]), trainDesps[candidates[i]].row(bucketNums[candidates[i]][j][l]));
							if (minVal2 == -1 || minVal2>dists[l])
							{
								minVal2 = dists[l];
								minVal_Idx2 = l;
							}
							if (minVal1 == -1 || minVal1 > minVal2)
							{
								swap(minVal_Idx1, minVal_Idx2);
								swap(minVal1, minVal2);
							}
						}
						//apply the threhold for the matching rejection
						if (minVal1 < minVal2*0.5 || minVal2 == -1)
						{
							matchLists[i].push_back(pair<int, int>(bucketNums[candidates[i]][j][l], bucketNum[j][minVal_Idx1]));
						}
					}

				}
			}
		}
	}
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