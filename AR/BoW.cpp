#include "BoW.h"



BoWMatcher::BoWMatcher(int _clusterCount, string _feature) 
:clusterCount(_clusterCount), feature(_feature)
{
	trainer = new BOWKMeansTrainer(clusterCount);
	ann = new ANN_Matcher(_feature);
	
}

BoWMatcher::~BoWMatcher()
{
}


void BoWMatcher::addDescriptor(Mat &descriptor)
{
	trainer->add(descriptor);
	trainDescriptors.push_back(descriptor.clone());
}

int BoWMatcher::getDescriptorCount()
{
	return trainer->descripotorsCount();
}

void BoWMatcher::train()
{
	vocabulary = trainer->cluster();
	ann->clear();
	ann->addRef(vocabulary);
	//imgDescriptors.resize(trainDescriptors.size());
	//for (int i = 0; i < trainDescriptors.size(); i++)
	//{
	//	imgDescriptors[i] = computeImgDescriptor(trainDescriptors[i]);
	//}
	trainIdfs_imgDesp();

}

void BoWMatcher::clear()
{
	trainer->clear();
	trainDescriptors.clear();
	imgDescriptors.clear();
}

Mat BoWMatcher::computeImgDescriptor(Mat descritpor)
{
	Mat imgDescriptor(1, clusterCount, CV_32F, Scalar::all(0.0));
	vector<MatchList> matchList;
	matchList = ann->match(descritpor,false);
	float * a = (float*)imgDescriptor.data;
	for (auto i : matchList[0])
	{
		a[i.first]++;
	}
	for (int i = 0; i < clusterCount; i++)
	{
		a[i] *= idfs[i];
	}
	imgDescriptor /= descritpor.rows;
	return imgDescriptor;
}
int BoWMatcher::match(Mat Descriptor,int n,vector<int> &matches)
{

	Mat imgDescriptor = computeImgDescriptor(Descriptor);	
	return matchImgDescritpors(imgDescriptor,n,matches);

}

int BoWMatcher::matchImgDescritpors(Mat imgDescriptor,int n,vector<int> &matches)
{
	matches.clear();
	vector<pair<float, int>> thetas;
	int trainSize = imgDescriptors.size();
	//float theta;
	//float a = norm(imgDescriptor);
	//float minTheta = 1000;
	for (int i = 0; i < trainSize; i++)
	{
		//float b = norm(imgDescriptors[i]);
		//Mat result = imgDescriptor*imgDescriptors[i].t();
		//float n = *(float*)result.data/(a*b);
		//theta = fabs(acos(n));
		//thetas.push_back(pair<float, int>(theta, i));
		Mat result = imgDescriptor - imgDescriptors[i];
		thetas.push_back(pair<float, int>(norm(result), i));
	}
	sort(thetas.begin(), thetas.end(), [](pair<float, int>a, pair<float, int>b){return a.first<b.first; });
	int max = n > thetas.size() ? thetas.size() : n;
	for (int i = 0; i < max; i++)
	{
		matches.push_back(thetas[i].second);
	}

	return matches[0];
}

Mat BoWMatcher::getTrainDescriptor(int n)
{
	return trainDescriptors[n];
}
Mat BoWMatcher::getImgDescriptor(int n)
{
	return imgDescriptors[n];
}


void BoWMatcher::trainIdfs_imgDesp()
{
	int imgCount = trainDescriptors.size();
	vector<vector<int>> tfs(imgCount);
	vector<int> temp_idf(clusterCount);
	for (auto &i : tfs)
	{
		i.resize(clusterCount);
	}
	for (int i = 0; i < imgCount; i++)
	{
		vector<MatchList> matchList;
		matchList = ann->match(trainDescriptors[i],false);
		for (auto j : matchList[0])
		{
			tfs[i][j.first]++;
		}
	}
	for (int i = 0; i < imgCount; i++)
	{
		for (int j = 0; j < clusterCount; j++)
		{
			if (tfs[i][j])
				temp_idf[j]++;
		}
	}

	idfs.resize(clusterCount);
	for (int i = 0; i < clusterCount; i++)
	{
//		idfs[i] = log(imgCount / temp_idf[i]);
		idfs[i] = 1;
	}
	for (int i = 0; i < clusterCount; i++)
	{
		cout << idfs[i] << " ";
	}
	cout << endl;
	Mat imgDescriptor(1, clusterCount, CV_32F, Scalar::all(0.0));
	float * a = (float*)imgDescriptor.data;
	for (int i = 0; i < imgCount; i++)
	{
		for (int j = 0; j < clusterCount; j++)
		{
			a[j] = tfs[i][j] * idfs[j] / trainDescriptors[i].rows;
		}
		imgDescriptors.push_back(imgDescriptor.clone());
	}
}
