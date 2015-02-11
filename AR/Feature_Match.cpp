#include "feature_match.h"
#include "CasHash.h"


Ptr<FeatureDetector> create_detector(string feature)
{
	Ptr<FeatureDetector> ret;
	if (feature == "SIFT")
	{
		int nFeature = 500;
		int noctaves = 3;
		double contrastThreshold = 0.04;
		double edgeThreshold = 10;
		double sigma = 1.6;
		ret = new SiftFeatureDetector(nFeature, noctaves,contrastThreshold,edgeThreshold,sigma);
	}
	else if (feature == "SURF")
	{
		double hessianThreshold = 500;
		int nOctaves = 4;
		int nOctavesLayers = 2;
		ret = new SurfFeatureDetector(hessianThreshold, nOctaves, nOctavesLayers);
	}
	else if (feature == "BRIEF")
	{
		int threshold = 40;
		ret = new FastFeatureDetector(threshold);
	}
	else if (feature == "ORB")
	{
		int nFeatures = 500;
		float scaleFactor = 1.2f;
		int nlevels = 8;
		int edgeThreshold = 31;
		ret = new OrbFeatureDetector(nFeatures, scaleFactor, nlevels, edgeThreshold);
		
	}
	else if (feature == "BRISK")
	{
		int thresh = 60;
		int octaves = 4;
		ret = new BriskFeatureDetector(thresh, octaves);
	}
	else if (feature == "CV_BRISK")
	{
		int nFeatures = 500;
		float scaleFactor = 1.2f;
		int nlevels = 8;
		int edgeThreshold = 31;
		ret = new OrbFeatureDetector(nFeatures, scaleFactor, nlevels, edgeThreshold);

	}
	return ret;
}

Ptr<DescriptorExtractor> create_descriptor(string feature)
{
	Ptr<DescriptorExtractor> ret;
	if (feature == "SIFT")
	{
		int nFeature = 500;
		int noctaves = 3;
		double contrastThreshold = 0.04;
		double edgeThreshold = 10;
		double sigma = 1.6;
		ret = new SiftDescriptorExtractor(nFeature, noctaves, contrastThreshold, edgeThreshold, sigma);
	}
	else if (feature == "SURF")
	{
		double hessianThreshold = 500;
		int nOctaves = 4;
		int nOctavesLayers = 2;
		ret = new SurfDescriptorExtractor(hessianThreshold, nOctaves, nOctavesLayers);
	}
	else if (feature == "BRIEF")
	{
		ret = new BriefDescriptorExtractor();
	}
	else if (feature == "ORB")
	{
		int nFeatures = 500;
		float scaleFactor = 1.2f;
		int nlevels = 8;
		int edgeThreshold = 31;
		ret = new OrbDescriptorExtractor(nFeatures, scaleFactor, nlevels, edgeThreshold);
	}
	else if (feature == "BRISK")
	{
		ret = new BriskDescriptorExtractor();
	}
	else if (feature == "CV_BRISK")
	{
		ret = DescriptorExtractor::create("BRISK");
	}
	return ret;

}



Feature_Match::Feature_Match(string featureType, int depth, int branch)
{
	detector = create_detector(featureType);
	extractor = create_descriptor(featureType);
	vtMatcher = new VTreeMatcher(depth, branch);
	flann = DescriptorMatcher::create("FlannBased");
}

Feature_Match::~Feature_Match()
{
}

void Feature_Match::setRefs(vector<Mat> &images)
{
	vector<KeyPoint> keypoints;
	MatchList list;
	Mat descriptor;
	refKeypoints.clear();
	vtMatcher->clear();

	for (auto &i : images)
	{
		refImages.push_back(i);
		detector->detect(i, keypoints);
		extractor->compute(i, keypoints, descriptor);
		refKeypoints.push_back(keypoints);
		vtMatcher->addDescriptor(descriptor);
	}
}

void Feature_Match::addRef(Mat image)
{
	vector<KeyPoint> keypoints;
	Mat descriptor;
	refImages.push_back(image);
	detector->detect(image, keypoints);
	extractor->compute(image, keypoints, descriptor);
	refKeypoints.push_back(keypoints);
	vtMatcher->addDescriptor(descriptor);
}

void Feature_Match::train()
{
	vtMatcher->train();
	ofstream out("refKeypoints.json");
	refs2json(out);
}

void Feature_Match::matchNimages(Mat image, int n, vector<Matches> &matches, int &keyPointSize, vector<int> &matchPointSize, double &detectfps, double &matchfps)
{
	matchPointSize.clear();
	matchPointSize.resize(n);
	matches.clear();
	matches.resize(n);
	Matches ret;
	vector<KeyPoint> keypoints;
	Mat descriptor;
	MatchList list;
	TickMeter tm;

	vector<Point2f> pts1;
	vector<Point2f> pts2;
	//detect the keypoints and compute the descriptor
	tm.start();
	detector->detect(image, keypoints);
	extractor->compute(image, keypoints, descriptor);
	tm.stop();
	detectfps = 1.0f / tm.getTimeSec();
	vector<int> matchIdxs;
	tm.reset();
	tm.start();
	vector<MatchList> matchLists;
	vtMatcher->match(descriptor, n, matchIdxs, matchLists);
	tm.stop();
	matchfps = tm.getTimeSec();
	matchfps = 1 / matchfps;

	vector<Mat> candidateDesps;

	for (auto i : matchIdxs)
	{
		candidateDesps.push_back(vtMatcher->getTrainDescriptor(i));
	}


	flann->clear();
	flann->add(candidateDesps);
	flann->train();
	vector<DMatch> matchList;
	tm.reset();
	tm.start();
	//	matchLists = annMatcher->match(descriptor, true);
	flann->match(descriptor, matchList);
	tm.stop();
	matchfps = tm.getTimeSec();
	matchfps = 1 / matchfps;
	for (int i = 0; i < n; i++)
	{
		matches[i].count = 0;
		matches[i].refIdx = matchIdxs[i];
	}
	for (auto i : matchList)
	{
		matches[i.imgIdx].count++;
		matches[i.imgIdx].queryPts.push_back(keypoints[i.queryIdx].pt);
		matches[i.imgIdx].trainPts.push_back(refKeypoints[matchIdxs[i.imgIdx]][i.trainIdx].pt);
	}
	//for (int i = 0; i < n; i++)
	//{
	//	matches[i].count = matchLists[i].size();
	//	matchPointSize[i] = matchLists[i].size();
	//	matches[i].refIdx = matchIdxs[i];
	//	for (int j = 0; j < matchLists[i].size(); j++)
	//	{
	//		matches[i].queryPts.push_back(keypoints[matchLists[i][j].second].pt);
	//		matches[i].trainPts.push_back(refKeypoints[matchIdxs[i]][matchLists[i][j].first].pt);
	//	}
	//}
	keyPointSize = keypoints.size();
	//	ofstream out("cur.json");
	//	cur2json(n, matchIdxs, keypoints, matchLists,out);
}

Ptr<FeatureDetector> Feature_Match::create_detector(string feature)
{
	Ptr<FeatureDetector> ret;
	if (feature == "SIFT")
	{
		int nFeature = 500;
		int noctaves = 3;
		double contrastThreshold = 0.04;
		double edgeThreshold = 10;
		double sigma = 1.6;
		ret = new SiftFeatureDetector(nFeature, noctaves, contrastThreshold, edgeThreshold, sigma);
	}
	else if (feature == "SURF")
	{
		double hessianThreshold = 1200;
		int nOctaves = 4;
		int nOctavesLayers = 2;
		ret = new SurfFeatureDetector(hessianThreshold, nOctaves, nOctavesLayers);
	}
	else if (feature == "BRIEF")
	{
		int threshold = 40;
		ret = new FastFeatureDetector(threshold);
	}
	else if (feature == "ORB")
	{
		int nFeatures = 500;
		float scaleFactor = 1.2f;
		int nlevels = 8;
		int edgeThreshold = 31;
		ret = new OrbFeatureDetector(nFeatures, scaleFactor, nlevels, edgeThreshold);

	}
	else if (feature == "BRISK")
	{
		int thresh = 60;
		int octaves = 4;
		ret = new BriskFeatureDetector(thresh, octaves);
	}
	else if (feature == "CV_BRISK")
	{
		int nFeatures = 500;
		float scaleFactor = 1.2f;
		int nlevels = 8;
		int edgeThreshold = 31;
		ret = new OrbFeatureDetector(nFeatures, scaleFactor, nlevels, edgeThreshold);
	}
	return ret;
}
Ptr<DescriptorExtractor> Feature_Match::create_descriptor(string feature)
{
	Ptr<DescriptorExtractor> ret;
	if (feature == "SIFT")
	{
		int nFeature = 500;
		int noctaves = 3;
		double contrastThreshold = 0.04;
		double edgeThreshold = 10;
		double sigma = 1.6;
		ret = new SiftDescriptorExtractor(nFeature, noctaves, contrastThreshold, edgeThreshold, sigma);
	}
	else if (feature == "SURF")
	{
		double hessianThreshold = 1200;
		int nOctaves = 4;
		int nOctavesLayers = 2;
		ret = new SurfDescriptorExtractor(hessianThreshold, nOctaves, nOctavesLayers);
	}
	else if (feature == "BRIEF")
	{
		ret = new BriefDescriptorExtractor();
	}
	else if (feature == "ORB")
	{
		int nFeatures = 500;
		float scaleFactor = 1.2f;
		int nlevels = 8;
		int edgeThreshold = 31;
		ret = new OrbDescriptorExtractor(nFeatures, scaleFactor, nlevels, edgeThreshold);
	}
	else if (feature == "BRISK")
	{
		ret = new BriskDescriptorExtractor();
	}
	else if (feature == "CV_BRISK")
	{
		ret = DescriptorExtractor::create("BRISK");
	}
	return ret;

}

Ptr<vector<KeyPoint>> Feature_Match::getKeypoints(int refIdx)
{
	return &refKeypoints[refIdx];
}

void Feature_Match::refs2json(ofstream &out)
{
	string a = "";
	a += "{\n";
	a += "\"keypoints\": [\n";
	int count = 0;
	for (int i = 0; i < refKeypoints.size(); i++)
	{
		if (!i)
			a += "[";
		else
			a += ",\n[";
		for (int j = 0; j < refKeypoints[i].size(); j++)
		{
			char b[100];
			if (!j)
				sprintf(b, "{\"x\":%.3f,\"y\":%.3f}\n", refKeypoints[i][j].pt.x, refKeypoints[i][j].pt.y);
			else
				sprintf(b, ",{\"x\":%.3f,\"y\":%.3f}\n", refKeypoints[i][j].pt.x, refKeypoints[i][j].pt.y);
			if (a.length() > 9000)
			{
				out << a;
				a = "";
			}
			a += string(b);
		}
		out << a<<"]";
		a = "";
	}
	out << "]\n}" << endl;
}

void Feature_Match::cur2json(int n, vector<int> indices, vector<KeyPoint> points, vector<MatchList> &list, ofstream &out)
{
	string a = "";
	a += "{\n\"matchedRefsIndices\": [";
	for (int i = 0; i < n; i++)
	{
		char b[10];
		if (!i)
		{
			sprintf(b, "%d", indices[i]);
		}
		else
		{
			sprintf(b, ",%d", indices[i]);
		}
		a += string(b);
	}
	a += "]\n";
	out << a;
	a = ",\"current keypoints\":[\n";
	for (int i = 0; i < points.size(); i++)
	{
		char b[100];
		if (!i)
			sprintf(b, "{\"x\":%.3f,\"y\":%.3f}\n", points[i].pt.x, points[i].pt.y);
		else
		{
			sprintf(b, ",{\"x\":%.3f,\"y\":%.3f}\n", points[i].pt.x, points[i].pt.y);
		}
		if (a.length() > 9000)
		{
			out << a;
			a = "";
		}
		a += string(b);
	}
	out << a << "]" << endl;
	a = ",\"match indices pair\":[\n";
	for (int j = 0; j < n; j++)
	{
		char c[20];
		//if (!j)
		//	sprintf(c, "\"list%d\":[", j);
		//else
		//{
		//	sprintf(c, ",\"list%d\":[", j);
		//}
		//a += string(c);
		if (!j)
			a += "[";
		else
		{
			a += ",[";
		}
		for (int i = 0; i < list[j].size(); i++)
		{
			char b[100];
			if (!i)
				sprintf(b, "{\"trainIdx\":%d,\"queryIdx\":%d}", list[j][i].first, list[j][i].second);
			else
				sprintf(b, ",\n{\"trainIdx\":%d,\"queryIdx\":%d}", list[j][i].first, list[j][i].second);
			a += string(b);
			if (a.size()>9000)
			{
				out << a;
				a = "";
			}
		}
		out <<a<< "]" << endl;
		a = " ";
	}
	out << "]\n}" << endl;
}



