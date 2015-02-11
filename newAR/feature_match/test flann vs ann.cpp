//#include "headers.h"
//#include "feature_match.h"
//
//int main()
//{
//
//	Ptr<FeatureDetector> detector = create_detector("SURF");
//	Ptr<DescriptorExtractor> extractor = create_descriptor("SURF");
//	Mat a;
//	a = imread("1.jpg");
//	vector<Mat> b;
//
//	for (int i = 2; i <= 12; i++)
//	{
//		char s[20];
//		sprintf(s, "%d.jpg", i);
//		Mat c = imread(s);
//		b.push_back(c.clone());
//	}
//	vector<KeyPoint> k1;
//	vector<vector<KeyPoint>> k2;
//	k2.resize(11);
//	Mat d1;
//	vector<Mat> d2;
//	d2.resize(11);
//	Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("FlannBased");
//	TickMeter tm1, tm2;
//	detector->detect(a, k1);
//	extractor->compute(a, k1, d1);
//	for (int i = 0; i <= 10; i++)
//	{
//		detector->detect(b[i], k2[i]);
//		extractor->compute(b[i], k2[i], d2[i]);
//	}
//	vector<DMatch> matchList1;
//	ANN_Matcher ann("SURF");
//	ann.setRefs(d2);
//	tm2.start();
//	vector<MatchList> list = ann.match(d1, true);
//	tm2.stop();
//	int count = 0;
//	for (auto i : list)
//	{
//		count += i.size();
//	}
//	cout << "ann points:" << count <<"   "<< "fps:" << 1 / (tm2.getTimeSec() / 11) << endl;
//	matcher->add(d2);
//	matcher->train();
//	tm1.start();
//	matcher->match(d1, matchList1);
//	tm1.stop();
//	cout << "flann points:"<<matchList1.size()<<"  "<<"fps"<<1 / (tm1.getTimeSec() / 11)<<endl;
//	system("pause");
//}