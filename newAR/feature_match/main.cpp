//#include "headers.h"
//#include "feature_match.h"
//
//
//
//int main()
//{
////	VideoCapture vc(0);
//	Ptr<FeatureDetector> detector = create_detector("SURF");
//	Ptr<DescriptorExtractor> extractor = create_descriptor("SURF");
//	vector<KeyPoint> akeypoints,bkeypoints;
//	vector<DMatch> matches;
//	Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("FlannBased");
//	Mat a, b;
//	a = imread("a.bmp");
//	b = imread("b.bmp");
//	detector->detect(a, akeypoints);
//	Mat ka, kb;
//	drawKeypoints(a, akeypoints, ka);
//	imwrite("ka.bmp", ka);
//	detector->detect(b, bkeypoints);
//	drawKeypoints(b, bkeypoints, kb);
//	imwrite("kB.bmp", kb);
//	Mat da, db;
//	extractor->compute(a, akeypoints, da);
//	extractor->compute(b, bkeypoints, db);
//	matcher->match(da, db, matches);
//	Mat m;
//	drawMatches(a, akeypoints, b, bkeypoints, matches, m);
//	imwrite("m.bmp", m);
//	system("pause");
//
//	
//}