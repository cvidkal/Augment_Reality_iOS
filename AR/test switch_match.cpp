//#include "headers.h"
//#include "feature_match.h"
//
//
//vector<Mat> refs;
//vector<vector<KeyPoint>> refPoints;
//vector<Mat> refDescriptors;
//vector<KeyPoint> briskPoints;
//Mat briskDescriptor;
//
//int main()
//{
//	Ptr<FeatureDetector> detector = FeatureDetector::create("SURF");
//	Ptr<DescriptorExtractor> extractor = DescriptorExtractor::create("SURF");
//	Ptr<FeatureDetector> detect_brisk = create_detector("CV_BRISK");
//	Ptr<DescriptorExtractor> extract_brisk = create_descriptor("CV_BRISK");
//	Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
//	vector<KeyPoint> keypoints;
//	Mat descriptor;
//	VideoCapture vc(1);
//	Mat frame;
//	Mat grey;
//	vector<vector<DMatch>> matches;
//	vector<Point2f> pts1, pts2;
//	MatchList list;
//	char c;
//	bool status1 = false, status2 = false;
//
//	vc >> frame;
//	waitKey(1000);
//
//	while (c = waitKey(10))
//	{
//		vc >> frame;
//		if (c == 'q')
//		{
//			status2 = !status2;
//			status1 = false;
//		}
//		if (status2)
//		{
//			if (status1)
//			{
//				Matches a;
//				a = Run(frame, "SURF", ANN, false);
//			}
//			else
//			{
//				if (c == ' ')
//				{
//					Run(frame, "SURF", ANN, true);
//					status1 = 1;
//				}
//			}
//		}
//		else
//		{
//			if (status1)
//			{
//				Matches a;
//				a = Run(frame, "SURF", CASHASH, false);
//				cout << a.count << endl;
//			}
//			else
//			{
//				if (c == ' ')
//				{
//					Run(frame, "SURF", CASHASH, true);
//					status1 = true;
//				}
//			}
//		}
//		imshow("test", frame);
//	}
//
//
//
//}