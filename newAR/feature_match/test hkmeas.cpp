//#include "headers.h"
//#include "hierarchical k-means.h"
//#include "feature_match.h"
//
//
//
//int main()
//{
//	VideoCapture vc(0);
//	Ptr<FeatureDetector> detector = create_detector("SURF");
//	Ptr<DescriptorExtractor> extractor = create_descriptor("SURF");
//	vector<KeyPoint> keypoints;
//	Mat desp;
//	hierarchical_kmeans cluster(2, 10);
//	char c;
//	Mat frame;
//	while (c = waitKey(10))
//	{
//		vc >> frame;
//		if (c == ' ')
//		{
//			detector->detect(frame, keypoints);
//			extractor->compute(frame, keypoints, desp);
//			cluster.add(desp.clone());
//		}
//		if (c == 'a')
//		{
//			vector<Mat> centers;
//			cluster.cluster(centers);
//		}
//		imshow("nl", frame);
//	}
//
//}