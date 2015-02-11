//#include <opencv2\features2d\features2d.hpp>
//#include <opencv2\core\core.hpp>
//#include <iostream>
//#include <opencv2\highgui\highgui.hpp>
//#include <opencv2\nonfree\features2d.hpp>
//#include <opencv2\nonfree\nonfree.hpp>
//#include <time.h>
//#include <ANN\ANN.h>
//
//using namespace cv;
//using namespace std;
//
//const char * detect[] = { "SIFT", "SURF", "FAST", "ORB", "BRISK" };
//const char* describe[] = { "SIFT", "SURF", "BRIEF", "ORB", "BRISK" };
//
//int main()
//{
//	char name[20];
//	cv::initModule_nonfree();
//
//	for (int j = 0; j < 5; j++)
//	{
//		cout << describe[j]<< ":" << endl;
//		Ptr<FeatureDetector> detector = FeatureDetector::create(detect[j]);
//		Ptr<DescriptorExtractor> descriptor = DescriptorExtractor::create(describe[j]);
//		double totoaldetect = 0;
//		double totoaldescriptor = 0;
//		for (int i = 1; i <= 20; i++)
//		{
//			sprintf(name, "%d.tiff", i);
//			Mat image = imread(name);
//
//			vector<KeyPoint> keypoints;
//			
//			double detecttime = clock();
//
//			detector->detect(image, keypoints);
//
//			detecttime = clock() - detecttime;
//			totoaldetect += detecttime;
//
//
//			Mat descriptors;
//			double descriptortime = clock();
//
//			descriptor->compute(image, keypoints, descriptors);
//			descriptortime = clock() - descriptortime;
//			totoaldescriptor += descriptortime;
//
//
//		}
//		cout << "detect time: " << 1/(totoaldetect / CLOCKS_PER_SEC / 20)<<"\t";
//		cout << "descriptor time: " << 1/(totoaldescriptor / CLOCKS_PER_SEC / 20);
//		cout << "total time" << 1 / ((totoaldetect + totoaldescriptor) / CLOCKS_PER_SEC / 20) << endl;
//		cout << endl;
//	}
//	system("pause");
//
//}