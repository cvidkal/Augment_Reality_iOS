//#include <opencv2\features2d\features2d.hpp>
//#include <opencv2\core\core.hpp>
//#include <iostream>
//#include <opencv2\highgui\highgui.hpp>
//#include <fstream>
//#include <opencv2\nonfree\features2d.hpp>
//#include <opencv2\nonfree\nonfree.hpp>
//using namespace std;
//using namespace cv;
//
//ofstream out1("out1.txt");
//ofstream out2("out2.txt");
//
//int main()
//{
//	initModule_nonfree();
//	Mat image1 = imread("1.jpg");
//	Mat image2 = imread("8.jpg");
//	Ptr<FeatureDetector> det = FeatureDetector::create("SIFT");
//	Ptr<DescriptorExtractor> des = DescriptorExtractor::create("SIFT");
//	vector<KeyPoint> key;
//	Mat descriptor;
//	det->detect(image1, key);
//	des->compute(image1, key, descriptor);
//	out1 << descriptor.size().height << " " << descriptor.size().width << " ";
//	for (int i = 0; i < descriptor.size().height; i++)
//	{
//		for (int j = 0; j < descriptor.size().width; j++)
//		{
//			out1 << static_cast<int>( descriptor.at<float>(i, j));
//			out1 << " ";
//		}
//		out1 << endl;
//	}
//	
//	det->detect(image2, key);
//	des->compute(image2, key, descriptor);
//	out2 << descriptor.size().height << " " << descriptor.size().width << " ";
//	for (int i = 0; i < descriptor.size().height; i++)
//	{
//		for (int j = 0; j < descriptor.size().width; j++)
//		{
//			out2 << static_cast<int>(descriptor.at<float>(i, j));
//			out2 << " ";
//		}
//		out2 << endl;
//	}
//}