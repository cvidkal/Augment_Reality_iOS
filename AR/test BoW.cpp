//#define _CRT_SECURE_NO_WARNINGS
//
//#include <iostream>
//#include "headers.h"
//#include "feature_match.h"
//#include <fstream>
//#include <opencv2\contrib\contrib.hpp>
//
//
//ifstream fin("in.txt");
//
//
//
//
//
//Vector<Mat> images;
//vector<Mat> descriptors;
//int imgMatch(vector<Mat> train, Mat query);
//
//
//void save(Mat i, ofstream &fout)
//{
//
//	int height = i.size().height;
//	int width = i.size().width;
//	fout << height << " " << width;
//	for (int j = 0; j < height; j++)
//	{
//		for (int k = 0; k < width; k++)
//		{
//			fout << " " << i.at<float>(j, k);
//		}
//	}
//	fout << endl;
//
//}
//
//void load(Mat a, ifstream &fin)
//{
//
//	int height, width;
//	fin >> height >> width;
//	a = Mat(height, width, CV_32F);
//	float b;
//	for (int j = 0; j < height; j++)
//	{
//		for (int k = 0; k < width; k++)
//		{
//			fin >> b;
//			a.at<float>(j, k) = b;
//		}
//	}
//
//}
//
////int main()
////{
////	Mat vocabulary;
////
////
////	int features = 0;
////	char filename[20];
////	Mat frame;
////	VideoCapture vc(1);
////	vc >> frame;
////	cvWaitKey(1999);
////	while (images.size()<=6)
////	{
////		vc >> frame;
////		char c = cvWaitKey(10);
////		imshow("ww", frame);
////		if (c == ' ')
////			images.push_back(frame.clone());
////	}
////
////	
////	Ptr<FeatureDetector> detector = create_detector("SURF");
////	Ptr<DescriptorExtractor> extractor = create_descriptor("SURF");
////	Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("FlannBased");
////	vector<vector<KeyPoint>> keypoints(images.size());
////
////	descriptors.resize(images.size());
////	vector<Mat> imgDescriptors(images.size());
////	BOWImgDescriptorExtractor imgExtractor(extractor, matcher);
////	for (int i = 0; i < images.size(); i++)
////	{
////		detector->detect(images[i], keypoints[i]);
////		if (keypoints.size()>2000)
////			keypoints.resize(2000);
////		extractor->compute(images[i], keypoints[i], descriptors[i]);
////
////	}
////
////		ofstream fout("in.txt");
////		BOWKMeansTrainer trainer = BOWKMeansTrainer(128);
////		for (auto i : descriptors)
////		{
////			trainer.add(i);
////		}
////		cout << trainer.descripotorsCount() << endl;
////		vocabulary = trainer.cluster();
////		save(vocabulary, fout);
////
////
////
////	imgExtractor.setVocabulary(vocabulary);
////	for (int i = 0; i < images.size(); i++)
////	{
////		imgExtractor.compute(images[i], keypoints[i], imgDescriptors[i]);
////	}
////	vector<KeyPoint> keypoint;
////	Mat imgDescriptor;
////	while (true)
////	{
////		TickMeter tm;
////		char c = cvWaitKey(10);
////		vc >> frame;
////		imshow("real", frame);
////		if (c == ' ')
////		{
////			detector->detect(frame, keypoint);
////			tm.start();
////			imgExtractor.compute(frame, keypoint, imgDescriptor);
////			int n = imgMatch(imgDescriptors, imgDescriptor);
////			tm.stop();
////			float a =tm.getTimeSec();
////			cout << 1 / a << endl;
////			imshow("origin", frame);
////			imshow("match", images[n]);
////			cout << n;
////			cvWaitKey(0);
////		}
////
////	}
////
////
////	system("pause");
////
////}
//
//int main()
//{
//	BoWMatcher bow(128, "SURF", 0);
//	Ptr<FeatureDetector> detector = create_detector("SURF");
//	Ptr<DescriptorExtractor> extractor = create_descriptor("SURF");
//	Mat frame;
//	VideoCapture vc(1);
//	vc >> frame;
//	cvWaitKey(1000);
//	vector<Mat> trainFrames;
//	vector<KeyPoint> keypoints;
//	Mat descriptor;
//	bool flag = false;
//	while (true)
//	{
//		char c = cvWaitKey(10);
//		vc >> frame;
//		if (!flag)
//		{
//			if (c == ' ')
//			{
//				trainFrames.push_back(frame.clone());
//				detector->detect(frame, keypoints);
//				extractor->compute(frame, keypoints, descriptor);
//				bow.addDescriptor(descriptor);
//			}
//			if (c == 'a')
//			{
//				bow.train();
//				flag = true;
//			}
//		}
//		else
//		{
//			TickMeter tm;
//			if (c == ' ')
//			{
//				detector->detect(frame, keypoints);
//				extractor->compute(frame, keypoints, descriptor);
//				tm.start();
//				vector<int> idxs;
//				int n = bow.match(descriptor,5,idxs);
//				for (int i = 0; i < idxs.size(); i++)
//				{
//					cout << idxs[i] << " ";
//				}
//				tm.stop();
//				float fps = 1.0f / tm.getTimeSec();
//				cout << fps << endl;
//				imshow("origin", frame);
//				imshow("match1", trainFrames[n]);
//				imshow("match2", trainFrames[idxs[1]]);
//				
//			}
//		}
//		imshow("real", frame);
//	}
//}
//
//
//int imgMatch(vector<Mat> train, Mat query)
//{
//	vector<float> thetas(train.size());
//	float a = norm(query);
//	float min = 1000;
//	int mini;
//	for (int i = 0; i < train.size(); i++)
//	{
//		float b = norm(train[i]);
//		Mat result = (query*train[i].t()) / (a*b);
//		float a = *(float*)result.data;
//		thetas[i] = fabs(acos(a));
//		if (thetas[i] < min)
//		{
//			min = thetas[i];
//			mini = i;
//		}
//	}
//	return mini;
//}
//
//
