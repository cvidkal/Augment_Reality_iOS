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
//	Ptr<FeatureDetector> detector = create_detector("SIFT");
//	Ptr<DescriptorExtractor> extractor = create_descriptor("SIFT");
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
//		if (status1)
//		{
//			if (status2)
//			{
//				double t = clock();
//				cvtColor(frame, grey, CV_BGR2GRAY);
//				detect_brisk->detect(grey, keypoints);
//				extract_brisk->compute(grey, keypoints, descriptor);
//				matcher->knnMatch(descriptor, briskDescriptor, matches, 2);
//				pts1.clear();
//				pts2.clear();
//				for (int i = 0; i < matches.size(); i++)
//				{
//					if ((matches[i][0].distance < matches[i][1].distance*0.6) && matches[i][0].distance<90)
//					{
//						pts1.push_back(briskPoints[matches[i][0].trainIdx].pt);
//						pts2.push_back(keypoints[matches[i][0].queryIdx].pt);
//					}
//				}
//				Mat mask;
//				if (pts1.size() < 16)
//				{
//					status2 = false;
//					continue;
//				}
//				findHomography(pts1, pts2, CV_RANSAC, 3.0, mask);
//				int count = 0;
//				for (int i = 0; i < pts1.size(); i++)
//				{
//					if (mask.at<char>(i,0))
//					{
//						line(frame,pts1[i],pts2[i], CV_RGB(255, 255, 2), 1);
//						count++;
//					}
//				}
//				if (c == 27)
//				{
//					return 0;
//				}
//				double t2 = clock() - t;
//				double fps = 1 / (t2 / CLOCKS_PER_SEC);
//				char str[50];
//				sprintf(str, "BRISK matchs: %d fps:%f", count,fps);
//				putText(frame, str, Point(0, 30), FONT_HERSHEY_COMPLEX, 1, CV_RGB(255, 0, 0));
//			}
//			else
//			{
//				
//				int matchnum;
//				detector->detect(frame, keypoints);
//				extractor->compute(frame, keypoints, descriptor);
//				double t = clock();
//				matchnum = getmatch(refDescriptors, descriptor, list, false);
//				double t2 = (clock() - t) / refs.size();
//				double fps = 1/(t2 / CLOCKS_PER_SEC);
//				if (matchnum >= 0)
//				{
//					//using brisk to track
//					cvtColor(refs[matchnum], grey, CV_BGR2GRAY);
//					status2 = true;
//					detect_brisk->detect(grey, briskPoints);
//					extract_brisk->compute(grey, briskPoints, briskDescriptor);
//				}
//				char str[50];
//				sprintf(str, "SURF matchs: %d      fps:%f", list.size(),fps);
//				putText(frame, str, Point(0, 30), FONT_HERSHEY_COMPLEX, 1, CV_RGB(255, 0, 0));
//				
//			}
//
//		}
//		else
//		{
//			if (c == ' ')
//			{
//				keypoints.clear();
//				refs.push_back(frame);
//				detector->detect(frame, keypoints);
//				extractor->compute(frame, keypoints, descriptor);
//				refPoints.push_back(keypoints);
//				refDescriptors.push_back(descriptor);
//			}
//			if (c == 'a')
//			{
//				status1 = true;
//				getmatch(refDescriptors, descriptor, list, true);
//				
//			}
//			char str[50];
//			sprintf(str, "reference frames: %d",refs.size());
//			putText(frame, str, Point(0, 30), FONT_HERSHEY_COMPLEX, 1, CV_RGB(255, 0, 0));
//		}
//		imshow("show", frame);
//
//		
//
//	}
//		system("pause");
//
//}
//
// 



