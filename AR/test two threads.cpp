//#include "headers.h"
//#include "feature_match.h"
//#include "Feature_Track.h"
//#include <thread>
//#include <future>
//#include <chrono>
//
//
//Ptr<Feature_Match> matcher;
//Ptr<Feature_Track> tracker;
//Mat backgroud(Mat frame,int &keyPointSize,vector<int> &matchPointSize,double &defectfps,double &matchfps)
//{
//	vector<Mat> H;
//	vector<Matches> matches;
//	matches = matcher->matchNimages(frame, 4);
//	matcher->matchNimages(frame, 4, matches, keyPointSize, matchPointSize, defectfps, matchfps);
//	H.resize(matches.size());
//	int maxVal = -1;
//	int maxIdx;
//	for (int i = 0; i < matches.size(); i++)
//	{
//		Mat mask;
//		int count = 0;
//		if (matches[i].count >= 8)
//		{
//			H[i] = findHomography(matches[i].trainPts, matches[i].queryPts, CV_RANSAC, 3, mask);
//			count = norm(mask, NORM_L1);
//			if (maxVal == -1 || maxVal < count)
//			{
//				maxIdx = i;
//				maxVal = count;
//			}
//		}
//	}
//	Mat ret;
//	if (maxVal == -1)
//	{
//		return ret;
//	}
//	warpPerspective(matcher->getRefImages(matches[maxIdx].refIdx), ret, H[maxIdx], frame.size());
//	return ret;
//
//}
//
//int main()
//{
//	matcher = new Feature_Match("SURF", 10, 2);
//	tracker = new Feature_Track("CV_BRISK");
//	VideoCapture vc(0);
//	Mat frame;
//	int refsCount = 0;
//	bool trained = false;
//	future<Mat> backThread;
//	vc >> frame;
//	cvWaitKey(1000);
//	Mat ref;
//	int keyPointSize; vector<int> matchPointSize; double defectfps; double matchfps;
//	while (char c = waitKey(10))
//	{
//		vc >> frame;
//		static bool isRun = false;
//		static bool matched = false;
//		if (!trained)
//		{
//
//			if (c == ' ')
//			{
//				matcher->addRef(frame);
//				refsCount++;
//			}
//			if (c == 'a')
//			{
//				matcher->train();
//				trained = true;
//				cout << refsCount << endl;
//			}
//		}
//		else
//		{
//			if (!isRun)
//			{
//				backThread = future<Mat>(async(std::launch::async, backgroud, frame.clone(), std::ref(keyPointSize), std::ref(matchPointSize), std::ref(defectfps), std::ref(matchfps)));
//				isRun = true;
//			}
//			else
//			{
//				if (!matched)
//				{
//					if (backThread.wait_for(chrono::seconds(0)) == future_status::ready)
//					{
//						Matches matches;
//						ref = backThread.get();
//						if (ref.empty())
//						{
//							isRun = false;
//						}
//						else
//						{
//							tracker->setRef(ref);
//							tracker->match(frame, matches,defectfps,matchfps);
//							if (matches.count < 30)
//							{
//								isRun = false;
//							}
//							else
//							{
//								matched = true;
//							}
//						}
//					}
//				}
//				else
//				{
//					Matches matches;
//					TickMeter tm;
//					tm.start();
//					tracker->match(frame, matches, defectfps, matchfps);
//					tm.stop();
//					double matchfps = 1 / tm.getTimeSec();
//					if (matches.count < 30)
//					{
//						isRun = false;
//						matched = false;
//					}
//					else
//					{
//						for (int i = 0; i < matches.count; i++)
//						{
//							line(frame, matches.trainPts[i], matches.queryPts[i], CV_RGB(255, 255, 2));
//						}
//						char str[50];
//						sprintf(str, "BRISK matchs: %d fps:%f", matches.count, matchfps);
//						putText(frame, str, Point(0, 30), FONT_HERSHEY_COMPLEX, 1, CV_RGB(255, 0, 0));
//					}
//				}
//			}
//
//		}
//		imshow("image", frame);
//	}
//}