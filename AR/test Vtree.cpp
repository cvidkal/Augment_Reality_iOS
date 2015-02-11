//#include "headers.h"
//#include "feature_match.h"
//#include "Feature_Track.h"
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
//	Feature_Match matcher = Feature_Match("SURF",12,2);
//	Feature_Track tracker = Feature_Track("CV_BRISK");
//	vector<KeyPoint> keypoints;
//	Mat descriptor;
//	VideoCapture vc(1);
//	Mat frame;
//	Mat grey;
//	vector<vector<DMatch>> matches;
//	vector<Point2f> pts1, pts2;
//	MatchList list;
//	vector<Mat> refs;
//	char c;
//	bool status1 = false, status2 = false;
//
//	vc >> frame;
//	waitKey(1000);
//
//	int keyPoints;
//	int matchPoints;
//	double detectfps, matchfps;
//	while (c = waitKey(10))
//	{
//		vc >> frame;
//		
//		if (status1)
//		{
//			static int matched = -1;
//			static vector<Mat> warpRefs;
//
//			int q;
//			cin >> q;
//			char s[20];
//			sprintf(s, "%d.jpg", q);
//			frame = imread(s);
//			Matches a;
//			double t = clock();
//			int n = 4;
//			vector<Matches> matches;
//			vector<int> imageIdxs;
//			int e;
//			vector<int> b;
//			double c, d;
//			matcher.matchNimages(frame, n, matches, e, b, c, d);
//			cout << "detect fps:"<<c<<"  match fps:"<<d << endl;
//			for (int i = 0; i < n; i++)
//			{
//				cout << matches[i].refIdx << " ";
//			}
//			vector<int> count(n);
//			int maxVal = -1;
//			int maxIdx;
//			for (int i = 0; i < matches.size(); i++)
//			{
//				Mat mask;
//				if (matches[i].count >= 8)
//				{
//					findHomography(matches[i].trainPts, matches[i].queryPts, CV_RANSAC, 3, mask);
//					count[i] = norm(mask, NORM_L1);
//					if (maxVal == -1 || maxVal < count[i])
//					{
//						maxIdx = i;
//						maxVal = count[i];
//					}
//				}
//			}
//			cout << count[maxIdx] << "/" << matches[maxIdx].count << endl;
//			//warpRefs.resize(matches.size());
//			//for (int i = 0; i < matches.size(); i++)
//			//{
//			//	Mat H;
//			//	if (matches[i].count >= 8)
//			//	{
//			//		H = findHomography(matches[i].trainPts, matches[i].queryPts, CV_RANSAC);
//			//		warpPerspective(matcher.getRefImages(matches[i].refIdx), warpRefs[i], H, frame.size());
//			//	}
//			//	else
//			//	{
//			//		warpRefs[i] = matcher.getRefImages(matches[i].refIdx);
//			//	}
//			//}
//			//vector<Matches> trackMatches;
//			//trackMatches.resize(warpRefs.size());
//			//int maxMatch = 0;
//			//for (int i = 0; i < warpRefs.size(); i++)
//			//{
//			//	tracker.match(warpRefs[i], frame, trackMatches[i]);
//			//	if (maxMatch < trackMatches[i].count)
//			//	{
//			//		matched = i;
//			//		maxMatch = trackMatches[i].count;
//			//	}
//			//}
//			//if (maxMatch < 50)
//			//{
//			//	matched = -1;
//			//}
//			//cout << endl;
//			//cout << "maxMatch:" << maxMatch << endl;
//
//
//			//else
//			//{
//			//	Matches matches;
//			//	tracker.match(warpRefs[matched], frame, matches);
//			//	if (matches.count < 50)
//			//	{
//			//		matched = -1;
//			//		continue;
//			//	}
//			//	for (int i = 0; i < matches.count; i++)
//			//	{
//			//		line(frame, matches.trainPts[i], matches.queryPts[i], CV_RGB(255, 255, 2));
//			//	}
//			//}
//
//		}
//		else
//		{
//			for (int i = 1; i <= 10; i++)
//			{
//				char s[20];
//				sprintf(s, "%d.jpg", i);
//				Mat a = imread(s);
//				refs.push_back(a.clone());
//				matcher.addRef(a);
//			}
//			TickMeter tm;
//			tm.start();
//			matcher.train();
//			tm.stop();
//			cout << "train time:" << tm.getTimeSec() << endl;
//			status1 = true;
//		}
//
////		imshow("test", frame);
//	}
//
//
//
//}