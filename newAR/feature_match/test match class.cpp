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
//	Feature_Match matcher = Feature_Match("SURF",512);
//	Feature_Track tracker = Feature_Track("CV_BRISK");
//	vector<KeyPoint> keypoints;
//	Mat descriptor;
//	VideoCapture vc(0);
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
//			if (matched < 0)
//			{
//				if (c == ' ')
//				{
//
//					Matches a;
//					double t = clock();
//					int n = 4;
//					vector<Matches> matches;
//					vector<int> imageIdxs;
//					int e;
//					vector<int> b;
//					double c, d;
//					matcher.matchNimages(frame, n, matches,e,b,c,d);
//					cout << d << endl;
//
//					warpRefs.resize(matches.size());
//					for (int i = 0; i < matches.size(); i++)
//					{
//						Mat H;
//						if (matches[i].count >= 8)
//						{
//							H = findHomography(matches[i].trainPts, matches[i].queryPts, CV_RANSAC);
//							warpPerspective(matcher.getRefImages(matches[i].refIdx), warpRefs[i], H, frame.size());
//						}
//						else
//						{
//							warpRefs[i] = matcher.getRefImages(matches[i].refIdx);
//						}
//					}
//					vector<Matches> trackMatches;
//					trackMatches.resize(warpRefs.size());
//					int maxMatch = 0;
//					for (int i = 0; i < warpRefs.size(); i++)
//					{
//						tracker.match(warpRefs[i], frame, trackMatches[i]);
//						if (maxMatch < trackMatches[i].count)
//						{
//							matched = i;
//							maxMatch = trackMatches[i].count;
//						}
//					}
//					if (maxMatch < 50)
//					{
//						matched = -1;
//					}
//				}
//			}
//			else
//			{
//				Matches matches;
//				tracker.match(warpRefs[matched], frame, matches);
//				if (matches.count < 50)
//				{
//					matched = -1;
//					continue;
//				}
//				for (int i = 0; i < matches.count; i++)
//				{
//					line(frame, matches.trainPts[i], matches.queryPts[i], CV_RGB(255, 255, 2));
//				}
//			}
//
//		}
//		else
//		{
//			if (c == ' ')
//			{
//				refs.push_back(frame.clone());
//				matcher.addRef(frame.clone());
//			}
//			if (c == 'a')
//			{
//				matcher.train();
//				status1 = true;
//			}
//			char str[50];
//			sprintf(str, "reference frames: %d", refs.size());
//			putText(frame, str, Point(0, 30), FONT_HERSHEY_COMPLEX, 1, CV_RGB(255, 0, 0));
//		}
//
//		imshow("test", frame);
//	}
//
//
//
//}