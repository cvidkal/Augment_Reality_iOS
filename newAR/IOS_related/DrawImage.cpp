//
//  DrawImage.cpp
//  newAR
//
//  Created by Shangjin Zhai on 14-11-22.
//  Copyright (c) 2014年 Shangjin Zhai. All rights reserved.
//

#include "DrawImage.h"

Mat background(Feature_Match &featurematch, Mat frame, vector<int> &markerNum,int &keyPointSize, double &detectfps, double &matchfps)
{
    vector<Mat> H;
    vector<Matches> matches;
    vector<int> matchPointSize;
    featurematch.matchNimages(frame, 4, matches, keyPointSize, matchPointSize, detectfps, matchfps);
    H.resize(4);
    vector<pair<int,int>> counts;
    int maxVal = -1;
    int maxIdx;
    for (int i = 0; i < matches.size(); i++)
    {
        Mat mask;
        int count = 0;
        if (matches[i].count >= 8)
        {
            H[i] = findHomography(matches[i].trainPts, matches[i].queryPts, CV_RANSAC, 3, mask);
            count = norm(mask, NORM_L1);
            if (maxVal == -1 || maxVal < count)
            {
                maxIdx = i;
                maxVal = count;
            }
            counts.push_back(pair<int,int>(matches[i].refIdx,count));
        }
    }
    sort(counts.begin(), counts.end(), [](pair<int,int>a,pair<int,int>b){return a.second>b.second;});
    markerNum.clear();
    for(auto i:counts)
    {
        if (i.second>20) {
            markerNum.push_back(i.first);
        }
    }
    Mat ret;
    if (maxVal == -1)
    {
        return ret;
    }
    warpPerspective(featurematch.getRefImages(matches[maxIdx].refIdx), ret, H[maxIdx], frame.size());
    return ret;
}

void drawFeature(int &keypointSize, Mat &matOutput, Feature_Match &featureMatch)
{
    std::vector<KeyPoint> keypointsScene;
//    featureMatch.getDetector()->detect(matOutput, keypointsScene);
    
    for(int i = 0; i < keypointsScene.size(); i++)
    {
        circle(matOutput, CvPoint(keypointsScene[i].pt), 0.75, Scalar(0, 0, 255), 4);
    }
    keypointSize = (int)keypointsScene.size();
}

bool drawMatcher(int &keypointSize, int &matchpointSize, Mat &matOutput, double &detectfps, double &matchfps, Feature_Match &featureMatch, int &markerIdx, future<Mat> &backThread, bool &isMatched, Feature_Track &featureTrack, bool &backgroundIsRun,Mat IMUR,Mat &CameraPose)
{
    Matches matches;
    static Mat ref;
    static double t_matchfps;
    static double t_detectfps;
    static int t_keypointSize;
    static TickMeter tm;
    static vector<int> markerNums;
    if(!backgroundIsRun)
    {
        tm.reset();
        tm.start();
        backThread = future<Mat>(async(std::launch::async,background,std::ref(featureMatch),matOutput.clone(), std::ref(markerNums),std::ref(t_keypointSize), std::ref(t_detectfps), std::ref(t_matchfps)));
        backgroundIsRun = YES;
    }
    else
    {
        if(!isMatched)
        {
            if(backThread.wait_for(chrono::seconds(0)) == future_status::ready)
            {
                detectfps = t_detectfps;
                matchfps = t_matchfps;
                cout<<"detectfps:"<<detectfps<<endl;
                cout<<"matchfps:"<<matchfps<<endl;
                keypointSize = t_keypointSize;
                ref = backThread.get();
                if(ref.rows <= 0)
                {
                    backgroundIsRun = NO;
                }
                else
                {
                    featureTrack.setKMatrix(480, 360, focus);
                    featureTrack.setRef(ref,markerNums[0],IMUR);
                    featureTrack.match(matOutput, matches,detectfps,matchfps);
                    if(matches.count < 20)
                        backgroundIsRun = NO;
                    else
                        isMatched = YES;
                }
            }
        }
        else
        {
            bool status = featureTrack.track(matOutput,CameraPose,IMUR,detectfps,matchfps,keypointSize);
            return status;
            
//            cout<<CameraPose<<endl;
//            featureTrack.match(matOutput, matches,detectfps,matchfps);
            if(!status)
            {
                isMatched = NO;
                backgroundIsRun = NO;
            }
            /*
            if(matches.count > 30)
            {
                for (int i = 0; i < matches.count; i++)
                {
                    line(matOutput, matches.trainPts[i], matches.queryPts[i], CV_RGB(255, 255, 2));
                }
            }
            */
        }

    }
    return false;
}