#include "hierarchical k-means.h"
#include <queue>


void hierarchical_kmeans::add(Mat dat)
{
	data.push_back(dat.clone());
}

int hierarchical_kmeans::cluster(vector<Mat> &centers)
{
	centers.clear();
	int rowsCount = 0;
	int dim = data[0].cols;
	for (int i = 0; i < data.size(); i++)
	{
		rowsCount += data[i].rows;
	}
	Mat mergeMat = Mat(rowsCount, dim, data[0].type());
	int start = 0;
	for (int i = 0; i < data.size();i++)
	{
		Mat subMat = mergeMat.rowRange(start, start + data[i].rows);
		data[i].copyTo(subMat);
		start += data[i].rows;
	}
	Mat bestLabels;
	Mat center;
	vector<int> labels(rowsCount);
	vector<Mat> subs;
	vector<vector<int>> index;
	bool depthFlag = false;//Shorten the depth if depth is too large.
	for (int i = 0; i < depth; i++)
	{
		int width = pow(branch, i);
		subs.clear();
		subs.resize(width);
		index.clear();
		index.resize(width); 
		bestLabels.release();
		center.release();
		for (int j = 0; j < rowsCount; j++)
		{
			subs[labels[j]].push_back(mergeMat.row(j));
			index[labels[j]].push_back(j);
		}
		for (int j = 0; j < width; j++)
		{
			if (subs[j].rows < branch)
			{
				depthFlag = true;
				bestLabels = Mat(subs[j].rows, 1, CV_32S);
				for (int i = 0; i < subs[j].rows; i++)
				{
					center.push_back(subs[j].row(i));
					bestLabels.at<int>(i, 0) = i;
				}
				for (int i = subs[j].rows; i < branch; i++)
				{
					center.push_back(subs[j].row(0));
				}
			}
			else
			{
				kmeans(subs[j], branch, bestLabels, TermCriteria(0, 0, 0), 3, KMEANS_PP_CENTERS, center);
			}
			centers.push_back(center.clone());
			for (int k = 0; k < bestLabels.rows; k++)
			{
				labels[index[j][k]] = labels[index[j][k]] * branch + bestLabels.at<int>(k, 0);
			}
		}
		if (depthFlag)
		{
			depth = i + 1;
			break;
		}
	}
	clear();
	return depth;
}

void hierarchical_kmeans::clear()
{
	data.clear();
}


hierarchical_kmeans::hierarchical_kmeans(int _depth, int _branch)
:depth(_depth), branch(_branch)
{
}

hierarchical_kmeans::~hierarchical_kmeans()
{
}