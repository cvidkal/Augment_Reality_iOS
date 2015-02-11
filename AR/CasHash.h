#ifndef __CASHASH_H__
#define __CASHASH_H__

#include "headers.h"


class CasHash
{
public:
	CasHash(std::string featureType);
	~CasHash();
	void getNmatch(Mat descriptor2, int n, vector<MatchList> &lists);
	void setRefs(vector<Mat> refDescriptors);
	void addRef(Mat refDescriptor);
	void train();
	void clear();


private:

	void freeData(ImageData& imageData);
	void processData(ImageData& imageData, Mat descriptor, string feature);

	HashConvertor *stHashConvertor = new HashConvertor();
	CasHashMatcher *stCasHashMatcher = new CasHashMatcher();
	std::vector<ImageData> stImageDataList;
	int refsCount=0;
	MatchList maxlist;
	string _featureType;

};




#endif