//#include "headers.h"
////#include "feature_match.h"
//
//
//
//int main()
//{
//	VideoCapture vc(0);
//	Mat frame;
//	vc >> frame;
//	cvWaitKey(1000);
//	vc >> frame;
//	int c = 0;
//	for (int i = 0; i < 100; i++)
//	{
//		vc >> frame;
//		if (i % 20 == 0)
//		{
//			char s[10];
//			sprintf(s, "eee%d.jpg", i / 20);
//			imwrite(s, frame);
//		}
//	}
//	system("pause");
//
//	
//}