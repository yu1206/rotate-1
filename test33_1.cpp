#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream> 
#include <io.h>
#include <Windows.h>
#include "opencv\cv.h"
#include "opencv\highgui.h"
#include <direct.h>
using namespace cv;
using namespace std;

extern int readDir(char *dirName, vector<string> &filesName);
extern void readTxt(const char* anno_file, vector<string>& v_img_);
extern void coordinates(Point2d src, float angle, Point2d & dst);
#define M_PI 3.14159265358979323846

// 旋转中心，坐标为车牌中心，旋转区域车牌区域
void J_Rotate_src_33_1(Mat src, Point pt5, Point pt6, Point pt7, Point pt8, float angle, Mat dst,
	int & l1, int & t1, int & r1, int & b1)
{
	int x1 = min({ pt5.x, pt6.x, pt7.x, pt8.x });
	int y1 = min({ pt5.y, pt6.y, pt7.y, pt8.y });
	int x2 = max({ pt5.x, pt6.x, pt7.x, pt8.x });
	int y2 = max({ pt5.y, pt6.y, pt7.y, pt8.y });

	Point center;
	center.x = (x1 + x2) / 2; center.y = (y1 + y2) / 2;

	const double cosAnglePoint = cos(angle);
	const double sinAnglePoint = sin(angle);

	const double cosAnglePixel = cos(-angle);
	const double sinAnglePixel = sin(-angle);

	//原图像四个角的坐标变为以旋转中心的坐标系
	Point2d leftTop(pt5.x - center.x, -pt5.y + center.y); //(x1,y1)
	Point2d rightTop(pt6.x - center.x, -pt6.y + center.y); // (x2,y1)
	Point2d leftBottom(pt7.x - center.x, -pt7.y + center.y); //(x1,y2)
	Point2d rightBottom(pt8.x - center.x, -pt8.y + center.y); // (x2,y2)

	//以center为中心旋转后四个角的坐标
	Point2d transLeftTopPoint, transRightTopPoint, transLeftBottomPoint, transRightBottomPoint;
	coordinates(leftTop, angle, transLeftTopPoint);
	coordinates(rightTop, angle, transRightTopPoint);
	coordinates(leftBottom, angle, transLeftBottomPoint);
	coordinates(rightBottom, angle, transRightBottomPoint);


	//以center为中心旋转后像素坐标
	Point2d transLeftTopPixel, transRightTopPixel, transLeftBottomPixel, transRightBottomPixel;
	coordinates(leftTop, -angle, transLeftTopPixel);
	coordinates(rightTop, -angle, transRightTopPixel);
	coordinates(leftBottom, -angle, transLeftBottomPixel);
	coordinates(rightBottom, -angle, transRightBottomPixel);
	double left = min({ transLeftTopPixel.x, transRightTopPixel.x, transLeftBottomPixel.x, transRightBottomPixel.x });
	double right = max({ transLeftTopPixel.x, transRightTopPixel.x, transLeftBottomPixel.x, transRightBottomPixel.x });
	double top = min({ transLeftTopPixel.y, transRightTopPixel.y, transLeftBottomPixel.y, transRightBottomPixel.y });
	double down = max({ transLeftTopPixel.y, transRightTopPixel.y, transLeftBottomPixel.y, transRightBottomPixel.y });



	int width = static_cast<int>(abs(left - right) + 0.5);
	int height = static_cast<int>(abs(top - down) + 0.5);

	// 左上角为原点的坐标
	Point pt1, pt2;
	pt1.x = transLeftTopPoint.x + center.x, pt1.y = -transLeftTopPoint.y + center.y;
	pt2.x = transRightTopPoint.x + center.x, pt2.y = -transRightTopPoint.y + center.y;
	Point pt3, pt4;
	pt3.x = transLeftBottomPoint.x + center.x, pt3.y = -transLeftBottomPoint.y + center.y;
	pt4.x = transRightBottomPoint.x + center.x, pt4.y = -transRightBottomPoint.y + center.y;

	int left1 = min({ pt1.x, pt2.x, pt3.x, pt4.x });
	int right1 = max({ pt1.x, pt2.x, pt3.x, pt4.x });
	int top1 = min({ pt1.y, pt2.y, pt3.y, pt4.y });
	int down1 = max({ pt1.y, pt2.y, pt3.y, pt4.y });

	const double num1 = -abs(left) * cosAnglePixel - abs(top) * sinAnglePixel + center.x;
	const double num2 = abs(left) * sinAnglePixel - abs(top) * cosAnglePixel + center.y;

	Vec3b *p;
	for (int i = 0; i < height; i++)
	{
		p = dst.ptr<Vec3b>(i + top1);
		for (int j = 0; j < width; j++)
		{
			//坐标变换
			int x = static_cast<int>(j  * cosAnglePixel + i * sinAnglePixel + num1 + 0.5);
			int y = static_cast<int>(-j * sinAnglePixel + i * cosAnglePixel + num2 + 0.5);

			if (x >= 0 && y >= 0 && x < src.cols && y < src.rows)
				p[j + left1] = src.ptr<Vec3b>(y)[x];
		}
	}

	l1 = left1; t1 = top1; r1 = right1; b1 = down1;
}

// 旋转中心，坐标为车牌中心，旋转区域车牌区域
void J_Rotate_src_33_2_pixel(Mat src, Point pt5, Point pt6, Point pt7, Point pt8, float angle, Mat dst,
	int & l1, int & t1, int & r1, int & b1)
{
	int x1 = min({ pt5.x, pt6.x, pt7.x, pt8.x });
	int y1 = min({ pt5.y, pt6.y, pt7.y, pt8.y });
	int x2 = max({ pt5.x, pt6.x, pt7.x, pt8.x });
	int y2 = max({ pt5.y, pt6.y, pt7.y, pt8.y });

	Point center;
	center.x = (x1 + x2) / 2; center.y = (y1 + y2) / 2;
	const double cosAngle = cos(angle);
	const double sinAngle = sin(angle);

	//原图像四个角的坐标变为以旋转中心的坐标系
	Point2d leftTop(pt5.x - center.x, -pt5.y + center.y); //(x1,y1)
	Point2d rightTop(pt6.x - center.x, -pt6.y + center.y); // (x2,y1)
	Point2d leftBottom(pt7.x - center.x, -pt7.y + center.y); //(x1,y2)
	Point2d rightBottom(pt8.x - center.x, -pt8.y + center.y); // (x2,y2)

	//以center为中心旋转后四个角的坐标
	Point2d transLeftTop, transRightTop, transLeftBottom, transRightBottom;
	coordinates(leftTop, angle, transLeftTop);
	coordinates(rightTop, angle, transRightTop);
	coordinates(leftBottom, angle, transLeftBottom);
	coordinates(rightBottom, angle, transRightBottom);


	double left = min({ transLeftTop.x, transRightTop.x, transLeftBottom.x, transRightBottom.x });
	double right = max({ transLeftTop.x, transRightTop.x, transLeftBottom.x, transRightBottom.x });
	double top = min({ transLeftTop.y, transRightTop.y, transLeftBottom.y, transRightBottom.y });
	double down = max({ transLeftTop.y, transRightTop.y, transLeftBottom.y, transRightBottom.y });

	int width = static_cast<int>(abs(left - right) + 0.5);
	int height = static_cast<int>(abs(top - down) + 0.5);

	// 左上角为原点的坐标
	Point pt1, pt2;
	pt1.x = transLeftTop.x + center.x, pt1.y = -transLeftTop.y + center.y;
	pt2.x = transRightTop.x + center.x, pt2.y = -transRightTop.y + center.y;
	Point pt3, pt4;
	pt3.x = transLeftBottom.x + center.x, pt3.y = -transLeftBottom.y + center.y;
	pt4.x = transRightBottom.x + center.x, pt4.y = -transRightBottom.y + center.y;

	int left1 = min({ pt1.x, pt2.x, pt3.x, pt4.x });
	int right1 = max({ pt1.x, pt2.x, pt3.x, pt4.x });
	int top1 = min({ pt1.y, pt2.y, pt3.y, pt4.y });
	int down1 = max({ pt1.y, pt2.y, pt3.y, pt4.y });

	const double num1 = -abs(left) * cosAngle - abs(top) * sinAngle + center.x;
	const double num2 = abs(left) * sinAngle - abs(top) * cosAngle + center.y;

	Vec3b *p;
	for (int i = 0; i < height; i++)
	{
		p = dst.ptr<Vec3b>(i + top1);
		for (int j = 0; j < width; j++)
		{
			//坐标变换
			int x = static_cast<int>(j  * cosAngle + i * sinAngle + num1 + 0.5);
			int y = static_cast<int>(-j * sinAngle + i * cosAngle + num2 + 0.5);

			if (x >= 0 && y >= 0 && x < src.cols && y < src.rows)
				p[j + left1] = src.ptr<Vec3b>(y)[x];
		}
	}

	l1 = left1; t1 = top1; r1 = right1; b1 = down1;
}

// 旋转中心，坐标为车牌中心，旋转区域车牌区域
void J_Rotate_src_33_2_point(Mat src, Point pt5, Point pt6, Point pt7, Point pt8, float angle, Mat dst,
	int & l1, int & t1, int & r1, int & b1)
{
	int x1 = min({ pt5.x, pt6.x, pt7.x, pt8.x });
	int y1 = min({ pt5.y, pt6.y, pt7.y, pt8.y });
	int x2 = max({ pt5.x, pt6.x, pt7.x, pt8.x });
	int y2 = max({ pt5.y, pt6.y, pt7.y, pt8.y });

	Point center;
	center.x = (x1 + x2) / 2; center.y = (y1 + y2) / 2;
	const double cosAngle = cos(angle);
	const double sinAngle = sin(angle);

	//原图像四个角的坐标变为以旋转中心的坐标系
	Point2d leftTop(pt5.x - center.x, -pt5.y + center.y); //(x1,y1)
	Point2d rightTop(pt6.x - center.x, -pt6.y + center.y); // (x2,y1)
	Point2d leftBottom(pt7.x - center.x, -pt7.y + center.y); //(x1,y2)
	Point2d rightBottom(pt8.x - center.x, -pt8.y + center.y); // (x2,y2)

	//以center为中心旋转后四个角的坐标
	Point2d transLeftTop, transRightTop, transLeftBottom, transRightBottom;
	coordinates(leftTop, angle, transLeftTop);
	coordinates(rightTop, angle, transRightTop);
	coordinates(leftBottom, angle, transLeftBottom);
	coordinates(rightBottom, angle, transRightBottom);


	double left = min({ transLeftTop.x, transRightTop.x, transLeftBottom.x, transRightBottom.x });
	double right = max({ transLeftTop.x, transRightTop.x, transLeftBottom.x, transRightBottom.x });
	double top = min({ transLeftTop.y, transRightTop.y, transLeftBottom.y, transRightBottom.y });
	double down = max({ transLeftTop.y, transRightTop.y, transLeftBottom.y, transRightBottom.y });

	int width = static_cast<int>(abs(left - right) + 0.5);
	int height = static_cast<int>(abs(top - down) + 0.5);

	// 左上角为原点的坐标
	Point pt1, pt2;
	pt1.x = transLeftTop.x + center.x, pt1.y = -transLeftTop.y + center.y;
	pt2.x = transRightTop.x + center.x, pt2.y = -transRightTop.y + center.y;
	Point pt3, pt4;
	pt3.x = transLeftBottom.x + center.x, pt3.y = -transLeftBottom.y + center.y;
	pt4.x = transRightBottom.x + center.x, pt4.y = -transRightBottom.y + center.y;

	int left1 = min({ pt1.x, pt2.x, pt3.x, pt4.x });
	int right1 = max({ pt1.x, pt2.x, pt3.x, pt4.x });
	int top1 = min({ pt1.y, pt2.y, pt3.y, pt4.y });
	int down1 = max({ pt1.y, pt2.y, pt3.y, pt4.y });

	const double num1 = -abs(left) * cosAngle - abs(top) * sinAngle + center.x;
	const double num2 = abs(left) * sinAngle - abs(top) * cosAngle + center.y;

	Vec3b *p;
	for (int i = 0; i < height; i++)
	{
		p = dst.ptr<Vec3b>(i + top1);
		for (int j = 0; j < width; j++)
		{
			//坐标变换
			int x = static_cast<int>(j  * cosAngle + i * sinAngle + num1 + 0.5);
			int y = static_cast<int>(-j * sinAngle + i * cosAngle + num2 + 0.5);

			if (x >= 0 && y >= 0 && x < src.cols && y < src.rows)
				p[j + left1] = src.ptr<Vec3b>(y)[x];
		}
	}

	l1 = left1; t1 = top1; r1 = right1; b1 = down1;
}
void RotatePoint_33_1(Point pt1, float centerX, float centerY, float angle, Point & pt)
{
	int x =pt1.x+ centerX;
	int y = -pt1.y+centerY;

	float theta = angle * M_PI / 180;
	int rx = int( x * std::cos(theta) - y * std::sin(theta));
	int ry = int(  x * std::sin(theta) + y * std::cos(theta));
	pt.x = rx; pt.y = ry;
}
void DrawLine_33_1(cv::Mat img, Point pt1, Point pt2, Point pt3, Point pt4)
{
	int thick = 1;

	CvScalar green = CV_RGB(0, 255, 0);
	cv::line(img, pt1, pt2, green, thick);
	cv::line(img, pt2, pt4, green, thick);
	cv::line(img, pt4, pt3, green, thick);
	cv::line(img, pt3, pt1, green, thick);
}
void DrawLine_33_1(cv::Mat img, std::vector<cv::Point> pointList)
{
	int thick = 1;
	CvScalar cyan = CV_RGB(0, 255, 255);
	CvScalar blue = CV_RGB(0, 0, 255);
	cv::line(img, pointList[0], pointList[1], cyan, thick);
	cv::line(img, pointList[1], pointList[2], cyan, thick);
	cv::line(img, pointList[2], pointList[3], cyan, thick);
	cv::line(img, pointList[3], pointList[0], blue, thick);
}
void DrawFace_33_1(cv::Mat img, Point pt11, Point pt22, Point pt33, Point pt44, float angle)
{
	
	int x1 = min({ pt11.x, pt22.x, pt33.x, pt44.x });
	int y1 = min({ pt11.y, pt22.y, pt33.y, pt44.y });
	int x2 = max({ pt11.x, pt22.x, pt33.x, pt44.x });
	int y2 = max({ pt11.y, pt22.y, pt33.y, pt44.y });
	int centerX = (x1 + x2) / 2;
	int centerY = (y1 + y2) / 2;
	std::vector<cv::Point> pointList;
	Point pt1, pt2, pt3, pt4;
	RotatePoint_33_1(pt11, centerX, centerY, angle, pt1);
	RotatePoint_33_1(pt22, centerX, centerY, angle, pt2);
	RotatePoint_33_1(pt33, centerX, centerY, angle, pt3);
	RotatePoint_33_1(pt44, centerX, centerY, angle, pt4);
	pointList.push_back(pt1);
	pointList.push_back(pt2);
	pointList.push_back(pt3);
	pointList.push_back(pt4);
	DrawLine_33_1(img, pointList);
}
// 只旋转车牌区域，以车牌中心为旋转中心，坐标原点
int test33_1(int argc, char *argv[])
{




	string outputdraw = "I:/mtcnn-train/rotateResult/draw";
	mkdir(outputdraw.c_str());
	string inputtxt = "I:/mtcnn-train/rotateResult/9src/000_one.txt";

	vector<string> v_img_;
	readTxt((char*)inputtxt.c_str(), v_img_);
	srand((unsigned)time(NULL));
	for (int i = 0; i < v_img_.size(); i++)
	{
		string 	annotation = v_img_[i];
		cout << annotation << endl;
		int npos = annotation.find_last_of('/');
		int npos2 = annotation.find_last_of('.');
		string name1 = annotation.substr(npos + 1, npos2 - npos - 1);
		string im_path;
		istringstream is(annotation);
		int label; int x1, y1, x2, y2; 
		Point pt5, pt6, pt7, pt8;
		int degree;

		is >> im_path >> label >> x1 >> y1 >> x2 >> y2 >> pt5.x>>pt5.y>>pt6.x>>pt6.y>>pt7.x>>pt7.y
			>>pt8.x>>pt8.y>>degree;


		Mat img = imread(im_path.c_str());
		if (img.data == NULL)
		{

			printf("图像为空!\n");
			cout << im_path.c_str() << endl;
			system("pause");
		}




		Mat dst = img.clone();
		Point pt1, pt2, pt3, pt4;
		pt1.x = x1; pt1.y = y1; pt2.x = x2; pt2.y = y2;
		int left = 0; int top = 0; int right = 0; int bottom = 0;

		/*Mat drawimg2 = dst.clone();

		DrawFace_33_1(drawimg2, pt1, pt2,pt3,pt4, -degree);
		string str3 = outputdraw + "/" + name1 + "-b.jpg";
		imwrite(str3.c_str(), drawimg2);*/


		Mat drawimg3 = dst.clone();
		rectangle(drawimg3, pt1, pt2, Scalar(0, 0, 255));
		DrawLine_33_1(drawimg3, pt5, pt6, pt7, pt8);
		string str4 = outputdraw + "/" + name1 + "-a.jpg";
		imwrite(str4.c_str(), drawimg3);

		
		double radian = -M_PI*degree*1.0 / 180;
		int left_temp = 0; int top_temp = 0; int right_temp = 0; int bottom_temp = 0;
		J_Rotate_src_33_2_pixel(img, pt5, pt6, pt7, pt8, radian, dst, left_temp, top_temp, right_temp, bottom_temp);
		Mat dst_temp=img.clone();
		double radian_temp = M_PI*degree*1.0 / 180;
		J_Rotate_src_33_2_point(img, pt5, pt6, pt7, pt8, radian_temp, dst_temp, left, top, right, bottom);

		pt3.x = left; pt3.y = top; pt4.x = right; pt4.y = bottom;

		Mat drawimg = dst.clone();
		rectangle(drawimg, pt3, pt4, Scalar(0, 0, 255));

		string str2 = outputdraw + "/" + name1 + "-c.jpg";
		imwrite(str2.c_str(), drawimg);



		int jjjjjj = 90;

	}


	return 0;
}