#include <opencv2/highgui.hpp>
#include <iostream>
#include <stdlib.h> 　　  
#include <stdio.h> 
#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>


using namespace std;
using namespace cv;

struct border_img
{
	int left;
	int right;
	int top;
	int bottom;
};

//函数功能：当出现上/下/左/右边界时，输出out为二进制转为十进制后数值(二进制从左至右顺序：左右上下)；当到达task2停止点时，为左上边界，输出15
border_img border_dectection_task2(Mat& Image)
{
	border_img border_re;

	int flag = 0;      //是否到边界
	int left = 0, right = 0, top = 0, bottom = 0;    //是否到左、右、上、下边
	int cnt = 0;       //判断边界时判断白色像素个数
	int out = 0;       //输出
	int right_area = 0, bottom_area = 0;  //左矩形、上矩形面积
	Point p_2;         //判断边界时的扫描点
	int bottom_left = 640, bottom_right = 0;
	int left_top = 480, left_bottom = 0;
	int cnt_line = 640;

					   //色彩分割
	for (int i = 0; i < Image.rows; i++)

	{
		for (int j = 0; j < Image.cols; j++)

		{
			Point p_1;
			p_1.x = j;
			p_1.y = i;
			/*cout << "p_1.x"<< p_1.x << endl;
			cout << "p_1.y" << p_1.y << endl;*/
			if ((Image.at<Vec3b>(p_1)[0] - Image.at<Vec3b>(p_1)[2]> 20) && (Image.at<Vec3b>(p_1)[0] - Image.at<Vec3b>(p_1)[2]<60))
			{

				Image.at<Vec3b>(p_1)[0] = 255;//b
				Image.at<Vec3b>(p_1)[1] = 255;//g
				Image.at<Vec3b>(p_1)[2] = 255;//r
				flag = 1;
			}
			else
			{
				Image.at<Vec3b>(p_1)[0] = 0;//b
				Image.at<Vec3b>(p_1)[1] = 0;//g
				Image.at<Vec3b>(p_1)[2] = 0;//r
			}

		}

	}
	//显示处理后图像
	//namedWindow("黑白图片", 0);
	//imshow("黑白图片", Image);
	//判断边界
	if (flag == 1)
	{
		//left
		left_top = 480; left_bottom = 0;
		cnt_line = 0;
		for (int j = 0; j < 50; j++)
		{
			for (int i = 0; i < Image.rows; i++)
			{

				p_2.x = j;
				p_2.y = i;
				if ((Image.at<Vec3b>(p_2)[0] == 255) && (Image.at<Vec3b>(p_2)[1] == 255) && (Image.at<Vec3b>(p_2)[2] == 255))
				{
					if (i < left_top)
						left_top = i;
					if (i > left_bottom)
						left_bottom = i;
					if (j > cnt_line)
						cnt_line = j;
				}
			}
		}
		if (left_bottom - left_top >= Image.rows*0.8 && cnt_line > 10)
		{
			left = 1;

		}
		else
		{
			left = 0;
		}

		//right
		left_top = 480; left_bottom = 0;
		cnt_line = 640;
		for (int j = Image.cols - 1; j >Image.cols - 100; j--)
		{
			for (int i = 0; i < Image.rows; i++)
			{

				p_2.x = j;
				p_2.y = i;
				if ((Image.at<Vec3b>(p_2)[0] == 255) && (Image.at<Vec3b>(p_2)[1] == 255) && (Image.at<Vec3b>(p_2)[2] == 255))
				{
					if (i < left_top)
						left_top = i;
					if (i > left_bottom)
						left_bottom = i;
					if (j < cnt_line)
						cnt_line = j;
				}
			}
		}
		if (left_bottom - left_top >= Image.rows*0.7 && 640 - cnt_line > 10)
		{
			right = 1;
		}
		else
		{
			right = 0;
		}
		

		//top
		bottom_left = 640; bottom_right = 0;
		cnt_line = 0;
		for (int j = 0; j <50; j++)
		{
			for (int i = 0; i < Image.cols; i++)
			{
				p_2.x = i;
				p_2.y = j;
				if ((Image.at<Vec3b>(p_2)[0] == 255) && (Image.at<Vec3b>(p_2)[1] == 255) && (Image.at<Vec3b>(p_2)[2] == 255))
				{
					if (i < bottom_left)
						bottom_left = i;
					if (i > bottom_right)
						bottom_right = i;
					if (j > cnt_line)
						cnt_line = j;
				}
			}
		}
		if (bottom_right - bottom_left >= Image.cols*0.7 && cnt_line > 10)
		{
			top = 1;
		}
		else
		{
			top = 0;
		}

		//bottom
		bottom_left = 640; bottom_right = 0;
		cnt_line = 480;
		for (int j = Image.rows - 1; j > Image.rows - 200; j--)
		{
			for (int i = 0; i < Image.cols; i++)
			{
				p_2.x = i;
				p_2.y = j;
				if ((Image.at<Vec3b>(p_2)[0] == 255) && (Image.at<Vec3b>(p_2)[1] == 255) && (Image.at<Vec3b>(p_2)[2] == 255))
				{
					if (i < bottom_left)
						bottom_left = i;
					if (i > bottom_right)
						bottom_right = i;
					if (j < cnt_line)
						cnt_line = j;
				}
			}
		}
		//cout << "$$$$$$$$$$$$cnt_line:" << cnt_line << endl;
		if (bottom_right - bottom_left >= Image.cols*0.7 && 480 - cnt_line > 100)
		{
			bottom = 1;
		}
		else
		{
			bottom = 0;
		}
	}
	else
	{
		left = 0;
		right = 0;
		top = 0;
		bottom = 0;
	}

	border_re.left = left;
	border_re.right = right;
	border_re.bottom = bottom;
	border_re.top = top;

	return border_re;
}