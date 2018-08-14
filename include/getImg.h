#pragma once
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "api/RpcLibClientBase.hpp"
#include "opencv2/opencv.hpp"

#define PIXEL_u 15
#define PIXEL_v 10
#define DIS_obstacle 4.0

using namespace std;
using namespace cv;
using namespace msr::airlib;

typedef ImageCaptureBase::ImageResponse ImageResponse;

//返回值为(640/2,480/2)的像素值
int imageResponse2Mat(ImageResponse image_info, Mat &image_mat)
{
	float data, distance = 0.0f;
	int flag = 1; //0：不可通行，1：可通行，-2：向左，2：向右
	//std::cout << "image_info.height=" << image_info.height << "\timage_info.width" << image_info.width << endl;
	for (int i = 0; i < image_info.height; i++)
	{
		for (int j = 0; j < image_info.width; j++)
		{
			// -- 取出深度值
			data = image_info.image_data_float.data()[i*image_info.width + j];
			image_mat.at<ushort>(i,j) = (int)data;
			//if (i == image_info.height / 2 && j == image_info.width / 2)
			//{
			//	distance = data;
			//}
			if (i > 240 - PIXEL_v && i < 240 + PIXEL_v && j > 320 - PIXEL_u && j < 320 + PIXEL_u)
			{
				if(data < DIS_obstacle)
					flag = 0;
			}

			if (data > 5.0f)
			{
				//image_mat.at<ushort>(i, j) = 0;
				image_mat.at<ushort>(i, j) = 65535;
			}
			else
			{
				image_mat.at<ushort>(i, j) = (int)(data * 10000 + 0.5f);  // 单位 0.1mm
			}
			//if (480 / 2 == i && 640 / 2 == j)
			//	printf("data:%dmm\n", ushort(data*1000+0.5f));//add by LB
		}
	}

	if (flag == 0)
	{
		for (int k = 0; k < 640 - 2* PIXEL_u; k++)
		{
			int ok = 1;
			for (int i = 240 - PIXEL_v; i < 240 + PIXEL_v; i++)
			{
				for (int j = k; j < k + 2 * PIXEL_u; j++)
				{
					// -- 取出深度值
					data = image_info.image_data_float.data()[i*image_info.width + j];
					if (data < DIS_obstacle)
						ok = 0;
				}
			}
			if (ok == 1)
			{
				if (k < 320 - PIXEL_u)
					flag = -2;
				else
					flag = 2;

				k = 640 - 2 * PIXEL_u + 1;
			}
		}
	}

	return flag;
}




