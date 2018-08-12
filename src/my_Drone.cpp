//airsim
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "api/RpcLibClientBase.hpp"
//opencv
#include "opencv2/opencv.hpp"

#include <iostream>
#include "WINDOWS.h"

#include "timer.h" 
#include <pthread.h>

//命名空间
//using namespace std;
using namespace cv;
using namespace msr::airlib;

//airsim 相关
msr::airlib::MultirotorRpcLibClient client("localhost", 41451, 60000);//连接localhost:41451 	
typedef ImageCaptureBase::ImageRequest ImageRequest;
typedef ImageCaptureBase::ImageResponse ImageResponse;
typedef ImageCaptureBase::ImageType ImageType;

//线程
DWORD WINAPI Key_Scan(LPVOID pVoid);//__stdcall 函数在返回到调用者之前将参数从栈中删除
DWORD WINAPI get_img(LPVOID pVoid);
DWORD WINAPI Control_Z_Thread(LPVOID pVoid);//控制四轴高度
HANDLE hTimer1;
pthread_mutex_t mutex1 = PTHREAD_MUTEX_INITIALIZER;//互斥锁					

//按键
volatile int key_value_cv = -1;
static int key_control(int key);//按键控制

//图像
cv::Mat front_image, down_image,test_image;

int main()
{
	while (RpcLibClientBase::ConnectionState::Connected != client.getConnectionState())
		client.confirmConnection();//连接场景

	while (!client.isApiControlEnabled())
		client.enableApiControl(true);//获取api控制

	client.armDisarm(true);//解锁飞控
						  
	client.hover();//hover模式
	
	//首先创建可等候定时器
	hTimer1 = CreateWaitableTimer(NULL, FALSE, NULL);
	//设置定时器时间
	INT64 nDueTime = -0 * _SECOND;//定时器生效时间，立即
	SetWaitableTimer(hTimer1, (PLARGE_INTEGER)&nDueTime, 50, NULL, NULL, FALSE);//50表示定时器周期50ms
																				//DWORD nThreadID = 0;
	HANDLE hThread1 = CreateThread(NULL, 0, Key_Scan, NULL, 0, NULL);// &nThreadID);
	HANDLE hThread2 = CreateThread(NULL, 0, get_img, NULL, 0, NULL);// &nThreadID);
	HANDLE hThread3 = CreateThread(NULL, 0, Control_Z_Thread, NULL, 0, NULL);// &nThreadID);

	while (true)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(20));//

		if (-1 != key_value_cv)//如果有按键按下
		{
			if (-1 == key_control(key_value_cv))
			{
				//摧毁所有OpenCV窗口
				cv::destroyAllWindows();
				//关闭线程
				CloseHandle(hThread1);
				CloseHandle(hThread2);
				CloseHandle(hThread3);
				//关闭定时器
				CloseHandle(hTimer1);

				return 0;
			}
		}
	}

	//等待线程返回
	WaitForSingleObject(hThread1, INFINITE);
	WaitForSingleObject(hThread2, INFINITE);
	CloseHandle(hThread1);
	CloseHandle(hThread2);
	CloseHandle(hThread3);
	//关闭定时器
	CloseHandle(hTimer1);
	return 0;
}


DWORD WINAPI Control_Z_Thread(LPVOID pVoid)
{
	int i = 0;
	while (true)
	{
		//等待定时器时间到达
		WaitForSingleObject(hTimer1, INFINITE);
	}

}

DWORD WINAPI Key_Scan(LPVOID pVoid)
{
	clock_t time_1;// = clock();//get time
	
	while (true)
	{
		//等待定时器时间到达
		WaitForSingleObject(hTimer1, INFINITE);

		pthread_mutex_lock(&mutex1);//获得锁

		//显示图像
		if (!front_image.empty())
		{
			cv::imshow("FROWARD", front_image);
		}
		if (!down_image.empty())
		{
			cv::imshow("DOWN", down_image);
		}

		//按键扫描
		if (-1 == key_value_cv)
		{
			key_value_cv = cv::waitKeyEx(1);
		}
		else
		{
			cv::waitKey(1);
		}

		while (-1 != cv::waitKey(1));//把缓冲区读完后，才会显示1ms图像

		pthread_mutex_unlock(&mutex1);//释放锁
	}
	return 0;
}

DWORD WINAPI get_img(LPVOID pVoid)
{
	int i = 0;
	clock_t time_1;// = clock();//get time

	while (true)
	{
		//等待定时器时间到达
		WaitForSingleObject(hTimer1, INFINITE);

		pthread_mutex_lock(&mutex1);//获得锁
		if (++i >= 6)//50*6=300ms执行一次
		{
			i = 0;
			time_1 = clock();//get time

			std::vector<ImageRequest> request;
			request = { ImageRequest(0, ImageType::Scene) , ImageRequest(0, ImageType::DepthPerspective, true), ImageRequest(3, ImageType::Scene) };

			std::vector<ImageResponse>& response = client.simGetImages(request);

			if (response.size() > 0)
			{
				front_image = cv::imdecode(response.at(0).image_data_uint8, cv::IMREAD_COLOR);	//;前视图
				down_image = cv::imdecode(response.at(2).image_data_uint8, cv::IMREAD_COLOR);	//下视图	
			}
		}
		pthread_mutex_unlock(&mutex1);//释放锁
	}
	return 0;
}

static int key_control(int key)//按键控制
{
	clock_t time_1;// = clock();//get time

	pthread_mutex_lock(&mutex1);//获得锁
	switch (key)
	{
	case 'w':
		client.moveByAngleThrottle(0.0f, 0.0f, 0.7f, 0.0f, 0.2f);
		break;
	case 's':
		client.moveByAngleThrottle(0.0f, 0.0f, 0.5f, 0.0f, 0.2f);
		break;
	case 'a'://旋转时会下降...
		client.moveByAngleThrottle(0.0f, 0.0f, 0.575, -1.0f, 0.2f);
		break;
	case 'd':
		client.moveByAngleThrottle(0.0f, 0.0f, 0.575, 1.0f, 0.2f);
		break;

	//下面是以机头方向前后左右
	case 'i'://pitch y轴逆时针角度
		client.moveByAngleThrottle(-0.1f, 0.0f, 0.575, 0.0f, 0.2f);
		break;
	case 'k'://pitch y轴逆时针角度
		client.moveByAngleThrottle(0.1f, 0.0f, 0.575, 0.0f, 0.2f);
		break;
	case 'j'://roll x轴逆时针角度
		client.moveByAngleThrottle(0.0f, -0.1f, 0.575, 0.0f, 0.2f);
		break;
	case 'l'://roll x轴逆时针角度
		client.moveByAngleThrottle(0.0f, 0.1f, 0.575, 0.0f, 0.2f);
		break;
	default:
		break;
	}

	key_value_cv = -1;
	pthread_mutex_unlock(&mutex1);//释放锁
	return 0;
}







