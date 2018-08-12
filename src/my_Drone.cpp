//airsim
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "api/RpcLibClientBase.hpp"
//opencv
#include "opencv2/opencv.hpp"

#include <iostream>
#include "WINDOWS.h"

#include "timer.h" 
//#include <pthread.h>
#include <thread>

//命名空间
using namespace std;
using namespace cv;
using namespace msr::airlib;

//airsim 相关
msr::airlib::MultirotorRpcLibClient client("localhost", 41451, 60000);//连接localhost:41451 	
typedef ImageCaptureBase::ImageRequest ImageRequest;
typedef ImageCaptureBase::ImageResponse ImageResponse;
typedef ImageCaptureBase::ImageType ImageType;

//线程
HANDLE hTimer1;//定时器
static std::mutex g_mutex;//互斥锁
void Key_Scan(void);
void get_img(void);
//按键
volatile int key_value_cv = -1;
static int key_control(int key);//按键控制

//图像
cv::Mat front_image, down_image,test_image;

//全局标志位
bool flag_exit = 0;//如果未true则表示推出程序
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

	printf("线程初始化\n");
	std::thread t_Key_Scan(Key_Scan);
	std::thread t_get_img(get_img);
	
	printf("线程初始化完成\n");
	//t_get_img.detach();
	//t_Key_Scan.detach();
	t_get_img.join();//阻塞，等待该线程退出
	t_Key_Scan.join();
	//关闭定时器
	CloseHandle(hTimer1);
	//摧毁所有OpenCV窗口
	cv::destroyAllWindows();
	
	printf("所有线程退出，程序结束\n");
	return 0;
}

//
//DWORD WINAPI Control_Z_Thread(LPVOID pVoid)
//{
//	int i = 0;
//	while (true)
//	{
//		//等待定时器时间到达
//		WaitForSingleObject(hTimer1, INFINITE);
//	}
//
//}

void Key_Scan(void)
{
	clock_t time_1;// = clock();//get time
	
	while (true)
	{
		//等待定时器时间到达
		WaitForSingleObject(hTimer1, INFINITE);
		if (flag_exit)//退出线程
		{
			return;
		}

		//显示图像
		g_mutex.lock();//锁
		if (!front_image.empty())
		{
			cv::imshow("FROWARD", front_image);
		}
		if (!down_image.empty())
		{
			cv::imshow("DOWN", down_image);
		}
		g_mutex.unlock();//释放锁
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
		key_control(key_value_cv);//执行按键功能

		//g_mutex.unlock();//释放锁
		
		
	}
	
}

void get_img(void)
{
	int i = 0;
	clock_t time_1;// = clock();//get time

	while (true)
	{
		//等待定时器时间到达
		WaitForSingleObject(hTimer1, INFINITE);
		if (flag_exit)//退出线程
		{
			return;
		}
		//g_mutex.lock();//获得锁
		if (++i >= 6)//50*6=300ms执行一次
		{
			i = 0;
			time_1 = clock();//get time

			std::vector<ImageRequest> request = { ImageRequest(0, ImageType::Scene) , ImageRequest(0, ImageType::DepthPerspective, true), ImageRequest(3, ImageType::Scene) };

			std::vector<ImageResponse>& response = client.simGetImages(request);

			if (response.size() > 0)
			{

				//仅仅是改变指针指向，不是image copy，所以不用加锁
				g_mutex.lock();//锁
				front_image = cv::imdecode(response.at(0).image_data_uint8, cv::IMREAD_COLOR);	//;前视图
				down_image = cv::imdecode(response.at(2).image_data_uint8, cv::IMREAD_COLOR);	//下视图	
				g_mutex.unlock();//释放锁
			}
		}
		//g_mutex.unlock();//释放锁
	}
	
}

static int key_control(int key)//按键控制
{
	clock_t time_1;// = clock();//get time

	switch (key)
	{
	case 27://ESC
		flag_exit = true;//退出标志位置位
		break;
	case 32://空格
		client.hover();//hover模式
		printf("hover\n");
		break;
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
	return 0;
}







