//airsim
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "api/RpcLibClientBase.hpp"

#include "opencv2/opencv.hpp"
#include <iostream>
#include "WINDOWS.h"
#include "timer.h" 
#include "pthread.h"
//#include "squares-by.hpp"

#include "PIDController.h"
#include "PID.h"

#include "yolo_v2_class.hpp"

#include "wgs_conversions/wgs_conversions.h"

//钻圈部分增加的头文件
//#include "saveImage.h"
#include "getImg.h"
//检测aruco二维码
#include "detect.h"
//命名空间
//using namespace std;
using namespace cv;
using namespace msr::airlib;

/*
相机ID 0至4分别对应于中央前方，左前方，右前方，中央下方，中央后方。
比赛时只能用底部摄像头场景图，前置摄像头的场景图和深度图。
*/
//定义相机ID
#define CAMERA_FRONT 0
#define CAMERA_FRONT_LEFT 1
#define CAMERA_FRONT_RIGHT 2
#define CAMERA_BELOW 3
#define CAMERA_BEHIND 4


//airsim 相关
typedef ImageCaptureBase::ImageRequest ImageRequest;
typedef ImageCaptureBase::ImageResponse ImageResponse;
typedef ImageCaptureBase::ImageType ImageType;
msr::airlib::MultirotorRpcLibClient client("localhost", 41451, 60000);//连接localhost:41451 
																	  /* 如果is_rate = true，则yaw_or_rate被解释为以度/秒为单位的角速度，
																	  这意味着您希望车辆在移动时以该角速度绕其轴线连续旋转。
																	  如果is_rate = false，则yaw_or_rate被解释为度数角度，
																	  这意味着您希望车辆旋转到特定角度（即偏航）并在移动时保持该角度。
																	  */
YawMode yaw_mode(true, 0);
//DrivetrainType driveTrain = DrivetrainType::ForwardOnly;
DrivetrainType driveTrain = DrivetrainType::MaxDegreeOfFreedom;

float roll = 0.1f, roll_temp;//绕x轴逆时针 //单位是弧度
float pitch = 0.1f, pitch_temp;//绕y轴逆时针  
float yaw = 0.0f; //绕z轴逆时针
float duration = 0.2f;//持续时间
float throttle = 0.575f;
float yaw_rate = 0.1f;
float altitude_last;//上一次的GPS高度，用来测试油门
float altitude_cur;//当前的GPS高度，用来测试油门

//传感器数据
GpsData			 GPS_data;
BarometerData	 Barometer_data;
MagnetometerData Magnetometer_data;
ImuData			 Imu_data;
//根据GPS，世界坐标系转为机体坐标系 y对应latitude，x对应longitude
// body_x = xcos + ysin
// body_y = ycos - xsin
double body_x = GPS_data.longitude * cos(0) + GPS_data.latitude * sin(0);
double body_y = GPS_data.latitude * cos(0) - GPS_data.longitude * sin(0);
double longitude_temp = (long)(GPS_data.longitude * 1e6) % 1000000;//保留小数点后6位数
double latitude_temp = (long)(GPS_data.latitude * 1e6) % 1000000;//保留小数点后6位数



//线程
DWORD WINAPI Key_Scan(LPVOID pVoid);//__stdcall 函数在返回到调用者之前将参数从栈中删除
DWORD WINAPI get_img(LPVOID pVoid);
DWORD WINAPI Control_Z_Thread(LPVOID pVoid);//控制四轴高度
HANDLE hTimer1;
pthread_mutex_t mutex1 = PTHREAD_MUTEX_INITIALIZER;//互斥锁									  
//int pthread_mutex_destroy(pthread_mutex_t *mutex1); //互斥锁的销毁

//按键
volatile int key_value_cv = -1;
static int key_control(int key);//按键控制

//图像
std::vector<ImageRequest> request = { ImageRequest(0, ImageType::Scene), ImageRequest(0, ImageType::DepthPerspective, true), ImageRequest(3, ImageType::Scene) };
std::vector<ImageResponse> response;
cv::Mat rgb_image_front;	//前视场景图
cv::Mat rgb_image_front_show;
cv::Mat depth_image_front;	//前视深度图
cv::Mat rgb_image_down;		//下视角场景图
std::string save = "D:\\My Documents\\AirSim\\my_Drone\\" + std::string("rgb_image.jpg");//图像保存路径
std::vector<std::vector<Point>> squares;
Point squares_centre;//检测到的停机拍中心在相机图像中的坐标

//std::vector<ImageRequest> request = { ImageRequest(CAMERA_BELOW, ImageType::Scene) };//创建获取正下方的场景图(默认压缩)的请求
//std::vector<ImageResponse> response;//响应
//cv::Mat rgb_image;
//PID
uint8_t Congtrol_Z_flag = 0;//1表示要控制高度
uint8_t Congtrol_Centre_flag = 0;//1表示要控制停机牌中心
double Control_Z = 14.3f;//控制的高度
double Control_Z_tolerance = 0.5f;// 0.01f;//控制高度的容忍
PID my_PID;
uint8_t Congtrol_dis_flag = 0;//1表示要控制前后距离为3m
//yolo
int gpu_id = 0;//用gpu0
std::string cfg_filename = "F:\\VS_Project\\source\\repos\\getimgTest\\getimgTest\\yolov3-voc-0616.cfg";//网络
std::string weight_filename = "F:\\VS_Project\\source\\repos\\getimgTest\\getimgTest\\yolov3-voc-0616_14500.weights";//by权重
//string pic_filename = "F:\\VS_Project\\source\\repos\\yolo_cfg_weight_names\\number_pic\\10_81.jpg";//待检测图片
//string name_filename = "F:\\VS_Project\\source\\repos\\yolo_cfg_weight_names\\yolo-obj.names";//各个标号对应的类别
//Detector my_yolo(cfg_filename, weight_filename, gpu_id);//初始化检测器
//Mat srcImage = imread(pic_filename);//读取图片
//vector<bbox_t> box = my_yolo.detect(srcImage);//yolo检测图片

//Detector detector_number(cfg_filename, weight_filename, gpu_id);//初始化检测器
//Detector detector_number("yolo-voc.cfg", "yolo-voc.weights");
std::vector<bbox_t> result_vec_front;	//前视图的检测结果
std::vector<bbox_t> result_vec_down;	//下视图的检测结果

//调转180°
void turn180(double reference);
double angle_cur;//atan2(y,x)=atan2(y/x)
//去指定的gps点
void goto_gps(double latitude, double longitude, double tolerance); //latitude，longitude为要去的经纬度
PID_GPS my_PID_GPS;//用于GPS定点
uint8_t Congtrol_GPS_flag = 0;//1表示开启GPS定点

//定义小树林区域
PID_TREE my_PID_TREE;//用于二维码左右对齐
int avoid_obstacle_flag = 0;
Point2f right_down(121.453281, 31.029951);
Point2f left_down(121.453264,31.030317);
Point2f left_up(121.453589, 31.030314 );
Point2f right_up(121.453582, 31.029956 );
//每次搜索的起点
Point2d num0_Point(121.454678, 31.030126);//停机牌0的坐标
//Point2d start_point[] = { Point2d(121.453323,31.029982), Point2d(121.453323,31.030050), 
//						  Point2d(121.453256,31.030117), Point2d(121.453265,31.030211), 
//						  Point2d(121.453244,31.030270)};
//Point2d start_point[] = { Point2d(121.453323,31.029991), Point2d(121.453324,31.030086),
//						  Point2d(121.453261,31.030146), Point2d(121.453249,31.030208),
//						  Point2d(121.453267,31.030287) };
Point2d start_point[] = { Point2d(121.453329,31.030004), Point2d(121.453318,31.030078),
						  Point2d(121.453267,31.030165), Point2d(121.453270,31.030224),
						  Point2d(121.453267,31.030291) };
//Point2d start_point[] = { Point2d(121.453254,31.030276) };
uint32_t start_point_ord = 0;//起始点序号，即下一个起始点的序号
//根据小树林区域四个点得到的四条直线
#define f1(x) (-10.318182*x + 1284.210212)
#define f2(x) (-0.764706*x + 123.906550)
#define f3(x) (0.091703*x + 19.892723)
#define f4(x) (-9.622222*x + 1199.680314)
#define f5(x) (0.208000*x + 5.767641)
#define f6(x) (0.672131*x - 50.602664)

//搜索小树林开启标志 1为开启搜索

uint8_t Congtrol_search_flag = 0;
uint8_t sprint_flag = 0;//1为冲刺，冲过树桩
bool Is_InArea(double latitude, double longitude)//给一个GPS的latitude和longitude判断是否在小树林区域内。，在区域内返回TRUE
{
	//for debug
	//printf("f1:%d\n", start_point[0].y > f1(start_point[0].x));
	//printf("f2:%d\n", start_point[1].y > f2(start_point[1].x));
	//printf("f3:%d\n", start_point[2].y > f3(start_point[2].x));
	//printf("f4:%d\n", start_point[3].y > f4(start_point[3].x));
	//printf("f5:%d\n", start_point[4].y > f5(start_point[4].x));
	//printf("f6:%d\n", start_point[5].y > f6(start_point[5].x));

	//printf("f1:%d\n", start_point[0].y < f1(start_point[0].x));
	//printf("f2:%d\n", start_point[1].y < f2(start_point[1].x));
	//printf("f3:%d\n", start_point[2].y < f3(start_point[2].x));
	//printf("f4:%d\n", start_point[3].y > f4(start_point[3].x));
	//printf("f5:%d\n", start_point[4].y > f5(start_point[4].x));
	//printf("f6:%d\n", start_point[5].y > f6(start_point[5].x));
	//latitude 为y
	if (latitude < f1(longitude))
		if (latitude < f2(longitude))
			if (latitude < f3(longitude)) 
				if (latitude > f4(longitude))
					if (latitude > f5(longitude))
						if (latitude > f6(longitude))
							return TRUE;

					
	return FALSE;
}
bool GetMarker = false;
bool moveToCentre(Point point1, Point point2);
void saveRGBImage(int id, ImageResponse& image_info_rgb);
void saveDepthImage(int id, ImageResponse& image_info_depth);
int main()
{
	//Initial = 0, Connected, Disconnected, Reset, Unknown
	while (RpcLibClientBase::ConnectionState::Connected != client.getConnectionState())
		client.confirmConnection();//连接场景
	while (!client.isApiControlEnabled())
		client.enableApiControl(true);//获取api控制
	client.armDisarm(true);//解锁飞控
						   //client.takeoff(1.0f);//最大起飞等待1s
	client.hover();//hover模式

	//初始化PID参数
	//PID_position_x.setParam();//位置控制PID
	//PID_position_y.setParam();//位置控制PID

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
				pthread_mutex_lock(&mutex1);//获得锁
				//关闭定时器
				CloseHandle(hTimer1);
				Sleep(300);//关闭时崩溃可能是线程正在与UE4通信，等一下
				//关闭线程
				CloseHandle(hThread1);
				CloseHandle(hThread2);
				CloseHandle(hThread3);
				//摧毁所有OpenCV窗口
				cv::destroyAllWindows();

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
		if (++i >= 6)//50*6=300ms执行一次
		{
			i = 0;
			//pthread_mutex_lock(&mutex1);//获得锁
			//如果需要控制高度
			if (1 == Congtrol_Z_flag)
			{
				////更新GPS数据
				//GPS_data = client.getGpsLocation();
				//更新高度数据
				Barometer_data = client.getBarometerdata();
				//if(Control_Z - GPS_data.altitude > Control_Z_tolerance)
				//	client.moveByAngleThrottle(0.0f, 0.0f, 0.03 + 0.575, 0.0f, 0.05f);//油门0.575大概是平衡
				//else if (Control_Z - GPS_data.altitude < -Control_Z_tolerance)
				//	client.moveByAngleThrottle(0.0f, 0.0f, -0.03 + 0.575, 0.0f, 0.05f);//油门0.575大概是平衡

				//P 控制油门
				double Throttle_Z = my_PID.PIDZ(Control_Z, Control_Z_tolerance);//设定高度，容忍
				if (Throttle_Z > Control_Z_tolerance*0.1 || Throttle_Z < -Control_Z_tolerance*0.1)
				{
					//client.moveByAngleThrottle(0.0f, 0.0f, Throttle_Z + 0.575, 0.0f, 0.05f);//油门0.6大概是平衡，持续50ms，与周期相同
					client.moveByAngleThrottle(0.0f, 0.0f, Throttle_Z + 0.575, 0.0f, 0.3f);//油门0.6大概是平衡，持续50ms，与周期相同
				}

			}
			//pthread_mutex_unlock(&mutex1);//释放锁
		}
	}

}

DWORD WINAPI Key_Scan(LPVOID pVoid)
{
	clock_t time_1;// = clock();//get time
				   //namedWindow("get_img");

	while (true)
	{
		//等待定时器时间到达
		WaitForSingleObject(hTimer1, INFINITE);

		pthread_mutex_lock(&mutex1);//获得锁

									//显示图像
		if (!rgb_image_front_show.empty())//显示前置摄像头的场景图
		{
			imshow("front_image_rgb", rgb_image_front_show);
		}
		if (!depth_image_front.empty())//显示前置摄像头的深度图
		{
			imshow("front_image_depth", depth_image_front);
		}
		if (!rgb_image_down.empty())//显示前置摄像头的深度图
		{
			imshow("down_image_rgb", rgb_image_down);
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
									 //std::cout << key_value_cv << std::endl;//显示键值
									 //if (-1 != key_value_cv)
									 //{
									 //	while (-1 != cv::waitKey(1));//把缓冲区读完后，才会显示1ms图像
									 //	if (-1 == key_control(key_value_cv))
									 //		break;
									 //}

		pthread_mutex_unlock(&mutex1);//释放锁
	}
	return 0;
}

DWORD WINAPI get_img(LPVOID pVoid)
{
	ofstream file("E:\\result.txt");
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
			//打印是否在小树林区域内
			//获得当前GPS数据
			GPS_data = client.getGpsLocation();
			//printf("Is_InArea:%d \n", Is_InArea(GPS_data.latitude, GPS_data.longitude));

			time_1 = clock();//get time
			//获取请求
			response = client.simGetImages(request);
			//printf("request time = %dms\n", clock() - time_1);
			if (response.size() > 0)
			{
				// -- down_rgb
				ImageResponse image_info_2 = response[2];
				rgb_image_down = cv::imdecode(image_info_2.image_data_uint8, cv::IMREAD_COLOR);

				
				// -- front_rgb
				ImageResponse image_info_0 = response[0];
				rgb_image_front = cv::imdecode(image_info_0.image_data_uint8, cv::IMREAD_COLOR);
				rgb_image_front.copyTo(rgb_image_front_show);
				//检测aruco markers
				int right_marker = -1;
				right_marker = detect_aruco_marker(rgb_image_front_show, markerCorners, markerIds);
				//画出可通行区域，要放在 检测aruco markers 后面
				circle(rgb_image_front_show, Point(640 / 2, 480 / 2), 4, Scalar(0, 255, 0), -1);//画出图像中心
				rectangle(rgb_image_front_show, Point(300, 230), Point(340, 250), Scalar(255, 0, 0), 2, CV_AA);
				
				// -- front_depth
				ImageResponse image_info_1 = response[1];
				depth_image_front = Mat(image_info_1.height, image_info_1.width, CV_16UC1);
				// 转换深度图获得浮点数的深度图
				avoid_obstacle_flag = imageResponse2Mat(image_info_1, depth_image_front);//返回值为距离

				switch (Congtrol_search_flag)
				{
				case 0:break;
				case 1:
					if (start_point_ord < sizeof(start_point) / sizeof(Point2d))
					{
						my_PID.PIDReset();//复位参数
						my_PID_GPS.PIDReset();//复位参数
						my_PID_TREE.PIDReset();//复位参数
						//Sleep(2000);//等待惯性消除
						//定高18.3
						Control_Z = 18.3;
						//Control_Z = 33.3;
						Congtrol_Z_flag = 1;//开启定高
						printf("定高%f\n", Control_Z);
						do//等待定高完成
						{
							Sleep(100);
							//更新高度数据
							Barometer_data = client.getBarometerdata();
						} while (abs(Barometer_data.altitude - Control_Z) > Control_Z_tolerance);
						//Sleep(2000);//等待惯性消除
						//printf("去左下角\n");
						//goto_gps(left_down.y, left_down.x, 0.2e-4);//去左下角
						cout << "移动到第一个起始点:" << start_point[start_point_ord] << endl;
						goto_gps(start_point[start_point_ord].y, start_point[start_point_ord].x, 0.2e-4);//去第一个起始点
						Sleep(2000);//等待惯性消除
								   //定高14.3
						printf("定高14.3\n");
						Control_Z = 14.3;
						Congtrol_Z_flag = 1;//开启定高
						do//等待定高完成
						{
							Sleep(100);
							//更新高度数据
							Barometer_data = client.getBarometerdata();
						} while (abs(Barometer_data.altitude - Control_Z) > Control_Z_tolerance);
						Sleep(2000);//等待惯性消除
						goto_gps(start_point[start_point_ord].y, start_point[start_point_ord].x, 0.2e-4);//再控制一次GPS
						Sleep(2000);//等待惯性消除
						//对准小树林
						printf("对准小树林\n");
						turn180(-99.49);
						//printf("控制前后距离，3m\n");

						start_point_ord++;//下一个起始点序号+1
						printf("避障搜索状态\n");
						Congtrol_search_flag = 4;//进入下一个状态，避障	
					}
					else
					{
						printf("返回初始点\n");
						Congtrol_search_flag = 9;//进入返回状态,回到起始点
					}
					break;
				default:break;
				case 4://避障策略

					   //if超出f2范围？抬升高度，移动到下一个GPS点，或者第一个起始点找到了两个二维码，也返回
					   //if (!Is_InArea(GPS_data.latitude, GPS_data.longitude))
					if (GPS_data.latitude > f1(GPS_data.longitude) || (1 == start_point_ord && detected_arucoID.size() >= 2) 
																   || (2 == start_point_ord && detected_arucoID.size() >= 4) 
																   || (3 == start_point_ord && detected_arucoID.size() >= 6)
																   || (4 == start_point_ord && detected_arucoID.size() >= 9))
					{
						my_PID.PIDReset();//复位参数
						my_PID_GPS.PIDReset();//复位参数
						my_PID_TREE.PIDReset();//复位参数
						if (start_point_ord < sizeof(start_point) / sizeof(Point2d))
						{
							//Congtrol_search_flag = 2;//进入下一个状态,调节前后距离
							//if (1 == start_point_ord)//如果是第一个起始点，左移一定距离，防止撞树
							//{
							//	Congtrol_Z_flag = 1;//关闭定高
							//	printf("第一个起始点，左移一定距离，防止撞树\n");
							//	std::this_thread::sleep_for(std::chrono::duration<double>(0.05));//延时50ms等定高关闭
							//	client.moveByAngleThrottle(0.0f, -0.2f, throttle, 0.0f, 3.0);
							//	std::this_thread::sleep_for(std::chrono::duration<double>(3.0));
							//}
							Sleep(2000);//等待惯性消除
							//定高18.3
							Control_Z = 18.3;
							//Control_Z = 33.3;
							Congtrol_Z_flag = 1;//开启定高
							printf("定高%f\n", Control_Z);
							do//等待定高完成
							{
								Sleep(100);
								//更新高度数据
								Barometer_data = client.getBarometerdata();
							} while (abs(Barometer_data.altitude - Control_Z) > Control_Z_tolerance);
							//Sleep(2000);//等待惯性消除
							printf("移动到下一个起始点[%d]:", start_point_ord);
							cout <<  start_point[start_point_ord] << endl;
							goto_gps(start_point[start_point_ord].y, start_point[start_point_ord].x, 0.2e-4);
							Sleep(2000);//等待惯性消除
									   //定高14.3
							printf("定高14.3\n");
							Control_Z = 14.3;
							Congtrol_Z_flag = 1;//开启定高
							do//等待定高完成
							{
								Sleep(100);
								//更新高度数据
								Barometer_data = client.getBarometerdata();
							} while (abs(Barometer_data.altitude - Control_Z) > Control_Z_tolerance);
							Sleep(2000);//等待惯性消除
							goto_gps(start_point[start_point_ord].y, start_point[start_point_ord].x, 0.2e-4);//再控制一次GPS
							Sleep(2000);//等待惯性消除
							//对准小树林
							printf("对准小树林\n");
							turn180(-99.49);
							//printf("控制前后距离，3m\n");

							start_point_ord++;//下一个起始点序号+1
						}
						else
						{
							printf("返回初始点\n");
							Congtrol_search_flag = 9;//进入返回状态,回到起始点
						}


					}
					else//否则避障策略
					{
						cout << "avoid_obstacle_flag: " << avoid_obstacle_flag << ":";
						switch (avoid_obstacle_flag)
						{
						case 0:
							cout << "Impassability！" << endl;
							client.moveByAngleThrottle(0.1f, 0.0f, throttle, 0.0f, 0.3f);
							;//返回起点或者向后？
							break;
						case 1:
							cout << "Forward" << endl;

							Congtrol_Z_flag = 1;//开启定高
							//如果检测到二维码
							//if (markerIds.size() > 0)
							if (-1 != right_marker && centerpoint[right_marker].x > 100)//如果检测到二维码并且偏右边
							{
								//获取当前距离
								double distance_cur = depth_image_front.at<uint16_t>(centerpoint[right_marker].y, centerpoint[right_marker].x);
								circle(rgb_image_front_show, Point(centerpoint[right_marker].x, centerpoint[right_marker].y), 4, Scalar(0, 255, 0), -1);//画出图像中心
								printf("distance_cur:%fmm\n", distance_cur);//add by LB
								//距离大于4.9m？
								if (distance_cur > 4.9*10000)
								{
									//移动
									GetMarker = moveToCentre(markerCorners.at(right_marker).at(0), markerCorners.at(right_marker).at(2));
								}
								else
								{
									//保存二维码截图，二维码ID，深度图数据，
									printf("保存二维码信息\n");
									detected_arucoID.push_back(markerIds[right_marker]);//二维码ID
									//打印已保存的二维码
									printf("已检测的二维码数目:%d\n", detected_arucoID.size());
									for (int i = 0; i<detected_arucoID.size(); i++)
										printf("%d\n", detected_arucoID.at(i));
									;//二维码截图
									;//深度图数据
									saveRGBImage(markerIds.at(right_marker), response[0]);
									saveDepthImage(markerIds.at(right_marker), response[1]);
									if (file.is_open())
									{
										printf("保存到result\n");
										file << markerIds.at(right_marker) << " " << markerCorners.at(right_marker).at(0).x << " " << markerCorners.at(right_marker).at(0).y
											<< " " << markerCorners.at(right_marker).at(2).x << " " << markerCorners.at(right_marker).at(2).y << "\n";
									}
									else
									{
										printf("file 没打开\n");
									}

									if (detected_arucoID.size() >= 10)//如果找完了所有二维码，返回初始点
									{
										printf("返回初始点\n");
										Congtrol_search_flag = 9;//进入返回状态,回到起始点
									}
									else
									{
										printf("避障搜索状态\n");
										Congtrol_search_flag = 4;
									}
								}
							}
							else//未检测到二维码
							{
								//client.moveByAngleThrottle(-0.1f, 0.0f, throttle, 0.0f, 0.3f);
								client.moveByAngleThrottle(-0.2f, 0.0f, throttle, 0.0f, 0.3f);//小幅度向前
							}
						

							break;
						case -2:
							cout << "Move Left" << endl;
							client.moveByAngleThrottle(0.0f, -0.15f, throttle, 0.0f, 0.3f);
							//sprint_flag = 1;
							break;
						case 2:
							cout << "Move Right" << endl;
							client.moveByAngleThrottle(0.0f, 0.15f, throttle, 0.0f, 0.3f);
							//sprint_flag = 1;
							break;
						default:
							break;
						}
					}

					break;
				case 9:
					Congtrol_search_flag = 10;//下一个状态
					//定高18.3
					Control_Z = 24.0;
					//Control_Z = 33.3;
					Congtrol_Z_flag = 1;//开启定高
					printf("定高%f\n", Control_Z);
					do//等待定高完成
					{
						Sleep(100);
						//更新高度数据
						Barometer_data = client.getBarometerdata();
					} while (abs(Barometer_data.altitude - Control_Z) > Control_Z_tolerance);
					Sleep(2000);//等待惯性消除
					//printf("移动到下一个起始点\n");
					cout << "返回起始点" << num0_Point << endl;
					goto_gps(num0_Point.y, num0_Point.x, 0.6e-4);
					Sleep(500);//等待惯性消除
							   //定高14.3
					printf("定高14.3\n");
					Control_Z = 14.3;
					Congtrol_Z_flag = 1;//开启定高
					do//等待定高完成
					{
						Sleep(100);
						//更新高度数据
						Barometer_data = client.getBarometerdata();
					} while (abs(Barometer_data.altitude - Control_Z) > Control_Z_tolerance);
					Sleep(2000);//等待惯性消除
								//对准停机牌
					;
					//降落
					Congtrol_Z_flag = 0;//关闭定高
					client.land(60);
					//std::this_thread::sleep_for(std::chrono::duration<double>(1));
					break;
				}
			}

		}

		
		pthread_mutex_unlock(&mutex1);//释放锁
		
	}
	file.close();
	return 0;
}

static int key_control(int key)//按键控制
{
	clock_t time_1;// = clock();//get time



	pthread_mutex_lock(&mutex1);//获得锁
	switch (key)
	{
	case 7340032://F1
		std::cout << "press 'F1' for help" << std::endl;
		std::cout << "press 'ESC' exit" << std::endl;
		std::cout << "press 'T' Drone takeoff" << std::endl;
		std::cout << "press 'W' Drone rise" << std::endl;
		std::cout << "press 'S' Drone descend" << std::endl;
		std::cout << "press Arrow keys: Drone forward,retreat,left,right" << std::endl;
		std::cout << "press 'I' or 'K' or 'J' or 'L': Drone forward,retreat,left,right" << std::endl;
		break;
	case 32://空格
		client.moveByAngleThrottle(pitch, roll, throttle, yaw_rate, duration);
		break;
	case 27://ESC
		std::cout << "disconnection" << std::endl;
		pthread_mutex_unlock(&mutex1);//释放锁
		return -1;
		break;
	case 't'://take off
		std::cout << "takeoff,wait 1s " << std::endl;
		client.takeoff(1.0f);//最大起飞等待1s
		std::this_thread::sleep_for(std::chrono::duration<double>(1));
		client.moveByAngleThrottle(-0.01f, 0.0f, throttle, 0.0f, 0.05f);//执行一次后，GPS高度会变成负数，指导环境复位
		std::cout << "takeoff OK" << std::endl;

		break;
	case 'b'://land
		std::cout << "land" << std::endl;
		client.land(5.0f);//最大起飞等待1s
		std::this_thread::sleep_for(std::chrono::duration<double>(5.0f));
		std::cout << "land OK" << std::endl;

		break;

	case 'w':
		client.moveByAngleThrottle(0.0f, 0.0f, 0.7f, 0.0f, 0.2f);

		//roll += 0.1f;//绕x轴逆时针 //单位是弧度
		//printf("\n pitch:%f, roll:%f, throttle:%f, yaw_rate:%f, duration:%f\n", pitch, roll, throttle, yaw_rate, duration);
		break;
	case 's':
		client.moveByAngleThrottle(0.0f, 0.0f, 0.5f, 0.0f, 0.2f);
		//roll -= 0.1f;//绕x轴逆时针 //单位是弧度
		//printf("\n pitch:%f, roll:%f, throttle:%f, yaw_rate:%f, duration:%f\n", pitch, roll, throttle, yaw_rate, duration);


		break;
	case 'a'://旋转时会下降...
		client.moveByAngleThrottle(0.0f, 0.0f, throttle, -1.0f, 0.2f);
		//client.moveByAngleThrottle(0.0f, 0.0f, 0.0f, 0.0f, 0.2f);
		//pitch += 0.1f;//绕y轴逆时针  
		//printf("\n pitch:%f, roll:%f, throttle:%f, yaw_rate:%f, duration:%f\n", pitch, roll, throttle, yaw_rate, duration);


		break;
	case 'd':
		client.moveByAngleThrottle(0.0f, 0.0f, throttle, 1.0f, 0.2f);

		break;

	case 2490368://高度控制增加
		Control_Z += 1.0f;
		printf("Control_Z=%f\n", Control_Z);
		break;
	case 2621440:////高度控制减少
		Control_Z -= 1.0f;
		printf("Control_Z=%f\n", Control_Z);

		break;
	case 2424832://向左飘（-Y方向）


		break;
	case 2555904://向右飘 （Y方向）

		break;

		//下面是以机头方向前后左右
	case 'i'://pitch y轴逆时针角度
		client.moveByAngleThrottle(-0.1f, 0.0f, throttle, 0.0f, 0.2f);
		//duration += 0.1f;//持续时间
		//printf("\n pitch:%f, roll:%f, throttle:%f, yaw_rate:%f, duration:%f\n", pitch, roll, throttle, yaw_rate, duration);


		break;
	case 'k'://pitch y轴逆时针角度
		client.moveByAngleThrottle(0.1f, 0.0f, throttle, 0.0f, 0.2f);
		//duration -= 0.1f;//持续时间
		//printf("\n pitch:%f, roll:%f, throttle:%f, yaw_rate:%f, duration:%f\n", pitch, roll, throttle, yaw_rate, duration);



		break;
	case 'j'://roll x轴逆时针角度
		client.moveByAngleThrottle(0.0f, -0.1f, throttle, 0.0f, 0.2f);

		break;
	case 'l'://roll x轴逆时针角度
		client.moveByAngleThrottle(0.0f, 0.1f, throttle, 0.0f, 0.2f);

		break;
	case '1'://显示GPS,用时

		time_1 = clock();//get time
		//下面耗时是period的两倍
		GPS_data = client.getGpsLocation();
		Barometer_data = client.getBarometerdata();
		Magnetometer_data = client.getMagnetometerdata();
		Imu_data = client.getImudata();

		std::cout << std::endl;
		std::cout << "GPS[latitude longitude altitude]:" << GPS_data.to_string()<< std::endl; ;//打印GPS
		std::cout << "Barometer_data:" << Barometer_data << std::endl;
		std::cout << "Magnetometer_data:" << Magnetometer_data.magnetic_field_body << std::endl;
		std::cout << "Imu_data.angular_velocity:" << Imu_data.angular_velocity << std::endl;
		std::cout << "Imu_data.linear_acceleration:" << Imu_data.linear_acceleration << std::endl;//xy?z,单位是g（重力加速度？）
		//std::cout << "Position[x y z]:" << client.getPosition() << std::endl; ;//打印坐标
		//printf("time_delta:\r\n");
		//printf("%d\r\n%d\r\n%d\r\n%d\r\n", time_2 - time_1, time_3 - time_2, time_4 - time_3, time_5 - time_4);
		
		std::cout << "Magnetometer_data[xyz]:" << Magnetometer_data.magnetic_field_body[0] << Magnetometer_data.magnetic_field_body[1] << Magnetometer_data.magnetic_field_body[2] << std::endl;
		angle_cur = atan2(Magnetometer_data.magnetic_field_body[1], Magnetometer_data.magnetic_field_body[0]);
		std::cout << "角度:" << 180.0f * angle_cur / M_PI << std::endl; ;//atan2(y,x)=atan2(y/x)
		
		//longitude_temp = (long)(GPS_data.longitude * 1e6) % 1000000 / 1e3;//保留小数点后6位数
		//latitude_temp = (long)(GPS_data.latitude * 1e6) % 1000000 / 1e3;//保留小数点后6位数
		longitude_temp = GPS_data.longitude;
		latitude_temp = GPS_data.latitude;
		/*body_x = GPS_data.longitude * cos(angle_cur) + GPS_data.latitude * sin(angle_cur);
		body_y = GPS_data.latitude * cos(angle_cur) - GPS_data.longitude * sin(angle_cur);*/
		body_x = longitude_temp * cos(angle_cur) + latitude_temp * sin(angle_cur);
		body_y = latitude_temp * cos(angle_cur) - longitude_temp * sin(angle_cur);
		printf("%f \n",cos(M_PI));
		printf("body_x:%f,body_y:%f \n", body_x, body_y);
		printf("time :%d ms\n", clock() - time_1);
		break;
	case '2'://切换控制高度
		if (1 == Congtrol_Z_flag)
		{
			Congtrol_Z_flag = 0;

		}
		else
		{
			Congtrol_Z_flag = 1;
		}
		printf("Congtrol_Z_flag=%d\n", Congtrol_Z_flag);


		break;
	case '3'://切换控制停机拍中心

		if (1 == Congtrol_Centre_flag)
		{
			Congtrol_Centre_flag = 0;

		}
		else
		{
			Congtrol_Centre_flag = 1;
			my_PID.PIDReset();//复位参数
		}
		printf("Congtrol_Centre_flag=%d\n", Congtrol_Centre_flag);
		break;
	case '4'://保存图像

		std::cout << save << std::endl;
		//imwrite(save, rgb_image);



		break;
	case '5'://测试掉头180°
		//flag 0 表示对准0°  1对准80.51065   2对准-99.49
		//turn180();

		break;
	case '6'://小树林搜索
		printf("小树林搜索\n");

		//开启定高
		Control_Z = 14.3f;//控制的高度，二维码就在这个高度
		Congtrol_Z_flag = 1;
		//掉头180°
		//turn180();

		break;
	case '7'://切换控制前后距离
		if (1 == Congtrol_dis_flag)
		{
			Congtrol_dis_flag = 0;

		}
		else
		{
			Congtrol_dis_flag = 1;
		}
		printf("Congtrol_dis_flag=%d\n", Congtrol_dis_flag);

		break;
	case '8'://切换搜索小树林 debug
		if (0 != Congtrol_search_flag)
		{
			Congtrol_search_flag = 0;

		}
		else
		{
			my_PID.PIDReset();//复位参数
			my_PID_GPS.PIDReset();//复位参数
			my_PID_TREE.PIDReset();//复位参数
			//Congtrol_search_flag = 1;
			//for debuge
			Congtrol_search_flag = 4;//for debug
			start_point_ord = 1;//for debug
			
		}
		printf("Congtrol_search_flag=%d\n", Congtrol_search_flag);

		break;
	
	case '9'://切换搜索小树林 test
		if (0 != Congtrol_search_flag)
		{
			Congtrol_search_flag = 0;

		}
		else
		{
			my_PID.PIDReset();//复位参数
			my_PID_GPS.PIDReset();//复位参数
			my_PID_TREE.PIDReset();//复位参数
			Congtrol_search_flag = 1;
		}
		printf("Congtrol_search_flag=%d\n", Congtrol_search_flag);

		

		break;
	case '0'://降落

		break;
	case 'z'://测试掉头
		//flag 0 表示对准0°  1对准80.51065   2对准-99.49
		turn180(0.0);

		break;
	case 'x'://测试掉头
			 //flag 0 表示对准0°  1对准80.51065   2对准-99.49
		turn180(80.51065);

		break;
	case 'c'://测试掉头
			 //flag 0 表示对准0°  1对准80.51065   2对准-99.49
		turn180(-99.49);

		break;
	case 'v'://去指定的gps点
		//flag 0 表示对准0°  1对准80.51065   2对准-99.49
		//goto_gps(31.030126, 121.454678);
		

		if (1 == Congtrol_GPS_flag)
		{
			Congtrol_GPS_flag = 0;

		}
		else
		{
			Congtrol_GPS_flag = 1;
			my_PID_GPS.PIDReset();//复位参数
			//turn180(0.0);//调整机头指向北极
		}
		printf("Congtrol_GPS_flag=%d\n", Congtrol_GPS_flag);
		
		goto_gps(31.030126, 121.454678, 0.6e-4);
		break;
	case 'n'://去指定的gps点
			 //flag 0 表示对准0°  1对准80.51065   2对准-99.49
			 //goto_gps(31.030126, 121.454678);
		if (1 == Congtrol_GPS_flag)
		{
			Congtrol_GPS_flag = 0;

		}
		else
		{
			Congtrol_GPS_flag = 1;
			my_PID_GPS.PIDReset();//复位参数
								  //turn180(0.0);//调整机头指向北极
		}
		printf("Congtrol_GPS_flag=%d\n", Congtrol_GPS_flag);

		//goto_gps(31.029951, 121.453281);
		goto_gps(right_down.y, right_down.x, 0.2e-4);
		break;
	case 'm'://去指定的gps点
			 //flag 0 表示对准0°  1对准80.51065   2对准-99.49
			 //goto_gps(31.030126, 121.454678);
		if (1 == Congtrol_GPS_flag)
		{
			Congtrol_GPS_flag = 0;

		}
		else
		{
			Congtrol_GPS_flag = 1;
			my_PID_GPS.PIDReset();//复位参数
								  //turn180(0.0);//调整机头指向北极
		}
		printf("Congtrol_GPS_flag=%d\n", Congtrol_GPS_flag);

		goto_gps(left_down.y, left_down.x, 0.2e-4);
		break;
	

	case '+'://
		kd += 0.000001;
		my_PID.PIDReset();//复位参数
		printf("kd=%f\n", kd);
		break;
	case '-'://
		kd += 0.000001;
		my_PID.PIDReset();//复位参数
		printf("kd=%f\n", kd);
		break;
	default:
		; break;

	}

	key_value_cv = -1;//

	pthread_mutex_unlock(&mutex1);//释放锁
	return 0;
}



void turn180(double reference) //flag 0 表示对准0°  1对准80.51065   2对准-99.49
{
	double tolerance = 0.5;//容忍1.5°误差
	double reference0 = 0.0;//设定值
	double reference1 = 80.51065;//设定值
	double reference2 = -99.49;//设定值
	double lf;
	double kp = 0.05;
	uint8_t time_temp = 0;//计数，连续N次在范围内认为已对准
	printf("->:%f ing...\n", reference);
	while (true)
	{
		//计算机头相对于世界坐标系角度
		Magnetometer_data = client.getMagnetometerdata();
		angle_cur = 180.0f * atan2(Magnetometer_data.magnetic_field_body[1], Magnetometer_data.magnetic_field_body[0]) / M_PI;//atan2(y,x)=atan2(y/x)


		if (angle_cur < reference || angle_cur >(reference + tolerance))
		{
			if (time_temp != 0)
				kp = 0.0001;
			else
				kp = 0.02;
			time_temp = 0;
			if (angle_cur < reference) {
				lf = kp * (reference - angle_cur);
			}
			else if (angle_cur > reference + tolerance) {
				lf = kp * (reference + tolerance - angle_cur);
			}
			else {
				lf = 0;
			}
			lf = -lf;
			CLIP3(-0.5f, lf, 0.5f);
			client.moveByAngleThrottle(0.0f, 0.0f, throttle, lf, 0.05f);
			std::this_thread::sleep_for(std::chrono::duration<double>(0.05f));
		}
		else
		{
			time_temp++;
			printf("time_temp:%d\n", time_temp);
			if (time_temp > 2)
			{
				printf("->:%f OK!\n", reference);
				client.moveByAngleThrottle(0.0f, 0.0f, throttle, 0.0, 0.1f);//停止转动
				std::this_thread::sleep_for(std::chrono::duration<double>(0.1f));
				break;
			}
			std::this_thread::sleep_for(std::chrono::duration<double>(0.05f));

		}

	}
}

void goto_gps(double latitude, double longitude, double tolerance) //latitude，longitude为要去的经纬度
{
	//double tolerance = 0.2e-4;//容忍误差 精调
	//double tolerance = 0.6e-4;//容忍误差 粗调
	double angle_tolerance = 15.0;//容忍角度误差
	uint8_t flag = 0;//
	GpsData GPS_data_cur;

	turn180(0.0);//调整机头指向北极
	while (true)
	{
		//获取当前角度值
		Magnetometer_data = client.getMagnetometerdata();
		angle_cur = 180.0f * atan2(Magnetometer_data.magnetic_field_body[1], Magnetometer_data.magnetic_field_body[0]) / M_PI;
		//std::cout << "角度:" << angle_cur << std::endl; ;//atan2(y,x)=atan2(y/x)
		if(angle_cur < 0.0-angle_tolerance || angle_cur > 0.0+angle_tolerance)
			turn180(0.0);//调整机头指向北极
		//获得当前GPS数据
		GPS_data_cur = client.getGpsLocation();
		//printf("gps_cur:%f,%f \n", GPS_data_cur.latitude, GPS_data_cur.longitude);
		
		flag = 0;
		double latitude_delta = latitude - GPS_data_cur.latitude;
		double longitude_delta = longitude - GPS_data_cur.longitude;
		pitch = my_PID_GPS.PIDX(latitude_delta, 0.15, tolerance);//0.436是25°，限制角度最大25°，容许误差0.2e-5
		roll = my_PID_GPS.PIDY(longitude_delta, 0.15, tolerance);//0.436是25°，限制角度最大25°
		printf("gps_err2:%lf,tolerance2:%lf \n", (latitude_delta*latitude_delta + longitude_delta*longitude_delta) * 1e8, tolerance * tolerance * 1e8);//打印当前gps误差
		if (tolerance * tolerance > (latitude_delta*latitude_delta + longitude_delta*longitude_delta))
		{
			GPS_data_cur = client.getGpsLocation();
			printf("goto_gps:%lf,%lf OK!,gps_err2:%lf,tolerance2:%lf\n", latitude, longitude, (latitude_delta*latitude_delta + longitude_delta*longitude_delta) * 1e8, tolerance * tolerance * 1e8);
			
			break;
		}
		client.moveByAngleThrottle(pitch, -roll, throttle, 0.0f, 0.250);//
		std::this_thread::sleep_for(std::chrono::duration<double>(0.300));


	}
}
bool moveToCentre(Point point1, Point point2)
{
	//ImageResponse image_info = response[1];
	//int x0 = 280, y0 = 200, x1 = 360, y1 = 280;
	int pixel_safe[] = { 250 ,100};//左边安全距离,右边安全距离
	//float depth_safe = 4;
	//pixel_safe = 0.5f / ((float)point2.y - (float)point1.y)*(float)1.0;
	//Point center;
	//center.x = (point2.x + point1.x) / 2;
	//center.y = (point2.y + point1.y) / 2;
	int move_direction;    //0:中心距离画面左边最近    1：距离上边最近   2：距离右边最近   3：距离下边最近
	int distance_min;
	int distance_toside[2] = { point1.x , 640 - point2.x };    //中心与图像四个边的距离

	distance_min = 2000;
	for (int i = 0; i< sizeof(distance_toside)/sizeof(int); i++)
	{
		if (distance_min>distance_toside[i])
		{
			distance_min = distance_toside[i];
			move_direction = i;
		}
	}
	if (distance_min < pixel_safe[move_direction])
	{
		switch (move_direction)
		{
		case 0:
			client.moveByAngleThrottle(0.0f, -0.2f, throttle, 0.0f, 0.3f);
			cout << "moveleft" << endl;
			break;
		case 1:
			client.moveByAngleThrottle(0.0f, 0.2f, throttle, 0.0f, 0.3f);
			cout << "moveright" << endl;
			break;
		}
	}
	else
	{
		client.moveByAngleThrottle(-0.25f, 0.0f, throttle, 0.0f, 0.3f);//小幅度向前
		cout << "movefront" << endl;
	}


	return false;
}

void saveRGBImage(int id, ImageResponse& image_info_rgb)
{
	char filename[100];
	sprintf_s(filename, "E:\\images\\%d.png", id);
	std::ofstream file(filename, std::ios::binary);
	file.write(reinterpret_cast<const char*>(image_info_rgb.image_data_uint8.data()), image_info_rgb.image_data_uint8.size());
	file.close();
	cout << filename << endl;
}

void saveDepthImage(int id, ImageResponse& image_info_depth)
{
	char filename[100];
	sprintf_s(filename, "E:\\depth\\%d.pfm", id);
	Utils::writePfmFile(image_info_depth.image_data_float.data(), image_info_depth.width, image_info_depth.height, filename);
	cout << filename << endl;
}
