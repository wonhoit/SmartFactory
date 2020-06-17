//include system libraries
#include <stdio.h>
#include <stdint.h> // unint8_t definitions
#include <stdlib.h> // for exit(int);
#include <string.h> // for errno
#include <errno.h>  // error output

// wiring Pi
#include <wiringPi.h>
#include <wiringSerial.h>

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>

using namespace cv;
using namespace std;

#define COLOR_MASK_NUM 4

int threshold1 = 30;
int start_x = 10, start_y = 50, end_x = 1270, end_y = 450;	// roi

Vec3b lower_blue1, upper_blue1, lower_blue2, upper_blue2, lower_blue3, upper_blue3;
Mat img_color;

// Find Serial device on Raspberry with ~ls /dev/tty*
// ARDUINO_UNO "/dev/ttyACM0"
// FTDI_PROGRAMMER "/dev/ttyUSB)"
// HARDWARE_UART "/dev/ttyAMA0"
char device[] = "/dev/ttyACM0";

// filedescriptor
int fd;
unsigned long baud = 115200;
unsigned long currentTime = 0;

// prototypes declare
void serial_init(void);
void setup(void);
void dummy(int, void*) { };
void mouse_callback(int event, int x, int y, int flags, void *param);
void save_video(VideoCapture cap, Mat frame);
void ready_to_send(char* sendFlag);
int mapping_axis(int inputValue, int inputMin, int inputMax, int outMin, int outMax);
Mat create_color_mask(Mat color_img, int hue, int saturation, int value, int thresholdColor);

int main()
{
	String calAxis, colorType;
	int centerX, centerY, centerZ = 0;
	int xIntPre, yIntPre = -1;
	char sendFlag = '\0';

	namedWindow("img_color");
	setMouseCallback("img_color", mouse_callback);
	// track bar
	createTrackbar("threshold", "img_color", &threshold1, 255, dummy);
	setTrackbarPos("threshold", "img_color", 30);

	Mat img_hsv;
	VideoCapture cap(-1);
	if (!cap.isOpened()) {

		cout << "카메라를 열 수 없습니다." << endl;
		return -1;
	}
	cap.set(CAP_PROP_FRAME_WIDTH, 1280);
	cap.set(CAP_PROP_FRAME_HEIGHT, 960);
	// cap.set(CAP_PROP_FPS, 30);
	int fps = cap.get(CAP_PROP_FPS);
	
	Scalar labelColor(255, 255, 255);	// white

	serial_init();
	
	while(1)
	{
		cap.read(img_color);

		// ROI
		Mat img_camera_roi(img_color, Rect(start_x, start_y, end_x - start_x, end_y - start_y));
		
		threshold1 = getTrackbarPos("threshold", "img_color");
		cvtColor(img_camera_roi, img_hsv, COLOR_BGR2HSV);

		Mat color_mask[COLOR_MASK_NUM];
		color_mask[0] = create_color_mask(img_hsv, 4, 203, 136, 50);	// red mask and orange
		color_mask[1] = create_color_mask(img_hsv, 63, 110, 111, 30);	// green mask
		color_mask[2] = create_color_mask(img_hsv, 112, 154, 116, 50);	// blue mask
		color_mask[3] = create_color_mask(img_hsv, 26, 212, 202, 80);	// yellow mask

		// 마스크 이미지로 원본 이미지에서 범위에 해당되는 영상 부분을 획득
		Mat img_result;
		Mat img_labels, stats, centroids;
		int numOfLabels;

		for (int i = 0 ; i < COLOR_MASK_NUM; i++) {
			bitwise_and(img_camera_roi, img_camera_roi, img_result, color_mask[i]);
			numOfLabels = connectedComponentsWithStats(color_mask[i], img_labels, stats, centroids, 8, CV_32S);
			switch(i) {	// 0 : red | 1: green | 2: blue | 3: yellow
				case 0:
					colorType = "Red";
					labelColor =  Scalar(0, 0, 255);	// red
					break;
				case 1:
					colorType = "Green";
					labelColor =  Scalar(0, 255, 0);	// green
					break;
				case 2:
					colorType = "Blue";
					labelColor =  Scalar(255, 0, 0);	// blue
					break;
				case 3:
					colorType = "Yellow";
					labelColor =  Scalar(0, 255, 255);	// yellow
					break;
			}

			for (int j = 1; j < numOfLabels; j++) {
				int area = stats.at<int>(j, CC_STAT_AREA);
				int left = stats.at<int>(j, CC_STAT_LEFT);
				int top = stats.at<int>(j, CC_STAT_TOP);
				int width = stats.at<int>(j, CC_STAT_WIDTH);
				int height = stats.at<int>(j, CC_STAT_HEIGHT);

				centerX = centroids.at<double>(j, 0);
				centerY = centroids.at<double>(j, 1);

				if (area > 400)
				{
					// labeling
					circle(img_camera_roi, Point(centerX, centerY), 5, Scalar(255, 0, 0), 1);
					rectangle(img_camera_roi, Point(left, top), Point(left + width, top + height), labelColor, 1);
					putText(img_camera_roi, colorType, Point(left+2, top), FONT_HERSHEY_SIMPLEX, 0.4, labelColor, 1);
					putText(img_camera_roi, to_string(start_x + centerX) + "," + to_string(start_y + centerY), 
						Point(left+2, top+10), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(255, 255, 255), 1);

					ready_to_send(&sendFlag);
				    if( (sendFlag == '$') && (millis() - currentTime > 1000) ) {
						cout << "\n#1 Xaxis = " << centerX << ", Yaxis = " << centerY << endl;
						centerX = mapping_axis(centerX, 0, img_camera_roi.cols, -220, 220);
						centerY = mapping_axis(centerY, 0, img_camera_roi.rows, 288, 140);
						cout << "#2 Xaxis = " << centerX << ", Yaxis = " << centerY << endl;
						colorType = colorType[0];
						calAxis = '@' + colorType + to_string(centerX) + ',' + to_string(centerY) + ',' + to_string(centerZ);
						cout << "to serial >> " << calAxis << endl;
						serialPuts(fd, calAxis.c_str());    // colorType, x, y, z
						currentTime = millis();
			            xIntPre = -1;
			            yIntPre = -1;
			            sendFlag = '\0';
						serialFlush(fd);
					}
				}
			}
		}
		// imshow("img_camera_roi", img_camera_roi);
		rectangle(img_color, Point(start_x, start_y), Point(end_x, end_y), Scalar(0, 255, 0), 2);
		imshow("img_color", img_color);
		// imshow("img_mask", img_mask);
		// imshow("img_result", img_result);
		
		int wait = int(1.0 / fps * 1000);

		if (waitKey(wait)>=0)
			break;
	}
	

	return 0;
}

int mapping_axis(int inputValue, int fromLow, int fromHigh, int toLow, int toHigh) {
	return  (inputValue - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;;
}

void serial_init(){
    fflush(stdout);

    // get filedescriptor
    if (( fd = serialOpen (device, baud)) < 0 ) {
        fprintf (stderr, "Unable to open serial device : %s\n", strerror (errno));
        exit(1);    // error
    }

    // setup GPIO in wiringPi mode
    if (wiringPiSetup () == -1) {
        fprintf (stdout, "Unable to start wiringPi: %s\n", strerror (errno));
        exit(1);    // error
    }
}

void serial_send() {
    if(millis() - currentTime > 5000){
        serialPuts(fd, "0, 280, 0\n");    // x, y, z
        currentTime = millis();
        serialFlush(fd);
    }

    if(serialDataAvail (fd)) {
        char newChar = serialGetchar(fd);
        printf("%c", newChar);
        fflush(stdout);
    }
}

void ready_to_send(char* sendFlag) {
    if(serialDataAvail (fd)) {
        char newChar = serialGetchar(fd);
        if( newChar == '$') *sendFlag = newChar;
		printf("%c", newChar);
        fflush(stdout);
    }
}

Mat create_color_mask(Mat img_hsv, int hue, int saturation, int value, int thresholdColor){
	if (hue < 10)
	{
		// cout << "case 1" << endl;
		lower_blue1 = Vec3b(hue - 10 + 180, thresholdColor, thresholdColor);
		upper_blue1 = Vec3b(180, 255, 255);
		lower_blue2 = Vec3b(0, thresholdColor, thresholdColor);
		upper_blue2 = Vec3b(hue, 255, 255);
		lower_blue3 = Vec3b(hue, thresholdColor, thresholdColor);
		upper_blue3 = Vec3b(hue+10, 255, 255);
	}
	else if (hue > 170)
	{
		// cout << "case 2" << endl;
		lower_blue1 = Vec3b(hue, thresholdColor, thresholdColor);
		upper_blue1 = Vec3b(180, 255, 255);
		lower_blue2 = Vec3b(0, thresholdColor, thresholdColor);
		upper_blue2 = Vec3b(hue + 10 - 180, 255, 255);
		lower_blue3 = Vec3b(hue - 10, thresholdColor, thresholdColor);
		upper_blue3 = Vec3b(hue, 255, 255);
	}
	else
	{
		// cout << "case 3" << endl;
		lower_blue1 = Vec3b(hue, thresholdColor, thresholdColor);
		upper_blue1 = Vec3b(hue + 10, 255, 255);
		lower_blue2 = Vec3b(hue - 10, thresholdColor, thresholdColor);
		upper_blue2 = Vec3b(hue, 255, 255);
		lower_blue3 = Vec3b(hue - 10, thresholdColor, thresholdColor);
		upper_blue3 = Vec3b(hue, 255, 255);
	}

	Mat img_mask1, img_mask2, img_mask3, img_mask;
	inRange(img_hsv, lower_blue1, upper_blue1, img_mask1);
	inRange(img_hsv, lower_blue2, upper_blue2, img_mask2);
	inRange(img_hsv, lower_blue3, upper_blue3, img_mask3);
	img_mask = img_mask1 | img_mask2 | img_mask3;

	// morphologyEx 추가
	int morph_size = 2;
	Mat element = getStructuringElement(MORPH_RECT, Size(2 * morph_size + 1, 2 * morph_size + 1),
		Point(morph_size, morph_size));
	morphologyEx(img_mask, img_mask, MORPH_OPEN, element);
	morphologyEx(img_mask, img_mask, MORPH_CLOSE, element);

	return img_mask;
}

void mouse_callback(int event, int x, int y, int flags, void *param)
{
	if (event == EVENT_LBUTTONDOWN)
	{
		Vec3b color_pixel = img_color.at<Vec3b>(y, x);

		Mat bgr_color = Mat(1, 1, CV_8UC3, color_pixel);

		Mat hsv_color;
		cvtColor(bgr_color, hsv_color, COLOR_BGR2HSV);

		int hue = hsv_color.at<Vec3b>(0, 0)[0];
		int saturation = hsv_color.at<Vec3b>(0, 0)[1];
		int value = hsv_color.at<Vec3b>(0, 0)[2];

		cout << "hue = " << hue << endl;
		cout << "saturation = " << saturation << endl;
		cout << "value = " << value << endl;

		if (hue < 10)
		{
			cout << "case 1" << endl;
			lower_blue1 = Vec3b(hue - 10 + 180, threshold1, threshold1);
			upper_blue1 = Vec3b(180, 255, 255);
			lower_blue2 = Vec3b(0, threshold1, threshold1);
			upper_blue2 = Vec3b(hue, 255, 255);
			lower_blue3 = Vec3b(hue, threshold1, threshold1);
			upper_blue3 = Vec3b(hue+10, 255, 255);
		}
		else if (hue > 170)
		{
			cout << "case 2" << endl;
			lower_blue1 = Vec3b(hue, threshold1, threshold1);
			upper_blue1 = Vec3b(180, 255, 255);
			lower_blue2 = Vec3b(0, threshold1, threshold1);
			upper_blue2 = Vec3b(hue + 10 - 180, 255, 255);
			lower_blue3 = Vec3b(hue - 10, threshold1, threshold1);
			upper_blue3 = Vec3b(hue, 255, 255);
		}
		else
		{
			cout << "case 3" << endl;
			lower_blue1 = Vec3b(hue, threshold1, threshold1);
			upper_blue1 = Vec3b(hue + 10, 255, 255);
			lower_blue2 = Vec3b(hue - 10, threshold1, threshold1);
			upper_blue2 = Vec3b(hue, 255, 255);
			lower_blue3 = Vec3b(hue - 10, threshold1, threshold1);
			upper_blue3 = Vec3b(hue, 255, 255);
		}

		cout << "hue = " << hue << endl;
		cout << "#1 = " << lower_blue1 << "~" << upper_blue1 << endl;
		cout << "#2 = " << lower_blue2 << "~" << upper_blue2 << endl;
		cout << "#3 = " << lower_blue3 << "~" << upper_blue3 << endl;
	}
}

