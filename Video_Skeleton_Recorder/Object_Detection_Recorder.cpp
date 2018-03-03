// Created by Heresy @ 2015/02/03
// Blog Page: https://kheresy.wordpress.com/2015/02/11/k4w-v2-part-6-draw-with-opengl/
//
// This is used to read RGB and Depth frame.
//
// Modified by HsinWei @ 2018 / 01 / 30

// Standard Library
#include <iostream>

#include <fstream>
#include <string>

// OpenCV Header
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

// Kinect for Windows SDK Header
#include <Kinect.h>

#include <nlohmann/json.hpp>

// for convenience
using json = nlohmann::json;

using namespace std;
// Parameters
int countdown = 3;
int ROI_point_X = 480, ROI_point_Y = 240, ROI_width = 700, ROI_height = 500;

// global objects
IKinectSensor*		pSensor = nullptr;
IColorFrameReader*	pColorFrameReader = nullptr;
IDepthFrameReader*	pDepthFrameReader = nullptr;
ICoordinateMapper*	pCoordinateMapper = nullptr;

int		iColorWidth = 0, iColorHeight = 0;
UINT	uDepthPointNum = 0;
UINT	uColorPointNum = 0;
UINT	uColorBufferSize = 0;

UINT16*	pDepthBuffer = nullptr;
BYTE*	pColorBuffer = nullptr;
CameraSpacePoint* pCSPoints = nullptr;

void writeCoordinateInfo(ofstream&, CameraSpacePoint*);

int main(int argc, char** argv)
{
	// 1. Sensor related code
	cout << "Try to get default sensor" << endl;
	{
		if (GetDefaultKinectSensor(&pSensor) != S_OK)
		{
			cerr << "Get Sensor failed" << endl;
			return -1;
		}

		cout << "Try to open sensor" << endl;
		if (pSensor->Open() != S_OK)
		{
			cerr << "Can't open sensor" << endl;
			return -1;
		}
	}

	// 2. Color related code
	cout << "Try to get color source" << endl;
	{
		// Get frame source
		IColorFrameSource* pFrameSource = nullptr;
		if (pSensor->get_ColorFrameSource(&pFrameSource) != S_OK)
		{
			cerr << "Can't get color frame source" << endl;
			return -1;
		}

		// Get frame description
		cout << "get color frame description" << endl;
		IFrameDescription* pFrameDescription = nullptr;
		if (pFrameSource->get_FrameDescription(&pFrameDescription) == S_OK)
		{
			pFrameDescription->get_Width(&iColorWidth);
			pFrameDescription->get_Height(&iColorHeight);

			uColorPointNum = iColorWidth * iColorHeight;
			uColorBufferSize = uColorPointNum * 4 * sizeof(BYTE);

			pCSPoints = new CameraSpacePoint[uColorPointNum];
			pColorBuffer = new BYTE[4 * uColorPointNum];
		}
		pFrameDescription->Release();
		pFrameDescription = nullptr;

		// get frame reader
		cout << "Try to get color frame reader" << endl;
		if (pFrameSource->OpenReader(&pColorFrameReader) != S_OK)
		{
			cerr << "Can't get color frame reader" << endl;
			return -1;
		}

		// release Frame source
		cout << "Release frame source" << endl;
		pFrameSource->Release();
		pFrameSource = nullptr;
	}

	// 3. Depth related code
	cout << "Try to get depth source" << endl;
	{
		// Get frame source
		IDepthFrameSource* pFrameSource = nullptr;
		if (pSensor->get_DepthFrameSource(&pFrameSource) != S_OK)
		{
			cerr << "Can't get depth frame source" << endl;
			return -1;
		}

		// Get frame description
		cout << "get depth frame description" << endl;
		IFrameDescription* pFrameDescription = nullptr;
		if (pFrameSource->get_FrameDescription(&pFrameDescription) == S_OK)
		{
			int	iDepthWidth = 0,
				iDepthHeight = 0;
			pFrameDescription->get_Width(&iDepthWidth);
			pFrameDescription->get_Height(&iDepthHeight);
			uDepthPointNum = iDepthWidth * iDepthHeight;
			pDepthBuffer = new UINT16[uDepthPointNum];
		}
		pFrameDescription->Release();
		pFrameDescription = nullptr;

		// get frame reader
		cout << "Try to get depth frame reader" << endl;
		if (pFrameSource->OpenReader(&pDepthFrameReader) != S_OK)
		{
			cerr << "Can't get depth frame reader" << endl;
			return -1;
		}

		// release Frame source
		cout << "Release frame source" << endl;
		pFrameSource->Release();
		pFrameSource = nullptr;
	}

	// 4. Coordinate Mapper
	if (pSensor->get_CoordinateMapper(&pCoordinateMapper) != S_OK)
	{
		cerr << "get_CoordinateMapper failed" << endl;
		return -1;
	}


	// file output stream
	ofstream csvfile;
	string filename = "D:/GoogleDrive/Graduate/Research/Research_HsinWei/Programs/data/ObjectDetection/rawData/original.csv";
	csvfile.open(filename);

	// Prepare OpenCV data
	cv::Mat	mImg(iColorHeight, iColorWidth, CV_8UC4);


	// countdown to start
	cout << "Prepare hardward...\n";
	while (countdown > 0) {
		cout << countdown << endl;
		countdown--;
		Sleep(1000);
	}

	// Enter main loop
	int step = 0;
	while (step<1)
	{
		// Read color data
		IColorFrame* pCFrame = nullptr;
		// Read depth data
		IDepthFrame* pDFrame = nullptr;
		if (pColorFrameReader->AcquireLatestFrame(&pCFrame) == S_OK && pDepthFrameReader->AcquireLatestFrame(&pDFrame) == S_OK)
		{
			pCFrame->CopyConvertedFrameDataToArray(uColorBufferSize, pColorBuffer, ColorImageFormat_Bgra);
			pDFrame->CopyFrameDataToArray(uDepthPointNum, pDepthBuffer);
			
			// map to camera space
			pCoordinateMapper->MapColorFrameToCameraSpace(uDepthPointNum, pDepthBuffer, uColorPointNum, pCSPoints);
			
			// Store Image
			mImg.data = pColorBuffer;
			// cv::Mat roi_img = mImg(cv::Rect(ROI_point_X, ROI_point_Y, ROI_width, ROI_height));

			cv::imwrite("D:/GoogleDrive/Graduate/Research/Research_HsinWei/Programs/data/ObjectDetection/rawData/original.jpg", mImg);
			// cv::imwrite("D:/GoogleDrive/Graduate/Research/Research_HsinWei/Programs/data/ObjectDetection/rawData/roi_img.jpg", roi_img);


			std::ifstream i("D:/GoogleDrive/Graduate/Research/Research_HsinWei/Programs/data/ObjectDetection/rawData/output.json");
			json j;
			i >> j;
			for (json::iterator it = j.begin(); it != j.end(); ++it) {
				std::cout << it.key() << " : " << it.value() << "\n";
			}
			// Store Coordinate info
			// writeCoordinateInfo(csvfile, pCSPoints);

			pCFrame->Release();
			pCFrame = nullptr;
		}
		else
		{
			cout << "111111";
			continue;
		}

		/*
		// Read depth data
		IDepthFrame* pDFrame = nullptr;
		if (pDepthFrameReader->AcquireLatestFrame(&pDFrame) == S_OK)
		{
			pDFrame->CopyFrameDataToArray(uDepthPointNum, pDepthBuffer);

			pDFrame->Release();
			pDFrame = nullptr;

			// map to camera space
			pCoordinateMapper->MapColorFrameToCameraSpace(uDepthPointNum, pDepthBuffer, uColorPointNum, pCSPoints);

		}
		*/
		++step;
	}


	cout << "Release color & Depth frame Reader";
	// release color frame reader
	pColorFrameReader->Release();
	pColorFrameReader = nullptr;

	// release Depth frame reader
	pDepthFrameReader->Release();
	pDepthFrameReader = nullptr;

	// Close Sensor
	pSensor->Close();

	cout << "Release Sensor";
	// Release Sensor
	pSensor->Release();
	pSensor = nullptr;

	//system("pause");
}

void writeCoordinateInfo(ofstream& csvout, CameraSpacePoint* pCSPoints) {
	for (int y = ROI_point_Y; y < ROI_point_Y + ROI_height; y++)
	{
		for (int x = ROI_point_X; x < ROI_point_X + ROI_width; x++)
		{
			int index = y * 1920 + x;
			csvout << pCSPoints[index].X << "," << pCSPoints[index].Y << "," << pCSPoints[index].Z << "\n";
		}
	}
}