#pragma once
#include "informationShareClass.h"
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <vector>			// stc::vector

class CarDetection
{
public:
	CarDetection(std::vector<cv::Point2f> lane1s, std::vector<cv::Point2f> lane2s);
	~CarDetection();
	void setInfoPackage(InformationShareClass *laneShare1, InformationShareClass *laneShare2);
	bool setSource(std::string file);
	void setOutputImage(cv::Mat *outImageL);
	void loopingThread();
	void stopThread();

private:
	InformationShareClass *car1, *car2;     // Übergabeparameter der Autos
	std::vector<cv::Point2f> lane1, lane2;  // Punktpositionen zum auswerten
	cv::VideoCapture *cap;					// Videoquelle
	cv::Mat *outImage;						// Ausgabebild
	bool stop = false;						// Stoppen der Endlosschleife
};

//int trackpoints;
//float trackAngles[trackpoints];


//int carPosition;
//int carType;

//Oli: GUI, Fahrzeugerkennung
//Domi: Threading, Bluetoothcom.