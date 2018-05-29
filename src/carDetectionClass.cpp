#define _USE_MATH_DEFINES
#include "carDetectionClass.h"
#include <opencv2/highgui.hpp>
#include <iostream>

// --------------------------------------------------------------------------
// Konstruktor
// --------------------------------------------------------------------------
CarDetection::CarDetection(std::vector<cv::Point2f> lane1s, std::vector<cv::Point2f> lane2s)
{
	// Setzen der Auswertepunkte
	lane1 = lane1s;
	lane2 = lane2s;
}

// --------------------------------------------------------------------------
// Destruktor
// --------------------------------------------------------------------------
CarDetection::~CarDetection()
{
	cap->release();
}

// --------------------------------------------------------------------------
// Setzen der Austauschklassen
// --------------------------------------------------------------------------
void CarDetection::setInfoPackage(InformationShareClass *laneShare1, InformationShareClass *laneShare2)
{
	car1 = laneShare1;
	car2 = laneShare2;
}

// --------------------------------------------------------------------------
// Setzt das Ausgabebild
// --------------------------------------------------------------------------
void CarDetection::setOutputImage(cv::Mat *outImageL)
{
	outImage = outImageL;
}

// --------------------------------------------------------------------------
// Setzt die Videoquelle (Wenn kein File dann von der Kamera)
// --------------------------------------------------------------------------
bool CarDetection::setSource(std::string file)
{
	// Öffne die Kamera als Quelle
	if (file.empty())
	{
		cap = new cv::VideoCapture(0);
		if (!cap->isOpened())
		{
			std::cout << "FEHLER: Die Kamera konnte nicht angesprochen werden" << std::endl;
			return false;
		}
		cap->set(CV_CAP_PROP_FPS, 30);
		cap->set(CV_CAP_PROP_FRAME_WIDTH, 1640);
		cap->set(CV_CAP_PROP_FRAME_HEIGHT, 1232);
		//cap->set(CV_CAP_PROP_FPS, 10);
		//cap->set(CV_CAP_PROP_FRAME_WIDTH, 640);
		//cap->set(CV_CAP_PROP_FRAME_HEIGHT, 576);

	}
	else // Lade eine Datei als Quelle
	{
		cap = new cv::VideoCapture(file);
		if (!cap->isOpened())
		{
			std::cout << "FEHLER: Datei nicht gefunden mit dem Videostream" << std::endl;
			return false;
		}
	}
	return true;
}

// --------------------------------------------------------------------------
// Setzt das Ausgabebild
// --------------------------------------------------------------------------
void CarDetection::stopThread()
{
	stop = true;
}

// --------------------------------------------------------------------------
// Dauerschleife zur Auswertung
// --------------------------------------------------------------------------
void CarDetection::loopingThread()
{
	cv::Mat image;
	do {
		*cap >> image;
		if (!image.empty())
		{
			*outImage = image;
		}
	} while(!stop);
}