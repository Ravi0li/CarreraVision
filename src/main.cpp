#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <boost/thread.hpp>

#include "trackdetectionClass.h"
#include "commandlineparser.h"
#include "informationShareClass.h"
#include "carDetectionClass.h"
#include "carControlDomiClass.h"

void carDetectionThread ();
void carControlThread ();

int main(int argc, const char** argv)
{
	// Übergabeparameter auswerten
	cv::CommandLineParser parser(argc, argv, clpKeys);
	if (parser.has("help"))
	{
		clpHelp();
		return 0;
	}

	// Trackimage laden
	cv::Mat image;
	if (parser.has("trackrecord"))
	{
		// Trackimage von der Kamera
		cv::VideoCapture cap(0);
		if (!cap.isOpened())
		{
			std::cout << "Cannot open Camera" << std::endl;
			return -1;
		}
		cap.set(CV_CAP_PROP_FPS, 10);
		cap.set(CV_CAP_PROP_FRAME_WIDTH,3280);
		cap.set(CV_CAP_PROP_FRAME_HEIGHT,2464);
		cap >> image;
		std::cout << "Bildaufloesung    X: " << image.cols << "    Y: " << image.rows << std::endl;
		if (image.empty())
		{
			std::cout << "Cannot grab a image from Camera" << std::endl;
			return -1;
		}
		cv::namedWindow("Input", CV_GUI_NORMAL);
		cv::imshow("Input", image);
	}
	else
	{
		// Trackimage laden
		std::string TrackimgFile = parser.get<std::string>("trackimg");
		image = cv::imread(TrackimgFile, 1);
		if (image.empty())
		{
			std::cout << "Cannot read image file: " << TrackimgFile << std::endl;
			return -1;
		}
	}

	// Parameterdatei öffnen
	std::string paraFile = parser.get<std::string>("para");
	cv::FileStorage para;
	if (!para.open(paraFile, cv::FileStorage::READ))
	{
		std::cout << "Cannot read para file: " << paraFile << std::endl;
		return -1;
	}

	// Strecken auswertung
	TrackDetection trackDetection(para["track_detection"]);
	trackDetection.setDebugWin(parser.get<bool>("debugwin"));
	trackDetection.setPicture(image);
	trackDetection.calculate(0.20f);
	image = trackDetection.getResultPicture();
	std::vector<cv::Point2f> lane1, lane2;
	trackDetection.getPointLines(&lane1, &lane2);
		
	// Bluetoothcommunication vorbereiten
	// bluetoothConnectionClass BLECon;
	// BLECon.connect();

	// Vorbereiten der Streckenauswertung
	InformationShareClass infoPackage1, infoPackage2;
	CarDetection carDetection(lane1, lane2);
	carDetection.setInfoPackage(&infoPackage1, &infoPackage2);
	carDetection.setOutputImage(&image);
	if (!carDetection.setSource("./demo/Videos/ExampleE1_iso200_b5000.mp4"))
	{
		std::cout << "Programmabbruch" << std::endl;
		std::getchar();
		return -1;
	}

	// Bluetooth Funktionalität herstellen
	BluetoothConnectionClass BLECon;
	bool BTconnected = false;
	// Versuche COM-Port zu öffnen, 5 Versuche
	/*BLECon.disconnect();
	for (int countOpen = 0; countOpen < 5; countOpen++)
	{
		if (BLECon.connect() == 1)
		{
			// erfolgreich
			BTconnected = true;
			break;
		}
	}*/

	// Vorbereiten der Regelung
	CarControlDomiClass carControlDomi1(&infoPackage1, lane1.size(), &lane1, &BLECon, 1, 0.01f);
	CarControlDomiClass carControlDomi2(&infoPackage2, lane2.size(), &lane2, &BLECon, 2, 0.01f);

	// Threads
	boost::thread_group tgroup;
	tgroup.create_thread(boost::bind(&CarDetection::loopingThread, &carDetection));
	tgroup.create_thread(boost::bind(&CarControlDomiClass::loopingThread, &carControlDomi1));
	tgroup.create_thread(boost::bind(&CarControlDomiClass::loopingThread, &carControlDomi2));
	
	// Fenster zur anzeige anlegen
	cv::namedWindow("Result", CV_GUI_NORMAL);
	cv::resizeWindow("Result", 700, 500);
	do
	{
		cv::Mat imageText;
		image.copyTo(imageText);
		cv::putText(imageText, "Stellsignal 1: " + std::to_string(BLECon.getSetValue1()), cv::Point(20, 40), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 255), 2);
		cv::putText(imageText, "Stellsignal 2: " + std::to_string(BLECon.getSetValue2()), cv::Point(20, 70), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 255), 2);
		cv::imshow("Result", imageText);
	} while (-1 == cv::waitKey(10));

	// Warten bis threads ordnungsgemäß beendet sind
	carDetection.stopThread();
	carControlDomi1.stopThread();
	carControlDomi2.stopThread();
	tgroup.join_all();

	// Anzeigen
	//cv::namedWindow("Result", CV_GUI_NORMAL);
	//cv::resizeWindow("Result", 700, 500);
	//cv::imshow("Result", image);
	//cv::waitKey(0);

	return 0;
}

void carDetectionThread ()
{

}

void carControlThread ()
{

}
