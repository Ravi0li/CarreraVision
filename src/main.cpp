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

static void onMouse(int event, int x, int y, int, void*);

struct ParamOnMouse
{
public:
	CarDetection* carDetection;
	CarControlDomiClass* carControlDomi1;
	CarControlDomiClass* carControlDomi2;
};

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
			std::getchar();
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
			std::getchar();
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
			std::getchar();
			return -1;
		}
	}

	// Parameterdatei öffnen
	std::string paraFile = parser.get<std::string>("para");
	cv::FileStorage para;
	if (!para.open(paraFile, cv::FileStorage::READ))
	{
		std::cout << "Cannot read para file: " << paraFile << std::endl;
		std::getchar();
		return -1;
	}

	// Strecken auswertung
	TrackDetection trackDetection(para["track_detection"]);
	trackDetection.setDebugWin(parser.get<bool>("debugwin"));
	trackDetection.setPicture(image);
	if (!trackDetection.calculate(0.10f))
	{
		std::cout << "Programmabbruch" << std::endl;
		std::getchar();
		return -1;
	}
	image = trackDetection.getResultPicture();
	std::vector<cv::Point2f> lane1, lane2;
	trackDetection.getPointLines(&lane1, &lane2);	

	// Vorbereiten der Streckenauswertung
	InformationShareClass infoPackage1, infoPackage2;
	CarDetection carDetection(para["car_detection"], lane1, lane2);
	carDetection.setInfoPackage(&infoPackage1, &infoPackage2);
	carDetection.setOutputImage(&image);
	if (!carDetection.setSource(parser.get<std::string>("trackvid")))
	{
		cv::waitKey(10);
		std::cout << "Programmabbruch" << std::endl;
		std::getchar();
		return -1;
	}

	// Bluetooth Funktionalität herstellen
	BluetoothConnectionClass BLECon(para["bluetooth_connection"]);
	bool BTconnected = false;
	// Versuche COM-Port zu öffnen, 5 Versuche
	BLECon.disconnect();
	for (int countOpen = 0; countOpen < 5; countOpen++)
	{
		if (BLECon.connect() == 1)
		{
			std::cout << "Bluetooth Verbindung hergestellt." << std::endl;

			// erfolgreich
			BTconnected = true;
			break;
		}
	}

	// Vorbereiten der Regelung
	CarControlDomiClass carControlDomi1(para["car_control_domi"], &infoPackage1, (int)lane1.size(), &lane1, &BLECon, 1, 0.10f);
	CarControlDomiClass carControlDomi2(para["car_control_domi"], &infoPackage2, (int)lane2.size(), &lane2, &BLECon, 2, 0.10f);

	// Threads
	boost::thread_group tgroup;
	tgroup.create_thread(boost::bind(&CarDetection::loopingThread, &carDetection));
	tgroup.create_thread(boost::bind(&CarControlDomiClass::loopingThread, &carControlDomi1));
	tgroup.create_thread(boost::bind(&CarControlDomiClass::loopingThread, &carControlDomi2));
	
	// Fenster zur anzeige anlegen
	cv::namedWindow("Result", CV_GUI_NORMAL);
	cv::resizeWindow("Result", 700, 500);
	ParamOnMouse param;
	param.carDetection = &carDetection;
	param.carControlDomi1 = &carControlDomi1;
	param.carControlDomi2 = &carControlDomi2;
	cv::setMouseCallback("Result", onMouse, &param);
	do
	{
		cv::Mat imageText;
		image.copyTo(imageText);
		cv::putText(imageText, "Stellsignal 1: " + std::to_string(BLECon.getSetValue1()), cv::Point(20, 40), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 255), 2);
		cv::putText(imageText, "Stellsignal 2: " + std::to_string(BLECon.getSetValue2()), cv::Point(20, 70), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 255), 2);
		cv::putText(imageText, "Kallibrieren", cv::Point(20, 100), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 255), 2);
		cv::putText(imageText, "Steuerung 1 Richtung", cv::Point(20, 130), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 255), 2);
		cv::putText(imageText, "Steuerung 2 Richtung", cv::Point(20, 160), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 255), 2);
		cv::putText(imageText, "Ansicht umschalten", cv::Point(20, 190), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 255), 2);
		cv::putText(imageText, "Steuerung Aus. Rich", cv::Point(20, 220), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 255), 2);
		std::string outFps = "FPS: " + std::to_string(carDetection.getFrameRate()) + " " + std::to_string(carControlDomi1.getFrameRate()) + " " + std::to_string(carControlDomi2.getFrameRate());
		cv::putText(imageText, outFps , cv::Point(20, 250), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 255), 2);
		cv::imshow("Result", imageText);

	} while (-1 == cv::waitKey(30));

	// Warten bis threads ordnungsgemäß beendet sind
	carDetection.stopThread();
	carControlDomi1.stopThread();
	carControlDomi2.stopThread();
	tgroup.join_all();

	return 0;
}

static void onMouse(int event, int x, int y, int, void* param)
{
	if (event != cv::EVENT_LBUTTONDOWN)
		return;

	// Auf Klicks reagieren
	ParamOnMouse* paramOnMouse = (ParamOnMouse*)(param);
	if (x > 20 && x < 200)
	{
		// Referenzpunkte neu berechnen
		if (y > 80 && y < 100)
			paramOnMouse->carDetection->resetRefValue();
		// Kanal 1 schalten
		if (y > 110 && y < 130)
			paramOnMouse->carControlDomi1->toggleDirection();
		// Kanal 2 schalten
		if (y > 140 && y < 160)
			paramOnMouse->carControlDomi2->toggleDirection();
		// Ansicht umschalten
		if (y > 170 && y < 190)
			paramOnMouse->carDetection->ChangePaintMode();
		// Ansicht umschalten
		if (y > 200 && y < 220)
		{
			paramOnMouse->carControlDomi1->ChangeVelocityDirection();
			paramOnMouse->carControlDomi2->ChangeVelocityDirection();
		}
	}
}

