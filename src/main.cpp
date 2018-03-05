#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>

#include "trackdetectionClass.h"
#include "commandlineparser.h"

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
	trackDetection.calculate();
	image = trackDetection.getResultPicture();
	
	// Anzeigen
	cv::namedWindow("Result", CV_GUI_NORMAL);
	cv::resizeWindow("Result", 700, 500);
	cv::imshow("Result", image);
	cv::waitKey(0);

	return 0;
}
