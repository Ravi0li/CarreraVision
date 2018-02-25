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
	std::string TrackimgFile = parser.get<std::string>("trackimg");
	cv::Mat image = cv::imread(TrackimgFile, 1);
	if (image.empty())
	{
		std::cout << "Cannot read image file: " << TrackimgFile << std::endl;
		return -1;
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