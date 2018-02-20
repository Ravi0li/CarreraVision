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

	// Datei laden
	std::string filename = parser.get<std::string>("trackimg");
	cv::Mat image = cv::imread(filename, 1);
	if (image.empty())
	{
		std::cout << "Cannot read image file: " << filename << std::endl;
		return -1;
	}

	// Strecken auswertung
	TrackDetection trackDetection;
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