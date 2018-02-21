#include "trackdetectionClass.h"
#include "debugWinOrganizerClass.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

// --------------------------------------------------------------------------
// Initialisieren
// --------------------------------------------------------------------------
TrackDetection::TrackDetection()
{
	debugWin = false;
}

// --------------------------------------------------------------------------
// Setzen des Streckenbildes
// --------------------------------------------------------------------------
void TrackDetection::setPicture(cv::Mat _inputImage)
{
	inputImage = _inputImage;
}

// --------------------------------------------------------------------------
// Setzen ob alle Zusatzfenster angezeigt werden
// --------------------------------------------------------------------------
void TrackDetection::setDebugWin(bool _debugWin)
{
	debugWin = _debugWin;
}

// --------------------------------------------------------------------------
// Auswertung des Streckenbildes
// --------------------------------------------------------------------------
void TrackDetection::calculate()
{
	cv::Mat workImage;
	workImage = inputImage;

	calHSVRange(&workImage);
	calMorphology(&workImage);
	
	outputImage = workImage;
}

// --------------------------------------------------------------------------
// Rückgabe des Bildes mit allen Auswerteinformationen
// --------------------------------------------------------------------------
cv::Mat TrackDetection::getResultPicture()
{
	return outputImage;
}

// --------------------------------------------------------------------------
// Filtert die Daten anhand der HSV Daten
// --------------------------------------------------------------------------
void TrackDetection::calHSVRange(cv::Mat *image)
{
	// Konvertieren in HSV
	// H: 0 - 180, S: 0 - 255, V: 0 - 255
	cv::cvtColor(*image, *image, CV_RGB2HSV);

	// Anzeigen eines Histogram
	if (debugWin)
		showHistogram(*image, "Historgramm HSV", 0, 0);

	// Range Operationen
	cv::inRange(*image, cv::Scalar(100, 100, 40), cv::Scalar(140, 256, 230), *image);

	// Anzeigen des Ergebnisses
	if (debugWin)
		DebugWinOrganizer::addWindow(*image, "nach HSV-Range-Filter");
}

// --------------------------------------------------------------------------
// Ausführen von Morphologischen Operationen
// --------------------------------------------------------------------------
void TrackDetection::calMorphology(cv::Mat *image)
{
	// Morphologische Operation
	cv::Mat pattern1(5, 5, CV_8U, cv::Scalar(1));
	cv::morphologyEx(*image, *image, cv::MORPH_OPEN, pattern1);
	//cv::Mat pattern2(60, 60, CV_8U, cv::Scalar(1));
	//cv::Mat pattern2 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(21, 21), cv::Point(10, 10));
	//cv::morphologyEx(workImage, workImage, cv::MORPH_CLOSE, pattern2);

	// Anzeigen des Ergebnisses
	if (debugWin)
		DebugWinOrganizer::addWindow(*image, "nach Morphologischen Filter");
}


// --------------------------------------------------------------------------
// Zeigt ein Histogram der Daten an
// --------------------------------------------------------------------------
void TrackDetection::showHistogram(cv::Mat image, std::string title, int posX, int posY)
{
	std::vector<cv::Mat> hsv;
	cv::split(image, hsv);
	int numbins = 256;
	float range[] = { 0, 256 };
	const float* histRange = { range };
	cv::Mat b_hist, g_hist, r_hist;
	cv::calcHist(&hsv[0], 1, 0, cv::Mat(), b_hist, 1, &numbins, &histRange);
	cv::calcHist(&hsv[1], 1, 0, cv::Mat(), g_hist, 1, &numbins, &histRange);
	cv::calcHist(&hsv[2], 1, 0, cv::Mat(), r_hist, 1, &numbins, &histRange);
	int width = 512;
	int height = 300;
	cv::Mat histImage(height, width, CV_8UC3, cv::Scalar(20, 20, 20));
	cv::normalize(b_hist, b_hist, 0, height, cv::NORM_MINMAX);
	cv::normalize(g_hist, g_hist, 0, height, cv::NORM_MINMAX);
	cv::normalize(r_hist, r_hist, 0, height, cv::NORM_MINMAX);
	int binStep = cvRound((float)width / (float)numbins);
	for (int i = 1; i< numbins; i++)
	{
		line(histImage,
			cv::Point(binStep*(i - 1), height - cvRound(b_hist.at<float>(i - 1))),
			cv::Point(binStep*(i), height - cvRound(b_hist.at<float>(i))),
			cv::Scalar(255, 0, 0));
		line(histImage,
			cv::Point(binStep*(i - 1), height - cvRound(g_hist.at<float>(i - 1))),
			cv::Point(binStep*(i), height - cvRound(g_hist.at<float>(i))),
			cv::Scalar(0, 255, 0));
		line(histImage,
			cv::Point(binStep*(i - 1), height - cvRound(r_hist.at<float>(i - 1))),
			cv::Point(binStep*(i), height - cvRound(r_hist.at<float>(i))),
			cv::Scalar(0, 0, 255));
	}

	// Anzeigen im Fenster
	DebugWinOrganizer::addWindow(histImage, title);
}