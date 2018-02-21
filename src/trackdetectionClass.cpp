#include "trackdetectionClass.h"
#include "debugWinOrganizerClass.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <vector>
#include <iostream>
#include <cmath>
#include <random>

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
	std::vector<cv::KeyPoint> keypoints = calBlobDetection(&workImage);
	calSearchLines(keypoints);

	
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
// Ausführen von Morphologischen Operationen
// --------------------------------------------------------------------------
std::vector<cv::KeyPoint> TrackDetection::calBlobDetection(cv::Mat *image)
{
	// Parameter für BlobDetection
	cv::SimpleBlobDetector::Params params;
	params.minThreshold = 10;
	params.maxThreshold = 30;
	params.filterByArea = false;
	params.filterByCircularity = false;
	params.filterByConvexity = false;
	params.filterByInertia = false;
	params.filterByColor = true;
	params.blobColor = 255;

	// BlobDetection durchführen
	std::vector<cv::KeyPoint> keypoints;
	cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
	//cv::SimpleBlobDetector detector(params);
	detector->detect(*image, keypoints);

	// Textausgabe
	std::cout << "Trackdetection: Es wurden " << keypoints.size() << " Keypoints gefunden" << std::endl;

	// Anzeigen des Ergebnisses
	if (debugWin)
	{
		cv::Mat keypointsImage(image->rows, image->cols, image->type());
		cv::drawKeypoints(keypointsImage, keypoints, keypointsImage, cv::Scalar(255, 255, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
		DebugWinOrganizer::addWindow(keypointsImage, "nach der BlobDetection (Keypoints)");
	}

	return keypoints;
}

// --------------------------------------------------------------------------
// Allgorithmus um die Keypoints zu verbinden
// --------------------------------------------------------------------------
std::vector<std::vector<cv::Point2f>> TrackDetection::calSearchLines(std::vector<cv::KeyPoint> keypoints)
{
	std::vector<std::vector<cv::Point2f>> lines;
	
	// Parameter
	float maxDistance = 160;							// Maximaler Abstand der zwei Punkte zueinander haben darf
	//float maxAngle = 1.5;								// Maximaler Winkel der zum nächsten Punkt auftreten darf (Rad)
	float maxDistancePow = std::pow(maxDistance, 2);	// Einmalig Expotentialwert berechnen zum schnelleren Auswerten

	// Alle Punkte durch gehen
	while (keypoints.size())
	{
		// Erster Punkt der Linie wegspeichern
		cv::KeyPoint lastKeypoint = keypoints.at(0);
		std::vector<cv::Point2f> line;
		line.push_back(lastKeypoint.pt);
		keypoints.erase(keypoints.begin());
		// Alle anderen Keypoints durchgehen und immer der Linie anhängen
		while (keypoints.size())
		{
			std::vector<cv::KeyPoint>::iterator nextKeypoint;
			float bestDistance = maxDistancePow;
			for (std::vector<cv::KeyPoint>::iterator it = keypoints.begin(); it != keypoints.end(); ++it)
			{
				float xDiff = it->pt.x - lastKeypoint.pt.x;
				float yDiff = it->pt.y - lastKeypoint.pt.y;
				float distance = std::pow(xDiff, 2) + std::pow(yDiff, 2);
				if (distance < bestDistance)
				{
					// Neuer nächster Nachbar gefunden
					bestDistance = distance;
					nextKeypoint = it;
				}
			}
			// Kein nächster Nachbar gefunden dann Linie beenden
			if (bestDistance == maxDistancePow)
				break;
			// nächsten Punkt auf der Linie abspeichern
			lastKeypoint = *nextKeypoint;
			line.push_back(nextKeypoint->pt);
			keypoints.erase(nextKeypoint);
		}
		// Linie zu der Linienliste hinzufügen
		lines.push_back(line);
	}

	// Textausgabe
	std::cout << "Trackdetection: Es wurden " << lines.size() << " Linien aus den Keypoints berechnet" << std::endl;
	for(std::vector<cv::Point2f> line : lines)
		std::cout << "Trackdetection: Linie mit " << line.size() << " Punkten erkannt" << std::endl;

	// Anzeigen des Ergebnisses
	if (debugWin)
	{
		cv::Mat lineImage(inputImage.rows, inputImage.cols, inputImage.type(), cv::Scalar(255, 255, 255));
		for (std::vector<cv::Point2f> line : lines)
		{
			// Eine Linie in einer Farbe zeichnen
			cv::Point2f lastPt = cv::Point2f(-1, -1);
			cv::Scalar lineColor = hsvScalar(rand()%180, 255, 255);
			for (cv::Point2f pt : line)
			{
				if (lastPt.x != -1)
					cv::line(lineImage, lastPt, pt, lineColor, 10);
				lastPt = pt;
			}
		}
		DebugWinOrganizer::addWindow(lineImage, "nach der Liniendetektion");
	}

	return lines;
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

// --------------------------------------------------------------------------
// gibt einen Scalar wert zurück anhand von RGB Werten
// --------------------------------------------------------------------------
cv::Scalar TrackDetection::hsvScalar(double h, double s, double v)
{
	cv::Mat rgb;
	cv::Mat hsv(1, 1, CV_8UC3, cv::Scalar(h, s, v));
	cv::cvtColor(hsv, rgb, CV_HSV2RGB);
	return cv::Scalar((int)rgb.at<cv::Vec3b>(0, 0)[0], (int)rgb.at<cv::Vec3b>(0, 0)[1], (int)rgb.at<cv::Vec3b>(0, 0)[2]);
}