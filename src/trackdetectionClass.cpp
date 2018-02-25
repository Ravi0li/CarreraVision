#define _USE_MATH_DEFINES
#include "trackdetectionClass.h"
#include "debugWinOrganizerClass.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <vector>
#include <iostream>
#include <cmath>
#include <random>
#include <time.h>
#include <string>

// --------------------------------------------------------------------------
// Initialisieren
// --------------------------------------------------------------------------
TrackDetection::TrackDetection(cv::FileNode _para)
{
	para = _para;
	debugWin = false;
	srand((unsigned int)time(NULL));
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

	clock_t start, finisch;
	start = clock();

	// Alle Verarbeitungsschritte
	calHSVRange(&workImage);
	calMorphology(&workImage);
	std::vector<cv::KeyPoint> keypoints = calBlobDetection(&workImage);
	int counter = 0;
	std::vector<std::vector<cv::Point2f>> lines;
	do {
		counter++;
		if (counter > 5)
		{
			std::cout << "Fehler: Es konnten keine sinvollen Streckenbegrenzungen berechnet werden!" << std::endl;
			outputImage = workImage;
			return;
		}
		lines = calSearchLines(keypoints);
	} while (!calCheckLines(&lines));
	calCreatTrackMask(&workImage, lines);

	finisch = clock();
	std::cout << "Streckenerkennung abgeschlossen (" << finisch - start << "ms)";
	
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
	// Parameter auslesen
	cv::FileNode val = para["hsv_range"];

	// Konvertieren in HSV
	// H: 0 - 180, S: 0 - 255, V: 0 - 255
	cv::cvtColor(*image, *image, CV_RGB2HSV);

	// Anzeigen eines Histogram
	if (debugWin)
		showHistogram(*image, "Historgramm HSV", 0, 0);

	// Range Operationen
	cv::inRange(*image, cv::Scalar((int)val["min_h"], (int)val["min_s"], (int)val["min_v"]), cv::Scalar((int)val["max_h"], (int)val["max_s"], (int)val["max_v"]), *image);

	// Anzeigen des Ergebnisses
	if (debugWin)
		DebugWinOrganizer::addWindow(*image, "nach HSV-Range-Filter");
}

// --------------------------------------------------------------------------
// Ausführen von Morphologischen Operationen
// --------------------------------------------------------------------------
void TrackDetection::calMorphology(cv::Mat *image)
{
	// Parameter auslesen
	cv::FileNode val = para["morphology"];
	
	// Morphologische Operation
	cv::Mat pattern1((int)val["morph_size"], (int)val["morph_size"], CV_8U, cv::Scalar(1));
	cv::morphologyEx(*image, *image, cv::MORPH_OPEN, pattern1);

	// Alternatives Pattern
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
	// Parameter auslesen
	cv::FileNode val = para["blob_detection"];
	
	// Parameter für BlobDetection
	cv::SimpleBlobDetector::Params params;
	params.minThreshold = (float)val["min_threshold"];
	params.maxThreshold = (float)val["max_threshold"];
	params.filterByArea = false;
	params.filterByCircularity = false;
	params.filterByConvexity = false;
	params.filterByInertia = false;
	params.filterByColor = true;
	params.blobColor = (int)val["blob_color"];

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
	// Parameter auslesen
	cv::FileNode val = para["search_lines"];
	
	// Parameter
	std::vector<std::vector<cv::Point2f>> lines;
	float maxDistance = (float)val["max_distance"];		// Maximaler Abstand der zwei Punkte zueinander haben darf
	float maxAngle = (float)val["max_angle"];			// Maximale Winkelabweichung zu geradeaus der zum nächsten Punkt auftreten darf (Rad)
	float maxDistancePow = std::pow(maxDistance, 2);	// Einmalig Expotentialwert berechnen zum schnelleren Auswerten
	float k = -maxDistancePow / std::pow(maxAngle, 2);	// Stauchungsfaktor der Parabel

	// Alle Punkte durch gehen
	while (keypoints.size())
	{
		// Erster Punkt der Linie wegspeichern
		int numKey = rand() % keypoints.size();
		std::vector<cv::Point2f> line;
		line.push_back(keypoints.at(numKey).pt);
		keypoints.erase(keypoints.begin() + numKey);
		float lastAngle = -99;
		bool again = false;
		// In der anderen Richtung die Suche wiederholen
		do
		{
			again = !again;
			// Alle anderen Keypoints durchgehen und immer der Linie anhängen
			while (keypoints.size())
			{
				std::vector<cv::KeyPoint>::iterator nextKeypoint;
				float bestDistance = maxDistancePow;
				float bestAngle;
				for (std::vector<cv::KeyPoint>::iterator it = keypoints.begin(); it != keypoints.end(); ++it)
				{
					// Abstands und Winkelberechnungen
					float xDiff = it->pt.x - line.at(line.size() - 1).x;
					float yDiff = it->pt.y - line.at(line.size() - 1).y;
					float distance = std::pow(xDiff, 2) + std::pow(yDiff, 2);
					float angle = atan2(yDiff, xDiff) + (float)M_PI;
					float difAngle = abs(lastAngle - angle);
					if (difAngle > M_PI) difAngle = 2 * (float)M_PI - difAngle;
					// Prüfen ob Punkt im zugelassenen Winkel und Abstand hat
					float maxDistanceFromFunction = k * pow(difAngle, 2) + maxDistancePow;
					if (lastAngle == -99 || (distance < maxDistanceFromFunction && difAngle < maxAngle))
					{
						// Prüfen ob er den neuen Bestwert hat
						if ((distance) < bestDistance)
						{
							// Neuer nächster Nachbar gefunden
							bestDistance = distance;
							bestAngle = angle;
							nextKeypoint = it;
						}
					}
				}
				// Kein nächster Nachbar gefunden dann Linie beenden
				if (bestDistance == maxDistancePow)
					break;
				// nächsten Punkt auf der Linie abspeichern
				line.push_back(nextKeypoint->pt);
				keypoints.erase(nextKeypoint);
				lastAngle = bestAngle;
			}
			// Reinfolge der Punkte rückwärts setzen und dort weiter nach Punkten suchen
			std::reverse(line.begin(), line.end());
			lastAngle = -99;
			if (line.size() >= 2)
			{
				float xDiff = line.at(line.size() - 1).x - line.at(line.size() - 2).x;
				float yDiff = line.at(line.size() - 1).y - line.at(line.size() - 2).y;
				lastAngle = atan2(yDiff, xDiff) + (float)M_PI;
			}
		} while (again);
		// Linie zu der Linienliste hinzufügen
		lines.push_back(line);
	}

	// Textausgabe
	std::cout << "Trackdetection: Es wurden " << lines.size() << " Linien aus den Keypoints berechnet" << std::endl;
	int counter = 0;
	for (std::vector<cv::Point2f> line : lines)
	{
		counter++;
		std::cout << "Trackdetection: Linie " << counter << " mit " << line.size() << " Punkten erkannt" << std::endl;
	}

	// Anzeigen des Ergebnisses
	if (debugWin)
	{
		cv::Mat lineImage(inputImage.rows, inputImage.cols, inputImage.type(), cv::Scalar(255, 255, 255));
		int counter = 0;
		for (std::vector<cv::Point2f> line : lines)
		{
			counter++;
			// Eine Linie in einer Farbe zeichnen
			cv::Point2f lastPt = cv::Point2f(-1, -1);
			cv::Scalar lineColor = hsvScalar(rand()%180, 255, 255);
			for (cv::Point2f pt : line)
			{
				if (lastPt.x != -1)
					cv::line(lineImage, lastPt, pt, lineColor, 10);
				else
				{
					// Beschriftung zeichnen
					cv::putText(lineImage, std::to_string(counter), cv::Point((int)pt.x, (int)pt.y), cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 2, lineColor, 5);
				}
				lastPt = pt;
			}
		}
		DebugWinOrganizer::addWindow(lineImage, "nach der Liniendetektion");
	}

	return lines;
}

// --------------------------------------------------------------------------
// Prüft ob es zwei Linien gibt die sinvoll eine Strecke umschliesen
// --------------------------------------------------------------------------
bool TrackDetection::calCheckLines(std::vector<std::vector<cv::Point2f>> *lines)
{
	// Parameter auslesen
	cv::FileNode val = para["search_lines"];

	// Parameter
	float maxDistance = (float)val["max_distance"];		// Maximaler Abstand der zwei Punkte zueinander haben darf
	float maxAngle = (float)val["max_angle"];			// Maximale Winkelabweichung zu geradeaus der zum nächsten Punkt auftreten darf (Rad)
	float maxDistancePow = std::pow(maxDistance, 2);	// Einmalig Expotentialwert berechnen zum schnelleren Auswerten
	float k = -maxDistancePow / std::pow(maxAngle, 2);	// Stauchungsfaktor der Parabel

	// Alle Linien durchgehen
	for (std::vector<std::vector<cv::Point2f>>::iterator line = lines->begin(); line != lines->end();)
	{
		// Eine Sinnvolle Linie hat min. 10 Punkte
		if (line->size() > 10)
		{
			// Berechnung aller Distanzen und Winkel
			float xDiff = line->at(0).x - line->at(line->size() - 1).x;
			float yDiff = line->at(0).y - line->at(line->size() - 1).y;
			float distance = std::pow(xDiff, 2) + std::pow(yDiff, 2);
			xDiff = line->at(1).x - line->at(0).x;
			yDiff = line->at(1).y - line->at(0).y;
			float angle1 = atan2(yDiff, xDiff) + (float)M_PI;
			xDiff = line->at(line->size() - 1).x - line->at(line->size() - 2).x;
			yDiff = line->at(line->size() - 1).y - line->at(line->size() - 2).y;
			float angle2 = atan2(yDiff, xDiff) + (float)M_PI;
			float difAngle = abs(angle1 - angle2);
			if (difAngle > (float)M_PI) difAngle = 2 * (float)M_PI - difAngle;
			float maxDistanceFromFunction = k * pow(difAngle, 2) + maxDistancePow;
			// Prüfen ob auch Verbindugslinie passt
			if (distance < maxDistanceFromFunction && difAngle < maxAngle)
				++line;
			else
				line = lines->erase(line);
		}
		else
		{
			line = lines->erase(line);
		}
	}

	// Prüfen ob genau zwei Linien übrig bleiben
	if (lines->size() == 2)
	{
		std::cout << "Es konnten zwei sinvolle Linien berechnet werden" << std::endl;
		return true;
	}
	else
	{
		std::cout << "Warnung: Es konnten keine zwei sinvolle Linien berechnet werden" << std::endl;
		return false;
	}
}

// --------------------------------------------------------------------------
// Erstellt die Maske für die Strecke
// --------------------------------------------------------------------------
void TrackDetection::calCreatTrackMask(cv::Mat *image, std::vector<std::vector<cv::Point2f>> lines)
{
	*image = cv::Scalar(0, 0, 0);

	std::vector<cv::Point> paintVec1;
	for (int i = 0; i < lines.at(0).size(); i++)
	{
		cv::Point p = lines.at(0)[i];
		paintVec1.push_back(p);
	}
	std::vector<cv::Point> paintVec2;
	for (int i = 0; i < lines.at(1).size(); i++)
	{
		cv::Point p = lines.at(1)[i];
		paintVec2.push_back(p);
	}
	std::vector<std::vector<cv::Point>> paintVecs;
	paintVecs.push_back(paintVec1);
	paintVecs.push_back(paintVec2);
	cv::fillPoly(*image, paintVecs, cv::Scalar(255, 255, 255));


	/*
	for (int j = 0; j < lines.at(0).size(); j++)
	{
		cv::line(*image, lines.at(0).at(j), lines.at(0).at((j+1)%lines.at(0).size()), cv::Scalar(255, 0, 0), 3, 8);
	}
	for (int j = 0; j < lines.at(1).size(); j++)
	{
		cv::line(*image, lines.at(1).at(j), lines.at(1).at((j + 1) % lines.at(1).size()), cv::Scalar(255, 0, 0), 3, 8);
	}*/
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