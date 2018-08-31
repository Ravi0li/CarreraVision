#define _USE_MATH_DEFINES
#include "trackdetectionClass.h"
#include "debugWinOrganizerClass.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <vector>			// stc::vector
#include <iostream>			// std::cout
#include <cmath>			// atan2()
#include <random>			// rand()
#include <time.h>			// time()
#include <string>			// toString()
#include <algorithm>		// std::min_element() std::max_element()

// --------------------------------------------------------------------------
// Initialisieren
// --------------------------------------------------------------------------
TrackDetection::TrackDetection(cv::FileNode _para)
{
	para = _para;
	debugWin = false;
	unitTestPic = false;
	srand((unsigned int)time(NULL));
}

// --------------------------------------------------------------------------
// Setzen des Streckenbildes
// --------------------------------------------------------------------------
void TrackDetection::setPicture(cv::Mat _inputImage)
{
	_inputImage.copyTo(inputImage);
}

// --------------------------------------------------------------------------
// Setzen ob alle Zusatzfenster angezeigt werden
// --------------------------------------------------------------------------
void TrackDetection::setDebugWin(bool _debugWin)
{
	debugWin = _debugWin;
}

// --------------------------------------------------------------------------
// Setzen ob alle Zusatzfenster angezeigt werden
// --------------------------------------------------------------------------
void TrackDetection::setUnitTestPic(bool _unitTestPic)
{
	unitTestPic = _unitTestPic;
}

// --------------------------------------------------------------------------
// Auswertung des Streckenbildes
// --------------------------------------------------------------------------
bool TrackDetection::calculate(float pointDistanceInMeter)
{
	cv::Mat binaryImage;
	inputImage.copyTo(binaryImage);

	clock_t start, finisch;
	start = clock();

	// Alle Verarbeitungsschritte abarbeiten
	calHSVRange(&binaryImage);
	calMorphology(&binaryImage);
	std::vector<cv::KeyPoint> keypoints = calBlobDetection(&binaryImage);
	int counter = 0;
	std::vector<std::vector<cv::Point2f>> lines;
	do {
		counter++;
		if (counter > 10)
		{
			std::cout << "Fehler: Es konnten keine sinvollen Streckenbegrenzungen berechnet werden!" << std::endl;
			binaryImage.copyTo(outputImage);
			return false;
		}
		lines = calSearchLinesStraight(keypoints);
		lines = calSearchLinesCurved(lines);
	} while (!calCheckLines(&lines));
	if (unitTestPic)
		cv::putText(unitTestPic2, "Wiederholung: " + std::to_string(counter), cv::Point(20, 50), cv::FONT_HERSHEY_PLAIN, 4, cv::Scalar(0,0,0), 3);
	calCreatTrackMask(lines);
	if (!calLanes(lines, pointDistanceInMeter)) return false;

	finisch = clock();
	std::cout << "Streckenerkennung abgeschlossen (" << finisch - start << "ms)" << std::endl;
	
	return true;
}

// --------------------------------------------------------------------------
// Rückgabe des Bildes mit allen Auswerteinformationen
// --------------------------------------------------------------------------
cv::Mat TrackDetection::getResultPicture()
{
	return outputImage;
}

// --------------------------------------------------------------------------
// Rückgabe der Bildmaske zum Ausblenden von Teilen
// --------------------------------------------------------------------------
cv::Mat TrackDetection::getMaskPicture()
{
	return maskImage;
}

// --------------------------------------------------------------------------
// Gibt die Punktereihe zurück
// --------------------------------------------------------------------------
void TrackDetection::getPointLines(std::vector<cv::Point2f> *lane1, std::vector<cv::Point2f> *lane2)
{
	*lane1 = lane1r;
	*lane2 = lane2r;
}

// --------------------------------------------------------------------------
// gibt alle Zwischenbilder für den Unit-test zurück
// --------------------------------------------------------------------------
void TrackDetection::getUnitTestPic(cv::Mat *pic1, cv::Mat *pic2, cv::Mat *pic3, cv::Mat *pic4)
{
	*pic1 = unitTestPic1;
	*pic2 = unitTestPic2;
	*pic3 = unitTestPic3;
	*pic4 = unitTestPic4;
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

	// Range Funktion mit Erweiterung
	//cv::inRange(*image, cv::Scalar(100, 100, 40), cv::Scalar(140, 256, 230), *image);
	//cv::inRange(*image, cv::Scalar((int)val["min_h"], (int)val["min_s"], (int)val["min_v"]), cv::Scalar((int)val["max_h"], (int)val["max_s"], (int)val["max_v"]), *image);
	struct Operator
	{
		void operator ()(cv::Vec3b &pixel, const int * position) const
		{
			int minS = (int)(180 - 0.7 * pixel[2]);
			if (pixel[0] > 115 && pixel[0] < 145 && pixel[1] > minS)
				pixel[2] = pixel[1] = pixel[0] = 255;
			else
				pixel[2] = pixel[1] = pixel[0] = 0;
		}
	};
	image->forEach<cv::Vec3b>(Operator());

	// Anzeigen des Ergebnisses
	if (debugWin)
		DebugWinOrganizer::addWindow(*image, "nach HSV-Range-Filter");
}

// --------------------------------------------------------------------------
// Code um per klick HSV-Werte auszulesen
// --------------------------------------------------------------------------
/*cv::Mat *tImg;
tImg = new cv::Mat(*image);
cv::namedWindow("HSV", CV_GUI_NORMAL);
cv::resizeWindow("HSV", 700, 500);
cv::imshow("HSV", *tImg);
setMouseCallback("HSV", onMouse, tImg);
static void onMouse(int event, int x, int y, int, void* userInput)
{
	if (event != cv::EVENT_LBUTTONDOWN)
		return;
	cv::Mat* img = (cv::Mat*)userInput;
	cv::Scalar color = img->at<cv::Vec3b>(cv::Point(x, y));
	std::cout << "H: " << color.val[0] << "     S: " << color.val[1] << "     V: " << color.val[2] << std::endl;
}*/

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
	if (unitTestPic)
		image->copyTo(unitTestPic1);
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
	detector->detect(*image, keypoints);

	// Textausgabe
	std::cout << "Trackdetection: Es wurden " << keypoints.size() << " Keypoints gefunden" << std::endl;

	// Prüfen nach Verschmolzenen Punkten (doppelt große Blobs)
	calBlobDetectionMeldedPoints(&keypoints);

	// Anzeigen des Ergebnisses
	if (debugWin)
	{
		cv::Mat keypointsImage(image->rows, image->cols, image->type(), cv::Scalar(0, 0, 0));
		cv::drawKeypoints(keypointsImage, keypoints, keypointsImage, cv::Scalar(255, 255, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
		DebugWinOrganizer::addWindow(keypointsImage, "nach der BlobDetection (Keypoints)");
	}

	return keypoints;
}

// --------------------------------------------------------------------------
// Prüfen nach Verschmolzenen Punkten (doppelt große Blobs)
// --------------------------------------------------------------------------
void TrackDetection::calBlobDetectionMeldedPoints(std::vector<cv::KeyPoint> *keypoints)
{
	if (keypoints->size() <= 2)
		return;

	// Parameter auslesen
	cv::FileNode val = para["blob_detection"];

	// Größten Punkt suchen
	cv::KeyPoint maxSizePoint = *std::max_element(keypoints->begin(), keypoints->end(), [](cv::KeyPoint a, cv::KeyPoint b) { return a.size < b.size; });
	int max = (int)(maxSizePoint.size) + 1;
	// Histogramm aller Punktgrößen erstellen
	std::vector<int> hist;
	for (int i = 0; i < max; i++)
		hist.push_back(0);
	for (cv::KeyPoint p : *keypoints)
		hist[(int)(p.size)]++;
	// Häufigsten Wert finden
	int maxPos = (int)distance(hist.begin(), std::max_element(hist.begin(), hist.end()));
	// Suchen nach Doppelt großen Punkten
	float rangePosMin = maxPos * (float)val["double_points_min_faktor"];
	float rangePosMax = maxPos * (float)val["double_points_max_faktor"];
	std::cout << "Es wird nach verschmolzenen Puntken gesucht" << std::endl;
	std::cout << "   Min: " << rangePosMin << "     Max : " << rangePosMax << std::endl;
	for (cv::KeyPoint& p : *keypoints)
	{
		if (p.size > rangePosMin && p.size < rangePosMax)
		{
			std::cout << "Verschmolzener Punkt entdeckt (size:" << p.size << ")" << std::endl;
			// Wenn ein Doppelt großer Punkt gefunden wurde einfach noch einmal zur Liste hinzufügen
			p.response = 1;
			cv::KeyPoint newPoint(p);
			newPoint.size = 0;
			newPoint.response = 1;
			keypoints->push_back(newPoint);
		}
	}
}

// --------------------------------------------------------------------------
// Allgorithmus um die Keypoints zu verbinden aber nur gerade Linien
// --------------------------------------------------------------------------
std::vector<std::vector<cv::Point2f>> TrackDetection::calSearchLinesStraight(std::vector<cv::KeyPoint> keypoints)
{
	// Parameter auslesen
	cv::FileNode val = para["search_lines_straight"];

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
		cv::KeyPoint lastPoint;
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
					bool samePoint = (it->pt.x == line.at(line.size() - 1).x) && (it->pt.y == line.at(line.size() - 1).y);
					float xDiff = it->pt.x - line.at(line.size() - 1).x;
					float yDiff = it->pt.y - line.at(line.size() - 1).y;
					float distance = std::pow(xDiff, 2) + std::pow(yDiff, 2);
					float angle = atan2(yDiff, xDiff) + (float)M_PI;
					float difAngle = abs(lastAngle - angle);
					if (difAngle > M_PI) difAngle = 2 * (float)M_PI - difAngle;
					if (!samePoint && (lastAngle == -99 || (distance < maxDistancePow && difAngle < maxAngle)))
					{
						// Prüfen ob er den neuen Bestwert hat
						if (distance < bestDistance)
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
				lastPoint = *nextKeypoint;
				keypoints.erase(nextKeypoint);
				lastAngle = bestAngle;
			}
			// Reinfolge der Punkte rückwärts setzen und dort weiter nach Punkten suchen
			std::reverse(line.begin(), line.end());
			lastAngle = -99;
			lastPoint = cv::KeyPoint();
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

	// Suchen nach Linien mit nur zwei oder drei Punkten und löse diese wieder auf
	std::vector<std::vector<cv::Point2f>> newLines;
	for (std::vector<std::vector<cv::Point2f>>::iterator it = lines.begin(); it != lines.end(); ++it)
	{
		// Alle Linien mit weniger als fünf Punkten auflösen
		if (it->size() < 5)
		{
			// Für alle Punkt eine eigene Linie anlegen
			for (std::vector<cv::Point2f>::iterator it2 = it->begin() + 1; it2 != it->end();)
			{
				std::vector<cv::Point2f> newLine;
				newLine.push_back(*it2);
				newLines.push_back(newLine);
				it->erase(it2);
				it2 = it->begin() + 1;
			}
		}
	}
	lines.insert(lines.end(), newLines.begin(), newLines.end());

	// Anzeigen des Ergebnisses
	if (debugWin || unitTestPic)
	{
		cv::Mat lineImage(inputImage.rows, inputImage.cols, inputImage.type(), cv::Scalar(255, 255, 255));
		int counter = 0;
		for (std::vector<cv::Point2f> line : lines)
		{
			counter++;
			// Eine Linie in einer Farbe zeichnen
			cv::Point2f lastPt = cv::Point2f(-1, -1);
			cv::Scalar lineColor = hsvScalar(rand() % 180, 255, 255);
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
		if (debugWin)
			DebugWinOrganizer::addWindow(lineImage, "nach der Liniendetektion-Gerade");
		if (unitTestPic)
			lineImage.copyTo(unitTestPic2);
	}

	return lines;
}

// --------------------------------------------------------------------------
// Allgorithmus um die Keypoints zu verbinden
// --------------------------------------------------------------------------
std::vector<std::vector<cv::Point2f>> TrackDetection::calSearchLinesCurved(std::vector<std::vector<cv::Point2f>> inLines)
{
	// Parameter auslesen
	cv::FileNode val = para["search_lines_curved"];
	
	// Parameter
	std::vector<std::vector<cv::Point2f>> lines;
	float maxDistance = (float)val["max_distance"];		// Maximaler Abstand der zwei Punkte zueinander haben darf
	float maxAngle = (float)val["max_angle"];			// Maximale Winkelabweichung zu geradeaus der zum nächsten Punkt auftreten darf (Rad)
	float maxDistancePow = std::pow(maxDistance, 2);	// Einmalig Expotentialwert berechnen zum schnelleren Auswerten
	float k = -maxDistancePow / std::pow(maxAngle, 2);	// Stauchungsfaktor der Parabel

	// Alle Geraden Linien bzw einzelnen Punkte durch gehen
	while (inLines.size())
	{
		// Erster Punkt der Linie wegspeichern
		int numKey = rand() % inLines.size();
		std::vector<cv::Point2f> line = inLines[numKey];
		inLines.erase(inLines.begin() + numKey);
		float lastAngle = -99;
		if (line.size() > 1)
		{
			int la = (int)line.size() - 1;
			float xDiff = line[la].x - line[la-1].x;
			float yDiff = line[la].y - line[la - 1].y;
			lastAngle = atan2(yDiff, xDiff) + (float)M_PI;
		}
		cv::Point2f lastPoint;
		bool again = false;
		// In der anderen Richtung die Suche wiederholen
		do
		{
			again = !again;
			// Alle anderen Keypoints durchgehen und immer der Linie anhängen
			while (inLines.size())
			{
				std::vector<std::vector<cv::Point2f>>::iterator nextLine;
				bool reverseCopy;
				float bestDistance2 = maxDistancePow;
				float bestAngle;
				for (std::vector<std::vector<cv::Point2f>>::iterator it = inLines.begin(); it != inLines.end(); ++it)
				{
					// Abstands und Winkelberechnungen für ersten Punkt der Linie
					bool samePoint = ((*it)[0].x == line.at(line.size() - 1).x) && ((*it)[0].y == line.at(line.size() - 1).y);
					float xDiff = (*it)[0].x - line.at(line.size() - 1).x;
					float yDiff = (*it)[0].y - line.at(line.size() - 1).y;
					float distance = std::pow(xDiff, 2) + std::pow(yDiff, 2);
					float angle = atan2(yDiff, xDiff) + (float)M_PI;
					float difAngle = abs(lastAngle - angle);
					if (difAngle > M_PI) difAngle = 2 * (float)M_PI - difAngle;
					float w = 1 + abs(difAngle)*2;
					if (w > 3) w = 3;
					float distance2 = distance * pow(w,2);
					// Prüfen ob Punkt im zugelassenen Winkel und Abstand hat
					float maxDistanceFromFunction = k * pow(difAngle, 2) + maxDistancePow;
					if (!samePoint && (lastAngle == -99 || (distance < maxDistanceFromFunction && difAngle < maxAngle)))
					{
						// Prüfen ob er den neuen Bestwert hat
						if (distance2 < bestDistance2)
						{
							// Neuer nächster Nachbar gefunden
							bestDistance2 = distance2;
							bestAngle = angle;
							nextLine = it;
							reverseCopy = false;
						}
					}
					// Abstands und Winkelberechnungen für letzten Punkt der Linie
					if (it->size() > 1)
					{
						int la = (int)it->size() - 1;
						bool samePoint = ((*it)[la].x == line.at(line.size() - 1).x) && ((*it)[la].y == line.at(line.size() - 1).y);
						float xDiff = (*it)[la].x - line.at(line.size() - 1).x;
						float yDiff = (*it)[la].y - line.at(line.size() - 1).y;
						float distance = std::pow(xDiff, 2) + std::pow(yDiff, 2);
						float angle = atan2(yDiff, xDiff) + (float)M_PI;
						float difAngle = abs(lastAngle - angle);
						if (difAngle > M_PI) difAngle = 2 * (float)M_PI - difAngle;
						float w = 1 + abs(difAngle) * 2;
						if (w > 3) w = 3;
						float distance2 = distance * pow(w, 2);
						// Prüfen ob Punkt im zugelassenen Winkel und Abstand hat
						float maxDistanceFromFunction = k * pow(difAngle, 2) + maxDistancePow;
						if (!samePoint && (lastAngle == -99 || (distance < maxDistanceFromFunction && difAngle < maxAngle)))
						{
							// Prüfen ob er den neuen Bestwert hat
							if (distance2 < bestDistance2)
							{
								// Neuer nächster Nachbar gefunden
								bestDistance2 = distance2;
								bestAngle = angle;
								nextLine = it;
								reverseCopy = true;
							}
						}
					}
				}
				// Kein nächster Nachbar gefunden dann Linie beenden
				if (bestDistance2 == maxDistancePow)
					break;
				// nächsten Punkt auf der Linie abspeichern
				if (reverseCopy)
					std::reverse(nextLine->begin(), nextLine->end());
				line.insert(line.end(), nextLine->begin(), nextLine->end());
				lastPoint = line.back();
				inLines.erase(nextLine);
				lastAngle = bestAngle;
			}
			// Reinfolge der Punkte rückwärts setzen und dort weiter nach Punkten suchen
			std::reverse(line.begin(), line.end());
			lastAngle = -99;
			lastPoint = cv::Point2f();
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
	if (debugWin || unitTestPic)
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
		if (debugWin)
			DebugWinOrganizer::addWindow(lineImage, "nach der Liniendetektion-Kurven");
		if (unitTestPic)
			lineImage.copyTo(unitTestPic2);
	}

	return lines;
}

// --------------------------------------------------------------------------
// Prüft ob es zwei Linien gibt die sinvoll eine Strecke umschliesen
// --------------------------------------------------------------------------
bool TrackDetection::calCheckLines(std::vector<std::vector<cv::Point2f>> *lines)
{
	// Parameter auslesen
	cv::FileNode val = para["search_lines_checkends"];

	// Parameter
	float maxDistance = (float)val["max_distance"];		// Maximaler Abstand der zwei Punkte zueinander haben darf
	float maxAngle = (float)val["max_angle"];			// Maximale Winkelabweichung zu geradeaus der zum nächsten Punkt auftreten darf (Rad)
	float maxSafeDistance = (float)val["safe_distance"];// Abstand in dem der Winkel egal ist
	float maxDistancePow = std::pow(maxDistance, 2);	// Einmalig Expotentialwert berechnen zum schnelleren Auswerten
	float maxSafeDistancePow = std::pow(maxSafeDistance, 2);
	float k = -maxDistancePow / std::pow(maxAngle, 2);	// Stauchungsfaktor der Parabel

	// Alle Linien durchgehen
	for (std::vector<std::vector<cv::Point2f>>::iterator line = lines->begin(); line != lines->end();)
	{
		// Eine Sinnvolle Linie hat min. 10 Punkte
		if (line->size() > 25)
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
			while (difAngle > (float)M_PI) difAngle = 2 * (float)M_PI - difAngle;
			float maxDistanceFromFunction = k * pow(difAngle, 2) + maxDistancePow;
			// Prüfen ob auch Verbindugslinie passt
			if (distance < maxDistanceFromFunction && difAngle < maxAngle || distance < maxSafeDistancePow)
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
void TrackDetection::calCreatTrackMask(std::vector<std::vector<cv::Point2f>> lines)
{
	// umkopieren der Vektoren in das richtige Format
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

	// Zeichnen der Maske
	maskImage.create(inputImage.rows, inputImage.cols, CV_8U);
	maskImage.setTo(cv::Scalar(0));
	cv::fillPoly(maskImage, paintVecs, cv::Scalar(255));

	// Debugfenster anzeigen
	if (debugWin)
		DebugWinOrganizer::addWindow(maskImage, "Generierte Maske zum Ausblenden");

	// Anwenden der Maske auf das Ausgabebild
	inputImage.copyTo(outputImage, maskImage);
}

// --------------------------------------------------------------------------
// Berechnet die Fahrspuren und zeichnet sie ein
// --------------------------------------------------------------------------
bool TrackDetection::calLanes(std::vector<std::vector<cv::Point2f>> lines, float pointDistanceInMeter)
{
	// Die zwei auszuwertenden Spuren teilen
	std::vector<cv::Point2f> line1, line2;
	linesDir line1dir, line2dir;
	line1 = lines[0];
	line2 = lines[1];

	// Ausrichtung der Seitenlinie zur Fahrspur analysieren
	if (!calLanesSideDirection(&line1, &line2, &line1dir, &line2dir)) return false;

	// Querlinien zur Fahrspurmittelung auslesen
	std::vector<std::pair<cv::Point2f, cv::Point2f>> crosslines;
	calLanesCrossLines(line1, line2, line1dir, &crosslines, false);
	calLanesCrossLines(line2, line1, line2dir, &crosslines, true);

	// Aussortieren von zu kleinen und großen Crosslines
	calLanesCrossLinesFilter(&crosslines, pointDistanceInMeter);

	// Spurenlinien berechnen (unregelmäsige Abstände)
	std::vector<cv::Point2f> lane1i, lane2i;
	calLanesIrregular(&lane1i, &lane2i, crosslines);

	// Spurenlinien berechnen (regelmäsige Abstände)
	calLanesRegular(lane1i, lane2i);

	return true;
}

// --------------------------------------------------------------------------
// Analysiert ob die Fahrspur rechts oder Links von den Linien liegen
// --------------------------------------------------------------------------
bool TrackDetection::calLanesSideDirection(std::vector<cv::Point2f> *line1, std::vector<cv::Point2f> *line2, linesDir *line1dir, linesDir *line2dir)
{
	cv::Mat lineDirectionImage;
	if (debugWin)
		outputImage.copyTo(lineDirectionImage);
	
	// Ausrichtung der ersten Linie zur Fahrspur analysieren
	int count = 0;
	bool left, right;
	int line1id;
	cv::Point2f line1Vec;
	bool trySwitch = true;
	do {
		count++;
		// Zzfälligen Punkt wählen
		line1id = rand() % (line1->size() - 1);
		// Vector und Punkt berechnen
		cv::Point2f vector, point;
		vector.x = (*line1)[line1id + 1].x - (*line1)[line1id].x;
		vector.y = (*line1)[line1id + 1].y - (*line1)[line1id].y;
		point.x = ((*line1)[line1id + 1].x + (*line1)[line1id].x) * 0.5f;
		point.y = ((*line1)[line1id + 1].y + (*line1)[line1id].y) * 0.5f;
		// Alle Punkte auf der anderen Gerade durchgehen und schauen auf welcher Seite sie von dem Vektor liegen
		left = false;
		right = false;
		for (int i = 0; i < line2->size(); i++)
		{
			float r = vector.y * ((*line2)[i].x - point.x) - vector.x * ((*line2)[i].y - point.y);
			if (r > 0) right = true;
			if (r < 0) left = true;
		}
		// Ist die Ausrichtung an dieser Stelle eindeutig?
		if (right && !left || !right && left)
		{
			float t = vector.x;
			vector.x = vector.y;
			vector.y = t;
			if (right) vector.y = -vector.y;
			if (left) vector.x = -vector.x;
			line1Vec.x = vector.x;
			line1Vec.y = vector.y;
			if (debugWin)
			{
				cv::Point2f midPoint2(point.x + vector.x, point.y + vector.y);
				cv::line(lineDirectionImage, point, midPoint2, cv::Scalar(255,0,0), 10);
			}
		}
		// Nach 50 Punkten sollte ein eindeutiger Punkt dabei sein
		if (count == 50)
		{
			// Evtl, wurde die innere Bahn zur Analyse verwendet, probieren mal die andere aus
			if (trySwitch)
			{
				trySwitch = false;
				std::vector<cv::Point2f> lineTemp;
				lineTemp = *line1;
				*line1 = *line2;
				*line2 = lineTemp;
				count = 0;
			}
			else
			{
				std::cout << "Fehler: Auf der 1. Linie konnte kein eindeutiger Richtung zur Fahrspuhrenmittelung gefunden werden." << std::endl;
				return false;
			}
		}
	} while (!(right && !left || !right && left));
	if (right) *line1dir = RIGHT;
	if (left) *line1dir = LEFT;
	std::cout << "Linienausrichtung der aeusseren Spur zur Fahrspur erfolgreich erkannt" << std::endl;

	// Ausrichtung der zweiten Linie zur Fahrspur erkennen
	int id;
	float bestDistance = 3E+38f;
	// Prüfen welches der nächste Punkt ist
	for (int i = 0; i < line2->size() - 1; i++)
	{
		float distance = pow((*line2)[i].x - (*line1)[line1id].x, 2) + pow((*line2)[i].y - (*line1)[line1id].y, 2);
		if (bestDistance > distance)
		{
			bestDistance = distance;
			id = i;
		}
	}
	// Ausrichtung zur ersten Linie prüfen
	cv::Point2f vector, point;
	vector.x = (*line2)[id + 1].x - (*line2)[id].x;
	vector.y = (*line2)[id + 1].y - (*line2)[id].y;
	point.x = ((*line2)[id + 1].x + (*line2)[id].x) * 0.5f;
	point.y = ((*line2)[id + 1].y + (*line2)[id].y) * 0.5f;
	// Vektor nach rechts drehen
	float t = vector.x;
	vector.x = vector.y;
	vector.y = t;
	vector.y = -vector.y;
	// Prüfen ob drehung sinvoll und dann abspeichern
	float lenghtAdd = pow(line1Vec.x + vector.x, 2) + pow(line1Vec.y + vector.y, 2);
	float lenghtSub = pow(line1Vec.x - vector.x, 2) + pow(line1Vec.y - vector.y, 2);
	if (lenghtAdd > lenghtSub) *line2dir = LEFT;
	else                       *line2dir = RIGHT;
	// Debuganzeige
	if (debugWin && *line2dir == RIGHT)
	{
		cv::Point2f midPoint2(point.x + vector.x, point.y + vector.y);
		cv::line(lineDirectionImage, point, midPoint2, cv::Scalar(0, 0, 255), 10);
	}
	else if (debugWin)
	{
		cv::Point2f midPoint2(point.x - vector.x, point.y - vector.y);
		cv::line(lineDirectionImage, point, midPoint2, cv::Scalar(0, 0, 255), 10);
	}
	if (debugWin)
		DebugWinOrganizer::addWindow(lineDirectionImage, "Ausrichtung der Seitenlinien");

	return 1;
}

// --------------------------------------------------------------------------
// Gibt alle Querlinien zur auswertung der Fahrspuren zurück
// In dieser Funktion wird viel mit double gearbeitet, da bei senkrechten und
// wagerechten Crosslines Werte naha 0 raus kommen. Hire wird die doppelte
// Genauigkeit benötigt.
// --------------------------------------------------------------------------
void TrackDetection::calLanesCrossLines(std::vector<cv::Point2f> baseLines, std::vector<cv::Point2f> targetLines, linesDir baseDir, std::vector<std::pair<cv::Point2f, cv::Point2f>> *crosslines, bool invertLines)
{
	cv::Mat crossLinesImage;
	if (debugWin)
		outputImage.copyTo(crossLinesImage);
	
	// Alle Abschnitte der Linie durchgehen
	for (int i = 0; i < baseLines.size(); i++)
	{
		// Immer zwei Punkte aus der Linienfolge nehmen
		cv::Point2d l1p1, l1p2;
		l1p1 = baseLines[i];
		if (i < baseLines.size() - 1)
			l1p2 = baseLines[i + 1];
		else
			l1p2 = baseLines[0];
		// Berechnen des Mittelpunktes und des Vektors
		cv::Point2d vector, point;
		vector.x = l1p2.x - l1p1.x;
		vector.y = l1p2.y - l1p1.y;
		point.x = (l1p1.x + l1p2.x) * 0.5;
		point.y = (l1p1.y + l1p2.y) * 0.5;
		// Vektor drehen
		double t = vector.x;
		vector.x = vector.y;
		vector.y = t;
		if (baseDir == RIGHT) vector.y = -vector.y;
		if (baseDir == LEFT) vector.x = -vector.x;
		// zwei Punkte auf der Senkrechten speichern
		l1p1 = point;
		l1p2 = point + vector;
		// Prüfen welche Strecke sich von der anderen Linienfolge mit der Senkrechten schneidet
		double bestDistance = 3E+38f;
		cv::Point2d bestPoint(-1, -1);
		for (int i = 0; i < targetLines.size(); i++)
		{
			cv::Point2d l2p1, l2p2;
			l2p1 = targetLines[i];
			if (i < targetLines.size() - 1)
				l2p2 = targetLines[i + 1];
			else
				l2p2 = targetLines[0];
			// Schnittpunkt berechnen beider Geraden
			cv::Point2d stp;
			stp.x = (-l1p2.x*l2p1.x*l1p1.y + l1p2.x*l2p2.x*l1p1.y + l1p1.x*l2p1.x*l1p2.y - l1p1.x*l2p2.x*l1p2.y + l1p1.x*l2p2.x*l2p1.y - l1p2.x*l2p2.x*l2p1.y - l1p1.x*l2p1.x*l2p2.y + l1p2.x*l2p1.x*l2p2.y) /
				(-l2p1.x*l1p1.y + l2p2.x*l1p1.y + l2p1.x*l1p2.y - l2p2.x*l1p2.y + l1p1.x*l2p1.y - l1p2.x*l2p1.y - l1p1.x*l2p2.y + l1p2.x*l2p2.y);
			stp.y = (l2p2.y - l2p1.y) / (l2p2.x - l2p1.x) * (stp.x - l2p1.x) + l2p1.y;
			// Breite der CrossLine berechnen
			double distance = pow(stp.x - point.x, 2) + pow(stp.y - point.y, 2);
			// Prüfen ob der Schnittpunkt auch auf der Strecke zwischen den Punkten liegt
			double vecFac2 = (stp.x - l2p1.x) / (l2p2.x - l2p1.x);
			double vecFac1 = (stp.x - point.x) / (vector.x);
			// Winkel auf zweite Linie berechnen, dieser sollte fast senkrecht sein
			double diffAngle = angleDiffFromVec(stp, point, stp, l2p1);
			// Wenn alles erfüllt, dann als neusten nächsten Schnittpunkt merken
			if (bestDistance > distance && vecFac2 >= 0 && vecFac2 <= 1 && vecFac1 > 0 && diffAngle > 1.38 && diffAngle < 1.78)
			{
				bestDistance = distance;
				bestPoint = stp;
			}
		}
		// Wenn eine Sinvolle Crossline gefunden wurde abspeichern
		if (bestPoint.x != -1)
		{
			std::pair<cv::Point2f, cv::Point2f> crossline;
			// Ausrichtung der Crossline beachten damit am ende alle in die selbe Richtung gehen
			if (invertLines)
			{
				crossline.first = bestPoint;
				crossline.second = point;
			}
			else
			{
				crossline.first = point;
				crossline.second = bestPoint;
			}
			crosslines->push_back(crossline);
			// Infofenster zum Debuggen
			if (debugWin)
			{
				cv::line(crossLinesImage, crossline.first, crossline.second, cv::Scalar(0, 0, 255), 2);
				cv::circle(crossLinesImage, crossline.first, 2, cv::Scalar(0, 255, 0), 4);
				cv::circle(crossLinesImage, crossline.second, 2, cv::Scalar(255, 0, 170), 4);
			}
		}
	}

	// Debugfenster anzeigen
	if (debugWin)
		DebugWinOrganizer::addWindow(crossLinesImage, "Querlinien einzeichnen für eine Seite");
}

// --------------------------------------------------------------------------
// analysiert die Querlinien und wirft zu kurze oder lange weg
// --------------------------------------------------------------------------
void TrackDetection::calLanesCrossLinesFilter(std::vector<std::pair<cv::Point2f, cv::Point2f>> *crosslines, float pointDistanceInMeter)
{
	// Parameter auslesen
	cv::FileNode val = para["lanes_detection"];

	// kleine Hilfsfunktion um Distance zu berechnen
	struct {
		int operator()(std::pair<cv::Point2f, cv::Point2f> a)
		{
			return (int)((int)(pow(a.second.x - a.first.x, 2) + pow(a.second.y - a.first.y, 2)) / pow(5, 2));
			// Immer 5 Pixel zusammen ziehen (Da ohne Wurzel zum Quadrat)
		}
	} getDis;

	// Nach der längsten Crossline suchen
	std::pair<cv::Point2f, cv::Point2f> maxSizePoint = *std::max_element(crosslines->begin(), crosslines->end(),
		                                                       [](std::pair<cv::Point2f, cv::Point2f> a, std::pair<cv::Point2f, cv::Point2f> b)
	                                                               { return pow(a.second.x-a.first.x,2)+pow(a.second.y - a.first.y, 2) < pow(b.second.x - b.first.x, 2) + pow(b.second.y - b.first.y, 2); });
	int max = getDis(maxSizePoint) + 1;
	// Histogramm aller Linienlängen erstellen
	std::vector<int> hist;
	for (int i = 0; i < max; i++)
		hist.push_back(0);
	for (std::pair<cv::Point2f, cv::Point2f> p : *crosslines)
		hist[getDis(p)]++;
	// Häufigsten Wert finden
	int maxPos = (int)distance(hist.begin(), std::max_element(hist.begin(), hist.end()));
	// Suchen nach zur kurzen oder langen Linien
	float rangePosMin = maxPos * pow((float)val["cross_lane_filter_min_faktor"],2);
	float rangePosMax = maxPos * pow((float)val["cross_lane_filter_max_faktor"],2);
	std::cout << "Es wird nach zu kurzen oder langen Crosslines gesucht" << std::endl;
	std::cout << "   Min: " << rangePosMin << "     Max : " << rangePosMax << std::endl;
	auto p = std::begin(*crosslines);
	while (p != std::end(*crosslines))
	{
		if (getDis(*p) < rangePosMin || getDis(*p) > rangePosMax)
		{
			std::cout << "Eine zu kurze oder lange Crossline wurde entfernt (laenge:" << getDis(*p) << ")" << std::endl;
			// Linie aus dem Vorrat entfernen
			p = crosslines->erase(p);
		}
		else
			++p;
	}
	// Pixel pro Meter berechnen und Abstand zwischen zwei Punkten am ende
	float trackWidth = 0.25;
	float pixlePerMeter = sqrt((float)maxPos*pow(5,2)) / trackWidth;
	pixelBetweenPoints = (int)(pointDistanceInMeter * pixlePerMeter);
	std::cout << "Abstand zwischen den Auswertepunkten: " << pixelBetweenPoints << std::endl;

	// Infofenster zum Debuggen
	if (debugWin || unitTestPic)
	{
		cv::Mat crossLinesFilterImage;
		outputImage.copyTo(crossLinesFilterImage);
		for (std::pair<cv::Point2f, cv::Point2f> points : *crosslines)
		{
			cv::line(crossLinesFilterImage, points.first, points.second, cv::Scalar(0, 0, 255), 2);
			cv::circle(crossLinesFilterImage, points.first, 2, cv::Scalar(0, 255, 0), 4);
			cv::circle(crossLinesFilterImage, points.second, 2, cv::Scalar(255, 0, 170), 4);
		}
		if (debugWin)
			DebugWinOrganizer::addWindow(crossLinesFilterImage, "alle auswertbaren Crosslines nach Filterung");
		if (unitTestPic)
			crossLinesFilterImage.copyTo(unitTestPic3);
	}
}

// --------------------------------------------------------------------------
// Berechnet je Crossline zwei Punkte für beide Fahrspuhren und erstellt diese
// --------------------------------------------------------------------------
void TrackDetection::calLanesIrregular(std::vector<cv::Point2f> *lane1i, std::vector<cv::Point2f> *lane2i, std::vector<std::pair<cv::Point2f, cv::Point2f>> crosslines)
{
	cv::Mat lanesImage;
	if (debugWin || unitTestPic)
		outputImage.copyTo(lanesImage);
	
	// Crosslines sortieren nach der Reihenfolge
	calLanesIrregularSortCrosslines(&crosslines);
	// Parameter auslesen
	cv::FileNode val = para["lanes_detection"];
	// Prozentuale Aufteilung der Farbahn
	float sideToLane1 = (float)val["side_to_lane"];
	float sideToLane1Cross = (float)val["side_to_lane_cross"];
	float sideToLane2 = 1 - sideToLane1;
	float sideToLane2Cross = 1 - sideToLane1Cross;
	int invertCount = 0;
	bool invert = false;
	// Alle Querlinien durchgehen und Punkte zu den Spuren hinzufügen
	for (int i = 0; i < crosslines.size(); i++)
	{
		// Vektor zwischen den zwei punkten berechnen
		cv::Point2f vec;
		vec.x = crosslines[i].second.x - crosslines[i].first.x;
		vec.y = crosslines[i].second.y - crosslines[i].first.y;
		// Jeder Spur pro Crossline einen Punkt hinzufügen
		cv::Point2f lane1Point, lane2Point;
		lane1Point.x = crosslines[i].first.x + sideToLane1 * vec.x;
		lane2Point.x = crosslines[i].first.x + sideToLane2 * vec.x;
		lane1Point.y = crosslines[i].first.y + sideToLane1 * vec.y;
		lane2Point.y = crosslines[i].first.y + sideToLane2 * vec.y;
		// Krezungsbereich?
		if (calLanesIrregularJunctionDetection(lane1Point) && calLanesIrregularJunctionDetection(lane2Point))
		{
			lane1Point.x = crosslines[i].first.x + sideToLane1Cross * vec.x;
			lane2Point.x = crosslines[i].first.x + sideToLane2Cross * vec.x;
			lane1Point.y = crosslines[i].first.y + sideToLane1Cross * vec.y;
			lane2Point.y = crosslines[i].first.y + sideToLane2Cross * vec.y;
			// Dabuginformationen anzeigen
			if (debugWin || unitTestPic)
			{
				cv::circle(lanesImage, lane1Point, 3, cv::Scalar(0, 0, 255), 6);
				cv::circle(lanesImage, lane2Point, 3, cv::Scalar(0, 0, 255), 6);
			}
			// Bei Vollständiger Kreuzung die letzten Punkte neu sortieren
			invertCount++;
			if (invertCount == 4)
			{
				invert = !invert;
				invertCount = 0;
				cv::Point2f l1 = lane1i->back();
				lane1i->pop_back();
				cv::Point2f l2 = lane2i->back();
				lane2i->pop_back();
				lane1i->push_back(l2);
				lane2i->push_back(l1);
				std::cout << "Streckenweiche erkannt" << std::endl;
			}
		}
		else
			invertCount = 0;
		// Wenn nötig Datenströme gekreuzt umkopieren
		if (invert)
		{
			cv::Point2f temp = lane1Point;
			lane1Point = lane2Point;
			lane2Point = temp;
		}
		// Fügt Punkte zu den Lines hinzu
		lane1i->push_back(lane1Point);
		lane2i->push_back(lane2Point);
	}
	// Beide Linien in selbe Richtung starten lassen
	calLanesIrregularStartDirection(lane1i, lane2i);
	// Debugfenster anzeigen
	if (debugWin || unitTestPic)
	{
		cv::circle(lanesImage, (*lane1i)[0], 3, cv::Scalar(255, 255, 255), 2);
		cv::circle(lanesImage, (*lane2i)[0], 3, cv::Scalar(255, 255, 255), 2);
		for (int i = 0; i < lane1i->size() - 1; i++)
			cv::line(lanesImage, (*lane1i)[i], (*lane1i)[i + 1], cv::Scalar(255, 0, 0), 2);
		cv::line(lanesImage, (*lane1i)[0], (*lane1i)[lane1i->size()-1], cv::Scalar(255, 0, 0), 2);
		for (int i = 0; i < lane2i->size() - 1; i++)
			cv::line(lanesImage, (*lane2i)[i], (*lane2i)[i + 1], cv::Scalar(0, 255, 0), 2);
		cv::line(lanesImage, (*lane2i)[0], (*lane2i)[lane2i->size() - 1], cv::Scalar(0, 255 , 0), 2);
		if (debugWin)
			DebugWinOrganizer::addWindow(lanesImage, "Spuren eingezeichnet mit unregelmäßigen Abständen");
		if (unitTestPic)
			lanesImage.copyTo(unitTestPic4);
	}
}

// --------------------------------------------------------------------------
// Sortiert alle Crosslines nach der Streckenreinfolge
// --------------------------------------------------------------------------
void TrackDetection::calLanesIrregularSortCrosslines(std::vector<std::pair<cv::Point2f, cv::Point2f>> *crosslines)
{
	// Umkopieren der unsortierten Crosslines
	std::vector<std::pair<cv::Point2f, cv::Point2f>> crosslinesUnsort = *crosslines;
	crosslines->clear();
	crosslines->push_back(crosslinesUnsort[0]);
	crosslinesUnsort.erase(crosslinesUnsort.begin());
	// Mit zufälligen beginnen und durcharbeiten
	while (crosslinesUnsort.size() > 0)
	{
		std::pair<cv::Point2f, cv::Point2f> lastPoints = (*crosslines)[crosslines->size() - 1];
		cv::Point2f lastPoint((lastPoints.first.x + lastPoints.second.x) * 0.5f, (lastPoints.first.y + lastPoints.second.y) * 0.5f);
		double lastAngle = angleFromVec(lastPoints.first, lastPoints.second);
		float bestDistance = 3E+38f;
		int id = -1;
		// Alle Crosslines durchgehen und schauen welche den geringsten Abstand zur letzten hat
		for (int i = 0; i < crosslinesUnsort.size(); i++)
		{
			cv::Point2f midPoint((crosslinesUnsort[i].first.x + crosslinesUnsort[i].second.x) * 0.5f, (crosslinesUnsort[i].first.y + crosslinesUnsort[i].second.y) * 0.5f);
			float distance = pow(midPoint.x - lastPoint.x, 2) + pow(midPoint.y - lastPoint.y, 2);
			double angleDiff = angleDiffFromVec(crosslinesUnsort[i].first, crosslinesUnsort[i].second, lastAngle);
			if (distance < bestDistance && angleDiff < 0.60)
			{
				bestDistance = distance;
				id = i;
			}
		}
		if (id == -1)
		{
			std::cout << "Fehler: Vorzeitiger Abbruch der Sortierung der Crosslines (verbleibend " << crosslinesUnsort.size() << ")" << std::endl;
			break;
		}
		crosslines->push_back(crosslinesUnsort[id]);
		crosslinesUnsort.erase(crosslinesUnsort.begin() + id);
	}
}

// --------------------------------------------------------------------------
// Sortiert die Punkte in die richtige Reinfolge mit Abstandsverfahren
// Wird in aktueller Version nicht mehr verwendet
// --------------------------------------------------------------------------
void TrackDetection::calLanesIrregularSortLanes(std::vector<cv::Point2f> laneUnsort, std::vector<cv::Point2f> *laneSort)
{
	// ersten Linienpunkt hinzufügen
	laneSort->push_back(laneUnsort[0]);
	laneUnsort.erase(laneUnsort.begin());
	// Alle Punkte anfügen je nach Abstand
	double bestDistance = 3E+38f;
	while (laneUnsort.size() > 0)
	{
		cv::Point2f lastPoint = (*laneSort)[laneSort->size()-1];
		float bestDistance = 3E+38f;
		int id;
		for (int i = 0; i < laneUnsort.size(); i++)
		{
			float distance = pow(laneUnsort[i].x - lastPoint.x, 2) + pow(laneUnsort[i].y - lastPoint.y, 2);
			if (distance < bestDistance)
			{
				bestDistance = distance;
				id = i;
			}
		}
		// Besten Punkt abspeichern
		laneSort->push_back(laneUnsort[id]);
		laneUnsort.erase(laneUnsort.begin() + id);
	}
}

// --------------------------------------------------------------------------
// Prüft ob eine Kreuzung vorhanden ist
// --------------------------------------------------------------------------
bool TrackDetection::calLanesIrregularJunctionDetection(cv::Point2f pos)
{
	// Parameter einlesen
	cv::FileNode val = para["lanes_detection"];
	int boxSize = (int)val["junction_detection_box_size"];

	// Prüfen ob der Punkt in der Auswertemaske ist, wenn nicht handelt es sich um eine Brücke, dann kann es keine Kreuzung sein
	if (maskImage.at<uchar>(pos) == 0)
		return false;

	// Auswerterbereich definieren
	cv::Rect R((int)pos.x - (int)(boxSize / 2), (int)pos.y - (int)(boxSize / 2), boxSize, boxSize);
	// Ausschneiden
	cv::Mat ROI = inputImage(R);
	cv::cvtColor(ROI, ROI, CV_RGB2GRAY);
	// leichtes weichzeichnen
	cv::medianBlur(ROI, ROI, (int)val["junction_detection_blur_size"]);
	// Histogramm erstellen
	cv::Mat hist;
	int numbins = 100;
	float range[] = { 0, 256 };
	const float* histRange = { range };
	cv::calcHist(&ROI, 1, 0, cv::Mat(), hist, 1, &numbins, &histRange);	
	// Zählen wie verschieden die Werte im Histogramm sind
	int count = 0;
	for (int i = 1; i < numbins; i++)
		if (hist.at<float>(i - 1) != 0) count++;
	// Erkennung einer Weiche
	if (count <= (int)val["junction_detection_count_limit"])
		return true;
	
	return false;
}

// --------------------------------------------------------------------------
// Prüft ob beide Linien in die selbe Richtung starten und passt dies
// gegebendenfalls an
// --------------------------------------------------------------------------
void TrackDetection::calLanesIrregularStartDirection(std::vector<cv::Point2f> *lane1i, std::vector<cv::Point2f> *lane2i)
{
	float vecX1 = (*lane1i)[1].x - (*lane1i)[0].x;
	float vecX2 = (*lane2i)[1].x - (*lane2i)[0].x;
	float vecY1 = (*lane1i)[1].y - (*lane1i)[0].y;
	float vecY2 = (*lane2i)[1].y - (*lane2i)[0].y;
	float vecadd = pow(vecX1 + vecX2, 2) + pow(vecY1 + vecY2, 2);
	float vecsub = pow(vecX1 - vecX2, 2) + pow(vecY1 - vecY2, 2);
	if (vecadd < vecsub)
		std::reverse(lane2i->begin(), lane2i->end());
}

// --------------------------------------------------------------------------
// Berechnet die Punktpositionen an denen später das Auto abgefragt wird
// gegebendenfalls an
// --------------------------------------------------------------------------
void TrackDetection::calLanesRegular(std::vector<cv::Point2f> lane1i, std::vector<cv::Point2f> lane2i)
{
	cv::Mat lanesImage;
	if (debugWin || unitTestPic)
		outputImage.copyTo(lanesImage);

	// Berechnen der zwei Spuren
	calLanesRegularSingle(&lane1i, &lane1r);
	calLanesRegularSingle(&lane2i, &lane2r);

	// Debugfenster anzeigen
	if (debugWin || unitTestPic)
	{
		for (cv::Point2f p : lane1r) 
			cv::circle(lanesImage, p, 2, cv::Scalar(255, 255, 255), 2);
		for (cv::Point2f p : lane2r)
			cv::circle(lanesImage, p, 2, cv::Scalar(255, 255, 255), 2);
		/*
		if (debugWin)
			DebugWinOrganizer::addWindow(lanesImage, "Spuren eingezeichnet mit unregelmäßigen Abständen");
		if (unitTestPic)
			lanesImage.copyTo(unitTestPic4);*/
		lanesImage.copyTo(outputImage);
	}
}

// --------------------------------------------------------------------------
// Berechnet die Punktpositionen an denen später das Auto abgefragt wird
// Einer Spur
// --------------------------------------------------------------------------
void TrackDetection::calLanesRegularSingle(std::vector<cv::Point2f> *lanei, std::vector<cv::Point2f> *laner)
{
	// Ersten Punkte festlegen
	laner->push_back((*lanei)[0]);
	cv::Point2f pL = (*lanei)[0];
	cv::Point2f p1 = pL;
	cv::Point2f p2;
	lanei->push_back((*lanei)[0]);
	lanei->erase(lanei->begin());
	// Regelmäßige Abstände herausfinden
	for (std::vector<cv::Point2f>::iterator it = lanei->begin(); it != lanei->end();)
	{
		cv::Point2f p2 = *it;
		float dis = sqrt(pow(p2.x - pL.x, 2) + pow(p2.y - pL.y, 2));
		if (dis > pixelBetweenPoints)
		{
			// Schnittpunkt berechnen und hinzufügen
			cv::Point2f pTop;
			bool found = false;
			float deltaX = (p2.x - p1.x) / dis;
			float deltaY = (p2.y - p1.y) / dis;
			for (int i = 0; i <= dis + 1 && !found; i++)
			{
				float x = (p1.x + deltaX * i) - pL.x;
				float y = (p1.y + deltaX * i) - pL.y;
				float dis2 = sqrt(pow(x, 2) + pow(y, 2));
				if (dis2 > pixelBetweenPoints)
				{
					pTop.x = p1.x + deltaX * i;
					pTop.y = p1.y + deltaY * i;
					found = true;
				}
			}
			if (found)
			{
				// Gefunden Punkt hinzufügen
				laner->push_back(pTop);
				// Abgearbeitete Punkte löschen
				while (p2 != *lanei->begin())
					lanei->erase(lanei->begin());
				// Letzen Punkt ändern
				pL = pTop;
				p1 = pL;
				// Wieder auf ersten Punkt setzen
				it = lanei->begin();
			}
			else
			{
				p1 = p2;
				it++;
			}
		}
		else
		{
			p1 = p2;
			it++;
		}
	}
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

// --------------------------------------------------------------------------
// Berechnet den Winkelunterschied zweier Vektoren
// --------------------------------------------------------------------------
double TrackDetection::angleDiffFromVec(cv::Point2d a1, cv::Point2d a2, cv::Point2d b1, cv::Point2d b2)
{
	double angle1 = atan2(a2.y - a1.y, a2.x - a1.x);
	double angle2 = atan2(b2.y - b1.y, b2.x - b1.x);
	double angleDiff = abs(angle1 - angle2);
	if (angleDiff > M_PI) angleDiff = 2 * (float)M_PI - angleDiff;
	return angleDiff;
}

// --------------------------------------------------------------------------
// Berechnet den Winkelunterschied eines Vektors und eines Winkels
// --------------------------------------------------------------------------
double TrackDetection::angleDiffFromVec(cv::Point2d a1, cv::Point2d a2, double angle2)
{
	double angle1 = atan2(a2.y - a1.y, a2.x - a1.x);
	double angleDiff = abs(angle1 - angle2);
	if (angleDiff > M_PI) angleDiff = 2 * (float)M_PI - angleDiff;
	return angleDiff;
}

// --------------------------------------------------------------------------
// Berechnet den Winkeleines Vektors
// --------------------------------------------------------------------------
double TrackDetection::angleFromVec(cv::Point2d a1, cv::Point2d a2)
{
	return atan2(a2.y - a1.y, a2.x - a1.x);
}