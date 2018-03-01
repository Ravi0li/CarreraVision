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
// Auswertung des Streckenbildes
// --------------------------------------------------------------------------
bool TrackDetection::calculate()
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
		if (counter > 5)
		{
			std::cout << "Fehler: Es konnten keine sinvollen Streckenbegrenzungen berechnet werden!" << std::endl;
			binaryImage.copyTo(outputImage);
			return false;
		}
		lines = calSearchLines(keypoints);
	} while (!calCheckLines(&lines));
	calCreatTrackMask(lines);
	if (!calLanes(lines)) return false;

	finisch = clock();
	std::cout << "Streckenerkennung abgeschlossen (" << finisch - start << "ms)";
	
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
		cv::Mat keypointsImage(image->rows, image->cols, image->type(), cv::Scalar(0, 0, 0));
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
bool TrackDetection::calLanes(std::vector<std::vector<cv::Point2f>> lines)
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

	// Spurenlinien berechnen (unregelmäsige Abstände)
	std::vector<cv::Point2f> lane1i, lane2i;
	calLanesIrregular(&lane1i, &lane2i, crosslines);

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
			// Wenn alles erfüllt, dann als neusten nächsten Schnittpunkt merken
			if (bestDistance > distance && vecFac2 >= 0 && vecFac2 <= 1 && vecFac1 > 0)
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
// Berechnet zwei Linien mit Punkten unregelmäsigen Abstandes, die die
// Fahrspuren darstellen
// --------------------------------------------------------------------------
void TrackDetection::calLanesIrregular(std::vector<cv::Point2f> *lane1i, std::vector<cv::Point2f> *lane2i, std::vector<std::pair<cv::Point2f, cv::Point2f>> crosslines)
{
	// Prozentuale Aufteilung der Farbahn
	float sideToLane1 = 0.28;
	float sideToLane2 = 1 - sideToLane1;
	// Alle Querlinien durchgehen und Punkte zu den Spuren hinzufügen
	std::vector<cv::Point2f> lane1u, lane2u;
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
		lane1u.push_back(lane1Point);
		lane2u.push_back(lane2Point);
		//cv::circle(outputImage, lane1Point, 2, cv::Scalar(0, 255, 0), 4);
		//cv::circle(outputImage, lane2Point, 2, cv::Scalar(255, 0, 0), 4);
	}
	// Sortieren der Punkte nach ihrer Reinfolge
	calLanesIrregularSort(lane1u, lane1i);
	calLanesIrregularSort(lane2u, lane2i);
	// Debugfenster anzeigen
	if (debugWin)
	{
		cv::Mat lanesImage;
		outputImage.copyTo(lanesImage);
		for (int i = 0; i < lane1i->size() - 1; i++)
			cv::line(lanesImage, (*lane1i)[i], (*lane1i)[i + 1], cv::Scalar(255, 0, 0), 3);
		for (int i = 0; i < lane2i->size() - 1; i++)
			cv::line(lanesImage, (*lane2i)[i], (*lane2i)[i + 1], cv::Scalar(0, 255, 0), 3);
		DebugWinOrganizer::addWindow(lanesImage, "Spuren eingezeichnet mit unregelmäßigen Abständen");
	}
}

// --------------------------------------------------------------------------
// Sortiert die Punkte in die richtige Reinfolge mit Abstandsverfahren
// --------------------------------------------------------------------------
void TrackDetection::calLanesIrregularSort(std::vector<cv::Point2f> laneUnsort, std::vector<cv::Point2f> *laneSort)
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