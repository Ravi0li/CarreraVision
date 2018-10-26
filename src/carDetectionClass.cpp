#define _USE_MATH_DEFINES
#include "carDetectionClass.h"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <thread>
#include <numeric>

// --------------------------------------------------------------------------
// Konstruktor
// --------------------------------------------------------------------------
CarDetection::CarDetection(std::vector<cv::Point2f> lane1s, std::vector<cv::Point2f> lane2s)
{
	// Setzen der Auswertepunkte
	lane1 = lane1s;
	lane2 = lane2s;
	// Anlegen der Mittelwertarrays
	firstRound = true;
	mid1.insert(mid1.begin(), lane1.size(), cv::Vec3b());
	mid2.insert(mid2.begin(), lane2.size(), cv::Vec3b());
	// Zeichenmodus
	paintMode = 1;
}

// --------------------------------------------------------------------------
// Destruktor
// --------------------------------------------------------------------------
CarDetection::~CarDetection()
{
	cap->release();
}

// --------------------------------------------------------------------------
// Setzen der Austauschklassen
// --------------------------------------------------------------------------
void CarDetection::setInfoPackage(InformationShareClass *laneShare1, InformationShareClass *laneShare2)
{
	car1 = laneShare1;
	car2 = laneShare2;
	car1->SetPosition(-1);
	car2->SetPosition(-1);
	// Auswertepattern setzen
	analysePattern.push_back(cv::Point2f(0,  0));
	analysePattern.push_back(cv::Point2f(1,  0));
	analysePattern.push_back(cv::Point2f(-1, 0));
	analysePattern.push_back(cv::Point2f(0,  1));
	analysePattern.push_back(cv::Point2f(0, -1));
	//analysePattern.push_back(cv::Point2f(3,  0));
	//analysePattern.push_back(cv::Point2f(-3, 0));
	//analysePattern.push_back(cv::Point2f(0,  3));
	//analysePattern.push_back(cv::Point2f(0, -3));
}

// --------------------------------------------------------------------------
// Setzt das Ausgabebild
// --------------------------------------------------------------------------
void CarDetection::setOutputImage(cv::Mat *outImageL)
{
	outImage = outImageL;
}

// --------------------------------------------------------------------------
// �ndert den Zeichenmodus
// --------------------------------------------------------------------------
void CarDetection::ChangePaintMode()
{
	if (paintMode == 1)
		paintMode = 2;
	else if (paintMode == 2)
		paintMode = 1;
}

// --------------------------------------------------------------------------
// Setzt die Videoquelle (Wenn kein File dann von der Kamera)
// --------------------------------------------------------------------------
bool CarDetection::setSource(std::string file)
{
	// �ffne die Kamera als Quelle
	if (file.empty())
	{
		cap = new cv::VideoCapture(0);
		if (!cap->isOpened())
		{
			std::cout << "FEHLER: Die Kamera konnte nicht angesprochen werden" << std::endl;
			return false;
		}
		cap->set(CV_CAP_PROP_FRAME_WIDTH, 820); //1640
		cap->set(CV_CAP_PROP_FRAME_HEIGHT, 616);  //1232
		cap->set(CV_CAP_PROP_FPS, 40);
		cap->set(CV_CAP_PROP_BUFFERSIZE, 1);
		cap->set(CV_CAP_PROP_ISO_SPEED, 200);
		cap->set(CV_CAP_PROP_EXPOSURE, 1);
		std::cout << "Videostream von der Kamera wird gestartet" << std::endl;
	}
	else // Lade eine Datei als Quelle
	{
		cap = new cv::VideoCapture(file);
		if (!cap->isOpened())
		{
			std::cout << "FEHLER: Datei nicht gefunden mit dem Videostream" << std::endl;
			return false;
		}
		float width = cap->get(CV_CAP_PROP_FRAME_WIDTH);
		downScall = width / 3280.0;
		std::cout << "Videostream wird aus Datei geladen" << std::endl;
	}
	return true;
}

// --------------------------------------------------------------------------
// Stopt den Thread
// --------------------------------------------------------------------------
void CarDetection::stopThread()
{
	stop = true;
}

// --------------------------------------------------------------------------
// Dauerschleife zur Auswertung
// --------------------------------------------------------------------------
void CarDetection::loopingThread()
{
	cv::Mat image;
	startFrame = std::chrono::high_resolution_clock::now();
	do {
		// Frameratenmessung
		auto now = std::chrono::high_resolution_clock::now();
		int diff = std::chrono::duration_cast<std::chrono::milliseconds>(now - startFrame).count();
		if (diff > 1000)
		{
			frameMutex.lock();
			frameRate = countFrame / (diff/1000.0);
			frameMutex.unlock();
			countFrame = 0;
			startFrame = now;
		}
		countFrame++;

		// Neues Bild einlesen
		*cap >> image;

		if (!image.empty())
		{
			// Beim ersten Durchlauf Referenzpixel lesen
			if (firstRound)
			{
				getRefValues(image, &lane1, &mid1);
				getRefValues(image, &lane2, &mid2);
				firstRound = false;
				std::cout << "Referenzpunkte neu berechnet." << std::endl;
			}
			// Auswerten welche Pixel getriggert sind
			std::vector<bool> trig1(lane1.size(), false);
			getTrigerInfo(&image, &lane1, &mid1, &trig1);
			std::vector<bool> trig2(lane2.size(), false);
			getTrigerInfo(&image, &lane2, &mid2, &trig2);
			// Auswerten der Triggerergebnisse
			getTrigerResult(&image, &lane1, &trig1, car1);
			getTrigerResult(&image, &lane2, &trig2, car2);
			// Zeichnet Beschleunigungspunkte
			if (paintMode == 2)
			{
				paintTrackVelocity(&image, &lane1, car1);
				paintTrackVelocity(&image, &lane2, car2);
			}
			// Ergebniss anzeigen
			image.copyTo(*outImage);
		}
		// Delay <- Bei echtzeit entfernen!!!
		//std::this_thread::sleep_for(std::chrono::milliseconds(50));
	} while(!stop);
}

// --------------------------------------------------------------------------
// Referenzpunkt einlesen
// --------------------------------------------------------------------------
void CarDetection::getRefValues(cv::Mat image, std::vector<cv::Point2f> *lane, std::vector<cv::Vec3i> *mid)
{
	for (int i = 0; i < lane->size(); i++)
		(*mid)[i] = getAllPixel(image, (*lane)[i]);
}

// --------------------------------------------------------------------------
// Pr�fen welche Punkte getriggert sind
// --------------------------------------------------------------------------
void CarDetection::getTrigerInfo(cv::Mat *image, std::vector<cv::Point2f> *lane, std::vector<cv::Vec3i> *mid, std::vector<bool> *trig)
{
	// zu Triggernder Grenzwert
	int maxTrig = 100;
	for (int i = 0; i < lane->size(); i++)
	{
		cv::Point2f pos((*lane)[i].x * downScall, (*lane)[i].y * downScall);
		cv::Vec3i p = getAllPixel(*image, (*lane)[i]);
		int r = (int)abs(p[0] - (*mid)[i][0]);
		int g = (int)abs(p[1] - (*mid)[i][1]);
		int b = (int)abs(p[2] - (*mid)[i][2]);
		int dif = (r + g + b) / analysePattern.size();
		// Pr�fen ob Punkt getriggert ist
		if (dif > maxTrig)
		{
			// Langsames angleichen
			//(*mid)[i] = (*mid)[i] * 0.95 + getAllPixel(*image, (*lane)[i]) * 0.05;
			// Trigger merken
			(*trig)[i] = true;
			// Punkt einzeichnen
			if (paintMode == 1)
				cv::circle(*image, pos, 2, cv::Scalar(255, 0, 0), 2);
		}
		else
		{
			// Langsames angleichen
			(*mid)[i] = (*mid)[i] * 0.7 + getAllPixel(*image, (*lane)[i]) * 0.3;
			// Punkt einzeichnen
			if (paintMode == 1)
				cv::circle(*image, pos, 2, cv::Scalar(255, 255, 255), 2);
		}
	}
}

// --------------------------------------------------------------------------
// Triggerpunkte auswerten
// --------------------------------------------------------------------------
void CarDetection::getTrigerResult(cv::Mat *image, std::vector<cv::Point2f> *lane, std::vector<bool> *trig, InformationShareClass *car)
{
	std::vector<int> truePos;
	for (int i = 0; i < lane->size(); i++)
		if ((*trig)[i] == true)
			truePos.push_back(i);
	// Wenn nichts getriggert Autoposition unbekannt
	if (truePos.size() == 0)
		car->SetPosition(-1);
	// Autoposition neu finden
	if (car->GetPosition() == -1)
	{
		if (truePos.size() <= 3 && truePos.size() >= 1 && (truePos.front() - truePos.back()) < 5)
			car->SetPosition((int)std::accumulate(truePos.begin(), truePos.end(), 0) / (int)truePos.size());
	}
	// Autoposition mitf�hren
	else
	{
		std::vector<int> usedPos;
		int j = car->GetPosition() - 5;
		if (j < 0) j += (int)lane->size();
		for (int i = 0; i < 10; i++)
		{
			if ((*trig)[j] == true)
				usedPos.push_back(j);
			j++;
			if (j >= lane->size())
				j -= (int)lane->size();
		}
		// Pr�fen ob �berhaupt Punkte benachbart vorhanden sind
		if (usedPos.size() == 0)
			car->SetPosition(-1);
		else
			car->SetPosition((int)std::accumulate(usedPos.begin(), usedPos.end(), 0) / (int)usedPos.size());
	}
	// Einzeichnen des Autos
	if (car->GetPosition() != -1 && paintMode == 1)
	{
		cv::Point2f pos((*lane)[car->GetPosition()].x * downScall, (*lane)[car->GetPosition()].y * downScall);
		cv::circle(*image, pos, 5, cv::Scalar(0, 0, 255), 5);
	}
}

// --------------------------------------------------------------------------
// Pixel auslesen
// --------------------------------------------------------------------------
cv::Vec3i CarDetection::getPixel(cv::Mat image, cv::Point2f p, cv::Point2f offset)
{
	return image.at<cv::Vec3b>((int)(p.y * downScall) + offset.x, (int)(p.x * downScall) + offset.y);
}

// --------------------------------------------------------------------------
// Pixel auslesen mit gesammten Pattern
// --------------------------------------------------------------------------
cv::Vec3i CarDetection::getAllPixel(cv::Mat image, cv::Point2f p)
{
	cv::Vec3i res(0, 0, 0);
	for (cv::Point2f &offset : analysePattern)
		res += getPixel(image, p, offset);
	return res;
}

// --------------------------------------------------------------------------
// Referenzwerte zur�cksetzen
// --------------------------------------------------------------------------
void CarDetection::resetRefValue()
{
	firstRound = true;
}

// --------------------------------------------------------------------------
// Referenzwerte zur�cksetzen
// --------------------------------------------------------------------------
void CarDetection::paintTrackVelocity(cv::Mat *image, std::vector<cv::Point2f> *lane, InformationShareClass *car)
{
	car->lock();
	int* values = car->GetTrackVelocity();
	car->unlock();
	for (int i = 0; i < lane->size(); i++)
	{
		cv::Point2f pos((*lane)[i].x * downScall, (*lane)[i].y * downScall);
		cv::circle(*image, pos, 2, hsvScalar(0, values[i], 255), 2);
	}
}

// --------------------------------------------------------------------------
// gibt einen Scalar wert zur�ck anhand von RGB Werten
// --------------------------------------------------------------------------
cv::Scalar CarDetection::hsvScalar(double h, double s, double v)
{
	cv::Mat rgb;
	cv::Mat hsv(1, 1, CV_8UC3, cv::Scalar(h, s, v));
	cv::cvtColor(hsv, rgb, CV_HSV2RGB);
	return cv::Scalar((int)rgb.at<cv::Vec3b>(0, 0)[0], (int)rgb.at<cv::Vec3b>(0, 0)[1], (int)rgb.at<cv::Vec3b>(0, 0)[2]);
}

// --------------------------------------------------------------------------
// zeigt die aktuelle Framerate an
// --------------------------------------------------------------------------
int CarDetection::getFrameRate()
{
	frameMutex.lock();
	int ret = frameRate;
	frameMutex.unlock();
	return ret;
}