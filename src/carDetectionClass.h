#pragma once
#include "informationShareClass.h"
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <vector>			// stc::vector
#include <chrono>
#include <mutex>

class CarDetection
{
public:
	CarDetection(cv::FileNode _para, std::vector<cv::Point2f> lane1s, std::vector<cv::Point2f> lane2s);
	~CarDetection();
	void setInfoPackage(InformationShareClass *laneShare1, InformationShareClass *laneShare2);
	bool setSource(std::string file);
	void setOutputImage(cv::Mat *outImageL);
	void loopingThread();
	void stopThread();
	void resetRefValue();
	void ChangePaintMode();
	int getFrameRate();
	void frameOutLock() { outImageMutex.lock(); };
	void frameOutUnlock() { outImageMutex.unlock(); };
	void startDebugSave() { if (debugBuffer.size() == 0) debugBufferOn = true; };
	void saveDebugBufferIfFull();

private:
	void getRefValues(cv::Mat image, std::vector<cv::Point2f> *lane, std::vector<cv::Vec3i> *mid);
	void getTrigerInfo(cv::Mat *image, cv::Mat *paintImage, std::vector<cv::Point2f> *lane, std::vector<cv::Vec3i> *mid, std::vector<bool> *trig);
	void getTrigerResult(cv::Mat *image, cv::Mat *paintImage, std::vector<cv::Point2f> *lane, std::vector<bool> *trig, InformationShareClass *car);
	cv::Vec3i getPixel(cv::Mat image, cv::Point2f p, cv::Point2f offset);
	cv::Vec3i getAllPixel(cv::Mat image, cv::Point2f p);
	void paintTrackVelocity(cv::Mat *image, std::vector<cv::Point2f> *lane, InformationShareClass *car);
	cv::Scalar hsvScalar(double h, double s, double v);

	cv::FileNode para;						// Parameter aus XML
	InformationShareClass *car1, *car2;     // Übergabeparameter der Autos
	std::vector<cv::Point2f> lane1, lane2;  // Punktpositionen zum auswerten
	std::vector<cv::Vec3i> mid1, mid2;		// Mittelwerte von denen die Abweichung berechnet wird
	std::vector<cv::Point2f> analysePattern;// Welche Punkte sollen um einen Triggerpunkt ausgewertet werden
	bool firstRound;						// Ersten durchlauf erkennen
	cv::VideoCapture *cap;					// Videoquelle
	cv::Mat *outImage;						// Ausgabebild
	std::vector<cv::Mat> debugBuffer;		// temporäre Bilder zum debuggen
	bool debugBufferOn;						// sollen die Bilder abgespeichert werden		
	bool stop = false;						// Stoppen der Endlosschleife
	int paintMode;							// Was soll gezeichnet werden
	float downScall = 0.25;					// Wie viel kleiner ist der Videostream
	std::chrono::high_resolution_clock::time_point startFrame;	// Zeit zur Frameratenmessung
	int countFrame = 0;						// Zähler für Frameratenanzeige
	int frameRate = 0;						// Aktuelle Framerate
	std::mutex frameMutex;					// Mutex für die Framerate
	std::mutex outImageMutex;				// Mutex für den OutFrame
	bool fromFile = true;					// ist der Stream aus einer Datei oder aus einem File
};

//int trackpoints;
//float trackAngles[trackpoints];


//int carPosition;
//int carType;

//Oli: GUI, Fahrzeugerkennung
//Domi: Threading, Bluetoothcom.