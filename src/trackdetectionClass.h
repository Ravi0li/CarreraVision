#pragma once
#include <opencv2/core.hpp>

class TrackDetection
{
public:
	TrackDetection();
	void setPicture(cv::Mat _inputImage);
	void setDebugWin(bool _debugWin);
	void calculate();
	cv::Mat getResultPicture();

private:
	cv::Mat inputImage;		// Uhrsprüngliches Bild
	cv::Mat outputImage;	// Bild mit allen Auswertungen
	bool debugWin;			// Sollen alle Zusatzfenster angezeigt werden
};