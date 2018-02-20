#include "trackdetectionClass.h"

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
	outputImage = inputImage;
}

// --------------------------------------------------------------------------
// Rückgabe des Bildes mit allen Auswerteinformationen
// --------------------------------------------------------------------------
cv::Mat TrackDetection::getResultPicture()
{
	return outputImage;
}