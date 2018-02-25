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
	cv::Mat workImage;		// Bild an dem gearbeitet wird
	cv::Mat outputImage;	// Bild mit allen Auswertungen
	bool debugWin;			// Sollen alle Zusatzfenster angezeigt werden

	void calHSVRange(cv::Mat *image);
	void calMorphology(cv::Mat *image);
	std::vector<cv::KeyPoint> calBlobDetection(cv::Mat *image);
	std::vector<std::vector<cv::Point2f>> calSearchLines(std::vector<cv::KeyPoint> keypoints);
	bool calCheckLines(std::vector<std::vector<cv::Point2f>> *lines);
	void calCreatTrackMask(cv::Mat *image, std::vector<std::vector<cv::Point2f>> lines);

	void showHistogram(cv::Mat image, std::string title, int posX, int posY);
	cv::Scalar hsvScalar(double h, double s, double v);
};