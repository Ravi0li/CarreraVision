#pragma once
#include <opencv2/core.hpp>

class TrackDetection
{
public:
	TrackDetection(cv::FileNode _para);
	void setPicture(cv::Mat _inputImage);
	void setDebugWin(bool _debugWin);
	bool calculate();
	cv::Mat getResultPicture();
	cv::Mat getMaskPicture();

private:
	enum linesDir { RIGHT, LEFT };

	cv::Mat inputImage;		// Uhrsprüngliches Bild
	cv::Mat maskImage;		// Maske zum ausbleden aller Teile die nicht zur Strecke gehören
	cv::Mat outputImage;	// Bild mit allen Auswertungen
	bool debugWin;			// Sollen alle Zusatzfenster angezeigt werden
	cv::FileNode para;	    // Parameter zur Auswertung

	// Interne Funktionen für die Streckenverarbeitung
	void calHSVRange(cv::Mat *image);
	void calMorphology(cv::Mat *image);
	std::vector<cv::KeyPoint> calBlobDetection(cv::Mat *image);
	void calBlobDetectionMeldedPoints(std::vector<cv::KeyPoint> *keypoints);
	std::vector<std::vector<cv::Point2f>> calSearchLines(std::vector<cv::KeyPoint> keypoints);
	bool calCheckLines(std::vector<std::vector<cv::Point2f>> *lines);
	void calCreatTrackMask(std::vector<std::vector<cv::Point2f>> lines);
	bool calLanes(std::vector<std::vector<cv::Point2f>> lines);
	bool calLanesSideDirection(std::vector<cv::Point2f> *line1, std::vector<cv::Point2f> *line2, linesDir *line1dir, linesDir *line2dir);
	void calLanesCrossLines(std::vector<cv::Point2f> baseLines, std::vector<cv::Point2f> targetLines, linesDir baseDir, std::vector<std::pair<cv::Point2f, cv::Point2f>> *crosslines, bool invertLines);
	void calLanesCrossLinesFilter(std::vector<std::pair<cv::Point2f, cv::Point2f>> *crosslines);
	void calLanesIrregular(std::vector<cv::Point2f> *lane1i, std::vector<cv::Point2f> *lane2i, std::vector<std::pair<cv::Point2f, cv::Point2f>> crosslines);
	void calLanesIrregularSortCrosslines(std::vector<std::pair<cv::Point2f, cv::Point2f>> *crosslines);
	void calLanesIrregularSortLanes(std::vector<cv::Point2f> laneUnsort, std::vector<cv::Point2f> *laneSort);
	bool calLanesIrregularJunctionDetection(cv::Point2f pos);
	void calLanesIrregularStartDirection(std::vector<cv::Point2f> *lane1i, std::vector<cv::Point2f> *lane2i);

	// Hilfsfunktionen für die Streckenverarbeitung
	void showHistogram(cv::Mat image, std::string title, int posX, int posY);
	cv::Scalar hsvScalar(double h, double s, double v);

};