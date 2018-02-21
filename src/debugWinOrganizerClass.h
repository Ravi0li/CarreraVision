#pragma once
#include <opencv2/highgui.hpp>

class DebugWinOrganizer
{
public:
	static void addWindow(cv::Mat image, std::string title);

private:
	static int counter;					// Zählt wie viele Fenster bereits offen sind
	static const int winSizeX = 470;	// Größe der Debug Fenster in X
	static const int winSizeY = 400;	// Größe der Debug Fenster in Y
	static const int stepX = 470;		// Um wie viel sollen die Fenster nach X versetzt sein
	static const int stepY = 400;		// Um wie viel sollen die Fenster nach Y versetzt sein
	static const int maxEveryRow = 4;   // Wie viele Fenster dürfen maximal nebeneinander sein

	DebugWinOrganizer() {}
};