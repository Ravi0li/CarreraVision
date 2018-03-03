#pragma once
#include <opencv2/highgui.hpp>

class DebugWinOrganizer
{
public:
	static void addWindow(cv::Mat image, std::string title);

private:
	static int counter;					// Zählt wie viele Fenster bereits offen sind
	static const int winSizeX = 350;	// Größe der Debug Fenster in X
	static const int winSizeY = 250;	// Größe der Debug Fenster in Y
	static const int stepX = 200;		// Um wie viel sollen die Fenster nach X versetzt sein
	static const int stepY = 250;		// Um wie viel sollen die Fenster nach Y versetzt sein
	static const int maxEveryRow = 9;   // Wie viele Fenster dürfen maximal nebeneinander sein

	DebugWinOrganizer() {}
};