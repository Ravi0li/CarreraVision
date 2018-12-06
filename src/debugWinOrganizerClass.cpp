#include "debugWinOrganizerClass.h"

// Static Variable vordefinieren
int DebugWinOrganizer::counter = 0;

// --------------------------------------------------------------------------
// Fügt ein Debug Fenster hinzu
// --------------------------------------------------------------------------
void DebugWinOrganizer::addWindow(cv::Mat image, std::string title)
{
	// Position berechnen
	int posX = (counter % maxEveryRow) * stepX;
	int posY = (counter / maxEveryRow) * stepY;

	// Hochzählen
	counter++;

	// Titel formatieren
	title = "(" + std::to_string(counter) + ") - " + title;
	
	// Anzeigen
	cv::namedWindow(title, CV_GUI_NORMAL);
	cv::resizeWindow(title, winSizeX, winSizeY);
	cv::moveWindow(title, posX, posY);
	cv::imshow(title, image);

	//static int i = 1;
	//cv::imwrite(std::to_string(i)+".jpg", image);
	//i++;
}