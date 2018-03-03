#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include "../src/trackdetectionClass.h"

int count = 8;
int sizeX = 3280;
int sizeY = 2464;
int faktor = 2;

void resizeSave(cv::Mat *imageOut, int x, int y, cv::Mat imageIn)
{
	int newX = sizeX / faktor;
	int newY = sizeY / faktor;
	cv::Mat smallImage;
	cv::resize(imageIn, smallImage, cv::Size(newX, newY));
	smallImage.copyTo((*imageOut)(cv::Rect(x * newX, y * newY, smallImage.cols, smallImage.rows)));
}

int main(int argc, const char** argv)
{
	std::cout << "---------------------------------" << std::endl;
	std::cout << "----------- Unit-Test -----------" << std::endl;
	std::cout << "---------------------------------" << std::endl;

	// Parameterdatei öffnen
	std::string paraFile = "./para.xml";
	cv::FileStorage para;
	if (!para.open(paraFile, cv::FileStorage::READ))
	{
		std::cout << "Cannot read para file: " << paraFile << std::endl;
		return -1;
	}

	// Ausgabebild vorbereiten
	cv::Mat outImage(sizeY * count / faktor, sizeX * 5 / faktor, 16);

	// Alle Demofiles durchgehen
	for (int i = 1; i <= count; i++)
	{
		cv::Mat imageOut1, imageOut2, imageOut3, imageOut4;
		// Datei öffnen
		std::string filename = "./demo/example" + std::to_string(i) + ".png";
		std::cout << std::endl << std::endl;
		std::cout << "-------------- " << filename << " --------------" << std::endl;
		cv::Mat imageIn = cv::imread(filename, 1);
		if (imageIn.empty())
		{
			std::cout << "Cannot read image file: " << filename << std::endl;
			continue;
		}
		TrackDetection trackDetection(para["track_detection"]);
		trackDetection.setUnitTestPic(true);
		trackDetection.setPicture(imageIn);
		trackDetection.calculate();
		trackDetection.getUnitTestPic(&imageOut1, &imageOut2, &imageOut3, &imageOut4);
		// Ergebnisse abspeichern
		resizeSave(&outImage, 0, i - 1, imageIn);
		if (!imageOut1.empty())
			resizeSave(&outImage, 1, i - 1, imageOut1);
		if (!imageOut2.empty())
			resizeSave(&outImage, 2, i - 1, imageOut2);
		if (!imageOut3.empty())
			resizeSave(&outImage, 3, i - 1, imageOut3);
		if (!imageOut4.empty())
			resizeSave(&outImage, 4, i - 1, imageOut4);
	}

	// Ausgabe speichern
	std::cout << std::endl << std::endl;
	std::cout << "-------------- Ergebnisse werden gespeichert --------------" << std::endl;
	std::cout << "bitte warten..." << std::endl;
	cv::imwrite("./unit_test.jpg", outImage);

	// Ende
	std::cout << std::endl << std::endl;
	std::cout << "----------------------------------" << std::endl;
	std::cout << "-------------- Ende --------------" << std::endl;
	std::cout << "----------------------------------" << std::endl;
	std::cin.get();
	return 0;
}