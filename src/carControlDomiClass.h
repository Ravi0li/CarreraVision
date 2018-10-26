#pragma once
#include "informationShareClass.h"
#include "bluetoothConnectionClass.h"
#include <opencv2/core.hpp>
#include <string>
#include <chrono>
#include <mutex>

class CarControlDomiClass
{
public:
	CarControlDomiClass(InformationShareClass* infoPackage, int countTrackpoints, std::vector<cv::Point2f>* cartesianTrackPoints, BluetoothConnectionClass* bluetoothObject, int channel, float pointDistance);
	void loopingThread();
	void stopThread();
	~CarControlDomiClass();
	void toggleDirection();
	void ChangeVelocityDirection();
	int getFrameRate();

private:

	// Diese Werte müssen einmalig per Kalibrierung festgelegt und händisch gemessen werden

	float refCarMass;		// Masse des kalibrierten Referenzautos, Einheit [kg]
	float actualCarMass;	// Masse des aktuellen Autos, Einheit [kg]
	float refRadius;		// Radius der kalibrierten Referenzkurve, Einheit [m]
	int refVelocity;		// Geschwindigkeit (= Stellsignal) der Referenzkurve, bei der das Auto gerade nicht aus der Kurve geflogen ist
	float brakingFactor;	// Bestimmt die maximal mögliche Bremsstärke des Autos
	
	// Für Berechnung zur Laufzeit
	float correctionFactor;																			// (Empirischer) Korrekturfaktor um Stellsignal runterzuskalieren: Spielraum lassen
	int* trackVelocityNoBraking;																	// (maximale) Kurvengeschwindigkeit für jeden Punkt auf der Strecke ohne Bremsalgorithmus
	int* trackVelocityDirection1;																			// (maximale) Kurvengeschwindigkeit für jeden Punkt auf der Strecke mit Bremsalgorithmus (Richtung 1)
	int* trackVelocityDirection2;																		// (maximale) Kurvengeschwindigkeit für jeden Punkt auf der Strecke mit Bremsalgorithmus (Richtung 2)
	int* trackVelocityDirectionDrive;
	std::vector<cv::Point2f>* cartesianTrackPoints;													// Array mit allen Streckenpunkten in kartesischer Form
	float pointDistance;																			// Abstand zwischen zwei Punkten (Trackpoints), Einheit [m]
	bool stop = false;																				// Stopt den Thread
	int minimumVelocity;																			// Kleinstes Stellsignal der Strecke
	int direction;																					// Richtung: Kann drei Werte annehmen: 1, 0 , -1

	// Frameraten messung
	std::chrono::high_resolution_clock::time_point startFrame;	// Zeit zur Frameratenmessung
	int countFrame = 0;											// Zähler für Frameratenanzeige
	int frameRate = 0;											// Aktuelle Framerate
	std::mutex frameMutex;										// Mutex für die Framerate

	// Schnittstelle
	InformationShareClass* infoPackage;							// Zeiger auf Schnittstellen Klasse
	int countTrackpoints;										// Anzahl Streckenpunkte
	BluetoothConnectionClass* bluetoothObject;					// Zeiger auf Bluetooth Objekt
	int channel;												// Spur 1 oder 2

	// Methoden
	int calculateCurrentControlInput(float radiusCurrent);
	void calculateGlobalControlInput();
	float distanceBetweenPoints(cv::Point2f point1, cv::Point2f point2);
	float twiceSignedArea(cv::Point2f point1, cv::Point2f point2, cv::Point2f point3);
	void smoothTrackVelocity();
	void calculateBreakpoints();
	void outputArrayAsCSV(int* arrayToConvert, int length, std::string label);
	void calculateMinmumControlInput();
};