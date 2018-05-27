#pragma once
#include "informationShareClass.h"
#include "bluetoothConnectionClass.h"
#include <boost/geometry.hpp>

class CarControlDomiClass
{
public:
	CarControlDomiClass(InformationShareClass* infoPackage, int countTrackpoints, boost::geometry::model::d2::point_xy<float>* cartesianTrackPoints, BluetoothConnectionClass* bluetooth, float pointDistance);
	~CarControlDomiClass();

private:

	// Diese Werte müssen einmalig per Kalibrierung festgelegt und händisch gemessen werden

	float refCarMass;		// Masse des kalibrierten Referenzautos, Einheit [kg]
	float actualCarMass;	// Masse des aktuellen Autos, Einheit [kg]
	float refRadius;		// Radius der kalibrierten Referenzkurve, Einheit [m]
	int refVelocity;		// Geschwindigkeit (= Stellsignal) der Referenzkurve, bei der das Auto gerade nicht aus der Kurve geflogen ist
	
	// Für Berechnung zur Laufzeit
	float correctionFactor;																			// (Empirischer) Korrekturfaktor um Stellsignal runterzuskalieren: Spielraum lassen
	int* trackVelocity;																				// (maximale) Kurvengeschwindigkeit für jeden Punkt auf der Strecke
	boost::geometry::model::d2::point_xy<float>* cartesianTrackPoints;								// Array mit allen Streckenpunkten in kartesischer Form
	float pointDistance;																			// Abstand zwischen zwei Punkten (Trackpoints), Einheit [m]

	// Schnittstelle
	InformationShareClass* infoPackage;		// Zeiger auf Schnittstellen Klasse
	int countTrackpoints;					// Anzahl Streckenpunkte
	BluetoothConnectionClass* bluetooth;	// Zeiger auf Bluetooth Klasse, TODO: Zeiger auf Methode wäre sinnvoller!

	// Methoden
	int calculateCurrentControlInput(float radiusCurrent);
	void calculateGlobalControlInput();
	float distanceBetweenPoints(boost::geometry::model::d2::point_xy<float> point1, boost::geometry::model::d2::point_xy<float> point2);
	float twiceSignedArea(boost::geometry::model::d2::point_xy<float> point1, boost::geometry::model::d2::point_xy<float> point2, boost::geometry::model::d2::point_xy<float> point3);
	void loopingThread();

};