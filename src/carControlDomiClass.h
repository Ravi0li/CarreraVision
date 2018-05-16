#pragma once
#include "informationShareClass.h"
#include "bluetoothConnectionClass.h"

class CarControlDomiClass
{
public:
	CarControlDomiClass(InformationShareClass* infoPackage, int countTrackpoints, double* trackAngles, BluetoothConnectionClass* bluetooth);
	~CarControlDomiClass();

private:

	// Diese Werte müssen einmalig per Kalibrierung festgelegt und händisch gemessen werden

	double carMassRef;		// Masse des kalibrierten Referenzautos, Einheit [kg]
	double carMassActual;	// Masse des aktuellen Autos, Einheit [kg]
	double radiusRef;		// Radius der kalibrierten Referenzkurve, Einheit [m]
	int velocityRef;		// Geschwindigkeit (= Stellsignal) der Referenzkurve, bei der das Auto gerade nicht aus der Kurve geflogen ist
	
	// Für Berechnung zur Laufzeit
	double correctionFactor;	// (Empirischer) Korrekturfaktor um Stellsignal runterzuskalieren: Spielraum lassen
	double* trackVelocity;		// (maximale) Kurvengeschwindigkeit für jeden Punkt auf der Strecke

	// Schnittstelle
	InformationShareClass* infoPackage;		// Zeiger auf Schnittstellen Klasse
	int countTrackpoints;					// Anzahl Trackpoints
	double* trackAngles;					// Array mit Kurvenwinkel für alle Streckenpunkte
	BluetoothConnectionClass* bluetooth;	// Zeiger auf Bluetooth Klasse, TODO: Zeiger auf Methode wäre sinnvoller!

	// Methoden
	int calculateCurrentControlInput(double radiusCurrent);
	void calculateGlobalControlInput();

};