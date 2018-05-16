#pragma once
#include "informationShareClass.h"
#include "bluetoothConnectionClass.h"

class CarControlDomiClass
{
public:
	CarControlDomiClass(InformationShareClass* infoPackage, int countTrackpoints, double* trackAngles, BluetoothConnectionClass* bluetooth);
	~CarControlDomiClass();

private:

	// Diese Werte m�ssen einmalig per Kalibrierung festgelegt und h�ndisch gemessen werden

	double carMassRef;		// Masse des kalibrierten Referenzautos, Einheit [kg]
	double carMassActual;	// Masse des aktuellen Autos, Einheit [kg]
	double radiusRef;		// Radius der kalibrierten Referenzkurve, Einheit [m]
	int velocityRef;		// Geschwindigkeit (= Stellsignal) der Referenzkurve, bei der das Auto gerade nicht aus der Kurve geflogen ist
	
	// F�r Berechnung zur Laufzeit
	double correctionFactor;	// (Empirischer) Korrekturfaktor um Stellsignal runterzuskalieren: Spielraum lassen
	double* trackVelocity;		// (maximale) Kurvengeschwindigkeit f�r jeden Punkt auf der Strecke

	// Schnittstelle
	InformationShareClass* infoPackage;		// Zeiger auf Schnittstellen Klasse
	int countTrackpoints;					// Anzahl Trackpoints
	double* trackAngles;					// Array mit Kurvenwinkel f�r alle Streckenpunkte
	BluetoothConnectionClass* bluetooth;	// Zeiger auf Bluetooth Klasse, TODO: Zeiger auf Methode w�re sinnvoller!

	// Methoden
	int calculateCurrentControlInput(double radiusCurrent);
	void calculateGlobalControlInput();

};