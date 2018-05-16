#include "carControlDomiClass.h"
#include <cmath>

// --------------------------------------------------------------------------
// Kalibrierte Werte festlegen
// --------------------------------------------------------------------------
CarControlDomiClass::CarControlDomiClass(InformationShareClass* infoPackage, int countTrackpoints, double* trackAngles, BluetoothConnectionClass* bluetooth)
{
	// Diese Werte müssen einmalig per Kalibrierung festgelegt und händisch gemessen werden
	carMassRef = 0.150;
	carMassActual = 0.150;	
	radiusRef = 0.5;		
	velocityRef = 120;

	// Andere Instanzvariablen mit Standardwerten belegen
	correctionFactor = 0.85;

	// Schnittstelle
	this->infoPackage = infoPackage;
	this->countTrackpoints = countTrackpoints;
	this->trackAngles = trackAngles;
	this->bluetooth = bluetooth;

	trackVelocity = new double[countTrackpoints];
}

// --------------------------------------------------------------------------
// Stellwert auf Grundlage des aktuellen Kurvenradius berechnen
// --------------------------------------------------------------------------
int CarControlDomiClass::calculateCurrentControlInput(double radiusCurrent)
{
	int velocity = 0;	// Stellsignal für Geschwindigkeit
	double massRatio = carMassRef / carMassActual;
	double radiusRation = radiusCurrent / radiusRef;

	// Stellsignal auf Basis der Gesetze der Zentripetalkraft berechnen
	velocity = (int) correctionFactor * sqrt(massRatio * radiusRation * pow(velocityRef, 2));

	return velocity;
}

void CarControlDomiClass::calculateGlobalControlInput()
{
	// Aktuellen Kurvenradius berechnen
	// Hierfür werden jeweils 2 Punkte vor und nach dem aktuell betrachteten für die Berechnung herangezogen

	int windowSize = 5;
	
	for (int i = 0; i < countTrackpoints; i++)
	{
		// TODO: Randbedingungen abfangen
		//trackAngles[]
	}
	
}

CarControlDomiClass::~CarControlDomiClass()
{
	delete[] trackVelocity;
}



