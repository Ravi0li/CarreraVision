#include "carControlDomiClass.h"
#include <cmath>
#include <boost/thread.hpp>

#define M_PI			3.14159265358979323846
#define MAX_VELOCITY	255
#define MIN_VELOCITY	0

// --------------------------------------------------------------------------
// Kalibrierte Werte festlegen
// --------------------------------------------------------------------------
CarControlDomiClass::CarControlDomiClass(InformationShareClass* infoPackage, int countTrackpoints, boost::geometry::model::d2::point_xy<float>* cartesianTrackPoints, BluetoothConnectionClass* bluetooth, float pointDistance)
{
	// Diese Werte müssen einmalig per Kalibrierung festgelegt und händisch gemessen werden
	refCarMass = (float) 0.150;		// kg
	actualCarMass =  (float) 0.150;	// kg
	refRadius = (float) 0.5;		// m
	refVelocity = (float) 120;		// zwischen 0 und 255

	// Korrekturfaktor bestimmt Verhältnis zum berechneten maximal möglichen Stellsignal
	correctionFactor = (float) 0.85;

	// Schnittstelle
	this->infoPackage = infoPackage;
	this->countTrackpoints = countTrackpoints;
	this->cartesianTrackPoints = cartesianTrackPoints;
	this->bluetooth = bluetooth;
	this->pointDistance = pointDistance;

	trackVelocity = new int[countTrackpoints];
}

// --------------------------------------------------------------------------
// Stellwert auf Grundlage des aktuellen Kurvenradius berechnen
// --------------------------------------------------------------------------
int CarControlDomiClass::calculateCurrentControlInput(float currentRadius)
{
	int velocity = 0;	// Stellsignal für Geschwindigkeit
	float massRatio = refCarMass / actualCarMass;

	//  Verhältnis von Referenzradius und aktuellem Radius
	float radiusRation = currentRadius / refRadius;

	// Abfragen, ob Radius legitim ist, da eine Gerade z.B. keinen Radius besitzt
	if (currentRadius != std::numeric_limits<float>::infinity())
	{
		// Stellsignal auf Basis der Gesetze der Zentripetalkraft berechnen
		velocity = (int)correctionFactor * sqrt(massRatio * radiusRation * pow(refVelocity, 2));
		velocity = std::min(std::max(MIN_VELOCITY, velocity), MAX_VELOCITY);
	}
	else
	{
		velocity = MAX_VELOCITY;
	}


	return velocity;
}

void CarControlDomiClass::calculateGlobalControlInput()
{
	// Aktuellen Kurvenradius auf Basis der Menger Krümmung berechnen
	// Hierfür werden jeweils 3 benachbarte Punkte betrachtet
	float currentRadius = 0.0;

	for (int i = 0; i < countTrackpoints; i++)
	{
		// R = (|Punkt1-Punkt2| * |Punkt2-Punkt3| * |Punkt3-Punkt1|) / (4 * A)
		float diff1, diff2, diff3, area4;

		if (i == 0)
		{
			// Erster Streckenpunkt
			diff1 = distanceBetweenPoints(cartesianTrackPoints[countTrackpoints - 1], cartesianTrackPoints[0]);	// Abstand Punkt1 - Punkt2
			diff2 = distanceBetweenPoints(cartesianTrackPoints[0], cartesianTrackPoints[1]);					// Abstand Punkt2 - Punkt3
			diff3 = distanceBetweenPoints(cartesianTrackPoints[1], cartesianTrackPoints[countTrackpoints - 1]);	// Abstand Punkt3 - Punkt1

			// Berechne vierfache Dreiecksfläche 
			area4 = 2.0 * abs(twiceSignedArea(cartesianTrackPoints[countTrackpoints - 1], cartesianTrackPoints[0], cartesianTrackPoints[1]));
		}
		else if (i == countTrackpoints - 1)
		{
			// Letzter Streckenpunkt
			diff1 = distanceBetweenPoints(cartesianTrackPoints[countTrackpoints - 2], cartesianTrackPoints[countTrackpoints - 1]);	// Abstand Punkt1 - Punkt2
			diff2 = distanceBetweenPoints(cartesianTrackPoints[countTrackpoints - 1], cartesianTrackPoints[0]);						// Abstand Punkt2 - Punkt3
			diff3 = distanceBetweenPoints(cartesianTrackPoints[0], cartesianTrackPoints[countTrackpoints - 2]);						// Abstand Punkt3 - Punkt1

			// Berechne vierfache Dreiecksfläche 
			area4 = 2.0 * abs(twiceSignedArea(cartesianTrackPoints[i - 2], cartesianTrackPoints[countTrackpoints - 1], cartesianTrackPoints[0]));
		}
		else
		{
			// "Normalfall"
			diff1 = distanceBetweenPoints(cartesianTrackPoints[i - 1], cartesianTrackPoints[i]);	// Abstand Punkt1 - Punkt2
			diff2 = distanceBetweenPoints(cartesianTrackPoints[i], cartesianTrackPoints[i + 1]);	// Abstand Punkt2 - Punkt3
			diff3 = distanceBetweenPoints(cartesianTrackPoints[i + 1], cartesianTrackPoints[i]);	// Abstand Punkt3 - Punkt1	
			
			// Berechne vierfache Dreiecksfläche 
			area4 = 2.0 * abs(twiceSignedArea(cartesianTrackPoints[i - 1], cartesianTrackPoints[i], cartesianTrackPoints[i + 1]));
		}
	
		// Berechne Krümmungsradius
		if (area4 < 0.05 * pointDistance)
		{
			// Hier kann man davon ausgehen, dass die Punkte auf einer Geraden liegen
			// Dieser Fall muss abgefangen werden, da sonst der Wertebereich der float Variable evtl. überschritten wird
			currentRadius = std::numeric_limits<float>::infinity();
		}
		else
		{
			currentRadius = (diff1 * diff2 * diff3) / area4;
		}
		
		// Maximal mögliches Stellsignal berechnen
		trackVelocity[i] = calculateCurrentControlInput(currentRadius);

	}	
}

// --------------------------------------------------------------------------
// Abstand zwischen zwei Punkten berechnen
// --------------------------------------------------------------------------
float CarControlDomiClass::distanceBetweenPoints(boost::geometry::model::d2::point_xy<float> point1, boost::geometry::model::d2::point_xy<float> point2)
{
	float dx = point1.x() - point2.x();
	float dy = point1.y() - point2.y();

	return sqrt(dx * dx + dy * dy);
}

// --------------------------------------------------------------------------
// Zweifache (vorzeichenbehaftete) Fläche des Dreiecks berechnen, welches durch 3 Punkte gegeben ist
// --------------------------------------------------------------------------
float CarControlDomiClass::twiceSignedArea(boost::geometry::model::d2::point_xy<float> point1, boost::geometry::model::d2::point_xy<float> point2, boost::geometry::model::d2::point_xy<float> point3)
{
	return (point2.x() - point1.x())*(point3.y() - point1.y()) - (point2.y() - point1.y())*(point3.x() - point1.x());
}

// --------------------------------------------------------------------------
// Thread zum Ansteuern des Autos
// --------------------------------------------------------------------------
void CarControlDomiClass::loopingThread()
{	
	int delaySamples = 5;

	// Maximal mögliche Geschwindigkeit / Stellsignal für jeden Streckenabschnitt berechnen
	calculateGlobalControlInput();

	// TODO: bisschen mehr Intelligenz braucht man wohl
	while (1)
	{
		int position = 0;

		// TODO: 15 ms warten

		infoPackage->lock();
		position = infoPackage->GetPosition();
		infoPackage->unlock();

		// Stellsignal per BT senden
		bluetooth->sendChannel1(trackVelocity[(position + delaySamples) % countTrackpoints]);
		//oder
		//bluetooth->sendChannel2;
	}
}

// --------------------------------------------------------------------------
// Aufräumen
// --------------------------------------------------------------------------
CarControlDomiClass::~CarControlDomiClass()
{
	delete[] trackVelocity;
	delete[] cartesianTrackPoints;
}



