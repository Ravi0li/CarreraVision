#define DEBUG_CAR_CONTROL

#include "carControlDomiClass.h"
#include <cmath>
#include <boost/thread.hpp>
#include <iostream>
#include <string>
#include <algorithm>

#ifdef DEBUG_CAR_CONTROL
#include <fstream>
#endif

#define M_PI			3.14159265358979323846
#define MAX_VELOCITY	255
#define MIN_VELOCITY	0



// --------------------------------------------------------------------------
// Kalibrierte Werte festlegen
// --------------------------------------------------------------------------
CarControlDomiClass::CarControlDomiClass(InformationShareClass* infoPackage, int countTrackpoints, std::vector<cv::Point2f>* cartesianTrackPoints, BluetoothConnectionClass* bluetoothObject, int channel, float pointDistance)
{
	// Diese Werte müssen einmalig per Kalibrierung festgelegt und händisch gemessen werden
	refCarMass = (float) 0.150;		// kg
	actualCarMass =  (float) 0.150;	// kg
	refRadius = (float) 0.25;		// m
	refVelocity = 100;				// zwischen 0 und 255
	brakingFactor = (float) 20.0;	// ohne Einheit

	// Korrekturfaktor bestimmt Verhältnis zum berechneten maximal möglichen Stellsignal
	correctionFactor = (float) 0.85;

	// Schnittstelle
	this->infoPackage = infoPackage;
	this->countTrackpoints = countTrackpoints;
	this->cartesianTrackPoints = cartesianTrackPoints;
	this->bluetoothObject = bluetoothObject;
	this->channel = channel;
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
		velocity = correctionFactor * sqrt(massRatio * radiusRation * (float) pow(refVelocity, 2));
		velocity = std::min(std::max(MIN_VELOCITY, velocity), MAX_VELOCITY);
	}
	else
	{
		velocity = MAX_VELOCITY;
	}

	return velocity;
}

// --------------------------------------------------------------------------
// Berechnete Stellsignale glätten
// --------------------------------------------------------------------------
void CarControlDomiClass::smoothTrackVelocity()
{
	const int windowSize = 7;	// sollte ungerade sein
	int* filteredVelocity = new int[countTrackpoints];
	int* medianBuffer = new int[windowSize];

	// Array initialisieren
	for (int i = 0; i < countTrackpoints; i++)
	{
		filteredVelocity[i] = 0;
	}

	// Mittelwert berechnen
	/*for (int i = 0; i < countTrackpoints; i++)
	{
		for (int j = 0; j < windowSize; j++)
		{
			if ((i < windowSize / 2) && (j < windowSize / 2))
			{
				filteredVelocity[i] += trackVelocity[countTrackpoints - windowSize / 2 + j];
			}
			else
			{
				filteredVelocity[i] += trackVelocity[(i - windowSize / 2 + j) % countTrackpoints];
			}
			
			filteredVelocity[i] /= windowSize;
		}	
	}*/

	// besser: Median berechnen, TODO: braucht so noch ewig -> Herolds geilen Algorithmus mit qsort verwenden!
	for (int i = 0; i < countTrackpoints; i++)
	{
		for (int j = 0; j < windowSize; j++)
		{
			if ((i < windowSize / 2) && (j < windowSize / 2))
			{
				medianBuffer[j] = trackVelocity[countTrackpoints - windowSize / 2 + j];
			}
			else
			{
				medianBuffer[j] = trackVelocity[(i - windowSize / 2 + j) % countTrackpoints];
			}

		}

		// Median für aktuelles Fenster ermitteln
		std::sort(medianBuffer, medianBuffer + windowSize);

		filteredVelocity[i] = medianBuffer[windowSize / 2];
	}

	// Gefiltertes Array kopieren
	for (int i = 0; i < countTrackpoints; i++)
	{
		trackVelocity[i] = filteredVelocity[i];
	}

	delete[] filteredVelocity;
}

// --------------------------------------------------------------------------
// Berechnete Stellsignale mit Bremspunkten versehen
// --------------------------------------------------------------------------
void CarControlDomiClass::calculateBreakpoints()
{


}

// --------------------------------------------------------------------------
// Array als XML in Datei speichern zum Debuggen
// --------------------------------------------------------------------------
void CarControlDomiClass::outputArrayAsCSV(int* arrayToConvert, int length, std::string label)
{
	std::ofstream fileHandle;

	fileHandle.open(label + ".csv");

	// Alle Streckenpunkte als CSV einfügen
	for (int i = 0; i < length; i++)
	{
		fileHandle << arrayToConvert[i] << ";" << "\n";
	}
	
	fileHandle.close();
}

void CarControlDomiClass::calculateGlobalControlInput()
{
	// Aktuellen Kurvenradius auf Basis der Menger Krümmung berechnen
	// Hierfür werden jeweils 3 benachbarte Punkte betrachtet
	float currentRadius = 0.0;

	for (int i = 0; i < countTrackpoints; i++)
	{
		float diff1, diff2, diff3, area4;

		if (i == 0)
		{
			// Erster Streckenpunkt
			diff1 = distanceBetweenPoints((*cartesianTrackPoints)[countTrackpoints - 1], (*cartesianTrackPoints)[0]);	// Abstand Punkt1 - Punkt2
			diff2 = distanceBetweenPoints((*cartesianTrackPoints)[0], (*cartesianTrackPoints)[1]);					// Abstand Punkt2 - Punkt3
			diff3 = distanceBetweenPoints((*cartesianTrackPoints)[1], (*cartesianTrackPoints)[countTrackpoints - 1]);	// Abstand Punkt3 - Punkt1

			// Berechne vierfache Dreiecksfläche 
			area4 = 2.0 * abs(twiceSignedArea((*cartesianTrackPoints)[countTrackpoints - 1], (*cartesianTrackPoints)[0], (*cartesianTrackPoints)[1]));
		}
		else if (i == countTrackpoints - 1)
		{
			// Letzter Streckenpunkt
			diff1 = distanceBetweenPoints((*cartesianTrackPoints)[countTrackpoints - 2], (*cartesianTrackPoints)[countTrackpoints - 1]);	// Abstand Punkt1 - Punkt2
			diff2 = distanceBetweenPoints((*cartesianTrackPoints)[countTrackpoints - 1], (*cartesianTrackPoints)[0]);						// Abstand Punkt2 - Punkt3
			diff3 = distanceBetweenPoints((*cartesianTrackPoints)[0], (*cartesianTrackPoints)[countTrackpoints - 2]);						// Abstand Punkt3 - Punkt1

			// Berechne vierfache Dreiecksfläche 
			area4 = 2.0 * abs(twiceSignedArea((*cartesianTrackPoints)[i - 2], (*cartesianTrackPoints)[countTrackpoints - 1], (*cartesianTrackPoints)[0]));
		}
		else
		{
			// "Normalfall"
			diff1 = distanceBetweenPoints((*cartesianTrackPoints)[i - 1], (*cartesianTrackPoints)[i]);	// Abstand Punkt1 - Punkt2
			diff2 = distanceBetweenPoints((*cartesianTrackPoints)[i], (*cartesianTrackPoints)[i + 1]);	// Abstand Punkt2 - Punkt3
			diff3 = distanceBetweenPoints((*cartesianTrackPoints)[i + 1], (*cartesianTrackPoints)[i]);	// Abstand Punkt3 - Punkt1	
			
			// Berechne vierfache Dreiecksfläche 
			area4 = 2.0 * abs(twiceSignedArea((*cartesianTrackPoints)[i - 1], (*cartesianTrackPoints)[i], (*cartesianTrackPoints)[i + 1]));
		}
	
		// Berechne Krümmungsradius
		// R = (|Punkt1-Punkt2| * |Punkt2-Punkt3| * |Punkt3-Punkt1|) / (4 * A)
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
		trackVelocity[i] = calculateCurrentControlInput(currentRadius/1000); // WORKAROUND! TODO!
	}


	#ifdef DEBUG_CAR_CONTROL
		const std::string s1("ohne_glaetten_ohne_bremsen");
		outputArrayAsCSV(trackVelocity, countTrackpoints, s1);
	#endif

	// Stellsignale glätten
	smoothTrackVelocity();

	#ifdef DEBUG_CAR_CONTROL
		const std::string s2("mit_glaetten_ohne_bremsen");
		outputArrayAsCSV(trackVelocity, countTrackpoints, s2);
	#endif
}

// --------------------------------------------------------------------------
// Abstand zwischen zwei Punkten berechnen
// --------------------------------------------------------------------------
float CarControlDomiClass::distanceBetweenPoints(cv::Point2f point1, cv::Point2f point2)
{
	float dx = point1.x - point2.x;
	float dy = point1.y - point2.y;

	return sqrt(dx * dx + dy * dy);
}

// --------------------------------------------------------------------------
// Zweifache (vorzeichenbehaftete) Fläche des Dreiecks berechnen, welches durch 3 Punkte gegeben ist
// --------------------------------------------------------------------------
float CarControlDomiClass::twiceSignedArea(cv::Point2f point1, cv::Point2f point2, cv::Point2f point3)
{
	return (point2.x - point1.x)*(point3.y - point1.y) - (point2.y - point1.y)*(point3.x - point1.x);
}

// --------------------------------------------------------------------------
// Thread zum Ansteuern des Autos
// --------------------------------------------------------------------------
void CarControlDomiClass::loopingThread()
{	
	int delaySamples = 0;
	
	// Maximal mögliche Geschwindigkeit / Stellsignal für jeden Streckenabschnitt berechnen
	calculateGlobalControlInput();

	// TODO: bisschen mehr Intelligenz braucht man wohl
	while (!stop)
	{
		int position = 0;

		//boost::this_thread::sleep
		std::this_thread::sleep_for(std::chrono::milliseconds(100));

		/*for (int i = 0; i < countTrackpoints; i++)
		{
			std::cout << i << ": ";
			std::cout << trackVelocity[i] << std::endl;
		}*/

		infoPackage->lock();
		position = infoPackage->GetPosition();
		infoPackage->unlock();

		// Stellsignal per BT senden
		if (channel == 1)
		{
			if (position != -1)
			{
				bluetoothObject->sendChannel1(trackVelocity[(position + delaySamples) % countTrackpoints]);
				std::cout << "Sende Kanal 1" << std::endl;
			}
		}
		else if (channel == 2)
		{
			if (position != -1)
			{
				bluetoothObject->sendChannel2(trackVelocity[(position + delaySamples) % countTrackpoints]);
				std::cout << "Sende Kanal 2" << std::endl;
			}
		}
	}
}

// --------------------------------------------------------------------------
// Stopt den Thread
// --------------------------------------------------------------------------
void CarControlDomiClass::stopThread()
{
	stop = true;
}

// --------------------------------------------------------------------------
// Aufräumen
// --------------------------------------------------------------------------
CarControlDomiClass::~CarControlDomiClass()
{
	delete[] trackVelocity;
}



