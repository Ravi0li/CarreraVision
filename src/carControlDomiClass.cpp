#define DEBUG_CAR_CONTROL

#include "carControlDomiClass.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <cmath>
#include <boost/thread.hpp>
#include <iostream>
#include <string>
#include <algorithm>
#include <thread>

#ifdef DEBUG_CAR_CONTROL
#include <fstream>
#endif

#define M_PI			3.14159265358979323846
#define MAX_VELOCITY	180
#define MIN_VELOCITY	0

// --------------------------------------------------------------------------
// Kalibrierte Werte festlegen
// --------------------------------------------------------------------------
CarControlDomiClass::CarControlDomiClass(InformationShareClass* infoPackage, int countTrackpoints, std::vector<cv::Point2f>* cartesianTrackPoints, BluetoothConnectionClass* bluetoothObject, int channel, float pointDistance)
{
	// Diese Werte müssen einmalig per Kalibrierung festgelegt und händisch gemessen werden
	refCarMass = (float) 0.100;		// kg
	actualCarMass =  (float) 0.150;	// kg
	refRadius = (float) 0.25;		// m
	refVelocity = 120;				// zwischen 0 und 255
	brakingFactor = (float) 20.0;	// ohne Einheit

	// Korrekturfaktor bestimmt Verhältnis zum berechneten maximal möglichen Stellsignal
	correctionFactor = (float) 1.00;

	// Schnittstelle
	this->infoPackage = infoPackage;
	this->countTrackpoints = countTrackpoints;
	this->cartesianTrackPoints = cartesianTrackPoints;
	this->bluetoothObject = bluetoothObject;
	this->channel = channel;
	this->pointDistance = pointDistance;
	this->minimumVelocity = MIN_VELOCITY;
	this->direction = 1;																					

	trackVelocityNoBraking = new int[countTrackpoints];
	trackVelocityDirection1 = new int[countTrackpoints];
	trackVelocityDirection2 = new int[countTrackpoints];

	trackVelocityDirectionDrive = trackVelocityDirection1;
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

	// besser: Median berechnen, TODO: braucht so noch ewig -> Herolds geilen Algorithmus mit qsort verwenden!
	for (int i = 0; i < countTrackpoints; i++)
	{
		for (int j = 0; j < windowSize; j++)
		{
			if ((i < windowSize / 2) && (j < windowSize / 2))
			{
				medianBuffer[j] = trackVelocityNoBraking[countTrackpoints - windowSize / 2 + j];
			}
			else
			{
				medianBuffer[j] = trackVelocityNoBraking[(i - windowSize / 2 + j) % countTrackpoints];
			}

		}

		// Median für aktuelles Fenster ermitteln
		std::sort(medianBuffer, medianBuffer + windowSize);

		filteredVelocity[i] = medianBuffer[windowSize / 2];
	}

	// Gefiltertes Array kopieren
	for (int i = 0; i < countTrackpoints; i++)
	{
		trackVelocityNoBraking[i] = filteredVelocity[i];
	}

	delete[] filteredVelocity;
}

// --------------------------------------------------------------------------
// Berechnete Stellsignale mit Bremspunkten versehen
// --------------------------------------------------------------------------
void CarControlDomiClass::calculateBreakpoints()
{
	/*cv::Mat src, dst1, dst2;
	cv::Mat kernel;
	cv::Point anchor;

	// Stellsignale laden
	src = cv::Mat::zeros(1, countTrackpoints, CV_8U);
	dst1 = cv::Mat::zeros(1, countTrackpoints, CV_8U);
	dst2 = cv::Mat::zeros(1, countTrackpoints, CV_8U);

	for (int i = 0; i < countTrackpoints; i++)
	{
		src.at<uchar>(cv::Point(i, 0)) = (uchar) trackVelocityNoBraking[i];
	}

	// Argumente für Filterung
	anchor = cv::Point(-1, -1);		// Mittelpunkt des Filterkernels verwenden								

	// Filtermasken festlegen und filtern
	kernel = (cv::Mat_<double>(1, 7) << 0, 0, 0, 0.33, 0.33, 0.33, 0.33);
	filter2D(src, dst1, -1, kernel, anchor, 0);
	//filter2D(src, dst1, -1, kernel, anchor, 0, cv::BORDER_WRAP);

	kernel = (cv::Mat_<double>(1, 7) << 0.33, 0.33, 0.33, 0.33, 0, 0, 0);
	//filter2D(src, dst2, -1, kernel, anchor, 0, cv::BORDER_WRAP);

	for (int i = 0; i < countTrackpoints; i++)
	{
		trackVelocityDirection1[i] = dst1.at<uchar>(i);
		trackVelocityDirection2[i] = dst2.at<uchar>(i);
	}*/

	for (int i = 0; i < countTrackpoints; i++)
	{
		int newVal = 0, useVal;
		useVal = trackVelocityNoBraking[i];
		newVal = trackVelocityNoBraking[(i+1) % countTrackpoints];
		if (newVal < useVal) useVal = newVal;
		newVal = trackVelocityNoBraking[(i+2) % countTrackpoints];
		if (newVal < useVal) useVal = newVal;
		newVal = trackVelocityNoBraking[(i+3) % countTrackpoints];
		if (newVal < useVal) useVal = newVal;
		newVal = trackVelocityNoBraking[(i+4) % countTrackpoints];
		if (newVal < useVal) useVal = newVal;
		trackVelocityDirection1[i] = useVal;
	}

	for (int i = 4; i < countTrackpoints+4; i++)
	{
		int newVal = 0, useVal;
		useVal = trackVelocityNoBraking[i % countTrackpoints];
		newVal = trackVelocityNoBraking[(i - 1) % countTrackpoints];
		if (newVal < useVal) useVal = newVal;
		newVal = trackVelocityNoBraking[(i - 2) % countTrackpoints];
		if (newVal < useVal) useVal = newVal;
		newVal = trackVelocityNoBraking[(i - 3) % countTrackpoints];
		if (newVal < useVal) useVal = newVal;
		newVal = trackVelocityNoBraking[(i - 4) % countTrackpoints];
		if (newVal < useVal) useVal = newVal;
		trackVelocityDirection2[i] = useVal;
	}

	#ifdef DEBUG_CAR_CONTROL
		const std::string s1("mit_glaetten_mit_bremsen1");
		outputArrayAsCSV(trackVelocityDirection1, countTrackpoints, s1);

		const std::string s2("mit_glaetten_mit_bremsen2");
		outputArrayAsCSV(trackVelocityDirection2, countTrackpoints, s2);
	#endif
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

// --------------------------------------------------------------------------
// Kleinstes Stellsignal der Strecke berechnen. Wird verwendet, wenn 
// keine Position übergeben wird. Das Array mit den Stellsignalen sollte 
// vorher bereits gefiltert sein.
// --------------------------------------------------------------------------
void CarControlDomiClass::calculateMinmumControlInput()
{
	int min = MAX_VELOCITY;

	for (int i = 0; i < countTrackpoints; i++)
	{
		if (trackVelocityNoBraking[i] < min)
		{
			min = trackVelocityNoBraking[i];
		}
	}

	minimumVelocity = min;
}

// --------------------------------------------------------------------------
// Aktuellen Kurvenradius auf Basis der Menger Krümmung berechnen
// --------------------------------------------------------------------------
void CarControlDomiClass::calculateGlobalControlInput()
{
	// Hierfür werden jeweils 3 benachbarte Punkte betrachtet
	float currentRadius = 0.0;

	for (int i = 0; i < countTrackpoints; i++)
	{
		float diff1, diff2, diff3, area4;

		if (i == 0)
		{
			// Erster Streckenpunkt
			diff1 = distanceBetweenPoints((*cartesianTrackPoints)[countTrackpoints - 1], (*cartesianTrackPoints)[0]);		// Abstand Punkt1 - Punkt2
			diff2 = distanceBetweenPoints((*cartesianTrackPoints)[0], (*cartesianTrackPoints)[1]);							// Abstand Punkt2 - Punkt3
			diff3 = distanceBetweenPoints((*cartesianTrackPoints)[1], (*cartesianTrackPoints)[countTrackpoints - 1]);		// Abstand Punkt3 - Punkt1

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
		trackVelocityNoBraking[i] = calculateCurrentControlInput(currentRadius/1000);
	}


	#ifdef DEBUG_CAR_CONTROL
		const std::string s1("ohne_glaetten_ohne_bremsen");
		outputArrayAsCSV(trackVelocityNoBraking, countTrackpoints, s1);
	#endif

	// Stellsignale glätten
	smoothTrackVelocity();

	// Minimum berechnen als "Standardwert", falls keine Position übergeben wird
	calculateMinmumControlInput();

	// Stellsignale mit Bremspunkten berechnen
	calculateBreakpoints();

	#ifdef DEBUG_CAR_CONTROL
		const std::string s2("mit_glaetten_ohne_bremsen");
		outputArrayAsCSV(trackVelocityNoBraking, countTrackpoints, s2);
	#endif

	infoPackage->lock();
	infoPackage->SetTrackVelocity(trackVelocityDirectionDrive);
	infoPackage->unlock();
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
		std::this_thread::sleep_for(std::chrono::milliseconds(10));

		/*for (int i = 0; i < countTrackpoints; i++)
		{
			std::cout << i << ": ";
			std::cout << trackVelocity[i] << std::endl;
		}*/

		infoPackage->lock();
		position = infoPackage->GetPosition();
		infoPackage->unlock();

		if (trackVelocityDirectionDrive[position] <= 120)
			delaySamples = 2;
		else if (trackVelocityDirectionDrive[position] <= 160 && trackVelocityDirectionDrive[position] > 120)
			delaySamples = 4;
		else if (trackVelocityDirectionDrive[position] > 160)
			delaySamples = 6;
	

		// Stellsignal per BT senden
		if (channel == 1)
		{
			if (position != -1)
			{
				bluetoothObject->sendChannel1(direction * trackVelocityDirectionDrive[(position + delaySamples) % countTrackpoints]);
			}
			else
			{
				bluetoothObject->sendChannel1(direction * minimumVelocity);
			}

			//std::cout << "Sende Kanal 1" << std::endl;
		}
		else if (channel == 2)
		{
			if (position != -1)
			{
				bluetoothObject->sendChannel2(direction * trackVelocityDirectionDrive[(position + delaySamples) % countTrackpoints]);
			}
			else
			{
				bluetoothObject->sendChannel2(direction * minimumVelocity);
			}

			//std::cout << "Sende Kanal 2" << std::endl;
		}
	}
}

// --------------------------------------------------------------------------
// Umschalten der Richtung, Abfolge der Werte: -1 -> 0 -> 1 -> -1 ...
// --------------------------------------------------------------------------
void CarControlDomiClass::toggleDirection()
{
	direction = ((direction + 2) % 3) - 1;
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
	delete[] trackVelocityNoBraking;
	//delete[] trackVelocityDirection1;
	//delete[] trackVelocityDirection2;
}

// --------------------------------------------------------------------------
// 
// --------------------------------------------------------------------------
void CarControlDomiClass::ChangeVelocityDirection()
{
	if (trackVelocityDirectionDrive == trackVelocityDirection1)
	{
		trackVelocityDirectionDrive = trackVelocityDirection2;
	}
	else if (trackVelocityDirectionDrive == trackVelocityDirection2)
	{
		trackVelocityDirectionDrive = trackVelocityDirection1;
	}

	infoPackage->lock();
	infoPackage->SetTrackVelocity(trackVelocityDirectionDrive);
	infoPackage->unlock();
}



