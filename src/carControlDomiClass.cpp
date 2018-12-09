#define DEBUG_CAR_CONTROL

#include "carControlDomiClass.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <cmath>
#include <boost/thread.hpp>
#include <iostream>
#include <string>
#include <algorithm>
#include <thread>
#include <deque>
#include <numeric>       // accumulate

#ifdef DEBUG_CAR_CONTROL
#include <fstream>
#endif

#define M_PI			3.14159265358979323846

// --------------------------------------------------------------------------
// Kalibrierte Werte festlegen
// --------------------------------------------------------------------------
CarControlDomiClass::CarControlDomiClass(cv::FileNode _para, InformationShareClass* infoPackage, int countTrackpoints, std::vector<cv::Point2f>* cartesianTrackPoints, BluetoothConnectionClass* bluetoothObject, int channel, float pointDistance)
{
	para = _para;

	// Diese Werte müssen einmalig per Kalibrierung festgelegt und händisch gemessen werden
	refCarMass = (float)para["ref_car_mass"];		// kg
	actualCarMass = (float)para["actual_car_mass"];	// kg
	refRadius = (float)para["ref_radius"];			// m
	refVelocity = (int)para["ref_velocity"];		// zwischen 0 und 255
	brakingFactor = (float)para["braking_factor"];	// ohne Einheit

	// Korrekturfaktor bestimmt Verhältnis zum berechneten maximal möglichen Stellsignal
	correctionFactor = (float)para["correction_factor"];

	// Schnittstelle
	this->infoPackage = infoPackage;
	this->countTrackpoints = countTrackpoints;
	this->cartesianTrackPoints = cartesianTrackPoints;
	this->bluetoothObject = bluetoothObject;
	this->channel = channel;
	this->pointDistance = pointDistance;
	this->minimumVelocity = 0;
	this->direction = 1;																					

	trackVelocityNoBraking.resize(countTrackpoints, 0);
	trackVelocityDirection1.resize(countTrackpoints, 0);
	trackVelocityDirection2.resize(countTrackpoints, 0);

	trackVelocityDirectionDrive = &trackVelocityDirection1;
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
		velocity = std::min(std::max((int)para["min_velocity"], velocity), (int)para["max_velocity"]);
	}
	else
	{
		velocity = (int)para["max_velocity"];
	}

	return velocity;
}

// --------------------------------------------------------------------------
// Berechnete Stellsignale glätten
// --------------------------------------------------------------------------
void CarControlDomiClass::smoothTrackVelocity()
{
	const int windowSize = (int)para["window_size_soothless"];	// sollte ungerade sein
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
std::vector<int> CarControlDomiClass::calculateBreakpointAlgorithm(const std::vector<int> &noBreaking)
{
	// Parameter
	int showInFuture = (int)para["break_show_in_future_width"];
	int showInFutureComplex = (int)para["complex_break_show_in_future_width"];

	// Kleinsten Wert aus den nächsten 5 Werten nehmen
	std::vector<int> simpleBreak;
	std::deque<int> area(noBreaking.end() - showInFuture, noBreaking.end());
	for (size_t i = 0; i < countTrackpoints; ++i)
	{
		area.push_back(noBreaking[i]);
		int minValue = *std::min_element(area.begin(), area.end());
		simpleBreak.push_back(minValue);
		area.pop_front();
	}

	// verstärkes Bremsen bei großen Unterschieden
	std::deque<int> complexBreak;
	std::deque<int> areaSuper(simpleBreak.begin(), simpleBreak.begin() + showInFutureComplex);
	for (int i = countTrackpoints - 1; i >= 0; --i)
	{
		areaSuper.push_front(simpleBreak[i]);
		float avg = std::accumulate(areaSuper.begin(), areaSuper.end(), 0) / (float)areaSuper.size();
		float factor = simpleBreak[i] / avg;
		if (factor > 1)
			factor = 1.f;
		complexBreak.push_front((int)(simpleBreak[i] * factor));
		areaSuper.pop_back();
	}

	// rueckgabe
	std::vector<int> ret(complexBreak.begin(), complexBreak.end());
	return ret;
}

// --------------------------------------------------------------------------
// Berechnete Stellsignale mit Bremspunkten versehen
// --------------------------------------------------------------------------
void CarControlDomiClass::calculateBreakpoints()
{
	// Berechnen der Zukunftswerte
	if (countTrackpoints > 10)
	{
		// Richtung 1
		trackVelocityDirection1 = calculateBreakpointAlgorithm(trackVelocityNoBraking);

		// Richtung 2
		std::vector<int> trackVelocityNoBrakingReverse(countTrackpoints);
		std::reverse_copy(trackVelocityNoBraking.begin(), trackVelocityNoBraking.end(), trackVelocityNoBrakingReverse.begin());
		std::vector<int> trackVelocityDirection2Reverse = calculateBreakpointAlgorithm(trackVelocityNoBrakingReverse);
		trackVelocityDirection2.reserve(countTrackpoints);
		std::reverse_copy(trackVelocityDirection2Reverse.begin(), trackVelocityDirection2Reverse.end(), trackVelocityDirection2.begin());
	}
	else
	{
		trackVelocityDirection1 = trackVelocityNoBraking;
		trackVelocityDirection2 = trackVelocityNoBraking;
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
void CarControlDomiClass::outputArrayAsCSV(const std::vector<int>& arrayToConvert, int length, std::string label)
{
	std::ofstream fileHandle;

	fileHandle.open(label + ".csv");
	if (!fileHandle)
		return;

	// Alle Streckenpunkte als CSV einfügen
	for (int i = 0; i < length; i++)
	{
		if (!fileHandle)
			return;
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
	int min = (int)para["max_velocity"];

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
	startFrame = std::chrono::high_resolution_clock::now();
	while (!stop)
	{
		// Frameratenmessung
		auto now = std::chrono::high_resolution_clock::now();
		int diff = (int)std::chrono::duration_cast<std::chrono::milliseconds>(now - startFrame).count();
		if (diff > 1000)
		{
			frameMutex.lock();
			frameRate = (int)(countFrame / (diff / 1000.0));
			frameMutex.unlock();
			countFrame = 0;
			startFrame = now;
		}
		countFrame++;

		int position = 0;

		std::this_thread::sleep_for(std::chrono::milliseconds(20));

		infoPackage->lock();
		position = infoPackage->GetPosition();
		infoPackage->unlock();

		// Prüft ob sich position verändert hat
		if (position == samePosBrain)
		{
			samePosCounter++;
		}
		else
		{
			samePosBrain = position;
			samePosCounter = 0;
		}
		
		// Prüfen ob eine Position bekannt ist
		if (position != -1 && samePosCounter < (int)para["default_speed_after_x_same_pos"])
		{
			if ((*trackVelocityDirectionDrive)[position] <= 100)
				delaySamples = (int)para["delay_samples_slow"];
			else
				delaySamples = (int)para["delay_samples_fast"];
			if (trackVelocityDirectionDrive == &trackVelocityDirection1)
				delaySamples = -delaySamples;
			
			// Stellsignal erstellen
			int usePos = position + delaySamples;
			if (usePos < 0) usePos += trackVelocityDirectionDrive->size();
			usePos = usePos % countTrackpoints;
			bluetoothObject->setSendValue(channel, direction * (*trackVelocityDirectionDrive)[usePos]);
		}
		else
		{
			// kein stellsignal bekannt, also Standardwert
			bluetoothObject->setSendValue(channel, direction * minimumVelocity);
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
// Wechseln der Fahrtrichtung
// --------------------------------------------------------------------------
void CarControlDomiClass::ChangeVelocityDirection()
{
	if (trackVelocityDirectionDrive == &trackVelocityDirection1)
	{
		trackVelocityDirectionDrive = &trackVelocityDirection2;
	}
	else if (trackVelocityDirectionDrive == &trackVelocityDirection2)
	{
		trackVelocityDirectionDrive = &trackVelocityDirection1;
	}

	infoPackage->lock();
	infoPackage->SetTrackVelocity(trackVelocityDirectionDrive);
	infoPackage->unlock();
}

// --------------------------------------------------------------------------
// zeigt die aktuelle Framerate an
// --------------------------------------------------------------------------
int CarControlDomiClass::getFrameRate()
{
	frameMutex.lock();
	int ret = frameRate;
	frameMutex.unlock();
	return ret;
}


