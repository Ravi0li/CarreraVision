#pragma once

// --------------------------------------------------------------------------
// Hilfe anzeigen für den Command Line Parser
// --------------------------------------------------------------------------
static void clpHelp()
{
	std::cout << "Automatisierte Steuerung einer Carrerabahn Profi" << std::endl
		      << std::endl
		      << "CarreraVision [-h] [-trackimg BILD.jpg] [-para PARAM.xml] [-debugwin]" << std::endl
		      << std::endl
		      << "  -h           Zeigt eine Hilfe an" << std::endl
			  << "  -trackimg    Gibt eine Bilddatei vor, aus der die Strecke berechnet wird" << std::endl
			  << "  -para        Gibt eine Datei vor aus der die Parameter zur Bildverarbeitung gelesen werden" << std::endl
			  << "  -debugwin    Aktiviert die Anzeige der zusätzlichen Fenster zum Debuggen" << std::endl;
}

// --------------------------------------------------------------------------
// Keys nach denen der Command Line Parser arbeitet
// --------------------------------------------------------------------------
const char* clpKeys =
{
	"{help h    |               | Hilfe}"
	"{trackimg  | ./example.png | Bilder der Strecke}"
	"{para      | ./para.xml    | Datei mit allen Parametern zur Verarbeitung}"
	"{debugwin  |               | Zeigt alle Fenster an zum Debuggen}"
};