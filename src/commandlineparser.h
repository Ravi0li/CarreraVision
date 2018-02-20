#pragma once

// --------------------------------------------------------------------------
// Hilfe anzeigen für den Command Line Parser
// --------------------------------------------------------------------------
static void clpHelp()
{
	std::cout << "Hier soll eine Hilfe stehen" << std::endl;
}

// --------------------------------------------------------------------------
// Keys nach denen der Command Line Parser arbeitet
// --------------------------------------------------------------------------
const char* clpKeys =
{
	"{help h    |               | }"
	"{trackimg  | ./example.png | image of the Track}"
	"{debugwin  |               | zeigt alle Fenster an zum Debuggen}"
};