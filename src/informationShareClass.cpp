#include "informationShareClass.h"

InformationShareClass::InformationShareClass()
{
	carPosition = -1;
	carType = 0;
	time = 0;
	trackVelocity = NULL;
	picID = 0;
}

void InformationShareClass::lock() 
{
	mtx_.lock();
}

void InformationShareClass::unlock() 
{
	mtx_.unlock();
}

void InformationShareClass::SetPosition(int carPosition)
{
	this->carPosition = carPosition;
}

void InformationShareClass::SetType(int carType)
{
	this->carType = carType;
}

void InformationShareClass::SetTime(int time)
{
	this->time = time;
}

void InformationShareClass::SetTrackVelocity(int* trackVelocity)
{
	this->trackVelocity = trackVelocity;
}

void InformationShareClass::DecPicID()
{
	picID++;
}

int InformationShareClass::GetPosition()
{
	return carPosition;
}

int InformationShareClass::GetType()
{
	return carType;
}

int InformationShareClass::GetTime()
{
	return time;
}

int* InformationShareClass::GetTrackVelocity()
{
	return this->trackVelocity;
}

int InformationShareClass::GetPicID()
{
	return picID;
}