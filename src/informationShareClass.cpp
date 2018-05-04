#include "informationshareClass.h"

InformationShareClass::InformationShareClass()
{
	carPosition = 0;
	carType = 0;
	time = 0;
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