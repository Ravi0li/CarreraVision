#pragma once
#include <boost/thread/mutex.hpp>

class InformationShareClass
{
private:
	boost::mutex mtx_;
	int carPosition;
	int carType;
	long time;
public:
	InformationShareClass();
	void lock() ;
	void unlock();
	void SetPosition(int carPosition);
	void SetType(int carType);
	void SetTime(int time);
	int GetPosition();
	int GetType();
	int GetTime();
};
