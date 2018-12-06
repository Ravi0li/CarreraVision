#pragma once
#include <boost/thread/mutex.hpp>

class InformationShareClass
{
private:
	boost::mutex mtx_;
	int carPosition;
	int carType;
	long time;
	int* trackVelocity;
	int picID;
public:
	InformationShareClass();
	void lock() ;
	void unlock();
	void SetPosition(int carPosition);
	void SetType(int carType);
	void SetTime(int time);
	void SetTrackVelocity(int* trackVelocity);
	void DecPicID();
	int GetPosition();
	int GetType();
	int GetTime();
	int* GetTrackVelocity();
	int GetPicID();
};
