#pragma once
#include <boost/thread/mutex.hpp>
#include <vector>

class InformationShareClass
{
private:
	boost::mutex mtx_;
	int carPosition;
	int carType;
	long time;
	std::vector<int> *trackVelocity;
	int picID;
public:
	InformationShareClass();
	void lock() ;
	void unlock();
	void SetPosition(int carPosition);
	void SetType(int carType);
	void SetTime(int time);
	void SetTrackVelocity(std::vector<int> *trackVelocity);
	void DecPicID();
	int GetPosition();
	int GetType();
	int GetTime();
	std::vector<int>* GetTrackVelocity();
	int GetPicID();
};
