#ifndef PID_SGD_h
#define PID_SGD_h
#define LIBRARY_VERSION	0.0.1

#include <mutex>
#include <tuple> // C++11

// #define PID_TRACE
using namespace std;
enum PID_FLAGS
{
	PID_P = 0b001,
	PID_I = 0b010,
	PID_D = 0b100,
};

class PID_SGD
{
public:
	//Constants used in some of the functions below
	#define AUTOMATIC	1
	#define MANUAL	0
	#define DIRECT  0
	#define REVERSE  1

	#define P_ON_M 0 // http://brettbeauregard.com/blog/2017/06/introducing-proportional-on-measurement/

	#define P_ON_E 1

	PID_SGD(double iSetpoint,
		// int iP_On,
		// int iMaxLoss,
		int iControllerDirection,
		int iPIDFlags);

	
	// * sets PID to either Manual (0) or Auto (non-0)
	void SetMode(int iMode);

	// * performs the PID calculation.  it should be
	//   called every time loop() cycles. ON/OFF and
	//   calculation frequency can be set using SetMode
	//   SetSampleTime respectively
	// bool Compute();
	std::tuple<bool, double> Compute(double input);

	// clamps the output to a specific range. 0-255 by default, but
	// it's likely the user will want to change this depending on
	// the application
	void SetOutputLimits(double iMin, double iMax);


	//available but not commonly used functions ********************************************************
	// * While most users will set the tunings once in the 
        //   constructor, this function gives the user the option
        //   of changing tunings during runtime for Adaptive control
	void SetTunings(double iKp, double iKi, double iKd);

	// Sets the Direction, or "Action" of the controller. DIRECT
	// means the output will increase when error is positive. REVERSE
	// means the opposite.  it's very unlikely that this will be needed
	// once it is set in the constructor.
	void SetControllerDirection(int iDirection);

	// Sets the frequency, in Milliseconds, with which 
	// the PID calculation is performed.  default is 100
	void SetSampleTime(int iNewSampleTimeInMilliSecond);

	double GetKp();
	double GetKi();
	double GetKd();
	int GetMode();
	int GetDirection();
	double GetTotalError();
	double GetSetpoint();

	void SetSetpoint(double iSetpoint);

	void SetMaxLoss(double iMaxLoss);

	double GetLastError();
	// double GetOutput() {return *mpOutput;}

	void DumpLearningRate();
	void LoadLearningRate();

private:
	void LogTrace(char* iStr);
	void LogTrace(double iVal);

	void Initialize();
	void ResetTotal(unsigned long iNow);
	unsigned long millis();
    
	// * (P)roportional Tuning Parameter
	double mKp;

	// * (I)ntegral Tuning Parameter
	double mKi;

	// * (D)erivative Tuning Parameter
	double mKd;

	int mControllerDirection;
	// int mP_On;
	int mPIDFlags;

#ifdef PID_TRACE
	String mDebugStr;
#endif
	double mTotalError;
	double mTimeSpan;
	double mLastInput;
	double mLastError;
	double mLastDInput;

	// * Pointers to the Input, Output, and Setpoint variables
	// This creates a hard link between the variables and the 
	// PID, freeing the user from having to constantly tell us
	// what these values are.  with pointers we'll just know.
	// double* mpInput;
	// double* mpOutput;
	double mSetpoint;
			  
	unsigned long mLastTime;
	double mOutputSum;

	unsigned long mSampleTimeInMilliSecond;
	double mOutMin;
	double mOutMax;
	bool mbInAuto;
	// bool mP_On_E;
	double mMaxLoss;
	unsigned long mLastErrorPoint;
	double mLastOutput;

	recursive_mutex mMutex;
};
#endif

