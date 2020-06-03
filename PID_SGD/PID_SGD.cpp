/**********************************************************************************************
 * PID Library for C++11
 * by Mingjun Zhu <deepdatago@gmail.com>
 * based on the work from Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 * Brett's work is at https://github.com/br3ttb/Arduino-PID-Library/
 *
 * Major changes are:
 *
 * 1) Add ResetTotal method.  This is to reset ouput sum to 0, when error is less than
 *    the predefined max loss, or when error sways pass the setpoint
 *
 * 2) Add recursive mutex for multi-threaded environment
 *
 * // TODO: for mP_On_E == 0, or P_ON_M is on, I haven't tested this case so I didn't do learning.
 * // This case should be reviewed in future
 * Remove mP_On_E and get rid of P_ON_M as I don't find a clear use case as explained in
 * http://brettbeauregard.com/blog/2017/06/proportional-on-measurement-the-code/
 * We rely on caller to provide meaningful input
 *
 * Note: naming convention on variables:
 * 	"m" stands for member
 * 	"b" stands for boolean
 * 	"mb" stands for member boolean
 * 	"i" stands for input
 * 	"ip" stands for input pointer
 * 	"r" stands for reference
 * 	"ir" stands for input reference
 * 	"l" stands for local
 *
 * This Library is licensed under the MIT License
 **********************************************************************************************/

#include "PID_SGD.h"
// #include <stdlib.h>
#include <cmath>
#include <sys/time.h>
#include <fstream>
#include <sstream>
#include <unistd.h>

#define LEARNING_RATE_FILE "PID_rate.txt"

using namespace std;

PID_SGD::PID_SGD(double iSetpoint,
        // int iP_On,
        // int iMaxLoss,
	int iControllerDirection,
	int iPIDFlags):

	// mMaxLoss will determine if we are in good state, or need to continue the training
		mMaxLoss(0),

		mLastDInput(0.0),

	// PID are set to 1.0 by default, user can call SetTunings to give different
	// initial values
		mKp(1.0),
		mKi(1.0),
		mKd(1.0),
		mLastErrorPoint(millis()),
		mLastError(0),
		mLastOutput(0.0),
		mbInAuto(false),
		mSetpoint(iSetpoint),
		mPIDFlags(iPIDFlags),
		mTotalError(0.0)

{
#ifdef PID_TRACE
	mDebugStr = "";
#endif
	//default output limit corresponds to the arduino pwm limits
	SetOutputLimits(0, 255);

	//default Controller Sample Time is 0.1 seconds
	mSampleTimeInMilliSecond = 100;

	SetControllerDirection(iControllerDirection);
	SetTunings(mKp, mKi, mKd);

	mLastTime = millis()- mSampleTimeInMilliSecond;
	LoadLearningRate();
}

unsigned long PID_SGD::millis()
{
	struct timeval tp;
	gettimeofday(&tp, NULL);
	return( tp.tv_sec * 1000 + tp.tv_usec / 1000);
}

/* Compute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid Output needs to be computed.  returns true when the output is computed,
 *   false when nothing has been done.
 **********************************************************************************/
std::tuple<bool, double> PID_SGD::Compute(double input)
{
	unique_lock<recursive_mutex> lLock(mMutex);
#ifdef PID_TRACE
	mDebugStr = "";
#endif
	double output = 0.0;
	static unsigned long timer = millis();
	if(!mbInAuto)
		return std::make_tuple(false, 0.0);

	unsigned long now = millis();
	/*
	if (now - timer > 5000)
	{
		// DumpLearningRate();
		timer = millis();
	}
	*/
	unsigned long timeChange = (now - mLastTime);
	// Reason we don't do time comparison here, is because this time duration
	// check is for Arduino board to avoid too many calculations that caused
	// instability of output.  However, at this place, we always want to
	// compute.
	// if(timeChange>=mSampleTimeInMilliSecond)
	{
		unique_lock<recursive_mutex> lLock(mMutex);
		/*Compute all the working error variables*/
		// double input = *mpInput;

		#ifdef PID_TRACE
		// LogTrace(" input: ");
		// LogTrace(input);
		#endif

		double error = mSetpoint - input;
		#ifdef PID_TRACE
		// LogTrace(" error: ");
		// LogTrace(error);
		#endif

		if (abs(error) < mMaxLoss)
		{
			// *mpOutput = mLastOutput;
			ResetTotal(now);
			return std::make_tuple(true, mLastOutput);
		}

		double dInput = (input - mLastInput);
		#ifdef PID_TRACE
		// LogTrace(" dInput: ");
		// LogTrace(dInput);
		#endif

		double lastOutputSum = mOutputSum;
		#ifdef PID_TRACE
		// LogTrace(" lastOutputSum: ");
		// LogTrace(lastOutputSum);
		#endif

		#ifdef PID_TRACE
		LogTrace("\nPrev mTotalError: ");
		LogTrace(mTotalError);
		LogTrace(" current mTotalError: ");
		LogTrace(mTotalError+error);

		#ifdef PID_TRACE
		LogTrace(" current mKi: ");
		LogTrace(mKi);
		#endif

		#endif

		mTotalError += error;

		if ((mLastError / error) < 0)
		{
			// just changed to the opposite error, so previous total error doesn't apply
			ResetTotal(now);
		}
		else if (mPIDFlags & PID_I)
		{
			mOutputSum = mKi * mSampleTimeInMilliSecond / 1000 *  mTotalError;
		}
		#ifdef PID_TRACE
		LogTrace(" mOutputSum1: ");
		LogTrace(mOutputSum);
		#endif


		/*
		// Add Proportional on Measurement, if P_ON_M is specified
		// explained in http://brettbeauregard.com/blog/2017/06/proportional-on-measurement-the-code/
		if(!mP_On_E) // this equal to P_ON_M
		{
			// TODO: do we need to call CalcSGD on mKp?
			mOutputSum-= mKp * dInput;
		}
		*/

		if(mOutputSum > mOutMax)
			mOutputSum= mOutMax;
		else if(mOutputSum < mOutMin)
			mOutputSum= mOutMin;

		/*Add Proportional on Error, if P_ON_E is specified*/
		if(mPIDFlags & PID_P)
		{
			mLastError = mSetpoint - mLastInput;
			output = mKp * error;
		}
		else
		{
			output = 0;
		}

		if (mPIDFlags & PID_D)
		{
			output += mOutputSum - (mKd / mSampleTimeInMilliSecond / 1000) * dInput;
		}
		output += mOutputSum;
		#ifdef PID_TRACE
		// LogTrace(" output3: ");
		// LogTrace(output);
		#endif

		mLastDInput = dInput;

		if(output > mOutMax)
			output = mOutMax;
		else if(output < mOutMin)
			output = mOutMin;

		// *mpOutput = output;

		#ifdef PID_TRACE
		// LogTrace(" output_final: ");
		// LogTrace(output);
		#endif
		mLastOutput = output;

		/*Remember some variables for next time*/
		mLastInput = input;
		mLastTime = now;
		return std::make_tuple(true, output);
	}
	return std::make_tuple(false, 0.0);
}

/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted.
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/
// void PID_SGD::SetTunings(double iKp, double iKi, double iKd, int iPOn)
void PID_SGD::SetTunings(double iKp, double iKi, double iKd)
{
	unique_lock<recursive_mutex> lLock(mMutex);
	// mP_On_E = (iPOn == P_ON_E);

	mKp = iKp;
	mKi = iKi;
	mKd = iKd;

	if(mControllerDirection == REVERSE)
	{
		mKp = (0 - mKp);
		mKi = (0 - mKi);
		mKd = (0 - mKd);
	}
}

/* SetSampleTime(...) *********************************************************
 * sets the period, in Milliseconds, at which the calculation is performed
 ******************************************************************************/
void PID_SGD::SetSampleTime(int iNewSampleTimeInMilliSecond)
{
	if (iNewSampleTimeInMilliSecond <= 0)
		return;

	unique_lock<recursive_mutex> lLock(mMutex);
	mSampleTimeInMilliSecond = iNewSampleTimeInMilliSecond;
}

/* SetOutputLimits(...)****************************************************
 *     This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the output will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 **************************************************************************/
void PID_SGD::SetOutputLimits(double iMin, double iMax)
{
	if(iMin >= iMax)
		return;

	unique_lock<recursive_mutex> lLock(mMutex);
	mOutMin = iMin;
	mOutMax = iMax;

	if(mbInAuto)
	{
		/*
		if(*mpOutput > mOutMax)
			*mpOutput = mOutMax;
		else if(*mpOutput < mOutMin)
			*mpOutput = mOutMin;
		*/

		if(mOutputSum > mOutMax)
			mOutputSum= mOutMax;
		else if(mOutputSum < mOutMin)
			mOutputSum= mOutMin;
	}
}

/* SetMode(...)****************************************************************
 * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 ******************************************************************************/
void PID_SGD::SetMode(int iMode)
{
	bool newAuto = (iMode == AUTOMATIC);
	unique_lock<recursive_mutex> lLock(mMutex);
	if(newAuto && !mbInAuto)
	{  /*we just went from manual to auto*/
	        // PID_SGD::Initialize();
	}
	mbInAuto = newAuto;
}

/* Initialize()****************************************************************
 *	does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 ******************************************************************************/
/*
void PID_SGD::Initialize()
{
	unique_lock<recursive_mutex> lLock(mMutex);
	mOutputSum = *mpOutput;
	mLastInput = *mpInput;
	mLastDInput = *mpInput;

	if(mOutputSum > mOutMax)
		mOutputSum = mOutMax;
	else if(mOutputSum < mOutMin)
		mOutputSum = mOutMin;
}
*/
/* SetControllerDirection(...)*************************************************
 * The PID will either be connected to a DIRECT acting process (+Output leads
 * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
 * know which one, because otherwise we may increase the output when we should
 * be decreasing.  This is called from the constructor.
 ******************************************************************************/
void PID_SGD::SetControllerDirection(int iDirection)
{
	unique_lock<recursive_mutex> lLock(mMutex);
	if(mbInAuto && iDirection != mControllerDirection)
	{
		mKp = (0 - mKp);
		mKi = (0 - mKi);
		mKd = (0 - mKd);
	}
	mControllerDirection = iDirection;
}

int PID_SGD::GetMode()
{
	unique_lock<recursive_mutex> lLock(mMutex);
	return  mbInAuto ? AUTOMATIC : MANUAL;
}

int PID_SGD::GetDirection()
{
	unique_lock<recursive_mutex> lLock(mMutex);
	return mControllerDirection;
}

double PID_SGD::GetKp()
{
	unique_lock<recursive_mutex> lLock(mMutex);
	return  mKp;
}

double PID_SGD::GetKi()
{
	unique_lock<recursive_mutex> lLock(mMutex);
	return  mKi;
}

double PID_SGD::GetKd()
{
	unique_lock<recursive_mutex> lLock(mMutex);
	return  mKd;
}

double PID_SGD::GetTotalError()
{
	unique_lock<recursive_mutex> lLock(mMutex);
	return  mTotalError;
}

double PID_SGD::GetSetpoint()
{
	unique_lock<recursive_mutex> lLock(mMutex);
	return  mSetpoint;
}

void PID_SGD::SetSetpoint(double iSetpoint)
{
	unique_lock<recursive_mutex> lLock(mMutex);
	mSetpoint = iSetpoint;
}

void PID_SGD::LogTrace(double iVal)
{
#ifdef PID_TRACE
	mDebugStr += iVal;
#endif
}

void PID_SGD::LogTrace(char* iStr)
{
#ifdef PID_TRACE
	mDebugStr += iStr;
#endif
}

void PID_SGD::ResetTotal(unsigned long iNow)
{
	unique_lock<recursive_mutex> lLock(mMutex);
	mOutputSum = 0;
	mTotalError = 0;
	mLastErrorPoint = iNow;
}

void PID_SGD::SetMaxLoss(double iLoss)
{
	unique_lock<recursive_mutex> lLock(mMutex);
	mMaxLoss = iLoss;
}

double PID_SGD::GetLastError()
{
	unique_lock<recursive_mutex> lLock(mMutex);
	return  mLastError;
}

void PID_SGD::DumpLearningRate()
{
	// define LEARNING_RATE_FILE "PID_rate.txt"
	// std::ifstream infile("thefile.txt");
	std::ofstream lRateFile (LEARNING_RATE_FILE);
	if (lRateFile.is_open())
	{
		lRateFile << "mKp " << mKp<<std::endl;
		lRateFile << "mKi " << mKi<<std::endl;
		lRateFile << "mKd " << mKd<<std::endl;
		lRateFile.close();
	}	
}

void PID_SGD::LoadLearningRate()
{
	// define LEARNING_RATE_FILE "PID_rate.txt"
	printf("Enter: LoadLearningRate\n");
	std::ifstream lRateFile(LEARNING_RATE_FILE);
	std::string lLine;
	while(std::getline(lRateFile, lLine))
	{
		printf("line: %s\n", lLine.c_str());
		std::istringstream stream(lLine);
		std::string lVar;
		double lValue;
		if (!(stream >> lVar >> lValue))
			break;
		unique_lock<recursive_mutex> lLock(mMutex);
		if (lVar.compare("mKp")==0)
			mKp = lValue;
		else if (lVar.compare("mKi")==0)
			mKi = lValue;
		else if (lVar.compare("mKd")==0)
			mKd = lValue;
	}
}
