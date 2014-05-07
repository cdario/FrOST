/* -----------------------------------------------------------------------
* QP Simulation plugin
*
* Run the selected controller algorithm to compute control sequences 
* based on upstream data and arrival predictions.
*
* ----------------------------------------------------------------------- */


#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <vector>
#include <math.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <iterator>
#include <string>
#include <time.h> 
#include <windows.h>
#include <process.h>
#include <random>

#include <Environment.h>

#ifdef WIN32
#include <Windows.h>
#else
#include <sys/time.h>
#include <ctime>
#endif


extern "C" {
#include "programmer.h"
}

#include "REAP1.h"
#include "JunctionCore.h"

using namespace std;
using namespace CORE;

/* ---------------------------------------------------------------------
* constants
* --------------------------------------------------------------------- */

#define 	PHASE_COUNT 3       /* the number of phases */
#define 	MOVEMENT_COUNT 10       /* based on no of phases */
#define		INITIAL_PHASE_INDEX 2   
#define		NUM_LANES 2
#define		MIN_GREEN 5
#define		GREEN_EXTENSION 5
#define		MAX_GREEN 50
#define		ALL_RED 2
#define		HORIZON_SIZE 15 //70 for isolated
#define		MAX_SEQUENCE 7
#define		UPSTREAM_DETECTOR_DISTANCE 150 //700       /* metres */
#define		VEHICLE_LENGTH 5       /* metres */
//#define		EXPECTED_STOPLINE_ENTRIES 5 
/* ---------------------------------------------------------------------
* data structures
* --------------------------------------------------------------------- */

//typedef struct LOOPUPDATA_s    LOOPUPDATA;
//
//struct LOOPUPDATA_s
//{
//	LOOP* upstreamDecLoop;
//	DETECTOR * upstreamDetector;
//	int lane;		//outer 1; inner 2
//	int lastCount;
//	int approach;		// 4 directions, clockwise
//};
//
//typedef struct LOOPSTDATA_s    LOOPSTDATA;
//
//struct LOOPSTDATA_s
//{
//	LOOP* stopLineDecLoop;
//	DETECTOR * stopLineDetector;
//	int lane;
//	int lastCount;
//	int approach;
//};
//
//
//enum loopType{
//	UPSTREAM, DOWNSTREAM
//};
//
//typedef struct LOOPDATA_s    LOOPDATA;
//
//struct LOOPDATA_s
//{
//	LOOP* loop;
//	DETECTOR * detector;
//	loopType type;
//	int lane;
//	int lastCount;
//	int approach;
//};
//
//
//typedef struct ARRIVALDATA_s    ARRIVALDATA;
//
//struct ARRIVALDATA_s		/*	 represent vehicles inside the prediction horizon	*/
//{
//	float arrivalTime;
//	float detectionTime;
//	float speed;
//	int phase;
//};
//
//typedef struct QUEUEDATA_s    QUEUEDATA;
//
//struct QUEUEDATA_s
//{
//	float detectionTime;
//	//int stoppedDelay;
//	float queueLength; // metre
//	int phase;
//};
//
//typedef struct SIGPRI_s    SIGPRI;
//struct SIGPRI_s
//{
//	string inlink;
//	string outlink;
//	int	  priority;    
//};
//
//typedef struct CONTROLDATA_s    CONTROLDATA;
//struct CONTROLDATA_s
//{
//	int phase;
//	int duration;
//};

/* ---------------------------------------------------------------------
* network elements
* --------------------------------------------------------------------- */
//
//static char*	junctionNode =     "5";   //node to control
//NODE*	jNode = NULL;
//
////NEW 3x3 grid
//static char* junctionList [9] = {"5 ", "3 ", "11 ",
//	"8 ", "9 ", "12 ",
//	"13 ","14 ","15 "};
//std::vector<NODE*> junctions;
//
//static LOOPUPDATA loopUpDetectorData[8]; /* 4 upstrDetectors, 2 loops each */
//static LOOPSTDATA loopStDetectorData[8]; /* 4 stoplDetectors, 2 loops each */
//static DETECTOR* upstrDetectors[4];  /* 4-arm intersection  per approach */
//static DETECTOR* stoplDetectors[4]; 

/* ---------------------------------------------------------------------
* phasing and prediction
* --------------------------------------------------------------------- */

//const char*	phases [3] = {"A", "B", "C"}; /* A = WE - SE ; B = WN - ES (protected left turn) ; C = NS - SN */
//std::vector<ARRIVALDATA> detectedArrivals;
//std::vector<std::vector<SIGPRI> > phasing;
//std::vector<std::vector<int> > arrivalsHorizon;
//double leftTurnProportion = 0.1; /* simplified turning proportions, must agree OD Matrix */ //nbefore 0.2
//double rightTurnProportion = 0.1;
//const char * phasing_file = "c:\\temp\\phasing.txt";
//
////const char * phasing_file = "phasing.txt";
//
//int eQueueCount[3] = {0,0,0};	// in vehicles per phase
//// queue = stoplinearrivals - actualdepartures
//int stoplineArrivals[3] = {0,0,0}; // total vehicles arriving at stop line 
//int currentPhaseIndex = 0;
//float timeToRed = 0;
//float timeToNext = 0;
/* ---------------------------------------------------------------------
* controller
* --------------------------------------------------------------------- */

//vector<REAP1::ReAP1> instances;
//vector<int> control;
//vector<CONTROLDATA> controlSeq;
//vector<CONTROLDATA> tempSeq;
//
//HANDLE hThread = NULL;
//unsigned threadID;
//bool  isThreadRunning = false;
//bool isSequenceReady = false;
//bool  isFirstTime = true;
//bool isAllRed = false;
//
//bool observeNow = true;	//NEW 
//double prevDelay = 0.0;	//NEW 
//double currentDelay = 0.0;//NEW 
//
//float lastControlTime = 0.0;
//int nextPhase = INITIAL_PHASE_INDEX;
//int nextControl = 0;
//int currentControl = 0;
//int seqIndex = 0;
//
//REAP1::ReAP1Policy::REAP1STATE xState;
//bool actionTaken = false;
//int action = -1;

JavaVM* jvm_r;

/* Returns the amount of milliseconds elapsed since the UNIX epoch. Works on both
* windows and linux. */

typedef long long int64; 
typedef unsigned long long uint64;

int64 GetTimeMs64()
{
#ifdef WIN32
	/* Windows */
	FILETIME ft;
	LARGE_INTEGER li;

	/* Get the amount of 100 nano seconds intervals elapsed since January 1, 1601 (UTC) and copy it
	* to a LARGE_INTEGER structure. */
	GetSystemTimeAsFileTime(&ft);
	li.LowPart = ft.dwLowDateTime;
	li.HighPart = ft.dwHighDateTime;

	uint64 ret = li.QuadPart;
	ret -= 116444736000000000LL; /* Convert from file time to UNIX epoch time. */
	ret /= 10000; /* From 100 nano seconds (10^-7) to 1 millisecond (10^-3) intervals */

	return ret;
#else
	/* Linux */
	struct timeval tv;

	gettimeofday(&tv, NULL);

	uint64 ret = tv.tv_usec;
	/* Convert from micro seconds (10^-6) to milliseconds (10^-3) */
	ret /= 1000;

	/* Adds the seconds (10^0) after converting them to milliseconds (10^-3) */
	ret += (tv.tv_sec * 1000);

	return ret;
#endif
}
//
////const std::string currentDateTime() {
////    time_t     now = time(0);
////    struct tm  tstruct;
////    char       buf[80];
////    tstruct = *localtime(&now);
////    // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
////    // for more information about date/time format
////    strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);
////
////    return buf;
////}
//
///* -----------------------------------------------------------------------
//* Runs algorithm on a separate thread and updates control sequence
//* --------------------------------------------------------------------- */
//
//unsigned __stdcall ThreadFunc( void* data )
//{
//	isThreadRunning = true;
//	clock_t tStart = clock();
//	//TODO: check to clear control
//	int cPhase = nextPhase;
//	/* RL ------interaction steps-------	*/
//	if(!actionTaken)
//	{
//		action = instances[0].selectAction(xState); //5	// TODO: check whether to init -state field- somewhere else...  
//		control.clear();
//		switch(action)
//		{
//		case 0:	cPhase = currentPhaseIndex;	// back to current phase; TODO: manage all-red
//			control.push_back(GREEN_EXTENSION);	//NEW
//			//currentControl += GREEN_EXTENSION;	//extend
//			break;
//		case 1: control.push_back(MIN_GREEN);	// switch next
//			break;
//		case 2: control.push_back(0); control.push_back(MIN_GREEN); // skip next and switch
//			break;
//		}
//		//qps_GUI_printf("\a ACTION %i",action);
//	}
//	else{
//		/*	When controller has completed the last action	*/
//		instances[0].setNewState(xState);	//6.1
//		instances[0].setNewReward(currentDelay - prevDelay); //6.2
//		instances[0].updateQ();	//7
//		instances[0].updateState(); //8
//		qps_GUI_printf("\a REWARD {%4.2f} and UPDATE", currentDelay - prevDelay);
//		actionTaken = false;	//NEW
//	}
//	/* ------end RL interaction steps-------	*/
//
//	//	control = instances[0].RunREAP();
//	/* to run it at a predetermined frequency, add ms to ttaken and sleep, e.g, Sleep( 5000L - ttaken ); */
//	double ttaken = (double)(clock() - tStart)/CLOCKS_PER_SEC;
//	if (!isAllRed)
//	{				/* late check to avoid algorithm latency issues */
//		string str;
//		std::stringstream message;
//		//cPhase = nextPhase;
//		tempSeq.clear();			
//		tempSeq.reserve(MAX_SEQUENCE+1);		// TODO: check it
//
//		for (vector<int>::const_iterator i = control.begin(); i != control.end(); ++i)
//		{
//			int dur = *i;
//			if (dur > 0)				/*	skip phase	*/
//			{
//				CONTROLDATA ctrl;
//				ctrl.phase = cPhase;
//				ctrl.duration = dur;
//				auto bg =  tempSeq.begin();
//				tempSeq.insert(bg,ctrl);
//			}
//
//			message << phases[cPhase] << ":" << dur << "\t";
//
//			cPhase = (cPhase == 2) ? 0: cPhase+1;				/* update phase	*/
//		}
//
//		float hh = qpg_CFG_simulationTime();
//		float mm =  fmod(hh, 60); 
//		hh = hh / 60;
//		qps_GUI_printf("\a REAP: %im %4.2fs \t%4.2fs \t %s ACTION %i",(int)hh ,mm, ttaken, message.str().c_str(), action);
//	}
//	isThreadRunning = false;
//	isSequenceReady = tempSeq.size() > 0;
//	return 0;
//} 

/* ---------------------------------------------------------------------
* split functions
* --------------------------------------------------------------------- */
//
//std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems) {
//	std::stringstream ss(s);
//	std::string item;
//	while (std::getline(ss, item, delim)) {
//		elems.push_back(item);
//	}
//	return elems;
//}
//
//std::vector<std::string> split(const std::string &s, char delim) {
//	std::vector<std::string> elems;
//	split(s, delim, elems);
//	return elems;
//}

/* ---------------------------------------------------------------------
* read phase configuration from file
* --------------------------------------------------------------------- */
//
//int toPrioEnum(string val)
//{
//	if (val == "APIPRI_MAJOR")
//		return APIPRI_MAJOR;
//	if (val == "APIPRI_MEDIUM")
//		return APIPRI_MEDIUM;
//	if (val == "APIPRI_MINOR")
//		return APIPRI_MINOR;
//
//	return APIPRI_BARRED;
//}
//
//void loadPhasingFile(void)
//{
//	string line;
//	ifstream myfile (phasing_file);
//	int iPhase = 0;
//	int iMov = 0;
//
//	if (myfile.is_open())
//	{
//		while ( getline (myfile,line) )
//		{
//			std::vector<std::string> prio = split(line, ' ');
//
//			phasing[iPhase][iMov].inlink = prio[0];
//			phasing[iPhase][iMov].outlink = prio[1];
//			phasing[iPhase][iMov].priority = toPrioEnum(prio[2]);
//
//			if (iMov == 9)
//			{
//				iMov = 0;
//				iPhase ++;
//			}
//			else
//				iMov++;
//		}
//		myfile.close();
//		qps_GUI_printf(">>> Loaded phasing specs from file"); 
//	}
//
//	else qps_GUI_printf(">>> Unable to open phasing file"); 
//
//}

/* ---------------------------------------------------------------------
* called on startup, initialise and instantiate variables
* --------------------------------------------------------------------- */

JunctionCore junction5;

void qpx_NET_postOpen(void)
{

	/* 
	*1 load large phasing file for all junctions

	for each junction
	obtain NODE* instance
	set external controller
	load phasing file *1
	keep and initialise arrivals horizon	
	initialise loop detector data structures
	create controller instance 
	*/

	std::vector<int64> stamps;
	//int64 tempI = 0;
	

	junction5 = JunctionCore();
	junction5.id = 5;
	junction5.node = qpg_NET_node((char*)&junction5.id);
	qps_NDE_externalController(junction5.node,PTRUE);
	junction5.phasing.resize(PHASE_COUNT);

	for (int p=0; p < PHASE_COUNT; p++)
	{
		junction5.phasing[p].resize(MOVEMENT_COUNT);
	}
	//tempI = GetTimeMs64();stamps.push_back(tempI);

	junction5.loadPhasingFile();
	//tempI = GetTimeMs64(); 	stamps.push_back(tempI);

	// clockwise
	junction5.arrivalsHorizon.resize(HORIZON_SIZE);
	for (int h= 0; h < HORIZON_SIZE; h++)
	{
		junction5.arrivalsHorizon[h].resize(PHASE_COUNT);

		for (int p=0; p < PHASE_COUNT; p++)
		{
			// TODO: this step might be slowing down onload();
			junction5.arrivalsHorizon[h][p]=0; //Init all to zero
		}
	}
	//tempI = GetTimeMs64(); 	stamps.push_back(tempI);

	junction5.upstreamDetectors[0] = qpg_NET_detector("u5NS");
	junction5.upstreamDetectors[1] = qpg_NET_detector("u5EW");
	junction5.upstreamDetectors[2] = qpg_NET_detector("u5SN");
	junction5.upstreamDetectors[3] = qpg_NET_detector("u5WE");

	junction5.stoplineDetectors[0] = qpg_NET_detector("s5NS");
	junction5.stoplineDetectors[1] = qpg_NET_detector("s5EW");
	junction5.stoplineDetectors[2] = qpg_NET_detector("s5SN");
	junction5.stoplineDetectors[3] = qpg_NET_detector("s5WE");

	int idxApproach = 0;
	for (int i = 0; i < 8 ; i ++)	// 2 lanes at each upstream
	{
		int laneDet = i%NUM_LANES + 1;  // 1 or 2
		junction5.upstreamLoopData[i].detector = junction5.upstreamDetectors[idxApproach];
		junction5.upstreamLoopData[i].loop = qpg_DTC_multipleLoop(junction5.upstreamDetectors[idxApproach], laneDet);
		junction5.upstreamLoopData[i].lane = laneDet;
		junction5.upstreamLoopData[i].lastCount = 0;
		junction5.upstreamLoopData[i].type = UPSTREAM;

		junction5.upstreamLoopData[i].approach = idxApproach; //NEW

		junction5.stoplineLoopData[i].detector = junction5.stoplineDetectors[idxApproach];
		junction5.stoplineLoopData[i].loop = qpg_DTC_multipleLoop(junction5.stoplineDetectors[idxApproach], laneDet);
		junction5.stoplineLoopData[i].lane = laneDet;
		junction5.stoplineLoopData[i].lastCount = 0;
		junction5.stoplineLoopData[i].type = STOPLINE;

		junction5.stoplineLoopData[i].approach = idxApproach; //NEW

		idxApproach = idxApproach + i%2; // +1 every 2 cycles
		/*
		loop 0	1	2	3	4	5	6	7	
		appr 0	0	1	1	2	2	3	3
		*/
	}
	//tempI = GetTimeMs64(); 	stamps.push_back(tempI);
	/********		 Agent instance(s)		******/

	junction5.agentController = REAP1::ReAP1();

	//instances.push_back(rp);  /*	includes policy instance */
	//tempI = GetTimeMs64(); 	stamps.push_back(tempI);
	junction5.agentController.setMaxPhCompute(MAX_SEQUENCE);
	junction5.agentController.setOutput(false);
	//junction5.agentController.setInitialPhase(2);
	junction5.agentController.setStartupLostTime(2.0);
	junction5.agentController.setMinGreenTime(MIN_GREEN);
	junction5.agentController.setMaxGreenTime(MAX_GREEN);
	junction5.agentController.setRedTime(ALL_RED);

	junction5.agentController.setSaturationFlow(0, 1800); //in vehicles-per-hour per-lane 
	junction5.agentController.setSaturationFlow(1, 1400);
	junction5.agentController.setSaturationFlow(2, 1800);

	junction5.agentController.setLanePhases(0, 1); //no. of lanes for each phase, used for saturation flow
	junction5.agentController.setLanePhases(1, 1);
	junction5.agentController.setLanePhases(2, 2);

	//tempI = GetTimeMs64(); 	stamps.push_back(tempI);

	REAP1::ReAP1Policy::REAP1STATE inState;
	inState.greenRemaining = MAX_GREEN;
	inState.phaseIndex = 2;
	std::vector<int> ques; ques.push_back(0); ques.push_back(0); ques.push_back(0);
	inState.queueLengths = ques; 
	junction5.agentController.getPolicy().setState(inState);
	junction5.agentController.setInitialState(inState);		//3
	junction5.xState = inState;

	junction5.currentPhaseIndex = inState.phaseIndex;		//NEW
	junction5.timeToRed = (float)inState.greenRemaining;
	srand((int)time(NULL));

	// initialise communication with JADE here and intersection agents

	//1.	start jade platform, i.e., container and junction agents

	//tempI = GetTimeMs64(); 	stamps.push_back(tempI);

	Environment envo = Environment();

	//JavaVM* 
	jvm_r = envo.startJVM();

	char * junctions_a = "5 3 11 8 9 12 13 14 15";

	int stat = -10;
	if(jvm_r != NULL)
		stat = envo.startPlatform(jvm_r, junctions_a);

	if (stat == 0)
		qps_GUI_printf(">>> JADE Platform main-cointaner started!");
	else
		qps_GUI_printf(">>> Error starting JADE platform main-container (error code=%i)", stat);

	//envo.close();	//untested

	//1.1	create identical junction agents in JADE

	// loop over structure of nodes!
	//env.addJunction(junctionNode);
	//tempI = GetTimeMs64(); 	stamps.push_back(tempI);

	/*
	for(std::vector<int64>::iterator it = stamps.begin(); it != stamps.end(); ++it) {
	qps_GUI_printf("%lu", *it);
	}
	*/

	//jvm_r->DetachCurrentThread();
	//jvm_r->DestroyJavaVM();

	//envo.close();
}

void printMe(char* message)
{
	qps_GUI_printf(message);
}

void qpx_NET_close()
{
	/*Environment env = Environment();
	jvm_r = env.startJVM();
	jvm_r->DetachCurrentThread();
	jvm_r->DestroyJavaVM();
	env.close();*/
}

/* ---------------------------------------------------------------------
* Include GUI elements
* --------------------------------------------------------------------- */

void qpx_DRW_modelView(void)
{
	//TODO: Implement if needed
}

/* ---------------------------------------------------------------------
* time to arrival in mps
* --------------------------------------------------------------------- */
//
//float getEstimatedArrivalTime (float time, float speed, int distance)
//{
//	return time + distance/speed; //sec + metres / metres/sec
//}

/* ---------------------------------------------------------------------
* Step adjustement for the horizon, only for 0.5s resolution
* --------------------------------------------------------------------- */
//
//float adjustToStep(float num)
//{
//	float decimal = num - floor(num);
//
//	if (decimal >= 0  && decimal <= 3)
//	{
//		return floor(num);
//	}
//	else
//	{
//		if(decimal >= 4  && decimal <= 6)
//		{
//			return (float)(floor(num) + 0.5);
//		}
//		else
//		{
//			if (decimal >= 7) //include 0 next int
//				return ceil(num);
//		}
//	}
//	return floor(num);
//}

/* ---------------------------------------------------------------------
* determines current state of the prediction horizon for this arrival
* --------------------------------------------------------------------- */
//
//float getHorizonStep(ARRIVALDATA arrival, float simulationTime)
//{
//	float elapsedTime = simulationTime - arrival.detectionTime;
//	float distanceToStopline = UPSTREAM_DETECTOR_DISTANCE - arrival.speed * elapsedTime; // toDetector - travelled
//	float timeToStopline = distanceToStopline / arrival.speed;
//	return adjustToStep(timeToStopline); // floor, .5 or ceiling
//}

/* ---------------------------------------------------------------------
* Initialise the horizon to zero
* --------------------------------------------------------------------- */
//
//void clearHorizon()
//{
//	for (int h=0; h< HORIZON_SIZE; h++)
//	{
//		for (int p= 0; p <PHASE_COUNT; p++)	
//		{
//			arrivalsHorizon[h][p]= 0; 
//		}
//	}	
//}

/* ---------------------------------------------------------------------
* include in the horizon or remove them from the detection data
* --------------------------------------------------------------------- */
//
//void updateHorizon( float simulationTime)
//{
//	clearHorizon();
//
//	for(unsigned int i=0; i<detectedArrivals.size(); i++) // for all detected vehicles
//	{
//		if (detectedArrivals[i].arrivalTime < simulationTime) // if estimated arrival has passed
//		{														
//			int iPhase = detectedArrivals[i].phase;
//			std::vector<ARRIVALDATA>::iterator it = detectedArrivals.begin();
//			std::advance(it, i);
//			detectedArrivals.erase(it);		// remove vehicle from the vector!
//			// difference between counts then is queueing!
//			stoplineArrivals[iPhase] += 1;		// NEW : EAT has elapsed, then add to stopline arrivals
//		}
//		else	// if vehicle still in the link, add it to the current horizon
//		{
//			float arrivalTimeStep  = getHorizonStep(detectedArrivals[i], simulationTime);
//			int horizonTime= (int)(arrivalTimeStep * 2); 
//			/*
//			0 0.5 1 1.5 2 2.5 ... 34
//			0  1  2  3  4  5 ... 69
//			*/
//
//			if (horizonTime < HORIZON_SIZE) //70-s (0-69)
//			{
//				ARRIVALDATA detected = detectedArrivals[i];
//				arrivalsHorizon[horizonTime][detected.phase]+=1;
//
//				//	if(horizonTime < 1)	//imminent arrivals; single point in time to avoid multiple count
//				//	{
//				//		qps_GUI_printf("+++ ARRIVAL by %i", detectedArrivals[i].phase);	// NEW add arrivals counter; tested OK
//				//		stoplineArrivals[detectedArrivals[i].phase] += 1;
//				//	}
//			}
//		}
//	}
//}

/* ---------------------------------------------------------------------
* return total expected arrivals (5 steps) per phase
//* --------------------------------------------------------------------- */
//
//int lookupExpectedArrivals(int phaseIdx, int nEntries)		//NEW
//{	
//	int exArrivals = 0;
//
//	for(int h=1; h < nEntries; h++)
//	{
//		exArrivals = arrivalsHorizon[nEntries][phaseIdx];
//	}
//
//	return exArrivals;
//}

/* ---------------------------------------------------------------------
* determine phase (A or B) for vehicle using a detector 
//* --------------------------------------------------------------------- */
//
//int getPhaseByProbability(int detectorIndex)
//{
//	int phase = 2;
//	//srand(time(0));
//	/*	C {0, 1, 4, 5} A or B {2, 3, 6, 7}	*/	
//	if (detectorIndex > 5 || (detectorIndex > 1 && detectorIndex < 4)) // A or B
//	{
//		double udr =  qpg_UTL_randomDouble(APIRNG_MISC, 1);
//		//	qps_GUI_printf("--rnd  %4.2f", udr);
//		phase = udr > leftTurnProportion ? 0 : 1; //A=0;  B = 1; C=2,
//	}
//
//	/*
//	loop 0	1	2	3	4	5	6	7	
//	appr 0	0	1	1	2	2	3	3
//	*/
//	return phase;
//}
//
//// returns 2 if phase C; 0 if phase A or B
//int getDetectorPhaseGroup(int detectorIndex)
//{
//	if (detectorIndex > 5 || (detectorIndex > 1 && detectorIndex < 4))
//		return 0;
//	else
//		return 2;
//}

/* ---------------------------------------------------------------------
* Use upstream and stopline detector data to estimate queue lengths per phase
* --------------------------------------------------------------------- */
//
//void estimateQueues(float currentTime)		//NEW
//{
//	//int exArrivals[3] = {0,0,0};
//	//eQueueCount[0]= 0;eQueueCount[1]= 0;eQueueCount[2]= 0;
//	int stopLineDepartures[3] = {0,0,0};
//
//	for(int detectorIndex=0; detectorIndex < 8 ; detectorIndex++) 	/* check all loop detectors  (stopline); two per approach	*/
//	{
//		LOOP* stoplineLoop = loopStDetectorData[detectorIndex].stopLineDecLoop;
//		int currentStopCount = qpg_DTL_count(stoplineLoop, 0); 		/*	zero to count all vehicle types	*/
//
//		//int	iPhase = getPhaseByProbability(detectorIndex);	// prev implem - noisy function; find another solution
//
//		int phGroup = getDetectorPhaseGroup(detectorIndex);
//
//		if (phGroup == 2)
//			stopLineDepartures[phGroup] += currentStopCount;
//		else
//		{
//			int leftTurners = (int) (currentStopCount * leftTurnProportion); //NEW!!! distribute based on turning rate!
//			stopLineDepartures[0] += currentStopCount - leftTurners;	//TODO: allocate left over unit somewhere?
//			stopLineDepartures[1] += leftTurners;
//		}																//TODO: use currentPhaseIndex to discount difference btwn counts?
//
//		//stopLineDepartures[iPhase] += currentStopCount;	// prev implementation
//		loopStDetectorData[detectorIndex].lastCount = currentStopCount;	// previous reading
//	}
//
//	int diff;											/* deal with error from turning probability */
//	if (stoplineArrivals[1] < stopLineDepartures[1])
//	{
//		diff = stopLineDepartures[1] - stoplineArrivals[1];
//		stoplineArrivals[0] -= diff;
//		stoplineArrivals[1] += diff;
//	}
//	else
//	{
//		if (stoplineArrivals[0] < stopLineDepartures[0])
//		{
//			diff = stopLineDepartures[0] - stoplineArrivals[0];
//			stoplineArrivals[1] -= diff;
//			stoplineArrivals[0] += diff;
//		}
//	}
//	for (int phi = 0; phi < PHASE_COUNT; phi++)
//	{
//		eQueueCount[phi] = stoplineArrivals[phi] - stopLineDepartures[phi]; //- currentStopCount;
//
//		if (phi != currentPhaseIndex && eQueueCount[phi] > 2)
//		{
//			eQueueCount[phi] += 2;		/*	stopped vehicles on stop line detector for alls phases (A B c) */
//			if (phi == 2)
//				eQueueCount[phi] += 2;	/*	stopped vehicles on stop line detector for phase C*/
//
//		}
//	}
//
//	//qps_GUI_printf(">>> stopline arrivals: A[%i] B[%i] C[%i]", stoplineArrivals[0], stoplineArrivals[1], stoplineArrivals[2]); 
//	//qps_GUI_printf(">>> stopline count   : A[%i] B[%i] C[%i]", stopLineDepartures[0], stopLineDepartures[1], stopLineDepartures[2]); 
//
//	//qps_GUI_printf(">> Current queues : A[%i] B[%i] C[%i]", eQueueCount[0], eQueueCount[1], eQueueCount[2]); 
//
//	/*
//	the difference between the number of expected arrival vehicles and the actually departed vehicles at the stop line
//	expected arrival vehicles: from the arrivals Horizon (expected)
//	actually departed vehicles at stop line
//	*/
//}
//
//void estimateDelay()			//TODO: is just delay of queueing vehicles
//{
//	prevDelay = currentDelay;
//	for (int i=0; i<PHASE_COUNT; i++)
//	{
//		if(currentPhaseIndex != i)
//			currentDelay += 0.5 * eQueueCount[i];	// queue by time step
//
//	}
//}
//
//void updateState()
//{
//	xState.phaseIndex = currentPhaseIndex;
//	xState.greenRemaining = (int)timeToRed;	//TODO: deal with loss of data
//	std::vector<int> quLengths;
//	int eq;
//	for (eq= 2; eq >= 0; eq-- )
//	{
//		if (eQueueCount[eq] > 10)     //TODO: queue groups 
//			quLengths.push_back(10);
//		else
//			quLengths.push_back(eQueueCount[eq]);
//	}
//	//quLengths.push_back(eQueueCount[2]);quLengths.push_back(eQueueCount[1]);quLengths.push_back(eQueueCount[0]); 
//	xState.queueLengths = quLengths;
//
//}

/* ---------------------------------------------------------------------
* print current horizon to file
* --------------------------------------------------------------------- */
//
//void printVectorToFile()
//{
//	ofstream myfile;
//	myfile.open ("C:\\temp\\arrival-horizon.txt");
//	if (myfile.is_open())
//	{
//		for(int h=0; h<70; h++){
//			for(int p=0; p<3; p++)
//			{
//				myfile << arrivalsHorizon[h][p] << " ";
//			}
//			myfile << "\n";
//		}
//	}
//	myfile.close();
//}

/* ---------------------------------------------------------------------
* sets a phase in the controller
* --------------------------------------------------------------------- */
//
//void setController(int ph, int pri)
//{
//	int priority;
//
//	for (unsigned int i = 0; i < MOVEMENT_COUNT; i++)
//	{
//		char inlnk[10];
//		char outlnk[10];
//		strcpy_s(inlnk,  phasing[ph][i].inlink.c_str());
//		strcpy_s(outlnk, phasing[ph][i].outlink.c_str());
//
//		if (pri == APIPRI_BARRED)
//			priority = pri;
//		else
//			priority = phasing[ph][i].priority;
//
//		qps_LNK_priority(qpg_NET_link(inlnk), qpg_NET_link(outlnk), priority);
//	}     
//}
//
//void setControllerAllRed()
//{
//	setController(0, APIPRI_BARRED);
//	qps_GUI_printf("--------->Controller set to RED for %is", ALL_RED); 
//}
//
//void setControllerNext(int ph)
//{
//	currentPhaseIndex = ph;
//	setController(ph, APIPRI_MAJOR);
//	qps_GUI_printf("------------->Controller set to [%s] for %is", phases[ph], currentControl); 
//}
//
//void probeUpstreamDetectors(float currentTime)
//{
//	/* ---------------------------------------------------------------------
//	* probe loop detectors for vehicles
//	* --------------------------------------------------------------------- */
//	for(int i=0; i < 8 ; i ++) 	/* check all loop detectors (upstream); two per approach	*/
//	{
//		LOOP* upstLoop = loopUpDetectorData[i].upstreamDecLoop;
//		int currentCount = qpg_DTL_count(upstLoop, 0); 		/*	zero to count all vehicle types	*/
//
//		if (currentCount != loopUpDetectorData[i].lastCount) 		/*	add new vehicles detectedArrivals	*/
//		{
//			ARRIVALDATA dta;	
//			dta.speed = qpg_DTL_speed(upstLoop, APILOOP_COMPLETE);	
//			if (dta.speed == 0.0)	/* validating overflown queue, replace with arbitrarily low speed */
//				dta.speed = (float)0.1;
//			dta.arrivalTime = getEstimatedArrivalTime(currentTime, dta.speed, UPSTREAM_DETECTOR_DISTANCE);
//			dta.detectionTime = currentTime;
//			dta.phase = getPhaseByProbability(i);
//			detectedArrivals.push_back(dta);
//			loopUpDetectorData[i].lastCount = currentCount;
//		}
//	}
//}
//
//void manageThread()
//{
//	if(!isThreadRunning)		/*	manage algorithm thread 	*/
//	{	
//		if(hThread != NULL){
//			__try{
//				CloseHandle (hThread);	/*  close finished thread	*/
//			}
//			__except(EXCEPTION_EXECUTE_HANDLER)	/*	triggered when debugging	*/
//			{
//				qps_GUI_printf("CloseHandle Exception Gracefully Managed!"); /* debugger glitch:  http://stackoverflow.com/q/1550315/777877 */
//			}
//		}
//		if (isAllRed)
//		{
//			hThread = new HANDLE();
//		}
//		else
//		{
//			hThread = (HANDLE)_beginthreadex( NULL, 0, &ThreadFunc, NULL, 0, &threadID);		/* init new thread */
//		}
//	}
//}
//
//const std::vector<CONTROLDATA> getTempSeq(){
//	return tempSeq;
//}


/* ---------------------------------------------------------------------
* every simulated step, works best at 0.5s resolution
* --------------------------------------------------------------------- */

void qpx_NET_timeStep()
{
	float step = qpg_CFG_timeStep();
	float currentTime = qpg_CFG_simulationTime();

	junction5.probeUpstreamDetectors(currentTime);
	//probeUpstreamDetectors(currentTime);

	/* ---------------------------------------------------------------------
	* populate prediction horizon with arrival and queue estimates
	* --------------------------------------------------------------------- */
	junction5.updateHorizon(currentTime);
	junction5.estimateQueues(currentTime);
	junction5.estimateDelay();
	junction5.updateState();	//pre-5

	junction5.manageThread();

	junction5.timeToRed = junction5.lastControlTime + junction5.currentControl; /*	switch points	*/
	junction5.timeToNext = junction5.timeToRed + ALL_RED;
	//qps_GUI_printf("toRed %f --- toNxt %f", timeToRed, timeToNext);
	if (junction5.isSequenceReady)
	{
		if (junction5.action == 0)
		{
			junction5.timeToNext = junction5.timeToRed;
			junction5.isAllRed = false;
		}

		if (!junction5.isAllRed)									/*	discard sequences till next phase	*/	
		{
			//NEW!! TODO: test
			std::vector<CONTROLDATA> _new;
			_new = junction5.tempSeq;
			std::vector<CONTROLDATA>::const_iterator i = _new.begin();
			junction5.controlSeq.clear();							
			junction5.controlSeq.reserve(_new.size());
			while(i != _new.end())
			{
				junction5.controlSeq.push_back(*i);
				++i;
			}

		}
		else
			//{
			//	if (timeToNext >= currentTime)
			//	{
			//		isAllRed = false;	// TODO: improve fix
			//	}
			//}

			if (junction5.isFirstTime)
			{
				junction5.timeToNext = currentTime;
				junction5.isFirstTime =false;
				junction5.isAllRed = false;
			}

			if (junction5.timeToRed-1 == currentTime)	//NEW	// TODO: tweak based on algo time
			{								
				junction5.actionTaken = true;		// control point: 2 secs before end of phase; compute next action
			}

			if (junction5.timeToRed == currentTime)	//NEW
			{
				if (junction5.action != 0)
				{
					junction5.setControllerAllRed();
					junction5.isAllRed = true;
				}
				else
				{
					junction5.isAllRed = false;
				}
			}

			if (junction5.timeToNext == currentTime)
			{
				if(junction5.controlSeq.size() > 0)						/* at this point, control and temp seqs are the same */
				{
					CONTROLDATA nxt = junction5.controlSeq.back();
					junction5.controlSeq.pop_back();
					if (junction5.isAllRed)
						junction5.tempSeq.clear();
					//tempSeq.pop_back();
					junction5.currentControl = nxt.duration;
					junction5.setControllerNext(nxt.phase);
					junction5.nextPhase = nxt.phase;

					//* MOVED HERE
					junction5.nextPhase = (junction5.nextPhase == 2) ? 0: junction5.nextPhase+1;		/*	prepare next phase	*/
					junction5.lastControlTime = currentTime;
				}

				//* WAS HERE
				junction5.isAllRed = false;
			}else							//TODO: manage all red lock
			{
				if (junction5.timeToNext < currentTime)
				{
					junction5.isAllRed = false;
					junction5.lastControlTime = currentTime;
				}
			}
	}
}

/* ---------------------------------------------------------------------
* mouse and keyboard events
* --------------------------------------------------------------------- */

void qpx_GUI_keyPress(int key, int ctrl, int shift, int left, int middle, int right)
{
	if(key == 0x33 && middle) /* print to file on 3 key + mid click */
	{
		junction5.printVectorToFile();
		qps_GUI_printf("********* PRINTED ************");
	}
}


