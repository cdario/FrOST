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
	int64 tempI = 0;
	ostringstream convert;

	junction5 = JunctionCore();
	junction5.id = 5;
	std::string s;
	std::stringstream outId;
	outId << junction5.id;
	junction5.node = qpg_NET_node(const_cast<char*>(outId.str().c_str()));
	qps_NDE_externalController(junction5.node,PTRUE);
	junction5.phasing.resize(PHASE_COUNT);

	for (int p=0; p < PHASE_COUNT; p++)
	{
		junction5.phasing[p].resize(MOVEMENT_COUNT);
	}
	//tempI = GetTimeMs64();stamps.push_back(tempI);

	junction5.loadPhasingFile("c:\\temp\\phasing.txt");
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
	junction5.upstreamDetectors.resize(4);
	junction5.upstreamDetectors[0] = qpg_NET_detector("u5NS");
	junction5.upstreamDetectors[1] = qpg_NET_detector("u5EW");
	junction5.upstreamDetectors[2] = qpg_NET_detector("u5SN");
	junction5.upstreamDetectors[3] = qpg_NET_detector("u5WE");
	junction5.stoplineDetectors.resize(4);
	junction5.stoplineDetectors[0] = qpg_NET_detector("s5NS");
	junction5.stoplineDetectors[1] = qpg_NET_detector("s5EW");
	junction5.stoplineDetectors[2] = qpg_NET_detector("s5SN");
	junction5.stoplineDetectors[3] = qpg_NET_detector("s5WE");

	int idxApproach = 0;
	junction5.upstreamLoopData.resize(8);
	junction5.stoplineLoopData.resize(8);
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
	tempI = GetTimeMs64(); 	stamps.push_back(tempI);
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

	 //initialise communication with JADE here and intersection agents

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


