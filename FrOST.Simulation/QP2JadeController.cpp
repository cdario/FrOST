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


vector< vector<string> > nodeConfig;
vector<JunctionCore> junctionIn;


std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems) {
	std::stringstream ss(s);
	std::string item;
	while (std::getline(ss, item, delim)) {
		elems.push_back(item);
	}
	return elems;
}

std::vector<std::string> split(const std::string &s, char delim) {
	std::vector<std::string> elems;
	split(s, delim, elems);
	return elems;
}

void loadNodeConfig(char* config_file)
{
	string line;
	ifstream myfile (config_file);
	vector<string> directionSet;

	directionSet.resize(5);
	if(myfile.is_open())
	{
		while ( std::getline(myfile,line) )
		{
			std::vector<std::string> dire = split(line, ' ');

			for (unsigned int y =0; y<dire.size(); y++)
			{
				directionSet[y] = dire[y];
			}
			nodeConfig.push_back(directionSet);
		}
		myfile.close();
		qps_GUI_printf(">>> Loaded Node configuration from file"); 
	}

	else qps_GUI_printf(">>> Unable to open node config file"); 

	//vector<vector(int)> allDirections;
	
}

void qpx_NET_postOpen(void)
{
	loadNodeConfig("c:\\temp\\node_config.txt");	//contains controlled nodes configuration
	std::vector<int64> stamps;
	int64 tempI = 0;
	for(unsigned i=0; i < nodeConfig.size(); i++)
	{
		vector<string> tJunction = nodeConfig[i];
		JunctionCore junctionX = JunctionCore(); //create new junction 
		junctionX.id = tJunction[0];
		junctionX.node = qpg_NET_node(const_cast<char*>(tJunction[0].c_str()));

		qps_NDE_externalController(junctionX.node,PTRUE);
		junctionX.phasing.resize(PHASE_COUNT);

		for (int p=0; p < PHASE_COUNT; p++)
		{
			junctionX.phasing[p].resize(MOVEMENT_COUNT);
		}
		//tempI = GetTimeMs64();stamps.push_back(tempI);

		//junctionX.loadPhasingFileGrid("c:\\temp\\node_config_+id+.txt", tJunction);
		junctionX.loadPhasingFileGrid("c:\\temp\\phasing_4dir.txt", tJunction);
		//tempI = GetTimeMs64(); 	stamps.push_back(tempI);

		// clockwise
		junctionX.arrivalsHorizon.resize(HORIZON_SIZE);
		for (int h= 0; h < HORIZON_SIZE; h++)
		{
			junctionX.arrivalsHorizon[h].resize(PHASE_COUNT);

			for (int p=0; p < PHASE_COUNT; p++)
			{
				// TODO: this step might be slowing down onload();
				junctionX.arrivalsHorizon[h][p]=0; //Init all to zero
			}
		}
		//tempI = GetTimeMs64(); 	stamps.push_back(tempI);
		junctionX.upstreamDetectors.resize(4);
		junctionX.upstreamDetectors[0] = qpg_NET_detector("u5NS");
		junctionX.upstreamDetectors[1] = qpg_NET_detector("u5EW");
		junctionX.upstreamDetectors[2] = qpg_NET_detector("u5SN");
		junctionX.upstreamDetectors[3] = qpg_NET_detector("u5WE");
		junctionX.stoplineDetectors.resize(4);
		junctionX.stoplineDetectors[0] = qpg_NET_detector("s5NS");
		junctionX.stoplineDetectors[1] = qpg_NET_detector("s5EW");
		junctionX.stoplineDetectors[2] = qpg_NET_detector("s5SN");
		junctionX.stoplineDetectors[3] = qpg_NET_detector("s5WE");

		int idxApproach = 0;
		junctionX.upstreamLoopData.resize(8);
		junctionX.stoplineLoopData.resize(8);
		for (int i = 0; i < 8 ; i ++)	// 2 lanes at each upstream
		{
			int laneDet = i%NUM_LANES + 1;  // 1 or 2
			junctionX.upstreamLoopData[i].detector = junctionX.upstreamDetectors[idxApproach];
			junctionX.upstreamLoopData[i].loop = qpg_DTC_multipleLoop(junctionX.upstreamDetectors[idxApproach], laneDet);
			junctionX.upstreamLoopData[i].lane = laneDet;
			junctionX.upstreamLoopData[i].lastCount = 0;
			junctionX.upstreamLoopData[i].type = UPSTREAM;

			junctionX.upstreamLoopData[i].approach = idxApproach; //NEW

			junctionX.stoplineLoopData[i].detector = junctionX.stoplineDetectors[idxApproach];
			junctionX.stoplineLoopData[i].loop = qpg_DTC_multipleLoop(junctionX.stoplineDetectors[idxApproach], laneDet);
			junctionX.stoplineLoopData[i].lane = laneDet;
			junctionX.stoplineLoopData[i].lastCount = 0;
			junctionX.stoplineLoopData[i].type = STOPLINE;

			junctionX.stoplineLoopData[i].approach = idxApproach; //NEW

			idxApproach = idxApproach + i%2; // +1 every 2 cycles
			/*
			loop 0	1	2	3	4	5	6	7	
			appr 0	0	1	1	2	2	3	3
			*/
		}
		tempI = GetTimeMs64(); 	stamps.push_back(tempI);
		/********		 Agent instance(s)		******/

		junctionX.agentController = REAP1::ReAP1();

		//instances.push_back(rp);  /*	includes policy instance */
		//tempI = GetTimeMs64(); 	stamps.push_back(tempI);
		junctionX.agentController.setMaxPhCompute(MAX_SEQUENCE);
		junctionX.agentController.setOutput(false);
		//junction5.agentController.setInitialPhase(2);
		junctionX.agentController.setStartupLostTime(2.0);
		junctionX.agentController.setMinGreenTime(MIN_GREEN);
		junctionX.agentController.setMaxGreenTime(MAX_GREEN);
		junctionX.agentController.setRedTime(ALL_RED);

		junctionX.agentController.setSaturationFlow(0, 1800); //in vehicles-per-hour per-lane 
		junctionX.agentController.setSaturationFlow(1, 1400);
		junctionX.agentController.setSaturationFlow(2, 1800);

		junctionX.agentController.setLanePhases(0, 1); //no. of lanes for each phase, used for saturation flow
		junctionX.agentController.setLanePhases(1, 1);
		junctionX.agentController.setLanePhases(2, 2);

		//tempI = GetTimeMs64(); 	stamps.push_back(tempI);

		REAP1::ReAP1Policy::REAP1STATE inState;
		inState.greenRemaining = MAX_GREEN;
		inState.phaseIndex = 2;
		std::vector<int> ques; ques.push_back(0); ques.push_back(0); ques.push_back(0);
		inState.queueLengths = ques; 
		junctionX.agentController.getPolicy().setState(inState);
		junctionX.agentController.setInitialState(inState);		//3
		junctionX.xState = inState;

		junctionX.currentPhaseIndex = inState.phaseIndex;		//NEW
		junctionX.timeToRed = (float)inState.greenRemaining;
		srand((int)time(NULL));
		junctionIn.push_back(junctionX);
	}
	//initialise communication with JADE here and intersection agents
	//1.	start jade platform, i.e., container and junction agents
	//tempI = GetTimeMs64(); 	stamps.push_back(tempI);

	std::ostringstream oss;
	for(unsigned ix=0; ix<junctionIn.size(); ix++)
	{
		oss << junctionIn[ix].id << " ";
	}

	char * junctions_a = const_cast<char*>(oss.str().c_str());

	//= "5 3 11 8 9 12 13 14 15";

	Environment envo = Environment();
	//JavaVM* 
	jvm_r = envo.startJVM();

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

	for (unsigned i = 0; i < junctionIn.size(); i++)
	{

		junctionIn[i].probeUpstreamDetectors(currentTime);
		//probeUpstreamDetectors(currentTime);

		/* ---------------------------------------------------------------------
		* populate prediction horizon with arrival and queue estimates
		* --------------------------------------------------------------------- */
		junctionIn[i].updateHorizon(currentTime);
		junctionIn[i].estimateQueues(currentTime);
		junctionIn[i].estimateDelay();
		junctionIn[i].updateState();	//pre-5

		junctionIn[i].manageThread();

		junctionIn[i].timeToRed = junctionIn[i].lastControlTime + junctionIn[i].currentControl; /*	switch points	*/
		junctionIn[i].timeToNext = junctionIn[i].timeToRed + ALL_RED;
		//qps_GUI_printf("toRed %f --- toNxt %f", timeToRed, timeToNext);
		if (junctionIn[i].isSequenceReady)
		{
			if (junctionIn[i].action == 0)
			{
				junctionIn[i].timeToNext = junctionIn[i].timeToRed;
				junctionIn[i].isAllRed = false;
			}

			if (!junctionIn[i].isAllRed)									/*	discard sequences till next phase	*/	
			{
				//NEW!! TODO: test
				std::vector<CONTROLDATA> _new;
				_new = junctionIn[i].tempSeq;
				std::vector<CONTROLDATA>::const_iterator iterator = _new.begin();
				junctionIn[i].controlSeq.clear();							
				junctionIn[i].controlSeq.reserve(_new.size());
				while(iterator != _new.end())
				{
					junctionIn[i].controlSeq.push_back(*iterator);
					++iterator;
				}

			}
			else
				//{
				//	if (timeToNext >= currentTime)
				//	{
				//		isAllRed = false;	// TODO: improve fix
				//	}
				//}

				if (junctionIn[i].isFirstTime)
				{
					junctionIn[i].timeToNext = currentTime;
					junctionIn[i].isFirstTime =false;
					junctionIn[i].isAllRed = false;
				}

				if (junctionIn[i].timeToRed-1 == currentTime)	//NEW	// TODO: tweak based on algo time
				{								
					junctionIn[i].actionTaken = true;		// control point: 2 secs before end of phase; compute next action
				}

				if (junctionIn[i].timeToRed == currentTime)	//NEW
				{
					if (junctionIn[i].action != 0)
					{
						junctionIn[i].setControllerAllRed();
						junctionIn[i].isAllRed = true;
					}
					else
					{
						junctionIn[i].isAllRed = false;
					}
				}

				if (junctionIn[i].timeToNext == currentTime)
				{
					if(junctionIn[i].controlSeq.size() > 0)						/* at this point, control and temp seqs are the same */
					{
						CONTROLDATA nxt = junctionIn[i].controlSeq.back();
						junctionIn[i].controlSeq.pop_back();
						if (junctionIn[i].isAllRed)
							junctionIn[i].tempSeq.clear();
						//tempSeq.pop_back();
						junctionIn[i].currentControl = nxt.duration;
						junctionIn[i].setControllerNext(nxt.phase);
						junctionIn[i].nextPhase = nxt.phase;

						//* MOVED HERE
						junctionIn[i].nextPhase = (junctionIn[i].nextPhase == 2) ? 0: junctionIn[i].nextPhase+1;		/*	prepare next phase	*/
						junctionIn[i].lastControlTime = currentTime;
					}

					//* WAS HERE
					junctionIn[i].isAllRed = false;
				}else							//TODO: manage all red lock
				{
					if (junctionIn[i].timeToNext < currentTime)
					{
						junctionIn[i].isAllRed = false;
						junctionIn[i].lastControlTime = currentTime;
					}
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
		//junctionIn[0].printVectorToFile();
		qps_GUI_printf("********* PRINTED ************");
	}
}


