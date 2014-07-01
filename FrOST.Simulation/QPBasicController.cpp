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

extern "C" {
#include "programmer.h"
}

#include "Cop97A.h"

using namespace std;

/* ---------------------------------------------------------------------
 * constants
 * --------------------------------------------------------------------- */

#define 	PHASE_COUNT 3       /* the number of phases */
#define 	MOVEMENT_COUNT 10       /* the number of phases */
#define		INITIAL_PHASE_INDEX 2   
#define		NUM_LANES 2
#define		MIN_GREEN 5
#define		MAX_GREEN 50
#define		ALL_RED 2
#define		HORIZON_SIZE 70
#define		MAX_SEQUENCE 7
#define		UPSTREAM_DETECTOR_DISTANCE 700       /* metres */

/* ---------------------------------------------------------------------
 * data structures
 * --------------------------------------------------------------------- */

typedef struct LOOPDATA_s    LOOPDATA;

struct LOOPDATA_s
{
	LOOP* upstreamDecLoop;
	DETECTOR * upstreamDetector;
	int lane;
	int lastCount;
	int phase;
};

typedef struct ARRIVALDATA_s    ARRIVALDATA;

struct ARRIVALDATA_s
{
	float arrivalTime;
	float detectionTime;
	float speed;
	int phase;
};

typedef struct SIGPRI_s    SIGPRI;
struct SIGPRI_s
{
	string inlink;
	string outlink;
	int	  priority;    
};

typedef struct CONTROLDATA_s    CONTROLDATA;
struct CONTROLDATA_s
{
	int phase;
	int duration;
};

/* ---------------------------------------------------------------------
 * network elements
 * --------------------------------------------------------------------- */

static char*	junctionNode =     "5";   //node to control
NODE*	jNode = NULL;
static LOOPDATA loopDetectorData[8]; /* 4 upstrDetectors, 2 loops each */
static DETECTOR* upstrDetectors[4];  /* 4-arm intersection  per approach */
static DETECTOR* stoplDetectors[4]; 

/* ---------------------------------------------------------------------
 * phasing and prediction
 * --------------------------------------------------------------------- */

const char*	phases [3] = {"A", "B", "C"}; /* A = WE - SE ; B = WN - ES (protected left turn) ; C = NS - SN */
std::vector<ARRIVALDATA> detectedArrivals;
std::vector<std::vector<SIGPRI> > phasing;
std::vector<std::vector<int> > arrivalsHorizon;
double leftTurnProportion = 0.1; /* simplified turning proportions, must agree OD Matrix */ //nbefore 0.2
double rightTurnProportion = 0.1;
const char * phasing_file = "c:\\PARAMICS\\phasing.txt";
//const char * phasing_file = "phasing.txt";

/* ---------------------------------------------------------------------
 * controller
 * --------------------------------------------------------------------- */

vector<COP97A::Cop97A> instances;
vector<int> control;
vector<CONTROLDATA> controlSeq;
vector<CONTROLDATA> tempSeq;

HANDLE hThread = NULL;
unsigned threadID;
bool  isThreadRunning = false;
bool isSequenceReady = false;
bool  isFirstTime = true;
bool isAllRed = false;

float lastControlTime = 0.0;
int nextPhase = INITIAL_PHASE_INDEX;
int nextControl = 0;
int currentControl = 0;
int seqIndex = 0;

/* -----------------------------------------------------------------------
* Runs algorithm on a separate thread and updates control sequence
* --------------------------------------------------------------------- */

unsigned __stdcall COPThreadFunc( void* data )
{
	isThreadRunning = true;
	
	clock_t tStart = clock();
	instances[0].setArrivals(arrivalsHorizon);	/*	set to latest horizon */
	control = instances[0].RunCOP();
	/* to run it at a predetermined frequency, add ms to ttaken and sleep, e.g, Sleep( 5000L - ttaken ); */
	double ttaken = (double)(clock() - tStart)/CLOCKS_PER_SEC;
	if (!isAllRed)
	{				/* late check to avoid algorithm latency issues */
		string str;
		std::stringstream message;
		int cPhase = nextPhase;
		tempSeq.clear();			
		tempSeq.reserve(MAX_SEQUENCE+1);		// TODO: check it

		for (vector<int>::iterator i = control.begin(); i != control.end(); ++i)
		{
			int dur = *i;
			if (dur > 0)				/*	skip phase	*/
			{
				CONTROLDATA ctrl;
				ctrl.phase = cPhase;		
				ctrl.duration = dur;
				tempSeq.insert(tempSeq.begin(),ctrl);
			}

			message << phases[cPhase] << ":" << dur << "\t";
			cPhase = (cPhase == 2) ? 0: cPhase+1;				/* update phase	*/
		}

		float hh = qpg_CFG_simulationTime();
		float mm =  fmod(hh, 60); 
		hh = hh / 60;
		qps_GUI_printf("\a COP: %im %4.2fs \t%4.2fs \t %s ",(int)hh ,mm, ttaken, message.str().c_str());
	}
	isThreadRunning = false;
	isSequenceReady = tempSeq.size() > 0;
	return 0;
} 

/* ---------------------------------------------------------------------
* split functions
* --------------------------------------------------------------------- */

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

/* ---------------------------------------------------------------------
* read phase configuration from file
* --------------------------------------------------------------------- */

int toPrioEnum(string val)
{
	if (val == "APIPRI_MAJOR")
		return APIPRI_MAJOR;
	if (val == "APIPRI_MEDIUM")
		return APIPRI_MEDIUM;
	if (val == "APIPRI_MINOR")
		return APIPRI_MINOR;

	return APIPRI_BARRED;
}

void loadPhasingFile(void)
{
	string line;
	ifstream myfile (phasing_file);
	int iPhase = 0;
	int iMov = 0;

	if (myfile.is_open())
	{
		while ( getline (myfile,line) )
		{
			std::vector<std::string> prio = split(line, ' ');

			phasing[iPhase][iMov].inlink = prio[0];
			phasing[iPhase][iMov].outlink = prio[1];
			phasing[iPhase][iMov].priority = toPrioEnum(prio[2]);

			if (iMov == 9)
			{
				iMov = 0;
				iPhase ++;
			}
			else
				iMov++;
		}
		myfile.close();
	}

	else qps_GUI_printf("Unable to open file"); 

}

/* ---------------------------------------------------------------------
* called on startup, initialise and instantiate variables
* --------------------------------------------------------------------- */

void qpx_NET_postOpen(void)
{
	jNode = qpg_NET_node(junctionNode);
	qps_NDE_externalController(jNode,PTRUE);

	phasing.resize(PHASE_COUNT);
	for (int p=0; p < PHASE_COUNT; p++)
	{
		phasing[p].resize(MOVEMENT_COUNT);
	}

	loadPhasingFile();

	// clockwise
	arrivalsHorizon.resize(HORIZON_SIZE);
	for (int h= 0; h < HORIZON_SIZE; h++)
	{
		arrivalsHorizon[h].resize(PHASE_COUNT);

		for (int p=0; p < PHASE_COUNT; p++)
		{
			arrivalsHorizon[h][p]= 0; //Init all to zero
		}
	}

	upstrDetectors[0] = qpg_NET_detector("NS_UPSTREAM_DETECTOR");
	upstrDetectors[1] = qpg_NET_detector("EW_UPSTREAM_DETECTOR");
	upstrDetectors[2] = qpg_NET_detector("SN_UPSTREAM_DETECTOR");
	upstrDetectors[3] = qpg_NET_detector("WE_UPSTREAM_DETECTOR");

	stoplDetectors[0] = qpg_NET_detector("NS_STOPLINE_DETECTOR");
	stoplDetectors[1] = qpg_NET_detector("EW_STOPLINE_DETECTOR");
	stoplDetectors[2] = qpg_NET_detector("SN_STOPLINE_DETECTOR");
	stoplDetectors[3] = qpg_NET_detector("WE_STOPLINE_DETECTOR");

	int idxApproach = 0;
	for (int i = 0; i < 8 ; i ++)	
	{
		int laneDet = i%NUM_LANES + 1;  // 1 or 2
		loopDetectorData[i].upstreamDetector = upstrDetectors[idxApproach];
		loopDetectorData[i].upstreamDecLoop = qpg_DTC_multipleLoop(upstrDetectors[idxApproach], laneDet);
		loopDetectorData[i].lane = laneDet; 
		loopDetectorData[i].lastCount = 0;

		if (idxApproach == 0  || idxApproach == 2)	// Phase C 
		{
			loopDetectorData[i].phase = 2;
		}
		else // Approaches 1 and 3
		{
			if (laneDet == 1)
				loopDetectorData[i].phase = 0;	//Phase A
			else // TODO: Assumption- all vehicles on the inner lane are turning left, not used now, using probability dist
				loopDetectorData[i].phase = 1;	//Phase B
		}

		idxApproach = idxApproach + i%2; // +1 every 2 cycles
		/*
			loop 0	1	2	3	4	5	6	7	
			appr 0	0	1	1	2	2	3	3
		*/
	}

	/********		 COP instance(s)		******/

	instances.push_back(COP97A::Cop97A(2, 70));  //empty with initial phase and horizon

	instances[0].setMaxPhCompute(MAX_SEQUENCE);
	instances[0].setOutput(false);
	//instances[0].setInitialPhase(2);
	instances[0].setStartupLostTime(2.0);
	instances[0].setMinGreenTime(MIN_GREEN);
	instances[0].setMaxGreenTime(MAX_GREEN);
	instances[0].setRedTime(ALL_RED);

	instances[0].setSaturationFlow(0, 1800); //in vehicles-per-hour per-lane 
	instances[0].setSaturationFlow(1, 1400);
	instances[0].setSaturationFlow(2, 1800);

	instances[0].setLanePhases(0, 1); //no. of lanes for each phase, used for saturation flow
	instances[0].setLanePhases(1, 1);
	instances[0].setLanePhases(2, 2);

}


/* ---------------------------------------------------------------------
* Include GUI elements
* --------------------------------------------------------------------- */

void qpx_DRW_modelView(void)
{
	//TODO: Implement
}

/* ---------------------------------------------------------------------
* time to arrival in mps
* --------------------------------------------------------------------- */

float getEstimatedArrivalTime (float time, float speed, int distance)
{
	return time + distance/speed; //sec + metres / metres/sec
}

/* ---------------------------------------------------------------------
* Step adjustement for the horizon, only for 0.5s resolution
* --------------------------------------------------------------------- */

float adjustToStep(float num)
{
	float decimal = num - floor(num);

	if (decimal >= 0  && decimal <= 3)
	{
		return floor(num);
	}
	else
	{
		if(decimal >= 4  && decimal <= 6)
		{
			return (float)(floor(num) + 0.5);
		}
		else
		{
			if (decimal >= 7) //include 0 next int
				return ceil(num);
		}
	}
	return floor(num);
}

/* ---------------------------------------------------------------------
* determines current state of the prediction horizon for this arrival
* --------------------------------------------------------------------- */

float getHorizonStep(ARRIVALDATA arrival, float simulationTime)
{
	float elapsedTime = simulationTime - arrival.detectionTime;
	float distanceToStopline = UPSTREAM_DETECTOR_DISTANCE - arrival.speed * elapsedTime; // toDetector - travelled
	float timeToStopline = distanceToStopline / arrival.speed;
	return adjustToStep(timeToStopline); // floor, .5 or ceiling
}

/* ---------------------------------------------------------------------
* Initialise the horizon to zero
* --------------------------------------------------------------------- */

void clearHorizon()
{
	for (int h=0; h< HORIZON_SIZE; h++)
	{
		for (int p= 0; p <PHASE_COUNT; p++)	
		{
			arrivalsHorizon[h][p]= 0; 
		}
	}	
}

/* ---------------------------------------------------------------------
* include in the horizon or remove them from the detection data
* --------------------------------------------------------------------- */

void updateHorizon( float simulationTime)
{
	clearHorizon();

	for(unsigned int i=0; i<detectedArrivals.size(); i++) // for all detected vehicles
	{
		if (detectedArrivals[i].arrivalTime < simulationTime) // if estimated arrival has passed
		{
			std::vector<ARRIVALDATA>::iterator it = detectedArrivals.begin();
			std::advance(it, i);
			detectedArrivals.erase(it);		// remove vehicle from the vector!
		}
		else	// if vehicle still in the link, add it to the current horizon
		{
			float arrivalTimeStep  = getHorizonStep(detectedArrivals[i], simulationTime);
			int horizonTime= (int)(arrivalTimeStep * 2); 
			/*
			0 0.5 1 1.5 2 2.5 ... 34
			0  1  2  3  4  5 ... 69
			*/

			if (horizonTime < HORIZON_SIZE) //70-s (0-69)
			{
				ARRIVALDATA detected = detectedArrivals[i];
				arrivalsHorizon[horizonTime][detected.phase]+=1;
			}
		}
	}
}

/* ---------------------------------------------------------------------
* Use upstream and stopline detector data to estimate queue lengths
* --------------------------------------------------------------------- */

void estimateQueues(float currentTime)
{
	//TODO: using diff btwn num. of expected arrival and actually departed vehicles...
}

/* ---------------------------------------------------------------------
* print current horizon to file
* --------------------------------------------------------------------- */

void printVectorToFile()
{
	ofstream myfile;
	myfile.open ("C:\\PARAMICS\\arrival-horizon.txt");
	if (myfile.is_open())
	{
		for(int h=0; h<70; h++){
			for(int p=0; p<3; p++)
			{
				myfile << arrivalsHorizon[h][p] << " ";
			}
			myfile << "\n";
		}
	}
	myfile.close();
}

/* ---------------------------------------------------------------------
* determine phase (A or B) for vehicle using a detector 
* --------------------------------------------------------------------- */

int getPhase(int detectorIndex)
{
	
	/* TODO: Use better distribution e.g
	 *
	 * std::linear_congruential<int, 16807, 0, (int)((1U << 31) - 1)> eng;
	 * eng.seed(eng);
	 * std::tr1::uniform_real<double> unif(0, 1);
	 * double udr = unif(eng);
	 * OR
	 * int qpg_UTL_randomNormalInteger(int stream, int mean, int std_dev); 
	 */

	double udr = ((double) rand() / (RAND_MAX+1));		/*	a uniformly distributed random value	*/
	int phase = udr > leftTurnProportion ? 0 : 1; //A=0;  B = 1; C=2,

	/*	A {1, 2, 4, 5} B or C {2, 3, 6, 7}	*/	
	if (detectorIndex < 2 || (detectorIndex > 3 && detectorIndex < 6)) // A
		phase = 2;

	/*
	loop 0	1	2	3	4	5	6	7	
	appr 0	0	1	1	2	2	3	3
	*/
	return phase;
}

/* ---------------------------------------------------------------------
* sets a phase in the controller
* --------------------------------------------------------------------- */

void setController(int ph, int pri)
{
	int priority;

	for (unsigned int i = 0; i < MOVEMENT_COUNT; i++)
	{
		char inlnk[10];
		char outlnk[10];
		strcpy_s(inlnk,  phasing[ph][i].inlink.c_str());
		strcpy_s(outlnk, phasing[ph][i].outlink.c_str());

		if (pri == APIPRI_BARRED)
			priority = pri;
		else
			priority = phasing[ph][i].priority;

		qps_LNK_priority(qpg_NET_link(inlnk), qpg_NET_link(outlnk), priority);
	}     
}

void setControllerAllRed()
{
	setController(0, APIPRI_BARRED);
	qps_GUI_printf("--------->Controller set to RED for %is", ALL_RED); 
}

void setControllerNext(int ph)
{
	setController(ph, APIPRI_MAJOR);
	qps_GUI_printf("------------->Controller set to [%s] for %is", phases[ph], currentControl); 
}

/* ---------------------------------------------------------------------
* every simulated step, works best at 0.5s resolution
* --------------------------------------------------------------------- */

void qpx_NET_timeStep()
{
	float step = qpg_CFG_timeStep();
	float currentTime = qpg_CFG_simulationTime();

	for(int i=0; i < 8 ; i ++) 	/* check all upstream loop detectors; two per approach	*/
	{
		LOOP* theLoop = loopDetectorData[i].upstreamDecLoop;
		int currentCount = qpg_DTL_count(theLoop, 0); 		/*	zero to count all vehicle types	*/

		if (currentCount != loopDetectorData[i].lastCount) 		/*	add new vehicles detectedArrivals	*/
		{
			// TODO:  if queue reaches upstream then rubbish data... VALIDATE
			ARRIVALDATA dta;
			dta.speed = qpg_DTL_speed(theLoop, APILOOP_COMPLETE);
			dta.arrivalTime = getEstimatedArrivalTime(currentTime, dta.speed, UPSTREAM_DETECTOR_DISTANCE);
			dta.detectionTime = currentTime;
			dta.phase = getPhase(i);
			detectedArrivals.push_back(dta);
			loopDetectorData[i].lastCount = currentCount;
		}
	}

	updateHorizon(currentTime);
	estimateQueues(currentTime);

	if(!isThreadRunning)		/*	manage algorithm thread 	*/
	{	
		if(hThread != NULL)
			CloseHandle (hThread);	/*  close finished thread	*/

		if (isAllRed)
			hThread = new HANDLE();
		else
			hThread = (HANDLE)_beginthreadex( NULL, 0, &COPThreadFunc, NULL, 0, &threadID);		/* init new thread */
			
	}

	float timeToRed = lastControlTime + currentControl; /*	switch points */
	float timeToNext = timeToRed + ALL_RED;

	if (isSequenceReady)
	{
		if (!isAllRed)									/* discard sequences till next phase*/	
		{
			std::vector<CONTROLDATA> _new (tempSeq);
			controlSeq.clear();
			controlSeq.reserve(_new.size());
			controlSeq.swap(_new);
		}

		if (isFirstTime)
		{
			timeToNext = currentTime;
			isFirstTime =false;
			isAllRed = false;
		}

		if (timeToRed == currentTime)
		{
			setControllerAllRed();
			isAllRed = true;
		}

		if (timeToNext == currentTime)
		{
			if(controlSeq.size() > 0)						/* at this point, control and temp seqs are the same */
			{
				CONTROLDATA nxt = controlSeq.back();
				controlSeq.pop_back();
				if (isAllRed)
					tempSeq.pop_back();								
				currentControl = nxt.duration;
				setControllerNext(nxt.phase);
				nextPhase = nxt.phase;
			}
			else
			{
				currentControl = MAX_GREEN;				/*	if no control seq, set max	green*/
				setControllerNext(nextPhase);
			}

			isAllRed = false;
			nextPhase = (nextPhase == 2) ? 0: nextPhase+1;		/*	prepare next phase	*/
			lastControlTime = currentTime;
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
		printVectorToFile();
		qps_GUI_printf("********* PRINTED ************");
	}
}
