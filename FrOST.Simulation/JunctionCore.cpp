#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <string>
#include <windows.h>
#include <fstream>
#include <process.h>
#include <time.h>

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

namespace CORE{

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

	/*definition*/
	string id;
	NODE* node;
	vector<LOOPDATA> upstreamLoopData;
	vector<LOOPDATA>  stoplineLoopData;
	vector<DETECTOR*> upstreamDetectors;
	vector<DETECTOR*>  stoplineDetectors;
	//const char*	phases [3] = {"A", "B", "C"};
	vector<char*> phases;
	double leftTurnProportion = 0.1; /* simplified turning proportions, must agree OD Matrix */ //nbefore 0.2
	double rightTurnProportion = 0.1;
	vector<string>  directions;

	/*state*/
	vector<vector<SIGPRI> > phasing;	//switching rules
	vector<vector<int> > arrivalsHorizon;	//predicted for each approach
	vector<ARRIVALDATA> detectedArrivals;	//vehicles reaching the intersection
	vector<int> stoplineArrivals; // total vehicles arriving at stop line 
	vector<int> eQueueCount; // in vehicles per phase
	int currentPhaseIndex = 0;
	float timeToRed = 0;
	float timeToNext = 0;

	/*controller*/
	REAP1::ReAP1 agentController;
	vector<int> control;
	vector<CONTROLDATA> controlSeq;
	vector<CONTROLDATA> tempSeq;
	HANDLE hThread = NULL;
	unsigned threadID;

	/*flags*/
	bool isThreadRunning = false;
	bool isSequenceReady = false;
	bool isFirstTime = true;
	bool isAllRed = false;

	/*reward model*/
	double prevDelay = 0.0;	//NEW 
	double currentDelay = 0.0;//NEW 

	float lastControlTime = 0.0;
	int nextPhase = INITIAL_PHASE_INDEX;
	int nextControl = 0;
	int currentControl = 0;
	int seqIndex = 0;

	REAP1::ReAP1Policy::REAP1STATE xState;
	bool actionTaken = false;
	int action = -1;

	/*files*/
	char * phasing_file;


	/* ---------------------------------------------------------------------
	* FUNCTIONS
	* --------------------------------------------------------------------- */

	JunctionCore::JunctionCore(){

		stoplineArrivals.resize(PHASE_COUNT);
		eQueueCount.resize(PHASE_COUNT);

		isThreadRunning = false;
		isSequenceReady = false;
		isFirstTime = true;
		isAllRed = false;

		leftTurnProportion = 0.1; /* simplified turning proportions, must agree OD Matrix */ //nbefore 0.2
		rightTurnProportion = 0.1;

		prevDelay = 0.0;	//NEW 
		currentDelay = 0.0;//NEW 

		lastControlTime = 0.0;
		nextPhase = INITIAL_PHASE_INDEX;
		nextControl = 0;
		currentControl = 0;
		seqIndex = 0;

		actionTaken = false;
		action = -1;

		phases.resize(3);
		char* vv[3] = { "A","B","C" };
		phases.assign(&vv[0], &vv[0]+3);
		//phases = {"A","B","C"};

		directions.resize(5);
	}

	JunctionCore::~JunctionCore(){

	}

	/* ---------------------------------------------------------------------
	* Thread function!
	* --------------------------------------------------------------------- */

	unsigned __stdcall JunctionCore::ThreadStaticFunc( void* data ){
		JunctionCore * pthX = (JunctionCore*)data;   // the tricky cast
		pthX->ThreadFunc(data);           // now call the true entry-point-function
		// A thread terminates automatically if it completes execution,
		// or it can terminate itself with a call to _endthread().
		return 1;          // the thread exit code
	}

	unsigned __stdcall JunctionCore::ThreadFunc(void * data)
	{
		clock_t tStart = clock();
		//TODO: check to clear control
		int cPhase = nextPhase;
		isThreadRunning = true;
		/* RL ------interaction steps-------	*/
		if(!actionTaken)
		{
			action = agentController.selectAction(xState); //5	// TODO: check whether to init -state field- somewhere else...  
			control.clear();
			switch(action)
			{
			case 0:	cPhase = currentPhaseIndex;	// back to current phase; TODO: manage all-red
				control.push_back(GREEN_EXTENSION);	//NEW
				//currentControl += GREEN_EXTENSION;	//extend
				break;
			case 1: control.push_back(MIN_GREEN);	// switch next
				break;
			case 2: control.push_back(0); control.push_back(MIN_GREEN); // skip next and switch
				break;
			}
			//qps_GUI_printf("\a ACTION %i",action);
		}
		else{
			/*	When controller has completed the last action	*/
			agentController.setNewState(xState);	//6.1
			agentController.setNewReward(currentDelay - prevDelay); //6.2
			agentController.updateQ();	//7
			agentController.updateState(); //8
			qps_GUI_printf("\aJ-%s: Obtained REWARD of {%4.2fs}", id.c_str(),currentDelay - prevDelay);
			actionTaken = false;	//NEW
		}
		/* ------end RL interaction steps-------	*/

		//	control = agentController.RunREAP();
		/* to run it at a predetermined frequency, add ms to ttaken and sleep, e.g, Sleep( 5000L - ttaken ); */
		double ttaken = (double)(clock() - tStart)/CLOCKS_PER_SEC;
		if (!isAllRed)
		{				/* late check to avoid algorithm latency issues */
			string str;
			std::stringstream message;
			//cPhase = nextPhase;
			tempSeq.clear();			
			tempSeq.reserve(MAX_SEQUENCE+1);		// TODO: check it

			for (vector<int>::const_iterator i = control.begin(); i != control.end(); ++i)
			{
				int dur = *i;
				if (dur > 0)				/*	skip phase	*/
				{
					CONTROLDATA ctrl;
					ctrl.phase = cPhase;
					ctrl.duration = dur;
					auto bg =  tempSeq.begin();
					tempSeq.insert(bg,ctrl);
				}

				message << phases[cPhase] << ":" << dur << "\t";

				cPhase = (cPhase == 2) ? 0: cPhase+1;				/* update phase	*/
			}

			float hh = qpg_CFG_simulationTime();
			float mm =  fmod(hh, 60); 
			hh = hh / 60;
			char * act = "> NONE";
			switch(action)
			{
			case 0:	act = "> EXTEND";
				break;
			case 1: act = "> NEXT";
				break;
			case 2: act = "> SKIP-&-NEXT";
				break;
			}

			qps_GUI_printf("\aJ-%s %i:%4.1f \t %s %s",id.c_str(),(int)hh ,mm, message.str().c_str(), act);
			// ttaken -> for Q-Learning simple table look-up
			//qps_GUI_printf("\a Core: %im:%4.1fs \t%4.2fs \t %s %s",(int)hh ,mm, ttaken, message.str().c_str(), act);
		}
		isThreadRunning = false;
		isSequenceReady = tempSeq.size() > 0;
		return 0;
	} 

	/* ---------------------------------------------------------------------
	* split functions
	* --------------------------------------------------------------------- */

	std::vector<std::string> &JunctionCore::split(const std::string &s, char delim, std::vector<std::string> &elems) {
		std::stringstream ss(s);
		std::string item;
		while (std::getline(ss, item, delim)) {
			elems.push_back(item);
		}
		return elems;
	}

	std::vector<std::string> JunctionCore::split(const std::string &s, char delim) {
		std::vector<std::string> elems;
		split(s, delim, elems);
		return elems;
	}

	/* ---------------------------------------------------------------------
	* read phase configuration from file
	* --------------------------------------------------------------------- */

	int JunctionCore::toPrioEnum(string val)
	{
		if (val == "APIPRI_MAJOR")
			return APIPRI_MAJOR;
		if (val == "APIPRI_MEDIUM")
			return APIPRI_MEDIUM;
		if (val == "APIPRI_MINOR")
			return APIPRI_MINOR;

		return APIPRI_BARRED;
	}

	void JunctionCore::loadPhasingFile(char * phasing_file)
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
			qps_GUI_printf(">>> Loaded phasing specs from file"); 
		}

		else qps_GUI_printf(">>> Unable to open phasing file"); 

	}

	string  JunctionCore::string_replace( string src, string const& target, string const& repl)
	{
		// handle error situations/trivial cases
 
		if (target.length() == 0) {
		// searching for a match to the empty string will result in
		// an infinite loop
		// it might make sense to throw an exception for this case
		return src;
		}
 
		if (src.length() == 0) {
		return src; // nothing to match against
		}
 
		for (size_t idx = src.find( target); idx != string::npos; idx = src.find( target, idx)) {
		src.replace( idx, target.length(), repl);
		idx += repl.length();
		}
 
		return src;
	}

	string  JunctionCore::replaceDir(string line)
	{
		line = string_replace(line, "o",directions[0]);
		line = string_replace(line, "n",directions[1]);
		line = string_replace(line, "s",directions[2]);
		line = string_replace(line, "e",directions[3]);
		line = string_replace(line, "w",directions[4]);

		return line;	
	}

	void JunctionCore::loadPhasingFileGrid(char * phasing_file, vector<string> dirs)
	{
		string line;
		ifstream myfile (phasing_file);
		int iPhase = 0;
		int iMov = 0;
		directions = dirs;
		if(myfile.is_open())
		{
			while (getline (myfile,line))
			{
				//nsew#
				/*line.replace(line.begin(), line.end(), "o", dirs[0].c_str());
				line.replace(line.begin(), line.end(), "n", dirs[1].c_str());
				line.replace(line.begin(), line.end(), "s", dirs[2].c_str());
				line.replace(line.begin(), line.end(), "e", dirs[3].c_str());
				line.replace(line.begin(), line.end(), "w", dirs[4].c_str());*/


				//string origin = dirs[0];
				//string north = dirs[1];
				//string south = dirs[2];
				//string east = dirs[3];
				//string west = dirs[4];
					
				string repline = replaceDir(line);
				std::vector<std::string> prio =  split(repline, ' ');
				phasing[iPhase][iMov].inlink = replaceDir(prio[0]);
				phasing[iPhase][iMov].outlink = replaceDir(prio[1]);
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
			qps_GUI_printf(">>> Loaded phasing specs from file"); 
		}

		else qps_GUI_printf(">>> Unable to open phasing file"); 

	}

	//string JunctionCore::replaceNode (string id)
	//{
	//	if (id == "n") return directions[0];
	//	if (id == "s") return directions[1];
	//	if (id == "e") return directions[2];
	//	if (id == "w") return directions[3];
	//}
	//
	//
	//std::string JunctionCore::toNodeDirection(std::string link)
	//{
	//	/*
	//			N0
	//		W3		E2
	//			S1
	//	*/
	//	std::vector<std::string> nodeIds = split(link, ':');
	//	//	o n s e w
	//	ostringstream ost;
	//	ost << replaceNode(nodeIds[0]) << ":" << replaceNode([nodeIds[1]);
	//	string ans= ost.str();
	//	return ans;
	//}

	//void JunctionCore::loadPhasingFileGrid(char * phasing_file)
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
	//			phasing[iPhase][iMov].inlink = toNodeDirection(prio[0]);
	//			phasing[iPhase][iMov].outlink = toNodeDirection(prio[1]);
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
	* time to arrival in mps
	* --------------------------------------------------------------------- */

	float JunctionCore::getEstimatedArrivalTime (float time, float speed, int distance)
	{
		return time + distance/speed; //sec + metres / metres/sec
	}

	/* ---------------------------------------------------------------------
	* Step adjustement for the horizon, only for 0.5s resolution
	* --------------------------------------------------------------------- */

	float JunctionCore::adjustToStep(float num)
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

	float JunctionCore::getHorizonStep(ARRIVALDATA arrival, float simulationTime)
	{
		float elapsedTime = simulationTime - arrival.detectionTime;
		float distanceToStopline = UPSTREAM_DETECTOR_DISTANCE - arrival.speed * elapsedTime; // toDetector - travelled
		float timeToStopline = distanceToStopline / arrival.speed;
		return adjustToStep(timeToStopline); // floor, .5 or ceiling
	}

	/* ---------------------------------------------------------------------
	* Initialise the horizon to zero
	* --------------------------------------------------------------------- */

	void JunctionCore::clearHorizon()
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

	void JunctionCore::updateHorizon( float simulationTime)
	{
		clearHorizon();

		for(unsigned int i=0; i<detectedArrivals.size(); i++) // for all detected vehicles
		{
			if (detectedArrivals[i].arrivalTime < simulationTime) // if estimated arrival has passed
			{														
				int iPhase = detectedArrivals[i].phase;
				std::vector<ARRIVALDATA>::iterator it = detectedArrivals.begin();
				std::advance(it, i);
				detectedArrivals.erase(it);		// remove vehicle from the vector!
				// difference between counts then is queueing!
				stoplineArrivals[iPhase] += 1;		// NEW : EAT has elapsed, then add to stopline arrivals
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

					//	if(horizonTime < 1)	//imminent arrivals; single point in time to avoid multiple count
					//	{
					//		qps_GUI_printf("+++ ARRIVAL by %i", detectedArrivals[i].phase);	// NEW add arrivals counter; tested OK
					//		stoplineArrivals[detectedArrivals[i].phase] += 1;
					//	}
				}
			}
		}
	}

	/* ---------------------------------------------------------------------
	* return total expected arrivals (5 steps) per phase
	* --------------------------------------------------------------------- */

	int JunctionCore::lookupExpectedArrivals(int phaseIdx, int nEntries)		//NEW
	{	
		int exArrivals = 0;

		for(int h=1; h < nEntries; h++)
		{
			exArrivals = arrivalsHorizon[nEntries][phaseIdx];
		}

		return exArrivals;
	}

	/* ---------------------------------------------------------------------
	* determine phase (A or B) for vehicle using a detector 
	* --------------------------------------------------------------------- */

	int JunctionCore::getPhaseByProbability(int detectorIndex)
	{
		int phase = 2;
		//srand(time(0));
		/*	C {0, 1, 4, 5} A or B {2, 3, 6, 7}	*/	
		if (detectorIndex > 5 || (detectorIndex > 1 && detectorIndex < 4)) // A or B
		{
			double udr =  qpg_UTL_randomDouble(APIRNG_MISC, 1);
			//	qps_GUI_printf("--rnd  %4.2f", udr);
			phase = udr > leftTurnProportion ? 0 : 1; //A=0;  B = 1; C=2,
		}

		/*
		loop 0	1	2	3	4	5	6	7	
		appr 0	0	1	1	2	2	3	3
		*/
		return phase;
	}

	/* ---------------------------------------------------------------------
	* returns 2 if phase C; 0 if phase A or B
	* --------------------------------------------------------------------- */

	int JunctionCore::getDetectorPhaseGroup(int detectorIndex)
	{
		if (detectorIndex > 5 || (detectorIndex > 1 && detectorIndex < 4))
			return 0;
		else
			return 2;
	}

	/* ---------------------------------------------------------------------
	* Use upstream and stopline detector data to estimate queue lengths per phase
	* --------------------------------------------------------------------- */

	void JunctionCore::estimateQueues(float currentTime)		//NEW
	{
		//int exArrivals[3] = {0,0,0};
		//eQueueCount[0]= 0;eQueueCount[1]= 0;eQueueCount[2]= 0;
		int stopLineDepartures[3] = {0,0,0};

		for(int detectorIndex=0; detectorIndex < 8 ; detectorIndex++) 	/* check all loop detectors  (stopline); two per approach	*/
		{
			LOOP* stoplineLoop = stoplineLoopData[detectorIndex].loop;
			int currentStopCount = qpg_DTL_count(stoplineLoop, 0); 		/*	zero to count all vehicle types	*/

			//int	iPhase = getPhaseByProbability(detectorIndex);	// prev implem - noisy function; find another solution

			int phGroup = getDetectorPhaseGroup(detectorIndex);

			if (phGroup == 2)
				stopLineDepartures[phGroup] += currentStopCount;
			else
			{
				int leftTurners = (int) (currentStopCount * leftTurnProportion); //NEW!!! distribute based on turning rate!
				stopLineDepartures[0] += currentStopCount - leftTurners;	//TODO: allocate left over unit somewhere?
				stopLineDepartures[1] += leftTurners;
			}																//TODO: use currentPhaseIndex to discount difference btwn counts?

			//stopLineDepartures[iPhase] += currentStopCount;	// prev implementation
			stoplineLoopData[detectorIndex].lastCount = currentStopCount;	// previous reading
		}

		int diff;											/* deal with error from turning probability */
		if (stoplineArrivals[1] < stopLineDepartures[1])
		{
			diff = stopLineDepartures[1] - stoplineArrivals[1];
			stoplineArrivals[0] -= diff;
			stoplineArrivals[1] += diff;
		}
		else
		{
			if (stoplineArrivals[0] < stopLineDepartures[0])
			{
				diff = stopLineDepartures[0] - stoplineArrivals[0];
				stoplineArrivals[1] -= diff;
				stoplineArrivals[0] += diff;
			}
		}
		for (int phi = 0; phi < PHASE_COUNT; phi++)
		{
			eQueueCount[phi] = stoplineArrivals[phi] - stopLineDepartures[phi]; //- currentStopCount;

			if (phi != currentPhaseIndex && eQueueCount[phi] > 2)
			{
				eQueueCount[phi] += 2;		/*	stopped vehicles on stop line detector for alls phases (A B c) */
				if (phi == 2)
					eQueueCount[phi] += 2;	/*	stopped vehicles on stop line detector for phase C*/

			}
		}

		//qps_GUI_printf(">>> stopline arrivals: A[%i] B[%i] C[%i]", stoplineArrivals[0], stoplineArrivals[1], stoplineArrivals[2]); 
		//qps_GUI_printf(">>> stopline count   : A[%i] B[%i] C[%i]", stopLineDepartures[0], stopLineDepartures[1], stopLineDepartures[2]); 

		//qps_GUI_printf(">> Current queues : A[%i] B[%i] C[%i]", eQueueCount[0], eQueueCount[1], eQueueCount[2]); 

		/*
		the difference between the number of expected arrival vehicles and the actually departed vehicles at the stop line
		expected arrival vehicles: from the arrivals Horizon (expected)
		actually departed vehicles at stop line
		*/
	}

	void JunctionCore::estimateDelay()			//TODO: is just delay of queueing vehicles
	{
		prevDelay = currentDelay;
		for (int i=0; i<PHASE_COUNT; i++)
		{
			if(currentPhaseIndex != i)
				currentDelay += 0.5 * eQueueCount[i];	// queue by time step

		}
	}

	void JunctionCore::updateState()
	{
		xState.phaseIndex = currentPhaseIndex;
		xState.greenRemaining = (int)timeToRed;	//TODO: deal with loss of data
		std::vector<int> quLengths;
		int eq;
		for (eq= 2; eq >= 0; eq-- )
		{
			if (eQueueCount[eq] > 10)     //TODO: queue groups 
				quLengths.push_back(10);
			else
				quLengths.push_back(eQueueCount[eq]);
		}
		//quLengths.push_back(eQueueCount[2]);quLengths.push_back(eQueueCount[1]);quLengths.push_back(eQueueCount[0]); 
		xState.queueLengths = quLengths;

	}

	/* ---------------------------------------------------------------------
	* print current horizon to file
	* --------------------------------------------------------------------- */

	void JunctionCore::printVectorToFile()
	{
		ofstream myfile;

		std::ostringstream oss;
		oss << "C:\\temp\\arrival-horizon_" << id << ".txt"; 
		std::string file_name = oss.str(); 
		myfile.open(file_name.c_str()); 
		myfile.open (file_name);	//TODO: test
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
	* sets a phase in the controller
	* --------------------------------------------------------------------- */

	void JunctionCore::setController(int ph, int pri)
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

	void JunctionCore::setControllerAllRed()
	{
		setController(0, APIPRI_BARRED);
		qps_GUI_printf("--------->J-%s set to RED for %is",id.c_str(), ALL_RED); 
	}

	void JunctionCore::setControllerNext(int ph)
	{
		currentPhaseIndex = ph;
		setController(ph, APIPRI_MAJOR);
		qps_GUI_printf("------------->J-%s set to [%s] for %is",id.c_str(), phases[ph], currentControl); 
	}

	/* ---------------------------------------------------------------------
	* probe loop detectors for vehicles
	* --------------------------------------------------------------------- */
	void JunctionCore::probeUpstreamDetectors(float currentTime)
	{
		for(int i=0; i < 8 ; i ++) 	/* check all loop detectors (upstream); two per approach	*/
		{
			LOOP* upstLoop = upstreamLoopData[i].loop;
			int currentCount = qpg_DTL_count(upstLoop, 0); 		/*	zero to count all vehicle types	*/

			if (currentCount != upstreamLoopData[i].lastCount) 		/*	add new vehicles detectedArrivals	*/
			{
				ARRIVALDATA dta;	
				dta.speed = qpg_DTL_speed(upstLoop, APILOOP_COMPLETE);	
				if (dta.speed == 0.0)	/* validating overflown queue, replace with arbitrarily low speed */
					dta.speed = (float)0.1;
				dta.arrivalTime = getEstimatedArrivalTime(currentTime, dta.speed, UPSTREAM_DETECTOR_DISTANCE);
				dta.detectionTime = currentTime;
				dta.phase = getPhaseByProbability(i);
				detectedArrivals.push_back(dta);
				upstreamLoopData[i].lastCount = currentCount;
			}
		}
	}

	void JunctionCore::manageThread()
	{
		if(!isThreadRunning)		/*	manage algorithm thread 	*/
		{	
			if(hThread != NULL){
				__try{
					CloseHandle (hThread);	/*  close finished thread	*/
				}
				__except(EXCEPTION_EXECUTE_HANDLER)	/*	triggered when debugging	*/
				{
					qps_GUI_printf("CloseHandle Exception Gracefully Managed!"); /* debugger glitch:  http://stackoverflow.com/q/1550315/777877 */
				}
			}
			if (isAllRed)
			{
				hThread = new HANDLE();
			}
			else
			{								//start_address,stack_size, Arglist, Security, Initflag, Thrdaddr
				//hThread = (HANDLE)_beginthreadex( NULL, 0, &JunctionCore::ThreadStaticFunc, NULL, 0, &threadID);		/* init new thread */

				hThread = (HANDLE)_beginthreadex( NULL, // security
					0,  // stack size
					&JunctionCore::ThreadStaticFunc, // entry-point-function
					(LPVOID)this, // arg list holding the "this" pointer
					0, // so we can later call ResumeThread()
					&threadID);		/* init new thread */

			}
		}
	}

	const std::vector<CONTROLDATA> JunctionCore::getTempSeq(){
		return tempSeq;
	}

}