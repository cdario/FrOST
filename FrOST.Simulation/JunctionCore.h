//#ifdef FROSTCORE_EXPORTS
//#define   __declspec(dllexport) 
//#else
//#define   __declspec(dllimport) 
//#endif

#ifndef FROST_ALGORITHMS_CORE
#define FROST_ALGORITHMS_CORE

#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <string>
#include <windows.h>

#ifdef WIN32
#include <Windows.h>
#else
#include <sys/time.h>
#include <ctime>
#endif


extern "C" {
#include "programmer.h"
}

#include "AppQLearning.h"
//#include "REAP1.h"

using namespace std;

namespace CORE{

	enum loopType{
		UPSTREAM, STOPLINE
	};

	typedef struct LOOPDATA_s    LOOPDATA;
	struct LOOPDATA_s
	{
		LOOP* loop;
		DETECTOR * detector;
		loopType type;
		int lane;
		int lastCount;
		int approach;
	};

	typedef struct SIGPRI_s    SIGPRI;
	struct SIGPRI_s
	{
		string inlink;
		string outlink;
		int	  priority;    
	};

	typedef struct ARRIVALDATA_s    ARRIVALDATA;
	struct ARRIVALDATA_s		/*	 represent vehicles inside the prediction horizon	*/
	{
		float arrivalTime;
		float detectionTime;
		float speed;
		int phase;
	};

	typedef struct CONTROLDATA_s    CONTROLDATA;
	struct CONTROLDATA_s
	{
		int phase;
		int duration;
	};

	class JunctionCore {

	public:
		/*definition*/
		string id;
		NODE* node;
		vector<LOOPDATA> upstreamLoopData;
		vector<LOOPDATA>  stoplineLoopData;
		vector<DETECTOR*> upstreamDetectors;
		vector<DETECTOR*>  stoplineDetectors;
		//const char*	phases [3];
		vector<char*> phases;
		double leftTurnProportion; /* simplified turning proportions, must agree OD Matrix */ //nbefore 0.2
		double rightTurnProportion;
		vector<string> directions;

		/*state*/
		vector<vector<SIGPRI> > phasing;	//switching rules
		vector<vector<int> > arrivalsHorizon;	//predicted for each approach
		vector<ARRIVALDATA> detectedArrivals;	//vehicles reaching the intersection
		vector<int> stoplineArrivals; // total vehicles arriving at stop line 
		vector<int> eQueueCount; // in vehicles per phase
		int currentPhaseIndex;
		float timeToRed;
		float timeToNext;

		/*controller*/
		//REAP1::ReAP1 agentController;
		APPQL::AppQLearning agentController;
		vector<int> control;
		vector<CONTROLDATA> controlSeq;
		vector<CONTROLDATA> tempSeq;
		HANDLE hThread;
		unsigned threadID;

		/*flags*/
		bool isThreadRunning;
		bool isSequenceReady;
		bool isFirstTime;
		bool isAllRed;

		/*reward model*/
		double prevDelay;	//NEW 
		double currentDelay;//NEW 

		/*phases*/
		float lastControlTime;
		int nextPhase;
		int nextControl;
		int currentControl;
		int seqIndex;

		/*policy*/
		//REAP1::ReAP1Policy::REAP1STATE xState;
		APPQL::AppQLearningPolicy::AppQLearningSTATE xState;
		bool actionTaken;
		int action;

		/*files*/
		const char * phasing_file;
		//const char * log_file;

		/***********methods*********/

		JunctionCore();
		~JunctionCore();

		/*estimations*/
		float getEstimatedArrivalTime (float time, float speed, int distance);
		void estimateQueues(float currentTime);
		void estimateDelay();
		int lookupExpectedArrivals(int phaseIdx, int nEntries);
		int getPhaseByProbability(int detectorIndex);

		/*horizon*/
		float adjustToStep(float num);
		void clearHorizon();
		float getHorizonStep(ARRIVALDATA arrival, float simulationTime);
		void updateHorizon( float simulationTime);
		void probeUpstreamDetectors(float currentTime);

		/*info and control*/
		int getDetectorPhaseGroup(int detectorIndex);
		void updateState();
		void setController(int ph, int pri);	//NEW! unimplemented
		void setControllerAllRed();
		void setControllerNext(int ph);

		/*threading*/
		void manageThread();
		const std::vector<CONTROLDATA> getTempSeq();
		static unsigned __stdcall ThreadStaticFunc( void* data );
		unsigned __stdcall ThreadFunc(void* data);

		/*helpers*/
		void loadPhasingFile(char* phasing_file);
		void loadPhasingFileGrid(char* phasing_file, vector<string> dirs);
		void printVectorToFile();
		int toPrioEnum(string val);
		std::vector<std::string> split(const std::string &s, char delim);
		std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems);
		//std::string JunctionCore::toNodeDirection(std::string nodeIds);
		//string JunctionCore::replaceNode (string id);
		string  replaceDir(string line);
		string  string_replace( string src, string const& target, string const& repl);

	};
}

#endif