#ifdef FROSTALGORITHMS_EXPORTS
#define  APPQL_API __declspec(dllexport) 
#else
#define APPQL_API  __declspec(dllimport) 
#endif

#ifndef FROST_ALGORITHMS_APPQL
#define FROST_ALGORITHMS_APPQL

// Compile Options:  /GX
namespace std {
#include <cstdlib>

};	
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <set>
#include <random>
#include <algorithm>
#include "AppQLearningPolicy.h"

//using namespace System;

namespace APPQL {

	class AppQLearning
	{
	public: 
		//AppQLearning();
		APPQL_API AppQLearning();
		APPQL_API AppQLearning(char*, int);	//load from file
		APPQL_API AppQLearning(std::vector<int>, int, int);	//load from vector
		APPQL_API AppQLearning(char*, int, int, int); //load from string
		APPQL_API AppQLearning(char*, int, int); //load from file with Horizon
		APPQL_API AppQLearning::AppQLearning(std::vector<std::vector<int> >, int iphase, int horizon); // load from multiarray
		APPQL_API AppQLearning::AppQLearning(int iphase, int horizon);
		APPQL_API std::vector<int> getFeasibleGreens(int, int);
		APPQL_API int getInitialPhase();
		APPQL_API  int getRed();
		APPQL_API int getArrivals(int, int, int);
		APPQL_API std::vector<int> getOptimalControl(); 
		APPQL_API float getSaturationFlow(int); 
		APPQL_API void setSaturationFlow(int phi, float satFlow);
		APPQL_API void setStartupLostTime(float time);
		APPQL_API void setInitialPhase(int p);
		APPQL_API void setMinGreenTime(int mingreen);
		APPQL_API void setMaxGreenTime(int maxgreen);
		APPQL_API void setRedTime(int redd);
		APPQL_API void setLanePhases(int phi, int lanes);
		APPQL_API void setHorizon(int h);
		APPQL_API void setMaxPhCompute(int mp);
		APPQL_API void setArrivals(std::vector<std::vector<int> > arrivals);
		APPQL_API int getArrivalEarliest(int, int, int, int); //NEW

		APPQL_API void resizeArrivals();
		APPQL_API void initMatrices(int);
		APPQL_API void printVector(std::vector<int> );
		APPQL_API void printMatrix(std::vector<std::vector<int> > );
		APPQL_API std::vector<int> printSequence(int[], int);
		APPQL_API void printArrivals();
				
		void AppQLearning::updateReward(int nReward);
		void AppQLearning::selectNextAction();

		APPQL_API std::vector<int> Run();
		APPQL_API bool loadFromFile(char*);
		APPQL_API bool loadFromSeq(char*, unsigned int, int);
		APPQL_API bool loadFromVector(std::vector<int>, int);
		APPQL_API void initParameters();

		APPQL_API void setOutput(bool);
		//void setNextPhase(int phaseIndex);
	
		APPQL_API APPQL::AppQLearningPolicy getPolicy();
		APPQL_API bool getRandomFlag();
		APPQL_API void setAlpha(double a);
		APPQL_API double getAlpha();
		APPQL_API void setGamma(double g);
		APPQL_API double getGamma();
		APPQL_API void setEpsilon(double e);
		APPQL_API double getEpsilon();
		APPQL_API void initPolicy();
		APPQL_API bool validAction(int action);		/* check whether action is permitted */
		
		/*	Invoked by thread in the controller	*/
		APPQL_API int selectAction(APPQL::AppQLearningPolicy::AppQLearningSTATE iState);

		APPQL_API void setNewState(APPQL::AppQLearningPolicy::AppQLearningSTATE iState);
		APPQL_API void setNewReward(double oReward);
		APPQL_API void setNewStateReward(APPQL::AppQLearningPolicy::AppQLearningSTATE iState, double oReward);
		APPQL_API void setInitialState(APPQL::AppQLearningPolicy::AppQLearningSTATE st);
		APPQL_API void updateQ();
		APPQL_API void updateState();

	private:

		enum PIEnum {
			QUEUES, STOPS, DELAY
		};

		int PI;
		int red;
		int mingreen;
		int maxgreen;
		float startupLostTime; //time from total halt to free-flow (2 secs)
		unsigned int T; //planning horizon
		unsigned int M; //maximum number of phases to compute
		int initialPhase; // = 2;								//-----------------> state rep
		int idxCurrentPh; //= initialPhase; // set initial phase to C (2)
		//float satHeadway; // avg headway between vehicles during saturated flow
		//float satFlowRate; // = 0.0; No.Lanes / satHeadway
		float satFlows[3]; //per phase
		int lanePhases[3]; //per phase
		bool output;
		char phaseSeq[3]; // A, B, C
		std::vector<int> phases; // A = 0, B = 1, C = 2
		std::vector<int> optControlSequence; // A = 0, B = 1, C = 2
		std::vector< std::vector<int> > arrivalData;		//---------------------> state rep
		std::vector< std::vector<int> > v; //v_j(s_j);
		std::vector< std::vector<int> > x_star; // optimal solutions x*_j(s_j)
		
		/* ---------------------------------------------------------------------
		* RL
		* --------------------------------------------------------------------- */
		
		double epsilon;		/*	for epsilon-greedy	*/
	    double temp;

		double alpha;		/*	learning rate	*/
		double gamma;		/*	discount factor	*/
		double lambda;

		APPQL::AppQLearningPolicy::AppQLearningSTATE state;
		APPQL::AppQLearningPolicy::AppQLearningSTATE newState;
		APPQL::AppQLearningPolicy policy;
		int action;				//TODO: Use enum instead
		double reward;
		bool random;

		std::mt19937 eng;

	};
}

#endif