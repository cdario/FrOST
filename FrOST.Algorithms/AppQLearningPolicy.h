#ifdef FROSTALGORITHMS_EXPORTS
#define  APPQLPOLICY_API __declspec(dllexport) 
#else
#define APPQLPOLICY_API  __declspec(dllimport) 
#endif

#ifndef FROST_ALGORITHMS_APPQLPOLICY
#define FROST_ALGORITHMS_APPQLPOLICY

// Compile Options:  /GX
namespace std {
#include <cstdlib>
};	
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <map>
#include <set>


//using namespace System;

namespace APPQL {

	class AppQLearningPolicy{

	public: 
		APPQLPOLICY_API AppQLearningPolicy();

		/* ---------------------------------------------------------------------
		* State variables
		* --------------------------------------------------------------------- */

		struct AppQLearningSTATE_s	// i.e. a road section connected to the intersection
		{	
			std::vector<int> queueLengths;		/*		per phase		*/
			int phaseIndex;
			int greenRemaining;
		};

		struct AppQLearningQVALUES_s			/*		per action 		*/
		{	
			double qValue1;					
			double qValue2;
			double qValue3;
		};

		typedef struct AppQLearningSTATE_s	AppQLearningSTATE;
		typedef struct AppQLearningQVALUES_s	AppQLearningValues;

		AppQLearningPolicy::AppQLearningSTATE tState;
		void AppQLearningPolicy::updateState(AppQLearningSTATE nState);
		int AppQLearningPolicy::printQs();

		int nStates;		/*	(15000) queue length (capped) * phase * remaining (max green)  = 100(3-ph) * 3 * 50	*/
		int nActions;		/*	(3) 0: extend current; 1: apply next phase; 2: skip next and apply 2nd next	*/
		std::vector<double> tQvalues;	/* q-values for the set actions at  the current state */

		/* ---------------------------------------------------------------------
		* Reward definition
		* --------------------------------------------------------------------- */

		int AppQLearningPolicy::reward;		// savings in delay

		/* ---------------------------------------------------------------------
		* Q structures
		* --------------------------------------------------------------------- */
		struct stateCompare {
			inline bool operator()(const AppQLearningSTATE & a, const AppQLearningSTATE & b) const {
				if (a.phaseIndex < b.phaseIndex) return true;
				else{
					if (a.phaseIndex > b.phaseIndex) return false;
					else
					{	// phase equals
						if(a.greenRemaining < b.greenRemaining) return true;
						else
						{
							if(a.greenRemaining > b.greenRemaining) return false;
							else
							{
								//green equals
								if(a.queueLengths[0] < b.queueLengths[0]) return true;
								else
								{
									if (a.queueLengths[0] > b.queueLengths[0]) return false;
									else
										//queue0 equals
										if (a.queueLengths[1] < b.queueLengths[1]) return true;
									else
									{
										if (a.queueLengths[1] > b.queueLengths[1]) return false;
										else
										{
												//queue1 equals
											if (a.queueLengths[2] < b.queueLengths[2]) return true;
											else
											{
												if (a.queueLengths[2] > b.queueLengths[2]) return false;
													//else
														//return true;	
											}
										}
									}
								}
							}
						}
					}
				}
				return false;// same ; irreflexive, strict weak order
			}
		};

//		typedef std::map<REAP1STATE,std::vector< double>, comparatorState> Qtable;
//		Qtable Q;
		std::map<AppQLearningSTATE,AppQLearningValues, stateCompare> Q;
		//std::map <REAP1STATE, std::vector< double>, comparatorState> Q;
		APPQLPOLICY_API void initQValues(double iValue);	/*	1	*/
		APPQLPOLICY_API std::vector< double> getQvalues(AppQLearningSTATE state);
		APPQLPOLICY_API double getQvalue(AppQLearningSTATE state, int action);
		APPQLPOLICY_API double getMaxQvalue(AppQLearningSTATE state);
		APPQLPOLICY_API AppQLearningPolicy::AppQLearningSTATE getStateInstance(std::vector<int> pQueues, int iPhase, int rGreen);
		

		///* ---------------------------------------------------------------------
		//* Function Approximation
		//* --------------------------------------------------------------------- */

		/**
		 * features to model: 
		 *
		 * max_queue_lengths =		10*3 			* approach_state = 		1 to 16
		 * active_phase_index = 	3
		 * elapsed_green =			11
		 */

		/**
		 * f vector of features representing state
		 */

		
		int nFeatures;



		std::vector<double> approxFeatures;	// f_n

		/**
		 *  theta(k,a) to be learned, (approximator)
		 */
		std::vector< std::vector<double> > approxParameters;	// theta^a_n

		double AppQLearningPolicy::getApproxFeature(int feature);
		double AppQLearningPolicy::getApproxParameter(int action, int feature);	

		APPQLPOLICY_API void updateApproxFeatures(AppQLearningSTATE state);		/*	2	*/	//NEW
		APPQLPOLICY_API void AppQLearningPolicy::setApproxParam(int action, int kParam, double newParamValue);	//NEW
		APPQLPOLICY_API void AppQLearningPolicy::setApproxFeature(int kParam, double newFeatureValue);	//NEW

	};
}

#endif