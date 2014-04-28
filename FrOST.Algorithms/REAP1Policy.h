#ifdef FROSTALGORITHMS_EXPORTS
#define  REAP1POLICY_API __declspec(dllexport) 
#else
#define REAP1POLICY_API  __declspec(dllimport) 
#endif

#ifndef FROST_ALGORITHMS_REAP1POLICY
#define FROST_ALGORITHMS_REAP1POLICY

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

namespace REAP1 {

	class ReAP1Policy
	{
	public: 
		REAP1POLICY_API ReAP1Policy();

		/* ---------------------------------------------------------------------
		* State variables
		* --------------------------------------------------------------------- */

		struct REAP1STATE_s	// i.e. a road section connected to the intersection
		{	
			std::vector<int> queueLengths;		/*		per phase		*/
			int phaseIndex;
			int greenRemaining;
		};

		struct REAP1QVALUES_s
		{	
			double qValue1;
			double qValue2;
			double qValue3;
		};

		typedef struct REAP1STATE_s	REAP1STATE;
		typedef struct REAP1QVALUES_s	REAP1QVALUES;

		ReAP1Policy::REAP1STATE tState;
		void ReAP1Policy::updateState(REAP1STATE nState);
		int ReAP1Policy::printQs();

		int nStates;		/*	(15000) queue length (capped) * phase * remaining (max green)  = 100(3-ph) * 3 * 50	*/
		int nActions;		/*	(3) 0: extend current; 1: apply next phase; 2: skip next and apply 2nd next	*/
		std::vector<double> tQvalues;	/* q-values for the set actions at  the current state */

		/* ---------------------------------------------------------------------
		* Reward definition
		* --------------------------------------------------------------------- */

		int ReAP1Policy::reward;

		/* ---------------------------------------------------------------------
		* Q structures
		* --------------------------------------------------------------------- */
		struct stateCompare {
			inline bool operator()(const REAP1STATE & a, const REAP1STATE & b) const {
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
		std::map<REAP1STATE,REAP1QVALUES, stateCompare> Q;

		//std::map <REAP1STATE, std::vector< double>, comparatorState> Q;
		REAP1POLICY_API void initQValues(double iValue);	/*	1	*/
		REAP1POLICY_API REAP1STATE setState(REAP1STATE state);		/*	2	*/
		REAP1POLICY_API std::vector< double> getQvalues(REAP1STATE state);
		REAP1POLICY_API void setQvalue(REAP1STATE state, int action, double newQ);
		REAP1POLICY_API double getQvalue(REAP1STATE state, int action);
		REAP1POLICY_API double getMaxQvalue(REAP1STATE state);
		REAP1POLICY_API ReAP1Policy::REAP1STATE getStateInstance(std::vector<int> pQueues, int iPhase, int rGreen);
		
		//private:

		///* ---------------------------------------------------------------------
		//* State variables
		//* --------------------------------------------------------------------- */

	};
}

#endif