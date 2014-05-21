#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include "AppQLearningPolicy.h"
#include <algorithm>
#include <iomanip>
#include <random>
#include <float.h>

namespace std {
	#include <cstdlib>
};
#include <vector>

using namespace std;

namespace APPQL{

	AppQLearningPolicy::AppQLearningPolicy(){
		
		//initQValues(0.0000000000000000001 * rand());	//TODO: remove
		std::vector<int> queueSt;
		queueSt.clear();queueSt.push_back(0);queueSt.push_back(0);queueSt.push_back(0);
		tState = getStateInstance(queueSt, 0, 0);	// TODO: improve initial state
		nStates = Q.size();
		nActions = 3;
		
		nFeatures = 5;	// 3 max queue lengths + phase index + green remaining
		approxFeatures = new std::vector(nFeatures; 0.0);	
		approxParameters.resize(nActions);
		for (int ip=0; ip < approxParameters.size(); ip++)
		{
			approxParameters(ip).resize(nFeatures);
		}
	}

/* ---------------------------------------------------------------------
* learning & logic
* --------------------------------------------------------------------- */
	
	AppQLearningPolicy::AppQLearningSTATE AppQLearningPolicy::getStateInstance(std::vector<int> pQueues, int iPhase, int rGreen)
	{
		AppQLearningPolicy::AppQLearningSTATE state;
		state.queueLengths.swap(pQueues);	// TODO: test this swap
		state.phaseIndex = iPhase;
		state.greenRemaining = rGreen;
		return state;
	}
	
	AppQLearningPolicy::AppQLearningSTATE AppQLearningPolicy::setState(AppQLearningPolicy::AppQLearningSTATE state)
	{
		tState = state;
		return tState;
	}
	// TODO: make this function faster, slows down DLL load
	void AppQLearningPolicy::initQValues(double iValue){
		
		return; //TODO: remove

/**
 * 		queue per phase 	phase 		elapsed green (x5)
 * 		10^3 {10*10*10} * 3 {1-3} * 11 {5-55}					= 33000 states
 *
 *		approach state
 * 		16 {4-4} * 3 {1-3} * 11 {5-55}					= 528 states
 * 		
 */

		// TODO: try without initialisating- When updating set rnd value and add key-value pair
		for (int cph=0; cph<3; cph++)	/*	build all possible state representations	*/
		{
			for (int gr=0; gr<=55; gr++)
			{
				for (int qu0=0; qu0<=10; qu0++)		/*	queue lengths combinations per phase */
				{
					for (int qu1=0; qu1<=10; qu1++)
					{
						for (int qu2=0; qu2<=10; qu2++)
						{
							std::vector<int> queueSt;
							//std::vector<int> queueSt;
							queueSt.push_back(qu0);queueSt.push_back(qu1);queueSt.push_back(qu2);
							AppQLearningPolicy::AppQLearningSTATE nstate;
							nstate.queueLengths.swap(queueSt);	// TODO: test this swap
							nstate.phaseIndex = cph;
							nstate.greenRemaining = gr;

							//AppQLearningSTATE nState = getStateInstance(queueSt, gr, cph);
																					
							//std::vector<double> qValues;	/*	add to Q values	; number of actions = 3 */
							//qValues.push_back(iValue);qValues.push_back(iValue);qValues.push_back(iValue);
							AppQLearningValues qValues;
							qValues.qValue1 = iValue;
							qValues.qValue2 = iValue;
							qValues.qValue3 = iValue;

							Q[nstate] = qValues;
							//Q.insert(std::make_pair(nstate,qValues));
						}
					}
				}	/*------------	end queue loops	*/
			}
		}
		//queueSt.clear();queueSt.push_back(0);queueSt.push_back(0);queueSt.push_back(0);
		//tState = getNextState(getStateInstance(queueSt, 0, 0));
	}

	std::vector< double> AppQLearningPolicy::getQvalues(AppQLearningSTATE state){
		std::vector< double> newQ;

		for(unsigned action = 0; action < nActions; ac++)
		{
			newQ.push_back(getQvalue(state, action));
		}

		return newQ;
	}

	int AppQLearningPolicy::printQs(){
	
		//std::string fname = "Qvalues.txt";


		////int WriteFile(std::string fname, std::map<std::string, std::string> *m) {
		//	int count = 0;
		//	if (Q.empty())
		//		return 0;
 
		//	FILE *fp = fopen(fname.c_str(), "w");
		//	if (!fp)
		//		return -errno;
  		
		//	typedef std::map<AppQLearningSTATE, AppQLearningValues>::iterator reapIT;

		//	for (int i=0; i < Q.size(); i++)
		//	{
		//		Q
		//	
		//	}


		//	for(reapIT it = Q.begin(); it != Q.end(); it++) {
		//		// iterator->first = key
		//		// iterator->second = value
		//		// Repeat if you also want to iterate through the second map.
		//	}

		//	for(std::map<AppQLearningSTATE, AppQLearningValues>::iterator it = Q.begin(); it != Q.end(); it++) {
		//		
		//		it
		//		fprintf(fp, "%s=%s\n", it->first.c_str(), it->second.c_str());
		//		count++;
		//	}
  		//   
		//	fclose(fp);
		//	return count;
		//}
	
		return 0;
	}

	void AppQLearningPolicy::setQvalue(AppQLearningSTATE state, int action, double newQ){
		
		// TODO: UPDATE THETAS for Q-APPROXIMATORS
		/**
		 * 		Update approximator parameters theta
		 * 		Update approximator feature based on state
		 */
		
		for (int i = 0; i < nFeatures; ++i)
		{

			approxParameters[action][i] += alpha()

			approxParameters[action][0] = 
			approxParameters[action][1]
			approxParameters[action][2]
			approxParameters[action][3]
			approxParameters[action][4]
		}

		approxFeatures[0] = state.queueLengths[0];
		approxFeatures[1] = state.queueLengths[1];
		approxFeatures[2] = state.queueLengths[2];
		approxFeatures[3] = state.phaseIndex;
		approxFeatures[4] = state.greenRemaining;





		switch (action)
		{
			case 0: Q[state].qValue1 = newQ;	// TODO: Set a distinct Q-value based on each action...
						break;
			case 1: Q[state].qValue2 = newQ;
						break;
			case 2: Q[state].qValue3 = newQ;
						break;
		}
		
	}

	/**
	 * Updated to FUNCTION APPROXIMATION Q(s,a) = SUM( theta(a,n)*f(n) ) based on Q-APPROX from current Thetas
	 * @param  state
	 * @param  action
	 * @return	Q-value
	 */
	double AppQLearningPolicy::getQvalue(AppQLearningSTATE state, int action){

		double q_value = 0;

		for (int n=0; n <= NUM_FEATURES; n++)
		{
			q_value += approxParameters[action][n] * approxFeatures[n];
		}

		return q_value;	

		// switch (action)
		// {
		// 	case 0: return Q[state].qValue1; break;
		// 	case 1: return Q[state].qValue2; break;
		// 	case 2: return Q[state].qValue3; break;
		// 	default: return 0.0; break;
		// }
	}

	double AppQLearningPolicy::getMaxQvalue(AppQLearningSTATE state){
		
		return 0.001;//TODO: remove
		double maxQ = -DBL_MAX;
		std::vector< double> qValues = getQvalues(state);
		for (unsigned int a= 0; a< qValues.size(); a++)
		{
			if (qValues[a] > maxQ)
				maxQ = qValues[a];
		}
		return maxQ;
	}
}