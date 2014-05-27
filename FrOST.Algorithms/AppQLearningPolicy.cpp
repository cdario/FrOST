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
		approxFeatures.resize(nFeatures);
		approxParameters.resize(nActions);
		for (unsigned ip=0; ip < approxParameters.size(); ip++)
		{
			approxParameters[ip].resize(nFeatures);
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
	
	void AppQLearningPolicy::updateApproxFeatures(AppQLearningPolicy::AppQLearningSTATE state)
	{
		//map state to features in the approximator
		approxFeatures[0] = state.queueLengths[0];
		approxFeatures[1] = state.queueLengths[1];
		approxFeatures[2] = state.queueLengths[2];
		approxFeatures[3] = state.phaseIndex;
		approxFeatures[4] = state.greenRemaining;		
		tState = state;
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

		for(int a = 0; a < nActions; ++a)
		{
			newQ.push_back(getQvalue(state, a));
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
	void AppQLearningPolicy::setApproxParam(int action, int kParam, double newParamValue){
		approxParameters[action][kParam] = newParamValue;
	}

	void AppQLearningPolicy::setApproxFeature(int kParam, double newFeatureValue){
		approxFeatures[kParam] = newFeatureValue;
	}

	/**
	 * for linear function approximation  based on Q-APPROX from current thetas
	 * approx_params changes affect the value of the following estimations (generalisation)
	 * @param  state 	not used- encoded in approxFeatues
	 * @param  action
	 * @return	Q-value
	 */
	double AppQLearningPolicy::getQvalue(AppQLearningSTATE state, int action){

		double q_value = 0;

		for (int feat=0; feat < nFeatures; feat++)
		{
			q_value += getApproxParameter(action,feat) * getApproxFeature(feat); /*	Q(s,a) = SUM_n( theta(a,n)*f(n) )	*/


			//TODO: print and check convergence?
		}

		return q_value;	
	}

	double AppQLearningPolicy::getMaxQvalue(AppQLearningSTATE state){
		
		double maxQ = -DBL_MAX;
		std::vector< double> qValues = getQvalues(state);
		for (unsigned int a= 0; a < qValues.size(); a++)
		{
			if (qValues[a] > maxQ)
				maxQ = qValues[a];
		}
		return maxQ;
	}

	double AppQLearningPolicy::getApproxFeature(int feature)
	{

		return approxFeatures[feature];
	}

	double AppQLearningPolicy::getApproxParameter(int action, int feature)	
	{
		return approxParameters[action][feature];
	}
}