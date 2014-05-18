//************************************************

// This is the main DLL file.
//#include "stdafx.h"
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include "REAP1Policy.h"
#include <algorithm>
#include <iomanip>
#include <random>
#include <float.h>

/*
MS bug and workaround: use std::vector  http://support.microsoft.com/kb/243444
MUST include <vector>
*/

// Compile Options:  /GX
namespace std {
	#include <cstdlib>
};
#include <vector>

using namespace std;

namespace REAP1{

	ReAP1Policy::ReAP1Policy(){
		
		//initQValues(0.0000000000000000001 * rand());	//TODO: remove
		std::vector<int> queueSt;
		queueSt.clear();queueSt.push_back(0);queueSt.push_back(0);queueSt.push_back(0);
		tState = getStateInstance(queueSt, 0, 0);	// TODO: improve initial state
		nStates = Q.size();
		nActions = 3;
	}

/* ---------------------------------------------------------------------
* learning & logic
* --------------------------------------------------------------------- */
	
	ReAP1Policy::REAP1STATE ReAP1Policy::getStateInstance(std::vector<int> pQueues, int iPhase, int rGreen)
	{
		ReAP1Policy::REAP1STATE state;
		state.queueLengths.swap(pQueues);	// TODO: test this swap
		state.phaseIndex = iPhase;
		state.greenRemaining = rGreen;
		return state;
	}
	
	ReAP1Policy::REAP1STATE ReAP1Policy::setState(ReAP1Policy::REAP1STATE state)
	{
		tState = state;
		return tState;
	}
	// TODO: make this function faster, slows down DLL load
	void ReAP1Policy::initQValues(double iValue){
		
		return; //TODO: remove
		// TODO: try without initialisating- When updating set rnd value and add key-value pair
		for (int cph=0; cph<3; cph++)	/*	build all possible state representations	*/
		{
			for (int gr=0; gr<=50; gr++)
			{
				for (int qu0=0; qu0<=10; qu0++)		/*----------	queue lengths combinations per phase */
				{
					for (int qu1=0; qu1<=10; qu1++)
					{
						for (int qu2=0; qu2<=10; qu2++)
						{
							std::vector<int> queueSt;
							//std::vector<int> queueSt;
							queueSt.push_back(qu0);queueSt.push_back(qu1);queueSt.push_back(qu2);
							ReAP1Policy::REAP1STATE nstate;
							nstate.queueLengths.swap(queueSt);	// TODO: test this swap
							nstate.phaseIndex = cph;
							nstate.greenRemaining = gr;

							//REAP1STATE nState = getStateInstance(queueSt, gr, cph);
																					
							//std::vector<double> qValues;	/*	add to Q values	; number of actions = 3 */
							//qValues.push_back(iValue);qValues.push_back(iValue);qValues.push_back(iValue);
							REAP1QVALUES qValues;
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

	std::vector< double> ReAP1Policy::getQvalues(REAP1STATE state){
		std::vector< double> newQ;
		//newQ.push_back(Q[state].qValue3);
		//newQ.push_back(Q[state].qValue2);
		//newQ.push_back(Q[state].qValue1);

		newQ.push_back(0.0);
		newQ.push_back(0.0);
		newQ.push_back(0.0);	//TODO: remove



		//newQ.reserve(Q[state].size());
		//newQ.swap (Q[state]);
		return ( newQ );		//TODO: verify passing by value
	}

	int ReAP1Policy::printQs(){
	
		//std::string fname = "Qvalues.txt";


		////int WriteFile(std::string fname, std::map<std::string, std::string> *m) {
		//	int count = 0;
		//	if (Q.empty())
		//		return 0;
 
		//	FILE *fp = fopen(fname.c_str(), "w");
		//	if (!fp)
		//		return -errno;
  //   

		//	typedef std::map<REAP1STATE, REAP1QVALUES>::iterator reapIT;

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

		//	for(std::map<REAP1STATE, REAP1QVALUES>::iterator it = Q.begin(); it != Q.end(); it++) {
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

	void ReAP1Policy::setQvalue(REAP1STATE state, int action, double newQ){
		return; // TODO: remove
		switch (action)
		{
			case 0: Q[state].qValue1 = newQ;
						break;
			case 1: Q[state].qValue2 = newQ;
						break;
			case 2: Q[state].qValue3 = newQ;
						break;
		}
		//Q[state][action] = newQ;
	}

	double ReAP1Policy::getQvalue(REAP1STATE state, int action){
		//return Q[state][action];
		return 0.0;	//TODO: quick fix

		switch (action)
		{
			case 0: return Q[state].qValue1; break;
			case 1: return Q[state].qValue2; break;
			case 2: return Q[state].qValue3; break;
			default: return 0.0; break;
		}
	}

	double ReAP1Policy::getMaxQvalue(REAP1STATE state){
		
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