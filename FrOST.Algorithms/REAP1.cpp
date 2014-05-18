//************************************************

// This is the main DLL file.
//#include "stdafx.h"
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <random>
#include <algorithm>

#include <iomanip>
#include <float.h>
#include <ctime>
#include <cstdlib>
#include "REAP1.h"
#include "REAP1Policy.h"

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

/* ---------------------------------------------------------------------
 * access
 * --------------------------------------------------------------------- */

	void ReAP1::setOutput(bool option){
		output = option;
	}

	void ReAP1::setInitialPhase(int ph){
		idxCurrentPh = initialPhase = ph;
	}
	
	void  ReAP1::setSaturationFlow(int phi, float satFlow) {
		satFlows[phi] = satFlow;
	}

	void  ReAP1::setLanePhases(int phi, int lanes) {
		lanePhases[phi] = lanes;
	}
	
	void ReAP1::setStartupLostTime(float time){
		startupLostTime = time;
	}
		
	void ReAP1::setMinGreenTime(int mgreen){
		mingreen = mgreen;
	}
	
	void ReAP1::setMaxGreenTime(int mxgreen){
		maxgreen = mxgreen;
	}

	void ReAP1::setRedTime(int rd){
		red= rd;
	}

	void ReAP1::setHorizon(int h){
		T= h;
		resizeArrivals();
	}
	
	void ReAP1::setMaxPhCompute(int mp){
		M= mp;
	}

	void ReAP1::setArrivals(std::vector<std::vector<int> > arrivals){
		arrivalData = arrivals;
	};

	int ReAP1::getArrivals(int a, int b, int phi) {

		if (a == b)
			return 0;

		int vehicles = 0;
		int cc = a;

		do {
			vehicles += arrivalData[cc][phi];
			cc++;
		} while (cc < b); // for [a,b), with a!=b

		return vehicles;
	}
		
	float  ReAP1::getSaturationFlow(int phi) { // sat-flow rate per phase, not per lane. in vehicles per sec
		// NOTE: Simplicity assumption return 0;
		float sf = satFlows[phi];
		if (sf <0) return 0;
		
		return (satFlows[phi]/3600)*lanePhases[phi]; // in vphpl, to vpspl
	}

	std::vector<int> ReAP1::getOptimalControl(){
		return optControlSequence;
	}; 

	int ReAP1::getInitialPhase(){
		return initialPhase;
	}

	int ReAP1::getRed(){
		return red;
	}

/* ---------------------------------------------------------------------
 * initialisation
 * --------------------------------------------------------------------- */

	void ReAP1::initParameters(){
		PI = DELAY;
		red = 1; //1
		mingreen = 2; //2
		maxgreen = 50;
		startupLostTime = 0;
		T = 10; //planning horizon
		M = 9; //maximum number of phases to compute (1 to M-1)
		phaseSeq[0]= 'A';phaseSeq[1]= 'B';phaseSeq[2]= 'C';
		setSaturationFlow(0, -1.0); setSaturationFlow(1, -1.0); setSaturationFlow(2, -1.0);
		phases = std::vector<int>(phaseSeq, phaseSeq + sizeof (phaseSeq) /sizeof (phaseSeq[0]));
		output = false;
		resizeArrivals();
	}

	void ReAP1::resizeArrivals()
	{
		arrivalData.resize(T);
		for (unsigned int i = 0; i < T; ++i) {
			arrivalData[i].resize(phases.size());
		}
	}

	void  ReAP1::initMatrices(int init) {
			for (unsigned int i = 0; i < v.size(); ++i) {
				for (unsigned int j = 0; j < v[i].size(); ++j) {
					if (i == 0) {
						x_star[i][j] = v[i][j] = 0;

					} else {
						x_star[i][j] = v[i][j] = init;
					}
				}
			}
		}
		
/* ---------------------------------------------------------------------
 * constructors
 * --------------------------------------------------------------------- */

	ReAP1::ReAP1(char* file, int iphase){
		initParameters();
		if(!loadFromFile(file))
			exit(0);
		idxCurrentPh = initialPhase = iphase;
	};

	ReAP1::ReAP1(char* file, int iphase, int horizon){
		initParameters();
		setHorizon(horizon);
		if(!loadFromFile(file))
			exit(0);
		idxCurrentPh = initialPhase = iphase;
	};

	ReAP1::ReAP1(char* data, int size, int nphases, int iphase){
		initParameters();
		if(!loadFromSeq(data, size, nphases))
			exit(0);
		idxCurrentPh = initialPhase = iphase;
	};

	ReAP1::ReAP1(std::vector<int> data, int nphases, int iphase){
		initParameters();
		if(!loadFromVector(data, nphases))
			exit(0);
		idxCurrentPh = initialPhase = iphase;
	};

	ReAP1::ReAP1(std::vector<std::vector<int> > data, int iphase, int horizon){
		initParameters();
		setHorizon(horizon);
		arrivalData = data;
		idxCurrentPh = initialPhase = iphase;
	};

	ReAP1::ReAP1(int iphase, int horizon){
		initParameters();
		setInitialPhase(iphase);
		setHorizon(horizon);
		idxCurrentPh = initialPhase;
	
	};

	ReAP1::ReAP1(){

		initPolicy();
    	action = 0;		
		// set default values
		epsilon = 0.1;
		temp = 1;

		alpha = 1; 
		gamma = 0.1;
		lambda = 0.1;  

		random = false;

		/* ------Agent initialised -----*/
	}

/* ---------------------------------------------------------------------
 * in/out
 * --------------------------------------------------------------------- */
		
	bool  ReAP1::loadFromFile(char* filename) {
		unsigned int x, y;
		ifstream in(filename);

		if (!in) {
			cout << "Cannot open file.\n";
			return false;
		}

		for (y = 0; y < T; y++) {
			for (x = 0; x < phases.size(); x++) {

				in >> arrivalData[y][x];
			}
		}
		in.close();

		return true;
	};

	bool ReAP1::loadFromSeq(char* data, unsigned int size, int nPhases) {
		
		//ifstream in(filename);
		int ic = 0;
		int dataI;
		for (unsigned int ix = 0; ix < size; ++ix)
		{
			dataI = data[ix];
			dataI -= 48; //0 = 48, 1 = 49

			// TODO: works only for 1 digit data
			if(dataI >= 0) // skip spaces 
			{
				arrivalData[ic/nPhases][ic%nPhases] = dataI; 
				ic++;
			}
		}

		return true;
	}

	bool ReAP1::loadFromVector(std::vector<int> data, int nPhases) {
		
		for (unsigned int ix = 0; ix < data.size(); ++ix)
		{
			arrivalData[ix/nPhases][ix%nPhases] = data[ix];
		}
		return true;
	}

	void  ReAP1::printArrivals() {
		cout << "\n\n"; 
		for (unsigned int p=0; p < phases.size(); ++p)
		{
			cout << "\t" << phaseSeq[p];
		}
		cout << "\n\n";
		for (unsigned int i = 0; i < T; ++i) {
			cout << i+1 << "\t";
			for (unsigned int j = 0; j < phases.size(); ++j) {
				cout << arrivalData[i][j] << "\t";
			}
			cout << endl;
		}
		cout << endl;
	}

	void  ReAP1::printVector(vector<int> values) {

		cout << flush << "[  ";
		for (vector<int>::iterator i = values.begin(); i != values.end(); ++i) {
			if (i!= values.begin())
				cout << setfill (' ' ) << setw (3);

			int ix = *i;
			if (ix < 0)
				cout << "-";
			else
				cout <<ix;
		}
		cout << "  ]";
	}

	void  ReAP1::printMatrix(vector<vector<int> > values) {

		for (unsigned int i = 0; i < values.size(); ++i) {
			printVector(values[i]);
			cout << endl;
		}
	}

	vector<int>  ReAP1::printSequence(int arry[], int sz) {

		vector<int> seq;
		cout << "[ ";
		for (int i = 0; i < sz; i++) {
			seq.push_back(arry[i]);
			cout << phaseSeq[(i+initialPhase)%phases.size()]<<":"<< arry[i] << " ";
		}
		cout << "]";

		return seq;
	}

/* ---------------------------------------------------------------------
 * learning 
 * --------------------------------------------------------------------- */
	
	void ReAP1::setInitialState(REAP1::ReAP1Policy::REAP1STATE st)
	{
		state = st;
	}

	void ReAP1::setAlpha(double a){
		if (a >=0 && a< 1)
			alpha = a;
	}

	double ReAP1::getAlpha(){
		return alpha;
	}

	void ReAP1::setGamma(double g){
		if (g >=0 && g< 1)
			gamma = g;
	}

	double ReAP1::getGamma(){
		return gamma;
	}

	void ReAP1::setEpsilon(double e){
		if (e >=0 && e < 1)
			epsilon = e;
	}

	double ReAP1::getEpsilon(){
		return epsilon;
	}
	
	REAP1::ReAP1Policy ReAP1::getPolicy(){
		return policy;
	}
	
	bool ReAP1::getRandomFlag(){
		return random;
	}

	bool ReAP1::validAction(int action){
		if (action >= 0 && action <3)	//TODO: improve
			return true;
		else
			return false;
	}
	
	void ReAP1::initPolicy(){
		policy = REAP1::ReAP1Policy();
	}

	//5
	/*	 update state and select action	based on e-greedy	*/
	/*	 invoked by the controller	*/
	int ReAP1::selectAction(REAP1::ReAP1Policy::REAP1STATE iState){		
		std::vector<double> qVals = policy.getQvalues(iState);		//TODO: check for vector iterator incompatibility
		int sAction = -1;

		// based on e-greedy
		random = false;
		double maxQ = -DBL_MAX;
		std::vector<int> dblVals;
		dblVals.resize(qVals.size());
		int maxdv = 0;

		//	double udr = ((double) rand() / (RAND_MAX+1));	//OLD

		
		std::uniform_real_distribution<> realDist(0,1);
		std::uniform_int_distribution<> intDist(0,	qVals.size());
		

		double udr = realDist(eng);

		if(udr < epsilon)		/*	exploring	*/
		{	
			sAction = -1;			// TODO: check use
			random = true;
		}
		else{
			for(unsigned int ac= 0; ac < qVals.size(); ac++)
			{
				if(qVals[ac] > maxQ){		/*	exploiting	*/
					sAction = action;		/*	iterate between qVals and pick maximum	*/
					maxQ = qVals[action];
					maxdv = 0;
					dblVals[maxdv] = sAction;
				}
				else
				{ 
					if(qVals[ac] == maxQ){
						maxdv++;
						dblVals[maxdv] = action;
					}
				}
			}

			if(maxdv > 0){
				//int rndIndex = (int) (rand()*(maxdv + 1));
				std::uniform_int_distribution<> intDist2(0,	maxdv);
				int rndIndex =	intDist2(eng);	//rand() % (maxdv+1); // 0 to (maxdv)		//NEW 
				sAction = dblVals[rndIndex];
			}
		}

		if(sAction == -1){		/*	random action iff qvals = 0 or exploring	*/
			sAction = 	intDist(eng);				//(int) (rand()%qVals.size());
		}

		while (!validAction(sAction)){
			sAction = 	intDist(eng);	//(int) (rand()%qVals.size());
		}
	
	
		//update agent's action
		action = sAction;
		return action;
	}

	//6.1
	/*	after applying action, update state in the agent	*/
	/*	 invoked by the controller	*/
	void ReAP1::setNewState(REAP1::ReAP1Policy::REAP1STATE iState){
		newState = iState;
	}

	//6.2
	/*	after applying action, update state in the agent	*/
	/*	 invoked by the controller	*/
	void ReAP1::setNewReward(double oReward){
		reward = oReward;
	}

	//6
	/*	after applying action, update state in the agent	*/
	/*	 invoked by the controller	*/
	void ReAP1::setNewStateReward(REAP1::ReAP1Policy::REAP1STATE iState, double oReward){
		newState = iState;
		reward = oReward;
	}

	//7
	void ReAP1::updateQ(){
		
		double tQ;		// Q-learning
		double maxQ;
		double nQ;

		//action = selectAction(state);
		//newState = Controller.getNextState(action);
		//reward = Controller.getReward();

		tQ = policy.getQvalue(state, action);
		maxQ = policy.getMaxQvalue(newState);
		// update rule for the Q-function, based on Bellman
		nQ = tQ + alpha * (reward + gamma * maxQ - tQ);		/*	compute new Q */
		policy.setQvalue(state, action, nQ);		/* update Q-table*/

	}

	//8
	/*	update state in agent*/
	/*	 invoked by the controller	*/
	void ReAP1::updateState(){
		state = newState;
	}
	
	vector<int> ReAP1::RunREAP() {

		std::vector<int> optControlSequence;

		optControlSequence.push_back(55);
		optControlSequence.push_back(55);
		optControlSequence.push_back(55);
		optControlSequence.push_back(55);
		optControlSequence.push_back(55);

		cout << "REAP started...\n";
		//selectNextAction();
		//updateState(NULL);
		//updateReward();

		cout << "\nOptimal Control Sequence: \n\n";
		cout << "\n\n...REAP ended\n\n";
		return optControlSequence;

	}; 
}