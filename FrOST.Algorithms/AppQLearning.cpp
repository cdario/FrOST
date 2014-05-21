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
#include "AppQLearning.h"
#include "AppQLearningPolicy.h"

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

namespace APPQL{

/* ---------------------------------------------------------------------
 * access
 * --------------------------------------------------------------------- */

	void AppQLearning::setOutput(bool option){
		output = option;
	}

	void AppQLearning::setInitialPhase(int ph){
		idxCurrentPh = initialPhase = ph;
	}
	
	void  AppQLearning::setSaturationFlow(int phi, float satFlow) {
		satFlows[phi] = satFlow;
	}

	void  AppQLearning::setLanePhases(int phi, int lanes) {
		lanePhases[phi] = lanes;
	}
	
	void AppQLearning::setStartupLostTime(float time){
		startupLostTime = time;
	}
		
	void AppQLearning::setMinGreenTime(int mgreen){
		mingreen = mgreen;
	}
	
	void AppQLearning::setMaxGreenTime(int mxgreen){
		maxgreen = mxgreen;
	}

	void AppQLearning::setRedTime(int rd){
		red= rd;
	}

	void AppQLearning::setHorizon(int h){
		T= h;
		resizeArrivals();
	}
	
	void AppQLearning::setMaxPhCompute(int mp){
		M= mp;
	}

	void AppQLearning::setArrivals(std::vector<std::vector<int> > arrivals){
		arrivalData = arrivals;
	};

	int AppQLearning::getArrivals(int a, int b, int phi) {

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
		
	float  AppQLearning::getSaturationFlow(int phi) { // sat-flow rate per phase, not per lane. in vehicles per sec
		// NOTE: Simplicity assumption return 0;
		float sf = satFlows[phi];
		if (sf <0) return 0;
		
		return (satFlows[phi]/3600)*lanePhases[phi]; // in vphpl, to vpspl
	}

	std::vector<int> AppQLearning::getOptimalControl(){
		return optControlSequence;
	}; 

	int AppQLearning::getInitialPhase(){
		return initialPhase;
	}

	int AppQLearning::getRed(){
		return red;
	}

/* ---------------------------------------------------------------------
 * initialisation
 * --------------------------------------------------------------------- */

	void AppQLearning::initParameters(){
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

	void AppQLearning::resizeArrivals()
	{
		arrivalData.resize(T);
		for (unsigned int i = 0; i < T; ++i) {
			arrivalData[i].resize(phases.size());
		}
	}

	void  AppQLearning::initMatrices(int init) {
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

	AppQLearning::AppQLearning(char* file, int iphase){
		initParameters();
		if(!loadFromFile(file))
			exit(0);
		idxCurrentPh = initialPhase = iphase;
	};

	AppQLearning::AppQLearning(char* file, int iphase, int horizon){
		initParameters();
		setHorizon(horizon);
		if(!loadFromFile(file))
			exit(0);
		idxCurrentPh = initialPhase = iphase;
	};

	AppQLearning::AppQLearning(char* data, int size, int nphases, int iphase){
		initParameters();
		if(!loadFromSeq(data, size, nphases))
			exit(0);
		idxCurrentPh = initialPhase = iphase;
	};

	AppQLearning::AppQLearning(std::vector<int> data, int nphases, int iphase){
		initParameters();
		if(!loadFromVector(data, nphases))
			exit(0);
		idxCurrentPh = initialPhase = iphase;
	};

	AppQLearning::AppQLearning(std::vector<std::vector<int> > data, int iphase, int horizon){
		initParameters();
		setHorizon(horizon);
		arrivalData = data;
		idxCurrentPh = initialPhase = iphase;
	};

	AppQLearning::AppQLearning(int iphase, int horizon){
		initParameters();
		setInitialPhase(iphase);
		setHorizon(horizon);
		idxCurrentPh = initialPhase;
	
	};

	AppQLearning::AppQLearning(){

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
		
	bool  AppQLearning::loadFromFile(char* filename) {
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

	bool AppQLearning::loadFromSeq(char* data, unsigned int size, int nPhases) {
		
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

	bool AppQLearning::loadFromVector(std::vector<int> data, int nPhases) {
		
		for (unsigned int ix = 0; ix < data.size(); ++ix)
		{
			arrivalData[ix/nPhases][ix%nPhases] = data[ix];
		}
		return true;
	}

	void  AppQLearning::printArrivals() {
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

	void  AppQLearning::printVector(vector<int> values) {

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

	void  AppQLearning::printMatrix(vector<vector<int> > values) {

		for (unsigned int i = 0; i < values.size(); ++i) {
			printVector(values[i]);
			cout << endl;
		}
	}

	vector<int>  AppQLearning::printSequence(int arry[], int sz) {

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
	
	void AppQLearning::setInitialState(APPQL::AppQLearningPolicy::AppQLearningSTATE st)
	{
		state = st;
	}

	void AppQLearning::setAlpha(double a){
		if (a >=0 && a< 1)
			alpha = a;
	}

	double AppQLearning::getAlpha(){
		return alpha;
	}

	void AppQLearning::setGamma(double g){
		if (g >=0 && g< 1)
			gamma = g;
	}

	double AppQLearning::getGamma(){
		return gamma;
	}

	void AppQLearning::setEpsilon(double e){
		if (e >=0 && e < 1)
			epsilon = e;
	}

	double AppQLearning::getEpsilon(){
		return epsilon;
	}
	
	APPQL::AppQLearningPolicy AppQLearning::getPolicy(){
		return policy;
	}
	
	bool AppQLearning::getRandomFlag(){
		return random;
	}

	bool AppQLearning::validAction(int action){
		if (action >= 0 && action <3)	//TODO: improve
			return true;
		else
			return false;
	}
	
	void AppQLearning::initPolicy(){
		policy = APPQL::AppQLearningPolicy();
	}

	//5
	/*	 update state and select action	based on e-greedy	*/
	/*	 invoked by the controller	*/
	int AppQLearning::selectAction(APPQL::AppQLearningPolicy::AppQLearningSTATE iState){		
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
	void AppQLearning::setNewState(APPQL::AppQLearningPolicy::AppQLearningSTATE iState){
		newState = iState;
	}

	//6.2
	/*	after applying action, update state in the agent	*/
	/*	 invoked by the controller	*/
	void AppQLearning::setNewReward(double oReward){
		reward = oReward;
	}

	//6
	/*	after applying action, update state in the agent	*/
	/*	 invoked by the controller	*/
	void AppQLearning::setNewStateReward(APPQL::AppQLearningPolicy::AppQLearningSTATE iState, double oReward){
		newState = iState;
		reward = oReward;
	}

	//7
	void AppQLearning::updateQ(){
		
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
		
		/**
		 * NEW! compute and set ApproxParameters()
		 */
		 for (int param = 0; param < policy.nParameters; ++param)
		 {
		 	// TODO: implement partial derivative?
		 	newParam = oldParam + alpha*(reward + gamma*maxQ - tQ)*partialD(tQ, oldParam);
		 	policy.setApproxParam(action, param, newValue);	
		 }

		policy.setQvalue(state, action, nQ);		/* 	update Q-table 	*/

	}

	//8
	/*	update state in agent*/
	/*	 invoked by the controller	*/
	void AppQLearning::updateState(){
		state = newState;
	}
	
	vector<int> AppQLearning::Run() {

		std::vector<int> optControlSequence;

		optControlSequence.push_back(55);
		optControlSequence.push_back(55);
		optControlSequence.push_back(55);
		optControlSequence.push_back(55);
		optControlSequence.push_back(55);

		cout << "Q-Learner started...\n";
		//selectNextAction();
		//updateState(NULL);
		//updateReward();

		cout << "\nOptimal Control Sequence: \n\n";
		cout << "\n\n...Q-Learner ended\n\n";
		return optControlSequence;

	}; 
}