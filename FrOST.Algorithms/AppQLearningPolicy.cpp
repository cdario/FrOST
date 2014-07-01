#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include "AppQLearningPolicy.h"
#include <algorithm>
#include <iomanip>
#include <random>
#include <float.h>


#ifdef WIN32
#include <Windows.h>
#else
#include <sys/time.h>
#include <ctime>
#endif

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

		runId = "";
		loadApproxParameters();

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
		approxParameters.at(action).at(kParam) = newParamValue;
		storeApproxParameters();
	}

	void AppQLearningPolicy::setApproxFeature(int kParam, double newFeatureValue){
		approxFeatures.at(kParam) = newFeatureValue;
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

	bool AppQLearningPolicy::compareStates(AppQLearningPolicy::AppQLearningSTATE a, AppQLearningPolicy::AppQLearningSTATE b)	
	{
		bool flag = true;
		for (unsigned i = 0; i < a.queueLengths.size(); ++i)
		{
			flag = flag & (a.queueLengths.at(i) == b.queueLengths.at(i));
		}

		flag = flag & (a.phaseIndex == b.phaseIndex) & (a.greenRemaining == b.greenRemaining);

		return flag;
	}

	// append parameters for each step in this run, and keep latest run 
	void AppQLearningPolicy::storeApproxParameters()
	{
		if(runId == "")
			runId = GetTimeMs64();

		string fileName = "c:\\PARAMICS\\runs\\approx-parameters-run-";
		fileName.append(runId);
		fileName.append(".txt");

		//All output operations happen at the end of the file, appending to its existing contents.
		ofstream foutAll;
		foutAll.open(fileName.c_str(),ios::app);

		//ofstream foutAll("c:\\PARAMICS\\approx-parameters-run.txt",ios::app);
		//Any contents that existed in the file before it is open are discarded.
		ofstream foutOne("c:\\PARAMICS\\approx-parameters-last-run.txt",ios::trunc);	

		if (!foutAll && !foutOne)	
			cout << "could not open file";
		else
		{
			for (unsigned action = 0; action < approxParameters.size(); ++action)
			{
				for (unsigned feature = 0; feature < approxParameters[action].size(); ++feature)
				{
					foutAll <<approxParameters[action][feature]<<"\t";
					foutOne <<approxParameters[action][feature]<<"\t";
				}
				foutAll <<"\n";
				foutOne <<"\n";
			}
		}

		if(foutAll){
			foutAll<<"\r\n";	// multiple saves
			foutAll.close();
		}
		if(foutOne){
			foutOne.close();
		}
	}

	// load last experimental set after reloading the network
	void AppQLearningPolicy::storeLastRunApproxParameters()
	{
		//appending last run to history
		ofstream foutAll("c:\\PARAMICS\\approx-parameters-run-history.txt",ios::app);	

		if (!foutAll)	
			cout << "could not open file";
		else
		{
			for (unsigned action = 0; action < approxParameters.size(); ++action)
			{
				for (unsigned feature = 0; feature < approxParameters[action].size(); ++feature)
				{
					foutAll <<approxParameters[action][feature]<<"\t";
				}
				foutAll <<"\n";
			}
		}

		if(foutAll){
			foutAll<<"\r\n";	// multiple saves
			foutAll.close();
		}
	}

	// read parameters from last experiment
	void AppQLearningPolicy::loadApproxParameters()
	{	
		string actionRecord;
		ifstream fin("c:\\PARAMICS\\approx-parameters-last-run.txt",ios::in);

		if (!fin)	
			cout << "could not open file";
		else
		{
			double paramValue;

			for (unsigned action = 0; action < approxParameters.size(); ++action)
			{

				getline(fin, actionRecord);	// use getline to manage tabs
				std::vector<std::string> record = split(actionRecord, '\t');	

				for (unsigned feature = 0; feature < approxParameters[action].size(); ++feature)
				{
					std::stringstream ss;
				    ss << record.at(feature);
					ss >> paramValue;
					
					approxParameters[action][feature] = paramValue;
				}
			}
		}

		if(fin){
			fin.close();
		}
		storeLastRunApproxParameters();

	}

	std::vector<std::string> &AppQLearningPolicy::split(const std::string &s, char delim, std::vector<std::string> &elems) {
		std::stringstream ss(s);
		std::string item;
		while (std::getline(ss, item, delim)) {
			elems.push_back(item);
		}
		return elems;
	}

	std::vector<std::string> AppQLearningPolicy::split(const std::string &s, char delim) {
		std::vector<std::string> elems;
		split(s, delim, elems);
		return elems;
	}
	/* Returns the amount of milliseconds elapsed since the UNIX epoch. Works on both
	* windows and linux. */

	
	std::string AppQLearningPolicy::GetTimeMs64()
	{
	#ifdef WIN32
		/* Windows */
		FILETIME ft;
		LARGE_INTEGER li;

		/* Get the amount of 100 nano seconds intervals elapsed since January 1, 1601 (UTC) and copy it
		* to a LARGE_INTEGER structure. */
		GetSystemTimeAsFileTime(&ft);
		li.LowPart = ft.dwLowDateTime;
		li.HighPart = ft.dwHighDateTime;

		uint64 ret = li.QuadPart;
		ret -= 116444736000000000LL; /* Convert from file time to UNIX epoch time. */
		ret /= 10000; /* From 100 nano seconds (10^-7) to 1 millisecond (10^-3) intervals */

		string Result;          // string which will contain the result
		ostringstream convert;   // stream used for the conversion
		convert << ret;      // insert the textual representation of 'Number' in the characters in the stream
		Result = convert.str();
		return Result;
	#else
		/* Linux */
		struct timeval tv;

		gettimeofday(&tv, NULL);

		uint64 ret = tv.tv_usec;
		/* Convert from micro seconds (10^-6) to milliseconds (10^-3) */
		ret /= 1000;

		/* Adds the seconds (10^0) after converting them to milliseconds (10^-3) */
		ret += (tv.tv_sec * 1000);

		string Result;          // string which will contain the result
		ostringstream convert;   // stream used for the conversion
		convert << ret;      // insert the textual representation of 'Number' in the characters in the stream
		Result = convert.str();
		return Result;
		
	#endif
	}

}

