#include <windows.h>
#include <jni.h>      

class Environment {

public:
	TCHAR* dllpath;
	//static JavaVM* Environment::jvm;	// throwing exception if using this
	static jclass platformClass;
	static jmethodID startJadeMethodID;
	static jmethodID addJunctionMethodID;
	static jmethodID updJunctionMethodID;

	JavaVM* startJVM();
	int startPlatform(JavaVM* jvm, char* junctions);
	int initJunction(int id);
	int updateJunctions(JavaVM* jvm, char* junctions, char* newValues);
	HMODULE jniModule;
	void close();

	Environment();

};