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
	std::string updateJunctions(JavaVM* jvm, char* junctions_values);
	HMODULE jniModule;
	void close();

	Environment();

};