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
	int startPlatform(JavaVM* jvm);
	int initJunction(int id);
	HMODULE jniModule;
	void close();

	Environment();

};