#include <windows.h>
#include <jni.h>      

class Environment {

public:
	TCHAR* dllpath;
	static JavaVM* Environment::jvm;	// throwing exception if using this
	
	JavaVM* startJVM();
	int startPlatform(JavaVM* jvm);

	int flag;

};