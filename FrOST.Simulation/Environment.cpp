#include <windows.h>
#include <jni.h>       /* where everything is defined */
#include <stdio.h>
#include <iostream>
#include "Environment.h"

//class Environment {
//
//public:
//	TCHAR* dllpath;
//	static jclass platformClass;
//
//	JavaVM* startJVM();
//	int startPlatform(JavaVM*);
//	int initJunction(int id);
//	HMODULE jniModule;
//	void close();
//
//	Environment();
//
//};

//Cache Class, Method, Field IDs

jclass Environment::platformClass = NULL;
jmethodID Environment::startJadeMethodID = NULL;
jmethodID Environment::addJunctionMethodID = NULL;
jmethodID Environment::updJunctionMethodID = NULL;

Environment::Environment()
{
	dllpath = NULL;
	platformClass = NULL;

}

int Environment::initJunction(int id)
{
	return id;
}

JavaVM* Environment::startJVM()
{
	/***	Finding the jvm.dll	***/
	DWORD retval;
	HKEY jKey;	// fetch jvm.dll path from registry
	if (retval = RegOpenKeyEx(HKEY_LOCAL_MACHINE, TEXT("SOFTWARE\\JavaSoft\\Java Runtime Environment"), 0, KEY_READ, &jKey))
	{
		RegCloseKey(jKey);
		//manage exception
	}

	TCHAR versionString[16]; // version no. => 16 chars
	DWORD bufsize = 16 * sizeof(TCHAR);
	if (retval = RegGetValue(jKey, NULL, TEXT("CurrentVersion"), RRF_RT_REG_SZ, NULL, versionString, &bufsize))
	{
		RegCloseKey(jKey);
		//manage exception
	}

	dllpath = new TCHAR[512];
	bufsize = 512 * sizeof(TCHAR);
	retval = RegGetValue(jKey, versionString, TEXT("RuntimeLib"), RRF_RT_REG_SZ, NULL, dllpath, &bufsize);
	RegCloseKey(jKey);
	if (retval)
	{
		delete[] dllpath;
		//manage exception
	}

	/***	Loading the jvm.dll and functions GetCreatedJavaVMs & CreateJavaVM ***/

	jniModule = LoadLibraryA(dllpath);

	delete[] dllpath;
	if (jniModule == NULL)
		return NULL;

	typedef int (JNICALL * JNI_CreateJavaVM)(JavaVM** jvm, JNIEnv** env, JavaVMInitArgs* initargs);
	JNI_CreateJavaVM createJavaVM = (JNI_CreateJavaVM)GetProcAddress(jniModule, "JNI_CreateJavaVM");

	/*** Creating the JVM ***/
	JavaVMInitArgs initArgs;

	JavaVMOption* options = new JavaVMOption[4]; 
	options[0].optionString = "-Djava.compiler=NONE";           /* disable JIT */
	options[1].optionString = "-Djava.class.path=C:\\Users\\cesar\\Dropbox\\CODE\\HLEA4TC\\dist\\HLEA4TC.jar";	
	options[2].optionString = "-Djava.library.path=c:\\Program Files\\Paramicsv6";  /* set native library path */
	options[3].optionString = "-verbose:jni";                   /* print JNI-related messages */

	initArgs.version = JNI_VERSION_1_6;
	initArgs.nOptions = 4;
	initArgs.options = options;
	initArgs.ignoreUnrecognized = false;

	JavaVM* jvm;
	JNIEnv* env;

	typedef jint (JNICALL * GetCreatedJavaVMs)(JavaVM**, jsize, jsize*);
	GetCreatedJavaVMs JNI_GetCreatedJavaVMs;

	JNI_GetCreatedJavaVMs = (GetCreatedJavaVMs)GetProcAddress(jniModule, "JNI_GetCreatedJavaVMs");
	
	int n;
	retval = JNI_GetCreatedJavaVMs(&jvm,1, (jsize*) &n);

	if (retval == JNI_OK)
	{
		if (n == 0)
		{
			retval = createJavaVM(&jvm, &env, &initArgs);	//create new
		}else
		{
			if (n != 1)
			{
				return NULL;
			}
		}
	}
	return jvm;
}

int Environment::startPlatform(JavaVM* jvm){

	JNIEnv* env;
	bool mustDetach = false;

	jint retval = jvm->GetEnv((void**)&env, JNI_VERSION_1_6);
	if (retval == JNI_EDETACHED)
	{
		JavaVMAttachArgs args;
		args.version = JNI_VERSION_1_6;
		args.name = NULL;
		args.group = NULL;
		retval = jvm->AttachCurrentThread((void **)&env, &args);
		mustDetach = true; 
	}
	if (retval != JNI_OK)
		throw retval;

	if (retval == JNI_OK)
	{
		static const char* const jvmClassName = "com/cdario/hlea4tc/integration/PlatformMediator";
		jclass clazz =  env->FindClass(jvmClassName);
		platformClass = (jclass)env->NewGlobalRef(clazz);

		if (env->ExceptionCheck())
		{
			env->ExceptionOccurred();
			return JNI_ABORT;
		}
		if (platformClass == NULL){
			return JNI_ABORT;
		}

		//jmethodID mid = env->GetStaticMethodID(platformClass, "startJadePlatform", "()Z");	//determine signature via javap -s
		startJadeMethodID = env->GetStaticMethodID(platformClass, "startJadePlatform", "()Z");//determine signature via javap -s
		addJunctionMethodID = env->GetStaticMethodID(platformClass, "initJunctionAgent", "(Ljava/lang/String;)Z");//determine signature via javap -s
		updJunctionMethodID = env->GetStaticMethodID(platformClass, "updJunctionAgent", "(Ljava/lang/String;I)Z");//determine signature via javap -s

		if (startJadeMethodID == NULL 
			|| addJunctionMethodID == NULL
			|| updJunctionMethodID == NULL
			){
				return JNI_EINVAL;
		}

		BOOL returnedValue = env->CallStaticBooleanMethod(platformClass, startJadeMethodID);

		env->DeleteLocalRef(clazz);	//delete local ref if global OK

		if (mustDetach)
		{
			jvm->DetachCurrentThread();
			jvm->DestroyJavaVM();
		}

		if (returnedValue) return JNI_OK;
	}
	return JNI_ERR;
}

void Environment::close(){
	
	FreeLibrary(jniModule);
	//and release globals
}