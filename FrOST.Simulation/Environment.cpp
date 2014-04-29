#include <windows.h>
#include <jni.h>       /* where everything is defined */
#include <stdio.h>
#include <iostream>

enum STATUS_CODE {
			OK, INCOMPLETE, FAILED, NOT_OK
};


class Environment {

public:
	TCHAR* dllpath;
	//JavaVM* Environment::jvm;	// throwing exception if using this
	JavaVM* startJVM();
	int startPlatform(JavaVM*);
	Environment();

	int flag;
};


Environment::Environment()
{
	dllpath = NULL;
	flag = 0;
	//jvm = NULL;
	//mustDetach = false;
}

JavaVM* Environment::startJVM()
{
	flag = 1;
	/***	Finding the jvm.dll	***/
	DWORD retval;
	HKEY jKey;	// fetch jvm.dll path from registry
	if (retval = RegOpenKeyEx(HKEY_LOCAL_MACHINE, TEXT("SOFTWARE\\JavaSoft\\Java Runtime Environment"), 0, KEY_READ, &jKey))
	{
		RegCloseKey(jKey);
		//throw new   gcnew System::ComponentModel::Win32Exception(retval);	// assuming you're using C++/CLI
	}

	TCHAR versionString[16]; // version numbers shouldn't be longer than 16 chars
	DWORD bufsize = 16 * sizeof(TCHAR);
	if (retval = RegGetValue(jKey, NULL, TEXT("CurrentVersion"), RRF_RT_REG_SZ, NULL, versionString, &bufsize))
	{
		RegCloseKey(jKey);
		//throw gcnew System::ComponentModel::Win32Exception(retval);	// assuming you're using C++/CLI
	}

	dllpath = new TCHAR[512];
	bufsize = 512 * sizeof(TCHAR);
	retval = RegGetValue(jKey, versionString, TEXT("RuntimeLib"), RRF_RT_REG_SZ, NULL, dllpath, &bufsize);
	RegCloseKey(jKey);
	if (retval)
	{
		delete[] dllpath;
		//throw gcnew System::ComponentModel::Win32Exception(retval);	// assuming you're using C++/CLI 
	}

	/***	Loading the jvm.dll and getting the CreateJavaVM function	***/

	HMODULE jniModule = LoadLibraryA(dllpath);

	delete[] dllpath;
	if (jniModule == NULL)
		throw;
		//throw gcnew System::ComponentModel::Win32Exception();

	typedef int (JNICALL * JNI_CreateJavaVM)(JavaVM** jvm, JNIEnv** env, JavaVMInitArgs* initargs);
	JNI_CreateJavaVM createJavaVM = (JNI_CreateJavaVM)GetProcAddress(jniModule, "JNI_CreateJavaVM");

	/*** Creating the JVM ***/
	JavaVMInitArgs initArgs;

	JavaVMOption* options = new JavaVMOption[4]; //holds various JVM optional settings
	options[0].optionString = "-Djava.compiler=NONE";           /* disable JIT */
	options[1].optionString = "-Djava.class.path=C:\\Users\\cesar\\Dropbox\\CODE\\HLEA4TC\\dist\\HLEA4TC.jar";	
	options[2].optionString = "-Djava.library.path=c:\\Program Files\\Paramicsv6";  /* set native library path */
	options[3].optionString = "-verbose:jni";                   /* print JNI-related messages */
	
	initArgs.version = JNI_VERSION_1_6;
	initArgs.nOptions = 4;
	initArgs.options = options;
    initArgs.ignoreUnrecognized = false;

	flag = 2;
	JavaVM* jvm;
	JNIEnv* env;
	flag = 3;

	//NEW
	typedef jint (JNICALL * GetCreatedJavaVMs)(JavaVM**, jsize, jsize*);
	GetCreatedJavaVMs JNI_GetCreatedJavaVMs;

	JNI_GetCreatedJavaVMs = (GetCreatedJavaVMs)GetProcAddress(jniModule, "JNI_GetCreatedJavaVMs");

	int n;
	retval = JNI_GetCreatedJavaVMs(&jvm,1, (jsize*) &n);
	flag = 31;
	
	if (retval == JNI_OK)
	{
		if (n == 0)
		{
			//create new
			retval = createJavaVM(&jvm, &env, &initArgs);
		}else
		{
			if (n != 1)
			{
				flag = 32;
				return NULL;
			}
		}
	}

	//here n == 1 
	//flag = retval;
	return jvm;
	
	
	//if (retval == JNI_EEXIST)
	//{
	//	
	//	int n; 
	//	jint res = JNI_GetCreatedJavaVMs(&jvm, 1, (jsize*) &n);
	//	flag = 32;
	//}
	

	/*if (retval != JNI_OK)
	{
		flag = retval;
		return NULL;
	}*/

	// out jvm pointer in the attribute
	//if (env->GetJavaVM(&jvm) != JNI_OK)
	//	return NULL;

	//throw gcnew System::Exception(); // beyond the scope of this answer

	/*
	You would probably launch the JVM at the startup of your application. 
	Unless you are 100% sure that you will only ever invoke Java code 
	from the thread that just created the JVM, you can throw away the 
	env pointer, but you have to keep the jvm pointer.
	*/
	//jvm2 = jvm;

	//make jvm a global?

	//return jvm;
}

int Environment::startPlatform(JavaVM* jvm){
	
	//JNIEnv * env;
 //   // double check it's all ok
 //   int getEnvStat = jvm->GetEnv((void **)&env, JNI_VERSION_1_6);
 //   if (getEnvStat == JNI_EDETACHED) {
 //       std::cout << "GetEnv: not attached" << std::endl;
 //       if (jvm->AttachCurrentThread((void **) &env, NULL) != 0) {
 //           std::cout << "Failed to attach" << std::endl;
 //       }
 //   } else if (getEnvStat == JNI_OK) {
 //       //
 //   } else if (getEnvStat == JNI_EVERSION) {
 //       std::cout << "GetEnv: version not supported" << std::endl;
 //   }

	/*
	first call: app still in the same thread that invoked the JVM
				and have env pointer, then no validation

	*/


	//DWORD retval = startJVM();

	/*** Getting the JNI environment (if not in the same thread, and lost env pointer) */

	//env = NULL;
	

	//env->GetJavaVM(&jvm);


	/*
		get the JVM from ... globals
	*/
	//JavaVM* jvm;

	flag = 4;
	JNIEnv* env;
	bool mustDetach = false;
	// discover env thread using the JVM
	
	jint retval = jvm->GetEnv((void**)&env, JNI_VERSION_1_6);
	//return retval;
	flag = 5;
	if (retval == JNI_EDETACHED)
	{
		JavaVMAttachArgs args;
		args.version = JNI_VERSION_1_6;
		args.name = NULL;
		args.group = NULL;
		retval = jvm->AttachCurrentThread((void **)&env, &args);
		mustDetach = true; // to clean up afterwards
		flag = 6;
	}
	if (retval != JNI_OK)
		throw retval;
	
	flag = 7;
	//invokeJavaCode(env); // next step
	
	//if (true)
	if (retval == JNI_OK)
	{
		flag = 8;
		//TCHAR* dllpath = environment.dllpath;
	
		/**	Invoking Java code: needs env*/

	static const char* const jvmClassName = "com/cdario/hlea4tc/integration/PlatformMediator";

	jclass clazz =  env->FindClass(jvmClassName);
	if (env->ExceptionCheck())
	{
		env->ExceptionOccurred();
		return FAILED;
	}
	//jclass clazz =  env->FindClass("java/lang/Integer");
	if (clazz == NULL){
		return FAILED;
		//throw new Exception;
	}

	jmethodID mid = env->GetStaticMethodID(clazz, "startJadePlatform", "()Z");
	if (mid == NULL){
		return INCOMPLETE;
		//throw NULL;
	}

	//jobject newObj = env->NewObject(clazz, mid);

	BOOL returnedValue = env->CallStaticBooleanMethod(clazz, mid);
	//<type> returnedValue = env->CallStatic<type>Method(clazz, mid, <arguments>);
	//TODO: determine method signature... javap -s
	// type can be any primitive, matching arguments in the java method
	//http://stackoverflow.com/questions/17305524/simplicity-for-executing-java-from-c
	flag = 9;

	/*FROM OPTIONAL*/
	if (mustDetach)
	{
		jvm->DetachCurrentThread();
		flag = 10;
	}
	/* FROM OPTIONAL */


	if (returnedValue)
		return OK;
	}

	return NOT_OK;
}