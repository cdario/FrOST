// FrOST.Console.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"


int _tmain(int argc, _TCHAR* argv[])
{
	
	Environment env = Environment();
	
	JavaVM* jvm_r = env.startJVM();
	int stat = env.startPlatform(jvm_r);

	if (stat == 0)
		printf("all  OK");
	else
		printf(">>> Error starting JADE platform main-container %i", stat);
		
	return 0;
}

