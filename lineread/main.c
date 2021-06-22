
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "Database.h"
#include "ParameterIF.h"
#include "test.h"



int main(int argc, char* argv[])
{
	int N = 100;
	char* calx=(char*)malloc(sizeof(char)*N);
	char* vehx= (char*)malloc(sizeof(char) * N);

	
	char* config = argv[1];

	

	// 调用Parameter class
	_Param test;
	void* Parameter = ParameterIFCreate();

	bool ret = GetXmlPath(Parameter,calx, vehx, config);
	printf("end[%s,%s] \n", calx, vehx);
	test = Update(Parameter);


	if (calx!=NULL)
	{
		free(calx);
		calx = NULL;
	}
	if (vehx!=NULL)
	{
		free(vehx);
		vehx = NULL;
	}

	
	// 使用完销毁Parameter 类
	ParameterIFDestroy(Parameter);

	system("PAUSE");
	return 0;
}