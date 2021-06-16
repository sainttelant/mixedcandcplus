
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "Database.h"
#include "ParameterIF.h"

#define N 100

bool GetXmlPath(char* calibxml, char* vehiclexml, char* configxml)
{
	FILE* fp;
	char line[1000];
	char str[N + 1];
	if ((fp = fopen(configxml, "rt")) == NULL) {
		printf("Cannot open file, press any key to exit!\n");
		//getch();
		return false;
	}

	char* cali = "calibrationxml";
	char* vehi = "vehiclexml";
	char* substr = "../";

	while (!feof(fp))
	{
		fgets(line, 1000, fp);
		
		
		char* res_cal = strstr(line, cali);
		if (res_cal!=NULL)
		{
			char* s = strstr(line, substr);
			strcpy(calibxml, s);
		}
		char* res_veh = strstr(line, vehi);
		if (res_veh != NULL)
		{
			char* s = strstr(line, substr);
			strcpy(vehiclexml, s);
		}
		
	}
	
	printf(">>>>>>><<<<<<<<< \n");

	
	fclose(fp);
	return true;
}


int main(int argc, char* argv[])
{

	char* calx=(char*)malloc(sizeof(char)*N);
	char* vehx= (char*)malloc(sizeof(char) * N);

	
	char* config = argv[1];

	bool ret = GetXmlPath(calx, vehx, config);


	printf("end[%s,%s] \n", calx,vehx);

	// 调用Parameter class
	_Param test;
	void* Parameter = ParameterIFCreate();
	test = Update(Parameter, calx);

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