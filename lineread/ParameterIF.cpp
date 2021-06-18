#include "ParameterIF.h"
#include "Parameter.h"
#include "tinystr.h"
#include "tinyxml.h"



	extern void* ParameterIFCreate()
	{
		return new Parameter;
	}

	extern void  ParameterIFDestroy(void* p_ParameterIF)
	{
		Parameter* temp = (Parameter*)p_ParameterIF;
		delete temp;
	}

	extern bool GetXmlPath(void* p_ParameterIF,char* calibxml, char* vehiclexml, char* configxml)
	{
		Parameter* temp = (Parameter*)p_ParameterIF;
		return temp->GetXmlPath(calibxml,vehiclexml,configxml);

	}

	extern _Param Update(void* p_ParameterIF)
	{
		Parameter* temp = (Parameter*)p_ParameterIF;
		return temp->Update();
	}

	



