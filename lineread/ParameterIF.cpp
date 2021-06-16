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

	extern _Param Update(void* p_ParameterIF,char* m_path)
	{
		Parameter* temp = (Parameter*)p_ParameterIF;
		return temp->Update(m_path);
	}

	



