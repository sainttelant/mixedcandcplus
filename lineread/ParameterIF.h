#pragma once
#ifndef ParameterIF_H
#define ParameterIF_H
#include "Database.h"


#ifdef __cplusplus
extern "C" {
#endif

	

	extern void* ParameterIFCreate( );
	extern void  ParameterIFDestroy(void * p_ParameterIF);
	extern _Param Update(void * p_ParameterIF,char* m_path);




#ifdef __cplusplus
}
#endif
#endif