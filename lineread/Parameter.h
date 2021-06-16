#pragma once
#ifndef Parameter_H
#define Parameter_H
#include "Database.h"
#include "CalibrationTool.hpp"

class Parameter
{
	public:
		static Parameter &Instance() {
			static Parameter m_ucitparam;
			return m_ucitparam;
		};

		Parameter() {};
		virtual ~Parameter() {};
		
		_Param Update(char* m_path)
		{
			// construct some parameters what you want
			// for example

			cv::Mat m_rt44, m_crt44, m_crt33, m_crt31;
			
			UcitCalibrate::longandlat originallpoll;
			std::vector<double> m_ghostdis;
			cv::Mat camrainst;

			// remove "\n"
			m_path[strlen(m_path)-2]=0;
			std::string path = m_path;
			UcitCalibrate::CalibrationTool::getInstance().ReadCalibrateParam(path,
				m_rt44, 
				m_crt44,
				m_crt33, 
				m_crt31,
				originallpoll,
				m_ghostdis, 
				camrainst);

			_Param temp;
			temp.a = 0;
			temp.b = 0;
			temp.c = false;
			return temp;
		};
	protected:
	private:
		
};

#endif