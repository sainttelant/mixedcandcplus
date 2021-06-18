#pragma once
#ifndef Parameter_H
#define Parameter_H
#include "Database.h"
#include "CalibrationTool.hpp"


#define N 100
class Parameter
{
	struct CalibrateParam
	{
		cv::Mat m_rt44;
		cv::Mat	m_crt44;
		cv::Mat m_crt33;
		cv::Mat m_crt31;
		UcitCalibrate::longandlat originallpoll;
		std::vector<double> m_ghostdis;
		cv::Mat camrainst;
	};

	struct vehicleParam
	{
		// need to be done!
		int x;
	};

	public:
		static Parameter &Instance() {
			static Parameter m_ucitparam;
			return m_ucitparam;
		};

		Parameter();
		virtual ~Parameter();

		bool GetXmlPath(char* calibxml, char* vehiclexml, char* configxml);
		
		_Param Update();
		
		CalibrateParam GetCalibrationParam();
		vehicleParam GetVehicleParam();

	protected:
	private:
		char* m_calibpath;
		char* m_vehipath;
		CalibrateParam m_calibration;
		vehicleParam m_vehicleParam;


		
};

#endif