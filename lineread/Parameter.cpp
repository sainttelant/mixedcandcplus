#include "Parameter.h"

Parameter::Parameter():
	m_calibration()
	,m_calibpath(0)
	,m_vehipath(0)
	,m_vehicleParam()
{

}
Parameter::~Parameter()
{

}

_Param Parameter::Update()
{
	// construct some parameters what you want
	// for example


	// remove "\n"
	m_calibpath[strlen(m_calibpath) - 2] = 0;
	std::string path = m_calibpath;
	printf("calibrate xml path %s \n", path.c_str());


	UcitCalibrate::CalibrationTool::getInstance().ReadCalibrateParam(path,
		m_calibration.m_rt44,
		m_calibration.m_crt44,
		m_calibration.m_crt33,
		m_calibration.m_crt31,
		m_calibration.originallpoll,
		m_calibration.m_ghostdis,
		m_calibration.camrainst);

	// need to be add vehicle reading and others
	m_vehipath[strlen(m_vehipath) - 2] = 0;
	
	printf("vehicle xml path %s \n",m_vehipath);


	// for testing basic functions, 
	// if wanted to deploy _Param in main.c application
	// the _Param should be the configured 
	// in basic class,which should be identified by c

	_Param temp;
	temp.latitude = m_calibration.originallpoll.latitude;
	temp.longtitude = m_calibration.originallpoll.longtitude;
	temp.c = false;
	return temp;
};

Parameter::CalibrateParam Parameter::GetCalibrationParam()
{
	return m_calibration;
}

Parameter::vehicleParam Parameter::GetVehicleParam()
{
	return m_vehicleParam;
}

bool Parameter::GetXmlPath(char* calibxml, char* vehiclexml, char* configxml)
{
	FILE* fp;
	char line[1000];
	char str[N + 1];
	if ((fp = fopen(configxml, "rt")) == NULL) {
		printf("Cannot open file, press any key to exit!\n");
		//getch();
		return false;
	}
	// the following should be changed according to the real inputs, be attention!
	const char* cali = "calibrationxml";
	const char* vehi = "vehiclexml";
	const char* substr = "../";

	while (!feof(fp))
	{
		fgets(line, 1000, fp);


		char* res_cal = strstr(line, cali);
		if (res_cal != NULL)
		{
			char* s = strstr(line, substr);
			strcpy(calibxml, s);
			m_calibpath = calibxml;
		}
		char* res_veh = strstr(line, vehi);
		if (res_veh != NULL)
		{
			char* s = strstr(line, substr);
			strcpy(vehiclexml, s);
			m_vehipath = vehiclexml;
		}

	}
	printf(">>>>>>><<<<<<<<< \n");
	fclose(fp);
	return true;
}
