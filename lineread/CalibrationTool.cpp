#include "CalibrationTool.hpp"
namespace UcitCalibrate
{
	
	CalibrationTool::CalibrationTool()
		: m_PI(3.1415926535898)
		, m_earthR(6378245)
		, m_earthR_Polar(6378245)
		, m_earthrL(6378137)
		, m_earthrS(6356752.3142)
		, m_originlongitude(0)
		, m_originlatitude(0)
		, m_gpsworlds()
		, m_worldBoxPoints()
		, gps_longPick()
		, gps_latiPick()
		, measures_pick()
		, imagePixel_pick()
		, m_radarheight(0)
		, m_cameraintrisic()
		, m_cameraRMatrix()
		, m_cameraTMatrix()
		, m_cameradiff()
		, m_RadarRT()
		, m_cameraRMatrix33()
		, m_cameraRTMatrix44()
		, m_blind()
	{

	};

	CalibrationTool:: ~CalibrationTool()
	{

	};

	void CalibrationTool::SetPi(double pai)
	{
		m_PI = pai;
	}

	double CalibrationTool::rad(double d)
	{
		return d * m_PI / 180.0;
	}

	double CalibrationTool::deg(double x)
	{
		return x * 180 / m_PI;
	}

	double CalibrationTool::GetDistance(longandlat point1, longandlat point2)
	{
		double radpoint1_lat = rad(point1.latitude);
		double radpoint2_lat = rad(point2.latitude);
		double a = radpoint1_lat - radpoint2_lat;
		double b = rad(point1.longtitude) - rad(point2.longtitude);
		double s = 2*asin(sqrt(pow(sin(a/2),2)+ \
			cos(radpoint1_lat) * cos(radpoint2_lat) * pow(sin(b / 2), 2)));
		s = s * m_earthrL;
		return s;
	}

	void CalibrationTool::RadarSpeedHeading(RadarSpeed& m_speed, RadarHeading& m_radarhead)
	{
		m_radarhead.speed_value = sqrt(pow(m_speed.vx, 2) + pow(m_speed.vy, 2));
		double m_world2radar_theta = deg(acos(m_RadarRT.at<double>(0, 0)));
		if (m_speed.vy!=0)
		{
			double theta = atan(m_speed.vx / m_speed.vy);
			
			// 计算雷达速度的反正切
			double m_radarrange = deg(atan(m_speed.vx / m_speed.vy));
			std::cout << "雷达反正切" << m_radarrange << std::endl;
			if (m_speed.vy > 0)
			{
				// 此时背向雷达运动
				m_radarhead.theta = m_world2radar_theta + m_radarrange;
			}
			else {
				// 此时迎面驶来
				m_radarhead.theta = 180 + m_world2radar_theta + m_radarrange;
			}
		}
		else
		{
			if (m_speed.vx>0)
			{
				m_radarhead.theta = 90 + m_world2radar_theta;
			}
			else
			{
				m_radarhead.theta = 270 + m_world2radar_theta;
			}
		}
		
	}

	double CalibrationTool::CalculateHeading(longandlat point1, longandlat point2)
	{
		double y = sin(point2.longtitude - point1.longtitude) * cos(point2.latitude);
		double x = cos(point1.latitude) * sin(point2.latitude) - sin(point1.latitude) * cos(point2.latitude) * \
			cos(point2.longtitude - point1.longtitude);
		double heading = atan2(y, x);
		heading = deg(heading);
		if (heading < 0)
		{
			heading = heading + 360;
		}
		return heading;
	}



	bool CalibrationTool::ReadPickpointXml(std::string m_xmlpath, 
		std::vector<unsigned int>& pickpoints, 
		std::vector<cv::Point2d> &rawpoint, 
		std::map<int, cv::Point2d> &map_points, 
		std::map<int, double> &map_long, 
		std::map<int, double> &map_lan, 
		double &reflectheight, 
		double &installheight, 
		std::map<int, cv::Point3d> &map_Measures, 
		longandlat &originpoll, 
		std::vector<double>& ghostdistort,
		cv::Mat &camerainstrinic)
	{
		//读取xml文件中的参数值
		TiXmlDocument* Document = new TiXmlDocument();
		if (!Document->LoadFile(m_xmlpath.c_str()))
		{
			std::cout << "无法加载xml文件！" << std::endl;
			std::cin.get();
			return false;
		}
		TiXmlElement* RootElement = Document->RootElement();		//根目录

		TiXmlElement* NextElement = RootElement->FirstChildElement();		//根目录下的第一个节点层
		
		while (NextElement != NULL)		//判断有没有读完
		{
			if (NextElement->ValueTStr()=="handreflectorHeight")
			{
				TiXmlElement* BoxElement = NextElement->FirstChildElement();
				while (BoxElement != nullptr)
				{
					reflectheight = atof(BoxElement->GetText());
					BoxElement = BoxElement->NextSiblingElement();
				}
			}
			if (NextElement->ValueTStr()=="installraderheight")
			{
				TiXmlElement* BoxElement = NextElement->FirstChildElement();
				while (BoxElement != NULL)
				{
					installheight = atof(BoxElement->GetText());
					BoxElement = BoxElement->NextSiblingElement();
				}
			}


			if (NextElement->ValueTStr() == "pickpoint")		//读到object节点
			{
				TiXmlElement* BoxElement = NextElement->FirstChildElement();
				while (BoxElement!=nullptr)
				{
					unsigned int point = atof(BoxElement->GetText());		
					pickpoints.push_back(point);
					BoxElement = BoxElement->NextSiblingElement();
				}	
			}
			if (NextElement->ValueTStr() == "originpoll")
			{
				TiXmlElement* BoxElement = NextElement->FirstChildElement();
				double tempoll;
				while (BoxElement != nullptr)
				{
					if (BoxElement->ValueTStr()=="lng")
					{
						tempoll = atof(BoxElement->GetText());
						originpoll.longtitude = tempoll;
					}
					else if (BoxElement->ValueTStr()=="lat")
					{
						originpoll.latitude = atof(BoxElement->GetText());
					}
					BoxElement = BoxElement->NextSiblingElement();
				}
			}

			if (NextElement->ValueTStr()=="pixelcoord")
			{
				TiXmlElement* BoxElement = NextElement->FirstChildElement();
				int count = 0;
				cv::Point2d temp;
				while (BoxElement != nullptr)
				{
					double pixelx, pixely;
					std::string pixel = BoxElement->GetText();
					if (BoxElement->ValueTStr()=="xcoord")
					{
						pixelx = atof(pixel.c_str());
						temp.x = pixelx;
					}
					else
					{
						pixely = atof(pixel.c_str());
						temp.y = pixely;
					}

					pixelx = atof(pixel.c_str());
					BoxElement = BoxElement->NextSiblingElement();
					if (count%2!=0)
					{
						rawpoint.push_back(temp);
					}
					count += 1;
				}
			}
			std::vector<cv::Point3d> v_temp;
			if (NextElement->ValueTStr()=="radarmeasure")
			{
				TiXmlElement* BoxElement = NextElement->FirstChildElement();
				int count = 0;
				cv::Point3d temp;
				
				while (BoxElement != nullptr)
				{
					double horizontal, vertical;
					std::string pixel = BoxElement->GetText();
					if (BoxElement->ValueTStr() == "horizontalDis")
					{
						horizontal = atof(pixel.c_str());
						temp.x = horizontal;
					}
					else
					{
						vertical = atof(pixel.c_str());
						temp.y = vertical;
					}
					
					BoxElement = BoxElement->NextSiblingElement();
					temp.z = reflectheight;
					if (count % 2 != 0)
					{
						v_temp.push_back(temp);
					}
					count += 1;
				}

			}

			for (int i=0;i<v_temp.size();++i)
			{
				map_Measures[i + 1] = v_temp[i];
			}

			for (int i=0;i<rawpoint.size();++i)
			{
				map_points[i+1] = rawpoint[i];
			}

			if (NextElement->ValueTStr() == "gps")
			{
				TiXmlElement* BoxElement = NextElement->FirstChildElement();
				std::vector<double> longs, lans;
				while (BoxElement != nullptr)
				{
					std::string gps = BoxElement->GetText();
					double gps_d = atof(gps.c_str());
					if (BoxElement->ValueTStr()=="lon")
					{
						longs.push_back(gps_d);
					}
					else
					{
						lans.push_back(gps_d);
					}
					BoxElement = BoxElement->NextSiblingElement();
				}

				for (int i=0;i<longs.size();++i)
				{
					map_long[i + 1] = longs[i];
				}
				for (int k= 0; k< lans.size();++k)
				{
					map_lan[k + 1] = lans[k];
				}
			}

			// 读取相机畸变系数
		
			if (NextElement->ValueTStr()=="distort")
			{
				TiXmlElement* BoxElement = NextElement->FirstChildElement();
				
				while (BoxElement!=nullptr)
				{
					if (BoxElement->ValueTStr()=="value0")
					{
						ghostdistort.push_back(atof(BoxElement->GetText()));
					}
					if (BoxElement->ValueTStr() == "value1")
					{
						ghostdistort.push_back(atof(BoxElement->GetText()));
					}
					if (BoxElement->ValueTStr() == "value2")
					{
						ghostdistort.push_back(atof(BoxElement->GetText()));
					}
					if (BoxElement->ValueTStr() == "value3")
					{
						ghostdistort.push_back(atof(BoxElement->GetText()));
					}
					if (BoxElement->ValueTStr() == "value4")
					{
						ghostdistort.push_back(atof(BoxElement->GetText()));
					}
					BoxElement = BoxElement->NextSiblingElement();
				
				}
			}

			camerainstrinic = cv::Mat::eye(3, 3, cv::DataType<double>::type);
			if (NextElement->ValueTStr()=="camerainstrinic")
			{
				TiXmlElement* BoxElement = NextElement->FirstChildElement();
				while (BoxElement!=nullptr)
				{
					if (BoxElement->ValueTStr()=="fx")
					{
						camerainstrinic.at<double>(0, 0) = atof(BoxElement->GetText());
					}
					if (BoxElement->ValueTStr()=="fy")
					{
						camerainstrinic.at<double>(1, 1) = atof(BoxElement->GetText());
					}
					if (BoxElement->ValueTStr()=="cx")
					{
						camerainstrinic.at<double>(0, 2) = atof(BoxElement->GetText());
					}
					if (BoxElement->ValueTStr()=="cy")
					{
						camerainstrinic.at<double>(1, 2) = atof(BoxElement->GetText());
					}
					BoxElement = BoxElement->NextSiblingElement();
				}
			}

			NextElement = NextElement->NextSiblingElement();
		}
		delete Document;
		std::cout << "完成xml的读取" << std::endl;
		return true;
	}

	bool CalibrationTool::ReadCalibrateParam(std::string m_xmlpath, 
		cv::Mat &raderRT44, 
		cv::Mat &cameraRT44, 
		cv::Mat &cameraRT33, 
		cv::Mat &cameraRT31,
		longandlat& originpoll,
		std::vector<double>& ghostdistort,
		cv::Mat& camerainstrinic)
	{
		//读取xml文件中的参数值
		TiXmlDocument* Document = new TiXmlDocument();
		if (!Document->LoadFile(m_xmlpath.c_str()))
		{
			std::cout << "无法加载标定参数xml文件！" << std::endl;
			cin.get();
			return false;
		}
		TiXmlElement* RootElement = Document->RootElement();		//根目录

		TiXmlElement* NextElement = RootElement->FirstChildElement();		//根目录下的第一个节点层
		// 初始化mat矩阵
		raderRT44 = cv::Mat::eye(4, 4, cv::DataType<double>::type);
		cameraRT44 = cv::Mat::zeros(4, 4, cv::DataType<double>::type);
		cameraRT31 = cv::Mat::zeros(3, 1, cv::DataType<double>::type);
		cameraRT33 = cv::Mat::ones(3, 3, cv::DataType<double>::type);

		while (NextElement != NULL)		//判断有没有读完
		{
			if (NextElement->ValueTStr() == "radarRT44")
			{
				TiXmlElement* BoxElement = NextElement->FirstChildElement();
				// 这里使用了mat的ptr指针，用法很有意思，请参考：
				// https://blog.csdn.net/github_35160620/article/details/51708659
				while (BoxElement != nullptr)
				{
					if (BoxElement->ValueTStr()=="index")
					{
						for (int rows =0; rows<4; rows++)
						{
							for (int cols =0; cols<4; cols++)
							{
								double value = atof(BoxElement->GetText());
								cout<<"get value from calibrate xml:"<<value<<endl;
								raderRT44.ptr<double>(rows)[cols] = value;
								BoxElement = BoxElement->NextSiblingElement();
							}
						}
					}	
				}
			}

			if (NextElement->ValueTStr()=="cameraRT44")
			{
				TiXmlElement* BoxElement = NextElement->FirstChildElement();
				// 这里使用了mat的at方式
				while (BoxElement != nullptr)
				{
					if (BoxElement->ValueTStr() == "indexcamera")
					{
						for (int rows = 0; rows < 4; rows++)
						{
							for (int cols = 0; cols < 4; cols++)
							{
								double value = atof(BoxElement->GetText());
								cout << "get value from calibrate xml:" << value << endl;
								cameraRT44.at<double>(rows,cols) = value;
								BoxElement = BoxElement->NextSiblingElement();
							}
						}
					}
				}
			}
			
			if (NextElement->ValueTStr() == "cameraRT33")
			{
				TiXmlElement* BoxElement = NextElement->FirstChildElement();
				// 这里使用了mat的at方式
				while (BoxElement != nullptr)
				{
					if (BoxElement->ValueTStr() == "indexcamera33")
					{
						for (int rows = 0; rows < 3; rows++)
						{
							for (int cols = 0; cols < 3; cols++)
							{
								double value = atof(BoxElement->GetText());
								cout << "get value from calibrate xml:" << value << endl;
								cameraRT33.at<double>(rows, cols) = value;
								BoxElement = BoxElement->NextSiblingElement();
							}
						}
					}
				}
			}

			if (NextElement->ValueTStr() == "cameraRT31")
			{
				TiXmlElement* BoxElement = NextElement->FirstChildElement();
				// 这里使用了mat的at方式
				while (BoxElement != nullptr)
				{
					if (BoxElement->ValueTStr() == "indexcamera31")
					{
						for (int rows = 0; rows < 3; rows++)
						{
							for (int cols = 0; cols < 1; cols++)
							{
								double value = atof(BoxElement->GetText());
								cout << "get value from calibrate xml:" << value << endl;
								cameraRT31.at<double>(rows, cols) = value;
								BoxElement = BoxElement->NextSiblingElement();
							}
						}
					}
				}
			}

			if (NextElement->ValueTStr() == "originpoll")
			{
				TiXmlElement* BoxElement = NextElement->FirstChildElement();
				double tempoll;
				while (BoxElement != nullptr)
				{
					if (BoxElement->ValueTStr() == "lng")
					{
						tempoll = atof(BoxElement->GetText());
						originpoll.longtitude = tempoll;
					}
					else if (BoxElement->ValueTStr() == "lat")
					{
						originpoll.latitude = atof(BoxElement->GetText());
					}
					BoxElement = BoxElement->NextSiblingElement();
				}
			}

			/*cameradistort = cv::Mat::eye(5, 1, cv::DataType<double>::type);
			if (NextElement->ValueTStr() == "cameradistort")
			{
				TiXmlElement* BoxElement = NextElement->FirstChildElement();

				while (BoxElement != nullptr)
				{
					for (int i = 0; i < 5; i++)
					{
						
							cameradistort.at<double>(i) = atof(BoxElement->GetText());
							BoxElement = BoxElement->NextSiblingElement();
						
					}

				}
			}*/

			if (NextElement->ValueTStr() == "distort")
			{
				TiXmlElement* BoxElement = NextElement->FirstChildElement();

				while (BoxElement != nullptr)
				{
					if (BoxElement->ValueTStr() == "value0")
					{
						ghostdistort.push_back(atof(BoxElement->GetText()));
					}
					if (BoxElement->ValueTStr() == "value1")
					{
						ghostdistort.push_back(atof(BoxElement->GetText()));
					}
					if (BoxElement->ValueTStr() == "value2")
					{
						ghostdistort.push_back(atof(BoxElement->GetText()));
					}
					if (BoxElement->ValueTStr() == "value3")
					{
						ghostdistort.push_back(atof(BoxElement->GetText()));
					}
					if (BoxElement->ValueTStr() == "value4")
					{
						ghostdistort.push_back(atof(BoxElement->GetText()));
					}
					BoxElement = BoxElement->NextSiblingElement();

				}
			}

			camerainstrinic = cv::Mat::eye(3, 3, cv::DataType<double>::type);
			if (NextElement->ValueTStr() == "camerainstrinic")
			{
				TiXmlElement* BoxElement = NextElement->FirstChildElement();
				while (BoxElement != nullptr)
				{
					if (BoxElement->ValueTStr() == "fx")
					{
						camerainstrinic.at<double>(0, 0) = atof(BoxElement->GetText());
					}
					if (BoxElement->ValueTStr() == "fy")
					{
						camerainstrinic.at<double>(1, 1) = atof(BoxElement->GetText());
					}
					if (BoxElement->ValueTStr() == "cx")
					{
						camerainstrinic.at<double>(0, 2) = atof(BoxElement->GetText());
					}
					if (BoxElement->ValueTStr() == "cy")
					{
						camerainstrinic.at<double>(1, 2) = atof(BoxElement->GetText());
					}
					BoxElement = BoxElement->NextSiblingElement();
				}
			}


			NextElement = NextElement->NextSiblingElement();
		}
		delete Document;
		std::cout << "完成标定参数xml的读取" << std::endl;
		return true;
	}

	bool CalibrationTool::ReadParaXml(std::string m_strXmlPath, std::vector<BoxSize>& vecNode)
	{
		BoxSize* pNode = new BoxSize;

		//读取xml文件中的参数值
		TiXmlDocument* Document = new TiXmlDocument();
		if (!Document->LoadFile(m_strXmlPath.c_str()))
		{
			std::cout << "无法加载xml文件！" << std::endl;
			std::cin.get();
			return false;
		}
		TiXmlElement* RootElement = Document->RootElement();		//根目录

		TiXmlElement* NextElement = RootElement->FirstChildElement();		//根目录下的第一个节点层
		//for(NextElement;NextElement;NextElement = NextElement->NextSiblingElement())
		while (NextElement != NULL)		//判断有没有读完
		{
			if (NextElement->ValueTStr() == "object")		//读到object节点
			{
				//NextElement = NextElement->NextSiblingElement();

				TiXmlElement* BoxElement = NextElement->FirstChildElement();
				while (BoxElement->ValueTStr() != "bndbox")		//读到box节点
				{
					BoxElement = BoxElement->NextSiblingElement();
				}
				//索引到xmin节点
				TiXmlElement* xminElemeng = BoxElement->FirstChildElement();
				{
					//分别读取四个数值
					pNode->xMin = atof(xminElemeng->GetText());
					TiXmlElement* yminElemeng = xminElemeng->NextSiblingElement();
					pNode->yMin = atof(yminElemeng->GetText());
					TiXmlElement* xmaxElemeng = yminElemeng->NextSiblingElement();
					pNode->xMax = atof(xmaxElemeng->GetText());
					TiXmlElement* ymaxElemeng = xmaxElemeng->NextSiblingElement();
					pNode->yMax = atof(ymaxElemeng->GetText());

					//加入到向量中
					vecNode.push_back(*pNode);
				}
			}
			NextElement = NextElement->NextSiblingElement();
		}

		//释放内存
		delete pNode;
		delete Document;
		std::cout << "完成xml的读取" << std::endl;
		return true;

	}

	void CalibrationTool::SetRadarHeight(double radar_height)
	{
		m_radarheight = radar_height;
	}

	void CalibrationTool::SetCoordinateOriginPoint(double longitude, double latitude)
	{
		m_originlongitude = longitude;
		m_originlatitude = latitude;
	}

	void CalibrationTool::SetCameraInstrinic(double fx, double fy, double cx, double cy)
	{
		m_cameraintrisic = cv::Mat::eye(3, 3, cv::DataType<double>::type);
		m_cameraintrisic.at<double>(0, 0) = fx;
		m_cameraintrisic.at<double>(1, 1) = fy;
		m_cameraintrisic.at<double>(0, 2) = cx;
		m_cameraintrisic.at<double>(1, 2) = cy;
	}

	void CalibrationTool::SetCameraDiff(double df1, double df2, double df3, double df4, double df5)
	{
		m_cameradiff = cv::Mat::eye(5, 1, cv::DataType<double>::type);
		m_cameradiff.at<double>(0, 0) = df1;
		m_cameradiff.at<double>(1, 0) = df2;
		m_cameradiff.at<double>(2, 0) = df3;
		m_cameradiff.at<double>(3, 0) = df4;
		m_cameradiff.at<double>(4, 0) = df5;
	}


	void CalibrationTool::Gps2WorldCoord(std::vector<double> P1_lo, std::vector<double> P1_la)
	{
		if (P1_la.size()!= P1_lo.size())
		{
			printf("input the longitude and latitude can't be paired!!! return");
			return;
		}
		m_gpsworlds.clear();
		double val = m_PI / 180.0;
		
		for (int i = 0; i < P1_la.size(); i++)
		{
			GpsWorldCoord GpsWorldtmp;
			GpsWorldtmp.X = 2 * m_PI * (m_earthR_Polar * cos(P1_la[i] * val)) * ((P1_lo[i] - m_originlongitude) / 360);
			GpsWorldtmp.Y = 2 * m_PI * m_earthR * ((P1_la[i] - m_originlatitude) / 360);
			GpsWorldtmp.Distance = sqrt(GpsWorldtmp.X * GpsWorldtmp.X + GpsWorldtmp.Y * GpsWorldtmp.Y);
			printf("P[%d]::Gps world:x=%.10f Gps World y=%.10f Distance of world=%.10f \n", i + 1, GpsWorldtmp.X, GpsWorldtmp.Y, \
			GpsWorldtmp.Distance);
			m_gpsworlds.push_back(GpsWorldtmp);
		}

		printf("******************* \n");
		printf("******************* \n");
		printf("******************* \n");
		printf("******************* \n");
	}

	void CalibrationTool::WorldCoord2Gps(std::vector<longandlat>& m_longandlat, std::vector<GpsWorldCoord>& m_Gpsworld)
	{
		if (m_Gpsworld.empty())
		{
			printf("input m_gps's world coordinate error! return!");
			return;
		}

		double val = m_PI / 180.0;
		for (int i=0; i< m_Gpsworld.size();++i)
		{
			longandlat temp;
			temp.latitude = (m_Gpsworld[i].Y * 360) / (2 * m_PI * m_earthR) + m_originlatitude;
			temp.longtitude = (m_Gpsworld[i].X * 360) / (2 * m_PI * (m_earthR_Polar * cos(temp.latitude * val))) + m_originlongitude;
			m_longandlat.push_back(temp);
		}
	}

	void CalibrationTool::radarworld2Gps(GpsWorldCoord &m_gpsworldcoord, longandlat &m_gpslongandlat)
	{
		cv::Mat RadarPoint = cv::Mat::ones(4, 1, cv::DataType<double>::type);
		cv::Mat world_point = cv::Mat::ones(4, 1, cv::DataType<double>::type);
		cv::Mat imagetmp = cv::Mat::ones(3, 1, cv::DataType<double>::type);
		RadarPoint.at<double>(0, 0) = -m_gpsworldcoord.X;
		RadarPoint.at<double>(1, 0) = m_gpsworldcoord.Y;
		RadarPoint.at<double>(2, 0) = m_radarheight;
		world_point = m_RadarRT * RadarPoint;
		
		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 1; j++)
			{
				double value = world_point.at<double>(i, j);
				printf("世界坐标:[%.16f] \n", value);
			}
		}
		double val = m_PI / 180.0;
		double worldX = world_point.at<double>(0, 0);
		double worldY = world_point.at<double>(1, 0);
		printf("worldXY[%.5f,%.5f] \n", worldX, worldY);
		m_gpslongandlat.latitude = (worldY * 360) / (2 * m_PI * m_earthR) + m_originlatitude;
		m_gpslongandlat.longtitude = (worldX * 360) / (2 * m_PI * (m_earthR_Polar * cos(m_gpslongandlat.latitude * val))) + m_originlongitude;

		
		
		/*m_gpslongandlat.latitude = (m_gpsworldcoord.Y * 360) / (2 * m_PI * m_earthR) + m_originlatitude;
		m_gpslongandlat.longtitude = (m_gpsworldcoord.X * 360) / (2 * m_PI * (m_earthR_Polar * cos(m_gpslongandlat.latitude * val))) + m_originlongitude;*/

	}

	void CalibrationTool::Gps2radarworld(longandlat& m_gpslongandlat, GpsWorldCoord& m_gpsworldcoord)
	{
		double val = m_PI / 180.0;

			GpsWorldCoord temp;
			temp.X = 2 * m_PI * (m_earthR_Polar * cos(m_gpslongandlat.latitude * val)) * ((m_gpslongandlat.longtitude - m_originlongitude) / 360);
			temp.Y = 2 * m_PI * m_earthR * ((m_gpslongandlat.latitude - m_originlatitude) / 360);
			temp.Distance = sqrt(m_gpsworldcoord.X * m_gpsworldcoord.X + m_gpsworldcoord.Y * m_gpsworldcoord.Y);
			
			cv::Mat Distance_W4 = Mat::ones(4, 1, cv::DataType<double>::type);
			cv::Mat radar_Dis = Mat::ones(4, 1, cv::DataType<double>::type);
			Distance_W4.at<double>(0, 0) = temp.X;
			Distance_W4.at<double>(1, 0) = temp.Y;
			Distance_W4.at<double>(2, 0) = m_radarheight;
			Distance_W4.at<double>(3, 0) = 0;
			radar_Dis = m_RadarRT.inv() * Distance_W4;
			m_gpsworldcoord.X = -radar_Dis.at<double>(0, 0);
			m_gpsworldcoord.Y = radar_Dis.at<double>(1, 0);
			m_gpsworldcoord.Distance = radar_Dis.at<double>(2, 0);

	}

	bool CalibrationTool::CalculateBlind()
	{
		// 计算边界值，y = 0的对应的区域
		Point2d raderpixelPoints, point1, pointend;
		for (float i = -100; i < 100; i += 0.5)
		{
			UcitCalibrate::WorldDistance worldDistance;
			worldDistance.X = i;
			worldDistance.Y = 0;
			worldDistance.Height = 1.2;
			if (i == -99)
			{
				Distance312Pixel(worldDistance, point1);
				raderpixelPoints = point1;
			}
			if (i == 99)
			{
				Distance312Pixel(worldDistance, pointend);
				raderpixelPoints = pointend;
			}
			Distance312Pixel(worldDistance, raderpixelPoints);
			
		}
		m_blind.b = (pointend.y - point1.y) / (pointend.x - point1.x);
		m_blind.k = point1.y - m_blind.b * point1.x;
		
		return true;
	}
	BlindArea CalibrationTool::GetBlind()
	{
		return m_blind;
	}

	std::vector<GpsWorldCoord> CalibrationTool::GetGpsworlds()
	{
		return m_gpsworlds;
	}

	void CalibrationTool::SetWorldBoxPoints()
	{
		if (!m_worldBoxPoints.empty())
		{
			m_worldBoxPoints.clear();
		}
		cv::Point3d tmpPoint;
		for (int i = 0; i < m_gpsworlds.size(); i++)
		{
			tmpPoint.x = m_gpsworlds[i].X;
			tmpPoint.y = m_gpsworlds[i].Y;
			// 预估测量高度为1.2f
			tmpPoint.z = m_radarheight;
			m_worldBoxPoints.push_back(tmpPoint);
		}
	
	}

	void CalibrationTool::PickRawGPSPoints4VectorPairs(std::vector<unsigned int> pointsSet, std::map<int, double>& Gps_longtitude, std::map<int, double>& Gps_latitudes)
	{
		if (pointsSet.empty())
		{
			return;
		}

		gps_latiPick.clear();
		gps_longPick.clear();

		std::map<int, double>::iterator itergpslong = Gps_longtitude.begin();
		std::map<int, double>::iterator itergpslat = Gps_latitudes.begin();
		for (int i =0; i< pointsSet.size(); i++)
		{
			itergpslong = Gps_longtitude.find(pointsSet[i]);
			itergpslat = Gps_latitudes.find(pointsSet[i]);
			if (itergpslong != Gps_longtitude.end())
			{
				printf("Pick the Point[%d] for calibration, be attention!! \n", pointsSet[i]);
				gps_longPick.push_back(itergpslong->second);
				gps_latiPick.push_back(itergpslat->second);
			}
		}
	}

	void CalibrationTool::PickMeasureMentValue4RadarRT(std::vector<unsigned int> pointsSet, std::map<int, cv::Point3d>& measurements)
	{
		if (pointsSet.empty())
		{
			return;
		}
		measures_pick.clear();
		std::map<int, cv::Point3d>::iterator iter	= measurements.begin();
		for (int i = 0; i < pointsSet.size(); i++)
		{
			iter = measurements.find(pointsSet[i]);
			if (iter != measurements.end())
			{
				printf("Pick the Point[%d] for calibration, be attention!! \n", pointsSet[i]);
				measures_pick.push_back(iter->second);
			}
		}
	}

	void CalibrationTool::PickImagePixelPoints4PnPsolve(vector<unsigned int>pointsSet, std::map<int, Point2d>& imagesPixel)
	{
		if (pointsSet.empty())
		{
			return;
		}
		imagePixel_pick.clear();
		std::map<int, Point2d>::iterator iter = imagesPixel.begin();
		for (int i = 0; i < pointsSet.size(); i++)
		{
			iter = imagesPixel.find(pointsSet[i]);
			if (iter != imagesPixel.end())
			{
				printf("Pick the Point[%d] for calibration, be attention!! \n", pointsSet[i]);
				imagePixel_pick.push_back(iter->second);
			}
		}
	}

	std::vector<cv::Point3d> CalibrationTool::GetWorldBoxPoints()
	{
		return m_worldBoxPoints;
	}

	void CalibrationTool::codeRotateByX(double y, double z, double thetax, double& outy, double& outz)
	{
		double y1 = y;
		double z1 = z;
		double rx = thetax * m_PI/ 180;
		outy = cos(rx) * y1 - sin(rx) * z1;
		outz = cos(rx) * z1 + sin(rx) * y1;
	}

	void CalibrationTool::codeRotateByY(double x, double z, double thetay, double& outx, double& outz)
	{
		double x1 = x;
		double z1 = z;
		double ry = thetay * m_PI / 180;
		outx = cos(ry) * x1 + sin(ry) * z1;
		outz = cos(ry) * z1 - sin(ry) * x1;
	}

	void CalibrationTool::codeRotateByZ(double x, double y, double thetaz, double& outx, double& outy)
	{
		double x1 = x;
		double y1 = y;
		double rz = thetaz * m_PI / 180;
		outx = cos(rz) * x1 - sin(rz) * y1;
		outy = sin(rz) * x1 + cos(rz) * y1;
	}

	cv::Point3d CalibrationTool::RotateByVector(double old_x, double old_y, double old_z, double vx, double vy, double vz, double theta)
	{
		double r = theta * m_PI / 180;
		double c = cos(r);
		double s = sin(r);
		double new_x = (vx * vx * (1 - c) + c) * old_x + (vx * vy * (1 - c) - vz * s) * old_y + (vx * vz * (1 - c) + vy * s) * old_z;
		double new_y = (vy * vx * (1 - c) + vz * s) * old_x + (vy * vy * (1 - c) + c) * old_y + (vy * vz * (1 - c) - vx * s) * old_z;
		double new_z = (vx * vz * (1 - c) - vy * s) * old_x + (vy * vz * (1 - c) + vx * s) * old_y + (vz * vz * (1 - c) + c) * old_z;
		return cv::Point3d(new_x, new_y, new_z);
	}


	// 获得雷达坐标系到世界坐标系的R_T矩阵
	cv::Mat CalibrationTool::Get3DR_TransMatrix(const std::vector<cv::Point3d>& srcPoints, const std::vector<cv::Point3d>& dstPoints)
	{
		double srcSumX = 0.0f;
		double srcSumY = 0.0f;
		double srcSumZ = 0.0f;

		double dstSumX = 0.0f;
		double dstSumY = 0.0f;
		double dstSumZ = 0.0f;

		//至少三组点
		if (srcPoints.size() != dstPoints.size() || srcPoints.size() < 3)
		{
			printf("input the radar points are less than three pairs, can't be calculated!!!!");
			return cv::Mat();
		}

		int pointsNum = srcPoints.size();
		for (int i = 0; i < pointsNum; ++i)
		{
			srcSumX += srcPoints[i].x;
			srcSumY += srcPoints[i].y;
			srcSumZ += srcPoints[i].z;

			dstSumX += dstPoints[i].x;
			dstSumY += dstPoints[i].y;
			dstSumZ += dstPoints[i].z;
		}

		cv::Point3d centerSrc, centerDst;

		centerSrc.x = double(srcSumX / pointsNum);
		centerSrc.y = double(srcSumY / pointsNum);
		centerSrc.z = double(srcSumZ / pointsNum);

		centerDst.x = double(dstSumX / pointsNum);
		centerDst.y = double(dstSumY / pointsNum);
		centerDst.z = double(dstSumZ / pointsNum);

		//Mat::Mat(int rows, int cols, int type)
		cv::Mat srcMat(3, pointsNum, CV_64FC1);
		cv::Mat dstMat(3, pointsNum, CV_64FC1);
		//---Modify
		for (int i = 0; i < pointsNum; ++i)//N组点
		{
			//三行
			srcMat.at<double>(0, i) = srcPoints[i].x - centerSrc.x;
			srcMat.at<double>(1, i) = srcPoints[i].y - centerSrc.y;
			srcMat.at<double>(2, i) = srcPoints[i].z - centerSrc.z;

			dstMat.at<double>(0, i) = dstPoints[i].x - centerDst.x;
			dstMat.at<double>(1, i) = dstPoints[i].y - centerDst.y;
			dstMat.at<double>(2, i) = dstPoints[i].z - centerDst.z;

		}

		cv::Mat matS = srcMat * dstMat.t();

		cv::Mat matU, matW, matV;
		cv::SVDecomp(matS, matW, matU, matV);

		cv::Mat matTemp = matU * matV;
		double det = cv::determinant(matTemp);//行列式的值

		double datM[] = { 1, 0, 0, 0, 1, 0, 0, 0, det };
		cv::Mat matM(3, 3, CV_64FC1, datM);

		cv::Mat matR = matV.t() * matM * matU.t();

		double* datR = (double*)(matR.data);
		double delta_X = centerDst.x - (centerSrc.x * datR[0] + centerSrc.y * datR[1] + centerSrc.z * datR[2]);
		double delta_Y = centerDst.y - (centerSrc.x * datR[3] + centerSrc.y * datR[4] + centerSrc.z * datR[5]);
		double delta_Z = centerDst.z - (centerSrc.x * datR[6] + centerSrc.y * datR[7] + centerSrc.z * datR[8]);

		//生成RT齐次矩阵(4*4)
		cv::Mat R_T = (cv::Mat_<double>(4, 4) <<
			matR.at<double>(0, 0), matR.at<double>(0, 1), matR.at<double>(0, 2), delta_X,
			matR.at<double>(1, 0), matR.at<double>(1, 1), matR.at<double>(1, 2), delta_Y,
			matR.at<double>(2, 0), matR.at<double>(2, 1), matR.at<double>(2, 2), delta_Z,
			0, 0, 0, 1
			);

		m_RadarRT = R_T;
		return R_T;

	}

	cv::Mat CalibrationTool::GetRadarRTMatrix()
	{
		return m_RadarRT;
	}

	cv::Mat CalibrationTool::GetCameraRT44Matrix()
	{
		return m_cameraRTMatrix44;
	}

	bool CalibrationTool::SetCameraRT44(cv::Mat CmRT44)
	{
		if (CmRT44.cols!=4 && CmRT44.rows!=4)
		{
			printf("Set cameraRT matrix failed!!!!!!!\n");
			return false;
		}
		else
		{
			m_cameraRTMatrix44 = CmRT44;
			printf("Manual Set cameraRT successful!!!!!!!\n");
		}
		return true;

	}

	bool CalibrationTool::SetCameraRT33(cv :: Mat CmRT33)
		{
			if (CmRT33.cols!=3 && CmRT33.rows!=3)
		{
			//printf("Set cameraRT33 matrix failed!!!!!!!\n");
			return false;
		}
		else
		{
			m_cameraRMatrix33 = CmRT33;
			//printf("Manual Set cameraRT33 successful!!!!!!!\n");
		}
		return true;
		}

	bool CalibrationTool::SetRadarRT44(cv::Mat RadRT44)
	{
		if (RadRT44.cols != 4 && RadRT44.rows != 4)
		{
			printf("Set RadarRT matrix failed!!!!!!!\n");
			return false;
		}
		else
		{
			m_RadarRT = RadRT44;
			printf("Manual Set RadarRT successful!!!!!!!\n");
		}
		return true;
	}

	bool CalibrationTool::SetCameraTMatrix(cv :: Mat CTmatrix)
	{
		m_cameraTMatrix = CTmatrix;
		return true;
	}

	std::vector<cv::Point3d> CalibrationTool::GetMeasureMentPoint()
	{
		return measures_pick;
	}

	void CalibrationTool::CameraPixel2World(cv::Point2d m_pixels, cv::Point3d& m_world, cv::Mat rotate33)
	{
		double s;
		/////////////////////2D to 3D///////////////////////
		cv::Mat imagepixel = cv::Mat::ones(3, 1, cv::DataType<double>::type); //u,v,1
		imagepixel.at<double>(0, 0) = m_pixels.x;
		imagepixel.at<double>(1, 0) = m_pixels.y;

		// 内参矩阵的逆乘以像素坐标变成相机坐标系
		cv::Mat tempMat = rotate33.inv() * m_cameraintrisic.inv() * imagepixel;
		cv::Mat tempMat2 = rotate33.inv() * m_cameraTMatrix;
		double tmp2 = tempMat2.at<double>(2,0);
		double tmp = tempMat.at<double>(2, 0);

		s = m_radarheight + tmp2;
		s /= tmp;
		cout << "s : " << s << endl;

		cv::Mat camera_cordinates = -rotate33.inv() * m_cameraTMatrix;
		cv::Mat wcPoint = rotate33.inv() * (m_cameraintrisic.inv() * s * imagepixel - m_cameraTMatrix);
		m_world.x = wcPoint.at<double>(0, 0);
		m_world.y = wcPoint.at<double>(1, 0);
		m_world.z = wcPoint.at<double>(2, 0);
		cout << "Pixel2World :" << wcPoint << endl;
	}

	void CalibrationTool::CalibrateCamera(bool rasac, bool useRTK, std::vector<unsigned int> pickPoints)
	{
		cv::Mat rvec1(3, 1, cv::DataType<double>::type);  //旋转向量
		cv::Mat tvec1(3, 1, cv::DataType<double>::type);  //平移向量
		m_cameraRMatrix = Mat::zeros(3, 1, cv::DataType<double>::type);
		m_cameraTMatrix = Mat::zeros(3, 1, cv::DataType<double>::type);
		m_cameraRMatrix33 = Mat::zeros(3, 3, cv::DataType<double>::type);
		//调用 pnp solve 函数
		if (useRTK)
		{
			if (rasac)
			{
				cv::solvePnPRansac(m_worldBoxPoints, imagePixel_pick, m_cameraintrisic, m_cameradiff, m_cameraRMatrix, \
					m_cameraTMatrix, false, SOLVEPNP_ITERATIVE);
			}
			else
			{
				if (pickPoints.size() > 4)
				{
					cv::solvePnP(m_worldBoxPoints, imagePixel_pick, m_cameraintrisic, m_cameradiff, m_cameraRMatrix, \
						m_cameraTMatrix, false, SOLVEPNP_ITERATIVE);
				}
				else
				{
					cv::solvePnP(m_worldBoxPoints, imagePixel_pick, m_cameraintrisic, m_cameradiff, m_cameraRMatrix, \
						m_cameraTMatrix, false, SOLVEPNP_P3P);
				}
			}
		}
		else
		{
			if (rasac)
			{
				cv::solvePnPRansac(measures_pick, imagePixel_pick, m_cameraintrisic, m_cameradiff, m_cameraRMatrix, \
					m_cameraTMatrix, false, SOLVEPNP_ITERATIVE);
			}
			else
			{
				if (pickPoints.size() > 4)
				{
					cv::solvePnP(measures_pick, imagePixel_pick, m_cameraintrisic, m_cameradiff, m_cameraRMatrix, \
						m_cameraTMatrix, false, SOLVEPNP_ITERATIVE);
				}
				else
				{
					cv::solvePnP(measures_pick, imagePixel_pick, m_cameraintrisic, m_cameradiff, m_cameraRMatrix, \
						m_cameraTMatrix, false, SOLVEPNP_P3P);
				}
			}
		}
		cv::Rodrigues(m_cameraRMatrix, m_cameraRMatrix33);
		cout << "rotate33:" << m_cameraRMatrix33 << endl;
		hconcat(m_cameraRMatrix33, m_cameraTMatrix, m_cameraRTMatrix44);
		CalculateBlind();
	}

	void CalibrationTool::Pixel2Distance31(Point2d pixels, WorldDistance &Distances)
	{
	
		cv::Point3d tmp;
		CameraPixel2World(pixels, tmp, m_cameraRMatrix33);
		cv::Mat Distance_W4 = Mat::ones(4, 1, cv::DataType<double>::type);
		Point3d Distance_world;
		printf("tmp(%.3f,%.3f,%.3f)\n", tmp.x, tmp.y, tmp.z);
		Distance_W4.at<double>(0, 0) = tmp.x;
		Distance_W4.at<double>(1, 0) = tmp.y;
		Distance_W4.at<double>(2, 0) = tmp.z;
		Distance_W4.at<double>(3, 0) = 0;
		cv::Mat radar_Dis = Mat::ones(4, 1, cv::DataType<double>::type);
		cout << "radar inverse:" << m_RadarRT.inv() << endl;
		radar_Dis = m_RadarRT.inv() * Distance_W4;
		
		Distances.X = -radar_Dis.at<double>(0, 0);
		Distances.Y = radar_Dis.at<double>(1, 0);
		Distances.Height = radar_Dis.at<double>(2, 0);
		cout << "Distance:X" << Distances.X << "\t" << "Distance:Y" << Distances.Y << endl;
	}

	void CalibrationTool::Distance312Pixel(WorldDistance Distances, cv::Point2d& pixels)
  {
    cv::Mat RadarPoint = cv::Mat::ones(4, 1, cv::DataType<double>::type);
    cv::Mat world_point = cv::Mat::ones(4, 1, cv::DataType<double>::type);
    cv::Mat imagetmp = cv::Mat::ones(3, 1, cv::DataType<double>::type);
    RadarPoint.at<double>(0, 0) = -Distances.X;
    RadarPoint.at<double>(1, 0) = Distances.Y;
    RadarPoint.at<double>(2, 0) = Distances.Height;
    world_point = m_RadarRT * RadarPoint;
    // 其实m_cameraRTMatrix44 是3*4的
    
    if (m_cameraRTMatrix44.rows==4)
    {
      cv::Mat dst;
      int a = 3;
      for (int i = 0; i < m_cameraRTMatrix44.rows; i++)
      {
        if (i != a) //第i行不是需要删除的
        {
          dst.push_back(m_cameraRTMatrix44.row(i)); //把message的第i行加到dst矩阵的后面
        }
      }
      m_cameraRTMatrix44 = dst.clone();
    }
    imagetmp = m_cameraintrisic * m_cameraRTMatrix44 * world_point;
//    std::cout << "RTMatrix:" << m_cameraRTMatrix44 << std::endl;
//	std::cout<<"RT_"<<RT_<<std::endl;
    //image_points = m_Calibrations.m_cameraintrisic * RT_ * RadarPoint;
    cv::Mat D_Points = cv::Mat::ones(3, 1, cv::DataType<double>::type);
    D_Points.at<double>(0, 0) = imagetmp.at<double>(0, 0) / imagetmp.at<double>(2, 0);
    D_Points.at<double>(1, 0) = imagetmp.at<double>(1, 0) / imagetmp.at<double>(2, 0);
    
    
    pixels.x = D_Points.at<double>(0, 0);
    pixels.y = D_Points.at<double>(1, 0);

	pixels.x = pixels.x;
	pixels.y = pixels.y;

	pixels.x = pixels.x;
	pixels.y = pixels.y;
    //std::string raders = "radarPoints";
    //std::cout << "Pixel_2D_X:\t" << pixels.x << "\t" << "Pixel_2D_Y:\t" << pixels.y << std::endl;
    //circle(sourceImage, raderpixelPoints, 10, Scalar(200, 0, 255), -1, LINE_AA);
  }
}

