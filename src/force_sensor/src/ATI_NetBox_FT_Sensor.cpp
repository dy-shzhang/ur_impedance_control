/*
 * Software License Agreement (BSD 3-Clause License)
 *
 *   ATI_NetBox_FT_Sensor - A simple application of library FTSensors
 *   Copyright (c) 2016-2017, Simone Ciotti (simone.ciotti@centropiaggio.unipi.it)
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of copyright holder(s) nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
//#include <ATI_NetBox_FT_Sensor.h>
#include <cstdio>
#include <iostream>
#include <FTSensors.h>
#include <Eigen/Dense>

int main(int argc, char** argv)
{
	FTSensors::ATI::NetFT netft;
	double sens_range_fx, sens_range_fy, sens_range_fz, sens_range_tx, sens_range_ty, sens_range_tz;

	std::cout << "Set IP (192.168.1.30)\tResult:" << netft.setIP("192.168.1.30") << "\n\n";

	std::cout << "Get IP: " << netft.getIP() << "\n\n";
	std::cout << "Get port: " << netft.getPort() << "\n\n";

	std::cout << "Set Filter Frequency. Result: " << netft.setFilterFrequency(FTSensors::ATI::FilterFrequency::FILTER_152_HZ) << "\n\n";
	std::cout << "Get Filter Frequency: " << netft.getFilterFrequency() << "\n\n";

	std::cout << "Set Force Unit. Result: " << netft.setForceUnit(FTSensors::ATI::ForceUnit::N) << "\n\n";
	std::cout << "Get Force Unit: " << netft.getForceUnit() << "\n\n";
	netft.getForceSensingRange(sens_range_fx,sens_range_fy,sens_range_fz);
	std::cout << "Get Force Sensing Range (x,y,z): " << sens_range_fx << "\t" << sens_range_fy << "\t" << sens_range_fz << "\n\n";

	std::cout << "Set Torque Unit. Result: " << netft.setTorqueUnit(FTSensors::ATI::TorqueUnit::Nmm) << "\n\n";
	std::cout << "Get Torque Unit: " << netft.getTorqueUnit() << "\n\n";
	netft.getTorqueSensingRange(sens_range_tx,sens_range_ty,sens_range_tz);
	std::cout << "Get Torque Sensing Range (x,y,z): " << sens_range_tx << "\t" << sens_range_ty << "\t" << sens_range_tz << "\n\n";

	std::cout << "Set Data Rate. Result: " << netft.setDataRate(10) << "\n\n";
	std::cout << "Get Data Rate: " << netft.getDataRate() << "\n";

	std::cout << "Start Data Stream and perform the calibration. Result: "<< netft.startDataStream(true) << "\n\n";

	std::string c;
	do
	{
		std::cout << "Enter 'd' to get a new data\nEnter 'c' to perform calibration\nEnter 'r' to reset the data stream without perform the calibration\nEnter 'q' to exit\n\n";
		std::cin >> c;
		std::cout << "\n\n";

		if(c == "c")
		{
			if(!(netft.calibration()))
				std::cout << "Calibration Error\n\n";

			double fbx,fby,fbz;
			double tbx,tby,tbz;

			netft.getBias(fbx,fby,fbz,tbx,tby,tbz);
			
			std::cout << "Force Bias: " << fbx << "," << fby << "," << fbz << " " << netft.getForceUnit() << "\n";
			std::cout << "\nTorque Bias: " << tbx << "," << tby << "," << tbz << " " << netft.getTorqueUnit() << "\n\n";
		}
		else
		{
			if(c == "r")
			{
				netft.stopDataStream();
				netft.startDataStream(false);
			}
			else
			{
				unsigned int pn;
				double pt;
				
				double fx,fy,fz,tx,ty,tz;

				if(!(netft.getData(pn,pt,fx,fy,fz,tx,ty,tz)))
					std::cout << "Data read error\n";

				std::cout << (pt/1000000000) << "sec\t" << pn << "#\t" << fx << "N\t" << fy << "N\t" << fz <<"N\n\n";
				std::cout << (pt/1000000000) << "sec\t" << pn << "#\t" << tx << "Nmm\t" << ty << "Nmm\t" << tz <<"Nmm\n\n";
				
				/*
				std::vector<double > f(3);
				std::vector<double > t(3);
				
				if(!(netft.getData(pn,pt,f,t)))
					std::cout << "Data read error\n";

				std::cout << (pt/1000000000) << "sec\t" << pn << "#\t" << f[0] << "N\t" << f[1] << "N\t" << f[2] <<"N\n\n";
				*/
				/*
				Eigen::Vector3d f;
				Eigen::Vector3d t;
				
				if(!(netft.getData(pn,pt,f,t)))
					std::cout << "Data read error\n";

				std::cout << (pt/1000000000) << "sec\t" << pn << "#\t" << f[0] << "N\t" << f[1] << "N\t" << f[2] <<"N\n\n";
				*/
			}
		}
	}
	while(c != "q");

	std::cout << "Stop Data Stream. Result: " << netft.stopDataStream() << "\n\n";

	return 0;
}