/*
 * Software License Agreement (BSD 3-Clause License)
 *
 *   FTSensors - Configure and read data from a force/torque sensor
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
#ifndef FT_SENSORS_H
#define FT_SENSORS_H

#include <string>
#include <sstream>
#include <vector>
#include <boost/lexical_cast.hpp>
#include <boost/asio.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/chrono.hpp>
#include <boost/timer/timer.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread/thread.hpp>
#include <boost/algorithm/string/erase.hpp>
#include <Eigen/Dense>

namespace FTSensors
{
    /*
        sensor manufacturer
    */
    namespace ATI
    {
        enum class FilterFrequency: unsigned short int {NO_FILTER = 0, FILTER_5_HZ = 8, FILTER_8_HZ = 7, FILTER_18_HZ = 6, FILTER_35_HZ = 5, FILTER_73_HZ = 4, FILTER_152_HZ = 3, FILTER_326_HZ = 2, FILTER_838_HZ = 1, FILTER_1500_HZ = 9, FILTER_2000_HZ = 10, FILTER_2500_HZ = 11, FILTER_3000_HZ = 12, NO_VALUE = 256};
        enum class ForceUnit: unsigned short int {lbf = 1, N = 2, klbf = 3, KN = 4, kgf = 5, gf = 6, NO_VALUE = 256};
        enum class TorqueUnit: unsigned short int {lbf_in = 1, lbf_ft = 2, Nm = 3, Nmm = 4, kgf_cm = 5, KNm = 6, NO_VALUE = 256};
        
        std::ostream& operator<<(std::ostream& os, FilterFrequency ff);
        std::ostream& operator<<(std::ostream& os, ForceUnit fu);
        std::ostream& operator<<(std::ostream& os, TorqueUnit tu);

        /*
            sensor type
        */
        class NetFT
        {
            private:
                enum class NetBoxParameter: unsigned short int {
                    ACTIVE_CONFIGURATION,
                    FILTER_FREQUENCY,
                    COUNTS_PER_FORCE,
                    COUNTS_PER_TORQUE,
                    FORCE_UNIT,
                    TORQUE_UNIT,
                    FORCE_SENSING_RANGE_X,
                    FORCE_SENSING_RANGE_Y,
                    FORCE_SENSING_RANGE_Z,
                    TORQUE_SENSING_RANGE_X,
                    TORQUE_SENSING_RANGE_Y,
                    TORQUE_SENSING_RANGE_Z,
                    DATA_RATE,
                    RDT_INTERFACE,
                    RDT_BUFFER,
                    MULTI_UNIT_SYNC,
                    THRESHOLD_MONITORING,
                    TOOL_TRANSFORMATION_DISTANCE_X, TOOL_TRANSFORMATION_DISTANCE_Y, TOOL_TRANSFORMATION_DISTANCE_Z,
                    TOOL_TRANSFORMATION_ROTATION_X, TOOL_TRANSFORMATION_ROTATION_Y, TOOL_TRANSFORMATION_ROTATION_Z
                };

                static const unsigned int MAX_TIME_SET_GET_NETBOX_PARMAS_SEC = 10;

                /*
                    fixed structure to send RDT (RawDataTransfer) request
                */
                struct Request
                {
                    unsigned short int command_header;//required
                    unsigned short int command;//command to execute
                    unsigned int sample_count;//samples to output (0 = infinite)
                };

                /*
                    fixed structure to receive RDT (RawDataTransfer) data
                */
                struct Record
                {
                    unsigned int rdt_sequence;//RDT packet sequence number since communication start
                    unsigned int ft_sequence;//RDT packet sequence number since the sensor poweron
                    unsigned int status;//system status code

                    //force and torque are in sensor tick
                    int Fx;//x-axis force
                    int Fy;//y-axis force
                    int Fz;//z-axis force
                    int Tx;//x-axis torque
                    int Ty;//y-axis torque
                    int Tz;//z-axis torque
                }; //288 bit

                Request request;//Instance of struct Request to send RDT request
				
				//sensor IP
				std::string IP;
				//sensor Port
				static const unsigned short int port = 49152;
				//all programs that use asio need to have at least one boost::asio::io_service object
				boost::asio::io_service udpIOService;
				//pointer to a boost::asio::ip::udp::socket object to receive/send requests on UDP
				boost::shared_ptr<boost::asio::ip::udp::socket > udpSock;
				
                FilterFrequency filterFrequency;//data filter frequency in Hz
                ForceUnit forceUnit;//force measurement unit
                TorqueUnit torqueUnit;//torque measurement unit
                double countsPerForce;//value to convert Record.Fi from sensor tick to forceUnit
                double countsPerTorque;//value to convert Record.Ti from sensor tick to torqueUnit
                double forceSensingRange;
                double torqueSensingRange;
                unsigned short int dataRate;//data rate in Hz (from 1 to 7000)
				
                Eigen::Vector3d forceBias;// sensor force bias in forceUnit
                Eigen::Vector3d torqueBias;//sensor torque bias in torqueUnit

                boost::timer::cpu_timer packet_time;

                /*
                    send an HTTP GET request to the sensor to configure one of its parameters

                    @return true if the request is performed correctly, otherwise return false
                */
				bool setNetBoxParameter(NetBoxParameter nbp, unsigned short int param_value);

                /*
                    Get the specified NetBox parameter in a std::string format

                    @return true if the request is performed correctly, otherwise return false 
                */
                bool getNetBoxParameter(NetBoxParameter nbp, double& param_value);

			public:
                NetFT();
				/*
                    destructor
                */
                ~NetFT();

                /*
                    sensor internal calibration

                    if samples_number == 0 then samples_number = this->dataRate

                    @return true if the calibration is performed correctly, otherwise return false
                */
                bool calibration(unsigned int samples_number = 0);

                /*
                    get the sensor bias
                */
                template<typename T, typename U>
                void getBias(T& f,U& t);

                void getBias(double& fx, double& fy, double& fz, double& tx, double& ty, double& tz);

				/*
                    set sensor IP address

					@param	ip		sensor IP address

					return:
						true	if ip is a valid IP address in the form xxx.yyy.zzz.www and the sensor is ready to connect
						false	otherwise
				*/
                bool setIP(std::string ip);
                /*
                    get the sensor IP address
                */
                std::string getIP();
                /*
                    get the communication port
                */
                unsigned short int getPort();

                /*
                    set data filter frequency in Hz

					parameter:
						f	data filter frequency in Hz (0 (no filter), 5, 8, 18, 35, 73, 152, 326, 838, 1500, 2000, 2500, 3000)

					return:
						true	if ff is valid
						false	otherwise
                */
                bool setFilterFrequency(FilterFrequency ff);
                /*
                    get data filter frequency in Hz
                */
                FilterFrequency getFilterFrequency();

                /*
                    set force measurement unit

					parameters:
						fu	force unit (lbf, N, klbf, kN, kgf, gf)

					return:
						true	if fu is valid
						false	otherwise
                */
                bool setForceUnit(ForceUnit fu);
                /*
                    get force measurement unit
                */
                ForceUnit getForceUnit();

                template<typename T>
                bool getForceSensingRange(T& f);

                bool getForceSensingRange(double& fx, double& fy, double& fz);

                /*
                    set sensor torque measurement unit

                    parameters:
						tu	torque unit (lbf-in, lbf-ft, N-m, N-mm, kgf-cm, kN-m)

					return:
						true	if tu is valid
						false	otherwise
                */
                bool setTorqueUnit(TorqueUnit tu);
                /*
                    get torque measurement unit
                */
                TorqueUnit getTorqueUnit();

                template<typename T>
                bool getTorqueSensingRange(T& t);

                bool getTorqueSensingRange(double& tx, double& ty, double& tz);

				/*
                    set data rate in Hz

					Parameters:
						dr	data rate in Hz (from 1 to 7000)

					Return:
						true	if dr is a valid value
						false	otherwise
                */
                bool setDataRate(unsigned short int dr);
                /*
                    get data rate in Hz
                */
                unsigned short int getDataRate();
 
                /*
                    1. connect to the sensor
                    2. send the HTTP GET requests to configure the sensor
                    3. start the data stream

					Parameters:
						calibration  if true perform the sensor calibration
					
					Return:
                        true	
                        false	it is impossible communicate to sensor
                */
                bool startDataStream(bool calibration = true);

                /*
                    get a new data

                    Parameters:
                        pn       data packet number
                        pt       data packet time
                        f        force vector
                        t        torque vector

                    Return:
                        true    
                        false   A timeout occurred
                */
                template<typename T, typename U>
                bool getData(unsigned int& pn, double& pt, T& f, U& t);
                template<typename T, typename U>
                bool getData(T& f, U& t);
                /*
                    get a new data

					Parameters:
						pn       data packet number
						pt       data packet time
						fx       force along x-axis
						fy       force along y-axis
						fz       force along z-axis
						tx       torque along x-axis
						ty       torque along y-axis
						tz       torque along z-axis

                    Return:
                        true	
                        false	A timeout occurred
                */
                bool getData(unsigned int& pn, double& pt, double& fx, double& fy, double& fz, double& tx, double& ty, double& tz);
                bool getData(double& fx, double& fy, double& fz, double& tx, double& ty, double& tz);

                /*
                    1. stop the data stream
                    2. close connection to the sensor

					Return:
						true	
                        false	it is impossible communicate to sensor
				*/
                bool stopDataStream();
        };
    }
}

#endif // FT_SENSORS_H