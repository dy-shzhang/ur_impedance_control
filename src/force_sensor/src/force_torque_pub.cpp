#include "ros/ros.h"
#include "std_msgs/String.h"
#include "force_sensor/FTSensors.h"
#include "force_sensor/Force_Torque.h"
#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char *argv[])
{
	/**
	 * The ros::init() function needs to see argc and argv so that it can perform
	 * any ROS arguments and name remapping that were provided at the command line.
	 * For programmatic remappings you can use a different version of init() which takes
	 * remappings directly, but for most command-line programs, passing argc and argv is
	 * the easiest way to do it.  The third argument to init() is the name of the node.
	 *
	 * You must call one of the versions of ros::init() before using any other
	 * part of the ROS system.
	 */
	ros::init(argc, argv, "force_torque_pub");

	/**
	 * NodeHandle is the main access point to communications with the ROS system.
	 * The first NodeHandle constructed will fully initialize this node, and the last
	 * NodeHandle destructed will close down the node.
	 */
	ros::NodeHandle n;

	ROS_INFO("begin to connect");
	FTSensors::ATI::NetFT netft;
	netft.setIP("192.168.1.30");
	ROS_INFO("set frequency");
	netft.setFilterFrequency(FTSensors::ATI::FilterFrequency::FILTER_5_HZ); //NO_FILTER FILTER_35_HZ
	
	netft.setForceUnit(FTSensors::ATI::ForceUnit::N);
	netft.setTorqueUnit(FTSensors::ATI::TorqueUnit::Nm);
	ROS_INFO("set data rate");
	netft.setDataRate(2000);
	netft.startDataStream(true);
	ROS_INFO("ATI_FORCE_SENSOR READY!");
	/**
	 * The advertise() function is how you tell ROS that you want to
	 * publish on a given topic name. This invokes a call to the ROS
	 * master node, which keeps a registry of who is publishing and who
	 * is subscribing. After this advertise() call is made, the master
	 * node will notify anyone who is trying to subscribe to this topic name,
	 * and they will in turn negotiate a peer-to-peer connection with this
	 * node.  advertise() returns a Publisher object which allows you to
	 * publish messages on that topic through a call to publish().  Once
	 * all copies of the returned Publisher object are destroyed, the topic
	 * will be automatically unadvertised.
	 *
	 * The second parameter to advertise() is the size of the message queue
	 * used for publishing messages.  If messages are published more quickly
	 * than we can send them, the number here specifies how many messages to
	 * buffer up before throwing some away.
	 */
	ros::Publisher chatter_pub = n.advertise<force_sensor::Force_Torque>("/force", 10);
	force_sensor::Force_Torque msgToSent;
	msgToSent.fx=msgToSent.fy=msgToSent.fz=msgToSent.tx=msgToSent.ty=msgToSent.tz=0;
	ros::Rate loop_rate(1000);

	/**
	 * A count of how many messages we have sent. This is used to create
	 * a unique string for each message.
	 */
	int count = 0;
	//unsigned int pn;
	//double pt;
	force_sensor::Force_Torque oldmsg;
	force_sensor::Force_Torque msg;
	while (ros::ok())
	{
		/**
		 * This is a message object. You stuff it with data, and then publish it.
		 */

		int nc=0;
		while(nc<11 && !(netft.getData(msg.fx,msg.fy,msg.fz,msg.tx,msg.ty,msg.tz))){
			ROS_INFO("Force_Torque again");
			nc++;
		}
		if(nc==11){
			msg =oldmsg;
			ROS_INFO("KONG 11");
		}
		else{
			oldmsg =msg;
			ROS_INFO("FEI KONG");		
		}

		{
			msgToSent.fx += msg.fx/4.;
			msgToSent.fy += msg.fy/4.;
			msgToSent.fz += msg.fz/4.;

			msgToSent.tx += msg.tx/4.;
			msgToSent.ty += msg.ty/4.;
			msgToSent.tz += msg.tz/4.;
		}
		//ROS_INFO("%s", msg.data.c_str());

		/**
		 * The publish() function is how you send messages. The parameter
		 * is the message object. The type of this object must agree with the type
		 * given as a template parameter to the advertise<>() call, as was done
		 * in the constructor above.
		 */
		 if(count==3){
			 chatter_pub.publish(msgToSent);
			 msgToSent.fx=msgToSent.fy=msgToSent.fz=msgToSent.tx=msgToSent.ty=msgToSent.tz=0;
			 count=0;
		 }
		 else
		 	count++;
		ros::spinOnce();
		loop_rate.sleep();
	}
	netft.stopDataStream();
	ros::shutdown();
	return 0;
}
