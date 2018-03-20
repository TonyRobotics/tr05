#include <iostream>
#include <cstdlib>
#include <string>
#include <cstring>
#include <cctype>
#include <thread>
#include <chrono>
#include "mqtt/async_client.h"

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <ros/serialization.h>
ros::NodeHandle *nh;
ros::Publisher pub_control_vel;

const std::string SERVER_ADDRESS("tcp://localhost:1883");
const std::string CLIENT_ID("robot_tr05");
const std::string TOPIC("control_vel");

const int	QOS = 0;
const int	N_RETRY_ATTEMPTS = 5;

class action_listener : public virtual mqtt::iaction_listener
{
	std::string name_;
	void on_failure(const mqtt::token& tok) override {
		std::cout << name_ << " failure";
		if (tok.get_message_id() != 0)
			std::cout << " for token: [" << tok.get_message_id() << "]" << std::endl;
		std::cout << std::endl;
	}
	void on_success(const mqtt::token& tok) override {
		std::cout << name_ << " success";
		if (tok.get_message_id() != 0)
			std::cout << " for token: [" << tok.get_message_id() << "]" << std::endl;
		auto top = tok.get_topics();
		if (top && !top->empty())
			std::cout << "\ttoken topic: '" << (*top)[0] << "', ..." << std::endl;
		std::cout << std::endl;
	}
public:
	action_listener(const std::string& name) : name_(name) {}
};

class callback : public virtual mqtt::callback,
					public virtual mqtt::iaction_listener{
	int nretry_;
	mqtt::async_client& cli_;
	mqtt::connect_options& connOpts_;
	action_listener subListener_;
	void reconnect() {
		std::this_thread::sleep_for(std::chrono::milliseconds(2500));
		try {
			cli_.connect(connOpts_, nullptr, *this);
		}
		catch (const mqtt::exception& exc) {
			std::cerr << "Error: " << exc.what() << std::endl;
			exit(1);
		}
	}
	void on_failure(const mqtt::token& tok) override {
		std::cout << "Connection failed" << std::endl;
		if (++nretry_ > N_RETRY_ATTEMPTS)
			exit(1);
		reconnect();
	}
	void on_success(const mqtt::token& tok) override {
		std::cout << "\nConnection success" << std::endl;
		std::cout << "\nSubscribing to topic '" << TOPIC << "'\n"
			<< "\tfor client " << CLIENT_ID
			<< " using QoS" << QOS << "\n"
			<< "\nPress Q<Enter> to quit\n" << std::endl;

		cli_.subscribe(TOPIC, QOS, nullptr, subListener_);
	}
	void connection_lost(const std::string& cause) override {
		std::cout << "\nConnection lost" << std::endl;
		if (!cause.empty())
			std::cout << "\tcause: " << cause << std::endl;

		std::cout << "Reconnecting..." << std::endl;
		nretry_ = 0;
		reconnect();
	}
	void message_arrived(mqtt::const_message_ptr msg) override {
		std::cout << "Message arrived" << std::endl;
		std::cout << "\ttopic: '" << msg->get_topic() << "'" << std::endl;
		std::cout << "\tpayload: '" << msg->to_string() << "'\n" << std::endl;
                if(msg->get_topic() == "control_vel"){
                    ROS_INFO("Received control_vel");
                    uint8_t * payloadptr = (uint8_t *)msg->get_payload().c_str();
                    uint32_t serial_size = msg->get_payload().size();
                    ros::serialization::IStream stream(payloadptr, serial_size);
                    geometry_msgs::Twist control_vel;
                    ros::serialization::deserialize(stream, control_vel);
                    control_vel.linear.x -= 100;
                    control_vel.angular.z -= 100;
                    ROS_INFO("deserialized vel: (%f, %f)", control_vel.linear.x, control_vel.angular.z);
                    pub_control_vel.publish(control_vel); 
                }

	}

	void delivery_complete(mqtt::delivery_token_ptr token) override {}

public:
	callback(mqtt::async_client& cli, mqtt::connect_options& connOpts)
				: nretry_(0), cli_(cli), connOpts_(connOpts), subListener_("Subscription") {}
};

int main(int argc, char* argv[])
{
        ros::init(argc, argv, "mqtt_client_pub");
        nh = new ros::NodeHandle();  
        pub_control_vel = nh->advertise<geometry_msgs::Twist>("cmd_vel", 5);

	mqtt::connect_options connOpts;
	connOpts.set_keep_alive_interval(20);
	connOpts.set_clean_session(true);
	mqtt::async_client client(SERVER_ADDRESS, CLIENT_ID);
	callback cb(client, connOpts);
	client.set_callback(cb);
	try {
		std::cout << "Connecting to the MQTT server..." << std::flush;
		client.connect(connOpts, nullptr, cb);
	}
	catch (const mqtt::exception&) {
		std::cerr << "\nERROR: Unable to connect to MQTT server: '"
			<< SERVER_ADDRESS << "'" << std::endl;
		return 1;
	}
	while (std::tolower(std::cin.get()) != 'q')
		;
	try {
		std::cout << "\nDisconnecting from the MQTT server..." << std::flush;
		client.disconnect()->wait();
		std::cout << "OK" << std::endl;
	}
	catch (const mqtt::exception& exc) {
		std::cerr << exc.what() << std::endl;
		return 1;
	}
 	return 0;
}

