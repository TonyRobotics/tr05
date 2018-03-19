#include "ros/ros.h"

#include <ros/serialization.h>

#include <iostream>
#include <cstdlib>
#include <string>
#include <thread>	// For sleep
#include <chrono>
#include <cstring>
#include "mqtt/async_client.h"

#include "pthread.h"
#include <patrol_msgs/patrol_data.h>

const int  QOS = 0;
const long TIMEOUT = 10000L;

class action_listener : public virtual mqtt::iaction_listener{
    std::string name_;
    virtual void on_failure(const mqtt::itoken& tok) {
    std::cout << name_ << " failure";
    if (tok.get_message_id() != 0)
        std::cout << " (token: " << tok.get_message_id() << ")" << std::endl;
    std::cout << std::endl;
}
virtual void on_success(const mqtt::itoken& tok) {
    std::cout << name_ << " success";
    if (tok.get_message_id() != 0)
        std::cout << " (token: " << tok.get_message_id() << ")" << std::endl;
    if (!tok.get_topics().empty())
        std::cout << "\ttoken topic: '" << tok.get_topics()[0] << "', ..." << std::endl;
    std::cout << std::endl;
}
public:
    action_listener(const std::string& name) : name_(name) {}
};

class callback : public virtual mqtt::callback,
		         public virtual mqtt::iaction_listener{
    int nretry_;
    mqtt::async_client& cli_;
    action_listener& listener_;

    void reconnect() {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        mqtt::connect_options connOpts;
        connOpts.set_keep_alive_interval(20);
        connOpts.set_clean_session(true);
        try {
            cli_.connect(connOpts, nullptr, *this);
        }
        catch (const mqtt::exception& exc) {
            std::cerr << "Error: " << exc.what() << std::endl;
            exit(1);
        }
    }
    // Re-connection failure
    virtual void on_failure(const mqtt::itoken& tok) {
        std::cout << "Reconnection failed." << std::endl;
        if (++nretry_ > 5)
            exit(1);
        reconnect();
    }
    // Re-connection success
    virtual void on_success(const mqtt::itoken& tok) {
        std::cout << "Reconnection success" << std::endl;;
        cli_.subscribe("helloooooo", QOS, nullptr, listener_);
    }
    virtual void connection_lost(const std::string& cause) {
        std::cout << "\nConnection lost" << std::endl;
        if (!cause.empty())
            std::cout << "\tcause: " << cause << std::endl;
        std::cout << "Reconnecting." << std::endl;
        nretry_ = 0;
        reconnect();
    }
    virtual void message_arrived(const std::string& topic, mqtt::message_ptr msg) {
    }
    virtual void delivery_complete(mqtt::idelivery_token_ptr token) {}
public:
    callback(mqtt::async_client& cli, action_listener& listener) : cli_(cli), listener_(listener) {
    }
};

class TopicMan{
public:
    // constructor
    TopicMan(std::string address, std::string client_id);
    ~TopicMan();
private:
    mqtt::async_client *client;
    mqtt::itoken_ptr conntok;
    action_listener *subListener;
    callback *cb;
};

TopicMan::TopicMan(std::string address, std::string client_id){
    client = new mqtt::async_client(address.c_str(), client_id.c_str());
    subListener = new action_listener("Subscription2");
    cb = new callback(*client, *subListener);
    client->set_callback(*cb);
    
    mqtt::connect_options connOpts;
    connOpts.set_keep_alive_interval(20);
    connOpts.set_clean_session(true);
    conntok = client->connect(connOpts);
    ROS_INFO("Waiting for the connection...");
    conntok->wait_for_completion();
    ROS_INFO("Connected.");

    patrol_msgs::patrol_data patrol_data;
    geometry_msgs::Point pp;
    pp.x = 1; pp.y = 0; patrol_data.positions.push_back(pp);
    pp.x = 8; pp.y = 0; patrol_data.positions.push_back(pp);
    pp.x = 8; pp.y = -2; patrol_data.positions.push_back(pp);
    pp.x = 8; pp.y = 0; patrol_data.positions.push_back(pp);
    pp.x = 5; pp.y = 0; patrol_data.positions.push_back(pp);

    geometry_msgs::Quaternion q;
    q.w = 1;
    patrol_msgs::patrol_node nn;
    nn.orientation = q;
    
    nn.index = 0;
    nn.reading.data = "你好,济南汤尼机器人科技有限公司";
    patrol_data.nodes.push_back(nn);

    nn.index = 2;
    nn.reading.data = "一二三,666666,22222,111111";
    patrol_data.nodes.push_back(nn);

    nn.index = 4;
    nn.reading.data = "再见,111,222,333";
    patrol_data.nodes.push_back(nn);

    patrol_data.name.data = "test2";

    /****** post task****/
    //uint32_t serial_size = ros::serialization::serializationLength(patrol_data);
    //boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
    //ros::serialization::OStream stream(buffer.get(), serial_size);
    //ros::serialization::Serializer<patrol_msgs::patrol_data>::write(stream, patrol_data);
    //printf("serialize ok, size: %d\n", serial_size);
    //client->publish("post_task", (char*)buffer.get(), serial_size, QOS, false)->wait_for_completion(TIMEOUT);
    //printf("pub to topic ok \n");
    
    /****** do task****/
    std_msgs::String send_str;
    //send_str.data = "创客咖啡接待路线一";
    send_str.data = "test2";
    uint32_t serial_size = ros::serialization::serializationLength(send_str);
    boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
    ros::serialization::OStream stream(buffer.get(), serial_size);
    ros::serialization::Serializer<std_msgs::String>::write(stream, send_str);
    printf("serialize ok, size: %d\n", serial_size);
    client->publish("do_task", (char*)buffer.get(), serial_size, QOS, false)->wait_for_completion(TIMEOUT);
    printf("pub to topic ok \n");
    /******************/

}

TopicMan::~TopicMan(){
    ROS_INFO("Disconnecting...");
    conntok = client->disconnect();
    conntok->wait_for_completion();
    ROS_INFO("Disconnected OK.");
}

int main(int argc, char **argv){
    //if(argc != 2){
    //    printf("argument is not enough. example: mqtt_client pub tcp://112.230.206.158:1883\n");
    //    return 1;
    //} 
    //TopicMan topic_man("tcp://112.230.206.158:1883", "test_pub_mqtt_001");
    //TopicMan topic_man("tcp://192.168.100.103:1883", "test_pub_mqtt_001");
    //TopicMan topic_man("tcp://192.168.2.92:1883", "test_pub_mqtt_001");
    TopicMan topic_man("tcp://localhost:1883", "test_pub_mqtt_001");
    
    return 0;
}
