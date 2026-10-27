#include <gazebo/gazebo_client.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <google/protobuf/text_format.h>
#include <iostream>
#include <string>
#include <thread>
#include <chrono>
int main(int argc, char **argv) {
  gazebo::client::setup(argc,argv);
  gazebo::transport::NodePtr node(new gazebo::transport::Node()); node->Init();
  auto pub=node->Advertise<gazebo::msgs::Visual>("~/visual");
  pub->WaitForConnection(); std::cout << "READY" << std::endl;
  std::string line;
  while (std::getline(std::cin,line)) {
    gazebo::msgs::Visual msg;
    if (google::protobuf::TextFormat::ParseFromString(line,&msg) && msg.IsInitialized()) pub->Publish(msg);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  gazebo::client::shutdown();
}
