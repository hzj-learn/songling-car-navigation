#include"../include/base_vel_mux/base_vel_mux.hpp"
using namespace base_vel_mux;


std::string unresolvedName(const std::string &name){
    size_t pos = name.find_last_of('/');
    return name.substr(pos + 1);
}


int main(int argc, char *argv[])
{
 ros::init(argc,argv,"cmd_vel_mux");
 ros::NodeHandle nh("~");

 std::string resloved_name=nh.getUnresolvedNamespace();//getUnresolvedNamespace 返回的是"/node_name"
 // std::string node_name=unresolvedName(resloved_name);//去除"/"

 CmdVelMux cmdvelmux(nh,resloved_name);
 cmdvelmux.init();
 ros::spin();
  return 0;
}



