// ros
#include <urdf/model.h>
#include <collada_urdf/collada_urdf.h>
//#include <ros.h>
//#include <std_msgs/UInt16.h>


int main(int argc, char** argv)
{	
	ros::init(argc, argv, "my_parser");
  	if(argc != 2)
	{
    	ROS_ERROR("Need a urdf file as argument");
    	return -1;
	}
	std::string urdf_file = argv[1];

	urdf::Model model;
	if(!model.initFile(urdf_file))
	{
    	ROS_ERROR("Failed to parse urdf file");
    	return -1;
	}
	ROS_INFO("Successfully parsed urdf file");

	boost::shared_ptr<DAE> dom;
	if(!collada_urdf::colladaFromUrdfModel(my_model, dom))
	{
	    ROS_ERROR("Failed to construct COLLADA DOM");
	    return -1;
	}
	collada_urdf::colladaToFile(dom, urdf_file + ".dae");
	

	return 0;
}
