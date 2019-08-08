//histogram_match.cpp
//
//
#include "realtime_histogram_match.hpp"

H_match::H_match(ros::NodeHandle n,ros::NodeHandle private_nh_):
	input_txtfile("/home/amsl/Pictures/ros_catkin_ws/scan_context/save_pr/list.txt")
{	
	pr_num_vis_pub = n.advertise<visualization_msgs::MarkerArray>("/pr/num/vis", 10, true);
	score_vis_pub = n.advertise<std_msgs::Float64MultiArray>("/score/itst/vis", 10);
	score_best_pub = n.advertise<std_msgs::Int32>("/score/itst/best", 10);
	
	pc_sub = n.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1, &H_match::pcCallback, this);
 	pr_trajectory_vis();

	desc = new descriptor();
		
	hist_ope = new histogram_operation();
	hist_ope->read_ref_histogram();

}

std::vector<std::string> 
H_match::split(const std::string &str, char sep)
{
	std::vector<std::string> v;
	std::stringstream ss(str);
	std::string buffer;
	while( getline(ss, buffer, sep) ) {
		v.push_back(buffer);
	}
	return v;
}


visualization_msgs::Marker
H_match::text_vis(const double now_x,const double now_y,const double now_z){
	
	static int num = 0;

	visualization_msgs::Marker m;
	m.header.stamp = ros::Time(0); 
	m.header.frame_id = "/map";
	m.ns = "histogram";
	m.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	m.action = visualization_msgs::Marker::ADD;
	m.lifetime = ros::Duration(0);
	// 形
	m.color.r = 1.0;
	m.color.g = 1.0;
	m.color.b = 1.0;
	m.color.a = 1.0; 

	m.pose.orientation.x = 0.0;
	m.pose.orientation.y = 0.0;
	m.pose.orientation.z = 0.0;
	m.pose.orientation.w = 1.0;
	m.pose.position.x = now_x;
	m.pose.position.y = now_y;	
	m.pose.position.z = now_z;

	m.scale.z = 5.0; 

	m.id = num;
	m.text =  std::to_string(num).substr(0,4);

	num++;

	return m;
}


void
H_match::pr_trajectory_vis(){
	
	std::string reading_line_buffer;
	reading_file.open(input_txtfile, std::ios::in);
	std::vector<std::string> v;
		
	visualization_msgs::MarkerArray m_array;
	std::cout<<"------read pr file --------"<<std::endl;
	while (!reading_file.eof())
	{
		// read by line
		std::getline(reading_file, reading_line_buffer);
		if(!reading_line_buffer.empty()){
			v = split(reading_line_buffer,',');
			std::cout<<atof(v[0].c_str())<<","<<atof(v[1].c_str())<<","<<atof(v[2].c_str())<<std::endl;
			m_array.markers.push_back(text_vis( atof(v[0].c_str()), atof(v[1].c_str()), atof(v[2].c_str()) ));
		}
	}
	pr_num_vis_pub.publish(m_array);

}



void 
H_match::pcCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr input_pc (new pcl::PointCloud<pcl::PointXYZI>);
	std::vector<std::vector<int> > histogram;
	std_msgs::Float64MultiArray score;
	std_msgs::Int32 best_score;

  	pcl::fromROSMsg(*msg, *input_pc);
   
	desc->itst_descriptor(input_pc, histogram);
	hist_ope->match_histogram_pc(histogram, score, best_score);
	
	score_vis_pub.publish(score);
	score_best_pub.publish(best_score);

}
