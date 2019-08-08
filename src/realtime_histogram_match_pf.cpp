//histogram_match.cpp
//
//
#include "realtime_histogram_match_pf.hpp"

H_match_pf::H_match_pf(ros::NodeHandle n,ros::NodeHandle private_nh_):
	input_txtfile("/home/amsl/Pictures/ros_catkin_ws/scan_context/save_pr/list.txt")
{	
	pr_num_vis_pub = n.advertise<visualization_msgs::MarkerArray>("/pr/num/vis", 10, true);
	score_vis_pub = n.advertise<std_msgs::Float64MultiArray>("/score/itst/vis", 10);
	score_best_pub = n.advertise<std_msgs::Int32>("/score/itst/best", 10);

	pf_best_pub = n.advertise<geometry_msgs::PoseStamped>("/pf_cloud/best",10);	//結果の表示のみとか


	pc_sub = n.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1, &H_match_pf::pcCallback, this);
 	pr_trajectory_vis();

	desc = new descriptor();
		
	hist_ope = new histogram_operation();
	hist_ope->read_ref_histogram();

	
	private_nh_.param("PF_RANGE", P_RANGE, {5.0});	//パーティクルの範囲
	private_nh_.param("PF_INTER", P_INTER, {0.5});	//間引く距離
	
	P_NUM = pow(( 1 +  (P_RANGE * 2) / P_INTER), 2.0);

	cout<<"----- Particle Number : "<< P_NUM <<" -----"<<endl;


}

std::vector<std::string> 
H_match_pf::split(const std::string &str, char sep)
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
H_match_pf::text_vis(const double now_x,const double now_y,const double now_z){
	
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
H_match_pf::pr_trajectory_vis(){
	
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
H_match_pf::init_pf(){	//初期化

	descriptor::Particle p;	
	// geometry_msgs::Pose pf_geo;
	pf_cloud.clear();
	for(double pose_x = (-1)*P_RANGE; pose_x<=P_RANGE; pose_x += P_INTER){
		for(double pose_y = (-1)*P_RANGE; pose_y<=P_RANGE; pose_y += P_INTER){
			p.x = pose_x;
			p.y = pose_y;
			p.weight = 0.0;

			pf_cloud.push_back(p);
			
			// pf_array.poses.push_back(pf2geo(p));
		}
	}
	
	// pf_array.header.stamp = ros::Time(0);
	// pf_pub.publish(pf_array);
	cout<<"particle generated"<<endl;

}



void
H_match_pf::pub_pf(descriptor::Particle pf){
	
	geometry_msgs::Pose pf_geo;
	
	pf_geo.position.x = pf.x;
	pf_geo.position.y = pf.y;
	pf_geo.position.z = 0.0;
	pf_geo.orientation.x = 0.0;
	pf_geo.orientation.y = 0.0;
	pf_geo.orientation.z = 0.0;
	pf_geo.orientation.w = 1.0;


	//best_pf
	geometry_msgs::PoseStamped pf_best;
	pf_best.pose = pf_geo;
	pf_best.header.frame_id = "/velodyne";
	pf_best.header.stamp= ros::Time(0);
	pf_best_pub.publish(pf_best);

}




void 
H_match_pf::pcCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr input_pc (new pcl::PointCloud<pcl::PointXYZI>);
	std::vector<std::vector<int> > histogram;
	
	std_msgs::Float64MultiArray score_array;
	std_msgs::Int32 best_score;
	FINAL_ANS final_ans;
	
	double score;

  	pcl::fromROSMsg(*msg, *input_pc);

	init_pf();
  
	int pf_num = 0;
	bool flag = false;
	
	for(auto pf : pf_cloud){
		desc->itst_descriptor(input_pc, histogram, pf);
		hist_ope->match_histogram_pc_pf(histogram, score);
		
		if(!flag){
			final_ans.score = score;
			final_ans.num= pf_num;
			flag = true;
		}
		else{
			if(score > final_ans.score){
				final_ans.score = score;
				final_ans.num = pf_num; 
			}	
		}

		score_array.data.push_back(score);
		pf_num++;
	}
	
	score_vis_pub.publish(score_array);

	best_score.data = final_ans.num;
	score_best_pub.publish(best_score);
	
	pub_pf(pf_cloud[final_ans.num]);

}

