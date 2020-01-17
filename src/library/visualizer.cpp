#include <visualization_msgs/Marker.h>
#include <visualizer.h>


void visualizer::vis_split_pc(ros::NodeHandle n, ros::Publisher split_pc_pub,
                              std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr > split_pc)
{
  sensor_msgs::PointCloud2 output_pc;
  pcl::PointCloud<pcl::PointXYZI>::Ptr output_pc_ (new pcl::PointCloud<pcl::PointXYZI>);

  for(size_t i = 0; i < split_pc.size(); i++){
    for(size_t j = 0; j < split_pc[i] -> points.size(); j++){
      pcl::PointXYZI temp; 
      temp.x = split_pc[i] -> points[j].x;
      temp.y = split_pc[i] -> points[j].y;
      temp.z = split_pc[i] -> points[j].z;
      // temp.intensity = i * 12 + 10;
      int intensity_temp = i * 26 + 10;
      if(intensity_temp > 256){
        intensity_temp -= 256;
      }
      temp.intensity = intensity_temp;
      
      output_pc_ -> points.push_back(temp);
    }
  }

  toROSMsg(*output_pc_, output_pc);
  output_pc.header.frame_id = frame_id_split_pc;
  output_pc.header.stamp = ros::Time::now();
  split_pc_pub.publish(output_pc);
}



void visualizer::vis_split_pc(ros::NodeHandle n, ros::Publisher split_pc_pub,
                              std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr > split_pc)
{
  sensor_msgs::PointCloud2 output_pc;
  pcl::PointCloud<pcl::PointXYZI>::Ptr output_pc_ (new pcl::PointCloud<pcl::PointXYZI>);

  for(size_t i = 0; i < split_pc.size(); i++){
    for(size_t j = 0; j < split_pc[i] -> points.size(); j++){
      pcl::PointXYZI temp; 
      temp.x = split_pc[i] -> points[j].x;
      temp.y = split_pc[i] -> points[j].y;
      temp.z = split_pc[i] -> points[j].z;
      // temp.intensity = i * 12 + 10;
      int intensity_temp = i * 26 + 10;
      if(intensity_temp > 256){
        intensity_temp -= 256;
      }
      temp.intensity = intensity_temp;
      
      output_pc_ -> points.push_back(temp);
    }
  }

  toROSMsg(*output_pc_, output_pc);
  output_pc.header.frame_id = frame_id_split_pc;
  output_pc.header.stamp = ros::Time::now();
  split_pc_pub.publish(output_pc);
}



void visualizer::vis_RF(ros::NodeHandle n, ros::Publisher RF_pub,
                        std::vector<Eigen::Vector3d> eigenvector)
{
  geometry_msgs::Point start;
  start.x = 0.0;
  start.y = 0.0;
  start.z = 0.0;
  
  geometry_msgs::Point end;
  end.x = 2 * eigenvector[0](0);
  end.y = 2 * eigenvector[0](1);
  end.z = 2 * eigenvector[0](2);

  visualization_msgs::Marker RFmarker;
  RFmarker.header.frame_id = frame_id_RF;
  RFmarker.header.stamp = ros::Time::now();
  RFmarker.ns = "RF";
  RFmarker.id = 0;
  RFmarker.lifetime = ros::Duration();
  RFmarker.type = visualization_msgs::Marker::ARROW;
  RFmarker.action = visualization_msgs::Marker::ADD;
  RFmarker.points.push_back(start);
  RFmarker.points.push_back(end);
  RFmarker.scale.x = 0.1;
  RFmarker.scale.y = 0.2;
  RFmarker.scale.z = 0.3;
  RFmarker.color.r = 1.0f;
  RFmarker.color.g = 0.0f;
  RFmarker.color.b = 0.0f;
  RFmarker.color.a = 1.0f;
  RF_pub.publish(RFmarker);

  end.x = 2.0 * eigenvector[1](0);
  end.y = 2.0 * eigenvector[1](1);
  end.z = 2.0 * eigenvector[1](2);
  visualization_msgs::Marker RFmarker1;
  RFmarker1.header.frame_id = frame_id_RF;
  RFmarker1.header.stamp = ros::Time::now();
  RFmarker1.ns = "RF";
  RFmarker1.id = 1;
  RFmarker1.lifetime = ros::Duration();
  RFmarker1.type = visualization_msgs::Marker::ARROW;
  RFmarker1.action = visualization_msgs::Marker::ADD;
  RFmarker1.points.push_back(start);
  RFmarker1.points.push_back(end);
  RFmarker1.scale.x = 0.1;
  RFmarker1.scale.y = 0.2;
  RFmarker1.scale.z = 0.3;
  RFmarker1.color.r = 0.0f;
  RFmarker1.color.g = 1.0f;
  RFmarker1.color.b = 0.0f;
  RFmarker1.color.a = 1.0f;
  RF_pub.publish(RFmarker1);

  end.x = 2.0 * eigenvector[2](0);
  end.y = 2.0 * eigenvector[2](1);
  end.z = 2.0 * eigenvector[2](2);
  visualization_msgs::Marker RFmarker2;
  RFmarker2.header.frame_id = frame_id_RF;
  RFmarker2.header.stamp = ros::Time::now();
  RFmarker2.ns = "RF";
  RFmarker2.id = 2;
  RFmarker2.lifetime = ros::Duration();
  RFmarker2.type = visualization_msgs::Marker::ARROW;
  RFmarker2.action = visualization_msgs::Marker::ADD;
  RFmarker2.points.push_back(start);
  RFmarker2.points.push_back(end);
  RFmarker2.scale.x = 0.1;
  RFmarker2.scale.y = 0.2;
  RFmarker2.scale.z = 0.3;
  RFmarker2.color.r = 0.0f;
  RFmarker2.color.g = 0.0f;
  RFmarker2.color.b = 1.0f;
  RFmarker2.color.a = 1.0f;
  RF_pub.publish(RFmarker2);
}

