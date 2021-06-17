#include "commonroad/CommonRoad.hpp"
#include "opendrive/OpenDrive.hpp"
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <cmath>
#include <vector>

int main(int argc, char **argv) {
  ros::init(argc, argv, "visualize_lanes");
  ros::NodeHandle n;
  ros::Publisher marker_pub =
      n.advertise<visualization_msgs::MarkerArray>("lane_marker_array", 10);
  // currently publishes every 10 seconds
  ros::Rate r(1);
  ros::NodeHandle nh;

  // get parameters from params.yaml
  std::string pathOD;
  nh.getParam("/pathToOpenDrive", pathOD);

  // create opendrive data structure and load info from OpenDrive File
  opendrive::OpenDriveData odr;
  bool bSuccess = opendrive::Load(pathOD, odr);

  // check if the file was loaded successfully
  if (!bSuccess) {
    ROS_FATAL("Unable to load Map file!");
    exit(0);
  }

  // marker arry to store all individual line strips
  visualization_msgs::MarkerArray line_strips;

  while (ros::ok()) {
    int i = 0;
    int count_road = 0; // TEMP
    // iterate trough all roads in the OpenDrive Road file
    for (const opendrive::RoadInformation &road : odr.roads) {

      // TEMP
      // if(count_road++ != 3) continue;

      // DEBUG
      // ROS_INFO("Road name: %s", road.attributes.name.c_str());

      // if(road.attributes.name != "Road 27") continue;

      //************************************************************************
      // Extract base lines
      //************************************************************************
      // create marker for the originally defined road geometry
      visualization_msgs::Marker lineStripMain;
      lineStripMain.header.frame_id = "map";
      lineStripMain.header.stamp = ros::Time::now();
      lineStripMain.ns = "opendrive_visualize";
      lineStripMain.action = visualization_msgs::Marker::ADD;
      lineStripMain.pose.orientation.w = 1.0;
      lineStripMain.id = i;
      i++;
      lineStripMain.type = visualization_msgs::Marker::LINE_STRIP;

      // STRIP markers use only the x component of scale, for the line width
      lineStripMain.scale.x = 0.5;
      // STRIP is white
      lineStripMain.color.a = 1.0;
      lineStripMain.color.r = 1.0;
      lineStripMain.color.g = 1.0;
      lineStripMain.color.b = 1.0;

      int count_geo = 0;

      // iterate trough all geometries defining the road
      for (const std::unique_ptr<opendrive::GeometryAttributes> &geometry :
           road.geometry_attributes) {

        // TEMP: FOR CHECKING
        // if(count_geo++ == 2) break;

        // check if the geometry is a line
        if (geometry->type == opendrive::GeometryType::LINE) {
          // create a new point to store the start position of the geometry
          geometry_msgs::Point p;
          p.z = 0;
          // line defined by s reference frame
          p.x = geometry->start_position_x;
          p.y = geometry->start_position_y;
          // add the point to the visualization_msgs::Marker::LINE_STRIP
          lineStripMain.points.push_back(p);

          // the loop arrives a the last element in the geometry
          if (road.geometry_attributes.back() == geometry) {
            // calculate the last point of the lineStrip based on line heading
            // and line length
            p.x = geometry->start_position_x +
                  cos(geometry->heading) * geometry->length;
            p.y = geometry->start_position_y +
                  sin(geometry->heading) * geometry->length;
            // add the point to the visualization_msgs::Marker::LINE_STRIP
            lineStripMain.points.push_back(p);
          }
        }
        // check if the geometry is a arc
        else if (geometry->type == opendrive::GeometryType::ARC) {
          // create a new point to store the start position of the geometry
          geometry_msgs::Point p;
          p.z = 0;
          // line defined by s reference frame
          p.x = geometry->start_position_x;
          p.y = geometry->start_position_y;
          // add the point to the visualization_msgs::Marker::LINE_STRIP
          lineStripMain.points.push_back(p);

          // the loop arrives a the last element in the geometry
          if (road.geometry_attributes.back() == geometry) {
            // calculate the last point of the lineStrip based on line heading
            // and line length
            p.x = geometry->start_position_x +
                  cos(geometry->heading) * geometry->length;
            p.y = geometry->start_position_y +
                  sin(geometry->heading) * geometry->length;
            // add the point to the visualization_msgs::Marker::LINE_STRIP
            lineStripMain.points.push_back(p);
          }
        }
      }
      // add the newly created Line Strip to the Marker Array
      ROS_INFO("Road name: %s", road.attributes.name.c_str());
      if (lineStripMain.points.size() > 1) {
        line_strips.markers.push_back(lineStripMain);
      }

      //************************************************************************
      // Extract left lane lines by widths
      //************************************************************************
      // get all width entries in the lane section description
      std::vector<opendrive::LaneWidth> leftDrivingWidths, leftShoulderWidths,
          leftSidewalkWidth;
      for (const opendrive::LaneSection &lane_section :
           road.lanes.lane_sections) {
        for (const opendrive::LaneInfo &lane : lane_section.left) {
          // DEBUG
          ROS_INFO("Lane type: %d", lane.attributes.type);
          for (const opendrive::LaneWidth &width : lane.lane_width) {
            // store the width for later use
            if (lane.attributes.type == opendrive::LaneType::Driving) {
              leftDrivingWidths.push_back(width);
            }
          }
        }
      }

      // create marker for the Marker defined by the lane width and original
      // geometry
      visualization_msgs::Marker leftLineStripWidth;
      leftLineStripWidth.header.frame_id = "map";
      leftLineStripWidth.header.stamp = ros::Time::now();
      leftLineStripWidth.ns = "opendrive_visualize";
      leftLineStripWidth.action = visualization_msgs::Marker::ADD;
      leftLineStripWidth.pose.orientation.w = 1.0;
      leftLineStripWidth.id = i;
      i++;
      leftLineStripWidth.type = visualization_msgs::Marker::LINE_STRIP;

      // STRIP markers use only the x component of scale, for the line width
      leftLineStripWidth.scale.x = 0.5;
      // STRIP is white
      leftLineStripWidth.color.a = 1.0;
      leftLineStripWidth.color.r = 1.0;
      leftLineStripWidth.color.g = 1.0;
      leftLineStripWidth.color.b = 0.0;

      // DEBUG
      // if(leftLineStripWidth.id != 53) continue;

      // iterate trough all geometries defining the road
      for (const std::unique_ptr<opendrive::GeometryAttributes> &geometry :
           road.geometry_attributes) {
        // check if the geometry is a line
        if (geometry->type == opendrive::GeometryType::LINE) {

          // line defined by lane rWidth reference frame
          for (const opendrive::LaneWidth width : leftDrivingWidths) {
            // calculate the difference between width and geometry s position
            double dS = width.soffset - geometry->start_position;
            // in case the difference is smaller 0.1 meter
            // if (abs(dS) < 0.1) {
            // create a new point to store the start position of the geometry
            // + the width vector defined by the heading + pi/2 and the lane
            // width

            // DEBUG
            ROS_INFO("Lane width: %f", width.a);

            geometry_msgs::Point p;
            p.z = 0.0;
            p.x = geometry->start_position_x +
                  cos(geometry->heading + M_PI / 2) * width.a;
            p.y = geometry->start_position_y +
                  sin(geometry->heading + M_PI / 2) * width.a;
            // add to LANE_STRIP
            leftLineStripWidth.points.push_back(p);

            // the loop arrives a the last element in the geometry
            if (road.geometry_attributes.back() == geometry) {
              // calculate the last point of the lineStrip based on
              // line heading and line length + the width vector defined by
              // the heading + pi/2 and the lane width
              p.x = geometry->start_position_x +
                    cos(geometry->heading) * geometry->length +
                    cos(geometry->heading + M_PI / 2) * width.a;
              p.y = geometry->start_position_y +
                    sin(geometry->heading) * geometry->length +
                    sin(geometry->heading + M_PI / 2) * width.a;
              // add to LANE_STRIP
              leftLineStripWidth.points.push_back(p);
            }
            // }
          }
        }
        // check if the geometry is a arc
        else if (geometry->type == opendrive::GeometryType::ARC) {

          // line defined by lane rWidth reference frame
          for (const opendrive::LaneWidth width : leftDrivingWidths) {
            // calculate the difference between width and geometry s position
            double dS = width.soffset - geometry->start_position;
            // in case the difference is smaller 0.1 meter
            // if (abs(dS) < 0.1) {

            // DEBUG
            ROS_INFO("Lane width: %f", width.a);

            // create a new point to store the start position of the geometry
            // + the width vector defined by the heading + pi/2 and the lane
            // width
            geometry_msgs::Point p;
            p.z = 0;
            p.x = geometry->start_position_x +
                  cos(geometry->heading + M_PI / 2) * width.a;
            p.y = geometry->start_position_y +
                  sin(geometry->heading + M_PI / 2) * width.a;
            // add to LANE_STRIP
            leftLineStripWidth.points.push_back(p);

            // the loop arrives a the last element in the geometry
            if (road.geometry_attributes.back() == geometry) {
              // calculate the last point of the lineStrip based on
              // line heading and line length + the width vector defined by
              // the heading + pi/2 and the lane width
              p.x = geometry->start_position_x +
                    cos(geometry->heading) * geometry->length +
                    cos(geometry->heading + M_PI / 2) * width.a;
              p.y = geometry->start_position_y +
                    sin(geometry->heading) * geometry->length +
                    sin(geometry->heading + M_PI / 2) * width.a;
              // add to LANE_STRIP
              leftLineStripWidth.points.push_back(p);
            }
            //}
          }
        }
      }

      // add the newly created Line Strip to the Marker Array
      if (leftLineStripWidth.points.size() > 1) {
        line_strips.markers.push_back(leftLineStripWidth);
      }

      //************************************************************************
      // Extract right lane lines by widths
      //************************************************************************
      // get all width entries in the lane section description
      std::vector<opendrive::LaneWidth> rightDrivingWidths, rightShoulderWidths,
          rightSidewalkWidth;
      for (const opendrive::LaneSection &lane_section :
           road.lanes.lane_sections) {
        for (const opendrive::LaneInfo &lane : lane_section.right) {
          // DEBUG
          ROS_INFO("Lane type: %d", lane.attributes.type);
          for (const opendrive::LaneWidth &width : lane.lane_width) {
            // store the width for later use
            if (lane.attributes.type == opendrive::LaneType::Driving) {
              rightDrivingWidths.push_back(width);
            }
          }
        }
      }

      // create marker for the Marker defined by the lane width and original
      // geometry
      visualization_msgs::Marker rightLineStripWidth;
      rightLineStripWidth.header.frame_id = "map";
      rightLineStripWidth.header.stamp = ros::Time::now();
      rightLineStripWidth.ns = "opendrive_visualize";
      rightLineStripWidth.action = visualization_msgs::Marker::ADD;
      rightLineStripWidth.pose.orientation.w = 1.0;
      rightLineStripWidth.id = i;
      i++;
      rightLineStripWidth.type = visualization_msgs::Marker::LINE_STRIP;

      // STRIP markers use only the x component of scale, for the line width
      rightLineStripWidth.scale.x = 0.5;
      // STRIP is white
      rightLineStripWidth.color.a = 1.0;
      rightLineStripWidth.color.r = 1.0;
      rightLineStripWidth.color.g = 1.0;
      rightLineStripWidth.color.b = 0.0;

      // DEBUG
      // if(rightLineStripWidth.id != 53) continue;

      // iterate trough all geometries defining the road
      for (const std::unique_ptr<opendrive::GeometryAttributes> &geometry :
           road.geometry_attributes) {
        // check if the geometry is a line
        if (geometry->type == opendrive::GeometryType::LINE) {

          // line defined by lane rWidth reference frame
          for (const opendrive::LaneWidth width : rightDrivingWidths) {
            // calculate the difference between width and geometry s position
            double dS = width.soffset - geometry->start_position;
            // in case the difference is smaller 0.1 meter
            // if (abs(dS) < 0.1) {
            // create a new point to store the start position of the geometry
            // + the width vector defined by the heading + pi/2 and the lane
            // width

            // DEBUG
            ROS_INFO("Lane width: %f", width.a);

            geometry_msgs::Point p;
            p.z = 0.0;
            p.x = geometry->start_position_x +
                  cos(geometry->heading + M_PI / 2) * width.a;
            p.y = geometry->start_position_y +
                  sin(geometry->heading + M_PI / 2) * width.a;
            // add to LANE_STRIP
            rightLineStripWidth.points.push_back(p);

            // the loop arrives a the last element in the geometry
            if (road.geometry_attributes.back() == geometry) {
              // calculate the last point of the lineStrip based on
              // line heading and line length + the width vector defined by
              // the heading + pi/2 and the lane width
              p.x = geometry->start_position_x +
                    cos(geometry->heading) * geometry->length +
                    cos(geometry->heading + M_PI / 2) * width.a;
              p.y = geometry->start_position_y +
                    sin(geometry->heading) * geometry->length +
                    sin(geometry->heading + M_PI / 2) * width.a;
              // add to LANE_STRIP
              rightLineStripWidth.points.push_back(p);
            }
            // }
          }
        }
        // check if the geometry is a arc
        else if (geometry->type == opendrive::GeometryType::ARC) {

          // line defined by lane rWidth reference frame
          for (const opendrive::LaneWidth width : rightDrivingWidths) {
            // calculate the difference between width and geometry s position
            double dS = width.soffset - geometry->start_position;
            // in case the difference is smaller 0.1 meter
            // if (abs(dS) < 0.1) {

            // DEBUG
            ROS_INFO("Lane width: %f", width.a);

            // create a new point to store the start position of the geometry
            // + the width vector defined by the heading + pi/2 and the lane
            // width
            geometry_msgs::Point p;
            p.z = 0;
            p.x = geometry->start_position_x +
                  cos(geometry->heading + M_PI / 2) * width.a;
            p.y = geometry->start_position_y +
                  sin(geometry->heading + M_PI / 2) * width.a;
            // add to LANE_STRIP
            rightLineStripWidth.points.push_back(p);

            // the loop arrives a the last element in the geometry
            if (road.geometry_attributes.back() == geometry) {
              // calculate the last point of the lineStrip based on
              // line heading and line length + the width vector defined by
              // the heading + pi/2 and the lane width
              p.x = geometry->start_position_x +
                    cos(geometry->heading) * geometry->length +
                    cos(geometry->heading + M_PI / 2) * width.a;
              p.y = geometry->start_position_y +
                    sin(geometry->heading) * geometry->length +
                    sin(geometry->heading + M_PI / 2) * width.a;
              // add to LANE_STRIP
              rightLineStripWidth.points.push_back(p);
            }
            //}
          }
        }
      }

      // add the newly created Line Strip to the Marker Array
      if (rightLineStripWidth.points.size() > 1) {
        line_strips.markers.push_back(rightLineStripWidth);
      }
    }

    // publish the Marker Array
    marker_pub.publish(line_strips);
    // clear the Marker Array for next loop
    line_strips.markers.clear();
    r.sleep();
  }
}
