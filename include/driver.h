#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <spot_leash/hallway_lanes.h>
#include <sstream>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <vector>
#include "amrl_msgs/VisualizationMsg.h"
#include "geometry_msgs/Point.h"
#include "ros/ros.h"
#include "spot_leash/amrl_viz_tools.h"
#include "std_msgs/String.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"



#ifndef DRIVER_H
#define DRIVER_H

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

namespace spot {


    class Driver {
        private:
            MoveBaseClient _ac;
            ros::Publisher _vis_pub;
            tf2_ros::TransformListener tfL;
            tf2_ros::Buffer tfBuffer;

            /* Relating to BWI step state */
            geometry_msgs::Point _next_point;

            /* Used by twist driver for BWI/ROS stack */
            ros::Publisher _twist_pub;

            /* Singletone AMRL Viz message (see visualize method todo) */
            amrl_msgs::VisualizationMsg _amrl_viz_msg;

            ros::Publisher _amrl_viz_pub;

        public:
            Driver(HallwayLanes hallway_lanes, ros::Publisher vis_pub): 
                _ac("move_base", true), _hallway_lanes(hallway_lanes), 
                _vis_pub(vis_pub), tfL(tfBuffer)  {
                    _next_point = hallway_lanes.getLane2().top_point;
                }

            Driver(HallwayLanes hallway_lanes, 
                    ros::Publisher vis_pub, ros::Publisher twist_pub, ros::Publisher amrl_viz_pub,
                    geometry_msgs::Point north_star): 
                _ac("move_base", true), _hallway_lanes(hallway_lanes), 
                _vis_pub(vis_pub), tfL(tfBuffer), _twist_pub(twist_pub), _amrl_viz_pub(amrl_viz_pub) {
                    _amrl_viz_msg = getEmptyAmrlVizMessage();

                    _next_point = hallway_lanes.getLane2().top_point;
                    _north_star = north_star;
                }


            // Get next point if no person is detected
            geometry_msgs::Point getGoalPoint(geometry_msgs::Point base_point);

            /* Execute sense -> act for the spot_leash hallway project */
            bool step();
            bool resetSpin();

            /* Clear amrl messages (see todo below) */
            void flushAmrlViz();

            /* Visualize amrl messages. TODO: Move this to separate visualizer */
            void visualizeAmrl();
    };
}

#endif

