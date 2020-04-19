// Vehicle Controller
#include <cmath>
#include "math.h"
#include "ros/ros.h"
#include "ros_to_system.h"
#include "BaseVehicle.hpp"
#include "simple_car_model/VehicleMoveCommand.h"
#include "simple_car_model/Waypoint.h"
#include "simple_car_model/Trajectory.h"

#define PI 3.141592653589793
#define RAD_TO_DEG (180.0/PI)
#define DEG_TO_RAD (PI/180.0)

#define RATE (10)
float x_error;
float y_error;
float velocity_error;
float angle_error;


#define Kp_lin_vel 1.0;
#define Kp_ang_vel 1.0;

BaseVehicle vehicle;
simple_car_model::Waypoint wpt_tracking_goal;
BaseVehicle wpt_tracking_goal_state;
BaseVehicle* wpt_tracking_vehicle;
//float wpt_tracking_goal = 0.0;

void move_command_callback( simple_car_model::VehicleMoveCommand move_command){
    std::cout << "Got move command\n";
    std::cout << "  Move Command: vel" << move_command.linear_vel << "\n";
    std::cout << "  Move Command: steering vel" << move_command.steering_angle_vel << "\n";

    vehicle.linear_vel = move_command.linear_vel;
    vehicle.steering_angle_vel = move_command.steering_angle_vel;
}

void veh_state_callback( simple_car_model::VehicleState veh_state ){
    //simple_car::state_to_model( vehicle, veh_state );
}

void tracked_goal_callback( simple_car_model::Waypoint wpnt ){
    std::cout << "Got tracked goal\n";
    wpt_tracking_goal = wpnt;
}


int main( int argc, char** argv){

    ros::init(argc, argv, "vehicle_controller");
    ros::NodeHandle node_handle;
    ros::Subscriber sub_command = node_handle.subscribe("vehicle_move_command", 1000, move_command_callback);
    ros::Publisher move_command_pub = node_handle.advertise<simple_car_model::VehicleMoveCommand>( "vehicle_move_command", 1 );
    ros::Subscriber sub = node_handle.subscribe("vehicle_state", 1, veh_state_callback);
    ros::Subscriber sub_tracked = node_handle.subscribe("tracked_goal", 1, tracked_goal_callback);



    ros::Rate rate(RATE);

    while(ros::ok() ){
        //if( !wpt_tracking_goal ){
            simple_car::state_to_model( *wpt_tracking_vehicle, wpt_tracking_goal->state );
            simple_car_model::VehicleMoveCommand move_command;
            x_error = sub_tracked.pos.x - sub.pos.x;
            y_error = sub_tracked.pos.y - sub.pos.y;
            velocity_error = sqtr( pow( x_error , 2.0 ) + pow( y_error , 2.0 ) );
            move_command.linear_vel = Kp_lin_vel * velocity_error;

            angle_error = atan2( y_error , x_error ) * RAD_TO_DEG;
            move_command.steering_angle_vel = Kp_ang_vel * angle_error;

            move_command_pub.publish( move_command );
        //}



        //if (sub_command.linear_vel_error){

       // }
       // else {

       // }




//es(1).x1 = error(1);
  //es(1).x = setpoint1(end) - theta1;
  //es(1).xd = theta1dot;
  //error(1) = es(1).x;
//function u = PDcontroller(gains, e)
   // u = gains.kp * e.x - gains.kd * e.xd + gains.ki * e.x1;
//end

    //move_command_pub.publish( move_command );

	ros::spinOnce();
	rate.sleep();
    }

    return 0;
}
