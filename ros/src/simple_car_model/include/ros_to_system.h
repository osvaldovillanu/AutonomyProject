#ifndef _ROS_TO_SYSTEM_SIMPLE_H_
#define _ROS_TO_SYSTEM_SIMPLE_H_

#include "simple_car_model/VehicleState.h"


namespace simple_car{

    void model_to_state( simple_car_model::VehicleState& veh_state,
			 BaseVehicle veh_model ){

	veh_state.pos.x			= veh_model.pos.x;
	veh_state.pos.y			= veh_model.pos.y;
	veh_state.steering_angle	= veh_model.steering_angle;
	veh_state.vehicle_angle		= veh_model.vehicle_angle;
	//veh_state.vehicle_width		= veh_model.width;
	//veh_state.vehicle_length	= veh_model.length;
	veh_state.linear_vel		= veh_model.linear_vel;
	veh_state.steering_angle_vel	= veh_model.steering_angle_vel;
	}


    void state_to_model( BaseVehicle& veh_model,
			 simple_car_model::VehicleState veh_state ){

	veh_model.pos.x			= veh_state.pos.x;
	veh_model.pos.y			= veh_state.pos.y;
	veh_model.steering_angle	= veh_state.steering_angle;
	veh_model.vehicle_angle		= veh_state.vehicle_angle;
	//veh_model.width			= veh_state.vehicle_width;
	//veh_model.length		= veh_state.vehicle_length;
	veh_model.linear_vel		= veh_state.linear_vel;
	veh_model.steering_angle_vel	= veh_state.steering_angle_vel;

    }


};

#endif
