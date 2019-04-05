/*
 * params.h
 *
 * This script establishes some parameters for running the simmulator.
 *
 * There was some tuning involved with some of these parameters, to find the best performance of the car.
 * ---------------------------------------------------------------------
 *
 */

#ifndef PARAMS_H
#define PARAMS_H

// Width of the lanes (m)
const double lane_width		= 4.0;		

// Minimum safety distance (m) with respect to the car in the front
const double min_distance	= 30.0;

// Minimum safety distance (m) to consider when changing lanes
const double min_change_distance = 15.0;

// Minimum critical distance (m). The car must avoid at all cost to come closer than this to cars in the front 
const double min_critical_distance = 10.0;

// Maximum speed limit (mph). Ideally, the ego car's speed should always be around this value
const double max_speed	= 48.5;		// max reference speed in the limit	(mph)

// Number of future points to plan ahead for the car
const int num_steps = 50;

//Amount of velocity change to the car's speed at a single time interval
const float acc = 0.224;   // 5 m/s^2 aprox.

//Amount of velocity change to the car's speed at critical moments
const float acc_critical = 0.4;   // 

#endif