#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include "params.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  // Start on lane 1 (the middle lane)
  int lane = 1;

  // Target velocity for the car (mph)
  double ref_vel = 0.0; //it starts as 0 so it can be gradually (i.e. comfortably) incremented

  h.onMessage([&lane,&ref_vel,&map_waypoints_x,&map_waypoints_y,
               &map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          int prev_size = previous_path_x.size();

          if (prev_size > 0){
            car_s = end_path_s;
          }

          bool too_close = false; // Flag to tell if car is too close to the one in front
          bool critically_close = false; // Flag to tell if car is critically close to the one in front
          bool prepare_for_change_lane = false; // Flag to tell if the car should try to change lanes
          bool change_lane_left = false; // Flag to change to the left lane
          bool change_lane_right = false; // Flag to change to the right lane

          // Vectors to store the cars that the ego vehicle sees on its right and left lanes
          vector<vector<double>> left_sensed_cars = {};
			    vector<vector<double>> right_sensed_cars = {};

          // Find ref_v to use
          // Loop through all the cars detected by the sensors on the same side of the road
          for (unsigned int i=0; i<sensor_fusion.size(); i++){
            float d = sensor_fusion[i][6];
            // if the car is in my lane
            if (d_in_lane(d, lane, lane_width)){
              //  Check the ith car's speed and s position
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx*vx + vy*vy);
              double check_car_s = sensor_fusion[i][5];

              // if using previous path points, project the ith car s position in time
              check_car_s += (double)prev_size*0.02*check_speed;

              // Check if the ith car is in front of me and closer than a minimum safety distance
              if ((check_car_s > car_s) && (check_car_s - car_s < min_distance)){
                // Activate flag to reduce the ego car's target velocity, so it doesn't crash with the ith car.
                too_close = true;
                // Tell the car to look for possible lane changes
                prepare_for_change_lane = true;

                // Check if the ith car is closer than the critical safety distance
                if (check_car_s - car_s < min_critical_distance){
                  critically_close = true;
                }
              }
            } else if ( (lane > 0) && d_in_lane(d, lane-1, lane_width) ){
              // If the ith car is at the left, add it to this vector
              left_sensed_cars.push_back(sensor_fusion[i]);
              
            } else if ( (lane < 2) && d_in_lane(d, lane+1, lane_width) ){
              // If the ith car is at the right, add it to this vector
              right_sensed_cars.push_back(sensor_fusion[i]);
            }
          }

          // If the 'critically_close' or 'too_close' flags are active, reduce the car's velocity by 
          // the determined acceleration.
          // If not, and the car's speed is less than the maximum, speed up
          if (critically_close){
            ref_vel -= acc_critical;
          } else {
            if (too_close){
            ref_vel -= acc;
            } else if (ref_vel < max_speed){
            ref_vel += acc;
            }
          }
          
          // If the flag to prepare for change lane is active, the ego vehicle will look for the chance of getting on another line
          if (prepare_for_change_lane){
            if (left_sensed_cars.size() == 0){
              change_lane_left = true; // Automatically true if there is not any car on the left lane.
            } else {
              change_lane_left = true;
              for (unsigned int j=0; j<left_sensed_cars.size(); ++j){
                float d = left_sensed_cars[j][6];
                double vx = left_sensed_cars[j][3];
                double vy = left_sensed_cars[j][4];
                double check_speed = sqrt(vx*vx + vy*vy);
                double check_car_s = left_sensed_cars[j][5];

                // if using previous path points, project the ith car s position in time
                check_car_s += (double)prev_size*0.02*check_speed;

                // Check if the ith car is closer than a minimum lane change distance (both in in front or behind)
                if (fabs(check_car_s - car_s) < min_change_distance){
                  change_lane_left = false;
                }
              }
            }

            if (right_sensed_cars.size() == 0){
              change_lane_right = true; // Automatically true if there is not any car on the right lane.
            } else {
              change_lane_right = true;
              for (unsigned int j=0; j<right_sensed_cars.size(); ++j){
                float d = right_sensed_cars[j][6];
                double vx = right_sensed_cars[j][3];
                double vy = right_sensed_cars[j][4];
                double check_speed = sqrt(vx*vx + vy*vy);
                double check_car_s = right_sensed_cars[j][5];

                // if using previous path points, project the ith car s position in time
                check_car_s += (double)prev_size*0.02*check_speed;

                // Check if the ith car is closer than a minimum lane change distance (both in in front or behind)
                if (fabs(check_car_s - car_s) < min_change_distance){
                  change_lane_right = false;
                }
              }
            }
          }

          // Change the target lane, if any of the conditions is met.
          if (change_lane_left && lane > 0){
            lane -= 1;
            prepare_for_change_lane = false;
          } else if (change_lane_right && lane < 2){
            prepare_for_change_lane = false;
            lane += 1;
          } else {
            prepare_for_change_lane = false;
          }           
          
          // Restore the default value of the flags
          change_lane_left = false;
          change_lane_right = false;

          // Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
          vector<double> ptsx;
          vector<double> ptsy;

          // reference x,y and yaw values
          // either we will reference the starting poing as where the car is or at the previous path's end point
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // if prev_size is almost empty, use the car as starting reference
          if (prev_size < 2){
            // Use 2 points that make the path tangent to the car
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          } else {
            // Redefine reference state as previous path's end point
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];

            double ref_x_prev = previous_path_x[prev_size - 2];
            double ref_y_prev = previous_path_y[prev_size - 2];
            ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);
            
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }

          // Add evenly 30m spaced frenet coords points ahead of the starting reference
          vector<double> next_wp0 = getXY(car_s+30,(2+lane_width*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+60,(2+lane_width*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+90,(2+lane_width*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          for (int i=0; i<ptsx.size(); i++){
            // shift car reference angle to 0 dgrees
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;

            ptsx[i] = (shift_x*cos(0-ref_yaw) - shift_y*sin(0-ref_yaw));
            ptsy[i] = (shift_x*sin(0-ref_yaw) + shift_y*cos(0-ref_yaw));
          }

          // Create a spline
          tk::spline sp;

          // set (x,y) points to the spline
          sp.set_points(ptsx, ptsy);

          // Define the (x,y) points that will be used for the planner
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // Starts with all of the (unused) previous path points from last iteration
          for (int i=0; i<prev_size; i++){
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // Calculate how to break up spline points so that we travel at our desired reference velocity
          double target_x = 30.0;
          double target_y = sp(target_x);
          double target_dist = sqrt(target_x*target_x + target_y*target_y);

          double x_add_on = 0;
          // Number of steps to take to cover the target distance at the desired speed
          double N = target_dist/(0.02*ref_vel/2.24); //MPH to m/s

          // Fill up the rest of the path planner after filling it with previous points.
          // It will always output 50 points (this can be parameterizable)
          for (int i=1; i<=num_steps-prev_size; i++){
            
            double x_point = x_add_on + target_x/N;
            double y_point = sp(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            // Rotate back to normal after rotating it earlier
            x_point = x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw);
            y_point = x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw);

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

          json msgJson;
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}