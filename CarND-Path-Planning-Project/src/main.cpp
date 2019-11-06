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

using namespace std;

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

// must defind M_PI for use in Visual Studio
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

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

  // Car's lane. Starting at middle lane.
  int lane = 1;

  // Reference velocity.
  double ref_vel = 0.0; // mph

  h.onMessage([&ref_vel, &lane, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy]
    (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {

		//ofstream myfile;
		//myfile.open("testing.txt", std::ios_base::app);

    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
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

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          int prev_size = previous_path_x.size();

          if (prev_size > 0) {
            car_s = end_path_s;
          }

					// initialize variables for tracking where other cars are
					bool too_close = false;
					bool left_lane_safe = true;
					bool right_lane_safe = true;

					int other_lane = 0;
					// use a "slow down" variable so we can vary how fast we slow down
					double slow_down = 0.0;

					// loop through other cars on the road
          for ( int i = 0; i < sensor_fusion.size(); i++ ) {
            float d = sensor_fusion[i][6];

						// determine which lane the other car is in
						if (d > 0 && d < 4) {
							other_lane = 0;
						}
						else if (d > 4 && d < 8) {
							other_lane = 1;
						}
						else if (d > 8 && d < 12) {
							other_lane = 2;
						}
						else{
							continue;
						}

						double vx = sensor_fusion[i][3];
						double vy = sensor_fusion[i][4];
						double check_speed = sqrt(vx*vx + vy*vy);
						double check_car_s = sensor_fusion[i][5];
						// if using previous points we can project s value outwards
						check_car_s += ((double)prev_size*.02*check_speed);
						// check s values greater than mine and s gap
						// this check is for cars in front of us
						if ((check_car_s > car_s) && ((check_car_s - car_s) < 30)) {
							// step on the brakes even harder, the other car is getting real close
							// do not want to slow down more than this or else we will exceed jerk limits
							//if (check_car_s - car_s < 10) {
							//	slow_down = .336;
							//}
							// step on the brakes a bit harder, the other car is getting closer
							if (check_car_s - car_s < 20) {
								slow_down = .280;
							}
							// set initial slow down value for when car approaching
							else {
								slow_down = .224;
							}
							// if car is in same lane as us, then need to slow down
							if (lane == other_lane) {
								too_close = true;
							}
							// if car is close and in lane to the left of us, that lane is not safe for moving into
							else if (lane > other_lane) {
								left_lane_safe = false;

							}
							// if car is close and in lane to the right of us, that lane is not safe for moving into
							else if (lane < other_lane) {
								right_lane_safe = false;
							}
						}
						// this check is for cars behind us
						else if ((check_car_s < car_s) && ((car_s - check_car_s) < 30)) {
							// if car is close and in lane to the left of us, that lane is not safe for moving into
								if (lane > other_lane) {
									left_lane_safe = false;
								}
								// if car is close and in lane to the right of us, that lane is not safe for moving into
								else if (lane < other_lane) {
									right_lane_safe = false;
								}
						}
					}

					//myfile << "TooCloseVal:" << too_close << "leftLaneSafe:" << left_lane_safe << " rightLaneSafe:" << right_lane_safe << " lane:" << lane << " other_lane:" << other_lane << endl;

					// if a car is in our lane and too close, look at available options
					if (too_close == true) {
						// actions to take if we are in left lane
						if (lane == 0) {
							if (right_lane_safe == true) {
								lane++;
							}
						}
						// actions to take if we are in center lane
						else if (lane == 1) {
							if (right_lane_safe == true && lane < 2) {
								lane++;
							}
							else if (left_lane_safe == true && lane > 0) {
								lane--;
							}
						}
						// actions to take if we are in right lane
						else if (lane == 2) {
							if (left_lane_safe == true) {
								lane--;
							}
						}
						ref_vel -= slow_down;
					}
					// this is primarily for speeding up when we are first starting. Value is higher than "standard" speed up so we can get going quicker
					else if (ref_vel < 20) {
						ref_vel += 0.75;
					}
					// if we are not too close to another vehicle then speed up
					else if (ref_vel < 49.5) {
						ref_vel += .224;
					}
					// this keeps us from going over the speed limit. Probably need one of these in my car:)
					else if (ref_vel > 49.8){
						ref_vel = 49.8;
					}

        	vector<double> ptsx;
          vector<double> ptsy;

					// create list of widely spaced x,y waypoint, evenly spaced at 30m
					// later, we will interoplate these waypoints with a spline and fill it in with more points that control spline
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // Do I have have previous points
          if ( prev_size < 2 ) {
          	// There are not too many...
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          } else {
            // Use the last two points.
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

					// in Frenet add evenly 30m spaced points ahead of the starting reference
					vector<double> next_wp0 = getXY(car_s + 30, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
					vector<double> next_wp1 = getXY(car_s + 60, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
					vector<double> next_wp2 = getXY(car_s + 90, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

					ptsx.push_back(next_wp0[0]);
					ptsx.push_back(next_wp1[0]);
					ptsx.push_back(next_wp2[0]);

					ptsy.push_back(next_wp0[1]);
					ptsy.push_back(next_wp1[1]);
					ptsy.push_back(next_wp2[1]);

          for ( int i = 0; i < ptsx.size(); i++ ) {
						// shift car reference angle to 0 degrees
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;

						ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y*sin(0 - ref_yaw));
						ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y*cos(0 - ref_yaw));
          }

            // create a spline
          tk::spline s;
					//set (x,y) points to the spline
          s.set_points(ptsx, ptsy);

          // define the actual (x,y) points we will use for the planner
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          for ( int i = 0; i < prev_size; i++ ) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // Calculate distance y position on 30 m ahead.
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x*target_x + target_y*target_y);

          double x_add_on = 0;

					// fill up the rest of our path planner after filling it with previous points, here we will always output 50 points
					for (int i = 1; i <= 50 - prev_size; i++) {
						double N = (target_dist / (.02*ref_vel / 2.24));
						double x_point = x_add_on + (target_x) / N;
						double y_point = s(x_point);

						x_add_on = x_point;

						double x_ref = x_point;
						double y_ref = y_point;

						// rotate back to normal after rotating it earlier
						x_point = (x_ref * cos(ref_yaw) - y_ref*sin(ref_yaw));
						y_point = (x_ref * sin(ref_yaw) + y_ref*cos(ref_yaw));

						x_point += ref_x;
						y_point += ref_y;

						next_x_vals.push_back(x_point);
						next_y_vals.push_back(y_point);


					}

          json msgJson;

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          //this_thread::sleep_for(chrono::milliseconds(1000));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
				ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    } 
  });

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