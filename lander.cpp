// Mars lander simulator
// Version 1.10
// Mechanical simulation functions
// Gabor Csanyi and Andrew Gee, August 2017

// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation, to make use of it
// for non-commercial purposes, provided that (a) its original authorship
// is acknowledged and (b) no modified versions of the source code are
// published. Restriction (b) is designed to protect the integrity of the
// exercise for future generations of students. The authors would be happy
// to receive any suggested modifications by private correspondence to
// ahg@eng.cam.ac.uk and gc121@eng.cam.ac.uk.

#include "lander.h"
#include <cmath>
#include <iostream>

void autopilot1 (void)
  // Autopilot to adjust the engine throttle, parachute and attitude control
{

  // Using the control theory given in the handout, we derive a final equation that governs the throttle.
  // It is dependent on three values --> delta_PID, K_h and K_p.
  // Note: Using the definition of the vector3d position, we can define the unit radial vector as position.norm()

  // Define constants
  double delta_PID, K_h, K_p, P_out, CURRENT_LANDER_MASS, CURRENT_LANDER_WEIGHT;
  
  CURRENT_LANDER_MASS = UNLOADED_LANDER_MASS + fuel * FUEL_CAPACITY * FUEL_DENSITY;
  CURRENT_LANDER_WEIGHT = ((GRAVITY * MARS_MASS * CURRENT_LANDER_MASS) / (position.abs2()));

  delta_PID = CURRENT_LANDER_WEIGHT / MAX_THRUST; // Needs to balance the weight of the lander when P_out == 0.
  K_h = 0.017;
  K_p = 0.2;
  P_out = - K_p * (0.5 + K_h * (position.abs()-MARS_RADIUS) + velocity * position.norm());
  
  if (P_out <= -delta_PID) {
    throttle = 0;
  } else if (P_out >= (1-delta_PID)) {
    throttle =1;
  } else {
    throttle = delta_PID + P_out;
  }

  if ((position.abs()-MARS_RADIUS) <= 5000 && velocity.abs() <= 200.0) {
    parachute_status = DEPLOYED;
  }
}

void autopilot (void)
{
  double delta_PID, K_d, K_p, P_out, CURRENT_LANDER_MASS, CURRENT_LANDER_WEIGHT, target_altitude, speed;

  CURRENT_LANDER_MASS = UNLOADED_LANDER_MASS + fuel * FUEL_CAPACITY * FUEL_DENSITY;
  CURRENT_LANDER_WEIGHT = ((GRAVITY * MARS_MASS * CURRENT_LANDER_MASS) / (position.abs2()));

  delta_PID = CURRENT_LANDER_WEIGHT / MAX_THRUST; // Needs to balance the weight of the lander when P_out == 0.
  K_d = 0.0;
  K_p = 1;
  target_altitude = 500.0;
  speed = -velocity*position.norm(); // To find the derivative of altitude, resolve velocity in the direction of position
  P_out = K_p * (target_altitude - (position.abs()-MARS_RADIUS)) + K_d * speed;
  
  if (P_out <= -delta_PID) {
    throttle = 0;
  } else if (P_out >= (1-delta_PID)) {
    throttle =1;
  } else {
    throttle = delta_PID + P_out;
  }
}

void numerical_dynamics (void)
  // This is the function that performs the numerical integration to update the
  // lander's pose. The time step is delta_t (global variable).
{
  // INSERT YOUR CODE HERE
  static vector3d old_position;
  vector3d F_gravity, F_thrust, F_drag_lander, F_drag_chute, F_drag, F_total, acceleration, new_position, empty3d;
  double CURRENT_LANDER_MASS, AREA_LANDER, AREA_CHUTE;
  
  CURRENT_LANDER_MASS = UNLOADED_LANDER_MASS + fuel * FUEL_CAPACITY * FUEL_DENSITY;
  AREA_LANDER = 3.141592654 * (pow(LANDER_SIZE, 2));
  AREA_CHUTE = 5 * (pow(2.0*LANDER_SIZE, 2));
  
  F_gravity = -position.norm() * ((GRAVITY * MARS_MASS * CURRENT_LANDER_MASS) / (position.abs2()));
  
  F_thrust = thrust_wrt_world();
  
  F_drag_lander = -velocity.norm() * (0.5 * atmospheric_density(position) * DRAG_COEF_LANDER * AREA_LANDER * velocity.abs2());
  if (parachute_status == DEPLOYED) {
    F_drag_chute = -velocity.norm() * (0.5 * atmospheric_density(position) * DRAG_COEF_CHUTE * AREA_CHUTE * velocity.abs2());
    F_drag = F_drag_lander + F_drag_chute;
  } else {
    F_drag = F_drag_lander;
  }

  F_total = F_gravity + F_thrust + F_drag;

  acceleration = F_total / CURRENT_LANDER_MASS;

  // Euler integration
  /*
  position = position + delta_t * velocity;
  velocity = velocity + delta_t * acceleration;  
  */

  // Verlet integration
  // To cater to the extra position value needed, set up 2 other vector3d. If old_position == empty3d, then we know that old_position has just been initialised,
  // and so, this is the first time step. Alternatively, just check if simulation_time == 0.0.
  // NOTE: First method does not work. Not sure why.
  
  if (simulation_time == 0.0) {
    new_position = position + delta_t * velocity;
    velocity = velocity + delta_t * acceleration;
  } else {
    new_position = 2 * position - old_position + acceleration * delta_t * delta_t;
    velocity = (new_position - old_position) / (2 * delta_t); 
  }

  old_position = position;
  position = new_position;

  // Here we can apply an autopilot to adjust the thrust, parachute and attitude
  if (autopilot_enabled) autopilot();

  // Here we can apply 3-axis stabilization to ensure the base is always pointing downwards
  if (stabilized_attitude) attitude_stabilization();
}

void initialize_simulation (void)
  // Lander pose initialization - selects one of 10 possible scenarios
{
  // The parameters to set are:
  // position - in Cartesian planetary coordinate system (m)
  // velocity - in Cartesian planetary coordinate system (m/s)
  // orientation - in lander coordinate system (xyz Euler angles, degrees)
  // delta_t - the simulation time step
  // boolean state variables - parachute_status, stabilized_attitude, autopilot_enabled
  // scenario_description - a descriptive string for the help screen

  scenario_description[0] = "circular orbit";
  scenario_description[1] = "descent from 10km";
  scenario_description[2] = "elliptical orbit, thrust changes orbital plane";
  scenario_description[3] = "polar launch at escape velocity (but drag prevents escape)";
  scenario_description[4] = "elliptical orbit that clips the atmosphere and decays";
  scenario_description[5] = "descent from 200km";
  scenario_description[6] = "descent from 500m";
  scenario_description[7] = "descent from 510m";
  scenario_description[8] = "descent from 700m";
  scenario_description[9] = "";

  switch (scenario) {

  case 0:
    // a circular equatorial orbit
    position = vector3d(1.2*MARS_RADIUS, 0.0, 0.0);
    velocity = vector3d(0.0, -3247.087385863725, 0.0);
    orientation = vector3d(0.0, 90.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 1:
    // a descent from rest at 10km altitude
    position = vector3d(0.0, -(MARS_RADIUS + 10000.0), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = false;
    break;

  case 2:
    // an elliptical polar orbit
    position = vector3d(0.0, 0.0, 1.2*MARS_RADIUS);
    velocity = vector3d(3500.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 3:
    // polar surface launch at escape velocity (but drag prevents escape)
    position = vector3d(0.0, 0.0, MARS_RADIUS + LANDER_SIZE/2.0);
    velocity = vector3d(0.0, 0.0, 5027.0);
    orientation = vector3d(0.0, 0.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 4:
    // an elliptical orbit that clips the atmosphere each time round, losing energy
    position = vector3d(0.0, 0.0, MARS_RADIUS + 100000.0);
    velocity = vector3d(4000.0, 0.0, 0.0);
    orientation = vector3d(0.0, 90.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 5:
    // a descent from rest at the edge of the exosphere
    position = vector3d(0.0, -(MARS_RADIUS + EXOSPHERE), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = false;
    break;

  case 6:
    // a descent from rest at 500m altitude
    position = vector3d(0.0, -(MARS_RADIUS + 500.0), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.01;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = true;
    break;

  case 7:
    // a descent from rest at 510m altitude
    position = vector3d(0.0, -(MARS_RADIUS + 510.0), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.01;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = true;
    break;

  case 8:
    // a descent from rest at 700m altitude
    position = vector3d(0.0, -(MARS_RADIUS + 700.0), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.01;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = true;
    break;

  case 9:
    break;

  }
}
