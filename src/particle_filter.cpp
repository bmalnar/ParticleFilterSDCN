/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

   // Initialize the number of particles, M

   num_particles = NUM_PARTICLES;

   // Initialize standard deviations for position params

   std_x = std[0];
   std_y = std[1];
   std_theta = std[2];

   // Create normal distributions for x, y, and theta

   normal_distribution<double> distribution_x(x, std_x);
   normal_distribution<double> distribution_y(y, std_y);
   normal_distribution<double> distribution_theta(theta, std_theta);

   // Create M particles, normal distribution with GPS values as mean

   for (int i = 0; i < num_particles; i++) { 

      Particle p;
      p.id = i;
      p.x = distribution_x(rand_engine);
      p.y = distribution_y(rand_engine);
      p.theta = distribution_theta(rand_engine);
      p.weight = 1.0;

      particles.push_back(p);
   }

   // Set the flag to true

   is_initialized = true;   
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

   // Create normal distributions for x, y, and theta, for the zero mean
   // We will use this to add noise 

   printDebug("Enter ParticleFilter::prediction");

   normal_distribution<double> dist_x_zero_mean(0, std_x);
   normal_distribution<double> dist_y_zero_mean(0, std_y);
   normal_distribution<double> dist_theta_zero_mean(0, std_theta);

   // Predict the new state for all the particles

   for (int i = 0; i < num_particles; i++) { 

      Particle p = particles[i];
      double theta = particles[i].theta;

      // Check the magnitude of yaw rate,a nd if it is near zero, 
      // ignore the velocity/yaw_rate ration, otherwise do not ingore it

      if (fabs(yaw_rate) < EPS) { 

         particles[i].x += velocity * delta_t * cos(theta);
         particles[i].y += velocity * delta_t * sin(theta);
         // particles[i].theta does not change if yaw_rate is small enough to ignore

      } else {

         particles[i].x += velocity / yaw_rate * (sin(theta + yaw_rate * delta_t) - sin(theta));
         particles[i].y += velocity / yaw_rate * (cos(theta) - cos(theta + yaw_rate * delta_t));
         particles[i].theta += yaw_rate * delta_t;
      }

      // Add some noise

      particles[i].x += dist_x_zero_mean(rand_engine);
      particles[i].y += dist_y_zero_mean(rand_engine);
      particles[i].theta += dist_theta_zero_mean(rand_engine);
   }  

   printDebug("Exit ParticleFilter::prediction");
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

   for (unsigned int i = 0; i < observations.size(); i++) { 

      double min_dist = numeric_limits<double>::max();
      int id = -1;

      for (unsigned int j = 0; j < predicted.size(); j++) { 

         double x_dist = observations[i].x - predicted[j].x;
         double y_dist = observations[i].y - predicted[j].y;

         double dist = x_dist * x_dist + y_dist * y_dist;

         if (dist < min_dist) { 

            min_dist = dist;
            id = predicted[j].id;
         }
      }

      observations[i].id = id;
   }
   printDebug("Exit ParticleFilter::dataAssociation");
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

   // Cache the square of sensor range for distance comparison

   double sensor_range_sq = sensor_range * sensor_range;

   for (int i = 0; i < num_particles; i++) { 

      // Get the particle params

      double p_x = particles[i].x;
      double p_y = particles[i].y;
      double p_theta = particles[i].theta;

      // This vector will only hold landmarks that are within 
      // the sensor range of the particle
  
      vector<LandmarkObs> landmarks_in_range;

      for (unsigned int j = 0; j < map_landmarks.landmark_list.size(); j++) {

         float lm_x = map_landmarks.landmark_list[j].x_f;
         float lm_y = map_landmarks.landmark_list[j].y_f;

         double dist_x  = p_x - lm_x;
         double dist_y  = p_y - lm_y;
         double dist_sq = dist_x * dist_x + dist_y * dist_y;

         // Add the landmark to observations if within range of the sensor

         if (dist_sq <= sensor_range_sq) {

            int lm_id = map_landmarks.landmark_list[j].id_i;

            landmarks_in_range.push_back(LandmarkObs{lm_id, lm_x, lm_y});
         }
      }

      // Transform observations to map coordinates from vehicle coordinates

      vector<LandmarkObs> observations_transformed;

      for (unsigned int j = 0; j < observations.size(); j++) {

         double x_trans = cos(p_theta) * observations[j].x - sin(p_theta) * observations[j].y + p_x;
         double y_trans = sin(p_theta) * observations[j].x + cos(p_theta) * observations[j].y + p_y;
         observations_transformed.push_back(LandmarkObs{observations[j].id, x_trans, y_trans});
      }

      // Perform data association for the current particle

      dataAssociation(landmarks_in_range, observations_transformed);

      // Update the weight of each particle by using multivariate Gaussian 
      // Start with the weight initialized at 1.0

      particles[i].weight = 1.0;

      for (unsigned int j = 0; j < observations_transformed.size(); j++) {

         double obs_x = observations_transformed[j].x;
         double obs_y = observations_transformed[j].y;

         double pred_x, pred_y;

         int associated_id = observations_transformed[j].id;

         for (unsigned int k = 0; k < landmarks_in_range.size(); k ++) { 

            if (associated_id == landmarks_in_range[k].id) {

               pred_x = landmarks_in_range[k].x;
               pred_y = landmarks_in_range[k].y;
               break;
            }
         }

         double d_x = obs_x - pred_x;
         double d_y = obs_y - pred_y;
         double s_x = std_landmark[0];
         double s_y = std_landmark[1];

         double obs_w = exp(-1.0 * d_x * d_x / (2 * s_x * s_x) - d_y * d_y / (2 * s_y * s_y)) / (2*M_PI*s_x*s_y);

         if (obs_w == 0) {
            particles[i].weight *= EPS;
         } else {
            particles[i].weight *= obs_w;
         }
      }
   }
   printDebug("Exit ParticleFilter::updateWeights");
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

   printDebug("Enter ParticleFilter::resample");

   vector<Particle> new_particles;

   // Collect all the weights of the particles, and store the max weight

   vector<double> weights;
   double w_max = numeric_limits<double>::min();

   for (int i = 0; i < num_particles; i++) {

      weights.push_back(particles[i].weight);
 
      if (particles[i].weight > w_max) { 
         w_max = particles[i].weight;
      }
   }

   // We are using resampling wheel for the resampling process
   // For that, we need to generate the random starting index

   uniform_int_distribution<int> distribution_int(0, num_particles - 1);
   auto index = distribution_int(rand_engine);

   // We also need the uniform distribution of double type

   uniform_real_distribution<double> distribution_double(0.0, w_max);

   // Resampling wheel algorithm, as described in Sebastian Thrun's lessons on
   // using resampling wheel for resampling particles in the particle filter

   double beta = 0.0;

   for (int i = 0; i < num_particles; i++) {

      beta += distribution_double(rand_engine) * 2.0;
  
      while (beta > weights[index]) {

         beta -= weights[index];
         index = (index + 1) % num_particles;
      }

      new_particles.push_back(particles[index]);
   }

   // Assign the particles to the new sampled particles

   particles = new_particles;
   printDebug("Exit ParticleFilter::resample");
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;

    return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
void ParticleFilter::printDebug(const std::string s) {

  // Print the string if the debug flag is set to true, otherwise do nothing
  if (print_flag_ == true) {
    std::cout << s << std::endl;
  }
}

