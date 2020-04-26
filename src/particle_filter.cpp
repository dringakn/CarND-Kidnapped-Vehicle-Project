#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>
#include "helper_functions.h"

using std::string; // Use string without std namespace for later use
using std::vector; // Use vector without std namespace for later use

// Static members definations
std::default_random_engine ParticleFilter::rng;
std::normal_distribution<double> ParticleFilter::normDistX;
std::normal_distribution<double> ParticleFilter::normDistY;
std::normal_distribution<double> ParticleFilter::normDistTheta;

void ParticleFilter::init(double x, double y, double theta, double std[])
{
  num_particles = 50;             // Set the number of particles
  particles.resize(num_particles); // Change the number of particles
  weights.resize(num_particles);   // Change the number of
  // Initialize Normal/Gaussian noise generators for the particles
  normDistX.param(std::normal_distribution<double>(x, std[0]).param());
  normDistY.param(std::normal_distribution<double>(y, std[1]).param());
  normDistTheta.param(std::normal_distribution<double>(theta, std[2]).param());

  // Initialize position (based on estimates of x, y, theta and their uncertainties from GPS) and all weights to 1.
  int idx = 0;
  for (auto &pt : particles)
  {
    pt.id = idx;
    pt.x = normDistX(rng);
    pt.y = normDistY(rng);
    pt.theta = normDistTheta(rng);
    pt.weight = weights[idx] = 1;
    idx++;
  }
  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[],
                                double velocity, double yaw_rate)
{
  // Extract and Initialize Normal/Gaussian noise generators for the particles
  normDistX.param(std::normal_distribution<double>(0, std_pos[0]).param());
  normDistY.param(std::normal_distribution<double>(0, std_pos[1]).param());
  normDistTheta.param(std::normal_distribution<double>(0, std_pos[2]).param());
  double x, y, theta;
  for (auto &pt : particles){
    // Extract each particles pose information
    x = pt.x;
    y = pt.y;
    theta = pt.theta;
    // Preict particle pose
    if (fabs(yaw_rate) >= 0.0001)
    {
      /* If there's as some yaw rotation */
      pt.x = x + velocity / yaw_rate * (sin(theta + yaw_rate * delta_t) - sin(theta));
      pt.y = y + velocity / yaw_rate * (cos(theta) - cos(theta + yaw_rate * delta_t));
      pt.theta = theta + yaw_rate * delta_t;
    }
    else
    {
      pt.x = x + velocity * delta_t * cos(theta);
      pt.y = y + velocity * delta_t * sin(theta);
    }
    // Add random normal noise
    pt.x += normDistX(rng);
    pt.y += normDistY(rng);
    pt.theta += normDistTheta(rng);
  }
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted,
                                     vector<LandmarkObs> &observations)
{
  /**
   * Find the predicted measurement that is closest to each observed measurement 
   * and assign the observed measurement to this particular landmark.
   * Nearest Neighbourhood Search:
   *    The number of observations may be less than the total number of landmarks 
   *    as some of the landmarks may be outside the range of sensor range.
   */

  double lmDist;
  for (size_t i = 0; i < observations.size(); i++) {
    double minDist = std::numeric_limits<double>::max(); // Initialize to maximum value
    int minIDX = -1;                        // Initialize idx
    for (size_t j = 0; j < predicted.size(); j++) {
      lmDist = dist(observations[i].x, observations[i].y, predicted[j].x, predicted[j].y);
      if (lmDist < minDist) {
        minIDX = predicted[j].id;
        minDist = lmDist;
      }
      observations[i].id = minIDX;
    } // predicted
  } // observations

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   const vector<LandmarkObs> &observations,
                                   const Map &map_landmarks)
{
  /**
   * NOTE: The observations are given in the VEHICLE'S coordinate system. 
   *   Your particles are located according to the MAP'S coordinate system. 
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */
  vector<Map::single_landmark_s> lms = map_landmarks.landmark_list;
  double gaussNorm = 1 / (2 * M_PI * std_landmark[0] * std_landmark[1]);
  double varX = 2 * pow(std_landmark[0], 2);// Variance in landmark x-measurements
  double varY = 2 * pow(std_landmark[1], 2);// Variance in landmark y-measurements
  double x, y, theta, weight; // Particle pose and weight.
  double exponent, lmX, lmY;
  double lmDist;

  for (int i = 0; i < num_particles; ++i)
  {
    x = particles[i].x;
    y = particles[i].y;
    theta = particles[i].theta;
    weight = 1.0;

    // Search for the nearby landmarks in the list
    vector<LandmarkObs> nearbyLMS;
    LandmarkObs lm;
    for (unsigned int k = 0; k < lms.size(); ++k)
    {
      lmDist = dist(x, y, lms[k].x_f, lms[k].y_f); //distance between particle and landmark
      if (lmDist < sensor_range)// select landmarks with in sensor range
      {
        lm.id = lms[k].id_i;
        lm.x = lms[k].x_f;
        lm.y = lms[k].y_f;
        nearbyLMS.push_back(lm);
      }
    }

    double xm, ym, xv, yv;
    for (unsigned int j = 0; j < observations.size(); ++j)
    {
      // Transform position from vehicle to map coordinate
      xv = observations[j].x; // Landmark position x-coordinate in vehilce frame
      yv = observations[j].y; // Landmark position y-coordinate in vehilce frame
      xm = x + cos(theta) * xv - sin(theta) * yv; // Landmark position x-coordinate in map frame
      ym = y + sin(theta) * xv + cos(theta) * yv; // Landmark position y-coordinate in map frame

      // Search for the nearest landmark idx
      double minDist = std::numeric_limits<double>::max(); // Initialize to maximum value
      int minIDX = 0;  // Initialize idx
      for (unsigned int n = 0; n < nearbyLMS.size(); ++n)
      {
        lmDist = dist(xm, ym, nearbyLMS[n].x, nearbyLMS[n].y);
        if (lmDist < minDist)
        {
          minIDX = nearbyLMS[n].id;
          minDist = lmDist;
        }
      }

      // Search for the nearest landmark postion
      for (unsigned int n = 0; n < nearbyLMS.size(); ++n)
      {
        if (nearbyLMS[n].id == minIDX)
        {
          lmX = nearbyLMS[n].x;
          lmY = nearbyLMS[n].y;
          break;
        }
      }

      // Multivariate-Gaussian probability
      exponent = (pow(xm - lmX, 2) / varX) + (pow(ym - lmY, 2) / varY);
      weight *= gaussNorm * exp(-exponent);
    }

    particles[i].weight = weight;
    weights[i] = weight;
  }
}

void ParticleFilter::resample()
{
  // Resample particles with replacement with probability proportional to their weight. 
  vector<Particle> newParticles(num_particles);
  int idx = rand() % num_particles;
  double maxW = *max_element(weights.begin(), weights.end()); // Search the maximum weight of the particle
  double beta = 0;
  for (int i = 0; i < num_particles; ++i)
  {
    beta += (rand() / (RAND_MAX + 1.0)) * (2 * maxW);
    while (weights[idx] < beta)
    {
      beta -= weights[idx];
      idx = (idx + 1) % num_particles;
    }
    newParticles[i] = particles[idx];
  }
  particles = newParticles;
}

void ParticleFilter::SetAssociations(Particle &particle,
                                     const vector<int> &associations,
                                     const vector<double> &sense_x,
                                     const vector<double> &sense_y)
{
  // particle: the particle to which assign each listed association,
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations = associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best)
{
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length() - 1); // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord)
{
  vector<double> v;

  if (coord == "X")
  {
    v = best.sense_x;
  }
  else
  {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length() - 1); // get rid of the trailing space
  return s;
}

// Convert coordinates from particle to map 
LandmarkObs transformVehicle2Map(Particle pt, LandmarkObs obs) {
  LandmarkObs lm;
  lm.id = obs.id;
  lm.x = obs.x * cos(pt.theta) - obs.y * sin(pt.theta) + pt.x;
  lm.y = obs.x * sin(pt.theta) + obs.y * cos(pt.theta) + pt.y;
  return lm;
}