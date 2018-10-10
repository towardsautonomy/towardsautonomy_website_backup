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

/* Random Engine */
static default_random_engine gen;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of
	//   x, y, theta and their uncertainties from GPS) and all weights to 1.
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

	/* Check if already initialized */
	if(is_initialized == true) {
		return;
	}
	num_particles = 500;
	/* Create Normal Distribution for x, y, and theta */
	normal_distribution<double> N_x(x, std[0]);
	normal_distribution<double> N_y(y, std[1]);
	normal_distribution<double> N_theta(theta, std[2]);

	for(uint16_t i=0; i < num_particles; i++) {
		Particle temp_p;
		/* Set Particle ID */
		temp_p.id = i;
		/* Set Particle initial position and orientation */
		temp_p.x = N_x(gen);
		temp_p.y = N_y(gen);
    temp_p.theta = N_theta(gen);
		/* Set the particle weight to 1 */
		temp_p.weight = 1.0;
		/* Push this particle back into the particle vector */
		particles.push_back(temp_p);
		weights.push_back(1.0);
	}
	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	/* Create Normal Distribution for x, y, and theta with mean 0 and std deviation std_pos */
	normal_distribution<double> N_x(0, std_pos[0]);
  normal_distribution<double> N_y(0, std_pos[1]);
  normal_distribution<double> N_theta(0, std_pos[2]);

  for (uint16_t i = 0; i < num_particles; i++) {
    /* Predict new states */
    if (fabs(yaw_rate) < 0.00001) {
      particles[i].x += velocity * delta_t * cos(particles[i].theta);
      particles[i].y += velocity * delta_t * sin(particles[i].theta);
    }
    else {
      particles[i].x += velocity / yaw_rate * (sin(particles[i].theta + yaw_rate*delta_t) - sin(particles[i].theta));
      particles[i].y += velocity / yaw_rate * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate*delta_t));
      particles[i].theta += yaw_rate * delta_t;
    }

    /* Add Noise */
    particles[i].x += N_x(gen);
    particles[i].y += N_y(gen);
    particles[i].theta += N_theta(gen);
  }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
	//   implement this method and use it as a helper during the updateWeights phase.
	for (uint16_t i = 0; i < observations.size(); i++) {

    /* Get Observation */
    LandmarkObs obs_landmark = observations[i];

    /* Initialize minimum distance as max() */
    double min_dist = std::numeric_limits<double>::max();

    /* Temp map id of landmark; initialize to -1 */
    int temp_map_id = -1;

    for (uint16_t j = 0; j < predicted.size(); j++) {
      /* Grab one prediction */
      LandmarkObs pred_landmark = predicted[j];

      /* compute distance between observed and predicted landmarks */
      double cur_dist = dist(obs_landmark.x, obs_landmark.y, pred_landmark.x, pred_landmark.y);

      /* Get the predicted landmark closest the observed landmark */
      if (cur_dist < min_dist) {
        min_dist = cur_dist;
        temp_map_id = pred_landmark.id;
      }
    }

    /* Set the observation's map id to the nearest predicted landmark's map id */
    observations[i].id = temp_map_id;
  }
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

	/*
	 * For each particle, (a) Get the landmarks within sensor range
	 *  									(b) convert the observations from car coordinate to map coordinate
	 *										(c) Associate each observed landmark with a map landmark
	 *										(d) Compute weight for each particle using multi-variate gaussian dist.
	 */
	for(uint16_t i=0; i<num_particles; i++) {
			/* Get all the map landmarks within the sensor range in a LandmarkObs vector */
			std::vector<LandmarkObs> pred_landmark;
			LandmarkObs landmark;
			for(uint16_t j=0; j<map_landmarks.landmark_list.size(); j++) {
				landmark.id = map_landmarks.landmark_list[j].id_i;
				landmark.x = map_landmarks.landmark_list[j].x_f;
				landmark.y = map_landmarks.landmark_list[j].y_f;
				/* Check if the landmark is within sensor range */
				if(dist(landmark.x, landmark.y, particles[i].x, particles[i].y) < sensor_range) {
					pred_landmark.push_back(landmark);
				}
			}

			/*
			 * Convert each particle from Car coordinate to map coordinate using homogeneous transformation
			 * and perform data association with landmarks
			 */
			vector<LandmarkObs> transformed_obs;
			LandmarkObs transformed;
	    for (uint16_t j = 0; j < observations.size(); j++) {
				transformed.id = -1;	// No landmark is associated yet
	      transformed.x = cos(particles[i].theta)*observations[j].x - sin(particles[i].theta)*observations[j].y + particles[i].x;
	      transformed.y = sin(particles[i].theta)*observations[j].x + cos(particles[i].theta)*observations[j].y + particles[i].y;
				transformed_obs.push_back(transformed);
	    }
			/* Perform data association to associate each transformed observation with one of the landmarks */
			dataAssociation(pred_landmark, transformed_obs);

			/* Vector of associated landmarks and x,y */
		  vector<double> obs_x;
			vector<double> obs_y;
			vector<int> associated_landmark;

			/* Reset the particle weight as 1 */
			particles[i].weight = 1.0;

			/* Update the weights of each particle using a multi-variate Gaussian distribution */
			for (uint16_t j = 0; j < transformed_obs.size(); j++) {
				/* Predicted X, Y */
				double pred_x, pred_y;

				/* Observed X, Y, and associated landmark id */
				obs_x.push_back(transformed_obs[j].x);
				obs_y.push_back(transformed_obs[j].y);
				associated_landmark.push_back(transformed_obs[j].id);
				// get the x,y coordinates of the prediction associated with the current observation
				for (uint16_t k = 0; k < pred_landmark.size(); k++) {
					/* Find the landmark parameters associated with the transformed observation */
					if (pred_landmark[k].id == associated_landmark[j]) {
						pred_x = pred_landmark[k].x;
						pred_y = pred_landmark[k].y;
					}
				}

				/* Calculate new weight for this particle using multi-variate Gaussian distribution */
				double sig_x = std_landmark[0];
				double sig_y = std_landmark[1];
				/* Calculate normalization term */
				double gauss_norm = (1/(2*M_PI*sig_x*sig_y));
				/* Calculate exponent */
				double exponent = (pow((obs_x[j] - pred_x), 2)/(2*pow(sig_x, 2))) + (pow((obs_y[j] - pred_y), 2)/(2*pow(sig_y, 2)));
				/* calculate weight using normalization terms and exponent */
				double weight = gauss_norm * exp(-exponent);
				//if(weight == 0) weight = 0.00001;
				/* Multiply this new weight with the previous weight */
				particles[i].weight *= weight;
				/* Set associations */
				SetAssociations(particles[i], associated_landmark, obs_x, obs_y);
			}
			weights[i] = particles[i].weight;
	}
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight.
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	/* Create a new vector of resampled particles */
	vector<Particle> resampled_particles;

	/* Get a random index between 0 and num_particles-1 */
	std::uniform_int_distribution<int> uniform_dist_index(0, num_particles-1);
	int index = uniform_dist_index(gen);
	/* Get maximum weight */
	double max_weight = *std::max_element(weights.begin(), weights.end());
	/* Get a random value between 0 and 2*max_weight */
	std::uniform_real_distribution<double> uniform_dist_weight(0.0, 2.0*max_weight);
	/* beta */
	double beta = 0.0;
	/* Use the resampling wheel approach to resample the particles */
	for(uint16_t i = 0; i < num_particles; i++) {
		beta += uniform_dist_weight(gen);
		while(beta > weights[index]) {
			beta -= weights[index];
			index = (index + 1) % num_particles;
		}
		resampled_particles.push_back(particles[i]);
	}
	/* Copy resampled particles back into the particles vector */
	particles = resampled_particles;

	/* Copy the particles weight into the vector 'weights' */
	for (uint16_t i = 0; i < num_particles; i++) {
		weights[i] = particles[i].weight;
	}
}

void ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations,
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
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
