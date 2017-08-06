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
#include <sstream>
#include <string>
#include <iterator>
//#include "map.h"

//#include "Eigen/Dense"

//David Simon code modified from lecture

//using Eigen::MatrixXd;
//using Eigen::VectorXd;
using std::vector;

#include "particle_filter.h"
//ParticleFilter pf;
using namespace std;
//is_initialized(false);
//pf.initialized==false; //???
//!pf.initialized();
void ParticleFilter::init(double x, double y, double theta, double std[]) {
    // TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
    //   x, y, theta and their uncertainties from GPS) and all weights to 1. 
    // Add random Gaussian noise to each particle.
    // NOTE: Consult particle_filter.h for more information about this method (and others in this file).
    //is_initialized = false;
    num_particles = 100;
    cout<<"number of particles"<<endl<<num_particles<<endl<<endl;
    default_random_engine gen;
    //vector<Particle> particles = pf.particles;
    //initialize
    
    
    //num_particles = weights.size(); ??problem
	
	//VectorXd weights = VectorXd(num_particles);
    //VectorXd particles = VectorXd(3);

    // This line creates a normal (Gaussian) distribution for x
    //normal_distribution<double> dist_x(x, std[0]);
    //normal_distribution<double> dist_y(y, std[1]);
    //normal_distribution<double> dist_theta(theta, std[2]);
    double weight = 1.0;
    cout<<"weight"<<endl<<weight<<endl<<endl;
	
	//added
	//for (int i = 1; i <= num_particles; i++){
        //Particle particles;
        //particles.id = i;
        //particles.x = x;
        //particles.y = y;
        //particles.theta = theta;
        //particles.weight = weight;}

    // This line creates a normal (Gaussian) distribution for x
    normal_distribution<double> dist_x(x, std[0]);
    normal_distribution<double> dist_y(y, std[1]);
    normal_distribution<double> dist_theta(theta, std[2]);
    cout<<"past normal dist"<<endl<<endl;
    //int i;
    for (int i = 1; i <= num_particles; i++){
        //std::vector<Particle> particles;
        //double particle[0], particle[1], particle[2];
        cout<<"i"<<endl<<i<<endl<<endl;
        //particles << dist_x(gen), dist_y(gen), dist_theta(gen), weight;
        Particle particles;
        particles.id = i;
        particles.x = dist_x(gen);//had particles[i].x
        cout<<"particle x"<<endl<<particles.x<<endl<<endl;
        particles.y = dist_y(gen); 
        cout<<"particle y"<<endl<<particles.y<<endl<<endl;
        particles.theta = dist_theta(gen);
        cout<<"particle theta"<<endl<<particles.theta<<endl<<endl;
		particles.weight = weight;
		cout<<"particle weight"<<endl<<particles.weight<<endl<<endl;
        //if(i > num_particles){is_initialized=true;}
}
        //weights.fill(1.0);}
	cout<<"before initialization command"<<endl;
    is_initialized = true;
    //initialized()=true;
    //is_initialized(true);
	//ParticleFilter() : is_initialized(true);

    //initialized() = true;
    cout<<"after initialization command"<<endl;
    //return;
    //Prediction(delta_t, std_pos[], velocity, yaw_rate);
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
    // TODO: Add measurements to each particle and add random Gaussian noise.
    // NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
    //  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
    //  http://www.cplusplus.com/reference/random/default_random_engine/
    //default_random_engine gen;
    //vector<Particle> particles = pf.particles;
    //int num_particles = 100;
    cout<<"just before prediction loop"<<endl<<endl;
    for (int i = 1; i <= num_particles; ++i) {
        // particles[i].x, particles[i].y, particles[i].theta, particles[i].weight;
        Particle particles;
        particles.id = i;
        particles.x = particles.x + (velocity/yaw_rate)*(sin(particles.theta + yaw_rate * delta_t) - sin(particles.theta));
        particles.y = particles.y + (velocity/yaw_rate)*(cos(particles.theta) - cos(particles.theta + yaw_rate * delta_t));
        particles.theta = particles.theta + yaw_rate * delta_t;
        default_random_engine gen;
        //add Gaussian noise
        normal_distribution<double> dist_x(particles.x, std_pos[0]);
        normal_distribution<double> dist_y(particles.y, std_pos[1]);
        normal_distribution<double> dist_theta(particles.theta, std_pos[2]);
     
        // particles[i].x, particles[i].y, particles[i].theta, particles[i].weight;
        particles.x = dist_x(gen);
        particles.y = dist_y(gen);
        particles.theta = dist_theta(gen);}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
    // TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
    //   observed measurement to this particular landmark.
    // NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
    //   implement this method and use it as a helper during the updateWeights phase.



}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
        std::vector<LandmarkObs> observations, Map map_landmarks) {
    // TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
    //   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
    // NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
    //   according to the MAP'S coordinate system. You will need to transform between the two systems.
    //   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
    //   The following is a good resource for the theory:
    //   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
    //   and the following is a good resource for the actual equation to implement (look at equation 
    //   3.33
    // was  Map map_landmarks
	// landmark_list.size
	//VectorXd landmark_list = VectorXd(42);
    //VectorXd pred_landmark = VectorXd(42); //# of landmarks
    //VectorXd error = VectorXd(42); //# of landmarks
    //VectorXd transfx = VectorXd(observations.size());
    //VectorXd transfy = VectorXd(observations.size());
    //vector<map_landmarks>landmark_list(42);
	//std::vector<single_landmark_s> landmark_list(42);
	//Map::single_landmark_s landmarks = map_landmarks.landmark_list;???? needed
	//vector<Particle> particles = pf.particles;
    for(int i = 0; i < num_particles; ++i) {  
    //transform each particle sensor readings

        for(int j = 0; j < observations.size(); j++){
            LandmarkObs obs;
            //obs.x = x_sense[j];
            //obs.y = y_sense[j];
            //noisy_observations.push_back(obs);//??
            //transfx(j) = particles[0] + particles[0] * cos(particles[2]) + particles[1] * sin(particles[2]);
            //transfy(j) = particles[1]+ particles[0] * -sin(particles[2]) + particles[1] * cos(particles[2]);
            Particle particles;
			particles.id = j;
		   //particles[j].x, particles[j].y, particles[j].theta;
            transfx[j] = particles.x + obs.x * cos(particles.theta) + obs.y * sin(particles.theta);
            transfy[j] = particles.y + obs.x * -sin(particles.theta) + obs.y * cos(particles.theta);
            

            for(int k = 0; k <= 42; k++){
                //double dist(double x1, double y1, double x2, double y2)
                //pred_landmark(k) = dist(transfx, transfy, map_landmarks, map_landmarks);
				//map_landmarks list;
                pred_landmark[k] = sqrt((map_landmarks.landmark_list[k].x_f - transfx[k])*(map_landmarks.landmark_list[k].x_f - transfx[k])+(map_landmarks.landmark_list[k].y_f - transfy[k])*(map_landmarks.landmark_list[k].y_f - transfy[k]));
                //if (dist <= sensor_range){
                if (pred_landmark[k] < pred_landmark[k-1]){
                    error[k] = pred_landmark[k];}
                else if (pred_landmark[k] > pred_landmark[k-1]){
                    error[k] = pred_landmark[k-1];}
                
                }
                //if(k > 1 and pred_landmark[k] > pred_landmark[k-1]){
                //lowdist = pred_landmark;
                //else if(dist > sensor_range){
                //cout << "landmark out of sensor range"<< endl;}
            //error(j) = min_element(pred_landmark);
            weights[j] = (1/(2*M_PI*std_landmark[0]*std_landmark[1]))*exp((-error[j] * error[j])/(2*std_landmark[0]*std_landmark[1])); 
            weights[j] = weights[j] * weights[j-1];


}
}
}


void ParticleFilter::resample() {
    // TODO: Resample particles with replacement with probability proportional to their weight. 
    // NOTE: You may find std::discrete_distribution helpful here.
    //   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
    
	
	//std::discrete_distribution<>(weights);
	
	num_particles = 100;
    default_random_engine gen; //??? needed
    //vector<Particle> particles = pf.particles;
    //initialize
    
    //VectorXd weights = VectorXd(num_particles);
    //VectorXd particles = VectorXd(3);

    // This line creates a normal (Gaussian) distribution for x
    //normal_distribution<double> dist_x(sense_x, sigma_pos[0]);
    //normal_distribution<double> dist_y(sense_y, sigma_pos[1]);
    //normal_distribution<double> dist_theta(sense_theta, sigma_pos[2]);

    int sum = 0;
    for (int i = 0; i <= num_particles; ++i) {
        double sumweight = ++weights[i];}
    
    for (int i = 0; i <=num_particles; ++i){
        multiples[i] = (int)((weights[i]/sumweight)*100); //can add 0.5 per internet
        if (multiples[i] > 0){
            
            for (int counter = sum; counter <= sum + multiples[i]; ++counter){
                Particle particles;
                resampled[counter] = particles;} //??? need .x, .y ??
}
        else if (multiples[i] = 0){ //needed??
}
        sum = sum + multiples[i];
        }

    for (int i = 0; i <=num_particles; ++i) {
        Particle particles;
        particles.id = i;
        particles = resampled[i];}        

    
}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

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