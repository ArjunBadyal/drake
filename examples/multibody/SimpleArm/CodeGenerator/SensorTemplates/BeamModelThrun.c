//
// Created by arjunbadyal on 13/02/2023.
//

//SimpleArm:BeamModelThrun

//Inputs
double p_hit;
double p_max;
double p_short;
double p_rand;

double z_map; //must be manually connected to chosen physics engine
double z_observed; //must be manually connected to chosen physics engine

// Likelihood of scan z_t

double p;
double q;



void BeamModelThrun(double K, double z_map, double z_observed)
{
  double k;
  double p;
  double q;

  q = 1.0;
  for(k = 1; k<=K; k++){
  p =
  }
}
