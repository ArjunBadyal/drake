//
// Created by arjunbadyal on 11/02/2023.
//
#include <math.h>
#include <gsl/gsl_matrix.h>
#include<stdlib.h>

// Input p-model elements
double Mass_BaseLink = 100.0;
double BodyFrame1_BaseLink = (0.0,0.0,0.25,0.0,0.0,0.0);
double OutboardMob1_ElbowJoint = (0.0);
double Hinge_ElbowJoint;
int Hinge_dof;
int n;
double Mass_IntermediateLink;
double Pose_IntermediateLink;
double Pose_WristJoint;
double Hinge_WristJoint;
char Gripper;
int main() {
  int N = 0;
  for (int i = 1; i = n; i++) {
    N += Hinge_dof;
  }
  int j;
  //int* ptr = (int*)malloc(j * sizeof(int));

  gsl_matrix* phi = gsl_matrix_alloc(N, N);
  printf("N is %d", N);
  // SKO Matrix

  //
  return 0;
}


//Cholesky factorisation step (numerical recipies in C)
void Cholesky(float **a, int n, float p[])
{
  void nrerror(char error_text[]);
  int i, j, k;
  float sum;

  for (i = 1; i<=n; i++){
    for (j = i; j<=n; j++){
      if (i == j){
        p[i] = sqrt(sum);
      }
      else a[j][i] = sum/p[i];
    }
  }
}
