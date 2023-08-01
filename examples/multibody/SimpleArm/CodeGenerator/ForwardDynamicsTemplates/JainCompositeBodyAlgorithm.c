//
// Created by arjunbadyal on 13/02/2023.
//
#include <math.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>

//Algorithm 3.2 p.g. 45. Recursive computation of link spatial velocites.
// TODO inputs gsl_matrix phi_adj, gsl_matrix H_adj
void RecursiveLinkVelocities(int k, int n){
  gsl_vector * V = gsl_vector_alloc(n+1);
  gsl_vector_set(V, n, 1.0);
  for(k = 1; k < n; k++){
    /*double V_k = gsl_blas_dgemv(CblasNoTrans, 1.0, phi_adj, gsl_vector_get(V,k+1)) +
                 gsl_blas_dgemv(CblasNoTrans, 1.0, gsl_matrix_get(H_adj,k), gsl_vector_get(V,k+1));*/
    gsl_vector_set(V, k, 1.0);
  }
}