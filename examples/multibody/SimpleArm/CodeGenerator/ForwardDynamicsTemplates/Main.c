//
// Created by arjunbadyal on 15/02/2023.
//
#include "ForwardDynamics.h"



int main() {
  //graph *g1 = create_graph(5);
  int n = 6;
  //double **w1 = dmatrix(0,n,0,n);
  //w1[0][0] = 1;
  Matrix phi;
  phi.cols = n;
  phi.rows = n;
  //printf("Size %d", phi.cols);
  phi.data= dmatrix(0,n,0,n);
  /*Matrix *sub = create_submatrix(&phi, 0, 0, 3, 3);
  for (int i = 0; i < sub->rows; i++) {
    for (int j = 0; j < sub->cols; j++) {
      sub->data[i][j] = 1.0;
    }
  }*/

  Matrix id = create_identity_matrix(3);

  replace_submatrix(&phi,0,0,3,3,&id);
  replace_submatrix(&phi,3,0,3,3,&id);

  //phi = lX();


  // Print the original matrix to verify that the submatrix has been modified
  for (int i = 0; i < phi.rows; i++) {
    for (int j = 0; j < phi.cols; j++) {
      printf("%f ", phi.data[i][j]);
    }
    printf("\n");
  }
  Vector v  = *dvector(0,2);
  v.data[0] = 1;
  v.data[1] = 2;
  v.data[2]= 3;
  Vector u = *dvector(0,2);
  for (int i = 0; i< v.size; i++){
    u.data[i] = 0;
  }
  Matrix L = lX(u,v);

  for (int i = 0; i < L.rows; i++) {
    for (int j = 0; j < L.cols; j++) {
      printf("%f ", L.data[i][j]);
    }
    printf("\n");
  }

  /*add_edge(g1,0,1,0.0);
  add_edge(g1,1,2,1.0);
  add_edge(g1,2,3,2.0);
  add_edge(g1,2,2,3.0);
  print_graph(g1);
  destroy_graph(g1);*/
}