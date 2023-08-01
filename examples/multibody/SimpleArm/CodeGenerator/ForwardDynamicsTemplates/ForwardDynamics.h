#pragma once
#include <stdbool.h>
#include <math.h>
#include <gsl/gsl_matrix.h>
#include<stdlib.h>


#ifdef __cplusplus
extern "C" {
#endif
//numerical Recipes errors
//void nrerror(char error_text[]);


int SKO();


typedef struct{
  int rows;
  int cols;
  //double **(*dmatrix)(long nrl, long nrh, long ncl, long nch);
  double **data;
}Matrix;

typedef struct {
  int size;       // The size of the vector
  double *data;   // A pointer to the first element of the data array
} Vector;
//graph building
typedef struct AdjMat graph; //adjacency matrix data type

Vector *dvector(long nl, long nh);

double **dmatrix(long nrl, long nrh, long ncl, long nch);

void init_block(Matrix *m, int row_start, int col_start, int block_rows, int block_cols, double val);

Matrix *create_submatrix(Matrix *m, int row_start, int col_start, int sub_rows, int sub_cols);

void replace_submatrix(Matrix* m, int row_start, int col_start, int sub_rows, int sub_cols, Matrix* sub);

Matrix create_identity_matrix(int n);

graph* create_graph(int numnodes);
void destroy_graph(graph *g); //deallocates graph from memory

void print_graph(graph *g); //Prints the graph

bool add_edge(graph *g, unsigned int parent_node, unsigned int child_node, double **weight); //Adds edge to graph

bool has_edge(graph *g, unsigned int from_node, unsigned int to_node); //Checks if the graph has an edge

Matrix lX(Vector x, Vector y);

//CBA Jain
Matrix create_l;
Matrix Create_phi();

#ifdef __cplusplus
}
#endif