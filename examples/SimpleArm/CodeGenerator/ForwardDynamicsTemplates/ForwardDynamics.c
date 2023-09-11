//
// Created by arjunbadyal on 11/02/2023.
//
#include"ForwardDynamics.h"
#define NR_END 1
//#define N 1;

// Input p-model elements
/*double Mass_BaseLink = 100.0;
double BodyFrame1_BaseLink = (0.0,0.0,0.25,0.0,0.0,0.0);
double OutboardMob1_ElbowJoint = (0.0);
double Hinge_ElbowJoint;

double Mass_IntermediateLink;
double Pose_IntermediateLink;
double Pose_WristJoint;
double Hinge_WristJoint;
char Gripper;*/

//Begin Matrix Library

//Vector allocation Numerical Recipes in C
Vector *dvector(long nl, long nh)
{
  double *v;
  v = (double*)malloc((size_t) ((nh - nl + 1 + NR_END)*sizeof (double)));
  Vector *d = (Vector *)malloc(sizeof(Vector));
  d->size = nh - nl + 1;
  d->data = v;
  return d;
}



//Matrix allocation Numerical Recipes in C
double **dmatrix(long nrl, long nrh, long ncl, long nch)
/* allocate a double matrix with subscript range m[nrl..nrh][ncl..nch] */
{
  long i, nrow=nrh-nrl+1,ncol=nch-ncl+1;
  double **m;

  /* allocate pointers to rows */
  m=(double **) malloc((size_t)((nrow+NR_END)*sizeof(double*)));
  //if (!m) nrerror("allocation failure 1 in matrix()");
  m += NR_END;
  m -= nrl;

  /* allocate rows and set pointers to them */
  m[nrl]=(double *) malloc((size_t)((nrow*ncol+NR_END)*sizeof(double)));
  //if (!m[nrl]) nrerror("allocation failure 2 in matrix()");
  m[nrl] += NR_END;
  m[nrl] -= ncl;

  for(i=nrl+1;i<=nrh;i++) m[i]=m[i-1]+ncol;

  /* return pointer to array of pointers to rows */
  return m;
}

/*void init_block(Matrix *m, int row_start, int col_start, int block_rows, int block_cols, double val) {
  for (int i = row_start; i < row_start + block_rows; i++) {
    for (int j = col_start; j < col_start + block_cols; j++) {
      m->data[i][j] = val;
    }
  }
}*/

Matrix *create_submatrix(Matrix *m, int row_start, int col_start, int sub_rows, int sub_cols) {
  Matrix *sub = (Matrix *)malloc(sizeof(Matrix));
  sub->rows = sub_rows;
  sub->cols = sub_cols;

  // Allocate memory for the submatrix
  sub->data = dmatrix(0,sub_rows,0,sub_cols);

  // Copy the elements from the original matrix to the submatrix
  for (int i = 0; i < sub_rows; i++) {
    for (int j = 0; j < sub_cols; j++) {
      m->data[i + row_start][j + col_start] = sub->data[i][j];
    }
  }

  return sub;
}

Matrix create_identity_matrix(int n) {
  Matrix id_matrix;
  id_matrix.rows = n;
  id_matrix.cols = n;

  // Allocate memory for the matrix
  id_matrix.data = (double**)malloc(n * sizeof(double*));
  for (int i = 0; i < n; i++) {
    id_matrix.data[i] = (double*)malloc(n * sizeof(double));
  }

  // Initialize the matrix to the identity matrix
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < n; j++) {
      if (i == j) {
        id_matrix.data[i][j] = 1.0;
      } else {
        id_matrix.data[i][j] = 0.0;
      }
    }
  }
  return id_matrix;
}

void replace_submatrix(Matrix* m, int row_start, int col_start, int sub_rows, int sub_cols, Matrix* sub) {
  for (int i = 0; i < sub_rows; i++) {
    for (int j = 0; j < sub_cols; j++) {
      m->data[row_start + i][col_start + j] = sub->data[i][j];
    }
  }
}





//Adjacency matrix
struct AdjMat {
  int numnodes; //number of nodes
  double **edges; //number of edges using double pointer for 2d array
};

double sizeofMatrix(double **Mat){
    return sizeof(Mat);
}

graph* create_graph(int numnodes){
  graph *g = malloc(sizeof(*g)); //allocates memory of size g
  if (g== NULL){
    return NULL;
  }
  g->numnodes = numnodes;
  //allocate our matrix
  g->edges = dmatrix(0,numnodes, 0, numnodes);
  if (g->edges == NULL){
    free(g);
    return NULL;
  }

  for(int i=0; i< g-> numnodes; i++) {
    g-> edges[i] = calloc(sizeof (double), g->numnodes);
    if (g->edges[i] ==NULL){
      //cleanup again
      destroy_graph(g);
      return NULL;
    }
  }
  return g;
}
void destroy_graph(graph *g){
  if (g->edges ==NULL) {
    free(g);
    return;
  }
  for (int i=0; i < g->numnodes; i++){
    if(g->edges[i] != NULL){
      free(g);
    }
  }
  free(g->edges);
  //free(g);
}
//Can be used with vizgraph to print in dot format
void print_graph(graph *g){
  printf("digraph {\n");
  for (int parent=0; parent < g-> numnodes; parent++) {
    for (int child = 0; child< g->numnodes; child++) {
      if (g->edges[parent][child] != 0.0) {
        printf("weight %f;\n", g->edges[parent][child]);
        printf("%d -> %d;\n", parent, child);
      }
    }
  }
  printf("}\n");
}
bool add_edge(graph *g, unsigned int parent_node, unsigned int child_node, double **weight){
  if (has_edge(g, parent_node,child_node)) {
    return false;
  }

  g->edges[parent_node][child_node] = weight[1][2];
  return true;
}
bool has_edge(graph *g, unsigned int from_node, unsigned int to_node){
  return g->edges[from_node][to_node] != 0.0;
}



gsl_matrix *GenerateSKO(int n) {
  int Hinge_dof = 1;
  int N = 0;
  for (int i = 1; i <= n; i++) {
    N += Hinge_dof;
  }
  //int* ptr = (int*)malloc(j * sizeof(int));

  gsl_matrix *phi = gsl_matrix_alloc(N, N);
  printf("N is %d", N);
  // SKO Matrix

  //
  return phi;

}

//A function to calculate the rotation matrix between two reference frames F and G.
Matrix R(Vector F, Vector g){
  Matrix result;
  result.data= dmatrix(0,3,0,3);
  result.cols=3;
  result.rows=3;
}

//A function to calculate l(x,y)X where x and y are vectors.

Matrix lX(Vector x, Vector y){
  Vector l = *dvector(0,2);
  for(int i = 0; i <= 2; i++){
    l.data[i] = y.data[i] - x.data[i];
  }
  double a = l.data[0];
  double b = l.data[1];
  double c = l.data[2];

  Matrix result;
  result.data= dmatrix(0,3,0,3);
  result.cols=3;
  result.rows=3;
  result.data[0][0] = 0.0;
  result.data[1][1] = 0.0;
  result.data[2][2] = 0.0;
  result.data[0][1] = -c;
  result.data[1][0] = c;
  result.data[0][2]= b;
  result.data[2][0]= -b;
  result.data[1][2] = -a;
  result.data[2][1] = a;
  return result;
}
//End Matrix Library



//Begin Matrix Solvers
//Cholesky factorisation step (numerical recipies in C)
void Cholesky(float **a, int n, float p[])
{
  int i, j, k;
  float sum;

  for (i = 1; i<=n; i++){
    for (j = i; j<=n; j++){
      for(sum = a[i][j], k=i-1; k>=1; k--) sum -= a[i][k]*a[j][k];
      if (i == j){
        p[i] = sqrt(sum);
      }
      else a[j][i] = sum/p[i];
    }
  }
}
//End Matrix Solvers

//Begin CBA


/*
Matrix Create_phi(){
  Matrix *lx = (Matrix *) malloc(N * sizeof(Matrix));

  // Allocate memory for each matrix in the array
  for (int i = 0; i < N; i++) {
    lx[i].rows = 3;
    lx[i].cols = 3;
    lx[i].data = (double **) malloc(3 * sizeof(double *));
    for (int j = 0; j < 3; j++) {
      for (int k = 0; k < 3; k++){
        lx[i].data =
      }
      //lx[i].data[j] = (double *) malloc(3 * sizeof(double));
    }
  }
  for (int i = 0; i < N; i++) {
    for (int j = 0; j < N; j++) {
      free(lx[i].data[j]);
    }
    free(lx[i].data);
  }
  free(lx);

  Matrix id = create_identity_matrix(3);
  Matrix phi;
  phi.cols = 6;
  phi.rows = 6;
  phi.data= dmatrix(0,6,0,6);
  replace_submatrix(&phi,0,0,3,3,&id);
  replace_submatrix(&phi,3,0,3,3,&id);

}
*/


