//
// Created by arjunbadyal on 14/02/2023.
//
#pragma once
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>

#ifndef DRAKE_JAINCOMPOSITEBODYALGORITHM_H
#define DRAKE_JAINCOMPOSITEBODYALGORITHM_H
void RecursiveLinkVelocities(int k, int n, gsl_matrix phi_adj, gsl_matrix H_adj);
#endif  // DRAKE_JAINCOMPOSITEBODYALGORITHM_H
