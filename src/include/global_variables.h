//
// Created by riccardo on 15/04/21.

#ifndef glob_var
#define glob_var
#include <set>
#include <vector>

#include "support.h"

using namespace std;

extern vector<vector<bool>> observability_matrix;

extern vector<vector<bool>> P_of_X_for_simulations_T;
extern int totalNumberOfParticleinside;
extern std::set<int> possible_obj_poses;
extern vector<vector<bool>> ObsMat;
extern vector<int> VisitedPoses;
extern vector<double> GaussProbVector;
extern int len_obsvM;
extern vector<vector<double>> gaussProbMat;

#endif
