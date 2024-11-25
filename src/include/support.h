#ifndef SUPPORT_H
#define SUPPORT_H
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "activevisualsearch.h"
using namespace std;
using json = nlohmann::json;

json getDetectorStat(int object_id);

int getFileLength();

vector<vector<bool>> getMatrixClassifier();

vector<vector<int>> getMatrix(string file);
vector<vector<int>> getMatrixG();

vector<bool> getColumnObservMatrix(int id);

vector<bool> getColumnObservMatrix_fromFile(int id, string file_matrix_L);

vector<vector<bool>> getObservMatrix_fromFile(string file_matrix_L);

vector<double> set_uniform_probability();

void updateGaussProb(int position);

void normalize_vector(vector<double> GaussProbVector);
vector<vector<double>> getGaussMatrix_fromFile(string gauss_prob_matrix);

vector<vector<bool>> getObservMatrix_fromFile(string file_matrix_L);

vector<bool> getParticlesThatAreInsideHomeByIndex(const string& filename);
int getMnumber();

int getMnumber_from_file(string file_matrix);

void getListViews(string ListImage[]);

void getNameIstances(string ListNameIstances[]);

bool isPresent(int id, vector<bool> m, json detector_stat);

bool checkStatus(vector<double> GaussProbVector, int id_position);

void int2vector(int observation, int array[]);

#endif /* SUPPORT_H */
