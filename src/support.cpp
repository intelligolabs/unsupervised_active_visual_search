#include "support.h"

#include <stdio.h>
#include <stdlib.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iostream>
#include <map>
#include <nlohmann/json.hpp>  // for nlohmann::json
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "global_variables.h"
using json = nlohmann::json;

#define c_position 6
#define objects 33

using namespace std;

////
//
// see readme.md for a detailed description of the file!
//
////

const int FileLength = getFileLength();  // Number of graph nodes, i.e pose of the agent inside the house
vector<vector<bool>> MatrixClassifier = getMatrixClassifier();
vector<vector<int>> MatrixG = getMatrixG(); // populate the graph matrix, nodes (poses) and edges(action) to other poses

std::mt19937 rng(std::random_device{}()); // Create a random number generator
std::uniform_real_distribution<double> dist(0.0, 1.0); // Create a uniform real distribution in the range [0, 1]


int getFileLength() {
    fstream ListViewFile;
    ListViewFile.open("./input_pomp_be_pd/ListView.txt", ios::in);

    string s;
    int length = 0;

    while (!ListViewFile.eof()) {
        getline(ListViewFile, s, '\n');
        length++;
    }
    ListViewFile.close();
    return length;
}

void getListViews(string ListImage[]) {
    fstream ListViewFile;
    ListViewFile.open("./input_pomp_be_pd/ListView.txt", ios::in);

    for (int i = 0; i < FileLength; i++) {
        ListViewFile >> ListImage[i];
    }

    ListViewFile.close();
}

void istancesVector(int myVector[]) {
    for (int i = 0; i < objects; i++) myVector[i] = 0;

    for (int i = 0; i < FileLength; i++) {
        for (int j = 0; j < objects; j++) {
            if (MatrixClassifier[i][j] == 1 && myVector[j] == 0)
                myVector[j] = 1;
        }
    }
}

vector<vector<bool>> getMatrixClassifier() {
    fstream MatrixClassifierFile;
    MatrixClassifierFile.open("./input_pomp_be_pd/MatrixL_dt.txt", ios::in);

    string line, temp = "";
    int val = 0, k = 0, counter = 0;

    vector<vector<bool>> MatrixClassifier(FileLength, vector<bool>(objects));
    for (int i = 0; i < FileLength; i++) {
        getline(MatrixClassifierFile, line);
        counter = 0;
        k = 0;

        while (k < (line.size())) {
            temp = "";

            while (isspace(line[k]) == 0 && k < line.size()) {
                temp = temp + line[k];
                k++;
            }
            k++;
            val = stoi(temp);
            MatrixClassifier[i][counter + 1] = val;
            counter++;
        }
       
    }

    MatrixClassifierFile.close();
    return MatrixClassifier;
}

vector<vector<int>> getMatrixG() {
    fstream MatrixGFile;
    MatrixGFile.open("./input_pomp_be_pd/MatrixG.txt", ios::in);

    string line, temp = "";
    int val = 0, counter = 0, k = 0;

    vector<vector<int>> matrix(FileLength, vector<int>(c_position));
    for (int i = 0; i < FileLength; i++) {
        getline(MatrixGFile, line);
        counter = 0;
        k = 0;

        while (k < (line.size())) {
            temp = "";

            while (isspace(line[k]) == 0 && k < line.size()) {
                temp = temp + line[k];
                k++;
            }
            k++;
            val = stoi(temp);
            matrix[i][counter] = val;
            counter++;
        }
       
    }

    MatrixGFile.close();
    return matrix;
}

vector<vector<int>> getMatrix(string file) {
    fstream MatrixGFile;
    MatrixGFile.open(file, ios::in);

    string line, temp = "";
    int val = 0, counter = 0, k = 0;

    vector<vector<int>> matrix(FileLength, vector<int>(264));
    for (int i = 0; i < FileLength; i++) {
        getline(MatrixGFile, line);
        counter = 0;
        k = 0;

        while (k < (line.size())) {
            temp = "";

            while (isspace(line[k]) == 0 && k < line.size()) {
                temp = temp + line[k];
                k++;
            }
            k++;
            val = stoi(temp);
            matrix[i][counter] = val;
            counter++;
        }
        
    }

    MatrixGFile.close();
    return matrix;
}

int getMnumber() {
    int number = 0;
    fstream file;
    file.open("./input_pomp_be_pd/ObservMatrix.txt", ios::in);
    string line = "";
    getline(file, line, '\n');
    line.erase(std::remove(line.begin(), line.end(), ' '), line.end());
    number = line.size();
    return number;
}

int getMnumber_from_file(string file_matrix) {
    int number = 0;
    fstream file;
    file.open(file_matrix, ios::in);
    string line = "";
    getline(file, line, '\n');
    line.erase(std::remove(line.begin(), line.end(), ' '), line.end());
    number = line.size();
    return number;
}

vector<bool> getColumnObservMatrix(int id) {
    vector<bool> columnSearched(FileLength);
    ifstream file("./input_pomp_be_pd/ObservMatrix.txt", ios::in);
    string line = "";
    string temp = "";
    int i = 0;

    while (getline(file, line)) {
        line.erase(std::remove(line.begin(), line.end(), ' '), line.end());
        temp = "";
        temp += line[id];
        columnSearched[i] = stoi(temp);
        i++;
    }
    return columnSearched;
}

vector<bool> getColumnObservMatrix_fromFile(int id, string file_matrix_L) {
    vector<bool> columnSearched(FileLength);

    ifstream file(file_matrix_L, ios::in);
    string line = "";
    string temp = "";
    int i = 0;

    while (getline(file, line)) {
        line.erase(std::remove(line.begin(), line.end(), ' '), line.end());
        temp = "";
        temp += line[id];
        columnSearched[i] = stoi(temp);
        i++;
    }
    return columnSearched;
}

vector<vector<bool>> getObservMatrix_fromFile(string file_matrix_L) {
    vector<vector<bool>> rows;

    ifstream file(file_matrix_L, ios::in);
    string line = "";
    string temp = "";
    int i = 0;

    while (getline(file, line)) {
        line.erase(std::remove(line.begin(), line.end(), ' '), line.end());


        vector<bool> col;
        for (int j = 0; j < line.size(); j++) {
            temp = "";
            temp += line[j];

            col.push_back(stoi(temp));
        }
        rows.push_back(col);
        i++;
    }
    return rows;
}

bool indexInto(int vector[], int id, int length) {
    for (int i = 0; i < length; i++) {
        if (vector[i] == id) return true;
    }
    return false;
}

void getNameIstances(string ListNameIstances[]) {
    fstream ListNameIstancesFile;
    ListNameIstancesFile.open("./input_pomp_be_pd/instance_id_map.txt",
                              ios::in);

    for (int i = 0; i < objects; i++) {
        ListNameIstancesFile >> ListNameIstances[i];
    }

    ListNameIstancesFile.close();
}

bool isPresent(int idPosition, vector<bool> l, json detector_stat) {
    // used in the simulation phase
    return l[idPosition] == 1;
}

vector<vector<double>> getGaussMatrix_fromFile(string gauss_prob_matrix) {
    ifstream data(gauss_prob_matrix, ios::in);

    string line;
    vector<vector<double>> parsedCsv;
    while (getline(data, line)) {
        stringstream lineStream(line);
        string cell;
        vector<double> parsedRow;
        while (getline(lineStream, cell, ' ')) {
            double cell_d = stod(cell);
            parsedRow.push_back(cell_d);
        }
        parsedCsv.push_back(parsedRow);
    }
    return parsedCsv;
};

json getDetectorStat(int object_id) {
    // read precision and recall for each target object
    std::ifstream json_file("input_pomp_be_pd/detector_stat.json");
    json json;
    json_file >> json;

    return json[std::to_string(object_id)];
}

void updateGaussProb(int position) {
    double max = -1;
    double sum = 0;
    for (int i = 0; i < len_obsvM; i++) {
        GaussProbVector[i] = GaussProbVector[i] * gaussProbMat[position][i];
        sum += GaussProbVector[i];
    }

    for (int i = 0; i < len_obsvM; i++) GaussProbVector[i] /= sum;

    double sum_check = 0;
    for (int i = 0; i < len_obsvM; i++) sum_check += GaussProbVector[i];
    cout << "Check gauss prob:" << sum_check << "\n" << endl;
}

vector<bool> getParticlesThatAreInsideHomeByIndex(const string &filename) {
    vector<bool> values;

    ifstream input_file(filename);

    string line;
    int count = 0;
    while (getline(input_file, line)) {
        stringstream ss(line);
        string value;
        while (ss >> value) {
            if (value == "1") count++;
            values.push_back(value == "1");
        }
    }
    cout << "total particles inside home are " << count << endl;

    input_file.close();

    return values;
}

bool checkStatus(vector<double> GaussProbVector, int id_position) {

    int CHANCE = 10;
    double threshold = (1 / (double)totalNumberOfParticleinside) * CHANCE;
    double max = 0.0;
    for (int i = 0; i < len_obsvM; i++) {
        if (GaussProbVector[i] > threshold) {
            cout << "Found a value > than the thresh, now check if it is in "
                    "the fov"
                 << endl;
            if (observability_matrix[id_position][i] == 1) {
                cout << "Finishing planner, we see the object (prob="
                     << GaussProbVector[i] << ") with threshold =" << threshold
                     << "\n";
                return true;
            };
        }

        if (GaussProbVector[i] > max) {
            max = GaussProbVector[i];
        }
    }
    return false;
}

void normalize_vector(vector<double> GaussProbVector) {
    for (int i = 0; i < len_obsvM; i++) {
        GaussProbVector[i] = GaussProbVector[i] / double(len_obsvM);
    }
    return;
}

void int2vector(int num, int vector[]) {
    int bit_index = 0;
    int positions[objects];
    istancesVector(positions);
    int length = objects;

    for (int i = 0; i < length && bit_index < sizeof(int) * 8; i++) {
        if (positions[i] == 0) {
            vector[i] = 0;
        } else {
            vector[i] = (bool)(num & (1U << bit_index));
            bit_index++;
        }
    }
}
