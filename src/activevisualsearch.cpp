#include "activevisualsearch.h"

#include <stdlib.h>

#include <nlohmann/json.hpp>

#include "global_variables.h"
using json = nlohmann::json;
#include <algorithm>
#include <cstdlib>
#include <functional>
#include <random>
#include <set>
#include <vector>

#include "support.h"
#include "utils.h"
#define numObjects 12
#define numActions 6

using namespace std;
using namespace UTILS;

int fileLength = getFileLength();
int real_count = 0;
std::set<int> possible_obj_poses;
vector<int> VisitedPoses;
vector<vector<bool>> observability_matrix;

/** why we transpose this matrix? So, obs.matrix is of shape (robot_poses x
Cells_in_the_home), and indicate given a robot pose, the cells that can contains
an object. Let's talk about how the simulations work. They start from the
starting position, and they choose a random cells in the home as possible
candidate for the target.

If we tramspose the obs matrix, having shape (cells_in_the_home x robot poses)
we can optized the process (at least from a programming c++ point of view). In
fact, we choose a random location (cells_in_the_home) and we get in return all
the robot poses. So, given a cells_in_the_home, I can see if the cells (possibly
containing a target) can be seen by a certain robot pose. If you look the method
isPresent(..), you'll see that it traverse the robot poses looking for a 1, i.e
a pose that can view my target
*/
vector<bool> isParticlesInsideHome;
int totalNumberOfParticleinside;

vector<vector<bool>> P_of_X_for_simulations_T;

json detector_stat;
vector<vector<bool>> matrixClassifier = getMatrixClassifier();
int len_obsvM = getMnumber();
string listNameIstances[numObjects];
vector<vector<double>> gaussProbMat;
vector<double> GaussProbVector;
bool first_pass = true;

std::uniform_int_distribution<> distrib;
std::random_device
    rd;  // Will be used to obtain a seed for the random number engine
std::mt19937 gen(rd());

ACTIVEVISUALSEARCH::ACTIVEVISUALSEARCH(int startpos_, int target_id_,
                                       bool real_, bool penality_,
                                       string file_matrix_L_,
                                       string file_gauss_prob_matrix_,
                                       string observability_matrix_path_real_,
                                       string particle_inside_home_file_)
    : SmartMoveProb(0.95), UncertaintyCount(0) {
    NumActions = 6;
    NumObservations = 1 << 1;
    RewardRange = 1000;
    Discount = 0.97;
    matrixG = getMatrixG();

    file_matrix_L = file_matrix_L_;
    file_gauss_prob_matrix = file_gauss_prob_matrix_;
    real = real_;

    // gauss probabilites matrix - for each position, for this object target id
    gaussProbMat = getGaussMatrix_fromFile(file_gauss_prob_matrix);

    detector_stat =
        getDetectorStat(target_id_);  // get the detector stat for this home and
                                      // for this obj, i.e precision and recall
    cout << "Detector stat" << detector_stat << endl;

    real = real_;
    if (real)
        m_objs = getMnumber_from_file(file_matrix_L) - 1;
    else
        m_objs = getMnumber_from_file(
            observability_matrix_path_real_);  // return the number of cells in
                                               // the home
    cout << "m_objs" << m_objs << endl;

    observability_matrix =
        getObservMatrix_fromFile(observability_matrix_path_real_);

    if (!real && first_pass) {
        first_pass = false;
        distrib = uniform_int_distribution<>(0, m_objs);

        isParticlesInsideHome =
            getParticlesThatAreInsideHomeByIndex(particle_inside_home_file_);

        for (int i = 0; i < m_objs; i++) {  // the number of cells in the home
            bool exist = false;
            for (int j = 0; j < observability_matrix.size(); j++) {
                if (observability_matrix[j][i] == 1) exist = true;
            }
            if (exist and isParticlesInsideHome[i])
                possible_obj_poses.insert(
                    i);  // if this cells it's seen by a robot pose, it can be a
                         // possible obj. poses
        }

        int counterinside = 0;
        int counterOutside = 0;
        for (int i = 0; i < isParticlesInsideHome.size(); i++) {
            if (isParticlesInsideHome[i]) {
                counterinside++;
            } else
                counterOutside++;
        }

        totalNumberOfParticleinside = counterinside;

        cout << "---Particle inside home: " << counterinside << endl;
        cout << "---Particle outside home: " << counterOutside << endl;
        cout << "---Total: " << counterOutside + counterinside << endl;
        cout << "---END: " << endl;


        GaussProbVector.resize(isParticlesInsideHome.size());
        for (int i = 0; i < GaussProbVector.size(); i++)
            GaussProbVector[i] = isParticlesInsideHome[i] == 0
                                     ? 0
                                     : 1. / totalNumberOfParticleinside;

      
        double sum = 0;
        for (int i = 0; i < GaussProbVector.size(); i++) {
            sum += GaussProbVector[i];
        }
        cout << "---Sum Gauss prob uniform: " << sum << endl;

        vector<vector<bool>> P_of_X_for_simulations;
        double THRESHOLD = 0.01;
        for (int i = 0; i < observability_matrix.size(); i++) {
            vector<bool> rows;
            for (int j = 0; j < observability_matrix[0].size();
                 j++) {  

                if (i == startpos_) {
                    if (observability_matrix[i][j] == 1)
                        rows.push_back(GaussProbVector[j] > THRESHOLD ? 1 : 0);
                    else
                        rows.push_back(0);
                } else {
                    
                    if (observability_matrix[i][j] == 1)
                        rows.push_back(1);
                    else
                        rows.push_back(0);
                }
            }
            P_of_X_for_simulations.push_back(rows);
        }

        // transpose
        for (int j = 0; j < P_of_X_for_simulations[0].size(); j++) {
            vector<bool> col;
            for (int i = 0; i < P_of_X_for_simulations.size(); i++) {
                col.push_back(P_of_X_for_simulations[i][j]);
            }
            P_of_X_for_simulations_T.push_back(col);
        }
    }

    start_pos = startpos_;
    target_id = target_id_;
    penality = penality_;
}

STATE *ACTIVEVISUALSEARCH::Copy(const STATE &state) const {
    const ACTIVEVISUALSEARCH_STATE &rockstate =
        safe_cast<const ACTIVEVISUALSEARCH_STATE &>(state);
    ACTIVEVISUALSEARCH_STATE *newstate = MemoryPool.Allocate();
    *newstate = rockstate;
    return newstate;
}

void ACTIVEVISUALSEARCH::Validate(const STATE &state) const {}

STATE *ACTIVEVISUALSEARCH::CreateStartState() const {
    ACTIVEVISUALSEARCH_STATE *rockstate = MemoryPool.Allocate();
    rockstate->idPosition = start_pos;
    rockstate->reached = false;
    rockstate->observed_num = 0;
    int val;

    distrib = uniform_int_distribution<>(0, possible_obj_poses.size());
    val = *next(possible_obj_poses.begin(), distrib(gen));
    if (real) {
        val = target_id;
        VisitedPoses.push_back(start_pos);
    }

    rockstate->history.push_back(rockstate->idPosition);
    std::vector<bool> l_t(m_objs, 0);
    l_t[val] = 1;
    rockstate->L_t = l_t;
    if (real) {
        rockstate->L = getColumnObservMatrix_fromFile(
            val, file_matrix_L);  // not used by this version of pomp-be-pd
    } else {
        rockstate->L = P_of_X_for_simulations_T[val];
    }

    return rockstate;
}

void ACTIVEVISUALSEARCH::FreeState(STATE *state) const {
    ACTIVEVISUALSEARCH_STATE *rockstate =
        safe_cast<ACTIVEVISUALSEARCH_STATE *>(state);
    MemoryPool.Free(rockstate);
}

bool ACTIVEVISUALSEARCH::Step(STATE &state, int action, int &observation,
                              double &reward) const {
    ACTIVEVISUALSEARCH_STATE &ViewState =
        safe_cast<ACTIVEVISUALSEARCH_STATE &>(state);
    reward = 0;
    observation = 0;

    if (action < 6)  // move
    {
        switch (action) {
            case 0:  // ccw
                if (matrixG[ViewState.idPosition][0] != -1) {
                    ViewState.idPosition = matrixG[ViewState.idPosition][0];
                    break;
                } else {
                    reward = -1;
                    return false;
                }

            case 1:  // cw
                if (matrixG[ViewState.idPosition][1] != -1) {
                    ViewState.idPosition = matrixG[ViewState.idPosition][1];
                    break;
                } else {
                    reward = -1;
                    return false;
                }

            case 2:  // forward
                if (matrixG[ViewState.idPosition][2] != -1) {
                    ViewState.idPosition = matrixG[ViewState.idPosition][2];
                    break;
                } else {
                    reward = -1;
                    return false;
                }

            case 3:  // backward
                if (matrixG[ViewState.idPosition][3] != -1) {
                    ViewState.idPosition = matrixG[ViewState.idPosition][3];
                    break;
                } else {
                    reward = -1;
                    return false;
                }

            case 4:  //-left
                if (matrixG[ViewState.idPosition][4] != -1) {
                    ViewState.idPosition = matrixG[ViewState.idPosition][4];
                    break;
                } else {
                    reward = -1;
                    return false;
                }

            case 5:  // right
                if (matrixG[ViewState.idPosition][5] != -1) {
                    ViewState.idPosition = matrixG[ViewState.idPosition][5];
                    break;
                } else {
                    reward = -1;
                    return false;
                }
        }
    } else {
        cout << " #### Action id greater than 6 ####" << endl;
        return false;
    }

    if (real)  // this represent the real world, not the simulated one
        updateGaussProb(ViewState.idPosition);

    if (!real) {
        // i am in the simulated world, so I can plan ahead
        if (isPresent(ViewState.idPosition, ViewState.L, detector_stat)) {
            reward = 1000;
            observation = 1;
            ViewState.reached = true;
            return true;
        } else {
            ViewState.observed_num = 0;
            reward = -1;
            if (std::find(VisitedPoses.begin(), VisitedPoses.end(),
                          ViewState.idPosition) != VisitedPoses.end()) {
                if (penality) {
                    reward = -10;
                }
            }
            ViewState.history.push_back(ViewState.idPosition);
            return false;
        }
    }

    if (real) {
        if ((checkStatus(GaussProbVector, ViewState.idPosition))) {
            cout << "#### P(x) is greater than thresh! ####" << endl;
            reward = 1000;
            ViewState.reached = true;
            return true;
        } else {
            reward = -1;
            if (std::find(VisitedPoses.begin(), VisitedPoses.end(),
                          ViewState.idPosition) != VisitedPoses.end()) {
                if (penality) {
                    reward = -10;
                }
            }
            ViewState.history.push_back(ViewState.idPosition);

            return false;
        }
    }
}

bool ACTIVEVISUALSEARCH::LocalMove(STATE &state, const HISTORY &history,
                                   int stepObs, const STATUS &status) const {
    distrib = uniform_int_distribution<>(0, possible_obj_poses.size());

    ACTIVEVISUALSEARCH_STATE &rockstate =
        safe_cast<ACTIVEVISUALSEARCH_STATE &>(state);
    int val;
    val = *next(possible_obj_poses.begin(), distrib(gen));

    std::vector<bool> l_t(m_objs, 0);
    l_t[val] = 1;
    rockstate.L_t = l_t;
    rockstate.L = P_of_X_for_simulations_T[val];
    rockstate.history.clear();

    return true;
}

void ACTIVEVISUALSEARCH::GenerateLegal(const STATE &state,
                                       const HISTORY &history,
                                       vector<int> &legal,
                                       const STATUS &status) const {
    const ACTIVEVISUALSEARCH_STATE &ViewState =
        safe_cast<const ACTIVEVISUALSEARCH_STATE &>(state);

    legal.clear();
    int pos;

    for (int i = 0; i < numActions; i++) {
        if (matrixG[ViewState.idPosition][i] != -1) {
            pos = matrixG[ViewState.idPosition][i];
            if (pos != -1 && i != 4 && i != 5) legal.push_back(i);
        }
    }
}

void ACTIVEVISUALSEARCH::GeneratePreferred(const STATE &state,
                                           const HISTORY &history,
                                           vector<int> &actions,
                                           const STATUS &status) const {
    return;
}

int ACTIVEVISUALSEARCH::SelectTarget(
    const ACTIVEVISUALSEARCH_STATE &rockstate) const {
    return 0;
}

void ACTIVEVISUALSEARCH::DisplayBeliefs(const BELIEF_STATE &beliefState,
                                        std::ostream &ostr) const {
    std::vector<double> belief(this->matrixG.size(), 0);

    for (int i = 0; i < beliefState.GetNumSamples(); i++) {
        const ACTIVEVISUALSEARCH_STATE *pocstate =
            safe_cast<const ACTIVEVISUALSEARCH_STATE *>(
                beliefState.GetSample(i));
        std::vector<bool> l_bool = pocstate->L;
        std::vector<double> l_double(l_bool.begin(), l_bool.end());
        std::transform(belief.begin(), belief.end(), l_double.begin(),
                       belief.begin(), std::plus<double>());
    }

    for (int i = 0; i < belief.size(); i++) {
        ostr.precision(4);
        ostr << belief[i] / beliefState.GetNumSamples() << " ";
    }
    ostr << endl;
    ostr.flush();
}

void ACTIVEVISUALSEARCH::SaveGaussVector(std::ostream &ostr) const {
    for (int i = 0; i < len_obsvM; i++) {
        ostr.precision(4);
        if (i != len_obsvM - 1)
            ostr << GaussProbVector[i] << " ";
        else
            ostr << GaussProbVector[i];
    }
    ostr << endl;
}

void ACTIVEVISUALSEARCH::DisplayBeliefs_t(const BELIEF_STATE &beliefState,
                                          std::ostream &ostr) const {
    const ACTIVEVISUALSEARCH_STATE *pocstate =
        safe_cast<const ACTIVEVISUALSEARCH_STATE *>(beliefState.GetSample(0));
    std::vector<double> belief(pocstate->L_t.size(), 0);

    for (int i = 0; i < beliefState.GetNumSamples(); i++) {
        const ACTIVEVISUALSEARCH_STATE *pocstate =
            safe_cast<const ACTIVEVISUALSEARCH_STATE *>(
                beliefState.GetSample(i));
        std::vector<bool> l_bool = pocstate->L_t;
        std::vector<double> l_double(l_bool.begin(), l_bool.end());
        std::transform(belief.begin(), belief.end(), l_double.begin(),
                       belief.begin(), std::plus<double>());
    }

    for (int i = 0; i < belief.size(); i++) {
        ostr.precision(4);
        ostr << belief[i] / beliefState.GetNumSamples() << " ";
    }
    ostr << endl;
    ostr.flush();
}

void ACTIVEVISUALSEARCH::DisplayBeliefs_end(const BELIEF_STATE &beliefState,
                                            STATE &state,
                                            std::ostream &ostr) const {
    ACTIVEVISUALSEARCH_STATE &real =
        safe_cast<ACTIVEVISUALSEARCH_STATE &>(state);

    std::vector<double> belief(this->matrixG.size(), 0);

    int val = 0;
    for (int i = 0; i < beliefState.GetNumSamples(); i++) {
        const ACTIVEVISUALSEARCH_STATE *pocstate =
            safe_cast<const ACTIVEVISUALSEARCH_STATE *>(
                beliefState.GetSample(i));
        if (pocstate->L[real.idPosition] == 1) {
            val++;
            std::vector<bool> l_bool = pocstate->L;
            std::vector<double> l_double(l_bool.begin(), l_bool.end());
            std::transform(belief.begin(), belief.end(), l_double.begin(),
                           belief.begin(), std::plus<double>());
        }
    }

    for (int i = 0; i < belief.size(); i++) {
        ostr.precision(4);
        if (val > 0) {
            ostr << belief[i] / val << " ";
        } else {
            ostr << belief[i] << " ";
        }
    }
    ostr << endl;
}

void ACTIVEVISUALSEARCH::DisplayBeliefs_t_end(const BELIEF_STATE &beliefState,
                                              STATE &state,
                                              std::ostream &ostr) const {
    const ACTIVEVISUALSEARCH_STATE *pocstate =
        safe_cast<const ACTIVEVISUALSEARCH_STATE *>(beliefState.GetSample(0));

    ACTIVEVISUALSEARCH_STATE &real =
        safe_cast<ACTIVEVISUALSEARCH_STATE &>(state);
    std::vector<double> belief(pocstate->L_t.size(), 0);

    int val = 0;
    for (int i = 0; i < beliefState.GetNumSamples(); i++) {
        const ACTIVEVISUALSEARCH_STATE *pocstate =
            safe_cast<const ACTIVEVISUALSEARCH_STATE *>(
                beliefState.GetSample(i));
        if (pocstate->L[real.idPosition] == 1) {
            val++;
            std::vector<bool> l_bool = pocstate->L_t;
            std::vector<double> l_double(l_bool.begin(), l_bool.end());
            std::transform(belief.begin(), belief.end(), l_double.begin(),
                           belief.begin(), std::plus<double>());
        }
    }

    for (int i = 0; i < belief.size(); i++) {
        ostr.precision(4);
        if (val > 0) {
            ostr << belief[i] / val << " ";
        } else {
            ostr << belief[i] << " ";
        }
    }
    ostr << endl;
}

void ACTIVEVISUALSEARCH::DisplayState(const STATE &state,
                                      std::ostream &ostr) const {
    cout << " DisplayState " << endl;
    const ACTIVEVISUALSEARCH_STATE &ViewState =
        safe_cast<const ACTIVEVISUALSEARCH_STATE &>(state);
    string ListImage[fileLength];
    getListViews(ListImage);

    ostr << "Pos :";
    ostr << ListImage[ViewState.idPosition] << " Id pos "
         << ViewState.idPosition;
    ostr << endl;
}

void ACTIVEVISUALSEARCH::DisplayObservation(const STATE &state, int observation,
                                            std::ostream &ostr) const {
    const ACTIVEVISUALSEARCH_STATE &ViewState =
        safe_cast<const ACTIVEVISUALSEARCH_STATE &>(state);
    getNameIstances(listNameIstances);

    int vectorObservation[numObjects];
    int2vector(observation, vectorObservation);
    cout << " observation: " << endl;

    for (int i = 1; i < numObjects; i++) {
        if (vectorObservation[i] == 1)
            ostr << "[" << listNameIstances[i] << "]";
    }
    ostr << endl;
}

void ::ACTIVEVISUALSEARCH::DisplayProbability() const {
    cout << "Probability:[";
    for (int i = 0; i < len_obsvM; i++) cout << GaussProbVector[i] << " ";
    cout << "]" << endl;
}

void ACTIVEVISUALSEARCH::DisplayAction(int action, std::ostream &ostr) const {
    cout << " action : " << action << " ";
    ostr << endl;

    switch (action) {
        case 0:
            ostr << "ccw";
            ostr << endl;
            break;
        case 1:
            ostr << "cw";
            ostr << endl;
            break;
        case 2:
            ostr << "forward";
            ostr << endl;
            break;
        case 3:
            ostr << "backward";
            ostr << endl;
            break;
        case 4:
            ostr << "left";
            ostr << endl;
            break;
        case 5:
            ostr << "left";
            ostr << endl;
            break;
        default:
            break;
    }
}