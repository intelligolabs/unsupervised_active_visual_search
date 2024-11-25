#ifndef ACTIVEVISUALSEARCH_H
#define ACTIVEVISUALSEARCH_H

#include <string>
#include <vector>

#include "coord.h"
#include "global_variables.h"
#include "grid.h"
#include "simulator.h"
#include "support.h"

using namespace std;

class ACTIVEVISUALSEARCH_STATE : public STATE {
   public:
    int idPosition;
    vector<bool> L;
    vector<bool> L_t;
    int val;
    bool reached;
    int observed_num;

    vector<int> history;
    int Target;
};

class ACTIVEVISUALSEARCH : public SIMULATOR {
   public:
    ACTIVEVISUALSEARCH(int startpos_, int target_id_, bool real, bool penality,
                       string file_matrix_L, string gauss_prob_matrix_,
                       string observability_matrix_path_real_,
                       string particle_inside_home_file_);

    vector<vector<int>> matrixH;
    vector<vector<int>> matrixG;
    int start_pos;
    int countsx;
    int target_id;
    string file_matrix_L;
    string file_gauss_prob_matrix;
    int m_objs;
    bool penality;
    bool real;
    virtual STATE *Copy(const STATE &state) const;
    virtual void Validate(const STATE &state) const;
    virtual STATE *CreateStartState() const;
    virtual void FreeState(STATE *state) const;
    virtual bool Step(STATE &state, int action, int &observation,
                      double &reward) const;

    virtual void DisplayProbability() const;
    void GenerateLegal(const STATE &state, const HISTORY &history,
                       std::vector<int> &legal, const STATUS &status) const;
    void GeneratePreferred(const STATE &state, const HISTORY &history,
                           std::vector<int> &legal, const STATUS &status) const;
    virtual bool LocalMove(STATE &state, const HISTORY &history,
                           int stepObservation, const STATUS &status) const;
    virtual void DisplayBeliefs(const BELIEF_STATE &beliefState,
                                std::ostream &ostr) const;
    virtual void DisplayBeliefs_t(const BELIEF_STATE &beliefState,
                                  std::ostream &ostr) const;
    virtual void DisplayBeliefs_end(const BELIEF_STATE &beliefState,
                                    STATE &state, std::ostream &ostr) const;
    virtual void DisplayBeliefs_t_end(const BELIEF_STATE &beliefState,
                                      STATE &state, std::ostream &ostr) const;
    virtual void SaveGaussVector(std::ostream &ostr) const;
    virtual void DisplayState(const STATE &state, std::ostream &ostr) const;
    virtual void DisplayObservation(const STATE &state, int observation,
                                    std::ostream &ostr) const;
    virtual void DisplayAction(int action, std::ostream &ostr) const;

   protected:
    int GetObservation(const ACTIVEVISUALSEARCH_STATE &rockstate,
                       int rock) const;
    int SelectTarget(const ACTIVEVISUALSEARCH_STATE &rockstate) const;

    double HalfEfficiencyDistance;
    double SmartMoveProb;
    int UncertaintyCount;

    int targetObj;

   private:
    mutable MEMORY_POOL<ACTIVEVISUALSEARCH_STATE> MemoryPool;
};

#endif
