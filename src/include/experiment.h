#ifndef EXPERIMENT_H
#define EXPERIMENT_H

#include <fstream>

#include "mcts.h"
#include "simulator.h"
#include "statistic.h"

//----------------------------------------------------------------------------

struct RESULTS {
    void Clear();

    STATISTIC Time;
    STATISTIC Reward;
    STATISTIC DiscountedReturn;
    STATISTIC UndiscountedReturn;
    STATISTIC steps;
};

inline void RESULTS::Clear() {
    Time.Clear();
    Reward.Clear();
    DiscountedReturn.Clear();
    UndiscountedReturn.Clear();
}

//----------------------------------------------------------------------------

class EXPERIMENT {
   public:
    struct PARAMS {
        PARAMS();

        int NumRuns;
        int NumSteps;
        int SimSteps;
        double TimeOut;
        int MinDoubles, MaxDoubles;
        int TransformDoubles;
        int TransformAttempts;
        double Accuracy;
        int UndiscountedHorizon;
        bool AutoExploration;
    };

    EXPERIMENT(const SIMULATOR& real, const SIMULATOR& simulator,
               const std::string& outputFile, EXPERIMENT::PARAMS& expParams,
               MCTS::PARAMS& searchParams);

    void Run(int nStates, int run);
    void MultiRun();
    void MultiRun(int nStates);
    void DiscountedReturn();
    void AverageReward();

   private:
    const SIMULATOR& Real;
    const SIMULATOR& Simulator;
    EXPERIMENT::PARAMS& ExpParams;
    MCTS::PARAMS& SearchParams;
    RESULTS Results;

    std::ofstream OutputFile;
};

//----------------------------------------------------------------------------

#endif  // EXPERIMENT_H
