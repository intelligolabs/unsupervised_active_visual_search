#include "experiment.h"

#include <boost/format.hpp>
#include <fstream>

#include "activevisualsearch.h"
#include "boost/timer.hpp"
#include "global_variables.h"
#include "support.h"

using namespace std;
using boost::format;

EXPERIMENT::PARAMS::PARAMS()
    : NumRuns(1000),
      NumSteps(100000),
      SimSteps(1000),
      TimeOut(3600),
      MinDoubles(0),
      MaxDoubles(20),
      TransformDoubles(-1),
      TransformAttempts(1000),
      Accuracy(0.01),
      UndiscountedHorizon(1000),
      AutoExploration(true) {}

EXPERIMENT::EXPERIMENT(const SIMULATOR &real, const SIMULATOR &simulator,
                       const string &outputFile, EXPERIMENT::PARAMS &expParams,
                       MCTS::PARAMS &searchParams)
    : Real(real),
      Simulator(simulator),
      OutputFile(outputFile.c_str()),
      ExpParams(expParams),
      SearchParams(searchParams) {
    if (ExpParams.AutoExploration) {
        if (SearchParams.UseRave)
            SearchParams.ExplorationConstant = 0;
        else
            SearchParams.ExplorationConstant = simulator.GetRewardRange();
    }
    MCTS::InitFastUCB(SearchParams.ExplorationConstant);
}

void EXPERIMENT::Run(int nStates, int run) {
    boost::timer timer;
    cout << "Start" << endl;

    boost::format fmt =
        boost::format("./out/position_%1%_%2%.txt") % nStates % run;
    std::string file_name = fmt.str();
    std::ofstream SaveFile(file_name);

    boost::format fmt2 =
        boost::format("./out/belief_%1%_%2%.txt") % nStates % run;
    std::string file_name_belief = fmt2.str();
    std::ofstream SaveFile_belief(file_name_belief);

    boost::format fmt3 =
        boost::format("./out/belief_t_%1%_%2%.txt") % nStates % run;
    std::string file_name_belief_t = fmt3.str();
    std::ofstream SaveFile_belief_t(file_name_belief_t);

    boost::format fmt4 =
        boost::format("./out/gauss_vector_prob%1%_%2%.txt") % nStates % run;
    std::string gauss_probability_file = fmt4.str();
    std::ofstream SaveGauss(gauss_probability_file);

    cout << "Printing config" << endl;
    cout << "--------------------------" << endl;

    cout << "Maximum Step number:" << ExpParams.NumSteps << endl;
    cout << "Simulator starting states number:" << SearchParams.NumStartStates
         << endl;
    cout << "--------------------------" << endl;

    double undiscountedReturn = 0.0;
    double discountedReturn = 0.0;
    double discount = 1.0;
    bool terminal = false;
    bool outOfParticles = false;
    int t;

    STATE *realState = Real.CreateStartState();

    Real.SaveGaussVector(SaveGauss);

    MCTS mcts(Simulator, SearchParams);

    cout << "ExpParams.NumSteps:" << ExpParams.NumSteps << endl;
    for (t = 0; t < 150; t++) {
        cout << "######################### Running iteration "
                "#########################:"
             << t << endl;
        int observation;
        double reward;
        int action = mcts.SelectAction();  // run all the simulations and then
                                           // choose the best step
        terminal = Real.Step(*realState, action, observation,
                             reward);  // does the real step

        ACTIVEVISUALSEARCH_STATE &realStateAfterStep =
            safe_cast<ACTIVEVISUALSEARCH_STATE &>(
                *realState);  

        VisitedPoses.push_back(realStateAfterStep.idPosition);
        int count = 0;
        for (int cell = 0; cell < observability_matrix[0].size();
             cell++) {  
            if (observability_matrix[realStateAfterStep.idPosition][cell]) {
                count++;
                possible_obj_poses.erase(cell);
            }
        }
        cout << "removed " << count << " poses ("
             << observability_matrix[0].size() << ") out of a total of"
             << possible_obj_poses.size()
             << "[obj:" << observability_matrix.size() << "]" << endl;
        cout << "current count: " << possible_obj_poses.size() << endl;
        cout << "obj: " << observability_matrix[0].size() << endl;

        // terminal=observation;
        ACTIVEVISUALSEARCH_STATE &rockstate =
            safe_cast<ACTIVEVISUALSEARCH_STATE &>(*realState);
        SaveFile << action;
        SaveFile << "\n";

        Results.Reward.Add(reward);
        undiscountedReturn += reward;
        discountedReturn += reward * discount;
        discount *= Real.GetDiscount();

        if (SearchParams.Verbose >= 1) {
            Real.DisplayAction(action, cout);
            Real.DisplayState(*realState, cout);
            
            Real.DisplayReward(reward, cout);
            Real.DisplayBeliefs(mcts.BeliefState(), SaveFile_belief);
            Real.DisplayBeliefs_t(mcts.BeliefState(), SaveFile_belief_t);
            Real.SaveGaussVector(SaveGauss);
           
        }

        if (terminal) {
            SaveFile << "Object reached" << endl;
            cout << "Terminated" << endl;
            break;
        }

        
        outOfParticles =
            !mcts.Update(action, observation, reward); 
        if (outOfParticles) break;

        if (timer.elapsed() > ExpParams.TimeOut) {
            cout << "Timed out after " << t << " steps in "
                 << Results.Time.GetTotal() << "seconds" << endl;
            break;
        }
    }
    

    SaveFile.close();
    SaveFile_belief.close();
    SaveFile_belief_t.close();
    SaveGauss.close();
    if (outOfParticles) {
        cout << "Out of particles, finishing episode with SelectRandom" << endl;
        HISTORY history = mcts.GetHistory();
        while (++t < ExpParams.NumSteps) {
            int observation;
            double reward;

            int action =
                Simulator.SelectRandom(*realState, history, mcts.GetStatus());
            terminal = Real.Step(*realState, action, observation, reward);

            Results.Reward.Add(reward);
            undiscountedReturn += reward;
            discountedReturn += reward * discount;
            discount *= Real.GetDiscount();

            if (SearchParams.Verbose >= 1) {
                Real.DisplayAction(action, cout);
                Real.DisplayState(*realState, cout);
                Real.DisplayObservation(*realState, observation, cout);
                Real.DisplayReward(reward, cout);
            }

            if (terminal) {
                cout << "Terminated" << endl;
                break;
            }

            history.Add(action, observation);
        }
    }

    Results.Time.Add(timer.elapsed());
    Results.UndiscountedReturn.Add(undiscountedReturn);
    Results.DiscountedReturn.Add(discountedReturn);
    Results.steps.Add(t);
    cout << "Discounted return = " << discountedReturn
         << ", average = " << Results.DiscountedReturn.GetMean() << endl;
    cout << "Undiscounted return = " << undiscountedReturn
         << ", average = " << Results.UndiscountedReturn.GetMean() << endl;
}

void EXPERIMENT::MultiRun(int nStates) {
    for (int n = 0; n < ExpParams.NumRuns; n++) {
        cout << "Starting run " << n + 1 << " with "
             << SearchParams.NumSimulations << " simulations... " << endl;
        Run(nStates, n);
        if (Results.Time.GetTotal() > ExpParams.TimeOut) {
            cout << "Timed out after " << n << " runs in "
                 << Results.Time.GetTotal() << "seconds" << endl;
            break;
        }
    }
}

void EXPERIMENT::DiscountedReturn() {
    cout << "Main runs" << endl;
    OutputFile << "Simulations\tRuns\tSteps\tUndiscounted return\tUndiscounted "
                  "error\tDiscounted return\tDiscounted error\tTime\n";

    SearchParams.MaxDepth =
        Simulator.GetHorizon(ExpParams.Accuracy, ExpParams.UndiscountedHorizon);
    ExpParams.SimSteps =
        Simulator.GetHorizon(ExpParams.Accuracy, ExpParams.UndiscountedHorizon);
    ExpParams.NumSteps =
        Real.GetHorizon(ExpParams.Accuracy, ExpParams.UndiscountedHorizon);

    for (int i = ExpParams.MinDoubles; i <= ExpParams.MaxDoubles; i++) {
        SearchParams.NumSimulations = 1 << i;
        SearchParams.NumStartStates = 1 << i;
        if (i + ExpParams.TransformDoubles >= 0)
            SearchParams.NumTransforms = 1 << (i + ExpParams.TransformDoubles);
        else
            SearchParams.NumTransforms = 1;
        SearchParams.MaxAttempts =
            SearchParams.NumTransforms * ExpParams.TransformAttempts;

        Results.Clear();
        MultiRun(i);

        cout << "Simulations = " << SearchParams.NumSimulations << endl
             << "Runs = " << Results.Time.GetCount() << endl
             << "Undiscounted return = " << Results.UndiscountedReturn.GetMean()
             << " +- " << Results.UndiscountedReturn.GetStdErr() << endl
             << "Discounted return = " << Results.DiscountedReturn.GetMean()
             << " +- " << Results.DiscountedReturn.GetStdErr() << endl
             << "Time = " << Results.Time.GetMean() << endl;
        OutputFile << SearchParams.NumSimulations << "\t"
                   << Results.Time.GetCount() << "\t" << Results.steps.GetMean()
                   << "\t" << Results.UndiscountedReturn.GetMean() << "\t"
                   << Results.UndiscountedReturn.GetStdErr() << "\t"
                   << Results.DiscountedReturn.GetMean() << "\t"
                   << Results.DiscountedReturn.GetStdErr() << "\t"
                   << Results.Time.GetMean() << endl;
    }
}

void EXPERIMENT::AverageReward() {
    cout << "Main runs" << endl;
    OutputFile << "Simulations\tSteps\tAverage reward\tAverage time\n";

    SearchParams.MaxDepth =
        Simulator.GetHorizon(ExpParams.Accuracy, ExpParams.UndiscountedHorizon);
    ExpParams.SimSteps =
        Simulator.GetHorizon(ExpParams.Accuracy, ExpParams.UndiscountedHorizon);

    for (int i = ExpParams.MinDoubles; i <= ExpParams.MaxDoubles; i++) {
        SearchParams.NumSimulations = 1 << i;
        SearchParams.NumStartStates = 1 << i;
        if (i + ExpParams.TransformDoubles >= 0)
            SearchParams.NumTransforms = 1 << (i + ExpParams.TransformDoubles);
        else
            SearchParams.NumTransforms = 1;
        SearchParams.MaxAttempts =
            SearchParams.NumTransforms * ExpParams.TransformAttempts;

        Results.Clear();
        Run(i, 0);

        cout << "Simulations = " << SearchParams.NumSimulations << endl
             << "Steps = " << Results.Reward.GetCount() << endl
             << "Average reward = " << Results.Reward.GetMean() << " +- "
             << Results.Reward.GetStdErr() << endl
             << "Average time = "
             << Results.Time.GetMean() / Results.Reward.GetCount() << endl;
        OutputFile << SearchParams.NumSimulations << "\t"
                   << Results.Reward.GetCount() << "\t"
                   << Results.Reward.GetMean() << "\t"
                   << Results.Reward.GetStdErr() << "\t"
                   << Results.Time.GetMean() / Results.Reward.GetCount()
                   << endl;
    }
}

//----------------------------------------------------------------------------
