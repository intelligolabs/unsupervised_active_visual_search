#include <boost/program_options.hpp>

#include "activevisualsearch.h"
#include "experiment.h"
#include "global_variables.h"
#include "mcts.h"
#include <typeinfo>
using namespace boost::program_options;
#include <iostream>
using namespace std;

int main(int argc, char *argv[]) {
    // The following code is taken and changed from the original NeurIPS
    // Monte-Carlo Planning in Large POMDPs paper at:
    // https://papers.nips.cc/paper_files/paper/2010/hash/edfbe1afcf9246bb0d40eb4d8027d90f-Abstract.html

    /////////////////////////////
    // the folder 'input_pomp_be_pd' MUST EXISTS
    // otherwise the app will start but will doing nothing.
    /////////////////////////////
    int i = 0;
    std::cout
        << "#### POMP-BE-PD policy computation! Exiting when P(X) > threshold)"
        << std::endl;

    MCTS::PARAMS searchParams;
    EXPERIMENT::PARAMS expParams;
    SIMULATOR::KNOWLEDGE knowledge;
    string problem, outputfile, policy;
    int size, number, treeknowledge = 1, rolloutknowledge = 1,
                      smarttreecount = 10;
    double smarttreevalue = 1.0;
    bool use_gt = false;
    bool penality = false;
    int target_id = 0, startpos = 0;

    options_description desc("Allowed options");
    desc.add_options()("help", "produce help message")(
        "test", "run unit tests")("problem", value<string>(&problem),
                                  "problem to run")(
        "outputfile", value<string>(&outputfile)->default_value("output.txt"),
        "summary output file")("policy", value<string>(&policy),
                               "policy file (explicit POMDPs only)")(
        "size", value<int>(&size), "size of problem (problem specific)")(
        "number", value<int>(&number), "Insert the target of activevision")(
        "timeout", value<double>(&expParams.TimeOut), "timeout (seconds)")(
        "mindoubles", value<int>(&expParams.MinDoubles),
        "minimum power of two simulations")("maxdoubles",
                                            value<int>(&expParams.MaxDoubles),
                                            "maximum power of two simulations")(
        "runs", value<int>(&expParams.NumRuns), "number of runs")(
        "accuracy", value<double>(&expParams.Accuracy),
        "accuracy level used to determine horizon")(
        "horizon", value<int>(&expParams.UndiscountedHorizon),
        "horizon to use when not discounting")(
        "numsteps", value<int>(&expParams.NumSteps),
        "number of steps to run when using average reward")(
        "verbose", value<int>(&searchParams.Verbose), "verbosity level")(
        "autoexploration", value<bool>(&expParams.AutoExploration),
        "Automatically assign UCB exploration constant")(
        "exploration", value<double>(&searchParams.ExplorationConstant),
        "Manual value for UCB exploration constant")(
        "usetransforms", value<bool>(&searchParams.UseTransforms),
        "Use transforms")(
        "transformdoubles", value<int>(&expParams.TransformDoubles),
        "Relative power of two for transforms compared to simulations")(
        "transformattempts", value<int>(&expParams.TransformAttempts),
        "Number of attempts for each transform")(
        "userave", value<bool>(&searchParams.UseRave), "RAVE")(
        "ravediscount", value<double>(&searchParams.RaveDiscount),
        "RAVE discount factor")("raveconstant",
                                value<double>(&searchParams.RaveConstant),
                                "RAVE bias constant")(
        "treeknowledge", value<int>(&knowledge.TreeLevel),
        "Knowledge level in tree (0=Pure, 1=Legal, 2=Smart)")(
        "rolloutknowledge", value<int>(&knowledge.RolloutLevel),
        "Knowledge level in rollouts (0=Pure, 1=Legal, 2=Smart)")(
        "smarttreecount", value<int>(&knowledge.SmartTreeCount),
        "Prior count for preferred actions during smart tree search")(
        "smarttreevalue", value<double>(&knowledge.SmartTreeValue),
        "Prior value for preferred actions during smart tree search")(
        "disabletree", value<bool>(&searchParams.DisableTree),
        "Use 1-ply rollout action selection")("startpos", value<int>(&startpos),
                                              "initial position")(
        "target_id", value<int>(&target_id), "id of the object to search")(
        "use_gt", value<bool>(&use_gt), "id of the object to search")(
        "penality", value<bool>(&penality));

    variables_map vm;
    store(parse_command_line(argc, argv, desc), vm);
    notify(vm);

    for (const auto& option : vm) {
        cout << option.first << " = ";
        try {
            // Dynamically determine and print value type
            const auto& value = vm[option.first].value();
            if (typeid(string) == value.type()) {
                cout << vm[option.first].as<string>();
            } else if (typeid(int) == value.type()) {
                cout << vm[option.first].as<int>();
            } else if (typeid(double) == value.type()) {
                cout << vm[option.first].as<double>();
            } else if (typeid(bool) == value.type()) {
                cout << (vm[option.first].as<bool>() ? "true" : "false");
            } else {
                cout << "<unknown type>";
            }
        } catch (const exception& e) {
            cout << "<value not set or error: " << e.what() << ">";
        }
        cout << endl;
    }

    if (vm.count("help")) {
        cout << desc << "\n";
        return 1;
    }

    if (vm.count("problem") == 0) {
        cout << "No problem specified" << endl;
        return 1;
    }

    SIMULATOR *real = 0;
    SIMULATOR *simulator = 0;

    if (problem == "activevisualsearch") {
        cout << "Starting state:" << startpos << " For target id: " << target_id
             << endl;

        string particle_inside_home_file =
            "./input_pomp_be_pd/particles_that_are_inside_home.txt";
        if (use_gt) {
            real =
                new ACTIVEVISUALSEARCH(startpos, target_id, true, penality,
                                       "./input_pomp_be_pd/MatrixL_gt.txt",
                                       "./input_pomp_be_pd/GaussObsvMatrix.txt",
                                       "./input_pomp_be_pd/ObservMatrix.txt",
                                       particle_inside_home_file);
        } else {
            real =
                new ACTIVEVISUALSEARCH(startpos, target_id, true, penality,
                                       "./input_pomp_be_pd/MatrixL_dt.txt",
                                       "./input_pomp_be_pd/GaussObsvMatrix.txt",
                                       "./input_pomp_be_pd/ObservMatrix.txt",
                                       particle_inside_home_file);
        }
        string matrixL_dt;  // simulator don't use this info
        simulator = new ACTIVEVISUALSEARCH(
            startpos, target_id, false, penality, matrixL_dt,
            "./input_pomp_be_pd/GaussObsvMatrix.txt",
            "./input_pomp_be_pd/ObservMatrix.txt", particle_inside_home_file);
    } else {
        cout << "Unknown problem" << endl;
        exit(1);
    }

    simulator->SetKnowledge(knowledge);
    EXPERIMENT experiment(*real, *simulator, outputfile, expParams,
                          searchParams);
    experiment.DiscountedReturn();

    delete real;
    delete simulator;
    return 0;
}