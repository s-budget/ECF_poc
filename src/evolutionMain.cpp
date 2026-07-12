#include <ecf/ECF.h>

#include "include/simulator/SimulatorEngine.hpp"
#include "include/simulator/CityFlowEngine.hpp"
#include "include/procesing/roadnet_loader.h"
#include "include/agent/GplightagentWihtCurrentPhase.hpp"
#include "include/agent/ImprovedGPLightAgent.hpp"

#include "include/evolution/GPLightEvalOp.hpp"

#include <iostream>
#include <memory>
#include <vector>

#include "util.h"
#include "include/agent/Evaluator.hpp"

#include "evolutionMain.h"

using namespace std;
using namespace traffic;


/**
 * @brief Runs one genetic programming optimization process for traffic light control.
 *
 * Creates the simulation environment, initializes GP-based traffic light agents,
 * configures the evolutionary algorithm, and saves the best discovered urgency
 * function and its fitness result.
 */
int evolution_main(int argc, char** argv, bool verbose)
{
    // Disable console output for parallel runs or enable output logging for debugging.
    if (!verbose) {
        cout.rdbuf(nullptr);
        cerr.rdbuf(nullptr);
    }
    else {
        freopen("total_output.txt", "w", stdout);
        freopen("total_output.txt", "a", stderr);
    }

    // Reserve a unique directory for storing this experiment's results.
    std::string experiment_dir = util::reserve_experiment_dir();


    // Configure and initialize the traffic simulation engine.
    EngineConfig cfg;
    cfg.type        = SimulatorType::CityFlow;
    cfg.config_file = "src/data/cityflow_config_hangzhou_4x4.json";
    cfg.num_threads = 1;
    cfg.random_seed = 42;
    cfg.verbose     = false;

    auto engine = createEngine(cfg);
    engine->initialize();


    // Load intersection information and create the evaluator used by evolution.
    map<string, IntersectionData> intersections = loadFromConfig(cfg.config_file);
    evolution::Evaluator evaluator(engine, 3600, 5,5, false);


    // Create one GP-controlled agent for every intersection in the simulation.
    vector<shared_ptr<GPLightAgentWithCurrentPhase>> agents;
    for (const auto& id : engine->getIntersectionIDs()) {
        int phases = engine->getPhaseCount(id);

        agents.push_back(
            make_shared<GPLightAgentWithCurrentPhase>(id, phases, intersections[id])
        );
    }


    // Connect the evolutionary evaluation operator with the traffic agents.
    auto evalOp = std::make_shared<evolution::GPLightEvalOpCurrent>();
    evalOp->setup(evaluator, agents);


    // Create the GP tree genotype and configure the ECF evolutionary state.
    TreeP tree = make_shared<Tree::Tree>();
    StateP state(new State);
    state->addGenotype(tree);
    state->setEvalOp(evalOp);


    // Initialize ECF using the configured evolutionary parameters.
    const char* ecfArgv[] = { argv[0], "src/data/ecf_gplightplusWithCurrentPhase.xml" };
    int         ecfArgc   = 2;

    if (!state->initialize(ecfArgc, const_cast<char**>(ecfArgv))) {
        cerr << "[FATAL] ECF failed to initialise.\n";
        return 1;
    }


    // Run the evolutionary search for the best traffic light control function.
    state->run();


    // Retrieve and save the best evolved GP tree and its fitness value.
    IndividualP best = state->getHoF()->getBest().at(0);
    Tree::Tree* bestTree = static_cast<Tree::Tree*>(best->getGenotype().get());

    cout << "\nBest TM urgency function:\n";
    cout << bestTree->toString() << "\n";
    cout << "\nFitness (avg travel time): "
         << best->fitness->getValue() << " s\n";


    // Store the evolved tree and fitness for later ensemble construction.
    XMLNode xml;
    bestTree->write(xml);
    xml.writeToFile((experiment_dir+"/best_tree.xml").c_str());

    std::ofstream fitnessFile(experiment_dir+"/fitness.txt");
    fitnessFile << best->fitness->getValue() << "\n";


    // Save additional simulation results when verbose mode is enabled.
    if (verbose) {
        util::save_all_relevant_results(experiment_dir);
    }

    return 0;
}