#include "include/simulator/SimulatorEngine.hpp"
#include "include/simulator/CityFlowEngine.hpp"
#include "include/procesing/roadnet_loader.h"
#include "include/agent/GPLightAgent.hpp"
#include "include/agent/EnsembleGPLightAgent.hpp"

#include <iostream>
#include <memory>
#include <vector>
#include <map>
#include <filesystem>

#include "util.h"
#include "include/agent/GplightagentWihtCurrentPhase.hpp"
#include "include/simulator/SimulationRunner.hpp"

using namespace std;
using namespace traffic;
namespace fs = std::filesystem;


/**
 * @brief Evaluates every individual GP light agent separately.
 *
 * Loads all available trained experiments, creates the corresponding
 * intersection agents, and runs one simulation for each experiment.
 * The resulting fitness values are stored for later ensemble creation.
 */
int evaluation_for_all_singles_main(int argc, char** argv)
{
    // Configure and initialize the traffic simulation engine.
    EngineConfig cfg;
    cfg.type        = SimulatorType::CityFlow;
    cfg.config_file = "src/data/cityflow_config_hangzhou_4x4.json";
    cfg.num_threads = 1;
    cfg.random_seed = 42;
    cfg.verbose     = false;

    auto engine = createEngine(cfg);
    engine->initialize();

    map<string, IntersectionData> intersections = loadFromConfig(cfg.config_file);


    // Find all previously generated GP experiments available on disk.
    vector<int> all_ids;
    for (int i = 0; i <= 999; ++i)
    {
        ostringstream ss;
        ss << "experiments/experiment_" << setw(3) << setfill('0') << i;

        if (fs::exists(ss.str()))
            all_ids.push_back(i);
    }


    // Load the trained agent corresponding to every experiment for every intersection.
    map<string, vector<shared_ptr<GPLightAgentWithCurrentPhase>>> all_agents =
        util::makeGPLightAgentsWithCurrentFromExperiments(all_ids, engine, intersections);

    vector<string> intersection_ids = engine->getIntersectionIDs();


    // Run a separate simulation for every single experiment agent.
    for (size_t k = 0; k < all_ids.size(); ++k)
    {
        int experiment_id = all_ids[k];

        vector<shared_ptr<Agent>> trial_agents;

        // Select the same experiment agent across all intersections
        // to form a complete traffic control system.
        for (const auto& intersection_id : intersection_ids)
        {
            trial_agents.push_back(all_agents.at(intersection_id).at(k));
        }


        // Evaluate the selected single-agent configuration.
        traffic::SimulationRunner trial_runner(engine, 3600, 5, false, 5);
        trial_runner.setup();

        traffic::SimulationState trial_state = trial_runner.run(trial_agents);


        // Store the fitness results of this individual experiment.
        string filename = "experiments/SINGLE_AGENT_FITNESS_FOR_" +
                           to_string(experiment_id) + ".txt";

        ofstream f(filename);
        f << trial_state.mean_travel_time << '\n';
        f << trial_state.max_queue_size_until_now << '\n';
        f << trial_state.throughput << '\n';
        f << experiment_id << '\n';

        cout << "Ensemble result written to: " << filename << '\n';
    }

    return 0;
}