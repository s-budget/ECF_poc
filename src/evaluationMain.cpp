#include "include/simulator/SimulatorEngine.hpp"
#include "include/simulator/CityFlowEngine.hpp"
#include "include/procesing/roadnet_loader.h"
#include "include/agent/GPLightAgent.hpp"
#include "include/agent/EnsembleGPLightAgent.hpp"

#include <iostream>
#include <memory>
#include <vector>
#include <map>

#include "util.h"
#include "include/simulator/SimulationRunner.hpp"

using namespace std;
using namespace traffic;

int evaluation_main(int argc, char** argv)
{
    freopen("total_output.txt", "w", stdout);
    freopen("total_output.txt", "a", stderr);

    const vector<int> experiment_ids = { 1, 2 };

    EngineConfig cfg;
    cfg.type        = SimulatorType::CityFlow;
    cfg.config_file = "src/data/cityflow_config_hangzhou_4x4.json";
    cfg.num_threads = 1;
    cfg.random_seed = 42;
    cfg.verbose     = false;

    auto engine = createEngine(cfg);
    engine->initialize();

    map<string, IntersectionData> intersections = loadFromConfig(cfg.config_file);

    auto agents_per_intersection =
        util::makeGPLightAgentsFromExperiments(experiment_ids, engine, intersections);

    vector<shared_ptr<Agent>> ensemble_agents;
    for (const auto& id : engine->getIntersectionIDs()) {
        int phases = engine->getPhaseCount(id);

        vector<shared_ptr<Agent>> sub_agents(
            agents_per_intersection[id].begin(),
            agents_per_intersection[id].end());

        ensemble_agents.push_back(
            make_shared<EnsembleGPLightAgent>(
                id, phases, intersections.at(id),
                std::move(sub_agents)));
    }

    traffic::SimulationRunner runner(engine, 700, 5, false, 5);
    runner.setup();

    traffic::SimulationState sState = runner.run(ensemble_agents);
    cout << "Mean travel time: " << sState.mean_travel_time << " s\n";

    return static_cast<int>(sState.mean_travel_time);
}