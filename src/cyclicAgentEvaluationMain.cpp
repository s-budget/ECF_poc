#include "include/simulator/SimulatorEngine.hpp"
#include "include/simulator/CityFlowEngine.hpp"
#include "include/procesing/roadnet_loader.h"

#include <iostream>
#include <memory>
#include <vector>
#include <map>
#include <filesystem>
#include "include/agent/CyclicAgent.hpp"

#include "util.h"
#include "include/simulator/SimulationRunner.hpp"

using namespace std;
using namespace traffic;
namespace fs = std::filesystem;

int cyclicEvaluation_main(int argc, char** argv)
{

    EngineConfig cfg;
    cfg.type        = SimulatorType::CityFlow;
    cfg.config_file = "src/data/cityflow_config_hangzhou_4x4.json";
    cfg.num_threads = 1;
    cfg.random_seed = 42;
    cfg.verbose     = false;

    auto engine = createEngine(cfg);
    engine->initialize();

    map<string, IntersectionData> intersections = loadFromConfig(cfg.config_file);

    vector<shared_ptr<Agent>> agents;
    for (const auto& id : engine->getIntersectionIDs()) {
        int phases = engine->getPhaseCount(id);


        agents.push_back(
            make_shared<CyclicAgent>(
                id, phases, intersections.at(id)));
    }

    traffic::SimulationRunner runner(engine, 3600, 30, false, 5);
    runner.setup();

    traffic::SimulationState sState = runner.run(agents);

    // Write ensemble result file
    {
        string filename = "experiments/CYCLIC_RESULTS.txt";
        ofstream f(filename);
        f << sState.mean_travel_time << '\n';
        f << sState.max_queue_size_until_now << '\n';
        f << sState.throughput << '\n';

        cout << "Ensemble result written to: " << filename << '\n';
    }

    return static_cast<int>(sState.mean_travel_time);
}

