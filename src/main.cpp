#include "include/simulator/SimulatorEngine.hpp"
#include "include/simulator/CityFlowEngine.hpp"
#include "include/simulator/SimulationRunner.hpp"
#include "include/agent/Agent.hpp"
#include "include/procesing/roadnet_loader.h"

#include <iostream>
#include <memory>
#include <vector>

#include "include/agent/Evaluator.hpp"
#include "include/agent/WorstLanesAgent.hpp"

using namespace std;
using namespace traffic;

int main() {
    // redirect stdout and stderr to file for faster execution
    freopen("total_output.txt", "w", stdout);
    freopen("total_output.txt", "a", stderr);
    try {
        EngineConfig cfg;
        cfg.type        = SimulatorType::CityFlow;
        cfg.config_file = "data/cityflow_config.json";
        cfg.num_threads = 1;
        cfg.random_seed = 42;
        cfg.verbose     = false;

        auto engine = createEngine(cfg);
        engine->initialize();
        map<std::string, IntersectionData> intersections = loadFromConfig(cfg.config_file);
        evolution::Evaluator evaluator(engine, 700, 10, false);

        vector<shared_ptr<Agent>> agents;
        for (const auto& id : engine->getIntersectionIDs()) {
            int phases = engine->getPhaseCount(id);
            agents.push_back(make_shared<WorstLanesAgent>(id, phases, intersections[id]));
        }
        double score = evaluator.evaluate(agents);

    } catch (const exception& e) {
        cerr << "[FATAL] " << e.what() << "\n";
        return 1;
    } catch (...) {
        cerr << "[FATAL] unknown exception\n";
        return 1;
    }
    return 0;
}
