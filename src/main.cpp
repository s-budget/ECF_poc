#include "simulator/SimulatorEngine.hpp"
#include "simulator/CityFlowEngine.hpp"
#include "simulator/SimulationRunner.hpp"
#include "agent/Agent.hpp"
#include "procesing/roadnet_loader.h"

#include <iostream>
#include <memory>
#include <vector>

#include "agent/Evaluator.hpp"
#include "agent/WorstLanesAgent.hpp"

using namespace std;

int main() {
    // redirect stdout and stderr to file for faster execution
    freopen("total_output.txt", "w", stdout);
    freopen("total_output.txt", "a", stderr);
    try {

        traffic::EngineConfig cfg;
        cfg.type        = traffic::SimulatorType::CityFlow;
        cfg.config_file = "data/cityflow_config.json";
        cfg.num_threads = 1;
        cfg.random_seed = 42;
        cfg.verbose     = false;

        auto engine = createEngine(cfg);
        engine->initialize();
        std::map<std::string, IntersectionData> intersections = loadFromConfig(cfg.config_file);
        evolution::Evaluator evaluator(engine, 700, 10, true);

        vector<shared_ptr<traffic::Agent>> agents;
        for (const auto& id : engine->getIntersectionIDs()) {
            int phases = engine->getPhaseCount(id);
            agents.push_back(make_shared<traffic::WorstLanesAgent>(id, phases, intersections[id]));
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
