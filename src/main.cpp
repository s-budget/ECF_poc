#include <ecf/ECF.h>

#include "include/simulator/SimulatorEngine.hpp"
#include "include/simulator/CityFlowEngine.hpp"
#include "include/procesing/roadnet_loader.h"
#include "include/agent/GPLightAgent.hpp"
#include "include/evolution/GPLightEvalOp.hpp"

#include <iostream>
#include <memory>
#include <vector>

#include "include/agent/Evaluator.hpp"

using namespace std;
using namespace traffic;

int main(int argc, char** argv)
{
    freopen("total_output.txt", "w", stdout);
    freopen("total_output.txt", "a", stderr);
    EngineConfig cfg;
    cfg.type        = SimulatorType::CityFlow;
    cfg.config_file = "src/data/cityflow_config.json";
    cfg.num_threads = 1;
    cfg.random_seed = 42;
    cfg.verbose     = false;

    auto engine = createEngine(cfg);
    engine->initialize();

    map<string, IntersectionData> intersections = loadFromConfig(cfg.config_file);
    evolution::Evaluator evaluator(engine, 700, 10, false);

    vector<shared_ptr<GPLightAgent>> agents;
    for (const auto& id : engine->getIntersectionIDs()) {
        int phases = engine->getPhaseCount(id);
        agents.push_back(
            make_shared<GPLightAgent>(id, phases, intersections[id])
        );
    }

    auto evalOp = make_shared<evolution::GPLightEvalOp>();
    evalOp->setup(evaluator, agents);

    TreeP tree = make_shared<Tree::Tree>();
    StateP state(new State);
    state->addGenotype(tree);
    state->setEvalOp(evalOp);

    const char* ecfArgv[] = { argv[0], "src/data/ecf_gplightplus.xml" };
    int         ecfArgc   = 2;

    if (!state->initialize(ecfArgc, const_cast<char**>(ecfArgv))) {
        cerr << "[FATAL] ECF failed to initialise.\n";
        return 1;
    }

    state->run();

    IndividualP best = state->getHoF()->getBest().at(0);
    Tree::Tree* bestTree = static_cast<Tree::Tree*>(best->getGenotype().get());
    cout << "\nBest TM urgency function:\n";
    cout << bestTree->toString() << "\n";
    cout << "\nFitness (avg travel time): "
         << best->fitness->getValue() << " s\n";

    return 0;
}