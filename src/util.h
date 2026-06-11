#pragma once
#include <ecf/ECF.h>

struct IntersectionData;

namespace traffic {
    class SimulatorEngine;
    class GPLightAgent;
}

namespace util
{
    bool save_all_relevant_results();

    TreeP load_tree_from_experiment(
        int experimentNumber,
        StateP state
    );

    std::map<std::string, std::vector<std::shared_ptr<traffic::GPLightAgent>>>
makeGPLightAgentsFromExperiments(
    const std::vector<int>&                                  experiment_ids,
    const std::shared_ptr<traffic::SimulatorEngine>&                  engine,
    const std::map<std::string, IntersectionData>&  intersections);
}
