#include "util.h"

#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <ecf/ECF.h>

#include "include/agent/Evaluator.hpp"
#include "include/evolution/GPLightEvalOp.hpp"
#include "include/procesing/roadnet_loader.h"
#include "include/simulator/SimulatorEngine.hpp"


namespace traffic {
    class GPLightAgent;
}

namespace fs = std::filesystem;

namespace util
{

bool save_all_relevant_results()
{
    static const std::vector<std::string> files_to_copy =
    {
        "log.txt",
        "total_output.txt",
        "src/best_tree.xml",
        "src/data/replay.txt",
        "src/data/replay_roadnet.json"
    };

    fs::path experiment_dir;

    bool found = false;

    for (int i = 1; i <= 999; ++i)
    {
        std::ostringstream ss;
        ss << "experiments/experiment_"
           << std::setw(3)
           << std::setfill('0')
           << i;

        fs::path candidate(ss.str());

        if (!fs::exists(candidate))
        {
            experiment_dir = candidate;
            found = true;
            break;
        }
    }

    if (!found)
    {
        std::cerr << "No available experiment directory names.\n";
        return false;
    }

    fs::create_directory(experiment_dir);

    for (const auto& file : files_to_copy)
    {
        fs::path source(file);

        if (!fs::exists(source))
        {
            std::cerr << "Warning: file not found: "
                      << source << '\n';
            continue;
        }

        fs::path destination =
            experiment_dir / source.filename();

        try
        {
            fs::copy_file(
                source,
                destination,
                fs::copy_options::overwrite_existing
            );
        }
        catch (const std::exception& e)
        {
            std::cerr << "Failed to copy "
                      << source
                      << ": "
                      << e.what()
                      << '\n';
        }
    }

    std::cout << "Results saved to: "
              << experiment_dir.string()
              << '\n';

    return true;
}

TreeP load_tree_from_experiment(
    int experimentNumber, StateP state)
{
    if (experimentNumber < 1 || experimentNumber > 999)
    {
        throw std::runtime_error(
            "Experiment number must be between 1 and 999");
    }

    std::ostringstream ss;

    ss << "experiments/experiment_"
       << std::setw(3)
       << std::setfill('0')
       << experimentNumber
       << "/best_tree.xml";

    std::string filename = ss.str();

    if (!fs::exists(filename))
    {
        throw std::runtime_error(
            "Tree file does not exist: " + filename);
    }

    XMLNode xml =
        XMLNode::openFileHelper(
            filename.c_str(),
            "Tree");

    TreeP tree = std::shared_ptr<Tree::Tree>(
    static_cast<Tree::Tree*>(
        state->getIndividualObject()->getGenotype(0)->copy()));
    tree->read(xml);


    return tree;
}

std::map<std::string, std::vector<std::shared_ptr<traffic::GPLightAgent>>>
makeGPLightAgentsFromExperiments(
    const std::vector<int>&                                  experiment_ids,
    const std::shared_ptr<traffic::SimulatorEngine>&                  engine,
    const std::map<std::string, IntersectionData>&  intersections)
{
    // Build ECF state
    std::vector<std::shared_ptr<traffic::GPLightAgent>> agent_templates;
    for (const auto& id : engine->getIntersectionIDs()) {
        int phases = engine->getPhaseCount(id);
        agent_templates.push_back(
            std::make_shared<traffic::GPLightAgent>(id, phases, intersections.at(id)));
    }

    evolution::Evaluator evaluator(engine, 700, 5, 5, false);
    auto evalOp = std::make_shared<evolution::GPLightEvalOpBasic>();
    evalOp->setup(evaluator, agent_templates);

    TreeP  proto_tree(std::make_shared<Tree::Tree>());
    StateP state(new State);
    state->addGenotype(proto_tree);
    state->setEvalOp(evalOp);

    const char* ecfArgv[] = { "app", "src/data/ecf_gplightplus.xml" };
    int         ecfArgc   = 2;

    if (!state->initialize(ecfArgc, const_cast<char**>(ecfArgv)))
        throw std::runtime_error("[FATAL] ECF failed to initialise.");

    std::map<std::string, std::vector<std::shared_ptr<traffic::GPLightAgent>>> result;

    for (const auto& id : engine->getIntersectionIDs()) {
        int phases = engine->getPhaseCount(id);
        const IntersectionData& idata = intersections.at(id);

        for (int exp_id : experiment_ids) {
            TreeP tree = load_tree_from_experiment(exp_id, state);

            auto agent = std::make_shared<traffic::GPLightAgent>(id, phases, idata);
            agent->setTree(tree.get());

            result[id].push_back(
                std::shared_ptr<traffic::GPLightAgent>(
                    agent.get(),
                    [agent, tree](traffic::GPLightAgent*) mutable {
                        (void)agent;
                        (void)tree;
                    }));
        }
    }

    return result;
}

} // namespace util

