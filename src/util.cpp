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


// Forward declaration of GPLightAgent to avoid including its full definition here.
namespace traffic {
    class GPLightAgent;
}

namespace fs = std::filesystem;

namespace util
{

// Creates a new experiment directory with the first available ID.
// Example: experiments/experiment_001, experiments/experiment_002, ...
std::string reserve_experiment_dir()
{
    // Search through possible experiment IDs.
    for (int i = 1; i <= 999; ++i)
    {
        std::ostringstream ss;

        // Build directory name with three digit numbering.
        ss << "experiments/experiment_"
           << std::setw(3) << std::setfill('0') << i;

        fs::path candidate(ss.str());

        // Reserve the first directory name that does not already exist.
        if (!fs::exists(candidate))
        {
            fs::create_directory(candidate);

            std::cout << "Experiment directory reserved: "
                      << candidate.string() << '\n';

            return candidate.string();
        }
    }

    // No free experiment IDs remain.
    throw std::runtime_error("No available experiment directory names.");
}


// Copies important files produced during an experiment into the experiment folder.
// These files contain logs, simulation replay data, and debugging information.
bool save_all_relevant_results(const std::string& experiment_dir)
{
    // Files that should be preserved for later inspection.
    static const std::vector<std::string> files_to_copy =
    {
        "log.txt",
        "total_output.txt",
        "src/data/replay.txt",
        "src/data/replay_roadnet.json"
    };

    fs::path dir(experiment_dir);

    // Copy every relevant file if it exists.
    for (const auto& file : files_to_copy)
    {
        fs::path source(file);

        if (!fs::exists(source))
        {
            std::cerr << "Warning: file not found: " << source << '\n';
            continue;
        }

        try
        {
            // Copy the file into the experiment directory.
            fs::copy_file(
                source,
                dir / source.filename(),
                fs::copy_options::overwrite_existing
            );
        }
        catch (const std::exception& e)
        {
            std::cerr << "Failed to copy " << source
                      << ": " << e.what() << '\n';
        }
    }

    std::cout << "Results saved to: " << experiment_dir << '\n';

    return true;
}


// Loads a previously evolved GP tree from an experiment directory.
//
// The saved tree represents the evolved traffic light decision function.
TreeP load_tree_from_experiment(
    int experimentNumber, StateP state)
{
    // Validate experiment ID.
    if (experimentNumber < 1 || experimentNumber > 999)
    {
        throw std::runtime_error(
            "Experiment number must be between 1 and 999");
    }

    std::ostringstream ss;

    // Construct path to saved GP tree.
    ss << "experiments/experiment_"
       << std::setw(3)
       << std::setfill('0')
       << experimentNumber
       << "/best_tree.xml";

    std::string filename = ss.str();

    // Ensure the tree file exists.
    if (!fs::exists(filename))
    {
        throw std::runtime_error(
            "Tree file does not exist: " + filename);
    }

    // Load XML representation of the tree.
    XMLNode xml =
        XMLNode::openFileHelper(
            filename.c_str(),
            "Tree");

    // Create a new tree object using the same genotype type as the ECF state.
    TreeP tree = std::shared_ptr<Tree::Tree>(
        static_cast<Tree::Tree*>(
            state->getIndividualObject()->getGenotype(0)->copy()));

    // Fill the tree object with data from XML.
    tree->read(xml);

    return tree;
}


// Creates GPLight agents for a list of evolved experiments.
//
// Each experiment corresponds to one evolved GP controller.
// The returned map groups agents by intersection ID.
// For every intersection there is one agent for each selected experiment.
std::map<std::string, std::vector<std::shared_ptr<traffic::GPLightAgent>>>
makeGPLightAgentsFromExperiments(
    const std::vector<int>&                                  experiment_ids,
    const std::shared_ptr<traffic::SimulatorEngine>&                  engine,
    const std::map<std::string, IntersectionData>&  intersections)
{
    // Create temporary agents used only for initializing the ECF environment.
    std::vector<std::shared_ptr<traffic::GPLightAgent>> agent_templates;

    for (const auto& id : engine->getIntersectionIDs()) {

        int phases = engine->getPhaseCount(id);

        agent_templates.push_back(
            std::make_shared<traffic::GPLightAgent>(
                id,
                phases,
                intersections.at(id)));
    }


    // Create evaluator and evaluation operator required by ECF.
    // They are needed because ECF needs a valid evolutionary environment
    // in order to construct and load GP trees.
    evolution::Evaluator evaluator(engine, 700, 5, 5, false);

    auto evalOp = std::make_shared<evolution::GPLightEvalOpBasic>();

    evalOp->setup(evaluator, agent_templates);


    // Create an ECF state containing the correct GP genotype.
    TreeP proto_tree(std::make_shared<Tree::Tree>());

    StateP state(new State);

    state->addGenotype(proto_tree);
    state->setEvalOp(evalOp);


    // Initialize ECF using the GPLight configuration.
    const char* ecfArgv[] = {
        "app",
        "src/data/ecf_gplightplus.xml"
    };

    int ecfArgc = 2;


    if (!state->initialize(ecfArgc, const_cast<char**>(ecfArgv)))
        throw std::runtime_error("[FATAL] ECF failed to initialise.");


    // Final structure:
    // intersection ID -> list of agents using different evolved trees.
    std::map<std::string,
             std::vector<std::shared_ptr<traffic::GPLightAgent>>> result;


    // For every intersection create agents for every selected experiment.
    for (const auto& id : engine->getIntersectionIDs()) {

        int phases = engine->getPhaseCount(id);

        const IntersectionData& idata = intersections.at(id);


        for (int exp_id : experiment_ids)
        {
            // Load evolved GP tree from this experiment.
            TreeP tree = load_tree_from_experiment(exp_id, state);


            // Create agent and attach the loaded GP controller.
            auto agent =
                std::make_shared<traffic::GPLightAgent>(
                    id,
                    phases,
                    idata);

            agent->setTree(tree.get());


            // Store shared pointer with custom deleter.
            // The captured objects keep the agent and tree alive.
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


// Same as makeGPLightAgentsFromExperiments,
// but creates agents that also receive the current traffic light phase.
//
// These agents use an extended GP input space containing current phase data.
std::map<std::string, std::vector<std::shared_ptr<traffic::GPLightAgentWithCurrentPhase>>>
makeGPLightAgentsWithCurrentFromExperiments(
    const std::vector<int>&                                  experiment_ids,
    const std::shared_ptr<traffic::SimulatorEngine>&                  engine,
    const std::map<std::string, IntersectionData>&  intersections)
{
    // Create temporary agents for ECF initialization.
    std::vector<std::shared_ptr<traffic::GPLightAgentWithCurrentPhase>> agent_templates;

    for (const auto& id : engine->getIntersectionIDs()) {

        int phases = engine->getPhaseCount(id);

        agent_templates.push_back(
            std::make_shared<traffic::GPLightAgentWithCurrentPhase>(
                id,
                phases,
                intersections.at(id)));
    }


    // Initialize evaluator and ECF evaluation operator.
    evolution::Evaluator evaluator(engine, 700, 5, 5, false);

    auto evalOp =
        std::make_shared<evolution::GPLightEvalOpCurrent>();

    evalOp->setup(evaluator, agent_templates);


    // Create ECF state with the correct genotype definition.
    TreeP proto_tree(std::make_shared<Tree::Tree>());

    StateP state(new State);

    state->addGenotype(proto_tree);
    state->setEvalOp(evalOp);


    // Use the configuration containing current-phase primitives.
    const char* ecfArgv[] = {
        "app",
        "src/data/ecf_gplightplusWithCurrentPhase.xml"
    };

    int ecfArgc = 2;


    if (!state->initialize(ecfArgc, const_cast<char**>(ecfArgv)))
        throw std::runtime_error("[FATAL] ECF failed to initialise.");


    std::map<std::string,
             std::vector<std::shared_ptr<traffic::GPLightAgentWithCurrentPhase>>> result;


    // Create one agent per experiment per intersection.
    for (const auto& id : engine->getIntersectionIDs()) {

        int phases = engine->getPhaseCount(id);

        const IntersectionData& idata = intersections.at(id);


        for (int exp_id : experiment_ids)
        {
            // Load evolved controller.
            TreeP tree = load_tree_from_experiment(exp_id, state);


            // Create current-phase aware agent.
            auto agent =
                std::make_shared<traffic::GPLightAgentWithCurrentPhase>(
                    id,
                    phases,
                    idata);

            agent->setTree(tree.get());


            // Store while keeping both agent and tree alive.
            result[id].push_back(
                std::shared_ptr<traffic::GPLightAgentWithCurrentPhase>(
                    agent.get(),
                    [agent, tree](traffic::GPLightAgentWithCurrentPhase*) mutable {
                        (void)agent;
                        (void)tree;
                    }));
        }
    }

    return result;
}

} // namespace util