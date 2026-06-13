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
#include "include/simulator/SimulationRunner.hpp"

using namespace std;
using namespace traffic;
namespace fs = std::filesystem;


// Finds the best candidate to ADD to experiment_ids (lowest mean travel time)
int growStep(
    const vector<int>& experiment_ids,
    const map<int, int>& experiment_index,
    const shared_ptr<SimulatorEngine>& engine,
    const map<string, IntersectionData>& intersections,
    const map<string, vector<shared_ptr<GPLightAgent>>>& all_agents,
    const bool vote_for_all_phases)
{
    vector<int> candidates;
    for (const auto& [exp_id, _] : experiment_index)
        if (find(experiment_ids.begin(), experiment_ids.end(), exp_id) == experiment_ids.end())
            candidates.push_back(exp_id);

    int    best_candidate = -1;
    double best_fitness   = std::numeric_limits<double>::max();

    for (int candidate : candidates)
    {
        vector<int> trial_ids = experiment_ids;
        trial_ids.push_back(candidate);

        vector<shared_ptr<Agent>> trial_ensemble;
        for (const auto& intersection_id : engine->getIntersectionIDs())
        {
            int phases = engine->getPhaseCount(intersection_id);
            vector<shared_ptr<Agent>> sub_agents;
            for (int exp_id : trial_ids)
                sub_agents.push_back(all_agents.at(intersection_id)[experiment_index.at(exp_id)]);
            trial_ensemble.push_back(make_shared<EnsembleGPLightAgent>(
                intersection_id, phases, intersections.at(intersection_id), std::move(sub_agents),vote_for_all_phases));
        }

        traffic::SimulationRunner trial_runner(engine, 3600, 5, false, 5);
        trial_runner.setup();
        traffic::SimulationState trial_state = trial_runner.run(trial_ensemble);

        cout << "GROW candidate " << candidate << " fitness: " << trial_state.mean_travel_time << " s\n";

        if (trial_state.mean_travel_time < best_fitness)
        {
            best_fitness   = trial_state.mean_travel_time;
            best_candidate = candidate;
        }
    }

    cout << "GROW picked " << best_candidate << " (fitness: " << best_fitness << " s)\n";
    return best_candidate;
}

// Finds the best candidate to REMOVE from experiment_ids (lowest mean travel time without it)
int destroyStep(
    const vector<int>& experiment_ids,
    const map<int, int>& experiment_index,
    const shared_ptr<SimulatorEngine>& engine,
    const map<string, IntersectionData>& intersections,
    const map<string, vector<shared_ptr<GPLightAgent>>>& all_agents,
    const bool vote_for_all_phases)
{
    int    best_candidate = -1;
    double best_fitness   = std::numeric_limits<double>::max();

    for (int candidate : experiment_ids)
    {
        vector<int> trial_ids = experiment_ids;
        trial_ids.erase(std::remove(trial_ids.begin(), trial_ids.end(), candidate), trial_ids.end());

        vector<shared_ptr<Agent>> trial_ensemble;
        for (const auto& intersection_id : engine->getIntersectionIDs())
        {
            int phases = engine->getPhaseCount(intersection_id);
            vector<shared_ptr<Agent>> sub_agents;
            for (int exp_id : trial_ids)
                sub_agents.push_back(all_agents.at(intersection_id)[experiment_index.at(exp_id)]);
            trial_ensemble.push_back(make_shared<EnsembleGPLightAgent>(
                intersection_id, phases, intersections.at(intersection_id), std::move(sub_agents),vote_for_all_phases));
        }

        traffic::SimulationRunner trial_runner(engine, 3600, 5, false, 5);
        trial_runner.setup();
        traffic::SimulationState trial_state = trial_runner.run(trial_ensemble);

        cout << "DESTROY candidate " << candidate << " fitness: " << trial_state.mean_travel_time << " s\n";

        if (trial_state.mean_travel_time < best_fitness)
        {
            best_fitness   = trial_state.mean_travel_time;
            best_candidate = candidate;
        }
    }

    cout << "DESTROY picked " << best_candidate << " (fitness: " << best_fitness << " s)\n";
    return best_candidate;
}

int evaluation_main(int argc, char** argv, bool verbose, bool vote_for_all_phases)
{
    if (!verbose) {
        cout.rdbuf(nullptr);
        cerr.rdbuf(nullptr);
    }
    else {
        freopen("total_output.txt", "w", stdout);
        freopen("total_output.txt", "a", stderr);
    }

    // Expected: <exe> <mode> <id1> [id2] [id3] [id4] [id5]
    // mode: RANDOM | WEIGHTEDRANDOM | GROW | GROWDESTROY
    // ids:  1 to 5 integers

    if (argc < 4) {
        cerr << "Usage: " << argv[0]
             << " <RANDOM|WEIGHTEDRANDOM|GROW|GROWDESTROY>"
             << " <ensembleName>"
             << " <id1> [id2] [id3] [id4] [id5]\n";
        return 1;
    }

    const string mode                 = argv[1];
    const string ensembleExperimentName = argv[2];

    if (mode != "RANDOM" && mode != "WEIGHTEDRANDOM" &&
        mode != "GROW"   && mode != "GROWDESTROY")
    {
        cerr << "Unknown mode: " << mode
             << ". Expected: RANDOM, WEIGHTEDRANDOM, GROW, or GROWDESTROY\n";
        return 1;
    }

    const int n_ids = argc - 3;
    if (n_ids < 1 || n_ids > 5) {
        cerr << "Expected 1 to 5 experiment IDs, got " << n_ids << '\n';
        return 1;
    }

    vector<int> experiment_ids;
    experiment_ids.reserve(n_ids);
    for (int i = 0; i < n_ids; ++i) {
        try {
            experiment_ids.push_back(std::stoi(argv[3 + i]));
        } catch (const std::exception&) {
            cerr << "Invalid experiment ID: " << argv[3 + i] << '\n';
            return 1;
        }
    }

    cout << "Mode: " << mode << "\nExperiment IDs:";
    for (int id : experiment_ids) cout << ' ' << id;
    cout << '\n';

    EngineConfig cfg;
    cfg.type        = SimulatorType::CityFlow;
    cfg.config_file = "src/data/cityflow_config_hangzhou_4x4.json";
    cfg.num_threads = 1;
    cfg.random_seed = 42;
    cfg.verbose     = false;

    auto engine = createEngine(cfg);
    engine->initialize();

    map<string, IntersectionData> intersections = loadFromConfig(cfg.config_file);
    map<string, vector<shared_ptr<GPLightAgent>>> all_agents;
    map<int, int> experiment_index; // experiment_id -> index in the vectors above

    if (mode == "GROW" || mode == "GROWDESTROY")
    {
        vector<int> all_ids;
        for (int i = 1; i <= 999; ++i)
        {
            ostringstream ss;
            ss << "experiments/experiment_"
               << setw(3) << setfill('0') << i;
            if (fs::exists(ss.str()))
                all_ids.push_back(i);
        }

        all_agents = util::makeGPLightAgentsFromExperiments(
            all_ids, engine, intersections);

        for (int idx = 0; idx < (int)all_ids.size(); ++idx)
            experiment_index[all_ids[idx]] = idx;

        cout << "Loaded " << all_ids.size()
             << " experiments for " << mode << " mode.\n";
    }

    // ---- GROW: iteratively find the best set of 5 experiment IDs ----
    if (mode == "GROW")
    {
        while ((int)experiment_ids.size() < 5)
            experiment_ids.push_back(growStep(experiment_ids, experiment_index, engine, intersections, all_agents,vote_for_all_phases));
    }
    // ---- GROW: iteratively find the best set of 10 experiment IDs and then remove 5 usless ones ----
    if (mode == "GROWDESTROY")
    {
        while ((int)experiment_ids.size() < 10)
            experiment_ids.push_back(growStep(experiment_ids, experiment_index, engine, intersections, all_agents,vote_for_all_phases));

        while ((int)experiment_ids.size() > 5)
        {
            int worst = destroyStep(experiment_ids, experiment_index, engine, intersections, all_agents,vote_for_all_phases);
            experiment_ids.erase(std::remove(experiment_ids.begin(), experiment_ids.end(), worst), experiment_ids.end());
        }
    }

    // ---- RANDOM / WEIGHTEDRANDOM / final GROW evaluation ----
    // At this point experiment_ids is the final set regardless of mode


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
                std::move(sub_agents),vote_for_all_phases));
    }

    traffic::SimulationRunner runner(engine, 3600, 5, false, 5);
    runner.setup();

    traffic::SimulationState sState = runner.run(ensemble_agents);
    for (int id : experiment_ids)
        cout << id << ' ';
    cout << '\n';
    cout << "Mean travel time: " << sState.mean_travel_time << " s\n";

    // Write ensemble result file
    {
        string filename = "experiments/ENSEMBLE_FITNESS_FOR_" + ensembleExperimentName + ".txt";
        ofstream f(filename);
        f << sState.mean_travel_time << '\n';
        for (int id : experiment_ids)
            f << id << ' ';
        f << '\n';
        cout << "Ensemble result written to: " << filename << '\n';
    }

    return static_cast<int>(sState.mean_travel_time);
}

