#include "ensemblePreparationMain.h"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <numeric>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace
{

// Returns a map of experiment_id -> fitness value for all existing experiments
std::map<int, double> load_all_fitnesses()
{
    std::map<int, double> fitnesses;

    for (int i = 1; i <= 999; ++i)
    {
        std::ostringstream ss;
        ss << "experiments/experiment_"
           << std::setw(3) << std::setfill('0') << i;

        fs::path dir(ss.str());
        fs::path fitness_file = dir / "fitness.txt";

        if (!fs::exists(fitness_file))
            continue;

        std::ifstream f(fitness_file);
        double val;
        if (f >> val)
            fitnesses[i] = val;
        else
            std::cerr << "Warning: could not read fitness from "
                      << fitness_file << '\n';
    }

    return fitnesses;
}

// Sample `count` unique IDs from `pool` with equal probability
std::vector<int> sample_uniform(
    const std::vector<int>& pool,
    int count,
    std::mt19937& rng)
{
    if ((int)pool.size() < count)
        throw std::runtime_error(
            "Not enough experiments to sample " + std::to_string(count));

    std::vector<int> shuffled = pool;
    std::shuffle(shuffled.begin(), shuffled.end(), rng);
    return { shuffled.begin(), shuffled.begin() + count };
}

// Sample `count` unique IDs from `pool` weighted by 1/fitness (lower = better)
std::vector<int> sample_fitness_weighted(
    const std::vector<int>& pool,
    const std::map<int, double>& fitnesses,
    int count,
    std::mt19937& rng)
{
    if ((int)pool.size() < count)
        throw std::runtime_error(
            "Not enough experiments to sample " + std::to_string(count));

    // Build parallel arrays of ids and weights
    std::vector<int>    ids;
    std::vector<double> weights;
    ids.reserve(pool.size());
    weights.reserve(pool.size());

    for (int id : pool)
    {
        ids.push_back(id);
        weights.push_back(1.0 / fitnesses.at(id));
    }

    std::vector<int> result;
    result.reserve(count);

    // Weighted sampling without replacement:
    // each round pick one, then remove it and renormalize
    for (int pick = 0; pick < count; ++pick)
    {
        std::discrete_distribution<int> dist(weights.begin(), weights.end());
        int chosen_idx = dist(rng);
        result.push_back(ids[chosen_idx]);

        // Remove chosen from consideration
        ids.erase(ids.begin() + chosen_idx);
        weights.erase(weights.begin() + chosen_idx);
    }

    return result;
}

} // anonymous namespace

int ensemble_preparation_main(int argc, char** argv)
{
    constexpr int N_SINGLE   = 10;
    constexpr int N_SETS     = 20;
    constexpr int SET_SIZE   =  5;

    std::map<int, double> fitnesses = load_all_fitnesses();

    if (fitnesses.empty())
    {
        std::cerr << "[FATAL] No experiment fitness files found.\n";
        return 1;
    }

    std::cout << "Found " << fitnesses.size() << " experiments.\n";

    // Collect all available IDs into a vector
    std::vector<int> all_ids;
    all_ids.reserve(fitnesses.size());
    for (const auto& [id, _] : fitnesses)
        all_ids.push_back(id);

    std::mt19937 rng(std::random_device{}());

    // ---- File 1: 20 random single IDs ----
    {
        std::vector<int> singles = sample_uniform(all_ids, N_SINGLE, rng);

        std::ofstream f("ensemble_single.txt");
        for (int id : singles)
            f << id << '\n';

        std::cout << "Written: ensemble_single.txt\n";
    }

    // ---- File 2: 20 sets of 5, uniform random ----
    {
        std::ofstream f("ensemble_random.txt");
        for (int s = 0; s < N_SETS; ++s)
        {
            std::vector<int> set = sample_uniform(all_ids, SET_SIZE, rng);
            for (int i = 0; i < SET_SIZE; ++i)
            {
                f << set[i];
                if (i + 1 < SET_SIZE) f << ' ';
            }
            f << '\n';
        }

        std::cout << "Written: ensemble_random.txt\n";
    }

    // ---- File 3: 20 sets of 5, fitness-weighted ----
    {
        std::ofstream f("ensemble_fitness_weighted.txt");
        for (int s = 0; s < N_SETS; ++s)
        {
            std::vector<int> set =
                sample_fitness_weighted(all_ids, fitnesses, SET_SIZE, rng);
            for (int i = 0; i < SET_SIZE; ++i)
            {
                f << set[i];
                if (i + 1 < SET_SIZE) f << ' ';
            }
            f << '\n';
        }

        std::cout << "Written: ensemble_fitness_weighted.txt\n";
    }

    return 0;
}