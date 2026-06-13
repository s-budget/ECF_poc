#include "regenerateFitnessMain.h"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <regex>
#include <sstream>
#include <string>

namespace fs = std::filesystem;

int regenerate_fitness_main(int argc, char** argv)
{
    fs::path experiments_dir = (argc > 1) ? argv[1] : "experiments";

    if (!fs::exists(experiments_dir))
    {
        std::cerr << "[FATAL] Directory not found: "
                  << experiments_dir << '\n';
        return 1;
    }

    int found   = 0;
    int updated = 0;
    int failed  = 0;

    // Regex matches "Fitness (avg travel time): 73.03 s"
    const std::regex fitness_pattern(
        R"(Fitness \(avg travel time\):\s*([0-9]+(?:\.[0-9]+)?))");

    for (int i = 1; i <= 999; ++i)
    {
        std::ostringstream ss;
        ss << "experiment_"
           << std::setw(3) << std::setfill('0') << i;

        fs::path exp_dir    = experiments_dir / ss.str();
        fs::path output_file = exp_dir / "total_output.txt";

        if (!fs::exists(exp_dir))
            continue;

        if (!fs::exists(output_file))
        {
            std::cout << "SKIP   " << exp_dir
                      << "  (no total_output.txt)\n";
            continue;
        }

        ++found;

        // Read file line by line, keep updating match so we get the last one
        std::ifstream f(output_file);
        std::string   line;
        std::string   last_fitness;

        while (std::getline(f, line))
        {
            std::smatch m;
            if (std::regex_search(line, m, fitness_pattern))
                last_fitness = m[1].str();
        }

        if (last_fitness.empty())
        {
            std::cout << "FAIL   " << exp_dir
                      << "  (could not parse fitness)\n";
            ++failed;
            continue;
        }

        std::ofstream out(exp_dir / "fitness.txt");
        out << last_fitness << '\n';

        std::cout << "OK     " << exp_dir
                  << "  -> " << last_fitness << '\n';
        ++updated;
    }

    std::cout << "\nDone.  Found: " << found
              << "  Updated: " << updated
              << "  Failed: "  << failed << '\n';

    return (failed > 0) ? 1 : 0;
}