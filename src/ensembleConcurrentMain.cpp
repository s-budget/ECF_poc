#include "ensembleConcurrentMain.h"
#include "evaluationMain.h"  // wherever evaluation_main is declared

#include <future>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>

struct EvaluationCall {
    std::vector<std::string> args;
    bool vote_for_all_phases;
};

// Reads file where each row has 1-5 numbers
static std::vector<std::vector<std::string>> load_id_sets(const std::string& filepath)
{
    std::vector<std::vector<std::string>> result;
    std::ifstream f(filepath);
    if (!f) throw std::runtime_error("Cannot open: " + filepath);

    std::string line;
    while (std::getline(f, line))
    {
        if (line.empty()) continue;
        std::istringstream ss(line);
        std::vector<std::string> ids;
        std::string token;
        while (ss >> token)
            ids.push_back(token);
        if (!ids.empty())
            result.push_back(ids);
    }
    return result;
}
static std::vector<EvaluationCall> make_calls_for_mode(
   const std::vector<std::vector<std::string>>& id_sets,
   const std::string& mode)
{
    std::vector<EvaluationCall> calls;

    for (int i = 0; i < (int)id_sets.size(); ++i)
    {
        for (bool vote : {true, false})
        {
            std::string voting_name = vote ? "BORDA" : "PLURALITY";
            std::string name = "ensemble_" + mode + "_" + voting_name + "_" + std::to_string(i);

            std::vector<std::string> args = {"exe", mode, name};
            for (const auto& id : id_sets[i])
                args.push_back(id);

            calls.push_back({args, vote});
        }
    }
    return calls;
}

static std::vector<EvaluationCall> make_calls_random(const std::vector<std::vector<std::string>>& id_sets)
{
    return make_calls_for_mode(id_sets, "RANDOM");
}

static std::vector<EvaluationCall> make_calls_grow(const std::vector<std::vector<std::string>>& id_sets)
{
    return make_calls_for_mode(id_sets, "GROW");
}

static std::vector<EvaluationCall> make_calls_grow_destroy(const std::vector<std::vector<std::string>>& id_sets)
{
    return make_calls_for_mode(id_sets, "GROWDESTROY");
}

static std::vector<EvaluationCall> make_calls_weighted_random(const std::vector<std::vector<std::string>>& id_sets)
{
    return make_calls_for_mode(id_sets, "WEIGHTEDRANDOM");
}


int ensemble_concurrent_main(int argc, char** argv)
{
    /*std::vector<EvaluationCall> all_calls = {
        {{"exe", "RANDOM",      "ensemble_1", "1", "2", "3"}, false},
        {{"exe", "GROW",        "ensemble_2", "4", "5"},       true},
        {{"exe", "GROWDESTROY", "ensemble_3", "6", "7", "8"}, false},
        // ... add more calls here
    };*/

    std::string CALLS_FILE = "ensemble_single.txt";
    std::vector<std::vector<std::string>> id_sets = load_id_sets(CALLS_FILE);

    std::vector<EvaluationCall> all_calls;
    //for (auto& c : make_calls_random(id_sets))          all_calls.push_back(c);
    for (auto& c : make_calls_grow(id_sets))            all_calls.push_back(c);
    //for (auto& c : make_calls_grow_destroy(id_sets))    all_calls.push_back(c);
    //for (auto& c : make_calls_weighted_random(id_sets)) all_calls.push_back(c);

    std::vector<std::future<int>> futures;

    constexpr size_t BATCH_SIZE = 5;

    for (size_t start = 0; start < 5; start += BATCH_SIZE) {
        std::vector<std::future<int>> futures;

        size_t end = std::min(start + BATCH_SIZE, all_calls.size());

        // Launch up to 5 tasks
        for (size_t i = start; i < end; ++i) {
            const auto& call = all_calls[i];

            futures.push_back(std::async(std::launch::async, [call]() {
                std::vector<const char*> argv;

                for (size_t j = 0; j < call.args.size(); ++j) {
                    argv.push_back(call.args[j].c_str());
                }

                return evaluation_main(
                    static_cast<int>(argv.size()),
                    (char**)argv.data(),
                    false,
                    call.vote_for_all_phases
                );
            }));
        }

        // Wait for this batch to complete
        for (size_t i = 0; i < futures.size(); ++i) {
            futures[i].get();
        }
    }

    return 0;
}