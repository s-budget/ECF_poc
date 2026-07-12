#include "evolutionConcurrentMain.h"
#include "evolutionMain.h"  // wherever evaluation_main is declared

#include <future>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>


int evolution_concurrent_main(int argc, char** argv)
{
    constexpr size_t BATCH_SIZE = 5;

    // Execute evolution runs in fixed-size batches to limit resource usage.
    for (size_t start = 0; start < 50; start += BATCH_SIZE)
    {
        std::vector<std::future<int>> futures;

        size_t end = std::min(start + BATCH_SIZE, size_t(50));

        // Launch a limited number of evolution runs concurrently.
        for (size_t i = start; i < end; ++i)
        {
            futures.push_back(std::async(std::launch::async, [argc, argv]() {

                return evolution_main(argc, argv, false);
            }));
        }

        // Wait for the current batch to complete before starting the next one.
        for (auto& f : futures)
            f.get();
    }

    return 0;
}