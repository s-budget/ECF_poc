#include "ensembleConcurrentMain.h"
#include "evolutionMain.h"  // wherever evaluation_main is declared

#include <future>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>


int ensemble_concurrent_main(int argc, char** argv)
{

    std::vector<std::future<int>> futures;

    for (int i=0;i<20;i++) {
        futures.push_back(std::async(std::launch::async, [argc,argv]() {

            return evolution_main(argc, argv, false);
        }));
    }

    // Wait for all and collect results
    for (auto& f : futures)
        f.get();

    return 0;
}