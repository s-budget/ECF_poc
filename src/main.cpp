
#include "include/simulator/CityFlowEngine.hpp"
#include "evolutionMain.h"
#include "ensemblePreparationMain.h"
#include "evaluationMain.h"
#include "cyclicAgentEvaluationMain.h"
#include "regenerateFitnessMain.h"
#include "ensembleConcurrentMain.h"
#include "evaluationMainForAllSingleAgents.h"
#include "evolutionConcurrentMain.h"

namespace traffic {
   class EnsembleGPLightAgent;
}

using namespace std;


int main(int argc, char** argv)
{
   return evaluation_main(argc, argv,true,true);
   //return evolution_main(argc, argv,true);
   //return ensemble_preparation_main(argc, argv);
   //return regenerate_fitness_main(argc, argv);
   //return ensemble_concurrent_main(argc, argv);
   //return evolution_concurrent_main(argc, argv);
   //return cyclicEvaluation_main(argc,argv);
   //return evaluation_for_all_singles_main(argc, argv);
}