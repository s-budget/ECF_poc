
#include "include/simulator/CityFlowEngine.hpp"
#include "evolutionMain.h"
#include "evaluationMain.h"

namespace traffic {
   class EnsembleGPLightAgent;
}

using namespace std;


int main(int argc, char** argv)
{
   //return evaluation_main(argc, argv);
   return evolution_main(argc, argv);

}