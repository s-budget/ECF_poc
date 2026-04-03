#pragma once

#include "../simulator/SimulatorEngine.hpp"
#include "../simulator/SimulationRunner.hpp"
#include "Agent.hpp"

#include <memory>
#include <vector>

namespace evolution {
    class Evaluator {
    public:
        Evaluator(const shared_ptr<traffic::SimulatorEngine> & engine,
                  int                              max_steps       = 3600,
                  int                              action_interval = 1,
                  bool                             verbose         = false)
            : runner_(move(engine), max_steps, action_interval, verbose)
        {
            runner_.setup();
        }

        double evaluate(const vector<shared_ptr<traffic::Agent>>& agents) {
            traffic::SimulationState state = runner_.run(agents);
            return state.mean_waiting_time;//todo make fitness function from state
        }

    private:
        traffic::SimulationRunner runner_;
    };
}