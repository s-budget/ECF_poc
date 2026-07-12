#pragma once

#include "../simulator/SimulatorEngine.hpp"
#include "../simulator/SimulationRunner.hpp"
#include "Agent.hpp"

#include <memory>
#include <vector>

namespace evolution {

    /**
     * @brief Evaluates traffic control agents by running a simulation.
     *
     * The evaluator manages a simulation runner and converts the resulting
     * simulation state into a fitness value used by the evolution process.
     */
    class Evaluator {
    public:
        Evaluator(const shared_ptr<traffic::SimulatorEngine> & engine,
                  int                              max_steps       = 3600,
                  int                              action_interval = 10,
                  int                              red_interval=5,
                  bool                             verbose         = false)
            : runner_(move(engine), max_steps, action_interval, verbose, red_interval)
        {
            runner_.setup();
        }

        Evaluator(): runner_() {
        };

        /**
         * @brief Runs a simulation with the provided agents and returns their fitness.
         *
         * The current fitness is based on the mean travel time produced by the
         * simulation.
         */
        double evaluate(const vector<shared_ptr<traffic::Agent>>& agents) {
            traffic::SimulationState state = runner_.run(agents);
            return state.mean_travel_time; // TODO: make fitness function from state
        }

    private:
        // Handles simulation execution and state collection.
        traffic::SimulationRunner runner_;
    };

}