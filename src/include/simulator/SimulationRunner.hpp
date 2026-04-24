#pragma once

#include "SimulatorEngine.hpp"
#include "../agent/Agent.hpp"

#include <memory>
#include <vector>
#include <string>
#include <iostream>
#include <stdexcept>
using namespace std;
namespace traffic {

class SimulationRunner {
public:
    SimulationRunner(shared_ptr<SimulatorEngine>          engine,
                     int                                        max_steps     = 3600,
                     int                                        action_interval = 1,
                     bool                                       verbose       = false)
        : engine_(move(engine))
        , max_steps_(max_steps)
        , action_interval_(action_interval)
        , verbose_(verbose)
    {}

    SimulationRunner(){};

    void setup() {
        if (setup_done_) return;
        setup_done_ = true;
        if (verbose_)
            cout << "[SimulationRunner] Setup complete. ";
    }

    SimulationState run(const vector<shared_ptr<Agent> > &agents) {
        if (!setup_done_)
            throw runtime_error("SimulationRunner::run: call setup() first.");

        engine_->reset();
        for (auto& ag : agents) ag->reset();

        while (!engine_->isDone() && engine_->currentStep() < max_steps_) {
            int step = engine_->currentStep();
            bool is_action_step = (step % action_interval_ == 0);

            if (is_action_step) {
                SimulationState state = engine_->getState(verbose_, true);

                if (verbose_) {
                    printSimulationState(state);
                }

                vector<IntersectionAction> actions;
                for (auto& agent : agents) {
                    auto it = state.intersectionStates.find(agent->getIntersectionId());
                    if (it != state.intersectionStates.end()) {
                        actions.push_back(agent->selectAction(it->second));
                    }
                }
                engine_->applyActions(actions);
            }

            engine_->step();
        }

        // final state with full statistics
        SimulationState final_state = engine_->getState(true, false);
        printSimulationState(final_state);
        done_ = true;
        return final_state;
    }

    bool isDone() const { return done_; }

private:
    shared_ptr<SimulatorEngine>    engine_;
    int                                 max_steps_;
    int                                 action_interval_;
    bool                                verbose_;
    bool                                setup_done_ = false;
    bool                                done_       = false;
};

}