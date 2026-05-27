#pragma once

#include "SimulatorEngine.hpp"
#include "../agent/Agent.hpp"

#include <memory>
#include <vector>
#include <deque>
#include <map>
#include <string>
#include <iostream>
#include <stdexcept>
using namespace std;
namespace traffic {

// Action with an explicit duration in steps.
// In free mode (action_interval_ == -1) duration is always respected.
// In red/green mode (action_interval_ > 0) this struct is not used.


class SimulationRunner {
public:
    SimulationRunner(shared_ptr<SimulatorEngine> engine,
                     int                         max_steps       = 3600,
                     int                         action_interval = -1,   // free mode: unused; red/green mode: green phase duration (steps)
                     bool                        verbose         = false,
                     int                         red_interval    = 30)  // red phase duration — used ONLY in red/green mode
        : engine_(move(engine))
        , max_steps_(max_steps)
        , action_interval_(action_interval)
        , red_interval_(red_interval)
        , verbose_(verbose)
    {}

    SimulationRunner() {}

    void setup() {
        if (setup_done_) return;
        setup_done_ = true;
        if (verbose_)
            cout << "[SimulationRunner] Setup complete. ";
    }

    SimulationState run(const vector<shared_ptr<Agent>>& agents) {
        if (!setup_done_)
            throw runtime_error("SimulationRunner::run: call setup() first.");

        engine_->reset();
        for (auto& ag : agents) ag->reset();

        map<string, deque<TimedAction>> queues;
        map<string, int>               steps_left;

        if (action_interval_ == -1)
            runFreeMode(agents, queues, steps_left);
        else
            runRedGreenMode(agents);

        SimulationState final_state = engine_->getState(true, false);
        printSimulationState(final_state);
        done_ = true;
        return final_state;
    }

    bool isDone() const { return done_; }

private:
    // -----------------------------------------------------------------------
    // Mode 1 — free mode (action_interval_ == -1)
    // Each intersection independently moves to the next action once the
    // current one's duration expires. Agents are asked only when their queue
    // runs out.
    // -----------------------------------------------------------------------
    void runFreeMode(const vector<shared_ptr<Agent>>&    agents,
                     map<string, deque<TimedAction>>&     queues,
                     map<string, int>&                    steps_left)
    {
        while (!engine_->isDone() && engine_->currentStep() < max_steps_) {
            SimulationState state = engine_->getState(verbose_, true);
            vector<IntersectionAction> changed_actions;
            bool any_changed = false;

            for (auto& agent : agents) {
                const string& id = agent->getIntersectionId();
                auto& q  = queues[id];
                auto& sl = steps_left[id];

                if (q.empty()) {
                    // Queue exhausted — ask agent for a new series
                    auto its = state.intersectionStates.find(id);
                    if (its != state.intersectionStates.end()) {
                        auto series = agent->selectActions(its->second);
                        for (auto& ta : series) q.push_back(ta);
                        sl = q.empty() ? 0 : q.front().duration;
                        if (!q.empty()) {
                            changed_actions.push_back(q.front().action);
                            any_changed = true;
                        }
                    }
                } else {
                    sl--;
                    if (sl <= 0) {
                        q.pop_front();
                        if (!q.empty()) {
                            sl = q.front().duration;
                            changed_actions.push_back(q.front().action);
                            any_changed = true;
                        }
                        // queue just ran out — will refill next step
                    }
                    // else: still running current action, nothing to apply
                }
            }

            if (any_changed)
                engine_->applyActions(changed_actions);

            engine_->step();
        }
    }

    // -----------------------------------------------------------------------
    // Mode 2 — red/green single-action mode (action_interval_ > 0)
    // At the start of each cycle a fresh state is fetched and each agent
    // returns exactly 2 actions: [red, green].
    // The runner applies red to all intersections, waits red_interval_ steps,
    // then applies green to all and waits action_interval_ steps, then repeats.
    //
    // action_interval_  — duration of the green phase (steps)
    // red_interval_     — duration of the red phase (steps), used ONLY here
    // -----------------------------------------------------------------------
    void runRedGreenMode(const vector<shared_ptr<Agent>>& agents)
    {
        while (!engine_->isDone() && engine_->currentStep() < max_steps_) {
            // Fetch fresh state and ask agents at the start of each cycle
            SimulationState state = engine_->getState(verbose_, true);
            if (verbose_) printSimulationState(state);

            vector<IntersectionAction> reds, greens;
            for (auto& agent : agents) {
                const string& id = agent->getIntersectionId();
                auto its = state.intersectionStates.find(id);
                if (its != state.intersectionStates.end()) {
                    auto [red, green] = agent->selectRedGreenActions(its->second);
                    reds.push_back(red);
                    greens.push_back(green);
                }
            }

            // --- Red phase ---
            engine_->applyActions(reds);
            for (int i = 0; i < red_interval_ && !engine_->isDone() && engine_->currentStep() < max_steps_; i++)
                engine_->step();

            if (engine_->isDone() || engine_->currentStep() >= max_steps_) break;

            // --- Green phase ---
            engine_->applyActions(greens);
            for (int i = 0; i < action_interval_ && !engine_->isDone() && engine_->currentStep() < max_steps_; i++)
                engine_->step();
        }
    }

    // -----------------------------------------------------------------------

    shared_ptr<SimulatorEngine> engine_;
    int                         max_steps_;
    int                         action_interval_;
    int                         red_interval_;  // used ONLY in red/green mode
    bool                        verbose_;
    bool                        setup_done_ = false;
    bool                        done_       = false;
};

} // namespace traffic