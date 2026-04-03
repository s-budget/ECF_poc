#pragma once

#include "SimulatorEngine.hpp"
#include <engine/engine.h>

#include <unordered_set>
#include <stdexcept>
#include <iostream>
using namespace std;
namespace traffic {

class CityFlowEngine : public SimulatorEngine {
public:
    explicit CityFlowEngine(const EngineConfig& cfg)
        : SimulatorEngine(cfg) {}

    ~CityFlowEngine() override { shutdown(); }

    void initialize() override {
        if (initialized_) {
            throw runtime_error("CityFlowEngine: already initialized");
        }
        try {
            cf_engine_ = make_unique<CityFlow::Engine>(
                config_.config_file,
                config_.num_threads
            );
        } catch (const exception& e) {
            throw runtime_error(
                string("CityFlowEngine: failed to load config '")
                + config_.config_file + "': " + e.what());
        }
        cacheIntersectionIDs();
        cachePhaseCountForIntersection();

        interval_ = cf_engine_->getInterval();
        initialized_ = true;
        current_step_ = 0;

        if (config_.verbose) {
            cout << "[CityFlow] Initialized with "
                      << intersection_ids_.size() << " intersections\n";
        }
    }

    void step() override {
        ensureInitialized();
        cf_engine_->nextStep();
        ++current_step_;
    }

    void reset() override {
        ensureInitialized();
        cf_engine_->reset();
        current_step_ = 0;
    }

    void shutdown() override {
        cf_engine_.reset();
        initialized_ = false;
    }

    SimulationState getState(const bool getStatistics, const bool getAllIntersectionsStates) const override {
        ensureInitialized();

        SimulationState state;
        state.step         = current_step_;
        state.time_seconds = current_step_ * interval_;
        //for performance call getStatistics only at the end of simulation
        if (getStatistics) {
            auto metrics = cf_engine_->getFinalMetrics();
            state.mean_travel_time        = metrics.meanTravelTime;
            state.mean_waiting_time       = metrics.meanWaitTime;
            state.mean_delay              = metrics.meanDelay;
            state.total_travel_time       = metrics.totalTravelTime;
            state.total_waiting_time      = metrics.totalWaitTime;
            state.total_delay             = metrics.totalDelay;
            state.max_queue_size_until_now = metrics.maxQueueLength;
            state.throughput              = metrics.throughput;
            state.total_vehicles_started  = metrics.totalVehicles;
        }
        if (getAllIntersectionsStates) {
            for (const auto& id : intersection_ids_) {
                state.intersectionStates[id] = getIntersectionState(id);
                state.intersectionStates[id].step = current_step_;
            }
        }

        return state;
    }

    IntersectionState getIntersectionState(const string& id) const override {
      ensureInitialized();

      IntersectionState s;
      s.intersection_id = id;

        const auto& lanes = cf_engine_ -> getLaneVehiclesForIntersection(id);
        for (const auto& lane:lanes) {
            const auto& laneId = lane.first;
            const auto& vehicles = lane.second;
            s.queue_lengths_per_lane.insert({laneId,0});
            s.total_cars_per_lane.insert({laneId,0});
            s.avg_speed_per_lane.insert({laneId,0.0});
            s.waiting_times_sum_per_lane.insert({laneId,0.0});
            for (const auto& vehicle:vehicles) {
              s.total_cars_per_lane[laneId]++;
              if (vehicle->getSpeed() < 0.1) { //TODO: better waiting critera maybe?
                    s.queue_lengths_per_lane[laneId] += 1;
                    s.waiting_times_sum_per_lane[laneId] += vehicle->getStoppedSince()*interval_;
              }
              s.avg_speed_per_lane[laneId] += vehicle->getSpeed();

            }
            s.avg_speed_per_lane[laneId] = s.avg_speed_per_lane[laneId]/s.total_cars_per_lane[laneId];
        }
        return s;
    }

    vector<string> getIntersectionIDs() const override {
        return intersection_ids_;
    }

    int getPhaseCount(const string& id) const override {
        return phaseCounts_.at(id);
    }

    bool isDone() const override {
        return current_step_ >= max_steps_;
    }

    int currentStep() const override { return current_step_; }


    void applyActions(const vector<IntersectionAction>& actions) override {
        ensureInitialized();
        for (const auto& action : actions) {
            if (action.target_phase >= 0) {
                cf_engine_->setTrafficLightPhase(
                    action.intersection_id,
                    action.target_phase
                );
            }
        }
    }

    void setMaxSteps(int n) { max_steps_ = n; }

private:
    void ensureInitialized() const {
        if (!initialized_) {
            throw runtime_error("CityFlowEngine: not initialized – call initialize() first");
        }
    }

    void cacheIntersectionIDs() {
            const auto& ids = cf_engine_->getIntersectionIds();
            for (const auto& id : ids) {
                    intersection_ids_.push_back(id);
            }

    }
    void cachePhaseCountForIntersection() {
          for (const auto& id : intersection_ids_) {
            phaseCounts_.insert({id,cf_engine_->getTotalPhasesForIntersection(id)});
          }
    }

    unique_ptr<CityFlow::Engine> cf_engine_;
    vector<string>          intersection_ids_;
    map<string, int> phaseCounts_;
    bool  initialized_  = false;
    int   current_step_ = 0;
    double interval_    = -1.0;
    int   max_steps_    = 3600;
};

}
