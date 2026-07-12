#pragma once

#include "SimulatorEngine.hpp"
#include <engine/engine.h>

#include <unordered_set>
#include <stdexcept>
#include <iostream>

using namespace std;

namespace traffic {

/**
 * @brief SimulatorEngine implementation based on the CityFlow traffic simulator.
 *
 * This class provides an adapter between the generic SimulatorEngine interface
 * and the CityFlow C++ API. It is responsible for managing the simulator
 * lifecycle, retrieving simulation state, and applying traffic signal actions.
 */
class CityFlowEngine : public SimulatorEngine {
public:
    /**
     * @brief Constructs a CityFlow engine wrapper.
     *
     * @param cfg Simulator configuration.
     */
    explicit CityFlowEngine(const EngineConfig& cfg)
        : SimulatorEngine(cfg) {}

    /**
     * @brief Ensures the simulator is properly shut down.
     */
    ~CityFlowEngine() override { shutdown(); }

    /**
     * @brief Initializes the CityFlow simulation.
     *
     * Loads the configuration, creates the underlying CityFlow engine,
     * caches static network information, and prepares the simulation
     * for execution.
     */
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

        // Cache static information that does not change during simulation.
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

    /**
     * @brief Advances the simulation by one time step.
     */
    void step() override {
        ensureInitialized();
        cf_engine_->nextStep();
        ++current_step_;
    }

    /**
     * @brief Resets the simulation to its initial state.
     */
    void reset() override {
        ensureInitialized();
        cf_engine_->reset();
        current_step_ = 0;
    }

    /**
     * @brief Releases the underlying CityFlow engine.
     */
    void shutdown() override {
        cf_engine_.reset();
        initialized_ = false;
    }

    /**
     * @brief Retrieves the current simulation state.
     *
     * Depending on the supplied flags, this method can retrieve only the
     * current simulation time, global statistics, intersection states,
     * or all available information.
     *
     * @param getStatistics Whether to retrieve aggregated simulation statistics.
     * @param getAllIntersectionsStates Whether to retrieve the state of every intersection.
     * @return Current simulation state.
     */
    SimulationState getState(const bool getStatistics,
                             const bool getAllIntersectionsStates) const override {
        ensureInitialized();

        SimulationState state;
        state.step = current_step_;
        state.time_seconds = current_step_ * interval_;

        // Retrieve expensive global statistics only when requested.
        if (getStatistics) {
            auto metrics = cf_engine_->getFinalMetrics();
            state.mean_travel_time         = metrics.meanTravelTime;
            state.mean_waiting_time        = metrics.meanWaitTime;
            state.mean_delay               = metrics.meanDelay;
            state.total_travel_time        = metrics.totalTravelTime;
            state.total_waiting_time       = metrics.totalWaitTime;
            state.total_delay              = metrics.totalDelay;
            state.max_queue_size_until_now = metrics.maxQueueLength;
            state.throughput               = metrics.throughput;
            state.total_vehicles_started   = metrics.totalVehicles;
        }

        // Collect the current state of every intersection.
        if (getAllIntersectionsStates) {
            for (const auto& id : intersection_ids_) {
                state.intersectionStates[id] = getIntersectionState(id);
                state.intersectionStates[id].step = current_step_;
            }
        }

        return state;
    }

    /**
     * @brief Retrieves the current state of a single intersection.
     *
     * Lane-level traffic metrics are computed directly from the vehicles
     * currently present on each incoming lane.
     *
     * @param id Intersection identifier.
     * @return Current intersection state.
     */
    IntersectionState getIntersectionState(const string& id) const override {
        ensureInitialized();

        IntersectionState s;
        s.intersection_id = id;

        const auto& lanes = cf_engine_->getLaneVehiclesForIntersection(id);

        // Process each incoming lane independently.
        for (const auto& lane : lanes) {
            const auto& laneId = lane.first;
            const auto& vehicles = lane.second;

            // Initialize statistics for the current lane.
            s.queue_lengths_per_lane.insert({laneId, 0});
            s.total_cars_per_lane.insert({laneId, 0});
            s.avg_speed_per_lane.insert({laneId, 0.0});
            s.waiting_times_sum_per_lane.insert({laneId, 0.0});

            // Aggregate vehicle-level information into lane statistics.
            for (const auto& vehicle : vehicles) {
                s.total_cars_per_lane[laneId]++;

                // Vehicles moving slower than the threshold are considered queued.
                if (vehicle->getSpeed() < 0.1) { // TODO: better waiting criterion?
                    s.queue_lengths_per_lane[laneId] += 1;
                    s.waiting_times_sum_per_lane[laneId] +=
                        vehicle->getStoppedSince() * interval_;
                }

                s.avg_speed_per_lane[laneId] += vehicle->getSpeed();
            }

            // Compute average lane speed.
            s.avg_speed_per_lane[laneId] = s.total_cars_per_lane[laneId]
                ? s.avg_speed_per_lane[laneId] / s.total_cars_per_lane[laneId]
                : 0.0;
        }

        return s;
    }

    /**
     * @brief Returns identifiers of all intersections.
     */
    vector<string> getIntersectionIDs() const override {
        return intersection_ids_;
    }

    /**
     * @brief Returns the number of signal phases for an intersection.
     *
     * @param id Intersection identifier.
     */
    int getPhaseCount(const string& id) const override {
        return phaseCounts_.at(id);
    }

    /**
     * @brief Checks whether the configured simulation horizon has been reached.
     */
    bool isDone() const override {
        return current_step_ >= max_steps_;
    }

    /**
     * @brief Returns the current simulation step.
     */
    int currentStep() const override {
        return current_step_;
    }

    /**
     * @brief Applies a collection of traffic signal phase changes.
     *
     * Only actions with a valid target phase are forwarded to CityFlow.
     *
     * @param actions Traffic light control actions.
     */
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

    /**
     * @brief Sets the maximum number of simulation steps.
     *
     * @param n Maximum simulation length.
     */
    void setMaxSteps(int n) {
        max_steps_ = n;
    }

private:
    /**
     * @brief Verifies that the simulator has been initialized.
     *
     * @throws runtime_error if initialize() has not been called.
     */
    void ensureInitialized() const {
        if (!initialized_) {
            throw runtime_error("CityFlowEngine: not initialized – call initialize() first");
        }
    }

    /**
     * @brief Caches the identifiers of all intersections.
     *
     * This information is static and retrieved only once during initialization.
     */
    void cacheIntersectionIDs() {
        const auto& ids = cf_engine_->getIntersectionIds();

        for (const auto& id : ids) {
            intersection_ids_.push_back(id);
        }
    }

    /**
     * @brief Caches the number of traffic signal phases for each intersection.
     */
    void cachePhaseCountForIntersection() {
        for (const auto& id : intersection_ids_) {
            phaseCounts_.insert(
                {id, cf_engine_->getTotalPhasesForIntersection(id)}
            );
        }
    }

    /// Underlying CityFlow simulation engine.
    unique_ptr<CityFlow::Engine> cf_engine_;

    /// Cached list of intersection identifiers.
    vector<string> intersection_ids_;

    /// Cached number of signal phases for each intersection.
    map<string, int> phaseCounts_;

    /// Indicates whether the engine has been initialized.
    bool initialized_ = false;

    /// Current simulation step.
    int current_step_ = 0;

    /// Duration of one simulation step (seconds).
    double interval_ = -1.0;

    /// Maximum allowed simulation steps.
    int max_steps_ = 3600;
};

}