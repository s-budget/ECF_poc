#pragma once

#include <iomanip>
#include <iostream>
#include <string>
#include <vector>
#include <unordered_map>
#include <memory>
#include <set>
#include <stdexcept>
using namespace std;
namespace traffic {

struct IntersectionState {
    string  intersection_id ="-1";

    unordered_map<string, int> queue_lengths_per_lane;
    unordered_map<string, int> total_cars_per_lane;
    //curently meter/second
    unordered_map<string, double> avg_speed_per_lane;
    //number of seconds
    unordered_map<string, double> waiting_times_sum_per_lane;

    int step = 0;

};

struct IntersectionAction {
    string intersection_id;
    int target_phase = -1;
};

    struct SimulationState {
        int step = 0;
        double time_seconds = 0.0;

        unordered_map<string, IntersectionState> intersectionStates; // was: vector<IntersectionState> intersections

        double mean_travel_time   = 0.0;
        double mean_waiting_time  = 0.0;
        double mean_delay         = 0.0;
        double total_travel_time  = 0.0;
        double total_waiting_time = 0.0;
        double total_delay        = 0.0;
        int    max_queue_size_until_now = 0;
        int    throughput         = 0;
        int    total_vehicles_started = 0;
    };
    inline void printSimulationState(const SimulationState &state) {
    cout << "========================================\n";
    cout << "Step: " << state.step
              << "  Time: " << fixed << setprecision(1) << state.time_seconds << "s\n";
    cout << "----------------------------------------\n";

    for (const auto &kv : state.intersectionStates) {
        const auto &inter = kv.second;
        cout << "Intersection: " << inter.intersection_id << "\n";
        set<string> laneIds;
        for (const auto &kv : inter.queue_lengths_per_lane)     laneIds.insert(kv.first);
        for (const auto &kv : inter.total_cars_per_lane)        laneIds.insert(kv.first);
        for (const auto &kv : inter.avg_speed_per_lane)         laneIds.insert(kv.first);
        for (const auto &kv : inter.waiting_times_sum_per_lane) laneIds.insert(kv.first);

        for (const auto &laneId : laneIds) {
            int    queue  = inter.queue_lengths_per_lane.count(laneId)        ? inter.queue_lengths_per_lane.at(laneId)        : 0;
            int    total  = inter.total_cars_per_lane.count(laneId)           ? inter.total_cars_per_lane.at(laneId)           : 0;
            double speed  = inter.avg_speed_per_lane.count(laneId)            ? inter.avg_speed_per_lane.at(laneId)            : 0.0;
            double wait   = inter.waiting_times_sum_per_lane.count(laneId)    ? inter.waiting_times_sum_per_lane.at(laneId)    : 0.0;

            cout << "  Lane " << laneId << ":"
                      << "  queue=" << queue
                      << "  total=" << total
                      << "  speed=" << setprecision(2) << speed << "m/s"
                      << "  wait="  << setprecision(1) << wait  << "s\n";
        }
        cout << "----------------------------------------\n";
    }

    cout << "Vehicles  started=" << state.total_vehicles_started
              << "  throughput="      << state.throughput
              << "  max_queue="       << state.max_queue_size_until_now << "\n";
    cout << "Travel    mean="    << setprecision(2) << state.mean_travel_time
              << "s  total="          << state.total_travel_time  << "s\n";
    cout << "Waiting   mean="    << state.mean_waiting_time
              << "s  total="          << state.total_waiting_time << "s\n";
    cout << "Delay     mean="    << state.mean_delay
              << "s  total="          << state.total_delay        << "s\n";
    cout << "========================================\n\n";
}


enum class SimulatorType {
    CityFlow,
    SUMO,
    Unknown
};

struct EngineConfig {
    SimulatorType type        = SimulatorType::CityFlow;
    string   config_file;
    int           num_threads  = 1;
    int           random_seed  = 42;
    bool          headless     = true;
    bool          verbose      = false;
};

class SimulatorEngine {
public:
    explicit SimulatorEngine(const EngineConfig& cfg) : config_(cfg) {}
    virtual ~SimulatorEngine() = default;

    virtual void initialize() = 0;

    virtual void step() = 0;

    virtual void reset() = 0;

    virtual void shutdown() = 0;


    virtual SimulationState getState(const bool getStatistics, const bool getAllIntersectionsStates) const = 0;
    virtual IntersectionState getIntersectionState(const string& id) const = 0;
    virtual vector<string> getIntersectionIDs() const = 0;
    virtual int getPhaseCount(const string& intersection_id) const = 0;
    virtual bool isDone() const = 0;
    virtual int currentStep() const = 0;
    virtual void applyActions(const vector<IntersectionAction>& actions) = 0;

    const EngineConfig& config() const { return config_; }
    SimulatorType       type()   const { return config_.type; }

protected:
    EngineConfig config_;
};


shared_ptr<SimulatorEngine> createEngine(const EngineConfig& cfg);

}
