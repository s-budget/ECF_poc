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
    /**
     * @brief Represents the current traffic-related state of a single intersection.
     *
     * Stores per-lane traffic metrics collected from the simulator for one simulation step.
     */
    struct IntersectionState {
        /// Unique identifier of the intersection.
        string intersection_id = "-1";

        /// Number of queued vehicles per incoming lane.
        unordered_map<string, int> queue_lengths_per_lane;

        /// Total number of vehicles currently occupying each lane.
        unordered_map<string, int> total_cars_per_lane;

        /// Average vehicle speed for each lane (m/s).
        unordered_map<string, double> avg_speed_per_lane;

        /// Sum of waiting times of all vehicles on each lane (seconds).
        unordered_map<string, double> waiting_times_sum_per_lane;

        /// Simulation step when this state was collected.
        int step = 0;
    };
    /**
     * @brief Represents a control action applied to a traffic intersection.
     *
     * The action consists of selecting the target traffic light phase
     * for a specific intersection.
     */
struct IntersectionAction {
    /// Target intersection identifier.
    string intersection_id;

    /// Phase index to be activated.
    int target_phase = -1;
};

/**
 * @brief Represents the complete simulation state.
 *
 * Contains both global traffic statistics and the state of all monitored
 * intersections for the current simulation step.
 */
struct SimulationState {
    /// Current simulation step.
    int step = 0;

    /// Elapsed simulation time (seconds).
    double time_seconds = 0.0;

    /// State of every tracked intersection indexed by intersection ID.
    unordered_map<string, IntersectionState> intersectionStates;

    /// Global average travel time.
    double mean_travel_time = 0.0;

    /// Global average waiting time.
    double mean_waiting_time = 0.0;

    /// Global average delay.
    double mean_delay = 0.0;

    /// Total accumulated travel time.
    double total_travel_time = 0.0;

    /// Total accumulated waiting time.
    double total_waiting_time = 0.0;

    /// Total accumulated delay.
    double total_delay = 0.0;

    /// Largest queue observed since the simulation started.
    int max_queue_size_until_now = 0;

    /// Number of vehicles that successfully completed their routes.
    int throughput = 0;

    /// Total number of vehicles introduced into the simulation.
    int total_vehicles_started = 0;
};

/**
 * @brief Prints a human-readable representation of the simulation state.
 *
 * Outputs both global simulation statistics and detailed per-lane
 * information for every intersection.
 *
 * @param state Simulation state to print.
 */
inline void printSimulationState(const SimulationState &state) {
    cout << "========================================\n";
    cout << "Step: " << state.step
         << "  Time: " << fixed << setprecision(1) << state.time_seconds << "s\n";
    cout << "----------------------------------------\n";

    // Print the state of every intersection.
    for (const auto &kv : state.intersectionStates) {
        const auto &inter = kv.second;
        cout << "Intersection: " << inter.intersection_id << "\n";

        // Collect all lane IDs that appear in any metric so every lane
        // is printed even if some statistics are unavailable.
        set<string> laneIds;
        for (const auto &kv : inter.queue_lengths_per_lane)     laneIds.insert(kv.first);
        for (const auto &kv : inter.total_cars_per_lane)        laneIds.insert(kv.first);
        for (const auto &kv : inter.avg_speed_per_lane)         laneIds.insert(kv.first);
        for (const auto &kv : inter.waiting_times_sum_per_lane) laneIds.insert(kv.first);

        // Print all available metrics for every lane.
        for (const auto &laneId : laneIds) {
            int queue = inter.queue_lengths_per_lane.count(laneId)
                            ? inter.queue_lengths_per_lane.at(laneId)
                            : 0;

            int total = inter.total_cars_per_lane.count(laneId)
                            ? inter.total_cars_per_lane.at(laneId)
                            : 0;

            double speed = inter.avg_speed_per_lane.count(laneId)
                               ? inter.avg_speed_per_lane.at(laneId)
                               : 0.0;

            double wait = inter.waiting_times_sum_per_lane.count(laneId)
                              ? inter.waiting_times_sum_per_lane.at(laneId)
                              : 0.0;

            cout << "  Lane " << laneId << ":"
                 << "  queue=" << queue
                 << "  total=" << total
                 << "  speed=" << setprecision(2) << speed << "m/s"
                 << "  wait=" << setprecision(1) << wait << "s\n";
        }

        cout << "----------------------------------------\n";
    }

    // Print aggregated simulation statistics.
    cout << "Vehicles  started=" << state.total_vehicles_started
         << "  throughput=" << state.throughput
         << "  max_queue=" << state.max_queue_size_until_now << "\n";

    cout << "Travel    mean=" << setprecision(2) << state.mean_travel_time
         << "s  total=" << state.total_travel_time << "s\n";

    cout << "Waiting   mean=" << state.mean_waiting_time
         << "s  total=" << state.total_waiting_time << "s\n";

    cout << "Delay     mean=" << state.mean_delay
         << "s  total=" << state.total_delay << "s\n";

    cout << "========================================\n\n";
}

/**
 * @brief Supported traffic simulator backends.
 */
enum class SimulatorType {
    CityFlow,
    SUMO,
    Unknown
};

/**
 * @brief Configuration used to initialize a simulator engine.
 */
struct EngineConfig {
    /// Simulator backend implementation.
    SimulatorType type = SimulatorType::CityFlow;

    /// Path to the simulator configuration file.
    string config_file;

    /// Number of worker threads used by the simulator.
    int num_threads = 1;

    /// Random seed for deterministic execution.
    int random_seed = 42;

    /// Whether the simulator should run without a graphical interface.
    bool headless = true;

    /// Enables verbose logging.
    bool verbose = false;
};

/**
 * @brief Abstract interface for traffic simulation engines.
 *
 * This class defines a common API that allows the application to interact
 * with different simulator backends (e.g. CityFlow or SUMO) without
 * depending on implementation-specific details.
 */
class SimulatorEngine {
public:
    /**
     * @brief Constructs a simulator engine using the provided configuration.
     *
     * @param cfg Engine configuration.
     */
    explicit SimulatorEngine(const EngineConfig& cfg) : config_(cfg) {}

    /// Virtual destructor for proper polymorphic cleanup.
    virtual ~SimulatorEngine() = default;

    /// Initializes the simulation.
    virtual void initialize() = 0;

    /// Advances the simulation by one step.
    virtual void step() = 0;

    /// Resets the simulation to its initial state.
    virtual void reset() = 0;

    /// Releases simulator resources.
    virtual void shutdown() = 0;

    /**
     * @brief Retrieves the current simulation state.
     *
     * @param getStatistics Whether to compute global statistics.
     * @param getAllIntersectionsStates Whether to retrieve states for all intersections.
     * @return Current simulation state.
     */
    virtual SimulationState getState(const bool getStatistics,
                                     const bool getAllIntersectionsStates) const = 0;

    /**
     * @brief Retrieves the state of a specific intersection.
     *
     * @param id Intersection identifier.
     * @return Current intersection state.
     */
    virtual IntersectionState getIntersectionState(const string& id) const = 0;

    /**
     * @brief Returns identifiers of all intersections in the network.
     */
    virtual vector<string> getIntersectionIDs() const = 0;

    /**
     * @brief Returns the number of signal phases available for an intersection.
     *
     * @param intersection_id Intersection identifier.
     */
    virtual int getPhaseCount(const string& intersection_id) const = 0;

    /**
     * @brief Indicates whether the simulation has finished.
     */
    virtual bool isDone() const = 0;

    /**
     * @brief Returns the current simulation step.
     */
    virtual int currentStep() const = 0;

    /**
     * @brief Applies traffic signal actions to the simulator.
     *
     * @param actions Collection of intersection actions.
     */
    virtual void applyActions(const vector<IntersectionAction>& actions) = 0;

    /**
     * @brief Returns the engine configuration.
     */
    const EngineConfig& config() const { return config_; }

    /**
     * @brief Returns the simulator backend type.
     */
    SimulatorType type() const { return config_.type; }

protected:
    /// Configuration shared by all simulator implementations.
    EngineConfig config_;
};

/**
 * @brief Factory function for creating a simulator engine.
 *
 * Creates the appropriate simulator implementation based on the
 * selected simulator type in the configuration.
 *
 * @param cfg Engine configuration.
 * @return Shared pointer to the created simulator engine.
 */
shared_ptr<SimulatorEngine> createEngine(const EngineConfig& cfg);

}