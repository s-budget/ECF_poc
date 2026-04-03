# traffic_rl — Traffic Signal Optimisation with CityFlow + ECF

Evolutionary reinforcement learning for traffic signal control.  
Supports **CityFlow** (implemented) and **SUMO** (stub, ready to fill in).  
Uses **ECF** (Evolutionary Computation Framework) for the learning loop.

---

## Project Structure

```
traffic_rl/
│
├── include/
│   ├── simulator/
│   │   ├── SimulatorEngine.hpp   ← abstract interface + shared data types + factory
│   │   ├── CityFlowEngine.hpp    ← CityFlow implementation
│   │   └── SUMOEngine.hpp        ← SUMO stub (TODO comments)
│   │
│   ├── agent/
│   │   └── Agent.hpp             ← abstract Agent + FixedCycleAgent + MaxPressureAgent
│   │
│   └── learning/
│       ├── FitnessMetrics.hpp    ← per-episode metric accumulator
│       ├── TrafficEvaluator.hpp  ← ECF EvaluateOp (decodes individual → runs episode)
│       ├── LearningRunner.hpp    ← orchestrates ECF evolution loop
│       └── SimulationRunner.hpp  ← single-episode replay (no learning)
│
├── src/
│   ├── simulator/
│   │   └── EngineFactory.cpp     ← createEngine() factory
│   ├── main_train.cpp            ← binary: evolutionary training
│   └── main_simulate.cpp         ← binary: load agent + replay
│
├── data/
│   └── ecf_config.xml            ← ECF algorithm & population settings
│
└── CMakeLists.txt
```

---

## Components

### 1. SimulatorEngine (interface)

`include/simulator/SimulatorEngine.hpp`

All interaction with a traffic simulator goes through this abstract class.  
Construct with `EngineConfig { type, config_file, num_threads, ... }`.

Key methods:
- `initialize()` / `reset()` / `step()` / `shutdown()`
- `getState()` → `SimulationState` (all intersections)
- `getIntersectionState(id)` → `IntersectionState`
- `applyActions(vector<IntersectionAction>)` — control one or more intersections
- `isDone()` / `currentStep()`

To add a new simulator (e.g. Vissim), subclass `SimulatorEngine`, implement all virtual methods, and add a case to `createEngine()` in `EngineFactory.cpp`.

### 2. Agent (abstract base)

`include/agent/Agent.hpp`

One `Agent` instance controls **one intersection**.

Subclass and implement:
- `selectAction(IntersectionState)` → `IntersectionAction`
- `getParameters()` / `setParameters(vector<double>)` — ECF genome interface
- `clone()` — deep copy for ECF population

Provided concrete agents:
- `FixedCycleAgent` — rotates phases with ECF-evolved durations
- `MaxPressureAgent` — greedy max-queue heuristic with tunable min green time

### 3. LearningRunner (training)

`include/learning/LearningRunner.hpp`

Orchestrates the full ECF evolution loop.

```
LearningRunner(engine, agent_templates, LearningConfig)
    .setup()   ← init engine, wire ECF
    .run()     ← evolve until termination
    .getBestAgents()
```

For each ECF individual, `TrafficEvaluator`:
1. Decodes the genome into per-agent parameter slices.
2. Resets engine + agents.
3. Runs a full episode step-by-step.
4. Computes fitness from `FitnessMetrics`.

### 4. SimulationRunner (replay)

`include/learning/SimulationRunner.hpp`

Loads agents with fixed parameters and runs one episode.  
No ECF dependency — can be built without ECF.

---

## Build

### Dependencies

| Library | Version | URL |
|---------|---------|-----|
| CityFlow C++ | latest | https://github.com/cityflow-project/CityFlow |
| ECF | 1.5+ | https://ecf.zemris.fer.hr/ |
| Boost | 1.71+ | https://www.boost.org/ |
| CMake | 3.16+ | — |

### Steps

```bash
mkdir build && cd build

cmake .. \
    -DCITYFLOW_ROOT=/path/to/cityflow \
    -DECF_ROOT=/path/to/ecf \
    -DCMAKE_BUILD_TYPE=Release

make -j$(nproc)
```

Binaries land in `build/bin/`:
- `traffic_train`    — evolutionary training
- `traffic_simulate` — single-episode replay

### Running

**Training:**
```bash
./build/bin/traffic_train
# reads data/ecf_config.xml
# reads data/cityflow_config.json
# saves best agent to output/best_agent.xml
```

**Replay:**
```bash
./build/bin/traffic_simulate
# reads data/cityflow_config.json
# reads output/best_agent_params.txt
```

---

## ECF Configuration

`data/ecf_config.xml` controls:
- **Genotype**: `FloatingPoint` with `dimension = num_intersections × phases_per_agent`
- **Algorithm**: `DifferentialEvolution` (swap for GA, PSO, etc.)
- **Population size**, **max generations**, **stagnation stopping criterion**

For 2 intersections × 4 phases:  `<Entry key="dimension">8</Entry>`  
Bounds: `lbound=5` (min green 5s) → `ubound=90` (max green 90s)

---

## Adding SUMO Support

1. Open `include/simulator/SUMOEngine.hpp`.
2. Follow the `// TODO:` comments — each has the exact `libsumo::` API call.
3. Uncomment the SUMO `find_package` block in `CMakeLists.txt`.
4. In `main_train.cpp` / `main_simulate.cpp`, change `SimulatorType::CityFlow` to `SimulatorType::SUMO`.

---

## Extending Agents

Subclass `Agent`:

```cpp
class MyNNAgent : public traffic::Agent {
public:
    MyNNAgent(const std::string& id, int input_size, int hidden_size)
        : Agent(id, traffic::GenomeType::FloatVector)
        , weights_(input_size * hidden_size + hidden_size * 4, 0.0) {}

    traffic::IntersectionAction selectAction(
        const traffic::IntersectionState& state) override
    {
        // encode state → run through NN → decode action
        traffic::IntersectionAction action;
        action.intersection_id = intersection_id_;
        action.target_phase    = /* argmax of NN output */ 0;
        return action;
    }

    std::vector<double> getParameters() const override { return weights_; }
    void setParameters(const std::vector<double>& p) override { weights_ = p; }
    std::unique_ptr<traffic::Agent> clone() const override {
        return std::make_unique<MyNNAgent>(*this);
    }

private:
    std::vector<double> weights_;
};
```

---

## Fitness Function

Edit `FitnessMetrics::computeFitness()` in `include/learning/FitnessMetrics.hpp`:

```cpp
double computeFitness() const {
    return throughput_weight   * totalThroughput()
         - travel_time_weight  * meanTravelTime()
         - waiting_time_weight * meanWaitingTime();
}
```

All weights are `constexpr` in that function — tune freely.
