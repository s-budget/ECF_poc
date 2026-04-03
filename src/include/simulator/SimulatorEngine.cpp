#include "simulator/SimulatorEngine.hpp"
#include "simulator/CityFlowEngine.hpp"

namespace traffic {

    shared_ptr<SimulatorEngine> createEngine(const EngineConfig& cfg) {
        switch (cfg.type) {
            case SimulatorType::CityFlow:
                return make_shared<CityFlowEngine>(cfg);
            default:
                throw runtime_error("createEngine: unsupported simulator type");
        }
    }

}