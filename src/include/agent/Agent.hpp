#pragma once
#include "../procesing/roadnet_loader.h"
#include "../simulator/SimulatorEngine.hpp"
using namespace std;
namespace traffic {
    struct TimedAction {
        IntersectionAction action;
        int duration;
    };

    class Agent {
    public:
        Agent(string intersection_id, int phase_count, IntersectionData intersection_data)
            : intersection_id_(move(intersection_id))
            , phase_count_(phase_count)
            , intersectionRoadData(move(intersection_data))
        {}

        virtual ~Agent() = default;

        // Legacy single-action interface
        virtual IntersectionAction selectAction(const IntersectionState& state) {
            last_phase_ = (last_phase_ + 1) % phase_count_;
            last_action_ = IntersectionAction{ intersection_id_, last_phase_ };
            return last_action_;
        }

        // Free mode — return a series of actions with durations (at least one element).
        // Default: wraps selectAction in a single TimedAction with duration 1.
        virtual vector<TimedAction> selectActions(const IntersectionState& state) {
            return { TimedAction{ selectAction(state), 1 } };
        }

        // Red/green mode — return exactly {red_action, green_action}.
        // Default: red = current phase, green = next phase.
        virtual pair<IntersectionAction, IntersectionAction>
        selectRedGreenActions(const IntersectionState& state) {
            IntersectionAction red  { intersection_id_, last_phase_ };
            IntersectionAction green{ intersection_id_, (last_phase_ + 1) % phase_count_ };
            return { red, green };
        }

        void reset() {
            last_phase_  = 0;
            last_action_ = IntersectionAction{ intersection_id_, 0 };
        }

        const string&            getIntersectionId() const { return intersection_id_; }
        int                      getLastPhase()      const { return last_phase_; }
        const IntersectionAction& getLastAction()    const { return last_action_; }

    protected:
        string             intersection_id_;
        IntersectionData   intersectionRoadData;
        int                phase_count_   = 0;
        int                last_phase_    = 0;
        IntersectionAction last_action_;
    };

}