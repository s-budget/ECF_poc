#pragma once
#include "../procesing/roadnet_loader.h"
#include "../simulator/SimulatorEngine.hpp"

using namespace std;

namespace traffic {

    /// Action together with the number of simulation steps
    /// for which it should remain active.
    struct TimedAction {
        IntersectionAction action;
        int duration;
    };

    /// Base interface for all traffic light control agents.
    ///
    /// An Agent is responsible for selecting the traffic light phase
    /// for a single intersection. Different control strategies
    /// (fixed-time, evolutionary, reinforcement learning, etc.)
    /// should derive from this class and override the desired methods.
    class Agent {
    public:
        /// Construct an agent for a specific intersection.
        ///
        /// @param intersection_id Unique intersection identifier.
        /// @param phase_count Number of available traffic light phases.
        /// @param intersection_data Static road network information.
        Agent(string intersection_id, int phase_count, IntersectionData intersection_data)
            : intersection_id_(move(intersection_id))
            , phase_count_(phase_count)
            , intersectionRoadData(move(intersection_data))
        {}

        virtual ~Agent() = default;

        /// Select the next traffic light phase.
        ///
        /// Default implementation simply cycles through all available phases.
        virtual IntersectionAction selectAction(const IntersectionState& state) {
            last_phase_ = (last_phase_ + 1) % phase_count_;
            last_action_ = IntersectionAction{ intersection_id_, last_phase_ };
            return last_action_;
        }

        /// Compute an urgency score for every phase.
        ///
        /// These scores can be used for visualization or by higher-level
        /// controllers. The default implementation assigns zero urgency
        /// to every phase.
        virtual std::vector<double> getPhaseUrgencies(const IntersectionState& state)
        {
            return std::vector<double>(phase_count_, 0.0);
        }

        /// Select a sequence of timed actions.
        ///
        /// This interface is used by controllers that schedule several
        /// consecutive phase changes at once.
        ///
        /// Default implementation wraps the result of selectAction()
        /// into a single action lasting one simulation step.
        virtual vector<TimedAction> selectActions(const IntersectionState& state) {
            return { TimedAction{ selectAction(state), 1 } };
        }

        /// Select actions for red/green transition mode.
        ///
        /// Returns:
        ///   - first  -> phase applied during the red transition,
        ///   - second -> phase applied after the transition.
        ///
        /// Default implementation keeps the current phase during red
        /// and then advances to the next phase.
        virtual pair<IntersectionAction, IntersectionAction>
        selectRedGreenActions(const IntersectionState& state) {
            IntersectionAction red  { intersection_id_, last_phase_ };
            IntersectionAction green{ intersection_id_, (last_phase_ + 1) % phase_count_ };
            return { red, green };
        }

        /// Reset the agent to its initial state.
        void reset() {
            last_phase_  = 0;
            last_action_ = IntersectionAction{ intersection_id_, 0 };
        }

        /// Return the controlled intersection identifier.
        const string& getIntersectionId() const { return intersection_id_; }

        /// Return the last selected traffic light phase.
        int getLastPhase() const { return last_phase_; }

        /// Return the last action produced by the agent.
        const IntersectionAction& getLastAction() const { return last_action_; }

    protected:
        /// Identifier of the controlled intersection.
        string intersection_id_;

        /// Static road network information for this intersection.
        IntersectionData intersectionRoadData;

        /// Total number of traffic light phases.
        int phase_count_ = 0;

        /// Last selected phase.
        int last_phase_ = 0;

        /// Last action returned by the agent.
        IntersectionAction last_action_;
    };

}