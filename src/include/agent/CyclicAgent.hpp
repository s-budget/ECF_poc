#pragma once

#include "Agent.hpp"

namespace traffic {

    /// Simple fixed-time controller that cycles through all non-red phases.
    ///
    /// Phase 0 is reserved as the all-red transition phase and is only used
    /// during red/green switching. The normal control cycle repeatedly selects
    /// phases 1 .. (phase_count - 1).
    class CyclicAgent : public Agent {
    public:
        /// Construct a cyclic traffic light controller.
        CyclicAgent(
            std::string intersection_id,
            int phase_count,
            IntersectionData intersection_data)
            : Agent(
                std::move(intersection_id),
                phase_count,
                std::move(intersection_data))
        {
            // Initialize the controller in the all-red phase.
            // The first call to selectAction() will advance to phase 1.
            last_phase_ = 0;
            last_action_ = IntersectionAction{ intersection_id_, 0 };
        }

        /// Select the next green phase in cyclic order.
        ///
        /// Phase sequence:
        ///   1 -> 2 -> ... -> (phase_count-1) -> 1 -> ...
        ///
        /// Phase 0 is skipped because it is reserved for the red transition.
        IntersectionAction selectAction(
            const IntersectionState& state) override
        {
            (void)state;

            // Handle intersections with only a single available phase.
            if (phase_count_ <= 1) {
                last_phase_ = 0;
            } else {
                // Advance through all green phases and wrap back to phase 1.
                if (last_phase_ < 1 || last_phase_ >= phase_count_ - 1) {
                    last_phase_ = 1;
                } else {
                    ++last_phase_;
                }
            }

            // Store and return the selected action.
            last_action_ = IntersectionAction{
                intersection_id_,
                last_phase_
            };

            return last_action_;
        }

        /// Return the actions used in red/green transition mode.
        ///
        /// The controller first switches to the all-red phase (0),
        /// then activates the next green phase.
        std::pair<IntersectionAction, IntersectionAction>
        selectRedGreenActions(
            const IntersectionState& state) override
        {
            // Transition through the all-red phase.
            IntersectionAction red{
                intersection_id_,
                0
            };

            // Select the next green phase.
            IntersectionAction green = selectAction(state);

            return { red, green };
        }

        /// Reset the controller to its initial state.
        void reset()
        {
            last_phase_ = 0;
            last_action_ = IntersectionAction{
                intersection_id_,
                0
            };
        }
    };

} // namespace traffic