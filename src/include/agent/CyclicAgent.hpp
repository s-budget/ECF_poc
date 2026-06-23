#pragma once

#include "Agent.hpp"

namespace traffic {

    class CyclicAgent : public Agent {
    public:
        CyclicAgent(
            std::string intersection_id,
            int phase_count,
            IntersectionData intersection_data)
            : Agent(
                std::move(intersection_id),
                phase_count,
                std::move(intersection_data))
        {
            // Start before phase 1 so first call selects phase 1.
            last_phase_ = 0;
            last_action_ = IntersectionAction{ intersection_id_, 0 };
        }

        IntersectionAction selectAction(
            const IntersectionState& state) override
        {
            (void)state;

            // If only phase 0 exists, stay on it.
            if (phase_count_ <= 1) {
                last_phase_ = 0;
            } else {
                // Cycle through phases 1 .. phase_count_-1.
                if (last_phase_ < 1 || last_phase_ >= phase_count_ - 1) {
                    last_phase_ = 1;
                } else {
                    ++last_phase_;
                }
            }

            last_action_ = IntersectionAction{
                intersection_id_,
                last_phase_
            };

            return last_action_;
        }

        std::pair<IntersectionAction, IntersectionAction>
        selectRedGreenActions(
            const IntersectionState& state) override
        {
            IntersectionAction red{
                intersection_id_,
                0
            };

            IntersectionAction green = selectAction(state);

            return { red, green };
        }

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