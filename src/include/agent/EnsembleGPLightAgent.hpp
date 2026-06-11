#pragma once
#include "../agent/Agent.hpp"
#include "../simulator/SimulatorEngine.hpp"
#include "../procesing/roadnet_loader.h"

#include <vector>
#include <string>
#include <numeric>
#include <algorithm>

namespace traffic {

class EnsembleGPLightAgent : public Agent {
public:
    EnsembleGPLightAgent(
        const std::string&                    intersection_id,
        int                                   phase_count,
        IntersectionData                      intersection_data,
        std::vector<std::shared_ptr<Agent>>   sub_agents,
        int                                   default_red_phase = 0,
        int                                   action_duration   = 10,
        int                                   red_duration      = 5)
        : Agent(intersection_id, phase_count, std::move(intersection_data))
        , sub_agents_(std::move(sub_agents))
        , default_red_phase_(default_red_phase)
        , action_duration_(action_duration)
        , red_duration_(red_duration)
    {}

    IntersectionAction selectAction(const IntersectionState& state) override
    {
        last_phase_  = rankPhasesBorda(state);
        last_action_ = IntersectionAction{ intersection_id_, last_phase_ };
        return last_action_;
    }

    std::vector<TimedAction> selectActions(const IntersectionState& state) override
    {
        int prev_phase = last_phase_;
        last_phase_    = rankPhasesBorda(state);
        last_action_   = IntersectionAction{ intersection_id_, last_phase_ };

        if (last_phase_ != prev_phase) {
            IntersectionAction red{ intersection_id_, default_red_phase_ };
            return {
                TimedAction{ red,          red_duration_    },
                TimedAction{ last_action_, action_duration_ }
            };
        }
        return { TimedAction{ last_action_, action_duration_ } };
    }

    std::pair<IntersectionAction, IntersectionAction>
    selectRedGreenActions(const IntersectionState& state) override
    {
        int prev_phase = last_phase_;
        last_phase_    = rankPhasesBorda(state);
        last_action_   = IntersectionAction{ intersection_id_, last_phase_ };

        IntersectionAction green{ intersection_id_, last_phase_ };

        if (last_phase_ != prev_phase) {
            IntersectionAction red{ intersection_id_, default_red_phase_ };
            return { red, green };
        }
        return { green, green };
    }

private:
    std::vector<std::shared_ptr<Agent>> sub_agents_;
    int default_red_phase_;
    int action_duration_;
    int red_duration_;

    int rankPhasesBorda(const IntersectionState& state)
    {
        std::vector<int> borda_scores(phase_count_, 0);

        for (auto& agent : sub_agents_) {
            std::vector<double> urgencies = agent->getPhaseUrgencies(state);

            std::vector<int> order(phase_count_);
            std::iota(order.begin(), order.end(), 0);
            std::sort(order.begin(), order.end(), [&](int a, int b) {
                return urgencies[a] > urgencies[b];
            });

            for (int rank = 0; rank < phase_count_; ++rank)
                borda_scores[order[rank]] += (phase_count_ - 1 - rank);
        }

        return static_cast<int>(
            std::max_element(borda_scores.begin(), borda_scores.end())
            - borda_scores.begin());
    }
};

} // namespace traffic