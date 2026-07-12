#pragma once
#include "../agent/Agent.hpp"
#include "../simulator/SimulatorEngine.hpp"
#include "../procesing/roadnet_loader.h"

#include <vector>
#include <string>
#include <numeric>
#include <algorithm>

namespace traffic {

/// Agent that combines decisions from multiple sub-agents.
///
/// Each sub-agent represents a different traffic control strategy.
/// The ensemble combines their decisions using one of two voting methods:
///
///  - Borda voting:
///      Each agent ranks all phases by urgency and phases receive points
///      according to their ranking.
///
///  - Plurality voting:
///      Each agent selects one preferred phase and the phase with the most
///      votes is selected.
///
/// The selected phase can optionally be wrapped with an all-red transition
/// before applying the new green phase.
class EnsembleGPLightAgent : public Agent {
public:

    /// Create an ensemble traffic light controller.
    ///
    /// @param intersection_id Controlled intersection identifier.
    /// @param phase_count Number of available traffic light phases.
    /// @param intersection_data Static intersection road information.
    /// @param sub_agents Individual agents participating in voting.
    /// @param vote_for_all_phases If true use Borda voting, otherwise plurality.
    /// @param default_red_phase Phase used as transition/all-red state.
    /// @param action_duration Duration of the selected green action.
    /// @param red_duration Duration of the red transition phase.
    EnsembleGPLightAgent(
        const std::string&                    intersection_id,
        int                                   phase_count,
        IntersectionData                      intersection_data,
        std::vector<std::shared_ptr<Agent>>   sub_agents,
        bool                                  vote_for_all_phases = true,
        int                                   default_red_phase   = 0,
        int                                   action_duration     = 10,
        int                                   red_duration        = 5)
        : Agent(intersection_id, phase_count, std::move(intersection_data))
        , sub_agents_(std::move(sub_agents))
        , vote_for_all_phases_(vote_for_all_phases)
        , default_red_phase_(default_red_phase)
        , action_duration_(action_duration)
        , red_duration_(red_duration)
    {}

    /// Select the next phase using the configured voting strategy.
    IntersectionAction selectAction(const IntersectionState& state) override
    {
        // Determine winning phase from all sub-agent decisions.
        last_phase_  = electPhase(state);

        last_action_ = IntersectionAction{
            intersection_id_,
            last_phase_
        };

        return last_action_;
    }

    /// Select one or more timed actions.
    ///
    /// If the selected phase changes, insert a red transition phase before
    /// activating the new phase.
    std::vector<TimedAction> selectActions(const IntersectionState& state) override
    {
        int prev_phase = last_phase_;

        // Elect the new phase from sub-agent votes.
        last_phase_  = electPhase(state);
        last_action_ = IntersectionAction{
            intersection_id_,
            last_phase_
        };

        // Phase change requires a red transition.
        if (last_phase_ != prev_phase) {

            IntersectionAction red{
                intersection_id_,
                default_red_phase_
            };

            return {
                TimedAction{ red,          red_duration_    },
                TimedAction{ last_action_, action_duration_ }
            };
        }

        // Keep current phase if no change is required.
        return {
            TimedAction{ last_action_, action_duration_ }
        };
    }

    /// Return red and green actions for red/green transition mode.
    ///
    /// If the phase changes:
    ///     red phase -> selected green phase
    ///
    /// Otherwise:
    ///     keep current phase.
    std::pair<IntersectionAction, IntersectionAction>
    selectRedGreenActions(const IntersectionState& state) override
    {
        int prev_phase = last_phase_;

        // Select the phase preferred by the ensemble.
        last_phase_  = electPhase(state);
        last_action_ = IntersectionAction{
            intersection_id_,
            last_phase_
        };

        IntersectionAction green{
            intersection_id_,
            last_phase_
        };

        // Add red transition only when changing phases.
        if (last_phase_ != prev_phase) {

            IntersectionAction red{
                intersection_id_,
                default_red_phase_
            };

            return { red, green };
        }

        return { green, green };
    }


private:

    // Individual controllers participating in the ensemble vote.
    std::vector<std::shared_ptr<Agent>> sub_agents_;

    // Voting configuration.
    bool vote_for_all_phases_;

    // Phase used during transitions.
    int default_red_phase_;

    // Action timing parameters.
    int action_duration_;
    int red_duration_;


    /// Select the appropriate voting algorithm.
    ///
    /// Borda voting is used when agents provide urgency values for all phases.
    /// Plurality voting is used when each agent only selects one phase.
    int electPhase(const IntersectionState& state)
    {
        if (vote_for_all_phases_)
            return electPhasesBorda(state);
        else
            return electPhasesPlurality(state);
    }


    /// Elect a phase using Borda count voting.
    ///
    /// Every agent ranks all phases by urgency:
    ///     highest urgency -> highest score
    ///     lowest urgency  -> lowest score
    ///
    /// The phase with the highest accumulated score wins.
    int electPhasesBorda(const IntersectionState& state)
    {
        // Store accumulated Borda points for every phase.
        std::vector<int> borda_scores(phase_count_, 0);

        // Collect rankings from every sub-agent.
        for (auto& agent : sub_agents_) {

            std::vector<double> urgencies =
                agent->getPhaseUrgencies(state);

            // Create a list of phases ordered by urgency.
            std::vector<int> order(phase_count_);
            std::iota(order.begin(), order.end(), 0);

            std::sort(order.begin(), order.end(),
                [&](int a, int b) {
                    return urgencies[a] > urgencies[b];
                });

            // Convert ranking positions into Borda points.
            for (int rank = 0; rank < phase_count_; ++rank)
                borda_scores[order[rank]] +=
                    (phase_count_ - 1 - rank);
        }

        // Return phase with maximum score.
        return static_cast<int>(
            std::max_element(
                borda_scores.begin(),
                borda_scores.end())
            - borda_scores.begin());
    }


    /// Elect a phase using simple plurality voting.
    ///
    /// Each agent selects its preferred phase.
    /// The phase receiving the most votes wins.
    int electPhasesPlurality(const IntersectionState& state)
    {
        std::vector<int> vote_counts(phase_count_, 0);

        // Collect one vote from every sub-agent.
        for (auto& agent : sub_agents_) {

            IntersectionAction action =
                agent->selectAction(state);

            vote_counts[action.target_phase]++;
        }

        // Find phase with the highest number of votes.
        int best_phase = 0;

        for (int phase = 1; phase < phase_count_; ++phase)
            if (vote_counts[phase] >= vote_counts[best_phase])
                best_phase = phase;

        return best_phase;
    }
};

} // namespace traffic