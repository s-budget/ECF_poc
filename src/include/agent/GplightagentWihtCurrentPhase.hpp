#pragma once
#include "../agent/Agent.hpp"
#include "../simulator/SimulatorEngine.hpp"
#include "../procesing/roadnet_loader.h"

#include <ecf/ECF.h>
#include <ecf/tree/Tree.h>

#include <vector>
#include <string>
#include <limits>

namespace traffic {

class GPLightAgentWithCurrentPhase : public Agent {
public:
    GPLightAgentWithCurrentPhase(const std::string& intersection_id,
                 int                phase_count,
                 IntersectionData   intersection_data,
                 int                default_red_phase = 0,
                 int                action_duration   = 10,
                 int                red_duration      = 5)
        : Agent(intersection_id, phase_count, std::move(intersection_data))
        , default_red_phase_(default_red_phase)
        , action_duration_(action_duration)
        , red_duration_(red_duration)
    {}

    void setTree(Tree::Tree* tree) { tree_ = tree; }

    // Legacy single-action interface
    IntersectionAction selectAction(const IntersectionState& state) override
    {
        auto [best_phase, urgencies] = rankPhases(state);
        last_phase_  = best_phase;
        last_action_ = IntersectionAction{ intersection_id_, best_phase };
        return last_action_;
    }

    std::vector<double> getPhaseUrgencies(const IntersectionState& state)
    {
        auto [best_phase, urgencies] = rankPhases(state);
        return urgencies;
    }

    // Free mode:
    //   - phase changed -> [red(red_duration_), new_action(action_duration_)]
    //   - phase same    -> [action(action_duration_)]
    std::vector<TimedAction> selectActions(const IntersectionState& state) override
    {
        int prev_phase = last_phase_;
        auto [best_phase, urgencies] = rankPhases(state);
        last_phase_  = best_phase;
        last_action_ = IntersectionAction{ intersection_id_, best_phase };

        if (best_phase != prev_phase) {
            IntersectionAction red_action{ intersection_id_, default_red_phase_ };
            return {
                TimedAction{ red_action,   red_duration_    },
                TimedAction{ last_action_, action_duration_ }
            };
        } else {
            return {
                TimedAction{ last_action_, action_duration_ }
            };
        }
    }

    // Red/green mode:
    //   - phase changed -> red=default_red_phase_, green=new_phase
    //   - phase same    -> green=same_phase, green=same_phase
    std::pair<IntersectionAction, IntersectionAction>
    selectRedGreenActions(const IntersectionState& state) override
    {
        int prev_phase = last_phase_;
        auto [best_phase, urgencies] = rankPhases(state);
        last_phase_  = best_phase;
        last_action_ = IntersectionAction{ intersection_id_, best_phase };

        IntersectionAction green{ intersection_id_, best_phase };

        if (best_phase != prev_phase) {
            IntersectionAction red{ intersection_id_, default_red_phase_ };
            return { red, green };
        } else {
            return { green, green };
        }
    }


private:
    Tree::Tree* tree_             = nullptr;
    int         default_red_phase_ = 0;
    int         action_duration_   = 10;
    int         red_duration_      = 5;

    // -----------------------------------------------------------------------
    // Shared core: evaluate every phase, return {best_phase, urgency_per_phase}
    // -----------------------------------------------------------------------
    std::pair<int, std::vector<double>>
    rankPhases(const IntersectionState& state)
    {
        std::vector<double> urgencies(phase_count_);
        int    best_phase   = 0;
        double best_urgency = std::numeric_limits<double>::lowest();

        for (int phase = 0; phase < phase_count_; ++phase) {
            double phase_urgency = 0.0;
            for (const auto& tm_features : extractTMFeatures(state, phase,phase==last_phase_))
                phase_urgency += evaluateTree(tm_features);

            urgencies[phase] = phase_urgency;
            if (phase_urgency > best_urgency) {
                best_urgency = phase_urgency;
                best_phase   = phase;
            }
        }

        return { best_phase, urgencies };
    }

    double evaluateTree(const std::vector<double>& features)
    {
        static const std::string featureNames[] = {
            "W0", "W1", "W2", "W3",
            "C0", "C1", "C2", "C3",
            "P"
        };

        for (int i = 0; i < static_cast<int>(features.size()); ++i) {
            double val = features[i];
            tree_->setTerminalValue(featureNames[i], &val);
        }

        double result = 0.0;
        tree_->execute(&result);
        return result;
    }

    std::vector<std::vector<double>>
    extractTMFeatures(const IntersectionState& state, int phase, bool current_phase)
    {
        std::vector<std::vector<double>> tm_features;
        for (int i = 0; i < intersectionRoadData.phasesData[phase].movements.size(); i++) {
            double c0 = 0, w0 = 0;
            for (int j = 0; j < intersectionRoadData.phasesData[phase].movements[i].inboundLaneIds.size(); j++) {
                c0 += state.total_cars_per_lane.at(intersectionRoadData.phasesData[phase].movements[i].inboundLaneIds[j]);
                w0 += state.queue_lengths_per_lane.at(intersectionRoadData.phasesData[phase].movements[i].inboundLaneIds[j]);
            }

            double c1 = 0, w1 = 0;
            for (int j = 0; j < intersectionRoadData.roadTurnLanes[intersectionRoadData.phasesData[phase].movements[i].outboundRoadId][TurnType::TurnLeft].size(); j++) {
                c1 += state.total_cars_per_lane.at(intersectionRoadData.roadTurnLanes[intersectionRoadData.phasesData[phase].movements[i].outboundRoadId][TurnType::TurnLeft][j]);
                w1 += state.queue_lengths_per_lane.at(intersectionRoadData.roadTurnLanes[intersectionRoadData.phasesData[phase].movements[i].outboundRoadId][TurnType::TurnLeft][j]);
            }

            double c2 = 0, w2 = 0;
            for (int j = 0; j < intersectionRoadData.roadTurnLanes[intersectionRoadData.phasesData[phase].movements[i].outboundRoadId][TurnType::Straight].size(); j++) {
                c2 += state.total_cars_per_lane.at(intersectionRoadData.roadTurnLanes[intersectionRoadData.phasesData[phase].movements[i].outboundRoadId][TurnType::Straight][j]);
                w2 += state.queue_lengths_per_lane.at(intersectionRoadData.roadTurnLanes[intersectionRoadData.phasesData[phase].movements[i].outboundRoadId][TurnType::Straight][j]);
            }

            double c3 = 0, w3 = 0;
            for (int j = 0; j < intersectionRoadData.roadTurnLanes[intersectionRoadData.phasesData[phase].movements[i].outboundRoadId][TurnType::TurnRight].size(); j++) {
                c3 += state.total_cars_per_lane.at(intersectionRoadData.roadTurnLanes[intersectionRoadData.phasesData[phase].movements[i].outboundRoadId][TurnType::TurnRight][j]);
                w3 += state.queue_lengths_per_lane.at(intersectionRoadData.roadTurnLanes[intersectionRoadData.phasesData[phase].movements[i].outboundRoadId][TurnType::TurnRight][j]);
            }

            tm_features.push_back({ w0, w1, w2, w3, c0, c1, c2, c3, (current_phase?1.0:-1.0) });
        }
        return tm_features;
    }
};

} // namespace traffic