#pragma once
#include "../agent/Agent.hpp"
#include "../simulator/SimulatorEngine.hpp"
#include "../procesing/roadnet_loader.h"

#include <ecf/ECF.h>
#include <ecf/tree/Tree.h>

#include <vector>
#include <string>
#include <limits>
#include <map>
#include <set>
#include <tuple>

namespace traffic {

class ImprovedGPLightAgent : public Agent {
public:
    ImprovedGPLightAgent(const std::string& intersection_id,
                         int                phase_count,
                         IntersectionData   intersection_data,
                         int                default_red_phase = 0,
                         int                action_duration   = 10,
                         int                red_duration      = 5)
        : Agent(intersection_id, phase_count, std::move(intersection_data))
        , default_red_phase_(default_red_phase)
        , action_duration_(action_duration)
        , red_duration_(red_duration)
    {
        buildRedPhaseMap();
    }

    void setTree(Tree::Tree* tree) { tree_ = tree; }

    // Legacy single-action interface
    IntersectionAction selectAction(const IntersectionState& state) override
    {
        auto [best_phase, urgencies] = rankPhases(state);
        last_phase_  = best_phase;
        last_action_ = IntersectionAction{ intersection_id_, best_phase };
        return last_action_;
    }

    // Free mode:
    //   - phase changed -> [best_red(red_duration_), new_action(action_duration_)]
    //   - phase same    -> [action(action_duration_)]
    std::vector<TimedAction> selectActions(const IntersectionState& state) override
    {
        int prev_phase = last_phase_;
        auto [best_phase, urgencies] = rankPhases(state);
        last_phase_  = best_phase;
        last_action_ = IntersectionAction{ intersection_id_, best_phase };

        if (best_phase != prev_phase) {
            int red_phase = lookupRedPhase(prev_phase, best_phase);
            IntersectionAction red_action{ intersection_id_, red_phase };
            std::vector<TimedAction> actions;
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
    //   - phase changed -> red=best_red_phase, green=new_phase
    //   - phase same    -> green=same_phase,   green=same_phase
    std::pair<IntersectionAction, IntersectionAction>
    selectRedGreenActions(const IntersectionState& state) override
    {
        int prev_phase = last_phase_;
        auto [best_phase, urgencies] = rankPhases(state);
        last_phase_  = best_phase;
        last_action_ = IntersectionAction{ intersection_id_, best_phase };

        IntersectionAction green{ intersection_id_, best_phase };

        if (best_phase != prev_phase) {
            int red_phase = lookupRedPhase(prev_phase, best_phase);
            IntersectionAction red{ intersection_id_, red_phase };
            return { red, green };
        } else {
            return { green, green };
        }
    }

private:
    Tree::Tree* tree_              = nullptr;
    int         default_red_phase_ = 0;
    int         action_duration_   = 10;
    int         red_duration_      = 5;
    int total_usable_phases_ = 9;  //TODO very imporant, usabble green phases should be first in roadnet

    // (last_phase, new_phase) -> best red phase to use between them
    std::map<std::pair<int,int>, int> red_phase_map_;

    // -----------------------------------------------------------------------
    // Build the red phase map at construction time.
    // For each (from, to) pair, find the phase whose movement set exactly
    // matches the intersection of from's and to's movements.
    // If no such phase exists, fall back to default_red_phase_.
    // -----------------------------------------------------------------------
    void buildRedPhaseMap()
    {
        // Build a set of movement ids per phase for quick lookup
        std::vector<std::set<std::string>> phase_movements(phase_count_);
        for (int p = 0; p < phase_count_; ++p) {
            for (const auto& movement : intersectionRoadData.phasesData[p].movements)
                phase_movements[p].insert(movement.inboundRoadId+"->"+movement.outboundRoadId);
        }

        for (int from = 0; from < total_usable_phases_; ++from) {
            for (int to = 0; to < total_usable_phases_; ++to) {
                if (from == to) continue;

                // Shared movements between from and to
                std::set<std::string> shared;
                for (const auto& m : phase_movements[from])
                    if (phase_movements[to].count(m))
                        shared.insert(m);

                // Find a phase whose movement set is exactly the shared set
                int best_red = default_red_phase_;
                for (int candidate = 0; candidate < phase_count_; ++candidate) {
                    if (phase_movements[candidate] == shared) {
                        best_red = candidate;
                        break;
                    }
                }

                red_phase_map_[{from, to}] = best_red;
            }
        }
    }

    int lookupRedPhase(int from, int to) const
    {
        auto it = red_phase_map_.find({from, to});
        if (it != red_phase_map_.end()) {
            return it->second;
        }
        return default_red_phase_;
    }

    // -----------------------------------------------------------------------
    // Shared core: evaluate every phase, return {best_phase, urgency_per_phase}
    // -----------------------------------------------------------------------
    std::pair<int, std::vector<double>>
    rankPhases(const IntersectionState& state)
    {
        std::vector<double> urgencies(total_usable_phases_);
        int    best_phase   = 0;
        double best_urgency = std::numeric_limits<double>::lowest();

        for (int phase = 0; phase < total_usable_phases_; ++phase) {
            double phase_urgency = 0.0;
            for (const auto& tm_features : extractTMFeatures(state, phase))
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
            "C0", "C1", "C2", "C3"
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
    extractTMFeatures(const IntersectionState& state, int phase)
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

            tm_features.push_back({ w0, w1, w2, w3, c0, c1, c2, c3 });
        }
        return tm_features;
    }
};

} // namespace traffic