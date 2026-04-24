#pragma once
#include "../agent/Agent.hpp"
#include "../simulator/SimulatorEngine.hpp"
#include "../procesing/roadnet_loader.h"

#include <ecf/ECF.h>           // ECF core
#include <ecf/tree/Tree.h>     // Tree genotype

#include <vector>
#include <string>
#include <limits>

namespace traffic {

class GPLightAgent : public Agent {
public:
    GPLightAgent(const std::string& intersection_id,
                 int                phase_count,
                 IntersectionData   intersection_data)
        : Agent(intersection_id, phase_count, std::move(intersection_data))
    {}

    void setTree(Tree::Tree* tree) { tree_ = tree; }

    IntersectionAction selectAction(const IntersectionState& state) override
    {
        if (!tree_) {
            return Agent::selectAction(state);
        }

        int    best_phase   = 0;
        double best_urgency = std::numeric_limits<double>::lowest();

        for (int phase = 0; phase < phase_count_; ++phase)
        {

            std::vector<std::vector<double>> tms =
                extractTMFeatures(state, phase);

            double phase_urgency = 0.0;
            for (const auto& tm_features : tms) {
                phase_urgency += evaluateTree(tm_features);
            }

            if (phase_urgency > best_urgency) {
                best_urgency = phase_urgency;
                best_phase   = phase;
            }
        }

        last_phase_  = best_phase;
        last_action_ = IntersectionAction{ intersection_id_, best_phase };
        return last_action_;
    }

private:
    Tree::Tree* tree_ = nullptr;

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
      vector<vector<double>> tm_features;
      for (int i=0;i<intersectionRoadData.phasesData[phase].movements.size();i++) {
            double c0=0;
            double w0=0;
        for (int j=0;j<intersectionRoadData.phasesData[phase].movements[i].inboundLaneIds.size();j++) {
            c0+=state.total_cars_per_lane.at(intersectionRoadData.phasesData[phase].movements[i].inboundLaneIds[j]);
            w0+=state.queue_lengths_per_lane.at(intersectionRoadData.phasesData[phase].movements[i].inboundLaneIds[j]);
        }
          double c1=0;
          double w1=0;
        for (int j=0;j<intersectionRoadData.roadTurnLanes[intersectionRoadData.phasesData[phase].movements[i].outboundRoadId][TurnType::TurnLeft].size();j++) {
            c1+=state.total_cars_per_lane.at(intersectionRoadData.roadTurnLanes[intersectionRoadData.phasesData[phase].movements[i].outboundRoadId][TurnType::TurnLeft][j]);
            w1+=state.queue_lengths_per_lane.at(intersectionRoadData.roadTurnLanes[intersectionRoadData.phasesData[phase].movements[i].outboundRoadId][TurnType::TurnLeft][j]);
        }
          double c2=0;
          double w2=0;
          for (int j=0;j<intersectionRoadData.roadTurnLanes[intersectionRoadData.phasesData[phase].movements[i].outboundRoadId][TurnType::Straight].size();j++) {
              c2+=state.total_cars_per_lane.at(intersectionRoadData.roadTurnLanes[intersectionRoadData.phasesData[phase].movements[i].outboundRoadId][TurnType::Straight][j]);
              w2+=state.queue_lengths_per_lane.at(intersectionRoadData.roadTurnLanes[intersectionRoadData.phasesData[phase].movements[i].outboundRoadId][TurnType::Straight][j]);
          }
          double c3=0;
          double w3=0;
          for (int j=0;j<intersectionRoadData.roadTurnLanes[intersectionRoadData.phasesData[phase].movements[i].outboundRoadId][TurnType::TurnRight].size();j++) {
              c3+=state.total_cars_per_lane.at(intersectionRoadData.roadTurnLanes[intersectionRoadData.phasesData[phase].movements[i].outboundRoadId][TurnType::TurnRight][j]);
              w3+=state.queue_lengths_per_lane.at(intersectionRoadData.roadTurnLanes[intersectionRoadData.phasesData[phase].movements[i].outboundRoadId][TurnType::TurnRight][j]);
          }

          vector<double> tmInputs;
          tmInputs.push_back(w0);
          tmInputs.push_back(w1);
          tmInputs.push_back(w2);
          tmInputs.push_back(w3);
          tmInputs.push_back(c0);
          tmInputs.push_back(c1);
          tmInputs.push_back(c2);
          tmInputs.push_back(c3);
          tm_features.push_back(tmInputs);
      }
      return tm_features;
    }
};

}