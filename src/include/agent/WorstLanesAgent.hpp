#pragma once

#include "Agent.hpp"

namespace traffic {

    class WorstLanesAgent : public Agent {
    public:
        WorstLanesAgent(string intersection_id, int phase_count,  IntersectionData intersection_data )
            : Agent(move(intersection_id), phase_count, move(intersection_data))
        {}

        IntersectionAction selectAction(const IntersectionState& state) override {
            int maxCars=-1;
            int newPhase=-1;
            for (int i=0; i<phase_count_; i++) {
                int totalCarsForPhase=0;
                for (string const& laneId : intersectionRoadData.phaseLanes[i]) {
                    totalCarsForPhase+=state.total_cars_per_lane.at(laneId);
                }
                if (totalCarsForPhase > maxCars) {
                    maxCars=totalCarsForPhase;
                    newPhase=i;
                }
            }
            return IntersectionAction{intersection_id_,newPhase};
        }

        void reset() {
            Agent::reset();
        }

    private:
    };

}