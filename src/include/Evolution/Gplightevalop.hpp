#pragma once
#include <ecf/ECF.h>
#include <ecf/tree/Tree.h>

#include "../agent/Evaluator.hpp"
#include "../agent/GPLightAgent.hpp"

#include <vector>
#include <memory>

namespace evolution {


class GPLightEvalOp : public EvaluateOp {
public:
    GPLightEvalOp();

    bool initialize(StateP state) override
    {
        return true;
    }

    void setup(Evaluator evaluator,
           std::vector<std::shared_ptr<traffic::GPLightAgent>> agents)
    {
        evaluator_ = evaluator;
        agents_ = agents;
        for (auto& a : agents_)
            base_agents_.push_back(a);
    }

    FitnessP evaluate(IndividualP individual) override
    {

        Tree::Tree* tree =
            static_cast<Tree::Tree*>(individual->getGenotype().get());

        // Share the same tree across all intersection agents
        for (auto& agent : agents_) {
            agent->setTree(tree);
        }
        double att = evaluator_.evaluate(base_agents_);

        FitnessP fitness(new FitnessMin);
        fitness->setValue(att);
        return fitness;
    }

private:
    Evaluator evaluator_;
    std::vector<std::shared_ptr<traffic::GPLightAgent>> agents_;
    std::vector<std::shared_ptr<traffic::Agent>> base_agents_;

};

inline GPLightEvalOp::GPLightEvalOp(): evaluator_() {
}
}
