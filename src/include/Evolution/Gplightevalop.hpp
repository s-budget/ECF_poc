#pragma once
#include <ecf/ECF.h>
#include <ecf/tree/Tree.h>

#include "../agent/Evaluator.hpp"
#include "../agent/GPLightAgent.hpp"
#include "../agent/ImprovedGPLightAgent.hpp"

#include <vector>
#include <memory>

namespace evolution {

    template<typename TAgent>
    class GPLightEvalOp : public EvaluateOp {
    public:
        GPLightEvalOp() : evaluator_() {}

        bool initialize(StateP state) override { return true; }

        void setup(Evaluator evaluator,
                   std::vector<std::shared_ptr<TAgent>> agents)
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

            for (auto& agent : agents_)
                agent->setTree(tree);

            FitnessP fitness(new FitnessMin);
            fitness->setValue(evaluator_.evaluate(base_agents_));
            return fitness;
        }

    private:
        Evaluator evaluator_;
        std::vector<std::shared_ptr<TAgent>> agents_;
        std::vector<std::shared_ptr<traffic::Agent>> base_agents_;
    };

    using GPLightEvalOpBasic    = GPLightEvalOp<traffic::GPLightAgent>;
    using GPLightEvalOpImproved = GPLightEvalOp<traffic::ImprovedGPLightAgent>;

} // namespace evolution