#pragma once

#include <ecf/ECF.h>
#include <ecf/tree/Tree.h>

#include "../agent/Evaluator.hpp"
#include "../agent/GPLightAgent.hpp"
#include "../agent/ImprovedGPLightAgent.hpp"
#include "../agent/GplightagentWihtCurrentPhase.hpp"

#include <vector>
#include <memory>

namespace evolution {

    /**
     * @brief Evaluation operator for GP-based traffic light agents.
     *
     * Assigns an evolved GP tree to each agent and evaluates the resulting
     * traffic control policy using the simulator.
     */
    template<typename TAgent>
    class GPLightEvalOp : public EvaluateOp {
    public:
        GPLightEvalOp() : evaluator_() {}

        bool initialize(StateP state) override { return true; }

        /**
         * @brief Configures the evaluator with simulation evaluator and agents.
         *
         * Stores the agents that will receive the evolved GP tree during
         * evaluation and keeps a base agent list for simulation execution.
         */
        void setup(Evaluator evaluator,
                   std::vector<std::shared_ptr<TAgent>> agents)
        {
            evaluator_ = evaluator;
            agents_ = agents;
            for (auto& a : agents_)
                base_agents_.push_back(a);
        }

        /**
         * @brief Evaluates one GP individual using the traffic simulation.
         *
         * The individual's tree is assigned to all agents before running the
         * simulation. The returned fitness represents the achieved travel time.
         */
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

    // Evaluation operators for the different GP traffic light agent variants.
    using GPLightEvalOpBasic    = GPLightEvalOp<traffic::GPLightAgent>;
    using GPLightEvalOpImproved = GPLightEvalOp<traffic::ImprovedGPLightAgent>;
    using GPLightEvalOpCurrent = GPLightEvalOp<traffic::GPLightAgentWithCurrentPhase>;

} // namespace evolution