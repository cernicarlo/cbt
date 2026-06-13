#pragma once

#include "behaviortree_cpp/behavior_tree.h"

namespace chr = std::chrono;

class ContextualCondition : public BT::ConditionNode {
  public:
   ContextualCondition(const std::string& name,
                       const BT::NodeConfiguration& config)
       : BT::ConditionNode(name, config) {}

   /// Method to setup stuff happening in the node(such as name_, name_context_)
   virtual void construction() = 0;

   /// Method to setup the function/core of the condition -
   /// set success_
   virtual void setFunction() = 0;

  protected:
   // do not override this method
   // BT::ReturnStatus Tick();
   BT::NodeStatus tick() override final;
   std::string name_;
   std::string name_context_;
   bool emergency_;
   bool success_;

  private:
   bool first_tick_ = true;
};
