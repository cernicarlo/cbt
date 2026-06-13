#include "bt_policy/post_cond_control_nodes.h"

namespace BT {

std::pair<std::string, std::string> splitAtLastUnderscore(
    const std::string& input) {
   // Find the position of the last underscore
   size_t pos = input.rfind('_');
   if (pos != std::string::npos) {
      // Split the string into two parts
      std::string firstPart = input.substr(0, pos);
      std::string lastPart = input.substr(pos + 1);
      return {firstPart, lastPart};
   } else {
      // Return the input as the first part and an empty string as the second
      // part if no underscore is found
      std::cout << "no condition set in Post Node" << std::endl;
      return {input, ""};
   }
}

PostFallbackNode::PostFallbackNode(const std::string& name)
    : ControlNode::ControlNode(name, {}),
      current_child_idx_(0),
      all_skipped_(true) {
   setRegistrationID("PostFallback");

   // initialize node variables
   auto [condition, target] = splitAtLastUnderscore(name);
   name_ = name;
   target_ = target;
   condition_ = condition;
   // check if postcondition is satisfied
   char const* post = getenv(condition.c_str());
   if (post != NULL) {
      std::cout << "*************For PostFallback " << name
                << ", post-condition " << condition << " is " << post;
      if (post == target) {
         std::cout << "; so no need to execute it " << std::endl;
         post_condition_ = true;
      } else {
         std::cout << "; so I need to execute it " << std::endl;
         post_condition_ = false;
      }
   }
}

NodeStatus PostFallbackNode::tick() {
   if (post_condition_) {
      std::cout << "returning success from the tick" << std::endl;
      return NodeStatus::SUCCESS;
   }
   const size_t children_count = children_nodes_.size();

   if (status() == NodeStatus::IDLE) {
      all_skipped_ = true;
   }

   setStatus(NodeStatus::RUNNING);

   while (current_child_idx_ < children_count) {
      TreeNode* current_child_node = children_nodes_[current_child_idx_];

      auto prev_status = current_child_node->status();
      const NodeStatus child_status = current_child_node->executeTick();

      // switch to RUNNING state as soon as you find an active child
      all_skipped_ &= (child_status == NodeStatus::SKIPPED);

      switch (child_status) {
         case NodeStatus::RUNNING: {
            return child_status;
         }
         case NodeStatus::SUCCESS: {
            setenv(condition_.c_str(), target_.c_str(), 1);
            std::cout << "]]]]]]]]]]]]]]]]]]]]]]]]POSTFALLBACK: getenv("
                      << condition_.c_str()
                      << "):" << getenv(condition_.c_str()) << std::endl;
            std::cout << std::endl;
            resetChildren();
            current_child_idx_ = 0;
            return child_status;
         }
         case NodeStatus::FAILURE: {
            current_child_idx_++;
            // Return the execution flow if the child is async,
            // to make this interruptable.
            if (requiresWakeUp() && prev_status == NodeStatus::IDLE &&
                current_child_idx_ < children_count) {
               emitWakeUpSignal();
               return NodeStatus::RUNNING;
            }
         } break;
         case NodeStatus::SKIPPED: {
            // It was requested to skip this node
            current_child_idx_++;
         } break;
         case NodeStatus::IDLE: {
            throw LogicError("[", name(),
                             "]: A children should not return IDLE");
         }
      }  // end switch
   }     // end while loop

   // The entire while loop completed. This means that all the children returned
   // FAILURE.
   if (current_child_idx_ == children_count) {
      resetChildren();
      current_child_idx_ = 0;
   }

   // Skip if ALL the nodes have been skipped
   return all_skipped_ ? NodeStatus::SKIPPED : NodeStatus::FAILURE;
}

void PostFallbackNode::halt() {
   current_child_idx_ = 0;
   ControlNode::halt();
}

PostSequenceNode::PostSequenceNode(const std::string& name)
    : ControlNode::ControlNode(name, {}),
      current_child_idx_(0),
      all_skipped_(true) {
   setRegistrationID("PostSequence");

   // initialize node variables
   auto [condition, target] = splitAtLastUnderscore(name);
   name_ = name;
   target_ = target;
   condition_ = condition;
   std::cout << "*************For PostSequence costr" << std::endl;

   // check if postcondition is satisfied
   char const* post = getenv(condition.c_str());
   if (post != NULL) {
      std::cout << "*************For PostSequence " << name
                << ", post-condition" << condition << " is " << post;
      if (post == target) {
         std::cout << "; so no need to execute it " << std::endl;
         post_condition_ = true;
      } else {
         std::cout << "; so I need to execute it " << std::endl;
         post_condition_ = false;
      }
   }
}

void PostSequenceNode::halt() {
   current_child_idx_ = 0;
   ControlNode::halt();
}

NodeStatus PostSequenceNode::tick() {
   if (post_condition_) {
      std::cout << "returning success from the tick" << std::endl;
      return NodeStatus::SUCCESS;
   }
   const size_t children_count = children_nodes_.size();

   if (status() == NodeStatus::IDLE) {
      all_skipped_ = true;
   }

   setStatus(NodeStatus::RUNNING);

   while (current_child_idx_ < children_count) {
      TreeNode* current_child_node = children_nodes_[current_child_idx_];

      auto prev_status = current_child_node->status();
      const NodeStatus child_status = current_child_node->executeTick();

      // switch to RUNNING state as soon as you find an active child
      all_skipped_ &= (child_status == NodeStatus::SKIPPED);

      switch (child_status) {
         case NodeStatus::RUNNING: {
            return NodeStatus::RUNNING;
         }
         case NodeStatus::FAILURE: {
            // Reset on failure
            resetChildren();
            current_child_idx_ = 0;
            return child_status;
         }
         case NodeStatus::SUCCESS: {
            current_child_idx_++;
            // Return the execution flow if the child is async,
            // to make this interruptable.
            if (requiresWakeUp() && prev_status == NodeStatus::IDLE &&
                current_child_idx_ < children_count) {
               emitWakeUpSignal();
               return NodeStatus::RUNNING;
            }
         } break;

         case NodeStatus::SKIPPED: {
            // It was requested to skip this node
            current_child_idx_++;
         } break;

         case NodeStatus::IDLE: {
            throw LogicError("[", name(),
                             "]: A children should not return IDLE");
         }
      }  // end switch
   }     // end while loop

   // The entire while loop completed. This means that all the children returned
   // SUCCESS.
   if (current_child_idx_ == children_count) {
      resetChildren();
      current_child_idx_ = 0;
   }
   // Skip if ALL the nodes have been skipped
   return all_skipped_ ? NodeStatus::SKIPPED : onSuccess();
}

NodeStatus PostSequenceNode::onSuccess() {
   setenv(condition_.c_str(), target_.c_str(), 1);
   return NodeStatus::SUCCESS;
}

}  // namespace BT
