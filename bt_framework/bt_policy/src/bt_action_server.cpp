#include <actionlib/server/simple_action_server.h>
#include <bt_policy/BtAction.h>
#include <rclcpp/rclcpp.hpp>
#include "bt_policy/utils.h"

class BtActionServer {
  protected:
   rclcpp::Node::SharedPtr node_;
   actionlib::SimpleActionServer<bt_policy::BtAction> as_;
   std::string action_name_;
   bt_policy::BtFeedback feedback_;
   bt_policy::BtResult result_;

  public:
   BtActionServer(const std::string& name)
       : as_(nh_, name, boost::bind(&BtActionServer::executeCB, this, _1),
             false) {
      as_.start();
   }

   ~BtActionServer(void) {}

   void executeCB(const bt_policy::BtGoalConstPtr& goal) {
      // Action logic here
      RCLCPP_INFO(node_->get_logger(),"Received goal with name: %s, intention: %s, goal: %s",
               goal->name.c_str(), goal->intention.c_str(), goal->goal.c_str());

      // // do the stuff
      std::string feedback = "Processing...";
      feedback_.feedback = feedback;
      as_.publishFeedback(feedback_);
      ros::Duration(1.0).sleep();

      // Example of setting the result
      result_.success = setStateFromScenario(goal->name.c_str());
      as_.setSucceeded(result_);
   }

   bool setStateFromScenario(const std::string& name_) {
      // from the scneario path, take the entries in scenario
      std::filesystem::path absolutePath = __FILE__;
      std::filesystem::path relative_path_fs = absolutePath.parent_path().parent_path();
      std::string relative_path = relative_path_fs.string();
      std::string scenario_path = relative_path + "/scenario/";
      std::string path =
          utils::makePath("scenario", scenario_path);
      std::map<std::string, std::vector<std::string>> context_storage;
      std::vector<std::string> includes;
      utils::getElementsFromXML(path, context_storage, includes);

      std::optional<BT::NodeStatus> result;
      // checking if in scenario has been decided the failure
      result = utils::checkScenario(name_, context_storage);
      if (result.has_value()) {
         if (result.value() == BT::NodeStatus::FAILURE) {
            return false;
         } else {
            // TODO: for now, we make it directly successful, but it should go
            // first on running and then success
            return true;
         }
      }
      // TODO: for now, we make it directly successful, but it should go first
      // on running and then success
      else {
         return true;
      }
   }
};

int main(int argc, char** argv) {
   ros::init(argc, argv, "bt_action_server");
   BtActionServer server("Bt");
   ros::spin();
   return 0;
}
