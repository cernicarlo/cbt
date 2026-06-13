#ifndef NODE_UTILS_H
#define NODE_UTILS_H

#include <rclcpp/rclcpp.hpp>
#include <map>
#include <string>
#include <vector>
#include "behaviortree_cpp/bt_factory.h"

namespace nodes_utils
{

    // void nodeRegistrator(
    //     BT::BehaviorTreeFactory& factory,
    //     const std::map<std::string,
    //                    std::vector<std::pair<std::string, std::string>>>&
    //         elements_storage,
    //         rclcpp::Node::SharedPtr node, std::atomic<bool>& is_screen_busy);

    std::string registerFactoryBT(std::string &path,
                                  BT::BehaviorTreeFactory &factory,
                                  rclcpp::Node::SharedPtr node);

    template <typename NodeType>
    void registerCustomNode(BT::BehaviorTreeFactory &factory,
                            const std::string &registration_id, rclcpp::Node::SharedPtr node)
    {
        // Use the registration_id directly as the type identifier
        BT::NodeBuilder builder = [node](
                                      const std::string &name,
                                      const BT::NodeConfig &config)
        {
            return std::make_unique<NodeType>(node, name, config);
        };

        // Assuming you directly use the registration_id as the manifest type
        factory.registerBuilder<NodeType>(registration_id, builder);
    }

} // namespace nodes_utils

#endif // NODE_UTILS_H
