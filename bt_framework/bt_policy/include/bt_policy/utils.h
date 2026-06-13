#pragma once

#include <string.h>
#include <vector>
#include "behaviortree_cpp/bt_factory.h"
#include "bt_policy/tinyxml2.h"

namespace utils {

void camelToSnake(std::string& str) noexcept;
void snakeToCamel(std::string& s) noexcept;

std::string makePath(const std::string& name,
                     std::string path = "/home/carlo/ros_ws/src/bt_policy/bt_xml/");

void traverseElement(
    std::optional<std::string>& bt_id, BT::BehaviorTreeFactory& factory,
    tinyxml2::XMLElement* element, const std::string& indent,
    std::map<std::string, std::vector<std::pair<std::string, std::string>>>&
        elements_storage,
    std::vector<std::string>& includes);

void createElementsFromXML(
    std::optional<std::string>& bt_id, BT::BehaviorTreeFactory& factory,
    std::string& path, std::vector<std::string>& includes,
    std::map<std::string, std::vector<std::pair<std::string, std::string>>>&
        elements_storage);

std::optional<BT::NodeStatus> checkScenario(
    const std::string& name,
    const std::map<std::string, std::vector<std::string>>& context_storage);

void getElementsFromXML(
    const std::string& path,
    std::map<std::string, std::vector<std::string>>& context_storage,
    std::vector<std::string>& includes);

void traverseXML(
    tinyxml2::XMLElement* element,
    std::map<std::string, std::vector<std::string>>& context_storage,
    std::vector<std::string>& includes, std::string& type);
};  // namespace utils