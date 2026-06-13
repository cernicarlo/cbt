#include "bt_policy/bt_manager.hpp"
#include <cctype>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <ament_index_cpp/get_package_share_directory.hpp>

BtManager::BtManager(const std::string &main_bt_name,
                     const rclcpp::NodeOptions &options)
    : rclcpp::Node("bt_manager_node", options),
      main_bt_name_(main_bt_name)
{
    RCLCPP_INFO(get_logger(), "BtManager initialized");
    pushBt(main_bt_name_);
    resolutionMapFromYaml();

    help_request_service_ = create_service<bt_policy::srv::HelpRequest>(
        "help_request",
        std::bind(&BtManager::handleHelpRequestServiceCallback, this,
                  std::placeholders::_1, std::placeholders::_2));
}

bool BtManager::btExistsInDb(const std::string &bt_name) const
{
    const std::string db_dir =
        ament_index_cpp::get_package_share_directory("bt_policy") + "/bt_xml/";
    return std::filesystem::exists(db_dir + bt_name + ".xml");
}

void BtManager::resolutionMapFromYaml()
{
    resolution_map_file_path_ = ament_index_cpp::get_package_share_directory("bt_policy") + "/config/resolution_map.yaml";
    // YAML::Node config = YAML::LoadFile(yaml_path);

    try
    {
        YAML::Node config = YAML::LoadFile(resolution_map_file_path_);

        for (const auto &kv : config)
        {
            const std::string key = kv.first.as<std::string>();
            const YAML::Node &entry = kv.second;
            ResolutionMapEntry resolutionEntry;
            resolutionEntry.solution = entry["solution"].as<std::string>();
            resolutionEntry.cost = entry["cost"].as<int16_t>();
            resolution_map_[key] = resolutionEntry;
            RCLCPP_INFO(get_logger(), "Loaded mapping: %s -> %s",
                        key.c_str(), resolutionEntry.solution.c_str());
        }
    }
    catch (const YAML::Exception &e)
    {
        RCLCPP_FATAL(get_logger(), "Failed to load YAML mappings: %s", e.what());
        rclcpp::shutdown();
    }

    if (resolution_map_.empty())
    {
        RCLCPP_WARN(get_logger(), "No emergency mappings found in YAML");
    }
    else
    {
        RCLCPP_INFO(get_logger(), "Loaded %zu mappings from YAML", resolution_map_.size());
    }
}

/// @brief Handles interactive BT mapping creation for unknown keys
/// @return Pair<Success status, BT name (empty if rejected)>
static std::optional<std::pair<std::string, ResolutionMapEntry>> queryUserForBTMapping(
    const std::string &key,
    const std::string &postconditionKey,
    const std::string &btPostconKey)
{
    //     std::cout << "\nUnknown BT resolving this situation: " << key
    //               << "\nDo you know the correct Behavior Tree mapping? (y/n): ";

    //     char response{};
    //     if (!(std::cin >> response) || (std::tolower(response) != 'y'))
    //     {
    //         std::cin.clear();
    //         std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    //         return {false, ""};
    //     }
    //     else
    //     {
    //     }

    //     std::cin.ignore(); // Clear newline
    //     std::cout << "Enter BT name for key '" << key << "': ";

    //     std::string bt_name;
    //     std::getline(std::cin, bt_name);
    //     return {true, bt_name};
    // }
    std::cout << "\nUnknown BT mapping for the following keys:\n";
    std::cout << "  1. " << key << "\n";
    std::cout << "  2. " << postconditionKey << "\n";
    std::cout << "  3. " << btPostconKey << "\n";
    std::cout << "Select which key to provide a mapping for (1/2/3, or 0 to skip): ";

    int selection = 0;
    if (!(std::cin >> selection) || selection < 1 || selection > 3)
    {
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        std::cerr << "Invalid selection. Please enter 1, 2, or 3.\n";
        std::cout << "Skipping BT mapping creation.\n";
        return std::nullopt;
    }
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // clear input buffer

    const std::string *selectedKey = nullptr;
    switch (selection)
    {
    case 1:
        selectedKey = &key;
        break;
    case 2:
        selectedKey = &postconditionKey;
        break;
    case 3:
        selectedKey = &btPostconKey;
        break;
    default:
        return std::nullopt;
    }

    ResolutionMapEntry entry;
    std::cout << "Enter BT mapping for key '" << *selectedKey << "': ";
    std::getline(std::cin, entry.solution);

    if (entry.solution.empty())
    {
        std::cerr << "No BT mapping provided for key '" << *selectedKey << " aborting'.\n";
        return std::nullopt;
    }

    std::cout << "Enter cost for BT mapping (default 1): ";
    int16_t costInput;
    std::string costLine;
    std::getline(std::cin, costLine);
    if (!costLine.empty())
    {
        try
        {
            costInput = std::stoi(costLine);
        }
        catch (const std::invalid_argument &)
        {
            std::cerr << "Invalid cost input. Using default value 1.\n";
            costInput = 1;
        }
    }
    else
    {
        costInput = 1;
    }
    entry.cost = costInput;
    return std::make_pair(*selectedKey, entry);
}

/// @brief Saves a new or updated entry to the YAML file
/// @param key The key for the entry
/// @param entry The ResolutionMapEntry to save
/// @param yaml_path The path to the YAML file
static void saveEntryToYaml(const std::string &key, const ResolutionMapEntry &entry, const std::string &yaml_path)
{
    YAML::Node config;
    try
    {
        config = YAML::LoadFile(yaml_path);
    }
    catch (const YAML::BadFile &)
    {
        // File does not exist, create new
        config = YAML::Node(YAML::NodeType::Map);
    }

    config[key]["solution"] = entry.solution;
    config[key]["cost"] = entry.cost;

    std::ofstream fout(yaml_path);
    fout << config;
}

void BtManager::handleHelpRequestServiceCallback(
    const std::shared_ptr<bt_policy::srv::HelpRequest::Request> request,
    const std::shared_ptr<bt_policy::srv::HelpRequest::Response> response)
{
    std::lock_guard<std::mutex> lock(stack_mutex_);
    const auto &bt_post_condition = std::string(getenv("bt_post_condition"));
    const auto &key_post_condition = request->node_post_condition;
    const auto &key = request->node_failure;

    std::unordered_map<std::string, std::pair<std::string, int16_t>> solutions;

    // Push a resolution onto the stack, applying the two paper guards:
    //   D4: the corrective BT must be present in the DB. If not, prompt the
    //       operator to deploy it and confirm (or skip, which rejects).
    //   D3: a resolution already on the stack is discarded (not pushed twice).
    auto acceptResolution = [&](const std::string &solution) {
        while (!btExistsInDb(solution))
        {
            std::cout << "\nResolution '" << solution
                      << "' is not in the BT DB.\n"
                      << "Deploy its XML, then confirm: (y) deployed / (s) skip: ";
            char choice{};
            if (!(std::cin >> choice))
            {
                std::cin.clear();
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                continue;
            }
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            if (std::tolower(choice) == 's')
            {
                response->is_request_accepted = false;
                RCLCPP_WARN(get_logger(),
                            "Resolution '%s' not deployed; rejecting request",
                            solution.c_str());
                return;
            }
        }

        if (stack_contents_.count(solution) != 0)
        {
            RCLCPP_WARN(get_logger(),
                        "Resolution '%s' already on the stack; discarding",
                        solution.c_str());
            response->is_request_accepted = true;
            return;
        }

        bt_stack_.push(solution);
        stack_contents_.insert(solution);
        corrective_requester_[solution] = key_post_condition;
        response->is_request_accepted = true;
    };

    if (auto it = resolution_map_.find(bt_post_condition); it != resolution_map_.end())
    {
        solutions[bt_post_condition] = {it->second.solution, it->second.cost};
        RCLCPP_INFO(get_logger(), "bt_post_condition Mapped BT pushed: %s->%s w cost %d",
                    bt_post_condition.c_str(), it->second.solution.c_str(), it->second.cost);
    }

    if (auto it = resolution_map_.find(key_post_condition); it != resolution_map_.end())
    {
        solutions[key_post_condition] = {it->second.solution, it->second.cost};
        RCLCPP_INFO(get_logger(), "key_post_condition Mapped BT pushed: %s->%s w cost %d",
                    key_post_condition.c_str(), it->second.solution.c_str(), it->second.cost);
    }

    if (auto it = resolution_map_.find(key); it != resolution_map_.end())
    {
        solutions[key] = {it->second.solution, it->second.cost};
        RCLCPP_INFO(get_logger(), "key Mapped BT pushed: %s->%s w cost %d",
                    key.c_str(), it->second.solution.c_str(), it->second.cost);
    }

    if (!solutions.empty())
    {
        std::vector<std::string> min_keys;
        min_keys.reserve(solutions.size());
        int16_t min_value = std::numeric_limits<int16_t>::max();
        for (const auto &pair : solutions)
        {
            if (pair.second.second < min_value)
            {
                min_value = pair.second.second;
                min_keys.clear();
                min_keys.push_back(pair.first);
            }
            else if (pair.second.second == min_value)
            {
                min_keys.push_back(pair.first);
            }
        }

        std::string selected_key;
        if (std::find(min_keys.begin(), min_keys.end(), bt_post_condition) != min_keys.end())
            selected_key = bt_post_condition;
        else if (std::find(min_keys.begin(), min_keys.end(), key_post_condition) != min_keys.end())
            selected_key = key_post_condition;
        else if (std::find(min_keys.begin(), min_keys.end(), key) != min_keys.end())
            selected_key = key;
        else
            selected_key = min_keys.front(); // fallback, should not happen

        acceptResolution(solutions.at(selected_key).first);
    }
    else
    {
        const auto user_entry = queryUserForBTMapping(key, key_post_condition, bt_post_condition);

        if (user_entry && !user_entry->second.solution.empty())
        {
            resolution_map_[user_entry->first] = user_entry->second;
            saveEntryToYaml(user_entry->first, user_entry->second, resolution_map_file_path_);
            RCLCPP_INFO(get_logger(), "User-defined mapping: %s->%s",
                        user_entry->first.c_str(), user_entry->second.solution.c_str());
            acceptResolution(user_entry->second.solution);
        }
        else
        {
            response->is_request_accepted = false;
            RCLCPP_WARN(get_logger(), "No mapping provided for %s, %s or %s", key.c_str(), key_post_condition.c_str(), bt_post_condition.c_str());
        }
    }
}

void BtManager::notifyCorrectiveOutcome(const std::string &corrective_bt, bool success)
{
    std::lock_guard<std::mutex> lock(stack_mutex_);
    auto it = corrective_requester_.find(corrective_bt);
    if (it == corrective_requester_.end())
    {
        return; // not a tracked corrective (e.g. the main BT) — nothing to do
    }

    if (!success)
    {
        const std::string flag = it->second + "_recovery_failed";
        setenv(flag.c_str(), "true", 1);
        RCLCPP_WARN(get_logger(),
                    "Corrective BT '%s' FAILED; flagging requester '%s' to fail",
                    corrective_bt.c_str(), it->second.c_str());
    }
    corrective_requester_.erase(it);
}

void BtManager::pushBt(const std::string &bt_name)
{
    std::lock_guard<std::mutex> lock(stack_mutex_);
    bt_stack_.push(bt_name);
    stack_contents_.insert(bt_name);
    RCLCPP_INFO(get_logger(), "Pushed BT '%s' to stack", bt_name.c_str());
}

std::string BtManager::getStackTop()
{
    std::lock_guard<std::mutex> lock(stack_mutex_);
    return bt_stack_.top();
}

std::string BtManager::getStackTopAndPop()
{
    std::lock_guard<std::mutex> lock(stack_mutex_);
    std::string top = bt_stack_.top();
    bt_stack_.pop();
    stack_contents_.erase(top);
    return top;
}

bool BtManager::isStackEmpty()
{
    std::lock_guard<std::mutex> lock(stack_mutex_);
    return bt_stack_.empty();
}

void BtManager::popStack()
{
    std::lock_guard<std::mutex> lock(stack_mutex_);
    if (!bt_stack_.empty())
    {
        stack_contents_.erase(bt_stack_.top());
        bt_stack_.pop();
    }
}
