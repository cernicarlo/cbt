#include "bt_policy/utils.h"
#include <boost/foreach.hpp>

namespace utils {

std::string makePath(const std::string& name, std::string path) {
   path += name;
   path += ".xml";
   return path;
}

void traverseElement(
    std::optional<std::string>& bt_id, BT::BehaviorTreeFactory& factory,
    tinyxml2::XMLElement* element, const std::string& indent,
    std::map<std::string, std::vector<std::pair<std::string, std::string>>>&
        elements_storage,
    std::vector<std::string>& includes) {

   std::string key_name = element->Name();
   std::vector<std::pair<std::string, std::string>> attributes_vec;

   // Access all the attributes of the current element
   for (const tinyxml2::XMLAttribute* attribute = element->FirstAttribute();
        attribute != nullptr; attribute = attribute->Next()) {
      attributes_vec.push_back(
          std::make_pair(attribute->Name(), attribute->Value()));
   }
   if (key_name == "include") {
      std::string include_xml = attributes_vec.back().second;
      include_xml.erase(include_xml.begin(), include_xml.begin() + 2);
      include_xml.erase(include_xml.end() - 4, include_xml.end());
      includes.push_back(include_xml);
   }
   if (key_name == "BehaviorTree" && bt_id.has_value() == false) {
      bt_id = attributes_vec.back().second;
   }

   // TODO: bring outside function
   elements_storage.insert({key_name, attributes_vec});

   // Check eventual included tree
   if (bt_id.has_value() == true && !includes.empty()) {
      std::string next_xml = includes.back();
      includes.pop_back();
      next_xml = makePath(next_xml);
      createElementsFromXML(bt_id, factory, next_xml, includes,
                            elements_storage);
   };

   // Access all the child elements of the current element
   for (tinyxml2::XMLElement* childElement = element->FirstChildElement();
        childElement != nullptr;
        childElement = childElement->NextSiblingElement()) {
      // std::cout << "Child element name: " << childElement->Name() <<
      // std::endl;
      traverseElement(bt_id, factory, childElement, indent, elements_storage,
                      includes);
   }
}

void getElementsFromXML(
    const std::string& path,
    std::map<std::string, std::vector<std::string>>& context_storage,
    std::vector<std::string>& includes) {

   tinyxml2::XMLDocument xmlDoc;
   if (xmlDoc.LoadFile(path.c_str()) != tinyxml2::XML_SUCCESS) {
      std::cerr << "getElementsFromXML: Error loading file: " << path
                << std::endl;
   };

   // Access all the elements in the XML file
   tinyxml2::XMLElement* element = xmlDoc.RootElement();
   if (element == nullptr) {
      std::cerr << "Error: root node not found" << std::endl;
   }
   std::string type;  // TODO: unused (for now)
   std::string root_bt = "root";
   std::string root_scenario = "scenario";
   if (element->Name() == root_bt) {
      type = "bt";
   } else if (element->Name() == root_scenario) {
      type = "scenario";
   } else {
      type = "undefined";
   }

   traverseXML(element, context_storage, includes, type);
}

void traverseXML(
    tinyxml2::XMLElement* element,
    std::map<std::string, std::vector<std::string>>& context_storage,
    std::vector<std::string>& includes, std::string& type) {

   // TODO: make it universal
   std::string key_name = element->Name();
   std::string scenario_type = "scenario";
   if (type != scenario_type) {
      key_name += " ";
      key_name += element->FirstAttribute()->Name();
   }

   std::vector<std::string> values_vec;
   // Access all the attributes of the current element
   for (const tinyxml2::XMLAttribute* attribute = element->FirstAttribute();
        attribute != nullptr; attribute = attribute->Next()) {
      values_vec.push_back(attribute->Value());
   }
   std::string attribute = element->FirstAttribute()->Name();

   // TODO: be careful: inserting only the name
   if (attribute != "result") {
      context_storage.insert({key_name, values_vec});
   } else {

      context_storage.insert({element->Name(), values_vec});
   }
   // Access all the child elements of the current element
   for (tinyxml2::XMLElement* childElement = element->FirstChildElement();
        childElement != nullptr;
        childElement = childElement->NextSiblingElement()) {
      traverseXML(childElement, context_storage, includes, type);
   }
}

// TODO: find a more meaningful name
// Gives the result of a node according to the scenario instructions
std::optional<BT::NodeStatus> checkScenario(
    const std::string& name,
    const std::map<std::string, std::vector<std::string>>& context_storage) {
   std::pair<std::string, std::vector<std::string>> me;
   std::vector<std::string> elements;
   BOOST_FOREACH (me, context_storage) {
      if (me.first == name) {
         if (me.second.back() == "failure") {
            return BT::NodeStatus::FAILURE;
         }
         return BT::NodeStatus::SUCCESS;
      }
   }
   return std::nullopt;
}

void createElementsFromXML(
    std::optional<std::string>& bt_id, BT::BehaviorTreeFactory& factory,
    std::string& path, std::vector<std::string>& includes,
    std::map<std::string, std::vector<std::pair<std::string, std::string>>>&
        elements_storage) {
   tinyxml2::XMLDocument xmlDoc;
   if (xmlDoc.LoadFile(path.c_str()) != tinyxml2::XML_SUCCESS) {
      std::cerr << "createElementsFromXML: Error loading file: " << path
                << std::endl;
   };

   // Access all the elements in the XML file
   tinyxml2::XMLElement* rootElement = xmlDoc.RootElement();
   if (rootElement == nullptr) {
      std::cerr << "Error: root node not found" << std::endl;
   }

   traverseElement(bt_id, factory, rootElement, "     ", elements_storage,
                   includes);
}

void snakeToCamel(std::string& s) noexcept {
   bool tail = false;
   std::size_t n = 0;
   for (unsigned char c : s) {
      if (c == '-' || c == '_') {
         tail = false;
      } else if (tail) {
         s[n++] = c;
      } else {
         tail = true;
         s[n++] = std::toupper(c);
      }
   }
   s.resize(n);
}

void camelToSnake(std::string& str) noexcept {

   // Empty String
   std::string result = "";

   // Append first character(in lower case)
   // to result string
   char c = tolower(str[0]);
   result += (char(c));

   // Traverse the string from
   // ist index to last index
   for (int i = 1; i < str.length(); i++) {

      char ch = str[i];

      // Check if the character is upper case
      // then append '_' and such character
      // (in lower case) to result string
      if (isupper(ch)) {
         result = result + '_';
         result += char(tolower(ch));
      }

      // If the character is lower case then
      // add such character into result string
      else {
         result = result + ch;
      }
   }

   // return the result
   str = result;
}

}  // namespace utils
