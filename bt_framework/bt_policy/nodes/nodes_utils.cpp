#include "bt_policy/nodes_utils.h"
#include <boost/foreach.hpp>
#include <filesystem>

#include "bt_policy/post_cond_control_nodes.h"
#include "bt_policy/utils.h"


// Nodes to register
// #include "bt_policy/crossdoor_nodes.h"
// #include "bt_policy/dummy_nodes.h"


// #include "bt_policy/clean_structure_true_nodes.h"
// #include "bt_policy/handle_current_true_nodes.h"
// #include "bt_policy/is_free_parking_area.h"
// #include "bt_policy/map_sea_floor_emergency_nodes.h"
// #include "bt_policy/movebase_node.h"
// #include "bt_policy/oil_structure_inspection_true_nodes.h"
// #include "bt_policy/iauv_girona1000_survey_nodes.h"
// #include "bt_policy/park_area_node_inheritance.h"



namespace nodes_utils {

std::string registerFactoryBT(std::string& path,
    BT::BehaviorTreeFactory& factory,
    rclcpp::Node::SharedPtr node,
    std::atomic<bool>& is_screen_busy) {
    std::optional<std::string> bt_id;
    std::cout << "registering factory from BT @ " << path << std::endl;
    std::vector<std::string> includes;
    std::map<std::string, std::vector<std::pair<std::string, std::string>>>
    elements_storage;
    utils::createElementsFromXML(bt_id, factory, path, includes,
        elements_storage);
    // nodes_utils::nodeRegistrator(factory, elements_storage, node, is_screen_busy);
    // Parse the XML file and create a tree from it
    factory.registerBehaviorTreeFromFile(path);

    return bt_id.value();
};

/*
void nodeRegistrator(
    BT::BehaviorTreeFactory& factory,
    const std::map<std::string,
                   std::vector<std::pair<std::string, std::string>>>&
        elements_storage,
    rclcpp::Node::SharedPtr node, std::atomic<bool>& is_screen_busy) {
   // TODO: find a way to store, uniquely, keys/values
   std::pair<std::string, std::vector<std::pair<std::string, std::string>>> me;
   std::vector<std::string> elements;
   BOOST_FOREACH (me, elements_storage) {
      elements.push_back(me.first);
   }

   // check there are no double elements
   sort(elements.begin(), elements.end());
   elements.erase(unique(elements.begin(), elements.end()), elements.end());

   if (std::find(elements.begin(), elements.end(), "MoveBase") !=
       elements.end()) {
      factory.registerNodeType<MoveBaseAction>("MoveBase");
   }
   if (std::find(elements.begin(), elements.end(), "SaySomething") !=
       elements.end()) {
      factory.registerNodeType<DummyNodes::SaySomething>("SaySomething");
   }

   if (std::find(elements.begin(), elements.end(), "ParkAreaAction") !=
       elements.end()) {
      registerCustomNode<ParkAreaActionInheritance>(factory, "ParkAreaAction",
                                                    node, is_screen_busy);
   }
   if (std::find(elements.begin(), elements.end(), "IsFreeParkingArea") !=
       elements.end()) {
      factory.registerNodeType<IsFreeParkingArea>("IsFreeParkingArea");
   }

   // Custom Control Nodes
   if (std::find(elements.begin(), elements.end(), "PostFallback") !=
       elements.end()) {
      factory.registerNodeType<BT::PostFallbackNode>("PostFallback");
   }

   if (std::find(elements.begin(), elements.end(), "PostSequence") !=
       elements.end()) {
      factory.registerNodeType<BT::PostSequenceNode>("PostSequence");
   }

   // seafloor mapping nodes
   if (std::find(elements.begin(), elements.end(),
                 "NavigateToAreaOfInterest") != elements.end()) {
      registerCustomNode<SeaFloorMapping::NavigateToAreaOfInterest>(
          factory, "NavigateToAreaOfInterest", node, is_screen_busy);
   }

   if (std::find(elements.begin(), elements.end(), "isAUVCloseToArea") !=
       elements.end()) {
      factory.registerSimpleCondition(
          "isAUVCloseToArea", std::bind(SeaFloorMapping::isAUVCloseToArea));
   }

   if (std::find(elements.begin(), elements.end(), "MapSeaFloor") !=
       elements.end()) {
      registerCustomNode<SeaFloorMapping::MapSeaFloor>(factory, "MapSeaFloor",
                                                       node, is_screen_busy);
   }

   if (std::find(elements.begin(), elements.end(),
                 "IdentifyPointsOfInterest") != elements.end()) {
      registerCustomNode<SeaFloorMapping::IdentifyPointsOfInterest>(
          factory, "IdentifyPointsOfInterest", node, is_screen_busy);
   }

   if (std::find(elements.begin(), elements.end(),
                 "CollectEnvironmentalSamples") != elements.end()) {
      registerCustomNode<SeaFloorMapping::CollectEnvironmentalSamples>(
          factory, "CollectEnvironmentalSamples", node, is_screen_busy);
   }

   // utilize optical imaging emergency nodes
   if (std::find(elements.begin(), elements.end(),
                 "IncreaseImagingContrastAndFiltering") != elements.end()) {
      registerCustomNode<
          UtilizeOpticalImaging::IncreaseImagingContrastAndFiltering>(
          factory, "IncreaseImagingContrastAndFiltering", node, is_screen_busy);
   }

   if (std::find(elements.begin(), elements.end(),
                 "ActivateAcousticParticleDispersal") != elements.end()) {
      registerCustomNode<
          UtilizeOpticalImaging::ActivateAcousticParticleDispersal>(
          factory, "ActivateAcousticParticleDispersal", node, is_screen_busy);
   }

   if (std::find(elements.begin(), elements.end(), "MarkAreaAndReturnLater") !=
       elements.end()) {
      registerCustomNode<UtilizeOpticalImaging::MarkAreaAndReturnLater>(
          factory, "MarkAreaAndReturnLater", node, is_screen_busy);
   }

   // map sea floor emergency nodes
   if (std::find(elements.begin(), elements.end(), "AdjustSonarSettings") !=
       elements.end()) {
      registerCustomNode<MapSeaFloorEmergency::AdjustSonarSettings>(
          factory, "AdjustSonarSettings", node, is_screen_busy);
   }

   if (std::find(elements.begin(), elements.end(), "UtilizeOpticalImaging") !=
       elements.end()) {
      registerCustomNode<MapSeaFloorEmergency::UtilizeOpticalImaging>(
          factory, "UtilizeOpticalImaging", node, is_screen_busy);
   }

// iauv girona1000 survey

    if (std::find(elements.begin(), elements.end(), "PathRequest") !=
        elements.end()) {
        registerCustomNode<IauvGirona1000Survey::PathRequest>(
          factory, "PathRequest", node, is_screen_busy);
    }

    if (std::find(elements.begin(), elements.end(), "isPathClear") !=
       elements.end()) {
      factory.registerSimpleCondition(
          "isPathClear",
          std::bind(IauvGirona1000Survey::isPathClear));
   }

   if (std::find(elements.begin(), elements.end(), "Inspect") !=
       elements.end()) {
      registerCustomNode<IauvGirona1000Survey::Inspect>(
          factory, "Inspect", node, is_screen_busy);
   }


// oil structure inspection true nodes
   if (std::find(elements.begin(), elements.end(), "isIspectionDone") !=
       elements.end()) {
      factory.registerSimpleCondition(
          "isIspectionDone",
          std::bind(OilStructureInspectionTrue::isIspectionDone));
   }

   if (std::find(elements.begin(), elements.end(), "Deploy") !=
       elements.end()) {
      registerCustomNode<OilStructureInspectionTrue::Deploy>(
          factory, "Deploy", node, is_screen_busy);
   }
   if (std::find(elements.begin(), elements.end(), "PerformSurvey") !=
       elements.end()) {
      registerCustomNode<OilStructureInspectionTrue::PerformSurvey>(
          factory, "PerformSurvey", node, is_screen_busy);
   }
   if (std::find(elements.begin(), elements.end(), "IdentifyRepairNeed") !=
       elements.end()) {
      registerCustomNode<OilStructureInspectionTrue::IdentifyRepairNeed>(
          factory, "IdentifyRepairNeed", node, is_screen_busy);
   }
   if (std::find(elements.begin(), elements.end(), "MarkRepairPoints") !=
       elements.end()) {
      registerCustomNode<OilStructureInspectionTrue::MarkRepairPoints>(
          factory, "MarkRepairPoints", node, is_screen_busy);
   }
   if (std::find(elements.begin(), elements.end(), "NavigateToUploadPoint") !=
       elements.end()) {
      registerCustomNode<OilStructureInspectionTrue::NavigateToUploadPoint>(
          factory, "NavigateToUploadPoint", node, is_screen_busy);
   }
   if (std::find(elements.begin(), elements.end(), "UploadData") !=
       elements.end()) {
      registerCustomNode<OilStructureInspectionTrue::UploadData>(
          factory, "UploadData", node, is_screen_busy);
   }
   if (std::find(elements.begin(), elements.end(), "GoToDock") !=
       elements.end()) {
      registerCustomNode<OilStructureInspectionTrue::GoToDock>(
          factory, "GoToDock", node, is_screen_busy);
   }
   if (std::find(elements.begin(), elements.end(), "Recharge") !=
       elements.end()) {
      registerCustomNode<OilStructureInspectionTrue::Recharge>(
          factory, "Recharge", node, is_screen_busy);
   }
   if (std::find(elements.begin(), elements.end(), "SystemCheck") !=
       elements.end()) {
      registerCustomNode<OilStructureInspectionTrue::SystemCheck>(
          factory, "SystemCheck", node, is_screen_busy);
   }
   if (std::find(elements.begin(), elements.end(), "AreDataUploaded") !=
       elements.end()) {
      factory.registerSimpleCondition(
          "AreDataUploaded",
          std::bind(OilStructureInspectionTrue::AreDataUploaded));
   }

// clean targets nodes
   if (std::find(elements.begin(), elements.end(), "IdentifyCleaningTargets") !=
       elements.end()) {
      registerCustomNode<CleanStructureTrue::IdentifyCleaningTargets>(
          factory, "IdentifyCleaningTargets", node, is_screen_busy);
   }
   if (std::find(elements.begin(), elements.end(), "ApproachTarget") !=
       elements.end()) {
      registerCustomNode<CleanStructureTrue::ApproachTarget>(
          factory, "ApproachTarget", node, is_screen_busy);
   }
   if (std::find(elements.begin(), elements.end(), "CleanTarget") !=
       elements.end()) {
      registerCustomNode<CleanStructureTrue::CleanTarget>(
          factory, "CleanTarget", node, is_screen_busy);
   }

   // handle current nodes
   if (std::find(elements.begin(), elements.end(), "IsAUVStable") !=
       elements.end()) {
      factory.registerSimpleCondition(
          "IsAUVStable", std::bind(HandleCurrentTrue::IsAUVStable));
   }

   if (std::find(elements.begin(), elements.end(),
                 "ActivateCurrentCompensation") != elements.end()) {
      registerCustomNode<HandleCurrentTrue::ActivateCurrentCompensation>(
          factory, "ActivateCurrentCompensation", node, is_screen_busy);
   }
   if (std::find(elements.begin(), elements.end(), "RecordData") !=
       elements.end()) {
      registerCustomNode<HandleCurrentTrue::RecordData>(factory, "RecordData",
                                                        node, is_screen_busy);
   }
   if (std::find(elements.begin(), elements.end(), "SignalForHelp") !=
       elements.end()) {
      registerCustomNode<HandleCurrentTrue::SignalForHelp>(
          factory, "SignalForHelp", node, is_screen_busy);
   }
   if (std::find(elements.begin(), elements.end(), "DataBackup") !=
       elements.end()) {
      registerCustomNode<HandleCurrentTrue::DataBackup>(factory, "DataBackup",
                                                        node, is_screen_busy);
   }
   if (std::find(elements.begin(), elements.end(), "AscentToSurface") !=
       elements.end()) {
      registerCustomNode<HandleCurrentTrue::AscentToSurface>(
          factory, "AscentToSurface", node, is_screen_busy);
   }
   if (std::find(elements.begin(), elements.end(),
                 "WaitForRescueOrInstruction") != elements.end()) {
      registerCustomNode<HandleCurrentTrue::WaitForRescueOrInstruction>(
          factory, "WaitForRescueOrInstruction", node, is_screen_busy);
   }
}
*/

}  // namespace nodes_utils
