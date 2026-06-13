#include "bt_policy/dummy_nodes.h"
#include <stdlib.h>
#include "bt_policy/utils.h"

// This function must be implemented in the .cpp file to create
// a plugin that can be loaded at run-time
BT_REGISTER_NODES(factory) { DummyNodes::RegisterNodes(factory); }

namespace DummyNodes {

BT::NodeStatus CheckBattery() {
   std::cout << "[ Battery: OK ]" << std::endl;
   return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus CheckTemperature() {
   std::cout << "[ Temperature: OK ]" << std::endl;
   return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus SayHello() {
   std::cout << "Robot says: Hello World" << std::endl;
   return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus IsTetherEnough() {
    const char* is_tether_free = std::getenv("IS_TETHER_FREE");
    std::string false_cond = "false";
    if (is_tether_free != NULL) {
        bool result = is_tether_free == false_cond ? false : true;
        std::cout << "Tether free set to: " << (result ? "free" : "not free") << std::endl;
        return result ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
    std::cout << "Tether free not set -> defaulting to not enough" << std::endl;

    return BT::NodeStatus::FAILURE;
};

BT::NodeStatus AreAsvAndRovCloseEnough() {
    const char* Are_Asv_And_Rov_Close_Enough = std::getenv("Are_Asv_And_Rov_Close_Enough");
    std::string false_cond = "false";
    if (Are_Asv_And_Rov_Close_Enough != NULL) {
        bool result = Are_Asv_And_Rov_Close_Enough == false_cond ? false : true;
        std::cout << "Are_Asv_And_Rov_Close_Enough: " << (result ? "yes" : "no") << std::endl;
        return result ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
    std::cout << "Are_Asv_And_Rov_Close_Enough not set -> defaulting to not enough" << std::endl;

    return BT::NodeStatus::FAILURE;
};



BT::NodeStatus GripperInterface::open() {
   _opened = true;
   std::cout << "GripperInterface::open" << std::endl;
   return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus GripperInterface::close() {
   std::cout << "GripperInterface::close" << std::endl;
   _opened = false;
   return BT::NodeStatus::SUCCESS;
}

ApproachObject::~ApproachObject() {
   std::cout << "destroying " << this->name() << " that returned "
             << BT::NodeStatus::SUCCESS << std::endl;
}

BT::NodeStatus ApproachObject::tick() {
   /*
   std::cout << "ApproachObject: " << this->name() << std::endl;
   return BT::NodeStatus::SUCCESS; */
   std::cout << "I'm in emergency and this is the default name: " << std::endl;
   setenv("EMERGENCY", "true", 1);
   std::cout << "EMERGENCY = " << getenv("EMERGENCY") << std::endl;
   return BT::NodeStatus::SUCCESS;
}

/*BT::NodeStatus ApproachObject::tick() {
   std::cout << "I'm in emergency and this is the default name: " << std::endl;
   char env_em[] = "EMERGENCY=TRUE";
   putenv(env_em);
   return BT::NodeStatus::SUCCESS;
} */

BT::NodeStatus SaySomething::tick() {
   auto msg = getInput<std::string>("message");
   if (msg == "fail") {
      std::cout << "testing the failure" << std::endl;
      return BT::NodeStatus::FAILURE;
   }
   if (!msg) {
      throw BT::RuntimeError("missing required input [message]: ", msg.error());
   }

   std::cout << "Robot says: " << msg.value() << std::endl;
   return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus FreeTether::tick() {
   auto status_exp = getInput<std::string>("is_tether_free");
    
    if (!status_exp) {
        // Handle missing input error
        std::cerr << "Missing input: " << status_exp.error() << "\n";
        return BT::NodeStatus::FAILURE;
    }

    const std::string& status = status_exp.value();
    setenv("IS_TETHER_FREE", status.c_str(), 1);
    std::cout << "IS_TETHER_FREE: " << status << "\n";
    
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus IsObjectInspected::tick() {
    auto object = getInput<std::string>("object");

    if (!object) {
        // Handle missing input error
        std::cerr << "Missing input: " << object.error() << "\n";
        return BT::NodeStatus::FAILURE;
    }
    const std::string& success = "inspected";
    const std::string& object_name = object.value();

    const char* object_status = std::getenv(object_name.c_str());
    if (object_status == NULL) {
        std::cout
            << "[IsObjectInspected] this object has not been inspected yet: "
            << object_name.c_str() << std::endl;
        return BT::NodeStatus::FAILURE;
    }
    std::cout << "[IsObjectInspected] the object " << object_name.c_str()
              << " inspected: " << object_status
              << "(expected: " << success.c_str() << ")\n"
              << std::endl;

    // returning success by default everytime the object is detected
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus SetObjectInspected::tick() {
   auto object = getInput<std::string>("object");
    
    if (!object) {
        // Handle missing input error
        std::cerr << "Missing input: " << object.error() << "\n";
        return BT::NodeStatus::FAILURE;
    }
    const std::string& status = "inspected";
    const std::string& object_name = object.value();
    setenv(object_name.c_str(), status.c_str(), 1);
    std::cout << object_name << ": " << status << "\n";
    
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus TraverseDoor::tick() {
   auto status = getInput<std::string>("status");
   std::cout << "TraverseDoor: I'm a simple node just for testing. You can "
                "manually set my status to simulate what would happen in "
                "reality. If you set my status as 'fail', then I will fail!"
             << std::endl;
   if (status == "fail") {
      std::cout << "testing the failure" << std::endl;
      return BT::NodeStatus::FAILURE;
   }
   if (!status) {
      throw BT::RuntimeError("missing required input [message]: ",
                             status.error());
   }

   std::cout << "TraverseDoor status: " << status.value() << std::endl;
   return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus EmergencyCall::tick() {
   auto emergency = getInput<std::string>("emergency_call");
   if (emergency == "false") {
      std::cout << "false alarm" << std::endl;
      return BT::NodeStatus::FAILURE;
   }
   if (!emergency) {
      throw BT::RuntimeError("missing required input [emergency]: ",
                             emergency.error());
   }
   std::cout << "we have an emergency: " << emergency.value() << std::endl;
   setenv("EMERGENCY", emergency.value().data(), 1);
   return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus SaySomethingSimple(BT::TreeNode& self) {
   auto msg = self.getInput<std::string>("message");
   if (!msg) {
      throw BT::RuntimeError("missing required input [message]: ", msg.error());
   }

   std::cout << "Robot says: " << msg.value() << std::endl;
   return BT::NodeStatus::SUCCESS;
}

}  // namespace DummyNodes
