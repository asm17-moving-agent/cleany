#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/xml_parsing.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
namespace py = pybind11;

// Python start/poll/cancel must return promptly. ROS waits live in a worker.
class Operation final : public BT::StatefulActionNode {
public:
  Operation(const std::string& name, const BT::NodeConfig& config,
            py::object backend, std::string id)
      : StatefulActionNode(name, config), backend_(std::move(backend)), id_(std::move(id)) {}
  static BT::PortsList providedPorts() {
    return {BT::InputPort<std::string>("execution_id")};
  }
  BT::NodeStatus onStart() override {
    auto execution = getInput<std::string>("execution_id");
    if (!execution) { throw BT::RuntimeError(execution.error()); }
    operation_ = backend_.attr("start")(id_, execution.value()).cast<std::string>();
    return BT::NodeStatus::RUNNING;
  }
  BT::NodeStatus onRunning() override {
    const auto result = backend_.attr("poll")(operation_).cast<std::string>();
    if (result == "RUNNING") { return BT::NodeStatus::RUNNING; }
    if (result == "SUCCESS") { return BT::NodeStatus::SUCCESS; }
    if (result == "FAILURE") { return BT::NodeStatus::FAILURE; }
    throw BT::RuntimeError("Invalid operation status: ", result);
  }
  void onHalted() override { backend_.attr("cancel")(operation_); }
private:
  py::object backend_;
  std::string id_, operation_;
};

class Runner {
public:
  Runner(const std::string& xml, py::object backend, const std::string& execution) {
    const std::vector<std::string> ids = {
      "ValidateGoal", "PrepareTarget", "ReconstructTarget", "GenerateGrasp", "SelectArmAndPath",
      "MoveToPregrasp", "ApproachObject", "GraspObject", "ConfirmGrasp", "LiftObject", "ConfirmHeld",
      "CarryObject", "CheckPlacementTarget", "OpenGripperAtDestination", "ConfirmRelease", "ReturnArm",
      "VerifyPlacedObject", "FinalizeSuccess", "StopAndAssess", "FinalizeFailure"};
    for (const auto& id : ids) {
      factory_.registerBuilder<Operation>(id, [backend, id](const std::string& name, const BT::NodeConfig& cfg) {
        return std::make_unique<Operation>(name, cfg, backend, id);
      });
    }
    auto board = BT::Blackboard::create();
    board->set("execution_id", execution);
    tree_ = factory_.createTreeFromFile(xml, board);
    tree_.applyVisitor([this](BT::TreeNode* node) {
      subscriptions_.push_back(node->subscribeToStatusChange(
        [this](BT::TimePoint, const BT::TreeNode& n, BT::NodeStatus previous, BT::NodeStatus current) {
          py::dict event;
          event["uid"] = n.UID(); event["node_id"] = n.registrationName();
          event["previous"] = BT::toStr(previous); event["status"] = BT::toStr(current);
          transitions_.append(event);
        }));
    });
  }
  std::string tick() { return BT::toStr(tree_.tickExactlyOnce()); }
  void halt() { tree_.haltTree(); }
  py::dict snapshot() const {
    py::dict result;
    py::list nodes;
    tree_.applyVisitor([&nodes](const BT::TreeNode* node) {
      py::dict item;
      item["uid"] = node->UID(); item["node_id"] = node->registrationName();
      item["name"] = node->name(); item["status"] = BT::toStr(node->status());
      nodes.append(item);
    });
    result["nodes"] = nodes;
    result["xml"] = BT::WriteTreeToXML(tree_, true, true);
    return result;
  }
  py::list drain_transitions() { py::list out = transitions_; transitions_ = py::list(); return out; }
private:
  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;
  std::vector<BT::TreeNode::StatusChangeSubscriber> subscriptions_;
  py::list transitions_;
};
PYBIND11_MODULE(_bt_runner, m) {
  py::class_<Runner>(m, "Runner")
    .def(py::init<const std::string&, py::object, const std::string&>())
    .def("tick", &Runner::tick).def("halt", &Runner::halt)
    .def("snapshot", &Runner::snapshot).def("drain_transitions", &Runner::drain_transitions);
}
