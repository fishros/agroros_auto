#ifndef AGROROS_BEHAVIOR_TREE__BEHAVIOR_TREE_ENGINE_HPP_
#define AGROROS_BEHAVIOR_TREE__BEHAVIOR_TREE_ENGINE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/xml_parsing.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"


namespace agroros_behavior_tree
{

/**
 * @enum agroros_behavior_tree::BtStatus
 * @brief An enum class representing BT execution status
 */
enum class BtStatus { SUCCEEDED, FAILED, CANCELED };

/**
 * @class agroros_behavior_tree::BehaviorTreeEngine
 * @brief A class to create and handle behavior trees
 */
class BehaviorTreeEngine
{
public:
  /**
   * @brief A constructor for agroros_behavior_tree::BehaviorTreeEngine
   * @param plugin_libraries vector of BT plugin library names to load
   */
  explicit BehaviorTreeEngine(const std::vector<std::string> & plugin_libraries);
  virtual ~BehaviorTreeEngine() {}

  /**
   * @brief Function to execute a BT at a specific rate
   * @param tree BT to execute
   * @param onLoop Function to execute on each iteration of BT execution
   * @param cancelRequested Function to check if cancel was requested during BT execution
   * @param loopTimeout Time period for each iteration of BT execution
   * @return agroros_behavior_tree::BtStatus Status of BT execution
   */
  BtStatus run(
    BT::Tree * tree,
    std::function<void()> onLoop,
    std::function<bool()> cancelRequested,
    std::chrono::milliseconds loopTimeout = std::chrono::milliseconds(10));

  /**
   * @brief Function to create a BT from a XML string
   * @param xml_string XML string representing BT
   * @param blackboard Blackboard for BT
   * @return BT::Tree Created behavior tree
   */
  BT::Tree createTreeFromText(
    const std::string & xml_string,
    BT::Blackboard::Ptr blackboard);

  /**
   * @brief Function to create a BT from an XML file
   * @param file_path Path to BT XML file
   * @param blackboard Blackboard for BT
   * @return BT::Tree Created behavior tree
   */
  BT::Tree createTreeFromFile(
    const std::string & file_path,
    BT::Blackboard::Ptr blackboard);

  /**
   * @brief Function to explicitly reset all BT nodes to initial state
   * @param root_node Pointer to BT root node
   */
  void haltAllActions(BT::TreeNode * root_node);

protected:
  // The factory that will be used to dynamically construct the behavior tree
  BT::BehaviorTreeFactory factory_;
};

}  // namespace agroros_behavior_tree

#endif  // AGROROS_BEHAVIOR_TREE__BEHAVIOR_TREE_ENGINE_HPP_