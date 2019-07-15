/**
 * @file /include/yocs_cmd_vel_mux/cmd_vel_mux_nodelet.hpp
 *
 * @brief Structure for the yocs_cmd_vel_mux.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/yujin_ocs/hydro/yocs_cmd_vel_mux/LICENSE
 **/
/*****************************************************************************
 ** Ifdefs
 *****************************************************************************/

#ifndef YUJIN_OCS_CMD_VEL_MUX_HPP_
#define YUJIN_OCS_CMD_VEL_MUX_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <dynamic_reconfigure/server.h>

#include "yocs_cmd_vel_mux/reloadConfig.h"
#include "yocs_cmd_vel_mux/cmd_vel_subscribers.hpp"
#include "yocs_msgs/ChangeMuxPriority.h"
#include "yocs_msgs/ResetPendingMuxPriority.h"

#include <map>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace yocs_cmd_vel_mux {

/*****************************************************************************
 ** CmdVelMux
 *****************************************************************************/

class CmdVelMuxNodelet : public nodelet::Nodelet
{
public:
  virtual void onInit();

  CmdVelMuxNodelet()
  {
    cmd_vel_subs.allowed = VACANT;
    dynamic_reconfigure_server = NULL;
  }

  ~CmdVelMuxNodelet()
  {
    if (dynamic_reconfigure_server != NULL)
      delete dynamic_reconfigure_server;
  }

protected:
  /**
   * \brief ROS Service Callback to change priority temporarily/permanently for an input velocity topic
   *
   * \param req: A reference to the request to the service
   * \param idx: A reference to the response from the service (populated in the callback)
   *
   * \return true if the requested change was successful, false otherwise
   */
  bool changeTopicPriorityServiceCb(yocs_msgs::ChangeMuxPriority::Request& req,
                                    yocs_msgs::ChangeMuxPriority::Response& res);

  /**
   * \brief ROS Service Callback to fast-forward the reset for a temporary priority change
   *        of an input velocity topic
   *
   * \param req: A reference to the request to the service
   * \param idx: A reference to the response from the service (populated in the callback)
   *
   * \return true if the requested topic is an input to the velocity multiplexer, false otherwise
   */
  bool resetPendingTopicPriorityServiceCb(yocs_msgs::ResetPendingMuxPriority::Request& req,
                                          yocs_msgs::ResetPendingMuxPriority::Response& res);

private:
  static const unsigned int VACANT       = 666666;  /**< ID for "nobody" active input; anything big is ok */
  static const unsigned int GLOBAL_TIMER = 888888;  /**< ID for the global timer functor; anything big is ok */

  CmdVelSubscribers cmd_vel_subs;              /**< Pool of cmd_vel topics subscribers */
  ros::Publisher output_topic_pub;             /**< Multiplexed command velocity topic */
  std::string    output_topic_name;            /**< Multiplexed command velocity topic name */
  ros::Publisher active_subscriber;            /**< Currently allowed cmd_vel subscriber */
  ros::ServiceServer change_priority_service;  /** < Service to change priority of a topic */
  ros::ServiceServer reset_pending_topic_priority_service;  /** < Service to change priority of a topic */

  std::map<uint8_t, ros::Timer> reset_priority_timer_map;
  ros::Timer common_timer;                     /**< No messages from any subscriber timeout */
  double common_timer_period;                  /**< No messages from any subscriber timeout period */

  /**
   * \brief A ros::Timer callback to perform inactivity checks on velocity multiplexer topics
   *
   * \param event: The ros::Timer event
   * \param idx: The index of the timer/topic
   */
  void timerCallback(const ros::TimerEvent& event, unsigned int idx);

  /**
   * \brief A ros::Timer callback to perform a input topic priority reset
   *
   * \param event: The ros::Timer event
   * \param idx: The index of the timer/topic
   * \param priority: The priority to set for the input topic
   */
  void timerCallback(const ros::TimerEvent& event, unsigned int idx, uint8_t priority);

  /**
   * \brief Velocity callbacks for input topics to the velocity multiplexer
   *
   * \param msg: The geometry_msgs::Twist message on an input topic
   * \param idx: The index of the input topic maintained by the velocity multiplexer
   */
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg, unsigned int idx);

  /*********************
  ** Dynamic Reconfigure
  **********************/
  dynamic_reconfigure::Server<yocs_cmd_vel_mux::reloadConfig> * dynamic_reconfigure_server;
  dynamic_reconfigure::Server<yocs_cmd_vel_mux::reloadConfig>::CallbackType dynamic_reconfigure_cb;

  /**
   * \brief Dynamic Reconfigure callback to change Velocity Multiplexer configurations during runtime
   *
   * \param config: A reference to the new configuration that is requested
   * \param unused_level: A bitmask containing ORed values of the bits of all parameter classes that
   *                      have been changed
   */
  void reloadConfiguration(yocs_cmd_vel_mux::reloadConfig &config, uint32_t unused_level);

  /*********************
   ** Private Classes
   **********************/
  // Functor assigned to each incoming velocity topic to bind it to cmd_vel callback
  class CmdVelFunctor
  {
  private:
    unsigned int idx;
    CmdVelMuxNodelet* node;

  public:
    CmdVelFunctor(unsigned int idx, CmdVelMuxNodelet* node) :
        idx(idx), node(node)
    {
    }

    void operator()(const geometry_msgs::Twist::ConstPtr& msg)
    {
      node->cmdVelCallback(msg, idx);
    }
  };

  // Functor assigned to each velocity messages source to bind it to timer callback
  class TimerFunctor
  {
  private:
    unsigned int idx;
    CmdVelMuxNodelet* node;
    bool change_priority;
    uint8_t priority;

  public:
    TimerFunctor(unsigned int idx,
                 CmdVelMuxNodelet* node,
                 bool change_priority = false,
                 uint8_t priority = 0) :
        idx(idx),
        node(node),
        change_priority(change_priority),
        priority(priority)
    {
    }

    void operator()(const ros::TimerEvent& event)
    {
      if (!change_priority)
        node->timerCallback(event, idx);
      else
        node->timerCallback(event, idx, priority);
    }
  };
};

} // namespace yocs_cmd_vel_mux

#endif /* YUJIN_OCS_CMD_VEL_MUX_HPP_ */
