/**
 * @file      action-client.cpp
 *
 * @brief     node to launch action when button pressed
 *            based on the examples and librobotcontrol
 *						by James Strawson
 *
 * @author    usxbrix
 * @date      Mar 2019
 *
 * @license
 * MIT License
 *
 * Copyright (c) 2019 usxbrix
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <stdio.h>
#include <signal.h>
#include <rc/button.h>
#include <rc/time.h>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <playground_ros/FollowAction.h>

static int running = 0;

using namespace playground_ros;
typedef actionlib::SimpleActionClient<FollowAction> Client;

class MyActionClient
{
public:
  MyActionClient() : ac("object_follow_action", true)
  {
    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer();
    ROS_INFO("Action server started, sending goal.");
  }

  void register_cb(void)
  {
    rc_button_set_callbacks(RC_BTN_PIN_PAUSE, this->on_pause_press, this->on_pause_release);
  }

  void on_pause_press(void)
  {
    printf("Pause Pressed - Sending goal\n");

    // send a goal to the action
    playground_ros::FollowGoal goal;
    goal.signature = 2;
    ac.sendGoal(goal);

    // wait for the action to return
    bool finished_before_timeout = ac.waitForResult(ros::Duration(10.0));

    if (finished_before_timeout)
    {
      actionlib::SimpleClientGoalState state = ac.getState();
      ROS_INFO("Action finished: %s", state.toString().c_str());
    }
    else
      ROS_INFO("Action did not finish before the time out.");

    printf("__on_pause_press done");
    //return;
  }

  void on_pause_release(void)
  {
    printf("Pause Released\n");
    //return;
  }

  static void on_mode_press(void)
  {
    printf("Mode Pressed\n");
    return;
  }

  static void on_mode_release(void)
  {
    printf("Mode Released\n");
    return;
  }

private:
  //actionlib::SimpleActionClient<playground_ros::FollowAction> ac;
  Client ac;
};

// interrupt handler to catch ctrl-c
static void __signal_handler(__attribute__((unused)) int dummy)
{
  running = 0;
  return;
}

int main (int argc, char **argv)
{
  // initialize pause and mode buttons
  if (rc_button_init(RC_BTN_PIN_PAUSE, RC_BTN_POLARITY_NORM_HIGH, RC_BTN_DEBOUNCE_DEFAULT_US))
  {
    fprintf(stderr, "ERROR: failed to init buttons\n");
    return -1;
  }
  if (rc_button_init(RC_BTN_PIN_MODE, RC_BTN_POLARITY_NORM_HIGH, RC_BTN_DEBOUNCE_DEFAULT_US))
  {
    fprintf(stderr, "ERROR: failed to init buttons\n");
    return -1;
  }

  // set signal handler so the loop can exit cleanly
  signal(SIGINT, __signal_handler);
  running = 1;

  ros::init(argc, argv, "follow_client_cpp");

  // create the action client
  // true causes the client to spin its own thread
  //actionlib::SimpleActionClient<playground_ros::FollowAction> ac("object_follow_action", true);

  // ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  // ac.waitForServer();  // will wait for infinite time

  // ROS_INFO("Action server started, sending goal.");

	MyActionClient my_ac;
  // Assign callback functions
  // rc_button_set_callbacks(RC_BTN_PIN_PAUSE, &my_ac::on_pause_press, &my_ac::on_pause_release);
  //rc_button_set_callbacks(RC_BTN_PIN_MODE, my_ac.on_mode_press, my_ac.on_mode_release);
  my_ac.register_cb();
  // toggle leds till the program state changes
  printf("Press buttons to see response\n");
  while (running)
    rc_usleep(500000);

  // cleanup and exit
  rc_button_cleanup();
  return 0;
}