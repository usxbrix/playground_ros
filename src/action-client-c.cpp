/**
 * @file rc_test_buttons.c
 * @example    rc_test_buttons
 *
 * This is a very basic test of button functionality. It simply prints to the
 * screen when a button has been pressed or released.
 **/

#include <stdio.h>
#include <signal.h>
#include <rc/button.h>
#include <rc/time.h>
#include <rc/led.h>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <playground_ros/FollowAction.h>

static int running = 0;

using namespace playground_ros;
typedef actionlib::SimpleActionClient<FollowAction> Client;

//Client ac;
static actionlib::SimpleActionClient<playground_ros::FollowAction> ac;

/* 
void doneCb(const actionlib::SimpleClientGoalState& state,
            const FollowResultConstPtr& result)
{
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
  ROS_INFO("Answer: %i", result->sequence.back());
  ros::shutdown();
}

// Called once when the goal becomes active
void activeCb()
{
  ROS_INFO("Goal just went active");
}

// Called every time feedback is received for the goal
void feedbackCb(const FollowFeedbackConstPtr& feedback)
{
  ROS_INFO("Got Feedback of length %lu", feedback->sequence.size());
}
*/

static void __on_pause_press(void)
{
	printf("Pause Pressed - Sending goal\n");
	rc_led_set(RC_LED_GREEN,1);

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
	rc_led_set(RC_LED_GREEN,0);
	return;
}

static void __on_pause_release(void)
{
	printf("Pause Released\n");
	return;
}

static void __on_mode_press(void)
{
	printf("Mode Pressed\n");
	return;
}

static void __on_mode_release(void)
{
	printf("Mode Released\n");
	return;
}

// interrupt handler to catch ctrl-c
static void __signal_handler(__attribute__ ((unused)) int dummy)
{
	running=0;
	return;
}

int main (int argc, char **argv)
{
	// initialize pause and mode buttons
	if(rc_button_init(RC_BTN_PIN_PAUSE, RC_BTN_POLARITY_NORM_HIGH,
						RC_BTN_DEBOUNCE_DEFAULT_US)){
		fprintf(stderr,"ERROR: failed to init buttons\n");
		return -1;
	}
	if(rc_button_init(RC_BTN_PIN_MODE, RC_BTN_POLARITY_NORM_HIGH,
						RC_BTN_DEBOUNCE_DEFAULT_US)){
		fprintf(stderr,"ERROR: failed to init buttons\n");
		return -1;
	}

	// set signal handler so the loop can exit cleanly
	signal(SIGINT, __signal_handler);
	running = 1;

	ros::init(argc, argv, "follow_client_cpp");

  	// create the action client
  	// true causes the client to spin its own thread
  	actionlib::SimpleActionClient<playground_ros::FollowAction> ac("object_follow_action", true);

  	ROS_INFO("Waiting for action server to start.");
  	// wait for the action server to start
  	ac.waitForServer();  // will wait for infinite time

  	ROS_INFO("Action server started, sending goal.");

	// Assign callback functions
	rc_button_set_callbacks(RC_BTN_PIN_PAUSE, __on_pause_press, __on_pause_release);
	rc_button_set_callbacks(RC_BTN_PIN_MODE, __on_mode_press, __on_mode_release);


	//toggle leds till the program state changes
	printf("Press buttons to see response\n");
	while(running)	rc_usleep(500000);

	// cleanup and exit
	rc_button_cleanup();
	return 0;
}