#pragma once

// -------------------- ROS parameters names -------------------- //

// system
#define CONF_MONITOR_RATE "monitor_rate"
// ros topic
#define CONF_NAME_SPACE "name_space"
#define CONF_GET_STATE "topic_get_state"
#define CONF_GET_STATES "topic_get_states"
#define CONF_CHANGE_STATE "topic_change_state"
// abort action
#define CONF_ABORT "abort_action"
#define CONF_ABORT_STATE "state"
#define CONF_ABORT_TIMEOUT "timeout"
#define CONF_ABORT_TRANSITION "transition"

// -------------------- default parameters value -------------------- //

// system
#define DEFAULT_MONITOR_RATE 10 
// ros topic
#define DEFAULT_NAME_SPACE "alpha"
#define DEFAULT_TOPIC_GET_STATE "helm/get_state"
#define DEFAULT_TOPIC_GET_STATES "helm/get_states"
#define DEFAULT_TOPIC_CHANGE_STATE "helm/change_state"