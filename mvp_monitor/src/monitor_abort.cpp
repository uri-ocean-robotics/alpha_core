#include "monitor_abort.hpp"

MonitorAbort::MonitorAbort(
    const ros::NodeHandle &nh,
    const ros::NodeHandle &nh_private) 
    : nh_(nh), pnh_(nh_private) 
{
    // load all the parameters
    loadParameters();

    // init
    initialize();
}

void MonitorAbort::loadParameters()
{
    // -------------------- load system parameters -------------------- //
    
    if(pnh_.hasParam(CONF_MONITOR_RATE))
    {
        pnh_.getParam(CONF_MONITOR_RATE, monitor_rate_);
    }
    else
    {
        monitor_rate_ = DEFAULT_MONITOR_RATE;
        ROS_WARN(
            "MVP Monitor - configuration [%s] not exist, use the default value: %f", 
            CONF_MONITOR_RATE, monitor_rate_);
    }    

    // -------------------- load ros parameters -------------------- //

    if(pnh_.hasParam(CONF_NAME_SPACE))
    {
        pnh_.getParam(CONF_NAME_SPACE, name_space_);
    }
    else
    {
        ROS_ERROR(
            "MVP Monitor - configuration [%s] not exist, SHUT DOWN NOW!", 
            CONF_NAME_SPACE);
        ros::shutdown();
    } 

    if(pnh_.hasParam(CONF_GET_STATE))
    {
        pnh_.getParam(CONF_GET_STATE, srv_get_state_);
        srv_get_state_ = "/" + name_space_ + "/" + srv_get_state_;
    }
    else
    {
        srv_get_state_ = "/" + name_space_ + "/" + DEFAULT_SRV_GET_STATE;
        ROS_WARN(
            "MVP Monitor - configuration [%s] not exist, use the default value: %s", 
            CONF_GET_STATE, srv_get_state_.c_str());
    }

    if(pnh_.hasParam(CONF_GET_STATES))
    {
        pnh_.getParam(CONF_GET_STATES, srv_get_states_);
        srv_get_states_ = "/" + name_space_ + "/" + srv_get_states_;
    }
    else
    {
        srv_get_states_ = "/" + name_space_ + "/" + DEFAULT_SRV_GET_STATES;
        ROS_WARN(
            "MVP Monitor - configuration [%s] not exist, use the default value: %s", 
            CONF_GET_STATES, srv_get_states_.c_str());
    }

    if(pnh_.hasParam(CONF_CHANGE_STATE))
    {
        pnh_.getParam(CONF_CHANGE_STATE, srv_change_state_);
        srv_change_state_ = "/" + name_space_ + "/" + srv_change_state_;
    }
    else
    {
        srv_change_state_ = "/" + name_space_ + "/" + DEFAULT_SRV_CHANGE_STATE;
        ROS_WARN(
            "MVP Monitor - configuration [%s] not exist, use the default value: %s", 
            CONF_CHANGE_STATE, srv_change_state_.c_str());
    }

    // -------------------- load abort action parameters -------------------- //

    XmlRpc::XmlRpcValue abort_action_list;
    if(pnh_.hasParam(CONF_ABORT))
    {
        pnh_.getParam(CONF_ABORT, abort_action_list);
    }
    else
    {
        pnh_.getParam(CONF_ABORT, abort_action_list);
        ROS_ERROR(
            "MVP Monitor - configuration [%s] not exist, SHUT DOWN NOW!", 
            CONF_ABORT);
        ros::shutdown();
    }

    ROS_ASSERT(abort_action_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

    for(int32_t i = 0 ; i < abort_action_list.size() ; i++) {
        // check if each sub-configue exits
        bool has_state = abort_action_list[i].hasMember(CONF_ABORT_STATE);
        bool has_timeout = abort_action_list[i].hasMember(CONF_ABORT_TIMEOUT);
        bool has_transition = abort_action_list[i].hasMember(CONF_ABORT_TRANSITION);

        if(! has_state || ! has_timeout || !has_transition)
        {
            ROS_ERROR(
                "MVP Monitor - No.%d of abort action miss parts:[%s: %s], [%s: %s], [%s: %s], SHUT DOWN NOW!",
                i+1,
                CONF_ABORT_STATE, has_state ? "Given" : "Not Given",
                CONF_ABORT_TIMEOUT, has_timeout ? "Given" : "Not Given",
                CONF_ABORT_TRANSITION, has_transition ? "Given" : "Not Given");

            ros::shutdown();
        }

        AbortAction action;
        std::string state = static_cast<std::string>(abort_action_list[i][CONF_ABORT_STATE]);
        action.timeout = static_cast<double>(abort_action_list[i][CONF_ABORT_TIMEOUT]);
        action.transition = static_cast<std::string>(abort_action_list[i][CONF_ABORT_TRANSITION]);
        abort_action_[state] = action;
    }
}

void MonitorAbort::verifySrv()
{
    while(!ros::service::exists(srv_get_state_, false))
    {
        ROS_WARN("MVP_Monitor - no Service [%s], either Node not started or wrong Service Name", 
            srv_get_state_.c_str());
        ros::Duration(1.0).sleep();
    }

    while(!ros::service::exists(srv_get_states_, false))
    {
        ROS_WARN("MVP_Monitor - no Service [%s], either Node not started or wrong Service Name",
            srv_get_states_.c_str());
        ros::Duration(1.0).sleep();
    }

    while(!ros::service::exists(srv_change_state_, false))
    {
        ROS_WARN("MVP_Monitor - no Service [%s], either Node not started or wrong Service Name", 
            srv_change_state_.c_str());
        ros::Duration(1.0).sleep();
    }
}

void MonitorAbort::setupRos()
{
    // setup service clinet
    clinet_get_state_ = 
        nh_.serviceClient<mvp_msgs::GetState>(srv_get_state_);    

    clinet_get_states_ = 
        nh_.serviceClient<mvp_msgs::GetStates>(srv_get_states_);    

    clinet_change_state_ = 
        nh_.serviceClient<mvp_msgs::ChangeState>(srv_change_state_);   

    // setup timer callback to time counting
    timer_ = nh_.createTimer(
        ros::Duration(1.0 / monitor_rate_), 
        &MonitorAbort::timerCallback, this);        
}

void MonitorAbort::initialize()
{
    // verify the ROS service are correct
    verifySrv();

    // setup ROS
    setupRos();

    // verify the abort action parameters are correct
    while(!verifyAbortAction())
    {
        ros::Duration(1.0).sleep();
    }

    // check the current state
    while(!getState())
    {
        ros::Duration(1.0).sleep();
    }

    // update the info
    last_state_ = curr_state_;
    time_count_ = ros::Time::now().toSec();

    // report
    ROS_INFO("MVP_Monitor: abort action is initialized");
}

bool MonitorAbort::getState()
{
    // grab the current state
    mvp_msgs::GetState srv_get_state;

    if (!clinet_get_state_.call(srv_get_state)) {
        ROS_WARN("MVP_Monitor - abort action: can not get state");
        return false;
    }

    // check if this state inside our monitorng list
    auto found = std::find_if(
        abort_action_.begin(), 
        abort_action_.end(), 
        [&](const auto& monitored) 
        { 
            return srv_get_state.response.state.name == monitored.first; 
        });

    if(found == abort_action_.end())
    {
        // this is the state we are not minotoring 
        ROS_WARN("MVP_Monitor - abort action: %s not on our monitor list", 
                  srv_get_state.response.state.name.c_str());
        return false;
    }

    // save the state
    curr_state_ = srv_get_state.response.state;

    return true;
}

bool MonitorAbort::verifyAbortAction()
{
    // grab the current state
    mvp_msgs::GetStates srv_get_states;

    // --------------------------------------------------------------------- //
    // Grab all the states
    // --------------------------------------------------------------------- // 

    if (!clinet_get_states_.call(srv_get_states)) {
        ROS_WARN("MVP_Monitor - abort action: can not get all states");
        return false;
    }    

    auto &recv_states = srv_get_states.response.states;

    // --------------------------------------------------------------------- //
    // check the states are recognized by MVP Finite State Machine
    // --------------------------------------------------------------------- //

    for(const auto& [monitored_state, action] :  abort_action_)
    {
        // check if each monitored state
        auto frame = std::find_if(
            recv_states.begin(), recv_states.end(), [&](auto &recv_state) 
            { return recv_state.name == monitored_state; });
        
        // if not, send error and shut down the node
        if(frame == recv_states.end())
        {
            // get all states
            std::string str;
            for(const auto& recv_state : recv_states)
            {
                str += "[" + recv_state.name + "] ";
            }

            // send errors
            ROS_ERROR(
                "MVP Monitor - the monitored state [%s] is not in the MVP Finite State Machine",
                monitored_state.c_str());
            ROS_ERROR(
                "MVP_Monitor - The correct states are: %s, SHUT DOWN NOW", 
                str.c_str());
            ros::shutdown();
        }
    }

    // --------------------------------------------------------------------- //
    // check the transition are recognized by MVP Finite State Machine 
    // --------------------------------------------------------------------- //

    // check if each transition meet the MVP_Mission requirements 
    for(const auto& recv_state: recv_states)
    {
        // go to next state if this is not in our monitor list
        if(abort_action_.find(recv_state.name) == abort_action_.end())
        {
            continue;
        }

        // check the transition 
        auto action = abort_action_[recv_state.name];

        auto frame = std::find_if(
            recv_state.transitions.begin(),
            recv_state.transitions.end(),
            [&](const auto& transition)
            { return transition == action.transition; });

        // if transition is not matched, exit from this node
        if(frame == recv_state.transitions.end())
        {
            // get all transitions
            std::string str;
            for(const auto& transition : recv_state.transitions)
            {
                str += "[" + transition + "] ";
            }

            // print error
            ROS_ERROR("MVP_Monitor - The monitored transition [ %s ] of state [ %s ] not meet the requirements", 
                        action.transition.c_str(), recv_state.name.c_str());
            ROS_ERROR("MVP_Monitor - The correct transitions of state [ %s ] are: %s, SHUT DOWN NOW", 
                       recv_state.name.c_str(), str.c_str());

            ros::shutdown();
        }
    }

    return true;
}

void MonitorAbort::timerCallback(const ros::TimerEvent& event)
{
    //! DEBUG:
    // ROS_INFO("hi from MVP_Monitor timercallback");

    // grab state
    if(!getState())
    {
        return;
    }
    
    // check if state changes
    if(curr_state_ != last_state_)
    {
        ROS_INFO("MVP_Monitor - new state:%s, mode:%s", 
                  curr_state_.name.c_str(), 
                  curr_state_.mode.c_str());

        // mark the last state

        last_state_ = curr_state_;
        time_count_ = ros::Time::now().toSec();
    }

    // check if counted time is reach the max
    if(ros::Time::now().toSec() - time_count_ > 
       abort_action_[curr_state_.name].timeout)
    {
        // switch the state to given param
        mvp_msgs::ChangeState srv_change_state;

        srv_change_state.request.state = 
            abort_action_[curr_state_.name].transition;
        srv_change_state.request.caller = 
            ros::this_node::getName();

        // call the srv to change state
        if (!clinet_change_state_.call(srv_change_state)) {
            ROS_WARN("MVP_Monitor - abort action: call change_state failed");
            return;
        }        
    }
}
