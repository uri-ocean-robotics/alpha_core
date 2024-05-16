#include "monitor_abort.hpp"

MonitorAbort::MonitorAbort(
    const ros::NodeHandle &nh,
    const ros::NodeHandle &nh_private) 
    : nh_(nh), pnh_(nh_private) 
{
    // load all the parameters
    loadParameters();

    // setup service clinet
    clinet_get_state_ = nh_.serviceClient<mvp_msgs::GetState>(topic_get_state_);    

    clinet_get_states_ = nh_.serviceClient<mvp_msgs::GetStates>(topic_get_states_);    

    clinet_change_state_ = nh_.serviceClient<mvp_msgs::ChangeState>(topic_change_state_);   

    // init
    initialize();

    // setup timer callback to time counting
    timer_ = nh_.createTimer(ros::Duration(1.0 / monitor_rate_), &MonitorAbort::timerCallback, this);
}

void MonitorAbort::loadParameters()
{
    // load some parameters
    pnh_.param<double>("monitor_rate", monitor_rate_, 1.0);
    pnh_.param<std::string>("topic_get_state", topic_get_state_, "alpha_img/helm/get_state");
    pnh_.param<std::string>("topic_get_states", topic_get_states_, "alpha_img/helm/get_states");
    pnh_.param<std::string>("topic_change_state", topic_change_state_, "alpha_img/helm/change_state");

    // load abort actions
    XmlRpc::XmlRpcValue abort_action_list;
    pnh_.getParam(CONF_MONITOR_ABORT, abort_action_list);   

    ROS_ASSERT(abort_action_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

    for(int32_t i = 0 ; i < abort_action_list.size() ; i++) {

        AbortAction action;
        std::string state = static_cast<std::string>(abort_action_list[i][CONF_MONITOR_ABORT_STATE]);
        action.timeout = static_cast<double>(abort_action_list[i][CONF_MONITOR_ABORT_TIMEOUT]);
        action.transition = static_cast<std::string>(abort_action_list[i][CONF_MONITOR_ABORT_TRANSITION]);

        abort_action_[state] = action;
        
        //! DEBUG:
        // printf("state:%s, timeout:%f, transition:%s\n", state.c_str(), timeout, transition.c_str());
    }
}

void MonitorAbort::initialize()
{
    // check all the transitions of state are correct
    while(!getStates())
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
        ROS_ERROR("MVP_Monitor - abort action: can not get state");
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
        ROS_ERROR("MVP_Monitor - abort action: %s not on our monitor list", 
                  srv_get_state.response.state.name.c_str());
        return false;
    }

    // save the state
    curr_state_ = srv_get_state.response.state;

    return true;
}

bool MonitorAbort::getStates()
{
    // grab the current state
    mvp_msgs::GetStates srv_get_states;

    //! NOTE: this will block until srv available
    if (!clinet_get_states_.call(srv_get_states)) {
        ROS_ERROR("MVP_Monitor - abort action: can not get all states");
        return false;
    }    

    // check if each transition meet the MVP_Mission requirements 
    for(const auto& recv_state: srv_get_states.response.states)
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
            ROS_ERROR("MVP_Monitor - The correct transitions of state [ %s ] are: %s\n", 
                       recv_state.name.c_str(), str.c_str());

            std::exit(EXIT_FAILURE);
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

        // call the srv to change state
        if (!clinet_change_state_.call(srv_change_state)) {
            ROS_ERROR("MVP_Monitor - abort action: call change_state failed");
            return;
        }        
    }
}
