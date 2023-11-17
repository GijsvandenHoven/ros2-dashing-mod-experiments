#include "experiment_02/simple_node.hpp"

#include <sstream>
#include <chrono>

/**
 * This basic node inserts code for initializing some parameters
 * from the command line or a launch file.
 * 
 * This allows more flexible re-use of nodes in experiments.
 * 
 * The constructor initializes all params, 
 * such that users can depend on the params after the initializer list.
 */

ParameterizedNode::ParameterizedNode(const char * name) : Node(name) {

    DECLARE_PARAM(LAUNCH_OFFSET_PARAM, int64_t, 0)
    DECLARE_PARAM(HOLD_CALLBACK_PARAM, int64_t, 0)
    DECLARE_PARAM(MSG_SIZE_PARAM, int64_t, 256)
    DECLARE_PARAM(PUBLISHER_COOLDOWN_PARAM, int64_t, 50)

    read_all_params();
    list_all_params();

    SLEEP_MS(LAUNCH_OFFSET_PARAM)
}

void ParameterizedNode::read_all_params() {
    GET_PARAM(LAUNCH_OFFSET_PARAM)
    GET_PARAM(HOLD_CALLBACK_PARAM)
    GET_PARAM(MSG_SIZE_PARAM)
    GET_PARAM(PUBLISHER_COOLDOWN_PARAM)
}

// this splits up the printout when using multiple nodes from a launch file
// because the newline gives others a chance to butt in.
// #define USE_NEW_LINE

void ParameterizedNode::list_all_params() {
    #ifdef USE_NEW_LINE
    const char * sep = "\n\t";
    #else
    const char * sep = "\t";
    #endif
    std::stringstream ss;
    ss
        << ""       << this->get_fully_qualified_name() <<" is configured with:"
        << sep      << STREAM_PARAM(LAUNCH_OFFSET_PARAM)
        << sep      << STREAM_PARAM(HOLD_CALLBACK_PARAM)
        << sep      << STREAM_PARAM(MSG_SIZE_PARAM)
        << sep      << STREAM_PARAM(PUBLISHER_COOLDOWN_PARAM)
        << "\n";

        RCLCPP_INFO(this->get_logger(), ss.str());
}

#undef USE_NEW_LINE