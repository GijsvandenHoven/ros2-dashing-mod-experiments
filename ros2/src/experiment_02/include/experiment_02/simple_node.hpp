#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <thread>

#ifndef PARAMETERIZEDNODE_HPP_
#define PARAMETERIZEDNODE_HPP_

// use these with GET_PARAM and DECLARE_PARAM below.
#define LAUNCH_OFFSET_PARAM launch_offset_time_ms_
#define HOLD_CALLBACK_PARAM busy_wait_in_callback_ms_
#define MSG_SIZE_PARAM msg_size_bytes_
#define PUBLISHER_COOLDOWN_PARAM activate_rate_ms_

class ParameterizedNode : public rclcpp::Node {
    public:
        ParameterizedNode(const char * name);

    protected:
    // these should ideally be private but it does not mesh well with the macros.    
        int64_t LAUNCH_OFFSET_PARAM;
        int64_t HOLD_CALLBACK_PARAM;
        int64_t MSG_SIZE_PARAM;
        int64_t PUBLISHER_COOLDOWN_PARAM;

        void read_all_params();

        void list_all_params();
};

// consider this private
#define NAME(macro) #macro

// use this
#define GET_PARAM(var) do { \
    this->get_parameter(NAME(var), var); \
} while (0);

// and this
#define DECLARE_PARAM(who, type, init_val) do { \
    this->declare_parameter<type>( \
        NAME(who), \
        init_val \
    ); \
} while (0);

#define STREAM_PARAM(var) NAME(var) << ": " << var

// use only with millisecond-holding variables or get funny results.
#define BUSY_WAIT_MS(var) do { \
    if (var > 0) { \
        bool done = false; \
        const auto start = std::chrono::steady_clock::now(); \
        do { \
            auto now = std::chrono::steady_clock::now(); \
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - start); \
            done = duration.count() >= var; \
        } while (! done); \
    } \
} while (0);

#define SLEEP_MS(var) do { \
    if (var > 0) { \
        std::this_thread::sleep_for(std::chrono::milliseconds(var)); \
    } \
} while (0);

// usage:
// read without requesting/updating from rclcpp: PARAM_NAME
// read with requesting/updating latest: GET_PARAM(PARAM_NAME)

#endif