// holds #defines for information shared between publisher and subscriber

#ifndef PUB_SUB_CONFIG
#define PUB_SUB_CONFIG

#define BASIC_BENCH_DATA_BUF 1000U
#define BASIC_BENCH_WRITE_FOLDER "/home/ruben_jetson/gijs/june-measurements/ros2/results/basic_bench/"
#define BASIC_BENCH_JITTER_FOLDER "/home/ruben_jetson/gijs/june-measurements/ros2/results/basic_bench/activation_jitter/"
#define BASIC_BENCH_BATCH_PER_SIZE 5
#define BASIC_BENCH_PUBLISHER_EXTRA 100

// this is a convenience macro to have logging the same for me between ROS versions.
#define ROS_INFO(_STR, ...) RCLCPP_INFO(this->get_logger(), _STR, ##__VA_ARGS__)


#endif // PUB_SUB_CONFIG
