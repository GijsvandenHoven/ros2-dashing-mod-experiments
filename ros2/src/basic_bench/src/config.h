// holds #defines for information shared between publisher and subscriber

#ifndef PUB_SUB_CONFIG
#define PUB_SUB_CONFIG

#define BASIC_BENCH_DATA_BUF 1000U
#define BASIC_BENCH_WRITE_FOLDER "/home/ruben/june-measurements/ros2/results/basic_bench/"
#define BASIC_BENCH_JITTER_FOLDER "/home/ruben/june-measurements/ros2/results/basic_bench/activation_jitter/"
#define BASIC_BENCH_BATCH_PER_SIZE 5
#define BASIC_BENCH_PUBLISHER_EXTRA 100

// toggle a call to sleep. currently not possible, due to member function requirements in rclcpp::context while i have no such object
// #define USE_SLEEP


#endif // PUB_SUB_CONFIG