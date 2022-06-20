// holds #defines for information shared between publisher and subscriber

#ifndef PUB_SUB_CONFIG
#define PUB_SUB_CONFIG

#define BASIC_BENCH_DATA_BUF 1000U
#define BASIC_BENCH_WRITE_FOLDER "/home/ruben/june-measurements/ros1/results/basic_bench/"
#define BASIC_BENCH_JITTER_FOLDER "/home/ruben/june-measurements/ros2/results/basic_bench/activation_jitter/"
#define BASIC_BENCH_BATCH_PER_SIZE 5
#define BASIC_BENCH_PUBLISHER_EXTRA 100

// toggle for setting a transport hint in the subscriber.
#define USE_TCP_HINT
// toggle for std::chrono use. The VM seems to not support this for ROS1, need to update the compiler, but want to have the code ready.
// #define USE_CHRONO


#endif // PUB_SUB_CONFIG