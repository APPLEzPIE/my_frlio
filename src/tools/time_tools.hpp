#pragma once
#include <glog/logging.h>
#include <chrono>
#include <fstream>
#include <map>
#include <numeric>
#include <string>
#include <Eigen/Core>
#include <iostream>

/**
 * @brief 运行时计时与统计工具。
 *
 * 支持按函数名记录耗时、打印最近一次耗时明细、
 * 并将统计结果导出为日志文件。
 */
/// timer
class Timer {
public:
    struct Time_Recorder {
        Time_Recorder() = default;
        Time_Recorder(const std::string &name, double time_usage)
        {
            func_name_ = name;
            time_usage_in_ms_.emplace_back(time_usage);
        }
        std::string func_name_;
        std::vector<double> time_usage_in_ms_;
    };

    template <class F> static void evaluate(const std::string &func_name, F &&func)
    {
        auto t1 = std::chrono::high_resolution_clock::now();
        std::forward<F>(func)();
        auto t2 = std::chrono::high_resolution_clock::now();
        auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000;

        if (records_.find(func_name) != records_.end()) {
            records_[func_name].time_usage_in_ms_.emplace_back(time_used);
        } else {
            records_.insert({ func_name, Time_Recorder(func_name, time_used) });
        }
        records_tmp_.emplace_back(func_name, time_used);
    }

    static void print_recent_msg()
    {
        std::cout << ">>> ===== Printing run time =====" << std::endl;
        for (auto item : records_tmp_) {
            std::cout << "> [ " << item.first << " ] time usage: " << item.second << " ms." << std::endl;
        }
        std::cout << ">>> ===== Printing run time end =====" << std::endl;
    }

    static void reset()
    {
        records_tmp_.clear();
    }

    static void print_all_msg()
    {
        LOG(INFO) << ">>> ===== Printing run time =====";
        for (const auto &r : records_) {
            LOG(INFO) << "> [ " << r.first
                      << " ] average time usage: " << std::accumulate(r.second.time_usage_in_ms_.begin(), r.second.time_usage_in_ms_.end(), 0.0) / double(r.second.time_usage_in_ms_.size())
                      << " ms , called times: " << r.second.time_usage_in_ms_.size();
        }
        LOG(INFO) << ">>> ===== Printing run time end =====";
    }

    /// dump to a log file
    static void dump_into_file(const std::string &file_name)
    {
        std::ofstream ofs(file_name, std::ios::out);
        if (!ofs.is_open()) {
            LOG(ERROR) << "Failed to open file: " << file_name;
            return;
        } else {
            LOG(INFO) << "Dump Time Records into file: " << file_name;
        }

        ofs << ">>> ===== Printing run time =====" << std::endl;
        for (const auto &r : records_) {
            ofs << "> [ " << r.first
                << " ] average time usage: " << std::accumulate(r.second.time_usage_in_ms_.begin(), r.second.time_usage_in_ms_.end(), 0.0) / double(r.second.time_usage_in_ms_.size())
                << " ms , called times: " << r.second.time_usage_in_ms_.size() << std::endl;
        }
        ofs << ">>> ===== Printing run time end =====" << std::endl;

        size_t max_length = 0;
        for (const auto &iter : records_) {
            ofs << iter.first << ", ";
            if (iter.second.time_usage_in_ms_.size() > max_length) {
                max_length = iter.second.time_usage_in_ms_.size();
            }
        }
        ofs << std::endl;

        for (size_t i = 0; i < max_length; ++i) {
            for (const auto &iter : records_) {
                if (i < iter.second.time_usage_in_ms_.size()) {
                    ofs << iter.second.time_usage_in_ms_[i] << ",";
                } else {
                    ofs << ",";
                }
            }
            ofs << std::endl;
        }
        ofs.close();
    }

    /// get the average time usage of a function
    static double calculate_mean_time_cost(const std::string &func_name)
    {
        if (records_.find(func_name) == records_.end()) {
            return 0.0;
        }

        auto r = records_[func_name];
        return std::accumulate(r.time_usage_in_ms_.begin(), r.time_usage_in_ms_.end(), 0.0) / double(r.time_usage_in_ms_.size());
    }

    /// clean the records
    static void clear()
    {
        records_.clear();
    }

    //    private:
    static std::map<std::string, Time_Recorder> records_;
    static std::vector<std::pair<std::string, double>> records_tmp_;
};

std::map<std::string, Timer::Time_Recorder> Timer::records_;
std::vector<std::pair<std::string, double>> Timer::records_tmp_;