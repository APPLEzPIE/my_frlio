/*
 * @Author: zhaoxiaoyu01 zhaoxiaoyu01@baidu.com
 * @Date: 2023-03-09 21:32:25
 * @LastEditors: zhaoxiaoyu01 zhaoxiaoyu01@baidu.com
 * @LastEditTime: 2023-03-10 09:38:47
 * @FilePath: /fr-lio/src/memory_pool/custum_memory_pool.hpp
 * @Description: 
 * 
 * Copyright (c) 2023 by ${git_name_email}, All Rights Reserved. 
 */
#pragma once
#include <iostream>
#include <vector>
#include <functional>

template<typename DataType>
class Custom_Memory_Pool {
public:
    using ConstructFuncType = std::function<DataType*()>;
    Custom_Memory_Pool(ConstructFuncType construct_func, size_t increm_size = 1000)
        : construct_func_(construct_func),
          increm_size_(increm_size),
          remain_num_(0) {
    }

    Custom_Memory_Pool()
        : construct_func_(nullptr),
          increm_size_(1000),
          remain_num_(0) {}

    void set_construct_func(ConstructFuncType construct_func) {
        construct_func_ = construct_func;
    }
    
    void set_increm_size(int increm_size) {
        increm_size_ = increm_size;
    }

    void expand_pool(int n) {
        if(pool_.size() < n) {
            int stop_size = remain_num_ + n - pool_.size();
            pool_.resize(n);
            for(; remain_num_ < stop_size; remain_num_++) {
                pool_[remain_num_] = construct_func_();
            }
        }
    }

    DataType* allocate_safe() {
        if(remain_num_ == 0) {
           expand_pool(pool_.size()+increm_size_); 
        }
        remain_num_--;
        return pool_[remain_num_];
    }

    DataType* allocate_fast() {
        remain_num_--;
        return pool_[remain_num_];
    }

    void deallocate(DataType* data) {
        data->reset();
        pool_[remain_num_] = data;
        remain_num_++;
    }
private:
    size_t remain_num_;
    std::vector<DataType*> pool_;
    size_t increm_size_;
    ConstructFuncType construct_func_;
};