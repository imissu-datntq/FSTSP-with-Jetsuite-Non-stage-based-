////
//// Created by cuong on 4/12/24.
////
#pragma once
#ifndef OTHER_METHOD_SOLVER_H
#define OTHER_METHOD_SOLVER_H
#include "fstsp.h"


class OtherMethodSolver {
public:
    std::shared_ptr<Instance> instance;

    explicit OtherMethodSolver(std::shared_ptr<Instance> &instance) : instance(std::move(instance)) {}
    [[maybe_unused]] Result Roberti2020(Config &config);

//    [[maybe_unused]] Result Amico2021_3Index(int n_thread, int e);

//    [[maybe_unused]] Result Amico2021_2Index(int n_thread, int e);
};
#endif //OTHER_METHOD_SOLVER_H
