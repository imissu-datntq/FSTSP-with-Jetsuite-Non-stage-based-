//
// Created by cuong on 4/12/24.
//
#pragma once
#ifndef FSTSP_H
#define FSTSP_H
#include <memory>

#include "instance.h"
#include "cfg.h"
#include "solver.h"
#include <utility>
#include <vector>
class Sortie {
public:
    int launch;
    int rendezvous;
    int customer;

    Sortie(int l, int r, int c) {
        launch = l;
        rendezvous = r;
        customer = c;
    };
};
class WarmstartInfo {
public:
    std::vector<int> stage_to_node;
    std::vector<Sortie> sorties;

    WarmstartInfo(std::vector<int> s_t_n, std::vector<Sortie> st) {
        stage_to_node = std::move(s_t_n);
        sorties = std::move(st);
    };
};
class FSTSPSolver {
public:
    std::shared_ptr<Instance> instance;

    explicit FSTSPSolver(std::shared_ptr<Instance> instance) : instance(std::move(instance)) {}
    Result OriginalSolverCPLEX(Config &cfg);

    [[nodiscard]] Result mvdSolverWithLR(Config &cfg) const;

    [[maybe_unused]] [[nodiscard]] Result
    stage_based_fstsp(Config &cfg) const;

    [[maybe_unused]] [[nodiscard]] Result
    RV_FSTSP_MVD_Subset(Config &cfg) const;

    [[maybe_unused]] [[nodiscard]] Result
    RV_FSTSP_MVD_4_index(Config &cfg) const;

    [[maybe_unused]] [[nodiscard]] Result
    RV_FSTSP_MVD_4_index_t_D(Config &cfg) const;

    [[maybe_unused]] [[nodiscard]] Result
    RV_FSTSP_MVD_Re(Config &cfg) const;
    // [[maybe_unused]] [[nodiscard]] Result GeneralModel(Config &cfg) const;
    [[maybe_unused]] [[nodiscard]] Result
    FSTSP_lrs(Config &cfg) const;
    static IloNumArray TSP_MTZ(std::vector<std::vector<double> > &tau);

    static IloNumArray RevisitTSP(std::vector<std::vector<double> > &tau);



    // 1-visit, use wait(kk') instead of a[k] and d[k]
    [[maybe_unused]] [[nodiscard]] Result Stage_Based_FSTSP_Original(Config &cfg) const;
    [[nodiscard]] WarmstartInfo FSTSP_Warmstart(Config &cfg) const;
    Result RV_FSTSP_5_index(Config &cfg) const;
    Result RV_FSTSP_3_index(Config &cfg) const;
    Result FSTSP_2_index(Config &cfg) const;

    // ---------------------- NEW: LaTeX models ----------------------
    // Case 1 – Two-index model (drone returns to launch point) with JetSuite
    [[nodiscard]] Result FSTSP_2index_with_JetSuite(Config &cfg) const;

    // Case 2 – Three-index (no stage): drone retrieved at a different node, with JetSuite
    [[nodiscard]] Result FSTSP_3indexNoStage_with_JetSuite(Config &cfg) const;
    // ----------------------------------------------------------------
};
#endif //FSTSP_H
