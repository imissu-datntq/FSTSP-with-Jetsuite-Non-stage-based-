////
//// Created by cuong on 4/12/24.
////

#include <iostream>
#include <ostream>
#include "../include/other_method_solver.h"

#include <chrono>
//
//class Sortie;
//
//inline bool exist(const std::vector<int>& vec, int element) {
//    // Use std::find to search for the element in the vector
//    return std::find(vec.begin(), vec.end(), element) != vec.end();
//}
//
//[[maybe_unused]] Result OtherMethodSolver::Amico2021_3Index(int n_thread, int e) {
//    auto tau = instance->tau;
//    auto tau_prime = instance->tau_prime;
//    auto dtl = e;
//    //dtl = 5;
//    auto sl = 1, sr = 1;
//    auto n = instance->num_node;
//
//    auto c_prime = instance->c_prime;
//    std::vector<int> c_prime_0;
//    c_prime_0.push_back(0);
//    for (int i : c_prime) {
//        c_prime_0.push_back(i);
//    }
//    c_prime_0.push_back(n);
//
//    std::cout << "Printing number of nodes: " << n << std::endl;
//    std::vector<int> C;
//    std::vector<int> V;
//    for (int i = 0; i < n + 1; i++) {
//        if (i == 0 || i == n) {
//            V.push_back(i);
//        }
//        else {
//            V.push_back(i);
//            C.push_back(i);
//        }
//    }
//
//    // C_s : set C(customer) union s (source) ; source  = 0
//    // C_t : set C(customer) union t (terminal); terminal = n
//    // create.
//    std::vector<int> c_s;
//    std::vector<int> c_t;
//    for (int i = 0; i < n + 1; i++) {
//        if (i == 0) {
//            c_s.push_back(i);
//        }
//        else if (i == n) {
//            c_t.push_back(i);
//        }
//        else {
//            c_s.push_back(i);
//            c_t.push_back(i);
//        }
//    }
//
//    // paper notation synchronization
//    auto N = V;
//    auto N0 = c_s;
//    auto N_p = c_t;
//    auto M = 1e5;
//    std::cout << std::endl;
//    IloEnv env;
//    IloModel model(env);
//    IloCplex cplex(model);
//    cplex.setParam(IloCplex::Param::MIP::Tolerances::Integrality, 0);
//
//    IloArray<IloBoolVarArray> x(env, n + 1);
//    for (int i : N0) {
//        x[i] = IloBoolVarArray(env, n + 1);
//        for (int j : N_p) {
//            if (i != j) {
//                x[i][j] = IloBoolVar(env);
//            }
//        }
//    }
//    IloArray<IloArray<IloBoolVarArray>> y(env, n + 1);
//    for (int i : N0) {
//        y[i] = IloArray<IloBoolVarArray>(env, n + 1);
//        for (int j : c_prime) {
//            if (i != j) {
//                y[i][j] = IloBoolVarArray(env, n + 1);
//                for (int k : N_p) {
//                    if (i != k && j != k) {
//                        if (tau_prime[i][j] + tau_prime[j][k] + sr <= dtl) {
//                            y[i][j][k] = IloBoolVar(env);
//                        }
//                    }
//                }
//            }
//        }
//    }
//
//    IloNumVarArray w(env, n + 1), t(env, n + 1);
//    for (int i : N) {
//        w[i] = IloNumVar(env, 0, IloInfinity);
//        t[i] = IloNumVar(env, 0, IloInfinity);
//    }
//
//    IloBoolVarArray z(env, n + 1);
//    for (int i : N) {
//        z[i] = IloBoolVar(env);
//    }
//
//    IloExpr objective(env);
//
//    // Objective's first term
//    for (int i : N0) {
//        for (int j : N_p) {
//            if (i != j) {
//                objective += x[i][j] * tau[i][j];
//            }
//        }
//    }
//
//    // Objective's second term
//    IloExpr obj_2nd_term(env);
//    for (int j : c_prime) {
//        for (int k : N_p) {
//            if (j != k) {
//                if (tau_prime[0][j] + tau_prime[j][k] + sr <= dtl) {
//                    obj_2nd_term += y[0][j][k];
//                }
//            }
//        }
//    }
//    objective += sr * obj_2nd_term;
//
//    // Objective's third term
//    IloExpr obj_3rd_term(env);
//    for (int i : N0) {
//        if (i != 0) {
//            for (int j : c_prime) {
//                if (i != j) {
//                    for (int k : N_p) {
//                        if (i != k && j != k) {
//                            if (tau_prime[i][j] + tau_prime[j][k] + sr <= dtl) {
//                                obj_3rd_term += y[i][j][k];
//                            }
//                        }
//                    }
//                }
//            }
//        }
//    }
//    objective += (sl + sr) * obj_3rd_term;
//
//    // Fourth term
//    for (int i : N_p) {
//        objective += w[i];
//    }
//
//    // Constraints:
//
//    // C2
//    for (int j : C) {
//        IloExpr s1(env), s2(env);
//        for (int i : N0) {
//            if (i != j) {
//                s1 += x[i][j];
//            }
//        }
//        if (exist(c_prime, j)) {
//            for (int i : N0) {
//                for (int k : N_p) {
//                    if (i != j && i != k && j != k) {
//                        if (tau_prime[i][j] + tau_prime[j][k] + sr <= dtl) {
//                            s2 += y[i][j][k];
//                        }
//                    }
//                }
//            }
//        }
//        else {
//            std::cout << "customer " << j << " can't be served by drone!" << std::endl;
//            s2 = nullptr;
//            model.add(s1 == 1);
//            continue;
//        }
//        model.add(s1 + s2 == 1);
//    }
//
//    // C3
//    IloExpr c3_s1(env), c3_s2(env);
//    for (int j : N_p) {
//        c3_s1 += x[0][j];
//    }
//    for (int i : N0) {
//        c3_s2 += x[i][n];
//    }
//    model.add(c3_s1 == 1);
//    model.add(c3_s2 == 1);
//
//    // C4
//    for (int j : C) {
//        IloExpr c4_s1(env), c4_s2(env);
//        for (int i : N0) {
//            if (i != j) {
//                c4_s1 += x[i][j];
//            }
//        }
//        for (int i : N_p) {
//            if (i != j) {
//                c4_s2 += x[j][i];
//            }
//        }
//        model.add(c4_s1 == c4_s2);
//    }
//
//    // C5
//    for (int i : N0) {
//        for (int j : N_p) {
//            if (i != j) {
//                model.add(t[j] >= t[i] + tau[i][j] - M * (1 - x[i][j]));
//            }
//        }
//    }
//
//    // C6
//    for (int i : N0) {
//        for (int j : N_p) {
//            if (i != j) {
//                IloExpr c6_s1(env);
//                for (int k : c_prime) {
//                    if (i != j && i != k && j != k) {
//                        if (tau_prime[i][k] + tau_prime[k][j] + sr <= dtl) {
//                            c6_s1 += (M + tau_prime[i][k] + tau_prime[k][j]) * y[i][k][j];
//                        }
//                    }
//                }
//                model.add(t[j] >= t[i] + c6_s1 - M);
//            }
//        }
//    }
//
//    // C7
//    for (int i : N0) {
//        for (int j : N_p) {
//            if (i != j) {
//                model.add(w[j] >= t[j] - t[i] - tau[i][j] - M * (1 - x[i][j]));
//            }
//        }
//    }
//
//    // C8
//    for (int i : N0) {
//        for (int j : N_p) {
//            if (i != j) {
//                IloExpr c8_s1(env);
//                for (int k : c_prime) {
//                    if (i != j && i != k && j != k) {
//                        if (tau_prime[i][k] + tau_prime[k][j] + sr <= dtl) {
//                            c8_s1 += y[i][k][j];
//                        }
//                    }
//                }
//                model.add(t[j] - t[i] + sr - M * (1 - c8_s1) <= dtl);
//            }
//        }
//    }
//
//    // C9
//    for (int i : N_p) {
//        IloExpr c9_s1(env);
//        for (int j : N0) {
//            if (i != j) {
//                c9_s1 += x[j][i];
//            }
//        }
//        model.add(z[i] <= c9_s1);
//    }
//
//    // C10
//    for (int i : N0) {
//        IloExpr c10_s1(env);
//        for (int j : c_prime) {
//            for (int k : N_p) {
//                if (i != j && j != k && i != k) {
//                    if (tau_prime[i][j] + tau_prime[j][k] + sr <= dtl) {
//                        c10_s1 += y[i][j][k];
//                    }
//                }
//            }
//        }
//        model.add(c10_s1 <= z[i]);
//    }
//
//    // C11
//    for (int i : N0) {
//        for (int j : N_p) {
//            if (i != j) {
//                IloExpr c11_s1(env), c11_s2(env);
//                for (int l : N0) {
//                    for (int k : c_prime) {
//                        if (l != k && l != j && k != j) {
//                            if (tau_prime[l][k] + tau_prime[k][j] + sr <= dtl) {
//                                c11_s1 += y[l][k][j];
//                            }
//                        }
//                    }
//                }
//                for (int k : c_prime) {
//                    for (int l : N_p) {
//                        if (i != k && i != l && l != k) {
//                            if (tau_prime[i][k] + tau_prime[k][l] + sr <= dtl) {
//                                c11_s2 += y[i][k][l];
//                            }
//                        }
//                    }
//                }
//                model.add(z[j] <= z[i] - x[i][j] + c11_s1 - c11_s2 + 1);
//            }
//        }
//    }
//
//    // Add constraint
//    for (int j : c_prime) {
//        IloExpr sum(env);
//        for (int i : c_s) {
//            for (int k : c_t) {
//                if (i != j && j != k && i != k) {
//                    sum += y[i][j][k];
//                }
//            }
//        }
//        for (int i : c_prime) {
//            for (int k : c_t) {
//                if (i != j && j != k && i != k) {
//                    sum += y[j][i][k];
//                }
//            }
//        }
//        for (int k : c_s) {
//            for (int i : c_prime) {
//                if (i != j && j != k && i != k) {
//                    sum += y[k][i][j];
//                }
//            }
//        }
//        model.add(sum <= 1);
//    }
//    // C12
//    model.add(t[0] == 0);
//
//    // Valid inequalities
//
//    //    // C17
//    IloExpr c17_s1(env);
//    for (int j : N_p) {
//        c17_s1 += x[0][j];
//    }
//    model.add(z[0] <= c17_s1);
//
//    // C18
//    for (int i : N_p) {
//        IloExpr c18_s1(env);
//        for (int k : N0) {
//            for (int j : c_prime) {
//                if (i != j && j != k && i != k) {
//                    if (tau_prime[k][j] + tau_prime[j][i] + sr <= dtl) {
//                        c18_s1 += y[k][j][i];
//                    }
//                }
//            }
//        }
//        model.add(c18_s1 <= z[i]);
//    }
//
//    // C19
//    for (int i : c_prime) {
//        IloExpr c19_s1(env);
//        for (int j : N0) {
//            for (int k : N_p) {
//                if (i != j && j != k && i != k) {
//                    if (tau_prime[j][i] + tau_prime[i][k] + sr <= dtl) {
//                        c19_s1 += y[j][i][k];
//                    }
//                }
//            }
//        }
//        model.add(c19_s1 <= 1 - z[i]);
//    }
//
//    // C20
//    for (int j : N_p) {
//        IloExpr c20_s1(env);
//        for (int i : N0) {
//            for (int k : c_prime) {
//                if (i != j && j != k && i != k) {
//                    if (tau_prime[i][k] + tau_prime[k][j] + sr <= dtl) {
//                        c20_s1 += y[i][k][j];
//                    }
//                }
//            }
//        }
//        model.add(w[j] <= dtl * c20_s1);
//    }
//
//    // C21
//    for (int j : c_prime) {
//        IloExpr c21_s1(env);
//        for (int i : N0) {
//            for (int k : N_p) {
//                if (i != j && j != k && i != k) {
//                    if (tau_prime[i][j] + tau_prime[j][k] + sr <= dtl) {
//                        c21_s1 += y[i][j][k];
//                    }
//                }
//            }
//        }
//        model.add(w[j] <= dtl * (1 - c21_s1));
//    }
//    model.add(IloMinimize(env, objective));
//
//    auto startTime = std::chrono::high_resolution_clock::now();
//    cplex.solve();
//    auto endTime = std::chrono::high_resolution_clock::now();
//    // Calculate the elapsed time
//    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
//    if (cplex.getStatus() == IloAlgorithm::Infeasible) {
//        // Handle infeasibility
//        std::cout << "The problem is infeasible." << std::endl;
//        // You can also retrieve the infeasible constraints using cplex.getInfeasibility() method
//    }
//    else {
//        // Handle other solver errors
//        std::cerr << "Solver error: " << cplex.getStatus() << std::endl;
//    }
//    for (int i : c_s) {
//        for (int j : c_t) {
//            if (i != j) {
//                if (cplex.getValue(x[i][j]) == 1) {
//                    std::cout << i << " " << j << std::endl;
//                }
//            }
//        }
//    }
//    std::cout << "Drone sorties:" << std::endl;
//    for (int i : N0) {
//        for (int j : c_prime) {
//            for (int k : N_p) {
//                if (i != j && i != k && j != k) {
//                    if (tau_prime[i][j] + tau_prime[j][k] <= e - sr) {
//                        if (cplex.getValue(y[i][j][k]) == 1) {
//                            std::cout << i << " " << j << " " << k << std::endl;
//                        }
//                    }
//                }
//            }
//        }
//    }
//    std::cout << cplex.getObjValue() << std::endl;
//    std::vector<Sortie> st;
//    return Result(cplex.getObjValue(), duration.count() / 1000, cplex.getMIPRelativeGap());
//}
//
//[[maybe_unused]] Result OtherMethodSolver::Amico2021_2Index(int n_thread, int e) {
//    auto tau = instance->tau;
//    auto tau_prime = instance->tau_prime;
//    auto dtl = e;
//    //dtl = 5;
//    auto sl = 1, sr = 1;
//    auto n = instance->num_node;
//
//    auto c_prime = instance->c_prime;
//    std::vector<int> c_prime_0;
//    c_prime_0.push_back(0);
//    for (int i : c_prime) {
//        c_prime_0.push_back(i);
//    }
//    c_prime_0.push_back(n);
//
//    std::cout << "Printing number of nodes: " << n << std::endl;
//    std::vector<int> C;
//    std::vector<int> V;
//    for (int i = 0; i < n + 1; i++) {
//        if (i == 0 || i == n) {
//            V.push_back(i);
//        }
//        else {
//            V.push_back(i);
//            C.push_back(i);
//        }
//    }
//
//    // C_s : set C(customer) union s (source) ; source  = 0
//    // C_t : set C(customer) union t (terminal); terminal = n
//    // create.
//    std::vector<int> c_s;
//    std::vector<int> c_t;
//    for (int i = 0; i < n + 1; i++) {
//        if (i == 0) {
//            c_s.push_back(i);
//        }
//        else if (i == n) {
//            c_t.push_back(i);
//        }
//        else {
//            c_s.push_back(i);
//            c_t.push_back(i);
//        }
//    }
//
//    // paper notation synchronization
//    auto N = V;
//    auto N0 = c_s;
//    auto N_p = c_t;
//    auto M = 1e5;
//    std::cout << std::endl;
//    IloEnv env;
//    IloModel model(env);
//    IloCplex cplex(model);
//    cplex.setParam(IloCplex::Param::MIP::Tolerances::Integrality, 0);
//
//    IloNumVarArray w(env, n + 1), t(env, n + 1);
//    for (int i : N) {
//        w[i] = IloNumVar(env, 0, IloInfinity);
//        t[i] = IloNumVar(env, 0, IloInfinity);
//    }
//
//    IloBoolVarArray z(env, n + 1);
//    for (int i : N) {
//        z[i] = IloBoolVar(env);
//    }
//
//    IloArray<IloBoolVarArray> x(env, n + 1);
//    for (int i : N0) {
//        x[i] = IloBoolVarArray(env, n + 1);
//        for (int j : N_p) {
//            if (i != j) {
//                x[i][j] = IloBoolVar(env);
//            }
//        }
//    }
//    IloArray<IloBoolVarArray> gl(env, n + 1);
//    for (int i : N0) {
//        gl[i] = IloBoolVarArray(env, n + 1);
//        for (int j : c_prime) {
//            if (i != j) {
//                gl[i][j] = IloBoolVar(env);
//                if (tau_prime[i][j] < e) {
//                    model.add(gl[i][j] == 0);
//                }
//            }
//        }
//    }
//    IloArray<IloBoolVarArray> gr(env, n + 1);
//    for (int j : c_prime) {
//        gr[j] = IloBoolVarArray(env, n + 1);
//        for (int k : N_p) {
//            if (j != k) {
//                gr[j][k] = IloBoolVar(env);
//                if (tau_prime[j][k] < e) {
//                    model.add(gr[j][k] == 0);
//                }
//            }
//        }
//    }
//
//    // Objective
//    IloExpr objective(env);
//
//    // First term
//    for (int i : N0) {
//        for (int j : N_p) {
//            if (i != j) {
//                objective += x[i][j] * tau[i][j];
//            }
//        }
//    }
//
//    // Second term
//    IloExpr obj_2nd_term(env);
//    for (int i : N0) {
//        if (i != 0) {
//            for (int j : c_prime) {
//                if (i != j) {
//                    if (tau_prime[i][j] < e)
//                        obj_2nd_term += gl[i][j];
//                }
//            }
//        }
//    }
//    objective += sl * obj_2nd_term;
//
//    // Third term
//    IloExpr obj_3rd_term(env);
//    for (int j : c_prime) {
//        for (int k : N_p) {
//            if (j != k) {
//                if (tau_prime[j][k] < e)
//                    objective += gr[j][k];
//            }
//        }
//    }
//    objective += sr * obj_3rd_term;
//
//    // Fourth term
//    for (int i : N_p) {
//        objective += w[i];
//    }
//
//    // C23
//    for (int j : C) {
//        IloExpr c23_s1(env), c23_s2(env);
//        for (int i : N0) {
//            if (i != j) {
//                if (tau_prime[i][j] < e)
//                    c23_s1 += x[i][j];
//                c23_s2 += gl[i][j];
//            }
//        }
//        model.add(c23_s1 + c23_s2 == 1);
//    }
//
//    // C24
//    for (int i : N0) {
//        for (int j : N_p) {
//            if (i != j) {
//                if (tau_prime[i][j] < e)
//                    model.add(t[j] >= t[i] + tau_prime[i][j] - M * (1 - gl[i][j]));
//            }
//        }
//    }
//
//    // C25
//    for (int j : c_prime) {
//        for (int k : N_p) {
//            if (j != k) {
//                if (tau_prime[j][k] < e)
//
//                    model.add(t[k] >= t[j] + tau_prime[j][k] - M * (1 - gr[j][k]));
//            }
//        }
//    }
//
//    // C26
//    for (int i : N0) {
//        for (int j : c_prime) {
//            for (int k : N_p) {
//                if (i != j && i != k && j != k) {
//                    if (tau_prime[i][j] + tau_prime[j][k] + sr <= dtl) {
//                        model.add(t[k] - t[i] + sr - M * (2 - gl[i][j] - gr[j][k]) <= e);
//                    }
//                }
//            }
//        }
//    }
//
//    // C27
//    for (int j : c_prime) {
//        IloExpr s1(env), s2(env);
//        for (int i : N0) {
//            if (i != j) {
//                if (tau_prime[i][j] < e)
//                    s1 += gl[i][j];
//            }
//        }
//        for (int k : N_p) {
//            if (j != k) {
//                if (tau_prime[j][k] < e)
//                    s2 += gr[j][k];
//            }
//        }
//        model.add(s1 == s2);
//    }
//
//    // C28
//    for (int i : N0) {
//        IloExpr s(env);
//        for (int j : c_prime) {
//            if (i != j) {
//                if (tau_prime[i][j] < e)
//
//                    s += gl[i][j];
//            }
//        }
//        model.add(s <= z[i]);
//    }
//
//    // C29
//    for (int i : N0) {
//        for (int j : N_p) {
//            if (i != j) {
//                IloExpr s(env);
//                for (int k : c_prime) {
//                    if (k != j && i != k) {
//                        s += gr[k][j];
//                        s -= gl[i][k];
//                    }
//                }
//                model.add(z[j] <= z[i] - x[i][j] + s + 1);
//            }
//        }
//    }
//
//    // C30
//    for (int i : c_prime) {
//        for (int j : c_prime) {
//            if (i != j) {
//                if (tau_prime[i][j] < e)
//
//                    model.add(gl[i][j] + gr[i][j] <= 1);
//                model.add(gl[i][j] + gr[j][i] <= 1);
//            }
//        }
//    }
//
//    // C3
//    IloExpr c3_s1(env), c3_s2(env);
//    for (int j : N_p) {
//        c3_s1 += x[0][j];
//    }
//    for (int i : N0) {
//        c3_s2 += x[i][n];
//    }
//    model.add(c3_s1 == 1);
//    model.add(c3_s2 == 1);
//
//    // C4
//    for (int j : C) {
//        IloExpr c4_s1(env), c4_s2(env);
//        for (int i : N0) {
//            if (i != j) {
//                c4_s1 += x[i][j];
//            }
//        }
//        for (int i : N_p) {
//            if (i != j) {
//                c4_s2 += x[j][i];
//            }
//        }
//        model.add(c4_s1 == c4_s2);
//    }
//    // C5
//    for (int i : N0) {
//        for (int j : N_p) {
//            if (i != j) {
//                model.add(t[j] >= t[i] + tau[i][j] - M * (1 - x[i][j]));
//            }
//        }
//    }
//    // C7
//    for (int i : N0) {
//        for (int j : N_p) {
//            if (i != j) {
//                model.add(w[j] >= t[j] - t[i] - tau[i][j] - M * (1 - x[i][j]));
//            }
//        }
//    }
//    // C9
//    for (int i : N_p) {
//        IloExpr c9_s1(env);
//        for (int j : N0) {
//            if (i != j) {
//                c9_s1 += x[j][i];
//            }
//        }
//        model.add(z[i] <= c9_s1);
//    }
//
//    model.add(IloMinimize(env, objective));
//    cplex.solve();
//    std::cout << cplex.getObjValue() << std::endl;
//    for (int i : c_s) {
//        for (int j : c_t) {
//            if (i != j) {
//                if (cplex.getValue(x[i][j]) == 1) {
//                    std::cout << i << " " << j << std::endl;
//                }
//            }
//        }
//    }
//    std::cout << "gl:" << std::endl;
//    for (int i : N0) {
//        for (int j : c_prime) {
//            if (i != j) {
//                if (tau_prime[i][j] <= e) {
//                    if (cplex.getValue(gl[i][j]) == 1) {
//                        std::cout << i << " " << j << std::endl;
//                    }
//                }
//            }
//        }
//    }
//
//    std::cout << "gr:" << std::endl;
//    for (int j : c_prime) {
//        for (int k : N_p) {
//            if (k != j) {
//                if (tau_prime[j][k] <= e) {
//                    if (cplex.getValue(gr[j][k]) == 1) {
//                        std::cout << j << " " << k << std::endl;
//                    }
//                }
//            }
//        }
//    }
//
//    std::cout << cplex.getObjValue() << std::endl;
//    return Result();
//}
//

//// Paper: Exact methods for the traveling salesman problem with drone.
//// Link: https://drive.google.com/file/d/1aglcxNadkpMoxdb_o-bYVVgc7PAANxK6/view?usp=sharing
[[maybe_unused]] Result OtherMethodSolver::Roberti2020(Config &cfg) {
    auto tau = instance->tau;
    auto tau_prime = instance->tau_prime;
    auto dtl = cfg.dtl;
    //dtl = 5;
    auto sl = cfg.sl, sr = cfg.sr;
    auto n = instance->num_node;

    auto c_prime = instance->c_prime;
    std::vector<int> c_prime_0;
    c_prime_0.push_back(0);
    for (int i : c_prime) {
        c_prime_0.push_back(i);
    }
    c_prime_0.push_back(n);

    std::cout << "Printing number of nodes: " << n << std::endl;
    std::vector<int> C;
    std::vector<int> V;
    for (int i = 0; i < n + 1; i++) {
        if (i == 0 || i == n) {
            V.push_back(i);
        }
        else {
            V.push_back(i);
            C.push_back(i);
        }
    }

    // C_s : set C(customer) union s (source) ; source  = 0
    // C_t : set C(customer) union t (terminal); terminal = n
    // create.
    std::vector<int> c_s;
    std::vector<int> c_t;
    for (int i = 0; i < n + 1; i++) {
        if (i == 0) {
            c_s.push_back(i);
        }
        else if (i == n) {
            c_t.push_back(i);
        }
        else {
            c_s.push_back(i);
            c_t.push_back(i);
        }
    }

    std::cout << std::endl;
    IloEnv env;
    IloModel model(env);
    IloCplex cplex(model);
    cplex.setParam(IloCplex::Param::MIP::Tolerances::Integrality, 0);
    // Set the config parameters
    if (cfg.screen_mode == 0 || cfg.screen_mode == 1) {
        cplex.setOut(env.getNullStream());
        cplex.setError(env.getNullStream());
        cplex.setWarning(env.getNullStream());
    }
    if (cfg.tl > 0)
        cplex.setParam(IloCplex::Param::TimeLimit, cfg.tl);

    if (cfg.num_thread > 0)
        cplex.setParam(IloCplex::Param::Threads, cfg.num_thread);
    // notations
    auto N = C; // N: set of customers
    auto N_0 = c_s; // N_0 = s unions N
    auto N_0_p = c_t; // N_0' = t unions N
    auto s = 0;
    auto t = n;

    IloArray<IloBoolVarArray> xt(env, n + 1);
    for (int i : c_s) {
        xt[i] = IloBoolVarArray(env, n + 1);
        for (int j : c_t) {
            if (i != j) {
                xt[i][j] = IloBoolVar(env);
                if (i == s && j == t) {
                    model.add(xt[i][j] == 0);
                }
            }
        }
    }

    IloArray<IloBoolVarArray> xd(env, n + 1);
    for (int i : c_s) {
        xd[i] = IloBoolVarArray(env, n + 1);
        for (int j : c_t) {
            if (i != j) {
                xd[i][j] = IloBoolVar(env);
                if (i == s && j == t) {
                    model.add(xd[i][j] == 0);
                }
            }
        }
    }

    IloBoolVarArray yt(env, n);
    IloBoolVarArray yd(env, n);
    IloBoolVarArray yc(env, n);
    IloBoolVarArray l(env, n);
    for (int i : N) {
        yt[i] = IloBoolVar(env);
        yd[i] = IloBoolVar(env);
        yc[i] = IloBoolVar(env);
        l[i] = IloBoolVar(env);
    }

    // Variable z
    IloArray<IloBoolVarArray> z(env, n + 1);
    for (int i : c_s) {
        z[i] = IloBoolVarArray(env, n + 1);
        for (int j : N) {
            if (i != j) {
                z[i][j] = IloBoolVar(env);
            }
        }
    }
    IloNumVarArray a(env, n + 1);
    for (int i : V) {
        a[i] = IloNumVar(env, 0, IloInfinity, ILOFLOAT);
    }


    // Constraints::

    // 1b
    for (int i : N) {
        IloExpr s1(env), s2(env);
        for (int j : c_t) {
            if (i != j) {
                s1 += xt[i][j];
            }
        }
        for (int j : c_s) {
            if (i != j) {
                s2 += xt[j][i];
            }
        }
        model.add(s1 == s2);
    }

    // 1c
    for (int i : N) {
        IloExpr s1(env);
        for (int j : c_t) {
            if (i != j) {
                s1 += xt[i][j];
            }
        }
        model.add(s1 == yt[i] + yc[i]);
    }

    // 1d
    IloExpr s1_1d(env), s2_1d(env);
    for (int j : c_t) {
        s1_1d += xt[s][j];
    }
    for (int i : c_s) {
        s2_1d += xt[i][t];
    }
    model.add(s1_1d == 1);
    model.add(s1_1d == s2_1d);

    // 1e
    for (int i : N) {
        IloExpr s1(env), s2(env);
        for (int j : c_t) {
            if (i != j) {
                s1 += xd[i][j];
            }
        }
        for (int j : c_s) {
            if (i != j) {
                s2 += xd[j][i];
            }
        }
        model.add(s1 == s2);
    }

    // 1f
    for (int i : N) {
        IloExpr s1(env);
        for (int j : c_t) {
            if (i != j) {
                s1 += xd[i][j];
            }
        }
        model.add(s1 == yd[i] + yc[i]);
    }

    // 1g
    IloExpr s1_1g(env), s2_1g(env);
    for (int j : c_t) {
        s1_1g += xd[s][j];
    }
    for (int i : c_s) {
        s2_1g += xd[i][t];
    }
    model.add(s1_1g == 1);
    model.add(s1_1g == s2_1g);

    // 1h
    for (int i : N) {
        model.add(yt[i] + yd[i] + yc[i] == 1);
    }

    int M = 1e5;
    // 1i
    for (int i : c_s) {
        for (int j : c_t) {
            if (i != j) {
                model.add(a[i] + tau[i][j] <= a[j] + M * (1 - xt[i][j]));
            }
        }
    }

    // 1j
    for (int i : c_s) {
        for (int j : c_t) {
            if (i != j) {
                model.add(a[i] + tau_prime[i][j] <= a[j] + M * (1 - xd[i][j]));
            }
        }
    }

    // 1k
    for (int i : N) {
        for (int j : N) {
            if (i < j) {
                model.add(xd[i][j] + xd[j][i] <= yc[i] + yc[j]);
            }
        }
    }

    // 1l
    for (int i : N) {
        model.add(xd[s][i] + xd[i][t] <= 1);
    }

    // Valid inequalities
    IloExpr sum_t(env), sum_d(env);
    for (int i : c_s) {
        for (int j : c_t) {
            if (i != j) {
                sum_t += xt[i][j] * tau[i][j];
                sum_d += xd[i][j] * tau_prime[i][j];
            }
        }
    }
    model.add(sum_t <= a[t]);
    model.add(sum_d <= a[t]);

    // Incompatible drone customer(s)
    for (int heavy : instance->heavy) {
        model.add(yd[heavy] == 0);
        for (int j : c_s) {
            if (heavy != j) {
                model.add(z[j][heavy] == 0);
            }
        }
    }

    // Drone endurance
    // 5a
    for (int i : c_s) {
        for (int j : c_t) {
            if (i != j) {
                if (tau_prime[i][j] >= dtl - sr) {
                    model.add(xd[i][j] <= xt[i][j]);
                }
            }
        }
    }

    // 5b
    for (int j : N) {
        if (tau_prime[s][j] + tau_prime[j][t] > dtl - sr) {
            model.add(z[s][j] == 0);
        }
    }

    // 5c
    for (int i : N) {
        for (int j : N) {
            if (i != j) {
                if (tau_prime[i][j] + tau_prime[j][i] > dtl - sr) {
                    model.add(z[i][j] == 0);
                }
            }
        }
    }

    // 5d
    for (int i : N) {
        IloExpr s1(env), s2(env), s3(env);
        for (int j : c_s) {
            if (j != i) {
                s1 += tau_prime[j][i] * xd[j][i];
            }
        }
        for (int j : c_t) {
            if (j != i) {
                s2 += tau_prime[i][j] * xd[i][j];
            }
        }
        for (int j : N) {
            if (i != j) {
                s3 += (tau_prime[j][i] + tau_prime[i][j]) * z[j][i];
            }
        }
        model.add(s1 + s2 + (tau_prime[s][i] + tau_prime[i][t]) * z[s][i] + s3 <= dtl - sr + M * (1 - yd[i]));
    }

    // Drone cannot land and wait
    // 7a
    for (int j : c_s) {
        for (int k : c_t) {
            for (int i : N) {
                if (i != j && j != k && i != k) {
                    if (tau_prime[j][i] + tau_prime[i][k] <= dtl - sr) {
                        model.add(a[k] - a[j] <= dtl - sr + M * (2 - xd[j][i] - xd[i][k]) + M * (1 - yd[i]));
                    }
                }
            }
        }
    }

    // Launch and rendezvous time
    // c9
    for (int i : N) {
        model.add(l[i] >= yd[i] - xd[s][i]);
    }

    IloExpr objective(env);
    objective += a[t];
    for (int j : N) {
        objective += (tau_prime[s][j] + tau_prime[j][t] + sr) * z[s][j];
    }
    for (int i : N) {
        for (int j : N) {
            if (i != j) {
                objective += (tau_prime[i][j] + tau_prime[j][i] + sl + sr) * z[i][j];
            }
        }
        objective += sl * l[i];
        objective += sr * yd[i];
    }
    // Objective definition
    model.add(IloMinimize(env, objective));
    // export model
    cplex.exportModel("compact_model.lp");
    auto startTime = std::chrono::high_resolution_clock::now();
    cplex.solve();
    auto endTime = std::chrono::high_resolution_clock::now();
    // Calculate the elapsed time
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Solve time: " << duration.count() / 1000.0 << std::endl;
    std::cout << "Objective Value: " << cplex.getObjValue() << std::endl;
    std::cout << "Truck arcs:" << std::endl;
    for (int i : c_s) {
        for (int j : c_t) {
            if (i != j) {
                if (cplex.getValue(xt[i][j]) == 1) {
                    std::cout << i << " " << j << std::endl;
                }
            }
        }
    }
    std::cout << "Drone served customers:" << std::endl;
    for (int i : N) {
        if (cplex.getValue(yd[i]) == 1) {
            std::cout << "Customer " << i << " served by drone." << std::endl;
        }
    }
    std::cout << "Combined customers:" << std::endl;
    for (int i : N) {
        if (cplex.getValue(yc[i]) == 1) {
            std::cout << "Customer " << i << " is a combined customer." << std::endl;
        }
    }
    std::cout << "Drone arcs:" << std::endl;
    for (int i : c_s) {
        for (int j : c_t) {
            if (i != j) {
                if (cplex.getValue(xd[i][j]) == 1) {
                    std::cout << i << " " << j << " " << cplex.getValue(xd[i][j]) << std::endl;
                }
            }
        }
    }
    std::vector<Sortie> st;
    return Result(cplex.getObjValue(), duration.count() / 1000, cplex.getMIPRelativeGap());
}
