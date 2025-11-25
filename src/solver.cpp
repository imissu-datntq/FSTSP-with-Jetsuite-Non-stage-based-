// //
// // Created by cuong on 18/01/2024.
// //
// #include "../include/solver.h"

// #include <chrono>
// #include <functional>
// #include <iostream>
// #include <vector>
// #include <unordered_set>

// #include "../../../../../usr/include/complex.h"

// const int bigM = 500;
// double dp[23][1 << 23] = {0};
// int sz[1 << 23] = {0};
// double minPath[23];

// //Result::Result(double solver_cost, double recalculated_cost, std::vector<Sortie>& st) {
// //    cost = c;
// //    sortie = st;
// //}
// Result::Result(double c, double t, double gap) {
//     cost = c;
//     time_spent = t;
//     mip_gap = gap;
// }

// inline bool exist(const std::vector<int>& vec, int element) {
//     // Use std::find to search for the element in the vector
//     return std::find(vec.begin(), vec.end(), element) != vec.end();
// }

// void setPrint(std::vector<int>& set) {
//     for (int i : set) {
//         std::cout << i << " ";
//     }
//     std::cout << std::endl;
// }
// std::vector<std::pair<std::vector<int>, std::vector<int>>> generateSetsAndComplements(
//     const std::vector<int>& elements) {
//     auto n = elements.size();
//     std::vector<std::pair<std::vector<int>, std::vector<int>>> result;

//     // Total number of subsets is 2^n
//     int totalSubsets = 1 << n;

//     for (int i = 0; i < totalSubsets; ++i) {
//         std::vector<int> setS, setT;

//         for (int j = 0; j < n; ++j) {
//             // Check if jth bit is set in the binary representation of i
//             if (i & (1 << j)) {
//                 setS.push_back(elements[j]);
//             }
//             else {
//                 setT.push_back(elements[j]);
//             }
//         }
//         if (setS.size() != 0 && setT.size() != 0) {
//             result.push_back(std::make_pair(setS, setT));
//         }
//     }

//     return result;
// }
// void genSetsHelper_SortieGap(const std::vector<int>& nodes, int k, std::vector<int>& currentSet,
//                              std::vector<std::vector<int>>& result) {
//     // Base case: if currentSet has k nodes, add it to result
//     if (currentSet.size() == k) {
//         result.push_back(currentSet);
//         return;
//     }

//     // Recursive case: iterate through all nodes
//     for (int i = 0; i < nodes.size(); ++i) {
//         // Add the current node to the set
//         currentSet.push_back(nodes[i]);

//         // Recur with the updated set
//         genSetsHelper_SortieGap(nodes, k, currentSet, result);

//         // Backtrack: remove the last node from the set
//         currentSet.pop_back();
//     }
// }

// std::vector<std::vector<int>> genSetForSortieGap(const std::vector<int>& nodes, int k, int D) {
//     std::vector<std::vector<int>> result;
//     std::vector<int> currentSet;
//     genSetsHelper_SortieGap(nodes, k, currentSet, result);
//     std::vector<std::vector<int>> result2;
//     for (auto vec : result) {
//         // perform test
//         bool check = true;
//         for (int i = 0; i < vec.size(); i++) {
//             if (i < vec.size() - 1 && vec[i] == D) {
//                 check = false;
//             }
//             if (i < vec.size() - 1 && vec[i] == vec[i + 1]) {
//                 check = false;
//             }
//         }
//         if (check) {
//             result2.push_back(vec);
//         }
//     }

//     return result2;
// }

// inline double calculate_tour(std::vector<std::vector<double>>& tau, std::vector<int>& tour) {
//     double l = 0;
//     for (int i = 0; i < tour.size() - 1; i++) {
//         l += tau[tour[i]][tour[i + 1]];
//     }
//     return l;
// }

// inline double smallest_tour_length(int stage_gap, std::vector<std::vector<double>>& tau, std::vector<int>& V) {
//     int D = V.size() - 1;
//     auto sets = genSetForSortieGap(V, stage_gap + 1, D);
//     double smallest = std::numeric_limits<double>::max();
//     for (auto set : sets) {
//         auto l = calculate_tour(tau, set);
//         if (l < smallest) {
//             smallest = l;
//         }
//     }
//     return smallest;
// }

// Result Solver::mFSTSPSolve(int n_thread, double dtl, double sl, double sr) const {
//     auto tau = instance->tau;
//     auto tau_prime = instance->tau_prime;
//     auto n = instance->num_node;
//     std::vector<int> C;
//     for (int i = 0; i < n + 1; i++) {
//         if (i != 0 && i != n) {
//             C.push_back(i);
//         }
//     }

//     IloEnv env;
//     IloModel model(env);
//     IloCplex cplex(model);
//     cplex.setParam(IloCplex::Param::Emphasis::MIP, CPX_MIPEMPHASIS_OPTIMALITY);
//     cplex.setParam(IloCplex::Param::MIP::Tolerances::Integrality, 0);

//     const auto O = 0;
//     const auto D = n;
//     const auto K = n + 1;

//     // cs: points that from it vehicle can leave.
//     std::vector<int> c_s;

//     // ce: points that the truck can enter.
//     std::vector<int> c_t;

//     for (int i = O; i <= D; i++) {
//         if (i != D) {
//             c_s.push_back(i);
//             c_t.push_back(i);
//         }
//         else {
//             c_t.push_back(i);
//         }
//     }

//     IloArray<IloBoolVarArray> y(env, n + 1);
//     for (int i : c_s) {
//         y[i] = IloBoolVarArray(env, n + 1);
//         for (int j : c_t) {
//             if (i != j) {
//                 y[i][j] = IloBoolVar(env);
//             }
//         }
//     }
//     int nd = 3;
//     std::vector<int> d = {0, 1, 2};
//     IloArray<IloArray<IloBoolVarArray>> x(env, nd);
//     for (int v : d) {
//         x[v] = IloArray<IloBoolVarArray>(env, n + 1);
//         for (int i : c_s) {
//             x[v][i] = IloBoolVarArray(env, n + 1);
//             for (int j : c_t) {
//                 x[v][i][j] = IloBoolVar(env);
//             }
//         }
//     }
//     IloArray<IloArray<IloArray<IloBoolVarArray>>> gamma(env, n);
//     for (int h : C) {
//         gamma[h] = IloArray<IloArray<IloBoolVarArray>>(env, nd);
//         for (int v : d) {
//             gamma[h][v] = IloArray<IloBoolVarArray>(env, n + 1);
//             for (int i : c_s) {
//                 gamma[h][v][i] = IloBoolVarArray(env, n + 1);
//                 for (int j : c_t) {
//                     if (i != j) {
//                         gamma[h][v][i][j] = IloBoolVar(env);
//                     }
//                 }
//             }
//         }
//     }

//     IloArray<IloBoolVarArray> theta(env, n + 1);
//     for (int h : C) {
//         theta[h] = IloBoolVarArray(env, n + 1);
//         for (int v : d) {
//             theta[h][v] = IloBoolVar(env);
//         }
//     }
//     IloArray<IloArray<IloBoolVarArray>> omega(env, n + 1);
//     for (int h : C) {
//         omega[h] = IloArray<IloBoolVarArray>(env, nd);
//         for (int v : d) {
//             omega[h][v] = IloBoolVarArray(env, n + 1);
//             for (int i : c_s) {
//                 if (h != i) {
//                     omega[h][v][i] = IloBoolVar(env);
//                 }
//             }
//         }
//     }
//     IloArray<IloArray<IloBoolVarArray>> delta(env, n + 1);
//     for (int h : C) {
//         delta[h] = IloArray<IloBoolVarArray>(env, nd);
//         for (int v : d) {
//             delta[h][v] = IloBoolVarArray(env, n + 1);
//             for (int j : c_t) {
//                 if (h != j) {
//                     delta[h][v][j] = IloBoolVar(env);
//                 }
//             }
//         }
//     }

//     IloNumVarArray w(env, n);
//     for (int i : c_t) {
//         w[i] = IloNumVar(env, 0, IloInfinity);
//     }

//     IloArray<IloArray<IloBoolVarArray>> z(env, n + 1);
//     for (int i : c_s) {
//         z[i] = IloArray<IloBoolVarArray>(env, nd);
//         for (int v : d) {
//             z[i][v] = IloBoolVarArray(env, nd);
//             for (int w : d) {
//                 if (v != w) {
//                     z[i][v][w] = IloBoolVar(env);
//                 }
//             }
//         }
//     }
//     auto s = 0;
//     auto t = n;
//     // C1
//     IloExpr c1s1(env), c1s2(env);
//     for (int j : c_t) {
//         c1s1 += y[s][j];
//     }
//     for (int i : c_s) {
//         c1s2 += y[i][t];
//     }
//     model.add(c1s1 == c1s2);
//     model.add(c1s1 == 1);

//     // C2
//     for (int i : C) {
//         IloExpr c2s1(env), c2s2(env);
//         for (int j : c_t) {
//             if (i != j) {
//                 c2s1 += y[i][j];
//             }
//         }
//         for (int j : c_s) {
//             if (i != j) {
//                 c2s2 += y[j][i];
//             }
//         }
//         model.add(c2s1 == c2s2);
//         model.add(c2s1 <= 1);
//     }

//     // C3
//     auto setAndComps = generateSetsAndComplements(C);
//     for (auto& set : setAndComps) {
//         auto S = set.first;
//         if (S.size() < 2) {
//             continue;
//         }
//         if (S.size() == 2 && S[0] == s && S[1] == t) {
//             continue;
//         }
//         IloExpr sum1(env), sum2(env);
//         std::string cname = "C3";
//         for (auto i : S) {
//             cname += "_" + std::to_string(i);
//             if (i != t) {
//                 for (auto j : S) {
//                     if (j != s) {
//                         sum1 += y[i][j];
//                     }
//                 }
//             }
//         }

//         for (auto h : S) {
//             IloExpr sum3(env);
//             for (auto k : S) {
//                 if (h == k || h == s || h == t) {
//                     continue;
//                 }
//                 else {
//                     if (exist(C, k)) {
//                         sum3 += 1;
//                         for (int v : d) {
//                             sum3 -= theta[k][v];
//                         }
//                     }
//                 }
//             }

//             model.add(sum1 <= sum3);
//         }
//     }

//     // C4
//     for (int h : C) {
//         for (int v : d) {
//             IloExpr c4(env);
//             for (int j : c_t) {
//                 c4 += gamma[h][v][s][j];
//             }
//             model.add(c4 == omega[h][v][s]);
//         }
//     }

//     // C5
//     for (int h : C) {
//         for (int v : d) {
//             IloExpr c5(env);
//             for (int i : c_s) {
//                 c5 += gamma[h][v][i][t];
//             }
//             model.add(c5 == delta[h][v][t]);
//         }
//     }

//     // C6
//     for (int i : C) {
//         for (int h : C) {
//             if (i != h) {
//                 for (int v : d) {
//                     IloExpr c6s1(env), c6s2(env);
//                     for (int j : c_t) {
//                         c6s1 += gamma[h][v][i][j];
//                     }

//                     for (int j : c_s) {
//                         c6s2 += gamma[h][v][j][i];
//                     }
//                     model.add(c6s1 - c6s2 == omega[h][v][i] - delta[h][v][i]);
//                 }
//             }
//         }
//     }

//     // C7
//     for (int j : c_t) {
//         IloExpr c7s1(env);
//         for (int v : d) {
//             c7s1 += x[v][s][j];
//         }
//         model.add(y[s][j] + c7s1 <= 1);
//     }

//     // C8
//     for (int i : c_s) {
//         IloExpr c8s1(env);
//         for (int v : d) {
//             c8s1 += x[v][i][t];
//         }
//         model.add(y[i][t] + c8s1 <= 1);
//     }

//     // C9
//     for (int i : C) {
//         for (int j : C) {
//             if (i != j) {
//                 IloExpr c9s1(env);
//                 for (int v : d) {
//                     c9s1 += x[v][i][j] + x[v][j][i];
//                 }
//                 model.add(y[i][j] + c9s1 <= 1);
//             }
//         }
//     }

//     // C10
//     for (int h : C) {
//         IloExpr c10s1(env), c10s2(env);
//         for (int j : c_t) {
//             if (h != j) {
//                 c10s1 += y[h][j];
//             }
//         }
//         for (int v : d) {
//             c10s2 += theta[h][v];
//         }
//         model.add(c10s1 + c10s2 == 1);
//     }

//     // C11
//     for (int i : c_s) {
//         for (int j : c_t) {
//             if (i != j) {
//                 IloExpr c11s1(env);
//                 for (int h : C) {
//                     if (h != i && h != j) {
//                         for (int v : d) {
//                             c11s1 += gamma[h][v][i][j];
//                         }
//                     }
//                 }
//                 model.add(c11s1 <= y[i][j]);
//             }
//         }
//     }

//     // C12
//     for (int h : C) {
//         for (int v : d) {
//             IloExpr c12s1(env), c12s2(env);
//             for (int i : c_s) {
//                 if (i != h) {
//                     c12s1 += omega[h][v][i];
//                 }
//             }
//             for (int j : c_t) {
//                 if (j != h) {
//                     c12s2 += delta[h][v][j];
//                 }
//             }
//             model.add(c12s1 == theta[h][v]);
//             model.add(c12s2 == theta[h][v]);
//         }
//     }

//     // C13
//     for (int i : c_s) {
//         for (int j : c_t) {
//             if (i != j) {
//                 for (int v : d) {
//                     model.add(x[v][i][j] <= theta[i][v] + theta[j][v]);
//                 }
//             }
//         }
//     }

//     // C14
//     for (int i : c_s) {
//         for (int j : c_t) {
//             if (i != j) {
//                 for (int v : d) {
//                     model.add(x[v][i][j] <= omega[j][v][i] + delta[i][v][j]);
//                 }
//             }
//         }
//     }

//     // C15
//     for (int i : C) {
//         for (int v : d) {
//             IloExpr c15s1(env), c15s2(env);
//             for (int j : c_t) {
//                 if (i != j) {
//                     c15s1 += x[v][i][j];
//                 }
//             }
//             for (int h : C) {
//                 if (h != i) {
//                     c15s2 += omega[h][v][i];
//                 }
//             }
//             model.add(c15s1 == c15s2 + theta[i][v]);
//             model.add(c15s1 <= 1);
//             model.add(c15s2 + theta[i][v] <= 1);
//         }
//     }

//     // C16
//     for (int j : C) {
//         for (int v : d) {
//             IloExpr c16s1(env), c16s2(env);
//             for (int i : c_s) {
//                 if (i != j) {
//                     c16s1 += x[v][i][j];
//                 }
//             }
//             for (int h : C) {
//                 if (h != j) {
//                     c16s2 += delta[h][v][j];
//                 }
//             }
//             model.add(c16s1 == c16s2 + theta[j][v]);
//             model.add(c16s1 <= 1);
//             model.add(c16s2 + theta[j][v] <= 1);
//         }
//     }

//     // C18
//     for (int h : C) {
//         for (int v : d) {
//             IloExpr c17(env);
//             for (int i : c_s) {
//                 for (int j : c_t) {
//                     if (i != j) {
//                         c17 += gamma[h][v][i][j] * tau[i][j];
//                     }
//                 }
//             }
//             model.add(c17 <= (dtl - sr) * theta[h][v]);
//             IloExpr c18s1(env), c18s2(env);
//             for (int i : c_s) {
//                 if (i != h) {
//                     c18s1 += omega[h][v][i] * tau_prime[i][h];
//                 }
//             }
//             for (int j : c_t) {
//                 if (j != h) {
//                     c18s2 += delta[h][v][j] * tau_prime[h][j];
//                 }
//             }
//             model.add(c18s1 + c18s2 <= (dtl - sr) * theta[h][v]);
//         }
//     }

//     // Waiting time constraint
//     auto M = 1e5;
//     for (int h : instance->c_prime) {
//         for (int v : d) {
//             for (int j : c_t) {
//                 if (h != j) {
//                     IloExpr sum_d_launch(env), sum_d_rend(env), sum_truck(env);
//                     // model.add(w[j] <= )
//                 }
//             }
//         }
//     }
// }

// /*
//  * Main difference between TSP-D and FSTSP is the allowance of loops.
//  * That is, x^k_{ii} is permitted.
//  * With the incorporation of loops, how many stages will there be? Will it be K or more?
//  * */
// Result Solver::stage_based_rv_tsp_d(int n_thread, double dtl, double sl, double sr, bool use_tsp_as_warmstart) const {
//     try {
//         auto tau = instance->tau;
//         auto tau_prime = instance->tau_prime;
//         auto n = instance->num_node;
//         std::vector<int> C;
//         std::vector<int> V;
//         for (int i = 0; i < n + 1; i++) {
//             if (i != 0 && i != n) {
//                 C.push_back(i);
//             }
//             V.push_back(i);
//         }
//         std::cout << "Printing C': ";
//         setPrint(instance->c_prime);
//         C.shrink_to_fit();
//         V.shrink_to_fit();
//         IloEnv env;
//         IloModel model(env);
//         IloCplex cplex(model);
//         cplex.setParam(IloCplex::Param::MIP::Tolerances::Integrality, 0);
//         auto O = 0;
//         auto D = n;
//         int K = n + 1;
//         auto K_arc = K - 1;

//         // Variable declaration
//         // X^i_k (binary variable) và nhận giá trị một tương ứng với đỉnh thứ k của
//         //đường đi của vehicle là i; k \in 1..n;
//         IloArray<IloBoolVarArray> X(env, K + 1);
//         for (int k = 1; k <= K; k++) {
//             X[k] = IloBoolVarArray(env, D + 1);
//             for (int i = 0; i <= D; i++) {
//                 X[k][i] = IloBoolVar(env);
//                 model.add(X[k][i]);
//                 auto v_name = "X_" + std::to_string(k) + "_" + std::to_string(i);
//                 X[k][i].setName(v_name.c_str());
//             }
//             if (k > 1) {
//                 model.add(X[k][0] == 0);
//             }
//         }

//         model.add(X[1][0] == 1).setName("First stage must be source depot");

//         // x^k_(ij) (binary variable) và nhận giá trị một nếu Xk
//         // mô ta cạnh nối 2 đỉnh liên tiếp trên đường đi.
//         IloArray<IloArray<IloBoolVarArray>> x(env, K_arc + 1);
//         for (int k = 1; k <= K_arc; k++) {
//             x[k] = IloArray<IloBoolVarArray>(env, D);
//             for (int i = 0; i < D; i++) {
//                 x[k][i] = IloBoolVarArray(env, D + 1);
//                 for (int j = 1; j <= D; j++) {
//                     x[k][i][j] = IloBoolVar(env);
//                     auto v_name = "x_" + std::to_string(k) + "_" + std::to_string(i) + "_" + std::to_string(j);
//                     x[k][i][j].setName(v_name.c_str());
//                 }
//             }
//         }

//         //// phi^h equals to 1 if customer h is served by the drone
//         IloBoolVarArray phi(env, n);
//         for (int h : C) {
//             phi[h] = IloBoolVar(env);
//             auto v_name = "phi_" + std::to_string(h);
//             phi[h].setName(v_name.c_str());
//             if (exist(instance->heavy, h)) {
//                 model.add(phi[h] == 0);
//             }
//         }

//         IloArray<IloArray<IloBoolVarArray>> Y(env, K + 1), W(env, K + 1);
//         for (int k = 1; k <= K; k++) {
//             Y[k] = IloArray<IloBoolVarArray>(env, D + 1);
//             W[k] = IloArray<IloBoolVarArray>(env, D + 1);

//             for (int i = 0; i <= D; i++) {
//                 Y[k][i] = IloBoolVarArray(env, C.size() + 1);
//                 W[k][i] = IloBoolVarArray(env, C.size() + 1);
//                 for (int h : C) {
//                     if (i != h) {
//                         Y[k][i][h] = IloBoolVar(env);
//                         Y[k][i][h].setName(("Y_" + std::to_string(k) + "_"
//                             + std::to_string(i) + "_" + std::to_string(h)).c_str());

//                         W[k][i][h] = IloBoolVar(env);
//                         W[k][i][h].setName(("W_" + std::to_string(k) + "_"
//                             + std::to_string(i) + "_" + std::to_string(h)).c_str());

//                         if (i == 0 && k > 1) {
//                             model.add(Y[k][i][h] == 0);
//                         }
//                         if (i == D && k == 1) {
//                             model.add(W[k][i][h] == 0);
//                         }
//                         if (tau_prime[i][h] > dtl - sr) {
//                             model.add(Y[k][i][h] == 0);
//                         }
//                         if (tau_prime[h][i] > dtl - sr) {
//                             model.add(W[k][i][h] == 0);
//                         }
//                     }
//                 }
//             }
//         }

//         // arrival\departure variables a and d.
//         IloNumVarArray a(env, K + 1);
//         IloNumVarArray d(env, K + 1);
//         for (int k = 1; k <= K; k++) {
//             a[k] = IloNumVar(env, 0, IloInfinity, ILOFLOAT);
//             auto v_name = "a_" + std::to_string(k);
//             a[k].setName(v_name.c_str());
//             d[k] = IloNumVar(env, 0, IloInfinity, ILOFLOAT);
//             v_name = "d_" + std::to_string(k);
//             d[k].setName(v_name.c_str());
//             model.add(d[k] >= a[k]).setName(("C13_" + std::to_string(k)).c_str());
//         }

//         model.add(a[1] == 0).setName("arrival to depot at time 0");
//         model.add(d[1] == 0).setName("depart from depot at time 0");;

//         // $R_{k} = \sum_{k'}Z_{kk'}$: các đoạn bắt đầu từ k (C23)
//         IloBoolVarArray R(env, K + 1);
//         for (int k = 1; k < K; k++) {
//             R[k].setName(("R_" + std::to_string(k)).c_str());
//         }

//         //// aux var Z_{k, k_p, h}: sortie launch from k and rendezvous at k_p.
//         IloArray<IloArray<IloBoolVarArray>> Z(env, K);
//         for (int h : C) {
//             Z[h] = IloArray<IloBoolVarArray>(env, K + 1);
//             for (int k = 1; k <= K - 1; k++) {
//                 Z[h][k] = IloBoolVarArray(env, K + 1);
//                 for (int k_p = k + 1; k_p <= K; k_p++) {
//                     Z[h][k][k_p] = IloBoolVar(env);
//                     auto v_name = "Z_" + std::to_string(h) + "_" + std::to_string(k) + "_" + std::to_string(k_p);
//                     Z[h][k][k_p].setName(v_name.c_str());
//                 }
//             }
//         }
//         //// aux var z_{k, k_p}: sortie launch from k and rendezvous at k_p.
//         IloArray<IloBoolVarArray> z(env, K);
//         for (int k = 1; k <= K - 1; k++) {
//             z[k] = IloBoolVarArray(env, K + 1);
//             for (int k_p = k + 1; k_p <= K; k_p++) {
//                 z[k][k_p] = IloBoolVar(env);
//                 auto v_name = "z_" + std::to_string(k) + "_" + std::to_string(k_p);
//                 z[k][k_p].setName(v_name.c_str());
//             }
//         }

//         ////-----------------------------------------------------------------------------------------------
//         // WARMSTART CONFIG: using original TSP solution.
//         // Obtained with MTZ formulation.
//         // On variable X.
//         // if (use_tsp_as_warmstart) {
//         //     auto tsp_solution = TSP_MTZ(tau);
//         //     std::cout << tsp_solution.getSize() << std::endl;
//         //     IloNumVarArray X_warm_var(env);
//         //     IloNumArray X_warm_val(env);
//         //     for (int k = 1; k <= K - 1; k++) {
//         //         for (int i = 0; i <= D; i++) {
//         //             X_warm_var.add(X[k][i]);
//         //             if (tsp_solution[k - 1] == i) {
//         //                 X_warm_val.add(true);
//         //                 std::cout << "Warmstart X[" << k << "][" << i << "] == 1" << std::endl;
//         //             }
//         //             else {
//         //                 X_warm_val.add(false);
//         //             }
//         //         }
//         //     }
//         //     cplex.addMIPStart(X_warm_var, X_warm_val);
//         //     X_warm_var.end();
//         //     X_warm_val.end();
//         // }
//         ////-----------------------------------------------------------------------------------------------

//         //// Sortie maximum stage gap calculation.
//         /// Find maximum stage gap that a sortie can start and rendezvous.
//         /// For each stage gap in increasing order (k' - k), find the minimum tour length from stage k to k'.
//         /// Revisit(s) are also considered.
//         /// If the tour length > dtl - sr => there can be no sortie with stage gap greater or equal k' - k.
//         bool used_stage_gap = false;
//         int min_stage_gap = 0;
//         std::cout << "---------------------------------------------" << std::endl;
//         std::cout << "Calculate max stage gap for sortie.... " << std::endl;
//         for (int k = 1; k < K; k++) {
//             for (int kp = k + 1; kp <= K; kp++) {
//                 if (kp > k + 6) {
//                     goto after_z_cons;
//                 }
//                 double smallest_tour = smallest_tour_length(kp - k, tau, V);
//                 if (smallest_tour > dtl - sr) {
//                     std::cout << "Stage gap of " << kp - k << " with smallest length = " << smallest_tour <<
//                         " violated endurance constraint!" << std::endl;
//                     std::cout << "---------------------------------------------" << std::endl;
//                     min_stage_gap = kp - k;
//                     goto after_z_cons;
//                 }
//             }
//         }
//     after_z_cons:
//         if (min_stage_gap == 0) {
//             std::cout << "Stage gap calculation consumes too much memory. No constraint was added." << std::endl;
//             std::cout << "---------------------------------------------" << std::endl;
//         }
//         if (min_stage_gap != 0) {
//             used_stage_gap = true;
//             for (int k = 1; k < K; k++) {
//                 for (int kp = k + min_stage_gap; kp <= K; kp++) {
//                     model.add(z[k][kp] == 0);
//                     std::cout << "Variable z[" << k << "][" << kp << "] was set to 0." << std::endl;
//                     for (int h : instance->c_prime) {
//                         model.add(Z[h][k][kp] == 0);
//                         IloExpr start_stage_k(env), end_stage_kp(env);
//                         for (int i = 0; i < D; i++) {
//                             if (i != h) {
//                                 start_stage_k += Y[k][i][h];
//                                 // model.add(Z[h][k][kp] + Y[k][i][h] <= 1);
//                             }
//                         }
//                         for (int j = 1; j <= D; j++) {
//                             if (j != h) {
//                                 end_stage_kp += W[kp][j][h];
//                                 // model.add(Z[h][k][kp] + W[kp][j][h] <= 1);
//                             }
//                         }
//                         std::string cname =
//                             "Can't serve customer " + std::to_string(h) + " start at stage " + std::to_string(k) +
//                             ", end at stage " + std::to_string(kp);
//                         // Looks like quicker lower bound improvements with this constraints.
//                         model.add(start_stage_k + end_stage_kp <= 1).setName(cname.c_str());
//                     }
//                 }
//             }
//         }

//         ////-----------------------------------------------------------------------------------------------

//         // CONSTRAINTS DECLARATION.------------------------------------------------------------------------
//         // Constraint C1
//         for (int k = 1; k < K; k++) {
//             for (int i = 0; i < D; i++) {
//                 IloExpr sum(env);
//                 for (int j = 1; j <= D; j++) {
//                     sum += x[k][i][j];
//                 }
//                 model.add(X[k][i] == sum).setName(("C1_" + std::to_string(k) + "_" + std::to_string(i)).c_str());
//             }
//         }

//         for (int k = 2; k <= K; k++) {
//             for (int i = 1; i <= D; i++) {
//                 IloExpr sum(env);
//                 for (int j = 0; j < D; j++) {
//                     sum += x[k - 1][j][i];
//                 }
//                 // arcs entering i at stage k.
//                 model.add(X[k][i] == sum).setName(("C1p_" + std::to_string(k) + "_" + std::to_string(i)).c_str());
//             }
//         }

//         // C2: truck must depart from the depot at stage 1.
//         IloExpr C2(env);
//         for (int i = 1; i <= D; i++) {
//             C2 += x[1][O][i];
//         }

//         IloConstraint c2(C2 == 1);
//         model.add(c2).setName("Must start from source");

//         // C3: Terminal depot must be arrived only once.
//         IloExpr C3(env);
//         for (int k = 2; k <= K; k++) {
//             C3 += X[k][D];
//         }
//         model.add(C3 == 1).setName("Must visit terminal depot once");

//         // At most ONE node each stage.
//         for (int k = 1; k <= K; k++) {
//             IloExpr sum(env);
//             for (int i = 0; i <= D; i++)
//                 sum += X[k][i];
//             model.add(sum <= 1).setName(("C20_at_most_one_customer_at_stage_" + std::to_string(k)).c_str());
//         }

//         // $Z_{kk'} = \sum_{h}Z^h_{kk'}$: mỗi cặp (k,k') chỉ phục vụ tối đa một khách hàng (C22)
//         for (int k = 1; k < K; k++) {
//             for (int k_p = k + 1; k_p <= K; k_p++) {
//                 IloExpr expr(env);
//                 for (int h : C) {
//                     expr += Z[h][k][k_p];
//                 }
//                 model.add(expr == z[k][k_p]).setName(("C22_" + std::to_string(k) + "_" + std::to_string(k_p)).c_str());
//             }
//         }

//         //C20:$\sum_{k'>k}Z_{kk'} = \sum_{i,h}Y^k_{ih}$ : với mỗi $k$,
//         //ràng buộc liên kết drone đi ra từ stage $k$ và đoạn mà oto di chuyển không có drone. (C20)
//         for (int h : C) {
//             for (int k = 1; k <= K - 1; k++) {
//                 IloExpr expr(env);
//                 for (int k_p = k + 1; k_p <= K; k_p++) {
//                     expr += Z[h][k][k_p];
//                 }

//                 for (int i = 0; i < D; i++) {
//                     if (i != h && tau_prime[i][h] <= dtl - sr) {
//                         expr -= Y[k][i][h];
//                     }
//                 }
//                 model.add(expr == 0).setName(("C20_" + std::to_string(k) + "_" + std::to_string(h)).c_str());
//             }
//         }

//         for (int h : C) {
//             for (int k_p = 2; k_p <= K; k_p++) {
//                 IloExpr expr(env);
//                 for (int k = 1; k < k_p; k++) {
//                     expr += Z[h][k][k_p];
//                 }

//                 for (int i = 1; i <= D; i++) {
//                     if (i != h && tau_prime[h][i] <= dtl - sr) {
//                         expr -= W[k_p][i][h];
//                     }
//                 }
//                 model.add(expr == 0).setName(("C20p_" + std::to_string(k_p)
//                     + "_" + std::to_string(h)).c_str());
//             }
//         }

//         for (int k = 1; k < K; k++) {
//             IloExpr expr(env);
//             for (int k_p = k + 1; k_p <= K; k_p++) {
//                 expr += z[k][k_p];
//             }
//             model.add(R[k] == expr).setName(("C23_" + std::to_string(k)).c_str());
//         }

//         // modified C7
//         for (int k = 1; k <= K - 1; k++) {
//             for (int k_p = k + 1; k_p <= K; k_p++) {
//                 for (int l = k + 1; l < k_p; l++) {
//                     // tranh drone bay cac doan giao nhau.
//                     if (k < l) {
//                         model.add(z[k][k_p] + R[l] <= 1).setName(("C7m_" + std::to_string(k)
//                                 + "_" + std::to_string(k_p) + "_" + std::to_string(l))
//                             .c_str());
//                     }
//                 }
//             }
//         }
//         //// C17 - $X^k_i \geq \sum_h X^k_{ih}$ (C17) - chỉ bay drone ở nơi mà xe ở đó
//         for (int k = 1; k <= K; k++) {
//             for (int i = 0; i <= D; i++) {
//                 IloExpr expr(env);
//                 for (int h : C) {
//                     if (h != i) {
//                         if (tau_prime[i][h] <= dtl - sr) {
//                             expr += Y[k][i][h];
//                         }
//                     }
//                 }
//                 model.add(expr <= X[k][i]).setName(("C17_" + std::to_string(k) + "_" + std::to_string(i)).c_str());
//             }
//         }

//         // $X^k_i \geq \sum_h Y^k_{ih}$ (C17p) : chỉ bay drone tới nơi mà xe ở đó
//         for (int k = 1; k <= K; k++) {
//             for (int i = 0; i <= D; i++) {
//                 IloExpr expr(env);
//                 for (int h : C) {
//                     if (h != i) {
//                         if (tau_prime[h][i] <= dtl - sr) {
//                             expr += W[k][i][h];
//                         }
//                     }
//                 }
//                 model.add(expr <= X[k][i]).setName(("C17p_" + std::to_string(k) + "_" + std::to_string(i)).c_str());
//             }
//         }

//         // $\sum_{i} X^k_{ih}\tau^D_{ih} + \sum_{i} Y^{k'}_{ih}\tau^D_{hi} \leq D_d$
//         //- drone duration constraint cho mỗi $h$ (C19)
//         for (int h : C) {
//             IloExpr expr(env);

//             for (int k = 1; k <= K; k++) {
//                 for (int i = 0; i <= D; i++) {
//                     if (i != h && i != D && tau_prime[i][h] <= dtl - sr) {
//                         expr += Y[k][i][h] * tau_prime[i][h];
//                     }

//                     if (i != h && i != 0 && tau_prime[h][i] <= dtl - sr) {
//                         expr += W[k][i][h] * tau_prime[h][i];
//                     }
//                 }
//             }
//             model.add(expr <= (dtl - sr) * phi[h]).setName(("C19_" + std::to_string(h)).c_str());
//         }

//         // modified C7p - we can select at most one segment that contains the point l
//         for (int l = 2; l < K; l++) {
//             IloExpr expr(env);

//             for (int k = 1; k < l; k++)
//                 for (int k_p = l; k_p <= K; k_p++)
//                     expr += z[k][k_p];
//             model.add(expr <= 1).setName(("C7mm_" + std::to_string(l)).c_str());
//         }

//         for (int i = 0; i < D; i++) {
//             for (int k = 1; k <= K - 1; k++) {
//                 IloExpr lhs(env);

//                 for (int h : C) {
//                     if (i != h) {
//                         lhs += Y[k][i][h];
//                     }
//                 }
//                 model.add(lhs <= X[k][i]).setName(("C8_launch_" + std::to_string(i) + "_" + std::to_string(k)).c_str());
//             }
//         }

//         for (int j = 1; j <= D; j++) {
//             for (int k_p = 2; k_p <= K; k_p++) {
//                 IloExpr lhs(env);

//                 for (int h : C) {
//                     if (h != j) {
//                         lhs += W[k_p][j][h];
//                     }
//                 }

//                 model.add(lhs <= X[k_p][j]).setName(
//                     ("C8_rendezvous_" + std::to_string(j) + "_" + std::to_string(k_p)).c_str());
//             }
//         }

//         // $\phi_h = \sum_{k,i}X^k_{ih} = \sum_{k,i}Y^k_{ih}$
//         // - chỉ có duy nhất một điểm xuất phát và môt điểm đích cho
//         //mỗi khách hàng $h$ được phục vụ bởi drone (C18)
//         for (int h : C) {
//             IloExpr rhs(env);
//             for (int i = 0; i < D; i++) {
//                 if (i != h && tau_prime[i][h] <= dtl - sr) {
//                     for (int k = 1; k <= K - 1; k++) {
//                         rhs += Y[k][i][h];
//                     }
//                 }
//             }
//             // consistency constraint cho sortie phuc vu h.
//             model.add(phi[h] == rhs).setName(("C18_" + std::to_string(h)).c_str());
//         }

//         for (int h : C) {
//             IloExpr rhs(env);
//             for (int i = 1; i <= D; i++) {
//                 if (i != h && tau_prime[h][i] <= dtl - sr) {
//                     for (int k = 2; k <= K; k++) {
//                         rhs += W[k][i][h];
//                     }
//                 }
//             }
//             // consistency constraint cho sortie phuc vu h.
//             model.add(phi[h] == rhs).setName(("C18p_" + std::to_string(h)).c_str());
//         }

//         // Assignment constraint
//         //////////// C10: node_stage
//         for (int h : C) {
//             IloExpr sum_k(env);
//             for (int k = 2; k < K; k++) {
//                 sum_k += X[k][h];
//             }
//             // phuc vu h it nhat 1 lan.
//             model.add(phi[h] + sum_k >= 1).setName(("C10_" + std::to_string(h)).c_str());
//         }

//         /////////// C14: node_stage
//         for (int k = 1; k <= K_arc; k++) {
//             IloExpr sum(env);
//             for (int i = 0; i < D; i++) {
//                 for (int j = 1; j <= D; j++) {
//                     sum += x[k][i][j] * tau[i][j];
//                 }
//             }

//             model.add(a[k + 1] == d[k] + sum).setName(
//                 ("C14_" + std::to_string(k) + "_" + std::to_string(k + 1)).c_str());
//         }

//         ////////// C15: node_stage
//         ///// big M calculation
//         double M = 0;
//         for (int i = 0; i < n; i++) {
//             for (int j = 0; j < n; j++) {
//                 if (i < j) {
//                     M += tau[i][j];
//                 }
//             }
//         }

//         for (int k = 1; k <= K - 1; k++) {
//             for (int k_p = k + 1; k_p <= K; k_p++) {
//                 if (k < k_p) {
//                     model.add(a[k_p] - d[k] <= z[k][k_p] * (dtl - sr) + (1 - z[k][k_p]) * M).setName(
//                         ("C15_" + std::to_string(k) + "_" + std::to_string(k_p)).c_str());
//                 }
//             }
//         }

//         for (int k = 1; k <= K - 1; k++) {
//             for (int k_p = k + 1; k_p <= K; k_p++) {
//                 if (k < k_p) {
//                     IloExpr sum_length_y_k(env);
//                     IloExpr sum_length_w_k_p(env);
//                     IloExpr sum_w_k_p(env);
//                     IloExpr sum_y_k_p(env);

//                     for (int i = 0; i < D; i++) {
//                         for (int h : C) {
//                             if (i != h) {
//                                 sum_length_y_k += Y[k][i][h] * tau_prime[i][h];
//                             }
//                         }
//                     }
//                     for (int i = 0; i < D; i++) {
//                         for (int h : C) {
//                             if (i != h) {
//                                 sum_y_k_p += Y[k_p][i][h] * sl;
//                             }
//                         }
//                     }
//                     for (int j = 1; j <= D; j++) {
//                         for (int h : C) {
//                             if (h != j) {
//                                 sum_length_w_k_p += W[k_p][j][h] * tau_prime[h][j];
//                             }
//                         }
//                     }

//                     for (int j = 1; j <= D; j++) {
//                         for (int h : C) {
//                             if (h != j) {
//                                 sum_w_k_p += W[k_p][j][h] * sr;
//                             }
//                         }
//                     }

//                     model.add(
//                         d[k_p] >=
//                         d[k] + sum_length_y_k + sum_length_w_k_p + sum_y_k_p + sum_w_k_p - (1 - z[k][k_p]) * M);
//                 }
//             }
//         }

//         for (int k = 2; k <= K; k++) {
//             IloExpr sum(env);
//             IloExpr sum_w_K(env);
//             IloExpr sum_y_K(env);
//             for (int i = 0; i < D; i++) {
//                 for (int j = 1; j <= D; j++) {
//                     sum += x[k - 1][i][j] * tau[i][j];
//                 }
//             }
//             for (int i = 0; i < D; i++) {
//                 for (int h : C) {
//                     if (i != h && k != K) {
//                         sum_y_K += sl * Y[k][i][h];
//                     }
//                 }
//             }

//             for (int i = 1; i <= D; i++) {
//                 for (int h : C) {
//                     if (i != h) {
//                         sum_w_K += sr * W[k][i][h];
//                     }
//                 }
//             }
//             auto constr_name = "CC_d_" + std::to_string(k) + "_constr";
//             model.add(d[k] >= d[k - 1] + sum + sum_w_K + sum_y_K).setName(constr_name.c_str());
//         }

//         // SLOW
//         // (k,h): k chỉ có thể là đến hoặc đi của khách hàng h.
//         //    for (int h: instance->c_prime) {
//         //        for (int i: C) {
//         //            if (i != h) {
//         //                for (int k = 1; k < K; k++) {
//         //                    for (int kp = k + 1; kp < K; kp++) {
//         //                        model.add(Y[k][i][h] + W[kp][i][h] <= phi[h]);
//         //                    }
//         //                }
//         //                // SLOW VARIANT.
//         //            }
//         //        }
//         //    }

//         // LB constraint
//         IloExpr lb_truck_tour(env);
//         for (int k = 1; k < K; k++) {
//             for (int i = 0; i < D; i++) {
//                 for (int j = 1; j <= D; j++) {
//                     lb_truck_tour += x[k][i][j] * tau[i][j];
//                 }
//             }
//         }
//         model.add(d[K] >= lb_truck_tour).setName("Lower_bound_obj");

//         IloExpr lb_drone_tour(env);
//         for (int k = 1; k < K; k++) {
//             for (int i = 0; i < D; i++) {
//                 for (int h : C) {
//                     if (i != h) {
//                         lb_drone_tour += Y[k][i][h] * tau_prime[i][h];
//                     }
//                 }
//             }
//         }
//         for (int k = 2; k <= K; k++) {
//             for (int j = 1; j <= D; j++) {
//                 for (int h : C) {
//                     if (j != h) {
//                         lb_drone_tour += W[k][j][h] * tau_prime[h][j];
//                     }
//                 }
//             }
//         }
//         model.add(d[K] >= lb_drone_tour);

//         //    for (int k = 2; k < K / 2; k++) {
//         //        model.add(X[k][D] == 0);
//         //    }

//         double bestObjective = IloInfinity;
//         // BranchAndCutCallback bc(env, model, X, x, phi, Z, z, Y, W, K, D, bestObjective, C, tau, tau_prime);
//         // cplex.use(&bc);
//         model.add(IloMinimize(env, d[K]));
//         cplex.exportModel("cplex_model_1.lp");
//         double obj = 0;
//         double revisit_count = 0;
//         // Solve the model
//         std::vector<int> rev(n + 1, 0);
//         auto startTime = std::chrono::high_resolution_clock::now();
//         cplex.solve();
//         auto endTime = std::chrono::high_resolution_clock::now();
//         // Calculate the elapsed time
//         auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
//         // Check if the problem is infeasible

//         if (cplex.getStatus() == IloAlgorithm::Infeasible) {
//             // Handle infeasibility
//             std::cout << "The problem is infeasible." << std::endl;
//             std::cout << "Infeasibility at: " << cplex.getInfeasibility(c2) << std::endl;
//             // You can also retrieve the infeasible constraints using cplex.getInfeasibility() method
//         }

//         std::cout << "Feasible solution found!" << std::endl;
//         std::cout << "Truck nodes:" << std::endl;
//         for (int k = 1; k <= K; k++) {
//             for (int i = 0; i <= D; i++) {
//                 auto X_val = cplex.getValue(X[k][i]);
//                 if (X_val == 1) {
//                     rev[i]++;
//                     auto d_k = cplex.getValue(d[k]);
//                     auto a_k = cplex.getValue(a[k]);
//                     std::cout << "(" << a_k << ")-" << i << "(" << k << ")-(" << d_k << ")----->";
//                     break;
//                 }
//             }
//         }
//         std::cout << std::endl << "Truck arcs:" << std::endl;
//         std::map<int, std::pair<int, int>> map_stage_truck_arc;
//         for (int k = 1; k <= K_arc; k++) {
//             for (int i = 0; i < D; i++) {
//                 for (int j = 1; j <= D; j++) {
//                     auto X_val = cplex.getValue(x[k][i][j]);
//                     if (X_val == 1) {
//                         std::cout << "Arc " << k << " connecting " << i << " and " << j
//                             << " with cost " << tau[i][j] << " " << std::endl;
//                         obj += tau[i][j];
//                         map_stage_truck_arc[k] = std::make_pair(i, j);
//                         break;
//                     }
//                 }
//             }
//         }
//         int phi_check = 0;

//         for (int h : instance->c_prime) {
//             if (cplex.getValue(phi[h]) == 1) {
//                 std::cout << "Customer " << h << " was served by drone!" << std::endl;
//                 phi_check += 1;
//             }
//         }

//         std::cout << "Sorties information: ";
//         if (phi_check == 0) {
//             std::cout << "No sortie!" << std::endl;
//         }
//         else {
//             std::cout << phi_check << " sortie(s)." << std::endl;
//         }
//         for (int k = 1; k < K; k++) {
//             for (int kp = k + 1; kp <= K; kp++) {
//                 for (int h : instance->c_prime) {
//                     if (cplex.getValue(Z[h][k][kp]) == 1) {
//                         int start_node = -1, end_node = -1;
//                         for (int i = 0; i <= D; i++) {
//                             if (i != h) {
//                                 try {
//                                     auto Y_val = cplex.getValue(Y[k][i][h]);
//                                     if (Y_val == 1) {
//                                         start_node = i;
//                                     }
//                                     auto W_val = cplex.getValue(W[kp][i][h]);
//                                     if (W_val == 1) {
//                                         end_node = i;
//                                     }
//                                 }
//                                 catch (...) {}
//                             }
//                         }
//                         // Print.
//                         std::cout <<
//                             "-------------------------------------------------------------------------------------"
//                             << std::endl;
//                         std::cout << "Start node/Stage -------" << "Customer" << "-------" << "End node/Stage" <<
//                             std::endl;
//                         std::cout << "      " << start_node << "/" << k << "--------------" << h << "--------------" <<
//                             end_node << "/" << kp << std::endl;
//                         std::cout << "Sortie starts from stage " << k << " at node " << start_node << std::endl;
//                         std::cout << "Served customer " << h << std::endl;
//                         std::cout << "Sortie ends at stage " << kp << " at node " << end_node << std::endl;

//                         ////////////////////////////////////////////////////////////////////////////
//                         double truck_travel_time = 0;
//                         double drone_travel_time = tau_prime[start_node][h] + tau_prime[h][end_node];

//                         std::cout << std::endl;
//                         for (int k_start = k; k_start <= kp - 1; k_start++) {
//                             truck_travel_time += tau[map_stage_truck_arc[k_start].first][map_stage_truck_arc[k_start].
//                                 second];
//                         }
//                         auto drone_arrival_time = cplex.getValue(d[k]) + drone_travel_time;
//                         auto vehicle_departure_time = cplex.getValue(d[kp]);
//                         auto truck_arrival_time = cplex.getValue(a[kp]);
//                         std::cout << "Time information of this sortie:" << std::endl;
//                         std::cout << "Truck departure time from stage k = " << k << " is: " << cplex.getValue(d[k])
//                             << std::endl;
//                         std::cout << "Truck arcs during this sortie: ";
//                         for (int k_start = k; k_start <= kp; k_start++) {
//                             if (k_start == kp) {
//                                 std::cout << map_stage_truck_arc[k_start].first;
//                             }
//                             else {
//                                 std::cout << map_stage_truck_arc[k_start].first << "--->";
//                             }
//                         }
//                         std::cout << ". Total truck time = " << truck_travel_time << std::endl;
//                         std::cout << "Drone arcs: " << start_node << "--->" << h << "--->" << end_node << ". " <<
//                             "Total drone time: " << drone_travel_time << std::endl;
//                         std::cout << "Truck arrival time at k' = " << kp << " is: " << truck_arrival_time << std::endl;
//                         std::cout << "Drone arrival time at k' = " << kp << " is: " << drone_arrival_time << std::endl;
//                         std::cout << "Truck departure time at k' = " << cplex.getValue(d[kp]) << std::endl;
//                         std::cout <<
//                             "-------------------------------------------------------------------------------------"
//                             << std::endl;
//                     }
//                 }
//             }
//         }

//         std::cout << "Done!" << std::endl;
//         // std::cout << "-------------------------Re-calculated objective-----------------------" << std::endl;
//         // std::cout << obj << std::endl;
//         // std::cout << "------------------------------------------------------------------------" << std::endl;

//         double c = cplex.getObjValue();
//         std::cout << "Solve time: " << duration.count() / 1000.0 << std::endl;
//         cplex.end();
//         model.end();
//         env.end();
//         for (int i = 0; i < rev.size(); i++) {
//             if (rev[i] > 1) {
//                 revisit_count += rev[i] - 1;
//                 std::cout << "Node " << i << " was revisited " << rev[i] - 1 << " times!" << std::endl;
//             }
//         }

//         std::cout << "OBJECTIVE VALUE: " << c << ", NUMBER OF SORTIES: " << phi_check << "." << std::endl;
//         std::cout << "Number of revisit: " << revisit_count << std::endl;
//         return Result{c, duration.count() / 1000.0, 0};
//     }
//     catch (IloException& e) {
//         std::cout << "Exception: " << e.getMessage() << std::endl;
//     }
// }

// Result
// Solver::stage_based_no_rv_tsp_d(int n_thread, double dtl, double sl, double sr, bool use_tsp_as_warmstart) const {
//     auto tau = instance->tau;
//     auto tau_prime = instance->tau_prime;
//     auto n = instance->num_node;
//     std::vector<int> C;
//     std::vector<int> V;
//     for (int i = 0; i < n + 1; i++) {
//         if (i != 0 && i != n) {
//             C.push_back(i);
//         }
//         V.push_back(i);
//     }

//     IloEnv env;
//     IloModel model(env);
//     IloCplex cplex(model);
//     cplex.setParam(IloCplex::Param::MIP::Tolerances::Integrality, 0);
//     auto O = 0;
//     auto D = n;
//     int K = n + 1;
//     auto K_arc = K - 1;

//     // Variable declaration
//     // X^i_k (binary variable) và nhận giá trị một tương ứng với đỉnh thứ k của
//     //đường đi của vehicle là i; k \in 1..n;
//     IloArray<IloBoolVarArray> X(env, K + 1);
//     for (int k = 1; k <= K; k++) {
//         X[k] = IloBoolVarArray(env, D + 1);
//         for (int i = 0; i <= D; i++) {
//             X[k][i] = IloBoolVar(env);
//             model.add(X[k][i]);
//             auto v_name = "X_" + std::to_string(k) + "_" + std::to_string(i);
//             X[k][i].setName(v_name.c_str());
//         }
//         if (k > 1) {
//             model.add(X[k][0] == 0);
//         }
//     }

//     model.add(X[1][0] == 1).setName("First stage must be source depot");

//     // x^k_(ij) (binary variable) và nhận giá trị một nếu Xk
//     // mô ta cạnh nối 2 đỉnh liên tiếp trên đường đi.
//     IloArray<IloArray<IloBoolVarArray>> x(env, K_arc + 1);
//     for (int k = 1; k <= K_arc; k++) {
//         x[k] = IloArray<IloBoolVarArray>(env, D);
//         for (int i = 0; i < D; i++) {
//             x[k][i] = IloBoolVarArray(env, D + 1);
//             for (int j = 1; j <= D; j++) {
//                 x[k][i][j] = IloBoolVar(env);
//                 auto v_name = "x_" + std::to_string(k) + "_" + std::to_string(i) + "_" + std::to_string(j);
//                 x[k][i][j].setName(v_name.c_str());
//             }
//         }
//     }

//     //// phi^h equals to 1 if customer h is served by the drone
//     IloBoolVarArray phi(env, n);
//     for (int h : C) {
//         phi[h] = IloBoolVar(env);
//         auto v_name = "phi_" + std::to_string(h);
//         phi[h].setName(v_name.c_str());
//     }

//     for (int heavy : instance->heavy) {
//         if (heavy != D) {
//             model.add(phi[heavy] == 0);
//         }
//     }

//     IloArray<IloArray<IloBoolVarArray>> Y(env, K + 1), W(env, K + 1);
//     for (int k = 1; k <= K; k++) {
//         Y[k] = IloArray<IloBoolVarArray>(env, D + 1);
//         W[k] = IloArray<IloBoolVarArray>(env, D + 1);

//         for (int i = 0; i <= D; i++) {
//             Y[k][i] = IloBoolVarArray(env, C.size() + 1);
//             W[k][i] = IloBoolVarArray(env, C.size() + 1);
//             for (int h : C) {
//                 if (i != h) {
//                     Y[k][i][h] = IloBoolVar(env);
//                     Y[k][i][h].setName(("Y_" + std::to_string(k) + "_"
//                         + std::to_string(i) + "_" + std::to_string(h)).c_str());

//                     W[k][i][h] = IloBoolVar(env);
//                     W[k][i][h].setName(("W_" + std::to_string(k) + "_"
//                         + std::to_string(i) + "_" + std::to_string(h)).c_str());

//                     if (i == 0 && k > 1) {
//                         model.add(Y[k][i][h] == 0);
//                     }
//                     if (i == D && k == 1) {
//                         model.add(W[k][i][h] == 0);
//                     }
//                     if (tau_prime[i][h] > dtl - sr) {
//                         model.add(Y[k][i][h] == 0);
//                     }
//                     if (tau_prime[h][i] > dtl - sr) {
//                         model.add(W[k][i][h] == 0);
//                     }
//                 }
//             }
//         }
//     }

//     // arrival\departure variables a and d.
//     IloNumVarArray a(env, K + 1);
//     IloNumVarArray d(env, K + 1);
//     for (int k = 1; k <= K; k++) {
//         a[k] = IloNumVar(env, 0, IloInfinity, ILOFLOAT);
//         auto v_name = "a_" + std::to_string(k);
//         a[k].setName(v_name.c_str());
//         d[k] = IloNumVar(env, 0, IloInfinity, ILOFLOAT);
//         v_name = "d_" + std::to_string(k);
//         d[k].setName(v_name.c_str());
//         model.add(d[k] >= a[k]).setName(("C13_" + std::to_string(k)).c_str());
//     }

//     model.add(a[1] == 0).setName("arrival to depot at time 0");
//     model.add(d[1] == 0).setName("depart from depot at time 0");;

//     // $R_{k} = \sum_{k'}Z_{kk'}$: các đoạn bắt đầu từ k (C23)
//     IloBoolVarArray R(env, K + 1);
//     for (int k = 1; k < K; k++) {
//         R[k].setName(("R_" + std::to_string(k)).c_str());
//     }

//     //// aux var Z_{k, k_p, h}: sortie launch from k and rendezvous at k_p.
//     IloArray<IloArray<IloBoolVarArray>> Z(env, K);
//     for (int h : C) {
//         Z[h] = IloArray<IloBoolVarArray>(env, K + 1);
//         for (int k = 1; k <= K - 1; k++) {
//             Z[h][k] = IloBoolVarArray(env, K + 1);
//             for (int k_p = k + 1; k_p <= K; k_p++) {
//                 Z[h][k][k_p] = IloBoolVar(env);
//                 auto v_name = "Z_" + std::to_string(h) + "_" + std::to_string(k) + "_" + std::to_string(k_p);
//                 Z[h][k][k_p].setName(v_name.c_str());
//             }
//         }
//     }
//     //// aux var z_{k, k_p}: sortie launch from k and rendezvous at k_p.
//     IloArray<IloBoolVarArray> z(env, K);
//     for (int k = 1; k <= K - 1; k++) {
//         z[k] = IloBoolVarArray(env, K + 1);
//         for (int k_p = k + 1; k_p <= K; k_p++) {
//             z[k][k_p] = IloBoolVar(env);
//             auto v_name = "z_" + std::to_string(k) + "_" + std::to_string(k_p);
//             z[k][k_p].setName(v_name.c_str());
//         }
//     }

//     ////-----------------------------------------------------------------------------------------------
//     // WARMSTART CONFIG: using original TSP solution.
//     // Obtained with MTZ formulation.
//     // On variable X.
//     // if (use_tsp_as_warmstart) {
//     //     auto tsp_solution = TSP_MTZ(tau);
//     //     std::cout << tsp_solution.getSize() << std::endl;
//     //     IloNumVarArray X_warm_var(env);
//     //     IloNumArray X_warm_val(env);
//     //     for (int k = 1; k <= K - 1; k++) {
//     //         for (int i = 0; i <= D; i++) {
//     //             X_warm_var.add(X[k][i]);
//     //             if (tsp_solution[k - 1] == i) {
//     //                 X_warm_val.add(true);
//     //                 std::cout << "Warmstart X[" << k << "][" << i << "] == 1" << std::endl;
//     //             }
//     //             else {
//     //                 X_warm_val.add(false);
//     //             }
//     //         }
//     //     }
//     //     cplex.addMIPStart(X_warm_var, X_warm_val);
//     //     X_warm_var.end();
//     //     X_warm_val.end();
//     // }
//     ////-----------------------------------------------------------------------------------------------

//     //// Sortie maximum stage gap calculation.
//     /// Find maximum stage gap that a sortie can start and rendezvous.
//     /// For each stage gap in increasing order (k' - k), find the minimum tour length from stage k to k'.
//     /// Revisit(s) are also considered.
//     /// If the tour length > dtl - sr => there can be no sortie with stage gap greater or equal k' - k.
//     bool used_stage_gap = false;
//     int min_stage_gap = 0;
//     std::cout << "---------------------------------------------" << std::endl;
//     std::cout << "Calculate max stage gap for sortie.... " << std::endl;
//     for (int k = 1; k < K; k++) {
//         for (int kp = k + 1; kp <= K; kp++) {
//             if (kp > k + 6) {
//                 goto after_z_cons;
//             }
//             double smallest_tour = smallest_tour_length(kp - k, tau, V);
//             if (smallest_tour > dtl - sr) {
//                 std::cout << "Stage gap of " << kp - k << " with smallest length = " << smallest_tour <<
//                     " violated endurance constraint!" << std::endl;
//                 std::cout << "---------------------------------------------" << std::endl;
//                 min_stage_gap = kp - k;
//                 goto after_z_cons;
//             }
//         }
//     }
// after_z_cons:
//     if (min_stage_gap == 0) {
//         std::cout << "Stage gap calculation consumes too much memory. No constraint was added." << std::endl;
//         std::cout << "---------------------------------------------" << std::endl;
//     }
//     if (min_stage_gap != 0) {
//         used_stage_gap = true;
//         for (int k = 1; k < K; k++) {
//             for (int kp = k + min_stage_gap; kp <= K; kp++) {
//                 model.add(z[k][kp] == 0);
//                 std::cout << "Variable z[" << k << "][" << kp << "] was set to 0." << std::endl;
//                 for (int h : instance->c_prime) {
//                     model.add(Z[h][k][kp] == 0);
//                     IloExpr start_stage_k(env), end_stage_kp(env);
//                     for (int i = 0; i < D; i++) {
//                         if (i != h) {
//                             start_stage_k += Y[k][i][h];
//                             // model.add(Z[h][k][kp] + Y[k][i][h] <= 1);
//                         }
//                     }
//                     for (int j = 1; j <= D; j++) {
//                         if (j != h) {
//                             end_stage_kp += W[kp][j][h];
//                             // model.add(Z[h][k][kp] + W[kp][j][h] <= 1);
//                         }
//                     }
//                     std::string cname =
//                         "Can't serve customer " + std::to_string(h) + " start at stage " + std::to_string(k) +
//                         ", end at stage " + std::to_string(kp);
//                     // Looks like quicker lower bound improvements with this constraints.
//                     model.add(start_stage_k + end_stage_kp <= 1).setName(cname.c_str());
//                 }
//             }
//         }
//     }

//     ////-----------------------------------------------------------------------------------------------

//     // CONSTRAINTS DECLARATION.------------------------------------------------------------------------
//     // Constraint C1
//     for (int k = 1; k < K; k++) {
//         for (int i = 0; i < D; i++) {
//             IloExpr sum(env);
//             for (int j = 1; j <= D; j++) {
//                 sum += x[k][i][j];
//             }
//             model.add(X[k][i] == sum).setName(("C1_" + std::to_string(k) + "_" + std::to_string(i)).c_str());
//         }
//     }

//     for (int k = 2; k <= K; k++) {
//         for (int i = 1; i <= D; i++) {
//             IloExpr sum(env);
//             for (int j = 0; j < D; j++) {
//                 sum += x[k - 1][j][i];
//             }
//             // arcs entering i at stage k.
//             model.add(X[k][i] == sum).setName(("C1p_" + std::to_string(k) + "_" + std::to_string(i)).c_str());
//         }
//     }

//     // C2: truck must depart from the depot at stage 1.
//     IloExpr C2(env);
//     for (int i = 1; i <= D; i++) {
//         C2 += x[1][O][i];
//     }

//     IloConstraint c2(C2 == 1);
//     model.add(c2).setName("Must start from source");

//     // C3: Terminal depot must be arrived only once.
//     IloExpr C3(env);
//     for (int k = 2; k <= K; k++) {
//         C3 += X[k][D];
//     }
//     model.add(C3 == 1).setName("Must visit terminal depot once");

//     // At most ONE node each stage.
//     for (int k = 1; k <= K; k++) {
//         IloExpr sum(env);
//         for (int i = 0; i <= D; i++)
//             sum += X[k][i];
//         model.add(sum <= 1).setName(("C20_at_most_one_customer_at_stage_" + std::to_string(k)).c_str());
//     }

//     // $Z_{kk'} = \sum_{h}Z^h_{kk'}$: mỗi cặp (k,k') chỉ phục vụ tối đa một khách hàng (C22)
//     for (int k = 1; k < K; k++) {
//         for (int k_p = k + 1; k_p <= K; k_p++) {
//             IloExpr expr(env);
//             for (int h : C) {
//                 expr += Z[h][k][k_p];
//             }
//             model.add(expr == z[k][k_p]).setName(("C22_" + std::to_string(k) + "_" + std::to_string(k_p)).c_str());
//         }
//     }

//     //C20:$\sum_{k'>k}Z_{kk'} = \sum_{i,h}Y^k_{ih}$ : với mỗi $k$,
//     //ràng buộc liên kết drone đi ra từ stage $k$ và đoạn mà oto di chuyển không có drone. (C20)
//     for (int h : C) {
//         for (int k = 1; k <= K - 1; k++) {
//             IloExpr expr(env);
//             for (int k_p = k + 1; k_p <= K; k_p++) {
//                 expr += Z[h][k][k_p];
//             }

//             for (int i = 0; i < D; i++) {
//                 if (i != h && tau_prime[i][h] <= dtl - sr) {
//                     expr -= Y[k][i][h];
//                 }
//             }
//             model.add(expr == 0).setName(("C20_" + std::to_string(k) + "_" + std::to_string(h)).c_str());
//         }
//     }

//     for (int h : C) {
//         for (int k_p = 2; k_p <= K; k_p++) {
//             IloExpr expr(env);
//             for (int k = 1; k < k_p; k++) {
//                 expr += Z[h][k][k_p];
//             }

//             for (int i = 1; i <= D; i++) {
//                 if (i != h && tau_prime[h][i] <= dtl - sr) {
//                     expr -= W[k_p][i][h];
//                 }
//             }
//             model.add(expr == 0).setName(("C20p_" + std::to_string(k_p)
//                 + "_" + std::to_string(h)).c_str());
//         }
//     }

//     for (int k = 1; k < K; k++) {
//         IloExpr expr(env);
//         for (int k_p = k + 1; k_p <= K; k_p++) {
//             expr += z[k][k_p];
//         }
//         model.add(R[k] == expr).setName(("C23_" + std::to_string(k)).c_str());
//     }

//     // modified C7
//     for (int k = 1; k <= K - 1; k++) {
//         for (int k_p = k + 1; k_p <= K; k_p++) {
//             for (int l = k + 1; l < k_p; l++) {
//                 // tranh drone bay cac doan giao nhau.
//                 if (k < l) {
//                     model.add(z[k][k_p] + R[l] <= 1).setName(("C7m_" + std::to_string(k)
//                             + "_" + std::to_string(k_p) + "_" + std::to_string(l))
//                         .c_str());
//                 }
//                 // else {
//                 //                        model.add(R[k] <= 1);
//                 // }
//             }
//         }
//     }
//     //// C17 - $X^k_i \geq \sum_h X^k_{ih}$ (C17) - chỉ bay drone ở nơi mà xe ở đó
//     for (int k = 1; k <= K; k++) {
//         for (int i = 0; i <= D; i++) {
//             IloExpr expr(env);
//             for (int h : C) {
//                 if (h != i) {
//                     if (tau_prime[i][h] <= dtl - sr) {
//                         expr += Y[k][i][h];
//                     }
//                 }
//             }
//             model.add(expr <= X[k][i]).setName(("C17_" + std::to_string(k) + "_" + std::to_string(i)).c_str());
//         }
//     }

//     // $X^k_i \geq \sum_h Y^k_{ih}$ (C17p) : chỉ bay drone tới nơi mà xe ở đó
//     for (int k = 1; k <= K; k++) {
//         for (int i = 0; i <= D; i++) {
//             IloExpr expr(env);
//             for (int h : C) {
//                 if (h != i) {
//                     if (tau_prime[h][i] <= dtl - sr) {
//                         expr += W[k][i][h];
//                     }
//                 }
//             }
//             model.add(expr <= X[k][i]).setName(("C17p_" + std::to_string(k) + "_" + std::to_string(i)).c_str());
//         }
//     }

//     // $\sum_{i} X^k_{ih}\tau^D_{ih} + \sum_{i} Y^{k'}_{ih}\tau^D_{hi} \leq D_d$
//     //- drone duration constraint cho mỗi $h$ (C19)
//     for (int h : C) {
//         IloExpr expr(env);

//         for (int k = 1; k <= K; k++) {
//             for (int i = 0; i <= D; i++) {
//                 if (i != h && i != D && tau_prime[i][h] <= dtl - sr) {
//                     expr += Y[k][i][h] * tau_prime[i][h];
//                 }

//                 if (i != h && i != 0 && tau_prime[h][i] <= dtl - sr) {
//                     expr += W[k][i][h] * tau_prime[h][i];
//                 }
//             }
//         }
//         model.add(expr <= (dtl - sr) * phi[h]).setName(("C19_" + std::to_string(h)).c_str());
//     }

//     // modified C7p - we can select at most one segment that contains the point l
//     for (int l = 2; l < K; l++) {
//         IloExpr expr(env);

//         for (int k = 1; k < l; k++)
//             for (int k_p = l; k_p <= K; k_p++)
//                 expr += z[k][k_p];
//         model.add(expr <= 1).setName(("C7mm_" + std::to_string(l)).c_str());
//     }

//     for (int i = 0; i < D; i++) {
//         for (int k = 1; k <= K - 1; k++) {
//             IloExpr lhs(env);

//             for (int h : C) {
//                 if (i != h) {
//                     lhs += Y[k][i][h];
//                 }
//             }
//             model.add(lhs <= X[k][i]).setName(("C8_launch_" + std::to_string(i) + "_" + std::to_string(k)).c_str());
//         }
//     }

//     for (int j = 1; j <= D; j++) {
//         for (int k_p = 2; k_p <= K; k_p++) {
//             IloExpr lhs(env);

//             for (int h : C) {
//                 if (h != j) {
//                     lhs += W[k_p][j][h];
//                 }
//             }

//             model.add(lhs <= X[k_p][j]).setName(
//                 ("C8_rendezvous_" + std::to_string(j) + "_" + std::to_string(k_p)).c_str());
//         }
//     }

//     // $\phi_h = \sum_{k,i}X^k_{ih} = \sum_{k,i}Y^k_{ih}$
//     // - chỉ có duy nhất một điểm xuất phát và môt điểm đích cho
//     //mỗi khách hàng $h$ được phục vụ bởi drone (C18)
//     for (int h : C) {
//         IloExpr rhs(env);
//         for (int i = 0; i < D; i++) {
//             if (i != h && tau_prime[i][h] <= dtl - sr) {
//                 for (int k = 1; k <= K - 1; k++) {
//                     rhs += Y[k][i][h];
//                 }
//             }
//         }
//         // consistency constraint cho sortie phuc vu h.
//         model.add(phi[h] == rhs).setName(("C18_" + std::to_string(h)).c_str());
//     }

//     for (int h : C) {
//         IloExpr rhs(env);
//         for (int i = 1; i <= D; i++) {
//             if (i != h && tau_prime[h][i] <= dtl - sr) {
//                 for (int k = 2; k <= K; k++) {
//                     rhs += W[k][i][h];
//                 }
//             }
//         }
//         // consistency constraint cho sortie phuc vu h.
//         model.add(phi[h] == rhs).setName(("C18p_" + std::to_string(h)).c_str());
//     }

//     // Assignment constraint
//     //////////// C10: node_stage
//     for (int h : C) {
//         IloExpr sum_k(env);
//         for (int k = 2; k < K; k++) {
//             sum_k += X[k][h];
//         }
//         // phuc vu h it nhat 1 lan.
//         model.add(phi[h] + sum_k >= 1).setName(("C10_" + std::to_string(h)).c_str());
//     }

//     /////////// C14: node_stage
//     for (int k = 1; k <= K_arc; k++) {
//         IloExpr sum(env);
//         for (int i = 0; i < D; i++) {
//             for (int j = 1; j <= D; j++) {
//                 sum += x[k][i][j] * tau[i][j];
//             }
//         }

//         model.add(a[k + 1] == d[k] + sum).setName(("C14_" + std::to_string(k) + "_" + std::to_string(k + 1)).c_str());
//     }

//     ////////// C15: node_stage
//     ///// big M calculation
//     double M = 0;
//     for (int i = 0; i < n; i++) {
//         for (int j = 0; j < n; j++) {
//             if (i < j) {
//                 M += tau[i][j];
//             }
//         }
//     }

//     for (int k = 1; k <= K - 1; k++) {
//         for (int k_p = k + 1; k_p <= K; k_p++) {
//             if (k < k_p) {
//                 model.add(a[k_p] - d[k] <= z[k][k_p] * (dtl - sr) + (1 - z[k][k_p]) * M).setName(
//                     ("C15_" + std::to_string(k) + "_" + std::to_string(k_p)).c_str());
//             }
//         }
//     }

//     for (int k = 1; k <= K - 1; k++) {
//         for (int k_p = k + 1; k_p <= K; k_p++) {
//             if (k < k_p) {
//                 IloExpr sum_length_y_k(env);
//                 IloExpr sum_length_w_k_p(env);
//                 IloExpr sum_w_k_p(env);
//                 IloExpr sum_y_k_p(env);

//                 for (int i = 0; i < D; i++) {
//                     for (int h : C) {
//                         if (i != h) {
//                             sum_length_y_k += Y[k][i][h] * tau_prime[i][h];
//                         }
//                     }
//                 }
//                 for (int i = 0; i < D; i++) {
//                     for (int h : C) {
//                         if (i != h) {
//                             sum_y_k_p += Y[k_p][i][h] * sl;
//                         }
//                     }
//                 }
//                 for (int j = 1; j <= D; j++) {
//                     for (int h : C) {
//                         if (h != j) {
//                             sum_length_w_k_p += W[k_p][j][h] * tau_prime[h][j];
//                         }
//                     }
//                 }

//                 for (int j = 1; j <= D; j++) {
//                     for (int h : C) {
//                         if (h != j) {
//                             sum_w_k_p += W[k_p][j][h] * sr;
//                         }
//                     }
//                 }

//                 model.add(
//                     d[k_p] >=
//                     d[k] + sum_length_y_k + sum_length_w_k_p + sum_y_k_p + sum_w_k_p - (1 - z[k][k_p]) * M);
//             }
//         }
//     }

//     for (int k = 2; k <= K; k++) {
//         IloExpr sum(env);
//         IloExpr sum_w_K(env);
//         IloExpr sum_y_K(env);
//         for (int i = 0; i < D; i++) {
//             for (int j = 1; j <= D; j++) {
//                 sum += x[k - 1][i][j] * tau[i][j];
//             }
//         }
//         for (int i = 0; i < D; i++) {
//             for (int h : C) {
//                 if (i != h && k != K) {
//                     sum_y_K += sl * Y[k][i][h];
//                 }
//             }
//         }

//         for (int i = 1; i <= D; i++) {
//             for (int h : C) {
//                 if (i != h) {
//                     sum_w_K += sr * W[k][i][h];
//                 }
//             }
//         }
//         auto constr_name = "CC_d_" + std::to_string(k) + "_constr";
//         model.add(d[k] >= d[k - 1] + sum + sum_w_K + sum_y_K).setName(constr_name.c_str());
//     }

//     // SLOW
//     // (k,h): k chỉ có thể là đến hoặc đi của khách hàng h.
//     //    for (int h: instance->c_prime) {
//     //        for (int i: C) {
//     //            if (i != h) {
//     //                for (int k = 1; k < K; k++) {
//     //                    for (int kp = k + 1; kp < K; kp++) {
//     //                        model.add(Y[k][i][h] + W[kp][i][h] <= phi[h]);
//     //                    }
//     //                }
//     //                // SLOW VARIANT.
//     //            }
//     //        }
//     //    }

//     // LB constraint
//     IloExpr lb_truck_tour(env);
//     for (int k = 1; k < K; k++) {
//         for (int i = 0; i < D; i++) {
//             for (int j = 1; j <= D; j++) {
//                 lb_truck_tour += x[k][i][j] * tau[i][j];
//             }
//         }
//     }
//     model.add(d[K] >= lb_truck_tour).setName("Lower_bound_obj");

//     IloExpr lb_drone_tour(env);
//     for (int k = 1; k < K; k++) {
//         for (int i = 0; i < D; i++) {
//             for (int h : C) {
//                 if (i != h) {
//                     lb_drone_tour += Y[k][i][h] * tau_prime[i][h];
//                 }
//             }
//         }
//     }
//     for (int k = 2; k <= K; k++) {
//         for (int j = 1; j <= D; j++) {
//             for (int h : C) {
//                 if (j != h) {
//                     lb_drone_tour += W[k][j][h] * tau_prime[h][j];
//                 }
//             }
//         }
//     }
//     model.add(d[K] >= lb_drone_tour);

//     //    for (int k = 2; k < K / 2; k++) {
//     //        model.add(X[k][D] == 0);
//     //    }

//     double bestObjective = IloInfinity;
//     // BranchAndCutCallback bc(env, model, X, x, phi, Z, z, Y, W, K, D, bestObjective, C, tau, tau_prime);
//     // cplex.use(&bc);
//     model.add(IloMinimize(env, d[K]));
//     cplex.exportModel("cplex_model_1.lp");
//     double obj = 0;
//     double revisit_count = 0;
//     // Solve the model
//     std::vector<int> rev(n + 1, 0);
//     auto startTime = std::chrono::high_resolution_clock::now();
//     cplex.solve();
//     auto endTime = std::chrono::high_resolution_clock::now();
//     // Calculate the elapsed time
//     auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
//     // Check if the problem is infeasible

//     if (cplex.getStatus() == IloAlgorithm::Infeasible) {
//         // Handle infeasibility
//         std::cout << "The problem is infeasible." << std::endl;
//         std::cout << "Infeasibility at: " << cplex.getInfeasibility(c2) << std::endl;
//         // You can also retrieve the infeasible constraints using cplex.getInfeasibility() method
//     }

//     std::cout << "Feasible solution found!" << std::endl;
//     std::cout << "Truck nodes:" << std::endl;
//     for (int k = 1; k <= K; k++) {
//         for (int i = 0; i <= D; i++) {
//             auto X_val = cplex.getValue(X[k][i]);
//             if (X_val == 1) {
//                 rev[i]++;
//                 auto d_k = cplex.getValue(d[k]);
//                 auto a_k = cplex.getValue(a[k]);
//                 std::cout << "(" << a_k << ")-" << i << "(" << k << ")-(" << d_k << ")----->";
//                 break;
//             }
//         }
//     }
//     std::cout << "Truck arcs:" << std::endl;
//     std::map<int, std::pair<int, int>> map_stage_truck_arc;
//     for (int k = 1; k <= K_arc; k++) {
//         for (int i = 0; i < D; i++) {
//             for (int j = 1; j <= D; j++) {
//                 auto X_val = cplex.getValue(x[k][i][j]);
//                 if (X_val == 1) {
//                     std::cout << "Arc " << k << " connecting " << i << " and " << j
//                         << " with cost " << tau[i][j] << " " << std::endl;
//                     obj += tau[i][j];
//                     map_stage_truck_arc[k] = std::make_pair(i, j);
//                     break;
//                 }
//             }
//         }
//     }
//     std::cout << "Sorties information:" << std::endl;
//     for (int k = 1; k < K; k++) {
//         for (int kp = k + 1; kp <= K; kp++) {
//             for (int h : instance->c_prime) {
//                 if (cplex.getValue(Z[h][k][kp]) == 1) {
//                     int sv_i = -1, sv_j = -1, sv_k = -1, sv_kp = -1;
//                     for (int i = 0; i <= D; i++) {
//                         if (i != h) {
//                             try {
//                                 auto Y_val = cplex.getValue(Y[k][i][h]);
//                                 if (Y_val == 1) {
//                                     sv_i = i;
//                                     sv_k = k;
//                                 }
//                                 auto W_val = cplex.getValue(W[k][i][h]);
//                                 if (W_val == 1) {
//                                     sv_j = i;
//                                     sv_kp = k;
//                                 }
//                             }
//                             catch (...) {}
//                         }
//                     }
//                     // Print.
//                     std::cout << "-------------------------------------------------------------------------------------"
//                         << std::endl;
//                     std::cout << "Sortie starts from stage " << k << " at node " << sv_i << std::endl;
//                     std::cout << "Served customer " << h << ". phi[" << h << "]= " << cplex.getValue(phi[h])
//                         << std::endl;
//                     std::cout << "Sortie ends at stage " << kp << "at node " << sv_j << std::endl;

//                     ////////////////////////////////////////////////////////////////////////////
//                     double truck_travel_time = 0;
//                     double drone_travel_time = tau_prime[sv_i][h] + tau_prime[h][sv_j];
//                     std::cout << "Truck arcs during this sortie: ";
//                     for (int k_start = sv_k; k_start <= sv_kp; k_start++) {
//                         if (k_start == sv_kp) {
//                             std::cout << map_stage_truck_arc[k_start].first;
//                         }
//                         else {
//                             std::cout << map_stage_truck_arc[k_start].first << "--->";
//                         }
//                     }
//                     std::cout << std::endl;
//                     for (int k_start = sv_k; k_start <= sv_kp - 1; k_start++) {
//                         truck_travel_time += tau[map_stage_truck_arc[k_start].first][map_stage_truck_arc[k_start].
//                             second];
//                     }
//                     auto drone_arrival_time = cplex.getValue(d[sv_k]) + drone_travel_time;
//                     auto vehicle_departure_time = cplex.getValue(d[sv_kp]);
//                     auto truck_arrival_time = cplex.getValue(a[sv_kp]);
//                     std::cout << "Time information of this sortie:" << std::endl;
//                     std::cout << "Truck departure time from stage k = " << k << " is: " << cplex.getValue(d[k])
//                         << std::endl;
//                     std::cout << "Truck arcs during this sortie: ";
//                     for (int k_start = sv_k; k_start <= sv_kp; k_start++) {
//                         if (k_start == sv_kp) {
//                             std::cout << map_stage_truck_arc[k_start].first;
//                         }
//                         else {
//                             std::cout << map_stage_truck_arc[k_start].first << "--->";
//                         }
//                     }
//                     std::cout << ". Total truck time = " << truck_travel_time << std::endl;
//                     std::cout << "Truck arrival time at k' = " << kp << " is: " << truck_arrival_time << std::endl;
//                     std::cout << "Total drone time: " << drone_travel_time << std::endl;
//                     std::cout << "Drone arrival time at k' = " << kp << " is: " << drone_arrival_time << std::endl;
//                     std::cout << "Truck departure time at k' = " << cplex.getValue(d[sv_kp]);
//                     std::cout << "-------------------------------------------------------------------------------------"
//                         << std::endl;
//                 }
//             }
//         }
//     }
//     for (int h : C) {
//         if (cplex.getValue(phi[h]) == 1) {
//             std::cout << "Customer " << h << " served by drone." << std::endl;
//             int sv_i = -1, sv_j = -1, sv_k = -1, sv_kp = -1;
//             for (int k = 1; k <= K; k++) {
//                 for (int i = 0; i <= D; i++)
//                     if (i != h) {
//                         try {
//                             auto Y_val = cplex.getValue(Y[k][i][h]);
//                             if (Y_val == 1) {
//                                 sv_i = i;
//                                 sv_k = k;
//                             }
//                             auto W_val = cplex.getValue(W[k][i][h]);
//                             if (W_val == 1) {
//                                 sv_j = i;
//                                 sv_kp = k;
//                             }
//                         }
//                         catch (...) {}
//                     }
//             }

//             std::cout << "Drone fly from " << sv_i << " at stage " << sv_k <<
//                 " to serve " << h << " and then fly back to " << sv_j
//                 << " at stage " << sv_kp << ". " << std::endl;
//             obj += (sl + sr);
//             if (sv_i == O) {
//                 obj -= sl;
//             }
//             double drone_travel_time = tau_prime[sv_i][h] + tau_prime[h][sv_j];
//             double truck_travel_time = 0;
//             std::cout << "Truck arcs during this sortie: ";
//             for (int k_start = sv_k; k_start <= sv_kp; k_start++) {
//                 if (k_start == sv_kp) {
//                     std::cout << map_stage_truck_arc[k_start].first;
//                 }
//                 else {
//                     std::cout << map_stage_truck_arc[k_start].first << "--->";
//                 }
//             }
//             std::cout << std::endl;
//             for (int k_start = sv_k; k_start <= sv_kp - 1; k_start++) {
//                 truck_travel_time += tau[map_stage_truck_arc[k_start].first][map_stage_truck_arc[k_start].second];
//             }
//             std::cout << "Truck travel time from stage " << sv_k << " to " << sv_kp << " is: " << truck_travel_time <<
//                 std::endl;
//             if (drone_travel_time > truck_travel_time) {
//                 obj += drone_travel_time - truck_travel_time;
//             }
//             auto drone_arrival_time = cplex.getValue(d[sv_k]) + drone_travel_time;
//             auto vehicle_departure_time = cplex.getValue(d[sv_kp]);
//             auto truck_arrival_time = cplex.getValue(a[sv_kp]);
//             std::cout << "Total drone travel time: " << drone_travel_time << std::endl;

//             std::cout << "Drone arrival time: " << drone_arrival_time << std::endl;
//             std::cout << "Truck arrival time: " << truck_arrival_time << std::endl;

//             std::cout << "Truck departure time = max(d/a, t/a) plus (sl/sr): " << vehicle_departure_time << std::endl;
//             assert(drone_arrival_time <= vehicle_departure_time);
//             assert(abs(cplex.getValue(Z[h][sv_k][sv_kp]) - 1.0) < 1e-5);

//             assert(abs(cplex.getValue(z[sv_k][sv_kp]) - 1.0) < 1e-5);
//         }
//     }

//     std::cout << "Done!" << std::endl;
//     std::cout << "-------------------------Re-calculated objective-----------------------" << std::endl;
//     std::cout << obj << std::endl;
//     std::cout << "------------------------------------------------------------------------" << std::endl;

//     double c = cplex.getObjValue();
//     std::cout << "Solve time: " << duration.count() / 1000.0 << std::endl;
//     cplex.end();
//     model.end();
//     env.end();
//     for (int i = 0; i < rev.size(); i++) {
//         if (rev[i] > 1) {
//             revisit_count += rev[i] - 1;
//             std::cout << "Node " << i << " was revisited " << rev[i] - 1 << " times!" << std::endl;
//         }
//     }
//     std::cout << "Number of revisit: " << revisit_count << std::endl;
//     return Result{c, duration.count() / 1000.0, 0};
// }
