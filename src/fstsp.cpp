////
//// Created by cuong on 4/12/24.
////
//
//#include "../include/fstsp.h"
//#include "../include/solution.h"
//
//#include <chrono>
//#include <stack>
//#include <unordered_map>
//#include <filesystem>
//
//
//inline bool exist(const std::vector<int>& vec, int element) {
//    // Use std::find to search for the element in the vector
//    return std::find(vec.begin(), vec.end(), element) != vec.end();
//}
//
//struct MultivisitTuple {
//    int index;
//    int start_node;
//    int end_node;
//    std::vector<int> serve_customer;
//    std::vector<int> visit_seq;
//    double trip_l;
//};
//
//inline void generateCombinationsHelper(const std::vector<int>& C, int L, int start, std::vector<int>& current,
//                                       std::vector<std::vector<int>>& result) {
//    // Base case: If current combination is of size L, add it to the result
//    if (current.size() == L) {
//        result.push_back(current);
//        return;
//    }
//
//    // Recursive case: Generate combinations starting from each index in C
//    for (int i = start; i < C.size(); ++i) {
//        current.push_back(C[i]);
//        generateCombinationsHelper(C, L, i + 1, current, result);
//        current.pop_back();
//    }
//}
//
//inline std::vector<std::vector<int>> generateCombinations(const std::vector<int>& C, int L) {
//    std::vector<std::vector<int>> result;
//    std::vector<int> current;
//    generateCombinationsHelper(C, L, 0, current, result);
//    return result;
//}
//
//inline std::vector<std::vector<int>> generatePermutations(const std::vector<int>& nums) {
//    std::vector<std::vector<int>> result;
//
//    // Sort the input vector to ensure all permutations are unique
//    std::vector<int> sortedNums = nums;
//    std::sort(sortedNums.begin(), sortedNums.end());
//
//    // Generate permutations using std::next_permutation
//    do {
//        result.push_back(sortedNums);
//    }
//    while (std::next_permutation(sortedNums.begin(), sortedNums.end()));
//
//    return result;
//}
//
//inline bool areAllElementsDistinct(const std::vector<int>& vec) {
//    std::unordered_set<int> seen;
//
//    for (int num : vec) {
//        if (seen.count(num) > 0) {
//            // Duplicate found
//            return false;
//        }
//        seen.insert(num);
//    }
//
//    // No duplicates found
//    return true;
//}
//
//inline double calculate_tour(std::vector<std::vector<double>>& tau, std::vector<int>& tour) {
//    double l = 0;
//    for (int i = 0; i < tour.size() - 1; i++) {
//        l += tau[tour[i]][tour[i + 1]];
//    }
//    return l;
//}
//
//inline std::vector<MultivisitTuple>
//shortest_L_visit_sorties(int start, int rendezvous, int L, const std::vector<int>& C,
//                         std::vector<std::vector<double>>& tau_prime, double& dtl, double& sr) {
//    auto combination_of_L_from_C = generateCombinations(C, L);
//    std::vector<MultivisitTuple> result;
//    for (auto& combo : combination_of_L_from_C) {
//        double shortest = std::numeric_limits<double>::max();
//        bool got = false;
//        MultivisitTuple mt;
//        auto permutations = generatePermutations(combo);
//        for (auto& per : permutations) {
//            std::vector<int> temp;
//            temp.push_back(start);
//            for (int& elem : per) {
//                temp.push_back(elem);
//            }
//            temp.push_back(rendezvous);
//            if (areAllElementsDistinct(temp)) {
//                if (double l = calculate_tour(tau_prime, temp); l < shortest && l <= dtl - sr) {
//                    shortest = l;
//                    mt.visit_seq = temp;
//                    mt.trip_l = l;
//                    mt.serve_customer = per;
//                    got = true;
//                }
//            }
//        }
//        if (got) {
//            mt.start_node = start;
//            mt.end_node = rendezvous;
//            result.push_back(mt);
//        }
//    }
//    return result;
//}
//
//std::vector<MultivisitTuple> getSortiesFromLR(int l, int r, std::vector<MultivisitTuple>& mt) {
//    std::vector<MultivisitTuple> res;
//    // both are provided
//    if (l >= 0 && r > 0 && l != r) {
//        for (auto& m : mt) {
//            if (m.start_node == l && m.end_node == r) {
//                res.push_back(m);
//            }
//        }
//        return res;
//    }
//    else if (l >= 0 && r == -1) {
//        // only provide start. end = -1
//        for (auto& m : mt) {
//            if (m.start_node == l) {
//                res.push_back(m);
//            }
//        }
//        return res;
//    }
//    else if (l == -1 && r > 0) {
//        // only provide end. start = -1
//        for (auto& m : mt) {
//            if (m.end_node == r) {
//                res.push_back(m);
//            }
//        }
//        return res;
//    }
//    else {
//        std::cout << "No sorties with options was found. Provided parameters: launch = " << l << ", rendezvous = " << r
//            << std::endl;
//        return res;
//    }
//}
//
//inline std::vector<std::pair<std::vector<int>, std::vector<int>>> generateSetsAndComplements(
//    const std::vector<int>& elements) {
//    auto n = elements.size();
//    std::vector<std::pair<std::vector<int>, std::vector<int>>> result;
//
//    // Total number of subsets is 2^n
//    int totalSubsets = 1 << n;
//
//    for (int i = 0; i < totalSubsets; ++i) {
//        std::vector<int> setS, setT;
//
//        for (int j = 0; j < n; ++j) {
//            // Check if jth bit is set in the binary representation of i
//            if (i & (1 << j)) {
//                setS.push_back(elements[j]);
//            }
//            else {
//                setT.push_back(elements[j]);
//            }
//        }
//        if (setS.size() != 0 && setT.size() != 0) {
//            result.push_back(std::make_pair(setS, setT));
//        }
//    }
//
//    return result;
//}
//
//
//inline void setPrint(std::vector<int>& set) {
//    for (int i : set) {
//        std::cout << i << " ";
//    }
//    std::cout << std::endl;
//}
//
//inline void genSetsHelper_SortieGap(const std::vector<int>& nodes, int k, std::vector<int>& currentSet,
//                                    std::vector<std::vector<int>>& result) {
//    // Base case: if currentSet has k nodes, add it to result
//    if (currentSet.size() == k) {
//        result.push_back(currentSet);
//        return;
//    }
//
//    // Recursive case: iterate through all nodes
//    for (int i = 0; i < nodes.size(); ++i) {
//        // Add the current node to the set
//        currentSet.push_back(nodes[i]);
//
//        // Recur with the updated set
//        genSetsHelper_SortieGap(nodes, k, currentSet, result);
//
//        // Backtrack: remove the last node from the set
//        currentSet.pop_back();
//    }
//}
//
//inline std::vector<std::vector<int>> genSetForSortieGap(const std::vector<int>& nodes, int k, int D) {
//    std::vector<std::vector<int>> result;
//    std::vector<int> currentSet;
//    genSetsHelper_SortieGap(nodes, k, currentSet, result);
//    std::vector<std::vector<int>> result2;
//    for (auto vec : result) {
//        // perform test
//        bool check = true;
//        for (int i = 0; i < vec.size(); i++) {
//            if (i < vec.size() - 1 && vec[i] == D) {
//                check = false;
//            }
//            if (i < vec.size() - 1 && vec[i] == vec[i + 1]) {
//                check = false;
//            }
//        }
//        if (check) {
//            result2.push_back(vec);
//        }
//    }
//
//    return result2;
//}
//
//
//inline double smallest_tour_length(int stage_gap, std::vector<std::vector<double>>& tau, std::vector<int>& V) {
//    int D = V.size() - 1;
//    auto sets = genSetForSortieGap(V, stage_gap + 1, D);
//    double smallest = std::numeric_limits<double>::max();
//    for (auto set : sets) {
//        auto l = calculate_tour(tau, set);
//        if (l < smallest) {
//            smallest = l;
//        }
//    }
//    return smallest;
//}
//
//IloNumArray FSTSPSolver::TSP_MTZ(std::vector<std::vector<double>>& tau) {
//    int n = tau.size() - 1;
//
//    IloEnv env;
//    IloModel model(env);
//    IloCplex cplex(model);
//    cplex.setParam(IloCplex::Param::MIP::Tolerances::Integrality, 0);
//    IloArray<IloBoolVarArray> x(env, n);
//    for (int i = 0; i < n; i++) {
//        x[i] = IloBoolVarArray(env, n);
//        for (int j = 0; j < n; j++) {
//            model.add(x[i][j]);
//        }
//    }
//    IloNumVarArray u(env, n);
//    for (int i = 0; i < n; i++) {
//        u[i] = IloNumVar(env, 0, n - 1, ILOFLOAT);
//        model.add(u[i]);
//    }
//    IloRangeArray cons1(env, n);
//    for (int j = 0; j < n; ++j) {
//        IloExpr expr(env);
//        for (int i = 0; i < n; ++i) {
//            if (i != j) expr += x[i][j];
//        }
//        cons1[j] = IloRange(env, 1, expr, 1);
//        model.add(cons1[j]);
//    }
//
//    IloRangeArray cons2(env, n);
//    for (int i = 0; i < n; ++i) {
//        IloExpr expr(env);
//        for (int j = 0; j < n; ++j) {
//            if (i != j) expr += x[i][j];
//        }
//        cons2[i] = IloRange(env, 1, expr, 1);
//        model.add(cons2[i]);
//    }
//
//    model.add(IloRange(env, 0, u[0], 0));
//    IloArray<IloRangeArray> cons3(env, n);
//    for (int i = 0; i < n; ++i) {
//        cons3[i] = IloRangeArray(env, n);
//        for (int j = 1; j < n; ++j) {
//            if (i == j) continue;
//            cons3[i][j] = IloRange(env, u[i] - u[j] + n * x[i][j], n - 1);
//            model.add(cons3[i][j]);
//        }
//    }
//    IloExpr obj(env);
//    for (int i = 0; i < n; i++) {
//        for (int j = 0; j < n; j++) {
//            obj += tau[i][j] * x[i][j];
//        }
//    }
//    model.add(IloMinimize(env, obj));
//    cplex.solve();
//    std::cout << std::endl;
//    IloNumArray order(env);
//    for (int i = 0; i < n; i++) {
//        for (int vertex = 0; vertex < n; vertex++) {
//            if (abs(cplex.getValue(u[vertex]) - i) < 1e-5) {
//                order.add(vertex);
//                continue;
//            }
//        }
//    }
//    order.add(n);
//    std::cout << "TSP solution for this instance: ";
//    for (int i = 0; i < order.getSize(); i++) {
//        std::cout << order[i] << " ";
//    }
//    std::cout << std::endl;
//    cplex.end();
//    model.end();
//    env.end();
//    return order;
//}
//
//IloNumArray FSTSPSolver::RevisitTSP(std::vector<std::vector<double>>& tau) {
//    int n = tau.size() - 1;
//
//    IloEnv env;
//    IloModel model(env);
//    IloCplex cplex(model);
//    cplex.setParam(IloCplex::Param::MIP::Tolerances::Integrality, 0);
//    int D = n;
//    int K = n * 1.5;
//    IloArray<IloBoolVarArray> X(env, K + 1);
//    for (int k = 1; k <= K; k++) {
//        X[k] = IloBoolVarArray(env, D + 1);
//        for (int i = 0; i <= D; i++) {
//            X[k][i] = IloBoolVar(env);
//            model.add(X[k][i]);
//            auto v_name = "X_" + std::to_string(k) + "_" + std::to_string(i);
//            //std::cout << v_name << std::endl;
//            X[k][i].setName(v_name.c_str());
//            if (k > 1 && i == 0) {
//                model.add(X[k][0] == 0);
//            }
//        }
//    }
//
//    model.add(X[1][0] == 1).setName("start depot is the first node");
//    model.add(X[1][D] == 0).setName("ending depot cannot be the first node");
//
//
//    for (int k = 1; k <= K; k++) {
//        IloExpr sum(env);
//        for (int i = 0; i <= D; i++)
//            sum += X[k][i];
//        model.add(sum <= 1).setName(("C20_at_most_one_customer_at_stage_" + std::to_string(k)).c_str());
//    }
//
//    IloExpr arrival_depot(env);
//    for (int k = 1; k <= K; k++) {
//        arrival_depot += X[k][D];
//    }
//    model.add(arrival_depot == 1).setName("C21_arrival_depot_once");
//    IloExpr obj(env);
//    // x^k_(ij) (binary variable) và nhận giá trị một nếu Xk
//    // mô ta cạnh nối 2 đỉnh liên tiếp trên đường đi.
//    IloArray<IloArray<IloBoolVarArray>> x(env, K);
//    for (int k = 1; k < K; k++) {
//        x[k] = IloArray<IloBoolVarArray>(env, D);
//        for (int i = 0; i < D; i++) {
//            x[k][i] = IloBoolVarArray(env, D + 1);
//            for (int j = 1; j <= D; j++) {
//                if (i != j) {
//                    x[k][i][j] = IloBoolVar(env);
//                    auto v_name = "x_" + std::to_string(k) + "_" + std::to_string(i) + "_" + std::to_string(j);
//                    x[k][i][j].setName(v_name.c_str());
//                    obj += x[k][i][j] * tau[i][j];
//                    model.add(x[k][0][D] == 0);
//                }
//            }
//        }
//    }
//
//    model.add(IloMinimize(env, obj));
//    ////////// Constraint C1
//    for (int k = 1; k < K; k++) {
//        for (int i = 0; i < D; i++) {
//            {
//                // i == D => khong co i -> j.
//                IloExpr sum(env);
//                for (int j = 1; j <= D; j++) {
//                    if (i != j) {
//                        sum += x[k][i][j];
//                    }
//                }
//
//                // neu node i la node_stage k, i khac D thi kieu gi cung co canh i, j.
//                model.add(X[k][i] == sum).setName(("C1_" + std::to_string(k) + "_" + std::to_string(i)).c_str());
//            }
//        }
//    }
//
//    for (int k = 2; k <= K; k++) {
//        for (int i = 1; i <= D; i++) {
//            {
//                IloExpr sum(env);
//                for (int j = 0; j < D; j++) {
//                    if (i != j) {
//                        sum += x[k - 1][j][i];
//                    }
//                }
//                // arcs entering i at stage k.
//                model.add(X[k][i] == sum).setName(("C1p_" + std::to_string(k) + "_" + std::to_string(i)).c_str());
//            }
//        }
//    }
//
//    //////////// C2 - depart from the depot
//    IloExpr C2(env);
//    for (int i = 1; i <= D; i++) {
//        C2 += x[1][0][i];
//    }
//
//    IloConstraint c2(C2 == 1);
//    model.add(c2).setName("C2");
//
//    ///////////// C3: arc_stage
//    IloExpr C3(env);
//    for (int k = 2; k <= K; k++) {
//        {
//            C3 += X[k][D];
//        }
//    }
//    // , "C3"
//    model.add(C3 == 1).setName("C3"); // arrival to depot
//
//    for (int h = 1; h < D; h++) {
//        IloExpr sum(env);
//        for (int k = 2; k < K; k++) {
//            sum += X[k][h];
//        }
//        model.add(sum >= 1);
//    }
//
//    cplex.solve();
//    IloNumArray order(env);
//
//    for (int k = 1; k <= K; k++) {
//        for (int i = 0; i <= D; i++) {
//            if (abs(cplex.getValue(X[k][i]) - 1) < 1e-5) {
//                std::cout << "Stage " << k << " at " << i << std::endl;
//            }
//        }
//    }
//    for (int k = 1; k < K; k++) {
//        for (int i = 0; i < D; i++) {
//            for (int j = 1; j <= D; j++) {
//                if (i != j) {
//                    if (abs(cplex.getValue(x[k][i][j]) - 1) < 1e-5) {
//                        std::cout << i << " " << j << std::endl;
//                        order.add(i);
//                    }
//                }
//            }
//        }
//    }
//
//    order.add(D);
//    std::cout << "Revisit TSP solution for this instance: ";
//    for (int i = 0; i < order.getSize(); i++) {
//        std::cout << order[i] << " ";
//    }
//    std::cout << std::endl;
//    return order;
//}
//
//inline std::vector<int> extractVisitOrder(const std::vector<std::pair<int, int>>& arcs) {
//    std::vector<int> visitOrder;
//    std::unordered_set<int> visitedNodes;
//
//    // Extracting visit order
//    for (const auto& arc : arcs) {
//        if (visitedNodes.find(arc.first) == visitedNodes.end()) {
//            visitOrder.push_back(arc.first);
//            visitedNodes.insert(arc.first);
//        }
//        if (visitedNodes.find(arc.second) == visitedNodes.end()) {
//            visitOrder.push_back(arc.second);
//            visitedNodes.insert(arc.second);
//        }
//    }
//
//    return visitOrder;
//}
//
//WarmstartInfo FSTSPSolver::FSTSP_Warmstart(Config& cfg) const {
//    auto tau = instance->tau;
//
//    auto d = instance->tau_prime;
//    auto n = instance->num_node;
//    auto s = 0, t = n;
//    auto c_prime = instance->c_prime;
//    std::vector<int> c_prime_0;
//    c_prime_0.push_back(0);
//    for (int i : c_prime) {
//        c_prime_0.push_back(i);
//    }
//    c_prime_0.push_back(n);
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
//    std::cout << std::endl;
//    IloEnv env;
//    IloModel model(env);
//    IloCplex cplex(model);
//
//    // y: (i, j) in A, truck route
//    IloArray<IloBoolVarArray> y(env, n + 1);
//    for (int i : c_s) {
//        y[i] = IloBoolVarArray(env, n + 1);
//        for (int j : c_t) {
//            y[i][j] = IloBoolVar(env);
//            if (i == j) {
//                model.add(y[i][j] == 0);
//            }
//        }
//    }
//    model.add(y[s][t] == 0);
//
//
//    IloArray<IloBoolVarArray> x(env, n + 1);
//    for (int i : c_s) {
//        x[i] = IloBoolVarArray(env, n + 1);
//        for (int j : c_t) {
//            x[i][j] = IloBoolVar(env);
//            if (i == j) {
//                model.add(x[i][j] == 0);
//            }
//        }
//    }
//    model.add(x[s][t] == 0);
//
//
//    // gamma_h_ij
//    IloArray<IloArray<IloBoolVarArray>> gamma(env, n + 1);
//    for (int h : C) {
//        gamma[h] = IloArray<IloBoolVarArray>(env, n + 1);
//        for (int i : c_s) {
//            gamma[h][i] = IloBoolVarArray(env, n + 1);
//            for (int j : c_t) {
//                gamma[h][i][j] = IloBoolVar(env);
//                for (int heavy : instance->heavy) {
//                    if (h == heavy) {
//                        model.add(gamma[h][i][j] == 0);
//                    }
//                }
//                if (i == j) {
//                    model.add(gamma[h][i][j] == 0);
//                }
//            }
//        }
//        model.add(gamma[h][s][t] == 0);
//    }
//
//    IloBoolVarArray theta(env, n + 1);
//    for (int h : V) {
//        theta[h] = IloBoolVar(env);
//        if (!exist(instance->c_prime, h) || h == s || h == t) {
//            model.add(theta[h] == 0);
//        }
//    }
//
//    IloArray<IloBoolVarArray> omega(env, n + 1);
//    for (int h : C) {
//        omega[h] = IloBoolVarArray(env, n + 1);
//        for (int i : V) {
//            omega[h][i] = IloBoolVar(env);
//
//            for (int heavy : instance->heavy) {
//                if (h == heavy) {
//                    model.add(omega[h][i] == 0);
//                }
//            }
//            if (h == i || i == t) {
//                model.add(omega[h][i] == 0);
//            }
//        }
//    }
//    IloArray<IloBoolVarArray> delta(env, n + 1);
//    for (int h : C) {
//        delta[h] = IloBoolVarArray(env, n + 1);
//        for (int j : V) {
//            delta[h][j] = IloBoolVar(env);
//            for (int heavy : instance->heavy) {
//                if (h == heavy) {
//                    model.add(delta[h][j] == 0);
//                }
//            }
//            if (h == j || j == s) {
//                model.add(delta[h][j] == 0);
//            }
//        }
//    }
//
//    IloNumVarArray sigma(env, n + 1);
//    for (int h : c_t) {
//        sigma[h] = IloNumVar(env, 0, IloInfinity, ILOFLOAT);
//        for (int heavy : instance->heavy) {
//            if (h == heavy) {
//                model.add(sigma[h] == 0);
//            }
//        }
//    }
//
//    IloExpr objective(env);
//    for (int i : c_s) {
//        for (int j : c_t) {
//            objective += tau[i][j] * y[i][j];
//        }
//    }
//
//    for (int h : C) {
//        objective += (cfg.sl + cfg.sr) * theta[h];
//        objective -= cfg.sl * omega[h][s];
//    }
//    for (int h : c_t) {
//        objective += sigma[h];
//    }
//    IloExpr sum_theta(env);
//    // Constraint 1
//    IloExpr lhs_1(env), rhs_1(env);
//    for (int j : c_t) {
//        lhs_1 += y[s][j];
//    }
//    for (int i : c_s) {
//        rhs_1 += y[i][t];
//    }
//    model.add(lhs_1 == 1);
//    model.add(rhs_1 == 1);
//
//    // Constraint 2
//    for (int i : C) {
//        IloExpr lhs_2(env), rhs_2(env);
//        for (int j : c_t) {
//            lhs_2 += y[i][j];
//        }
//        for (int j : c_s) {
//            rhs_2 += y[j][i];
//        }
//        model.add(lhs_2 == rhs_2);
//        model.add(lhs_2 <= 1);
//        model.add(rhs_2 <= 1);
//    }
//
//    // Constraint 3
//    auto setAndComps = generateSetsAndComplements(C);
//    for (auto& set : setAndComps) {
//        auto S = set.first;
//        if (S.size() < 2) {
//            continue;
//        }
//        if (S.size() == 2 && S[0] == s && S[1] == t) {
//            continue;
//        }
//        IloExpr sum1(env), sum2(env);
//        std::string cname = "C3";
//        for (auto i : S) {
//            cname += "_" + std::to_string(i);
//            if (i != t) {
//                for (auto j : S) {
//                    if (j != s) {
//                        sum1 += y[i][j];
//                    }
//                }
//            }
//        }
//
//        for (auto h : S) {
//            IloExpr sum3(env);
//            for (auto k : S) {
//                if (h == k || h == s || h == t) {
//                    continue;
//                }
//                else {
//                    if (exist(C, k)) {
//                        sum3 += 1 - theta[k];
//                    }
//                }
//            }
//
//            model.add(sum1 <= sum3);
//        }
//    }
//    // Constraint 4
//    for (int h : C) {
//        IloExpr lhs_4(env);
//        std::string cname = "C4_h_" + std::to_string(h);
//        for (int j : c_t) {
//            lhs_4 += gamma[h][s][j];
//        }
//        model.add(lhs_4 == omega[h][s]);
//    }
//
//    // Constraint 5
//    for (int h : C) {
//        IloExpr lhs_5(env);
//        std::string cname = "C5_h_" + std::to_string(h);
//
//        for (int i : c_s) {
//            lhs_5 += gamma[h][i][t];
//        }
//        model.add(lhs_5 == delta[h][t]);
//    }
//    // Constraint 6
//    for (int i : C) {
//        for (int h : C) {
//            std::string cname = "C6_i_" + std::to_string(i) + "_h_" + std::to_string(h);
//            IloExpr sum1(env), sum2(env);
//            for (int j : c_t) {
//                sum1 += gamma[h][i][j];
//            }
//
//            for (int j : c_s) {
//                sum2 += gamma[h][j][i];
//            }
//            model.add(sum1 - sum2 == omega[h][i] - delta[h][i]);
//        }
//    }
//    // Constraint 7
//    for (int j : c_t) {
//        std::string cname = "C7_s_j=" + std::to_string(j);
//        model.add(y[s][j] + x[s][j] <= 1);
//    }
//    // Constraint 8
//    for (int i : c_s) {
//        std::string cname = "C8_i_" + std::to_string(i) + "_t";
//        model.add(y[i][t] + x[i][t] <= 1);
//    }
//    // Constraint 9
//    for (int i : C) {
//        for (int j : C) {
//            std::string cname = "C9_i_" + std::to_string(i) + "_j=" + std::to_string(j);
//            model.add(y[i][j] + x[i][j] + x[j][i] <= 1);
//        }
//    }
//
//    // Constraint 10
//    for (int h : C) {
//        IloExpr sum(env);
//        std::string cname = "C10_h_" + std::to_string(h);
//        for (int j : c_t) {
//            if (h != j) {
//                sum += y[h][j];
//            }
//        }
//        model.add(sum + theta[h] == 1);
//    }
//    // Constraint 11
//    for (int i : c_s) {
//        for (int j : c_t) {
//            if (i != j) {
//                std::string cname = "C11_i_" + std::to_string(i) + "_j=" + std::to_string(j);
//                IloExpr sum(env);
//                for (int h : c_prime) {
//                    sum += gamma[h][i][j];
//                }
//                model.add(sum <= y[i][j]);
//            }
//        }
//    }
//
//
//    // Constraint 12
//    for (int h : C) {
//        IloExpr sum1(env), sum2(env);
//
//        for (int i : V) {
//            if (i != h && i != t) {
//                sum1 += omega[h][i];
//            }
//        }
//        for (int j : V) {
//            if (j != s && j != h) {
//                sum2 += delta[h][j];
//            }
//        }
//        model.add(sum1 == theta[h]);
//        model.add(sum2 == theta[h]);
//    }
//    // Constraint 13
//    for (int i : c_s) {
//        for (int j : c_t) {
//            if (i != j) {
//                std::string cname = "C13_i_" + std::to_string(i) + "_j=" + std::to_string(j);
//                model.add(x[i][j] <= theta[i] + theta[j]);
//            }
//        }
//    }
//    //        // Constraint 14
//    for (int i : c_s) {
//        for (int j : c_t) {
//            if (i != s && j != t && i != j) {
//                std::string cname = "C14_i_" + std::to_string(i) + "_j=" + std::to_string(j);
//                model.add(x[i][j] <= omega[j][i] + delta[i][j]);
//            }
//        }
//    }
//    // Constraint 15
//    for (int i : c_s) {
//        IloExpr sum1(env), sum2(env);
//        std::string cname = "C15_i_" + std::to_string(i);
//        for (int j : c_t) {
//            if (i != j) {
//                sum1 += x[i][j];
//            }
//        }
//        for (int h : c_prime) {
//            sum2 += omega[h][i];
//        }
//        sum2 += theta[i];
//        model.add(sum1 == sum2);
//        model.add(sum1 <= 1);
//        model.add(sum2 <= 1);
//    }
//    // Constraint 16
//    for (int j : c_t) {
//        IloExpr sum1(env), sum2(env);
//        for (int i : c_s) {
//            sum1 += x[i][j];
//        }
//
//        for (int h : c_prime) {
//            sum2 += delta[h][j];
//        }
//        sum2 += theta[j];
//        model.add(sum1 == sum2);
//        model.add(sum1 <= 1);
//        model.add(sum2 <= 1);
//    }
//    // Constraint 17
//    for (int h : c_prime) {
//        IloExpr sum(env);
//        std::string cname = "C17_h_" + std::to_string(h);
//        for (int i : c_s) {
//            for (int j : c_t) {
//                sum += tau[i][j] * gamma[h][i][j];;
//            }
//        }
//        model.add(sum <= (cfg.dtl - cfg.sr) * theta[h]);
//    }
//    // Constraint 18
//    for (int h : c_prime) {
//        IloExpr sum1(env);
//        IloExpr sum2(env);
//        IloExpr sum3(env);
//        std::string c18_name = "C18_h_" + std::to_string(h);
//        std::string c19_name = "C19_h_" + std::to_string(h);
//
//        for (int i : c_s) {
//            sum1 += d[i][h] * omega[h][i];
//        }
//        for (int j : c_t) {
//            sum2 += d[h][j] * delta[h][j];
//        }
//
//        for (int i : c_s) {
//            for (int j : c_t) {
//                sum3 += tau[i][j] * gamma[h][i][j];
//            }
//        }
//        model.add(sum1 + sum2 <= (cfg.dtl - cfg.sr) * theta[h]);
//        model.add(sum1 + sum2 - sum3 <= sigma[h]);
//    }
//    model.add(IloMinimize(env, objective));
//    auto startTime = std::chrono::high_resolution_clock::now();
//    cplex.solve();
//    auto endTime = std::chrono::high_resolution_clock::now();
//    // Calculate the elapsed time
//    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
//    std::vector<std::pair<int, int>> arcs;
//    std::vector<Sortie> st;
//    std::cout << "Truck arcs:" << std::endl;
//    for (int i : c_s) {
//        for (int j : c_t) {
//            if (i != j) {
//                if (cplex.getValue(y[i][j]) == 1) {
//                    std::cout << i << " " << j << std::endl;
//                    arcs.push_back(std::make_pair(i, j));
//                }
//            }
//        }
//    }
//
//    for (int h : C) {
//        if (cplex.getValue(theta[h]) == 1) {
//            int start;
//            int end;
//            std::cout << "customer " << h << " is served by drone" << std::endl;
//            for (int i : c_s) {
//                if (h != i) {
//                    if (cplex.getValue(omega[h][i]) == 1) {
//                        std::cout << "start of this sortie: " << i << std::endl;
//                        start = i;
//                    }
//                }
//            }
//
//            for (int j : c_t) {
//                if (j != h) {
//                    if (cplex.getValue(delta[h][j]) == 1) {
//                        std::cout << "end of this sortie: " << j << std::endl;
//                        end = j;
//                    }
//                }
//            }
//            std::cout << std::endl;
//            auto st_if = Sortie(start, end, h);
//            st.push_back(st_if);
//        }
//    }
//    std::cout << cplex.getObjValue() << std::endl;
//    auto stage_node = extractVisitOrder(arcs);
//    for (int i = 0; i < stage_node.size(); i++) {
//        std::cout << "Stage " << i + 1 << " at node " << stage_node[i] << std::endl;
//    }
//    return WarmstartInfo(stage_node, st);
//}
//
//void dfs(int node, const std::vector<std::vector<int>>& graph, std::vector<bool>& visited,
//         std::vector<int>& current_path, std::vector<std::vector<int>>& cycles) {
//    visited[node] = true;
//    current_path.push_back(node);
//
//    for (int neighbor = 0; neighbor < graph[node].size(); ++neighbor) {
//        if (graph[node][neighbor]) {
//            if (!visited[neighbor]) {
//                dfs(neighbor, graph, visited, current_path, cycles);
//            }
//            else {
//                auto it = find(current_path.begin(), current_path.end(), neighbor);
//                if (it != current_path.end()) {
//                    // Found a cycle
//                    std::vector<int> cycle;
//                    for (auto it_cycle = it; it_cycle != current_path.end(); ++it_cycle) {
//                        cycle.push_back(*it_cycle);
//                    }
//                    cycles.push_back(cycle);
//                }
//            }
//        }
//    }
//
//    current_path.pop_back();
//    visited[node] = false;
//}
//
//std::vector<std::vector<int>> find_cycles(const std::vector<std::vector<int>>& graph) {
//    int num_nodes = graph.size();
//    std::vector<bool> visited(num_nodes, false);
//    std::vector<std::vector<int>> cycles;
//
//    for (int node = 0; node < num_nodes; ++node) {
//        std::vector<int> current_path;
//        dfs(node, graph, visited, current_path, cycles);
//    }
//
//    return cycles;
//}
//
//Result FSTSPSolver::mvdSolverWithLR(Config& cfg) const {
//    auto tau = instance->tau;
//    auto tau_prime = instance->tau_prime;
//    auto n = instance->num_node;
//    std::vector<int> C;
//    std::vector<int> V;
//    for (int i = 0; i < n + 1; i++) {
//        if (i != 0 && i != n) {
//            C.push_back(i);
//        }
//        V.push_back(i);
//    }
//
//    IloEnv env;
//    IloModel model(env);
//    IloCplex cplex(model);
//    auto O = 0;
//    auto D = n;
//    int K = n + 1;
//    auto K_arc = K - 1;
//    cplex.setParam(IloCplex::Param::Threads, cfg.num_thread);
//    // cplex.setParam(IloCplex::Param::MIP::Tolerances::Integrality, 0);
//    // Variable declaration
//    // X^i_k (binary variable) và nhận giá trị một tương ứng với đỉnh thứ k của
//    //đường đi của vehicle là i; k \in 1..n;
//    IloArray<IloBoolVarArray> X(env, K + 1);
//    for (int k = 1; k <= K; k++) {
//        X[k] = IloBoolVarArray(env, D + 1);
//        for (int i = 0; i <= D; i++) {
//            X[k][i] = IloBoolVar(env);
//            model.add(X[k][i]);
//            auto v_name = "X_" + std::to_string(k) + "_" + std::to_string(i);
//            X[k][i].setName(v_name.c_str());
//        }
//        if (k > 1) {
//            model.add(X[k][0] == 0);
//        }
//    }
//
//    model.add(X[1][0] == 1).setName("First stage must be source depot");
//
//    // x^k_(ij) (binary variable) và nhận giá trị một nếu Xk
//    // mô ta cạnh nối 2 đỉnh liên tiếp trên đường đi.
//    IloArray<IloArray<IloBoolVarArray>> x(env, K_arc + 1);
//    for (int k = 1; k <= K_arc; k++) {
//        x[k] = IloArray<IloBoolVarArray>(env, D);
//        for (int i = 0; i < D; i++) {
//            x[k][i] = IloBoolVarArray(env, D + 1);
//            for (int j = 1; j <= D; j++) {
//                if (i != j) {
//                    x[k][i][j] = IloBoolVar(env);
//                    auto v_name = "x_" + std::to_string(k) + "_" + std::to_string(i) + "_" + std::to_string(j);
//                    x[k][i][j].setName(v_name.c_str());
//                }
//            }
//        }
//    }
//
//    //// phi^h equals to 1 if customer h is served by the drone
//    IloBoolVarArray phi(env, n);
//    for (int h : C) {
//        phi[h] = IloBoolVar(env);
//        auto v_name = "phi_" + std::to_string(h);
//        phi[h].setName(v_name.c_str());
//        if (exist(instance->heavy, h)) {
//            model.add(phi[h] == 0);
//        }
//    }
//
//    IloArray<IloArray<IloBoolVarArray>> Y(env, K + 1), W(env, K + 1);
//    for (int k = 1; k <= K; k++) {
//        Y[k] = IloArray<IloBoolVarArray>(env, D + 1);
//        W[k] = IloArray<IloBoolVarArray>(env, D + 1);
//
//        for (int i = 0; i <= D; i++) {
//            Y[k][i] = IloBoolVarArray(env, C.size() + 1);
//            W[k][i] = IloBoolVarArray(env, C.size() + 1);
//            for (int h : C) {
//                if (i != h) {
//                    Y[k][i][h] = IloBoolVar(env);
//                    Y[k][i][h].setName(("Y_" + std::to_string(k) + "_"
//                        + std::to_string(i) + "_" + std::to_string(h)).c_str());
//
//                    W[k][i][h] = IloBoolVar(env);
//                    W[k][i][h].setName(("W_" + std::to_string(k) + "_"
//                        + std::to_string(i) + "_" + std::to_string(h)).c_str());
//
//                    if (i == 0 && k > 1) {
//                        model.add(Y[k][i][h] == 0);
//                    }
//                    if (i == D && k == 1) {
//                        model.add(W[k][i][h] == 0);
//                    }
//                    if (tau_prime[i][h] > cfg.dtl - cfg.sr) {
//                        model.add(Y[k][i][h] == 0);
//                    }
//                    if (tau_prime[h][i] > cfg.dtl - cfg.sr) {
//                        model.add(W[k][i][h] == 0);
//                    }
//                }
//            }
//        }
//    }
//
//    // arrival\departure variables a and d.
//    IloNumVarArray a(env, K + 1);
//    IloNumVarArray d(env, K + 1);
//    for (int k = 1; k <= K; k++) {
//        a[k] = IloNumVar(env, 0, IloInfinity, ILOFLOAT);
//        auto v_name = "a_" + std::to_string(k);
//        a[k].setName(v_name.c_str());
//        d[k] = IloNumVar(env, 0, IloInfinity, ILOFLOAT);
//        v_name = "d_" + std::to_string(k);
//        d[k].setName(v_name.c_str());
//        model.add(d[k] >= a[k]).setName(("C13_" + std::to_string(k)).c_str());
//    }
//
//    model.add(a[1] == 0).setName("arrival to depot at time 0");
//    model.add(d[1] == 0).setName("depart from depot at time 0");;
//
//    // $R_{k} = \sum_{k'}Z_{kk'}$: các đoạn bắt đầu từ k (C23)
//    IloBoolVarArray R(env, K + 1);
//    for (int k = 1; k < K; k++) {
//        R[k].setName(("R_" + std::to_string(k)).c_str());
//    }
//
//    //// aux var Z_{k, k_p, h}: sortie launch from k and rendezvous at k_p.
//    IloArray<IloArray<IloBoolVarArray>> Z(env, K);
//    for (int h : C) {
//        Z[h] = IloArray<IloBoolVarArray>(env, K + 1);
//        for (int k = 1; k <= K - 1; k++) {
//            Z[h][k] = IloBoolVarArray(env, K + 1);
//            for (int k_p = k + 1; k_p <= K; k_p++) {
//                Z[h][k][k_p] = IloBoolVar(env);
//                auto v_name = "Z_" + std::to_string(h) + "_" + std::to_string(k) + "_" + std::to_string(k_p);
//                Z[h][k][k_p].setName(v_name.c_str());
//            }
//        }
//    }
//    //// aux var z_{k, k_p}: sortie launch from k and rendezvous at k_p.
//    IloArray<IloBoolVarArray> z(env, K);
//    for (int k = 1; k <= K - 1; k++) {
//        z[k] = IloBoolVarArray(env, K + 1);
//        for (int k_p = k + 1; k_p <= K; k_p++) {
//            z[k][k_p] = IloBoolVar(env);
//            auto v_name = "z_" + std::to_string(k) + "_" + std::to_string(k_p);
//            z[k][k_p].setName(v_name.c_str());
//        }
//    }
//
//    ////-----------------------------------------------------------------------------------------------
//    // WARMSTART CONFIG: using original TSP solution.
//    // Obtained with MTZ formulation.
//    // On variable X.
//    if (cfg.use_tsp_for_warmstart) {
//        auto tsp_solution = TSP_MTZ(tau);
//        std::cout << tsp_solution.getSize() << std::endl;
//        IloNumVarArray X_warm_var(env);
//        IloNumArray X_warm_val(env);
//        for (int k = 1; k <= K - 1; k++) {
//            for (int i = 0; i <= D; i++) {
//                X_warm_var.add(X[k][i]);
//                if (tsp_solution[k - 1] == i) {
//                    X_warm_val.add(true);
//                    std::cout << "Warmstart X[" << k << "][" << i << "] == 1" << std::endl;
//                }
//                else {
//                    X_warm_val.add(false);
//                }
//            }
//        }
//        cplex.addMIPStart(X_warm_var, X_warm_val);
//        X_warm_var.end();
//        X_warm_val.end();
//    }
//    ////-----------------------------------------------------------------------------------------------
//
//    //// Sortie maximum stage gap calculation.
//    /// Find maximum stage gap that a sortie can start and rendezvous.
//    /// For each stage gap in increasing order (k' - k), find the minimum tour length from stage k to k'.
//    /// Revisit(s) are also considered.
//    /// If the tour length > cfg.dtl - cfg.sr => there can be no sortie with stage gap greater or equal k' - k.
//    bool used_stage_gap = false;
//    int min_stage_gap = 0;
//    std::cout << "---------------------------------------------" << std::endl;
//    std::cout << "Calculate max stage gap for sortie.... " << std::endl;
//    for (int k = 1; k < K; k++) {
//        for (int kp = k + 1; kp <= K; kp++) {
//            if (kp > k + 6) {
//                goto after_z_cons;
//            }
//            double smallest_tour = smallest_tour_length(kp - k, tau, V);
//            if (smallest_tour > cfg.dtl - cfg.sr) {
//                std::cout << "Stage gap of " << kp - k << " with smallest length = " << smallest_tour <<
//                    " violated endurance constraint!" << std::endl;
//                std::cout << "---------------------------------------------" << std::endl;
//                min_stage_gap = kp - k;
//                goto after_z_cons;
//            }
//        }
//    }
//after_z_cons:
//    if (min_stage_gap == 0) {
//        std::cout << "Stage gap calculation consumes too much memory. No constraint was added." << std::endl;
//        std::cout << "---------------------------------------------" << std::endl;
//    }
//    if (min_stage_gap != 0) {
//        used_stage_gap = true;
//        for (int k = 1; k < K; k++) {
//            for (int kp = k + min_stage_gap; kp <= K; kp++) {
//                model.add(z[k][kp] == 0);
//                std::cout << "Variable z[" << k << "][" << kp << "] was set to 0." << std::endl;
//                for (int h : instance->c_prime) {
//                    model.add(Z[h][k][kp] == 0);
//                    IloExpr start_stage_k(env), end_stage_kp(env);
//                    for (int i = 0; i < D; i++) {
//                        if (i != h) {
//                            start_stage_k += Y[k][i][h];
//                            // model.add(Z[h][k][kp] + Y[k][i][h] <= 1);
//                        }
//                    }
//                    for (int j = 1; j <= D; j++) {
//                        if (j != h) {
//                            end_stage_kp += W[kp][j][h];
//                            // model.add(Z[h][k][kp] + W[kp][j][h] <= 1);
//                        }
//                    }
//                    std::string cname = "Can't serve customer " + std::to_string(h) + " start at stage " +
//                        std::to_string(k) + ", end at stage " + std::to_string(kp);
//                    // Looks like quicker lower bound improvements with this constraints.
//                    model.add(start_stage_k + end_stage_kp <= 1).setName(cname.c_str());
//                }
//            }
//        }
//    }
//
//    ////-----------------------------------------------------------------------------------------------
//
//    // CONSTRAINTS DECLARATION.------------------------------------------------------------------------
//    // Constraint C1
//    for (int k = 1; k < K; k++) {
//        for (int i = 0; i < D; i++) {
//            IloExpr sum(env);
//            for (int j = 1; j <= D; j++) {
//                if (i != j) {
//                    sum += x[k][i][j];
//                }
//            }
//            model.add(X[k][i] == sum).setName(("C1_" + std::to_string(k) + "_" + std::to_string(i)).c_str());
//        }
//    }
//
//    //// 1-visit constraint
//    // for (int i:V) {
//    //     IloExpr sum_visit_i(env);
//    //     for (int k = 1; k <= K; k++) {
//    //         sum_visit_i += X[k][i];
//    //     }
//    //     model.add(sum_visit_i <= 1);
//    // }
//
//    for (int k = 2; k <= K; k++) {
//        for (int i = 1; i <= D; i++) {
//            IloExpr sum(env);
//            for (int j = 0; j < D; j++) {
//                if (i != j) {
//                    sum += x[k - 1][j][i];
//                }
//            }
//            // arcs entering i at stage k.
//            model.add(X[k][i] == sum).setName(("C1p_" + std::to_string(k) + "_" + std::to_string(i)).c_str());
//        }
//    }
//
//    // C2: truck must depart from the depot at stage 1.
//    IloExpr C2(env);
//    for (int i = 1; i <= D; i++) {
//        C2 += x[1][O][i];
//    }
//
//    IloConstraint c2(C2 == 1);
//    model.add(c2).setName("Must start from source");
//
//    // C3: Terminal depot must be arrived only once.
//    IloExpr C3(env);
//    for (int k = 2; k <= K; k++) {
//        C3 += X[k][D];
//    }
//    model.add(C3 == 1).setName("Must visit terminal depot once");
//
//    // At most ONE node each stage.
//    for (int k = 1; k <= K; k++) {
//        IloExpr sum(env);
//        for (int i = 0; i <= D; i++)
//            sum += X[k][i];
//        model.add(sum <= 1).setName(("C20_at_most_one_customer_at_stage_" + std::to_string(k)).c_str());
//    }
//
//    // $Z_{kk'} = \sum_{h}Z^h_{kk'}$: mỗi cặp (k,k') chỉ phục vụ tối đa một khách hàng (C22)
//    for (int k = 1; k < K; k++) {
//        for (int k_p = k + 1; k_p <= K; k_p++) {
//            IloExpr expr(env);
//            for (int h : C) {
//                expr += Z[h][k][k_p];
//            }
//            model.add(expr == z[k][k_p]).setName(("C22_" + std::to_string(k) + "_" + std::to_string(k_p)).c_str());
//        }
//    }
//
//    //C20:$\sum_{k'>k}Z_{kk'} = \sum_{i,h}Y^k_{ih}$ : với mỗi $k$,
//    //ràng buộc liên kết drone đi ra từ stage $k$ và đoạn mà oto di chuyển không có drone. (C20)
//    for (int h : C) {
//        for (int k = 1; k <= K - 1; k++) {
//            IloExpr expr(env);
//            for (int k_p = k + 1; k_p <= K; k_p++) {
//                expr += Z[h][k][k_p];
//            }
//
//            for (int i = 0; i < D; i++) {
//                if (i != h && tau_prime[i][h] <= cfg.dtl - cfg.sr) {
//                    expr -= Y[k][i][h];
//                }
//            }
//            model.add(expr == 0).setName(("C20_" + std::to_string(k) + "_" + std::to_string(h)).c_str());
//        }
//    }
//
//    for (int h : C) {
//        for (int k_p = 2; k_p <= K; k_p++) {
//            IloExpr expr(env);
//            for (int k = 1; k < k_p; k++) {
//                expr += Z[h][k][k_p];
//            }
//
//            for (int i = 1; i <= D; i++) {
//                if (i != h && tau_prime[h][i] <= cfg.dtl - cfg.sr) {
//                    expr -= W[k_p][i][h];
//                }
//            }
//            model.add(expr == 0).setName(("C20p_" + std::to_string(k_p)
//                + "_" + std::to_string(h)).c_str());
//        }
//    }
//
//    for (int k = 1; k < K; k++) {
//        IloExpr expr(env);
//        for (int k_p = k + 1; k_p <= K; k_p++) {
//            expr += z[k][k_p];
//        }
//        model.add(R[k] == expr).setName(("C23_" + std::to_string(k)).c_str());
//    }
//
//    // modified C7
//    for (int k = 1; k <= K - 1; k++) {
//        for (int k_p = k + 1; k_p <= K; k_p++) {
//            for (int l = k + 1; l < k_p; l++) {
//                // tranh drone bay cac doan giao nhau.
//                if (k < l) {
//                    model.add(z[k][k_p] + R[l] <= 1).setName(("C7m_" + std::to_string(k)
//                            + "_" + std::to_string(k_p) + "_" + std::to_string(l))
//                        .c_str());
//                }
//            }
//        }
//    }
//    //// C17 - $X^k_i \geq \sum_h X^k_{ih}$ (C17) - chỉ bay drone ở nơi mà xe ở đó
//    for (int k = 1; k <= K; k++) {
//        for (int i = 0; i <= D; i++) {
//            IloExpr expr(env);
//            for (int h : C) {
//                if (h != i) {
//                    if (tau_prime[i][h] <= cfg.dtl - cfg.sr) {
//                        expr += Y[k][i][h];
//                    }
//                }
//            }
//            model.add(expr <= X[k][i]).setName(("C17_" + std::to_string(k) + "_" + std::to_string(i)).c_str());
//        }
//    }
//
//    // $X^k_i \geq \sum_h Y^k_{ih}$ (C17p) : chỉ bay drone tới nơi mà xe ở đó
//    for (int k = 1; k <= K; k++) {
//        for (int i = 0; i <= D; i++) {
//            IloExpr expr(env);
//            for (int h : C) {
//                if (h != i) {
//                    if (tau_prime[h][i] <= cfg.dtl - cfg.sr) {
//                        expr += W[k][i][h];
//                    }
//                }
//            }
//            model.add(expr <= X[k][i]).setName(("C17p_" + std::to_string(k) + "_" + std::to_string(i)).c_str());
//        }
//    }
//
//    // $\sum_{i} X^k_{ih}\tau^D_{ih} + \sum_{i} Y^{k'}_{ih}\tau^D_{hi} \leq D_d$
//    //- drone duration constraint cho mỗi $h$ (C19)
//    for (int h : C) {
//        IloExpr expr(env);
//
//        for (int k = 1; k <= K; k++) {
//            for (int i = 0; i <= D; i++) {
//                if (i != h && i != D && tau_prime[i][h] <= cfg.dtl - cfg.sr) {
//                    expr += Y[k][i][h] * tau_prime[i][h];
//                }
//
//                if (i != h && i != 0 && tau_prime[h][i] <= cfg.dtl - cfg.sr) {
//                    expr += W[k][i][h] * tau_prime[h][i];
//                }
//            }
//        }
//        model.add(expr <= (cfg.dtl - cfg.sr) * phi[h]).setName(("C19_" + std::to_string(h)).c_str());
//    }
//
//    // modified C7p - we can select at most one segment that contains the point l
//    for (int l = 2; l < K; l++) {
//        IloExpr expr(env);
//
//        for (int k = 1; k < l; k++)
//            for (int k_p = l; k_p <= K; k_p++)
//                expr += z[k][k_p];
//        model.add(expr <= 1).setName(("C7mm_" + std::to_string(l)).c_str());
//    }
//
//    for (int i = 0; i < D; i++) {
//        for (int k = 1; k <= K - 1; k++) {
//            IloExpr lhs(env);
//
//            for (int h : C) {
//                if (i != h) {
//                    lhs += Y[k][i][h];
//                }
//            }
//            model.add(lhs <= X[k][i]).setName(("C8_launch_" + std::to_string(i) + "_" + std::to_string(k)).c_str());
//        }
//    }
//
//    for (int j = 1; j <= D; j++) {
//        for (int k_p = 2; k_p <= K; k_p++) {
//            IloExpr lhs(env);
//
//            for (int h : C) {
//                if (h != j) {
//                    lhs += W[k_p][j][h];
//                }
//            }
//
//            model.add(lhs <= X[k_p][j]).setName(
//                ("C8_rendezvous_" + std::to_string(j) + "_" + std::to_string(k_p)).c_str());
//        }
//    }
//
//
//    // $\phi_h = \sum_{k,i}X^k_{ih} = \sum_{k,i}Y^k_{ih}$
//    // - chỉ có duy nhất một điểm xuất phát và môt điểm đích cho
//    //mỗi khách hàng $h$ được phục vụ bởi drone (C18)
//    for (int h : C) {
//        IloExpr rhs(env);
//        for (int i = 0; i < D; i++) {
//            if (i != h && tau_prime[i][h] <= cfg.dtl - cfg.sr) {
//                for (int k = 1; k <= K - 1; k++) {
//                    rhs += Y[k][i][h];
//                }
//            }
//        }
//        // consistency constraint cho sortie phuc vu h.
//        model.add(phi[h] == rhs).setName(("C18_" + std::to_string(h)).c_str());
//    }
//
//
//    for (int h : C) {
//        IloExpr rhs(env);
//        for (int i = 1; i <= D; i++) {
//            if (i != h && tau_prime[h][i] <= cfg.dtl - cfg.sr) {
//                for (int k = 2; k <= K; k++) {
//                    rhs += W[k][i][h];
//                }
//            }
//        }
//        // consistency constraint cho sortie phuc vu h.
//        model.add(phi[h] == rhs).setName(("C18p_" + std::to_string(h)).c_str());
//    }
//
//
//    // Assignment constraint
//    //////////// C10: node_stage
//    for (int h : C) {
//        IloExpr sum_k(env);
//        for (int k = 2; k < K; k++) {
//            sum_k += X[k][h];
//        }
//        // phuc vu h it nhat 1 lan. == 1 thì thành phục vụ duy nhất 1 lần.
//        model.add(phi[h] + sum_k == 1).setName(("C10_" + std::to_string(h)).c_str());
//    }
//
//    /////////// C14: node_stage
//    for (int k = 1; k <= K_arc; k++) {
//        IloExpr sum(env);
//        for (int i = 0; i < D; i++) {
//            for (int j = 1; j <= D; j++) {
//                if (i != j) {
//                    sum += x[k][i][j] * tau[i][j];
//                }
//            }
//        }
//
//        model.add(a[k + 1] == d[k] + sum).setName(("C14_" + std::to_string(k) + "_" + std::to_string(k + 1)).c_str());
//    }
//
//    ////////// C15: node_stage
//    ///// big M calculation
//    double M = 0;
//    for (int i = 0; i < n; i++) {
//        for (int j = 0; j < n; j++) {
//            if (i < j) {
//                M += tau[i][j];
//            }
//        }
//    }
//
//    // for (int k = 1; k <= K - 1; k++) {
//    //     for (int k_p = k + 1; k_p <= K; k_p++) {
//    //         if (k < k_p) {
//    //             model.add(a[k_p] - d[k] <= z[k][k_p] * (cfg.dtl - cfg.sr) + (1 - z[k][k_p]) * M).setName(
//    //                 ("C15_" + std::to_string(k) + "_" + std::to_string(k_p)).c_str());
//    //         }
//    //     }
//    // }
//
//    for (int k = 1; k < K; k++) {
//        for (int k_p = k + 1; k_p <= K; k_p++) {
//            if (k < k_p) {
//                model.add(a[k_p] - d[k] <= z[k][k_p] * (cfg.dtl - cfg.sr) + (1 - z[k][k_p]) * M).setName(
//                    ("C15_" + std::to_string(k) + "_" + std::to_string(k_p)).c_str());
//                IloExpr truck_length(env);
//                for (int start = k; start < k_p; start++) {
//                    for (int i = 0; i < D; i++) {
//                        for (int j = 1; j <= D; j++) {
//                            if (i != j) {
//                                truck_length += x[start][i][j] * tau[i][j];
//                            }
//                        }
//                    }
//                }
//                model.add(truck_length <= z[k][k_p] * (cfg.dtl - cfg.sr) + (1 - z[k][k_p]) * M);
//            }
//        }
//    }
//
//    for (int k = 1; k <= K - 1; k++) {
//        for (int k_p = k + 1; k_p <= K; k_p++) {
//            if (k < k_p) {
//                IloExpr sum_length_y_k(env);
//                IloExpr sum_length_w_k_p(env);
//                IloExpr sum_w_k_p(env);
//                IloExpr sum_y_k_p(env);
//
//                for (int i = 0; i < D; i++) {
//                    for (int h : C) {
//                        if (i != h) {
//                            sum_length_y_k += Y[k][i][h] * tau_prime[i][h];
//                        }
//                    }
//                }
//                for (int i = 0; i < D; i++) {
//                    for (int h : C) {
//                        if (i != h) {
//                            sum_y_k_p += Y[k_p][i][h] * cfg.sl;
//                        }
//                    }
//                }
//                for (int j = 1; j <= D; j++) {
//                    for (int h : C) {
//                        if (h != j) {
//                            sum_length_w_k_p += W[k_p][j][h] * tau_prime[h][j];
//                        }
//                    }
//                }
//
//                for (int j = 1; j <= D; j++) {
//                    for (int h : C) {
//                        if (h != j) {
//                            sum_w_k_p += W[k_p][j][h] * cfg.sr;
//                        }
//                    }
//                }
//
//
//                model.add(
//                    d[k_p] >=
//                    d[k] + sum_length_y_k + sum_length_w_k_p + sum_y_k_p + sum_w_k_p - (1 - z[k][k_p]) * M);
//            }
//        }
//    }
//
//    for (int k = 2; k <= K; k++) {
//        IloExpr sum(env);
//        IloExpr sum_w_K(env);
//        IloExpr sum_y_K(env);
//        for (int i = 0; i < D; i++) {
//            for (int j = 1; j <= D; j++) {
//                if (i != j) {
//                    sum += x[k - 1][i][j] * tau[i][j];
//                }
//            }
//        }
//        for (int i = 0; i < D; i++) {
//            for (int h : C) {
//                if (i != h && k != K) {
//                    sum_y_K += cfg.sl * Y[k][i][h];
//                }
//            }
//        }
//
//        for (int i = 1; i <= D; i++) {
//            for (int h : C) {
//                if (i != h) {
//                    sum_w_K += cfg.sr * W[k][i][h];
//                }
//            }
//        }
//        auto constr_name = "CC_d_" + std::to_string(k) + "_constr";
//        model.add(d[k] >= d[k - 1] + sum + sum_w_K + sum_y_K).setName(constr_name.c_str());
//    }
//
//    // SLOW
//    // (k,h): k chỉ có thể là đến hoặc đi của khách hàng h.
//    if (!cfg.allow_loop) {
//        for (int h : instance->c_prime) {
//            for (int i : C) {
//                if (i != h) {
//                    for (int k = 1; k < K; k++) {
//                        for (int kp = k + 1; kp < K; kp++) {
//                            model.add(Y[k][i][h] + W[kp][i][h] <= phi[h]);
//                        }
//                    }
//                    // SLOW VARIANT.
//                }
//            }
//        }
//    }
//
//    // LB constraint
//    IloExpr lb_truck_tour(env);
//    for (int k = 1; k < K; k++) {
//        for (int i = 0; i < D; i++)
//            for (int j = 1; j <= D; j++)
//                if (i != j)
//                    lb_truck_tour += x[k][i][j] * tau[i][j];
//    }
//    model.add(d[K] >= lb_truck_tour).setName("Lower_bound_obj");
//
//    IloExpr lb_drone_tour(env);
//    for (int k = 1; k < K; k++) {
//        for (int i = 0; i < D; i++) {
//            for (int h : C) {
//                if (i != h) {
//                    lb_drone_tour += Y[k][i][h] * tau_prime[i][h];
//                }
//            }
//        }
//    }
//    for (int k = 2; k <= K; k++) {
//        for (int j = 1; j <= D; j++) {
//            for (int h : C) {
//                if (j != h) {
//                    lb_drone_tour += W[k][j][h] * tau_prime[h][j];
//                }
//            }
//        }
//    }
//    model.add(d[K] >= lb_drone_tour);
//
//    for (int k = 2; k < K / 2; k++) {
//        model.add(X[k][D] == 0);
//    }
//
//    model.add(IloMinimize(env, d[K]));
//    // model.add(IloMinimize(env, objective));
//    cplex.exportModel("cplex_model_1.lp");
//    std::vector<Sortie> st;
//    double obj = 0;
//    double revisit_count = 0;
//    // Solve the model
//    std::vector<int> rev(n + 1, 0);
//    auto startTime = std::chrono::high_resolution_clock::now();
//    cplex.solve();
//    auto endTime = std::chrono::high_resolution_clock::now();
//    // Calculate the elapsed time
//    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
//    // Check if the problem is infeasible
//
//    if (cplex.getStatus() == IloAlgorithm::Infeasible) {
//        // Handle infeasibility
//        std::cout << "The problem is infeasible." << std::endl;
//        std::cout << "Infeasibility at: " << cplex.getInfeasibility(c2) << std::endl;
//        // You can also retrieve the infeasible constraints using cplex.getInfeasibility() method
//    }
//
//    std::cout << "Feasible solution found!" << std::endl;
//    std::cout << "Truck nodes:" << std::endl;
//    for (int k = 1; k <= K; k++) {
//        for (int i = 0; i <= D; i++) {
//            auto X_val = cplex.getValue(X[k][i]);
//            //std::cout << "k = " << k << ", i = " << i << ":" << X_val << std::endl;
//            if (abs(X_val - 1) < 1e-5) {
//                rev[i]++;
//                auto d_k = cplex.getValue(d[k]);
//                auto a_k = cplex.getValue(a[k]);
//                std::cout << "Stage " << k << " at customer " << i << " with arrival time is: " << a_k << std::endl;
//                std::cout << "Stage " << k << " at customer " << i << " with departure time is: " << d_k << std::endl;
//                break;
//            }
//        }
//    }
//    std::cout << "Truck arcs:" << std::endl;
//    std::map<int, std::pair<int, int>> map_stage_truck_arc;
//    for (int k = 1; k <= K_arc; k++) {
//        for (int i = 0; i < D; i++)
//            for (int j = 1; j <= D; j++)
//                if (i != j) {
//                    auto X_val = cplex.getValue(x[k][i][j]);
//                    if (abs(X_val - 1) < 1e-5) {
//                        std::cout << "Arc " << k << " connecting " << i << " and " << j
//                            << " with cost " << tau[i][j] << " " << std::endl;
//                        obj += tau[i][j];
//                        map_stage_truck_arc[k] = std::make_pair(i, j);
//                        break;
//                    }
//                }
//    }
//
//    for (int h : C) {
//        if (abs(cplex.getValue(phi[h]) - 1) < 1e-5) {
//            std::cout << "Customer " << h << " served by drone." << std::endl;
//            int sv_i = -1, sv_j = -1, sv_k = -1, sv_kp = -1;
//            for (int k = 1; k <= K; k++) {
//                for (int i = 0; i <= D; i++)
//                    if (i != h) {
//                        try {
//                            auto Y_val = cplex.getValue(Y[k][i][h]);
//                            if (abs(Y_val - 1) < 1e-5) {
//                                sv_i = i;
//                                sv_k = k;
//                            }
//                            auto W_val = cplex.getValue(W[k][i][h]);
//                            if (abs(W_val - 1) < 1e-5) {
//                                sv_j = i;
//                                sv_kp = k;
//                            }
//                        }
//                        catch (...) {}
//                    }
//            }
//
//            std::cout << "Drone fly from " << sv_i << " at stage " << sv_k <<
//                " to serve " << h << " and then fly back to " << sv_j
//                << " at stage " << sv_kp << ". " << std::endl;
//            obj += (cfg.sl + cfg.sr);
//            if (sv_i == O) {
//                obj -= cfg.sl;
//            }
//            double drone_travel_time = tau_prime[sv_i][h] + tau_prime[h][sv_j];
//            double truck_travel_time = 0;
//            std::cout << "Truck arcs during this sortie: ";
//            for (int k_start = sv_k; k_start <= sv_kp; k_start++) {
//                if (k_start == sv_kp) {
//                    std::cout << map_stage_truck_arc[k_start].first;
//                }
//                else {
//                    std::cout << map_stage_truck_arc[k_start].first << "--->";
//                }
//            }
//            std::cout << std::endl;
//            for (int k_start = sv_k; k_start <= sv_kp - 1; k_start++) {
//                truck_travel_time += tau[map_stage_truck_arc[k_start].first][map_stage_truck_arc[k_start].second];
//            }
//            std::cout << "Truck travel time from stage " << sv_k << " to " << sv_kp << " is: " << truck_travel_time <<
//                std::endl;
//            if (drone_travel_time > truck_travel_time) {
//                obj += drone_travel_time - truck_travel_time;
//            }
//            auto drone_arrival_time = cplex.getValue(d[sv_k]) + drone_travel_time;
//            auto vehicle_departure_time = cplex.getValue(d[sv_kp]);
//            auto truck_arrival_time = cplex.getValue(a[sv_kp]);
//            std::cout << "Total drone travel time: " << drone_travel_time << std::endl;
//
//            std::cout << "Drone arrival time: " << drone_arrival_time << std::endl;
//            std::cout << "Truck arrival time: " << truck_arrival_time << std::endl;
//
//            std::cout << "Truck departure time = max(d/a, t/a) plus (cfg.sl/cfg.sr): " << vehicle_departure_time <<
//                std::endl;
//            assert(drone_arrival_time <= vehicle_departure_time);
//            assert(abs(cplex.getValue(Z[h][sv_k][sv_kp]) - 1.0) < 1e-5);
//
//            assert(abs(cplex.getValue(z[sv_k][sv_kp]) - 1.0) < 1e-5);
//        }
//    }
//
//    std::cout << "Done!" << std::endl;
//    std::cout << "-------------------------Re-calculated objective-----------------------" << std::endl;
//    std::cout << obj << std::endl;
//    std::cout << "------------------------------------------------------------------------" << std::endl;
//
//
//    double c = cplex.getObjValue();
//    std::cout << "Solve time: " << duration.count() / 1000.0 << std::endl;
//    cplex.end();
//    model.end();
//    env.end();
//    for (int i = 0; i < rev.size(); i++) {
//        if (rev[i] > 1) {
//            revisit_count += rev[i] - 1;
//            std::cout << "Node " << i << " was revisited " << rev[i] - 1 << " times!" << std::endl;
//        }
//    }
//    std::cout << "OBJECTIVE VALUE: " << c << ", NUMBER OF SORTIES: " << st.size() << "." << std::endl;
//    std::cout << "Number of revisit: " << revisit_count << std::endl;
//    return Result{c, duration.count() / 1000.0, cplex.getMIPRelativeGap()};
//}
//
//Result
//FSTSPSolver::RV_FSTSP_MVD_4_index(Config& cfg) const {
//    try {
//        auto tau = instance->tau;
//        auto tau_prime = instance->tau_prime;
//        auto n = instance->num_node;
//        std::vector<int> C;
//        std::vector<int> V;
//        std::vector<int> c_s;
//        std::vector<int> c_t;
//        for (int i = 0; i < n + 1; i++) {
//            if (i != 0 && i != n) {
//                C.push_back(i);
//            }
//            if (i != 0) {
//                c_t.push_back(i);
//            }
//            if (i != n) {
//                c_s.push_back(i);
//            }
//            V.push_back(i);
//        }
//
//        IloEnv env;
//        IloModel model(env);
//        IloCplex cplex(model);
//
//        auto s = 0;
//        auto t = n;
//
//        auto O = 0;
//        auto D = n;
//        int K = n + 1;
//        auto K_arc = K - 1;
//        std::vector<int> start_stage, end_stage;
//        for (int k = 1; k <= K; k++) {
//            if (k < K) {
//                start_stage.push_back(k);
//            }
//            if (k > 1) {
//                end_stage.push_back(k);
//            }
//        }
//        cplex.setParam(IloCplex::Param::Threads, cfg.num_thread);
//        // cplex.setParam(IloCplex::Param::MIP::Tolerances::Integrality, 0);
//        // Variable declaration
//        // X^i_k (binary variable) và nhận giá trị một tương ứng với đỉnh thứ k của
//        //đường đi của vehicle là i; k \in 1..n;
//        IloArray<IloBoolVarArray> X(env, K + 1);
//        for (int k = 1; k <= K; k++) {
//            X[k] = IloBoolVarArray(env, D + 1);
//            for (int i = 0; i <= D; i++) {
//                X[k][i] = IloBoolVar(env);
//                model.add(X[k][i]);
//                auto v_name = "X_" + std::to_string(k) + "_" + std::to_string(i);
//                X[k][i].setName(v_name.c_str());
//            }
//            if (k > 1) {
//                model.add(X[k][0] == 0);
//            }
//        }
//
//        model.add(X[1][0] == 1).setName("First stage must be source depot");
//
//        // x^k_(ij) (binary variable) và nhận giá trị một nếu Xk
//        // mô ta cạnh nối 2 đỉnh liên tiếp trên đường đi.
//        IloArray<IloArray<IloBoolVarArray>> x(env, K_arc + 1);
//        for (int k = 1; k <= K_arc; k++) {
//            x[k] = IloArray<IloBoolVarArray>(env, D);
//            for (int i = 0; i < D; i++) {
//                x[k][i] = IloBoolVarArray(env, D + 1);
//                for (int j = 1; j <= D; j++) {
//                    if (i != j) {
//                        x[k][i][j] = IloBoolVar(env);
//                        auto v_name = "x_t" + std::to_string(k) + "_" + std::to_string(i) + "_" + std::to_string(j);
//                        x[k][i][j].setName(v_name.c_str());
//                    }
//                }
//            }
//        }
//
//        //// phi^h equals to 1 if customer h is served by the drone
//        IloBoolVarArray phi(env, n);
//        for (int h : C) {
//            phi[h] = IloBoolVar(env);
//            auto v_name = "phi_" + std::to_string(h);
//            phi[h].setName(v_name.c_str());
//            if (exist(instance->heavy, h)) {
//                model.add(phi[h] == 0);
//            }
//        }
//
//        // arrival\departure variables a and d.
//        IloNumVarArray a(env, K + 1);
//        IloNumVarArray d(env, K + 1);
//        for (int k = 1; k <= K; k++) {
//            a[k] = IloNumVar(env, 0, IloInfinity, ILOFLOAT);
//            auto v_name = "a_" + std::to_string(k);
//            a[k].setName(v_name.c_str());
//            d[k] = IloNumVar(env, 0, IloInfinity, ILOFLOAT);
//            v_name = "d_" + std::to_string(k);
//            d[k].setName(v_name.c_str());
//            model.add(d[k] >= a[k]).setName(("C13_" + std::to_string(k)).c_str());
//        }
//
//        model.add(a[1] == 0).setName("arrival to depot at time 0");
//        model.add(d[1] == 0).setName("depart from depot at time 0");
//
//        // variable Z(k,k',i,j)
//        // drone bay từ (k, i), hạ cánh tại (k', j)
//        IloArray<IloArray<IloArray<IloBoolVarArray>>> Z(env, K);
//        for (int k = 1; k < K; k++) {
//            Z[k] = IloArray<IloArray<IloBoolVarArray>>(env, K + 1);
//            for (int kp = k + 1; kp <= K; kp++) {
//                Z[k][kp] = IloArray<IloBoolVarArray>(env, D);
//                for (int i : c_s) {
//                    Z[k][kp][i] = IloBoolVarArray(env, D + 1);
//                    for (int j : c_t) {
//                        Z[k][kp][i][j] = IloBoolVar(env);
//                        std::string vname = "Z_" + std::to_string(k) + "_" + std::to_string(kp) + "_" +
//                            std::to_string(i) +
//                            "_" + std::to_string(j);
//                        Z[k][kp][i][j].setName(vname.c_str());
//                        if (i == j) {
//                            model.add(Z[k][kp][i][j] == 0);
//                        }
//                    }
//                }
//                model.add(Z[k][kp][s][D] == 0);
//            }
//        }
//
//        // biến X_drone cho đường đi của drone: đi từ i đến j trong sortie từ stage (k -> k')
//        IloArray<IloArray<IloArray<IloBoolVarArray>>> X_drone(env, K);
//        for (int k = 1; k < K; k++) {
//            X_drone[k] = IloArray<IloArray<IloBoolVarArray>>(env, K + 1);
//            for (int kp = k + 1; kp <= K; kp++) {
//                X_drone[k][kp] = IloArray<IloBoolVarArray>(env, D);
//                for (int i : c_s) {
//                    X_drone[k][kp][i] = IloBoolVarArray(env, D + 1);
//                    for (int j : c_t) {
//                        X_drone[k][kp][i][j] = IloBoolVar(env);
//                        std::string vname = "X_drone_" + std::to_string(k) + "_" + std::to_string(kp) + "_" +
//                            std::to_string(i) +
//                            "_" + std::to_string(j);
//                        X_drone[k][kp][i][j].setName(vname.c_str());
//                        if (i == j) {
//                            model.add(X_drone[k][kp][i][j] == 0);
//                        }
//                    }
//                }
//                model.add(X_drone[k][kp][s][D] == 0);
//            }
//        }
//
//        IloArray<IloBoolVarArray> x_d(env, D);
//        for (int i : c_s) {
//            x_d[i] = IloBoolVarArray(env, D + 1);
//            for (int j : c_t) {
//                if (i != j) {
//                    std::string vname = "x_d_" + std::to_string(i) + "_" + std::to_string(j);
//                    x_d[i][j] = IloBoolVar(env);
//                    x_d[i][j].setName(vname.c_str());
//                }
//            }
//        }
//        // aux var z
//        //// aux var z_{k, k_p}: sortie launch from k and rendezvous at k_p.
//        IloArray<IloBoolVarArray> z(env, K);
//        for (int k = 1; k < K; k++) {
//            z[k] = IloBoolVarArray(env, K + 1);
//            for (int k_p = k + 1; k_p <= K; k_p++) {
//                z[k][k_p] = IloBoolVar(env);
//                auto v_name = "z_" + std::to_string(k) + "_" + std::to_string(k_p);
//                z[k][k_p].setName(v_name.c_str());
//            }
//        }
//
//        // Variable 11
//        // biến nhị phân mô tả khách hàng p được phục vụ bởi drone bay từ
//        // stage k tới stage k'
//        IloArray<IloArray<IloBoolVarArray>> zp(env, K);
//        for (int k : start_stage) {
//            zp[k] = IloArray<IloBoolVarArray>(env, K + 1);
//            for (int kp = k + 1; kp <= K; kp++) {
//                zp[k][kp] = IloBoolVarArray(env, D + 1);
//                for (int p : V) {
//                    zp[k][kp][p] = IloBoolVar(env);
//                    std::string vname = "zp_" + std::to_string(k) + "_" + std::to_string(kp) + "_" + std::to_string(p);
//                    zp[k][kp][p].setName(vname.c_str());
//                    if (exist(instance->heavy, p) || p == 0 || p == D) {
//                        model.add(zp[k][kp][p] == 0);
//                    }
//                }
//            }
//        }
//
//        // PREPROCESSING
//        ////-----------------------------------------------------------------------------------------------
//
//        //// Sortie maximum stage gap calculation.
//        /// Find maximum stage gap that a sortie can start and rendezvous.
//        /// For each stage gap in increasing order (k' - k), find the minimum tour length from stage k to k'.
//        /// Revisit(s) are also considered.
//        /// If the tour length > cfg.dtl - cfg.sr => there can be no sortie with stage gap greater or equal k' - k.
//        //     bool used_stage_gap = false;
//        //     int min_stage_gap = 0;
//        //     std::cout << "---------------------------------------------" << std::endl;
//        //     std::cout << "Calculate max stage gap for sortie.... " << std::endl;
//        //     for (int k = 1; k < K; k++) {
//        //         for (int kp = k + 1; kp <= K; kp++) {
//        //             if (kp > k + 6) {
//        //                 goto after_z_cons;
//        //             }
//        //             double smallest_tour = smallest_tour_length(kp - k, tau, V);
//        //             if (smallest_tour > cfg.dtl - cfg.sr) {
//        //                 std::cout << "Stage gap of " << kp - k << " with smallest length = " << smallest_tour <<
//        //                     " violated endurance constraint!" << std::endl;
//        //                 std::cout << "---------------------------------------------" << std::endl;
//        //                 min_stage_gap = kp - k;
//        //                 goto after_z_cons;
//        //             }
//        //         }
//        //     }
//        // after_z_cons:
//        //     if (min_stage_gap == 0) {
//        //         std::cout << "Stage gap calculation consumes too much memory. No constraint was added." << std::endl;
//        //         std::cout << "---------------------------------------------" << std::endl;
//        //     }
//        //     if (min_stage_gap != 0) {
//        //         used_stage_gap = true;
//        //         for (int k = 1; k < K; k++) {
//        //             for (int kp = k + min_stage_gap; kp <= K; kp++) {
//        //                 model.add(z[k][kp] == 0);
//        //                 std::cout << "Variable z[" << k << "][" << kp << "] was set to 0." << std::endl;
//        //                 for (int h : instance->c_prime) {
//        //                     model.add(zp[k][kp][h] == 0);
//        //
//        //                     std::string cname =
//        //                         "Can't serve customer " + std::to_string(h) + " start at stage " + std::to_string(k) +
//        //                         ", end at stage " + std::to_string(kp);
//        //                     // Looks like quicker lower bound improvements with this constraints.
//        //                 }
//        //             }
//        //         }
//        //     }
//
//        auto sset_cp = generateSetsAndComplements(instance->c_prime);
//        for (auto& ss : sset_cp) {
//            if (ss.first.size() < 2 || ss.first.size() > cfg.L) {
//                continue;
//            }
//
//            for (int k : start_stage) {
//                for (int kp = k + 1; kp <= K; kp++) {
//                    IloExpr sumxp(env);
//                    for (auto c : ss.first) {
//                        sumxp += zp[k][kp][c];
//                    }
//                    IloExpr sum_x_d_i_j(env);
//                    for (int i : ss.first) {
//                        for (int j : ss.first) {
//                            if (i != j) {
//                                sum_x_d_i_j += X_drone[k][kp][i][j];
//                            }
//                        }
//                    }
//                    int size = ss.first.size();
//                    // subtour elimination constraints.
//                    // model.add(sum_x_d_i_j <= size - z[k][kp]);
//                }
//            }
//        }
//
//        // Constraints definition
//        // Numbering based on Overleaf.////////
//
//        // Constraint 0 - forcing constraint
//        for (int k : start_stage) {
//            for (int kp = k + 1; kp <= K; kp++) {
//                for (int i : c_s) {
//                    for (int j : c_t) {
//                        if (i != j) {
//                            std::string cname = "C0_" + std::to_string(k) + "_" + std::to_string(kp) + "_" +
//                                std::to_string(i) + "_" + std::to_string(j);
//                            model.add(X_drone[k][kp][i][j] <= z[k][kp]).setName(cname.c_str());
//                        }
//                    }
//                }
//            }
//        }
//
//        // Constraint 1
//        // mỗi cặp stage, drone chỉ bay dc tối đa một lần.
//        for (int k : start_stage) {
//            for (int kp = k + 1; kp <= K; kp++) {
//                IloExpr sumZ(env);
//                for (int i : c_s) {
//                    for (int j : c_t) {
//                        sumZ += Z[k][kp][i][j];
//                    }
//                }
//                std::string cname = "C1_(k,k')_(" + std::to_string(k) + "," + std::to_string(kp) + ")";
//                model.add(z[k][kp] == sumZ).setName(cname.c_str());
//                model.add(sumZ <= 1);
//            }
//        }
//
//        // Constraint 2
//        // drone bay theo mỗi cạnh $(i,j)$ ko quá 1 lần
//        for (int i : c_s) {
//            for (int j : c_t) {
//                if (i != j) {
//                    IloExpr sum_xd_i_j(env);
//                    for (int k : start_stage) {
//                        for (int kp = k + 1; kp <= K; kp++) {
//                            sum_xd_i_j += X_drone[k][kp][i][j];
//                        }
//                    }
//                    model.add(x_d[i][j] == sum_xd_i_j);
//                    model.add(sum_xd_i_j <= 1);
//                }
//            }
//        }
//
//        // Constraint 2'
//        for (int i : c_s) {
//            for (int j : c_t) {
//                if (i != j) {
//                    IloExpr sum_x_k(env);
//                    for (int k = 1; k < K; k++) {
//                        sum_x_k += x[k][i][j];
//                    }
//                    model.add(sum_x_k <= K * (1 - x_d[i][j]));
//                }
//            }
//        }
//
//
//        // Constraint 3
//        // nếu có drone bay từ (k,i) và hạ cánh tại stage k' thì phải có một cạnh đi ra từ node i của drone
//        for (int k = 1; k < K; k++) {
//            for (int i : c_s) {
//                for (int kp = k + 1; kp <= K; kp++) {
//                    std::string cname = "C3_(k,k',i)_(" + std::to_string(k) + "," + std::to_string(kp) + "," +
//                        std::to_string(i) + ")";
//                    IloExpr sumZ(env), sum_X_drone(env);
//                    for (int j : c_t) {
//                        if (i != j) {
//                            sumZ += Z[k][kp][i][j];
//                            sum_X_drone += x_d[i][j];
//                        }
//                    }
//                    // model.add(zp[k][kp][i] + sumZ == sum_X_drone).setName(cname.c_str());
//                    // model.add(sum_X_drone <= 1);
//                }
//            }
//        }
//
//        // Constraint 4
//        // nếu có drone bay từ stage k và hạ cánh tại stage k', node j thì phải có một cạnh đi vào node j của drone.
//        for (int kp = 2; kp <= K; kp++) {
//            for (int j : c_t) {
//                for (int k = 1; k < kp; k++) {
//                    std::string cname = "C4_(k,k',j)_(" + std::to_string(k) + "," + std::to_string(kp) + "," +
//                        std::to_string(j) + ")";
//                    IloExpr sumZ(env), sum_X_drone(env);
//                    for (int i : c_s) {
//                        if (i != j) {
//                            sumZ += Z[k][kp][i][j];
//                            sum_X_drone += x_d[i][j];
//                        }
//                    }
//                    // model.add(zp[k][kp][j] + sumZ == sum_X_drone).setName(cname.c_str());
//                    // model.add(sum_X_drone <= 1);
//                }
//            }
//        }
//
//        // Constraint 7
//        // Ràng buộc thời gian bay của drone.
//        for (int k = 1; k < K; k++) {
//            for (int kp = k + 1; kp <= K; kp++) {
//                std::string cname = "C7_(k,k')_(" + std::to_string(k) + ", " + std::to_string(kp) + ")";
//                IloExpr sum_drone_movement(env);
//                for (int i : c_s) {
//                    for (int j : c_t) {
//                        if (i != j) {
//                            sum_drone_movement += X_drone[k][kp][i][j] * tau_prime[i][j];
//                        }
//                    }
//                }
//                model.add(sum_drone_movement <= (cfg.dtl - cfg.sr)).setName(cname.c_str());
//            }
//        }
//
//        // Constraint 8
//        // Ràng buộc về số cạnh di chuyển trong 1 sortie của drone: <= L+1. L là số khách hàng tối đa trong 1 sortie.
//        for (int k = 1; k < K; k++) {
//            for (int kp = k + 1; kp <= K; kp++) {
//                std::string cname = "C8_(k,k')_(" + std::to_string(k) + "," + std::to_string(kp) + ")";
//                IloExpr sum_X_drone(env);
//                for (int i : c_s) {
//                    for (int j : c_t) {
//                        if (i != j) {
//                            sum_X_drone += X_drone[k][kp][i][j];
//                        }
//                    }
//                }
//                model.add(2 * z[k][kp] <= sum_X_drone).setName(cname.c_str());
//                model.add(sum_X_drone <= (cfg.L + 1) * z[k][kp]);
//            }
//        }
//
//        // Constraint 10
//        // customer j chỉ được thăm bằng truck hoặc drone..
//        for (int j : C) {
//            std::string cname = "C10_j_" + std::to_string(j);
//            IloExpr sumX(env);
//            for (int k = 2; k < K; k++) {
//                sumX += X[k][j];
//            }
//            model.add(sumX <= K * (1 - phi[j])).setName(cname.c_str());
//        }
//
//        // Constraint 12: khách hàng j được phục vụ duy nhất 1 lần bởi drone.
//        for (int j : instance->c_prime) {
//            IloExpr sum_zp(env);
//            for (int k : start_stage) {
//                for (int kp = k + 1; kp <= K; kp++) {
//                    sum_zp += zp[k][kp][j];
//                }
//            }
//            model.add(phi[j] == sum_zp);
//        }
//
//        // Constraint 13
//        // Cân bằng customer j.
//        for (int k : start_stage) {
//            for (int kp = k + 1; kp <= K; kp++) {
//                for (int h : C) {
//                    IloExpr leave_h(env), enter_h(env);
//                    IloExpr start_at_h_k_kp(env), end_at_h_k_kp(env);
//
//                    for (int i : c_s) {
//                        if (i != h) {
//                            enter_h += X_drone[k][kp][i][h];
//                            end_at_h_k_kp += Z[k][kp][i][h];
//                        }
//                    }
//                    for (int j : c_t) {
//                        if (j != h) {
//                            leave_h += X_drone[k][kp][h][j];
//                            start_at_h_k_kp += Z[k][kp][h][j];
//                        }
//                    }
//
//                    std::string cname = "C13_(k,k',j)_(" + std::to_string(k) + "," + std::to_string(kp) + "," +
//                        std::to_string(h) + ")";
//                    model.add(2 * zp[k][kp][h] + start_at_h_k_kp + end_at_h_k_kp == leave_h + enter_h).setName(
//                        cname.c_str());
//                }
//            }
//        }
//
//        // Constraint 16
//        for (int j : instance->c_prime) {
//            for (int k : start_stage) {
//                for (int kp = k + 1; kp <= K; kp++) {
//                    for (int i : c_s) {
//                        if (i != j) {
//                            model.add(phi[j] + Z[k][kp][i][j] <= 1);
//                        }
//                    }
//                }
//            }
//        }
//
//        // Constraint 17
//        for (int i : instance->c_prime) {
//            for (int k : start_stage) {
//                for (int kp = k + 1; kp <= K; kp++) {
//                    for (int j : c_t) {
//                        if (i != j) {
//                            model.add(phi[i] + Z[k][kp][i][j] <= 1);
//                        }
//                    }
//                }
//            }
//        }
//
//        // Constraint 14
//        for (int k : start_stage) {
//            for (int i : c_s) {
//                IloExpr sumZ(env);
//                for (int kp = k + 1; kp <= K; kp++) {
//                    for (int j : c_t) {
//                        if (i != j) {
//                            sumZ += Z[k][kp][i][j];
//                        }
//                    }
//                }
//                model.add(sumZ <= X[k][i]);
//            }
//        }
//
//        // Constraint 15
//        for (int kp : end_stage) {
//            for (int j : c_t) {
//                IloExpr sumZ(env);
//                for (int k = 1; k < kp; k++) {
//                    for (int i : c_s) {
//                        if (i != j) {
//                            sumZ += Z[k][kp][i][j];
//                        }
//                    }
//                }
//                model.add(sumZ <= X[kp][j]);
//            }
//        }
//
//        // Constraint 18
//        // nếu (i , j) là điểm thả/bắt của sortie, drone không thể đi cạnh i -> j.
//        for (int k : start_stage) {
//            for (int kp = k + 1; kp <= K; kp++) {
//                for (int i : c_s) {
//                    for (int j : c_t) {
//                        if (i != j) {
//                            model.add(Z[k][kp][i][j] + x_d[i][j] <= 1);
//                        }
//                    }
//                }
//            }
//        }
//
//        // Constraint 19
//        for (int i : C) {
//            for (int j : C) {
//                if (i != j) {
//                    model.add(x_d[i][j] + x_d[j][i] <= 1);
//                }
//            }
//        }
//
//        for (int k = 1; k < K; k++) {
//            for (int i = 0; i < D; i++) {
//                IloExpr sum(env);
//                for (int j = 1; j <= D; j++) {
//                    if (i != j) {
//                        sum += x[k][i][j];
//                    }
//                }
//                model.add(X[k][i] == sum).setName(("C11_" + std::to_string(k) + "_" + std::to_string(i)).c_str());
//            }
//        }
//
//        for (int k = 2; k <= K; k++) {
//            for (int i = 1; i <= D; i++) {
//                IloExpr sum(env);
//                for (int j = 0; j < D; j++) {
//                    if (i != j) {
//                        sum += x[k - 1][j][i];
//                    }
//                }
//                // arcs entering i at stage k.
//                model.add(X[k][i] == sum).setName(("C12_" + std::to_string(k) + "_" + std::to_string(i)).c_str());
//            }
//        }
//
//        // C2: truck must depart from the depot at stage 1.
//        IloExpr C2(env);
//        for (int i = 1; i <= D; i++) {
//            C2 += x[1][O][i];
//        }
//
//        IloConstraint c2(C2 == 1);
//        model.add(c2).setName("Must start from source");
//
//        // C3: Terminal depot must be arrived only once.
//        IloExpr C3(env);
//        for (int k = 2; k <= K; k++) {
//            C3 += X[k][D];
//        }
//        model.add(C3 == 1).setName("Must visit terminal depot once");
//
//        // At most ONE node each stage.
//        for (int k = 1; k <= K; k++) {
//            IloExpr sum(env);
//            for (int i = 0; i <= D; i++)
//                sum += X[k][i];
//            model.add(sum <= 1).setName(("C13_at_most_one_customer_at_stage_" + std::to_string(k)).c_str());
//        }
//
//        IloBoolVarArray R(env, K + 1);
//        for (int k = 1; k < K; k++) {
//            R[k].setName(("R_" + std::to_string(k)).c_str());
//        }
//        // crossing constraint?
//        for (int k = 1; k < K; k++) {
//            IloExpr expr(env);
//            IloExpr expr_r2(env);
//            for (int k_p = k + 1; k_p <= K; k_p++) {
//                expr += z[k][k_p];
//            }
//            model.add(R[k] == expr).setName(("C14_" + std::to_string(k)).c_str());
//        }
//
//        // modified C7
//        for (int k = 1; k <= K - 1; k++) {
//            for (int k_p = k + 1; k_p <= K; k_p++) {
//                for (int l = k + 1; l < k_p; l++) {
//                    // tranh drone bay cac doan giao nhau.
//                    if (k < l) {
//                        model.add(z[k][k_p] + R[l] <= 1).setName(("C7m_" + std::to_string(k)
//                                + "_" + std::to_string(k_p) + "_" + std::to_string(l))
//                            .c_str());
//                    }
//                }
//            }
//        }
//
//        for (int h : C) {
//            IloExpr sumX(env);
//            for (int k = 2; k < K; k++) {
//                sumX += X[k][h];
//            }
//            model.add(sumX + phi[h] >= 1);
//        }
//
//        /////////// C14: node_stage
//        for (int k = 1; k <= K_arc; k++) {
//            IloExpr sum(env);
//            for (int i = 0; i < D; i++) {
//                for (int j = 1; j <= D; j++) {
//                    if (i != j) {
//                        sum += x[k][i][j] * tau[i][j];
//                    }
//                }
//            }
//
//            model.add(a[k + 1] == d[k] + sum).setName(
//                ("C14_" + std::to_string(k) + "_" + std::to_string(k + 1)).c_str());
//        }
//        double M = 0;
//        for (int i = 0; i < n; i++) {
//            for (int j = 0; j < n; j++) {
//                if (i < j) {
//                    M += tau[i][j];
//                }
//            }
//        }
//
//        for (int k : start_stage) {
//            for (int k_p = k + 1; k_p <= K; k_p++) {
//                if (k < k_p) {
//                    model.add(a[k_p] - d[k] <= z[k][k_p] * (cfg.dtl - cfg.sr) + (1 - z[k][k_p]) * M).setName(
//                        ("C15_" + std::to_string(k) + "_" + std::to_string(k_p)).c_str());
//                    IloExpr truck_length(env);
//                    for (int start = k; start < k_p; start++) {
//                        for (int i : c_s) {
//                            for (int j : c_t) {
//                                if (i != j) {
//                                    truck_length += x[start][i][j] * tau[i][j];
//                                }
//                            }
//                        }
//                    }
//                    model.add(truck_length <= z[k][k_p] * (cfg.dtl - cfg.sr) + (1 - z[k][k_p]) * M);
//                }
//            }
//        }
//
//        // Drone movement time constraint.
//        for (int k : start_stage) {
//            for (int kp = k + 1; kp <= K; kp++) {
//                IloExpr sum_drone_travel(env);
//                IloExpr start_at_kp(env);
//                for (int i : c_s) {
//                    for (int j : c_t) {
//                        if (i != j) {
//                            sum_drone_travel += X_drone[k][kp][i][j] * tau_prime[i][j];
//                        }
//                    }
//                }
//                for (int l = kp + 1; l <= K; l++) {
//                    start_at_kp += z[kp][l];
//                }
//                model.add(d[kp] >= d[k] + sum_drone_travel + cfg.sr + start_at_kp * cfg.sl - (1 - z[k][kp]) * M);
//            }
//        }
//
//        for (int kp = 2; kp <= K; kp++) {
//            IloExpr sum_truck_move(env);
//            for (int i : c_s) {
//                for (int j : c_t) {
//                    if (i != j) {
//                        sum_truck_move += x[kp - 1][i][j] * tau[i][j];
//                    }
//                }
//            }
//            IloExpr end_at_kp(env);
//            IloExpr start_at_kp(env);
//
//            for (int k = 1; k < kp; k++) {
//                end_at_kp += z[k][kp];
//            }
//            for (int l = kp + 1; l <= K; l++) {
//                start_at_kp += z[kp][l];
//            }
//            model.add(d[kp] >= d[kp - 1] + sum_truck_move + start_at_kp * cfg.sl + end_at_kp * cfg.sr);
//        }
//
//        for (int k = 1; k < K; k++) {
//            for (int i : c_s) {
//                IloExpr sumZ(env);
//                for (int kp = k + 1; kp <= K; kp++) {
//                    for (int j : c_t) {
//                        if (i != j) {
//                            sumZ += Z[k][kp][i][j];
//                        }
//                    }
//                }
//                model.add(sumZ <= X[k][i]);
//            }
//        }
//
//        for (int kp = 2; kp <= K; kp++) {
//            for (int j : c_t) {
//                IloExpr sumZ(env);
//                for (int k = 1; k < kp; k++) {
//                    for (int i : c_s) {
//                        if (i != j) {
//                            sumZ += Z[k][kp][i][j];
//                        }
//                    }
//                }
//                model.add(sumZ <= X[kp][j]);
//            }
//        }
//
//        // Lower bound constraint
//        IloExpr truck_length(env), drone_length(env);
//        for (int i : c_s) {
//            for (int j : c_t) {
//                if (i != j) {
//                    for (int k = 1; k < K_arc; k++) {
//                        truck_length += x[k][i][j] * tau[i][j];
//                    }
//                    drone_length += x_d[i][j] * tau_prime[i][j];
//                }
//            }
//        }
//        model.add(d[K] >= truck_length);
//        model.add(d[K] >= drone_length);
//
//        model.add(IloMinimize(env, d[K]));
//
//        cplex.exportModel("rv-mvd.lp");
//        auto startTime = std::chrono::high_resolution_clock::now();
//        if (cfg.use_cutting_plane) {
//            while (true) {
//                if (!cplex.solve()) {
//                    std::cout << "Unable to optimize model!" << std::endl;
//                }
//                for (int k = 1; k < K; k++) {
//                    for (int kp = k + 1; kp <= K; kp++) {
//                        for (int i : c_s) {
//                            for (int j : c_t) {
//                                if (cplex.getValue(Z[k][kp][i][j]) == 1) {
//                                    std::cout << "check z: " << k << " " << kp << " " << i << " " << j << std::endl;
//                                    std::cout << "-------------------------------------------------" << std::endl;
//                                    std::cout << "Start at stage " << k << " at node " << i << std::endl;
//                                    std::cout << "End at stage " << kp << " at node " << j << std::endl;
//                                    int start_node = i, end_node = j;
//                                    // Route in this sortie
//                                    std::vector<std::vector<int>> sortie_tour(D + 1);
//                                    for (int index = 0; index < D + 1; index++) {
//                                        sortie_tour[index].resize(D + 1);
//                                    }
//                                    for (int start : c_s) {
//                                        for (int end : c_t) {
//                                            if (start != end) {
//                                                if (cplex.getValue(X_drone[k][kp][start][end]) == 1) {
//                                                    sortie_tour[start][end] = 1;
//                                                }
//                                            }
//                                        }
//                                    }
//                                    auto cycles = find_cycles(sortie_tour);
//                                    if (cycles.size() == 0) {
//                                        goto LABEL_DONE;
//                                    }
//                                    std::cout << "Number of cycles: " << cycles.size() << std::endl;
//                                    for (auto& cyc : cycles) {
//                                        std::cout << "Printing cycle:";
//                                        setPrint(cyc);
//                                        IloExpr sum(env);
//                                        for (int c_i : cyc) {
//                                            for (int c_j : cyc) {
//                                                sum += X_drone[k][kp][c_i][c_j];
//                                            }
//                                        }
//                                        int cycle_size = cyc.size();
//                                        model.add(sum <= cycle_size - 1);
//                                    }
//
//
//                                    std::cout << "-------------------------------------------------" << std::endl;
//                                }
//                            }
//                        }
//                    }
//                }
//            }
//        }
//        else {
//            cplex.solve();
//            if (cfg.use_cplex_conflict_refiner) {
//                // A list of constraints to be considered by the conflict refiner.
//                IloConstraintArray constraints(env);
//
//                // Loop over all objects in the model and gather constraints.
//                for (IloModel::Iterator it(model); it.ok(); ++it) {
//                    IloExtractable ext = *it;
//                    if (ext.isVariable()) {
//                        IloNumVar v = ext.asVariable();
//                        // Add variable bounds to the constraints array.
//                        constraints.add(IloBound(v, IloBound::Lower));
//                        constraints.add(IloBound(v, IloBound::Upper));
//                    }
//                    else if (ext.isConstraint()) {
//                        IloConstraint c = ext.asConstraint();
//                        constraints.add(c);
//                    }
//                }
//
//                // Define preferences for the constraints. Here, we give all
//                // constraints a preference of 1.0, so they will be treated
//                // equally.
//                IloNumArray prefs(env, constraints.getSize());
//                for (int i = 0; i < prefs.getSize(); ++i)
//                    prefs[i] = 1.0;
//
//                // Run the conflict refiner. As opposed to letting the conflict
//                // refiner run to completion (as is done here), the user can set
//                // a resource limit (e.g., a time limit, an iteration limit, or
//                // node limit) and still potentially get a "possible" conflict.
//                if (cplex.refineConflict(constraints, prefs)) {
//                    // Display the solution status.
//                    IloCplex::CplexStatus status = cplex.getCplexStatus();
//                    std::cout << "Solution status = " << status << " (" <<
//                        static_cast<int>(status) << ")" << std::endl;
//
//                    // Get the conflict status for the constraints that were specified.
//                    IloCplex::ConflictStatusArray conflict = cplex.getConflict(constraints);
//
//                    // Print constraints that participate in the conflict.
//                    std::cout << "Conflict:" << std::endl;
//
//                    for (int i = 0; i < constraints.getSize(); ++i) {
//                        if (conflict[i] == IloCplex::ConflictMember ||
//                            conflict[i] == IloCplex::ConflictPossibleMember) {
//                            std::cout << "  " << constraints[i] << std::endl;
//                        }
//                    }
//
//                    std::cout << std::endl;
//
//                    // Write the identified conflict in the LP format.
//                    const char* confFile = "iloconflictex1.lp";
//                    std::cout << "Writing conflict file to '" << confFile << "'...." << std::endl;
//                    cplex.writeConflict(confFile);
//
//                    // Display the entire conflict subproblem.
//                    std::string line;
//                    std::ifstream file(confFile);
//                    if (file.is_open()) {
//                        while (getline(file, line)) {
//                            std::cout << line << std::endl;
//                        }
//                    }
//                    file.close();
//                }
//                else {
//                    std::cout << "A conflict was not identified." << std::endl;
//                    std::cout << "Exiting...." << std::endl;
//                }
//            }
//        }
//    LABEL_DONE:
//        auto endTime = std::chrono::high_resolution_clock::now();
//        // Calculate the elapsed time
//        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
//
//        std::cout << "Feasible solution found!" << std::endl;
//        std::cout << "Truck nodes:" << std::endl;
//        for (int k = 1; k <= K; k++) {
//            for (int i = 0; i <= D; i++) {
//                auto X_val = cplex.getValue(X[k][i]);
//                if (X_val == 1) {
//                    auto d_k = cplex.getValue(d[k]);
//                    auto a_k = cplex.getValue(a[k]);
//                    std::cout << "(" << a_k << ")-" << i << "(" << k << ")-(" << d_k << ")----->";
//                    break;
//                }
//            }
//        }
//        std::cout << std::endl << "Truck arcs:" << std::endl;
//        std::map<int, std::pair<int, int>> map_stage_truck_arc;
//        for (int k = 1; k <= K_arc; k++) {
//            for (int i = 0; i < D; i++) {
//                for (int j = 1; j <= D; j++) {
//                    if (i != j) {
//                        auto X_val = cplex.getValue(x[k][i][j]);
//                        if (X_val == 1) {
//                            std::cout << "Arc " << k << " connecting " << i << " and " << j
//                                << " with cost " << tau[i][j] << " " << std::endl;
//
//                            map_stage_truck_arc[k] = std::make_pair(i, j);
//                            break;
//                        }
//                    }
//                }
//            }
//        }
//        int phi_check = 0;
//
//        for (int h : instance->c_prime) {
//            if (cplex.getValue(phi[h]) == 1) {
//                std::cout << "Customer " << h << " was served by drone!" << std::endl;
//                phi_check += 1;
//            }
//        }
//
//        std::cout << "Sorties information: ";
//        if (phi_check == 0) {
//            std::cout << "No sortie!" << std::endl;
//        }
//        else {
//            std::cout << phi_check << " customer(s) served by drone." << std::endl;
//        }
//        for (int k = 1; k < K; k++) {
//            for (int kp = k + 1; kp <= K; kp++) {
//                for (int i : c_s) {
//                    for (int j : c_t) {
//                        if (cplex.getValue(Z[k][kp][i][j]) == 1) {
//                            std::cout << "check z: " << k << " " << kp << " " << i << " " << j << std::endl;
//                            std::cout << "-------------------------------------------------" << std::endl;
//                            std::cout << "Start at stage " << k << " at node " << i << std::endl;
//                            std::cout << "End at stage " << kp << " at node " << j << std::endl;
//                            int start_node = i, end_node = j;
//                            // Route in this sortie
//                            for (int start : c_s) {
//                                for (int end : c_t) {
//                                    if (start != end) {
//                                        if (cplex.getValue(X_drone[k][kp][start][end]) == 1) {
//                                            std::cout << start << "--->" << end << std::endl;
//                                        }
//                                    }
//                                }
//                            }
//                            std::cout << "-------------------------------------------------" << std::endl;
//                        }
//                    }
//                }
//            }
//        }
//
//        std::cout << "Drone path:" << std::endl;
//        for (int i : c_s) {
//            for (int j : c_t) {
//                if (i != j) {
//                    if (cplex.getValue(x_d[i][j]) == 1) {
//                        std::cout << i << " " << j << std::endl;
//                    }
//                }
//            }
//        }
//
//        std::cout << "---------------------------------------------------------------" << std::endl;
//        std::cout << "Drone path based on X_drone_k_kp_i_j: " << std::endl;
//        for (int k : start_stage) {
//            for (int kp = k + 1; kp <= K; kp++) {
//                for (int i : c_s) {
//                    for (int j : c_t) {
//                        if (i != j) {
//                            if (cplex.getValue(X_drone[k][kp][i][j]) == 1) {
//                                std::cout << "X_drone[" << k << "][" << kp << "][" << i << "][" << j << "] = 1" <<
//                                    std::endl;
//                            }
//                        }
//                    }
//                }
//            }
//        }
//        std::cout << "---------------------------------------------------------------" << std::endl;
//        std::cout << "Print zp:" << std::endl;
//        for (int j : instance->c_prime) {
//            for (int k : start_stage) {
//                for (int kp = k + 1; kp <= K; kp++) {
//                    if (cplex.getValue(zp[k][kp][j]) == 1) {
//                        std::cout << "zp[" << k << "][" << kp << "][" << j << "] = 1" << std::endl;
//                    }
//                }
//            }
//        }
//
//        std::cout << "---------------------------------------------------------------" << std::endl;
//        std::cout << "Print z:" << std::endl;
//        for (int k : start_stage) {
//            for (int kp = k + 1; kp <= K; kp++) {
//                if (cplex.getValue(z[k][kp]) == 1) {
//                    std::cout << "z[" << k << "][" << kp << "] = 1" << std::endl;
//                }
//            }
//        }
//        std::cout << "Done!" << std::endl;
//        // std::cout << "-------------------------Re-calculated objective-----------------------" << std::endl;
//        // std::cout << obj << std::endl;
//        // std::cout << "------------------------------------------------------------------------" << std::endl;
//
//        double c = cplex.getObjValue();
//        double gap = cplex.getMIPRelativeGap();
//        std::cout << "Solve time: " << duration.count() / 1000.0 << std::endl;
//        cplex.end();
//        model.end();
//        env.end();
//
//
//        std::cout << "OBJECTIVE VALUE: " << c << ", customer(s) served by drone: " << phi_check << "." << std::endl;
//        return Result{c, duration.count() / 1000.0, gap};
//    }
//    catch (IloException& e) {
//        std::cout << e.getMessage() << std::endl;
//    }
//}
//
//// cutting plane not invoked!
//
//
//Result FSTSPSolver::RV_FSTSP_MVD_Subset(Config& cfg) const {
//    auto tau = instance->tau;
//    auto tau_prime = instance->tau_prime;
//    auto n = instance->num_node;
//    std::vector<int> C;
//    std::vector<int> V;
//    std::vector<int> c_s;
//    std::vector<int> c_t;
//    for (int i = 0; i < n + 1; i++) {
//        if (i != 0 && i != n) {
//            C.push_back(i);
//        }
//        if (i != 0) {
//            c_t.push_back(i);
//        }
//        if (i != n) {
//            c_s.push_back(i);
//        }
//        V.push_back(i);
//    }
//
//    IloEnv env;
//    IloModel model(env);
//    IloCplex cplex(model);
//    cplex.setParam(IloCplex::Param::MIP::Tolerances::Integrality, 0);
//    cplex.setParam(IloCplex::Param::Threads, cfg.num_thread);
//    auto O = 0;
//    auto D = n;
//    auto K = n + 1;
//    auto K_arc = K - 1;
//
//    std::vector<MultivisitTuple> all_sortie;
//
//    // get set D = {i, Y, j}: {i \in c_s, j \in c_t, Y subset of C', |Y| <= L}
//    for (int n_customer = 1; n_customer <= cfg.L; n_customer++) {
//        for (int i = 0; i < D; i++) {
//            for (int j = 1; j <= D; j++) {
//                if (i != j) {
//                    auto mt = shortest_L_visit_sorties(i, j, n_customer, instance->c_prime, tau_prime, cfg.dtl, cfg.sr);
//                    for (auto& m : mt) {
//                        all_sortie.push_back(m);
//                    }
//                }
//            }
//        }
//    }
//
//    all_sortie.shrink_to_fit();
//    int s_index = 0;
//    for (auto& H : all_sortie) {
//        H.index = s_index;
//        s_index++;
//    }
//
//    auto S = all_sortie.size();
//    std::cout << "Number of possible sorties: " << S << std::endl;
//    std::vector<std::vector<int>> start_with_i(D), end_with_j(D + 1);
//    for (int i = 0; i <= D; i++) {
//        if (i < D) {
//            auto s_i = getSortiesFromLR(i, -1, all_sortie);
//            for (auto& s : s_i) {
//                start_with_i[i].push_back(s.index);
//            }
//        }
//        if (i > 0) {
//            auto s_j = getSortiesFromLR(-1, i, all_sortie);
//            for (auto& s : s_j) {
//                end_with_j[i].push_back(s.index);
//            }
//        }
//    }
//
//    std::unordered_map<std::pair<int, int>, std::vector<MultivisitTuple>, PairHash> start_i_end_j;
//    for (int i = 0; i < D; i++) {
//        for (int j = 1; j <= D; j++) {
//            if (i != j) {
//                start_i_end_j[std::make_pair(i, j)] = getSortiesFromLR(i, j, all_sortie);
//            }
//        }
//    }
//    std::vector<std::vector<int>> serve_h(D);
//    for (auto& H : all_sortie) {
//        for (int h : instance->c_prime) {
//            if (exist(H.serve_customer, h)) {
//                serve_h[h].push_back(H.index);
//            }
//        }
//    }
//
//
//    /// Variable declaration
//    // X^i_k (binary variable) và nhận giá trị một tương ứng với đỉnh thứ k của
//    //đường đi của vehicle là i; k \in 1..n;
//    IloArray<IloBoolVarArray> X(env, K + 1);
//    for (int k = 1; k <= K; k++) {
//        X[k] = IloBoolVarArray(env, D + 1);
//        for (int i = 0; i <= D; i++) {
//            X[k][i] = IloBoolVar(env);
//            model.add(X[k][i]);
//            auto v_name = "X_" + std::to_string(k) + "_" + std::to_string(i);
//            //std::cout << v_name << std::endl;
//            X[k][i].setName(v_name.c_str());
//        }
//        if (k > 1) {
//            model.add(X[k][0] == 0);
//        }
//    }
//
//    model.add(X[1][0] == 1).setName("First stage must be source depot");
//
//    // x^k_(ij) (binary variable) và nhận giá trị một nếu Xk
//    // mô ta cạnh nối 2 đỉnh liên tiếp trên đường đi.
//    IloArray<IloArray<IloBoolVarArray>> x(env, K_arc + 1);
//    for (int k = 1; k <= K_arc; k++) {
//        x[k] = IloArray<IloBoolVarArray>(env, D);
//        for (int i = 0; i < D; i++) {
//            x[k][i] = IloBoolVarArray(env, D + 1);
//            for (int j = 1; j <= D; j++) {
//                if (i != j) {
//                    x[k][i][j] = IloBoolVar(env);
//                    auto v_name = "x_" + std::to_string(k) + "_" + std::to_string(i) + "_" + std::to_string(j);
//                    x[k][i][j].setName(v_name.c_str());
//                }
//            }
//        }
//    }
//
//    model.add(x[1][0][D] == 0);
//
//    //// phi^h equals to 1 if customer h is served by the drone
//    IloBoolVarArray phi(env, n);
//    for (int h : C) {
//        phi[h] = IloBoolVar(env);
//        auto v_name = "phi_" + std::to_string(h);
//        phi[h].setName(v_name.c_str());
//        if (exist(instance->heavy, h)) {
//            model.add(phi[h] == 0);
//        }
//    }
//
//
//    IloArray<IloBoolVarArray> W(env, K + 1);
//    for (int kp = 2; kp <= K; kp++) {
//        W[kp] = IloBoolVarArray(env, all_sortie.size() + 1);
//        for (auto& H : all_sortie) {
//            W[kp][H.index] = IloBoolVar(env);
//        }
//    }
//
//    IloArray<IloBoolVarArray> Y(env, K + 1);
//    for (int k = 1; k < K; k++) {
//        Y[k] = IloBoolVarArray(env, all_sortie.size() + 1);
//        for (auto& H : all_sortie) {
//            Y[k][H.index] = IloBoolVar(env);
//        }
//    }
//
//    // arrival\departure variables a and d.
//    IloNumVarArray a(env, K + 1);
//    IloNumVarArray d(env, K + 1);
//    for (int k = 1; k <= K; k++) {
//        a[k] = IloNumVar(env);
//        auto v_name = "a_" + std::to_string(k);
//        a[k].setName(v_name.c_str());
//        d[k] = IloNumVar(env);
//        v_name = "d_" + std::to_string(k);
//        d[k].setName(v_name.c_str());
//        model.add(d[k] >= a[k]).setName(("C13_" + std::to_string(k)).c_str());
//    }
//
//    model.add(a[1] == 0).setName("arrival to depot at time 0");
//    model.add(d[1] == 0).setName("depart from depot at time 0");;
//
//    // $R_{k} = \sum_{k'}Z_{kk'}$: các đoạn bắt đầu từ k (C23)
//    IloBoolVarArray R(env, K + 1);
//    for (int k = 1; k < K; k++) {
//        R[k].setName(("R_" + std::to_string(k)).c_str());
//    }
//
//    //// aux var Z_{k, k_p, h}: sortie launch from k and rendezvous at k_p.
//    IloArray<IloArray<IloBoolVarArray>> Z(env, S + 1);
//    for (int H = 0; H < S; H++) {
//        Z[H] = IloArray<IloBoolVarArray>(env, K);
//        for (int k = 1; k < K; k++) {
//            Z[H][k] = IloBoolVarArray(env, K + 1);
//            for (int k_p = k + 1; k_p <= K; k_p++) {
//                Z[H][k][k_p] = IloBoolVar(env);
//                auto v_name = "Z_" + std::to_string(H) + "_" + std::to_string(k) + "_" + std::to_string(k_p);
//                Z[H][k][k_p].setName(v_name.c_str());
//            }
//        }
//    }
//    //// aux var z_{k, k_p}: sortie launch from k and rendezvous at k_p.
//    IloArray<IloBoolVarArray> z(env, K);
//    for (int k = 1; k < K; k++) {
//        z[k] = IloBoolVarArray(env, K + 1);
//        for (int k_p = k + 1; k_p <= K; k_p++) {
//            z[k][k_p] = IloBoolVar(env);
//            auto v_name = "z_" + std::to_string(k) + "_" + std::to_string(k_p);
//            z[k][k_p].setName(v_name.c_str());
//        }
//    }
//
//    ////-----------------------------------------------------------------------------------------------
//    // WARMSTART CONFIG: using original TSP solution.
//    // Obtained with MTZ formulation.
//    // On variable X.
//    if (cfg.use_tsp_for_warmstart) {
//        auto tsp_solution = TSP_MTZ(tau);
//        std::cout << tsp_solution.getSize() << std::endl;
//        IloNumVarArray X_warm_var(env);
//        IloNumArray X_warm_val(env);
//        for (int k = 1; k <= K - 1; k++) {
//            for (int i = 0; i <= D; i++) {
//                X_warm_var.add(X[k][i]);
//                if (tsp_solution[k - 1] == i) {
//                    X_warm_val.add(true);
//                    std::cout << "Warmstart X[" << k << "][" << i << "] == 1" << std::endl;
//                }
//                else {
//                    X_warm_val.add(false);
//                }
//            }
//        }
//        cplex.addMIPStart(X_warm_var, X_warm_val);
//        X_warm_var.end();
//        X_warm_val.end();
//    }
//    ////-----------------------------------------------------------------------------------------------
//
//    //// Sortie maximum stage gap calculation.
//    /// Find maximum stage gap that a sortie can start and rendezvous.
//    /// For each stage gap in increasing order (k' - k), find the minimum tour length from stage k to k'.
//    /// Revisit(s) are also considered.
//    /// If the tour length > cfg.dtl - cfg.sr => there can be no sortie with stage gap greater or equal k' - k.
//    bool used_stage_gap = false;
//    int min_stage_gap = 0;
//    std::cout << "---------------------------------------------" << std::endl;
//    std::cout << "Calculate max stage gap for sortie.... " << std::endl;
//    for (int k = 1; k < K; k++) {
//        for (int kp = k + 1; kp <= K; kp++) {
//            if (kp > k + 6) {
//                goto after_z_cons;
//            }
//            double smallest_tour = smallest_tour_length(kp - k, tau, V);
//            if (smallest_tour > cfg.dtl - cfg.sr) {
//                std::cout << "Stage gap of " << kp - k << " with smallest length = " << smallest_tour <<
//                    " violated endurance constraint!" << std::endl;
//                std::cout << "---------------------------------------------" << std::endl;
//                min_stage_gap = kp - k;
//                goto after_z_cons;
//            }
//        }
//    }
//after_z_cons:
//    if (min_stage_gap == 0) {
//        std::cout << "Stage gap calculation consumes too much memory. No constraint was added." << std::endl;
//        std::cout << "---------------------------------------------" << std::endl;
//    }
//    if (min_stage_gap != 0) {
//        used_stage_gap = true;
//        for (int k = 1; k < K; k++) {
//            for (int kp = k + min_stage_gap; kp <= K; kp++) {
//                model.add(z[k][kp] == 0);
//                std::cout << "Variable z[" << k << "][" << kp << "] was set to 0." << std::endl;
//                for (int H = 0; H < S; H++) {
//                    model.add(Z[H][k][kp] == 0);
//                }
//            }
//        }
//    }
//
//    // Constraint definition
//    model.add(X[1][O] == 1);
//    IloExpr c2(env);
//    for (int k = 2; k <= K; k++) {
//        c2 += X[k][D];
//    }
//    model.add(c2 == 1);
//
//    //  C3
//    for (int k = 1; k < K; k++) {
//        for (int i = 0; i < D; i++) {
//            IloExpr sum(env);
//            for (int j = 1; j <= D; j++) {
//                if (i != j) {
//                    sum += x[k][i][j];
//                }
//            }
//            model.add(X[k][i] == sum).setName(("C1_" + std::to_string(k) + "_" + std::to_string(i)).c_str());
//        }
//    }
//
//    for (int k = 2; k <= K; k++) {
//        for (int i = 1; i <= D; i++) {
//            IloExpr sum(env);
//            for (int j = 0; j < D; j++) {
//                if (i != j) {
//                    sum += x[k - 1][j][i];
//                }
//            }
//            // arcs entering i at stage k.
//            model.add(X[k][i] == sum).setName(("C1p_" + std::to_string(k) + "_" + std::to_string(i)).c_str());
//        }
//    }
//    // At most ONE node each stage.
//    for (int k = 1; k <= K; k++) {
//        IloExpr sum(env);
//        for (int i = 0; i <= D; i++)
//            sum += X[k][i];
//        model.add(sum <= 1).setName(("C20_at_most_one_customer_at_stage_" + std::to_string(k)).c_str());
//    }
//    // C4
//    for (int k = 1; k < K; k++) {
//        IloExpr lhs(env), rhs(env);
//        for (auto& H : all_sortie) {
//            for (int kp = k + 1; kp <= K; kp++) {
//                lhs += Z[H.index][k][kp];
//            }
//            rhs += Y[k][H.index];
//        }
//        model.add(lhs == rhs);
//    }
//
//    // C5
//    for (int kp = 2; kp <= K; kp++) {
//        IloExpr lhs(env), rhs(env);
//        for (auto& H : all_sortie) {
//            for (int k = 1; k < K; k++) {
//                lhs += Z[H.index][k][kp];
//            }
//            for (int i : c_s) {
//                rhs += W[kp][H.index];
//            }
//        }
//        model.add(lhs == rhs);
//    }
//
//    // C6
//    for (int h_i = 0; h_i < all_sortie.size(); h_i++) {
//        IloExpr start_H(env), end_H(env);
//        for (int k = 1; k < K; k++) {
//            start_H += Y[k][h_i];
//        }
//        for (int kp = 2; kp <= K; kp++) {
//            end_H += W[kp][h_i];
//        }
//        model.add(start_H == end_H);
//        model.add(start_H <= 1);
//    }
//
//    // C7
//    for (int k = 1; k < K; k++) {
//        IloExpr sum_z(env);
//        for (int kp = k + 1; kp <= K; kp++) {
//            IloExpr sum_Z(env);
//            for (auto& H : all_sortie) {
//                sum_Z += Z[H.index][k][kp];
//            }
//            sum_z += z[k][kp];
//            // C10
//            model.add(z[k][kp] == sum_Z);
//        }
//        model.add(sum_z <= 1);
//    }
//
//    // C8/9
//    for (int h : C) {
//        IloExpr sumY(env), sumW(env);
//        for (int k = 1; k < K; k++) {
//            for (int h_i : serve_h[h]) {
//                sumY += Y[k][h_i];
//            }
//        }
//        for (int kp = 2; kp <= K; kp++) {
//            for (int h_i : serve_h[h]) {
//                sumY += W[kp][h_i];
//            }
//        }
//        if (exist(instance->c_prime, h)) {
//            // model.add(phi[h] == sumY);
//            // model.add(phi[h] == sumW);
//            // causing bug.
//        }
//
//
//        // all must be served constraint
//        IloExpr sumX(env);
//        for (int k = 2; k < K; k++) {
//            sumX += X[k][h];
//        }
//        model.add(sumX + phi[h] >= 1);
//    }
//
//    for (int k = 1; k < K; k++) {
//        for (int i : c_s) {
//            IloExpr sumY(env);
//            for (auto h_i : start_with_i[i]) {
//                sumY += Y[k][h_i];
//            }
//            model.add(X[k][i] >= sumY);
//        }
//    }
//    for (int kp = 2; kp <= K; kp++) {
//        for (int j : c_t) {
//            IloExpr sumW(env);
//            for (auto h_i : end_with_j[j]) {
//                sumW += W[kp][h_i];
//            }
//            model.add(X[kp][j] >= sumW);
//        }
//    }
//    for (int k = 1; k < K; k++) {
//        IloExpr sumX(env), sum_z(env);
//        for (int i : c_s) {
//            sumX += X[k][i];
//        }
//        for (int kp = k + 1; kp <= K; kp++) {
//            sum_z += z[k][kp];
//        }
//        model.add(sumX >= sum_z);
//    }
//    for (int kp = 2; kp < K; kp++) {
//        IloExpr sumX(env), sum_z(env);
//        for (int j : c_t) {
//            sumX += X[kp][j];
//        }
//        for (int k = 1; k < kp; k++) {
//            sum_z += z[k][kp];
//        }
//        model.add(sumX >= sum_z);
//    }
//
//    // crossing constraint?
//    for (int k = 1; k < K; k++) {
//        IloExpr expr(env);
//        for (int k_p = k + 1; k_p <= K; k_p++) {
//            expr += z[k][k_p];
//        }
//        model.add(R[k] == expr).setName(("C23_" + std::to_string(k)).c_str());
//    }
//
//    // modified C7
//    for (int k = 1; k <= K - 1; k++) {
//        for (int k_p = k + 1; k_p <= K; k_p++) {
//            for (int l = k + 1; l < k_p; l++) {
//                // tranh drone bay cac doan giao nhau.
//                if (k < l) {
//                    model.add(z[k][k_p] + R[l] <= 1).setName(("C7m_" + std::to_string(k)
//                            + "_" + std::to_string(k_p) + "_" + std::to_string(l))
//                        .c_str());
//                }
//            }
//        }
//    }
//
//    for (int k = 1; k < K; k++) {
//        for (int i : c_s) {
//            IloExpr sumZHKK(env);
//            for (int kp = k + 1; kp <= K; kp++) {
//                for (auto h_i : start_with_i[i]) {
//                    sumZHKK += Z[h_i][k][kp];
//                }
//            }
//            model.add(sumZHKK <= X[k][i]);
//        }
//    }
//    for (int kp = 2; kp <= K; kp++) {
//        for (int j : c_t) {
//            IloExpr sumZHKK(env);
//            for (int k = 1; k < kp; k++) {
//                for (auto h_i : end_with_j[j]) {
//                    sumZHKK += Z[h_i][k][kp];
//                }
//            }
//            model.add(sumZHKK <= X[kp][j]);
//        }
//    }
//    // Crossing: if h is served in trip H_i, won't be no trip H_j
//    for (int h : instance->c_prime) {
//        IloExpr sum_ZH(env);
//        IloExpr sum_WH(env);
//        IloExpr sum_YH(env);
//
//        for (auto h_index : serve_h[h]) {
//            for (int k = 1; k < K; k++) {
//                sum_YH += Y[k][h_index];
//            }
//            for (int kp = 2; kp <= K; kp++) {
//                sum_WH += W[kp][h_index];
//            }
//        }
//        for (int k = 1; k < K; k++) {
//            for (int kp = k + 1; kp <= K; kp++) {
//                for (auto h_index : serve_h[h]) {
//                    sum_ZH += Z[h_index][k][kp];
//                }
//            }
//        }
//        model.add(phi[h] == sum_ZH);
//        model.add(sum_ZH <= 1);
//        model.add(sum_YH <= 1);
//        model.add(sum_WH <= 1);
//        model.add(sum_YH == sum_WH);
//    }
//
//    // time synchronization constraint
//    // truck move from stage k to stage k+1
//    for (int k = 1; k < K; k++) {
//        IloExpr sum_length(env);
//        for (int i : c_s) {
//            for (int j : c_t) {
//                if (i != j) {
//                    sum_length += x[k][i][j] * tau[i][j];
//                }
//            }
//        }
//        model.add(a[k + 1] == d[k] + sum_length);
//    }
//
//    // sortie length must not exceed: drone tour and truck tour.
//
//    // drone trip
//    const double M = 1e5;
//    for (int k = 1; k < K; k++) {
//        for (int kp = k + 1; kp <= K; kp++) {
//            IloExpr sl_start_kp(env);
//            IloExpr sr_ret_kp(env);
//            for (int h_index = 0; h_index < S; h_index++) {
//                if (kp != K) {
//                    sl_start_kp += Y[kp][h_index];
//                }
//                sr_ret_kp += W[kp][h_index];
//            }
//            for (int h_index = 0; h_index < S; h_index++) {
//                // model.add(d[kp] >= d[k] + all_sortie[h_index].trip_l + cfg.sr - (1 - Z[h_index][k][kp]) * M);
//            }
//        }
//    }
//
//    // truck trip
//    for (int k = 2; k <= K; k++) {
//        IloExpr sum(env);
//        IloExpr sum_w_K(env);
//        IloExpr sum_y_K(env);
//        for (int i = 0; i < D; i++) {
//            for (int j = 1; j <= D; j++) {
//                if (i != j) {
//                    sum += x[k - 1][i][j] * tau[i][j];
//                }
//            }
//        }
//        for (int h_index = 0; h_index < S; h_index++) {
//            if (k != K) {
//                sum_y_K += Y[k][h_index];
//            }
//            sum_w_K += W[k][h_index];
//        }
//
//        auto constr_name = "CC_d_" + std::to_string(k) + "_constr";
//        // model.add(d[k] >= d[k - 1] + sum + cfg.sr * sum_w_K + cfg.sl * sum_y_K).setName(constr_name.c_str());
//    }
//
//    model.add(IloMinimize(env, d[K]));
//    double revisit_count = 0;
//    std::vector<int> rev(n + 1, 0);
//    auto startTime = std::chrono::high_resolution_clock::now();
//    cplex.exportModel("rv-fstsp-mvd.lp");
//    bool relaxed = false;
//    while (!relaxed) {
//        if (!cplex.solve()) {
//            std::cout << "Unable to optimize model!" << std::endl;
//        }
//        // kiểm tra solution. Nếu sai thì giải thêm constraint và giải lại.
//    }
//    cplex.solve();
//    auto endTime = std::chrono::high_resolution_clock::now();
//    // Calculate the elapsed time
//    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
//
//    std::cout << "Feasible solution found!" << std::endl;
//    std::cout << "Truck nodes:" << std::endl;
//    for (int k = 1; k <= K; k++) {
//        for (int i = 0; i <= D; i++) {
//            auto X_val = cplex.getValue(X[k][i]);
//            if (X_val == 1) {
//                rev[i]++;
//                auto d_k = cplex.getValue(d[k]);
//                auto a_k = cplex.getValue(a[k]);
//                std::cout << "(Arrival = " << a_k << ") - Node " << i << "(Stage " << k << ") - (Departure = " << d_k <<
//                    ")" << std::endl;
//                break;
//            }
//        }
//    }
//
//    for (int k = 1; k <= K; k++) {
//        for (int i = 0; i <= D; i++) {
//            auto X_val = cplex.getValue(X[k][i]);
//            if (X_val == 1) {
//                std::cout << "Stage " << k << " at node " << i << std::endl;
//            }
//        }
//    }
//
//
//    std::cout << std::endl << "Truck arcs:" << std::endl;
//    std::map<int, std::pair<int, int>> map_stage_truck_arc;
//    for (int k = 1; k <= K_arc; k++) {
//        for (int i = 0; i < D; i++) {
//            for (int j = 1; j <= D; j++) {
//                if (i != j) {
//                    auto X_val = cplex.getValue(x[k][i][j]);
//                    if (X_val == 1) {
//                        std::cout << "Arc " << k << " connecting " << i << " and " << j
//                            << " with cost " << tau[i][j] << " " << std::endl;
//
//                        map_stage_truck_arc[k] = std::make_pair(i, j);
//                        break;
//                    }
//                }
//            }
//        }
//    }
//    int phi_check = 0;
//
//    std::cout << "Sorties information: ";
//    for (int k = 1; k < K; k++) {
//        for (int kp = k + 1; kp <= K; kp++) {
//            for (int h_index = 0; h_index < S; h_index++) {
//                if (cplex.getValue(Z[h_index][k][kp]) == 1) {
//                    phi_check++;
//                    int start_node = all_sortie[h_index].start_node, end_node = all_sortie[h_index].end_node;
//
//                    // Print.
//                    std::cout <<
//                        "-------------------------------------------------------------------------------------"
//                        << std::endl;
//                    std::cout << "Start node/Stage -------" << "Customer" << "-------" << "End node/Stage" <<
//                        std::endl;
//                    std::cout << "Visit sequence:";
//                    setPrint(all_sortie[h_index].visit_seq);
//                    std::cout << "Sortie starts from stage " << k << " at node " << start_node << std::endl;
//                    std::cout << "Served customer: ";
//                    setPrint(all_sortie[h_index].serve_customer);
//                    std::cout << "Sortie ends at stage " << kp << " at node " << end_node << std::endl;
//
//                    ////////////////////////////////////////////////////////////////////////////
//                    double truck_travel_time = 0;
//                    double drone_travel_time = all_sortie[h_index].trip_l;
//
//                    std::cout << std::endl;
//                    for (int k_start = k; k_start <= kp - 1; k_start++) {
//                        truck_travel_time += tau[map_stage_truck_arc[k_start].first][map_stage_truck_arc[k_start].
//                            second];
//                    }
//                    auto drone_arrival_time = cplex.getValue(d[k]) + drone_travel_time;
//                    auto vehicle_departure_time = cplex.getValue(d[kp]);
//                    auto truck_arrival_time = cplex.getValue(a[kp]);
//                    std::cout << "Time information of this sortie:" << std::endl;
//                    std::cout << "Truck departure time from stage k = " << k << " is: " << cplex.getValue(d[k])
//                        << std::endl;
//                    std::cout << "Truck arcs during this sortie: ";
//                    for (int k_start = k; k_start <= kp; k_start++) {
//                        if (k_start == kp) {
//                            std::cout << map_stage_truck_arc[k_start].first;
//                        }
//                        else {
//                            std::cout << map_stage_truck_arc[k_start].first << "--->";
//                        }
//                    }
//                    std::cout << ". Total truck time = " << truck_travel_time << std::endl;
//                    std::cout << "Total drone time: " << drone_travel_time << std::endl;
//                    std::cout << "Truck arrival time at k' = " << kp << " is: " << truck_arrival_time << std::endl;
//                    std::cout << "Drone arrival time at k' = " << kp << " is: " << drone_arrival_time << std::endl;
//                    std::cout << "Truck departure time at k' = " << cplex.getValue(d[kp]) << std::endl;
//                    std::cout <<
//                        "-------------------------------------------------------------------------------------"
//                        << std::endl;
//                }
//            }
//        }
//    }
//
//
//    std::cout << "Done!" << std::endl;
//    // std::cout << "-------------------------Re-calculated objective-----------------------" << std::endl;
//    // std::cout << obj << std::endl;
//    // std::cout << "------------------------------------------------------------------------" << std::endl;
//
//    double c = cplex.getObjValue();
//    std::cout << "Solve time: " << duration.count() / 1000.0 << std::endl;
//    cplex.end();
//    model.end();
//    env.end();
//    for (int i = 0; i < rev.size(); i++) {
//        if (rev[i] > 1) {
//            revisit_count += rev[i] - 1;
//            std::cout << "Node " << i << " was revisited " << rev[i] - 1 << " times!" << std::endl;
//        }
//    }
//
//
//    std::cout << "OBJECTIVE VALUE: " << c << ", NUMBER OF SORTIES: " << phi_check << "." << std::endl;
//    std::cout << "Number of revisit: " << revisit_count << std::endl;
//    return Result{c, duration.count() / 1000.0, cplex.getMIPRelativeGap()};
//}
//
//[[maybe_unused]] Result
//FSTSPSolver::stage_based_fstsp(Config& cfg) const {
//    try {
//        int bigM = 500;
//        double dp[23][1 << 23] = {0};
//        int sz[1 << 23] = {0};
//        double minPath[23];
//        auto tau = instance->tau;
//        auto tau_prime = instance->tau_prime;
//        auto n = instance->num_node;
//        std::vector<int> C;
//        std::vector<int> V;
//        std::vector<int> c_s;
//        std::vector<int> c_t;
//        for (int i = 0; i < n + 1; i++) {
//            if (i != 0 && i != n) {
//                C.push_back(i);
//            }
//            if (i != 0) {
//                c_t.push_back(i);
//            }
//            if (i != n) {
//                c_s.push_back(i);
//            }
//            V.push_back(i);
//        }
//
//        IloEnv env;
//        IloModel model(env);
//        IloCplex cplex(model);
//        //cplex.setParam(IloCplex::Param::MIP::Tolerances::Integrality, 0);
//
//        auto O = 0;
//        auto D = n;
//        auto K = n + 1;
//
//        IloArray<IloBoolVarArray> X(env, K + 1);
//        for (int k = 1; k <= K; k++) {
//            X[k] = IloBoolVarArray(env, D + 1);
//            for (int i = 0; i <= D; i++) {
//                X[k][i] = IloBoolVar(env);
//                auto v = "X_" + std::to_string(k) + "_" + std::to_string(i);
//                X[k][i].setName(v.c_str());
//                if (k == K && i != D)
//                    X[k][i].setBounds(0, 0);
//            }
//            if (k > 1) {
//                std::string cname = "X_k_" + std::to_string(k) + "_O == 0";
//                model.add(X[k][O] == 0).setName(cname.c_str());
//            }
//        }
//
//        for (int k = 1; k < K; k++) {
//            IloExpr expr(env);
//            for (int i = 0; i <= D; i++)
//                expr += X[k][i] - X[k + 1][i];
//            model.add(expr >= 0).setName(("k_before_k+1_" + std::to_string(k)).c_str());
//        }
//
//        IloArray<IloBoolVarArray> x(env, D);
//        for (int i : c_s) {
//            x[i] = IloBoolVarArray(env, D + 1);
//            for (int j : c_t) {
//                if (i != j) {
//                    auto v_name = "x_" + std::to_string(i) + "_" + std::to_string(j);
//                    x[i][j] = IloBoolVar(env);
//                    x[i][j].setName(v_name.c_str());
//                }
//            }
//        }
//
//        IloArray<IloBoolVarArray> Y(env, K);
//        for (int k = 1; k < K; k++) {
//            Y[k] = IloBoolVarArray(env, D);
//            for (int h : C) {
//                Y[k][h] = IloBoolVar(env);
//                auto v_name = "Y_" + std::to_string(k) + "_" + std::to_string(h);
//                Y[k][h].setName(v_name.c_str());
//                if (exist(instance->heavy, h)) {
//                    std::string cname = "Y_k_" + std::to_string(k) + "_h_" + std::to_string(h) + "==0";
//                    model.add(Y[k][h] == 0).setName(cname.c_str());
//                }
//            }
//        }
//
//        IloArray<IloBoolVarArray> W(env, K + 1);
//        for (int kp = 2; kp <= K; kp++) {
//            W[kp] = IloBoolVarArray(env, D);
//            for (int h : C) {
//                W[kp][h] = IloBoolVar(env);
//                auto v_name = "W_" + std::to_string(kp) + "_" + std::to_string(h);
//                W[kp][h].setName(v_name.c_str());
//                if (exist(instance->heavy, h)) {
//                    std::string cname = "W_kp_" + std::to_string(kp) + "_h_" + std::to_string(h) + "==0";
//                    model.add(W[kp][h] == 0).setName(cname.c_str());
//                }
//            }
//        }
//        IloArray<IloBoolVarArray> A(env, D);
//        for (int i : c_s) {
//            A[i] = IloBoolVarArray(env, D);
//            for (int h : C) {
//                if (i != h) {
//                    A[i][h] = IloBoolVar(env);
//                    auto v_name = "A_" + std::to_string(i) + "_" + std::to_string(h);
//                    A[i][h].setName(v_name.c_str());
//                    if (exist(instance->heavy, h)) {
//                        std::string cname = "A_i_" + std::to_string(i) + "_h_" + std::to_string(h) + "==0_heavy";
//                        model.add(A[i][h] == 0).setName(cname.c_str());
//                    }
//                    if (tau_prime[i][h] > cfg.dtl - cfg.sr) {
//                        std::string cname = "A_i_" + std::to_string(i) + "_h_" + std::to_string(h) +
//                            "==0_>cfg.dtl-cfg.sr";
//                        model.add(A[i][h] == 0).setName(cname.c_str());
//                    }
//                }
//            }
//        }
//        IloArray<IloBoolVarArray> B(env, D + 1);
//        for (int j : c_t) {
//            B[j] = IloBoolVarArray(env, D);
//            for (int h : C) {
//                if (j != h) {
//                    B[j][h] = IloBoolVar(env);
//                    auto v_name = "B_" + std::to_string(j) + "_" + std::to_string(h);
//                    B[j][h].setName(v_name.c_str());
//                    if (exist(instance->heavy, h)) {
//                        std::string cname = "B_j=" + std::to_string(j) + "_h_" + std::to_string(h) + "==0_heavy";
//                        model.add(B[j][h] == 0);
//                    }
//                    if (tau_prime[h][j] > cfg.dtl - cfg.sr) {
//                        std::string cname = "B_j=" + std::to_string(j) + "_h_" + std::to_string(h) +
//                            "==0_>cfg.dtl-cfg.sr";
//                        model.add(B[j][h] == 0).setName(cname.c_str());
//                    }
//                }
//            }
//        }
//
//        IloBoolVarArray phi(env, D);
//        for (int h : C) {
//            phi[h] = IloBoolVar(env);
//            auto v_name = "phi_" + std::to_string(h);
//            phi[h].setName(v_name.c_str());
//            if (exist(instance->heavy, h)) {
//                std::string cname = "h_" + std::to_string(h) + "_to_heavy";
//                model.add(phi[h] == 0).setName(cname.c_str());
//            }
//        }
//        IloArray<IloBoolVarArray> z(env, K);
//        for (int k = 1; k < K; k++) {
//            z[k] = IloBoolVarArray(env, K + 1);
//            for (int kp = k + 1; kp <= K; kp++) {
//                auto v_name = "z_" + std::to_string(k) + std::to_string(kp);
//                z[k][kp].setName(v_name.c_str());
//                z[k][kp] = IloBoolVar(env);
//            }
//        }
//
//        IloNumVarArray b(env, D + 1);
//        IloNumVarArray e(env, D + 1);
//        for (int i = 0; i <= D; i++) {
//            auto v_name = "b_" + std::to_string(i);
//            b[i] = IloNumVar(env, 0, IloInfinity, ILOFLOAT);
//            b[i].setName(v_name.c_str());
//            e[i] = IloNumVar(env, 0, IloInfinity, ILOFLOAT);
//            v_name = "e_" + std::to_string(i);
//            e[i].setName(v_name.c_str());
//        }
//
//        // Constraint 2
//        model.add(X[1][O] == 1).setName("start_at_O");
//
//        // Constraint 3
//        IloExpr c3(env);
//        for (int k = 2; k <= K; k++) {
//            c3 += X[k][D];
//        }
//        model.add(c3 == 1).setName("back_to_D");
//
//        // Constraint 4
//        for (int h : instance->c_prime) {
//            IloExpr sum_A(env), sum_B(env);
//            for (int i : c_s) {
//                if (i != h) {
//                    sum_A += A[i][h];
//                }
//            }
//            for (int j : c_t) {
//                if (j != h) {
//                    sum_B += B[j][h];
//                }
//            }
//            std::string c_name1 = "C4_lhs_h_" + std::to_string(h);
//            model.add(sum_A == phi[h]).setName(c_name1.c_str());
//            c_name1 = "C4_rhs_h_" + std::to_string(h);
//            model.add(sum_B == phi[h]).setName(c_name1.c_str());
//        }
//
//        // Constraint 5
//        for (int h : instance->c_prime) {
//            for (int i : C) {
//                if (i != h) {
//                    std::string cname = "C5_h_" + std::to_string(h) + "_i_" + std::to_string(i);
//                    model.add(A[i][h] + B[i][h] <= phi[h]).setName(cname.c_str());
//                }
//            }
//        }
//
//        // Constraint 6
//        for (int h : instance->c_prime) {
//            IloExpr sum_Y(env), sum_W(env);
//            for (int k = 1; k < K; k++) {
//                sum_Y += Y[k][h];
//            }
//            for (int kp = 2; kp <= K; kp++) {
//                sum_W += W[kp][h];
//            }
//            std::string cname = "C6_lhs_h_" + std::to_string(h);
//            model.add(sum_Y == phi[h]).setName(cname.c_str());
//            cname = "C6_rhs_h_" + std::to_string(h);
//            model.add(sum_W == phi[h]).setName(cname.c_str());
//        }
//
//        // Constraint 7
//        for (int h : instance->c_prime) {
//            for (int k = 2; k < K; k++) {
//                std::string cname = "C7_h_" + std::to_string(h) + "_k_" + std::to_string(k);
//                model.add(Y[k][h] + W[k][h] <= phi[h]).setName(cname.c_str());
//            }
//        }
//
//        // Constraint 8
//        for (int i : c_s) {
//            IloExpr lhs(env), rhs(env);
//            for (int k = 1; k < K; k++) {
//                lhs += X[k][i];
//            }
//            for (int h : instance->c_prime) {
//                if (i != h) {
//                    rhs += A[i][h];
//                }
//            }
//            std::string cname = "C8_i_" + std::to_string(i);
//            model.add(lhs >= rhs).setName(cname.c_str());
//        }
//
//        // Constraint 9
//        for (int j : c_t) {
//            IloExpr lhs(env), rhs(env);
//            for (int kp = 2; kp <= K; kp++) {
//                lhs += X[kp][j];
//            }
//            for (int h : instance->c_prime) {
//                if (j != h) {
//                    rhs += B[j][h];
//                }
//            }
//            std::string cname = "C9_j=" + std::to_string(j);
//            model.add(lhs >= rhs).setName(cname.c_str());
//        }
//
//        // Constraint 10
//        for (int k = 1; k < K; k++) {
//            IloExpr lhs(env), rhs(env);
//            for (int i : c_s) {
//                lhs += X[k][i];
//            }
//            for (int h : instance->c_prime) {
//                rhs += Y[k][h];
//            }
//            std::string cname = "C10_k_" + std::to_string(k);
//            model.add(lhs >= rhs).setName(cname.c_str());
//        }
//
//        // Constraint 11
//        for (int kp = 2; kp <= K; kp++) {
//            IloExpr lhs(env), rhs(env);
//            for (int j : c_t) {
//                lhs += X[kp][j];
//            }
//            for (int h : instance->c_prime) {
//                rhs += W[kp][h];
//            }
//            std::string cname = "C11_kp_" + std::to_string(kp);
//            model.add(lhs >= rhs).setName(cname.c_str());
//        }
//
//        // Constraint 12
//        for (int h : instance->c_prime) {
//            IloExpr s1(env), s2(env);
//            for (int i : c_s) {
//                if (i != h) {
//                    s1 += A[i][h] * tau_prime[i][h];
//                }
//            }
//            for (int j : c_t) {
//                if (j != h) {
//                    s2 += B[j][h] * tau_prime[h][j];
//                }
//            }
//            std::string cname = "C12_h_" + std::to_string(h);
//            model.add(s1 + s2 <= (cfg.dtl - cfg.sr) * phi[h]).setName(cname.c_str());
//        }
//
//        // Constraint 13
//        for (int k = 1; k < K; k++) {
//            IloExpr lhs(env), rhs(env);
//            for (int kp = k + 1; kp <= K; kp++) {
//                lhs += z[k][kp];
//            }
//            for (int h : instance->c_prime) {
//                rhs += Y[k][h];
//            }
//            std::string cname = "C13_k_" + std::to_string(k) + "_lhs=rhs";
//            model.add(lhs == rhs).setName(cname.c_str());
//            cname = "C13_k_" + std::to_string(k) + "<=1";
//            model.add(rhs <= 1);
//        }
//
//        // Constraint 14
//        for (int kp = 2; kp <= K; kp++) {
//            IloExpr lhs(env), rhs(env);
//            for (int k = 1; k < kp; k++) {
//                lhs += z[k][kp];
//            }
//            for (int h : instance->c_prime) {
//                rhs += W[kp][h];
//            }
//            std::string cname = "C14_kp_" + std::to_string(kp) + "_lhs=rhs";
//            model.add(lhs == rhs).setName(cname.c_str());
//            cname = "C14_kp_" + std::to_string(kp) + "<=1";
//            model.add(rhs <= 1);
//        }
//
//        // Constraint 15
//        for (int l = 1; l < K; l++) {
//            IloExpr sum(env);
//            for (int k = 1; k <= l; k++) {
//                for (int kp = k + 1; kp <= K; kp++) {
//                    if (k <= l && l < kp) {
//                        sum += z[k][kp];
//                    }
//                }
//            }
//            std::string cname = "C15_l=" + std::to_string((l));
//            model.add(sum <= 1).setName(cname.c_str());
//        }
//
//        // Constraint 16
//        for (int l = 2; l <= K; l++) {
//            IloExpr sum(env);
//            for (int k = 1; k < l; k++) {
//                for (int kp = l; kp <= K; kp++) {
//                    if (k < l && l <= kp) {
//                        sum += z[k][kp];
//                    }
//                }
//            }
//            std::string cname = "C16_l=" + std::to_string((l));
//            model.add(sum <= 1).setName(cname.c_str());
//        }
//        // Constraint 17
//        for (int h : C) {
//            IloExpr sum_X(env);
//            for (int k = 2; k < K; k++) {
//                sum_X += X[k][h];
//            }
//            std::string cname = "C17_h_" + std::to_string((h));
//            model.add(sum_X + phi[h] == 1).setName(cname.c_str());
//        }
//
//        // Constraint 18
//        // Tighter
//        for (int k = 1; k <= K; k++) {
//            IloExpr sum_X(env);
//            for (int i = 0; i <= D; i++) {
//                sum_X += X[k][i];
//            }
//            std::string cname = "C18_k_" + std::to_string(k);
//            model.add(sum_X <= 1).setName(cname.c_str());
//        }
//
//        // Constraint 19/20 rewrite
//        // Leaving i
//        for (int i : C) {
//            IloExpr sum_leave(env), sum_enter(env);
//            IloExpr lhs(env);
//            for (int j : c_t) {
//                if (i != j) {
//                    sum_leave += x[i][j];
//                }
//            }
//            for (int j : c_s) {
//                if (i != j) {
//                    sum_enter += x[j][i];
//                }
//            }
//            for (int k = 2; k < K; k++) {
//                lhs += X[k][i];
//            }
//            std::string eq_deg = "C19_20_i_" + std::to_string(i);
//            model.add(sum_leave == sum_enter).setName(eq_deg.c_str());
//            eq_deg = "C19_20_i_" + std::to_string(i) + "_deg<=1";
//            model.add(sum_leave <= 1).setName(eq_deg.c_str());
//            eq_deg = "C19_20_i_" + std::to_string(i) + "_stage_presence";
//            model.add(sum_leave == lhs).setName(eq_deg.c_str());
//        }
//
//        // Constraint 21
//        for (int k = 1; k < K; k++) {
//            for (int i : c_s) {
//                for (int j : c_t) {
//                    if (i != j) {
//                        std::string cname =
//                            "C21_k_" + std::to_string(k) + "_i_" + std::to_string(i) + "_j=" + std::to_string(j);
//                        model.add(X[k + 1][j] + 1 - x[i][j] >= X[k][i]).setName(cname.c_str());
//                    }
//                }
//            }
//        }
//
//        // Constraint 23
//        model.add(b[O] == 0);
//        model.add(e[O] == 0);
//
//        // Constraint 24
//        model.add(IloMinimize(env, e[D])).setName("Objective");
//
//        // Constraint 25
//        for (int i = 1; i <= D; i++) {
//            model.add(b[i] <= e[i]);
//        }
//        auto M = 1e5;
//        // Constraint 26
//        for (int i : c_s) {
//            for (int j : c_t) {
//                if (i != j) {
//                    std::string cname = "C26_i_" + std::to_string(i) + "_j=" + std::to_string(j);
//                    model.add(b[j] + M * (1 - x[i][j]) >= e[i] + tau[i][j] * x[i][j]).setName(cname.c_str());
//                }
//            }
//        }
//
//        // Constraint 27
//        for (int i : c_s) {
//            for (int h : instance->c_prime) {
//                if (i != h && tau_prime[i][h] <= cfg.dtl - cfg.sr) {
//                    std::string cname = "C27_i_" + std::to_string(i) + "_h_" + std::to_string(h);
//                    model.add(b[h] + M * (1 - A[i][h]) >= e[i] + tau_prime[i][h] * A[i][h]).setName(cname.c_str());
//                }
//            }
//        }
//
//        // Constraint 28
//        for (int j : c_t) {
//            for (int h : instance->c_prime) {
//                if (j != h && tau_prime[h][j] <= cfg.dtl - cfg.sr) {
//                    std::string cname = "C28_j=" + std::to_string(j) + "_h_" + std::to_string(h);
//                    model.add(e[j] + M * (1 - B[j][h]) >= e[h] + tau_prime[h][j] * B[j][h] + cfg.sr).setName(
//                        cname.c_str());
//                }
//            }
//        }
//
//        // Constraint 29
//        for (int i : c_s) {
//            for (int j : c_t) {
//                if (i != j) {
//                    IloExpr sum1(env), sum2(env);
//                    for (int h : instance->c_prime) {
//                        if (tau_prime[i][h] <= cfg.dtl - cfg.sr && tau_prime[h][j] <= cfg.dtl - cfg.sr) {
//                            sum1 += 0.5 * (A[i][h] + B[j][h]) * (cfg.dtl - cfg.sr);
//                            sum2 += A[i][h] + B[j][h];
//                        }
//                    }
//                    std::string cname = "C29_i_" + std::to_string(i) + "_j=" + std::to_string(j);
//                    model.add(b[j] <= e[i] + sum1 + M * (2 - sum2)).setName(cname.c_str());
//                }
//            }
//        }
//
//        IloExpr lb_truck(env), lb_drone(env);
//        for (int i : c_s) {
//            for (int j : c_t) {
//                if (i != j) {
//                    lb_truck += x[i][j] * tau[i][j];
//                }
//            }
//        }
//
//
//        for (int i = 0; i <= D; i++) {
//            for (int h : instance->c_prime) {
//                if (i != h) {
//                    if (i < D) {
//                        lb_drone += A[i][h] * tau_prime[i][h];
//                    }
//                    if (i > 0) {
//                        lb_drone += B[i][h] * tau_prime[h][i];
//                    }
//                }
//            }
//        }
//
//        for (int i = 0; i <= D; i++)
//            for (int h : instance->c_prime) {
//                if (i < D) {
//                    lb_truck += A[i][h] * cfg.sl;
//                    lb_drone += A[i][h] * cfg.sl;
//                }
//                if (i > 0) {
//                    lb_truck += B[i][h] * cfg.sr;
//                    lb_drone += B[i][h] * cfg.sr;
//                }
//            }
//
//        model.add(e[D] >= lb_truck).setName("e[D]>=truck_tour");
//        model.add(e[D] >= lb_drone).setName("e[D]>=drone_tour");
//
//        //drone di
//        //
//        for (int k = 1; k < K; k++)
//            for (int h : instance->c_prime) {
//                for (int i = 0; i < D; i++)
//                    if (i != h)
//                        model.add(Y[k][h] + 1 >= X[k][i] + A[i][h]);
//            }
//
//        for (int i = 0; i < D; i++)
//            for (int h : instance->c_prime)
//                if (i != h) {
//                    for (int k = 1; k < K; k++) {
//                        model.add(A[i][h] + 1 >= X[k][i] + Y[k][h]);
//                    }
//                }
//        for (int k = 1; k < K; k++)
//            for (int i = 0; i < D; i++)
//                for (int h : instance->c_prime)
//                    if (i != h) {
//                        {
//                            model.add(X[k][i] + 1 >= A[i][h] + Y[k][h]);
//                        }
//                    }
//
//
//        //drone den
//        for (int k = 2; k <= K; k++)
//            for (int h : instance->c_prime) {
//                for (int i = 1; i <= D; i++)
//                    if (i != h)
//                        model.add(W[k][h] + 1 >= X[k][i] + B[i][h]);
//            }
//
//
//        for (int i = 1; i <= D; i++)
//            for (int h : instance->c_prime) {
//                for (int k = 2; k <= K; k++)
//                    if (i != h)
//                        model.add(B[i][h] + 1 >= X[k][i] + W[k][h]);
//            }
//
//        for (int k = 2; k <= K; k++)
//            for (int i = 1; i <= D; i++)
//                for (int h : instance->c_prime) {
//                    if (i != h)
//                        model.add(X[k][i] + 1 >= B[i][h] + W[k][h]);
//                }
//
//        for (int k = 1; k <= K / 2; k++) {
//            x[k][D].setBounds(0, 0);
//            IloExpr expr(env);
//            if (k >= 2) {
//                for (int i = 1; i < D; i++)
//                    expr += X[k][i];
//                model.add(expr == 1);
//            }
//        }
//
//        IloExpr level_stage(env);
//        for (int k = 1; k <= K; k++)
//            level_stage += k * X[k][D];
//        for (int h : instance->c_prime)
//            level_stage += phi[h];
//
//        model.add(level_stage == K);
//
//
//        for (int k = 1; k < K; k++) {
//            IloExpr expr(env);
//
//            for (int i = 0; i <= D; i++) {
//                expr += X[k][i] - X[k + 1][i];
//            }
//
//            model.add(expr >= 0).setName(("anti_symmetric" + std::to_string(k)).c_str());
//        }
//
//        for (int L = 2, node_max_stage = K; L < node_max_stage; L++) {
//            for (int i = 1; i + L <= node_max_stage; i++) {
//                int j = i + L;
//                for (int l = 2; l < L; l++) {
//                    IloExpr expr(env);
//                    for (int k = i; k + l <= j; k++)
//                        for (int k_p = k + l; k_p <= j; k_p++)
//                            expr += z[k][k_p];
//                    model.add(expr <= (int)(L / l)).setName(("zlen_" + std::to_string(i)
//                        + "_" + std::to_string(j) + "_" +
//                        std::to_string(l)).c_str());
//                }
//            }
//        }
//
//
//        for (int s = 1, node_max_stage = K; s < (1 << node_max_stage); s++) {
//            for (int i = 0; i < node_max_stage; i++)
//                if ((1 << i) & s)
//                    for (int j = 0; j < node_max_stage; j++)
//                        if (((1 << j) & s) && (i != j))
//                            if (dp[j][s] == 0 || dp[j][s] > dp[i][s ^ (1 << j)] + tau[i][j]) {
//                                dp[j][s] = dp[i][s ^ (1 << j)] + tau[i][j];
//                            }
//        }
//
//        sz[0] = 0;
//
//        for (int i = 0, node_max_stage = K; i <= node_max_stage; i++) minPath[i] = 1e9;
//
//
//        for (int s = 1, node_max_stage = K; s < (1 << node_max_stage); s++) {
//            for (int i = 0; i < node_max_stage; i++)
//                if ((1 << i) & s) {
//                    sz[s] = sz[s ^ (1 << i)] + 1;
//                    minPath[sz[s] - 1] = std::min(minPath[sz[s] - 1], dp[i][s]);
//                }
//        }
//
//        for (int l = 1, node_max_stage = K; l < node_max_stage; l++)
//            if (minPath[l] > cfg.dtl) {
//                // any path of having l arcs has length more than cfg.dtl
//                for (int k = 1; k < node_max_stage; k++)
//                    for (int k_p = k + l; k_p <= node_max_stage; k_p++) {
//                        //std::cout << k << " " << k_p << std::endl;
//                        z[k][k_p].setBounds(0, 0);
//                    }
//                break;
//            }
//
//        for (int k = 1; k < K; k++)
//            for (int h : instance->c_prime) {
//                IloExpr expr(env);
//                for (int l = 1; l <= k; l++)
//                    expr += Y[l][h];
//                for (int l = 1; l <= k; l++)
//                    expr -= W[l + 1][h];
//                model.add(expr >= 0);
//            }
//
//        cplex.exportModel("model.lp");
//        cplex.setParam(IloCplex::Param::TimeLimit, 300);
//        cplex.solve();
//        std::cout << "Total travel time: " << cplex.getObjValue() << std::endl;
//        std::cout << "Truck stages:" << std::endl;
//        std::map<int, int> stage_location;
//
//        for (int k = 1; k <= K; k++) {
//            for (int i = 0; i <= D; i++) {
//                if (abs(cplex.getValue(X[k][i]) - 1) < 1e-5) {
//                    std::cout << "Stage " << k << " at " << i << std::endl;
//                    stage_location[k] = i;
//                }
//            }
//        }
//        std::cout << "Truck arcs:" << std::endl;
//        std::map<int, int> truck_arcs;
//        for (int i : c_s) {
//            for (int j : c_t) {
//                if (i != j) {
//                    if (abs(cplex.getValue(x[i][j]) - 1) < 1e-5) {
//                        std::cout << "Truck moves from " << i << " to " << j << ", cost = " << tau[i][j] << std::endl;
//                        truck_arcs[i] = j;
//                    }
//                }
//            }
//        }
//
//        std::map<int, int> drone_level;
//        for (int k = 1; k < K; k++)
//            for (int kp = k + 1; kp <= K; kp++) {
//                if (abs(cplex.getValue(z[k][kp]) - 1) < 1e-5) {
//                    std::cout << "Drone fly from level " << k << " to " << kp << std::endl;
//                    drone_level[k] = kp;
//                }
//            }
//
//        std::cout << "Drone served customers:" << std::endl;
//        for (int h : instance->c_prime) {
//            if (abs(cplex.getValue(phi[h]) - 1) < 1e-5) {
//                std::cout << "Customer " << h << " served by drone." << std::endl;
//                for (int i : c_s) {
//                    if (abs(cplex.getValue(A[i][h]) - 1) < 1e-5) {
//                        for (int k = 1; k < K; k++) {
//                            if (abs(cplex.getValue(Y[k][h]) - 1) < 1e-5) {
//                                //Y(3,0) = A(0,3) = 1; thưc tế X
//                                std::cout << "Launch at stage " << k << " at node " << i << std::endl;
//                                if (stage_location[k] != i) {
//                                    std::cout << "Drone departs from " << stage_location[k] << std::endl;
//                                }
//                            }
//                        }
//                    }
//                }
//                for (int j : c_t) {
//                    if (abs(cplex.getValue(B[j][h]) - 1) < 1e-5) {
//                        for (int k = 2; k <= K; k++) {
//                            if (abs(cplex.getValue(W[k][h]) - 1) < 1e-5) {
//                                //W(4,3) = B(1,3) = 1
//                                std::cout << "Rendezvous at stage " << k << " at node " << j << std::endl;
//                                if (stage_location[k] != j) {
//                                    std::cout << "Drone arrives at " << stage_location[k] << std::endl;
//                                }
//                            }
//                        }
//                    }
//                }
//            }
//        }
//        std::cout << "Timing: " << std::endl;
//        for (int i = 0; i <= D; i++) {
//            std::cout << "Arrival at " << i << " is: " << cplex.getValue(b[i]) << std::endl;
//            std::cout << "Departure at " << i << " is: " << cplex.getValue(e[i]) << std::endl;
//        }
//        return Result{cplex.getObjValue(), 1000.0, cplex.getMIPRelativeGap()};
//    }
//    catch (IloException& e) {
//        std::cout << "Exception: " << e.getMessage() << std::endl;
//    }
//}
//
//Result FSTSPSolver::OriginalSolverCPLEX(Config& cfg) {
//    auto tau = instance->tau;
//
//    auto d = instance->tau_prime;
//    auto n = instance->num_node;
//    auto s = 0, t = n;
//    auto c_prime = instance->c_prime;
//    std::vector<int> c_prime_0;
//    c_prime_0.push_back(0);
//    for (int i : c_prime) {
//        c_prime_0.push_back(i);
//    }
//    c_prime_0.push_back(n);
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
//    std::cout << std::endl;
//    IloEnv env;
//    IloModel model(env);
//    IloCplex cplex(model);
//
//    // y: (i, j) in A, truck route
//    IloArray<IloBoolVarArray> y(env, n + 1);
//    for (int i : c_s) {
//        y[i] = IloBoolVarArray(env, n + 1);
//        for (int j : c_t) {
//            y[i][j] = IloBoolVar(env);
//            if (i == j) {
//                model.add(y[i][j] == 0);
//            }
//        }
//    }
//    model.add(y[s][t] == 0);
//
//
//    IloArray<IloBoolVarArray> x(env, n + 1);
//    for (int i : c_s) {
//        x[i] = IloBoolVarArray(env, n + 1);
//        for (int j : c_t) {
//            x[i][j] = IloBoolVar(env);
//            if (i == j) {
//                model.add(x[i][j] == 0);
//            }
//        }
//    }
//    model.add(x[s][t] == 0);
//
//
//    // gamma_h_ij
//    IloArray<IloArray<IloBoolVarArray>> gamma(env, n + 1);
//    for (int h : C) {
//        gamma[h] = IloArray<IloBoolVarArray>(env, n + 1);
//        for (int i : c_s) {
//            gamma[h][i] = IloBoolVarArray(env, n + 1);
//            for (int j : c_t) {
//                gamma[h][i][j] = IloBoolVar(env);
//                for (int heavy : instance->heavy) {
//                    if (h == heavy) {
//                        model.add(gamma[h][i][j] == 0);
//                    }
//                }
//                if (i == j) {
//                    model.add(gamma[h][i][j] == 0);
//                }
//            }
//        }
//        model.add(gamma[h][s][t] == 0);
//    }
//
//    IloBoolVarArray theta(env, n + 1);
//    for (int h : V) {
//        theta[h] = IloBoolVar(env);
//        if (!exist(instance->c_prime, h) || h == s || h == t) {
//            model.add(theta[h] == 0);
//        }
//    }
//
//    IloArray<IloBoolVarArray> omega(env, n + 1);
//    for (int h : C) {
//        omega[h] = IloBoolVarArray(env, n + 1);
//        for (int i : V) {
//            omega[h][i] = IloBoolVar(env);
//
//            for (int heavy : instance->heavy) {
//                if (h == heavy) {
//                    model.add(omega[h][i] == 0);
//                }
//            }
//            if (h == i || i == t) {
//                model.add(omega[h][i] == 0);
//            }
//        }
//    }
//    IloArray<IloBoolVarArray> delta(env, n + 1);
//    for (int h : C) {
//        delta[h] = IloBoolVarArray(env, n + 1);
//        for (int j : V) {
//            delta[h][j] = IloBoolVar(env);
//            for (int heavy : instance->heavy) {
//                if (h == heavy) {
//                    model.add(delta[h][j] == 0);
//                }
//            }
//            if (h == j || j == s) {
//                model.add(delta[h][j] == 0);
//            }
//        }
//    }
//
//    IloNumVarArray sigma(env, n + 1);
//    for (int h : c_t) {
//        sigma[h] = IloNumVar(env, 0, IloInfinity, ILOFLOAT);
//        for (int heavy : instance->heavy) {
//            if (h == heavy) {
//                model.add(sigma[h] == 0);
//            }
//        }
//    }
//
//    IloExpr objective(env);
//    for (int i : c_s) {
//        for (int j : c_t) {
//            objective += tau[i][j] * y[i][j];
//        }
//    }
//
//    for (int h : C) {
//        objective += (cfg.sl + cfg.sr) * theta[h];
//        objective -= cfg.sl * omega[h][s];
//    }
//    for (int h : c_t) {
//        objective += sigma[h];
//    }
//    IloExpr sum_theta(env);
//    // Constraint 1
//    IloExpr lhs_1(env), rhs_1(env);
//    for (int j : c_t) {
//        lhs_1 += y[s][j];
//    }
//    for (int i : c_s) {
//        rhs_1 += y[i][t];
//    }
//    model.add(lhs_1 == 1);
//    model.add(rhs_1 == 1);
//
//    // Constraint 2
//    for (int i : C) {
//        IloExpr lhs_2(env), rhs_2(env);
//        for (int j : c_t) {
//            lhs_2 += y[i][j];
//        }
//        for (int j : c_s) {
//            rhs_2 += y[j][i];
//        }
//        model.add(lhs_2 == rhs_2);
//        model.add(lhs_2 <= 1);
//        model.add(rhs_2 <= 1);
//    }
//
//    // Constraint 3
//    auto setAndComps = generateSetsAndComplements(C);
//    for (auto& set : setAndComps) {
//        auto S = set.first;
//        if (S.size() < 2) {
//            continue;
//        }
//        if (S.size() == 2 && S[0] == s && S[1] == t) {
//            continue;
//        }
//        IloExpr sum1(env), sum2(env);
//        std::string cname = "C3";
//        for (auto i : S) {
//            cname += "_" + std::to_string(i);
//            if (i != t) {
//                for (auto j : S) {
//                    if (j != s) {
//                        sum1 += y[i][j];
//                    }
//                }
//            }
//        }
//
//        for (auto h : S) {
//            IloExpr sum3(env);
//            for (auto k : S) {
//                if (h == k || h == s || h == t) {
//                    continue;
//                }
//                else {
//                    if (exist(C, k)) {
//                        sum3 += 1 - theta[k];
//                    }
//                }
//            }
//
//            model.add(sum1 <= sum3);
//        }
//    }
//    // Constraint 4
//    for (int h : C) {
//        IloExpr lhs_4(env);
//        std::string cname = "C4_h_" + std::to_string(h);
//        for (int j : c_t) {
//            lhs_4 += gamma[h][s][j];
//        }
//        model.add(lhs_4 == omega[h][s]);
//    }
//
//    // Constraint 5
//    for (int h : C) {
//        IloExpr lhs_5(env);
//        std::string cname = "C5_h_" + std::to_string(h);
//
//        for (int i : c_s) {
//            lhs_5 += gamma[h][i][t];
//        }
//        model.add(lhs_5 == delta[h][t]);
//    }
//    // Constraint 6
//    for (int i : C) {
//        for (int h : C) {
//            std::string cname = "C6_i_" + std::to_string(i) + "_h_" + std::to_string(h);
//            IloExpr sum1(env), sum2(env);
//            for (int j : c_t) {
//                sum1 += gamma[h][i][j];
//            }
//
//            for (int j : c_s) {
//                sum2 += gamma[h][j][i];
//            }
//            model.add(sum1 - sum2 == omega[h][i] - delta[h][i]);
//        }
//    }
//    // Constraint 7
//    for (int j : c_t) {
//        std::string cname = "C7_s_j=" + std::to_string(j);
//        model.add(y[s][j] + x[s][j] <= 1);
//    }
//    // Constraint 8
//    for (int i : c_s) {
//        std::string cname = "C8_i_" + std::to_string(i) + "_t";
//        model.add(y[i][t] + x[i][t] <= 1);
//    }
//    // Constraint 9
//    for (int i : C) {
//        for (int j : C) {
//            std::string cname = "C9_i_" + std::to_string(i) + "_j=" + std::to_string(j);
//            model.add(y[i][j] + x[i][j] + x[j][i] <= 1);
//        }
//    }
//
//    // Constraint 10
//    for (int h : C) {
//        IloExpr sum(env);
//        std::string cname = "C10_h_" + std::to_string(h);
//        for (int j : c_t) {
//            if (h != j) {
//                sum += y[h][j];
//            }
//        }
//        model.add(sum + theta[h] == 1);
//    }
//    // Constraint 11
//    for (int i : c_s) {
//        for (int j : c_t) {
//            if (i != j) {
//                std::string cname = "C11_i_" + std::to_string(i) + "_j=" + std::to_string(j);
//                IloExpr sum(env);
//                for (int h : c_prime) {
//                    sum += gamma[h][i][j];
//                }
//                model.add(sum <= y[i][j]);
//            }
//        }
//    }
//
//
//    // Constraint 12
//    for (int h : C) {
//        IloExpr sum1(env), sum2(env);
//
//        for (int i : V) {
//            if (i != h && i != t) {
//                sum1 += omega[h][i];
//            }
//        }
//        for (int j : V) {
//            if (j != s && j != h) {
//                sum2 += delta[h][j];
//            }
//        }
//        model.add(sum1 == theta[h]);
//        model.add(sum2 == theta[h]);
//    }
//    // Constraint 13
//    for (int i : c_s) {
//        for (int j : c_t) {
//            if (i != j) {
//                std::string cname = "C13_i_" + std::to_string(i) + "_j=" + std::to_string(j);
//                model.add(x[i][j] <= theta[i] + theta[j]);
//            }
//        }
//    }
//    //        // Constraint 14
//    for (int i : c_s) {
//        for (int j : c_t) {
//            if (i != s && j != t && i != j) {
//                std::string cname = "C14_i_" + std::to_string(i) + "_j=" + std::to_string(j);
//                model.add(x[i][j] <= omega[j][i] + delta[i][j]);
//            }
//        }
//    }
//    // Constraint 15
//    for (int i : c_s) {
//        IloExpr sum1(env), sum2(env);
//        std::string cname = "C15_i_" + std::to_string(i);
//        for (int j : c_t) {
//            if (i != j) {
//                sum1 += x[i][j];
//            }
//        }
//        for (int h : c_prime) {
//            sum2 += omega[h][i];
//        }
//        sum2 += theta[i];
//        model.add(sum1 == sum2);
//        model.add(sum1 <= 1);
//        model.add(sum2 <= 1);
//    }
//    // Constraint 16
//    for (int j : c_t) {
//        IloExpr sum1(env), sum2(env);
//        for (int i : c_s) {
//            sum1 += x[i][j];
//        }
//
//        for (int h : c_prime) {
//            sum2 += delta[h][j];
//        }
//        sum2 += theta[j];
//        model.add(sum1 == sum2);
//        model.add(sum1 <= 1);
//        model.add(sum2 <= 1);
//    }
//    // Constraint 17
//    for (int h : c_prime) {
//        IloExpr sum(env);
//        std::string cname = "C17_h_" + std::to_string(h);
//        for (int i : c_s) {
//            for (int j : c_t) {
//                sum += tau[i][j] * gamma[h][i][j];;
//            }
//        }
//        model.add(sum <= (cfg.dtl - cfg.sr) * theta[h]);
//    }
//    // Constraint 18
//    for (int h : c_prime) {
//        IloExpr sum1(env);
//        IloExpr sum2(env);
//        IloExpr sum3(env);
//        std::string c18_name = "C18_h_" + std::to_string(h);
//        std::string c19_name = "C19_h_" + std::to_string(h);
//
//        for (int i : c_s) {
//            sum1 += d[i][h] * omega[h][i];
//        }
//        for (int j : c_t) {
//            sum2 += d[h][j] * delta[h][j];
//        }
//
//        for (int i : c_s) {
//            for (int j : c_t) {
//                sum3 += tau[i][j] * gamma[h][i][j];
//            }
//        }
//        model.add(sum1 + sum2 <= (cfg.dtl - cfg.sr) * theta[h]);
//        model.add(sum1 + sum2 - sum3 <= sigma[h]);
//    }
//    model.add(IloMinimize(env, objective));
//    auto startTime = std::chrono::high_resolution_clock::now();
//    cplex.solve();
//    auto endTime = std::chrono::high_resolution_clock::now();
//    // Calculate the elapsed time
//    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
//    std::cout << "Truck arcs:" << std::endl;
//    for (int i : c_s) {
//        for (int j : c_t) {
//            if (i != j) {
//                if (cplex.getValue(y[i][j]) == 1) {
//                    std::cout << i << " " << j << std::endl;
//                }
//            }
//        }
//    }
//    for (int h : C) {
//        if (cplex.getValue(theta[h]) == 1) {
//            std::cout << "customer " << h << " is served by drone" << std::endl;
//            for (int i : c_s) {
//                if (h != i) {
//                    if (cplex.getValue(omega[h][i]) == 1) {
//                        std::cout << "start of this sortie: " << i << std::endl;
//                    }
//                }
//            }
//            for (int j : c_t) {
//                if (j != h) {
//                    if (cplex.getValue(delta[h][j]) == 1) {
//                        std::cout << "end of this sortie: " << j << std::endl;
//                    }
//                }
//            }
//            std::cout << std::endl;
//        }
//    }
//    std::cout << cplex.getObjValue() << std::endl;
//    std::vector<Sortie> st;
//    return Result{cplex.getObjValue(), duration.count() / 1000.0, cplex.getMIPRelativeGap()};
//}
//
//Result FSTSPSolver::Stage_Based_FSTSP_Original(Config& cfg) const {
//    auto tau = instance->tau;
//    auto tau_prime = instance->tau_prime;
//    auto n = instance->num_node;
//    std::vector<int> C;
//    std::vector<int> V;
//    for (int i = 0; i < n + 1; i++) {
//        if (i != 0 && i != n) {
//            C.push_back(i);
//        }
//        V.push_back(i);
//    }
//
//    IloEnv env;
//    IloModel model(env);
//    IloCplex cplex(model);
//    auto O = 0;
//    auto D = n;
//    int K = n + 1;
//    auto K_arc = K - 1;
//    // cplex.setParam(IloCplex::Param::MIP::Tolerances::Integrality, 0);
//    // Variable declaration
//    // X^i_k (binary variable) và nhận giá trị một tương ứng với đỉnh thứ k của
//    //đường đi của vehicle là i; k \in 1..n;
//    IloArray<IloBoolVarArray> X(env, K + 1);
//    for (int k = 1; k <= K; k++) {
//        X[k] = IloBoolVarArray(env, D + 1);
//        for (int i = 0; i <= D; i++) {
//            X[k][i] = IloBoolVar(env);
//            model.add(X[k][i]);
//            auto v_name = "X_" + std::to_string(k) + "_" + std::to_string(i);
//            X[k][i].setName(v_name.c_str());
//        }
//        if (k > 1) {
//            model.add(X[k][0] == 0);
//        }
//    }
//
//    model.add(X[1][0] == 1).setName("First stage must be source depot");
//
//    // x^k_(ij) (binary variable) và nhận giá trị một nếu Xk
//    // mô ta cạnh nối 2 đỉnh liên tiếp trên đường đi.
//    IloArray<IloArray<IloBoolVarArray>> x(env, K_arc + 1);
//    for (int k = 1; k <= K_arc; k++) {
//        x[k] = IloArray<IloBoolVarArray>(env, D);
//        for (int i = 0; i < D; i++) {
//            x[k][i] = IloBoolVarArray(env, D + 1);
//            for (int j = 1; j <= D; j++) {
//                x[k][i][j] = IloBoolVar(env);
//                auto v_name = "x_" + std::to_string(k) + "_" + std::to_string(i) + "_" + std::to_string(j);
//                x[k][i][j].setName(v_name.c_str());
//                if (!cfg.allow_loop) {
//                    if (i == j) {
//                        model.add(x[k][i][j] == 0);
//                    }
//                }
//            }
//        }
//    }
//
//    //// phi^h equals to 1 if customer h is served by the drone
//    IloBoolVarArray phi(env, n);
//    for (int h : C) {
//        phi[h] = IloBoolVar(env);
//        auto v_name = "phi_" + std::to_string(h);
//        phi[h].setName(v_name.c_str());
//        if (exist(instance->heavy, h)) {
//            model.add(phi[h] == 0);
//        }
//    }
//
//    IloArray<IloArray<IloBoolVarArray>> Y(env, K + 1), W(env, K + 1);
//    for (int k = 1; k <= K; k++) {
//        Y[k] = IloArray<IloBoolVarArray>(env, D + 1);
//        W[k] = IloArray<IloBoolVarArray>(env, D + 1);
//
//        for (int i = 0; i <= D; i++) {
//            Y[k][i] = IloBoolVarArray(env, C.size() + 1);
//            W[k][i] = IloBoolVarArray(env, C.size() + 1);
//            for (int h : C) {
//                if (i != h) {
//                    Y[k][i][h] = IloBoolVar(env);
//                    Y[k][i][h].setName(("Y_" + std::to_string(k) + "_"
//                        + std::to_string(i) + "_" + std::to_string(h)).c_str());
//
//                    W[k][i][h] = IloBoolVar(env);
//                    W[k][i][h].setName(("W_" + std::to_string(k) + "_"
//                        + std::to_string(i) + "_" + std::to_string(h)).c_str());
//
//                    if (i == 0 && k > 1) {
//                        model.add(Y[k][i][h] == 0);
//                    }
//                    if (i == D && k == 1) {
//                        model.add(W[k][i][h] == 0);
//                    }
//                    if (tau_prime[i][h] > cfg.dtl - cfg.sr) {
//                        model.add(Y[k][i][h] == 0);
//                    }
//                    if (tau_prime[h][i] > cfg.dtl - cfg.sr) {
//                        model.add(W[k][i][h] == 0);
//                    }
//                }
//                else {
//                    model.add(Y[k][i][h] == 0);
//                    model.add(W[k][i][h] == 0);
//                }
//            }
//        }
//    }
//
//
//    // $R_{k} = \sum_{k'}Z_{kk'}$: các đoạn bắt đầu từ k (C23)
//    IloBoolVarArray R(env, K + 1);
//    for (int k = 1; k < K; k++) {
//        R[k].setName(("R_" + std::to_string(k)).c_str());
//    }
//
//    // aux var Z_{k, k_p, h}: sortie launch from k and rendezvous at k_p.
//    IloArray<IloArray<IloBoolVarArray>> Z(env, K);
//    for (int h : C) {
//        Z[h] = IloArray<IloBoolVarArray>(env, K + 1);
//        for (int k = 1; k <= K - 1; k++) {
//            Z[h][k] = IloBoolVarArray(env, K + 1);
//            for (int kp = k + 1; kp <= K; kp++) {
//                Z[h][k][kp] = IloBoolVar(env);
//                auto v_name = "Z_" + std::to_string(h) + "_" + std::to_string(k) + "_" + std::to_string(kp);
//                Z[h][k][kp].setName(v_name.c_str());
//            }
//        }
//    }
//
//    //// aux var z_{k, k_p}: sortie launch from k and rendezvous at k_p.
//    IloArray<IloBoolVarArray> z(env, K);
//    for (int k = 1; k < K; k++) {
//        z[k] = IloBoolVarArray(env, K + 1);
//        for (int kp = k + 1; kp <= K; kp++) {
//            z[k][kp] = IloBoolVar(env);
//            auto v_name = "z_" + std::to_string(k) + "_" + std::to_string(kp);
//            z[k][kp].setName(v_name.c_str());
//        }
//    }
//
//    IloArray<IloNumVarArray> wait(env, K);
//    for (int k = 1; k < K; k++) {
//        wait[k] = IloNumVarArray(env, K + 1);
//        for (int kp = k + 1; kp <= K; kp++) {
//            std::string cname = "wait_" + std::to_string(k) + "_" + std::to_string(kp);
//            wait[k][kp] = IloNumVar(env, 0, IloInfinity, ILOFLOAT, cname.c_str());
//        }
//    }
//
//
//    ////-----------------------------------------------------------------------------------------------
//    // WARMSTART CONFIG: using original TSP solution.
//    // Obtained with MTZ formulation.
//    // On variable X.
//    if (cfg.use_tsp_for_warmstart) {
//        auto tsp_solution = TSP_MTZ(tau);
//        std::cout << tsp_solution.getSize() << std::endl;
//        IloNumVarArray X_warm_var(env);
//        IloNumArray X_warm_val(env);
//        for (int k = 1; k <= K - 1; k++) {
//            for (int i = 0; i <= D; i++) {
//                X_warm_var.add(X[k][i]);
//                if (tsp_solution[k - 1] == i) {
//                    X_warm_val.add(true);
//                    std::cout << "Warmstart X[" << k << "][" << i << "] == 1" << std::endl;
//                }
//                else {
//                    X_warm_val.add(false);
//                }
//            }
//        }
//        cplex.addMIPStart(X_warm_var, X_warm_val);
//        X_warm_var.end();
//        X_warm_val.end();
//    }
//    ////-----------------------------------------------------------------------------------------------
//
//    //// Sortie maximum stage gap calculation.
//    /// Find maximum stage gap that a sortie can start and rendezvous.
//    /// For each stage gap in increasing order (k' - k), find the minimum tour length from stage k to k'.
//    /// Revisit(s) are also considered.
//    /// If the tour length > cfg.dtl - cfg.sr => there can be no sortie with stage gap greater or equal k' - k.
//    bool used_stage_gap = false;
//    int min_stage_gap = 0;
//    std::cout << "---------------------------------------------" << std::endl;
//    std::cout << "Calculate max stage gap for sortie.... " << std::endl;
//    for (int k = 1; k < K; k++) {
//        for (int kp = k + 1; kp <= K; kp++) {
//            if (kp > k + 6) {
//                goto after_z_cons;
//            }
//            double smallest_tour = smallest_tour_length(kp - k, tau, V);
//            if (smallest_tour > cfg.dtl - cfg.sr) {
//                std::cout << "Stage gap of " << kp - k << " with smallest length = " << smallest_tour <<
//                    " violated endurance constraint!" << std::endl;
//                std::cout << "---------------------------------------------" << std::endl;
//                min_stage_gap = kp - k;
//                goto after_z_cons;
//            }
//        }
//    }
//after_z_cons:
//    if (min_stage_gap == 0) {
//        std::cout << "Stage gap calculation consumes too much memory. No constraint was added." << std::endl;
//        std::cout << "---------------------------------------------" << std::endl;
//    }
//    if (min_stage_gap != 0) {
//        used_stage_gap = true;
//        for (int k = 1; k < K; k++) {
//            for (int kp = k + min_stage_gap; kp <= K; kp++) {
//                model.add(z[k][kp] == 0);
//                std::cout << "Variable z[" << k << "][" << kp << "] was set to 0." << std::endl;
//                for (int h : instance->c_prime) {
//                    model.add(Z[h][k][kp] == 0);
//                    IloExpr start_stage_k(env), end_stage_kp(env);
//                    for (int i = 0; i < D; i++) {
//                        if (i != h) {
//                            start_stage_k += Y[k][i][h];
//                            // model.add(Z[h][k][kp] + Y[k][i][h] <= 1);
//                        }
//                    }
//                    for (int j = 1; j <= D; j++) {
//                        if (j != h) {
//                            end_stage_kp += W[kp][j][h];
//                            // model.add(Z[h][k][kp] + W[kp][j][h] <= 1);
//                        }
//                    }
//                    std::string cname = "Can't serve customer " + std::to_string(h) + " start at stage " +
//                        std::to_string(k) + ", end at stage " + std::to_string(kp);
//                    // Looks like quicker lower bound improvements with this constraints.
//                    model.add(start_stage_k + end_stage_kp <= 1).setName(cname.c_str());
//                }
//            }
//        }
//    }
//
//    ////-----------------------------------------------------------------------------------------------
//
//    // CONSTRAINTS DECLARATION.------------------------------------------------------------------------
//
//    // A node cannot be revisit if (allow loop + not_allow_revisit).
//    if (cfg.allow_loop) {
//        for (int i : C) {
//            IloExpr sum_visit_i(env);
//            IloExpr sum_start_i(env);
//            IloExpr sum_end_i(env);
//
//            for (int k = 2; k < K; k++) {
//                sum_visit_i += X[k][i];
//            }
//            for (int k = 2; k < K; k++) {
//                for (int h : instance->c_prime) {
//                    if (i != h) {
//                        sum_start_i += Y[k][i][h];
//                    }
//                }
//            }
//            for (int k = 2; k < K; k++) {
//                for (int h : instance->c_prime) {
//                    if (i != h) {
//                        sum_end_i += W[k][i][h];
//                    }
//                }
//            }
//            model.add(sum_visit_i <= 1 + sum_start_i);
//            model.add(sum_visit_i <= 1 + sum_end_i);
//        }
//    }
//
//    if (!cfg.allow_revisit) {
//        // 1-visit constraint
//        for (int i : V) {
//            IloExpr sum_visit_i(env);
//            for (int k = 1; k <= K; k++) {
//                sum_visit_i += X[k][i];
//            }
//            model.add(sum_visit_i <= 1);
//        }
//    }
//
//    // Constraint C1
//    for (int k = 1; k < K; k++) {
//        for (int i = 0; i < D; i++) {
//            IloExpr sum(env);
//            for (int j = 1; j <= D; j++) {
//                sum += x[k][i][j];
//            }
//            model.add(X[k][i] == sum).setName(("C1_" + std::to_string(k) + "_" + std::to_string(i)).c_str());
//        }
//    }
//
//
//    for (int k = 2; k <= K; k++) {
//        for (int i = 1; i <= D; i++) {
//            IloExpr sum(env);
//            for (int j = 0; j < D; j++) {
//                sum += x[k - 1][j][i];
//            }
//            // arcs entering i at stage k.
//            model.add(X[k][i] == sum).setName(("C1p_" + std::to_string(k) + "_" + std::to_string(i)).c_str());
//        }
//    }
//
//    // C2: truck must depart from the depot at stage 1.
//    IloExpr C2(env);
//    for (int i = 1; i <= D; i++) {
//        C2 += x[1][O][i];
//    }
//
//    IloConstraint c2(C2 == 1);
//    model.add(c2).setName("Must start from source");
//
//    // C3: Terminal depot must be arrived only once.
//    IloExpr C3(env);
//    for (int k = 2; k <= K; k++) {
//        C3 += X[k][D];
//    }
//    model.add(C3 == 1).setName("Must visit terminal depot once");
//
//    // At most ONE node each stage.
//    for (int k = 1; k <= K; k++) {
//        IloExpr sum(env);
//        for (int i = 0; i <= D; i++)
//            sum += X[k][i];
//        model.add(sum <= 1).setName(("C20_at_most_one_customer_at_stage_" + std::to_string(k)).c_str());
//    }
//
//    // $Z_{kk'} = \sum_{h}Z^h_{kk'}$: mỗi cặp (k,k') chỉ phục vụ tối đa một khách hàng (C22)
//    for (int k = 1; k < K; k++) {
//        for (int kp = k + 1; kp <= K; kp++) {
//            IloExpr expr(env);
//            for (int h : C) {
//                expr += Z[h][k][kp];
//            }
//            model.add(expr == z[k][kp]).setName(("C22_" + std::to_string(k) + "_" + std::to_string(kp)).c_str());
//        }
//    }
//
//
//
//    //C20:$\sum_{k'>k}Z_{kk'} = \sum_{i,h}Y^k_{ih}$ : với mỗi $k$,
//    //ràng buộc liên kết drone đi ra từ stage $k$ và đoạn mà oto di chuyển không có drone. (C20)
//    for (int h : C) {
//        for (int k = 1; k <= K - 1; k++) {
//            IloExpr expr(env);
//            for (int k_p = k + 1; k_p <= K; k_p++) {
//                expr += Z[h][k][k_p];
//            }
//
//            for (int i = 0; i < D; i++) {
//                if (i != h && tau_prime[i][h] <= cfg.dtl - cfg.sr) {
//                    expr -= Y[k][i][h];
//                }
//            }
//            model.add(expr == 0).setName(("C20_" + std::to_string(k) + "_" + std::to_string(h)).c_str());
//        }
//    }
//
//    for (int h : C) {
//        for (int k_p = 2; k_p <= K; k_p++) {
//            IloExpr expr(env);
//            for (int k = 1; k < k_p; k++) {
//                expr += Z[h][k][k_p];
//            }
//
//            for (int i = 1; i <= D; i++) {
//                if (i != h && tau_prime[h][i] <= cfg.dtl - cfg.sr) {
//                    expr -= W[k_p][i][h];
//                }
//            }
//            model.add(expr == 0).setName(("C20p_" + std::to_string(k_p)
//                + "_" + std::to_string(h)).c_str());
//        }
//    }
//
//    for (int k = 1; k < K; k++) {
//        IloExpr expr(env);
//        for (int k_p = k + 1; k_p <= K; k_p++) {
//            expr += z[k][k_p];
//        }
//        model.add(R[k] == expr).setName(("C23_" + std::to_string(k)).c_str());
//    }
//
//    // modified C7
//    for (int k = 1; k <= K - 1; k++) {
//        for (int k_p = k + 1; k_p <= K; k_p++) {
//            for (int l = k + 1; l < k_p; l++) {
//                // tranh drone bay cac doan giao nhau.
//                if (k < l) {
//                    model.add(z[k][k_p] + R[l] <= 1).setName(("C7m_" + std::to_string(k)
//                            + "_" + std::to_string(k_p) + "_" + std::to_string(l))
//                        .c_str());
//                }
//            }
//        }
//    }
//    //// C17 - $X^k_i \geq \sum_h X^k_{ih}$ (C17) - chỉ bay drone ở nơi mà xe ở đó
//    for (int k = 1; k <= K; k++) {
//        for (int i = 0; i <= D; i++) {
//            IloExpr expr(env);
//            for (int h : C) {
//                if (h != i) {
//                    if (tau_prime[i][h] <= cfg.dtl - cfg.sr) {
//                        expr += Y[k][i][h];
//                    }
//                }
//            }
//            model.add(expr <= X[k][i]).setName(("C17_" + std::to_string(k) + "_" + std::to_string(i)).c_str());
//        }
//    }
//
//    // $X^k_i \geq \sum_h Y^k_{ih}$ (C17p) : chỉ bay drone tới nơi mà xe ở đó
//    for (int k = 1; k <= K; k++) {
//        for (int i = 0; i <= D; i++) {
//            IloExpr expr(env);
//            for (int h : C) {
//                if (h != i) {
//                    if (tau_prime[h][i] <= cfg.dtl - cfg.sr) {
//                        expr += W[k][i][h];
//                    }
//                }
//            }
//            model.add(expr <= X[k][i]).setName(("C17p_" + std::to_string(k) + "_" + std::to_string(i)).c_str());
//        }
//    }
//
//    // $\sum_{i} X^k_{ih}\tau^D_{ih} + \sum_{i} Y^{k'}_{ih}\tau^D_{hi} \leq D_d$
//    //- drone duration constraint cho mỗi $h$ (C19)
//    for (int h : C) {
//        IloExpr expr(env);
//
//        for (int k = 1; k <= K; k++) {
//            for (int i = 0; i <= D; i++) {
//                if (i != h && i != D && tau_prime[i][h] <= cfg.dtl - cfg.sr) {
//                    expr += Y[k][i][h] * tau_prime[i][h];
//                }
//
//                if (i != h && i != 0 && tau_prime[h][i] <= cfg.dtl - cfg.sr) {
//                    expr += W[k][i][h] * tau_prime[h][i];
//                }
//            }
//        }
//        model.add(expr <= (cfg.dtl - cfg.sr) * phi[h]).setName(("C19_" + std::to_string(h)).c_str());
//    }
//
//    // modified C7p - we can select at most one segment that contains the point l
//    for (int l = 2; l < K; l++) {
//        IloExpr expr(env);
//
//        for (int k = 1; k < l; k++)
//            for (int k_p = l; k_p <= K; k_p++)
//                expr += z[k][k_p];
//        model.add(expr <= 1).setName(("C7mm_" + std::to_string(l)).c_str());
//    }
//
//    for (int i = 0; i < D; i++) {
//        for (int k = 1; k <= K - 1; k++) {
//            IloExpr lhs(env);
//
//            for (int h : C) {
//                if (i != h) {
//                    lhs += Y[k][i][h];
//                }
//            }
//            model.add(lhs <= X[k][i]).setName(("C8_launch_" + std::to_string(i) + "_" + std::to_string(k)).c_str());
//        }
//    }
//
//    for (int j = 1; j <= D; j++) {
//        for (int k_p = 2; k_p <= K; k_p++) {
//            IloExpr lhs(env);
//
//            for (int h : C) {
//                if (h != j) {
//                    lhs += W[k_p][j][h];
//                }
//            }
//
//            model.add(lhs <= X[k_p][j]).setName(
//                ("C8_rendezvous_" + std::to_string(j) + "_" + std::to_string(k_p)).c_str());
//        }
//    }
//
//    // $\phi_h = \sum_{k,i}X^k_{ih} = \sum_{k,i}Y^k_{ih}$
//    // - chỉ có duy nhất một điểm xuất phát và môt điểm đích cho
//    //mỗi khách hàng $h$ được phục vụ bởi drone (C18)
//    for (int h : C) {
//        IloExpr rhs(env);
//        for (int i = 0; i < D; i++) {
//            if (i != h && tau_prime[i][h] <= cfg.dtl - cfg.sr) {
//                for (int k = 1; k <= K - 1; k++) {
//                    rhs += Y[k][i][h];
//                }
//            }
//        }
//        // consistency constraint cho sortie phuc vu h.
//        model.add(phi[h] == rhs).setName(("C18_" + std::to_string(h)).c_str());
//    }
//
//    for (int h : C) {
//        IloExpr rhs(env);
//        for (int i = 1; i <= D; i++) {
//            if (i != h && tau_prime[h][i] <= cfg.dtl - cfg.sr) {
//                for (int k = 2; k <= K; k++) {
//                    rhs += W[k][i][h];
//                }
//            }
//        }
//        // consistency constraint cho sortie phuc vu h.
//        model.add(phi[h] == rhs).setName(("C18p_" + std::to_string(h)).c_str());
//    }
//
//    // Assignment constraint
//    //////////// C10: node_stage
//    for (int h : C) {
//        IloExpr sum_k(env);
//        for (int k = 2; k < K; k++) {
//            sum_k += X[k][h];
//        }
//        // phuc vu h it nhat 1 lan. == 1 thì thành phục vụ duy nhất 1 lần.
//        model.add(phi[h] + sum_k >= 1).setName(("C10_" + std::to_string(h)).c_str());
//    }
//
//    ////////// C15: node_stage
//    ///// big M calculation
//    double M = 0;
//    for (int i = 0; i < n; i++) {
//        for (int j = 0; j < n; j++) {
//            if (i < j) {
//                M += tau[i][j];
//            }
//        }
//    }
//
//
//    for (int k = 1; k < K; k++) {
//        for (int k_p = k + 1; k_p <= K; k_p++) {
//            // model.add(a[k_p] - d[k] <= z[k][k_p] * (cfg.dtl - cfg.sr) + (1 - z[k][k_p]) * M).setName(
//            //     ("C15_" + std::to_string(k) + "_" + std::to_string(k_p)).c_str());
//            IloExpr truck_length(env);
//            for (int start = k; start < k_p; start++) {
//                for (int i = 0; i < D; i++) {
//                    for (int j = 1; j <= D; j++) {
//                        if (i != j) {
//                            truck_length += x[start][i][j] * tau[i][j];
//                        }
//                    }
//                }
//            }
//            model.add(truck_length <= z[k][k_p] * (cfg.dtl - cfg.sr) + (1 - z[k][k_p]) * M);
//        }
//    }
//    // SLOW
//    // (k,h): k chỉ có thể là đến hoặc đi của khách hàng h.
//    if (!cfg.allow_loop) {
//        for (int h : instance->c_prime) {
//            for (int i : C) {
//                if (i != h) {
//                    for (int k = 1; k < K; k++) {
//                        for (int kp = k + 1; kp < K; kp++) {
//                            model.add(Y[k][i][h] + W[kp][i][h] <= phi[h]);
//                        }
//                    }
//                    // SLOW VARIANT.
//                }
//            }
//        }
//    }
//
//
//    IloExpr objective(env);
//    for (int k = 1; k < K; k++) {
//        for (int i = 0; i < D; i++) {
//            for (int j = 1; j <= D; j++) {
//                if (i != j) {
//                    objective += x[k][i][j] * tau[i][j];
//                }
//            }
//        }
//    }
//    for (int h : instance->c_prime) {
//        objective += (cfg.sl + cfg.sr) * phi[h];
//    }
//
//    for (int kp = 2; kp <= K; kp++) {
//        objective -= cfg.sl * z[1][kp];
//    }
//    for (int k = 1; k < K; k++) {
//        for (int kp = k + 1; kp <= K; kp++) {
//            IloExpr sum_drone_start_k(env);
//            IloExpr sum_drone_end_kp(env);
//            IloExpr sum_truck_kkp(env);
//
//            for (int i = 0; i < D; i++) {
//                for (int h : instance->c_prime) {
//                    if (i != h) {
//                        sum_drone_start_k += Y[k][i][h] * tau_prime[i][h];
//                    }
//                }
//            }
//            for (int j = 1; j <= D; j++) {
//                for (int h : instance->c_prime) {
//                    if (j != h) {
//                        sum_drone_end_kp += W[kp][j][h] * tau_prime[h][j];
//                    }
//                }
//            }
//            for (int l = k; l <= kp - 1; l++) {
//                for (int i = 0; i < D; i++) {
//                    for (int j = 1; j <= D; j++) {
//                        if (i != j) {
//                            sum_truck_kkp += x[l][i][j] * tau[i][j];
//                        }
//                    }
//                }
//            }
//            model.add(wait[k][kp] >= sum_drone_start_k + sum_drone_end_kp - sum_truck_kkp - M * (1 - z[k][kp]));
//        }
//    }
//
//    for (int k = 1; k < K; k++) {
//        for (int kp = k + 1; kp <= K; kp++) {
//            objective += wait[k][kp];
//        }
//    }
//
//
//    // model.add(IloMinimize(env, d[K]));
//    model.add(IloMinimize(env, objective));
//    cplex.exportModel("cplex_model_1.lp");
//    std::vector<Sortie> st;
//    int loop_count_val = 0;
//    double obj = 0;
//    double revisit_count = 0;
//    // Solve the model
//    std::vector<int> rev(n + 1, 0);
//    auto startTime = std::chrono::high_resolution_clock::now();
//    cplex.solve();
//    auto endTime = std::chrono::high_resolution_clock::now();
//    // Calculate the elapsed time
//    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
//    std::vector<int> truck_order;
//    std::vector<std::vector<int>> drone_order;
//    // Check if the problem is infeasible
//
//    if (cplex.getStatus() == IloAlgorithm::Infeasible) {
//        // Handle infeasibility
//        std::cout << "The problem is infeasible." << std::endl;
//        std::cout << "Infeasibility at: " << cplex.getInfeasibility(c2) << std::endl;
//        // You can also retrieve the infeasible constraints using cplex.getInfeasibility() method
//    }
//
//    std::cout << "Feasible solution found!" << std::endl;
//    std::cout << "Truck nodes:" << std::endl;
//    for (int k = 1; k <= K; k++) {
//        for (int i = 0; i <= D; i++) {
//            auto X_val = cplex.getValue(X[k][i]);
//            //std::cout << "k = " << k << ", i = " << i << ":" << X_val << std::endl;
//            if (abs(X_val - 1) < 1e-5) {
//                rev[i]++;
//                std::cout << "Stage " << k << " at customer " << i << " with arrival time is: " << std::endl;
//                truck_order.push_back(i);
//                break;
//            }
//        }
//    }
//    std::cout << "Truck arcs:" << std::endl;
//    std::map<int, std::pair<int, int>> map_stage_truck_arc;
//    for (int k = 1; k <= K_arc; k++) {
//        for (int i = 0; i < D; i++) {
//            for (int j = 1; j <= D; j++) {
//                auto X_val = cplex.getValue(x[k][i][j]);
//                if (abs(X_val - 1) < 1e-5) {
//                    std::cout << "Arc " << k << " connecting " << i << " and " << j
//                        << " with cost " << tau[i][j] << " " << std::endl;
//                    obj += tau[i][j];
//                    map_stage_truck_arc[k] = std::make_pair(i, j);
//                    break;
//                }
//            }
//        }
//    }
//
//    for (int h : C) {
//        if (abs(cplex.getValue(phi[h]) - 1) < 1e-5) {
//            std::cout << "Customer " << h << " served by drone." << std::endl;
//            int sv_i = -1, sv_j = -1, sv_k = -1, sv_kp = -1;
//            for (int k = 1; k <= K; k++) {
//                for (int i = 0; i <= D; i++)
//                    if (i != h) {
//                        try {
//                            auto Y_val = cplex.getValue(Y[k][i][h]);
//                            if (abs(Y_val - 1) < 1e-5) {
//                                sv_i = i;
//                                sv_k = k;
//                            }
//                            auto W_val = cplex.getValue(W[k][i][h]);
//                            if (abs(W_val - 1) < 1e-5) {
//                                sv_j = i;
//                                sv_kp = k;
//                            }
//                        }
//                        catch (...) {}
//                    }
//            }
//            if (sv_i == sv_j) {
//                std::cout << "Loop occured!! Start and end node: " << sv_i << " " << sv_j << std::endl;
//                loop_count_val++;
//            }
//            std::cout << "Drone fly from " << sv_i << " at stage " << sv_k <<
//                " to serve " << h << " and then fly back to " << sv_j
//                << " at stage " << sv_kp << ". " << std::endl;
//            drone_order.push_back({sv_k, h, sv_kp});
//            obj += (cfg.sl + cfg.sr);
//            if (sv_i == O) {
//                obj -= cfg.sl;
//            }
//            double drone_travel_time = tau_prime[sv_i][h] + tau_prime[h][sv_j];
//            double truck_travel_time = 0;
//            std::cout << "Truck arcs during this sortie: ";
//            for (int k_start = sv_k; k_start <= sv_kp; k_start++) {
//                if (k_start == sv_kp) {
//                    std::cout << map_stage_truck_arc[k_start].first;
//                }
//                else {
//                    std::cout << map_stage_truck_arc[k_start].first << "--->";
//                }
//            }
//            std::cout << std::endl;
//            for (int k_start = sv_k; k_start <= sv_kp - 1; k_start++) {
//                truck_travel_time += tau[map_stage_truck_arc[k_start].first][map_stage_truck_arc[k_start].second];
//            }
//            std::cout << "Truck travel time from stage " << sv_k << " to " << sv_kp << " is: " << truck_travel_time <<
//                std::endl;
//            if (drone_travel_time > truck_travel_time) {
//                obj += drone_travel_time - truck_travel_time;
//            }
//            std::cout << "Total drone travel time: " << drone_travel_time << std::endl;
//
//            assert(abs(cplex.getValue(Z[h][sv_k][sv_kp]) - 1.0) < 1e-5);
//
//            assert(abs(cplex.getValue(z[sv_k][sv_kp]) - 1.0) < 1e-5);
//        }
//    }
//
//    for (int h : instance->c_prime) {
//        for (int k = 1; k < K; k++) {
//            for (int kp = k + 1; kp <= K; kp++) {
//                if (abs(cplex.getValue(Z[h][k][kp]) - 1) < 1e-5) {
//                    std::cout << "Customer " << h << " served, start stage = " << k << ", end stage = " << kp <<
//                        std::endl;
//                    std::cout << "Wait time for this sortie: " << cplex.getValue(wait[k][kp]) << std::endl;
//                }
//            }
//        }
//    }
//
//    std::cout << "Printing Y(k,i,h):" << std::endl;
//    for (int k = 1; k < K; k++) {
//        for (int i = 0; i < D; i++) {
//            for (int h : instance->c_prime) {
//                if (i != h) {
//                    if (abs(cplex.getValue(Y[k][i][h]) - 1) < 1e-6) {
//                        std::cout << "Y[" << k << "][" << i << "][" << h << "] == 1" << std::endl;
//                    }
//                }
//            }
//        }
//    }
//    std::cout << "Done!" << std::endl;
//    std::cout << "-------------------------Re-calculated objective-----------------------" << std::endl;
//    std::cout << obj << std::endl;
//    std::cout << "------------------------------------------------------------------------" << std::endl;
//
//    double obj2 = 0;
//    for (int k = 1; k < K; k++) {
//        for (int i = 0; i < D; i++) {
//            for (int j = 1; j <= D; j++) {
//                if (i != j) {
//                    obj2 += cplex.getValue(x[k][i][j]) * tau[i][j];
//                }
//            }
//        }
//    }
//    for (int h : instance->c_prime) {
//        obj2 += (cfg.sl + cfg.sr) * cplex.getValue(phi[h]);
//    }
//
//    for (int kp = 2; kp <= K; kp++) {
//        obj2 -= cfg.sl * cplex.getValue(z[1][kp]);
//    }
//    for (int k = 1; k < K; k++) {
//        for (int kp = k + 1; kp <= K; kp++) {
//            obj2 += cplex.getValue(wait[k][kp]);
//        }
//    }
//
//    std::cout << "obj22222222222222: " << obj2 << std::endl;
//
//    double c = cplex.getObjValue();
//    double gap = cplex.getMIPRelativeGap();
//    std::cout << "Solve time: " << duration.count() / 1000.0 << std::endl;
//    cplex.end();
//    model.end();
//    env.end();
//    for (int i = 0; i < rev.size(); i++) {
//        if (rev[i] > 1) {
//            revisit_count += rev[i] - 1;
//            std::cout << "Node " << i << " was revisited " << rev[i] - 1 << " times!" << std::endl;
//        }
//    }
//    std::cout << "OBJECTIVE VALUE: " << c << std::endl;
//    std::cout << "Number of loop(s):" << loop_count_val << std::endl;
//    std::cout << "Number of revisit: " << revisit_count << std::endl;
//
//    std::cout << "-------------------------Re-evaluate solution-----------------------" << std::endl;
//    std::sort(drone_order.begin(), drone_order.end(), [](const std::vector<int>& a, const std::vector<int>& b) {
//        return a[0] < b[0];
//    });
//    Solution solution(c, instance, &cfg, truck_order, drone_order);
//    // Project path (work for Linux)
//    string path = std::filesystem::current_path().string();
////    solution.write(path);
//    solution.isFeasible();
//
//    return Result{c, duration.count() / 1000.0, gap, loop_count_val};
//}
//
//Result
//FSTSPSolver::RV_FSTSP_MVD_Re(Config& cfg) const {
//    auto tau = instance->tau;
//    auto tau_prime = instance->tau_prime;
//
//    auto n = instance->num_node;
//    std::vector<int> C;
//    std::vector<int> V;
//    std::vector<int> c_s;
//    std::vector<int> c_t;
//    for (int i = 0; i < n + 1; i++) {
//        if (i != 0 && i != n) {
//            C.push_back(i);
//        }
//        if (i != 0) {
//            c_t.push_back(i);
//        }
//        if (i != n) {
//            c_s.push_back(i);
//        }
//        V.push_back(i);
//    }
//
//    IloEnv env;
//    IloModel model(env);
//    IloCplex cplex(model);
//
//    auto t = n;
//
//    auto O = 0;
//    auto D = n;
//    int K = n + 1;
//    auto K_arc = K - 1;
//    std::vector<int> start_stage, end_stage;
//    for (int k = 1; k <= K; k++) {
//        if (k < K) {
//            start_stage.push_back(k);
//        }
//        if (k > 1) {
//            end_stage.push_back(k);
//        }
//    }
//    cplex.setParam(IloCplex::Param::Threads, cfg.num_thread);
//    cplex.setParam(IloCplex::Param::TimeLimit, 3600.0);
//    // Variable declaration
//    // X^i_k (binary variable) và nhận giá trị một tương ứng với đỉnh thứ k của
//    //đường đi của vehicle là i; k \in 1..n;
//    IloArray<IloBoolVarArray> X(env, K + 1);
//    for (int k = 1; k <= K; k++) {
//        X[k] = IloBoolVarArray(env, D + 1);
//        for (int i = 0; i <= D; i++) {
//            X[k][i] = IloBoolVar(env);
//            model.add(X[k][i]);
//            auto v_name = "X_" + std::to_string(k) + "_" + std::to_string(i);
//            X[k][i].setName(v_name.c_str());
//        }
//        if (k > 1) {
//            model.add(X[k][0] == 0);
//        }
//    }
//
//    model.add(X[1][0] == 1).setName("First stage must be source depot");
//
//    // x^k_(ij) (binary variable) và nhận giá trị một nếu Xk
//    // mô ta cạnh nối 2 đỉnh liên tiếp trên đường đi.
//    IloArray<IloArray<IloBoolVarArray>> x(env, K_arc + 1);
//    for (int k = 1; k <= K_arc; k++) {
//        x[k] = IloArray<IloBoolVarArray>(env, D);
//        for (int i = 0; i < D; i++) {
//            x[k][i] = IloBoolVarArray(env, D + 1);
//            for (int j = 1; j <= D; j++) {
//                x[k][i][j] = IloBoolVar(env);
//                auto v_name = "x_t" + std::to_string(k) + "_" + std::to_string(i) + "_" + std::to_string(j);
//                x[k][i][j].setName(v_name.c_str());
//                if (!cfg.allow_loop) {
//                    if (i == j) {
//                        model.add(x[k][i][j] == 0);
//                    }
//                }
//            }
//        }
//    }
//
//    //// phi^h equals to 1 if customer h is served by the drone
//    IloBoolVarArray phi(env, n);
//    for (int h : C) {
//        phi[h] = IloBoolVar(env);
//        auto v_name = "phi_" + std::to_string(h);
//        phi[h].setName(v_name.c_str());
//        if (exist(instance->heavy, h)) {
//            model.add(phi[h] == 0);
//        }
//    }
//
//    // biến y cho đường đi của drone: đi từ i đến j trong sortie từ stage (k -> k')
//    IloArray<IloArray<IloBoolVarArray>> y(env, K);
//    for (int k : start_stage) {
//        y[k] = IloArray<IloBoolVarArray>(env, D);
//        for (int i : c_s) {
//            y[k][i] = IloBoolVarArray(env, D + 1);
//            for (int j : c_t) {
//                if (i != j) {
//                    y[k][i][j] = IloBoolVar(env);
//                }
//            }
//        }
//    }
//
//    // Biến zl(k, i): có 1 sortie bắt đầu từ stage k, đỉnh i.
//    IloArray<IloBoolVarArray> l(env, K);
//    for (int k : start_stage) {
//        l[k] = IloBoolVarArray(env, D);
//        for (int i : c_s) {
//            l[k][i] = IloBoolVar(env);
//        }
//    }
//
//    // Biến zr(k, j): sortie bắt đầu từ stage k có kết thúc tại đỉnh j.
//    IloArray<IloBoolVarArray> r(env, K);
//    for (int k : start_stage) {
//        r[k] = IloBoolVarArray(env, D + 1);
//        for (int j : c_t) {
//            r[k][j] = IloBoolVar(env);
//        }
//    }
//
//    // Biến zs(k, h): sortie bắt đầu từ stage k phục vụ khách hàng h bởi drone.
//    IloArray<IloBoolVarArray> s(env, K);
//    for (int k : start_stage) {
//        s[k] = IloBoolVarArray(env, D);
//        for (int h : C) {
//            s[k][h] = IloBoolVar(env);
//            if (exist(instance->heavy, h)) {
//                model.add(s[k][h] == 0);
//            }
//        }
//    }
//
//    IloArray<IloBoolVarArray> yd(env, D);
//    for (int i : c_s) {
//        yd[i] = IloBoolVarArray(env, D + 1);
//        for (int j : c_t) {
//            if (i != j) {
//                std::string vname = "x_d_" + std::to_string(i) + "_" + std::to_string(j);
//                yd[i][j] = IloBoolVar(env);
//                yd[i][j].setName(vname.c_str());
//            }
//        }
//    }
//    // aux var z
//    //// aux var z_{k, k_p}: sortie launch from k and rendezvous at k_p.
//    IloArray<IloBoolVarArray> z(env, K);
//    for (int k = 1; k < K; k++) {
//        z[k] = IloBoolVarArray(env, K + 1);
//        for (int k_p = k + 1; k_p <= K; k_p++) {
//            z[k][k_p] = IloBoolVar(env);
//            auto v_name = "z_" + std::to_string(k) + "_" + std::to_string(k_p);
//            z[k][k_p].setName(v_name.c_str());
//        }
//    }
//
//    double M = 0;
//    for (int i = 0; i < n; i++) {
//        for (int j = 0; j < n; j++) {
//            if (i < j) {
//                M += tau[i][j];
//            }
//        }
//    }
//    std::cout << "M value: " << M << std::endl;
//
//    IloBoolVarArray R(env, K + 1);
//    for (int k = 1; k < K; k++) {
//        R[k].setName(("R_" + std::to_string(k)).c_str());
//    }
//    // crossing constraint?
//    for (int k = 1; k < K; k++) {
//        IloExpr expr(env);
//        for (int k_p = k + 1; k_p <= K; k_p++) {
//            expr += z[k][k_p];
//        }
//        model.add(R[k] == expr).setName(("C14_" + std::to_string(k)).c_str());
//    }
//
//    IloArray<IloNumVarArray> wait(env, K);
//    for (int k : start_stage) {
//        wait[k] = IloNumVarArray(env, K + 1);
//        for (int kp = k + 1; kp <= K; kp++) {
//            wait[k][kp] = IloNumVar(env, 0, IloInfinity, ILOFLOAT);
//        }
//    }
//    ////-----------------------------------------------------------------------------------------------
//    // WARMSTART CONFIG: using original TSP solution.
//    // Obtained with MTZ formulation.
//    // On variable X.
//    if (cfg.use_tsp_for_warmstart) {
//        auto tsp_solution = TSP_MTZ(tau);
//        std::cout << tsp_solution.getSize() << std::endl;
//        IloNumVarArray X_warm_var(env);
//        IloNumArray X_warm_val(env);
//        for (int k = 1; k <= K - 1; k++) {
//            for (int i = 0; i <= D; i++) {
//                X_warm_var.add(X[k][i]);
//                if (tsp_solution[k - 1] == i) {
//                    X_warm_val.add(true);
//                    std::cout << "Warmstart X[" << k << "][" << i << "] == 1" << std::endl;
//                }
//                else {
//                    X_warm_val.add(false);
//                }
//            }
//        }
//        cplex.addMIPStart(X_warm_var, X_warm_val);
//        X_warm_var.end();
//        X_warm_val.end();
//    }
//    if (cfg.use_fstsp_for_warmstart) {}
//
//    // Constraints definition
//    // Numbering based on Overleaf.////////
//    // Constraint 0 - forcing constraint
//
//    // Preset constraints with config
//    if (!cfg.allow_revisit) {
//        // 1-visit constraint
//        for (int i : V) {
//            IloExpr sum_visit_i(env);
//            for (int k = 1; k <= K; k++) {
//                sum_visit_i += X[k][i];
//            }
//            model.add(sum_visit_i <= 1);
//        }
//    }
//    for (int h : C) {
//        IloExpr sumX(env);
//        for (int k = 2; k < K; k++) {
//            sumX += X[k][h];
//        }
//        if (cfg.allow_revisit) {
//            model.add(sumX + phi[h] >= 1);
//        }
//        else {
//            model.add(sumX + phi[h] == 1);
//        }
//    }
//    for (int k : start_stage) {
//        for (int i : c_s) {
//            for (int j : c_t) {
//                if (i != j) {
//                    model.add(y[k][i][j] <= R[k]);
//                }
//            }
//        }
//        IloExpr sum_zl(env), sum_zr(env);
//        for (int i : c_s) {
//            sum_zl += l[k][i];
//        }
//        for (int j : c_t) {
//            sum_zr += r[k][j];
//        }
//        model.add(sum_zl == R[k]);
//        model.add(sum_zr == R[k]);
//        for (int h : instance->c_prime) {
//            model.add(s[k][h] <= R[k]);
//        }
//    }
//
//    // Constraint 2
//    // drone bay theo mỗi cạnh $(i,j)$ ko quá 1 lần
//    for (int i : c_s) {
//        for (int j : c_t) {
//            if (i != j) {
//                IloExpr sum_xd_i_j(env);
//                for (int k : start_stage) {
//                    sum_xd_i_j += y[k][i][j];
//                }
//                model.add(yd[i][j] == sum_xd_i_j);
//                model.add(sum_xd_i_j <= 1);
//            }
//        }
//    }
//
//    // Constraint 7
//    // Ràng buộc thời gian bay của drone.
//    for (int k : start_stage) {
//        IloExpr sum_drone_movement(env);
//        for (int i : c_s) {
//            for (int j : c_t) {
//                if (i != j) {
//                    sum_drone_movement += y[k][i][j] * tau_prime[i][j];
//                }
//            }
//        }
//        model.add(sum_drone_movement <= (cfg.dtl - cfg.sr));
//    }
//
//    // Constraint 8
//    // Ràng buộc về số cạnh di chuyển trong 1 sortie của drone: <= L+1. L là số khách hàng tối đa trong 1 sortie.
//    for (int k = 1; k < K; k++) {
//        IloExpr sum_y(env);
//        for (int i : c_s) {
//            for (int j : c_t) {
//                if (i != j) {
//                    sum_y += y[k][i][j];
//                }
//            }
//        }
//        model.add(2 * R[k] <= sum_y);
//        model.add(sum_y <= (cfg.L + 1) * R[k]);
//    }
//
//    // Constraint 12: khách hàng j được phục vụ duy nhất 1 lần bởi drone.
//    for (int h : instance->c_prime) {
//        IloExpr sum_zs(env);
//        for (int k : start_stage) {
//            sum_zs += s[k][h];
//        }
//        model.add(phi[h] == sum_zs);
//    }
//
//    for (int k : start_stage) {
//        IloExpr sum_served(env);
//        for (int h : instance->c_prime) {
//            sum_served += s[k][h];
//        }
//        model.add(sum_served <= cfg.L);
//    }
//
//    // Constraint 13
//    // Cân bằng khách hàng h.
//    for (int k : start_stage) {
//        for (int h : C) {
//            IloExpr sum_out_h(env), sum_in_h(env);
//            for (int i : c_s) {
//                if (i != h) {
//                    sum_in_h += y[k][i][h];
//                }
//            }
//            for (int j : c_t) {
//                if (j != h) {
//                    sum_out_h += y[k][h][j];
//                }
//            }
//            model.add(2 * s[k][h] + l[k][h] + r[k][h] == sum_in_h + sum_out_h);
//        }
//    }
//
//    // Khách hàng h: chỉ có thể được phục vụ, hoặc là điểm thả, hoặc là điểm bắt.
//    for (int h : C) {
//        for (int k : start_stage) {
//            if (!cfg.allow_loop) {
//                model.add(s[k][h] + l[k][h] + r[k][h] <= 1);
//            }
//            else {
//                model.add(s[k][h] + l[k][h] <= 1);
//                model.add(s[k][h] + r[k][h] <= 1);
//            }
//        }
//    }
//
//
//    // Constraint 14
//    for (int k : start_stage) {
//        for (int i : c_s) {
//            model.add(l[k][i] <= X[k][i]);
//        }
//        for (int j : c_t) {
//            for (int kp = k + 1; kp <= K; kp++) {
//                model.add(r[k][j] <= X[kp][j] + 1 - z[k][kp]);
//            }
//        }
//    }
//
//    // Constraint 19
//    if (!cfg.allow_loop) {
//        for (int i : C) {
//            for (int j : C) {
//                if (i != j) {
//                    model.add(yd[i][j] + yd[j][i] <= 1);
//                }
//            }
//        }
//    }
//
//
//    for (int k = 1; k < K; k++) {
//        for (int i = 0; i < D; i++) {
//            IloExpr sum(env);
//            for (int j = 1; j <= D; j++) {
//                sum += x[k][i][j];
//            }
//            model.add(X[k][i] == sum).setName(("C11_" + std::to_string(k) + "_" + std::to_string(i)).c_str());
//        }
//    }
//
//    for (int k = 2; k <= K; k++) {
//        for (int i = 1; i <= D; i++) {
//            IloExpr sum(env);
//            for (int j = 0; j < D; j++) {
//                sum += x[k - 1][j][i];
//            }
//            // arcs entering i at stage k.
//            model.add(X[k][i] == sum).setName(("C12_" + std::to_string(k) + "_" + std::to_string(i)).c_str());
//        }
//    }
//
//    // C2: truck must depart from the depot at stage 1.
//    IloExpr C2(env);
//    for (int i = 1; i <= D; i++) {
//        C2 += x[1][O][i];
//    }
//
//    IloConstraint c2(C2 == 1);
//    model.add(c2).setName("Must start from source");
//
//    // C3: Terminal depot must be arrived only once.
//    IloExpr C3(env);
//    for (int k = 2; k <= K; k++) {
//        C3 += X[k][D];
//    }
//    model.add(C3 == 1).setName("Must visit terminal depot once");
//
//    // At most ONE node each stage.
//    for (int k = 1; k <= K; k++) {
//        IloExpr sum(env);
//        for (int i = 0; i <= D; i++)
//            sum += X[k][i];
//        model.add(sum <= 1).setName(("C13_at_most_one_customer_at_stage_" + std::to_string(k)).c_str());
//    }
//
//    // modified C7
//    for (int k = 1; k <= K - 1; k++) {
//        for (int k_p = k + 1; k_p <= K; k_p++) {
//            for (int stage_l = k + 1; stage_l < k_p; stage_l++) {
//                // tranh drone bay cac doan giao nhau.
//                if (k < stage_l) {
//                    model.add(z[k][k_p] + R[stage_l] <= 1).setName(("C7m_" + std::to_string(k)
//                            + "_" + std::to_string(k_p) + "_" + std::to_string(stage_l))
//                        .c_str());
//                }
//            }
//        }
//    }
//
//    // modified C7p - we can select at most one segment that contains the point l
//    for (int stage_l = 2; stage_l < K; stage_l++) {
//        IloExpr expr(env);
//
//        for (int k = 1; k < stage_l; k++)
//            for (int k_p = stage_l; k_p <= K; k_p++)
//                expr += z[k][k_p];
//        model.add(expr <= 1).setName(("C7mm_" + std::to_string(stage_l)).c_str());
//    }
//
//
//    for (int k = 1; k <= K - 1; k++) {
//        for (int k_p = k + 1; k_p <= K; k_p++) {
//            if (k < k_p) {
//                IloExpr truck_length(env);
//                for (int start = k; start < k_p; start++) {
//                    for (int i : c_s) {
//                        for (int j : c_t) {
//                            truck_length += x[start][i][j] * tau[i][j];
//                        }
//                    }
//                }
//                model.add(truck_length <= z[k][k_p] * (cfg.dtl - cfg.sr) + (1 - z[k][k_p]) * M);
//            }
//        }
//    }
//
//    for (int k : start_stage) {
//        IloExpr drone_move(env);
//        for (int i : c_s) {
//            for (int j : c_t) {
//                if (i != j) {
//                    drone_move += y[k][i][j] * tau_prime[i][j];
//                }
//            }
//        }
//        for (int kp = k + 1; kp <= K; kp++) {
//            IloExpr truck_move(env);
//            for (int start = k; start < kp; start++) {
//                for (int i : c_s) {
//                    for (int j : c_t) {
//                        if (i != j) {
//                            truck_move += x[start][i][j] * tau[i][j];
//                        }
//                    }
//                }
//            }
//            model.add(wait[k][kp] >= drone_move - truck_move - M * (1 - z[k][kp]));
//        }
//    }
//
//    // Additional
//    for (int i : c_s) {
//        IloExpr sum_d_out(env);
//        IloExpr sum_t_at(env);
//        for (int k : start_stage) {
//            sum_t_at += X[k][i];
//        }
//        for (int j : c_t) {
//            if (i != j) {
//                sum_d_out += yd[i][j];
//            }
//        }
//        // model.add(sum_d_out <= sum_t_at);
//    }
//
//    for (int j : c_t) {
//        IloExpr sum_d_in(env);
//        IloExpr sum_t_at(env);
//        for (int k : end_stage) {
//            sum_t_at += X[k][j];
//        }
//        for (int i : c_s) {
//            if (i != j) {
//                sum_d_in += yd[i][j];
//            }
//        }
//        // model.add(sum_d_in <= sum_t_at);
//    }
//    for (int k : start_stage) {
//        for (int i : c_s) {
//            IloExpr zl_s(env);
//            for (int j : c_t) {
//                if (i != j) {
//                    zl_s += y[k][i][j];
//                }
//            }
//            model.add(l[k][i] <= zl_s);
//        }
//        for (int j : c_t) {
//            IloExpr zr_s(env);
//            for (int i : c_s) {
//                if (i != j) {
//                    zr_s += y[k][i][j];
//                }
//            }
//            model.add(r[k][j] <= zr_s);
//        }
//    }
//
//    for (int k : start_stage) {
//        for (int i : c_s) {
//            for (int j : c_t) {
//                if (i != j) {
//                    // model.add(y[k][i][j] + zl[k][i] + zr[k][j] <= 2);
//                }
//            }
//        }
//    }
//    IloArray<IloNumVarArray> d(env, K);
//    for (int k : start_stage) {
//        d[k] = IloNumVarArray(env, D + 1);
//        for (int i : V) {
//            d[k][i] = IloNumVar(env, 0, IloInfinity, ILOFLOAT);
//        }
//    }
//    // model.add(d[1][0] == 0);
//
//    for (int k : start_stage) {
//        for (int i : c_s) {
//            for (int j : c_t) {
//                if (i != j) {
//                    if (!cfg.allow_loop) {
//                        model.add(d[k][j] >= d[k][i] + 0.01 - M * (1 - y[k][i][j]));
//                    }
//                    else {
//                        if (exist(c_s, j)) {
//                            model.add(d[k][j] >= d[k][i] + tau_prime[i][j] - M * l[k][j] - M * (1 - y[k][i][j]));
//                        }
//                    }
//                }
//            }
//        }
//    }
//
//    IloExpr objective(env);
//    for (int k : start_stage) {
//        for (int i : c_s) {
//            for (int j : c_t) {
//                objective += x[k][i][j] * tau[i][j];
//            }
//        }
//    }
//
//    for (int k : start_stage) {
//        objective += (cfg.sl + cfg.sr) * R[k];
//    }
//
//    objective -= R[1] * cfg.sl;
//    for (int k : start_stage) {
//        for (int kp = k + 1; kp <= K; kp++) {
//            objective += wait[k][kp];
//        }
//    }
//
//    model.add(IloMinimize(env, objective));
//    cplex.exportModel("rv-mvd.lp");
//    auto startTime = std::chrono::high_resolution_clock::now();
//    if (cfg.use_cutting_plane) {
//        // while (true) {
//        //     if (!cplex.solve()) {
//        //         std::cout << "Unable to optimize model!" << std::endl;
//        //     }
//        //     for (int k = 1; k < K; k++) {
//        //         for (int kp = k + 1; kp <= K; kp++) {
//        //             for (int i : c_s) {
//        //                 for (int j : c_t) {
//        //                     if (cplex.getValue(Z[k][kp][i][j]) == 1) {
//        //                         std::cout << "check z: " << k << " " << kp << " " << i << " " << j << std::endl;
//        //                         std::cout << "-------------------------------------------------" << std::endl;
//        //                         std::cout << "Start at stage " << k << " at node " << i << std::endl;
//        //                         std::cout << "End at stage " << kp << " at node " << j << std::endl;
//        //                         int start_node = i, end_node = j;
//        //                         // Route in this sortie
//        //                         std::vector<std::vector<int>> sortie_tour(D + 1);
//        //                         for (int index = 0; index < D + 1; index++) {
//        //                             sortie_tour[index].resize(D + 1);
//        //                         }
//        //                         for (int start : c_s) {
//        //                             for (int end : c_t) {
//        //                                 if (start != end) {
//        //                                     if (cplex.getValue(y[k][kp][start][end]) == 1) {
//        //                                         sortie_tour[start][end] = 1;
//        //                                     }
//        //                                 }
//        //                             }
//        //                         }
//        //                         auto cycles = find_cycles(sortie_tour);
//        //                         if (cycles.size() == 0) {
//        //                             goto LABEL_DONE;
//        //                         }
//        //                         std::cout << "Number of cycles: " << cycles.size() << std::endl;
//        //                         for (auto& cyc : cycles) {
//        //                             std::cout << "Printing cycle:";
//        //                             setPrint(cyc);
//        //                             IloExpr sum(env);
//        //                             for (int c_i : cyc) {
//        //                                 for (int c_j : cyc) {
//        //                                     sum += y[k][kp][c_i][c_j];
//        //                                 }
//        //                             }
//        //                             int cycle_size = cyc.size();
//        //                             model.add(sum <= cycle_size - 1);
//        //                         }
//        //
//        //
//        //                         std::cout << "-------------------------------------------------" << std::endl;
//        //                     }
//        //                 }
//        //             }
//        //         }
//        //     }
//        // }
//    }
//    else {
//        cplex.solve();
//        if (cfg.use_cplex_conflict_refiner) {
//            // A list of constraints to be considered by the conflict refiner.
//            IloConstraintArray constraints(env);
//
//            // Loop over all objects in the model and gather constraints.
//            for (IloModel::Iterator it(model); it.ok(); ++it) {
//                IloExtractable ext = *it;
//                if (ext.isVariable()) {
//                    IloNumVar v = ext.asVariable();
//                    // Add variable bounds to the constraints array.
//                    constraints.add(IloBound(v, IloBound::Lower));
//                    constraints.add(IloBound(v, IloBound::Upper));
//                }
//                else if (ext.isConstraint()) {
//                    IloConstraint c = ext.asConstraint();
//                    constraints.add(c);
//                }
//            }
//
//            // Define preferences for the constraints. Here, we give all
//            // constraints a preference of 1.0, so they will be treated
//            // equally.
//            IloNumArray prefs(env, constraints.getSize());
//            for (int i = 0; i < prefs.getSize(); ++i)
//                prefs[i] = 1.0;
//
//            // Run the conflict refiner. As opposed to letting the conflict
//            // refiner run to completion (as is done here), the user can set
//            // a resource limit (e.g., a time limit, an iteration limit, or
//            // node limit) and still potentially get a "possible" conflict.
//            if (cplex.refineConflict(constraints, prefs)) {
//                // Display the solution status.
//                IloCplex::CplexStatus status = cplex.getCplexStatus();
//                std::cout << "Solution status = " << status << " (" <<
//                    static_cast<int>(status) << ")" << std::endl;
//
//                // Get the conflict status for the constraints that were specified.
//                IloCplex::ConflictStatusArray conflict = cplex.getConflict(constraints);
//
//                // Print constraints that participate in the conflict.
//                std::cout << "Conflict:" << std::endl;
//
//                for (int i = 0; i < constraints.getSize(); ++i) {
//                    if (conflict[i] == IloCplex::ConflictMember ||
//                        conflict[i] == IloCplex::ConflictPossibleMember) {
//                        std::cout << "  " << constraints[i] << std::endl;
//                    }
//                }
//
//                std::cout << std::endl;
//
//                // Write the identified conflict in the LP format.
//                const char* confFile = "iloconflictex1.lp";
//                std::cout << "Writing conflict file to '" << confFile << "'...." << std::endl;
//                cplex.writeConflict(confFile);
//
//                // Display the entire conflict subproblem.
//                std::string line;
//                std::ifstream file(confFile);
//                if (file.is_open()) {
//                    while (getline(file, line)) {
//                        std::cout << line << std::endl;
//                    }
//                }
//                file.close();
//            }
//            else {
//                std::cout << "A conflict was not identified." << std::endl;
//                std::cout << "Exiting...." << std::endl;
//            }
//        }
//    }
//LABEL_DONE:
//    auto endTime = std::chrono::high_resolution_clock::now();
//    // Calculate the elapsed time
//    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
//    double truck_l = 0;
//    std::cout << "Feasible solution found!" << std::endl;
//    std::cout << "Truck nodes:" << std::endl;
//    for (int k = 1; k <= K; k++) {
//        for (int i = 0; i <= D; i++) {
//            auto X_val = cplex.getValue(X[k][i]);
//            if (abs(X_val - 1) < 1e-6) {
//                std::cout << "Truck at node " << i << " at stage " << k << std::endl;
//            }
//        }
//    }
//
//    std::cout << std::endl << "Truck arcs:" << std::endl;
//    std::map<int, std::pair<int, int>> map_stage_truck_arc;
//    for (int k = 1; k <= K_arc; k++) {
//        for (int i = 0; i < D; i++) {
//            for (int j = 1; j <= D; j++) {
//                if (i != j) {
//                    auto X_val = cplex.getValue(x[k][i][j]);
//                    if (abs(X_val - 1) < 1e-6) {
//                        std::cout << "Arc " << k << " connecting " << i << " and " << j
//                            << " with cost " << tau[i][j] << " " << std::endl;
//                        truck_l += tau[i][j];
//                        map_stage_truck_arc[k] = std::make_pair(i, j);
//                        break;
//                    }
//                }
//            }
//        }
//    }
//    std::cout << "Truck route length: " << truck_l << std::endl;
//    int phi_check = 0;
//
//    for (int h : instance->c_prime) {
//        if (cplex.getValue(phi[h]) == 1) {
//            std::cout << "Customer " << h << " was served by drone!" << std::endl;
//            phi_check += 1;
//        }
//    }
//
//    for (int k : start_stage) {
//        for (int h : instance->c_prime) {
//            if (cplex.getValue(s[k][h]) == 1) {
//                std::cout << "served customer " << h << " with st stage " << k << std::endl;
//                for (int i : c_s) {
//                    if (cplex.getValue(l[k][i]) == 1) {
//                        std::cout << "start from stage " << k << " at node " << i << std::endl;
//                    }
//                }
//                for (int j : c_t) {
//                    if (cplex.getValue(r[k][j]) == 1) {
//                        std::cout << "end at node " << j << std::endl;
//                    }
//                }
//                for (int kp = k + 1; kp <= K; kp++) {
//                    if (cplex.getValue(z[k][kp]) == 1) {
//                        std::cout << "end at stage " << kp << std::endl;
//                    }
//                }
//            }
//        }
//    }
//
//
//    std::cout << "Printing wait:" << std::endl;
//    for (int k : start_stage) {
//        for (int kp = k + 1; kp <= K; kp++) {
//            auto w = cplex.getValue(wait[k][kp]);
//            if (w > 0) {
//                std::cout << "wait[" << k << "][" << kp << "]== " << w << std::endl;
//            }
//        }
//    }
//
//    std::cout << "Printing z(k, k') and drone travel time: " << std::endl;
//    for (int k : start_stage) {
//        for (int kp = k + 1; kp <= K; kp++) {
//            if (cplex.getValue(z[k][kp]) == 1) {
//                std::cout << "---------------------------------------------------------------" << std::endl;
//                std::cout << "Sortie starts at stage " << k << " and ends at stage " << kp << std::endl;
//                for (int i : c_s) {
//                    if (abs(cplex.getValue(l[k][i]) - 1) < 1e-6) {
//                        std::cout << "Start node: " << i << std::endl;
//                    }
//                }
//                for (int j : c_t) {
//                    if (abs(cplex.getValue(r[k][j]) - 1) < 1e-6) {
//                        std::cout << "End node: " << j << std::endl;
//                    }
//                }
//                std::cout << "Customer served in this sortie:";
//                for (int h : instance->c_prime) {
//                    if (abs(cplex.getValue(s[k][h]) - 1) < 1e-6) {
//                        std::cout << h << " ";
//                    }
//                }
//                std::cout << std::endl;
//                std::cout << "Drone arcs during this sortie:" << std::endl;
//                double d_time = 0;
//                for (int i : c_s) {
//                    for (int j : c_t) {
//                        if (i != j) {
//                            if (abs(cplex.getValue(y[k][i][j]) - 1) < 1e-6) {
//                                std::cout << i << "-->" << j << ", l = " << tau_prime[i][j] << std::endl;
//                            }
//                            d_time += cplex.getValue(y[k][i][j]) * tau_prime[i][j];
//                        }
//                    }
//                }
//                std::cout << "Truck arcs during this sortie:" << std::endl;
//                double t_time = 0;
//                for (int start = k; start < kp; start++) {
//                    for (int i : c_s) {
//                        for (int j : c_t) {
//                            if (i != j && abs(cplex.getValue(x[start][i][j]) - 1) < 1e-6) {
//                                t_time += tau[i][j];
//                                std::cout << i << "---" << j << ", l = " << tau[i][j] << std::endl;
//                            }
//                        }
//                    }
//                }
//                std::cout << "Drone travel time during sortie: " << d_time << std::endl;
//                std::cout << "Truck travel time during sortie: " << t_time << std::endl;
//                std::cout << "Wait time for this sortie: " << cplex.getValue(wait[k][kp]) << std::endl;
//                std::cout << "---------------------------------------------------------------" << std::endl;
//            }
//        }
//    }
//
//    std::cout << "Printing zl:" << std::endl;
//    for (int k : start_stage) {
//        for (int i : c_s) {
//            if (cplex.getValue(l[k][i]) == 1) {
//                std::cout << "zl[" << k << "][" << i << "] = " << cplex.getValue(l[k][i]) << std::endl;
//            }
//        }
//    }
//    std::cout << "Printing zr:" << std::endl;
//    for (int k : start_stage) {
//        for (int j : c_t) {
//            if (cplex.getValue(r[k][j]) == 1) {
//                std::cout << "zr[" << k << "][" << j << "] = " << cplex.getValue(r[k][j]) << std::endl;
//            }
//        }
//    }
//    std::cout << "Printing zs:" << std::endl;
//    for (int k : start_stage) {
//        for (int h : instance->c_prime) {
//            if (cplex.getValue(s[k][h]) == 1) {
//                std::cout << "zs[" << k << "][" << h << "] = " << cplex.getValue(s[k][h]) << std::endl;
//            }
//        }
//    }
//
//    std::cout << "Printing x_d:" << std::endl;
//    for (int i : c_s) {
//        for (int j : c_t) {
//            if (i != j) {
//                if (cplex.getValue(yd[i][j]) == 1) {
//                    std::cout << "x_d[" << i << "][" << j << "] = 1" << std::endl;
//                }
//            }
//        }
//    }
//    //
//    // std::cout << "Sorties information: ";
//    // if (phi_check == 0) {
//    //     std::cout << "No sortie!" << std::endl;
//    // }
//    // else {
//    //     std::cout << phi_check << " customer(s) served by drone." << std::endl;
//    // }
//    // for (int k = 1; k < K; k++) {
//    //     for (int kp = k + 1; kp <= K; kp++) {
//    //         for (int i : c_s) {
//    //             for (int j : c_t) {
//    //                 if (abs(cplex.getValue(Z[k][kp][i][j]) - 1) < 1e-6) {
//    //                     std::cout << "check z: " << k << " " << kp << " " << i << " " << j << std::endl;
//    //                     std::cout << "-------------------------------------------------" << std::endl;
//    //                     std::cout << "Start at stage " << k << " at node " << i << std::endl;
//    //                     std::cout << "End at stage " << kp << " at node " << j << std::endl;
//    //                     int start_node = i, end_node = j;
//    //                     // Route in this sortie
//    //                     for (int start : c_s) {
//    //                         for (int end : c_t) {
//    //                             if (start != end) {
//    //                                 if (cplex.getValue(y[k][kp][start][end]) == 1) {
//    //                                     std::cout << start << "--->" << end << std::endl;
//    //                                 }
//    //                             }
//    //                         }
//    //                     }
//    //                     std::cout << "-------------------------------------------------" << std::endl;
//    //                 }
//    //             }
//    //         }
//    //     }
//    // }
//    //
//    // std::cout << "Drone path:" << std::endl;
//    // for (int i : c_s) {
//    //     for (int j : c_t) {
//    //         if (i != j) {
//    //             if (abs(cplex.getValue(x_d[i][j]) - 1) < 1e-6) {
//    //                 std::cout << i << " " << j << std::endl;
//    //             }
//    //         }
//    //     }
//    // }
//    //
//    // std::cout << "---------------------------------------------------------------" << std::endl;
//    // std::cout << "Drone path based on y_k_kp_i_j: " << std::endl;
//    // for (int k : start_stage) {
//    //     for (int kp = k + 1; kp <= K; kp++) {
//    //         for (int i : c_s) {
//    //             for (int j : c_t) {
//    //                 if (i != j) {
//    //                     if (abs(cplex.getValue(y[k][kp][i][j]) - 1) < 1e-6) {
//    //                         std::cout << "y[" << k << "][" << kp << "][" << i << "][" << j << "] = 1" <<
//    //                             std::endl;
//    //                     }
//    //                 }
//    //             }
//    //         }
//    //     }
//    // }
//    // std::cout << "---------------------------------------------------------------" << std::endl;
//    // std::cout << "Print zp:" << std::endl;
//    // for (int j : instance->c_prime) {
//    //     for (int k : start_stage) {
//    //         for (int kp = k + 1; kp <= K; kp++) {
//    //             if (abs(cplex.getValue(zp[k][kp][j]) - 1) < 1e-6) {
//    //                 std::cout << "zp[" << k << "][" << kp << "][" << j << "] = 1" << std::endl;
//    //             }
//    //         }
//    //     }
//    // }
//    //
//    // std::cout << "---------------------------------------------------------------" << std::endl;
//    // std::cout << "Print z:" << std::endl;
//    // for (int k : start_stage) {
//    //     for (int kp = k + 1; kp <= K; kp++) {
//    //         if (abs(cplex.getValue(z[k][kp]) - 1) < 1e-6) {
//    //             std::cout << "z[" << k << "][" << kp << "] = 1" << std::endl;
//    //         }
//    //     }
//    // }
//    std::cout << "Done!" << std::endl;
//    // std::cout << "-------------------------Re-calculated objective-----------------------" << std::endl;
//    // std::cout << obj << std::endl;
//    // std::cout << "------------------------------------------------------------------------" << std::endl;
//
//    double c = cplex.getObjValue();
//    double gap = cplex.getMIPRelativeGap();
//    std::cout << "Solve time: " << duration.count() / 1000.0 << std::endl;
//    cplex.end();
//    model.end();
//    env.end();
//
//
//    std::cout << "OBJECTIVE VALUE: " << c << "." << std::endl;
//    return Result{c, duration.count() / 1000.0, gap};
//}
//
//Result FSTSPSolver::RV_FSTSP_MVD_4_index_t_D(Config& cfg) const {
//    try {
//        auto tau = instance->tau;
//        auto tau_prime = instance->tau_prime;
//
//        auto n = instance->num_node;
//        std::vector<int> C;
//        std::vector<int> V;
//        std::vector<int> c_s;
//        std::vector<int> c_t;
//        for (int i = 0; i < n + 1; i++) {
//            if (i != 0 && i != n) {
//                C.push_back(i);
//            }
//            if (i != 0) {
//                c_t.push_back(i);
//            }
//            if (i != n) {
//                c_s.push_back(i);
//            }
//            V.push_back(i);
//        }
//
//        IloEnv env;
//        IloModel model(env);
//        IloCplex cplex(model);
//
//        auto s = 0;
//        auto t = n;
//
//        auto O = 0;
//        auto D = n;
//        int K = n + 1;
//        auto K_arc = K - 1;
//        std::vector<int> start_stage, end_stage;
//        for (int k = 1; k <= K; k++) {
//            if (k < K) {
//                start_stage.push_back(k);
//            }
//            if (k > 1) {
//                end_stage.push_back(k);
//            }
//        }
//        // cplex.setParam(IloCplex::Param::MIP::Tolerances::Integrality, 0);
//        cplex.setParam(IloCplex::Param::Threads, cfg.num_thread);
//        cplex.setParam(IloCplex::Param::TimeLimit, 3600.0);
//        // Variable declaration
//        // X^i_k (binary variable) và nhận giá trị một tương ứng với đỉnh thứ k của
//        //đường đi của vehicle là i; k \in 1..n;
//        IloArray<IloBoolVarArray> X(env, K + 1);
//        for (int k = 1; k <= K; k++) {
//            X[k] = IloBoolVarArray(env, D + 1);
//            for (int i = 0; i <= D; i++) {
//                X[k][i] = IloBoolVar(env);
//                model.add(X[k][i]);
//                auto v_name = "X_" + std::to_string(k) + "_" + std::to_string(i);
//                X[k][i].setName(v_name.c_str());
//            }
//            if (k > 1) {
//                model.add(X[k][0] == 0);
//            }
//        }
//
//        model.add(X[1][0] == 1).setName("First stage must be source depot");
//
//        // x^k_(ij) (binary variable) và nhận giá trị một nếu Xk
//        // mô ta cạnh nối 2 đỉnh liên tiếp trên đường đi.
//        IloArray<IloArray<IloBoolVarArray>> x(env, K_arc + 1);
//        for (int k = 1; k <= K_arc; k++) {
//            x[k] = IloArray<IloBoolVarArray>(env, D);
//            for (int i = 0; i < D; i++) {
//                x[k][i] = IloBoolVarArray(env, D + 1);
//                for (int j = 1; j <= D; j++) {
//                    x[k][i][j] = IloBoolVar(env);
//                    auto v_name = "x_t" + std::to_string(k) + "_" + std::to_string(i) + "_" + std::to_string(j);
//                    x[k][i][j].setName(v_name.c_str());
//                    if (!cfg.allow_loop) {
//                        if (i == j) {
//                            model.add(x[k][i][j] == 0);
//                        }
//                    }
//                }
//            }
//        }
//
//        //// phi^h equals to 1 if customer h is served by the drone
//        IloBoolVarArray phi(env, n);
//        for (int h : C) {
//            phi[h] = IloBoolVar(env);
//            auto v_name = "phi_" + std::to_string(h);
//            phi[h].setName(v_name.c_str());
//            if (exist(instance->heavy, h)) {
//                model.add(phi[h] == 0);
//            }
//        }
//
//
//        // arrival\departure variables a and d.
//        IloNumVarArray a(env, K + 1);
//        IloNumVarArray d(env, K + 1);
//        for (int k = 1; k <= K; k++) {
//            a[k] = IloNumVar(env, 0, IloInfinity, ILOFLOAT);
//            auto v_name = "a_" + std::to_string(k);
//            a[k].setName(v_name.c_str());
//            d[k] = IloNumVar(env, 0, IloInfinity, ILOFLOAT);
//            v_name = "d_" + std::to_string(k);
//            d[k].setName(v_name.c_str());
//            model.add(d[k] >= a[k]).setName(("C13_" + std::to_string(k)).c_str());
//        }
//
//        model.add(a[1] == 0).setName("arrival to depot at time 0");
//        model.add(d[1] == 0).setName("depart from depot at time 0");
//
//        // variable Z(k,k',i,j)
//        // drone bay từ (k, i), hạ cánh tại (k', j)
//        IloArray<IloArray<IloArray<IloBoolVarArray>>> Z(env, K);
//        for (int k = 1; k < K; k++) {
//            Z[k] = IloArray<IloArray<IloBoolVarArray>>(env, K + 1);
//            for (int kp = k + 1; kp <= K; kp++) {
//                Z[k][kp] = IloArray<IloBoolVarArray>(env, D);
//                for (int i : c_s) {
//                    Z[k][kp][i] = IloBoolVarArray(env, D + 1);
//                    for (int j : c_t) {
//                        Z[k][kp][i][j] = IloBoolVar(env);
//                        std::string vname = "Z_" + std::to_string(k) + "_" + std::to_string(kp) + "_" +
//                            std::to_string(i) +
//                            "_" + std::to_string(j);
//                        Z[k][kp][i][j].setName(vname.c_str());
//                        if (!cfg.allow_loop && i == j) {
//                            model.add(Z[k][kp][i][j] == 0);
//                        }
//                    }
//                }
//                model.add(Z[k][kp][s][D] == 0);
//            }
//        }
//
//        // biến X_drone cho đường đi của drone: đi từ i đến j trong sortie từ stage (k -> k')
//        IloArray<IloArray<IloArray<IloBoolVarArray>>> X_drone(env, K);
//        for (int k = 1; k < K; k++) {
//            X_drone[k] = IloArray<IloArray<IloBoolVarArray>>(env, K + 1);
//            for (int kp = k + 1; kp <= K; kp++) {
//                X_drone[k][kp] = IloArray<IloBoolVarArray>(env, D);
//                for (int i : c_s) {
//                    X_drone[k][kp][i] = IloBoolVarArray(env, D + 1);
//                    for (int j : c_t) {
//                        X_drone[k][kp][i][j] = IloBoolVar(env);
//                        std::string vname = "X_drone_" + std::to_string(k) + "_" + std::to_string(kp) + "_" +
//                            std::to_string(i) +
//                            "_" + std::to_string(j);
//                        X_drone[k][kp][i][j].setName(vname.c_str());
//                        if (i == j) {
//                            model.add(X_drone[k][kp][i][j] == 0);
//                        }
//                    }
//                }
//                model.add(X_drone[k][kp][s][D] == 0);
//            }
//        }
//
//        IloArray<IloBoolVarArray> x_d(env, D);
//        for (int i : c_s) {
//            x_d[i] = IloBoolVarArray(env, D + 1);
//            for (int j : c_t) {
//                if (i != j) {
//                    std::string vname = "x_d_" + std::to_string(i) + "_" + std::to_string(j);
//                    x_d[i][j] = IloBoolVar(env);
//                    x_d[i][j].setName(vname.c_str());
//                }
//            }
//        }
//        // aux var z
//        //// aux var z_{k, k_p}: sortie launch from k and rendezvous at k_p.
//        IloArray<IloBoolVarArray> z(env, K);
//        for (int k = 1; k < K; k++) {
//            z[k] = IloBoolVarArray(env, K + 1);
//            for (int k_p = k + 1; k_p <= K; k_p++) {
//                z[k][k_p] = IloBoolVar(env);
//                auto v_name = "z_" + std::to_string(k) + "_" + std::to_string(k_p);
//                z[k][k_p].setName(v_name.c_str());
//            }
//        }
//
//        // Variable 11
//        // biến nhị phân mô tả khách hàng p được phục vụ bởi drone bay từ
//        // stage k tới stage k'
//        IloArray<IloArray<IloBoolVarArray>> zp(env, K);
//        for (int k : start_stage) {
//            zp[k] = IloArray<IloBoolVarArray>(env, K + 1);
//            for (int kp = k + 1; kp <= K; kp++) {
//                zp[k][kp] = IloBoolVarArray(env, D);
//                for (int p : instance->c_prime) {
//                    zp[k][kp][p] = IloBoolVar(env);
//                    std::string vname = "zp_" + std::to_string(k) + "_" + std::to_string(kp) + "_" + std::to_string(p);
//                    zp[k][kp][p].setName(vname.c_str());
//                }
//            }
//        }
//
//        // Variable t^{D}_{j}
//        IloNumVarArray td(env, D);
//        for (int i : c_s) {
//            td[i] = IloNumVar(env, 0, IloInfinity, ILOFLOAT);
//            if (exist(instance->heavy, i)) {
//                model.add(td[i] == 0);
//            }
//        }
//        model.add(td[0] == 0);
//        double M = 0;
//        for (int i = 0; i < n; i++) {
//            for (int j = 0; j < n; j++) {
//                if (i < j) {
//                    M += tau[i][j];
//                }
//            }
//        }
//        for (int i : c_s) {
//            for (int j : instance->c_prime) {
//                if (i != j) {
//                    model.add(td[j] + M * (1 - x_d[i][j]) >= td[i] + tau_prime[i][j]);
//                }
//            }
//        }
//        IloArray<IloBoolVarArray> d_out(env, K);
//        for (int k : start_stage) {
//            d_out[k] = IloBoolVarArray(env, D);
//            for (int i : c_s) {
//                d_out[k][i] = IloBoolVar(env);
//            }
//        }
//
//        // Constraint 22
//        for (int k : start_stage) {
//            for (int i : c_s) {
//                IloExpr sum_d_out(env);
//                for (int kp = k + 1; kp <= K; kp++) {
//                    for (int j : c_t) {
//                        sum_d_out += Z[k][kp][i][j];
//                    }
//                }
//                model.add(d_out[k][i] == sum_d_out);
//            }
//        }
//        IloArray<IloBoolVarArray> d_in(env, K + 1);
//        for (int kp : end_stage) {
//            d_in[kp] = IloBoolVarArray(env, D + 1);
//            for (int j : c_t) {
//                d_in[kp][j] = IloBoolVar(env);
//            }
//        }
//
//        // Constraint 24
//        for (int kp : end_stage) {
//            for (int j : c_t) {
//                IloExpr sum_d_in(env);
//                for (int k = 1; k < kp; k++) {
//                    for (int i : c_s) {
//                        sum_d_in += Z[k][kp][i][j];
//                    }
//                }
//                model.add(d_in[kp][j] == sum_d_in);
//            }
//        }
//
//        ////-----------------------------------------------------------------------------------------------
//        // WARMSTART CONFIG: using original TSP solution.
//        // Obtained with MTZ formulation.
//        // On variable X.
//        if (cfg.use_tsp_for_warmstart) {
//            auto tsp_solution = TSP_MTZ(tau);
//            std::cout << tsp_solution.getSize() << std::endl;
//            IloNumVarArray X_warm_var(env);
//            IloNumArray X_warm_val(env);
//            for (int k = 1; k <= K - 1; k++) {
//                for (int i = 0; i <= D; i++) {
//                    X_warm_var.add(X[k][i]);
//                    if (tsp_solution[k - 1] == i) {
//                        X_warm_val.add(true);
//                        std::cout << "Warmstart X[" << k << "][" << i << "] == 1" << std::endl;
//                    }
//                    else {
//                        X_warm_val.add(false);
//                    }
//                }
//            }
//            cplex.addMIPStart(X_warm_var, X_warm_val);
//            X_warm_var.end();
//            X_warm_val.end();
//        }
//        if (cfg.use_fstsp_for_warmstart) {}
//        ////-----------------------------------------------------------------------------------------------
//        // PREPROCESSING
//        ////-----------------------------------------------------------------------------------------------
//
//        //// Sortie maximum stage gap calculation.
//        /// Find maximum stage gap that a sortie can start and rendezvous.
//        /// For each stage gap in increasing order (k' - k), find the minimum tour length from stage k to k'.
//        /// Revisit(s) are also considered.
//        /// If the tour length > cfg.dtl - cfg.sr => there can be no sortie with stage gap greater or equal k' - k.
//        //     bool used_stage_gap = false;
//        //     int min_stage_gap = 0;
//        //     std::cout << "---------------------------------------------" << std::endl;
//        //     std::cout << "Calculate max stage gap for sortie.... " << std::endl;
//        //     for (int k = 1; k < K; k++) {
//        //         for (int kp = k + 1; kp <= K; kp++) {
//        //             if (kp > k + 6) {
//        //                 goto after_z_cons;
//        //             }
//        //             double smallest_tour = smallest_tour_length(kp - k, tau, V);
//        //             if (smallest_tour > cfg.dtl - cfg.sr) {
//        //                 std::cout << "Stage gap of " << kp - k << " with smallest length = " << smallest_tour <<
//        //                     " violated endurance constraint!" << std::endl;
//        //                 std::cout << "---------------------------------------------" << std::endl;
//        //                 min_stage_gap = kp - k;
//        //                 goto after_z_cons;
//        //             }
//        //         }
//        //     }
//        // after_z_cons:
//        //     if (min_stage_gap == 0) {
//        //         std::cout << "Stage gap calculation consumes too much memory. No constraint was added." << std::endl;
//        //         std::cout << "---------------------------------------------" << std::endl;
//        //     }
//        //     if (min_stage_gap != 0) {
//        //         used_stage_gap = true;
//        //         for (int k = 1; k < K; k++) {
//        //             for (int kp = k + min_stage_gap; kp <= K; kp++) {
//        //                 model.add(z[k][kp] == 0);
//        //                 std::cout << "Variable z[" << k << "][" << kp << "] was set to 0." << std::endl;
//        //                 for (int h : instance->c_prime) {
//        //                     model.add(zp[k][kp][h] == 0);
//        //
//        //                     std::string cname =
//        //                         "Can't serve customer " + std::to_string(h) + " start at stage " + std::to_string(k) +
//        //                         ", end at stage " + std::to_string(kp);
//        //                     // Looks like quicker lower bound improvements with this constraints.
//        //                 }
//        //             }
//        //         }
//        //     }
//
//        // Constraints definition
//        // Numbering based on Overleaf.////////
//
//        // Constraint 0 - forcing constraint
//        for (int k : start_stage) {
//            for (int kp = k + 1; kp <= K; kp++) {
//                for (int i : c_s) {
//                    for (int j : c_t) {
//                        if (i != j) {
//                            std::string cname = "C0_" + std::to_string(k) + "_" + std::to_string(kp) + "_" +
//                                std::to_string(i) + "_" + std::to_string(j);
//                            model.add(X_drone[k][kp][i][j] <= z[k][kp]).setName(cname.c_str());
//                        }
//                    }
//                }
//            }
//        }
//
//        // Constraint 1
//        // mỗi cặp stage, drone chỉ bay dc tối đa một lần.
//        for (int k : start_stage) {
//            for (int kp = k + 1; kp <= K; kp++) {
//                IloExpr sumZ(env);
//                for (int i : c_s) {
//                    for (int j : c_t) {
//                        sumZ += Z[k][kp][i][j];
//                    }
//                }
//                std::string cname = "C1_(k,k')_(" + std::to_string(k) + "," + std::to_string(kp) + ")";
//                model.add(z[k][kp] == sumZ).setName(cname.c_str());
//                model.add(sumZ <= 1);
//            }
//        }
//
//        // Constraint 2
//        // drone bay theo mỗi cạnh $(i,j)$ ko quá 1 lần
//        for (int i : c_s) {
//            for (int j : c_t) {
//                if (i != j) {
//                    IloExpr sum_xd_i_j(env);
//                    for (int k : start_stage) {
//                        for (int kp = k + 1; kp <= K; kp++) {
//                            sum_xd_i_j += X_drone[k][kp][i][j];
//                        }
//                    }
//                    model.add(x_d[i][j] == sum_xd_i_j);
//                    model.add(sum_xd_i_j <= 1);
//                }
//            }
//        }
//
//        // Constraint 2'
//        for (int i : c_s) {
//            for (int j : c_t) {
//                if (i != j) {
//                    IloExpr sum_x_k(env);
//                    for (int k = 1; k < K; k++) {
//                        sum_x_k += x[k][i][j];
//                    }
//                    model.add(sum_x_k <= K * (1 - x_d[i][j]));
//                }
//            }
//        }
//
//
//        // Constraint 3
//        // nếu có drone bay từ (k,i) và hạ cánh tại stage k' thì phải có một cạnh đi ra từ node i của drone
//        for (int k = 1; k < K; k++) {
//            for (int i : c_s) {
//                for (int kp = k + 1; kp <= K; kp++) {
//                    std::string cname = "C3_(k,k',i)_(" + std::to_string(k) + "," + std::to_string(kp) + "," +
//                        std::to_string(i) + ")";
//                    IloExpr sum_X_drone(env);
//                    for (int j : c_t) {
//                        if (i != j) {
//                            sum_X_drone += x_d[i][j];
//                        }
//                    }
//                    // model.add(sumZ == sum_X_drone).setName(cname.c_str());
//                    model.add(sum_X_drone <= 1);
//                }
//            }
//        }
//
//        // Constraint 4
//        // nếu có drone bay từ stage k và hạ cánh tại stage k', node j thì phải có một cạnh đi vào node j của drone.
//        for (int kp = 2; kp <= K; kp++) {
//            for (int j : c_t) {
//                for (int k = 1; k < kp; k++) {
//                    std::string cname = "C4_(k,k',j)_(" + std::to_string(k) + "," + std::to_string(kp) + "," +
//                        std::to_string(j) + ")";
//                    IloExpr sum_X_drone(env);
//                    for (int i : c_s) {
//                        if (i != j) {
//                            sum_X_drone += x_d[i][j];
//                        }
//                    }
//                    // model.add(sumZ == sum_X_drone).setName(cname.c_str());
//                    model.add(sum_X_drone <= 1);
//                }
//            }
//        }
//
//        // Constraint 6
//        // i in set C. Might need to change to set V to accomodate all situations.
//        // Cân bằng tại mỗi node i.
//        for (int i : V) {
//            for (int k = 1; k < K; k++) {
//                for (int kp = k + 1; kp <= K; kp++) {
//                    std::string cname = "C6_i_" + std::to_string(i);
//                    IloExpr sum1(env), sum2(env), sum3(env), sum4(env);
//                    for (int j : c_s) {
//                        if (j != i) {
//                            sum3 += X_drone[k][kp][j][i];
//                        }
//                        sum1 += Z[k][kp][j][i];
//                    }
//                    for (int j : c_t) {
//                        if (i != t) {
//                            sum4 += Z[k][kp][i][j];
//                            if (i != j) {
//                                sum2 += X_drone[k][kp][i][j];
//                            }
//                        }
//                    }
//                    model.add(sum1 + sum2 == sum3 + sum4).setName(cname.c_str());
//                }
//            }
//        }
//
//        // Constraint 7
//        // Ràng buộc thời gian bay của drone.
//        for (int k = 1; k < K; k++) {
//            for (int kp = k + 1; kp <= K; kp++) {
//                std::string cname = "C7_(k,k')_(" + std::to_string(k) + ", " + std::to_string(kp) + ")";
//                IloExpr sum_drone_movement(env);
//                for (int i : c_s) {
//                    for (int j : c_t) {
//                        if (i != j) {
//                            sum_drone_movement += X_drone[k][kp][i][j] * tau_prime[i][j];
//                        }
//                    }
//                }
//                model.add(sum_drone_movement <= (cfg.dtl - cfg.sr) * z[k][kp]).setName(cname.c_str());
//            }
//        }
//
//        // Constraint 8
//        // Ràng buộc về số cạnh di chuyển trong 1 sortie của drone: <= L+1. L là số khách hàng tối đa trong 1 sortie.
//        for (int k = 1; k < K; k++) {
//            for (int kp = k + 1; kp <= K; kp++) {
//                std::string cname = "C8_(k,k')_(" + std::to_string(k) + "," + std::to_string(kp) + ")";
//                IloExpr sum_X_drone(env);
//                for (int i : c_s) {
//                    for (int j : c_t) {
//                        if (i != j) {
//                            sum_X_drone += X_drone[k][kp][i][j];
//                        }
//                    }
//                }
//                model.add(2 * z[k][kp] <= sum_X_drone).setName(cname.c_str());
//                model.add(sum_X_drone <= (cfg.L + 1) * z[k][kp]);
//            }
//        }
//
//        // Constraint 10
//        // customer j chỉ được thăm bằng truck hoặc drone..
//        for (int j : C) {
//            std::string cname = "C10_j_" + std::to_string(j);
//            IloExpr sumX(env);
//            for (int k = 2; k < K; k++) {
//                sumX += X[k][j];
//            }
//            // model.add(sumX <= K * (1 - phi[j])).setName(cname.c_str());
//        }
//
//        // Constraint 12: khách hàng j được phục vụ duy nhất 1 lần bởi drone.
//        for (int j : instance->c_prime) {
//            IloExpr sum_zp(env);
//            for (int k : start_stage) {
//                for (int kp = k + 1; kp <= K; kp++) {
//                    sum_zp += zp[k][kp][j];
//                }
//            }
//            model.add(phi[j] == sum_zp);
//        }
//
//        // Constraint 13
//        // Cân bằng customer j.
//        for (int k : start_stage) {
//            for (int kp = k + 1; kp <= K; kp++) {
//                for (int h : instance->c_prime) {
//                    IloExpr leave_h(env), enter_h(env);
//                    IloExpr start_at_h_k_kp(env), end_at_h_k_kp(env);
//
//                    for (int i : c_s) {
//                        if (i != h) {
//                            enter_h += X_drone[k][kp][i][h];
//                            end_at_h_k_kp += Z[k][kp][i][h];
//                        }
//                    }
//                    for (int j : c_t) {
//                        if (j != h) {
//                            leave_h += X_drone[k][kp][h][j];
//                            start_at_h_k_kp += Z[k][kp][h][j];
//                        }
//                    }
//
//                    std::string cname = "C13_(k,k',j)_(" + std::to_string(k) + "," + std::to_string(kp) + "," +
//                        std::to_string(h) + ")";
//                    model.add(2 * zp[k][kp][h] + start_at_h_k_kp + end_at_h_k_kp == leave_h + enter_h).setName(
//                        cname.c_str());
//                }
//            }
//        }
//        // Constraint 16
//        for (int j : instance->c_prime) {
//            for (int k : start_stage) {
//                for (int kp = k + 1; kp <= K; kp++) {
//                    for (int i : c_s) {
//                        if (i != j) {
//                            model.add(phi[j] + Z[k][kp][i][j] <= 1);
//                        }
//                    }
//                }
//            }
//        }
//
//        // Constraint 17
//        for (int i : instance->c_prime) {
//            for (int k : start_stage) {
//                for (int kp = k + 1; kp <= K; kp++) {
//                    for (int j : c_t) {
//                        if (i != j) {
//                            model.add(phi[i] + Z[k][kp][i][j] <= 1);
//                        }
//                    }
//                }
//            }
//        }
//
//        // Constraint 14
//        for (int k : start_stage) {
//            for (int i : c_s) {
//                IloExpr sumZ(env);
//                for (int kp = k + 1; kp <= K; kp++) {
//                    for (int j : c_t) {
//                        sumZ += Z[k][kp][i][j];
//                    }
//                }
//                model.add(sumZ <= X[k][i]);
//            }
//        }
//
//        // Constraint 15
//        for (int kp : end_stage) {
//            for (int j : c_t) {
//                IloExpr sumZ(env);
//                for (int k = 1; k < kp; k++) {
//                    for (int i : c_s) {
//                        sumZ += Z[k][kp][i][j];
//                    }
//                }
//                model.add(sumZ <= X[kp][j]);
//            }
//        }
//
//        // Constraint 18
//        // nếu (i , j) là điểm thả/bắt của sortie, drone không thể đi cạnh i -> j.
//        for (int k : start_stage) {
//            for (int kp = k + 1; kp <= K; kp++) {
//                for (int i : c_s) {
//                    for (int j : c_t) {
//                        if (i != j) {
//                            model.add(Z[k][kp][i][j] + x_d[i][j] <= 1);
//                        }
//                    }
//                }
//            }
//        }
//
//        // Constraint 19
//        if (!cfg.allow_loop) {
//            for (int i : C) {
//                for (int j : C) {
//                    if (i != j) {
//                        model.add(x_d[i][j] + x_d[j][i] <= 1);
//                    }
//                }
//            }
//        }
//
//
//        for (int k = 1; k < K; k++) {
//            for (int i = 0; i < D; i++) {
//                IloExpr sum(env);
//                for (int j = 1; j <= D; j++) {
//                    sum += x[k][i][j];
//                }
//                model.add(X[k][i] == sum).setName(("C11_" + std::to_string(k) + "_" + std::to_string(i)).c_str());
//            }
//        }
//
//        for (int k = 2; k <= K; k++) {
//            for (int i = 1; i <= D; i++) {
//                IloExpr sum(env);
//                for (int j = 0; j < D; j++) {
//                    sum += x[k - 1][j][i];
//                }
//                // arcs entering i at stage k.
//                model.add(X[k][i] == sum).setName(("C12_" + std::to_string(k) + "_" + std::to_string(i)).c_str());
//            }
//        }
//
//        // C2: truck must depart from the depot at stage 1.
//        IloExpr C2(env);
//        for (int i = 1; i <= D; i++) {
//            C2 += x[1][O][i];
//        }
//
//        IloConstraint c2(C2 == 1);
//        model.add(c2).setName("Must start from source");
//
//        // C3: Terminal depot must be arrived only once.
//        IloExpr C3(env);
//        for (int k = 2; k <= K; k++) {
//            C3 += X[k][D];
//        }
//        model.add(C3 == 1).setName("Must visit terminal depot once");
//
//        // At most ONE node each stage.
//        for (int k = 1; k <= K; k++) {
//            IloExpr sum(env);
//            for (int i = 0; i <= D; i++)
//                sum += X[k][i];
//            model.add(sum <= 1).setName(("C13_at_most_one_customer_at_stage_" + std::to_string(k)).c_str());
//        }
//
//        IloBoolVarArray R(env, K + 1);
//        for (int k = 1; k < K; k++) {
//            R[k].setName(("R_" + std::to_string(k)).c_str());
//        }
//        // crossing constraint?
//        for (int k = 1; k < K; k++) {
//            IloExpr expr(env);
//            IloExpr expr_r2(env);
//            for (int k_p = k + 1; k_p <= K; k_p++) {
//                expr += z[k][k_p];
//            }
//            model.add(R[k] == expr).setName(("C14_" + std::to_string(k)).c_str());
//        }
//
//        // modified C7
//        for (int k = 1; k <= K - 1; k++) {
//            for (int k_p = k + 1; k_p <= K; k_p++) {
//                for (int l = k + 1; l < k_p; l++) {
//                    // tranh drone bay cac doan giao nhau.
//                    if (k < l) {
//                        model.add(z[k][k_p] + R[l] <= 1).setName(("C7m_" + std::to_string(k)
//                                + "_" + std::to_string(k_p) + "_" + std::to_string(l))
//                            .c_str());
//                    }
//                }
//            }
//        }
//
//        // modified C7p - we can select at most one segment that contains the point l
//        for (int l = 2; l < K; l++) {
//            IloExpr expr(env);
//
//            for (int k = 1; k < l; k++)
//                for (int k_p = l; k_p <= K; k_p++)
//                    expr += z[k][k_p];
//            model.add(expr <= 1).setName(("C7mm_" + std::to_string(l)).c_str());
//        }
//
//        for (int h : C) {
//            IloExpr sumX(env);
//            for (int k = 2; k < K; k++) {
//                sumX += X[k][h];
//            }
//            model.add(sumX + phi[h] >= 1);
//        }
//
//        /////////// C14: node_stage
//        for (int k = 1; k <= K_arc; k++) {
//            IloExpr sum(env);
//            for (int i = 0; i < D; i++) {
//                for (int j = 1; j <= D; j++) {
//                    sum += x[k][i][j] * tau[i][j];
//                }
//            }
//
//            model.add(a[k + 1] == d[k] + sum).setName(
//                ("C14_" + std::to_string(k) + "_" + std::to_string(k + 1)).c_str());
//        }
//
//        for (int k = 1; k <= K - 1; k++) {
//            for (int k_p = k + 1; k_p <= K; k_p++) {
//                if (k < k_p) {
//                    model.add(a[k_p] - d[k] <= z[k][k_p] * (cfg.dtl - cfg.sr) + (1 - z[k][k_p]) * M).setName(
//                        ("C15_" + std::to_string(k) + "_" + std::to_string(k_p)).c_str());
//                    IloExpr truck_length(env);
//                    for (int start = k; start < k_p; start++) {
//                        for (int i : c_s) {
//                            for (int j : c_t) {
//                                truck_length += x[start][i][j] * tau[i][j];
//                            }
//                        }
//                    }
//                    model.add(truck_length <= z[k][k_p] * (cfg.dtl - cfg.sr) + (1 - z[k][k_p]) * M);
//                }
//            }
//        }
//
//        // Drone movement time constraint.
//        for (int k : start_stage) {
//            for (int kp = k + 1; kp <= K; kp++) {
//                IloExpr sum_drone_travel(env);
//                IloExpr start_at_kp(env);
//                for (int i : c_s) {
//                    for (int j : c_t) {
//                        if (i != j) {
//                            sum_drone_travel += X_drone[k][kp][i][j] * tau_prime[i][j];
//                        }
//                    }
//                }
//                for (int l = kp + 1; l <= K; l++) {
//                    start_at_kp += z[kp][l];
//                }
//                model.add(d[kp] >= d[k] + sum_drone_travel + cfg.sr + start_at_kp * cfg.sl - (1 - z[k][kp]) * M);
//            }
//        }
//
//        for (int kp = 2; kp <= K; kp++) {
//            IloExpr sum_truck_move(env);
//            for (int i : c_s) {
//                for (int j : c_t) {
//                    sum_truck_move += x[kp - 1][i][j] * tau[i][j];
//                }
//            }
//            IloExpr end_at_kp(env);
//            IloExpr start_at_kp(env);
//
//            for (int k = 1; k < kp; k++) {
//                end_at_kp += z[k][kp];
//            }
//            for (int l = kp + 1; l <= K; l++) {
//                start_at_kp += z[kp][l];
//            }
//            model.add(d[kp] >= d[kp - 1] + sum_truck_move + start_at_kp * cfg.sl + end_at_kp * cfg.sr);
//        }
//
//        for (int k = 1; k < K; k++) {
//            for (int i : c_s) {
//                IloExpr sumZ(env);
//                for (int kp = k + 1; kp <= K; kp++) {
//                    for (int j : c_t) {
//                        sumZ += Z[k][kp][i][j];
//                    }
//                }
//                model.add(sumZ <= X[k][i]);
//            }
//        }
//
//        for (int kp = 2; kp <= K; kp++) {
//            for (int j : c_t) {
//                IloExpr sumZ(env);
//                for (int k = 1; k < kp; k++) {
//                    for (int i : c_s) {
//                        if (i != j) {
//                            sumZ += Z[k][kp][i][j];
//                        }
//                    }
//                }
//                model.add(sumZ <= X[kp][j]);
//            }
//        }
//
//        // Lower bound constraint
//        IloExpr truck_length(env), drone_length(env);
//        for (int i : c_s) {
//            for (int j : c_t) {
//                if (i != j) {
//                    for (int k = 1; k < K_arc; k++) {
//                        truck_length += x[k][i][j] * tau[i][j];
//                    }
//                    drone_length += x_d[i][j] * tau_prime[i][j];
//                }
//            }
//        }
//        model.add(d[K] >= truck_length);
//        model.add(d[K] >= drone_length);
//
//        // Constraint 23
//        for (int k : start_stage) {
//            for (int i : c_s) {
//                for (int j : instance->c_prime) {
//                    model.add(td[j] + M * (2 - d_out[k][i] - x_d[i][j]) >= d[k] + tau_prime[i][j]);
//                }
//            }
//        }
//
//        // Constraint 25
//        for (int kp : end_stage) {
//            for (int j : c_t) {
//                for (int i : instance->c_prime) {
//                    model.add(d[kp] + M * (2 - d_in[kp][j] - x_d[i][j]) >= td[i] + tau_prime[i][j]);
//                }
//            }
//        }
//        model.add(IloMinimize(env, d[K]));
//        cplex.exportModel("rv-mvd.lp");
//        auto startTime = std::chrono::high_resolution_clock::now();
//        if (cfg.use_cutting_plane) {
//            while (true) {
//                if (!cplex.solve()) {
//                    std::cout << "Unable to optimize model!" << std::endl;
//                }
//                for (int k = 1; k < K; k++) {
//                    for (int kp = k + 1; kp <= K; kp++) {
//                        for (int i : c_s) {
//                            for (int j : c_t) {
//                                if (cplex.getValue(Z[k][kp][i][j]) == 1) {
//                                    std::cout << "check z: " << k << " " << kp << " " << i << " " << j << std::endl;
//                                    std::cout << "-------------------------------------------------" << std::endl;
//                                    std::cout << "Start at stage " << k << " at node " << i << std::endl;
//                                    std::cout << "End at stage " << kp << " at node " << j << std::endl;
//                                    int start_node = i, end_node = j;
//                                    // Route in this sortie
//                                    std::vector<std::vector<int>> sortie_tour(D + 1);
//                                    for (int index = 0; index < D + 1; index++) {
//                                        sortie_tour[index].resize(D + 1);
//                                    }
//                                    for (int start : c_s) {
//                                        for (int end : c_t) {
//                                            if (start != end) {
//                                                if (cplex.getValue(X_drone[k][kp][start][end]) == 1) {
//                                                    sortie_tour[start][end] = 1;
//                                                }
//                                            }
//                                        }
//                                    }
//                                    auto cycles = find_cycles(sortie_tour);
//                                    if (cycles.size() == 0) {
//                                        goto LABEL_DONE;
//                                    }
//                                    std::cout << "Number of cycles: " << cycles.size() << std::endl;
//                                    for (auto& cyc : cycles) {
//                                        std::cout << "Printing cycle:";
//                                        setPrint(cyc);
//                                        IloExpr sum(env);
//                                        for (int c_i : cyc) {
//                                            for (int c_j : cyc) {
//                                                sum += X_drone[k][kp][c_i][c_j];
//                                            }
//                                        }
//                                        int cycle_size = cyc.size();
//                                        model.add(sum <= cycle_size - 1);
//                                    }
//
//
//                                    std::cout << "-------------------------------------------------" << std::endl;
//                                }
//                            }
//                        }
//                    }
//                }
//            }
//        }
//        else {
//            cplex.solve();
//            if (cfg.use_cplex_conflict_refiner) {
//                // A list of constraints to be considered by the conflict refiner.
//                IloConstraintArray constraints(env);
//
//                // Loop over all objects in the model and gather constraints.
//                for (IloModel::Iterator it(model); it.ok(); ++it) {
//                    IloExtractable ext = *it;
//                    if (ext.isVariable()) {
//                        IloNumVar v = ext.asVariable();
//                        // Add variable bounds to the constraints array.
//                        constraints.add(IloBound(v, IloBound::Lower));
//                        constraints.add(IloBound(v, IloBound::Upper));
//                    }
//                    else if (ext.isConstraint()) {
//                        IloConstraint c = ext.asConstraint();
//                        constraints.add(c);
//                    }
//                }
//
//                // Define preferences for the constraints. Here, we give all
//                // constraints a preference of 1.0, so they will be treated
//                // equally.
//                IloNumArray prefs(env, constraints.getSize());
//                for (int i = 0; i < prefs.getSize(); ++i)
//                    prefs[i] = 1.0;
//
//                // Run the conflict refiner. As opposed to letting the conflict
//                // refiner run to completion (as is done here), the user can set
//                // a resource limit (e.g., a time limit, an iteration limit, or
//                // node limit) and still potentially get a "possible" conflict.
//                if (cplex.refineConflict(constraints, prefs)) {
//                    // Display the solution status.
//                    IloCplex::CplexStatus status = cplex.getCplexStatus();
//                    std::cout << "Solution status = " << status << " (" <<
//                        static_cast<int>(status) << ")" << std::endl;
//
//                    // Get the conflict status for the constraints that were specified.
//                    IloCplex::ConflictStatusArray conflict = cplex.getConflict(constraints);
//
//                    // Print constraints that participate in the conflict.
//                    std::cout << "Conflict:" << std::endl;
//
//                    for (int i = 0; i < constraints.getSize(); ++i) {
//                        if (conflict[i] == IloCplex::ConflictMember ||
//                            conflict[i] == IloCplex::ConflictPossibleMember) {
//                            std::cout << "  " << constraints[i] << std::endl;
//                        }
//                    }
//
//                    std::cout << std::endl;
//
//                    // Write the identified conflict in the LP format.
//                    const char* confFile = "iloconflictex1.lp";
//                    std::cout << "Writing conflict file to '" << confFile << "'...." << std::endl;
//                    cplex.writeConflict(confFile);
//
//                    // Display the entire conflict subproblem.
//                    std::string line;
//                    std::ifstream file(confFile);
//                    if (file.is_open()) {
//                        while (getline(file, line)) {
//                            std::cout << line << std::endl;
//                        }
//                    }
//                    file.close();
//                }
//                else {
//                    std::cout << "A conflict was not identified." << std::endl;
//                    std::cout << "Exiting...." << std::endl;
//                }
//            }
//        }
//    LABEL_DONE:
//        auto endTime = std::chrono::high_resolution_clock::now();
//        // Calculate the elapsed time
//        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
//
//        std::cout << "Feasible solution found!" << std::endl;
//        std::cout << "Truck nodes:" << std::endl;
//        for (int k = 1; k <= K; k++) {
//            for (int i = 0; i <= D; i++) {
//                auto X_val = cplex.getValue(X[k][i]);
//                if (abs(X_val - 1) < 1e-6) {
//                    auto d_k = cplex.getValue(d[k]);
//                    auto a_k = cplex.getValue(a[k]);
//                    std::cout << "(" << a_k << ")-" << i << "(" << k << ")-(" << d_k << ")----->";
//                    break;
//                }
//            }
//        }
//        std::cout << std::endl << "Truck arcs:" << std::endl;
//        std::map<int, std::pair<int, int>> map_stage_truck_arc;
//        for (int k = 1; k <= K_arc; k++) {
//            for (int i = 0; i < D; i++) {
//                for (int j = 1; j <= D; j++) {
//                    if (i != j) {
//                        auto X_val = cplex.getValue(x[k][i][j]);
//                        if (abs(X_val - 1) < 1e-6) {
//                            std::cout << "Arc " << k << " connecting " << i << " and " << j
//                                << " with cost " << tau[i][j] << " " << std::endl;
//
//                            map_stage_truck_arc[k] = std::make_pair(i, j);
//                            break;
//                        }
//                    }
//                }
//            }
//        }
//        int phi_check = 0;
//
//        for (int h : instance->c_prime) {
//            if (abs(cplex.getValue(phi[h]) - 1) < 1e-6) {
//                std::cout << "Customer " << h << " was served by drone!" << std::endl;
//                phi_check += 1;
//            }
//        }
//
//        std::cout << "Sorties information: ";
//        if (phi_check == 0) {
//            std::cout << "No sortie!" << std::endl;
//        }
//        else {
//            std::cout << phi_check << " customer(s) served by drone." << std::endl;
//        }
//        for (int k = 1; k < K; k++) {
//            for (int kp = k + 1; kp <= K; kp++) {
//                for (int i : c_s) {
//                    for (int j : c_t) {
//                        if (abs(cplex.getValue(Z[k][kp][i][j]) - 1) < 1e-6) {
//                            std::cout << "check z: " << k << " " << kp << " " << i << " " << j << std::endl;
//                            std::cout << "-------------------------------------------------" << std::endl;
//                            std::cout << "Start at stage " << k << " at node " << i << std::endl;
//                            std::cout << "End at stage " << kp << " at node " << j << std::endl;
//                            int start_node = i, end_node = j;
//                            // Route in this sortie
//                            for (int start : c_s) {
//                                for (int end : c_t) {
//                                    if (start != end) {
//                                        if (cplex.getValue(X_drone[k][kp][start][end]) == 1) {
//                                            std::cout << start << "--->" << end << std::endl;
//                                        }
//                                    }
//                                }
//                            }
//                            std::cout << "-------------------------------------------------" << std::endl;
//                        }
//                    }
//                }
//            }
//        }
//
//        std::cout << "Drone path:" << std::endl;
//        for (int i : c_s) {
//            for (int j : c_t) {
//                if (i != j) {
//                    if (abs(cplex.getValue(x_d[i][j]) - 1) < 1e-6) {
//                        std::cout << i << " " << j << std::endl;
//                    }
//                }
//            }
//        }
//
//        std::cout << "---------------------------------------------------------------" << std::endl;
//        std::cout << "Drone path based on X_drone_k_kp_i_j: " << std::endl;
//        for (int k : start_stage) {
//            for (int kp = k + 1; kp <= K; kp++) {
//                for (int i : c_s) {
//                    for (int j : c_t) {
//                        if (i != j) {
//                            if (abs(cplex.getValue(X_drone[k][kp][i][j]) - 1) < 1e-6) {
//                                std::cout << "X_drone[" << k << "][" << kp << "][" << i << "][" << j << "] = 1" <<
//                                    std::endl;
//                            }
//                        }
//                    }
//                }
//            }
//        }
//        std::cout << "---------------------------------------------------------------" << std::endl;
//        std::cout << "Print zp:" << std::endl;
//        for (int j : instance->c_prime) {
//            for (int k : start_stage) {
//                for (int kp = k + 1; kp <= K; kp++) {
//                    if (abs(cplex.getValue(zp[k][kp][j]) - 1) < 1e-6) {
//                        std::cout << "zp[" << k << "][" << kp << "][" << j << "] = 1" << std::endl;
//                    }
//                }
//            }
//        }
//
//        std::cout << "---------------------------------------------------------------" << std::endl;
//        std::cout << "Print z:" << std::endl;
//        for (int k : start_stage) {
//            for (int kp = k + 1; kp <= K; kp++) {
//                if (abs(cplex.getValue(z[k][kp]) - 1) < 1e-6) {
//                    std::cout << "z[" << k << "][" << kp << "] = 1" << std::endl;
//                }
//            }
//        }
//        std::cout << "Done!" << std::endl;
//        // std::cout << "-------------------------Re-calculated objective-----------------------" << std::endl;
//        // std::cout << obj << std::endl;
//        // std::cout << "------------------------------------------------------------------------" << std::endl;
//
//        double c = cplex.getObjValue();
//        double gap = cplex.getMIPRelativeGap();
//        std::cout << "Solve time: " << duration.count() / 1000.0 << std::endl;
//        cplex.end();
//        model.end();
//        env.end();
//
//
//        std::cout << "OBJECTIVE VALUE: " << c << ", NUMBER OF SORTIES: " << phi_check << "." << std::endl;
//        return Result{c, duration.count() / 1000.0, gap};
//    }
//    catch (IloException& e) {
//        std::cout << e.getMessage() << std::endl;
//    }
//}