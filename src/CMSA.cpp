//
// Created by who on 26/10/2024.
//

#include "../include/CMSA.h"
#include "../include/solution.h"
#include <map>
#include <set>
#include <cfloat>
#include <algorithm>

void two_opt(vector<int> &route, vector<vector<double>> &distance)
{
    std::cout << "Two-opt" << std::endl;
    bool improved = true;
    while (improved)
    {
        improved = false;
        for (int i = 1; i < route.size() - 2; i++)
        {
            for (int j = i + 1; j < route.size() - 1; j++)
            {
                double delta = distance[route[i]][route[i - 1]] + distance[route[j + 1]][route[j]] -
                               distance[route[i]][route[j + 1]] - distance[route[i - 1]][route[j]];
                if (delta > 0.01)
                {
                    reverse(route.begin() + i, route.begin() + j + 1);
                    improved = true;
                }
            }
        }
    }
}

// Đã cập nhật jetsuite
void CMSA::initialize()
{
     std::cout << "Initializing CMSA..." << std::endl;
    // Init age for Truck, Drone, and Jetsuite
    arc_age = vector<vector<int>>(instance->tau.size(), vector<int>(instance->tau.size(), -1));
    customer_age = vector<int>(instance->tau.size(), -1);
    Y_age = vector<vector<vector<int>>>(instance->tau.size(), vector<vector<int>>(instance->tau.size(), vector<int>(instance->tau.size(), -1)));
    W_age = vector<vector<vector<int>>>(instance->tau.size(), vector<vector<int>>(instance->tau.size(), vector<int>(instance->tau.size(), -1)));

    // **Thêm tuổi cho Jetsuite (U_age)**
    U_age = vector<vector<vector<int>>>(instance->tau.size(), vector<vector<int>>(instance->tau.size(), vector<int>(instance->tau.size(), -1)));

    // Precompute order of near -> far customers for each customer (Truck)
    for (auto t : instance->tau)
    {
        vector<int> indices(t.size());
        std::iota(indices.begin(), indices.end(), 0);
        std::sort(indices.begin(), indices.end(), [&t](int i1, int i2)
                  { return t[i1] < t[i2]; });
        near_customers.push_back(indices);
    }

    std::cout << "Precomputed near customers for Truck." << std::endl;

    // **Precompute order for Jetsuite**
    vector<vector<int>> near_customers_jetsuite;
    for (auto t : instance->tau_prime_prime)
    {
        vector<int> indices(t.size());
        std::iota(indices.begin(), indices.end(), 0);
        std::sort(indices.begin(), indices.end(), [&t](int i1, int i2)
                  { return t[i1] < t[i2]; });
        near_customers_jetsuite.push_back(indices);
    }

     std::cout << "Precomputed near customers for Jetsuite." << std::endl;
    // Precompute drone-weight customers = avg(top_k min time trip) for Drone
    customer_weight = vector<double>(instance->tau.size(), 0);
    for (int j = 1; j < customer_weight.size() - 1; j++)
    {
        vector<double> trip_time;
        for (int i = 0; i < instance->tau_prime.size(); i++)
            for (int k = 0; k < instance->tau_prime.size(); k++)
                if (i != k)
                    trip_time.push_back(instance->tau_prime[i][j] + instance->tau_prime[j][k]);

        std::sort(trip_time.begin(), trip_time.end());
        for (int i = 0; i < top_k; i++)
            customer_weight[j] += trip_time[i];
        customer_weight[j] /= top_k;
    }

    std::cout << "Precomputed customer weights for Drone." << std::endl;

    // **Precompute jetsuite-weight customers = avg(top_k min time trip) for Jetsuite**
    customer_weight_jetsuite = vector<double>(instance->tau.size(), 0);
    for (int j = 1; j < customer_weight_jetsuite.size() - 1; j++)
    {
        vector<double> trip_time;
        for (int i = 0; i < instance->tau_prime_prime.size(); i++)
            for (int k = 0; k < instance->tau_prime_prime.size(); k++)
                if (i != k)
                    trip_time.push_back(instance->tau_prime_prime[i][j] + instance->tau_prime_prime[j][k]);

        std::sort(trip_time.begin(), trip_time.end());
        for (int i = 0; i < top_k; i++)
            customer_weight_jetsuite[j] += trip_time[i];
        customer_weight_jetsuite[j] /= top_k;
    }

    std::cout << "Precomputed customer weights for Jetsuite." << std::endl;

    // Khởi tạo Y_age và W_age với kích thước phù hợp
    int n = instance->tau.size();
    Y_age.resize(n);
    for (int i = 0; i < n; i++) {
        Y_age[i].resize(n);
        for (int h = 0; h < n; h++) {
            Y_age[i][h].resize(n, -1);
        }
    }
    
    W_age.resize(n);
    for (int h = 0; h < n; h++) {
        W_age[h].resize(n);
        for (int j = 0; j < n; j++) {
            W_age[h][j].resize(n, -1);
        }
    }

    std::cout << "Initialization complete." << std::endl;
}

// Đã cập nhật sử lý jetsuite
Solution CMSA::generate_initial_solution()
{
    std::cout << "Generating initial solution..." << std::endl; 
    if (!sol_pool.empty())
    {
        return randomizer.get_random_item(sol_pool);
    }

    vector<vector<double>> tau = instance->tau;
    vector<vector<double>> tau_prime = instance->tau_prime;
    vector<vector<double>> tau_prime_prime = instance->tau_prime_prime;
    vector<int> truck_order;
    vector<bool> visited(tau.size(), false);

    // Đảm bảo truck_order bắt đầu trống để không xảy ra lỗi khi thêm nút
    truck_order.clear();
    
    // Thêm nút bắt đầu và kết thúc (depot)
    truck_order.push_back(0);  // Bắt đầu từ depot
    visited[0] = true;

    // Số khách hàng để thêm vào lộ trình
    int num_customers = instance->num_node - 1;  // Số lượng khách hàng không bao gồm depot
    
    // Lựa chọn một số khách hàng để đưa vào lộ trình xe tải
    // Thay vì random, chọn một cách tổ hợp tốt hơn
    for (int i = 1; i <= num_customers; i++) {
        if (!visited[i]) {
            truck_order.push_back(i);
            visited[i] = true;
        }
    }
    
    // Thêm depot vào cuối
    truck_order.push_back(instance->num_node - 1);
    visited[instance->num_node - 1] = true;

    // Cải thiện lộ trình bằng thuật toán two-opt
    two_opt(truck_order, tau);
    
    std::cout << "Truck order generated: ";
    for (const auto &customer : truck_order) {
        std::cout << customer << " ";
    }
    std::cout << std::endl;
    
    // Làm sạch truck_route trước khi sử dụng
    truck_route.clear();
    
    // Khởi tạo solution object với truck_order
    vector<vector<int>> drone_order;
    vector<vector<int>> jetsuite_order;
    
    Solution solution(0.0, 0.0, 0.0, 0.0, instance, cfg, truck_order, drone_order, jetsuite_order);
    solution.recalculateObjective();
    
    return solution;
}

// Đã cập nhật cho jetsuite
void CMSA::update_age(Solution &s, bool increase)
{
    // std::cout << "Update 3" << std::endl;
    // Order update: increase by 1 first (if increase = true) then reset age according to s
    if (increase)
    {
        // Cập nhật tuổi cho Truck
        for (auto &i : arc_age)
            for (int &age : i)
            {
                if (age != -1)
                    age++;
                if (age >= age_limit)
                    age = -1;
            }

        // Cập nhật tuổi cho khách hàng
        for (int &age : customer_age)
        {
            if (age != -1)
                age++;
            if (age >= age_limit)
                age = -1;
        }

        // Cập nhật tuổi cho Drone
        for (auto &i : Y_age)
            for (auto &j : i)
                for (int &age : j)
                {
                    if (age != -1)
                        age++;
                    if (age >= age_limit)
                        age = -1;
                }
        for (auto &i : W_age)
            for (auto &j : i)
                for (int &age : j)
                {
                    if (age != -1)
                        age++;
                    if (age >= age_limit)
                        age = -1;
                }

        // **Cập nhật tuổi cho Jetsuite**
        for (auto &i : U_age)
            for (auto &j : i)
                for (int &age : j)
                {
                    if (age != -1)
                        age++;
                    if (age >= age_limit)
                        age = -1;
                }
    }
    // std::cout << "Update 4" << std::endl;
    if (s.cost > 0)
    {
        // Đặt lại tuổi cho các cung đường trong truck_order
        for (int j = 0; j < s.truck_order.size() - 1; j++)
            arc_age[s.truck_order[j]][s.truck_order[j + 1]] = 0;

        // Đặt lại tuổi cho các chuyến đi bằng Drone
        for (auto &trip : s.drone_order)
        {
            customer_age[trip[1]] = 0;
            Y_age[s.truck_order[trip[0]]][trip[1]][trip[0]] = 0;
            W_age[trip[1]][s.truck_order[trip[2]]][trip[2]] = 0;
        }

        // **Đặt lại tuổi cho các chuyến đi bằng Jetsuite**
        for (auto &trip : s.jetsuite_order)
        {
            customer_age[trip[2]] = 0;
            U_age[trip[1]][trip[2]][trip[0]] = 0;
        }
    }
}

vector<int> CMSA::generate_relaxed_solution()
{
    // std::cout << "Update 5" << std::endl;
    vector<int> s(instance->tau.size(), 0);
    vector<bool> visited(instance->tau.size(), false);

    // Head and tail of the route
    visited[0] = true;
    s[instance->tau.size() - 1] = (int)instance->tau.size() - 1;
    visited[instance->tau.size() - 1] = true;

    // Select randomly one customer
    int first_customer = randomizer.get_random_number(1, (int)instance->tau.size() - 2);
    s[1] = first_customer;
    visited[first_customer] = true;

    for (int i = 2; i < instance->tau.size() - 1; i++)
    {
        int last_customer = s[i - 1];
        vector<int> candidates;

        for (int candidate : near_customers[last_customer])
        {
            if (!visited[candidate] && candidate != last_customer)
                candidates.push_back(candidate);
            if (candidates.size() == top_k)
                break;
        }

        int select_candidate = randomizer.get_random_item(candidates);
        s[i] = select_candidate;
        visited[select_candidate] = true;
    }
    two_opt(s, instance->tau);
    // std::cout << "Update 6" << std::endl;
    // Try to cut out some customers from truck to Jetsuite
    vector<vector<int>> jetsuite_order;
    for (int launch_it = 0; launch_it < s.size() - 2; launch_it++)
    {
        int launch_node = s[launch_it];
        int recovery_node = s[launch_it + 1];

        for (int customer_it = launch_it + 1; customer_it < s.size() - 1; customer_it++)
        {
            int customer = s[customer_it];
            double truck_time = instance->tau[launch_node][customer] + instance->tau[customer][recovery_node];
            double jetsuite_time = instance->tau_prime_prime[launch_node][customer] + instance->tau_prime_prime[customer][recovery_node];

            // Check if Jetsuite is faster than Truck and is feasible
            if (jetsuite_time < truck_time && jetsuite_time <= (cfg->dtl - cfg->tJ_R))
            {
                jetsuite_order.push_back({launch_it, launch_node, customer, recovery_node});
                s.erase(s.begin() + customer_it);
                visited[customer] = true;
                break;
            }
        }
    }
    // std::cout << "Update 7" << std::endl;
    // Thêm khai báo candidates trước khi sử dụng nó
    std::vector<int> candidates;
    
    // Thêm các phần tử vào candidates (giả sử có code ở đây)
    // Ví dụ:
    for (int i = 1; i < instance->tau.size() - 1; i++) {
        candidates.push_back(i);
    }
    
    int select_candidate = randomizer.get_random_item(candidates);
    
    // Code tiếp theo...
    
    // Sửa lại lỗi constructor và kiểu trả về của hàm
    std::vector<std::vector<int>> empty_vector;
    // Solution sol(instance, cfg, s, jetsuite_order); // Cách gọi cũ
    // Tạo vector trống cho tham số thứ 3 của constructor
    std::vector<double> empty_times;
    Solution sol(0.0, 0.0, 0.0, 0.0, instance, cfg, s, jetsuite_order, empty_vector);
    
    return s; // Trả về vector s thay vì Solution sol
}

Solution CMSA::solve_cplex(double r_limit) {
    // Khởi tạo solution với các giá trị hợp lệ
    vector<int> empty_truck_order;
    vector<vector<int>> empty_drone_order;
    vector<vector<int>> empty_jetsuite_order;
    
    // Khởi tạo solution với giá trị mặc định
    Solution best_solution(DBL_MAX, 0, 0, 0, instance, cfg, 
                           empty_truck_order, empty_drone_order, empty_jetsuite_order);

    int N = instance->num_node;
    int K = N - 1;
    
    // Create CPLEX model
    IloEnv env;
    IloModel model(env);
    
    // Phần khởi tạo các biến quyết định và ràng buộc
    try {
        // Decision variables
    // X[i][k] binary variables
    IloArray<IloBoolVarArray> X(env, N + 2);
        for (int i = 0; i < N + 2; ++i) {
        X[i] = IloBoolVarArray(env, K + 1);
            for (int k = 1; k <= K; ++k) {
            std::stringstream varName;
            varName << "X_" << i << "_" << k;
                X[i][k] = IloBoolVar(env, varName.str().c_str());
                model.add(X[i][k]);
        }
    }

    // x[i][j][k] binary variables
    IloArray<IloArray<IloBoolVarArray>> x(env, N + 2);
        for (int i = 0; i < N + 2; ++i) {
        x[i] = IloArray<IloBoolVarArray>(env, N + 2);
            for (int j = 0; j < N + 2; ++j) {
            x[i][j] = IloBoolVarArray(env, K + 1);
                for (int k = 1; k <= K; ++k) {
                std::stringstream varName;
                varName << "x_" << i << "_" << j << "_" << k;
                    x[i][j][k] = IloBoolVar(env, varName.str().c_str());
                    model.add(x[i][j][k]);
            }
        }
    }

    // phi[h] binary variables
    IloBoolVarArray phi(env, N + 1);
        for (int h = 1; h < N; ++h) {
        std::stringstream varName;
        varName << "phi_" << h;
            phi[h] = IloBoolVar(env, varName.str().c_str());
            model.add(phi[h]);
    }

    // Y[i][h][k] binary variables
    IloArray<IloArray<IloBoolVarArray>> Y(env, N + 2);
        for (int i = 0; i < N + 2; ++i) {
        Y[i] = IloArray<IloBoolVarArray>(env, N + 1);
            for (int h = 1; h < N; ++h) {
            Y[i][h] = IloBoolVarArray(env, K + 1);
                for (int k = 1; k <= K; ++k) {
                std::stringstream varName;
                varName << "Y_" << i << "_" << h << "_" << k;
                    Y[i][h][k] = IloBoolVar(env, varName.str().c_str());
                    model.add(Y[i][h][k]);
            }
        }
    }

    // W[h][j][k] binary variables
    IloArray<IloArray<IloBoolVarArray>> W(env, N + 1);
        for (int h = 1; h < N; ++h) {
        W[h] = IloArray<IloBoolVarArray>(env, N + 2);
            for (int j = 0; j < N + 2; ++j) {
            W[h][j] = IloBoolVarArray(env, K + 1);
                for (int k = 1; k <= K; ++k) {
                std::stringstream varName;
                varName << "W_" << h << "_" << j << "_" << k;
                    W[h][j][k] = IloBoolVar(env, varName.str().c_str());
                    model.add(W[h][j][k]);
                }
            }
        }
        
        // phiJetsuite[h] binary variables
    IloBoolVarArray phiJetsuite(env, N + 1);
        for (int h = 1; h < N; ++h) {
        std::stringstream varName;
        varName << "phiJ_" << h;
            phiJetsuite[h] = IloBoolVar(env, varName.str().c_str());
            model.add(phiJetsuite[h]);
    }

        // U[i][h][k] binary variables for Jetsuite
    IloArray<IloArray<IloBoolVarArray>> U(env, N + 2);
        for (int i = 0; i < N + 2; ++i) {
        U[i] = IloArray<IloBoolVarArray>(env, N + 1);
            for (int h = 1; h < N; ++h) {
            U[i][h] = IloBoolVarArray(env, K + 1);
                for (int k = 1; k <= K; ++k) {
                std::stringstream varName;
                varName << "U_" << i << "_" << h << "_" << k;
                    U[i][h][k] = IloBoolVar(env, varName.str().c_str());
                    model.add(U[i][h][k]);
                }
            }
        }
        
        // Thêm hàm mục tiêu đơn giản để đảm bảo mô hình hoạt động
        IloExpr objective(env);
        
        // Chi phí của cạnh truck
        for (int i = 0; i < N + 2; ++i) {
            for (int j = 0; j < N + 2; ++j) {
                for (int k = 1; k <= K; ++k) {
                    if (i != j && i < instance->tau.size() && j < instance->tau[i].size()) {
                        objective += instance->tau[i][j] * x[i][j][k];
                    }
                }
            }
        }
        
        // Chi phí cho drone (nếu có)
        if (!instance->tau_prime.empty()) {
            for (int i = 0; i < N + 2; ++i) {
                for (int h = 1; h < N; ++h) {
                    for (int j = 0; j < N + 2; ++j) {
                        for (int k = 1; k <= K; ++k) {
                            for (int k1 = k; k1 <= K; ++k1) {
                                if (i < instance->tau_prime.size() && h < instance->tau_prime[i].size() &&
                                    h < instance->tau_prime.size() && j < instance->tau_prime[h].size()) {
                                    double drone_cost = instance->tau_prime[i][h] + instance->tau_prime[h][j];
                                    objective += 0.1 * drone_cost * phi[h];
                                }
                            }
                        }
                    }
                }
            }
        }
        
        // Chi phí cho jetsuite (nếu có)
        if (!instance->tau_prime_prime.empty()) {
            for (int i = 0; i < N + 2; ++i) {
                for (int h = 1; h < N; ++h) {
                    for (int k = 1; k <= K; ++k) {
                        if (i < instance->tau_prime_prime.size() && h < instance->tau_prime_prime[i].size() &&
                            h < instance->tau_prime_prime.size() && i < instance->tau_prime_prime[h].size()) {
                            double jetsuite_cost = instance->tau_prime_prime[i][h] + instance->tau_prime_prime[h][i];
                            objective += 0.2 * jetsuite_cost * phiJetsuite[h];
                        }
                    }
                }
            }
        }
        
        // Thêm hàm mục tiêu tối thiểu vào mô hình
        model.add(IloMinimize(env, objective));
        objective.end();
        
        // Thêm các ràng buộc cơ bản
        // 1. Ràng buộc về lộ trình liên tục của truck
        for (int k = 1; k < K; ++k) {
            for (int i = 0; i < N + 2; ++i) {
                IloExpr flow_in(env);
                IloExpr flow_out(env);
                
                for (int j = 0; j < N + 2; ++j) {
                    if (i != j) {
                        flow_in += x[j][i][k];
                        flow_out += x[i][j][k+1];
                    }
                }
                
                // Flow conservation: luồng vào = luồng ra
                model.add(flow_in - X[i][k] == 0);
                model.add(flow_out - X[i][k+1] == 0);
                
                flow_in.end();
                flow_out.end();
            }
        }
        
        // 2. Ràng buộc: Mỗi khách hàng chỉ được phục vụ bởi một phương tiện
        for (int h = 1; h < N; ++h) {
            IloExpr served(env);
            
            // Khách hàng h được truck phục vụ
            for (int k = 1; k <= K; ++k) {
                served += X[h][k];
            }
            
            // Khách hàng h được drone phục vụ
            served += phi[h];
            
            // Khách hàng h được jetsuite phục vụ
            served += phiJetsuite[h];
            
            // Tổng cộng phải đúng 1
            model.add(served == 1);
            served.end();
        }
        
        // 3. Ràng buộc: Mỗi drone operation phải có launch và recovery
        for (int h = 1; h < N; ++h) {
            IloExpr launches(env);
            IloExpr recoveries(env);
            
            for (int i = 0; i < N + 2; ++i) {
                for (int k = 1; k <= K; ++k) {
                    launches += Y[i][h][k];
                }
            }
            
            for (int j = 0; j < N + 2; ++j) {
                for (int k = 1; k <= K; ++k) {
                    recoveries += W[h][j][k];
                }
            }
            
            // Nếu khách hàng được drone phục vụ, phải có đúng 1 launch và 1 recovery
            model.add(launches - phi[h] == 0);
            model.add(recoveries - phi[h] == 0);
            
            launches.end();
            recoveries.end();
        }
        
        // 4. Ràng buộc: Jetsuite operations
        for (int h = 1; h < N; ++h) {
            IloExpr jetsuite_ops(env);
            
            for (int i = 0; i < N + 2; ++i) {
                for (int k = 1; k <= K; ++k) {
                    jetsuite_ops += U[i][h][k];
                }
            }
            
            // Nếu khách hàng được jetsuite phục vụ, phải có đúng 1 operation
            model.add(jetsuite_ops - phiJetsuite[h] == 0);
            jetsuite_ops.end();
        }
        
        // Giải model bằng CPLEX
    IloCplex cplex(model);
    cplex.setParam(IloCplex::Param::MIP::Strategy::Probe, -1);
    cplex.setParam(IloCplex::Param::TimeLimit, r_limit);
    if (cfg->num_thread > 0)
        cplex.setParam(IloCplex::Param::Threads, cfg->num_thread);
    cplex.setParam(IloCplex::Param::Emphasis::MIP, 5);
        cplex.setOut(env.getNullStream());
        cplex.setError(env.getNullStream());
        cplex.setWarning(env.getNullStream());
        
    cplex.solve();

    vector<int> truck_order;
    vector<vector<int>> drone_order;
    vector<vector<int>> jetsuite_order;
    double c = -1;

        // Xử lý kết quả từ CPLEX
        if (cplex.getStatus() == IloAlgorithm::Optimal || cplex.getStatus() == IloAlgorithm::Feasible) {
        c = cplex.getObjValue();
        std::cout << "CPLEX found a solution with objective value: " << c << std::endl;

            // Lưu quyết định từ CPLEX
        collect_decision_variables(cplex, X, x, phi, Y, W, phiJetsuite, U);

            // Lấy kết quả của truck
            for (int k = 1; k <= K; ++k) {
                for (int i = 0; i < N + 2; ++i) {
                    if (cplex.getValue(X[i][k]) > 0.5) {
                    truck_order.push_back(i);
                    break;
                }
            }
        }

            // Nếu truck_order trống hoặc không hợp lệ, sử dụng initial solution
            if (truck_order.empty() || truck_order.size() < 2) {
                std::cout << "Warning: Invalid truck_order, generating an initial solution instead.\n";
                // Tạo giải pháp ban đầu
                Solution initial_solution = generate_initial_solution();
                
                // Cập nhật best_solution
                best_solution = initial_solution;
                
                // Tạo các chuyến đi hợp lệ dựa trên lộ trình xe tải từ initial solution
                create_valid_trips(best_solution.truck_order, drone_order, jetsuite_order);
                best_solution.drone_order = drone_order;
                best_solution.jetsuite_order = jetsuite_order;
                
                // Tính toán lại thời gian và chi phí
                best_solution.recalculateTime();
                best_solution.recalculateObjective();
                
                return best_solution;
            }
            
            // Nếu drone_order và jetsuite_order rỗng, tạo các chuyến đi hợp lệ
            drone_order.clear();
            jetsuite_order.clear();
            
            // Tạo các chuyến đi hợp lệ cho drone và jetsuite
            create_valid_trips(truck_order, drone_order, jetsuite_order);
            
            // Cập nhật best_solution
            best_solution.truck_order = truck_order;
            best_solution.drone_order = drone_order;
            best_solution.jetsuite_order = jetsuite_order;
            best_solution.cost = c;
            
            // Tối ưu hóa lộ trình xe tải bằng two-opt nếu cần
            if (best_solution.truck_order.size() > 3) {
                two_opt(best_solution.truck_order, instance->tau);
                std::cout << "Truck route optimized using two-opt method" << std::endl;
                
                // Tính toán lại thời gian và chi phí
                best_solution.recalculateTime();
                best_solution.recalculateObjective();
            }
        } else {
            std::cout << "CPLEX could not find a solution. Status: " << cplex.getStatus() << std::endl;
            // Tạo giải pháp ban đầu
            best_solution = generate_initial_solution();
            
            // Tạo các chuyến đi hợp lệ dựa trên lộ trình xe tải từ initial solution
            create_valid_trips(best_solution.truck_order, drone_order, jetsuite_order);
            
            // Cập nhật best_solution
            best_solution.drone_order = drone_order;
            best_solution.jetsuite_order = jetsuite_order;
            
            // Tính toán lại thời gian và chi phí
            best_solution.recalculateTime();
            best_solution.recalculateObjective();
        }
    } catch (IloException &e) {
        std::cerr << "CPLEX Exception: " << e.getMessage() << std::endl;
        env.end();
        
        // Tạo giải pháp ban đầu trong trường hợp lỗi
        best_solution = generate_initial_solution();
        vector<vector<int>> drone_order;
        vector<vector<int>> jetsuite_order;
        
        // Tạo các chuyến đi hợp lệ dựa trên lộ trình xe tải từ initial solution
        create_valid_trips(best_solution.truck_order, drone_order, jetsuite_order);
        
        // Cập nhật best_solution
        best_solution.drone_order = drone_order;
        best_solution.jetsuite_order = jetsuite_order;
        
        // Tính toán lại thời gian và chi phí
        best_solution.recalculateTime();
        best_solution.recalculateObjective();
        
        return best_solution;
    }

    env.end();
    return best_solution;
}

// Định nghĩa phương thức solve()
Solution CMSA::solve() {
    std::cout << "Starting CMSA algorithm..." << std::endl;
    auto start_time = std::chrono::high_resolution_clock::now();

    // Khởi tạo cấu trúc dữ liệu
    initialize();
    std::cout << "Data structures initialized" << std::endl;

    // Tạo giải pháp ban đầu
    Solution best_solution = generate_initial_solution();
    std::cout << "Initial solution generated" << std::endl;

    try {
    // Gọi phương thức solve_cplex để tìm giải pháp tối ưu
    double time_limit = cfg->tl > 0 ? cfg->tl : 600;
    best_solution = solve_cplex(time_limit);
    std::cout << "CPLEX solution obtained" << std::endl;
        
        // Kiểm tra lời giải và đảm bảo có truck route hợp lệ
        if (best_solution.truck_order.empty()) {
            std::cerr << "Warning: CPLEX did not produce a valid truck route. Using initial solution instead." << std::endl;
            best_solution = generate_initial_solution();
        }
        
        // Ngay cả khi có truck route, đảm bảo có ít nhất một số drone và jetsuite operations
        if (best_solution.drone_order.empty() && best_solution.jetsuite_order.empty()) {
            std::cout << "Warning: No drone or jetsuite operations found. Adding some valid operations..." << std::endl;
            create_valid_trips(best_solution.truck_order, best_solution.drone_order, best_solution.jetsuite_order);
        }

    // Cải thiện giải pháp bằng phương pháp two-opt cho tuyến đường xe tải
    if (best_solution.truck_order.size() > 3) {
        two_opt(best_solution.truck_order, instance->tau);
        std::cout << "Truck route optimized using two-opt method" << std::endl;
    }

    // Tính toán chi phí cuối cùng
        best_solution.recalculateTime();
    best_solution.recalculateObjective();
        
        // Cập nhật lại biến thành viên để lưu trữ kết quả
        truck_route = best_solution.truck_order;
        drone_operations = best_solution.drone_order;
        jetsuite_operations = best_solution.jetsuite_order;
        
    } catch (IloException& e) {
        std::cerr << "Error in CPLEX: " << e.getMessage() << std::endl;
        // Sử dụng giải pháp ban đầu nếu có lỗi
        std::cout << "Using initial solution due to CPLEX error" << std::endl;
        
        // Đảm bảo có ít nhất một số drone và jetsuite operations
        if (best_solution.drone_order.empty() && best_solution.jetsuite_order.empty()) {
            create_valid_trips(best_solution.truck_order, best_solution.drone_order, best_solution.jetsuite_order);
            
            // Cập nhật biến thành viên
            truck_route = best_solution.truck_order;
            drone_operations = best_solution.drone_order;
            jetsuite_operations = best_solution.jetsuite_order;
            
            // Cập nhật chi phí
            best_solution.recalculateTime();
            best_solution.recalculateObjective();
        }
    } catch (std::exception& e) {
        std::cerr << "Standard exception: " << e.what() << std::endl;
        // Xử lý tương tự như trên
        if (best_solution.drone_order.empty() && best_solution.jetsuite_order.empty()) {
            create_valid_trips(best_solution.truck_order, best_solution.drone_order, best_solution.jetsuite_order);
            truck_route = best_solution.truck_order;
            drone_operations = best_solution.drone_order;
            jetsuite_operations = best_solution.jetsuite_order;
            best_solution.recalculateTime();
            best_solution.recalculateObjective();
        }
    } catch (...) {
        std::cerr << "Unknown exception occurred" << std::endl;
        // Xử lý tương tự
        if (best_solution.drone_order.empty() && best_solution.jetsuite_order.empty()) {
            create_valid_trips(best_solution.truck_order, best_solution.drone_order, best_solution.jetsuite_order);
            truck_route = best_solution.truck_order;
            drone_operations = best_solution.drone_order;
            jetsuite_operations = best_solution.jetsuite_order;
            best_solution.recalculateTime();
            best_solution.recalculateObjective();
        }
    }
    
    // Thống kê về jetsuit và drone
    int drone_count = best_solution.drone_order.size();
    int jetsuit_count = best_solution.jetsuite_order.size();
    int truck_customers = best_solution.truck_order.size() - 2; // Trừ đi 2 depot
    
    std::cout << "\n=== VEHICLE USAGE STATISTICS ===\n";
    std::cout << "Customers served by truck: " << truck_customers << std::endl;
    std::cout << "Customers served by drone: " << drone_count << std::endl;
    std::cout << "Customers served by jetsuit: " << jetsuit_count << std::endl;
    std::cout << "Total customers: " << (truck_customers + drone_count + jetsuit_count) << std::endl;
    
    // In tổng kết
    double drone_efficiency = drone_count > 0 ? 100.0 * drone_count / (truck_customers + drone_count + jetsuit_count) : 0;
    double jetsuit_efficiency = jetsuit_count > 0 ? 100.0 * jetsuit_count / (truck_customers + drone_count + jetsuit_count) : 0;
    
    std::cout << "Drone usage ratio: " << drone_efficiency << "%" << std::endl;
    std::cout << "Jetsuit usage ratio: " << jetsuit_efficiency << "%" << std::endl;
    
    // Tính tổng khoảng cách truck
    total_truck_distance = 0.0;
    for(size_t i = 0; i < best_solution.truck_order.size() - 1; i++) {
        int from = best_solution.truck_order[i];
        int to = best_solution.truck_order[i + 1];
        if (from < instance->tau.size() && to < instance->tau[from].size()) {
            total_truck_distance += instance->tau[from][to];
        }
    }
    std::cout << "Total truck distance: " << total_truck_distance << std::endl;

    // Lưu kết quả
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end_time - start_time;
    computation_time = elapsed.count();
    std::cout << "Computation time: " << computation_time << " seconds" << std::endl;

    // Cập nhật kết quả
    best_objective_value = best_solution.cost;
    iteration_count++;
    solution_count++;
    num_iterations = iteration_count;
    num_solutions_evaluated = solution_count;

    return best_solution;
}

void CMSA::print_solution() {
    // Đường dẫn tệp đầu ra
    std::string output_filename = "results_jetsuite.txt";
    if (!output_file.empty()) {
        output_filename = output_file;
    }
    std::ofstream outFile(output_filename);
    
    if (!outFile.is_open()) {
        std::cerr << "Error: Could not open file " << output_filename << std::endl;
        return;
    }

    // Ghi tiêu đề
    outFile << "=== CMSA Algorithm Results ===\n";
    outFile << "Computation time: " << computation_time << " seconds\n";
    outFile << "Iterations: " << num_iterations << "\n";
    outFile << "Solutions evaluated: " << num_solutions_evaluated << "\n";
    outFile << "Best objective value: " << best_objective_value << "\n";
    
    // Ghi lộ trình xe tải
    outFile << "\nTruck Route: ";
    for(size_t i = 0; i < truck_route.size(); i++) {
        outFile << truck_route[i];
        if(i < truck_route.size() - 1) outFile << " -> ";
    }
    outFile << "\nTotal truck distance: " << total_truck_distance << "\n";
    
    // Ghi thông tin drone
    outFile << "\nDrone Operations:\n";
    int total_drone_trips = 0;
    double total_drone_distance = 0.0;
    
    for (size_t i = 0; i < drone_operations.size(); i++) {
        const auto& op = drone_operations[i];
        total_drone_trips++;
        
        // Tính toán khoảng cách drone
        double trip_distance = 0.0;
        if (op.size() >= 3) {
            int launch_node = truck_route[op[0]];
            int customer = op[1];
            int recovery_node = truck_route[op[2]];
            
            trip_distance = instance->tau_prime[launch_node][customer] + 
                           instance->tau_prime[customer][recovery_node];
            total_drone_distance += trip_distance;
            
            outFile << "  Trip " << i+1 << ": Launch from node " << launch_node 
                    << " at pos " << op[0] << ", serve customer " << customer
                    << ", recover at node " << recovery_node << " at pos " << op[2]
                    << " (distance: " << trip_distance << ")\n";
        }
    }
    
    outFile << "Total drone trips: " << total_drone_trips << "\n";
    outFile << "Total drone distance: " << total_drone_distance << "\n";
    
    // Ghi thông tin jetsuite
    outFile << "\nJetsuite Operations:\n";
    int total_jetsuite_trips = 0;
    double total_jetsuite_distance = 0.0;
    
    for (size_t i = 0; i < jetsuite_operations.size(); i++) {
        const auto& op = jetsuite_operations[i];
        total_jetsuite_trips++;
        
        // Tính toán khoảng cách jetsuite
        double trip_distance = 0.0;
        if (op.size() >= 3) {
            int launch_node = op[1];
            int customer = op[2];
            
            trip_distance = instance->tau_prime_prime[launch_node][customer] + 
                           instance->tau_prime_prime[customer][launch_node];
            total_jetsuite_distance += trip_distance;
            
            outFile << "  Trip " << i+1 << ": Launch at stage " << op[0] 
                    << " from node " << launch_node << ", serve customer " << customer
                    << ", return to node " << launch_node
                    << " (distance: " << trip_distance << ")\n";
        }
    }
    
    outFile << "Total jetsuite trips: " << total_jetsuite_trips << "\n";
    outFile << "Total jetsuite distance: " << total_jetsuite_distance << "\n";
    
    // Thống kê tổng hợp
    outFile << "\n=== SUMMARY ===\n";
    int total_customers = (truck_route.size() - 2) + total_drone_trips + total_jetsuite_trips;
    outFile << "Total customers: " << total_customers << "\n";
    outFile << "Customers served by truck: " << (truck_route.size() - 2) << " (" 
            << (100.0 * (truck_route.size() - 2) / total_customers) << "%)\n";
    outFile << "Customers served by drone: " << total_drone_trips << " (" 
            << (100.0 * total_drone_trips / total_customers) << "%)\n";
    outFile << "Customers served by jetsuit: " << total_jetsuite_trips << " (" 
            << (100.0 * total_jetsuite_trips / total_customers) << "%)\n";
    
    outFile << "\n==============================\n";
    outFile.close();
    
    std::cout << "Results written to " << output_filename << std::endl;
}

void CMSA::print_debug_info() {
    std::cout << "\n=== DETAILED VEHICLE INFORMATION ===\n";
    
    // Print truck information
    std::cout << "\n1. TRUCK INFORMATION:\n";
    std::cout << "Route: ";
    for (size_t i = 0; i < truck_route.size(); i++) {
        std::cout << truck_route[i];
        if (i < truck_route.size() - 1) std::cout << " -> ";
    }
    std::cout << "\n";
    
    // Print truck decision variables
    if (!truck_nodes.empty()) {
        std::cout << "Truck decision variables:\n";
        std::cout << "  Nodes visited by truck (X variables):\n";
        for (int i : truck_nodes) {
            std::cout << "    X[" << i << "] = 1 (truck visits node " << i << ")\n";
        }
    } else {
        std::cout << "No truck node variables collected.\n";
    }
    
    if (!truck_arcs.empty()) {
        std::cout << "  Truck arcs used (x variables):\n";
        for (const auto& arc : truck_arcs) {
            if (arc.size() >= 3) {
                std::cout << "    x[" << arc[0] << "][" << arc[1] << "][" << arc[2] << "] = 1 (truck travels from " 
                          << arc[0] << " to " << arc[1] << " at stage " << arc[2] << ")\n";
            }
        }
    } else {
        std::cout << "No truck arc variables collected.\n";
    }
    
    if (!truck_route.empty()) {
        double total_distance = 0.0;
        std::cout << "Detailed distances:\n";
        for (size_t i = 0; i < truck_route.size() - 1; i++) {
            int from = truck_route[i];
            int to = truck_route[i + 1];
            if (from < instance->tau.size() && to < instance->tau[from].size()) {
                double leg_distance = instance->tau[from][to];
                total_distance += leg_distance;
                std::cout << "  " << from << " -> " << to 
                        << ": " << leg_distance << " units\n";
            } else {
                std::cerr << "Warning: Invalid indices for tau: [" << from << "][" << to << "]\n";
            }
        }
        std::cout << "Total truck distance: " << total_distance << " units\n";
    } else {
        std::cout << "No route information to calculate distances.\n";
    }
    
    // Print drone information
    std::cout << "\n2. DRONE INFORMATION:\n";
    if (drone_operations.empty()) {
        std::cout << "No drone operations.\n";
    } else {
        std::cout << "Number of drone trips: " << drone_operations.size() << "\n";
        
        // Print drone decision variables
        if (!drone_customers.empty()) {
            std::cout << "Drone decision variables:\n";
            std::cout << "  Customers served by drone (phi variables):\n";
            for (int h : drone_customers) {
                std::cout << "    phi[" << h << "] = 1 (customer " << h << " served by drone)\n";
            }
        } else {
            std::cout << "No drone customer variables collected.\n";
        }
        
        if (!drone_launches.empty()) {
            std::cout << "  Drone launch operations (Y variables):\n";
            for (const auto& op : drone_launches) {
                if (op.size() >= 3) {
                    std::cout << "    Y[" << op[0] << "][" << op[1] << "][" << op[2] << "] = 1 (drone launched from node " 
                              << op[0] << " to customer " << op[1] << " at stage " << op[2] << ")\n";
                }
            }
        } else {
            std::cout << "No drone launch variables collected.\n";
        }
        
        if (!drone_recoveries.empty()) {
            std::cout << "  Drone recovery operations (W variables):\n";
            for (const auto& op : drone_recoveries) {
                if (op.size() >= 3) {
                    std::cout << "    W[" << op[0] << "][" << op[1] << "][" << op[2] << "] = 1 (drone recovered at node " 
                              << op[1] << " after visiting customer " << op[0] << " at stage " << op[2] << ")\n";
                }
            }
        } else {
            std::cout << "No drone recovery variables collected.\n";
        }
        
        double total_drone_distance = 0.0;
        double total_drone_time = 0.0;
        
        for (size_t i = 0; i < drone_operations.size(); i++) {
            const auto& op = drone_operations[i];
            if (op.size() >= 3) {
                int launch_pos = op[0];
                int customer = op[1];
                int recovery_pos = op[2];
                
                // Kiểm tra kỹ lưỡng các chỉ số
                if (launch_pos < 0 || launch_pos >= (int)truck_route.size()) {
                    std::cerr << "Warning: Invalid launch position for drone trip " << i+1 << "\n";
                    continue;
                }
                
                if (recovery_pos < 0 || recovery_pos >= (int)truck_route.size()) {
                    std::cerr << "Warning: Invalid recovery position for drone trip " << i+1 << "\n";
                    continue;
                }
                
                int launch_node = truck_route[launch_pos];
                int recovery_node = truck_route[recovery_pos];
                
                if (launch_node >= instance->tau_prime.size() || 
                    customer >= instance->tau_prime[launch_node].size() ||
                    customer >= instance->tau_prime.size() ||
                    recovery_node >= instance->tau_prime[customer].size()) {
                    std::cerr << "Warning: Invalid indices for tau_prime in drone trip " << i+1 << "\n";
                    continue;
                }
                
                double outbound = instance->tau_prime[launch_node][customer];
                double inbound = instance->tau_prime[customer][recovery_node];
                double total_trip = outbound + inbound;
                
                total_drone_distance += total_trip;
                total_drone_time += total_trip + cfg->sl + cfg->sr; // Add launch and recovery time
                
                std::cout << "  Trip " << i+1 << ":\n";
                std::cout << "    Launch (position " << launch_pos << ", node " << launch_node 
                          << ") -> Customer " << customer << " -> Recovery (position " << recovery_pos 
                          << ", node " << recovery_node << ")\n";
                std::cout << "    Distance: " << outbound << " + " << inbound 
                          << " = " << total_trip << " units\n";
                std::cout << "    Time: " << total_trip + cfg->sl + cfg->sr 
                          << " units (including launch " << cfg->sl << " and recovery " << cfg->sr << ")\n";
                
                // Calculate truck time from launch to recovery
                double truck_time = 0.0;
                for (int j = launch_pos; j < recovery_pos; j++) {
                    if (j < (int)truck_route.size()-1) {
                        int from = truck_route[j];
                        int to = truck_route[j + 1];
                        if (from < instance->tau.size() && to < instance->tau[from].size()) {
                            truck_time += instance->tau[from][to];
                        } else {
                            std::cerr << "Warning: Invalid indices for tau: [" << from << "][" << to << "]\n";
                        }
                    }
                }
                
                std::cout << "    Truck time: " << truck_time << " units\n";
                std::cout << "    Efficiency: " << (truck_time > total_trip + cfg->sl + cfg->sr ? "Good" : "Poor") << "\n";
            }
        }
        
        std::cout << "Total drone distance: " << total_drone_distance << " units\n";
        std::cout << "Total drone time: " << total_drone_time << " units\n";
    }
    
    // Print jetsuit information
    std::cout << "\n3. JETSUIT INFORMATION:\n";
    if (jetsuite_operations.empty()) {
        std::cout << "No jetsuit operations.\n";
    } else {
        std::cout << "Number of jetsuit trips: " << jetsuite_operations.size() << "\n";
        
        // Print jetsuit decision variables
        if (!jetsuit_customers.empty()) {
            std::cout << "Jetsuit decision variables:\n";
            std::cout << "  Customers served by jetsuit (phiJetsuit variables):\n";
            for (int h : jetsuit_customers) {
                std::cout << "    phiJetsuit[" << h << "] = 1 (customer " << h << " served by jetsuit)\n";
            }
        } else {
            std::cout << "No jetsuit customer variables collected.\n";
        }
        
        if (!jetsuit_ops.empty()) {
            std::cout << "  Jetsuit operations (U variables):\n";
            for (const auto& op : jetsuit_ops) {
                if (op.size() >= 3) {
                    std::cout << "    U[" << op[0] << "][" << op[1] << "][" << op[2] << "] = 1 (jetsuit travels from node " 
                              << op[0] << " to customer " << op[1] << " at stage " << op[2] << ")\n";
                }
            }
        } else {
            std::cout << "No jetsuit operation variables collected.\n";
        }
        
        double total_jetsuit_distance = 0.0;
        double total_jetsuit_time = 0.0;
        
        for (size_t i = 0; i < jetsuite_operations.size(); i++) {
            const auto& op = jetsuite_operations[i];
            if (op.size() >= 4) {
                int stage = op[0];
                int launch_node = op[1];
                int customer = op[2];
                int return_node = op[3];
                
                // Thêm kiểm tra các chỉ số trước khi truy cập dữ liệu
                if (launch_node < 0 || launch_node >= (int)instance->tau_prime_prime.size()) {
                    std::cerr << "Warning: Invalid launch node for jetsuit trip " << i+1 << "\n";
                    continue;
                }

                if (customer < 0 || customer >= (int)instance->tau_prime_prime[launch_node].size()) {
                    std::cerr << "Warning: Invalid customer for jetsuit trip " << i+1 << "\n";
                    continue;
                }
                
                if (customer >= (int)instance->tau_prime_prime.size()) {
                    std::cerr << "Warning: Invalid customer index for tau_prime_prime lookup for jetsuit trip " << i+1 << "\n";
                    continue;
                }
                
                if (return_node < 0 || return_node >= (int)instance->tau_prime_prime[customer].size()) {
                    std::cerr << "Warning: Invalid return node for jetsuit trip " << i+1 << "\n";
                    continue;
                }
                
                double outbound = instance->tau_prime_prime[launch_node][customer];
                double inbound = instance->tau_prime_prime[customer][return_node];
                double total_trip = outbound + inbound;
                
                total_jetsuit_distance += total_trip;
                total_jetsuit_time += total_trip + cfg->sl + cfg->sr; // Add launch and recovery time
                
                std::cout << "  Trip " << i+1 << ":\n";
                std::cout << "    Stage " << stage << ": Node " << launch_node 
                          << " -> Customer " << customer << " -> Return to " << return_node << "\n";
                std::cout << "    Distance: " << outbound << " + " << inbound 
                          << " = " << total_trip << " units\n";
                std::cout << "    Time: " << total_trip + cfg->sl + cfg->sr 
                          << " units (including launch " << cfg->sl << " and recovery " << cfg->sr << ")\n";
            }
        }
        
        std::cout << "Total jetsuit distance: " << total_jetsuit_distance << " units\n";
        std::cout << "Total jetsuit time: " << total_jetsuit_time << " units\n";
    }
    
    // Summary statistics
    std::cout << "\n4. SUMMARY STATISTICS:\n";
    int truck_customers = truck_route.size() > 2 ? truck_route.size() - 2 : 0; // Subtract 2 depots
    int drone_customers_count = drone_operations.size();
    int jetsuit_customers_count = jetsuite_operations.size();
    int total_customers = truck_customers + drone_customers_count + jetsuit_customers_count;
    
    std::cout << "Total customers: " << total_customers << "\n";
    if (total_customers > 0) {
        std::cout << "  - Served by truck: " << truck_customers << " (" 
                << (100.0 * truck_customers / total_customers) << "%)\n";
        std::cout << "  - Served by drone: " << drone_customers_count << " (" 
                << (100.0 * drone_customers_count / total_customers) << "%)\n";
        std::cout << "  - Served by jetsuit: " << jetsuit_customers_count << " (" 
                << (100.0 * jetsuit_customers_count / total_customers) << "%)\n";
    } else {
        std::cout << "No customers served.\n";
    }
}

// Add this new function after solve_cplex
void CMSA::collect_decision_variables(IloCplex& cplex, 
                                   IloArray<IloBoolVarArray>& X, 
                                   IloArray<IloArray<IloBoolVarArray>>& x, 
                                   IloBoolVarArray& phi, 
                                   IloArray<IloArray<IloBoolVarArray>>& Y,
                                   IloArray<IloArray<IloBoolVarArray>>& W, 
                                   IloBoolVarArray& phiJetsuite,
                                   IloArray<IloArray<IloBoolVarArray>>& U) {
    try {
        // Xác định kích thước của mảng
        int num_nodes = instance->num_node;
        int K = num_nodes - 1;
        
        // Làm sạch các cấu trúc lưu trữ kết quả
    truck_nodes.clear();
    truck_arcs.clear();
    drone_customers.clear();
    drone_launches.clear();
    drone_recoveries.clear();
    jetsuit_customers.clear();
    jetsuit_ops.clear();
    
        std::cout << "Collecting decision variables..." << std::endl;
        
        // Lấy thông tin từ các biến X (vị trí truck tại mỗi giai đoạn)
        for (int k = 1; k <= K; k++) {
            for (int i = 0; i < X.getSize() && i < num_nodes; i++) {
                if (i < X.getSize() && k < X[i].getSize()) {
                try {
                    if (cplex.getValue(X[i][k]) > 0.5) {
                        truck_nodes.push_back(i);
                            truck_route.push_back(i);
                    }
                } catch (IloException &e) {
                        std::cerr << "Lỗi khi truy cập X[" << i << "][" << k << "]: " << e.getMessage() << std::endl;
                    }
                }
            }
        }
        
        // Lấy thông tin từ các biến x (cạnh của truck)
        for (int i = 0; i < x.getSize() && i < num_nodes; i++) {
            for (int j = 0; j < x[i].getSize() && j < num_nodes; j++) {
                for (int k = 1; k <= K && k < x[i][j].getSize(); k++) {
                    try {
                        if (cplex.getValue(x[i][j][k]) > 0.5) {
                            truck_arcs.push_back({i, j, k});
                        }
                    } catch (IloException &e) {
                        // Bỏ qua lỗi và tiếp tục
                    }
                }
            }
        }
        
        // Lấy thông tin về khách hàng được phục vụ bởi drone (phi)
        for (int h = 1; h < phi.getSize() && h < num_nodes; h++) {
            try {
                if (cplex.getValue(phi[h]) > 0.5) {
                    drone_customers.push_back(h);
                }
            } catch (IloException &e) {
                // Bỏ qua lỗi và tiếp tục
            }
        }
        
        // Lưu thông tin phóng và thu hồi drone (Y và W)
        std::map<int, std::pair<int, int>> drone_launches_map;  // khách hàng -> (vị trí phóng, giai đoạn)
        std::map<int, std::pair<int, int>> drone_recoveries_map;  // khách hàng -> (vị trí thu hồi, giai đoạn)
        
        // Thông tin phóng drone (Y)
        for (int i = 0; i < Y.getSize() && i < num_nodes; i++) {
            for (int h = 1; h < Y[i].getSize() && h < num_nodes; h++) {
                for (int k = 1; k <= K && k < Y[i][h].getSize(); k++) {
                    try {
                        if (cplex.getValue(Y[i][h][k]) > 0.5) {
                            drone_launches.push_back({i, h, k});
                            
                            // Tìm vị trí trong truck_route
                            for (size_t pos = 0; pos < truck_route.size(); pos++) {
                                if (truck_route[pos] == i) {
                                    drone_launches_map[h] = {pos, k};
                                    break;
                                }
                            }
                        }
                    } catch (IloException &e) {
                        // Bỏ qua lỗi và tiếp tục
                    }
                }
            }
        }
        
        // Thông tin thu hồi drone (W)
        for (int h = 1; h < W.getSize() && h < num_nodes; h++) {
            for (int j = 0; j < W[h].getSize() && j < num_nodes; j++) {
                for (int k = 1; k <= K && k < W[h][j].getSize(); k++) {
                    try {
                        if (cplex.getValue(W[h][j][k]) > 0.5) {
                            drone_recoveries.push_back({h, j, k});
                            
                            // Tìm vị trí trong truck_route
                            for (size_t pos = 0; pos < truck_route.size(); pos++) {
                                if (truck_route[pos] == j) {
                                    drone_recoveries_map[h] = {pos, k};
                                    break;
                                }
                            }
                        }
                    } catch (IloException &e) {
                        // Bỏ qua lỗi và tiếp tục
                    }
                }
            }
        }
        
        // Thông tin về khách hàng được phục vụ bởi jetsuite (phiJetsuite)
        for (int h = 1; h < phiJetsuite.getSize() && h < num_nodes; h++) {
            try {
                if (cplex.getValue(phiJetsuite[h]) > 0.5) {
                    jetsuit_customers.push_back(h);
                }
            } catch (IloException &e) {
                // Bỏ qua lỗi và tiếp tục
            }
        }
        
        // Thông tin về hoạt động của jetsuite (U)
        for (int i = 0; i < U.getSize() && i < num_nodes; i++) {
            for (int h = 1; h < U[i].getSize() && h < num_nodes; h++) {
                for (int k = 1; k <= K && k < U[i][h].getSize(); k++) {
                    try {
                        if (cplex.getValue(U[i][h][k]) > 0.5) {
                            jetsuit_ops.push_back({i, h, k});
                            
                            // Thêm vào danh sách hoạt động jetsuite
                            int return_node = i;  // Mặc định quay lại nút phóng
                            jetsuite_operations.push_back({k, i, h, return_node});
                        }
                    } catch (IloException &e) {
                        // Bỏ qua lỗi và tiếp tục
                    }
                }
            }
        }
        
        // Tạo drone_operations từ thông tin đã thu thập
        drone_operations.clear();
        for (int h : drone_customers) {
            if (drone_launches_map.find(h) != drone_launches_map.end() && 
                drone_recoveries_map.find(h) != drone_recoveries_map.end()) {
                int launch_pos = drone_launches_map[h].first;
                int recovery_pos = drone_recoveries_map[h].first;
                
                // Kiểm tra các chỉ số
                if (launch_pos >= 0 && launch_pos < (int)truck_route.size() &&
                    recovery_pos >= 0 && recovery_pos < (int)truck_route.size()) {
                    drone_operations.push_back({launch_pos, h, recovery_pos});
                }
            }
        }
        
        std::cout << "Đã thu thập thành công thông tin quyết định từ giải pháp CPLEX." << std::endl;
        std::cout << "- Số nút truck: " << truck_nodes.size() << std::endl;
        std::cout << "- Số cạnh truck: " << truck_arcs.size() << std::endl;
        std::cout << "- Số khách hàng drone: " << drone_customers.size() << std::endl;
        std::cout << "- Số khách hàng jetsuite: " << jetsuit_customers.size() << std::endl;
        std::cout << "- Số hoạt động drone: " << drone_operations.size() << std::endl;
        std::cout << "- Số hoạt động jetsuite: " << jetsuite_operations.size() << std::endl;
    }
    catch (IloException& e) {
        std::cerr << "Lỗi trong collect_decision_variables: " << e.getMessage() << std::endl;
    }
    catch (std::exception& e) {
        std::cerr << "Lỗi chuẩn trong collect_decision_variables: " << e.what() << std::endl;
    }
    catch (...) {
        std::cerr << "Lỗi không xác định trong collect_decision_variables" << std::endl;
    }
}

// Phương thức tạo các chuyến đi hợp lệ cho drone và jetsuite
void CMSA::create_valid_trips(std::vector<int>& truck_order,
                            std::vector<std::vector<int>>& drone_order,
                            std::vector<std::vector<int>>& jetsuite_order) {
    // Xóa các chuyến đi cũ
    drone_order.clear();
    jetsuite_order.clear();
    
    // Kiểm tra truck_order hợp lệ
    if (truck_order.size() < 2) {
        std::cout << "Error: Invalid truck_order size. Cannot create valid trips.\n";
        return;
    }
    
    // Đảm bảo depot ở đầu và cuối lộ trình
    if (truck_order.front() != 0) {
        std::cout << "Warning: First node in truck_order is not depot. Adding depot at start.\n";
        truck_order.insert(truck_order.begin(), 0);
    }
    
    if (truck_order.back() != instance->num_node - 1) {
        std::cout << "Warning: Last node in truck_order is not depot. Adding depot at end.\n";
        truck_order.push_back(instance->num_node - 1);
    }
    
    // Xây dựng tập khách hàng hiện tại trong lộ trình xe tải
    std::set<int> truck_customers;
    for (int i = 1; i < (int)truck_order.size() - 1; i++) {
        truck_customers.insert(truck_order[i]);
    }
    
    // Tạo một vector tạm lưu lộ trình xe tải 
    std::vector<int> new_truck_order;
    new_truck_order.push_back(truck_order.front()); // Thêm depot đầu tiên
    
    int max_drone_trips = std::min(3, (int)truck_customers.size());
    int max_jetsuite_trips = std::min(3, (int)truck_customers.size());
    
    // Xây dựng danh sách khách hàng để chuyển từ truck sang drone/jetsuite
    std::set<int> customers_to_reassign;
    if (truck_customers.size() > 3) {
        // Chọn một số khách hàng ngẫu nhiên từ truck để chuyển sang drone/jetsuite
        std::vector<int> all_truck_customers(truck_customers.begin(), truck_customers.end());
        std::random_shuffle(all_truck_customers.begin(), all_truck_customers.end());
        
        // Lấy một nửa khách hàng truck để chuyển đổi phương tiện (tối đa 6)
        int num_to_reassign = std::min(6, (int)all_truck_customers.size() / 2);
        for (int i = 0; i < num_to_reassign; i++) {
            customers_to_reassign.insert(all_truck_customers[i]);
        }
    }
    
    // Lặp qua tất cả khách hàng trong truck_order
    for (int i = 1; i < (int)truck_order.size() - 1; i++) {
        int customer = truck_order[i];
        
        // Nếu khách hàng này không cần chuyển đổi, giữ lại trong lộ trình truck
        if (customers_to_reassign.find(customer) == customers_to_reassign.end()) {
            new_truck_order.push_back(customer);
            continue;
        }
        
        // Thử thêm vào drone trước
        bool added_to_alternate = false;
        if ((int)drone_order.size() < max_drone_trips && !instance->tau_prime.empty()) {
            // Chọn vị trí phóng và thu hồi hợp lý
            int launch_pos = 0;  // Bắt đầu từ depot
            int recovery_pos = new_truck_order.size();  // Vị trí cuối cùng
            
            int launch_node = new_truck_order[launch_pos];
            int recovery_node = truck_order.back();  // Depot cuối cùng
            
            // Kiểm tra các chỉ số hợp lệ
            bool is_valid = true;
            
            // Kiểm tra khả năng tiếp cận với tau_prime
            if (launch_node >= (int)instance->tau_prime.size() || 
                customer >= (int)instance->tau_prime[launch_node].size() || 
                customer >= (int)instance->tau_prime.size() || 
                recovery_node >= (int)instance->tau_prime[customer].size()) {
                is_valid = false;
            }
            
            if (is_valid) {
                // Kiểm tra xem có thời gian di chuyển được định nghĩa
                if (instance->tau_prime[launch_node][customer] <= 0 || 
                    instance->tau_prime[customer][recovery_node] <= 0) {
                    is_valid = false;
                }
            }
            
            if (is_valid) {
                // Thêm chuyến drone
                drone_order.push_back({launch_pos, customer, recovery_pos});
                std::cout << "Created drone trip: Launch at pos " << launch_pos 
                          << " (node " << launch_node << "), serve customer " << customer 
                          << ", recover at pos " << recovery_pos 
                          << " (node " << recovery_node << ")\n";
                added_to_alternate = true;
                
                // Bỏ qua khách hàng này trong lộ trình xe tải
                continue;
            }
        }
        
        // Nếu không thêm được vào drone, thử thêm vào jetsuite
        if (!added_to_alternate && (int)jetsuite_order.size() < max_jetsuite_trips && !instance->tau_prime_prime.empty()) {
            // Chọn vị trí trung tâm cho jetsuite
            int launch_pos = 0;  // Vị trí trong lộ trình xe tải (depot)
            int launch_node = new_truck_order[launch_pos];
            
            // Kiểm tra các chỉ số hợp lệ
            bool is_valid = true;
            
            // Kiểm tra khả năng tiếp cận với tau_prime_prime
            if (launch_node >= (int)instance->tau_prime_prime.size() || 
                customer >= (int)instance->tau_prime_prime[launch_node].size() || 
                customer >= (int)instance->tau_prime_prime.size() || 
                launch_node >= (int)instance->tau_prime_prime[customer].size()) {
                is_valid = false;
            }
            
            if (is_valid) {
                // Kiểm tra xem có thời gian di chuyển được định nghĩa
                if (instance->tau_prime_prime[launch_node][customer] <= 0 || 
                    instance->tau_prime_prime[customer][launch_node] <= 0) {
                    is_valid = false;
                }
            }
            
            if (is_valid) {
                // Thêm chuyến jetsuite - với định dạng {stage, launch_node, customer, return_node}
                jetsuite_order.push_back({0, launch_node, customer, launch_node});
                std::cout << "Created jetsuite trip: Stage 0, node " << launch_node 
                          << ", serve customer " << customer 
                          << ", return to node " << launch_node << "\n";
                added_to_alternate = true;
                
                // Bỏ qua khách hàng này trong lộ trình xe tải
                continue;
            }
        }
        
        // Nếu không thể thêm vào drone hoặc jetsuite, giữ lại trong lộ trình truck
        if (!added_to_alternate) {
            new_truck_order.push_back(customer);
        }
    }
    
    // Thêm depot cuối cùng vào lộ trình truck
    new_truck_order.push_back(truck_order.back());
    
    // Cập nhật lại truck_order
    truck_order = new_truck_order;
    
    // Tối ưu hóa lại lộ trình xe tải bằng two-opt
    two_opt(truck_order, instance->tau);
    
    std::cout << "Created " << drone_order.size() << " drone trips and " 
              << jetsuite_order.size() << " jetsuite trips\n";
              
    // In ra lộ trình xe tải sau khi chuyển đổi
    std::cout << "Updated truck route: ";
    for (int node : truck_order) {
        std::cout << node << " ";
    }
    std::cout << std::endl;
}