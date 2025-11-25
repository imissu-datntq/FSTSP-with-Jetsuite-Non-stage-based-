#ifndef RV_FSTSP_CMSA_H
#define RV_FSTSP_CMSA_H

#include <utility>
#include <ilcplex/ilocplex.h>
#include <chrono>
#include <random>
#include <numeric>
#include "common.h"
#include "instance.h"
#include "cfg.h"
#include "solution.h"
#include <vector>
#include <memory>
#include <fstream>
#include <map>

class CMSA
{
public:
    // General parameters
    double computation_time;
    double cplex_time;
    int num_sols;
    int age_limit;
    int top_k;             // top k for random greedy choice
    double drone_ratio;    // Ratio of customers sampling for drone
    double jetsuite_ratio; // Ratio of customers sampling for Jetsuite (NEW)
    double adapt_threshold;
    std::shared_ptr<Instance> instance;
    Config *cfg;
    
    // Thêm các biến để lưu kết quả
    double best_objective_value;
    int iteration_count;
    int solution_count;
    int num_iterations;
    int num_solutions_evaluated;
    std::vector<int> truck_route;
    double total_truck_distance;
    std::string output_file; // Thêm biến để lưu đường dẫn file output
    
    // Biến lưu hoạt động của drone và jetsuit
    std::vector<std::vector<int>> drone_operations;
    std::vector<std::vector<int>> jetsuite_operations;
    
    // Decision variables for detailed output
    std::vector<int> truck_nodes;  // Nodes visited by truck (X variables)
    std::vector<std::vector<int>> truck_arcs; // Arcs used by truck (x variables)
    std::vector<int> drone_customers; // Customers served by drone (phi variables)
    std::vector<std::vector<int>> drone_launches; // Drone launch operations (Y variables)
    std::vector<std::vector<int>> drone_recoveries; // Drone recovery operations (W variables)
    std::vector<int> jetsuit_customers; // Customers served by jetsuit (phiJetsuit variables)
    std::vector<std::vector<int>> jetsuit_ops; // Jetsuit operations (U variables)

    // Constructor
    CMSA(double computation_time, double cplex_time, int num_sols, int age_limit, int top_k, double drone_ratio,
         double jetsuite_ratio, double adapt_threshold, std::shared_ptr<Instance> instance, Config *cfg)
        : computation_time(computation_time),
          cplex_time(cplex_time),
          num_sols(num_sols),
          age_limit(age_limit),
          top_k(top_k),
          drone_ratio(drone_ratio),
          jetsuite_ratio(jetsuite_ratio),
          adapt_threshold(adapt_threshold),
          instance(std::move(instance)),
          cfg(cfg),
          iteration_count(0),
          solution_count(0),
          num_iterations(0),
          num_solutions_evaluated(0),
          best_objective_value(0.0),
          total_truck_distance(0.0)
    {
        seed = std::chrono::system_clock::now().time_since_epoch().count();
        generator = std::default_random_engine(seed);
        standard_distribution = std::uniform_real_distribution<double>(0.0, 1.0);
    };

    CMSA(std::shared_ptr<Instance> instance, Config *cfg)
        : computation_time(600),
          cplex_time(30),
          num_sols(5),
          age_limit(5),
          top_k(4),
          drone_ratio(0.1),
          jetsuite_ratio(0.05), // Default ratio for Jetsuite
          adapt_threshold(1.05),
          instance(std::move(instance)),
          cfg(cfg),
          iteration_count(0),
          solution_count(0),
          num_iterations(0),
          num_solutions_evaluated(0),
          best_objective_value(0.0),
          total_truck_distance(0.0)
    {
        seed = std::chrono::system_clock::now().time_since_epoch().count();
        generator = std::default_random_engine(seed);
        standard_distribution = std::uniform_real_distribution<double>(0.0, 1.0);
    };

    Solution solve();

    void initialize();
    Solution generate_initial_solution();
    void update_age(Solution &s, bool increase);
    std::vector<int> generate_relaxed_solution();
    Solution solve_cplex(double r_limit);
    
    // Thêm các hàm mới
    void print_solution();
    void print_debug_info();
    void create_valid_trips(std::vector<int>& truck_order,
                          std::vector<std::vector<int>>& drone_order,
                          std::vector<std::vector<int>>& jetsuite_order);
    void collect_decision_variables(IloCplex& cplex, 
                                   IloArray<IloBoolVarArray>& X, 
                                   IloArray<IloArray<IloBoolVarArray>>& x, 
                                   IloBoolVarArray& phi, 
                                   IloArray<IloArray<IloBoolVarArray>>& Y, 
                                   IloArray<IloArray<IloBoolVarArray>>& W, 
                                   IloBoolVarArray& phiJetsuite,
                                   IloArray<IloArray<IloBoolVarArray>>& U);

private:
    // Random factors
    unsigned seed;
    std::default_random_engine generator;
    std::uniform_real_distribution<double> standard_distribution;

    // Age for Truck, Drone, and Jetsuite decisions
    vector<vector<int>> arc_age;
    vector<int> customer_age;
    vector<vector<vector<int>>> Y_age;
    vector<vector<vector<int>>> W_age;
    vector<vector<vector<int>>> U_age; // Age tracking for Jetsuite (NEW)
    
    // Nearest -> farthest customers for each customer
    vector<vector<int>> near_customers;
    // Weight for Drone and Jetsuite customers
    vector<double> customer_weight;
    vector<double> jetsuite_weight; // Weight for Jetsuite (NEW)

    std::vector<double> customer_weight_jetsuite;
    std::vector<Solution> sol_pool;

    void precompute();

    // Random helper functions
    int get_random_number(int max)
    {
        double rnum = standard_distribution(generator);
        int num = int(double(max) * rnum);
        if (num == max)
            num = num - 1;
        return num;
    };

    int get_random_number(int min, int max)
    {
        double rnum = standard_distribution(generator);
        int num = int(double(max + 1 - min) * rnum) + min;
        return num;
    };

    int get_random_item(vector<int> &items)
    {
        if (items.empty())
            cout << "Error: empty items\n";
        int idx = get_random_number(items.size());
        return items[idx];
    };

    vector<int> sample_index_with_weight(vector<double> &weight, int no_sample)
    {
        vector<int> outcomes(no_sample, -1);
        vector<bool> chosen(weight.size(), false);
        double sum_weight = std::accumulate(weight.begin(), weight.end(), 0.0);

        for (int &outcome : outcomes)
        {
            double rnum = standard_distribution(generator) * sum_weight;
            for (int i = 0; i < weight.size(); i++)
            {
                if (!chosen[i])
                {
                    rnum -= weight[i];
                    if (rnum <= 0)
                    {
                        outcome = i;
                        chosen[i] = true;
                        sum_weight -= weight[i];
                        break;
                    }
                }
            }
        }
        return outcomes;
    }

    class Random {
    private:
        std::mt19937 generator;  // generator phải ở TRONG class Random
        
    public:
        Random() : generator(std::random_device{}()) {}
        
        // Định nghĩa hàm get_random_number
        int get_random_number(int min, int max) {
            std::uniform_int_distribution<int> distribution(min, max);
            return distribution(generator);
        }
        
        // Định nghĩa hàm get_random_item cho vector<int>
        int get_random_item(std::vector<int> &items) {
            if (items.empty()) return -1;
            return items[get_random_number(0, items.size() - 1)];
        }
        
        // Định nghĩa hàm get_random_item cho vector<Solution>
        Solution get_random_item(std::vector<Solution> &items) {
            if (items.empty()) throw std::runtime_error("Empty solution pool");
            return items[get_random_number(0, items.size() - 1)];
        }
    };
    
    Random randomizer;
};

#endif //RV_FSTSP_CMSA_H
