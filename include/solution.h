//
// Created by who on 31/07/2024. For a closer examination of the result, refer to this
//
#include "common.h"
#include "instance.h"
#include "cfg.h"
#include <cmath>

#ifndef RV_FSTSP_SOLUTION_H
#define RV_FSTSP_SOLUTION_H

class Solution
{
public:
    double cost;
    double lower_bound;
    double gap;
    double solve_time;
    shared_ptr<Instance> instance;
    Config *cfg;

    // Truck order
    vector<int> truck_order;

    // Drone order: <launch stage, customer, recovery stage>
    vector<vector<int>> drone_order;

    // Truck timing
    vector<double> truck_time;

    // Drone timing: <launch time, customer visit, recovery time>
    vector<vector<double>> drone_time;

    // **Jetsuite support**
    vector<vector<int>> jetsuite_order;   // <launch stage, customer, recovery>
    vector<vector<double>> jetsuite_time; // <launch time, customer visit, recovery time>

    // **Constructors**
    Solution(double cost, double lower_bound, double gap, double solve_time, shared_ptr<Instance> instance, Config *cfg,
             vector<int> &truck_order, vector<double> &truck_time, vector<vector<int>> &drone_order, vector<vector<double>> &drone_time,
             vector<vector<int>> &jetsuite_order, vector<vector<double>> &jetsuite_time) : cost(cost), instance(std::move(instance)), lower_bound(lower_bound), gap(gap), solve_time(solve_time),
                                                                                           cfg(cfg), truck_order(truck_order), truck_time(truck_time), drone_order(drone_order), drone_time(drone_time),
                                                                                           jetsuite_order(jetsuite_order), jetsuite_time(jetsuite_time) {}

    Solution(double cost, double lower_bound, double gap, double solve_time, shared_ptr<Instance> instance, Config *cfg,
             vector<int> &truck_order, vector<vector<int>> &drone_order, vector<vector<int>> &jetsuite_order) : cost(cost), instance(std::move(instance)), lower_bound(lower_bound), gap(gap), solve_time(solve_time), cfg(cfg),
                                                                                                                truck_order(truck_order), drone_order(drone_order), jetsuite_order(jetsuite_order) {}

    virtual ~Solution() = default;

    void recalculateTime();
    void recalculateObjective();
    void write();
    void write(const string &abs_path); // Add this line
    void print();                       // Add this line
    // Validation functions
    bool areCustomersVisited();
    bool isTimeTruckValid();
    bool isTimeDroneValid();
    bool isSynchronizeTime();
    bool isSatisfyOtherConstraints();
    bool isTimeJetsuiteValid(); // **New function**

    bool isFeasible()
    {
        bool is_feasible = true;
        if (cfg->screen_mode >= 1)
            cout << "\nChecking the feasibility of the solution...\n";

        if (truck_time.empty())
        {
            if (cfg->screen_mode >= 1)
                cout << "Time schedule is not given, recalculating...\n";
            recalculateTime();
        }

        double recalculated_obj;
        if (drone_time.empty() && jetsuite_time.empty())
            recalculated_obj = truck_time.back();
        else if (jetsuite_time.empty())
            recalculated_obj = max(truck_time.back(), drone_time.back().back());
        else
            recalculated_obj = max({truck_time.back(), drone_time.back().back(), jetsuite_time.back().back()});

        if (fabs(recalculated_obj - cost) > 0.01)
        {
            if (cfg->screen_mode >= 1)
            {
                cerr << "Error: Time step of truck, drone, or jetsuite is not synchronized or calculated properly\n";
                cout << "Current objective: " << cost << "\n";
                cout << "Recalculated objective: " << recalculated_obj << "\n";
            }
            is_feasible = false;
        }
        else
        {
            if (cfg->screen_mode >= 1)
                cout << "The objective matches the recalculated objective\n";
        }

        // Check feasibility of all elements
        if (!(areCustomersVisited() && isTimeTruckValid() && isTimeDroneValid() && isTimeJetsuiteValid() &&
              isSynchronizeTime() && isSatisfyOtherConstraints()))
        {
            is_feasible = false;
        }

        if (is_feasible)
        {
            if (cfg->screen_mode >= 1)
                cout << "The solution is feasible\n";
            return true;
        }
        return false;
    }
};

#endif // RV_FSTSP_SOLUTION_H
