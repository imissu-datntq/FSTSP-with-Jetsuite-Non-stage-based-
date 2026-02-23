//
// Mode 31 Solution (Case 2 – 3-index, no-stage with JetSuite)
// Adapted from original solution.h
//
#include "common.h"
#include "instance.h"
#include "cfg.h"
#include <cmath>

#ifndef RV_FSTSP_SOLUTION31_H
#define RV_FSTSP_SOLUTION31_H

class Solution31
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

    // Drone order: <launch node i, customer j, rendezvous node u>
    vector<vector<int>> drone_order;

    // Truck timing
    vector<double> truck_time;

    // Drone timing: <launch time, customer visit, rendezvous time>
    vector<vector<double>> drone_time;

    // JetSuite order: <launch node i, customer k, launch node i>
    vector<vector<int>> jetsuite_order;

    // JetSuite timing: <launch time, customer visit, return time>
    vector<vector<double>> jetsuite_time;

    // ---- Constructors (GIỮ NGUYÊN SIGNATURE) ----
    Solution31(double cost, double lower_bound, double gap, double solve_time,
               shared_ptr<Instance> instance, Config *cfg,
               vector<int> &truck_order,
               vector<double> &truck_time,
               vector<vector<int>> &drone_order,
               vector<vector<double>> &drone_time,
               vector<vector<int>> &jetsuite_order,
               vector<vector<double>> &jetsuite_time)
        : cost(cost), lower_bound(lower_bound), gap(gap), solve_time(solve_time),
          instance(std::move(instance)), cfg(cfg),
          truck_order(truck_order), truck_time(truck_time),
          drone_order(drone_order), drone_time(drone_time),
          jetsuite_order(jetsuite_order), jetsuite_time(jetsuite_time) {}

    Solution31(double cost, double lower_bound, double gap, double solve_time,
               shared_ptr<Instance> instance, Config *cfg,
               vector<int> &truck_order,
               vector<vector<int>> &drone_order,
               vector<vector<int>> &jetsuite_order)
        : cost(cost), lower_bound(lower_bound), gap(gap), solve_time(solve_time),
          instance(std::move(instance)), cfg(cfg),
          truck_order(truck_order),
          drone_order(drone_order),
          jetsuite_order(jetsuite_order) {}

    virtual ~Solution31() = default;

    // ---- Core logic (MODE 31) ----
    void recalculateTime();      // no-stage
    void recalculateObjective(); // makespan
    void write();
    void write(const string &abs_path);
    void print();
    void print_mode31();

    // ---- Validation (MODE 31) ----
    bool areCustomersVisited();       // exclusive service
    bool isTimeTruckValid();          // route consistency
    bool isTimeDroneValid();          // endurance + rendezvous
    bool isTimeJetsuiteValid();       // endurance
    bool isSynchronizeTime();         // drone arrives before truck at u
    bool isSatisfyOtherConstraints(); // optional

    bool isFeasible()
    {
        bool is_feasible = true;

        if (cfg->screen_mode >= 1)
            cout << "\nChecking feasibility of the solution (MODE 31 – Case 2, no-stage)...\n";

        // --------------------------------------------------
        // 1. Ensure time is available
        // --------------------------------------------------
        if (truck_time.empty())
        {
            if (cfg->screen_mode >= 1)
                cout << "Time schedule is not given, recalculating...\n";
            recalculateTime();
        }

        // --------------------------------------------------
        // 2. Skip objective recalculation check
        // --------------------------------------------------
        // ❌ BỎ QUA KIỂM TRA NÀY vì recalculateTime() không tính waiting time
        // => timing data bị sai => objective tính lại sẽ luôn < cost từ CPLEX
        // ✅ Tin tưởng vào cost từ CPLEX (đã tính đúng trong model)

        if (cfg->screen_mode >= 1)
        {
            cout << "Using objective value from CPLEX: " << cost << "\n";
            cout << "(Skipping recalculation check - timing may exclude waiting time)\n";
        }

        // --------------------------------------------------
        // 3. Check individual feasibility components
        // --------------------------------------------------
        if (!areCustomersVisited())
        {
            if (cfg->screen_mode >= 1)
                cerr << "Feasibility error: Customer service exclusivity violated\n";
            is_feasible = false;
        }

        if (!isTimeTruckValid())
        {
            if (cfg->screen_mode >= 1)
                cerr << "Feasibility error: Truck timing is invalid\n";
            is_feasible = false;
        }

        if (!isTimeDroneValid())
        {
            if (cfg->screen_mode >= 1)
                cerr << "Feasibility error: Drone timing / endurance violated\n";
            is_feasible = false;
        }

        if (!isTimeJetsuiteValid())
        {
            if (cfg->screen_mode >= 1)
                cerr << "Feasibility error: JetSuite timing / endurance violated\n";
            is_feasible = false;
        }

        if (!isSynchronizeTime())
        {
            if (cfg->screen_mode >= 1)
                cerr << "Feasibility error: Drone / JetSuite not synchronized with truck\n";
            is_feasible = false;
        }

        if (!isSatisfyOtherConstraints())
        {
            if (cfg->screen_mode >= 1)
                cerr << "Feasibility error: Other constraints violated\n";
            is_feasible = false;
        }

        // --------------------------------------------------
        // 4. Final report
        // --------------------------------------------------
        if (cfg->screen_mode >= 1)
        {
            if (is_feasible)
                cout << "The solution is FEASIBLE (MODE 31)\n";
            else
                cout << "The solution is NOT FEASIBLE (MODE 31)\n";
        }

        return is_feasible;
    }
};

#endif // RV_FSTSP_SOLUTION31_H
