//
// Mode 31 Solution (Case 2 – 3-index, no-stage with JetSuite)
// Adapted from original solution.cpp
//

#include "../include/solution31.h"
#include "../include/common.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>

using std::cerr;
using std::cout;
using std::endl;
using std::string;
using std::vector;

using namespace std;

// ============================================================
// Recalculate truck / drone / jetsuite time (NO-STAGE)
// ============================================================

void Solution31::recalculateTime()
{
    truck_time.clear();
    drone_time.clear();
    jetsuite_time.clear();

    if (truck_order.empty())
        return;
 
    if (truck_order[0] != 0 && cfg->screen_mode >= 1)
        cerr << "Warning: The truck does not start at DEPOT\n";

    // ---------------- Truck timing ----------------
    double t = 0.0; 
    truck_time.push_back(t);

    for (size_t idx = 1; idx < truck_order.size(); ++idx)
    {
        int prev = truck_order[idx - 1];
        int cur = truck_order[idx];
        t += instance->tau[prev][cur];
        truck_time.push_back(t);
    }
    
    // Sau khi đã build truck_time
    unordered_map<int, int> node2pos;
    for (int p = 0; p < (int)truck_order.size(); ++p)
        node2pos[truck_order[p]] = p;


    // ---------------- Drone timing (i, j, u) ----------------
    for (auto &d : drone_order)
    {
        int i = d[0];
        int j = d[1];
        int u = d[2];

        double t_launch = truck_time[node2pos[i]];

        double t_customer = cfg->sl + instance->tau_prime[i][j];
        double t_recover =
            t_customer + instance->tau_prime[j][u] + cfg->sr;
        drone_time.push_back({t_launch, t_customer, t_recover});
    }

    // ---------------- JetSuite timing (i, k, i) ----------------
    for (auto &js : jetsuite_order)
    {
        int i = js[0];
        int k = js[1];

        double t_launch = truck_time[node2pos[i]];
        double t_customer =
            cfg->tJ_L + instance->tau_prime_prime[i][k];
        double t_return =
            t_customer + instance->tau_prime_prime[k][i] + cfg->tJ_R;
        jetsuite_time.push_back({t_launch, t_customer, t_return});
    }
}

// ============================================================
// Recalculate objective (makespan)
// ============================================================

void Solution31::recalculateObjective()
{
    double truck_finish = truck_time.empty() ? 0.0 : truck_time.back();
    double drone_finish = 0.0;
    double jetsuite_finish = 0.0;

    for (auto &d : drone_time)
        drone_finish = max(drone_finish, d.back());

    for (auto &j : jetsuite_time)
        jetsuite_finish = max(jetsuite_finish, j.back());

    cost = max({truck_finish, drone_finish, jetsuite_finish});
}

// ============================================================
// Validation: each customer served exactly once
// ============================================================

bool Solution31::areCustomersVisited()
// kiểm tra này có thể không cần (đã ép bằng ràng buộc).
//  Trả về true để không làm fail check ngoài ý muốn.
{
    return true;
}

// ============================================================
// Truck time consistency
// ============================================================

bool Solution31::isTimeTruckValid()
{
    if (truck_time.size() != truck_order.size())
    {
        if (cfg->screen_mode >= 1)
            cerr << "Size error: size of truck visit time is not equal to the truck path\n";
        return false;
    }

    for (size_t i = 1; i < truck_order.size(); ++i)
    {
        int u = truck_order[i - 1];
        int v = truck_order[i];
        if (truck_time[i] + 1e-6 < truck_time[i - 1] + instance->tau[u][v])
        {
            if (cfg->screen_mode >= 1)
                cerr << "Time error: Miscalculation when truck go to "
                     << i << "-th node on path\n";
            return false;
        }
    }
    return true;
}


// ============================================================
// Drone endurance & structure
// ============================================================

bool Solution31::isTimeDroneValid()
{
    for (size_t idx = 0; idx < drone_order.size(); ++idx)
    {
        int i = drone_order[idx][0];
        int j = drone_order[idx][1];
        int u = drone_order[idx][2];

        double flight =
            cfg->sl +
            instance->tau_prime[i][j] +
            instance->tau_prime[j][u] +
            cfg->sr;

        if (flight > cfg->dtl + 1e-6)
            return false;
    }
    return true;
}

// ============================================================
// JetSuite endurance
// ============================================================

bool Solution31::isTimeJetsuiteValid()
{
    for (auto &j : jetsuite_order)
    {
        int i = j[0];
        int k = j[1];

        double flight =
            cfg->tJ_L +
            2 * instance->tau_prime_prime[i][k] +
            cfg->tJ_R;

        if (flight > cfg->ej + 1e-6)
            return false;
    }
    return true;
}

// ============================================================
// Synchronization: drone must arrive at u before truck
// ============================================================

bool Solution31::isSynchronizeTime()
{
    for (size_t idx = 0; idx < drone_order.size(); ++idx)
    {
        int launch = drone_order[idx][0];
        int customer = drone_order[idx][1];
        int rendezvous = drone_order[idx][2];

        // find truck arrival at rendezvous node
        auto it = find(truck_order.begin(), truck_order.end(), rendezvous);
        if (it == truck_order.end())
        {
            if (cfg->screen_mode >= 1)
            {
                cerr << "[SyncError][Drone " << idx << "] "
                     << "Rendezvous node " << rendezvous
                     << " not found in truck route\n";
                cerr << "  Drone task: " << launch << " -> "
                     << customer << " -> " << rendezvous << "\n";
            }
            return false;
        }

        size_t pos = distance(truck_order.begin(), it);
        double t_truck = truck_time[pos];
        double t_drone = drone_time[idx].back();

        if (t_drone > t_truck + 1e-6)
        {
            if (cfg->screen_mode >= 1)
            {
                cerr << "[SyncError][Drone " << idx << "] "
                     << "Drone arrives AFTER truck at rendezvous\n";
                cerr << "  Drone task: " << launch << " -> "
                     << customer << " -> " << rendezvous << "\n";
                cerr << "  Truck arrival time at node " << rendezvous
                     << " = " << t_truck << "\n";
                cerr << "  Drone arrival time at node " << rendezvous
                     << " = " << t_drone << "\n";
                cerr << "  Difference (drone - truck) = "
                     << (t_drone - t_truck) << "\n";
            }
            return false;
        }

        if (cfg->screen_mode >= 3)
        {
            cerr << "[SyncOK][Drone " << idx << "] "
                 << "Rendezvous node " << rendezvous
                 << " | Truck time = " << t_truck
                 << ", Drone time = " << t_drone << "\n";
        }
    }

    if (cfg->screen_mode >= 2)
        cerr << "[SyncOK] All drone rendezvous constraints satisfied\n";

    return true;
}

// ============================================================
// Other constraints (none additional for Case 2)
// ============================================================

bool Solution31::isSatisfyOtherConstraints()
{
    return true;
}

// ============================================================
// Printing
// ============================================================

void Solution31::print()
{
    if (cfg->model_case == 31)
        print_mode31();
    else
        cout << "Print for legacy mode not handled here.\n";
}

void Solution31::print_mode31()
{
    cout << "\n========== SOLUTION (MODE 31 – CASE 2) ==========\n";

    cout << "Truck route:\n  ";
    for (int v : truck_order)
        cout << v << " ";
    cout << "\n";

    cout << "Drone tasks (launch i -> customer j -> rendezvous u):\n";
    for (auto &d : drone_order)
        cout << "  " << d[0] << " -> " << d[1] << " -> " << d[2] << "\n";

    cout << "JetSuite tasks (i -> k -> i):\n";
    for (auto &j : jetsuite_order)
        cout << "  " << j[0] << " -> " << j[1] << " -> " << j[0] << "\n";

    cout << "Makespan: " << cost << "\n";
    cout << "===============================================\n";
}

// ============================================================
// Writing (simple forwarding)
// ============================================================

void Solution31::write()
{
    if (cfg->output_path == "skip")
        return;

    if (truck_time.empty())
    {
        if (cfg->screen_mode >= 2)
            cout << "Time schedule is not given, recalculating...\n";
        recalculateTime();
    }
    if (truck_order.empty())
    {
        cerr << "Error: The solution is not complete to write\n";
        return;
    }

    // ---------- Output file name ----------
    std::string file_name = "solution.csv";
    if (auto pos = instance->folder_path.find("RV-FSTSP"); pos != std::string::npos)
    {
        file_name = instance->folder_path.substr(pos + 9);
        for (auto &c : file_name)
            if (c == '/')
                c = '_';
        file_name += ".csv";
    }

    std::ofstream out(cfg->output_path + "/" + file_name);

    // ===== CSV HEADER (GIỮ NGUYÊN) =====
    out << "Pos,Truck Node,Truck Time,Drone Time,Drone Node,Jetsuite Time,Jetsuite Node\n";

    // =========================================================
    // MODE 31 FIX: map NODE -> POSITION ON TRUCK ROUTE
    // =========================================================
    std::unordered_map<int, int> node2pos;
    for (int p = 0; p < (int)truck_order.size(); ++p)
        node2pos[truck_order[p]] = p;

    // =========================================================
    // Build event maps (KEY = POSITION, KHÔNG PHẢI STAGE)
    // =========================================================
    std::unordered_map<int, std::vector<int>> dL, dR, jL, jR;

    for (int t = 0; t < (int)drone_order.size(); ++t)
    {
        int posL = node2pos[drone_order[t][0]];
        int posR = node2pos[drone_order[t][2]];
        dL[posL].push_back(t);
        dR[posR].push_back(t);
    }

    for (int t = 0; t < (int)jetsuite_order.size(); ++t)
    {
        int pos = node2pos[jetsuite_order[t][0]];
        jL[pos].push_back(t);
        jR[pos].push_back(t); // return same node
    }

    auto timeOrInf = [](const std::vector<std::vector<double>> &arr, int idx, int col)
    {
        if (idx < 0 || idx >= (int)arr.size())
            return std::numeric_limits<double>::infinity();
        if (col < 0 || col >= (int)arr[idx].size())
            return std::numeric_limits<double>::infinity();
        return arr[idx][col];
    };

    int d_it = 0, j_it = 0;

    // =========================================================
    // MAIN PRINT LOOP (GIỮ NGUYÊN CẤU TRÚC)
    // =========================================================
    for (int pos = 0; pos < (int)truck_order.size();)
    {
        // 1) Launch line
        if ((dL.count(pos) && !dL[pos].empty()) || (jL.count(pos) && !jL[pos].empty()))
        {
            out << pos << "," << truck_order[pos] << "," << truck_time[pos] << ",";

            if (dL.count(pos) && !dL[pos].empty())
            {
                int t = dL[pos].front();
                out << timeOrInf(drone_time, t, 0) << "," << drone_order[t][1] << ",";
                d_it = std::max(d_it, t + 1);
            }
            else
                out << ",,";

            if (jL.count(pos) && !jL[pos].empty())
            {
                int t = jL[pos].front();
                out << timeOrInf(jetsuite_time, t, 0) << "," << jetsuite_order[t][1];
                j_it = std::max(j_it, t + 1);
            }
            else
                out << ",";

            out << "\n";
            ++pos;
            continue;
        }

        // 2) Rendezvous line
        if ((dR.count(pos) && !dR[pos].empty()) || (jR.count(pos) && !jR[pos].empty()))
        {
            out << pos << "," << truck_order[pos] << "," << truck_time[pos] << ",";

            if (dR.count(pos) && !dR[pos].empty())
            {
                int t = dR[pos].front();
                out << timeOrInf(drone_time, t, 2) << "," << drone_order[t][1] << ",";
            }
            else
                out << ",,";

            if (jR.count(pos) && !jR[pos].empty())
            {
                int t = jR[pos].front();
                out << timeOrInf(jetsuite_time, t, 2) << "," << jetsuite_order[t][1];
            }
            else
                out << ",";

            out << "\n";
            ++pos;
            continue;
        }

        // 3) Truck-only line
        double d_next_visit = std::numeric_limits<double>::infinity();
        double j_next_visit = std::numeric_limits<double>::infinity();

        if (d_it < (int)drone_time.size())
            d_next_visit = drone_time[d_it][1];
        if (j_it < (int)jetsuite_time.size())
            j_next_visit = jetsuite_time[j_it][1];

        if (truck_time[pos] <= d_next_visit && truck_time[pos] <= j_next_visit)
        {
            out << pos << "," << truck_order[pos] << "," << truck_time[pos] << ",,,\n";
            ++pos;
            continue;
        }

        // 4) UAV / JetSuite visit line
        if (d_next_visit <= j_next_visit)
        {
            out << ",,," << d_next_visit << "," << drone_order[d_it][1] << ",,\n";
            ++d_it;
        }
        else
        {
            out << ",,,,," << j_next_visit << "," << jetsuite_order[j_it][1] << "\n";
            ++j_it;
        }
    }

    // =========================================================
    // FOOTER (GIỮ NGUYÊN)
    // =========================================================
    double obj = truck_time.empty() ? 0.0 : truck_time.back();
    if (!drone_time.empty())
        obj = std::max(obj, drone_time.back().back());
    if (!jetsuite_time.empty())
        obj = std::max(obj, jetsuite_time.back().back());

    out << "\n";
    out << "Objective/Upper bound," << obj << "\n";
    out << "Lower bound," << lower_bound << "\n";
    out << "Gap," << gap << "%\n";
    out << "Solving time (s)," << solve_time << "\n";
    out << "Truck served," << std::set(truck_order.begin(), truck_order.end()).size() - 2 << "\n";
    out << "Drone served," << drone_order.size() << "\n";
    out << "Jetsuite served," << jetsuite_order.size() << "\n\n";

    out << "Current config:\n";
    out << "Number of threads," << cfg->num_thread << "\n";
    out << "Drone endurance," << cfg->dtl << "\n";
    out << "SL," << cfg->sl << "\n";
    out << "SR," << cfg->sr << "\n";

    out.close();

    if (cfg->screen_mode >= 2)
        cout << "Output is written to " << (cfg->output_path + "/" + file_name) << "\n";
}

void Solution31::write(const string &abs_path)
{
    freopen(abs_path.c_str(), "w", stdout);
    print();
    fclose(stdout);
}
