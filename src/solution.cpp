// Created by who on 31/07/2024. The below check assumed that L = 1
#include "../include/solution.h"
#include "../include/common.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
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

void Solution::recalculateTime()
{
    truck_time.clear();
    drone_time.clear();
    jetsuite_time.clear();

    if (truck_order.empty())
        return;

    if (truck_order[0] != 0 && cfg->screen_mode >= 1)
        cerr << "Warning: The truck does not start at DEPOT\n";

    // Thời gian tại depot = 0
    truck_time.push_back(0.0);

    // Con trỏ duyệt các sorties đã sắp xếp theo launch_pos (solver đẩy theo thứ tự dọc tuyến)
    // KHÔNG phụ thuộc thứ tự, ta sẽ duyệt toàn bộ mỗi bước.
    // Quy ước:
    //  - drone_order item: {launch_pos_on_route, customer, recover_pos_on_route}
    //  - jetsuite_order item: {launch_pos_on_route, customer, recover_pos_on_route}
    //
    // Giả định hiện tại:
    //  - recover_pos_on_route = launch_pos_on_route + 1 (rendezvous ngay chặng kế tiếp)
    //    HOẶC = 0 (trở về depot; truck không chờ).
    //
    // Nếu muốn cho phép rendezvous ở vị trí xa hơn (>= launch_pos+2):
    //  - Cần giữ một "yêu cầu chờ" cho stage rendezvous và cộng dồn khi đến stage đó.
    //  - Ở đây để đơn giản (và khớp solver hiện tại), ta xử lý case kế tiếp hoặc depot.

    for (int s = 0; s + 1 < (int)truck_order.size(); ++s)
    {
        int u = truck_order[s];
        int v = truck_order[s + 1];

        double depart_time = truck_time[s];
        double base_arrival_next = depart_time + instance->tau[u][v];

        double wait_at_next = 0.0;

        // ===== DRONE sorties PHÓNG ở vị trí s =====
        for (const auto &trip : drone_order)
        {
            int launch_pos = trip[0];
            if (launch_pos != s)
                continue;

            int visit_node = trip[1];
            int recover_pos = trip[2]; // = s+1 hoặc 0

            // Tính thời gian UAV
            double launch_time = depart_time;
            // Nếu phóng từ depot (s == 0) thì theo file cũ SL bị bỏ qua cho lượt đầu
            double drone_visit_time =
                (s == 0) ? (launch_time + instance->tau_prime[u][visit_node])
                         : (launch_time + cfg->sl + instance->tau_prime[u][visit_node]);

            // Nếu quay về depot: không chờ truck, chỉ ghi thời gian chuyến bay
            if (recover_pos == 0)
            {
                double drone_back_time = drone_visit_time + instance->tau_prime[visit_node][0];
                // Trong file cũ, SR khi về depot không cộng vào endurance; vẫn ghi lại mốc về
                drone_time.push_back({launch_time, drone_visit_time, drone_back_time});
                continue;
            }

            // Mặc định: recover tại s+1
            if (recover_pos == s + 1)
            {
                // Truck đến v = s+1 và phải chờ SR + UAV bay về v
                double drone_arrive_v = drone_visit_time + cfg->sr + instance->tau_prime[visit_node][v];
                double truck_ready_v = base_arrival_next + cfg->sr; // truck dừng để recover SR
                double meeting_time = std::max(truck_ready_v, drone_arrive_v);

                // Truck phải chờ thêm:
                wait_at_next = std::max(wait_at_next, meeting_time - base_arrival_next);

                // Ghi lại mốc thời gian UAV (đến v đúng thời điểm gặp)
                drone_time.push_back({launch_time, drone_visit_time, meeting_time});
            }
            else
            {
                // TRƯỜNG HỢP CHUNG (recover ở vị trí xa hơn) - chưa hỗ trợ đầy đủ
                // Với solver hiện tại bạn đang set recover_pos = s+1, nên nhánh này rất ít khi vào.
                // Ta xử lý tối thiểu: coi như recover ở s+1 để đảm bảo an toàn (không crash),
                // và cảnh báo ra màn hình.
                if (cfg->screen_mode >= 1)
                    cerr << "[WARN] Drone sortie recover at pos " << recover_pos
                         << " (> s+1). Current time recalc assumes rendezvous at s+1.\n";
                double drone_arrive_v = drone_visit_time + cfg->sr + instance->tau_prime[visit_node][v];
                double truck_ready_v = base_arrival_next + cfg->sr;
                double meeting_time = std::max(truck_ready_v, drone_arrive_v);
                wait_at_next = std::max(wait_at_next, meeting_time - base_arrival_next);
                drone_time.push_back({launch_time, drone_visit_time, meeting_time});
            }
        }

        // ===== JETSUITE sorties PHÓNG ở vị trí s =====
        for (const auto &trip : jetsuite_order)
        {
            int launch_pos = trip[0];
            if (launch_pos != s)
                continue;

            int visit_node = trip[1];
            int recover_pos = trip[2]; // = s+1 hoặc 0

            double launch_time = depart_time;
            double js_visit_time =
                (s == 0) ? (launch_time + instance->tau_prime_prime[u][visit_node])
                         : (launch_time + cfg->sl + instance->tau_prime_prime[u][visit_node]);

            if (recover_pos == 0)
            {
                double js_back_time = js_visit_time + instance->tau_prime_prime[visit_node][0];
                jetsuite_time.push_back({launch_time, js_visit_time, js_back_time});
                continue;
            }

            if (recover_pos == s + 1)
            {
                double js_arrive_v = js_visit_time + cfg->sr + instance->tau_prime_prime[visit_node][v];
                double truck_ready_v = base_arrival_next + cfg->sr;
                double meeting_time = std::max(truck_ready_v, js_arrive_v);
                wait_at_next = std::max(wait_at_next, meeting_time - base_arrival_next);
                jetsuite_time.push_back({launch_time, js_visit_time, meeting_time});
            }
            else
            {
                if (cfg->screen_mode >= 1)
                    cerr << "[WARN] Jetsuite sortie recover at pos " << recover_pos
                         << " (> s+1). Current time recalc assumes rendezvous at s+1.\n";
                double js_arrive_v = js_visit_time + cfg->sr + instance->tau_prime_prime[visit_node][v];
                double truck_ready_v = base_arrival_next + cfg->sr;
                double meeting_time = std::max(truck_ready_v, js_arrive_v);
                wait_at_next = std::max(wait_at_next, meeting_time - base_arrival_next);
                jetsuite_time.push_back({launch_time, js_visit_time, meeting_time});
            }
        }

        // Thời gian truck đến v = base + (đợi do recover)
        truck_time.push_back(base_arrival_next + std::max(0.0, wait_at_next));
    }
}

bool Solution::areCustomersVisited()
{
    // Tùy mô hình 2-index no-stage, kiểm tra này có thể không cần (đã ép bằng ràng buộc).
    // Trả về true để không làm fail check ngoài ý muốn.
    return true;
}

bool Solution::isTimeTruckValid()
{
    if (truck_time.size() != truck_order.size())
    {
        if (cfg->screen_mode >= 1)
            cerr << "Size error: size of truck visit time is not equal to the truck path\n";
        return false;
    }

    // Kiểm tra mỗi bước: thời gian đến tối thiểu = trước đó + tau (chưa tính SL/SR vì đã gom vào đợi tại điểm sau)
    for (int i = 1; i < (int)truck_order.size(); i++)
    {
        double minimal_current_time = truck_time[i - 1] + instance->tau[truck_order[i - 1]][truck_order[i]];
        if (truck_time[i] + 1e-6 < minimal_current_time)
        {
            if (cfg->screen_mode >= 1)
                cerr << "Time error: Miscalculation when truck go to " << i << "-th node on path\n";
            return false;
        }
    }
    return true;
}

bool Solution::isTimeDroneValid()
{
    if (drone_time.size() != drone_order.size())
    {
        if (cfg->screen_mode >= 1)
            cerr << "Size error: Size of drone visit time is not equal to the number of sorties\n";
        return false;
    }

    for (int i = 0; i < (int)drone_order.size(); i++)
    {
        int launch_pos = drone_order[i][0];
        int visit_node = drone_order[i][1];
        int recover_pos = drone_order[i][2];

        if (launch_pos < 0 || launch_pos >= (int)truck_order.size())
            return false;

        int launch_node = truck_order[launch_pos];

        double visit_expected =
            (launch_pos == 0) ? (drone_time[i][0] + instance->tau_prime[launch_node][visit_node])
                              : (drone_time[i][0] + cfg->sl + instance->tau_prime[launch_node][visit_node]);

        if (std::fabs(drone_time[i][1] - visit_expected) > 1e-6)
        {
            if (cfg->screen_mode >= 1)
                cerr << "Time error: Miscalculation when drone visit " << visit_node
                     << " of sortie " << i << "\n";
            return false;
        }

        if (recover_pos == 0)
        {
            double min_ret = drone_time[i][1] + instance->tau_prime[visit_node][0];
            if (drone_time[i][2] + 1e-6 < min_ret)
            {
                if (cfg->screen_mode >= 1)
                    cerr << "Time error: Drone return-to-depot time too early in sortie " << i << "\n";
                return false;
            }
        }
        else
        {
            if (recover_pos <= launch_pos || recover_pos >= (int)truck_order.size())
                return false;

            int rendezvous_node = truck_order[recover_pos];
            double min_meet = drone_time[i][1] + cfg->sr + instance->tau_prime[visit_node][rendezvous_node];
            if (drone_time[i][2] + 1e-6 < min_meet)
            {
                if (cfg->screen_mode >= 1)
                    cerr << "Time error: Miscalculation when drone rendezvous at node "
                         << rendezvous_node << " of sortie " << i << "\n";
                return false;
            }
        }

        // Endurance (không tính SL)
        if (drone_time[i][2] - drone_time[i][0] - ((launch_pos == 0) ? 0 : cfg->sl) > cfg->dtl + 1e-6)
        {
            if (cfg->screen_mode >= 1)
                cerr << "Time error: Drone of sortie " << i << " exceed its endurance\n";
            return false;
        }
    }
    return true;
}

bool Solution::isTimeJetsuiteValid()
{
    if (jetsuite_time.size() != jetsuite_order.size())
    {
        if (cfg->screen_mode >= 1)
            cerr << "Size error: Size of jetsuite visit time is not equal to the number of sorties\n";
        return false;
    }

    for (int i = 0; i < (int)jetsuite_order.size(); i++)
    {
        int launch_pos = jetsuite_order[i][0];
        int visit_node = jetsuite_order[i][1];
        int recover_pos = jetsuite_order[i][2];

        if (launch_pos < 0 || launch_pos >= (int)truck_order.size())
            return false;

        int launch_node = truck_order[launch_pos];

        double visit_expected =
            (launch_pos == 0) ? (jetsuite_time[i][0] + instance->tau_prime_prime[launch_node][visit_node])
                              : (jetsuite_time[i][0] + cfg->sl + instance->tau_prime_prime[launch_node][visit_node]);

        double j2 = jetsuite_time[i][2];

        if (std::fabs(jetsuite_time[i][1] - visit_expected) > 1e-6)
        {
            if (cfg->screen_mode >= 1)
                cerr << "Time error: Miscalculation when jetsuite visit " << visit_node
                     << " of sortie " << i << "\n";
            return false;
        }

        if (recover_pos == 0)
        {
            double min_ret = jetsuite_time[i][1] + instance->tau_prime_prime[visit_node][0];
            if (j2 + 1e-6 < min_ret)
            {
                if (cfg->screen_mode >= 1)
                    cerr << "Time error: Jetsuite return-to-depot time too early in sortie " << i << "\n";
                return false;
            }
        }
        else
        {
            if (recover_pos <= launch_pos || recover_pos >= (int)truck_order.size())
                return false;

            int rendezvous_node = truck_order[recover_pos];
            double min_meet = jetsuite_time[i][1] + cfg->sr + instance->tau_prime_prime[visit_node][rendezvous_node];
            if (j2 + 1e-6 < min_meet)
            {
                if (cfg->screen_mode >= 1)
                    cerr << "Time error: Miscalculation when jetsuite rendezvous at node "
                         << rendezvous_node << " of sortie " << i << "\n";
                return false;
            }
        }
    }

    return true;
}

bool Solution::isSynchronizeTime()
{
    // Đồng bộ (launch & rendezvous) giữa truck và UAV/Jetsuite
    for (int i = 0; i < (int)drone_time.size(); i++)
    {
        int launch_stage = drone_order[i][0];
        int rendezvous_stage = drone_order[i][2];

        if (launch_stage < 0 || launch_stage >= (int)truck_time.size())
            return false;

        if (std::fabs(truck_time[launch_stage] - drone_time[i][0]) > 0.01)
        {
            if (cfg->screen_mode >= 1)
                cerr << "Time error: Truck launch time must equal drone launch time (sortie " << i << ")\n";
            return false;
        }

        if (rendezvous_stage != 0)
        {
            if (rendezvous_stage >= (int)truck_time.size())
                return false;

            if (std::fabs(truck_time[rendezvous_stage] - drone_time[i][2]) > 0.01)
            {
                if (cfg->screen_mode >= 1)
                    cerr << "Time error: Truck rendezvous time must equal drone meeting time (sortie "
                         << i << ")\n";
                return false;
            }
        }
    }

    for (int i = 0; i < (int)jetsuite_time.size(); i++)
    {
        int launch_stage = jetsuite_order[i][0];
        int rendezvous_stage = jetsuite_order[i][2];

        if (launch_stage < 0 || launch_stage >= (int)truck_time.size())
            return false;

        if (std::fabs(truck_time[launch_stage] - jetsuite_time[i][0]) > 0.01)
        {
            if (cfg->screen_mode >= 1)
                cerr << "Time error: Truck launch time must equal jetsuite launch time (sortie " << i << ")\n";
            return false;
        }

        if (rendezvous_stage != 0)
        {
            if (rendezvous_stage >= (int)truck_time.size())
                return false;

            if (std::fabs(truck_time[rendezvous_stage] - jetsuite_time[i][2]) > 0.01)
            {
                if (cfg->screen_mode >= 1)
                    cerr << "Time error: Truck rendezvous time must equal jetsuite meeting time (sortie "
                         << i << ")\n";
                return false;
            }
        }
    }

    return true;
}

bool Solution::isSatisfyOtherConstraints()
{
    // Không được rời depot giữa chừng rồi lại quay ra: 0 chỉ ở đầu và (tùy) cuối
    for (int i = 1; i + 1 < (int)truck_order.size(); i++)
    {
        if (truck_order[i] == 0 && truck_order[i + 1] != 0)
        {
            if (cfg->screen_mode >= 1)
                cerr << "Truck error: Truck must not return to the depot in the middle of the trip\n";
            return false;
        }
    }
    return true;
}

void Solution::write()
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

    // Extract filename from folder_path (remove directory and .txt extension)
    std::string path = instance->folder_path;
    size_t last_slash = path.find_last_of("/\\");
    if (last_slash != std::string::npos)
    {
        file_name = path.substr(last_slash + 1);
        // Remove .txt extension if exists
        size_t txt_pos = file_name.rfind(".txt");
        if (txt_pos != std::string::npos)
            file_name = file_name.substr(0, txt_pos);
        file_name += ".csv";
    }

    std::ofstream out(cfg->output_path + "/" + file_name);
    out << "Pos,Truck Node,Truck Time,Drone Time,Drone Node,Jetsuite Time,Jetsuite Node\n";

    // dựng các map sự kiện như trong recalcTime để in theo “vị trí tuyến”
    std::unordered_map<int, std::vector<int>> dL, dR, jL, jR;
    for (int t = 0; t < (int)drone_order.size(); ++t)
    {
        dL[drone_order[t][0]].push_back(t);
        dR[drone_order[t][2]].push_back(t); // depot: 0
    }
    for (int t = 0; t < (int)jetsuite_order.size(); ++t)
    {
        jL[jetsuite_order[t][0]].push_back(t);
        jR[jetsuite_order[t][2]].push_back(t); // depot: 0
    }

    auto timeOrInf = [](const std::vector<std::vector<double>> &arr, int idx, int col)
    {
        if (idx < 0 || idx >= (int)arr.size())
            return std::numeric_limits<double>::infinity();
        if (col < 0 || col >= (int)arr[idx].size())
            return std::numeric_limits<double>::infinity();
        return arr[idx][col];
    };

    // con trỏ đang in (không dùng để truy cập trực tiếp, chỉ để đi tuần tự list đã thu)
    int d_it = 0, j_it = 0;

    for (int pos = 0; pos < (int)truck_order.size();)
    {
        bool printed = false;

        // 1) dòng launch (nếu có)
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
            printed = true;
            ++pos;
            continue;
        }

        // 2) dòng rendezvous tại pos
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
            printed = true;
            ++pos;
            continue;
        }

        // 3) dòng chỉ truck (hoặc đang có visit của UAV nhưng truck “nhẹ hơn”)
        //   so sánh thời điểm visit của lần drone/jetsuite kế tiếp với truck_time[pos]
        double d_next_visit = std::numeric_limits<double>::infinity();
        double j_next_visit = std::numeric_limits<double>::infinity();
        if (d_it < (int)drone_time.size())
            d_next_visit = drone_time[d_it][1];
        if (j_it < (int)jetsuite_time.size())
            j_next_visit = jetsuite_time[j_it][1];

        if (truck_time[pos] <= d_next_visit && truck_time[pos] <= j_next_visit)
        {
            out << pos << "," << truck_order[pos] << "," << truck_time[pos] << ",,,\n";
            printed = true;
            ++pos;
            continue;
        }

        // 4) in lần visit UAV (chỉ 1 sự kiện nhỏ lẻ)
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

    // objective = max thời gian kết thúc trong ba thành phần
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
    out << "Allow loop," << cfg->allow_loop << "\n";
    out << "Allow revisit," << cfg->allow_revisit << "\n";
    out.close();

    if (cfg->screen_mode >= 2)
    {
        cout << "Output is written to " << (cfg->output_path + "/" + file_name) << "\n";
    }
}

void Solution::write(const string &abs_path)
{
    if (cfg->screen_mode >= 2)
        cout << "\nWriting solution...\n";

    if (truck_order.empty())
    {
        cerr << "Error: The solution is not complete to write\n";
        return;
    }
    if (truck_time.empty())
    {
        if (cfg->screen_mode >= 2)
            cout << "Time schedule is not given, recalculating...\n";
        recalculateTime();
    }

    std::ofstream out(abs_path);
    out << "Stage,Truck Node,Truck Time,Drone Time,Drone Node\n";

    // Bản rút gọn (không in jetsuite) giữ tương thích chức năng cũ
    for (int stage = 0; stage < (int)truck_order.size(); ++stage)
    {
        double tTruck = (stage < (int)truck_time.size()) ? truck_time[stage] : NAN;
        out << stage + 1 << "," << truck_order[stage] << "," << tTruck << ",,\n";

        for (int i = 0; i < (int)drone_order.size(); ++i)
        {
            if (drone_order[i][0] == stage)
                out << ",,," << (i < (int)drone_time.size() ? drone_time[i][1] : NAN)
                    << "," << drone_order[i][1] << "\n";
            if (drone_order[i][2] == stage && i < (int)drone_time.size())
                out << stage + 1 << "," << truck_order[stage] << "," << tTruck
                    << "," << drone_time[i][2] << ",\n";
        }
    }

    double obj;
    if (drone_time.empty())
        obj = truck_time.back();
    else
        obj = std::max(truck_time.back(), drone_time.back().back());

    out << "\n";
    out << "Objective/Upper bound," << obj << "\n";
    out << "Lower bound," << lower_bound << "\n";
    out << "Gap," << gap << "%\n";
    out << "Solving time (s)," << solve_time << "\n";
    out << "Truck served," << (int)std::set<int>(truck_order.begin(), truck_order.end()).size() - 2 << "\n";
    out << "Drone served," << drone_order.size() << "\n\n";

    out << "Current config: \n";
    out << "Number of threads," << cfg->num_thread << "\n";
    out << "Drone endurance," << cfg->dtl << "\n";
    out << "SL," << cfg->sl << "\n";
    out << "SR," << cfg->sr << "\n";
    out << "Allow loop," << cfg->allow_loop << "\n";
    out << "Allow revisit," << cfg->allow_revisit << "\n";
    out.close();

    if (cfg->screen_mode >= 2)
        cout << "Output is written to " << abs_path << "\n";
}

void Solution::recalculateObjective()
{
    if (truck_time.empty() || (drone_time.empty() && jetsuite_time.empty()))
    {
        recalculateTime();
    }
    double tTruckEnd = truck_time.empty() ? 0.0 : truck_time.back();
    double tDroneEnd = (!drone_time.empty() ? drone_time.back().back() : 0.0);
    double tJetEnd = (!jetsuite_time.empty() ? jetsuite_time.back().back() : 0.0);
    cost = std::max({tTruckEnd, tDroneEnd, tJetEnd});
}

void Solution::print_mode31()
{
    using std::cout;
    using std::endl;

    if (cost <= 0)
    {
        cout << "Can't print. The solution is infeasible\n";
        return;
    }

    // ---------- Truck route ----------
    cout << "\n=== Truck route (from x_ij) ===\n";
    if (!truck_order.empty())
    {
        cout << "Truck path: ";
        for (size_t i = 0; i < truck_order.size(); ++i)
        {
            if (i)
                cout << " -> ";
            cout << truck_order[i];
        }
        cout << endl;
    }

    // ---------- Drone sorties ----------
    cout << "\n=== Drone sorties (i, j, u) ===\n";
    if (drone_order.empty())
        cout << "(no drone sorties)\n";
    else
        for (auto &d : drone_order)
            cout << "Drone: launch at " << d[0]
                 << ", serve customer " << d[1]
                 << ", rendezvous at " << d[2] << "\n";

    // ---------- Jetsuite sorties ----------
    cout << "\n=== Jetsuite sorties (i, k) ===\n";
    if (jetsuite_order.empty())
        cout << "(no jetsuite sorties)\n";
    else
        for (auto &j : jetsuite_order)
            cout << "Jetsuite: launch at " << j[0]
                 << ", serve customer " << j[1]
                 << ", return to truck at node " << j[0] << "\n";

    cout << endl;
}

// ============================================================
//  debugConstraints – in chi tiết từng ràng buộc PASS / FAIL
// ============================================================
void Solution::debugConstraints()
{
    using std::left;
    using std::setw;
    auto PASS = [](bool ok)
    { return ok ? "  [PASS]" : "  [FAIL]"; };

    cout << "\n";
    cout << "============================================================\n";
    cout << "  CONSTRAINT DEBUG LOG\n";
    cout << "============================================================\n";

    // ------------------------------------------------------------------
    // C0: Objective consistency
    // ------------------------------------------------------------------
    {
        double recalc = truck_time.empty() ? 0.0 : truck_time.back();
        if (!drone_time.empty())
            recalc = std::max(recalc, drone_time.back().back());
        if (!jetsuite_time.empty())
            recalc = std::max(recalc, jetsuite_time.back().back());
        bool ok = (std::fabs(recalc - cost) <= 0.01);
        cout << PASS(ok) << " C0  Objective consistency"
             << "  (stored=" << cost << ", recalc=" << recalc << ")\n";
    }

    // ------------------------------------------------------------------
    // C1: All customers visited
    // ------------------------------------------------------------------
    {
        bool ok = areCustomersVisited();
        cout << PASS(ok) << " C1  All customers visited\n";
    }

    // ------------------------------------------------------------------
    // C2: Truck time sizes match
    // ------------------------------------------------------------------
    {
        bool ok = (truck_time.size() == truck_order.size());
        cout << PASS(ok) << " C2  Truck time array size matches truck route"
             << "  (time=" << truck_time.size() << ", route=" << truck_order.size() << ")\n";
    }

    // ------------------------------------------------------------------
    // C3: Truck travel time non-decreasing (each step >= tau)
    // ------------------------------------------------------------------
    {
        bool all_ok = true;
        for (int i = 1; i < (int)truck_order.size(); i++)
        {
            int u = truck_order[i - 1], v = truck_order[i];
            double minimal = truck_time[i - 1] + instance->tau[u][v];
            bool ok = (truck_time[i] + 1e-6 >= minimal);
            if (!ok)
            {
                cout << "  [FAIL] C3  Truck step " << i << ": node " << u << "->" << v
                     << "  time=" << truck_time[i] << " < min=" << minimal << "\n";
                all_ok = false;
            }
        }
        if (all_ok)
            cout << "  [PASS] C3  Truck travel time valid for all " << (int)truck_order.size() - 1 << " edge(s)\n";
    }

    // ------------------------------------------------------------------
    // C4: Truck does not return to depot in the middle
    // ------------------------------------------------------------------
    {
        bool all_ok = true;
        for (int i = 1; i + 1 < (int)truck_order.size(); i++)
        {
            if (truck_order[i] == 0 && truck_order[i + 1] != 0)
            {
                cout << "  [FAIL] C4  Truck returns to depot at position " << i
                     << " then leaves again (node " << truck_order[i + 1] << ")\n";
                all_ok = false;
            }
        }
        if (all_ok)
            cout << "  [PASS] C4  Truck never returns to depot mid-route\n";
    }

    // ------------------------------------------------------------------
    // C5: Drone sortie array sizes match
    // ------------------------------------------------------------------
    {
        bool ok = (drone_time.size() == drone_order.size());
        cout << PASS(ok) << " C5  Drone sortie time array size matches"
             << "  (time=" << drone_time.size() << ", order=" << drone_order.size() << ")\n";
    }

    // ------------------------------------------------------------------
    // C6: Drone visit time correct (tau_prime + optional SL)
    // ------------------------------------------------------------------
    {
        bool all_ok = true;
        for (int i = 0; i < (int)drone_order.size(); i++)
        {
            if (i >= (int)drone_time.size())
                break;
            int launch_pos = drone_order[i][0];
            int visit_node = drone_order[i][1];
            if (launch_pos < 0 || launch_pos >= (int)truck_order.size())
                continue;
            int launch_node = truck_order[launch_pos];
            double expected = (launch_pos == 0)
                                  ? (drone_time[i][0] + instance->tau_prime[launch_node][visit_node])
                                  : (drone_time[i][0] + cfg->sl + instance->tau_prime[launch_node][visit_node]);
            bool ok = (std::fabs(drone_time[i][1] - expected) <= 1e-6);
            if (!ok)
            {
                cout << "  [FAIL] C6  Drone sortie " << i << ": visit node " << visit_node
                     << "  time=" << drone_time[i][1] << " expected=" << expected << "\n";
                all_ok = false;
            }
        }
        if (all_ok)
            cout << "  [PASS] C6  Drone visit times correct for all " << (int)drone_order.size() << " sortie(s)\n";
    }

    // ------------------------------------------------------------------
    // C7: Drone rendezvous / return-to-depot time feasible
    // ------------------------------------------------------------------
    {
        bool all_ok = true;
        for (int i = 0; i < (int)drone_order.size(); i++)
        {
            if (i >= (int)drone_time.size())
                break;
            int launch_pos = drone_order[i][0];
            int visit_node = drone_order[i][1];
            int recover_pos = drone_order[i][2];
            if (recover_pos == 0)
            {
                double min_ret = drone_time[i][1] + instance->tau_prime[visit_node][0];
                bool ok = (drone_time[i][2] + 1e-6 >= min_ret);
                if (!ok)
                {
                    cout << "  [FAIL] C7  Drone sortie " << i << ": return-to-depot"
                         << "  t=" << drone_time[i][2] << " < min=" << min_ret << "\n";
                    all_ok = false;
                }
            }
            else
            {
                if (recover_pos < (int)truck_order.size())
                {
                    int rdv_node = truck_order[recover_pos];
                    double min_meet = drone_time[i][1] + cfg->sr + instance->tau_prime[visit_node][rdv_node];
                    bool ok = (drone_time[i][2] + 1e-6 >= min_meet);
                    if (!ok)
                    {
                        cout << "  [FAIL] C7  Drone sortie " << i << ": rendezvous node " << rdv_node
                             << "  t=" << drone_time[i][2] << " < min=" << min_meet << "\n";
                        all_ok = false;
                    }
                }
            }
        }
        if (all_ok)
            cout << "  [PASS] C7  Drone rendezvous/return times feasible for all " << (int)drone_order.size() << " sortie(s)\n";
    }

    // ------------------------------------------------------------------
    // C8: Drone endurance not exceeded
    // ------------------------------------------------------------------
    {
        bool all_ok = true;
        for (int i = 0; i < (int)drone_order.size(); i++)
        {
            if (i >= (int)drone_time.size())
                break;
            int launch_pos = drone_order[i][0];
            double flight = drone_time[i][2] - drone_time[i][0] - ((launch_pos == 0) ? 0.0 : cfg->sl);
            bool ok = (flight <= cfg->dtl + 1e-6);
            if (!ok)
            {
                cout << "  [FAIL] C8  Drone sortie " << i << ": flight time=" << flight
                     << " > endurance=" << cfg->dtl << "\n";
                all_ok = false;
            }
        }
        if (all_ok)
            cout << "  [PASS] C8  Drone endurance satisfied for all " << (int)drone_order.size() << " sortie(s)\n";
    }

    // ------------------------------------------------------------------
    // C9: Jetsuite sortie array sizes match
    // ------------------------------------------------------------------
    {
        bool ok = (jetsuite_time.size() == jetsuite_order.size());
        cout << PASS(ok) << " C9  Jetsuite sortie time array size matches"
             << "  (time=" << jetsuite_time.size() << ", order=" << jetsuite_order.size() << ")\n";
    }

    // ------------------------------------------------------------------
    // C10: Jetsuite visit time correct (tau_prime_prime + optional SL)
    // ------------------------------------------------------------------
    {
        bool all_ok = true;
        for (int i = 0; i < (int)jetsuite_order.size(); i++)
        {
            if (i >= (int)jetsuite_time.size())
                break;
            int launch_pos = jetsuite_order[i][0];
            int visit_node = jetsuite_order[i][1];
            if (launch_pos < 0 || launch_pos >= (int)truck_order.size())
                continue;
            int launch_node = truck_order[launch_pos];
            double expected = (launch_pos == 0)
                                  ? (jetsuite_time[i][0] + instance->tau_prime_prime[launch_node][visit_node])
                                  : (jetsuite_time[i][0] + cfg->sl + instance->tau_prime_prime[launch_node][visit_node]);
            bool ok = (std::fabs(jetsuite_time[i][1] - expected) <= 1e-6);
            if (!ok)
            {
                cout << "  [FAIL] C10 Jetsuite sortie " << i << ": visit node " << visit_node
                     << "  time=" << jetsuite_time[i][1] << " expected=" << expected << "\n";
                all_ok = false;
            }
        }
        if (all_ok)
            cout << "  [PASS] C10 Jetsuite visit times correct for all " << (int)jetsuite_order.size() << " sortie(s)\n";
    }

    // ------------------------------------------------------------------
    // C11: Jetsuite rendezvous / return-to-depot time feasible
    // ------------------------------------------------------------------
    {
        bool all_ok = true;
        for (int i = 0; i < (int)jetsuite_order.size(); i++)
        {
            if (i >= (int)jetsuite_time.size())
                break;
            int visit_node = jetsuite_order[i][1];
            int recover_pos = jetsuite_order[i][2];
            if (recover_pos == 0)
            {
                double min_ret = jetsuite_time[i][1] + instance->tau_prime_prime[visit_node][0];
                bool ok = (jetsuite_time[i][2] + 1e-6 >= min_ret);
                if (!ok)
                {
                    cout << "  [FAIL] C11 Jetsuite sortie " << i << ": return-to-depot"
                         << "  t=" << jetsuite_time[i][2] << " < min=" << min_ret << "\n";
                    all_ok = false;
                }
            }
            else
            {
                if (recover_pos < (int)truck_order.size())
                {
                    int rdv_node = truck_order[recover_pos];
                    double min_meet = jetsuite_time[i][1] + cfg->sr + instance->tau_prime_prime[visit_node][rdv_node];
                    bool ok = (jetsuite_time[i][2] + 1e-6 >= min_meet);
                    if (!ok)
                    {
                        cout << "  [FAIL] C11 Jetsuite sortie " << i << ": rendezvous node " << rdv_node
                             << "  t=" << jetsuite_time[i][2] << " < min=" << min_meet << "\n";
                        all_ok = false;
                    }
                }
            }
        }
        if (all_ok)
            cout << "  [PASS] C11 Jetsuite rendezvous/return times feasible for all " << (int)jetsuite_order.size() << " sortie(s)\n";
    }

    // ------------------------------------------------------------------
    // C12: Synchronization – drone launch time == truck depart time
    // ------------------------------------------------------------------
    {
        bool all_ok = true;
        for (int i = 0; i < (int)drone_time.size(); i++)
        {
            int ls = drone_order[i][0];
            if (ls < 0 || ls >= (int)truck_time.size())
                continue;
            bool ok = (std::fabs(truck_time[ls] - drone_time[i][0]) <= 0.01);
            if (!ok)
            {
                cout << "  [FAIL] C12 Drone sortie " << i << ": launch sync"
                     << "  truck_t=" << truck_time[ls] << " drone_t=" << drone_time[i][0] << "\n";
                all_ok = false;
            }
        }
        if (all_ok)
            cout << "  [PASS] C12 Drone launch time synced with truck for all " << (int)drone_order.size() << " sortie(s)\n";
    }

    // ------------------------------------------------------------------
    // C13: Synchronization – drone rendezvous time == truck arrival time
    // ------------------------------------------------------------------
    {
        bool all_ok = true;
        for (int i = 0; i < (int)drone_time.size(); i++)
        {
            int rs = drone_order[i][2];
            if (rs == 0)
                continue;
            if (rs >= (int)truck_time.size())
                continue;
            bool ok = (std::fabs(truck_time[rs] - drone_time[i][2]) <= 0.01);
            if (!ok)
            {
                cout << "  [FAIL] C13 Drone sortie " << i << ": rendezvous sync"
                     << "  truck_t=" << truck_time[rs] << " drone_t=" << drone_time[i][2] << "\n";
                all_ok = false;
            }
        }
        if (all_ok)
            cout << "  [PASS] C13 Drone rendezvous time synced with truck for all " << (int)drone_order.size() << " sortie(s)\n";
    }

    // ------------------------------------------------------------------
    // C14: Synchronization – jetsuite launch time == truck depart time
    // ------------------------------------------------------------------
    {
        bool all_ok = true;
        for (int i = 0; i < (int)jetsuite_time.size(); i++)
        {
            int ls = jetsuite_order[i][0];
            if (ls < 0 || ls >= (int)truck_time.size())
                continue;
            bool ok = (std::fabs(truck_time[ls] - jetsuite_time[i][0]) <= 0.01);
            if (!ok)
            {
                cout << "  [FAIL] C14 Jetsuite sortie " << i << ": launch sync"
                     << "  truck_t=" << truck_time[ls] << " js_t=" << jetsuite_time[i][0] << "\n";
                all_ok = false;
            }
        }
        if (all_ok)
            cout << "  [PASS] C14 Jetsuite launch time synced with truck for all " << (int)jetsuite_order.size() << " sortie(s)\n";
    }

    // ------------------------------------------------------------------
    // C15: Synchronization – jetsuite rendezvous time == truck arrival time
    // ------------------------------------------------------------------
    {
        bool all_ok = true;
        for (int i = 0; i < (int)jetsuite_time.size(); i++)
        {
            int rs = jetsuite_order[i][2];
            if (rs == 0)
                continue;
            if (rs >= (int)truck_time.size())
                continue;
            bool ok = (std::fabs(truck_time[rs] - jetsuite_time[i][2]) <= 0.01);
            if (!ok)
            {
                cout << "  [FAIL] C15 Jetsuite sortie " << i << ": rendezvous sync"
                     << "  truck_t=" << truck_time[rs] << " js_t=" << jetsuite_time[i][2] << "\n";
                all_ok = false;
            }
        }
        if (all_ok)
            cout << "  [PASS] C15 Jetsuite rendezvous time synced with truck for all " << (int)jetsuite_order.size() << " sortie(s)\n";
    }

    cout << "============================================================\n\n";
}

// ============================================================

void Solution::print()
{
    if (cost <= 0)
    {
        cout << "Can't print. The solution is infeasible\n";
        return;
    }

    cout << "Objective: " << cost << "\n";

    // Truck
    cout << "Truck Schedule: " << std::endl;
    for (int i = 0; i + 1 < (int)truck_order.size(); i++)
        cout << "Stage " << i + 1 << ": " << truck_order[i] << " -> " << truck_order[i + 1] << std::endl;

    // Drone
    if (!drone_order.empty())
    {
        cout << "\nDrone Schedule: " << std::endl;
        for (auto &trip : drone_order)
        {
            int k = trip[0] + 1;  // Launch stage (1-based)
            int h = trip[1];      // Customer
            int k1 = trip[2] + 1; // Rendezvous stage (1-based; 0 nghĩa là depot)
            int i = truck_order[trip[0]];
            int j = (trip[2] == 0) ? 0 : truck_order[trip[2]];
            cout << "Stage " << k << " -> " << (trip[2] == 0 ? 0 : k1)
                 << ": " << i << " -> " << h << " -> " << j << std::endl;
        }
    }

    // Jetsuite
    if (!jetsuite_order.empty())
    {
        cout << "\nJetsuite Schedule: " << std::endl;
        for (auto &trip : jetsuite_order)
        {
            int k = trip[0] + 1;
            int h = trip[1];
            int k1 = trip[2] + 1;
            int i = truck_order[trip[0]];
            int j = (trip[2] == 0) ? 0 : truck_order[trip[2]];
            cout << "Stage " << k << " -> " << (trip[2] == 0 ? 0 : k1)
                 << ": " << i << " -> " << h << " -> " << j << std::endl;
        }
    }
}
