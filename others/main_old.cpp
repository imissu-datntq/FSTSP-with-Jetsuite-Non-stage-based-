#include <iostream>
#include "../include/instance.h"
#include "../include/solver.h"
#include "../include/fstsp.h"
#include "../include/gen_instance.h"
#include <memory>

static std::vector<std::string> SplitStringWithDelimiter(const std::string& s, const std::string& delimiter) {
    std::vector<std::string> returnValue;
    std::string::size_type start = 0;
    std::string::size_type end = s.find(delimiter);

    while (end != std::string::npos) {
        returnValue.push_back(s.substr(start, end - start));
        start = end + 1;
        end = s.find(delimiter, start);
    }

    returnValue.push_back(s.substr(start));
    return returnValue;
}

int main(int argc, char** argv) {
    if (argc == 8) {
        // Generate instance script.
        auto n_c = std::string(argv[1]);
        auto side = std::string(argv[2]);
        auto dp = std::string(argv[3]);
        auto rd = std::string(argv[4]);
        auto ver = std::string(argv[5]);
        auto truck_speed = std::string(argv[6]);
        auto drone_speed = std::string(argv[7]);
        GenInstance(std::stoi(n_c), std::stod(side), dp, rd, std::stoi(ver), std::stod(truck_speed),
                    std::stod(drone_speed));
        return EXIT_SUCCESS;
    }

    std::string folder_path;
    double dtl_val;
    bool write = false;
    if (argc == 3) {
        folder_path = std::string(argv[1]);
        dtl_val = std::stod(std::string(argv[2]));
        write = true;
    }
    else {
//        folder_path = "/home/cuong/CLionProjects/RV-FSTSP/generated_instances/c_c/c_c_2";
//        folder_path = "/home/who/CLionProjects/RV-FSTSP/Niels_instances/uniform/uniform-1-n14.txt";
//        folder_path = "/home/who/CLionProjects/RV-FSTSP/Poikonen_instances/P3";
        folder_path = "/home/who/CLionProjects/RV-FSTSP/Murray_instances/FSTSP/FSTSP_10_customer_problems/20140810T123437v12";
        dtl_val = 20;
    }


    std::cout << "Instance name: " << folder_path << std::endl;
    std::cout << "Write arg val: " << write << std::endl;
    std::shared_ptr<Instance> instance = InstanceFactory::createInstance("M", folder_path);
    instance->read();
    FSTSPSolver solver(std::move(instance));
    // , 1, 1, 1, false
    Config cfg = Config(
            8, dtl_val, 1, 1, 1, false, false, false, false, false, false, true);
    std::cout << "---------------------------------------------------------------" << std::endl;
    std::cout << "Current config:" << std::endl;
    std::cout << "- Number of threads = " << cfg.num_thread << std::endl;
    std::cout << "- Drone endurance = " << cfg.dtl << std::endl;
    std::cout << "- SL = " << cfg.sl << std::endl;
    std::cout << "- SR = " << cfg.sr << std::endl;
    std::cout << "- L = " << cfg.L << std::endl;
    std::cout << "- Allow loop = " << cfg.allow_loop << std::endl;
    std::cout << "- Allow revisit = " << cfg.allow_revisit << std::endl;
    std::cout << "- Allow multi-visit drone = " << cfg.allow_multi_visit_drone << std::endl;
    std::cout << "- Use TSP optimal solution as warmstart: = " << cfg.use_tsp_for_warmstart << std::endl;
    std::cout << "- Use cutting plane method: = " << cfg.use_cutting_plane << std::endl;
    std::cout << "- Use CPLEX conflict refiner: = " << cfg.use_cplex_conflict_refiner << std::endl;

    std::cout << "---------------------------------------------------------------" << std::endl;
    Result result{};
    // if (cfg.allow_loop && (!cfg.allow_revisit)) {
    //     std::cout << "Must allow revisit for loop to work!!!" << std::endl;
    // }
    // if (cfg.allow_multi_visit_drone) {
    //     result = solver.RV_FSTSP_MVD_Re(cfg);
    // }
    // else {
//    std::cout << "--- Stage-based FSTSP ---" << std::endl;
//    result = solver.Stage_Based_FSTSP_Original(cfg);
    std::cout << "\n\n\n--- 5 index-based FSTSP ---" << std::endl;
    Result result2 = solver.FSTSP_5_index(cfg);
    // }
//     result = solver.OriginalSolverCPLEX(cfg);

    if (write) {
        std::cout << "In write mode" << std::endl;
        auto i_name_split = SplitStringWithDelimiter(folder_path, "/");
        auto i_name = i_name_split[i_name_split.size() - 1];
        auto instance_class = i_name_split[i_name_split.size()-2];
        std::string of = "/home/cuong/CLionProjects/RV-FSTSP/rv_rand.csv";
        std::ofstream out(of, std::ios::app);
        if (!out.is_open()) {
            std::cout << "Error opening file!" << std::endl;
        }
        out << cfg.L << "," << cfg.dtl << "," << i_name << "," << result.cost << "," << result.time_spent << "," <<
            result.loop_count << "\n";
    }
    return 0;
}


