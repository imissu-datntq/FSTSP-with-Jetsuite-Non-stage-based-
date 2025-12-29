#include "../include/common.h"
#include "../include/instance.h"
#include "../include/solver.h"
#include "../include/fstsp.h"
#include "boost/program_options.hpp"
#include "boost/regex.hpp"
#include "../include/CMSA.h"
#include "../include/other_method_solver.h"

int main(int argc, char **argv)
{
    namespace po = boost::program_options;
    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()("help", "produce help message")("input,i", po::value<string>()->required(), "Input file location")("output,o", po::value<string>()->default_value("skip"), "Output file location")("mode", po::value<int>()->default_value(3), "Solving mode. 33: CMSA + 3-index, 2: 2-index (no RV), 3: 3-index, 5: 5-index")("screen,s", po::value<int>()->default_value(2), "Screen option. (0: only notify if feasibility check failed, 1: results + check, 2: all)")("time,t", po::value<int>()->default_value(-1), "Time limit. Default is None")("thread", po::value<int>()->default_value(-1), "Number of threads")("dtl", po::value<double>()->default_value(9999), "Drone endurance")("sl", po::value<double>()->default_value(0), "Service time launching")("sr", po::value<double>()->default_value(0), "Service time recovering")("check", po::value<bool>()->default_value(true), "Is checking solution?")("revisit", po::value<bool>()->default_value(true), "Is allow revisit?")("loop", po::value<bool>()->default_value(true), "Is allow loop?")("test", po::value<int>()->default_value(0), "Test mode (3-index): 0 - normal, 1 - not tighten, 2 - z_{kk'}' = 0, k'-k >= 3")("debug,d", po::value<bool>()->default_value(false), "Enable detailed debug information")("jetsuit,j", po::value<bool>()->default_value(false), "Enable jetsuit usage");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        return 1;
    }
    po::notify(vm);

    std::cout << "mode: " << vm["mode"].as<int>() << ", sl: " << vm["sl"].as<double>() << ", sr: " << vm["sr"].as<double>()
              << ", revisit: " << vm["revisit"].as<bool>() << ", loop: " << vm["loop"].as<bool>() << std::endl;

    string folder_path = vm["input"].as<string>();
    string type_instance;
    if (boost::regex_search(folder_path, boost::regex(R"(.*Murray.*)")))
        type_instance = "M";
    else if (boost::regex_search(folder_path, boost::regex(R"(.*Niels.*)")))
        type_instance = "N";
    else if (boost::regex_search(folder_path, boost::regex(R"(.*Poikonen.*)")))
        type_instance = "P";
    else
    {
        std::cerr << "Cannot recognize the instance type" << std::endl;
        return 1;
    }

    Config cfg = Config(
        vm["dtl"].as<double>(),
        vm["sl"].as<double>(),
        vm["sr"].as<double>(),
        vm["screen"].as<int>(),
        vm["check"].as<bool>(),
        vm["time"].as<int>(),
        vm["output"].as<string>(),
        vm["revisit"].as<bool>(),
        vm["loop"].as<bool>(),
        vm["thread"].as<int>(),
        vm["test"].as<int>());

    std::shared_ptr<Instance> instance = InstanceFactory::createInstance(type_instance, folder_path);
    instance->read();
    std::cout << "Solving mode: " << vm["mode"].as<int>() << "-indexed base" << std::endl;
    FSTSPSolver solver(instance);

    // override config if maxradius is specified from instance
    if (instance->maxradius != -1)
    {
        if (instance->maxradius == INFINITY)
            cfg.dtl = 9999;
        else
            cfg.dtl = instance->maxradius;
    }

    // =====================================================
    // Set JetSuite enabled flag depending on solving mode
    // =====================================================
    int mode = vm["mode"].as<int>();

    // Mặc định: không dùng JetSuite
    cfg.jetsuite_enabled = false;

    // Các mode MIP sử dụng JetSuite
    if (mode == 21 || mode == 31)
    {
        cfg.jetsuite_enabled = true;
    }
    // CMSA (mode 33) dùng flag từ dòng lệnh --jetsuit
    else if (mode == 33)
    {
        cfg.jetsuite_enabled = vm["jetsuit"].as<bool>();
    }

    if (cfg.screen_mode >= 2)
    {
        std::cout << "Solving mode: " << vm["mode"].as<int>() << "-indexed base" << std::endl;
        std::cout << "Instance name: " << folder_path << std::endl;
        std::cout << "---------------------------------------------------------------" << std::endl;
        std::cout << "Current config:" << std::endl;
        std::cout << "- Number of threads = " << cfg.num_thread << std::endl;
        std::cout << "- Drone endurance = " << cfg.dtl << std::endl;
        std::cout << "- SL = " << cfg.sl << std::endl;
        std::cout << "- SR = " << cfg.sr << std::endl;
        if (vm["mode"].as<int>() != 21 || vm["mode"].as<int>() != 31){
            std::cout << "- Allow loop = " << cfg.allow_loop << std::endl;
            std::cout << "- Allow revisit = " << cfg.allow_revisit << std::endl;
        }
        std::cout << "- Allow multi-visit drone = " << cfg.allow_multi_visit_drone << std::endl;
        std::cout << "- Use TSP optimal solution as warmstart: = " << cfg.use_tsp_for_warmstart << std::endl;
        std::cout << "- Use cutting plane method: = " << cfg.use_cutting_plane << std::endl;
        std::cout << "- Use CPLEX conflict refiner: = " << cfg.use_cplex_conflict_refiner << std::endl;
        std::cout << "- Debug info: = " << vm["debug"].as<bool>() << std::endl;
        std::cout << "- Jetsuit enabled: = " << cfg.jetsuite_enabled << std::endl;
        std::cout << "---------------------------------------------------------------" << std::endl;
    }

    if (vm["mode"].as<int>() == 3)
        solver.RV_FSTSP_3_index(cfg);
    else if (vm["mode"].as<int>() == 33)
    {
        CMSA cmsa(instance, &cfg);

        // Configure CMSA options
        if (vm["jetsuit"].as<bool>())
        {
            cmsa.jetsuite_ratio = 0.2; // Increase jetsuit usage ratio
        }
        else
        {
            cmsa.jetsuite_ratio = 0.0; // Do not use jetsuit
        }

        // Set output file if specified
        if (vm["output"].as<string>() != "skip")
        {
            cmsa.output_file = vm["output"].as<string>();
        }

        // Run CMSA and measure execution time
        auto start_time = std::chrono::high_resolution_clock::now();
        Solution solution = cmsa.solve();
        auto end_time = std::chrono::high_resolution_clock::now();
        double elapsed_time = std::chrono::duration<double>(end_time - start_time).count();

        // Print final summary
        std::cout << "\n=== FINAL RESULTS ===\n";
        std::cout << "Execution time: " << elapsed_time << " seconds\n";
        std::cout << "Objective value: " << solution.cost << std::endl;
        std::cout << "Number of customers: " << (instance->num_node - 1) << std::endl;
        std::cout << "Customers served by truck: " << (solution.truck_order.size() - 2) << std::endl;
        std::cout << "Customers served by drone: " << solution.drone_order.size() << std::endl;
        std::cout << "Customers served by jetsuit: " << solution.jetsuite_order.size() << std::endl;
    }
    // ------------------- ADDED: new modes -------------------
    else if (vm["mode"].as<int>() == 21)
    {
        // Case 1 – Two-index with JetSuite (drone returns to launch)
        Result res1 = solver.FSTSP_2index_with_JetSuite(cfg);
    }
    else if (vm["mode"].as<int>() == 31)
    {
        // Case 2 – Three-index (no stage) with JetSuite (drone retrieved at different node)
        Result res2 = solver.FSTSP_3indexNoStage_with_JetSuite(cfg);
    }
    // --------------------------------------------------------
    else
    {
        std::cerr << "Please specify supported mode" << std::endl;
        return 1;
    }

    return 0;
}
