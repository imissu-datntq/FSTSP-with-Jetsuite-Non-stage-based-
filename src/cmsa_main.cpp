#include <iostream>
#include <fstream>
#include <vector>
#include "../include/common.h"
#include "../include/instance.h"
#include "../include/CMSA.h"
#include "../include/cfg.h"

using namespace std;

int main(int argc, char **argv) {
    // Check command line parameters
    if (argc < 2) {
        cerr << "Usage: " << argv[0] << " <instance_file> [options]" << endl;
        cerr << "Options:" << endl;
        cerr << "  -t <time_limit>        : Time limit (seconds)" << endl;
        cerr << "  -o <output_file>       : Output file" << endl;
        cerr << "  -s <screen_mode>       : Screen mode (0-2)" << endl;
        cerr << "  -dtl <drone_time_limit>: Drone time limit" << endl;
        cerr << "  -sl <service_launch>   : Drone/jetsuit launch time" << endl;
        cerr << "  -sr <service_recovery> : Drone/jetsuit recovery time" << endl;
        cerr << "  -j                     : Enable jetsuit usage" << endl;
        cerr << "  -d                     : Print detailed information" << endl;
        return 1;
    }

    string instance_file = argv[1];
    string output_file = "";
    int screen_mode = 2;
    double time_limit = 600.0;
    double drone_time_limit = 9999.0;
    double service_launch = 0.0;
    double service_recovery = 0.0;
    bool use_jetsuit = false;
    bool debug_info = false;

    // Process optional parameters
    for (int i = 2; i < argc; i++) {
        string arg = argv[i];
        if (arg == "-t" && i + 1 < argc) {
            time_limit = stod(argv[++i]);
        } else if (arg == "-o" && i + 1 < argc) {
            output_file = argv[++i];
        } else if (arg == "-s" && i + 1 < argc) {
            screen_mode = stoi(argv[++i]);
        } else if (arg == "-dtl" && i + 1 < argc) {
            drone_time_limit = stod(argv[++i]);
        } else if (arg == "-sl" && i + 1 < argc) {
            service_launch = stod(argv[++i]);
        } else if (arg == "-sr" && i + 1 < argc) {
            service_recovery = stod(argv[++i]);
        } else if (arg == "-j") {
            use_jetsuit = true;
        } else if (arg == "-d") {
            debug_info = true;
        }
    }

    // Determine instance type
    string type_instance;
    if (instance_file.find("Murray") != string::npos) {
        type_instance = "M";
    } else if (instance_file.find("Niels") != string::npos) {
        type_instance = "N";
    } else if (instance_file.find("Poikonen") != string::npos) {
        type_instance = "P";
    } else {
        cerr << "Cannot recognize the instance type from filename." << endl;
        return 1;
    }

    // Create configuration
    Config cfg = Config(
        drone_time_limit,    // dtl
        service_launch,      // sl
        service_recovery,    // sr
        screen_mode,         // screen_mode
        true,                // is_check
        time_limit,          // tl
        output_file,         // output_path
        true,                // allow_revisit
        true,                // allow_loop
        -1,                  // n_thread (use default)
        0                    // test_mode
    );

    // Print configuration info
    cout << "=== Configuration ===\n";
    cout << "Instance file: " << instance_file << endl;
    cout << "Time limit: " << time_limit << " seconds\n";
    cout << "Drone time limit: " << drone_time_limit << " units\n";
    cout << "Service launch: " << service_launch << " units\n";
    cout << "Service recovery: " << service_recovery << " units\n";
    cout << "Jetsuit enabled: " << (use_jetsuit ? "Yes" : "No") << endl;
    cout << "Debug info: " << (debug_info ? "Yes" : "No") << endl;
    cout << "====================\n\n";

    // Read instance and solve the problem
    shared_ptr<Instance> instance = InstanceFactory::createInstance(type_instance, instance_file);
    instance->read();

    // Build and solve with CMSA
    CMSA cmsa(instance, &cfg);
    
    // Set jetsuit usage ratio based on parameters
    if (use_jetsuit) {
        cmsa.jetsuite_ratio = 0.2; // Increase jetsuit usage ratio
    } else {
        cmsa.jetsuite_ratio = 0.0; // Do not use jetsuit
    }
    
    // Set output file
    cmsa.output_file = output_file;
    
    // Measure execution time
    auto start_time = chrono::high_resolution_clock::now();
    Solution solution = cmsa.solve();
    auto end_time = chrono::high_resolution_clock::now();
    double elapsed_time = chrono::duration<double>(end_time - start_time).count();

    // Print results
    cout << "\n=== FINAL RESULTS ===\n";
    cout << "Execution time: " << elapsed_time << " seconds\n";
    cout << "Objective value: " << solution.cost << endl;
    cout << "Number of customers: " << (instance->num_node - 1) << endl;
    cout << "Customers served by truck: " << (solution.truck_order.size() - 2) << endl;
    cout << "Customers served by drone: " << solution.drone_order.size() << endl;
    cout << "Customers served by jetsuit: " << solution.jetsuite_order.size() << endl;

    return 0;
}