//
// Created by ORG on 23/02/2024.
//

#ifndef UMV_FSTSP_GEN_INSTANCE_H
#define UMV_FSTSP_GEN_INSTANCE_H

#include <algorithm>
#include <vector>
#include <random>
#include <iostream>
#include <filesystem>
namespace fs = std::filesystem;

inline std::vector<std::vector<double>> generateSymmetricMatrix(int rows, int cols, double upperBound) {
    std::vector<std::vector<double>> matrix(rows, std::vector<double>(cols, 0.0));

    // Random number generator
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis(1.2, upperBound);

    // Fill the upper triangular part of the matrix with random values
    for (int i = 0; i < rows; ++i) {
        for (int j = i + 1; j < cols; ++j) {
            double randomValue = dis(gen);
            matrix[i][j] = randomValue;
            matrix[j][i] = randomValue; // Ensures symmetry
        }
    }

    // Set diagonal elements, last element of the first row, and last row to 0
    for (int i = 0; i < rows; ++i) {
        matrix[i][i] = 0.0; // Diagonal
        matrix[0][cols - 1] = 0.0; // Last element of the first row
        matrix[rows - 1][i] = 0.0; // Last row
    }

    return matrix;
}

inline double ins_perc_higher_dtl(std::vector<std::vector<double>>& tau_prime, double dtl) {
    double n = tau_prime.size() * tau_prime.size() - 2 * tau_prime.size() + 1;
    double count = 0;
    for (int i = 0; i < tau_prime.size(); i++) {
        for (int j = 0; j < tau_prime[i].size(); j++) {
            if (tau_prime[i][j] > dtl) {
                count++;
            }
        }
    }
    return 100.0 * count / n;
}

std::vector<int> generate1DMatrix(int n) {
    // Calculate the minimum number of elements (at least 0.5*n, rounded up)
    int minElements = static_cast<int>(std::ceil(0.7 * n)) + 1;

    // Create a shuffled list of numbers from 1 to n
    std::vector<int> shuffledList(n);
    std::iota(shuffledList.begin(), shuffledList.end(), 1);
    std::random_device rd;
    std::mt19937 gen(rd());
    std::shuffle(shuffledList.begin(), shuffledList.end(), gen);

    // Generate a random number between minElements and n for the size of the matrix
    std::uniform_int_distribution<int> dis(minElements, n);
    int size = dis(gen);

    // Select the required number of elements from the shuffled list
    std::vector<int> matrix(shuffledList.begin(), shuffledList.begin() + size);

    return matrix;
}

class GenInstance {
public:
    int n_customer;
    double dtl;
    std::vector<std::vector<double>> tau;
    std::vector<std::vector<double>> tau_prime;
    std::vector<std::tuple<int, double, double, double>> Nodes;
    std::vector<int> c_prime;
    std::vector<int> heavy;
    GenInstance() = default;
    GenInstance(int n_customer, double dtl, int v, bool gen_node);
    GenInstance(int n_customer, double dtl, int v, bool gen_node, double side_of_area);
    GenInstance(int n_customer, double side_of_area, const std::string& depot_location,
                const std::string& customer_distribution_type, int v, double truck_speed, double drone_speed);

    void calculate_avg_ins_perc_dtl_murray(std::string folder_path, double dtl);
};

class path;

inline void write_c_prime_to_csv(const std::vector<int>& matrix, const std::string& filename,
                                 const std::string& folderPath) {
    // Write the vector to a CSV file
    std::ofstream file(filename);
    if (file.is_open()) {
        for (size_t i = 0; i < matrix.size(); ++i) {
            file << matrix[i];
            if (i != matrix.size() - 1) {
                file << ",";
            }
        }
        file.close();
        std::cout << "Vector has been written to file '" << filename << "' successfully." << std::endl;
    }
    else {
        std::cerr << "Unable to open file '" << filename << "' for writing." << std::endl;
        return;
    }

    // Copy the file to the specified folder path
    fs::path srcPath(filename);
    fs::path destPath = fs::path(folderPath) / srcPath.filename();
    try {
        fs::copy_file(srcPath, destPath, fs::copy_options::overwrite_existing);
        std::cout << "File copied to folder path: " << destPath << std::endl;
    }
    catch (const fs::filesystem_error& e) {
        std::cerr << "Error copying file: " << e.what() << std::endl;
    }
}

inline void write_tau_to_csv(const std::vector<std::vector<double>>& matrix, const std::string& filename,
                             const std::string& folderPath) {
    // Write the matrix to a CSV file
    std::ofstream file(filename);
    if (file.is_open()) {
        for (const auto& row : matrix) {
            for (size_t i = 0; i < row.size(); ++i) {
                file << row[i];
                if (i != row.size() - 1) {
                    file << ",";
                }
            }
            file << std::endl;
        }
        file.close();
        std::cout << "Matrix has been written to file '" << filename << "' successfully." << std::endl;
    }
    else {
        std::cerr << "Unable to open file '" << filename << "' for writing." << std::endl;
        return;
    }

    // Copy the file to the specified folder path
    fs::path srcPath(filename);
    fs::path destPath = fs::path(folderPath) / srcPath.filename();
    try {
        fs::copy_file(srcPath, destPath, fs::copy_options::overwrite_existing);
        std::cout << "File copied to folder path: " << destPath << std::endl;
    }
    catch (const fs::filesystem_error& e) {
        std::cerr << "Error copying file: " << e.what() << std::endl;
    }
}

inline void write_node_file(const std::string& filename, const std::string& folderPath,
                            std::vector<std::tuple<int, double, double, double>>& nodes) {
    std::ofstream file(filename);
    if (file.is_open()) {
        for (auto& n : nodes) {
            file << get<0>(n) << "," << get<1>(n) << "," << get<2>(n) << "," << get<3>(n) << "\n";
        }
        file.close();
    }
    else {
        std::cerr << "Unable to open file '" << filename << "' for writing." << std::endl;
        return;
    }
    fs::path srcPath(filename);
    fs::path destPath = fs::path(folderPath) / srcPath.filename();
    try {
        fs::copy_file(srcPath, destPath, fs::copy_options::overwrite_existing);
        std::cout << "File copied to folder path: " << destPath << std::endl;
    }
    catch (const fs::filesystem_error& e) {
        std::cerr << "Error copying file: " << e.what() << std::endl;
    }
}

inline void create_and_write_node_file(int n_customer, std::vector<int>& c_prime, const std::string& filename,
                                       const std::string& folderPath) {
    std::ofstream file(filename);
    if (file.is_open()) {
        for (int i = 0; i < n_customer + 2; i++) {
            file << i << "," << 1 << "," << 1 << ",";
            if (i == 0) {
                file << 1 << "\n";
                continue;
            }

            bool check = false;
            for (int c : c_prime) {
                if (c == i) {
                    check = true;
                }
            }

            if (check || i == n_customer + 1) {
                file << 0 << "\n";
            }
            else {
                file << 1 << "\n";
            }
        }
        file.close();
    }
    else {
        std::cerr << "Unable to open file '" << filename << "' for writing." << std::endl;
        return;
    }
    fs::path srcPath(filename);
    fs::path destPath = fs::path(folderPath) / srcPath.filename();
    try {
        fs::copy_file(srcPath, destPath, fs::copy_options::overwrite_existing);
        std::cout << "File copied to folder path: " << destPath << std::endl;
    }
    catch (const fs::filesystem_error& e) {
        std::cerr << "Error copying file: " << e.what() << std::endl;
    }
}

inline void createDirectory(const std::string& directoryPath, const std::string& directoryName) {
    fs::path dirPath = fs::path(directoryPath) / directoryName;
    try {
        fs::create_directory(dirPath);
        std::cout << "Directory '" << directoryName << "' created successfully in '" << directoryPath << "'." <<
            std::endl;
    }
    catch (const fs::filesystem_error& e) {
        std::cerr << "Error creating directory: " << e.what() << std::endl;
    }
}

inline double generateRandomDouble(double lb, double ub) {
    // Create a random number engine
    std::random_device rd;
    std::mt19937 gen(rd());

    // Create a uniform real distribution for doubles between 0 and 25
    std::uniform_real_distribution<double> dist(lb, ub);

    // Generate a random double
    return dist(gen);
}

bool exist(const std::vector<int>& vec, int element) {
    // Use std::find to search for the element in the vector
    return std::find(vec.begin(), vec.end(), element) != vec.end();
}

inline double manhattanDistance(double x1, double y1, double x2, double y2) {
    return std::abs(x1 - x2) + std::abs(y1 - y2);
}

inline double euclideanDistance(double x1, double y1, double x2, double y2) {
    return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
}

inline double area_of_polygon(const std::vector<std::tuple<int, double, double, double>>& points) {
    double area = 0;

    for (int i = 0; i < points.size(); i++) {
        if (i != points.size() - 1) {
            auto p1 = points[i];
            auto p2 = points[i + 1];
            auto x1 = std::get<1>(p1);
            auto y1 = std::get<2>(p1);
            auto x2 = std::get<1>(p2);
            auto y2 = std::get<2>(p2);

            area += x1 * y2 - y1 * x2;
        }
        else {
            auto p1 = points[i];
            auto p2 = points[0];
            auto x1 = std::get<1>(p1);
            auto y1 = std::get<2>(p1);
            auto x2 = std::get<1>(p2);
            auto y2 = std::get<2>(p2);

            area += x1 * y2 - y1 * x2;
        }
    }
    return std::abs(area) / 2.0;
}

GenInstance::GenInstance(int n_customer, double side_of_area, const std::string& depot_location,
                         const std::string& customer_distribution_type, int v, double truck_speed, double drone_speed) {
    std::string instance_dir_name = "/home/cuong/CLionProjects/RV-FSTSP/generated_instances/" + depot_location + "_" +
        customer_distribution_type + "/" + depot_location + "_" +
        customer_distribution_type + "_" + std::to_string(v);
    std::cout << "Instance dir name: " << instance_dir_name << std::endl;
    std::string dir_path = "/home/cuong/CLionProjects/RV-FSTSP/generated_instances/" + depot_location + "_" +
        customer_distribution_type;
    std::string dir_name = depot_location + "_" +
        customer_distribution_type + "_" + std::to_string(v);
    createDirectory(dir_path, dir_name);
    c_prime = generate1DMatrix(n_customer);
    Nodes.resize(n_customer + 2);
    if (depot_location == "c") {
        // center of the map
        Nodes[0] = std::make_tuple(0, side_of_area / 2, side_of_area / 2, 0);
        Nodes[n_customer + 1] = std::make_tuple(n_customer + 1, side_of_area / 2, side_of_area / 2, 0);
        // generate the remaining points based on settings
    }
    else if (depot_location == "e") {
        // the smallest (bottom left corner)
        Nodes[0] = std::make_tuple(0, 0, 0, 0);
        Nodes[n_customer + 1] = std::make_tuple(n_customer + 1, 0, 0, 0);
    }
    else if (depot_location == "r") {
        double x_coord = generateRandomDouble(0, side_of_area);
        double y_coord = generateRandomDouble(0, side_of_area);
        Nodes[0] = std::make_tuple(0, x_coord, y_coord, 0);
        Nodes[n_customer + 1] = std::make_tuple(n_customer + 1, x_coord, y_coord, 0);
    }
    else {
        throw std::string("depot_location not supported: " + depot_location);
    }

    // customer generation
    if (customer_distribution_type == "r") {
        for (int h = 1; h <= n_customer; h++) {
            int heavy = 0;
            if (!exist(c_prime, h)) {
                heavy = 1;
            }

            Nodes[h] = std::make_tuple(h, generateRandomDouble(0, side_of_area), generateRandomDouble(0, side_of_area),
                                       heavy);
        }
    }
    else if (customer_distribution_type == "c" || customer_distribution_type == "rc") {
        // s points will be chosen as cluster seeds.
        auto cluster_dist = std::uniform_int_distribution<int>(3, 5);
        std::random_device rd;
        std::mt19937 gen(rd());
        const int s = cluster_dist(gen);
        std::cout << "Number of clusters: " << s << std::endl;
        // Generate cluster seeds
        bool accept_cluster_seed = false, dist_too_small = true;
        while (!accept_cluster_seed || dist_too_small) {
            std::vector<std::tuple<int, double, double, double>> clusters;
            for (int h = 1; h <= s; h++) {
                int heavy = 0;
                if (!exist(c_prime, h)) {
                    heavy = 1;
                }
                auto x = generateRandomDouble(0, side_of_area);
                auto y = generateRandomDouble(0, side_of_area);
                Nodes[h] = std::make_tuple(h, x, y, heavy);
                clusters.push_back(Nodes[h]);
            }
            int check = 0;
            std::vector<double> cluster_h_radius(s + 1, 1e9);
            for (int h = 1; h < s; h++) {
                for (int h2 = h + 1; h2 <= s; h2++) {
                    auto p1 = Nodes[h];
                    auto p2 = Nodes[h2];

                    auto x1 = std::get<1>(p1);
                    auto y1 = std::get<2>(p1);
                    auto x2 = std::get<1>(p2);
                    auto y2 = std::get<2>(p2);
                    auto dist = euclideanDistance(x1, y1, x2, y2);
                    if (dist < side_of_area / s + 1) {
                        dist_too_small = true;
                        check++;
                    }
                }
            }
            if (check == 0) {
                dist_too_small = false;
            }
            double area = area_of_polygon(clusters);
            if (area >= side_of_area * side_of_area / s) {
                accept_cluster_seed = true;
            }
        }

        std::vector<double> cluster_h_radius(s + 1, 1e9);
        for (int h = 1; h <= s; h++) {
            std::cout << "(" << std::get<1>(Nodes[h]) << "," << std::get<2>(Nodes[h]) << "),";
            for (int h2 = 1; h2 <= s; h2++) {
                if (h != h2) {
                    auto x1 = std::get<1>(Nodes[h]);
                    auto x2 = std::get<2>(Nodes[h]);
                    auto y1 = std::get<1>(Nodes[h2]);
                    auto y2 = std::get<2>(Nodes[h2]);

                    auto dist = euclideanDistance(x1, y1, x2, y2);
                    if (dist < cluster_h_radius[h]) {
                        cluster_h_radius[h] = dist / s * (s - 1.0);
                    }
                }
            }
            std::cout << std::endl;
        }
        // done with generating good clusters.
        // chia: rc va c
        if (customer_distribution_type == "c") {
            int n_customer_in_a_cluster = floor((n_customer - s) / s);
            int remain_for_random = n_customer - s - s * n_customer_in_a_cluster;


            int c = s + 1;
            for (int h = 1; h <= s; h++) {
                for (int gen_cus = 0; gen_cus < n_customer_in_a_cluster; gen_cus++) {
                    int heavy = 0;
                    if (!exist(c_prime, c)) {
                        heavy = 1;
                    }
                    Nodes[c] = std::make_tuple(
                        c, generateRandomDouble(std::max(0.0, std::get<1>(Nodes[h]) - cluster_h_radius[h]),
                                                std::min(side_of_area, std::get<1>(Nodes[h]) + cluster_h_radius[h])),
                        generateRandomDouble(std::max(0.0, std::get<2>(Nodes[h]) - cluster_h_radius[h]),
                                             std::min(side_of_area, std::get<2>(Nodes[h]) + cluster_h_radius[h])),
                        heavy);
                    c++;
                }
            }

            // remaining.
            for (int gen_cus = 0; gen_cus < remain_for_random; gen_cus++) {
                auto cl_dist_choose = std::uniform_int_distribution<int>(1, s);
                const int chosen_cluster = cl_dist_choose(gen);
                int heavy = 0;
                if (!exist(c_prime, c)) {
                    heavy = 1;
                }
                Nodes[c] = std::make_tuple(
                    c, generateRandomDouble(
                        std::max(0.0, std::get<1>(Nodes[chosen_cluster]) - cluster_h_radius[chosen_cluster]),
                        std::min(side_of_area, std::get<1>(Nodes[chosen_cluster]) + cluster_h_radius[chosen_cluster])),
                    generateRandomDouble(
                        std::max(0.0, std::get<2>(Nodes[chosen_cluster]) - cluster_h_radius[chosen_cluster]),
                        std::min(side_of_area, std::get<2>(Nodes[chosen_cluster]) + cluster_h_radius[chosen_cluster])),
                    heavy);
                c++;
            }
        }
        else if (customer_distribution_type == "rc") {
            int randomized_customer = ceil(n_customer / 2);
            int remain_for_random = n_customer - randomized_customer - s;
            int c = s + 1;
            for (int gen_cus = 0; gen_cus < randomized_customer; gen_cus++) {
                int heavy = 0;
                if (!exist(c_prime, c)) {
                    heavy = 1;
                }
                Nodes[c] = std::make_tuple(c, generateRandomDouble(0, side_of_area),
                                           generateRandomDouble(0, side_of_area), heavy);
                c++;
            }

            // remain for random cluster:
            for (int gen_cus = 0; gen_cus < remain_for_random; gen_cus++) {
                auto cl_dist_choose = std::uniform_int_distribution<int>(1, s);
                const int chosen_cluster = cl_dist_choose(gen);
                std::cout << "customer " << c << " chosen in cluster " << chosen_cluster << std::endl;
                int heavy = 0;
                if (!exist(c_prime, c)) {
                    heavy = 1;
                }
                Nodes[c] = std::make_tuple(
                    c, generateRandomDouble(
                        std::max(0.0, std::get<1>(Nodes[chosen_cluster]) - cluster_h_radius[chosen_cluster]),
                        std::min(side_of_area, std::get<1>(Nodes[chosen_cluster]) + cluster_h_radius[chosen_cluster])),
                    generateRandomDouble(
                        std::max(0.0, std::get<2>(Nodes[chosen_cluster]) - cluster_h_radius[chosen_cluster]),
                        std::min(side_of_area, std::get<2>(Nodes[chosen_cluster]) + cluster_h_radius[chosen_cluster])),
                    heavy);
                c++;
            }
        }
    }
    else {
        throw std::string("customer_distribution_type not supported: " + customer_distribution_type);
    }

    // done
    std::cout << "Nodes info: " << std::endl;
    for (auto& x : Nodes) {
        std::cout << std::get<0>(x) << ", " << std::get<1>(x) << ", " << std::get<2>(x) << ", " << std::get<3>(x) <<
            std::endl;
    }

    // assert that no nodes have the same coordinates
    for (int idx1 = 0; idx1 < n_customer; idx1++) {
        for (int idx2 = idx1 + 1; idx2 <= n_customer; idx2++) {
            if (std::get<1>(Nodes[idx1]) == std::get<1>(Nodes[idx2]) && std::get<2>(Nodes[idx1]) == std::get<2>(
                Nodes[idx2])) {
                std::cout << "Two nodes have the same coordinates!!" << std::endl;
                exit(1);
            }
        }
    }

    // assertion done, no points have same coords
    // proceed to generate instance files.
    tau.resize(n_customer + 2);
    tau_prime.resize(n_customer + 2);
    for (int i = 0; i < tau.size(); i++) {
        tau[i].resize(n_customer + 2);
        tau_prime[i].resize(n_customer + 2);
    }

    for (int i = 0; i < n_customer + 2; i++) {
        for (int j = 0; j < n_customer + 2; j++) {
            if (i == j || i == n_customer + 1) {
                tau[i][j] = 0;
                tau_prime[i][j] = 0;
            }
            else {
                tau[i][j] = manhattanDistance(
                    get<1>(Nodes[i]), get<2>(Nodes[i]), get<1>(Nodes[j]), get<2>(Nodes[j])) / truck_speed;
                tau_prime[i][j] = euclideanDistance(
                    get<1>(Nodes[i]), get<2>(Nodes[i]), get<1>(Nodes[j]), get<2>(Nodes[j])) / drone_speed;
            }
        }
    }

    write_node_file("nodes.csv",
                    instance_dir_name, Nodes);
    write_c_prime_to_csv(c_prime, "Cprime.csv",
                         instance_dir_name);
    write_tau_to_csv(tau, "tau.csv",
                     instance_dir_name);
    write_tau_to_csv(tau_prime, "tauprime.csv",
                     instance_dir_name);
}

// v: version number. just for naming purpose.
GenInstance::GenInstance(int n_customer, double dtl, int v, bool gen_node) {
    createDirectory("/home/cuong/CLionProjects/uMV-FSTSP/rand_generated_instances/", "v" + std::to_string(v));
    c_prime = generate1DMatrix(n_customer);
    if (!gen_node) {
        tau = generateSymmetricMatrix(n_customer + 2, n_customer + 2, 20);
        tau_prime = generateSymmetricMatrix(n_customer + 2, n_customer + 2, 20);
        create_and_write_node_file(n_customer, c_prime, "nodes.csv",
                                   "/home/cuong/CLionProjects/uMV-FSTSP/rand_generated_instances/v" + std::to_string(
                                       v));
    }
    else {
        // Generate node info then calculate.
        for (int i = 0; i <= n_customer + 1; i++) {
            double last = 0;
            if (i != n_customer + 1) {
                if (!exist(c_prime, i)) {
                    last = 1;
                }
                auto node = std::make_tuple(i, generateRandomDouble(0, 50), generateRandomDouble(0, 50), last);
                Nodes.push_back(node);
            }
            if (i == n_customer + 1) {
                auto n_clone = Nodes[0];
                get<0>(n_clone) = n_customer + 2;
                Nodes.push_back(n_clone);
            }
        }
        std::cout << "Printing nodes" << std::endl;
        for (int i = 0; i < n_customer + 2; i++) {
            std::cout << std::get<0>(Nodes[i]) << " " << std::get<1>(Nodes[i]) << " " << std::get<2>(Nodes[i]) << " " <<
                std::get<3>(Nodes[i]) << std::endl;
        }
        tau.resize(n_customer + 2);
        tau_prime.resize(n_customer + 2);
        for (int i = 0; i < tau.size(); i++) {
            tau[i].resize(n_customer + 2);
            tau_prime[i].resize(n_customer + 2);
        }
        for (int i = 0; i < n_customer + 2; i++) {
            for (int j = 0; j < n_customer + 2; j++) {
                if (i == j || i == n_customer + 1) {
                    tau[i][j] = 0;
                    tau_prime[i][j] = 0;
                }
                else {
                    // truck_speed = 2/3 drone_speed.
                    tau[i][j] = manhattanDistance(
                        get<1>(Nodes[i]), get<2>(Nodes[i]), get<1>(Nodes[j]), get<2>(Nodes[j])) / 2;
                    tau_prime[i][j] = euclideanDistance(
                        get<1>(Nodes[i]), get<2>(Nodes[i]), get<1>(Nodes[j]), get<2>(Nodes[j])) / 3;
                }
            }
        }
        std::cout << "Printing tau:" << std::endl;
        for (int i = 0; i < n_customer + 2; i++) {
            for (int j = 0; j < n_customer + 2; j++) {
                std::cout << tau[i][j] << " ";
            }
            std::cout << std::endl;
        }

        std::cout << "Printing tau_prime:" << std::endl;
        for (int i = 0; i < n_customer + 2; i++) {
            for (int j = 0; j < n_customer + 2; j++) {
                std::cout << tau_prime[i][j] << " ";
            }
            std::cout << std::endl;
        }
        write_node_file("nodes.csv",
                        "/home/cuong/CLionProjects/uMV-FSTSP/rand_generated_instances/v" + std::to_string(v), Nodes);
    }
    write_c_prime_to_csv(c_prime, "Cprime.csv",
                         "/home/cuong/CLionProjects/uMV-FSTSP/rand_generated_instances/v" + std::to_string(v));
    write_tau_to_csv(tau, "tau.csv",
                     "/home/cuong/CLionProjects/uMV-FSTSP/rand_generated_instances/v" + std::to_string(v));
    write_tau_to_csv(tau_prime, "tauprime.csv",
                     "/home/cuong/CLionProjects/uMV-FSTSP/rand_generated_instances/v" + std::to_string(v));
}

GenInstance::GenInstance(int n_customer, double dtl, int v, bool gen_node, double side_of_area) {
    createDirectory("/home/cuong/CLionProjects/uMV-FSTSP/50_dis_rand_ins/", "v" + std::to_string(v));
    c_prime = generate1DMatrix(n_customer);
    if (!gen_node) {
        tau = generateSymmetricMatrix(n_customer + 2, n_customer + 2, 20);
        tau_prime = generateSymmetricMatrix(n_customer + 2, n_customer + 2, 20);
        create_and_write_node_file(n_customer, c_prime, "nodes.csv",
                                   "/home/cuong/CLionProjects/uMV-FSTSP/rand_generated_instances/v" + std::to_string(
                                       v));
    }
    else {
        // Generate node info then calculate.
        for (int i = 0; i <= n_customer + 1; i++) {
            double last = 0;
            if (i != n_customer + 1) {
                if (!exist(c_prime, i)) {
                    last = 1;
                }
                if (i <= 1) {
                    auto node = std::make_tuple(i, generateRandomDouble(0, side_of_area / 3),
                                                generateRandomDouble(0, side_of_area / 3), last);
                    Nodes.push_back(node);
                }
                else if (i >= 2 && i <= 3) {
                    auto node = std::make_tuple(i, generateRandomDouble(side_of_area * 2 / 3, side_of_area),
                                                generateRandomDouble(0, side_of_area / 3), last);
                    Nodes.push_back(node);
                }
                else if (i >= 4 && i <= 6) {
                    auto node = std::make_tuple(i, generateRandomDouble(side_of_area / 3, side_of_area * 2 / 3),
                                                generateRandomDouble(side_of_area / 3, side_of_area * 2 / 3), last);
                    Nodes.push_back(node);
                }
                else if (i >= 7 && i <= 8) {
                    auto node = std::make_tuple(i, generateRandomDouble(0, side_of_area / 3),
                                                generateRandomDouble(side_of_area * 2 / 3, side_of_area), last);
                    Nodes.push_back(node);
                }
                else if (i >= 9 && i <= 10) {
                    auto node = std::make_tuple(i, generateRandomDouble(side_of_area * 2 / 3, side_of_area),
                                                generateRandomDouble(side_of_area * 2 / 3, side_of_area), last);
                    Nodes.push_back(node);
                }
            }
            if (i == n_customer + 1) {
                auto n_clone = Nodes[0];
                get<0>(n_clone) = n_customer + 1;
                Nodes.push_back(n_clone);
            }
        }
        std::cout << "Printing nodes" << std::endl;
        for (int i = 0; i < n_customer + 2; i++) {
            std::cout << std::get<0>(Nodes[i]) << " " << std::get<1>(Nodes[i]) << " " << std::get<2>(Nodes[i]) << " " <<
                std::get<3>(Nodes[i]) << std::endl;
        }
        tau.resize(n_customer + 2);
        tau_prime.resize(n_customer + 2);
        for (int i = 0; i < tau.size(); i++) {
            tau[i].resize(n_customer + 2);
            tau_prime[i].resize(n_customer + 2);
        }
        for (int i = 0; i < n_customer + 2; i++) {
            for (int j = 0; j < n_customer + 2; j++) {
                if (i == j || i == n_customer + 1) {
                    tau[i][j] = 0;
                    tau_prime[i][j] = 0;
                }
                else {
                    // truck_speed = 2/3 drone_speed.
                    tau[i][j] = manhattanDistance(
                        get<1>(Nodes[i]), get<2>(Nodes[i]), get<1>(Nodes[j]), get<2>(Nodes[j])) / 3;
                    tau_prime[i][j] = euclideanDistance(
                        get<1>(Nodes[i]), get<2>(Nodes[i]), get<1>(Nodes[j]), get<2>(Nodes[j])) / 15;
                }
            }
        }
        std::cout << "Printing tau:" << std::endl;
        for (int i = 0; i < n_customer + 2; i++) {
            for (int j = 0; j < n_customer + 2; j++) {
                std::cout << tau[i][j] << " ";
            }
            std::cout << std::endl;
        }

        std::cout << "Printing tau_prime:" << std::endl;
        for (int i = 0; i < n_customer + 2; i++) {
            for (int j = 0; j < n_customer + 2; j++) {
                std::cout << tau_prime[i][j] << " ";
            }
            std::cout << std::endl;
        }
        write_node_file("nodes.csv", "/home/cuong/CLionProjects/uMV-FSTSP/50_dis_rand_ins/v" + std::to_string(v),
                        Nodes);
        std::ofstream fout("/home/cuong/CLionProjects/uMV-FSTSP/avg_report.csv", std::ios::app);
        fout << ins_perc_higher_dtl(tau_prime, dtl) << "\n";
    }
    write_c_prime_to_csv(c_prime, "Cprime.csv",
                         "/home/cuong/CLionProjects/uMV-FSTSP/50_dis_rand_ins/v" + std::to_string(v));
    write_tau_to_csv(tau, "tau.csv", "/home/cuong/CLionProjects/uMV-FSTSP/50_dis_rand_ins/v" + std::to_string(v));
    write_tau_to_csv(tau_prime, "tauprime.csv",
                     "/home/cuong/CLionProjects/uMV-FSTSP/50_dis_rand_ins/v" + std::to_string(v));
}

void GenInstance::calculate_avg_ins_perc_dtl_murray(std::string folder_path, double dtl) {
    std::string tau_prime_path = folder_path + "/tauprime.csv";
    std::ifstream t_prime_ifs(tau_prime_path);
    std::vector<std::vector<double>> tau_prime(12, std::vector<double>(12));
    double d;
    char c;
    std::string str;
    for (int i = 0; i < 12; i++) {
        getline(t_prime_ifs, str);
        std::istringstream iss(str);
        for (int j = 0; j < 12; j++) {
            iss >> d >> c;
            tau_prime[i][j] = d;
        }
    }
    double count = 0;
    for (int i = 0; i < tau_prime.size(); i++) {
        for (int j = 0; j < tau_prime[i].size(); j++) {
            if (tau_prime[i][j] > dtl) {
                count++;
            }
        }
    }
    std::ofstream fout("/home/cuong/CLionProjects/uMV-FSTSP/avg_report.csv", std::ios::app);
    fout << folder_path << "," << 100.0 * count / 121.0 << "\n";
}
#endif //UMV_FSTSP_GEN_INSTANCE_H

