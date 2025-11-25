//
// Created by cuong on 18/01/2024.
//
#include <cmath>
#include <utility>
#include "../include/instance.h"

static std::vector<std::string> SplitStringWithDelimiter(const std::string &s, const std::string &delimiter)
{
    std::vector<std::string> returnValue;
    std::string::size_type start = 0;
    std::string::size_type end = s.find(delimiter);

    while (end != std::string::npos)
    {
        returnValue.push_back(s.substr(start, end - start));
        start = end + 1;
        end = s.find(delimiter, start);
    }

    returnValue.push_back(s.substr(start));
    return returnValue;
}

inline double manhattanDistance(double x1, double y1, double x2, double y2)
{
    return std::abs(x1 - x2) + std::abs(y1 - y2);
}

inline double euclideanDistance(double x1, double y1, double x2, double y2)
{
    return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
}

Instance::Instance(std::string &fp)
{
    folder_path = fp;
}

void MurrayInstance::read()
{
    //    std::cout << "Input is specified as type M: Murray instance.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-" << std::endl;

    std::string c_prime_path = folder_path + "/Cprime.csv";
    std::string nodes_path = folder_path + "/nodes.csv";
    std::string tau_path = folder_path + "/tau.csv";
    std::string tau_prime_path = folder_path + "/tauprime.csv";

    std::ifstream c_ifs(c_prime_path);
    std::ifstream n_ifs(nodes_path);
    std::ifstream t_ifs(tau_path);
    std::ifstream t_prime_ifs(tau_prime_path);

    std::string str;
    getline(c_ifs, str);
    auto c_split_str = SplitStringWithDelimiter(str, ",");
    for (auto &s : c_split_str)
    {
        c_prime.push_back(stoi(s));
    }
    std::vector<double> X_coord;
    std::vector<double> Y_coord;
    //    std::cout << std::endl;
    int n = 0;
    while (getline(n_ifs, str))
    {
        if (n == 0)
        {
            auto n_split_str = SplitStringWithDelimiter(str, ",");
            drone_speed = stod(n_split_str[n_split_str.size() - 1]);
            //            std::cout << "drone speed: " << drone_speed << " miles/minute" << std::endl;
            X_coord.push_back(stod(n_split_str[1]));
            Y_coord.push_back(stod(n_split_str[2]));
        }
        else
        {
            auto n_split_str = SplitStringWithDelimiter(str, ",");
            int x = stoi(n_split_str[n_split_str.size() - 1]);
            if (x == 1)
            {
                heavy.push_back(stoi(n_split_str[0]));
            }
            X_coord.push_back(stod(n_split_str[1]));
            Y_coord.push_back(stod(n_split_str[2]));
        }
        n++;
    }
    //    std::cout << "uav non-eligible customer: ";
    //    for (auto x: heavy) {
    //        std::cout << x << " ";
    //    }
    //    std::cout << std::endl;
    getline(n_ifs, str);
    n -= 1;
    // n = 20;

    num_node = n;
    tau.resize(num_node + 1);
    tau_prime.resize(num_node + 1);
    for (int i = 0; i < tau.size(); i++)
    {
        tau[i].resize(num_node + 1);
        tau_prime[i].resize(num_node + 1);
    }

    double d;
    char c;

    for (int i = 0; i < n + 1; i++)
    {
        getline(t_prime_ifs, str);
        std::istringstream iss(str);
        for (int j = 0; j < n + 1; j++)
        {
            iss >> d >> c;
            tau_prime[i][j] = d;
        }
    }

    for (int i = 0; i < n + 1; i++)
    {
        getline(t_ifs, str);
        std::istringstream iss(str);
        for (int j = 0; j < n + 1; j++)
        {
            iss >> d >> c;

            tau[i][j] = d;
        }
    }
    //    std::cout << "Printing tau:" << std::endl;
    //    for (int i = 0; i < n + 1; i++) {
    //        for (int j = 0; j < n + 1; j++) {
    //            std::cout << tau[i][j] << " ";
    //        }
    //        std::cout << std::endl;
    //    }
    //    std::cout << "Printing tau_prime:" << std::endl;
    //    for (int i = 0; i < n + 1; i++) {
    //        for (int j = 0; j < n + 1; j++) {
    //            std::cout << tau_prime[i][j] << " ";
    //        }
    //        std::cout << std::endl;
    //    }
}

void NielsInstance::read()
{
    //    std::cout << "Input is specified as type N: Niels instance.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-" << std::endl;
    std::ifstream fin(folder_path);
    std::string line;
    std::getline(fin, line);
    // // read restriction, if any
    // if (line[0] == '#') {
    //     while (true) {
    //         if (line.empty()) {
    //             std::getline(fin, line);
    //             break;
    //         }
    //         std::vector<std::string> line_split = SplitStringWithDelimiter(line, " ");
    //         if (line_split[0] == "#MAXFLY")
    //             maxradius = std::stod(line_split[1]);
    //         else if (line_split[0] == "#NOVISIT") {
    //             no_visit.insert(std::stoi(line_split[1]));
    //         }
    //         std::getline(fin, line);
    //     }
    // }
    // keep the previous implementation
    std::getline(fin, line);
    truck_speed_factor = std::stod(line);
    std::getline(fin, line);
    std::getline(fin, line);
    drone_speed_factor = std::stod(line);
    std::getline(fin, line);
    std::getline(fin, line);
    jetsuite_speed_factor = std::stod(line);
    std::cout << "Jetsuite speed factor: " << jetsuite_speed_factor << "\n";
    // Read number of nodes
    std::getline(fin, line); // Skip comment
    std::getline(fin, line);
    num_node = std::stoi(line);
    // std::cout << "Number of Node: " << num_node << "\n";
    std::vector<std::pair<double, double>> coord(num_node);
    std::getline(fin, line);
    std::getline(fin, line);
    auto s = SplitStringWithDelimiter(line, " ");
    coord[0] = std::make_pair(std::stod(s[0]), std::stod(s[1]));
    std::getline(fin, line);

    for (int i = 1; i < num_node; i++)
    {
        std::getline(fin, line);
        s = SplitStringWithDelimiter(line, " ");
        coord[i] = std::make_pair(std::stod(s[0]), std::stod(s[1]));
    }
    // std::cout << "Printing coordinations: " << std::endl;
    // for (auto c : coord)
    // {
    //     std::cout << c.first << " " << c.second << std::endl;
    // }
    // std::cout << "End of printing coord." << std::endl;
    tau.resize(num_node + 1);
    tau_prime.resize(num_node + 1);
    tau_prime_prime.resize(num_node + 1);
    for (int i = 0; i < tau.size(); i++)
    {
        tau[i].resize(num_node + 1);
        tau_prime[i].resize(num_node + 1);
        tau_prime_prime[i].resize(num_node + 1);
    }

    for (int i = 0; i < tau.size(); i++)
    {
        for (int j = 0; j < tau[i].size(); j++)
        {
            if (i == j)
            {
                tau[i][j] = 0;
                tau_prime[i][j] = 0;
            }
            else
            {
                tau[i][j] = euclideanDistance(coord[i].first, coord[i].second, coord[j].first, coord[j].second) * truck_speed_factor;
                tau[i][j] = euclideanDistance(coord[i].first, coord[i].second, coord[j].first, coord[j].second) * truck_speed_factor;
                tau_prime[i][j] = euclideanDistance(coord[i].first, coord[i].second, coord[j].first, coord[j].second) * drone_speed_factor;
            }
        }
    }
    for (int j = 0; j < tau.size(); j++)
    {
        tau[num_node][j] = 0;
        tau_prime[num_node][j] = 0;
    }

    for (int i = 0; i < tau.size(); i++)
    {
        tau[i][num_node] = tau[i][0];
        tau_prime[i][num_node] = tau_prime[i][0];
    }
    // std::cout << "Printing tau: " << std::endl;
    // for (int i = 0; i < tau.size(); i++)
    // {
    //     for (int j = 0; j < tau[i].size(); j++)
    //     {
    //         std::cout << tau[i][j] << " ";
    //     }
    //     std::cout << std::endl;
    // }

    // std::cout << "Printing tau_prime: " << std::endl;
    // for (int i = 0; i < tau_prime.size(); i++)
    // {
    //     for (int j = 0; j < tau_prime[i].size(); j++)
    //     {
    //         std::cout << tau_prime[i][j] << " ";
    //     }
    //     std::cout << std::endl;
    // }
    for (int i = 0; i < num_node; i++)
    {
        for (int j = 0; j < num_node; j++)
        {
            if (i == j)
            {
                tau_prime_prime[i][j] = 0;
            }
            else
            {
                // Calculate Euclidean distance and multiply by Jetsuite speed factor
                tau_prime_prime[i][j] = euclideanDistance(coord[i].first, coord[i].second,
                                                          coord[j].first, coord[j].second) /
                                        jetsuite_speed_factor;
            }
        }
    }

    // // Read tau_prime_prime (Jetsuite travel times)
    // std::getline(fin, line); // Skip Jetsuite travel times comment
    // for (int i = 0; i < num_node; i++)
    // {
    //     std::getline(fin, line);
    //     auto time_row = SplitStringWithDelimiter(line, " ");
    //     for (int j = 0; j < num_node; j++)
    //     {
    //         tau_prime_prime[i][j] = std::stod(time_row[j]);
    //     }
    // }
    // std::cout << "Number of nodes: " << num_node << std::endl;
    // std::cout << "Truck speed factor: " << truck_speed_factor << std::endl;
    // std::cout << "Drone speed factor: " << drone_speed_factor << std::endl;

    // std::cout << "Printing tau: " << std::endl;
    // for (int i = 0; i < tau.size(); i++)
    // {
    //     for (int j = 0; j < tau[i].size(); j++)
    //     {
    //         std::cout << tau[i][j] << " ";
    //     }
    //     std::cout << std::endl;
    // }

    // std::cout << "Printing tau_prime: " << std::endl;
    // for (int i = 0; i < tau_prime.size(); i++)
    // {
    //     for (int j = 0; j < tau_prime[i].size(); j++)
    //     {
    //         std::cout << tau_prime[i][j] << " ";
    //     }
    //     std::cout << std::endl;
    // }

    // std::cout << "Printing tau_prime_prime (Jetsuite):" << std::endl;
    // for (int i = 0; i < tau_prime_prime.size(); i++)
    // {
    //     for (int j = 0; j < tau_prime_prime[i].size(); j++)
    //     {
    //         std::cout << tau_prime_prime[i][j] << " ";
    //     }
    //     std::cout << std::endl;
    // }
}

void PoikonenInstance::read()
{
    //    std::cout << "Input is specified as type P: Poikonen instance.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-" << std::endl;

    std::string tau_path = folder_path + "/tauT.csv";
    std::string tau_prime_path = folder_path + "/tauD.csv";

    std::ifstream t_fin(tau_path);
    std::ifstream tp_fin(tau_prime_path);

    std::string line;
    while (std::getline(t_fin, line))
    { // Read each line of the CSV file
        std::vector<double> row;
        std::istringstream iss(line);
        std::string token;
        while (std::getline(iss, token, ','))
        { // Split each line into individual values
            try
            {
                double value = std::stod(token); // Convert string to integer
                row.push_back(value);            // Store the value in the row vector
            }
            catch (const std::exception &e)
            {
                std::cerr << "Error converting string to double: " << e.what() << std::endl;
            }
        }
        tau.push_back(row); // Add the row vector to the 2D vector representing the matrix
    }
    t_fin.close(); // Close the file

    while (std::getline(tp_fin, line))
    { // Read each line of the CSV file
        std::vector<double> row;
        std::istringstream iss(line);
        std::string token;
        while (std::getline(iss, token, ','))
        { // Split each line into individual values
            try
            {
                double value = std::stod(token); // Convert string to integer
                row.push_back(value);            // Store the value in the row vector
            }
            catch (const std::exception &e)
            {
                std::cerr << "Error converting string to double: " << e.what() << std::endl;
            }
        }
        tau_prime.push_back(row); // Add the row vector to the 2D vector representing the matrix
    }
    num_node = tau.size() - 1;
    for (int i = 1; i < tau.size() - 1; i++)
    {
        c_prime.push_back(i);
    }
    //    std::cout << "Printing tau: " << std::endl;
    //    for (int i = 0; i < tau.size(); i++) {
    //        for (int j = 0; j < tau[i].size(); j++) {
    //            std::cout << tau[i][j] << " ";
    //        }
    //        std::cout << std::endl;
    //    }
    //
    //    std::cout << "Printing tau_prime: " << std::endl;
    //    for (int i = 0; i < tau_prime.size(); i++) {
    //        for (int j = 0; j < tau_prime[i].size(); j++) {
    //            std::cout << tau_prime[i][j] << " ";
    //        }
    //        std::cout << std::endl;
    //    }
}
