//
// Created by cuong on 18/01/2024.
//
#pragma once
#ifndef UMV_FSTSP_INSTANCE_H
#define UMV_FSTSP_INSTANCE_H
#include <vector>
#include <memory>
#include <fstream>
#include <iostream>
#include <sstream>
#include "set"

class Instance
{
public:
    std::string folder_path;
    int num_node;
    std::vector<std::vector<double>> tau;
    std::vector<std::vector<double>> tau_prime;
    std::vector<std::vector<double>> tau_prime_prime;
    double drone_speed;
    double jetsuite_speed; // Jetsuite speed
    std::vector<int> c_prime;
    std::vector<int> heavy;
    // set of nodes that drone cannot visit
    std::set<int> no_visit = {};
    double maxradius = -1;

    explicit Instance(std::string &folder_path);

    virtual void read() = 0;

    virtual ~Instance() = default;

    //    Instance(const std::string &folder_path, string &dist_type);
    //    static std::vector<std::string> SplitStringWithDelimiter(const std::string& s, const std::string& delimiter);
};

class MurrayInstance final : public Instance
{
public:
    explicit MurrayInstance(std::string &folder_path) : Instance(folder_path) {}

    void read() override;
};

//
class NielsInstance final : public Instance
{
public:
    explicit NielsInstance(std::string &folder_path) : Instance(folder_path) {}
    double truck_speed_factor;
    double drone_speed_factor;
    double jetsuite_speed_factor; // Jetsuite speed factor
    void read() override;
};

class PoikonenInstance final : public Instance
{
public:
    explicit PoikonenInstance(std::string &folder_path) : Instance(folder_path) {}

    void read() override;
};

class InstanceFactory
{
public:
    static std::unique_ptr<Instance> createInstance(const std::string &type, std::string &folder_path)
    {
        if (type == "M")
        {
            return std::make_unique<MurrayInstance>(folder_path);
        }
        else if (type == "N")
        {
            return std::make_unique<NielsInstance>(folder_path);
        }
        else if (type == "P")
        {
            return std::make_unique<PoikonenInstance>(folder_path);
        }
        return nullptr; // Handle unsupported type
    }
};

#endif // UMV_FSTSP_INSTANCE_H
