// Create a log file class for now
#pragma once
// Xiaobin Xiong
// Aug 2st, 2023
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <string.h>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
using namespace Eigen;

// HOW TO USE this
// create an instance in any class. Data_Logger logger;
// initialize the logger by run: logger.init(...);
// add the data pointer to be logged: logger.add_data(&double); logger.add_data(VectorXd)
// in the loop function, run logging: logger.spin_logging();
// that's it, you will get to files in the folder you selected
// one contains the name, types and dimension of all variables,
// one has all the data in binary format.
enum data_types
{
    double_,
    int_,
    VectorXd_,
    VectorXi_,
    VectorXf_,
    quaterniond_
};

class Data_Logger
{
public:
    Data_Logger(std::string FileName, std::string FileLocation = "/log/")
    {
        init(FileName, FileLocation);
    };

    Data_Logger(){};

    ~Data_Logger()
    {
        std::cout << " finishing log file of " << log_file_name << std::endl;
        done_logging();
    };

    //
    void init(std::string FileName, std::string FileLocation = "/log_exp/")
    {
        log_file_name = FileName;
        std::string home = getenv("HOME");
        // home = "/home/well-lab";
        home = "/home/jkang";

        std::string path_Data = home + FileLocation + FileName + "_Data";
        std::cout << " creating log file for " << FileName << std::endl;

        logDataFile.open(path_Data, std::ios::out | std::ios::binary);

        std::string path_Name = home + FileLocation + FileName + "_Name.csv";
        logNameFile.open(path_Name, std::ofstream::out);
    };

    // assuming all data could be int, double, vectorXd, vectorXi
    // when doing logging all data will be converged to float

    void add_data(double *data_ptr, std::string data_name = "some_double")
    {
        void *new_data_prt = data_ptr;

        this->data_ptrs.push_back(new_data_prt);
        this->data_names.push_back(data_name);

        this->map_variable_ptr_to_length.insert({new_data_prt, 1});
        this->map_variable_ptr_to_type.insert({new_data_prt, "double"});

        logNameFile << data_name << ",";
        logNameFile << "double,";

        logNameFile << 1
                    << ",\n";
    }

    void add_data(int *data_ptr, std::string data_name = "some_int")
    {
        void *new_data_prt = data_ptr;

        this->data_ptrs.push_back(new_data_prt);
        this->data_names.push_back(data_name);

        this->map_variable_ptr_to_length.insert({new_data_prt, 1});
        this->map_variable_ptr_to_type.insert({new_data_prt, "int"});

        logNameFile << data_name << ",";
        logNameFile << "int,";
        logNameFile << 1 << ",\n";
    }

    void add_data_vectorXd(double *data_ptr, unsigned int length_, std::string data_name)
    {
        void *new_data_prt = data_ptr;

        this->data_ptrs.push_back(new_data_prt);
        this->data_names.push_back(data_name);
        std::cout << "len" << new_data_prt << std::endl;
        this->map_variable_ptr_to_length.insert({new_data_prt, length_});
        this->map_variable_ptr_to_type.insert({new_data_prt, "VectorXd"});

        logNameFile << data_name << ",";
        logNameFile << "VectorXd,";
        logNameFile << length_ << ",\n";
    }

    void add_data(Eigen::Quaterniond *quat, std::string data_name)
    {
        void *new_data_prt = quat;

        this->data_ptrs.push_back(new_data_prt);
        this->data_names.push_back(data_name);

        this->map_variable_ptr_to_length.insert({new_data_prt, 4});
        this->map_variable_ptr_to_type.insert({new_data_prt, "Quaterniond"});

        logNameFile << data_name << ",";
        logNameFile << "Quaterniond,";
        logNameFile << 4 << ",\n";
    }

    void add_data_vectorXi(int *data_ptr, unsigned int length_, std::string data_name)
    {
        void *new_data_prt = data_ptr;

        this->data_ptrs.push_back(new_data_prt);
        this->data_names.push_back(data_name);

        this->map_variable_ptr_to_length.insert({new_data_prt, length_});
        this->map_variable_ptr_to_type.insert({new_data_prt, "VectorXi"});

        logNameFile << data_name << ",";
        logNameFile << "VectorXi,";
        logNameFile << length_ << ",\n";
    }

    void add_data_vectorXf(float *data_ptr, unsigned int length_, std::string data_name)
    {
        void *new_data_prt = data_ptr;

        this->data_ptrs.push_back(new_data_prt);
        this->data_names.push_back(data_name);

        this->map_variable_ptr_to_length.insert({new_data_prt, length_});
        this->map_variable_ptr_to_type.insert({new_data_prt, "VectorXf"});

        logNameFile << data_name << ",";
        logNameFile << "VectorXf,";
        logNameFile << length_ << ",\n";
    }

    void add_data(std::vector<double> &vec, std::string data_name = "some_vec")
    {
        add_data_vectorXd(vec.data(), vec.size(), data_name);
    }

    void add_data(VectorXd &vec, std::string data_name = "some_vec")
    {
        add_data_vectorXd(vec.data(), vec.size(), data_name);
    }

    void add_data(VectorXf &vec, std::string data_name = "some_vec")
    {
        add_data_vectorXf(vec.data(), vec.size(), data_name);
    }

    void add_data(Vector3d &vec, std::string data_name = "some_vec")
    {
        add_data_vectorXd(vec.data(), vec.size(), data_name);
    }

    void add_data(Vector2d &vec, std::string data_name = "some_vec")
    {
        add_data_vectorXd(vec.data(), vec.size(), data_name);
    }

    void add_data(VectorXi &vec, std::string data_name = "some_vec")
    {
        add_data_vectorXi(vec.data(), vec.size(), data_name);
    }

    // logging data based on ros2 spin frequency
    // or objects that have function being called periodically
    void spin_logging()
    {
        for (auto i_ptr : this->data_ptrs)
        { // switch does not allow string input, convert it to enum types

            switch (string_to_types(map_variable_ptr_to_type[i_ptr]))
            {
            case int_:
            {
                int *int_ptr = (int *)i_ptr;
                log(*int_ptr);
            }
            break;
            case double_:
            {
                double *_double_ptr = (double *)i_ptr;
                log(*_double_ptr);
            }
            break;

            case VectorXi_:
            {
                unsigned int length = map_variable_ptr_to_length[i_ptr];
                VectorXi Xi(length);
                Xi << Map<VectorXi>((int *)i_ptr, length);
                log(Xi);
            }
            break;

            case quaterniond_:
            {
                unsigned int length = 4;
                VectorXd Xd(length);
                Eigen::Quaterniond *quat = (Eigen::Quaterniond *)i_ptr;
                Xd << quat->w(), quat->x(), quat->y(), quat->z();
                log(Xd);
            }
            break;

            case VectorXd_:
            {
                unsigned int length = map_variable_ptr_to_length[i_ptr];
                VectorXd Xd(length);
                Xd << Map<VectorXd>((double *)i_ptr, length);
                log(Xd);
            }
            break;

            case VectorXf_:
            {
                unsigned int length = map_variable_ptr_to_length[i_ptr];
                VectorXf Xf(length);
                Xf << Map<VectorXf>((float *)i_ptr, length);
                log(Xf);
            }
            break;
            }
        }
    };

private:
    // todo: use template
    void log(double data)
    {
        double data_d = static_cast<double>(data);
        logDataFile.write((char *)&(data_d), sizeof(double));
    };

    void log(int data)
    {
        float data_f = static_cast<float>(data);
        logDataFile.write((char *)&(data_f), sizeof(float));
    };

    void log(float data)
    {
        logDataFile.write((char *)&(data), sizeof(float));
    };

    void log(std::vector<float> vector_)
    {
        logDataFile.write(reinterpret_cast<char *>(vector_.data()), (vector_.size()) * sizeof(float));
    };

    void log(VectorXd vector_)
    {
        VectorXd temp = vector_.cast<double>();
        // std::cout << "temp" << temp << std::endl;

        logDataFile.write(reinterpret_cast<char *>(temp.data()), (temp.size()) * sizeof(double));
    };

    void log(VectorXi vector_)
    {
        VectorXf temp = vector_.cast<float>();
        logDataFile.write(reinterpret_cast<char *>(temp.data()), (temp.size()) * sizeof(float));
    };

    void log(VectorXf vector_)
    {
        logDataFile.write(reinterpret_cast<char *>(vector_.data()), (vector_.size()) * sizeof(float));
    };

    data_types string_to_types(std::string const &string_in)
    {
        if (string_in == "int")
            return int_;
        if (string_in == "double")
            return double_;
        if (string_in == "VectorXd")
            return VectorXd_;
        if (string_in == "VectorXf")
            return VectorXf_;
        if (string_in == "VectorXi")
            return VectorXi_;
        if (string_in == "Quaterniond")
            return quaterniond_;
    }
    std::string log_file_name;
    std::vector<std::string> data_names;
    std::vector<void *> data_ptrs;
    std::map<void *, unsigned int> map_variable_ptr_to_length;
    std::map<void *, std::string> map_variable_ptr_to_type;

    std::fstream logDataFile;
    std::fstream logNameFile;

    void done_logging()
    {
        logDataFile.close();
        logNameFile.close();
    };
};