#pragma once

#include <string>
#include <fstream>
#include <vector>
#include <utility> // std::pair

class CSVWriter
{
public:
    CSVWriter (std::string filename): my_file(filename){
    }
    ~CSVWriter (){
        my_file.close();
    }

    /**
    * @brief: Write one line to the csv
    * @param: param
    * @return:  return
    */
    void write_csv_line(const std::vector<std::string>& line_data, unsigned int precision=2){

        // Send column names to the stream
        for(int j = 0; j < line_data.size(); ++j)
        {
            my_file << line_data.at(j);
            if(j != line_data.size() - 1) my_file << ","; // No comma at end of line
        }
        my_file << "\n";
    }

private:
    std::ofstream my_file;
};

#include <vector>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <deque>

using namespace std::chrono; 
using namespace std::chrono_literals;
using std::to_string;

std::string to_specified_precision_str(const double& a, unsigned int precision){
    std::stringstream ss;
    ss << std::fixed<<std::setprecision(precision)<<a; 
    return ss.str();
}

/**
* @brief: Manager for 1. Writing sensor data to CSV file 2. filling in an estimate of current floor height, and time 3. switching floors based on user signal
*/
class ElevatorTestManager
{
    public:
        ElevatorTestManager (const std::string& csv_file_name, std::vector<std::string> header):
            csv_writer_(csv_file_name),
            experiment_start_time_(system_clock::now())
        {
            // {"time", "height", "ground_truth_height", "pressure"}
            header.insert(header.begin(), "time");
            header.insert(header.end(), "ground_truth_height");
            csv_writer_.write_csv_line(header);
        }
        ~ElevatorTestManager () = default;
        void switch_floors(){
            // we're switching floors
            if (!switching_floors_){
                switch_start_time_ = system_clock::now();
                switching_floors_ = true;
            }
            else{
                switch_end_time_ = system_clock::now();
                flush_cached_measurements_to_csv();
                current_floor_id_ = (current_floor_id_ + 1)%floors_.size();
                switching_floors_ = false;
            }
        }

        // will write data only when we're not switching floors. When switching floors, we stash the readings and flush them at last
        void write_data(std::vector<std::string> data_str){
            if (!switching_floors_){
                auto now = system_clock::now();
                auto time_elapsed = duration<double>(now - experiment_start_time_).count();
                // double time_elapsed = since(experiment_start_time_).count()/1000;       // ms by default 
                std::string time = to_specified_precision_str(time_elapsed, 2);
                write_full_data_to_csv(data_str, time, to_string(floors_.at(current_floor_id_)));
            }
            else{
                data_str_cache_.emplace_back(data_str);
            }
        }

        bool switching_floors(){
            return switching_floors_;
        }

    private:
        CSVWriter csv_writer_;
        unsigned int current_floor_id_ = 0;
        bool switching_floors_ = false;
        std::vector<double> floors_ = {1.2, 4.7};
        std::chrono::time_point<std::chrono::system_clock> experiment_start_time_;
        std::chrono::time_point<std::chrono::system_clock> switch_start_time_;
        std::chrono::time_point<std::chrono::system_clock> switch_end_time_;
        std::deque<std::vector<std::string>> data_str_cache_;

        /**
        * @brief: Tool function for writing data, time, and ground_truth_height together
        */
        void write_full_data_to_csv(std::vector<std::string>& data_str, const std::string& time, const std::string ground_truth_height){
           data_str.insert(data_str.begin(), time);
           data_str.insert(data_str.end(), ground_truth_height);
           csv_writer_.write_csv_line(data_str);
        }

        /**
        * @brief: Find the start and end time of floor switching, then write the time and linearly interpolated height differece to csv file
        */
        void flush_cached_measurements_to_csv(){
           double time_delta = duration<double>(switch_end_time_ - switch_start_time_).count()/(data_str_cache_.size()-1);  
           double start_time = duration<double>(switch_start_time_ - experiment_start_time_).count();
           unsigned int next_floor_id = (current_floor_id_ + 1)%floors_.size();
           double height_delta = (floors_[next_floor_id] - floors_[current_floor_id_])/(data_str_cache_.size()-1); 
           for (unsigned int i = 0; i < data_str_cache_.size(); ++i) {
               double time_elapsed = start_time + i * time_delta;
               std::string time = to_specified_precision_str(time_elapsed, 2);
               std::string ground_truth_height = to_string(floors_[current_floor_id_] + i * height_delta);
               auto& data_str = data_str_cache_[i];
               write_full_data_to_csv(data_str, time, ground_truth_height);
           }
           data_str_cache_.clear();
        }
};
