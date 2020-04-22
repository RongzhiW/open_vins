//
// Created by dji on 20-4-22.
//

#ifndef PROJECT_TIC_TOC_H
#define PROJECT_TIC_TOC_H
#include <chrono>
#include <cstdlib>
#include <ctime>
#include <string>

class TicToc {
  public:
    TicToc(const std::string& step_name="") : step_name_{step_name} {
       tic();
       sum_t = 0.0;
    }
    void tic() {
        start_ = std::chrono::system_clock::now();
    }
    double toc() {
        end_ = std::chrono::system_clock::now();
        auto elapsed_ms = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(end_ - start_);
        sum_t += elapsed_ms.count();
        return elapsed_ms.count();
    }

 private:
  std::chrono::time_point<std::chrono::system_clock> start_, end_;
    double sum_t;
    const std::string step_name_;
};

#endif //PROJECT_TIC_TOC_H
