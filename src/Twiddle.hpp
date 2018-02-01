#ifndef Twiddle_hpp
#define Twiddle_hpp

#include "PID.h"
#include <functional>
#include <array>
#include <string>
#include <vector>


typedef std::function<std::string(double)> Command;
typedef std::function<double(double)>     Step;

class Twiddle {
    Step _step;
    PID  _pid;
    
    int    _iteration;
    int    _index;
    int    _numberOfIterations;
    double _tolerance;
    double _total;
    double _best_err;
    bool   _resetRequired;
    int    _it;
    int    _n;
    
    std::array<double, 3> _p;
    std::array<double, 3> _dp;

    Step _step0;
    Step _step1;
    Step _step2;
    Step _step3;

    double sum();
    
    double _computeSteeringAngle(double cte, Step next);
    double _reset(Step next);
    double _stop();
    
    
    void initSteps();
public:
    Twiddle(int numberOfIterations = 10, double tolerance = 2.0);
    
    bool   isResetRequired();
    double computeSteeringAngle(double cte);
};

#endif /* Twiddle_hpp */