//
//  Twiddle.cpp
//  Pid
//
//  Created by Husam Senussi on 13/12/2017.
//  Copyright © 2017 Husam Senussi. All rights reserved.
//

#include "Twiddle.hpp"
#include <string>

#include <cmath>
#include <iostream>

namespace {
    void debug(const char *label , double error, double best_error, const std::array<double, 3>& p, const std::array<double, 3>& dp) {
        std::cout << "DEBUG - " << label << " - ETC : " << error << " Best Error: " << best_error << " p[ " << p[0] << ", " << p[1] << ", " << p[2] << " ] dp [ " << dp[0] << ", " << dp[1] << ", " << dp[2] << "]" << std::endl;   
    }
}


Twiddle::Twiddle(int numberOfIterations, double tolerance)
: _iteration(0),
_numberOfIterations(numberOfIterations),
_tolerance(tolerance),
_total(0.0),
_best_err(0.0557142 ),
_resetRequired(false),
_it(0),
_n(3),
//_p{{3.9369,4.88389, 0.0}},
//_dp{{0.0817821, 0.0999559, 0.0}}
//_p{{0.610137, 24.2477, 0.0}},
//_dp{{0.00908785, 0.370227, 0.0001}}
_p{{0.275822, 9.16176, 0.00135795}},
_dp{{0.0142061, 1.05666, 0.000235795}}{
    _pid.Init(_p[0], _p[2], _p[1]);
    
    initSteps();
    _step = [this](double cte) -> double {
        return _computeSteeringAngle(cte, _step0);
    };
}

double Twiddle::_computeSteeringAngle(double cte, Step next) {
    if (_iteration >= _numberOfIterations){
        // Add error
        _total += std::pow(cte,2);
        std::cout << "DEBUG - Total [" << _iteration  << "]  - ETC : " << cte << " Best Error: " << _best_err << " Total error : " << _total << std::endl;
    }
    
    if (_iteration == 2 * _numberOfIterations) {
        std::cout << "DEBUG - Next [" << _iteration  << "]  - ETC : " << cte << " Best Error: " << _best_err << " Total error : " << _total << std::endl;
        return next(_total / _numberOfIterations);
    }
    
    ++_iteration;
    
    _resetRequired = false;

    //  Update controller.
    _pid.UpdateError(cte);
    // Compute new steering angle.
    double angle =  _pid.TotalError();
    std::cout << "DEBUG - PID [" << _iteration  << "]  - ETC : " << cte << " p_error   : " << _pid._p_error << " d_error : " << _pid._d_error << " i_error : " << _pid._i_error << std::endl;
    std::cout << "DEBUG - Run [" << _iteration  << "]  - ETC : " << cte << " Best Error: " << _best_err << " p[ " << _p[0] << ", " << _p[1] << ", " << _p[2] << " ] dp [ " << _dp[0] << ", " << _dp[1] << ", " << _dp[2] << "] steering angle : " << angle << std::endl;   

    return angle;
}

void Twiddle::initSteps() {
    _step0 = [this](double err) -> double {
        _best_err = err;
        return _step1(err);
    };
    
    _step1 = [this](double err) -> double {
        if (_index == 0 && sum() <= _tolerance)
            return _stop();

        std::cout << "Iteration : " << (++_it) << ", best error : " << _best_err << std::endl;
        _p[_index] += _dp[_index];
        ::debug("Step 1", err, _best_err, _p, _dp);
        return _reset(_step2);
    };
    
    _step2 = [this](double err) -> double {
        if (err < _best_err){
            _best_err    = err;
            _dp[_index] *= 1.1;
            _index       = (_index + 1) % _n;
            ::debug("Step 2.a", err, _best_err, _p, _dp);
            
            return _step1(err);
        } else {
            _p[_index] -= 2 * _dp[_index];
            ::debug("Step 2.b", err, _best_err, _p, _dp);
            return _reset(_step3);
        }
    };
    
    _step3 = [this](double err) -> double  {
        if (err < _best_err) {
            _best_err    = err;
            _dp[_index] *= 1.1;
            ::debug("Step 3.a", err, _best_err, _p, _dp);
        } else {
            _p[_index]  += _dp[_index];
            _dp[_index] *= 0.9;
            ::debug("Step 3.b", err, _best_err, _p, _dp);
        }
        _index = (_index + 1) % _n;
        std::cout << _index << "\n";
        return _step1(err);
    };
}

double Twiddle::_reset(Step next) {
    _resetRequired = true;
    _total         = 0;
    _iteration     = 0;
    Step nextStep  = [this, next](double cte) -> double {
        return _computeSteeringAngle(cte, next);
    };
    this->_step.swap(nextStep);
    _pid.Init(_p[0], _p[2], _p[1]);
    
    return 0.0;
}

double Twiddle::_stop() {
    _resetRequired = true;
    _total         = 0;
    _iteration     = 0;
    Step nextStep  = [this](double cte) -> double {
        _resetRequired = false;
        //  Update controller.
        _pid.UpdateError(cte);
        // Compute new steering angle.
        return _pid.TotalError();
    };
    this->_step.swap(nextStep);
    _pid.Init(_p[0], _p[2], _p[1]);
    
    return 0.0;
}

bool Twiddle::isResetRequired() {
    return _resetRequired;
}

double Twiddle::computeSteeringAngle(double cte) {
    return _step(cte);
}

double Twiddle::sum() {
    return _dp[0] + _dp[1] + _dp[2];
}
