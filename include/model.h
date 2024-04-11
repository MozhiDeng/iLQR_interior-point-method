#ifndef _DYNAMICS_H_
#define _DYNAMICS_H_

#include "common.h"

class Model {
public:
    virtual VectorXd dynamics(const VectorXd& x, const VectorXd& u) = 0;
    //virtual double normal_comfort_cost(const VectorXd& x, const VectorXd& u) = 0;
    virtual double terminal_cost(const VectorXd& x, const VectorXd& terminal_state) = 0;
   // virtual double reference_line_cost(const VectorXd& x, const VecofVec2d& road_reference_line) = 0;
    virtual double cost(const VectorXd& x, const VectorXd& u, const VecofVec2d& road_reference_line) = 0;
    virtual VectorXd constrains_function(const VectorXd& x, 
        const VectorXd& u, 
        const VecofVec2d& road_reference_line,
        const VecofVec2d& road_left_side,
        const VecofVec2d& road_right_side) = 0;
    virtual VectorXd boundary_constrains_function(const VectorXd& x, 
        const VectorXd& u, 
        const VecofVec2d& road_reference_line,
        const VecofVec2d& road_left_side,
        const VecofVec2d& road_right_side) = 0;
    virtual VectorXd kappa_constrains_function(const VectorXd& x) = 0;

    VectorXd integrate_dynamics(const VectorXd& x, const VectorXd& u, double dt) {
        VectorXd x1 = x + dynamics(x, u)*dt;
        return x1;
    }

    int x_dims;
    int u_dims;
    int c_dims;
};

#endif