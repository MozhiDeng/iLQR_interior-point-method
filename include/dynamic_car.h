#ifndef _ACROBOT_H_
#define _ACROBOT_H_

#include "model.h"
#include "ipddp.h"

/*
    state = [r_x; r_y; heading; kappa]    x-coordinates; y-coordinates; car's heading; kappa
    control = [kappa_rate]    kappa_rate
    Target state = [65.0; -10.0; 0; 0]
*/  
static const double l_f = 3.0;
static const double l_r = -1.0;

class DynamicCar : public Model {
public:
    DynamicCar() {
        x_dims = 4;
        u_dims = 1;
        c_dims = 6;

        _w_kappa = 0.000001;
        _w_kappa_rate = 0.000001;

        _w_terminal_position = 0.000000000000;
        _w_terminal_heading  = 0.000000000000;

        _w_reference_position = 0.000000000001;
        _w_reference_heading  = 0.00000000001;

        speed_const = 9;
    }

    virtual VectorXd dynamics(const VectorXd& x, const VectorXd& u) override {
        VectorXd dx(x_dims);
        dx[0] = speed_const * cos(x[2]);
        dx[1] = speed_const * sin(x[2]);
        dx[2] = speed_const * x[3];
        dx[3] = u[0];
        return dx;
    }
    
    // reference line cost + normal comfort cost
    virtual double cost(const VectorXd& x, const VectorXd& u, const VecofVec2d& road_reference_line) override {
        double cost = 0.0;
        double cost_ref = reference_line_cost(x, road_reference_line);
        double cost_comfort = normal_comfort_cost(x, u);
        cost = cost_ref + cost_comfort;

        return cost;
    }

    // terminal cost
    virtual double terminal_cost(const VectorXd& x, const VectorXd& terminal_state) override {
        double cost_position = 0.0f;
        cost_position = 0.5 * _w_terminal_position * (x[0] - terminal_state[0]) * (x[0] - terminal_state[0]);
        cost_position += 0.5 * _w_terminal_position * (x[1] - terminal_state[1]) * (x[1] - terminal_state[1]);

        double cost_heading = 0.0f;
        cost_heading = 0.5 * _w_terminal_heading * (x[2] - terminal_state[2]) * (x[2] - terminal_state[2]);

        double terminal_cost = 0.0f;
        terminal_cost = cost_position + cost_heading;
        return terminal_cost;
    }

    // constrains function
    virtual VectorXd constrains_function(const VectorXd& x, 
        const VectorXd& u, 
        const VecofVec2d& road_reference_line,
        const VecofVec2d& road_left_side,
        const VecofVec2d& road_right_side) override {

        VectorXd constrains(c_dims);
        VectorXd boundary_constrains(4);
        VectorXd k_constrains(2);
        
        boundary_constrains = boundary_constrains_function(x, u, 
                road_reference_line, 
                road_left_side, 
                road_right_side);
       
        k_constrains = kappa_constrains_function(x);
        
        constrains(0) = boundary_constrains(0);
        constrains(1) = boundary_constrains(1);
        constrains(2) = boundary_constrains(2);
        constrains(3) = boundary_constrains(3);
        constrains(4) = k_constrains(0);
        constrains(5) = k_constrains(1);

        return constrains;
    }

    // road boundary constrains
    virtual VectorXd boundary_constrains_function(const VectorXd& x, 
        const VectorXd& u, 
        const VecofVec2d& road_reference_line,
        const VecofVec2d& road_left_side,
        const VecofVec2d& road_right_side) override {

        VectorXd boundary_constrains(4);
        get_circle_points(x);
        int f_index = 0;
        int r_index = 0;
        f_index = get_reference_close_point(x_f, road_reference_line);
        r_index = get_reference_close_point(x_r, road_reference_line);

        // x_f constrains function
        Vector2d f_vector_n1, f_vector_n2;
        f_vector_n1 = (road_right_side[f_index] - road_left_side[f_index])/
                    (road_right_side[f_index] - road_left_side[f_index]).norm();
        f_vector_n2 = -f_vector_n1;
        double a0, b0, a1, b1;
        a0 = f_vector_n1(0);
        b0 = f_vector_n1(1);
        a1 = f_vector_n2(0);
        b1 = f_vector_n2(1);
        Vector2d x0, x1;
        x0 = road_left_side[f_index];
        x1 = road_right_side[f_index];
      
        // x_r constrains function
        Vector2d r_vector_n1, r_vector_n2;
        r_vector_n1 = (road_right_side[r_index] - road_left_side[r_index])/
                    (road_right_side[r_index] - road_left_side[r_index]).norm();
        r_vector_n2 = -r_vector_n1;
        double a2, b2, a3, b3;
        a2 = r_vector_n1(0);
        b2 = r_vector_n1(1);
        a3 = r_vector_n2(0);
        b3 = r_vector_n2(1);
        Vector2d x2, x3;
        x2 = road_left_side[r_index];
        x3 = road_right_side[r_index];

        // car boundary constrains function
        boundary_constrains(0) = a0 * (x0[0] - x_f[0]) + b0 * (x0[1] - x_f[1]);
        boundary_constrains(1) = a1 * (x1[0] - x_f[0]) + b1 * (x1[1] - x_f[1]);
        boundary_constrains(2) = a2 * (x2[0] - x_r[0]) + b2 * (x2[1] - x_r[1]);
        boundary_constrains(3) = a3 * (x3[0] - x_r[0]) + b3 * (x3[1] - x_r[1]);
        
        return boundary_constrains;
    }

    // kappa constrains
    virtual VectorXd kappa_constrains_function(const VectorXd& x) override {
        VectorXd k_constrains(2);
        k_constrains(0) = x(3) - 0.2;
        k_constrains(1) = -x(3) - 0.2;
        
        return k_constrains;
    }

    // calculate circle point
    void get_circle_points(const VectorXd& x) {
        x_f[0] = x[0] + l_f * cos(x[2]);
        x_f[1] = x[1] + l_f * sin(x[2]);
        x_r[0] = x[0] + l_r * cos(x[2]);
        x_r[1] = x[1] + l_r * sin(x[2]); 
    }

    // get reference line close point index 
    int get_reference_close_point(const VectorXd& x_i, const VecofVec2d& road_reference_line) {
        
        int min_point_index;
        double l_i;
        double l_i_next;
        double l_i_prior;
        Vector2d x_i_xycoordinate;
        x_i_xycoordinate << x_i[0], x_i[1];
    
        for(int j=1; j<road_reference_line.size()-1; j++) {
            l_i = (x_i_xycoordinate-road_reference_line[j]).norm();
            l_i_prior = (x_i_xycoordinate-road_reference_line[j-1]).norm();
            l_i_next = (x_i_xycoordinate-road_reference_line[j+1]).norm();
            if(l_i_prior>l_i && l_i_next>l_i) {
                min_point_index = j;
                break;
            }
        }
        return min_point_index;
    }

    double normal_comfort_cost(const VectorXd& x, const VectorXd& u) {
        // normal comfort cost
        double cost_kappa = 0.0f;
        cost_kappa = 0.5 * _w_kappa * x[3] * x[3];

        double cost_kappa_rate = 0.0f;
        cost_kappa_rate = 0.5 * _w_kappa_rate * u[0] * u[0];

        double normal_comfort_cost = 0.0f;
        normal_comfort_cost = cost_kappa + cost_kappa_rate;

        return normal_comfort_cost;
    }

    // reference line cost
    // reference_line_point which is the min distance point
    double reference_line_cost(const VectorXd& x, const VecofVec2d& road_reference_line) {
        // reference is straight line 
        Vector2d ref_vector, ref_normal_vector;
        Vector2d close_point;
        double desired_heading = 0.0;
        ref_vector = (road_reference_line[5]-road_reference_line[0])/
                        (road_reference_line[5]-road_reference_line[0]).norm();
        ref_normal_vector[0] = -ref_vector[1];
        ref_normal_vector[1] = ref_vector[0];

        int close_point_index;
        close_point_index = get_reference_close_point(x, road_reference_line);
        close_point = road_reference_line[close_point_index];

        double projected_dist = (x[0] - close_point[0]) * ref_normal_vector[0] +
                (x[1] - close_point[1]) * ref_normal_vector[1];
        double heading_diff = x[3] - desired_heading;

        double cost_ref = 0.0;
        cost_ref = 0.5 * _w_reference_position * projected_dist * projected_dist + 
                0.5 * _w_reference_heading * heading_diff * heading_diff;

        return cost_ref;
    }


private:
    double speed_const;
    double _w_kappa, _w_kappa_rate;
    double _w_terminal_position, _w_terminal_heading;
    double  _w_reference_position, _w_reference_heading;
    Vector2d x_f, x_r;
}; 

#endif