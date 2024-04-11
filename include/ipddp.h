#ifndef _IPDDP_H_
#define _IPDDP_H_

#include "common.h"
#include "model.h"

#include <memory>
#include <numeric>
#include <stdexcept>
#include <vector>

static const int maxIter = 1000;
static const double r_envelop = 1.2;

static std::vector<double> alpha_vec = {1,	0.500000000000000,	0.250000000000000,	
    0.125000000000000,	0.0625000000000000,	0.0312500000000000,	0.0156250000000000,
    0.00781250000000000,	0.00390625000000000,	0.00195312500000000,	0.000976562500000000};
static Eigen::Map<VectorXd> Alpha(alpha_vec.data(), alpha_vec.size());

class IPDDP {
public:
  IPDDP(Model* p_dyn, double time_delta): dt(time_delta) {
    model.reset(p_dyn);

    Qx.resize(model->x_dims); 
    Qu.resize(model->u_dims);
    Qxx.resize(model->x_dims, model->x_dims);
    Qxu.resize(model->x_dims, model->u_dims);
    Quu.resize(model->u_dims, model->u_dims);

    ku_i.resize(model->u_dims);
    Ku_i.resize(model->u_dims, model->x_dims);
    ky_i.resize(model->c_dims);
    Ky_i.resize(model->c_dims, model->x_dims);
    ks_i.resize(model->c_dims);
    Ks_i.resize(model->c_dims, model->x_dims);


    Qux_reg.resize(model->x_dims, model->u_dims);
    QuuF.resize(model->u_dims, model->u_dims);

    Qsx.resize(model->c_dims, model->x_dims);
    Qsu.resize(model->c_dims, model->u_dims);
    
    tmp_filter(0) = std::numeric_limits<double>::infinity();
    tmp_filter(1) = std::numeric_limits<double>::infinity();
    filter.push_back(tmp_filter);

    terminal_state.resize(4);
    terminal_state << 55.0, 0.0, 0.1, 0;

    Eigen::initParallel();
  }
  IPDDP() = default;

  VectorXd terminal_state;
  VecofVec2d road_left_side;
  VecofVec2d road_right_side;
  VecofVec2d road_reference_line;

  std::shared_ptr<Model> model;   

  void generate_trajectory(int index_data);   
  void generate_trajectory(const VectorXd& x_0, 
      const VecOfVecXd& u0, 
      std::vector<std::vector<double>>& rode_date, 
      int index_data); //fresh start 
  // initial trajectory; x_0: initial state;  u_0: initial control sequence
  double init_traj(const VectorXd &x_0, 
      const VecOfVecXd &u_0, 
      std::vector<std::vector<double> > road_date,
      int index_data);   

private:
  double dt;
  int T;  // number of state transitions

  struct Alg{
    double tolerance;
    double mu;
    bool infeasible;
    Alg():tolerance(1e-4), mu(0.0), infeasible(true){};
  }alg;

  // n = dims(state), m = dims(control)
  VecOfMatXd fx; //nxnx(T+1)
  VecOfMatXd fu; //nxmx(T+1)
  VecOfVecXd cx; //nx(T+1)
  VecOfVecXd cu; //mx(T+1)
  VecOfMatXd cxx; //nxnx(T+1)
  VecOfMatXd cxu; //nxmx(T+1)
  VecOfMatXd cuu; //mxmx(T+1)

  Vector2d dV;    //2x1
  VecOfVecXd Vx;  //nx(T+1)
  VecOfMatXd Vxx; //nxnx(T+1)

  VecOfVecXd ku;
  VecOfVecXd ky;
  VecOfVecXd ks;

  VecOfMatXd Ku;
  VecOfMatXd Ky;
  VecOfMatXd Ks;

  VectorXd x0;
  VecOfVecXd xs;          // s: "step". current working sequence
  VecOfVecXd x_old;
  VecOfVecXd x_new;
  
  VecOfVecXd us;
  VecOfVecXd u_old;
  VecOfVecXd u_new;

  VecOfVecXd y_t;
  VecOfVecXd y_t_old;
  VecOfVecXd y_t_new;    // s: "step". current working sequence

  VecOfVecXd s;
  VecOfVecXd s_old;
  VecOfVecXd s_new;

  VecOfVecXd mu_set;
  VecOfVecXd mus_set;

  VecOfVecXd constrains;
  VecOfVecXd constrains_new;
  VecOfVecXd constrains_old;
  VecOfMatXd constrains_x;
  VecOfMatXd constrains_u;

  Vector2d tmp_filter;
  std::vector<Vector2d> filter;

  bool flagChange;
  double Qu_err, mu_err, constrains_err;
  double total_cost, bp_opterr;
  int bp_regularization, bp_failed, bp_recovery, fp_step, fp_failed, failed, iter;
  
  VectorXd s_curr, y_t_curr, constrains_curr;
  VectorXd Qx, Qu, ku_i, ky_i, ks_i, r_d, r_hat, y_inv, c_inv;
  MatrixXd Qxx, Quu, Qxu, Qux_reg, QuuF, Ku_i, Ky_i, Ks_i, Qsx, Qsu, Quu_reg, R, kK;

  void calculate_roadside_and_reference_line(std::vector<std::vector<double>>& road_date,
    VecofVec2d& road_left_side,
    VecofVec2d& road_right_side,
    VecofVec2d& road_reference_line);
  void compress_road_side();

  void forward_pass();
  void backward_pass();

  void get_dynamics_derivatives(const VecOfVecXd &x, const VecOfVecXd &u, VecOfMatXd& f_x, VecOfMatXd& f_u);
  void get_cost_derivatives(const VecOfVecXd &x, const VecOfVecXd &u, VecOfVecXd& c_x, VecOfVecXd& c_u);
  void get_cost_2nd_derivatives(const VecOfVecXd &x, 
    const VecOfVecXd &u, 
    VecOfMatXd& c_xx, 
    VecOfMatXd& c_xu, 
    VecOfMatXd& c_uu);
  void get_constrains_derivatives(const VecOfVecXd& x, 
  const VecOfVecXd& u, 
  VecOfMatXd& constrains_x, 
  VecOfMatXd& constrains_u);

  void calculate_cxx(const VecOfVecXd &x, const VecOfVecXd &u, VecOfMatXd& c_xx, int start_T, int end_T);
  void calculate_cxu(const VecOfVecXd &x, const VecOfVecXd &u, VecOfMatXd& c_xu, int start_T, int end_T);
  void calculate_cuu(const VecOfVecXd &x, const VecOfVecXd &u, VecOfMatXd& c_uu, int start_T, int end_T);


  double initialroll(const VectorXd& x0, const VecOfVecXd& u);
  void reset_filter();
  void reset_reg();
  
  void set_parameter_zeros(VecOfVecXd& u_new, 
      VecOfVecXd& y_new,  
      VecOfVecXd& s_new, 
      VecOfVecXd& constrains_new);
  
  bool check_y_snew(VectorXd& y_new, 
    VectorXd&s_new, 
    VectorXd& y_old, 
    VectorXd& s_old, 
    double& tau);
  bool check_c_snew(VectorXd& s_new, 
    VectorXd& s_old, 
    VectorXd& constrains_v_new, 
    VectorXd& constrains_v_old, 
    double& tau);
  bool check_candidate(std::vector<Vector2d>& filter, const Vector2d& candidate);

  void update_filter(std::vector<Vector2d>& filter, const Vector2d& candidate);
  void output_to_csv(const std::string filename);
  void output_road_to_csv(const std::string filename);
};

#endif