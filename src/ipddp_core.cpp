#define SHOWPROGRESS
#define TIMESTUFF
#define IPDDP_CORE

#include "ipddp.h"
#include <algorithm>
#include <numeric>
#include "operator.h"

using std::cout;
using std::endl;

// initialize: us, xs, y_t, s, mu_set
double IPDDP::init_traj(const VectorXd& x_0, 
        const VecOfVecXd& u_0, 
        std::vector<std::vector<double> > road_date,
        int index_data) {
    
    // initialize feasible area(circle point)
    calculate_roadside_and_reference_line(road_date,
        road_left_side,
        road_right_side,
        road_reference_line);
    compress_road_side();
    std::string compress_road_path = "/home/dmz/Baidu/ipddp_test/compress_road_path_data"; 
    std::string compress_fileName = compress_road_path + "/" + 
            "road_path_compress" + std::to_string(index_data) + ".csv";
    output_road_to_csv(compress_fileName);
    
    T = u_0.size();
    // initialize xs and us
    xs.resize(T+1);
    x0 = xs[0] = x_0;
    us = u_0;

    // initialization parameter: y_t, s, mu_set
    VectorXd y_t_init(6);
    y_t_init.setOnes();
    y_t.resize(T);
    for (int i=0; i<T; i++) y_t[i] = 0.01*y_t_init;

    VectorXd s_init(6); 
    s_init.setOnes();
    s.resize(T);
    for (int i=0; i<T; i++) s[i] = 0.1*s_init;

    VectorXd mu_init(6); 
    mu_init.setOnes();
    mu_set.resize(T);
    for (int i=0; i<T; i++) mu_set[i] = 0.001*mu_init;

    y_t_new.resize(T);
    s_new.resize(T);
    mus_set.resize(T);

    constrains.resize(T);
    constrains_x.resize(T);
    constrains_u.resize(T);
    
    // initialize:us, constrains, xs; return:total_cost
    double cost_init = initialroll(x0, us); 
    //cout << "cost_init = " << cost_init << endl;

    //allocate everything
    fx.resize(T+1);
    fu.resize(T+1);
    cx.resize(T+1);
    cu.resize(T+1);
    cxu.resize(T+1);
    cxx.resize(T+1);
    cuu.resize(T+1);

    dV = Vector2d(model->u_dims,1);
    Vx.resize(T+1);
    Vxx.resize(T+1);

    ku.resize(T);
    Ku.resize(T);
    ky.resize(T);
    Ky.resize(T);
    ks.resize(T);
    Ks.resize(T);

    std::fill(fx.begin(), fx.end(), MatrixXd::Zero(model->x_dims,model->x_dims));    // initialize the matrix container
    std::fill(fu.begin(), fu.end(), MatrixXd::Zero(model->x_dims,model->u_dims));
    std::fill(cx.begin(), cx.end(), VectorXd::Zero(model->x_dims));
    std::fill(cu.begin(), cu.end(), VectorXd::Zero(model->u_dims));
    std::fill(cxx.begin(), cxx.end(), MatrixXd::Zero(model->x_dims,model->x_dims));
    std::fill(cxu.begin(), cxu.end(), MatrixXd::Zero(model->x_dims,model->u_dims));
    std::fill(cuu.begin(), cuu.end(), MatrixXd::Zero(model->u_dims,model->u_dims));
    std::fill(Vx.begin(), Vx.end(), VectorXd::Zero(model->x_dims));
    std::fill(Vxx.begin(), Vxx.end(), MatrixXd::Zero(model->x_dims,model->x_dims));
    std::fill(ku.begin(), ku.end(), VectorXd::Zero(model->u_dims));
    std::fill(Ku.begin(), Ku.end(), MatrixXd::Zero(model->u_dims,model->x_dims));
    std::fill(ks.begin(), ks.end(), VectorXd::Zero(model->c_dims));
    std::fill(Ks.begin(), Ks.end(), MatrixXd::Zero(model->c_dims,model->x_dims));
    std::fill(ky.begin(), ky.end(), VectorXd::Zero(model->c_dims));
    std::fill(Ky.begin(), Ky.end(), MatrixXd::Zero(model->c_dims,model->x_dims));
    std::fill(constrains_x.begin(), constrains_x.end(), MatrixXd::Zero(model->c_dims, model->x_dims));
    std::fill(constrains_u.begin(), constrains_u.end(), MatrixXd::Zero(model->c_dims, model->u_dims));

}

void IPDDP::generate_trajectory(const VectorXd& x_0, 
        const VecOfVecXd& u0, 
        std::vector<std::vector<double>>& rode_date,
        int index_data) {
    
    // initialize: us, xs, constrains, y_t, s, mu_set
    init_traj(x_0, u0, rode_date, index_data);    

    generate_trajectory(index_data);
}

void IPDDP::generate_trajectory(int index_data) {

    #ifdef TIMESTUFF
    auto all_start = std::chrono::system_clock::now();
    #endif
    
    // initial mu
    if (alg.mu==0) alg.mu = total_cost/T/s[0].size();

    //cout << "alg.mu = " << alg.mu << endl;
    //cout << "alg.tolerance =" << alg.tolerance << endl;
    reset_filter();
    reset_reg();
   
    // constants, times, counters
    flagChange = true;

    for (iter=0; iter<maxIter; iter++) {

        #ifdef TIMESTUFF
        auto start = std::chrono::system_clock::now();
        #endif
        //cout << "--------------------" << endl;
        //cout << "iter = " << iter << endl;
        while(true) {
            if (flagChange) {
                get_dynamics_derivatives(xs, us, fx, fu);
                get_cost_derivatives(xs, us, cx, cu);
                get_cost_2nd_derivatives(xs, us, cxx, cxu, cuu);    // Update cxx, cxu, cuu
                get_constrains_derivatives(xs, us, constrains_x, constrains_u);
                flagChange = 0;
            }
            backward_pass();
            if(bp_failed==0) break;     
        } 

        forward_pass();

        #ifdef TIMESTUFF
        auto end = std::chrono::system_clock::now();
        long int time = std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count();
        #endif
        
        // terminal condition
        if (std::max<double>(bp_opterr, alg.mu) <= alg.tolerance) {
           // cout << "Optimality reached" << endl;
            break; 
        }

        // Update mu
        if (bp_opterr <= 0.2*alg.mu) {
            alg.mu = std::max(alg.tolerance/10, std::min<double>(0.2*alg.mu, pow(alg.mu, 1.2)));
            cout << "update mu" << endl;
            reset_filter();
            reset_reg();
        }
        //cout << "mu = " << alg.mu << endl;
    }

    #ifdef TIMESTUFF
    auto all_end = std::chrono::system_clock::now();
    #endif

    cout << "iter = " << iter << endl;
    std::string outputPath = "/home/dmz/Baidu/ipddp_test/road_path_planning_data";
    std::string fileName = outputPath + "/" +"ipddp_result" + std::to_string(index_data) + ".csv";
    //cout << fileName << endl;
    output_to_csv(fileName);

    //record result and print log
    double path_cost = 0;
    for(int i=0; i<T; i++) {
        path_cost += model->cost(xs[i], us[i], road_reference_line);
    }
    path_cost += model->terminal_cost(xs[T], terminal_state);
    cout << "path_cost = " << path_cost << endl;

    VectorXd constrains_vector(6);
    double max_breach_constrains = 0.0;
    for(int j=0; j<T; j++) {
        VectorXd boundary_constrains_vector(4);
        boundary_constrains_vector = model->boundary_constrains_function(xs[j], us[j], 
                road_reference_line, 
                road_left_side, 
                road_right_side);
        VectorXd kappa_constrains(2);
        kappa_constrains = model->kappa_constrains_function(xs[j]);
        constrains_vector(0) = boundary_constrains_vector(0);
        constrains_vector(1) = boundary_constrains_vector(1);
        constrains_vector(2) = boundary_constrains_vector(2);
        constrains_vector(3) = boundary_constrains_vector(3);
        constrains_vector(4) = kappa_constrains(0);
        constrains_vector(5) = kappa_constrains(1);

        for(int k=0; k<constrains_vector.size(); k++) {
            double abs_number = std::abs(constrains_vector(k));
            if(abs_number>max_breach_constrains) max_breach_constrains = abs_number;
        }
    }
    cout << "max_breach_constrains = " << max_breach_constrains << endl;

}

void IPDDP::calculate_roadside_and_reference_line(std::vector<std::vector<double>>& road_date,
    VecofVec2d& road_left_side,
    VecofVec2d& road_right_side,
    VecofVec2d& road_reference_line) {
    Eigen::Vector2d tmp_left_point, tmp_right_point, tmp_reference_point;

    for(const auto& rode_date_row : road_date) {

        tmp_reference_point[0] = rode_date_row[0];
        tmp_reference_point[1] = rode_date_row[1];
        tmp_left_point[0] = rode_date_row[2];
        tmp_left_point[1] = rode_date_row[3];
        tmp_right_point[0] = rode_date_row[4];
        tmp_right_point[1] = rode_date_row[5];

        road_reference_line.push_back(tmp_reference_point);
        road_left_side.push_back(tmp_left_point);
        road_right_side.push_back(tmp_right_point);
    }
}

void IPDDP::compress_road_side() {
    Vector2d left_to_right_vector, right_to_left_vector;
    for(int i=0; i<road_left_side.size(); i++) {
        left_to_right_vector = (road_right_side[i]-road_left_side[i])/(road_right_side[i]-road_left_side[i]).norm();
        right_to_left_vector = - left_to_right_vector;
        road_left_side[i] += left_to_right_vector * r_envelop;
        road_right_side[i] += right_to_left_vector * r_envelop;
    }
}

void IPDDP::forward_pass() {
        
    x_old = xs;
    u_old = us;
    y_t_old = y_t;
    s_old = s;
    constrains_old = constrains;

    double tau = std::max(0.99, 1-alg.mu);
    double alpha;

    // line-search
    for (int i=0; i<Alpha.size(); i++) {
        
        alpha = Alpha(i);
        fp_step = i + 1;
        failed = 0;

        // set zeros 
        x_new.resize(T+1);
        x_new[0] = x0;
        set_parameter_zeros(u_new, y_t_new, s_new, constrains_new);

        if (alg.infeasible) {
            
            for (unsigned int j=0; j<T; j++) {
                y_t_new[j] = y_t_old[j] + ky[j]*alpha + Ky[j]*(x_new[j] - x_old[j]).matrix();
                s_new[j] = s_old[j] + ks[j]*alpha + Ks[j]*(x_new[j] - x_old[j]).matrix();

                if(check_y_snew(y_t_new[j], s_new[j], y_t_old[j], s_old[j], tau)) {
                    failed = 1;
                    break;
                }
                u_new[j] = u_old[j] + ku[j]*alpha + Ku[j]*(x_new[j] - xs[j]);
                x_new[j+1] = model->integrate_dynamics(x_new[j], u_new[j], dt);
            }

        } else {
            for (unsigned int j=0; j<us.size(); j++) {
                s_new[j] = s_old[j] + ks[j]*alpha + Ks[j]*(x_new[j] - x_old[j]);
                u_new[j] = u_old[j] + ku[j]*alpha + Ku[j]*(x_new[j] - x_old[j]);
                constrains_new[j] = model->constrains_function(x_new[j], u_new[j],
                        road_reference_line,
                        road_left_side,
                        road_right_side);
              
                if (check_c_snew(constrains_new[j], constrains_old[j], s_new[j], s_old[j], tau)) {
                    failed = 1;
                    break;
                }
                // Calculate state x
                x_new[j+1] = model->integrate_dynamics(x_new[j], u_new[j], dt);
            }
        }

        if (failed!=0) {
            continue;
        } else {
            double new_cost = 0;
            for(int i=0; i<T; i++) {
                new_cost += model->cost(x_new[i], u_new[i], road_reference_line);
            }
            new_cost += model->terminal_cost(x_new[T], terminal_state);
            //cout << "new_cost = " << new_cost << endl;

            double log_cost, err;
            if (alg.infeasible) {
                // calculate log_cost
                double y_new_sum = 0.0;
                for (const auto& ypf : y_t_new) {
                    for (int i=0; i<ypf.size(); i++) {
                        double yd = ypf(i);
                        y_new_sum += std::log(yd); 
                    }
                }
                log_cost =  new_cost - alg.mu*y_new_sum;

                // calculate err
                for (int t=0; t<T; t++) {
                    constrains_new[t] = model->constrains_function(x_new[t], u_new[t], 
                            road_reference_line,
                            road_left_side,
                            road_right_side);
                }
                VecOfVecXd cy = (constrains_new+y_t_new);
                double norm_err = 0.0;
                for (const auto& cy_p : cy) {
                    for(int i=0; i<cy_p.size(); i++) {
                        norm_err += std::abs(cy_p(i));
                    }
                }
                err = std::max<double>(alg.tolerance, norm_err);
            } else {
                double log_c_sum = 0.0;
                for (const auto& vecxd : constrains_new) {
                    for (int i=0; i<vecxd.size(); i++) {
                        double x = vecxd(i);
                        log_c_sum += std::log(-x);
                    }
                }
                log_cost = new_cost - alg.mu*log_c_sum;
                err = 0;
            }
            Vector2d candidate;
            candidate << log_cost, err;  
            // cout << "candidate = \n" << candidate << endl;
            // for(int j=0; j<filter.size(); j++) {
            //     cout << "filter[" << j << "] = \n" << filter[j] << endl;
            // }

            if (check_candidate(filter, candidate)){
                failed = 2;
                continue;
            } else {
                update_filter(filter, candidate);
                break;
            }
        }
    }

    if (failed!=0) {
        bp_failed = failed;
    } else {
        xs = x_new;
        us = u_new;
        s = s_new;
        y_t = y_t_new;
        constrains = constrains_new;
        fp_failed = 0; 

        //cout << "bp_reg = " << bp_regularization << endl;                           // forward pass succeed
        //cout << "alpha = " << alpha << endl;  
        //cout << "-----------------------" << endl;  
    }
    flagChange = true;                          // forward_pass line search over and can do backward_pass
}

bool IPDDP::check_candidate(std::vector<Vector2d>& filter, const Vector2d& candidate) {
    bool checkflag;
    for(int i=0; i<filter.size(); i++) {
        if(candidate(0)>=filter[i](0) && candidate(1)>=filter[i](1)) {
            checkflag = true; 
        } else {
            checkflag = false;
            break;
        }
    }
    return checkflag;
}

void IPDDP::update_filter(std::vector<Vector2d>& filter, const Vector2d& candidate) {
    std::vector<int> index;
    for(int i=0; i<filter.size(); i++) {
        if(candidate(0)<=filter[i](0) && candidate(1)<=filter[i](1)) {
            index.push_back(i);
        }
    }
    filter.push_back(candidate);
    for(int j=index.size()-1; j>=0; j--) {
        filter.erase(filter.begin()+index[j]);
    }
}


/*
    INPUTS
      n = dims(state), m = dims(control), dim_c = dims(constrains)
      cx: nx(T+1)          cu: mx(T+1)
      cxx: nxnx(T+1)       cxu: nxmx(T+1)   cuu: mxmx(T+1)
      fx: nxnx(T+1)        fu: nx2x(T+1)    fxx: none
      fxu: None            fuu: none        u: 2xT
      s: dim_cxT           y_t: dim_cxT     constrains:dim_cxT
    OUTPUTS
      Vx: nx(T+1)          Vxx: nxnx(T+1)   dV: 2x1
      ku: mxT              Ku: mxnxT
      ks: dim_cxT          Ks: dim_cxnxT
      ky: dim_cxT          Ky: dim_cxnxT

*/
void IPDDP::backward_pass() {
    
    dV.setZero();
    Qu_err = 0;
    mu_err = 0;
    constrains_err = 0;

    if (bp_failed || fp_failed) {
        bp_regularization += 1;
    } else if (fp_step==1) {
        bp_regularization -= 1;
    } else if (fp_step <= 4) {
        bp_regularization = bp_regularization;
    } else {
        bp_regularization += 1;
    }

    if (bp_regularization<0) {
        bp_regularization = 0;
    } else if (bp_regularization>24) {
        bp_regularization = 24;
    }
 
    //cout <<"bp_reg = " << bp_regularization << endl;
    // cost-to-go at end
    Vx[T] = cx[T];
    Vxx[T] = cxx[T];

    for (int i=(T-1); i>=0; i--) {

        s_curr = s[i];
        y_t_curr = y_t[i];
        constrains_curr = constrains[i];
      
        Qsx = constrains_x[i];
        Qsu = constrains_u[i];
      
        Qx = cx[i] + (Qsx.transpose() * s_curr) + (fx[i].transpose() * Vx[i+1]);
        Qu = cu[i] + (Qsu.transpose() * s_curr) + (fu[i].transpose() * Vx[i+1]); 
        Qxx = cxx[i] + (fx[i].transpose() * Vxx[i+1] * fx[i]);
        Qxu = cxu[i] + (fx[i].transpose() * Vxx[i+1] * fu[i]);
        Quu = cuu[i] + (fu[i].transpose() * Vxx[i+1] * fu[i]);

        Eigen::DiagonalMatrix<double, Eigen::Dynamic> S(s_curr.size());
        S.diagonal() = s_curr;
       
        Quu_reg = Quu + cuu[i]*(pow(1.6, bp_regularization)-1);
        //Quu_reg = (1 + pow(1.6, bp_regularization)) * Quu;

        if (alg.infeasible) {
            r_d = s_curr.array() * y_t_curr.array() - alg.mu;   
            r_hat = (s_curr.array()*(constrains_curr+y_t_curr).array()).matrix() - r_d.matrix();

            y_inv = 1 / y_t_curr.array();
            Eigen::DiagonalMatrix<double, Eigen::Dynamic> SY_inv((s_curr.array()*y_inv.array()).size());
            SY_inv.diagonal() = (s_curr.array()*y_inv.array()).matrix();

            // Cholesky factorization
            Eigen::LLT<MatrixXd> llt_infeasible(Quu_reg+Qsu.transpose()*SY_inv*Qsu);
            if(llt_infeasible.info()==Eigen::Success) {
                R = llt_infeasible.matrixL();
            } else {
                bp_failed = 1;
                break;
            }

            MatrixXd tmp_infeasible;

            int tem_infeasible_rows = (Qu+Qsu.transpose()*(y_inv.array()*r_hat.array()).matrix()).rows();
            int tem_infeasible_cols = (Qu+Qsu.transpose()*(y_inv.array()*r_hat.array()).matrix()).cols() + 
                                        (Qxu.transpose() + Qsu.transpose() * SY_inv * Qsx).cols();
            tmp_infeasible.resize(tem_infeasible_rows, tem_infeasible_cols);
            tmp_infeasible << Qu + Qsu.transpose() * (y_inv.array()*r_hat.array()).matrix(),
                              Qxu.transpose()+ Qsu.transpose() * SY_inv * Qsx;

            kK = -R.partialPivLu().solve(R.transpose().partialPivLu().solve(tmp_infeasible));   // ???????

            ku_i = kK.col(0);
            Ku_i = kK.rightCols(kK.cols()-1);
            ks_i = y_inv.array() * (r_hat+S*Qsu*ku_i).array();
            Ks_i = SY_inv * (Qsx+Qsu*Ku_i);
            ky_i = -(constrains_curr+y_t_curr).matrix() - Qsu*ku_i.matrix();
            Ky_i = -Qsx - Qsu*Ku_i;

            Quu = Quu + Qsu.transpose() * SY_inv * Qsu;
            Qxu = Qxu + Qsx.transpose() * SY_inv * Qsu;
            Qxx = Qxx + Qsx.transpose() * SY_inv * Qsx;
           
            Qu = Qu + Qsu.transpose() * (y_inv.array()*r_hat.array()).matrix();
            Qx = Qx.matrix() + Qsx.transpose() * (y_inv.array()*r_hat.array()).matrix();
            
        } else {
            r_d = S * constrains_curr.matrix() + alg.mu * VectorXd::Ones(S.rows()).matrix();
            c_inv = 1 / constrains_curr.array();

            Eigen::DiagonalMatrix<double, Eigen::Dynamic> SC_inv((s_curr.array()*c_inv.array()).size());
            SC_inv.diagonal() = s_curr.array()*c_inv.array();

            //Cholesky factorization
            Eigen::LLT<MatrixXd> llt(Quu_reg-Qsu.transpose()*SC_inv*Qsu);
            if(llt.info()==Eigen::Success) {
                R = llt.matrixL();
            } else {
                bp_failed = 1;
                break;
            }


            MatrixXd tem_feasible;
            int tem_feasible_rows = (Qxu.transpose()-Qsu.transpose()*SC_inv*Qsx).rows();
            int tem_feasible_cols = (Qu-Qsu.transpose()*(c_inv.array()*r_d.array()).matrix()).cols() + 
                                    (Qxu.transpose()-Qsu.transpose()*SC_inv*Qsx).cols();
            tem_feasible.resize(tem_feasible_rows, tem_feasible_cols);
            tem_feasible << Qu-Qsu.transpose()*(c_inv.array()*r_d.array()).matrix(),
                            Qxu.transpose()-Qsu.transpose()*SC_inv*Qsx;

            kK = -R.partialPivLu().solve(R.transpose().colPivHouseholderQr().solve(tem_feasible));

            ku_i = kK.col(0);
            Ku_i = kK.rightCols(kK.cols()-1);
            ks_i = -c_inv.array() * (r_d+S*Qsu*ku_i).array();
            Ks_i = -SC_inv.toDenseMatrix() * (Qsx+Qsu*Ku_i);
            ky_i = MatrixXd::Zero(model->c_dims,1);
            Ky_i = MatrixXd::Zero(model->c_dims,model->x_dims);

            Quu = Quu - Qsu.transpose()*SC_inv*Qsu;
            Qxu = Qxu - Qsx.transpose()*SC_inv*Qsu;
            Qxx = Qxx - Qsx.transpose()*SC_inv*Qsx;

            Qu = Qu - Qsu.transpose()*(c_inv.array()*r_d.array()).matrix();
            Qx = Qx - Qsx.transpose()*(c_inv.array()*r_d.array()).matrix();
        }
        
        // Update cost-to-go approximation.
        dV(0) += ku_i.transpose()*Qu;
        dV(1) += 0.5*ku_i.transpose()*Quu*ku_i;

        Vx[i] = Qx.matrix() + Ku_i.transpose()*Qu.matrix() + Ku_i.transpose()*Quu*ku_i.matrix() + Qxu*ku_i.matrix();
        Vxx[i] = Qxx + Ku_i.transpose()*Qxu.transpose() + Qxu*Ku_i + Ku_i.transpose()*Quu*Ku_i;

        //cout << "Vx[" <<i<<"]"<<Vx[i]<<endl;
        // Save control gains
        ku[i] = ku_i;
        Ku[i] = Ku_i;
        ks[i] = ks_i;
        Ks[i] = Ks_i;
        ky[i] = ky_i;
        Ky[i] = Ky_i;

        // Optimality error
        Qu_err = std::max<double>(Qu_err, Qu.lpNorm<Eigen::Infinity>());
        mu_err = std::max<double>(mu_err, r_d.lpNorm<Eigen::Infinity>());
       
        if (alg.infeasible) {
            constrains_err = std::max<double>(constrains_err, (constrains_curr+y_t_curr).lpNorm<Eigen::Infinity>());
        }
        bp_failed = 0;
    }
    bp_opterr = std::max<double>(Qu_err, std::max<double>(mu_err, constrains_err));
    // cout<<"Qu_err = "<<Qu_err<<" mu_err = "<<mu_err<<" constrains_err = "<<constrains_err<<endl;
    // cout <<"bp_opterr = " << bp_opterr << endl;
}


// define reset filter 
void IPDDP::reset_filter() {
    double log_cost, err;
    if (alg.infeasible) {
        double y_sum = 0;
        for (const auto& yc : y_t) {
            for (int i=0; i<yc.size(); i++) {
                double yc_value = yc(i);
                y_sum += std::log(yc_value);
            }
        }

        log_cost = total_cost - alg.mu*y_sum;
        //cout << "log_cost = " << log_cost << endl;
        
        VecOfVecXd constrains_plus_y(y_t.size(),VectorXd(y_t[0].size()));
        constrains_plus_y = constrains + y_t;
        err = 0;
        for (const auto& cy : constrains_plus_y) {
            for (int i=0; i<cy.size(); i++) {
                double cy_value = cy(i);
                err += std::abs(cy_value);
            }
        }
        //cout << "err = " << err << endl;
        if(err<alg.tolerance) {
            err = 0;
        }

    } else {
        double constrains_sum = 0;
        for (const auto& c : constrains) {
            for (int i=0; i<c.size(); i++) {
                double c_value = c(i);
                constrains_sum += std::log(-c_value);
            }
        }
        log_cost = total_cost - alg.mu*constrains_sum;
        err = 0;
    }
    tmp_filter(0) = log_cost; tmp_filter(1) = err;
    filter.push_back(tmp_filter);
    fp_step = 0;
    fp_failed = 0;
}

void IPDDP::reset_reg() {
    bp_regularization = 0;
    bp_failed = 0;
    bp_recovery = 0;
}

void IPDDP::output_to_csv(const std::string filename) {
  FILE *XU = fopen(filename.c_str(), "w");

//   for(unsigned int i=1; i<=xs[0].size(); i++) fprintf(XU, "x%d, ", i);
//   for(unsigned int j=0; j<us[0].size(); j++) fprintf(XU, "u%d, ", j);
//   fprintf(XU, "\n");
  //fprintf(XU, "u%d\n", int(us[0].size()));

  for(int t=0; t<T; t++) {
    for(unsigned int i=0; i<xs[t].size(); i++) fprintf(XU, "%f, ", xs[t](i));
    for(unsigned int j=0; j<us[t].size()-1; j++) fprintf(XU, "%f, ", us[t](j));
    fprintf(XU, "%f\n", us[t](us[t].size()-1));
  }

  for(unsigned int i=0; i<xs[T].size(); i++) fprintf(XU, "%f, ", xs[T](i));
  fprintf(XU, "%f ", 0.0);

  fclose(XU);
  //cout << "Saved IPDDP result to " << filename << endl;
}

void IPDDP::output_road_to_csv(const std::string filename) {
    FILE *road_xu = fopen(filename.c_str(), "w");

    // for(unsigned int i=0; i<road_reference_line[0].size(); i++) fprintf(road_xu, "x_ref%d, ", i);
    // for(unsigned int j=0; j<road_left_side[0].size(); j++) fprintf(road_xu, "x_left_side%d, ", j);
    // for(unsigned int k=0; k<road_right_side[0].size(); k++) fprintf(road_xu, "x_right_side%d, ", k);
    // fprintf(road_xu, "\n");

    for(int t=0; t<road_reference_line.size(); t++) {
        for(unsigned int i=0; i<road_reference_line[t].size(); i++) {
            fprintf(road_xu, "%f, ", road_reference_line[t][i]);
        }
        for(unsigned int j=0; j<road_left_side[t].size(); j++) {
            fprintf(road_xu, "%f, ", road_left_side[t][j]);
        }
        for(unsigned int k=0; k<road_right_side[t].size(); k++) {
            fprintf(road_xu, "%f, ", road_right_side[t][k]);
        }
        fprintf(road_xu, "\n");
    }

    fclose(road_xu);
    //cout << "Save road date to " << filename << endl;
}


void IPDDP::set_parameter_zeros(VecOfVecXd& u_new, 
    VecOfVecXd& y_t_new,  
    VecOfVecXd& s_new, 
    VecOfVecXd& constrains_new) {

    VectorXd tmp_vector_ysc(model->c_dims);
    VectorXd tmp_vector_u(model->u_dims);
    tmp_vector_ysc.setZero();
    tmp_vector_u.setZero();
    u_new.resize(T);
    y_t_new.resize(T);
    s_new.resize(T);
    constrains_new.resize(T);
    for(int i=0; i<T; i++) {
        u_new[i] =  tmp_vector_u;
        y_t_new[i] = tmp_vector_ysc;
        s_new[i] = tmp_vector_ysc;
        constrains_new[i] = tmp_vector_ysc;
    }
}

bool IPDDP::check_y_snew(VectorXd& y_new, 
    VectorXd& s_new, 
    VectorXd& y_old, 
    VectorXd& s_old, 
    double& tau) {
    bool failed_flag = false;
    for (int k=0; k<y_new.size(); k++) {
        if(y_new[k]<(1-tau)*y_old[k] || s_new[k]<(1-tau)*s_old[k]) {
            failed_flag = true;
            break;
        }
    }
    return failed_flag;
}

bool IPDDP::check_c_snew(VectorXd& constrains_new, 
    VectorXd& constrains_old, 
    VectorXd& s_new, 
    VectorXd& s_old, 
    double& tau) {
    bool failed_flag = false;
    for (int k=0; k<s_new.size(); k++) {
        if (s_new[k] < (1-tau)*s_old[k] || constrains_new[k] > (1-tau)*constrains_old[k]) {
            failed_flag = true;
            break;
        }
    }
    return failed_flag;
}

// update: us; constrains; xs; return: total_cost
double IPDDP::initialroll(const VectorXd& x0, const VecOfVecXd& u) {
    total_cost = 0;

    VectorXd x_curr = x0;
    VectorXd u_curr;

    VecOfVecXd x_tmp_new(T+1);
    x_tmp_new[0] = x0;
    
    for(int t=0; t<T; t++) {
        u_curr = u[t];
        us[t] = u_curr;
        
        constrains[t] = model->constrains_function(x_curr, 
                u_curr, 
                road_reference_line, 
                road_left_side,
                road_right_side);
        total_cost += model->cost(x_curr, u_curr, road_reference_line);
        x_curr = model->integrate_dynamics(x_curr, u_curr, dt);
        x_tmp_new[t+1] = x_curr;
    }
    
    xs = x_tmp_new;
    //cout << "total_cost = \n" << total_cost << endl;
    total_cost += model->terminal_cost(xs[T], terminal_state);
    
    //cout << "model->final_cost(xs[T]) = \n" << model->terminal_cost(xs[T], terminal_state) << endl;
    return total_cost;
}
