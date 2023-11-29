
// Tuned admittance controller
// P
#include "../include/qp_controller.h"

VectorXd qp_controller(const Matrix<double,7,1> &q_, const Matrix<double,7,1> &dq_, 
                        size_t &counter, const Matrix<double,7,1> &q_desired,
                        const Matrix<double,3,1> &x_desired, const MatrixXd &F_ext,
                        const MatrixXd &dx, const MatrixXd &dx_last,
                        const MatrixXd &tau_ext)
{
    // Gain Tunning //////////////////////////////////////////////////
    c_float K_qp = 0.5; // for cost function
    c_float K_dx = 1; // for cartesian tracking
    c_float K_sing = 1; // for min. eigenvalue
    c_float w_r = 1; // for cost function
    c_float w_t = 4; // for cost function
    Matrix<double, 6, 6> K_adm;
    // Matrix<double, 7, 7> K_adm;
    Matrix<double, 6, 1> K_adm_vec;
    // first three rotation, last three translation
    // K_adm_vec << 10, 10, 10, 2, 2, 2; 
    // K_adm_vec << 0.1, 0.1, 0.1, 0.05, 0.05, 0.05; 

    K_adm_vec << 1, 1, 1, 0.5, 0.5, 0.5; 
    K_adm = K_adm_vec.asDiagonal();
    Matrix<double, 6, 6> D_adm;
    Matrix<double, 6, 1> D_adm_vec;
    D_adm_vec << 20, 20, 20, 20, 20, 20;
    // set min. allowed eigenvalue (min. ellipsoid axis length)
    c_float ev_min_r = 0.1;
    c_float ev_min_t = 0.02;
    ///////////////////////////////////////////////////////////////////
    Matrix<double,7,1> q_goal;
    q_goal = q_desired;
    q_goal(6) = 0.7945;
    // using std::chrono::high_resolution_clock;
    // using std::chrono::duration;
    // using std::chrono::milliseconds;

    // auto t1 = high_resolution_clock::now();
    
    // /////////////////////////////////////////////////////////
    // Robot definition
    DQ_SerialManipulatorMDH robot = FrankaRobot::kinematics();
    // std::shared_ptr<DQ_SerialManipulatorMDH> robot_ptr = std::make_shared<DQ_SerialManipulatorMDH> (robot);

    // Set link number and joint angle
    int n = 7;
    int m = 6; // Dimension of workspace
    // VectorXd q_ (n);
    // VectorXd q_1 (n);
    // q_1 << 1.15192, 0.383972, 0.261799, -1.5708, 0.0, 1.39626, 0.0 ; // validate with q_test in Matlab
    // q_ << -1.98968, -0.383972, -2.87979, -1.5708, 4.20539e-17, 1.39626, 0;

    // joint velocity bounds
    Matrix<double, 7, 1> dq_min = robot.get_lower_q_dot_limit();
    Matrix<double, 7, 1> dq_max = robot.get_upper_q_dot_limit();
    // joint position boundsq_goal
    Matrix<double, 7, 1> q_min = robot.get_lower_q_limit();
    Matrix<double, 7, 1> q_max = robot.get_upper_q_limit();
    // joint acceleration bounds
    Matrix<double, 7, 1> ddq_max;
    ddq_max.setConstant(10); // original: 500
    Matrix<double, 7, 1> dq_min_q;
    Matrix<double, 7, 1> dq_max_q;
    Matrix<double, 7, 1> dq_min_ddq;
    Matrix<double, 7, 1> dq_max_ddq;

    Matrix<double, 3, 1> err_int_;
    
    // // Auxiliar variables
    double dt = 1E-3;	// Time step
    int nbIter = 1; // Number of iterations (orig: 65)
    // int nbData = 1; // no trajectory
    // int t_all = nbIter*nbData;

    // Desired cartesian and manipulability pos/trajectory
    MatrixXd J_goal;
    MatrixXd J_geom_goal;
    MatrixXd J_geom_goal_r;
    MatrixXd J_geom_goal_t;
    MatrixXd Me_d;
    MatrixXd Me_d_r;
    MatrixXd Me_d_t;
    MatrixXd Me_d_axis;
    MatrixXd Me_ct_axis;
    MatrixXd M_diff_axis;
    Matrix<double, 1, 7> J_geom_goal_axis;
    J_goal = robot.pose_jacobian(q_goal);
    J_geom_goal = geomJac(robot, J_goal, q_goal, n);
    J_geom_goal_r = J_geom_goal.block(0,0,3,7);
    J_geom_goal_t = J_geom_goal.block(3,0,3,7);
    J_geom_goal_axis = J_geom_goal.row(3); // translation in x as primary tracking object
    Me_d_axis = J_geom_goal_axis*J_geom_goal_axis.transpose();
    Me_d = J_geom_goal*J_geom_goal.transpose();
    Me_d_r = J_geom_goal_r*J_geom_goal_r.transpose();
    Me_d_t = J_geom_goal_t*J_geom_goal_t.transpose();

    // Initialization dq q_track M ev_diff
    Matrix<double,7,1> dq_res_1;
    Matrix<double,7,1> dq_res_1_fil;
    
    Matrix<double,7,1> dq_res;
    MatrixXd J;
    MatrixXd J_geom;
    MatrixXd J_geom_t;
    MatrixXd J_geom_r;
    MatrixXd J_geom_t_axis;
    MatrixXd Me_ct(m,m);
    MatrixXd Me_ct_r(3,3);
    MatrixXd Me_ct_t(3,3);
    Tensor<double, 3> Me_track(m,m,nbIter);
    Tensor<double, 3> J_grad(m,n,n);
    Tensor<double, 3> J_grad_t(3,n,n);
    Tensor<double, 3> J_grad_r(3,n,n);
    Tensor<double, 3> J_grad_axis(1,n,n);
    Tensor<double, 3> Jm_t_axis_;
    MatrixXd Jm_t;
    MatrixXd Jm_t_t;
    MatrixXd Jm_t_r;
    MatrixXd Jm_t_axis;
    MatrixXd M_diff(m,m);
    MatrixXd M_diff_r(3,3);
    MatrixXd M_diff_t(3,3);
    VectorXd vec_M_diff(21);
    VectorXd vec_M_diff_r(6);
    VectorXd vec_M_diff_t(6);
    Matrix<double, 6, 1> ev_t;
    MatrixXd ev_diff;
    BDCSVD<MatrixXd> singularsolver;
    SparseMatrix<c_float> H_s;
    SparseMatrix<c_float> A_s;
    SparseMatrix<c_float> H_2_s;
    SparseMatrix<c_float> A_2_s;
    Matrix<double, 7, 7> I;
    I.setIdentity();
    Matrix<double, 3, 1> I_3;
    I_3.setIdentity();
    Matrix<double, 7, 6> I_76;
    I_76.setIdentity();
    // equality constraint right part
    Vector3d dxr;
    Matrix<double, 6, 1> dx_guid;

    // std::cout << "Starting control loop-------------------------------------------------" << std::endl;
    // Main control loop //////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////
    for (int i = 0; i<nbIter; i++){
        VectorXd qt = q_;
        // forward kinematic
        DQ xt = robot.fkm(qt);
        //test coordinate with frankaros
        // std::cout<<"fkm: ------- "<<xt<<std::endl;
        // Matrix4d tfm = dq2tfm(xt);
        // cartesian position
        Vector3d xt_t = xt.translation().vec3();
        // std::cout<<"xt: "<<xt_t<<std::endl;

        // Obtain the current analytical Jacobian, geom J and M 
        J = robot.pose_jacobian(qt);
        J_geom = geomJac(robot, J, qt, n); 
        J_geom_r = J_geom.block(0, 0, 3, n); 
        J_geom_t = J_geom.block(3, 0, 3, n); 
        J_geom_t_axis = J_geom_t.row(0); // translation in x as primary tracking object
        // std::cout<<"----------J_geom_t: "<<std::endl<<J_geom_t<<std::endl;
        // std::cout<<"----------J_geom_t_axis: "<<std::endl<<J_geom_t_axis<<std::endl;
        // Current Mani and Record Me_ct into Me_track
        Me_ct = J_geom*J_geom.transpose();
        Me_ct_r = J_geom_r*J_geom_r.transpose();
        Me_ct_t = J_geom_t*J_geom_t.transpose();
        Me_ct_axis = J_geom_t_axis*J_geom_t_axis.transpose();
        array<DenseIndex, 3> offset = {0, 0, i};
        array<DenseIndex, 3> extent = {m, m, 1};
        Tensor<double, 3> Me_ct_tensor = TensorMap<Tensor<double, 3>>(Me_ct.data(), 6, 6, 1);
        Me_track.slice(offset, extent) = Me_ct_tensor;

        // calculate distance between Me_d and Me_ct
        // MatrixXd Md = Me_d.pow(-0.5)*Me_ct*Me_d.pow(-0.5);
        // MatrixXd Md_log = Md.log();
        // double d = Md_log.norm();
        // std::cout<<"Current distance between M: "<<d<<std::endl;
        // check whether or not need to change to the next point on trajectory
        // if(d<=0.1 && counter<q_goal_traj.cols()-1){
        //     counter++;
        //     q_goal = q_goal_traj.col(counter);
        //     J_goal = robot.pose_jacobian(q_goal);
        //     J_geom_goal = geomJac(robot, J_goal, q_goal, n);
        //     J_geom_goal_axis = J_geom_goal.row(3); // translation in x as primary tracking object
        //     Me_d_axis = J_geom_goal_axis*J_geom_goal_axis.transpose();
        //     Me_d = J_geom_goal*J_geom_goal.transpose();
        // }

        J_grad = jacobianEst(qt, n, robot);
        array<DenseIndex, 3> offset_r = {0, 0, 0}; // translation in x
        array<DenseIndex, 3> extent_r = {3, 7, 7};
        J_grad_r = J_grad.slice(offset_r, extent_r);
        array<DenseIndex, 3> offset_t = {3, 0, 0}; // translation in x
        array<DenseIndex, 3> extent_t = {3, 7, 7};
        J_grad_t = J_grad.slice(offset_t, extent_t);

        array<DenseIndex, 3> offset_axis = {3, 0, 0}; // translation in x
        array<DenseIndex, 3> extent_axis = {1, 7, 7};
        J_grad_axis = J_grad.slice(offset_axis, extent_axis);
        // Compute manipulability Jacobian (red to matrix)
        Jm_t = redManipulabilityJacobian(J_geom, J_grad);
        Jm_t_t = redManipulabilityJacobian_3(J_geom_t, J_grad_t);
        Jm_t_r = redManipulabilityJacobian_3(J_geom_r, J_grad_r);
        Jm_t_axis_ = manipulabilityJacobian(J_geom_t_axis, J_grad_axis);
        Jm_t_axis = Map<MatrixXd> (Jm_t_axis_.data(), 1, 7);
        // std::cout<<"Jm_t_axis_: "<<std::endl<<Jm_t_axis_ <<std::endl;

        // Compute distance to desired manipulybility 
        M_diff = logmap(Me_d, Me_ct); // 6x6
        M_diff_r = logmap(Me_d_r, Me_ct_r); // 3x3
        M_diff_t = logmap(Me_d_t, Me_ct_t); // 3x3
        M_diff_axis = logmap(Me_d_axis, Me_ct_axis); // 1

        vec_M_diff = spd2vec_vec(M_diff); // 21x1
        vec_M_diff_r = spd2vec_vec(M_diff_r); // 21x1
        vec_M_diff_t = spd2vec_vec(M_diff_t); // 21x1
        // std::cout<<"M_diff: "<<std::endl<<M_diff<<std::endl;
        // std::cout<<"M_diff_axis: "<<std::endl<<M_diff_axis<<std::endl;
        // std::cout<<"vec_M_diff: "<<std::endl<<vec_M_diff<<std::endl;

        // Calculate eigenvalue of the current M (singular value of J_geom)
        ev_t = singularsolver.compute(J_geom).singularValues();
        // std::cout<<"ev_t: -------------------"<<std::endl<<ev_t<<std::endl;
        ev_diff = jacobianEstVector(qt, n, robot);

        ////////////////////// QP Controller using osqp-eigen /////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////////
        // Cost Function: Hessian matrix H and Gradient f
        // constexpr double tolerance = 1e-4;
        // Intergral term + Damping term
        // Matrix<double, 6, 1> admittance_signal = - K_adm * F_ext ;
        // - D_adm * (dx - dx_last)
        Matrix<double, 7, 1> admittance_signal = - tau_ext;
        // Matrix<double, 6, 1> admittance_signal = - 0.1 * F_ext;
        std::vector<double> ev_vec(ev_t.data(), ev_t.data() + ev_t.rows() * ev_t.cols());
        double ev_min_value = *std::min_element(ev_vec.begin(), ev_vec.end());
        
        // slack variable to avoid huge inv(J)
        double lambda = 0.02 - ev_min_value;
        Matrix<double, 6, 7> J_map = (J_geom.transpose()+lambda*I_76).completeOrthogonalDecomposition().pseudoInverse();
        // Matrix<double, 7, 7> NS_projector = I - J_geom.completeOrthogonalDecomposition().pseudoInverse() * J_geom;
        // Matrix<double, 7, 6> J_inv = J_geom.completeOrthogonalDecomposition().pseudoInverse();
        // Matrix<c_float, 7, 7> H = 0*Jm_t.transpose()*Jm_t + J_geom.transpose()*J_geom;
        
        Matrix<c_float, 7, 7> H = I;
        // Matrix<c_float, 7, 7> H = J_geom.transpose() * J_geom;

        H_s = H.sparseView();
        // std::cout<<"H: "<<std::endl<<H<<std::endl;
        H_s.pruned(1e-9); // set those who smaller than 0.01 as zero
        // std::cout<<"H_S : "<<std::endl<<H_s<<std::endl;
        // Matrix<c_float, 1, 7> f = -0*K_qp* vec_M_diff.transpose()*Jm_t - 2*admittance_signal.transpose()*J_geom;
        Matrix<c_float, 1, 7> f = - 2*admittance_signal.transpose()*J_map.transpose()*K_adm*J_geom; // 
        // Matrix<c_float, 1, 7> f = - 2*admittance_signal.transpose()*J_map.transpose()*K_adm*J_geom - 2 * dx.transpose() * J_geom;
        // Matrix<c_float, 1, 7> f = - 2*admittance_signal.transpose() * K_adm*J_geom; // 

        // Initialize matrices for constraints
        Matrix<c_float, 31, 1> lb;
        Matrix<c_float, 31, 7> A;
        Matrix<c_float, 31, 1> ub;
        // Constraints:
        // 1. Inequality Constraint:---------------------------------------
        // set min. allowed eigenvalue (min. ellipsoid axis length)
        Matrix<c_float, 6, 1> ev_min;
        Matrix<c_float, 6, 1> v_max;
        ev_min << ev_min_r, ev_min_r, ev_min_r, ev_min_t, ev_min_t, ev_min_t;
        v_max = (ev_min - ev_t) * K_sing; // gain: 1
        lb.block(0,0,6,1) = v_max;
        A.block(0,0,6,7) = ev_diff;
        ub.block(0,0,6,1).setConstant(OsqpEigen::INFTY);
        // lb.block(0,0,6,1).setZero(); 
        // A.block(0,0,6,7).setZero();
        // ub.block(0,0,6,1).setZero();

        // 2. Equality Constraint for cartesian tracking-------------------
        // tune the gain! To give cartesian tracking some space
        // dxr = (x_desired - xt_t) * K_dx;
        // lb.block(6,0,3,1) = dxr; 
        // A.block(6,0,3,7) = J_geom_t;
        // ub.block(6,0,3,1) = dxr;
        lb.block(6,0,3,1).setZero(); 
        A.block(6,0,3,7).setZero();
        ub.block(6,0,3,1).setZero();

        // 3. Equality Constraint for single axis ME tracking--------------
        lb.block(9,0,1,1).setZero();
        A.block(9,0,1,7).setZero();
        ub.block(9,0,1,1).setZero();
        // lb.block(9,0,1,1) = M_diff_axis;
        // A.block(9,0,1,7) = Jm_t_axis;
        // ub.block(9,0,1,1) = M_diff_axis;

        // 4. Bounds------------------------------------------------------
        // a. Regarding joint velocity:
        lb.block(10,0,7,1) = dq_min;
        A.block(10,0,7,7) = I;
        ub.block(10,0,7,1) = dq_max;
        // lb.block(10,0,7,1).setZero();
        // A.block(10,0,7,7).setZero();
        // ub.block(10,0,7,1).setZero();
        // b. Regarding joint position: 
        // tune the gain!!!
        dq_min_q = (q_min-qt)*1;
        dq_max_q = (q_max-qt)*1;
        lb.block(17,0,7,1) = dq_min_q;
        A.block(17,0,7,7) = I;
        ub.block(17,0,7,1) = dq_max_q;
        // lb.block(17,0,7,1).setZero();
        // A.block(17,0,7,7).setZero();
        // ub.block(17,0,7,1).setZero();
        // c. Regarding joint acceleration:
        dq_min_ddq = dq_ - ddq_max * 0.1 ; //*dt
        dq_max_ddq = dq_ + ddq_max * 0.1 ;
        lb.block(24,0,7,1) = dq_min_ddq;
        A.block(24,0,7,7) = I;
        ub.block(24,0,7,1) = dq_max_ddq;
        // lb.block(24,0,7,1).setZero();
        // A.block(24,0,7,7).setZero();
        // ub.block(24,0,7,1).setZero();

        // Matrix<double, 7, 3> M_lb;
        // Matrix<double, 7, 3> M_ub;
        // M_lb.block(0,0,7,1) = dq_min_q;
        // M_lb.block(0,1,7,1) = dq_min;
        // M_lb.block(0,2,7,1) = dq_min_ddq;
        // M_ub.block(0,0,7,1) = dq_max_q;
        // M_ub.block(0,1,7,1) = dq_max;
        // M_ub.block(0,2,7,1) = dq_max_ddq;
        // VectorXd lb_limits;
        // lb_limits = M_lb.rowwise().maxCoeff();
        // VectorXd ub_limits;
        // ub_limits = M_ub.rowwise().minCoeff();
        // joint limits
        // lb.block(10,0,7,1) = lb_limits;
        // A.block(10,0,7,7) = I;
        // ub.block(10,0,7,1) = ub_limits;
        // lb.block(10,0,7,1).setZero();
        // A.block(10,0,7,7).setZero();
        // ub.block(10,0,7,1).setZero();
        // --------------------------------------------------------------------
        A_s = A.sparseView();
        // std::cout<<"ub: "<<ub.transpose()<<std::endl;
        OsqpEigen::Solver solver;
        solver.settings()->setVerbosity(false); // print output or not
        solver.settings()->setAlpha(1.5); // ADMM relaxation parameter/step size/penalty parameter
        solver.data()->setNumberOfVariables(7);
        //eigenvalue (6) + x_t_tracking(3) + aixs tracking(1) + limits(7)
        solver.data()->setNumberOfConstraints(31); 
        solver.data()->setHessianMatrix(H_s);
        //solver.data()->setHessianMatrix(h_h);
        solver.data()->setGradient(f.transpose());
        solver.data()->setLinearConstraintsMatrix(A_s);
        solver.data()->setLowerBound(lb);
        solver.data()->setUpperBound(ub);
        solver.initSolver();
        solver.solveProblem();
        
        // dq_res = solver.getSolution();
        dq_res_1 = solver.getSolution();
        // Filter the dq from first QP layer
        // double Ts = 0.001;
        // double fc = 1;
        // double a = 2 * 3.14 * fc * Ts / (2 * 3.14 * fc * Ts + 1);

        // if (counter == 0){
        //     dq_res_1_fil = dq_res_1;
        //     //F_ext_fil = tau_ext_hat_filtered;
        //     dq_res_1_last.setZero();
        // }
        // else{
        //     dq_res_1_fil = dq_res_1_last + a * (dq_res_1 - dq_res_1_last);
        // }
        // dq_res_1_last = dq_res_1_fil;
        ///////////////////////////////////////////////////////////////////////////////////////////        
        //// Second QP ////////////////////////////////////////////////////////////////////////////
        // Cost Function: Hessian matrix H and Gradient f
        // velocity from 
        // Matrix<c_float, 7, 7> H_2 = Jm_t.transpose()*Jm_t;
        Matrix<c_float, 7, 7> H_2 = w_r* Jm_t_r.transpose()*Jm_t_r + w_t* Jm_t_t.transpose()*Jm_t_t;
        H_2_s = H_2.sparseView();
        // std::cout<<"H: "<<std::endl<<H<<std::endl;
        H_2_s.pruned(1e-9); // set those who smaller than 0.01 as zero
        // std::cout<<"H_S : "<<std::endl<<H_s<<std::endl;
        // Matrix<c_float, 1, 7> f = -0*K_qp* vec_M_diff.transpose()*Jm_t - 2*admittance_signal.transpose()*J_geom;
        // Matrix<c_float, 1, 7> f_2 = -K_qp* vec_M_diff.transpose()*Jm_t; // 
        Matrix<c_float, 1, 7> f_2 = -K_qp* (w_r* vec_M_diff_r.transpose()*Jm_t_r + w_t* vec_M_diff_t.transpose()*Jm_t_t); // 

        // Initialize matrices for constraints
        Matrix<c_float, 33, 1> lb_2;
        Matrix<c_float, 33, 7> A_2;
        Matrix<c_float, 33, 1> ub_2;
        // Constraints:
        // 1. Inequality Constraint:---------------------------------------
        // set min. allowed eigenvalue (min. ellipsoid axis length)
        // lb_2.block(0,0,6,1) = v_max;
        // A_2.block(0,0,6,7) = ev_diff;
        // ub_2.block(0,0,6,1).setConstant(OsqpEigen::INFTY);
        lb_2.block(0,0,6,1).setZero(); 
        A_2.block(0,0,6,7).setZero();
        ub_2.block(0,0,6,1).setZero();

        // 2. Equality Constraint for cartesian tracking 
        // (cartesian pose from guidance)-------------------
        // tune the gain! To give cartesian tracking some space
        dx_guid = J_geom * dq_res_1;
        Matrix<double,6,1> I_x;
        I_x.setIdentity();
        I_x(0,0) = 0.5;
        I_x(1,0) = 0.5;
        I_x(2,0) = 0.5;
        I_x(3,0) = 0.1;
        I_x(4,0) = 0.1;
        I_x(5,0) = 0.1;
        lb_2.block(6,0,6,1) = dx_guid - I_x; 
        A_2.block(6,0,6,7) = J_geom;
        ub_2.block(6,0,6,1) = dx_guid + I_x;
        // lb_2.block(6,0,6,1).setZero(); 
        // A_2.block(6,0,6,7).setZero();
        // ub_2.block(6,0,6,1).setZero();
        // lb_2.block(12,0,7,1) = lb_limits;
        // A_2.block(12,0,7,7) = I;
        // ub_2.block(12,0,7,1) = ub_limits;
        // lb_2.block(12,0,7,1).setZero();
        // A_2.block(12,0,7,7).setZero();
        // ub_2.block(12,0,7,1).setZero();
        // lb_2.block(12,0,7,1) = dq_min;
        // A_2.block(12,0,7,7) = I;
        // ub_2.block(12,0,7,1) = dq_max;
        lb_2.block(12,0,7,1).setZero();
        A_2.block(12,0,7,7).setZero();
        ub_2.block(12,0,7,1).setZero();
        // lb_2.block(19,0,7,1) = dq_min_q;
        // A_2.block(19,0,7,7) = I;
        // ub_2.block(19,0,7,1) = dq_max_q;
        lb_2.block(19,0,7,1).setZero();
        A_2.block(19,0,7,7).setZero();
        ub_2.block(19,0,7,1).setZero();
        // lb_2.block(26,0,7,1) = dq_min_ddq;
        // A_2.block(26,0,7,7) = I;
        // ub_2.block(26,0,7,1) = dq_max_ddq;
        lb_2.block(26,0,7,1).setZero();
        A_2.block(26,0,7,7).setZero();
        ub_2.block(26,0,7,1).setZero();
        // Cartesian constraint
        // dxr = (x_desired - xt_t) * K_dx;
        // lb_2.block(33,0,3,1) = dxr - 0.1*I_3; 
        // A_2.block(33,0,3,7) = J_geom_t;
        // ub_2.block(33,0,3,1) = dxr + 0.1*I_3;
        // --------------------------------------------------------------------
        
        A_2_s = A_2.sparseView();
        // std::cout<<"ub: "<<ub.transpose()<<std::endl;
        OsqpEigen::Solver solver_2;
        solver_2.settings()->setVerbosity(false); // print output or not
        solver_2.settings()->setAlpha(1.5); // ADMM relaxation parameter/step size/penalty parameter
        solver_2.data()->setNumberOfVariables(7);
        solver_2.data()->setNumberOfConstraints(33); 
        solver_2.data()->setHessianMatrix(H_2_s);
        //solver.data()->setHessianMatrix(h_h);
        solver_2.data()->setGradient(f_2.transpose());
        solver_2.data()->setLinearConstraintsMatrix(A_2_s);
        solver_2.data()->setLowerBound(lb_2);
        solver_2.data()->setUpperBound(ub_2);
        solver_2.initSolver();
        solver_2.solveProblem();

        dq_res = solver_2.getSolution();
        ////////////////////////////////////////////////////////////////////////////////////////////
        // std::cout<<"Solution dq_t: "<<std::endl<< dq_res.transpose() <<std::endl;
        
    }
    // std::cout << "Control finished..." << std::endl;
    // auto t2 = high_resolution_clock::now();

    // duration<double, std::milli> ms_double = t2 - t1;
    // std::cout << ms_double.count() << "ms\n";
  
    return dq_res;
}
