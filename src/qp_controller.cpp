#include "../include/qp_controller.h"

VectorXd qp_controller(const Matrix<double,7,1> &q_, const Matrix<double,7,1> &dq_, 
                        Index &counter, const Matrix<double,7,1> &q_desired,
                        const Matrix<double,3,1> &x_desired, const MatrixXd &F_ext,
                        const MatrixXd &F_ext_fil, const MatrixXd &F_ext_fil_last,
                        const MatrixXd &dx, const MatrixXd &dx_last,
                        const MatrixXd &tau_ext)
{
    // Gain Tunning //////////////////////////////////////////////////
    c_float K_qp = 3; // for cost function
    c_float K_dx = 1; // for cartesian tracking
    c_float K_sing = 1; // for min. eigenvalue
    Matrix<double, 6, 6> K_adm;
    // Matrix<double, 7, 7> K_adm;
    Matrix<double, 6, 1> K_adm_vec;
    // first three rotation, last three translation
    K_adm_vec << 0.00, 0.00, 0.00, 0.00, 0.0, 0.00; 
    // K_adm_vec << 0.5, 0.5, 0.5, 0.5, 0.5, 0.1, 0.5; 
    K_adm = K_adm_vec.asDiagonal();
    Matrix<double, 6, 6> D_adm;
    Matrix<double, 6, 1> D_adm_vec;
    D_adm_vec << 20, 20, 20, 20, 20, 20;
    // set min. allowed eigenvalue (min. ellipsoid axis length)
    c_float ev_min_r = 0.02;
    c_float ev_min_t = 0.02;
    ///////////////////////////////////////////////////////////////////
    Matrix<double,7,1> q_goal;
    q_goal = q_desired;
    
    // using std::chrono::high_resolution_clock;
    // using std::chrono::duration;
    // using std::chrono::milliseconds;

    // auto t1 = high_resolution_clock::now();
    
    // /////////////////////////////////////////////////////////
    // Robot definition
    DQ_SerialManipulatorMDH robot = FrankaRobot::kinematics();
    // std::shared_ptr<DQ_SerialManipulatorMDH> robot_ptr = std::make_shared<DQ_SerialManipulatorMDH> (robot);
    // DQ_SerialManipulatorMDH robot_ik = FrankaRobot::kinematics();
    // DQ offset_base = 1 + E_ * 0.5 * DQ(0, 0, 0, 0);
    // robot_ik.set_base_frame(offset_base);
    // robot_ik.set_reference_frame(offset_base);

    // DQ base_frame = vi.get_object_pose("Franka_joint1");
    // DQ eef_frame = vi.get_object_pose("Franka_connection");
    // std::cout<<"Originally base frame of robot:.... "<<robot.get_base_frame()<<std::endl;
    // std::cout<<"base frame in Vrep: "<<base_frame<<std::endl;
    // std::cout<<"eef frame in Vrep: "<<eef_frame<<std::endl;
    // robot.set_base_frame(base_frame);
    // robot.set_reference_frame(base_frame);
    // std::cout<<"Now base frame in Vrep:.... "<<robot.get_base_frame()<<std::endl;
    // std::cout<<"Now eef frame in Vrep:.... "<<robot.get_effector()<<std::endl;

    // Set link number and joint angle
    int n = 7;
    int m = 6; // Dimension of workspace
    // VectorXd q_ (n);
    // VectorXd q_1 (n);
    // q_1 << 1.15192, 0.383972, 0.261799, -1.5708, 0.0, 1.39626, 0.0 ; // validate with q_test in Matlab
    // q_ << -1.98968, -0.383972, -2.87979, -1.5708, 4.20539e-17, 1.39626, 0;
    // Matrix<double,7,1> q_add;
    Matrix<double,7,3> q_goal_traj;
    // q_goal<<-0.000545241, -0.787773, -0.00212514, -2.3583, 0.5, 1.57543, 0.795024;
    // q_goal << -0.5, -0.787773, -0.5, -2.3583, 0.5, 1.57543, 0.0;
    // q_goal_traj.col(0)<<-0.5, -0.787773, -0.5, -2.3583, 0.5, 1.57543, 0.0;
    // q_goal_traj.col(1)<<0.3, -0.787773, -0.5, -2.3583, 0.5, 1.57543, 0.0;
    // q_goal_traj.col(2)<<0.3, -0.787773, -0.5, -2.3583, 0.9, 1.57543, 0.0;

    // q_goal << -pi/2.0, 0.004, 0.0, -1.57156, 0.0, 1.57075, 0.0;
    // q_goal << 0.519784, 0.991963, 1.50832, -1.54527, -1.2189, 0.878087, 0.0;

    // joint velocity bounds
    Matrix<double, 7, 1> dq_min = robot.get_lower_q_dot_limit();
    Matrix<double, 7, 1> dq_max = robot.get_upper_q_dot_limit();
    // joint position boundsq_goal
    Matrix<double, 7, 1> q_min = robot.get_lower_q_limit();
    Matrix<double, 7, 1> q_max = robot.get_upper_q_limit();
    // joint acceleration bounds
    Matrix<double, 7, 1> ddq_max;
    ddq_max.setConstant(5); // original: 500
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
    MatrixXd Me_d;
    MatrixXd Me_d_axis;
    MatrixXd Me_ct_axis;
    MatrixXd M_diff_axis;
    Matrix<double, 1, 7> J_geom_goal_axis;
    J_goal = robot.pose_jacobian(q_goal);
    J_geom_goal = geomJac(robot, J_goal, q_goal, n);
    J_geom_goal_axis = J_geom_goal.row(3); // translation in x as primary tracking object
    Me_d_axis = J_geom_goal_axis*J_geom_goal_axis.transpose();
    Me_d = J_geom_goal*J_geom_goal.transpose();

    // test ik solver (without offset to synchronize with Vrep)
    // DQ x = robot_ik.fkm(q).normalize();
    // Matrix4d M_tf = dq2tfm(x);
    // std::array<double, 16> arr_tf;
    // std::copy(M_tf.data(), M_tf.data() + 16, arr_tf.begin());
    // double q7 = 0.0;
    // std::array<double, 7> qt_arr;
    // std::copy(q.data(), q.data() + q.size(), qt_arr.begin());
    // std::cout << "current q" << std::endl;
    // for (double val : qt_arr) {
    //     std::cout << val << " ";
    // }
    // std::cout << std::endl;
    // std::array<std::array<double, 7>,4> q_est = franka_IK_EE (arr_tf,
    //                                                 q7, qt_arr );
    // std::array<double, 7> q_est = franka_IK_EE_CC (arr_tf,
    //                                                 q7, qt_arr );
    // std::cout << "IK est. q: " << std::endl;
    // for(int i=0; i<4; i++){
    //     for (double val : q_est[i]) {
    //         std::cout << val << " ";  
    //     }
    //     std::cout << std::endl; 
    // }                                            

    // Initialization dq q_track M ev_diff
    // Vector3d inf_min;
    // inf_min.setConstant(1e-10);
    // MatrixXd qt_track(7,nbIter);
    // MatrixXd dq_track(7,nbIter);
    // MatrixXd x_t_track(3,nbIter);
    Matrix<double,7,1> dq_res;
    MatrixXd J;
    MatrixXd J_geom;
    MatrixXd J_geom_t;
    MatrixXd J_geom_t_axis;
    MatrixXd Me_ct(m,m);
    Tensor<double, 3> Me_track(m,m,nbIter);
    Tensor<double, 3> J_grad(m,n,n);
    Tensor<double, 3> J_grad_axis(1,n,n);
    Tensor<double, 3> Jm_t_axis_;
    MatrixXd Jm_t;
    MatrixXd Jm_t_axis;
    MatrixXd M_diff(m,m);
    VectorXd vec_M_diff(21);
    Matrix<double, 6, 1> ev_t;
    MatrixXd ev_diff;
    BDCSVD<MatrixXd> singularsolver;
    SparseMatrix<c_float> H_s;
    SparseMatrix<c_float> A_s;
    Matrix<double, 7, 7> I;
    I.setIdentity();
    Matrix<double, 7, 6> I_76;
    I_76.setIdentity();
    // eqaulity constraint right part
    Vector3d dxr;

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

        // x_t_track.col(i) = xt_t;
        // Vector3d x_goal = x_t_track.col(0);

        // Obtain the current analytical Jacobian, geom J and M 
        J = robot.pose_jacobian(qt);
        J_geom = geomJac(robot, J, qt, n); 
        J_geom_t = J_geom.block(3, 0, 3, n); 
        J_geom_t_axis = J_geom_t.row(0); // translation in x as primary tracking object
        // std::cout<<"----------J_geom_t: "<<std::endl<<J_geom_t<<std::endl;
        // std::cout<<"----------J_geom_t_axis: "<<std::endl<<J_geom_t_axis<<std::endl;
        // Current Mani and Record Me_ct into Me_track
        Me_ct = J_geom*J_geom.transpose();
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
        array<DenseIndex, 3> offset_axis = {3, 0, 0}; // translation in x
        array<DenseIndex, 3> extent_axis = {1, 7, 7};
        J_grad_axis = J_grad.slice(offset_axis, extent_axis);
        // Compute manipulability Jacobian (red to matrix)
        Jm_t = redManipulabilityJacobian(J_geom, J_grad);
        Jm_t_axis_ = manipulabilityJacobian(J_geom_t_axis, J_grad_axis);
        Jm_t_axis = Map<MatrixXd> (Jm_t_axis_.data(), 1, 7);
        // std::cout<<"Jm_t_axis_: "<<std::endl<<Jm_t_axis_ <<std::endl;

        // Compute distance to desired manipulybility 
        M_diff = logmap(Me_d, Me_ct); // 6x6
        M_diff_axis = logmap(Me_d_axis, Me_ct_axis); // 1

        vec_M_diff = spd2vec_vec(M_diff); // 21x1
        // std::cout<<"M_diff: "<<std::endl<<M_diff<<std::endl;
        // std::cout<<"M_diff_axis: "<<std::endl<<M_diff_axis<<std::endl;
        // std::cout<<"vec_M_diff: "<<std::endl<<vec_M_diff<<std::endl;

        // Calculate eigenvalue of the current M (singular value of J_geom)
        ev_t = singularsolver.compute(J_geom).singularValues();
        // std::cout<<"ev_t: -------------------"<<std::endl<<ev_t<<std::endl;
        ev_diff = jacobianEstVector(qt, n, robot);

        // ++++++++++++++++++++QP Controller using osqp-eigen+++++++++++++++++++++++++++++++++
        // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        // Cost Function: Hessian matrix H and Gradient f
        // constexpr double tolerance = 1e-4;
        // Intergral term + Damping term
        // Matrix<double, 6, 1> admittance_signal = - K_adm * F_ext ;
        // - D_adm * (dx - dx_last)
        Matrix<double, 7, 1> admittance_signal = - tau_ext;
        std::vector<double> ev_vec(ev_t.data(), ev_t.data() + ev_t.rows() * ev_t.cols());
        double ev_min_value = *std::min_element(ev_vec.begin(), ev_vec.end());
        
        // slack variable to avoid huge inv(J)
        double lambda;
        if (ev_min_value < 0.05){
            lambda = 0.001;
        }else{
            lambda = 0;
        }
        Matrix<double, 6, 7> J_map = (J_geom.transpose()+lambda*I_76).completeOrthogonalDecomposition().pseudoInverse();
        Matrix<double, 7, 6> J_inv = J_geom.completeOrthogonalDecomposition().pseudoInverse();
        // Matrix<c_float, 7, 7> H = 0*Jm_t.transpose()*Jm_t + J_geom.transpose()*J_geom;
        Matrix<c_float, 7, 7> H = 0*Jm_t.transpose()*Jm_t + I;
        std::cout<<"Calculated F: "<< (J_map * tau_ext).transpose()<<std::endl;
        H_s = H.sparseView();
        // std::cout<<"H: "<<std::endl<<H<<std::endl;
        H_s.pruned(1e-9); // set those who smaller than 0.01 as zero
        // std::cout<<"H_S : "<<std::endl<<H_s<<std::endl;
        // Matrix<c_float, 1, 7> f = -0*K_qp* vec_M_diff.transpose()*Jm_t - 2*admittance_signal.transpose()*J_geom;
        Matrix<c_float, 1, 7> f = -0*K_qp* vec_M_diff.transpose()*Jm_t - 2*admittance_signal.transpose()*J_map.transpose()*K_adm*J_geom; // 

        // Initialize matrices for constraints
        Matrix<c_float, 20, 1> lb;
        Matrix<c_float, 20, 7> A;
        Matrix<c_float, 20, 1> ub;
        // Constraints:
        // 1. Inequality Constraint:---------------------------------------
        // set min. allowed eigenvalue (min. ellipsoid axis length)
        Matrix<c_float, 6,1> ev_min;
        Matrix<c_float, 6,1> v_max;
        ev_min << ev_min_r, ev_min_r, ev_min_r, ev_min_t, ev_min_t, ev_min_t;
        v_max = (ev_min - ev_t) * K_sing; // gain: 3
        // lb.block(0,0,6,1) = v_max;
        // A.block(0,0,6,7) = ev_diff;
        // ub.block(0,0,6,1).setConstant(OsqpEigen::INFTY);
        lb.block(0,0,6,1).setZero(); 
        A.block(0,0,6,7).setZero();
        ub.block(0,0,6,1).setZero();

        // 2. Equality Constraint for cartesian tracking-------------------
        // MatrixXd admittance_signal = - K_adm * F_ext - D_adm * (F_ext_fil - F_ext_fil_last);
        
        // lb.block(6,0,6,1) = admittance_signal; 
        // A.block(6,0,6,7) = J_geom;
        // ub.block(6,0,6,1) = admittance_signal;
        lb.block(6,0,6,1).setZero(); 
        A.block(6,0,6,7).setZero();
        ub.block(6,0,6,1).setZero();

        // tune the gain! To give cartesian tracking some space
        // dxr = (x_desired - xt_t) * K_dx;
        // lb.block(6,0,3,1) = dxr; 
        // A.block(6,0,3,7) = J_geom_t;
        // ub.block(6,0,3,1) = dxr;

        // 3. Equality Constraint for single axis ME tracking--------------
        lb.block(12,0,1,1).setZero();
        A.block(12,0,1,7).setZero();
        ub.block(12,0,1,1).setZero();
        // lb.block(9,0,1,1) = M_diff_axis;
        // A.block(9,0,1,7) = Jm_t_axis;
        // ub.block(9,0,1,1) = M_diff_axis;

        // 4. Bounds------------------------------------------------------
        // a. Regarding joint velocity:
        // lb.block(10,0,7,1) = dq_min;
        // A.block(10,0,7,7) = I;
        // ub.block(10,0,7,1) = dq_max;
        // b. Regarding joint position: 
        // tune the gain!!!
        dq_min_q = (q_min-qt);
        dq_max_q = (q_max-qt);
        // lb.block(17,0,7,1) = dq_min_q;
        // A.block(17,0,7,7) = I;
        // ub.block(17,0,7,1) = dq_max_q;
        // c. Regarding joint acceleration:
        if (i == 0){
            dq_min_ddq = -ddq_max*2;
            dq_max_ddq = ddq_max*2;
        }
        else {
            // tune the gain!!!
            dq_min_ddq = dq_ - ddq_max * dt  ; //*dt
            dq_max_ddq = dq_ + ddq_max * dt  ;
        }
        // lb.block(24,0,7,1) = dq_min_ddq;
        // A.block(24,0,7,7) = I;
        // ub.block(24,0,7,1) = dq_max_ddq;
        // lb.block(24,0,7,1).setZero();
        // A.block(24,0,7,7).setZero();
        // ub.block(24,0,7,1).setZero();

        Matrix<double, 7, 3> M_lb;
        Matrix<double, 7, 3> M_ub;
        M_lb.block(0,0,7,1) = dq_min_q;
        M_lb.block(0,1,7,1) = dq_min;
        M_lb.block(0,2,7,1) = dq_min_ddq;
        M_ub.block(0,0,7,1) = dq_max_q;
        M_ub.block(0,1,7,1) = dq_max;
        M_ub.block(0,2,7,1) = dq_max_ddq;
        VectorXd lb_limits;
        lb_limits = M_lb.rowwise().maxCoeff();
        VectorXd ub_limits;
        ub_limits = M_ub.rowwise().minCoeff();
        // lb.block(13,0,7,1) = lb_limits;
        // A.block(13,0,7,7) = I;
        // ub.block(13,0,7,1) = ub_limits;
        lb.block(13,0,7,1).setZero();
        A.block(13,0,7,7).setZero();
        ub.block(13,0,7,1).setZero();
        // --------------------------------------------------------------------
        
        A_s = A.sparseView();
        // std::cout<<"ub: "<<ub.transpose()<<std::endl;
        OsqpEigen::Solver solver;
        solver.settings()->setVerbosity(false); // print output or not
        solver.settings()->setAlpha(1.5); // ADMM relaxation parameter/step size/penalty parameter
        solver.data()->setNumberOfVariables(7);
        //eigenvalue (6) + x_t_tracking(3) + aixs tracking(1) + limits(7)
        solver.data()->setNumberOfConstraints(20); 
        solver.data()->setHessianMatrix(H_s);
        //solver.data()->setHessianMatrix(h_h);
        solver.data()->setGradient(f.transpose());
        solver.data()->setLinearConstraintsMatrix(A_s);
        solver.data()->setLowerBound(lb);
        solver.data()->setUpperBound(ub);
        solver.initSolver();
        solver.solveProblem();
        // bool flag = solver.solveProblem() == OsqpEigen::ErrorExitFlag::NoError;
        // std::cout<<"No error: "<<flag<<std::endl;
        // Eigen::Matrix<c_float, 7, 1> expectedSolution;
        // expectedSolution << 0.3,  0.7;

        dq_res = solver.getSolution();
        // std::cout<<"Solution dq_t: "<<std::endl<<dq_track.col(i).transpose()<<std::endl;
        
    }
    // std::cout << "Control finished..." << std::endl;
    // auto t2 = high_resolution_clock::now();

    /* Getting number of milliseconds as a double. */
    // duration<double, std::milli> ms_double = t2 - t1;
    // std::cout << ms_double.count() << "ms\n";
  
    return dq_res;
}
