// This file is the implementation of Quadratic Programming
// Input: 
    // q_: current joint positions
    // dq_: current joint velocities
    // counter: current iteration
    // q_desired: reference joint positions
    // x_desired: reference Cartesian translation mean value + Cartesian translation sigma value
    // xt_mean: reference Cartesian pose
    // dx: current robot end-effector twist after Low-Pass-Filter
    // dx_last: Last timestep Cartesian twist
// Output
    // dq_res: the robot joint velocity control command

#include "../include/qp_controller.h"

VectorXd qp_controller(const Matrix<double,7,1> &q_, const Matrix<double,7,1> &dq_, 
                        Index &counter, const Matrix<double,7,1> &q_desired,
                        const Matrix<double,6,1> &x_desired,
                        const Matrix<double, 8, 1> &xt_mean,
                        const Matrix<double,6,1> &dx,
                        const Matrix<double,6,1> &dx_last,
                        const std::unique_ptr<franka_hw::FrankaModelHandle> &model_handle)
{
    // Matrix<double, 7, 7> mass_test = MassMatrix(q_);

    // c_float K_qp = 0.001; //for dyn Manip
    c_float K_qp = 0.5; //for velocity Manip
    // c_float K_cart = 2;
    c_float W = 1;
    Matrix<c_float, 8, 8> W_cart;
    Matrix<c_float, 8, 8> K_cart;
    c_float D_cart = 50;

    Matrix<c_float, 8, 1> W_cart_vec;
    Matrix<c_float, 8, 1> K_cart_vec;
    // W_cart_vec << 2, 2, 2, 2, 2, 2, 2, 2;
    W_cart_vec << 1, 1, 1, 1, 1, 1, 1, 1;
    // first 4: Rotation
    // last 4: Translation
    K_cart_vec << 1, 1, 1, 1, 2, 2, 2, 2;
    W_cart = W * W_cart_vec.asDiagonal();
    K_cart = K_cart_vec.asDiagonal();

    // K_sing = 1/T, T means the duration system takes to avoid singularity
    c_float K_sing = 1;
    c_float ev_min_r = 0.1;
    c_float ev_min_t = 0.02;
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
    // Set link number and joint angle
    int n = 7;
    int m = 6; // Dimension of workspace

    // get Mass Matrix from frankaros
    // get joint position in std::array form
    std::array<double, 7> q_array;
    Matrix<double, 7, 1>::Map(q_array.data()) = q_;
    // std::array<double, 49> mass_array = model_handle->getMass();
    Matrix<double, 3, 3> total_inertia_eigen;
    total_inertia_eigen << 0.001, 0      , 0     ,
                           0,     0.0025 , 0     ,
                           0    , 0      , 0.0017;
    std::array<double, 9> total_inertia;
    Matrix<double, 3, 3>::Map(total_inertia.data()) = total_inertia_eigen;

    total_inertia.fill(0);
    // double total_mass = 0.73;
    double total_mass = 0;
    std::array<double, 3> F_x_Ctotal;
    F_x_Ctotal.fill(0);
    F_x_Ctotal[2] = 0.1034;
    // Map<Matrix<double, 7, 7>> Mass(mass_array.data());
    
    // std::cout<<"Test Mass: \n"<<Mass<<std::endl<<Mass_<<std::endl;
    // std::cout<<"Mass: "<<std::endl<<Mass;

    // VectorXd q_ (n);
    // VectorXd q_1 (n);
    // q_1 << 1.15192, 0.383972, 0.261799, -1.5708, 0.0, 1.39626, 0.0 ; // validate with q_test in Matlab
    // q_ << -1.98968, -0.383972, -2.87979, -1.5708, 4.20539e-17, 1.39626, 0;
    // Matrix<double,7,1> q_add;

    // q_ << 0, 0, 0, -1.5708, 0.0, 1.3963, 0.0 ;
    // MatrixXd I = vi.get_inertia_matrix(vi.get_object_handle("Franka_joint7"));
    // std::cout<<"Inertia matrix:------------------ "<<std::endl<<I<<std::endl;

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

    // Desired cartesian and manipulability pos/trajectory
    MatrixXd J_goal;
    MatrixXd J_geom_goal;
    MatrixXd Me_d;
    MatrixXd Me_dyn_d;
    MatrixXd Me_d_axis;
    MatrixXd Me_ct_axis;
    MatrixXd M_diff_axis;
    Matrix<double, 1, 7> J_geom_goal_axis;
    J_goal = robot.pose_jacobian(q_goal);
    // J_goal = model_handle->getZeroJacobian();
    J_geom_goal = geomJac(robot, J_goal, q_goal, n);
    J_geom_goal_axis = J_geom_goal.row(3); // translation in x as primary tracking object
    Me_d_axis = J_geom_goal_axis*J_geom_goal_axis.transpose();
    Me_d = J_geom_goal*J_geom_goal.transpose();
    // desired Mass matrix
    // get joint position in std::array form
    // std::array<double, 7> q_goal_array;
    // Matrix<double, 7, 1>::Map(q_goal_array.data()) = q_goal;
    // std::array<double, 49> mass_array_d = model_handle -> getMass(q_goal_array, total_inertia, total_mass, F_x_Ctotal);
    // Map<Matrix<double, 7, 7>> Mass_d(mass_array_d.data());
    // 2. From github repo
    Matrix<double, 7, 7> Mass_d = MassMatrix(q_goal);
    Me_dyn_d = (J_geom_goal*Mass_d.inverse()) * (J_geom_goal*Mass_d.inverse()).transpose();

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
    MatrixXd L_ct;

    MatrixXd J_geom_t;
    MatrixXd J_geom_r;

    MatrixXd J_geom_t_axis;
    MatrixXd Me_ct(m,m);
    MatrixXd Me_dyn_ct(m,m);

    Tensor<double, 3> Me_track(m,m,nbIter);
    Tensor<double, 3> J_grad(m,n,n);
    Tensor<double, 3> J_dyn_grad(m,n,n);
    Tensor<double, 3> J_grad_axis(1,n,n);
    Tensor<double, 3> Jm_t_axis_;
    MatrixXd Jm_t;
    MatrixXd Jm_dyn_t;
    MatrixXd Jm_t_axis;
    MatrixXd M_diff(m,m);
    MatrixXd M_dyn_diff(m,m);

    VectorXd vec_M_diff(21);
    VectorXd vec_M_dyn_diff(21);

    Matrix<double, 6, 1> ev_t;
    MatrixXd ev_diff;
    BDCSVD<MatrixXd> singularsolver;
    SparseMatrix<c_float> H_s;
    SparseMatrix<c_float> A_s;
    Matrix<double, 7, 7> I;
    I.setIdentity();

    // std::cout << "Starting control loop-------------------------------------------------" << std::endl;
    // Main control loop //////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////
    for (int i = 0; i<nbIter; i++){
        VectorXd qt = q_;
        // qt_track.col(i) = qt;
        // forward kinematic model
        DQ xt = robot.fkm(qt);
        DQ xd = robot.fkm(q_desired);
        Matrix<double, 8, 1> xt_8 = xt.vec8();
        Matrix<double, 8, 1> xd_8 = xd.vec8();

        //test coordinate with frankaros
        // std::cout<<"xt_8: ------- "<<xt_8.transpose()<<std::endl;
        // Matrix4d tfm = dq2tfm(xt);
        // Vector3d xt_t = xt.translation().vec3();
        // Vector4d xt_r = xt.rotation().vec4();
        // Quaterniond rotationQuaternion(xt_r(0), xt_r(1), xt_r(2), xt_r(3));
        // Convert the rotation quaternion into a 3x3 rotation matrix
        // Matrix3d rotationMatrix = rotationQuaternion.toRotationMatrix();
        // Vector3d Euler = rotationMatrix.eulerAngles(0,1,2);    
        // double roll, pitch, yaw;
        // // roll = atan2(rotationMatrix(2, 1), rotationMatrix(2, 2));
        // // pitch = atan2(-rotationMatrix(2, 0), sqrt(rotationMatrix(2, 1) * rotationMatrix(2, 1) + rotationMatrix(2, 2) * rotationMatrix(2, 2)));
        // // yaw = atan2(rotationMatrix(1, 0), rotationMatrix(0, 0));
        // roll = Euler(0);
        // pitch = Euler(1);
        // yaw = Euler(2);
        // Matrix<double, 6, 1> xt_6;
        // xt_6(0,0)= roll;
        // xt_6(1,0)= pitch;
        // xt_6(2,0)= yaw;
        // xt_6.block(3,0,3,1) = xt_t;

        // Obtain the current analytical Jacobian, geom J and M 
        J = robot.pose_jacobian(qt);
        J_geom = geomJac(robot, J, qt, n); 
        // J_geom_t = J_geom.block(3, 0, 3, n); 
        // J_geom_r = J_geom.block(0, 0, 3, n); 
        // J_geom_t_axis = J_geom_t.row(0); // translation in x as primary tracking object
        // std::cout<<"----------J_geom_t: "<<std::endl<<J_geom_t<<std::endl;
        // std::cout<<"----------J_geom_t_axis: "<<std::endl<<J_geom_t_axis<<std::endl;

        // current Mass matrix
        // std::array<double, 49> mass_array_ = model_handle->getMass(q_array, total_inertia, total_mass, F_x_Ctotal);
        // Map<Matrix<double, 7, 7>> Mass(mass_array_.data());
        // From github repo
        Matrix<double, 7, 7> Mass = MassMatrix(q_);
        // Current velocity Manip. 
        Me_ct = J_geom*J_geom.transpose();
        // Current dyn. Manip.
        L_ct = J_geom*Mass.inverse();
        Me_dyn_ct = L_ct * L_ct.transpose();

        // Me_ct_axis = J_geom_t_axis*J_geom_t_axis.transpose();
        array<DenseIndex, 3> offset = {0, 0, i};
        array<DenseIndex, 3> extent = {m, m, 1};
        Tensor<double, 3> Me_ct_tensor = TensorMap<Tensor<double, 3>>(Me_ct.data(), 6, 6, 1);
        Me_track.slice(offset, extent) = Me_ct_tensor;

        Eigen::Matrix<double, 3, 1> dxr_t;
        // Vel. Manip. ////////////////////////////////////////////////////////////////////////////
        // derivative of J wrt q (for velocity manip.)
        J_grad = jacobianEst(q_, n, robot);
        // Compute velocity manipulability Jacobian (red to matrix)
        Jm_t = redManipulabilityJacobian(J_geom, J_grad);
        // Compute distance to desired manipulybility 
        M_diff = logmap(Me_d, Me_ct); // 6x6 // velocity Manip.
        vec_M_diff = spd2vec_vec(M_diff); // 21x1
        ////////////////////////////////////////////////////////////////////////////////////////////

        // Dyn. Manip. /////////////////////////////////////////////////////////////////////////////
        // Method 1:
        // derivative of J*M⁻¹ wrt q (for dynamic manip.)
        // J_dyn_grad = jacobianEstDynManip(q_, n, robot, model_handle, total_inertia, total_mass, F_x_Ctotal);
        // // Compute dynamic manipulability Jacobian (red to matrix)
        // Jm_dyn_t = redManipulabilityJacobian(L_ct, J_dyn_grad);
        // Method 2:
        // Tensor<double, 3> mass_diff = jacobianEstMass(q_, n, robot);
        // Jm_dyn_t = redManipulabilityJacobianDyn(L_ct, J_grad, mass_diff, Mass);
        // M_dyn_diff = logmap(Me_dyn_d, Me_dyn_ct); // 6x6 // dyn. Manip.
        // with weiting term on Tz:
        // Matrix<double, 6, 1> w_Me_vec;
        // w_Me_vec << 0,0,0,0,0,5;
        // Matrix<double, 6, 6> w_Me = w_Me_vec.asDiagonal();
        // M_dyn_diff =  logmap(Me_dyn_d, Me_dyn_ct); // 6x6 // dyn. Manip. 

        // vec_M_dyn_diff = spd2vec_vec(M_dyn_diff); // 21x1 // dyn. Manip.
        ////////////////////////////////////////////////////////////////////////////////////////////

        array<DenseIndex, 3> offset_axis = {3, 0, 0}; // translation in x
        array<DenseIndex, 3> extent_axis = {1, 7, 7};
        // J_grad_axis = J_grad.slice(offset_axis, extent_axis);
        
        // Jm_t_axis_ = manipulabilityJacobian(J_geom_t_axis, J_grad_axis);
        // Jm_t_axis = Map<MatrixXd> (Jm_t_axis_.data(), 1, 7);
        // std::cout<<"Jm_t_axis_: "<<std::endl<<Jm_t_axis_ <<std::endl;
        
        // M_diff_axis = logmap(Me_d_axis, Me_ct_axis); // 1

        // std::cout<<"M_diff: "<<std::endl<<M_diff<<std::endl;
        // std::cout<<"M_diff_axis: "<<std::endl<<M_diff_axis<<std::endl;
        // std::cout<<"vec_M_dyn_diff: "<<std::endl<<vec_M_dyn_diff<<std::endl;

        // Calculate eigenvalue of the current M (singular value of J_geom)
        ev_t = singularsolver.compute(J_geom).singularValues();
        // std::cout<<"ev_t: -------------------"<<std::endl<<ev_t<<std::endl;
        ev_diff = jacobianEstVector(qt, n, robot);

        // ++++++++++++++++++++QP Controller using osqp-eigen+++++++++++++++++++++++++++++++++
        // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        // constexpr double tolerance = 1e-4;
        // Initialization
        Matrix<c_float, 31, 1> lb;
        Matrix<c_float, 31, 7> A;
        Matrix<c_float, 31, 1> ub;

        Matrix<c_float, 7, 7> H = 1*Jm_t.transpose()*Jm_t + J.transpose() * W_cart * J;
        // Dyn. Manip.
        // Matrix<c_float, 7, 7> H = 1*Jm_dyn_t.transpose()*Jm_dyn_t + J.transpose() * W_cart * J;

        H_s = H.sparseView();
        // std::cout<<"H: "<<std::endl<<H<<std::endl;
        H_s.pruned(1e-9); // set those smaller than 0.01 as zero
        // std::cout<<"H_S : "<<std::endl<<H_s<<std::endl;
        // Matrix<c_float, 1, 7> f = -K_qp* vec_M_diff.transpose()*Jm_t - 2 * (xd.vec8() - xt.vec8()).transpose() * J;

        // For Experiment 1: add cartesian position offset
        // To showcase the manipulability adapting
        Matrix<double, 3, 1> x_offset = x_desired.block(0,0,3,1);
        DQ xt_offset_t = DQ(x_offset);
        DQ xt_mean_r = DQ(xt_mean.block(0,0,4,1));
        DQ xt_obj = xt_mean_r + E_ * 0.5 * xt_offset_t * xt_mean_r;
        Matrix<c_float, 1, 7> f = -2*K_qp* vec_M_diff.transpose()*Jm_t - 2*(xt_obj.vec8() - xt.vec8()).transpose()* W_cart*K_cart *J;
        // Dyn. Manip.
        // Matrix<c_float, 1, 7> f = -2*K_qp* vec_M_dyn_diff.transpose()*Jm_dyn_t - 2*(xt_obj.vec8() - xt.vec8()).transpose()* W_cart*K_cart *J;

        // std::cout<<"f: "<<std::endl<<f<<std::endl;

        // Constraints:////////////////////////////////////////////////////
        // 1. set min. allowed eigenvalue (min. ellipsoid axis length)
        Matrix<c_float, 6, 1> ev_min;
        Matrix<c_float, 6, 1> v_max;
        ev_min << ev_min_r, ev_min_r, ev_min_r, ev_min_t, ev_min_t, ev_min_t;
        v_max = (ev_min - ev_t) * K_sing;
        // lb.block(0,0,6,1) = v_max;
        // A.block(0,0,6,7) = ev_diff;
        // ub.block(0,0,6,1).setConstant(OsqpEigen::INFTY);
        lb.block(0,0,6,1).setZero();
        A.block(0,0,6,7).setZero();
        ub.block(0,0,6,1).setZero();

        // 2. cartesian translation interval given by ProMP
        Eigen::Matrix<double, 3, 1> I_cartesian;
        I_cartesian.setIdentity();
        // dxr_t = (x_desired.block(0,0,3,1) - xt_t) * 1;
        // dxr_t = (x_desired.block(0,0,3,1) - xt_t) * 1 ;
        // dxr_t = -vec3(xt.translation() - xd.translation());
        // lb.block(6,0,3,1) = dxr_t - x_desired.block(3,0,3,1)*0.1;
        // A.block(6,0,3,7) = J_geom_t;
        // ub.block(6,0,3,1) = dxr_t + x_desired.block(3,0,3,1)*0.1;
        lb.block(6,0,3,1).setZero();
        A.block(6,0,3,7).setZero();
        ub.block(6,0,3,1).setZero();

        // MatrixXd lower = (dxr_t - x_desired.block(3,0,3,1));
        // MatrixXd upper = (dxr_t + x_desired.block(3,0,3,1));
        // lb.block(6,0,3,1) = lower;
        // A.block(6,0,3,7) = J_geom_t;
        // ub.block(6,0,3,1) = upper;
        // + D_cart * (dx.block(0,0,3,1) - dx_last.block(0,0,3,1));
        // + D_cart * (dx.block(0,0,3,1) - dx_last.block(0,0,3,1));

        // 3. prioritize manip tracking of one axis
        // lb.block(9,0,1,1) = M_diff_axis;
        // A.block(9,0,1,7) = Jm_t_axis;
        // ub.block(9,0,1,1) = M_diff_axis;
        lb.block(9,0,1,1).setZero();
        A.block(9,0,1,7).setZero();
        ub.block(9,0,1,1).setZero();

        // Bounds /////////////////////////////////////////////////////
        // a. Regarding joint position (tested):
        dq_min_q = (q_min-qt) * 1;
        dq_max_q = (q_max-qt) * 1;
        // lb.block(10,0,7,1) = dq_min_q;
        // A.block(10,0,7,7) = I;
        // ub.block(10,0,7,1) = dq_max_q;
        lb.block(10,0,7,1).setZero();
        A.block(10,0,7,7).setZero();
        ub.block(10,0,7,1).setZero();

        // b. Regarding joint velocity limit (tested):
        // lb.block(17,0,7,1) = dq_min;
        // A.block(17,0,7,7) = I;
        // ub.block(17,0,7,1) = dq_max;
        lb.block(17,0,7,1).setZero();
        A.block(17,0,7,7).setZero();
        ub.block(17,0,7,1).setZero();

        // c. Regarding joint acceleration:
        dq_min_ddq = dq_ - ddq_max * 0.1 ; // dt
        dq_max_ddq = dq_ + ddq_max * 0.1 ; // dt

        // lb.block(24,0,7,1) = dq_min_ddq;
        // A.block(24,0,7,7)= I;
        // ub.block(24,0,7,1) = dq_max_ddq;
        lb.block(24,0,7,1).setZero();
        A.block(24,0,7,7).setZero();
        ub.block(24,0,7,1).setZero();
        // std::cout<<"lb_limits: "<<std::endl<<lb_limits<<std::endl;
        // std::cout<<"ub_limits: "<<std::endl<<ub_limits<<std::endl;

        // Matrix<double, 7, 3> M_lb;
        // Matrix<double, 7, 3> M_ub;
        // M_lb.block(0,0,7,1) = dq_min_q;
        // M_lb.block(0,1,7,1) = dq_min;
        // M_lb.block(0,2,7,1) = dq_min_ddq;
        // M_ub.block(0,0,7,1) = dq_max_q;
        // M_ub.block(0,1,7,1) = dq_max;
        // M_ub.block(0,2,7,1) = dq_max_ddq;
        // std::cout<<"M_lb: "<<std::endl<<M_lb<<std::endl;
        // std::cout<<"M_ub: "<<std::endl<<M_ub<<std::endl;
        // VectorXd lb_limits;
        // lb_limits = M_lb.rowwise().maxCoeff();
        // VectorXd ub_limits;
        // ub_limits = M_ub.rowwise().minCoeff();

        // Matrix<double, 3, 1> dxr_Jr;
        // dxr_Jr = -vec3(log(xt.rotation().conj()*xd.rotation()));
        // Matrix<double, 4, 1> dxr_Jr;
        // dxr_Jr = xt_mean.block(0,0,4,1) - xt.rotation().vec4();
        // lb.block(17,0,4,1) = dxr_Jr;
        // A.block(17,0,4,7) = J_geom_r;
        // ub.block(17,0,4,1) = dxr_Jr;
        // lb.block(17,0,4,1).setZero();
        // A.block(17,0,4,7).setZero();
        // ub.block(17,0,4,1).setZero();

        
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
        dq_res = solver.getSolution();

        // for (int j = 0; j<3; j++){
        // MatrixXd Mi;
        // Mi = J_geom_t * dq_res;
        // double middle;
        // middle = Mi(j, 0);

        // MatrixXd Le;
        // Le = (dxr_t - x_desired.block(3,0,3,1));
        // double left;
        // left = Le(j, 0);

        // MatrixXd Ri;
        // Ri = (dxr_t + x_desired.block(3,0,3,1));
        // double right;
        // right = Ri(j, 0);

        // if ( middle < left || middle > right) {
        //     ROS_ERROR_STREAM("Unbouded Error");
        //     std::cout << "middle" << middle <<  "left" << left << "right" <<right << std::endl;
        // }
        // }
    }
    // std::cout << "Control finished..." << std::endl;
    // auto t2 = high_resolution_clock::now();

    /* Getting number of milliseconds as a double. */
    // duration<double, std::milli> ms_double = t2 - t1;
    // std::cout << ms_double.count() << "ms\n";
  
    return dq_res;
}
