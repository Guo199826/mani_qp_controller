#include "../include/jacobianEst.h"

Tensor<double, 3> jacobianEst(const VectorXd& q, const int n,
    const DQ_SerialManipulator &robot) 
{
    double q_delta = 0.0001;
    VectorXd q_add(n);
    q_add.setZero();
    
    VectorXd q_ii;
    VectorXd q_i;
    // Matrix vs MatrixXd??? Matrix could not be returned??
    Matrix<double, 8, 7> J_ii;
    Matrix<double, 8, 7> J_i;
    Matrix<double, 6, 7> J_geom_ii;
    Matrix<double, 6, 7> J_geom_i;

    Tensor<double, 3> JE(6, 7, 7);
    // //test:
    // Matrix<double, 8, 7> J;
    // J = robot.pose_jacobian(q);
    // Matrix<double, 6, 7> J_geom;
    // J_geom = fct_geomJac(robot,J,q,n);
    // // std::cout<<"J_geom at q: "<<std::endl<<J_geom<<std::endl;

    Matrix<double, 6, 7> J_result;

    for (int i=0; i<n; i++){
        q_add(i) = q_delta;
        q_ii = q + q_add;
        q_i = q - q_add;
        
        J_ii = robot.pose_jacobian(q_ii);
        J_i = robot.pose_jacobian(q_i);
        // std::cout<<"J_ii of "<<i<<std::endl<<J_ii<<std::endl;
        // std::cout<<"J_i: "<<i<<std::endl<<J_i<<std::endl;
        J_geom_ii = geomJac(robot,J_ii,q_ii,n);
        J_geom_i = geomJac(robot,J_i,q_i,n);
        // std::cout<<"J_geom_ii: "<<i<<std::endl<<J_geom_ii<<std::endl;
        // std::cout<<"J_geom_i: "<<i<<std::endl<<J_geom_i<<std::endl;

        // Calculate the jacobian of geom jacobian
        array<DenseIndex, 3> offset = {0, 0, i};
        array<DenseIndex, 3> extent = {6, 7, 1};
        J_result = (J_geom_ii - J_geom_i)/(2*q_delta);
        // std::cout<<"J_result at vel of "<<i<<std::endl<<J_result<<std::endl;
        // constexpr int rank = sizeof... (Dims);
        Tensor<double, 3> J_result_m2t = TensorMap<Tensor<double, 3>>(J_result.data(), 6, 7, 1);
        // std::cout<<"Check result of (:,:,i): "<<std::endl<<J_result_m2t<<std::endl;
        // Tensor<double, 3> slice = JE.slice(offset, extent);
        JE.slice(offset, extent) = J_result_m2t;
        // std::cout<<"Jacobian of Jacobian: "<<i<<std::endl<<JE<<std::endl;
        q_add(i) = 0.0;
    }
    
    // std::cout<<"Jacobian of Jacobian test point: "<<std::endl<<JE(2,1,4)<<std::endl;
    // std::cout<<"Jacobian of Jacobian test point: "<<std::endl<<JE(4,4,3)<<std::endl;
    // std::cout<<"Jacobian of Jacobian test point: "<<std::endl<<JE(1,3,1)<<std::endl;

    return JE;
}

Tensor<double, 3> jacobianEstMass(const Matrix<double, 7, 1>& q, const int n,
    const DQ_SerialManipulator &robot) 
{
    double q_delta = 0.0001;
    // double q_delta = 0.000001;
    Matrix<double, 7, 1> q_add;
    q_add.setZero();
    
    Matrix<double, 7, 1> q_ii;
    Matrix<double, 7, 1> q_i;

    Tensor<double, 3> JE(7, 7, 7);
    // //test:
    // Matrix<double, 8, 7> J;
    // J = robot.pose_jacobian(q);
    // Matrix<double, 6, 7> J_geom;
    // J_geom = fct_geomJac(robot,J,q,n);
    // // std::cout<<"J_geom at q: "<<std::endl<<J_geom<<std::endl;

    Matrix<double, 7, 7> J_result;

    for (int i=0; i<n; i++){
        q_add(i) = q_delta;
        q_ii = q + q_add;
        q_i = q - q_add;
        
        // dyn. param
        // 1. From Frankaros getMass()
        // Mass matrix for q_ii:
        // std::array<double, 7> q_ii_array;
        // Matrix<double, 7, 1>::Map(q_ii_array.data()) = q_ii;
        // std::array<double, 49> mass_array_ii = model_handle->getMass(q_ii_array, total_inertia, total_mass, F_x_Ctotal);
        // Map<Matrix<double, 7, 7>> Mass_ii(mass_array_ii.data());
        // // std::cout<<"Mass_ii of "<<i<<std::endl<<Mass_ii<<std::endl;
        // // Mass matrix for q_i:
        // std::array<double, 7> q_i_array;
        // Matrix<double, 7, 1>::Map(q_i_array.data()) = q_i;
        // std::array<double, 49> mass_array_i = model_handle->getMass(q_i_array, total_inertia, total_mass, F_x_Ctotal);
        // Map<Matrix<double, 7, 7>> Mass_i(mass_array_i.data());
        // std::cout<<"Mass_i of "<<i<<std::endl<<Mass_i<<std::endl;
        // 2. From github repo function
        Matrix<double, 7, 7> Mass_ii = MassMatrix(q_ii);
        Matrix<double, 7, 7> Mass_i = MassMatrix(q_i);

        // Calculate the jacobian of geom jacobian
        array<DenseIndex, 3> offset = {0, 0, i};
        array<DenseIndex, 3> extent = {7, 7, 1};
        J_result = (Mass_ii - Mass_i)/(2*q_delta);
        // std::cout<<"J_result by dyn of "<<i<<std::endl<<J_result<<std::endl;
        // constexpr int rank = sizeof... (Dims);
        Tensor<double, 3> J_result_m2t = TensorMap<Tensor<double, 3>>(J_result.data(), 7, 7, 1);
        // std::cout<<"Check result of (:,:,i): "<<std::endl<<J_result_m2t<<std::endl;
        // Tensor<double, 3> slice = JE.slice(offset, extent);
        JE.slice(offset, extent) = J_result_m2t;
        // std::cout<<"Jacobian of Jacobian: "<<i<<std::endl<<JE<<std::endl;
        q_add(i) = 0.0;
    }
    
    // std::cout<<"Jacobian of Jacobian test point: "<<std::endl<<JE(2,1,4)<<std::endl;
    // std::cout<<"Jacobian of Jacobian test point: "<<std::endl<<JE(4,4,3)<<std::endl;
    // std::cout<<"Jacobian of Jacobian test point: "<<std::endl<<JE(1,3,1)<<std::endl;

    return JE;
}

Tensor<double, 3> jacobianEstDynManip(const Matrix<double, 7, 1>& q, const int n,
    const DQ_SerialManipulator &robot,
    const std::unique_ptr<franka_hw::FrankaModelHandle> &model_handle,
    const std::array<double, 9> &total_inertia,
    const double total_mass,
    const std::array<double, 3> &F_x_Ctotal) 
{
    double q_delta = 0.0001;
    // double q_delta = 0.000001;
    Matrix<double, 7, 1> q_add;
    q_add.setZero();
    
    Matrix<double, 7, 1> q_ii;
    Matrix<double, 7, 1> q_i;
    // Matrix vs MatrixXd??? Matrix could not be returned??
    Matrix<double, 8, 7> J_ii;
    Matrix<double, 8, 7> J_i;
    Matrix<double, 6, 7> J_geom_ii;
    Matrix<double, 6, 7> J_geom_i;

    Tensor<double, 3> JE(6, 7, 7);
    // //test:
    // Matrix<double, 8, 7> J;
    // J = robot.pose_jacobian(q);
    // Matrix<double, 6, 7> J_geom;
    // J_geom = fct_geomJac(robot,J,q,n);
    // // std::cout<<"J_geom at q: "<<std::endl<<J_geom<<std::endl;

    Matrix<double, 6, 7> J_result;

    for (int i=0; i<n; i++){
        q_add(i) = q_delta;
        q_ii = q + q_add;
        q_i = q - q_add;
        
        // dyn. param
        // 1. From Frankaros getMass()
        // Mass matrix for q_ii:
        // std::array<double, 7> q_ii_array;
        // Matrix<double, 7, 1>::Map(q_ii_array.data()) = q_ii;
        // std::array<double, 49> mass_array_ii = model_handle->getMass(q_ii_array, total_inertia, total_mass, F_x_Ctotal);
        // Map<Matrix<double, 7, 7>> Mass_ii(mass_array_ii.data());
        // // std::cout<<"Mass_ii of "<<i<<std::endl<<Mass_ii<<std::endl;
        // // Mass matrix for q_i:
        // std::array<double, 7> q_i_array;
        // Matrix<double, 7, 1>::Map(q_i_array.data()) = q_i;
        // std::array<double, 49> mass_array_i = model_handle->getMass(q_i_array, total_inertia, total_mass, F_x_Ctotal);
        // Map<Matrix<double, 7, 7>> Mass_i(mass_array_i.data());
        // std::cout<<"Mass_i of "<<i<<std::endl<<Mass_i<<std::endl;
        // 2. From github repo function
        Matrix<double, 7, 7> Mass_ii = MassMatrix(q_ii);
        Matrix<double, 7, 7> Mass_i = MassMatrix(q_i);

        J_ii = robot.pose_jacobian(q_ii);
        J_i = robot.pose_jacobian(q_i);
        // std::cout<<"J_ii of "<<i<<std::endl<<J_ii<<std::endl;
        // std::cout<<"J_i: "<<i<<std::endl<<J_i<<std::endl;
        J_geom_ii = geomJac(robot,J_ii,q_ii,n);
        J_geom_i = geomJac(robot,J_i,q_i,n);
        // std::cout<<"J_geom_ii: "<<i<<std::endl<<J_geom_ii<<std::endl;
        // std::cout<<"J_geom_i: "<<i<<std::endl<<J_geom_i<<std::endl;
        Matrix<double, 6, 7> L_ii = J_geom_ii* Mass_ii.inverse();
        Matrix<double, 6, 7> L_i = J_geom_i* Mass_i.inverse();

        // Calculate the jacobian of geom jacobian
        array<DenseIndex, 3> offset = {0, 0, i};
        array<DenseIndex, 3> extent = {6, 7, 1};
        J_result = (L_ii - L_i)/(2*q_delta);
        // std::cout<<"J_result by dyn of "<<i<<std::endl<<J_result<<std::endl;
        // constexpr int rank = sizeof... (Dims);
        Tensor<double, 3> J_result_m2t = TensorMap<Tensor<double, 3>>(J_result.data(), 6, 7, 1);
        // std::cout<<"Check result of (:,:,i): "<<std::endl<<J_result_m2t<<std::endl;
        // Tensor<double, 3> slice = JE.slice(offset, extent);
        JE.slice(offset, extent) = J_result_m2t;
        // std::cout<<"Jacobian of Jacobian: "<<i<<std::endl<<JE<<std::endl;
        q_add(i) = 0.0;
    }
    
    // std::cout<<"Jacobian of Jacobian test point: "<<std::endl<<JE(2,1,4)<<std::endl;
    // std::cout<<"Jacobian of Jacobian test point: "<<std::endl<<JE(4,4,3)<<std::endl;
    // std::cout<<"Jacobian of Jacobian test point: "<<std::endl<<JE(1,3,1)<<std::endl;

    return JE;
}
