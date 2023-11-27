/*
 * @Author: wpbit
 * @Date: 2023-11-27 14:07:12
 * @LastEditors: wpbit
 * @LastEditTime: 2023-11-27 16:18:55
 * @Description: 
 */
#include "./include/following.h"

int main()
{
    // 预测时刻
    int horizon = 3;

    // 输入 {a, b}, a->相对车速，自车-目标车; b->相对距离，目标车-自车 > 0
    std::vector<std::vector<double>> input{{5.0, 25.0}, {5.0, 15.0}, {5.0, 10.0}};

    int num_variables = 3*horizon - 1;
    int num_constraints = 5*horizon - 3;

    // QP参数矩阵
    // [v_r, d_r, a] * (horizon-1)
    // [v_r - 0, d_r - TARGET_D, a - 0]^T * [v_r - 0, d_r - TARGET_D, a - 0]的二次系数为H，一次为G
    Eigen::MatrixXd H = Eigen::MatrixXd::Identity(num_variables, num_variables);
    H(0, 0) = 0;
    H(1 ,1) = 0;
    Eigen::VectorXd G = Eigen::VectorXd::Zero(num_variables);
    for(int i = 1; i < horizon; ++i){
        G(3 * i + 1) = -2 * input[i][1];
    }
    // std::cout << H << std::endl;
    // std::cout << G << std::endl;

    // 约束矩阵
    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(num_constraints, num_variables);
    // 状态量和控制量不等式约束 3*horizon-1个
    for(int i = 0; i < num_variables; ++i){
        C(i, i) = 1;
    }
    // 状态方程等式约束 horizon-1个
    int start_index = num_variables;
    for(int i = 0; i < horizon-1; ++i){
        C(start_index + 2*i, 3 * (i+1)) = 1;
        C(start_index + 2*i, 3 * i) = -1;
        C(start_index + 2*i, 3 * i + 2) = -DT;
        C(start_index + 2*i+1, 3 * (i+1) + 1) = 1;
        C(start_index + 2*i+1, 3*i+1) = -1;
        C(start_index + 2*i+1, 3 * i) = DT;
        C(start_index + 2*i+1, 3 * i + 2) = 0.5 * DT * DT;
    }
    // std::cout << C << std::endl;

    // 约束汇总
    Eigen::VectorXd lowerBound(num_constraints);
    for(int i = 0; i < horizon; ++i){
        if(i == 0){
            lowerBound(i) = input[0][0];
            lowerBound(i+1) = input[0][1];
        }else{
            lowerBound(3 * i - 1) = MIN_ACC;
            lowerBound(3 * i) = -2.0;
            lowerBound(3 * i + 1) = MIN_D;
        }
    }
    for(int i = 0; i < horizon-1; ++i){
        lowerBound(start_index + 2*i) = -0.5;
        lowerBound(start_index + 2*i+1) = -0.5;
    }
    Eigen::VectorXd upperBound(num_constraints);
    for(int i = 0; i < horizon; ++i){
        if(i == 0){
            upperBound(i) = input[0][0];
            upperBound(i+1) = input[0][1];
        }else{
            upperBound(3 * i - 1) = MAX_ACC;
            upperBound(3 * i) = 2.0;
            upperBound(3 * i + 1) = MAX_D;
        }
    }
    for(int i = 0; i < horizon-1; ++i){
        upperBound(start_index + 2*i) = 0.5;
        upperBound(start_index + 2*i+1) = 0.5;
    }
    // std::cout << lowerBound << std::endl;
    // std::cout << upperBound << std::endl;

    // osqp求解
    Eigen::SparseMatrix<double> hessian;
    Eigen::VectorXd gradient;
    Eigen::SparseMatrix<double> linearMatrix;
    hessian = H.sparseView(); // 转换为稀疏矩阵
    gradient = G;
    linearMatrix = C.sparseView();

    static OsqpEigen::Solver solver;
    // solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);
    solver.data()->setNumberOfVariables(hessian.cols());
    solver.data()->setNumberOfConstraints(linearMatrix.rows());

    if (!solver.data()->setHessianMatrix(hessian))
        return false;
    if (!solver.data()->setLinearConstraintsMatrix(linearMatrix))
        return false;
    if (!solver.data()->setGradient(gradient))
        return false; // 注意，一次项系数set必须为一维数组，不能为矩阵
    if (!solver.data()->setLowerBound(lowerBound))
        return false;
    if (!solver.data()->setUpperBound(upperBound))
        return false;
    if (!solver.initSolver())
        return false;
    if (static_cast<int>(solver.solveProblem()) != 0)
        return false;

    Eigen::VectorXd output = solver.getSolution();
    cout << output << endl;

    return 0;
}