/*
 * @Description:
 * @Version: 2.0
 * @Author: ZHAO B.T.
 * @Date: 2023-11-23 09:33:52
 * @LastEditors: wpbit
 * @LastEditTime: 2023-11-27 14:09:17
 */
#include "./include/following.h"

int main()
{

    // 模型参数矩阵
    MatrixXd Q(3, 3); // 状态变量误差系数矩阵
    Q << 1, 0, 0,
        0, 1, 0,
        0, 0, 1;

    MatrixXd R(1, 1); // 控制变量误差矩阵
    R << 1;
    MatrixXd QN = Q;  // 末状态状态变量系数矩阵
    MatrixXd A(3, 3); // 状态矩阵
    A << 1, 0, 0,
        0, 1, DT,
        0, 0, 1;
    VectorXd B(3); // 控制矩阵
    B << DT, -0.5 * DT * DT, -DT;

    // 初始值
    VectorXd x0(3);
    x0 << 0, 20, 20;
    int nx = x0.size();
    VectorXd xr(3);
    xr << TARGET_SPEED, TARGET_D, 0;
    VectorXd u0(1);
    u0 << 0;
    int nu = u0.size();

    // 约束
    VectorXd umin(1);
    umin << MIN_ACC;
    VectorXd umax(1);
    umax << MAX_ACC;
    VectorXd xmin(3);
    xmin << MIN_SPEED, MIN_D, MIN_REL_SPEED;
    VectorXd xmax(3);
    xmax << MAX_SPEED, MAX_D, MAX_REL_SPEED;

    // 预测时域
    int N = 3;

    // QP参数矩阵
    MatrixXd I = MatrixXd::Identity(N, N);
    MatrixXd P1 = kroneckerProduct(I, Q);
    MatrixXd P2 = kroneckerProduct(I, R);
    MatrixXd P = MatrixXd::Zero((P1.rows() + QN.rows() + P2.rows()), (P1.cols() + QN.cols() + P2.cols()));
    P.block(0, 0, P1.rows(), P1.cols()) = P1;
    P.block(P1.rows(), P1.cols(), QN.rows(), QN.cols()) = QN;
    P.block(P1.rows() + QN.rows(), P1.cols() + QN.cols(), P2.rows(), P2.cols()) = P2;

    // cout << P << endl;
    // cout << P.cols() << endl;
    // cout << P.rows() << endl;

    VectorXd i = VectorXd::Ones(N);
    VectorXd q1 = kroneckerProduct(i, -Q * xr);
    VectorXd q2 = -QN * xr;
    VectorXd q = VectorXd::Zero((N + 1) * nx + N * nu);
    q.segment(0, q1.size()) = q1; // vector用segment对指定位置的向量赋值，matrix用block
    q.segment(q1.size(), q2.size()) = q2;

    // cout << q << endl;
    // cout << q.size() << endl;

    // 等式约束
    MatrixXd I1 = MatrixXd::Identity(N + 1, N + 1);
    MatrixXd I2 = MatrixXd::Identity(nx, nx);
    MatrixXd I3 = MatrixXd::Zero(N + 1, N + 1);
    for (size_t i = 1; i < N + 1; i++) // 注意，要检索第一条主对角线的元素，i要从1开始，因为起始位置为(1,0)
    {
        I3(i, i - 1) = 1;
    }
    MatrixXd A1 = kroneckerProduct(I1, -I2);
    MatrixXd A2 = kroneckerProduct(I3, A);
    MatrixXd Ax = A1 + A2;

    MatrixXd I4 = I1.block(0, 1, N + 1, N);
    MatrixXd Bu = kroneckerProduct(I4, B);

    MatrixXd Aeq((N + 1) * nx, (N + 1) * nx + N * nu);
    Aeq << Ax, Bu;

    VectorXd leq = VectorXd::Zero((N + 1) * nx);
    for (size_t j = 0; j < nx; j++)
    {
        leq(j) = -x0(j);
    }
    VectorXd ueq = leq;

    // cout << A << endl;
    // cout << Ax << endl;
    // cout << Ax.cols() << endl;
    // cout << Ax.rows() << endl;
    // cout << Bu << endl;
    // cout << Bu.cols() << endl;
    // cout << Bu.rows() << endl;
    // cout << I4 << endl;
    // cout << I4.cols() << endl;
    // cout << I4.rows() << endl;

    // cout << leq << endl;
    // cout << leq.size() << endl;

    // 不等式约束
    MatrixXd Aineq = MatrixXd::Identity(nx * (N + 1) + nu * N, nx * (N + 1) + nu * N);

    VectorXd I5 = VectorXd::Ones(N + 1);
    VectorXd I6 = VectorXd::Ones(N);
    VectorXd lineq1 = kroneckerProduct(I5, xmin);
    VectorXd lineq2 = kroneckerProduct(I6, umin);
    VectorXd lineq(nx * (N + 1) + nu * N);
    lineq << lineq1, lineq2;
    VectorXd uineq1 = kroneckerProduct(I5, xmax);
    VectorXd uineq2 = kroneckerProduct(I6, umax);
    VectorXd uineq(nx * (N + 1) + nu * N);
    uineq << uineq1, uineq2;

    // cout << uineq << endl;
    // cout << uineq.cols() << endl;
    // cout << uineq.rows() << endl;

    // 约束汇总
    MatrixXd G(2 * nx * (N + 1) + nu * N, nx * (N + 1) + nu * N);
    G << Aeq,
        Aineq;
    VectorXd lowerBound(2 * nx * (N + 1) + nu * N);
    lowerBound << leq, lineq;
    VectorXd upperBound(2 * nx * (N + 1) + nu * N);
    upperBound << ueq, uineq;

    // cout << G << endl;
    // cout << lowerBound << endl
    //      << endl;
    // cout << upperBound << endl;

    // osqp求解
    SparseMatrix<double> hessian;
    Eigen::VectorXd gradient;
    SparseMatrix<double> linearMatrix;
    hessian = P.sparseView(); // 转换为稀疏矩阵
    gradient = q;
    linearMatrix = G.sparseView();

    OsqpEigen::Solver solver;
    solver.settings()->setVerbosity(false);
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

    VectorXd output = solver.getSolution();
    // cout << output << endl;
    return 0;
}