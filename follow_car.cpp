/*
 * @Author: wpbit
 * @Date: 2023-11-27 14:07:12
 * @LastEditors: wpbit
 * @LastEditTime: 2023-12-05 16:43:46
 * @Description:
 */
#include "./include/following.h"

class solver
{
private:
    int horizon_ = 0;
    int num_variables_ = 0;
    int num_constraints_ = 0;

public:
    solver(int horizon);
    bool solve(std::vector<std::vector<double>> input, Eigen::VectorXd &out);
    ~solver();
};

solver::solver(int horizon)
{
    horizon_ = horizon;
    num_variables_ = 3 * horizon - 1;
    num_constraints_ = 5 * horizon - 3;
}

solver::~solver()
{
}

bool solver::solve(std::vector<std::vector<double>> input, Eigen::VectorXd &out)
{
    // for (int i = 0; i < horizon_; ++i)
    // {
    //     // input[i][0] -= 2.0;
    //     input[i][1] -= 20.0;
    // }

    // QP参数矩阵
    // [v_r, d_r, a] * (horizon-1)
    // [v_r - 0, d_r - TARGET_D, a - 0]^T * [v_r - 0, d_r - TARGET_D, a - 0]的二次系数为H，一次为G
    Eigen::MatrixXd H = Eigen::MatrixXd::Identity(num_variables_, num_variables_);
    // H(0, 0) = 0;
    // H(1 ,1) = 0;
    for (int i = 0; i < horizon_; ++i)
    {
        // 相对速度
        H(i * 3, i * 3) = 1;
        // 相对距离
        H(i * 3 + 1, i * 3 + 1) = 2.0;
        // 加速度
        if(i == horizon_ - 1) break;
        H(3 * i + 2, 3 * i + 2) = 1.5;
    }
    
    Eigen::VectorXd G = Eigen::VectorXd::Zero(num_variables_);
    for (int i = 0; i < horizon_; ++i)
    {
        G(3 * i + 1) = -2 * TARGET_D;
        G(3 * i) = 2 * (Speed_f - TARGET_SPEED);
    }
    // std::cout << H << std::endl;
    // std::cout << G << std::endl;

    // 约束矩阵
    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(num_constraints_, num_variables_);
    // 状态量和控制量不等式约束 3*horizon-1个
    for (int i = 0; i < num_variables_; ++i)
    {
        C(i, i) = 1;
    }
    // 状态方程等式约束 horizon-1个
    int start_index = num_variables_;
    for (int i = 0; i < horizon_ - 1; ++i)
    {
        C(start_index + 2 * i, 3 * (i + 1)) = 1;
        C(start_index + 2 * i, 3 * i) = -1;
        C(start_index + 2 * i, 3 * i + 2) = -DT;
        C(start_index + 2 * i + 1, 3 * (i + 1) + 1) = 1;
        C(start_index + 2 * i + 1, 3 * i + 1) = -1;
        C(start_index + 2 * i + 1, 3 * i) = DT;
        C(start_index + 2 * i + 1, 3 * i + 2) = 0.5 * DT * DT;
    }
    // std::cout << C << std::endl;

    // 约束汇总
    Eigen::VectorXd lowerBound(num_constraints_);
    for (int i = 0; i < horizon_; ++i)
    {
        if (i == 0)
        {
            lowerBound(i) = input[0][0];
            lowerBound(i + 1) = input[0][1];
            // lowerBound(i) = MIN_SPEED - Speed_f;
            // lowerBound(i + 1) = MIN_D;
        }
        else
        {
            lowerBound(3 * i - 1) = MIN_ACC;
            lowerBound(3 * i) = MIN_SPEED - Speed_f;
            lowerBound(3 * i + 1) = MIN_D;
        }
    }
    for (int i = 0; i < horizon_ - 1; ++i)
    {
        lowerBound(start_index + 2 * i) = 0;
        lowerBound(start_index + 2 * i + 1) = 0;
    }
    Eigen::VectorXd upperBound(num_constraints_);
    for (int i = 0; i < horizon_; ++i)
    {
        if (i == 0)
        {
            upperBound(i) = input[0][0];
            upperBound(i + 1) = input[0][1];
            // upperBound(i) = MAX_SPEED - Speed_f;
            // upperBound(i + 1) = MAX_D;
        }
        else
        {
            upperBound(3 * i - 1) = MAX_ACC;
            upperBound(3 * i) = MAX_SPEED - Speed_f;
            upperBound(3 * i + 1) = MAX_D;
        }
    }
    for (int i = 0; i < horizon_ - 1; ++i)
    {
        upperBound(start_index + 2 * i) = 0;
        upperBound(start_index + 2 * i + 1) = 0;
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

    out = solver.getSolution();
    // cout << out << endl;
    return true;
}

int main()
{
    // 预测时刻
    int horizon = 20;

    // 仿真参数初始化
    float S_f = S0_f;
    float S_r = S0_r;
    float Speed_r = Speed0_r;
    std::vector<float> Vec_Vr{-Speed_f};
    std::vector<float> Vec_t{0};
    std::vector<float> Vec_D{S0_f - S0_r};
    std::vector<float> Vec_Acc{0};

    // 输入 {a, b}, a->相对车速，自车-目标车; b->相对距离，目标车-自车 > 0
    std::vector<std::vector<double>> input;
    input.resize(horizon);
    input[0] = {-Speed_f, S0_f};
    for(int i = 1; i < horizon; ++i){
        input[i] = {-Speed_f, S0_f+Speed_f*DT*i};
    }
    
    for (int sim = 0; sim < 200; sim++)
    {

        Eigen::VectorXd out;
        solver test(horizon);
        test.solve(input, out);

        // 状态更新
        // 前车状态更新
        S_f = S_f + Speed_f * DT;
        // 后车状态更新
        float AccNew = out(2);
        float Vel = input[0][0] + Speed_f;
        if(Vel < 0) Vel = 0;
        if(Vel < 1e-5 && AccNew < 0) AccNew = 0;
        input[0][0] += AccNew * DT;
        input[0][1] = input[0][1] - Vel * DT - 0.5 * AccNew * DT * DT + Speed_f * DT;

        for(int i = 1; i < horizon; ++i){
            input[i][0] = out(3 * i);
            input[i][1] = out(3 * i + 1);
        }

        std::cout << "加速度:" << AccNew << std::endl;
        std::cout << "相对速度:" << input[0][0] << std::endl;
        std::cout << "相对车距:" << input[0][1] << std::endl;

        Vec_D.push_back(input[0][1]);
        Vec_Vr.push_back(input[0][0]);
        Vec_t.push_back((sim + 1) * DT);
        Vec_Acc.push_back(AccNew);
    }

    plt::figure_size(1500, 500);
    plt::subplot(1, 3, 1);
    plt::plot(Vec_t, Vec_Vr, "b-");
    plt::xlim(0, int(Vec_t.back()));
    plt::ylim(-5, 5);
    plt::xlabel("t (s)");
    plt::ylabel("Vel (km/h)");
    plt::title("Vel-t");
    plt::grid(1);

    plt::subplot(1, 3, 2);
    plt::plot(Vec_t, Vec_Acc, "b-");
    plt::xlim(0, int(Vec_t.back()));
    plt::ylim(-5, 5);
    plt::xlabel("t (s)");
    plt::ylabel("Acc (m/s2)");
    plt::title("Acc-t");
    plt::grid(1);

    plt::subplot(1, 3, 3);
    plt::plot(Vec_t, Vec_D, "r-");
    plt::xlim(0, int(Vec_t.back()));
    plt::ylim(0, 50);
    plt::xlabel("t (s)");
    plt::ylabel("D (m)");
    plt::title("D-t");
    plt::grid(1);
    plt::show();
    return 0;
}