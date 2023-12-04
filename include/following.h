/*
 * @Description:
 * @Version: 2.0
 * @Author: ZHAO B.T.
 * @Date: 2023-11-23 12:01:15
 * @LastEditors: ZHAO B.T.
 * @LastEditTime: 2023-12-04 22:17:14
 */
#ifndef FOLLOWING_H
#define FOLLOWING_H

// 自车信息
#define MAX_SPEED 40 / 3.6
#define MIN_SPEED 0 / 3.6
#define MAX_D 100 // 最大车距
#define MIN_D -20 // 最小车距
#define MAX_REL_SPEED 40 / 3.6
#define MIN_REL_SPEED -40 / 3.6
#define MAX_ACC 2.5
#define MIN_ACC -5.0
#define MAX_JERK 2.5
#define MIN_JERK -2.5
#define TARGET_SPEED 20 / 3.6
#define TARGET_D 25 // 目标车距
#define DT 0.2
#define S0_r 0           // 后车初始位置
#define Speed0_r 0 / 3.6 // 后车初始车速

// 前车信息
#define S0_f 20          // 前车初始位置
#define Speed_f 20 / 3.6 // 前车车速

#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <eigen3/unsupported/Eigen/KroneckerProduct>
#include "osqp/osqp.h"
#include "OsqpEigen/OsqpEigen.h"
// #include "matplotlibcpp.h"
// namespace plt = matplotlibcpp;
using namespace std;
using namespace Eigen;
using namespace OsqpEigen;

#endif