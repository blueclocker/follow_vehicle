/*
 * @Description:
 * @Version: 2.0
 * @Author: ZHAO B.T.
 * @Date: 2023-11-23 12:01:15
 * @LastEditors: ZHAO B.T.
 * @LastEditTime: 2023-11-26 22:47:56
 */
#ifndef FOLLOWING_H
#define FOLLOWING_H

#define MAX_SPEED 40 / 3.6
#define MIN_SPEED 0 / 3.6
#define MAX_D 30 // 最大车距
#define MIN_D 5  // 最小车距
#define MAX_REL_SPEED 40 / 3.6
#define MIN_REL_SPEED -40 / 3.6
#define MAX_ACC 2.5
#define MIN_ACC -5.0
#define MAX_JERK 2.5
#define MIN_JERK -2.5
#define TARGET_SPEED 20 / 3.6
#define TARGET_D 10 // 目标车距
#define DT 0.2

#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include </home/zbt/lib/eigen-3.4.0/unsupported/Eigen/KroneckerProduct>
#include "OsqpEigen/OsqpEigen.h"
#include <osqp/osqp.h>
// #include "matplotlibcpp.h"
// namespace plt = matplotlibcpp;
using namespace std;
using namespace Eigen;
using namespace OsqpEigen;

#endif