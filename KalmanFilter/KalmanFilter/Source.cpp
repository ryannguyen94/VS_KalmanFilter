/*
 * main.cpp
 *
 *  Created on: Mar 19, 2019
 *      Author: Ryan
 */

#include <iostream>
#include <Eigen/Dense>
#include <vector>

using namespace Eigen;

template<class matrix>
class kalmanFilter
{
public:
	matrix A, C, Q, R, P, Pprev, I, xPrev;

	kalmanFilter(matrix mA, matrix mC, matrix mQ, matrix mR, matrix x0)
	{
		A = mA;
		C = mC;
		Q = mQ;
		R = mR;
		xPrev = x0;
		P = matrix::Zero(A.rows(), A.cols());
		Pprev = P;
		I = matrix::Identity(A.rows(), A.cols());
	}

	VectorXd step(matrix sensorData)
	{
		matrix xPred = A * xPrev;
		matrix Ppred = A * Pprev*A.transpose() + Q;
		matrix K = Ppred * C.transpose()*(R + C * Ppred*C.transpose()).inverse();
		P = (I - K * C)*Ppred*(I - K * C).transpose() + K * R*K.transpose();
		matrix x = xPred + K * (sensorData - C * xPred);
		xPrev = x;
		return x;
	}
};

static std::vector<MatrixXd> generateData()
{
	std::vector<MatrixXd> sensorData;
	MatrixXd y0(2, 1);
	y0 << 0,
		  0;
	sensorData.push_back(y0);
	double ts = 0.01;
	double accel = 4.0;
	for (int i = 1; i < 10; i++)
	{
		double ws = sensorData.back()(0,0) + accel * ts;
		MatrixXd y(2, 1);
		y << ws,
			3.0;
		sensorData.push_back(y);
	}
	return sensorData;
}

int main()
{
	std::vector<MatrixXd> sensorData = generateData();
	double dt = 0.1;
	MatrixXd A(2, 2);
	A << 1, 0.1,
		0, 1;
	MatrixXd B(2, 1);
	B << 1,
		1;
	MatrixXd C(2, 2);
	C << 1, 0,
		0, 1;
	MatrixXd Q(2, 2);
	Q << 1, 0,
		0, 1;
	MatrixXd R(2, 2);
	R << 1, 0,
		0, 1;
	MatrixXd x0(2, 1);
	x0 << 0,
		  0;
	std::vector<MatrixXd> x;
	kalmanFilter<MatrixXd> kf(A, C, Q, R, x0);
	for (int i = 0; i < (int) sensorData.size(); i++)
	{
		x.push_back(kf.step(sensorData[i]));
		std::cout << x[i] << "\n";
	}
	std::cin.get();
}
