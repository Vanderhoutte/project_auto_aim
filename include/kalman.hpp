#pragma once
#ifndef KALMAN_H
#define KALMAN_H

#include <cmath>
#include <iostream>
#include "Eigen/Dense.cpp"


class kalman {
    private:
        int state_dim,mea_dim;
        Eigen::MatrixXd x_hat;
        Eigen::MatrixXd P;
    public:
        kalman(int state_dim,int mea_dim):state_dim(state_dim),mea_dim(mea_dim)
        {
            x_hat = Eigen::MatrixXd::Zero(state_dim,1);
            std::cout << "x_hat:" << x_hat << std::endl;
            P = Eigen::MatrixXd::Identity(state_dim,state_dim);
            std::cout << "P:" << P << std::endl;
            std::cout<<"kalman construct"<<std::endl;
        }

        Eigen::MatrixXd get_p() const
        {
            return P;
        }

        void predict(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& u, const Eigen::MatrixXd& Q,bool B_in = true)
        {
            std::cout << "predict" << std::endl;
            std::cout << "A:" << A << std::endl;
            std::cout << "B:" << B << std::endl;
            std::cout << "u:" << u << std::endl;
            if(B_in)
            {
                std::cout << "B_in" << std::endl;
                x_hat = A * x_hat + B * u;
                std::cout << x_hat << std::endl;
                P = A * P * A.transpose() + Q;
                std::cout << P << std::endl;
            }
            else
            {
                std::cout << "B_out" << std::endl;
                x_hat = A * x_hat;
                std::cout << x_hat << std::endl;
                P = A * P * A.transpose() + Q;
                std::cout << P << std::endl;
            }
        }

        void update(const Eigen::MatrixXd& H, const Eigen::MatrixXd& z, const Eigen::MatrixXd& R)
        {
            std::cout << "update" << std::endl;
            std::cout << "H:" <<H << std::endl;
            std::cout << "z:" << z << std::endl;
            std::cout << "R:" << R << std::endl;
            std::cout << "P:" << P << std::endl;
            Eigen::MatrixXd S = (H * P * H.transpose() + R);
            std::cout << "S:" << S << std::endl;
            Eigen::MatrixXd K = P * H.transpose() * S.inverse();
            std::cout << "K:" << K << std::endl;
            x_hat = x_hat + K * (z - H * x_hat);
            std::cout << "x_hat:" << x_hat << std::endl;
            P= (Eigen::MatrixXd::Identity(state_dim, state_dim) - K * H) * P;
            std::cout << "P:" << P << std::endl;
        }

        void elementChange_P(int n)
        {
            P *= std::pow(10,n);
            std::cout << P << std::endl;
        }

        void elementChange_X(Eigen::MatrixXd X_new)
        {
            x_hat = X_new;
        }

        Eigen::MatrixXd get_state() const
        {
            return x_hat;
        }
};

#endif 