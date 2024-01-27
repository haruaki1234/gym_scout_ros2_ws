#pragma once

#include <Eigen/Core>
#include <Eigen/LU>

namespace tlab
{

class KalmanFilter {
public:
    /**
     * @brief No initialization constructor.
     */
    KalmanFilter() {}

    /**
     * @brief constructor with initialization
     * @param x initial state
     * @param A coefficient matrix of x for process model
     * @param B coefficient matrix of u for process model
     * @param C coefficient matrix of x for measurement model
     * @param Q covariance matrix for process model
     * @param R covariance matrix for measurement model
     * @param P initial covariance of estimated state
     */
    KalmanFilter(const Eigen::MatrixXd& x, const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& C, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R, const Eigen::MatrixXd& P) { init(x, A, B, C, Q, R, P); }

    /**
     * @brief destructor
     */
    ~KalmanFilter() {}

    /**
     * @brief initialization of kalman filter
     * @param x initial state
     * @param A coefficient matrix of x for process model
     * @param B coefficient matrix of u for process model
     * @param C coefficient matrix of x for measurement model
     * @param Q covariance matrix for process model
     * @param R covariance matrix for measurement model
     * @param P initial covariance of estimated state
     */
    bool init(const Eigen::MatrixXd& x, const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& C, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R, const Eigen::MatrixXd& P)
    {
        if (x.cols() == 0 || x.rows() == 0 || A.cols() == 0 || A.rows() == 0 || B.cols() == 0 || B.rows() == 0 || C.cols() == 0 || C.rows() == 0 || Q.cols() == 0 || Q.rows() == 0 || R.cols() == 0 || R.rows() == 0 || P.cols() == 0 || P.rows() == 0) {
            return false;
        }
        x_ = x;
        A_ = A;
        B_ = B;
        C_ = C;
        Q_ = Q;
        R_ = R;
        P_ = P;
        return true;
    }

    /**
     * @brief initialization of kalman filter
     * @param x initial state
     * @param P initial covariance of estimated state
     */
    bool init(const Eigen::MatrixXd& x, const Eigen::MatrixXd& P0)
    {
        if (x.cols() == 0 || x.rows() == 0 || P0.cols() == 0 || P0.rows() == 0) {
            return false;
        }
        x_ = x;
        P_ = P0;
        return true;
    }

    /**
     * @brief set A of process model
     * @param A coefficient matrix of x for process model
     */
    void setA(const Eigen::MatrixXd& A) { A_ = A; }

    /**
     * @brief set B of process model
     * @param B coefficient matrix of u for process model
     */
    void setB(const Eigen::MatrixXd& B) { B_ = B; }

    /**
     * @brief set C of measurement model
     * @param C coefficient matrix of x for measurement model
     */
    void setC(const Eigen::MatrixXd& C) { C_ = C; }

    /**
     * @brief set covariance matrix Q for process model
     * @param Q covariance matrix for process model
     */
    void setQ(const Eigen::MatrixXd& Q) { Q_ = Q; }

    /**
     * @brief set covariance matrix R for measurement model
     * @param R covariance matrix for measurement model
     */
    void setR(const Eigen::MatrixXd& R) { R_ = R; }

    /**
     * @brief get current kalman filter state
     * @param x kalman filter state
     */
    void getX(Eigen::MatrixXd& x) const { x = x_; }

    /**
     * @brief get current kalman filter covariance
     * @param P kalman filter covariance
     */
    void getP(Eigen::MatrixXd& P) const { P = P_; }

    /**
     * @brief get component of current kalman filter state
     * @param i index of kalman filter state
     * @return value of i's component of the kalman filter state x[i]
     */
    double getXelement(unsigned int i) const { return x_(i); }

    /**
     * @brief calculate kalman filter state and covariance by prediction model with A, B, Q matrix.
     * This is mainly for EKF with variable matrix.
     * @param u input for model
     * @param A coefficient matrix of x for process model
     * @param B coefficient matrix of u for process model
     * @param Q covariance matrix for process model
     * @return bool to check matrix operations are being performed properly
     */
    bool predict(const Eigen::MatrixXd& u, const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& Q)
    {
        if (A.cols() != x_.rows() || B.cols() != u.rows()) {
            return false;
        }
        const Eigen::MatrixXd x_next = A * x_ + B * u;
        return predict(x_next, A, Q);
    }

    /**
     * @brief calculate kalman filter covariance with prediction model with x, A, Q matrix. This is
     * mainly for EKF with variable matrix.
     * @param x_next predicted state
     * @param A coefficient matrix of x for process model
     * @param Q covariance matrix for process model
     * @return bool to check matrix operations are being performed properly
     */
    bool predict(const Eigen::MatrixXd& x_next, const Eigen::MatrixXd& A, const Eigen::MatrixXd& Q)
    {
        if (x_.rows() != x_next.rows() || A.cols() != P_.rows() || Q.cols() != Q.rows() || A.rows() != Q.cols()) {
            return false;
        }
        x_ = x_next;
        P_ = A * P_ * A.transpose() + Q;
        return true;
    }

    /**
     * @brief calculate kalman filter covariance with prediction model with x, A, Q matrix. This is
     * mainly for EKF with variable matrix.
     * @param x_next predicted state
     * @param A coefficient matrix of x for process model
     * @return bool to check matrix operations are being performed properly
     */
    bool predict(const Eigen::MatrixXd& x_next, const Eigen::MatrixXd& A) { return predict(x_next, A, Q_); }

    /**
     * @brief calculate kalman filter state by prediction model with A, B and Q being class member
     * variables.
     * @param u input for the model
     * @return bool to check matrix operations are being performed properly
     */
    bool predict(const Eigen::MatrixXd& u) { return predict(u, A_, B_, Q_); }

    /**
     * @brief calculate kalman filter state by measurement model with y_pred, C and R matrix. This is
     * mainly for EKF with variable matrix.
     * @param y measured values
     * @param y output values expected from measurement model
     * @param C coefficient matrix of x for measurement model
     * @param R covariance matrix for measurement model
     * @return bool to check matrix operations are being performed properly
     */
    bool update(const Eigen::MatrixXd& y, const Eigen::MatrixXd& y_pred, const Eigen::MatrixXd& C, const Eigen::MatrixXd& R)
    {
        if (P_.cols() != C.cols() || R.rows() != R.cols() || R.rows() != C.rows() || y.rows() != y_pred.rows() || y.rows() != C.rows()) {
            return false;
        }
        const Eigen::MatrixXd PCT = P_ * C.transpose();
        const Eigen::MatrixXd K = PCT * ((R + C * PCT).inverse());

        if (isnan(K.array()).any() || isinf(K.array()).any()) {
            return false;
        }

        x_ = x_ + K * (y - y_pred);
        P_ = P_ - K * (C * P_);
        return true;
    }

    /**
     * @brief calculate kalman filter state by measurement model with C and R matrix. This is mainly
     * for EKF with variable matrix.
     * @param y measured values
     * @param C coefficient matrix of x for measurement model
     * @param R covariance matrix for measurement model
     * @return bool to check matrix operations are being performed properly
     */
    bool update(const Eigen::MatrixXd& y, const Eigen::MatrixXd& C, const Eigen::MatrixXd& R)
    {
        if (C.cols() != x_.rows()) {
            return false;
        }
        const Eigen::MatrixXd y_pred = C * x_;
        return update(y, y_pred, C, R);
    }

    /**
     * @brief calculate kalman filter state by measurement model with C and R being class member
     * variables.
     * @param y measured values
     * @return bool to check matrix operations are being performed properly
     */
    bool update(const Eigen::MatrixXd& y) { return update(y, C_, R_); }

protected:
    Eigen::MatrixXd x_; //!< @brief current estimated state
    Eigen::MatrixXd A_; //!< @brief coefficient matrix of x for process model x[k+1] = A*x[k] + B*u[k]
    Eigen::MatrixXd B_; //!< @brief coefficient matrix of u for process model x[k+1] = A*x[k] + B*u[k]
    Eigen::MatrixXd C_; //!< @brief coefficient matrix of x for measurement model y[k] = C * x[k]
    Eigen::MatrixXd Q_; //!< @brief covariance matrix for process model x[k+1] = A*x[k] + B*u[k]
    Eigen::MatrixXd R_; //!< @brief covariance matrix for measurement model y[k] = C * x[k]
    Eigen::MatrixXd P_; //!< @brief covariance of estimated state
};

} // namespace tlab