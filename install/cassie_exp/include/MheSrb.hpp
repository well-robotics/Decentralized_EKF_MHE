#ifndef MHE_SRB_HPP
#define MHE_SRB_HPP

#include <osqp/osqp.h>
#include <OsqpEigen/OsqpEigen.h>
#include <Eigen/Sparse>
#include <chrono>
#include "../include/EigenUtils.hpp"

using namespace Eigen;

namespace MHEutils
{

    struct CostItem
    {
        // Adding cost term 0.5(Ax - b)' Q (Ax - b)
        std::string CostName;
        std::map<std::string, SparseMatrix<double>> depVarMap;
        void addDependancy(std::string VarName, SparseMatrix<double> A)
        {
            depVarMap.insert({VarName, A});
        };
        VectorXd b;
        SparseMatrix<double> Q;
    };

    struct LinearConstraint
    {
        // lb < A x < ub
        std::string CostraintName;
        std::map<std::string, SparseMatrix<double>> depVarMap;
        void addDependancy(std::string VarName, SparseMatrix<double> A)
        {
            depVarMap.insert({VarName, A});
        };
        VectorXd lb, ub;
        bool equality = false;
        void updateBound(VectorXd lb_update, VectorXd ub_update, bool equality_update)
        {
            lb = lb_update;
            ub = ub_update;
            equality = equality_update;
        };
    };

    struct TimeStep
    {
        int Time;
        int Var_Size;
        int Constraint_Size;
        std::map<std::string, int> TimeVarMap;
        std::vector<std::string> TimeCost_stack;
        std::vector<std::string> TimeConstraint_stack;
    };
}

class MHEproblem
{

public:
    MHEproblem();

    int N;
    void setHorizon(int N, int dim_state, int dim_meas); // should be update accordingly based on current time

    void addVariable(std::string VarName_, int VarSize_); // name and inital guess
    void addCost(std::string CostName_, VectorXd b, SparseMatrix<double> Q);
    void addConstraints(std::string ConstraintName_, VectorXd lb, VectorXd ub);

    void addConstraintDependency(std::string ConstraintName_, std::string VarName, SparseMatrix<double> A);
    void updateConstraintBound(std::string ConstraintName_, VectorXd lb, VectorXd ub, bool equality_update);
    void updateCostGain(std::string CostName_, double scale_gain);
    void addCostDependency(std::string CostName_, std::string VarName, SparseMatrix<double> A);
    void getsolution(int T);

    void addTimeStep(int Time, int Var_Size, int Constraint_Size);

    OsqpEigen::Solver osqp;

    double osqp_infinity = OsqpEigen::INFTY;

    VectorXd solution;
    VectorXd x_solution_now;
    VectorXd w_solution_now;
    VectorXd v_solution_now;

    int var_size;

    bool formulateQP();

    bool updateQP(int T);
    void solveQP();
    bool initQP(int T);
    bool marginalizeQP(int T);
    void resetQP();

    void tic(std::string str, int mode = 0);
    void toc(std::string str);
    void vectorResize(VectorXd &vector, const int new_size);

    SparseMatrix<double> M_p;
    VectorXd n_p = VectorXd::Zero(21);

    // Sparse Matrixd inverse solver
    typedef Eigen::SparseMatrix<double> SparseMatrixType;
    typedef Eigen::SimplicialLLT<SparseMatrixType> SolverType;
    SolverType solver;
    SolverType solver_meas;
    SolverType solver_dyn;
    SolverType solver_corelate;

    void Update_Image_bound(std::map<int, Vector3d> vo_constraints_idx_regs,std::map<int, double> vo_reliability_idx_regs);

private:
    int nVar = 0;

    int nConstraints = 0;
    int nConstraints_new = 0;
    int nCost = 0;
    int nCost_new = 0;

    int nVarStart = 0;
    int nVarEnd = 0;

    void assembleConstraints();
    void assembleCost();

    std::map<int, MHEutils::TimeStep> Time_regs;

    std::map<std::string, int> var_idx_regs; // variable name and idx
    std::map<std::string, MHEutils::CostItem> Cost_regs;
    std::map<std::string, MHEutils::CostItem> Cost_new_regs;

    std::map<std::string, MHEutils::LinearConstraint> LinearConstraint_regs;
    std::map<std::string, MHEutils::LinearConstraint> LinearConstraint_new_regs;
    std::vector<std::string> LinearConstraint_new_name_vec;

    // used to store the assembled constraints and cost

    std::vector<SparseMatrix<double>> A_constraint_Matrices;
    std::vector<VectorXd> lb_vectors;
    std::vector<VectorXd> ub_vectors;

    // Temp for assemble
    std::vector<SparseMatrix<double>> A_cost_Matrices;
    std::vector<SparseMatrix<double>> Q_cost_Matrices;
    std::vector<VectorXd> b_cost_Vectors;

    // used to formulate optimization
    // constraints
    SparseMatrix<double> Aconstrsparse;
    VectorXd lb_all, ub_all;
    // costs
    SparseMatrix<double> A_cost_all;
    SparseMatrix<double> Q_cost_all;
    VectorXd b_cost_all;

    SparseMatrix<double> A_cost_new;
    SparseMatrix<double> Q_cost_new;
    VectorXd b_cost_new;

    SparseMatrix<double> Hsparse;
    VectorXd g = VectorXd::Zero(12);

    SparseMatrix<double> Hsparse_new;
    VectorXd g_new;

    SparseMatrix<double> Aconstrsparse_new;
    VectorXd lb_all_new;
    VectorXd ub_all_new;

    VectorXd x0; // initial guess

    // sparse inv solver definition
    SparseMatrix<double> I;
    SparseMatrix<double> I_meas;
    SparseMatrix<double> I_dyn;
    SparseMatrix<double> I_corelate;
    SparseMatrix<double> I_mag;

    int dim_state;
    int dim_meas;
    int dim_cam;

    // SparseLU<SparseMatrix<double>> solver_inv;

    void Log2txt(const MatrixXd matrix, std::string filename);

    void schurComplement(SparseMatrix<double> &S_out, VectorXd &v_out, const SparseMatrix<double> &A, const SparseMatrix<double> &B, const SparseMatrix<double> &C, const SparseMatrix<double> &D, const VectorXd &u, const VectorXd &v);

    // Marginalization Matrix predefine
    // ------------------------------------
    // MatrixXd Q_meas_marginalize = MatrixXd::Zero(12, 12);
    // MatrixXd H_meas_marginalize = MatrixXd::Zero(12, 21);
    // VectorXd y_meas_marginalize = VectorXd::Zero(12);

    // MatrixXd Q_contact_marginalize = MatrixXd::Zero(12, 12);
    // MatrixXd H_contact_marginalize = MatrixXd::Zero(12, 21);
    // VectorXd y_contact_marginalize = VectorXd::Zero(12);

    SparseMatrix<double> M_p_next;
    VectorXd n_p_next = VectorXd::Zero(12);
};

#endif // MHE_SRB_HPP