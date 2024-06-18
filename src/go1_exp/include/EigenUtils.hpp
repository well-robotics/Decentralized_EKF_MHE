// EigenUtils.h
#ifndef EIGEN_UTILS_H
#define EIGEN_UTILS_H
#include <Eigen/Sparse>

namespace EigenUtils
{
    using namespace Eigen;

    inline void SparseMatrixBlockAsign(SparseMatrix<double> &Aout, int i, int j, SparseMatrix<double> Ain)
    {
        // block assignment of sparse matrix. (didn't find something like this in Eigen, implement it here temporarly)
        int idxRow, idxCol;
        for (int k = 0; k < Ain.outerSize(); ++k)
            for (SparseMatrix<double>::InnerIterator it(Ain, k); it; ++it)
            {
                idxRow = i + it.row();
                idxCol = j + it.col();
                Aout.insert(idxRow, idxCol) = it.value();
                // Aout.coeffRef(idxRow, idxCol) = it.value();
            }
    }

    inline void SparseMatrixBlockAdd(SparseMatrix<double> &Aout, int i, int j, SparseMatrix<double> Ain)
    {
        // block assignment of sparse matrix. (didn't find something like this in Eigen, implement it here temporarly)
        int idxRow, idxCol;
        for (int k = 0; k < Ain.outerSize(); ++k)
            for (SparseMatrix<double>::InnerIterator it(Ain, k); it; ++it)
            {
                idxRow = i + it.row();
                idxCol = j + it.col();
                Aout.coeffRef(idxRow, idxCol) += it.value();
            }
    }

    inline void SparseMatrixBlockAsignFromDense(SparseMatrix<double> &Aout, int i, int j, MatrixXd Ain)
    {
        // block assignment of sparse matrix. (didn't find something like this in Eigen, implement it here temporarly)
        int idxRow, idxCol;
        for (int row = 0; row < Ain.rows(); ++row)
        {
            for (int col = 0; col < Ain.cols(); ++col)
            {
                idxRow = i + row;
                idxCol = j + col;
                Aout.insert(idxRow, idxCol) = Ain(row, col);
            }
        }
    }

    inline void SparseMatrixBlockAcquisition(SparseMatrix<double> &Aout, int i, int j, SparseMatrix<double> Ain)
    {
        // // block acquisition of sparse matrix. (almost the same behavior compared with .block() operation)
        int idxRow = 0;
        int idxCol = 0;
        for (int k = j; k < Ain.outerSize(); ++k)
        {
            for (SparseMatrix<double>::InnerIterator it(Ain, k); it; ++it)
            {

                if (it.row() >= i)
                {
                    idxRow = it.row() - i;
                    idxCol = it.col() - j;
                }
                Aout.insert(idxRow, idxCol) = it.value();
                // Aout.coeffRef(idxRow, idxCol) = it.value();
            }
        }
        // std::vector<Triplet<double>> tripletList;
        // for (int k = i; k < Ain.outerSize(); ++k)
        // {
        //     for (SparseMatrix<double>::InnerIterator it(Ain, k); it; ++it)
        //     {
        //         if (it.row() >= j)
        //         {
        //             tripletList.push_back(Triplet<double>(it.row() - i, it.col() - j, it.value()));
        //         }
        //     }
        // }
        // Aout.setFromTriplets(tripletList.begin(), tripletList.end());
    }

    inline void vectorResize(VectorXd &vector, const int new_size)
    {
        int old_size = vector.size();
        vector.conservativeResize(new_size);
        vector.segment(old_size, new_size - old_size).setZero();
    }

    inline void vector3dSkew(Matrix3d &skew_sym, const Vector3d &vector)
    {

        skew_sym << 0, -vector(2), vector(1),
            vector(2), 0, -vector(0),
            -vector(1), vector(0), 0;
    }

}
#endif // EIGEN_UTILS_H