#include "matrix.h"

namespace checkers {
// TODO test this all
template<typename T>
Matrix<T> Matrix<T>::operator- () const {
    Matrix<T> ret(*this);
    for(int i = 0; i < mat.size(); i++)
        for(int j = 0; j < mat[0].size(); j++)
            ret.mat[i][j] = -ret.mat[i][j];
    return ret;
}

template<typename T>
Matrix<T> Matrix<T>::operator+ (const Matrix<T> &mat_2) const {
    if(this->GetNumRows() != mat_2.GetNumRows() ||
       this->GetNumCols() != mat_2.GetNumCols()) throw;  // no catching
    Matrix<T> ret(*this);
    for(int i = 0; i < mat.size(); i++)
        for(int j = 0; j < mat[0].size(); j++)
            ret.mat[i][j] += mat_2.mat[i][j];
    return ret;
}

template<typename T>
Matrix<T> Matrix<T>::operator- (const Matrix<T> &mat_2) const {
    if(this->GetNumRows() != mat_2.GetNumRows() ||
       this->GetNumCols() != mat_2.GetNumCols()) throw;  // no catching
    Matrix<T> ret(*this);
    for(int i = 0; i < mat.size(); i++)
        for(int j = 0; j < mat[0].size(); j++)
            ret.mat[i][j] -= mat_2.mat[i][j];
    return ret;
}

template<typename T>
Matrix<T> Matrix<T>::operator* (const Matrix<T> &mat_2) const {
    if(this->GetNumCols() != mat_2.GetNumRows()) throw;  // no catching
    Matrix<T> ret(this->GetNumRows(), mat_2.GetNumCols());
    for(int i = 0; i < mat.size(); i++)
        for(int j = 0; j < mat_2.mat[0].size(); j++)
            for(int k = 0; k < mat[0].size(); k++)
                ret.mat[i][j] += this->mat[i][k] * mat_2.mat[k][j];
    return ret;
}

template<typename T>
Matrix<T>& Matrix<T>::operator+= (const Matrix<T> &mat_2) {
    if(this->GetNumRows() != mat_2.GetNumRows() ||
       this->GetNumCols() != mat_2.GetNumCols()) throw;  // no catching
    for(int i = 0; i < mat.size(); i++)
        for(int j = 0; j < mat[0].size(); j++)
            mat[i][j] += mat_2.mat[i][j];
    return *this;
}

template<typename T>
Matrix<T>& Matrix<T>::operator-= (const Matrix<T> &mat_2) {
    if(this->GetNumRows() != mat_2.GetNumRows() ||
       this->GetNumCols() != mat_2.GetNumCols()) throw;  // no catching
    for(int i = 0; i < mat.size(); i++)
        for(int j = 0; j < mat[0].size(); j++)
            mat[i][j] -= mat_2.mat[i][j];
    return *this;
}

template<typename T>
Matrix<T>& Matrix<T>::operator*= (const Matrix<T> &mat_2) {
    if(this->GetNumCols() != mat_2.GetNumRows()) throw;  // no catching
    Matrix<T> ret(this->GetNumRows(), mat_2.GetNumCols());
    for(int i = 0; i < mat.size(); i++)
        for(int j = 0; j < mat_2.mat[0].size(); j++)
            for(int k = 0; k < mat[0].size(); k++)
                ret.mat[i][j] += this->mat[i][k] * mat_2.mat[k][j];
    this->mat = std::move(ret.mat);
    return *this;
}

template<typename T>
Matrix<T>& Matrix<T>::MakeEye () {
    for(int i = 0; i < mat.size(); i++)
        for(int j = 0; j < mat[0].size(); j++)
            mat[i][j] = (i == j) ? 1 : 0;             // number format
    return *this;
}

template<typename T>
Matrix<T>& Matrix<T>::Transpose () {
    if(this->IsSquare()) {
        for(int i = 0; i < mat.size()/2; i++)
            for(int j = 0; j < mat[0].size()/2; j++)
                mat[i][j] = mat[j][i];
        return *this;
    }
    Matrix<T> ret(this->GetNumCols(), this->GetNumRows());
    for(int i = 0; i <= mat.size()/2; i++)
        for(int j = 0; j <= mat[0].size()/2; j++)
            ret.mat[j][i] = mat[i][j];
    mat = std::move(ret.mat);
    return *this;
}

}