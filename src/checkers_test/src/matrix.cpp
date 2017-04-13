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
Matrix<T>& Matrix<T>::MakeEye() {
    for(int i = 0; i < mat.size(); i++)
        for(int j = 0; j < mat[0].size(); j++)
            mat[i][j] = (i == j) ? 1 : 0;             // number format
    return *this;
}

template<typename T>
Matrix<T>& Matrix<T>::Transpose() {
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

template<typename T>
Matrix<T> Matrix<T>::GetTranspose() const {
    Matrix<T> ret(mat);
    ret.Transpose();
    return ret;
}

template<typename T>
vector<T> Matrix<T>::operator[] (int x) const {
    return mat[x];
}

template<typename T>
vector<T>& Matrix<T>::operator[] (int x) {
    return mat[x];
}

template<typename T>
HTMatrix<T>& HTMatrix<T>::Inverse() {
    HTMatrix<T> ret(*this);
    Matrix<T> rot(GetRot());
    rot.Transpose();
    Matrix<T> z(-rot * GetPosMat());
    for(int i = 0; i < 3; i++)
        for(int j = 0; j < 3; j++)
            ret.mat[i][j] = rot.mat[i][j];
    for(int i = 0; i < 3; i++) ret.mat[i][3] = z.mat[i][0];
    ret.mat[3][0] = ret.mat[3][1] = ret.mat[3][2] = 0;
    ret.mat[3][3] = 1;
    return ret;
}

template<typename T>
Matrix<T> HTMatrix<T>::GetRot() const {
    Matrix<T> rot(3);
    for(int i = 0; i < 3; i++)
        for(int j = 0; j < 3; j++)
            rot.mat[i][j] = Matrix<T>::mat[i][j];
    return rot;
}

template<typename T>
Matrix<T> HTMatrix<T>::GetPosMat() const {
    Matrix<T> pos(3, 1);
    for(int i = 0; i < 3; i++)
        pos.mat[i][0] = Matrix<T>::mat[i][3];
    return pos;
}

template<typename T>
vector<T> HTMatrix<T>::GetPosVec() const {
    vector<T> pos(3);
    for(int i = 0; i < 3; i++)
        pos[i] = Matrix<T>::mat[i][3];
    return pos;
}

}
