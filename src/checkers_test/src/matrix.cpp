#include "matrix.h"

namespace ch {
// TODO test this all
Matrix Matrix::operator- () const {
    Matrix ret(*this);
    for(int i = 0; i < mat.size(); i++)
        for(int j = 0; j < mat[0].size(); j++)
            ret.mat[i][j] = -ret.mat[i][j];
    return ret;
}

Matrix Matrix::operator+ (const Matrix &mat_2) const {
    if(this->GetNumRows() != mat_2.GetNumRows() ||
       this->GetNumCols() != mat_2.GetNumCols()) throw;  // no catching
    Matrix ret(*this);
    for(int i = 0; i < mat.size(); i++)
        for(int j = 0; j < mat[0].size(); j++)
            ret.mat[i][j] += mat_2.mat[i][j];
    return ret;
}

Matrix Matrix::operator- (const Matrix &mat_2) const {
    if(this->GetNumRows() != mat_2.GetNumRows() ||
       this->GetNumCols() != mat_2.GetNumCols()) throw;  // no catching
    Matrix ret(*this);
    for(int i = 0; i < mat.size(); i++)
        for(int j = 0; j < mat[0].size(); j++)
            ret.mat[i][j] -= mat_2.mat[i][j];
    return ret;
}

Matrix Matrix::operator* (const Matrix &mat_2) const {
    if(this->GetNumCols() != mat_2.GetNumRows()) throw;  // no catching
    Matrix ret(this->GetNumRows(), mat_2.GetNumCols());
    for(int i = 0; i < mat.size(); i++)
        for(int j = 0; j < mat_2.mat[0].size(); j++)
            for(int k = 0; k < mat[0].size(); k++)
                ret.mat[i][j] += this->mat[i][k] * mat_2.mat[k][j];
    return ret;
}

Matrix& Matrix::operator+= (const Matrix &mat_2) {
    if(this->GetNumRows() != mat_2.GetNumRows() ||
       this->GetNumCols() != mat_2.GetNumCols()) throw;  // no catching
    for(int i = 0; i < mat.size(); i++)
        for(int j = 0; j < mat[0].size(); j++)
            mat[i][j] += mat_2.mat[i][j];
    return *this;
}

Matrix& Matrix::operator-= (const Matrix &mat_2) {
    if(this->GetNumRows() != mat_2.GetNumRows() ||
       this->GetNumCols() != mat_2.GetNumCols()) throw;  // no catching
    for(int i = 0; i < mat.size(); i++)
        for(int j = 0; j < mat[0].size(); j++)
            mat[i][j] -= mat_2.mat[i][j];
    return *this;
}


Matrix& Matrix::operator*= (const Matrix &mat_2) {
    if(this->GetNumCols() != mat_2.GetNumRows()) throw;  // no catching
    Matrix ret(this->GetNumRows(), mat_2.GetNumCols());
    for(int i = 0; i < mat.size(); i++)
        for(int j = 0; j < mat_2.mat[0].size(); j++)
            for(int k = 0; k < mat[0].size(); k++)
                ret.mat[i][j] += this->mat[i][k] * mat_2.mat[k][j];
    this->mat = std::move(ret.mat);
    return *this;
}

Matrix& Matrix::MakeEye() {
    for(int i = 0; i < mat.size(); i++)
        for(int j = 0; j < mat[0].size(); j++)
            mat[i][j] = (i == j) ? 1 : 0;             // number format
    return *this;
}

Matrix& Matrix::Transpose() {
    if(this->IsSquare()) {
        for(int i = 0; i < mat.size()/2; i++)
            for(int j = 0; j < mat[0].size()/2; j++)
                mat[i][j] = mat[j][i];
        return *this;
    }
    Matrix ret(this->GetNumCols(), this->GetNumRows());
    for(int i = 0; i <= mat.size()/2; i++)
        for(int j = 0; j <= mat[0].size()/2; j++)
            ret.mat[j][i] = mat[i][j];
    mat = std::move(ret.mat);
    return *this;
}

Matrix Matrix::GetTranspose() const {
    Matrix ret(mat);
    ret.Transpose();
    return ret;
}


vector<double> Matrix::operator[] (int x) const {
    return mat[x];
}

vector<double>& Matrix::operator[] (int x) {
    return mat[x];
}

HTMatrix& HTMatrix::Inverse() {
    HTMatrix ret(*this);
    Matrix rot(GetRot());
    rot.Transpose();
    Matrix z(-rot * GetPosMat());
    for(int i = 0; i < 3; i++)
        for(int j = 0; j < 3; j++)
            ret.mat[i][j] = rot.mat[i][j];
    for(int i = 0; i < 3; i++) ret.mat[i][3] = z.mat[i][0];
    ret.mat[3][0] = ret.mat[3][1] = ret.mat[3][2] = 0;
    ret.mat[3][3] = 1;
    mat = std::move(ret.mat);
    return *this;
}

Matrix HTMatrix::GetRot() const {
    Matrix rot(3);
    for(int i = 0; i < 3; i++)
        for(int j = 0; j < 3; j++)
            rot.mat[i][j] = Matrix::mat[i][j];
    return rot;
}

Matrix HTMatrix::GetPosMat() const {
    Matrix pos(3, 1);
    for(int i = 0; i < 3; i++)
        pos.mat[i][0] = Matrix::mat[i][3];
    return pos;
}

vector<double> HTMatrix::GetPosVec() const {
    vector<double> pos(3);
    for(int i = 0; i < 3; i++)
        pos[i] = Matrix::mat[i][3];
    return pos;
}

}
