#ifndef MATRIX_H
#define MATRIX_H

#include <vector>
#include <utility>

namespace ch {

using std::vector;

class Matrix {
public:
	Matrix(int n, int m): mat(n, vector<double>(m, 0)) {}
	Matrix(const vector<vector<double> > &h_mat): mat(h_mat) {}
	Matrix(int n): mat(n, vector<double>(n, 0)) {}
    Matrix(const Matrix &h_mat): mat(h_mat.mat) {}
    Matrix(Matrix &&h_mat): mat(std::move(h_mat.mat)) {};
	~Matrix() {}

	int GetNumRows() const { return mat.size(); }
	int GetNumCols() const { if(GetNumRows()) return mat[0].size(); }
    bool IsSquare() const { return GetNumRows() == GetNumCols(); }

    Matrix operator- () const;
	Matrix operator+ (const Matrix &mat_2) const;
	Matrix operator- (const Matrix &mat_2) const;
    Matrix operator* (const Matrix &mat_2) const;
	Matrix& operator+= (const Matrix &mat_2);
	Matrix& operator-= (const Matrix &mat_2);
    Matrix& operator*= (const Matrix &mat_2);
    vector<double>& operator[] (int x);
    vector<double> operator[] (int x) const;
    Matrix& MakeEye();
    Matrix& Transpose();
    Matrix GetTranspose() const;

private:
	vector<vector<double> > mat;

friend class HTMatrix;
};

class HTMatrix: public Matrix {
public: // more checks needed
	HTMatrix(const vector<vector<double> > &h_mat): Matrix(h_mat) {
		if(h_mat.size() != 4 || h_mat[0].size() != 4) throw;  			// no catching
	}
    HTMatrix(const Matrix &h_mat): Matrix(h_mat) {
		if(h_mat.GetNumCols() != 4 || h_mat.GetNumRows() != 4) throw;	// no catching more
    }

	HTMatrix& Inverse();
	HTMatrix GetInverse() const;
    Matrix GetRot() const;
    vector<double> GetPosVec() const;
    Matrix GetPosMat() const;
};


}

#endif  //MATRIX_H
