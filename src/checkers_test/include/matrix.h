#ifndef MATRIX_H
#define MATRIX_H

#include <vector>

namespace checkers {

using std::vector;
template<typename T>  // numerical types
class Matrix {
public:
	Matrix(int n, int m): mat(n, vector<T>(m, *(new T()))) {}
	Matrix(const vector<vector<T> > &h_mat): mat(h_mat) {}
	Matrix(int n): mat(n, vector<T>(n, *(new T()))) {}
    Matrix(const Matrix<T> &h_mat): mat(h_mat.mat) {}
    Matrix(Matrix<T> &&h_mat): mat(std::move(h_mat.mat)) {};
	~Matrix() {}

	int GetNumRows() const { return mat.size(); }
	int GetNumCols() const { if(GetNumRows()) return mat[0].size(); }

	T det() const;

    Matrix<T> operator- () const;
	Matrix<T> operator+ (const Matrix<T> &mat_2) const;
	Matrix<T> operator- (const Matrix<T> &mat_2) const;
    Matrix<T> operator* (const Matrix<T> &mat_2) const;
	Matrix<T>& operator+= (const Matrix<T> &mat_2);
	Matrix<T>& operator-= (const Matrix<T> &mat_2);
    Matrix<T>& operator*= (const Matrix<T> &mat_2);
    Matrix<T>& MakeEye();
    Matrix<T>& Transpose();
    Matrix<T> GetTranspose() const;


private:
	vector<vector<T> > mat;
};

}

#endif  //MATRIX_H