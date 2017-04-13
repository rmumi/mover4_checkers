#ifndef MATRIX_H
#define MATRIX_H

#include <vector>
#include <utility>

namespace checkers {

using std::vector;
template<typename T>  // numerical types
class Matrix {
public:
	Matrix(int n, int m): mat(n, vector<T>(m, 0)) {}
	Matrix(const vector<vector<T> > &h_mat): mat(h_mat) {}
	Matrix(int n): mat(n, vector<T>(n, 0)) {}
    Matrix(const Matrix<T> &h_mat): mat(h_mat.mat) {}
    Matrix(Matrix<T> &&h_mat): mat(std::move(h_mat.mat)) {};
	~Matrix() {}

	int GetNumRows() const { return mat.size(); }
	int GetNumCols() const { if(GetNumRows()) return mat[0].size(); }
    bool IsSquare() const { return GetNumRows() == GetNumCols(); }

	// T det() const;
    int WTF();
    Matrix<T> operator- () const;
	Matrix<T> operator+ (const Matrix<T> &mat_2) const;
	Matrix<T> operator- (const Matrix<T> &mat_2) const;
    Matrix<T> operator* (const Matrix<T> &mat_2) const;
	Matrix<T>& operator+= (const Matrix<T> &mat_2);
	Matrix<T>& operator-= (const Matrix<T> &mat_2);
    Matrix<T>& operator*= (const Matrix<T> &mat_2);
    vector<T>& operator[] (int x);
    vector<T> operator[] (int x) const;
    Matrix<T>& MakeEye();
    Matrix<T>& Transpose();
    Matrix<T> GetTranspose() const;

private:
	vector<vector<T> > mat;
};

template<typename T>
class HTMatrix: public Matrix<T> {
public: // more checks needed
	HTMatrix(const vector<vector<double> > &h_mat): Matrix<T>(h_mat) {
		if(h_mat.size() != 4 || h_mat[0].size() != 4) throw;  			// no catching
	}
    HTMatrix(const Matrix<T> &h_mat): Matrix<T>(h_mat) {
		if(h_mat.GetNumCols() != 4 || h_mat.GetNumRows() != 4) throw;	// no catching more
    }

	HTMatrix<T>& Inverse();
	HTMatrix<T> GetInverse() const;
    Matrix<T> GetRot() const;
    vector<T> GetPosVec() const;
    Matrix<T> GetPosMat() const;
};


}

#endif  //MATRIX_H
