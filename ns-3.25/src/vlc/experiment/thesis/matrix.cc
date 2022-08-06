
#include "../thesis/matrix.h"

#include <ctype.h>
#include <iostream>
#include <fstream>
#include <string>
#include <cassert>
#include <algorithm>

using namespace std;
/*
map<uint16_t, map<uint16_t, double>> PseudoInverseMatrix (map<uint16_t,
		map<uint16_t, double>> &matrix) {
	map<uint16_t, map<uint16_t, double>> mat = matrix;
	map<uint16_t, map<uint16_t, double>> tmp;

	map<uint16_t, map<uint16_t, double>> transpose = Transpose(mat);
	tmp = Dot(mat, transpose);

	map<uint16_t, map<uint16_t, double>> inverse = Inverse(tmp);
	tmp = Dot(transpose, inverse);


	return tmp;
}

map<uint16_t, map<uint16_t, double>> Transpose(map<uint16_t, map<uint16_t, double>> &matrix) {

	map<uint16_t, map<uint16_t, double>> transpose_matrix;

	for(map<uint16_t, map<uint16_t, double>>::iterator row = matrix.begin();
			row != matrix.end(); ++row) {
		for(map<uint16_t, double>::iterator col = row->second.begin();
				col != row->second.end(); ++col) {
			transpose_matrix[col->first][row->first] = matrix[row->first][col->first];
		}
	}

	return transpose_matrix;
}

map<uint16_t, map<uint16_t, double>> Dot(map<uint16_t, map<uint16_t, double>> &a,
		map<uint16_t, map<uint16_t, double>> &b) {

	map<uint16_t, map<uint16_t, double>> matrix;
	map<uint16_t, map<uint16_t, double>> left = a, right = b;

	//if size not applicable, exception

	right = Transpose(right);

	for(map<uint16_t, map<uint16_t, double>>::iterator left_it = left.begin();
			left_it != left.end(); ++left_it) {
		for(map<uint16_t, map<uint16_t, double>>::iterator right_it = right.begin();
				right_it != right.end(); ++right_it) {

			uint16_t row = left_it->first, col = right_it->first;
			matrix[row][col] = 0;

			map<uint16_t, double>::iterator entry_left = left_it->second.begin();
			map<uint16_t, double>::iterator entry_right = right_it->second.begin();
			for(; entry_left != left_it->second.end() && entry_right != right_it->second.end()
						; ++entry_left, ++entry_right) {
				matrix[row][col] += entry_left->second * entry_right->second;
			}
		}
	}

	//PrintOutMatrix(matrix);


	return matrix;
}

map<uint16_t, map<uint16_t, double>> Inverse(map<uint16_t, map<uint16_t, double>> &matrix) {
	//make sure it's a square matrix
	//cout << "Inverse" << endl;
	map<uint16_t, map<uint16_t, double>> inverse;
	inverse = Adjoint(matrix);

	double D = determinant(matrix, matrix.size());

	//divided by determinant
	for(map<uint16_t, map<uint16_t, double>>::iterator it = inverse.begin(); it != inverse.end(); ++it){
		for(map<uint16_t, double>::iterator it2 = it->second.begin(); it2 != it->second.end(); ++it2) {
			if(D == 0)
				it2->second = 0;
			else
				it2->second /= D;
		}
	}

	return inverse;
}

double determinant(map<uint16_t, map<uint16_t, double>> &matrix, uint16_t n) {

	double D = 0;

	//Base case
	if(n == 1)
		return matrix.begin()->second.begin()->second;

	map<uint16_t, map<uint16_t, double>> cofactor;
	//uint32_t N = matrix.size();

	int sign = 1;

	 // Iterate for each element of first row
	//uint32_t last = matrix.size();
	map<uint16_t, map<uint16_t, double>>::iterator first_row = matrix.begin();

	for(map<uint16_t, double>::iterator col = first_row->second.begin();
			col != first_row->second.end(); ++col) {
		// Getting Cofactor of A[0][f]
		getCofactor(matrix, cofactor, first_row->first, col->first, n);
		D += sign * col->second * determinant(cofactor, n-1);

		sign = -sign;
	}

	return D;
}

map<uint16_t, map<uint16_t, double>> Adjoint(map<uint16_t, map<uint16_t, double>> &matrix) {

	map<uint16_t, map<uint16_t, double>> adj;

	uint32_t N = matrix.size();

    if (N == 1)
    {
    	map<uint16_t, map<uint16_t, double>>::iterator row = matrix.begin();
    	map<uint16_t, double>::iterator col = row->second.begin();
    	uint16_t row_id = row->first, col_id = col->first;

        adj[col_id][row_id] = 1;
        return adj;
    }

    int row_sign = 1, sign = 1;

    map<uint16_t, map<uint16_t, double>> cofactor;

    for (map<uint16_t, map<uint16_t, double>>::iterator row = matrix.begin();
    		row != matrix.end(); ++row) {
    	sign = row_sign;
        for (map<uint16_t, double>::iterator col = row->second.begin();
        		col != row->second.end(); ++col) {
        	uint16_t i = row->first, j = col->first;
            // Get cofactor of A[i][j]
            getCofactor(matrix, cofactor, i, j, N);

            // sign of adj[j][i] positive if sum of row
            // and column indexes is even.
            //sign = ((i+j)%2==0)? 1: -1;

            // Interchanging rows and columns to get the
            // transpose of the cofactor matrix
            adj[j][i] = (sign)*(determinant(cofactor, N-1));
            sign *= (-1);
        }
        row_sign *= (-1);
    }

    return adj;
}

void getCofactor(map<uint16_t, map<uint16_t, double>> &matrix, map<uint16_t, map<uint16_t, double>> &cofactor,
		uint16_t p, uint16_t q, uint16_t n) {

    // Looping for each element of the matrix
	for(map<uint16_t, map<uint16_t, double>>::iterator row = matrix.begin(); row != matrix.end(); ++row){
		for(map<uint16_t, double>::iterator col = row->second.begin(); col != row->second.end(); ++col) {
            //  Copying into temporary matrix only those element
            //  which are not in given row and column
			if(row->first != p && col->first != q) {
				cofactor[row->first][col->first] = col->second;
			}
		}
	}
}*/

vector<vector<double>> PseudoInverseMatrix (vector<vector<double>> &matrix) {
	vector<vector<double>> mat = matrix;
	vector<vector<double>> tmp;

	vector<vector<double>> transpose = Transpose(mat);
	tmp = Dot(mat, transpose);
	vector<vector<double>> inverse = Inverse(tmp);
	tmp = Dot(transpose, inverse);

	return tmp;
}

vector<vector<double>> Transpose(vector<vector<double>> &matrix) {
	uint16_t row_num = matrix.size(), col_num = matrix[0].size();

	vector<vector<double>> transpose_matrix;
	transpose_matrix.resize(col_num, vector<double>(row_num, 0));

	for(uint16_t col = 0; col < col_num; col++) {
		for(uint16_t row = 0; row < row_num; row++) {
			transpose_matrix[col][row] = matrix[row][col];
		}
	}

	return transpose_matrix;
}

vector<vector<double>> Dot(vector<vector<double>> &a, vector<vector<double>> &b) {
	vector<vector<double>> matrix;
	vector<vector<double>> left = a, right = b;
	//cout << left.size() << " " << right.size() << endl;
	matrix.resize(left.size(), vector<double>(right[0].size(), 0));
	//if size not applicable, exception
	right = Transpose(right);

	for(uint32_t i = 0; i < left.size(); i++) {
		for(uint32_t j = 0; j < right.size(); j++) {
			matrix[i][j] = 0;
			for(uint32_t k = 0; k < left[0].size(); k++) {
				matrix[i][j] += left[i][k] * right[j][k];
			}
			//std::cout << matrix[i][j] << " ";
		}
		//std::cout << std::endl;
	}

	return matrix;
}

vector<vector<double>> Inverse(vector<vector<double>> &matrix) {
	//make sure it's a square matrix
	vector<vector<double>> inverse;
	inverse = Adjoint(matrix);

	double D = determinant(matrix, matrix.size());

	//divided by determinant
	for(vector<vector<double>>::iterator it = inverse.begin(); it != inverse.end(); ++it){
		for(vector<double>::iterator it2 = it->begin(); it2 != it->end(); ++it2) {
			if(D == 0)
				*it2 = 0;
			else
				*it2 /= D;
		}
	}

	return inverse;
}

void getCofactor(vector<vector<double>> &matrix, vector<vector<double>> &cofactor,
		uint16_t p, uint16_t q, uint16_t n) {
	int i = 0, j = 0;

    // Looping for each element of the matrix
    for (int row = 0; row < n; row++)
    {
        for (int col = 0; col < n; col++)
        {
            //  Copying into temporary matrix only those element
            //  which are not in given row and column
            if (row != p && col != q)
            {
            	cofactor[i][j++] = matrix[row][col];

                // Row is filled, so increase row index and
                // reset col index
                if (j == n - 1)
                {
                    j = 0;
                    i++;
                }
            }
        }
    }
}

double determinant(vector<vector<double>> &matrix, uint16_t n) {

	double D = 0;

	//Base case
	if(n == 1)
		return matrix[0][0];

	vector<vector<double>> cofactor;
	uint32_t N = matrix.size();
    cofactor.resize(N, std::vector<double>(N, 0));

	int sign = 1;

	 // Iterate for each element of first row
	//uint32_t last = matrix.size();
	for(uint8_t i = 0; i < n; i++) {
		// Getting Cofactor of A[0][f]
		getCofactor(matrix, cofactor, 0, i, n);
		D += sign * matrix[0][i] * determinant(cofactor, n-1);

		sign = -sign;
	}

	return D;
}

vector<vector<double>> Adjoint(vector<vector<double>> &matrix) {

	vector<vector<double>> adj;

	uint32_t N = matrix.size();
	adj.resize(N, vector<double>(N, 0));

    if (N == 1)
    {
        adj[0][0] = 1;
        return adj;
    }

    int sign = 1;

    vector<vector<double>> cofactor;
    cofactor.resize(N, vector<double>(N, 0));

    for (int i=0; i<N; i++)
    {
        for (int j=0; j<N; j++)
        {
            // Get cofactor of A[i][j]
            getCofactor(matrix, cofactor, i, j, N);

            // sign of adj[j][i] positive if sum of row
            // and column indexes is even.
            sign = ((i+j)%2==0)? 1: -1;

            // Interchanging rows and columns to get the
            // transpose of the cofactor matrix
            adj[j][i] = (sign)*(determinant(cofactor, N-1));
        }
    }

    return adj;
}

void PrintOutMatrix(map<uint16_t, map<uint16_t, double>> &matrix) {
	for(map<uint16_t, map<uint16_t, double>>::iterator row = matrix.begin();
			row != matrix.end(); ++row) {
		for(map<uint16_t, double>::iterator col = row->second.begin();
				col != row->second.end(); ++col) {
			cout << col->second << " ";
		}
		cout << endl;
	}
}

void PrintOutMatrix(vector<vector<double>> &matrix) {
	double total = 0;
	for(vector<vector<double>>::iterator row = matrix.begin();
			row != matrix.end(); ++row) {
		for(vector<double>::iterator col = row->begin();
				col != row->end(); ++col) {
			cout << *col << " ";
			total += pow(*col, 2);
		}
		cout << endl;
	}
	cout << "Pre-coding cost: " << total << endl;
}

