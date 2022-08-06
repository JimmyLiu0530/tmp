/*
 * matrix.h
 *
 *  Created on: Jun 3, 2020
 *      Author: erik
 */

#ifndef SCRATCH_FUNCTION_MATRIX_H_
#define SCRATCH_FUNCTION_MATRIX_H_

#include <ctype.h>
#include <iostream>
#include <fstream>
#include <string>
#include <cassert>
#include <algorithm>
#include <map>

using namespace std;

//***for map***
//UE to AP
/*map<uint16_t, map<uint16_t, double>> PseudoInverseMatrix (map<uint16_t, map<uint16_t, double>> &matrix);
map<uint16_t, map<uint16_t, double>> Transpose(map<uint16_t, map<uint16_t, double>> &matrix);
map<uint16_t, map<uint16_t, double>> Dot(map<uint16_t, map<uint16_t, double>> &a, map<uint16_t, map<uint16_t, double>> &b);
map<uint16_t, map<uint16_t, double>> Inverse(map<uint16_t, map<uint16_t, double>> &matrix);
double determinant(map<uint16_t, map<uint16_t, double>> &matrix, uint16_t n);
map<uint16_t, map<uint16_t, double>> Adjoint(map<uint16_t, map<uint16_t, double>> &matrix);
void getCofactor(map<uint16_t, map<uint16_t, double>> &matrix, map<uint16_t, map<uint16_t, double>> &cofactor,
		uint16_t p, uint16_t q, uint16_t n);*/
//for testing
void PrintOutMatrix(map<uint16_t, map<uint16_t, double>> &matrix);
void PrintOutMatrix(vector<vector<double>> &matrix);

//***for vector***
vector<vector<double>> PseudoInverseMatrix (vector<vector<double>> &matrix);
vector<vector<double>> Transpose(vector<vector<double>> &matrix);
vector<vector<double>> Dot(vector<vector<double>> &a, vector<vector<double>> &b);
vector<vector<double>> Inverse(vector<vector<double>> &matrix);
double determinant(vector<vector<double>> &matrix, uint16_t n);
vector<vector<double>> Adjoint(vector<vector<double>> &matrix);
void getCofactor(vector<vector<double>> &matrix, vector<vector<double>> &cofactor,
		uint16_t p, uint16_t q, uint16_t n);



#endif /* SCRATCH_FUNCTION_MATRIX_H_ */
