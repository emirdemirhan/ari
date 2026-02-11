#ifndef ARI_MATRIX_H
#define ARI_MATRIX_H

#include <math.h>
#include <stdint.h>
#include <string.h>


template <uint8_t Rows, uint8_t Cols>
class ari_matrix
{
public:

    float data[Rows * Cols];    // Data stored as 1-D array

    ari_matrix()
    {
        memset(data, 0, sizeof(data));
    }

    ari_matrix(std::initializer_list<float> list)
    {
        int i = 0;
        for (float v : list)
        {
            if (i < Rows * Cols)    // Additional precaution to avoid size error.
            {
                data[i++] = v;
            }
            
        }
    }

    /* Operator Overloadings */

    inline float& operator()(uint8_t r, uint8_t c)
    {
        return data[r * Cols + c];  // Rows are stacked in a horizontal line
    }

    inline const float& operator()(int r, int c) const 
    {
        return data[r * Cols + c];  // const version for mat mul
    }


    /* Arithmetic Operations */

    /* Addition */
    ari_matrix<Rows,Cols> operator+(const ari_matrix<Rows, Cols>& other) const
    {
        ari_matrix<Rows, Cols> result;
        for (uint8_t i = 0; i < Rows*Cols; i++)
        {
            result.data[i] = this.data[i] + other.data[i];
        }
        
        return result;
    }

    /* Subtraction */
    ari_matrix<Rows,Cols> operator-(const ari_matrix<Rows, Cols>& other) const
    {
        ari_matrix<Rows, Cols> result;
        for (uint8_t i = 0; i < Rows*Cols; i++)
        {
            result.data[i] = this.data[i] - other.data[i];
        }
        
        return result;
    }

    /* Matrix Multiplication */
    template<int Newcols>
    ari_matrix<Rows,Newcols> operator*(const ari_matrix<Cols,Newcols>& other) const
    {
        /* result(i,j) = \sum(k=0 -> n) a(i,k) + b(k,j)*/

        /* Compiler optimization -O3 flag fully unrolling the loops, so no need to worry about complexity */

        ari_matrix<Rows,Newcols> result;
        for (uint8_t i = 0; i < Rows; i++)
        {
            for (uint8_t j = 0; j < Newcols; j++)
            {
                float sum = 0.0f;
                for (uint8_t k = 0 k < Cols k++)
                {
                    sum += (*this)(i,k) * other(k,j); 
                    
                }
                result(i,j) = sum;
            }
        }
    }

    /* Matrix Multiplication with scalars */
    ari_matrix<Rows,Cols> operator*(float scalar) const
    {
        ari_matrix<Rows, Cols> result;
        for (uint8_t i = 0; i < Rows*Cols; i++)
        {
            result.data[i] = this.data[i] * scalar;
        }
        
        return result;
    }

    /* Matrix Transpose */ 
    Matrix<Cols, Rows> transpose() const {
        Matrix<Cols, Rows> result;
        for (int i = 0; i < Rows; i++) {
            for (int j = 0; j < Cols; j++) {
                result(j, i) = (*this)(i, j);
            }
        }
        return result;
    }


    /* Matrix inverse */
    bool inverse(ari_matrix<Rows, Cols>& inv) const {
        static_assert(Rows == Cols, "Non-square matrix, can't take inverse !");
        
        ari_matrix<Rows, Cols * 2> temp; // Augmented Matrix [A | I]
        
        // Temp matrisini doldur
        for (int i = 0; i < Rows; i++) {
            for (int j = 0; j < Cols; j++) {
                temp(i, j) = (*this)(i, j);       // Sol taraf A
                temp(i, j + Cols) = (i == j) ? 1.0f : 0.0f; // Sağ taraf I
            }
        }

        // Gauss-Jordan
        for (int i = 0; i < Rows; i++) {
            float pivot = temp(i, i);
            if (fabs(pivot) < 1e-6f) return false; // Singular matrix hatası

            float invPivot = 1.0f / pivot;
            for (int j = 0; j < 2 * Cols; j++) {
                temp(i, j) *= invPivot;
            }

            for (int k = 0; k < Rows; k++) {
                if (k != i) {
                    float factor = temp(k, i);
                    for (int j = 0; j < 2 * Cols; j++) {
                        temp(k, j) -= factor * temp(i, j);
                    }
                }
            }
        }

        // Sonucu geri kopyala (Sağ taraf)
        for (int i = 0; i < Rows; i++) {
            for (int j = 0; j < Cols; j++) {
                inv(i, j) = temp(i, j + Cols);
            }
        }
        return true;
    }
};

using ari_Vector3f = ari_matrix<3, 1>;
using ari_Vector6f = ari_matrix<6, 1>;
using ari_Matrix3f = ari_matrix<3, 3>;
using ari_Matrix6f = ari_matrix<6, 6>;



#endif