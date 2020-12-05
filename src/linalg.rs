
use core::ops::{Add, AddAssign, Div, DivAssign, Index, IndexMut, Mul, MulAssign, Sub, SubAssign};

use libm::{powf, sqrtf};

#[derive(Debug, Copy, Clone)]
pub struct Matrix<T, const R: usize, const C: usize> {
    data: [[T; R]; C],
}

impl<T: Copy + Default, const R: usize, const C: usize> Matrix<T, R, C> {
    pub fn from_array(data: [[T; R]; C]) -> Self {
        return Matrix { data };
    }

    pub fn transpose(self) -> Matrix<T, C, R> {
        let mut mat: Matrix<T, C, R> = Default::default();

        for i in 0..R {
            for j in 0..C {
                mat[(j, i)] = self[(i, j)];
            }
        }

        return mat;
    }
}

impl<const N: usize> Matrix<f32, N, N> {
    pub fn identity() -> Self {
        let mut mat: Self = Default::default();

        for i in 0..N {
            mat[(i, i)] = 1.0;
        }

        return mat;
    }

    // Return the lower choleksy factorization
    // Does not check that the matrix meets requirements
    // for factorization.
    pub fn cholesky(self) -> Self {
        let mut mat: Self = Default::default();

        for i in 0..N {
            for j in 0..i {
                let mut tmp = self[(i, j)];
                // We are off the diagonal
                // subtract all preceeding products
                for k in 0..j {
                    tmp -= mat[(i, k)] * mat[(j, k)];
                }
                mat[(i, j)] = tmp / mat[(j, j)]
            }
            let mut tmp = self[(i, i)];
            // We are on a diagonal,
            // subtract square of each
            // preceeding Ljk in row
            for k in 0..i {
                tmp -= powf(mat[(i, k)], 2.0);
            }

            mat[(i, i)] = sqrtf(tmp)
        }

        return mat;
    }

    // This calculates the inverse of the matrix
    // using a cholesky decomposition
    pub fn invert_cholesky(self) -> Self {
        let mut mat: Self = Self::identity();

        let chol = self.cholesky();

        // First solve L*m = I
        // Foward substitution by columns
        // For each column
        for j in (0..N).rev() {
            mat[(j, j)] = 1.0 / chol[(j, j)];
            for k in (0..j).rev() {
                for i in (0..j).rev() {
                    mat[(k, j)] = mat[(k, j)] + mat[(k, i)] * chol[(i, j)];
                }
            }
            for k in (0..j).rev() {
                mat[(k, j)] = -mat[(j, j)] * mat[(k, j)];
            }
        }

        return mat;
    }
}

impl<T: Copy + Default, const R: usize, const C: usize> Default for Matrix<T, R, C> {
    fn default() -> Self {
        return Self {
            data: [[Default::default(); R]; C],
        };
    }
}

impl<T, const R: usize, const C: usize> Index<(usize, usize)> for Matrix<T, R, C> {
    type Output = T;
    fn index(&self, (r, c): (usize, usize)) -> &Self::Output {
        return &self.data[c][r];
    }
}

impl<T, const R: usize, const C: usize> IndexMut<(usize, usize)> for Matrix<T, R, C> {
    fn index_mut(&mut self, (r, c): (usize, usize)) -> &mut Self::Output {
        return &mut self.data[c][r];
    }
}

impl<T: Add<Output = T> + Copy + Default, const R: usize, const C: usize> Add for Matrix<T, R, C> {
    type Output = Self;
    fn add(self, other: Self) -> Self {
        let mut sum: Self = Default::default();
        for i in 0..R {
            for j in 0..C {
                sum[(i, j)] = self[(i, j)] + other[(i, j)];
            }
        }
        return sum;
    }
}

impl<T: Add<Output = T> + Copy + Default, const R: usize, const C: usize> AddAssign
    for Matrix<T, R, C>
{
    fn add_assign(&mut self, other: Self) {
        *self = *self + other;
    }
}

impl<T: Sub<Output = T> + Copy + Default, const R: usize, const C: usize> Sub for Matrix<T, R, C> {
    type Output = Self;
    fn sub(self, other: Self) -> Self {
        let mut diff: Self = Default::default();
        for i in 0..R {
            for j in 0..C {
                diff[(i, j)] = self[(i, j)] - other[(i, j)];
            }
        }
        return diff;
    }
}

impl<T: Sub<Output = T> + Copy + Default, const R: usize, const C: usize> SubAssign
    for Matrix<T, R, C>
{
    fn sub_assign(&mut self, other: Self) {
        *self = *self - other
    }
}

impl<T, const R: usize, const C: usize, const W: usize> Mul<Matrix<T, C, W>> for Matrix<T, R, C>
where
    T: Add<Output = T> + Mul<Output = T> + Copy + Default,
{
    type Output = Matrix<T, R, W>;
    fn mul(self, rhs: Matrix<T, C, W>) -> Self::Output {
        let mut mat: Self::Output = Default::default();

        // Any row column position in mat is composed
        // of the dot of that row in self and that column
        // in RHS

        // Get a row in mat
        for i in 0..R {
            // Get a column in mat
            for j in 0..W {
                // Get indicies into row/column
                // and compute dot product
                for k in 0..C {
                    mat[(i, j)] = mat[(i, j)] + self[(i, k)] * rhs[(k, j)];
                }
            }
        }

        return mat;
    }
}

impl<T, const R: usize, const C: usize> Mul<T> for Matrix<T, R, C>
where
    T: Add<Output = T> + Mul<Output = T> + Copy + Default,
{
    type Output = Matrix<T, R, C>;
    fn mul(self, rhs: T) -> Self::Output {
        let mut mat: Self::Output = Default::default();

        for i in 0..R {
            for j in 0..C {
                mat[(i, j)] = self[(i, j)] * rhs;
            }
        }

        return mat;
    }
}

impl<T, const R: usize, const C: usize> MulAssign<T> for Matrix<T, R, C>
where
    T: Add<Output = T> + Mul<Output = T> + Copy + Default,
{
    fn mul_assign(&mut self, rhs: T) {
        *self = *self * rhs;
    }
}

impl<T, const R: usize, const C: usize> Div<T> for Matrix<T, R, C>
where
    T: Add<Output = T> + Div<Output = T> + Copy + Default,
{
    type Output = Matrix<T, R, C>;
    fn div(self, rhs: T) -> Self::Output {
        let mut mat: Self::Output = Default::default();

        for i in 0..R {
            for j in 0..C {
                mat[(i, j)] = self[(i, j)] / rhs;
            }
        }

        return mat;
    }
}

impl<T, const R: usize, const C: usize> DivAssign<T> for Matrix<T, R, C>
where
    T: Add<Output = T> + Div<Output = T> + Copy + Default,
{
    fn div_assign(&mut self, rhs: T) {
        *self = *self / rhs;
    }
}
