use core::ops::{Add, Index, IndexMut, Mul};

#[derive(Debug, Copy, Clone)]
struct Matrix<T, const R: usize, const C: usize> {
    data: [[T; R]; C],
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