use core::ops::{Add, Mul};

#[derive(Debug, Copy, Clone)]
struct Vector<T: Copy, const N: usize> {
    data: [T; N],
}

impl<T: Copy + Default, const N: usize> Default for Vector<T, N> {
    fn default() -> Self {
        return Self {
            data: [Default::default(); N],
        };
    }
}

impl<T: Add<Output = T> + Copy + Default, const N: usize> Add for Vector<T, N> {
    type Output = Self;
    fn add(self, other: Self) -> Self {
        let mut sum: Self = Default::default();
        for i in 0..N {
            sum.data[i] = self.data[i] + other.data[i];
        }
        return sum;
    }
}

type Matrix<T: Copy, const R: usize, const C: usize> = Vector<Vector<T, R>, C>;
