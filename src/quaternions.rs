use core::ops::{Mul, Add, Sub};
use libm::{powf, sqrtf};

#[derive(Debug, Copy, Clone)]
pub struct Quaternion {
    pub a: f32,
    pub b: f32,
    pub c: f32,
    pub d: f32
}

impl Quaternion {
    pub(crate) fn new(a: f32, b: f32, c: f32, d: f32) -> Self {
        return Quaternion{a, b, c, d};
    }

    pub fn norm(&self) -> f32 {
        return sqrtf(powf(self.a, 2.) + powf(self.b, 2.) + powf(self.c, 2.) + powf(self.d, 2.));
    }

    pub fn conj(&self) -> Self {
        return Quaternion {
            a: self.a,
            b: -self.b,
            c: -self.c,
            d: -self.d
        }
    }

    pub fn normalized(&self) -> Self {
        let norm =self.norm();
        return Quaternion {
            a: self.a / norm,
            b: self.b / norm,
            c: self.c / norm,
            d: self.d / norm
        }
    }

    pub fn scale(&self, factor: f32) -> Self {
        return Quaternion {
            a: self.a * factor,
            b: self.b * factor,
            c: self.c * factor,
            d: self.d * factor
        }
    }
}

impl Mul for Quaternion {
    type Output = Self;

    fn mul(self, rhs: Self) -> Self {
        return Quaternion {
            a: self.a * rhs.a - self.b * rhs.b - self.c * rhs.c - self.d * rhs.d,
            b: self.a * rhs.b + self.b * rhs.a + self.c * rhs.d - self.d * rhs.c,
            c: self.a * rhs.c - self.b * rhs.d + self.c * rhs.a + self.d * rhs.b,
            d: self.a * rhs.d + self.b * rhs.c - self.c * rhs.b + self.d * rhs.a
        }
    }
}

impl Add for Quaternion {
    type Output = Self;

    fn add(self, rhs: Self) -> Self {
        return Quaternion {
            a: self.a + rhs.a,
            b: self.b + rhs.b,
            c: self.c + rhs.c,
            d: self.d + rhs.d
        }
    }
}

impl Sub for Quaternion {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self {
        return Quaternion {
            a: self.a - rhs.a,
            b: self.b - rhs.b,
            c: self.c - rhs.c,
            d: self.d - rhs.d
        }
    }
}