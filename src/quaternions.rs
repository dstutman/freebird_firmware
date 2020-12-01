use core::ops::{Add, Mul, Sub};
use libm::{powf, sqrtf};

#[derive(Debug, Copy, Clone)]
pub struct Quaternion {
    pub w: f32,
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Quaternion {
    pub(crate) fn new(w: f32, x: f32, y: f32, z: f32) -> Self {
        return Quaternion { w, x, y, z };
    }

    pub fn norm(&self) -> f32 {
        return sqrtf(powf(self.w, 2.) + powf(self.x, 2.) + powf(self.y, 2.) + powf(self.z, 2.));
    }

    pub fn conj(&self) -> Self {
        return Quaternion {
            w: self.w,
            x: -self.x,
            y: -self.y,
            z: -self.z,
        };
    }

    pub fn normalized(&self) -> Self {
        let norm = self.norm();
        return Quaternion {
            w: self.w / norm,
            x: self.x / norm,
            y: self.y / norm,
            z: self.z / norm,
        };
    }

    pub fn scale(&self, factor: f32) -> Self {
        return Quaternion {
            w: self.w * factor,
            x: self.x * factor,
            y: self.y * factor,
            z: self.z * factor,
        };
    }
}

impl Mul for Quaternion {
    type Output = Self;

    fn mul(self, rhs: Self) -> Self {
        return Quaternion {
            w: self.w * rhs.w - self.x * rhs.x - self.y * rhs.y - self.z * rhs.z,
            x: self.w * rhs.x + self.x * rhs.w + self.y * rhs.z - self.z * rhs.y,
            y: self.w * rhs.y - self.x * rhs.z + self.y * rhs.w + self.z * rhs.x,
            z: self.w * rhs.z + self.x * rhs.y - self.y * rhs.x + self.z * rhs.w,
        };
    }
}

impl Add for Quaternion {
    type Output = Self;

    fn add(self, rhs: Self) -> Self {
        return Quaternion {
            w: self.w + rhs.w,
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            z: self.z + rhs.z,
        };
    }
}

impl Sub for Quaternion {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self {
        return Quaternion {
            w: self.w - rhs.w,
            x: self.x - rhs.x,
            y: self.y - rhs.y,
            z: self.z - rhs.z,
        };
    }
}
