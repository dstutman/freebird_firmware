#![feature(const_generics)]
#![no_std]
#![no_main]

use core::sync::atomic::{AtomicUsize, Ordering};

use cortex_m::peripheral::syst::SystClkSource;
use cortex_m::{self, asm};
use cortex_m_rt::{entry, exception};
use futures::future::join;
use nalgebra::{Matrix6, Vector3};
use stm32f3::stm32f303;
use ukf::Matrix9;

use crate::executor::Executor;
use futures::TryFutureExt;

use core::fmt::write;
use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};

mod bmp;
mod executor;
mod i2c;
mod linalg;
mod lsm;
mod ukf;
mod usart;

static TICKS: AtomicUsize = AtomicUsize::new(0);

#[inline(always)]
fn get_ticks() -> usize {
    return TICKS.load(Ordering::Relaxed);
}

#[exception]
fn SysTick() {
    let _ = TICKS.fetch_add(1, Ordering::Relaxed);
}

#[entry]
fn main() -> ! {
    rtt_init_print!();
    let mut cp = stm32f303::CorePeripherals::take().unwrap();
    cp.SYST.set_clock_source(SystClkSource::Core);
    cp.SYST.set_reload(72000000 / 1000 - 1);
    cp.SYST.clear_current();
    cp.SYST.enable_counter();
    cp.SYST.enable_interrupt();

    let dp = stm32f303::Peripherals::take().unwrap();
    dp.FLASH.acr.write(|w| w.latency().ws2());
    while !dp.FLASH.acr.read().latency().is_ws2() {}
    dp.RCC.cr.write(|w| w.hsion().on());
    while !dp.RCC.cr.read().hsirdy().bit_is_set() {}
    dp.RCC
        .cfgr
        .write(|w| w.pllsrc().hsi_div_prediv().pllmul().mul9());
    dp.RCC.cr.write(|w| w.pllon().on());
    while !dp.RCC.cr.read().pllrdy().bit_is_set() {}
    dp.RCC.cfgr.write(|w| w.ppre1().div2());
    dp.RCC.cfgr.write(|w| w.sw().pll());
    while !dp.RCC.cfgr.read().sws().is_pll() {}

    dp.RCC.ahbenr.write(|w| w.iopben().enabled());
    let gpiob = dp.GPIOB;
    gpiob
        .moder
        .write(|w| w.moder8().alternate().moder9().alternate());
    gpiob
        .ospeedr
        .write(|w| w.ospeedr8().high_speed().ospeedr9().high_speed());
    gpiob
        .otyper
        .write(|w| w.ot8().push_pull().ot9().push_pull());
    gpiob
        .pupdr
        .write(|w| w.pupdr8().pull_up().pupdr9().pull_up());
    gpiob.afrh.write(|w| w.afrh8().af4().afrh9().af4());
    dp.RCC.apb1enr.write(|w| w.i2c1en().enabled());
    i2c::init(dp.I2C1);

    dp.RCC.ahbenr.write(|w| w.iopaen().enabled());
    let gpioa = dp.GPIOA;
    gpioa
        .moder
        .write(|w| w.moder2().alternate().moder3().alternate());
    gpioa
        .ospeedr
        .write(|w| w.ospeedr2().high_speed().ospeedr3().high_speed());
    gpioa
        .otyper
        .write(|w| w.ot2().push_pull().ot3().push_pull());
    gpioa
        .pupdr
        .write(|w| w.pupdr2().floating().pupdr3().floating());
    gpioa.afrl.write(|w| w.afrl2().af7().afrl3().af7());
    dp.RCC.apb1enr.modify(|_, w| w.usart2en().enabled());
    usart::init(dp.USART2);

    rprintln!("Starting executor...");
    let exec = Executor::new();
    exec.run(test_future());
}

// Just for debug serial
// From https://stackoverflow.com/questions/39488327/how-to-format-output-to-a-byte-array-with-no-std-and-no-allocator
use core::f32::consts::PI;
use core::fmt::{self, Write};

struct Wrapper<'a> {
    buf: &'a mut [u8],
    offset: usize,
}

impl<'a> Wrapper<'a> {
    fn new(buf: &'a mut [u8]) -> Self {
        Wrapper {
            buf: buf,
            offset: 0,
        }
    }
}

impl<'a> fmt::Write for Wrapper<'a> {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        let bytes = s.as_bytes();

        // Skip over already-copied data
        let remainder = &mut self.buf[self.offset..];
        // Check if there is space remaining (return error instead of panicking)
        if remainder.len() < bytes.len() {
            return Err(core::fmt::Error);
        }
        // Make the two slices the same length
        let remainder = &mut remainder[..bytes.len()];
        // Copy
        remainder.copy_from_slice(bytes);

        // Update offset to avoid overwriting
        self.offset += bytes.len();

        Ok(())
    }
}

async fn test_future() {
    //let lsm = lsm::Settings::default().init().await.unwrap();
    //loop {
    //    let mut xvg = 0.0;
    //    let mut yvg = 0.0;
    //    let mut zvg = 0.0;
    //    for _ in 0..1000 {
    //        let sample = lsm.acceleration().await.unwrap();
    //        xvg +=  sample.ax / 1000.0;
    //        yvg += sample.ay / 1000.0;
    //        zvg += sample.az / 1000.0;
    //    }
    //    rprintln!("Acc Sample: ({}, {}, {})", xvg, yvg, zvg);
    //}
    let lsm = lsm::Settings::default().init().await.unwrap();
    let bmp = bmp::Settings::default().init().await.unwrap();
    let mut last_ticks = 0;
    //let mut last_quat = Quaternion::new(1., 0., 0., 0.);
    let mut abx = 0.0;
    let mut aby = 0.0;
    let mut abz = 0.0;
    let mut gbx = 0.0;
    let mut gby = 0.0;
    let mut gbz = 0.0;
    let mut n = 0;
    while n < 1000 {
        n += 1;
        let acc_sample = lsm.acceleration().await.unwrap();
        abx += acc_sample.ax;
        aby += acc_sample.ay;
        abz += acc_sample.az;
        let gyro_sample = lsm.angular_rate().await.unwrap();
        gbx += gyro_sample.gx;
        gby += gyro_sample.gy;
        gbz += gyro_sample.gz;
    }
    abx /= n as f32;
    aby /= n as f32;
    abz /= n as f32;

    abz -= 1.0;

    gbx /= n as f32;
    gby /= n as f32;
    gbz /= n as f32;

    loop {
        if get_ticks() - last_ticks > 20 {
            let acc_sample = lsm.acceleration().await.unwrap();
            let rate_sample = lsm.angular_rate().await.unwrap();
            let bmp_sample = bmp.pressure_temperature().await.unwrap();
            let mut filter = ukf::UKF::new();
            let prediction = filter.predict(
                //Matrix6::<f32>::new(
                //    0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
                //    0., 1., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 1.,
                //),
                Matrix6::<f32>::identity(),
                (get_ticks() - last_ticks) as f32 / 1000.0,
            );

            let ab = Vector3::<f32>::new(
                acc_sample.ax - abx,
                acc_sample.ay - aby,
                acc_sample.az - abz,
            )
            .normalize();
            let fake_mag = prediction
                .pose
                .transform_vector(&Vector3::new(-1.0, 0.0, 0.0));

            let pred = filter.update(
                ukf::Observation::new(
                    ab.x,
                    ab.y,
                    ab.z,
                    (rate_sample.gx - gbx) * PI / 180.0,
                    (rate_sample.gy - gby) * PI / 180.0,
                    (rate_sample.gz - gbz) * PI / 180.0,
                    fake_mag.x,
                    fake_mag.y,
                    fake_mag.z,
                ),
                Matrix9::<f32>::identity() * 0.01,
            );
            //// Construct a quaternion rotating from the Earth
            //// frame to the measured frame.
            //let gyro_quat = {
            //    let dt: f32 = (get_ticks() - last_ticks) as f32 / 1000.;
            //    Quaternion::new(
            //        1.,
            //        rate_sample.gx / 180. * PI * dt / 2.,
            //        rate_sample.gy / 180. * PI * dt / 2.,
            //        -rate_sample.gz / 180. * PI * dt / 2.,
            //    )
            //    .normalized()
            //        * last_quat
            //};
            //let acc_quat = {
            //    use libm::{powf, sqrtf};
            //    // Because the expected gravity vector is [0 0 1] in our coordinate system
            //    // The dot product of the gravity vector and the observation is the measured
            //    // z value.
            //    // The sensor z is inverted WRT our coordinate system.
            //    let dot = acc_sample.az
            //        / sqrtf(
            //            powf(acc_sample.ax, 2.) + powf(acc_sample.ay, 2.) + powf(acc_sample.az, 2.),
            //        );
            //    let cos_ht = sqrtf((1. + dot) / 2.);
            //    let sin_ht = sqrtf((1. - dot) / 2.);
            //    Quaternion::new(
            //        cos_ht,
            //        acc_sample.ax * sin_ht,
            //        acc_sample.ay * sin_ht,
            //        acc_sample.az * sin_ht,
            //    )
            //};
            //last_quat = (acc_quat.scale(0.05) + gyro_quat.scale(0.95)).normalized();
            let mut msg = [0 as u8; 70];
            //rprintln!("{:?}", last_quat);
            write!(
                Wrapper::new(&mut msg),
                "w{}wa{}ab{}bc{}c\r\n",
                pred.pose.scalar(),
                pred.pose.vector()[0],
                pred.pose.vector()[1],
                pred.pose.vector()[2]
            )
            .unwrap();
            usart::write_message(&msg).await.unwrap();
            rprintln!(
                "(ax, ay, az): ({:.2}, {:.2}, {:.2}), (gx, gy, gz): ({:.2}, {:.2}, {:.2}), (p, t): ({:.2}, {:.2})",
                acc_sample.ax - abx,
                acc_sample.ay - aby,
                acc_sample.az - abz,
                rate_sample.gx - gbx,
                rate_sample.gy - gby,
                rate_sample.gz - gbz,
                bmp_sample.press,
                bmp_sample.temp
            );
            last_ticks = get_ticks();
        }
    }
}
