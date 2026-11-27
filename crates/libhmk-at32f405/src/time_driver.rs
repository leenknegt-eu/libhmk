//! `embassy-time` driver on the 32-bit TMR2 (plus mode) running at 1 MHz.
//!
//! Uses the half-period scheme of the embassy STM32 driver: the period counter is advanced
//! on overflow and at the counter midpoint (channel 2), so `now()` is monotonic without a
//! critical section. Channel 1 is the alarm.

use core::cell::{Cell, RefCell};
use core::sync::atomic::{compiler_fence, AtomicU32, Ordering};

use at32f4xx_pac::at32f405::{interrupt, Interrupt, CRM, TMR2};
use critical_section::{CriticalSection, Mutex};
use embassy_time_driver::Driver;
use embassy_time_queue_utils::Queue;

use crate::board::APB1_TIMER_HZ;

const TICK_HZ: u32 = 1_000_000;
const _: () = assert!(embassy_time_driver::TICK_HZ == TICK_HZ as u64);

// TMR register bits (same layout as the STM32 general purpose timers)
const CTRL1_TMREN: u32 = 1 << 0;
const CTRL1_PMEN: u32 = 1 << 10;
const IEN_OVF: u32 = 1 << 0;
const IEN_C1: u32 = 1 << 1;
const IEN_C2: u32 = 1 << 2;
const SWEVT_OVF: u32 = 1 << 0;

fn regs() -> &'static at32f4xx_pac::at32f405::tmr2::RegisterBlock {
    unsafe { &*TMR2::ptr() }
}

fn calc_now(period: u32, counter: u32) -> u64 {
    ((period as u64) << 31) + ((counter ^ ((period & 1) << 31)) as u64)
}

struct TimerDriver {
    period: AtomicU32,
    alarm_at: Mutex<Cell<u64>>,
    queue: Mutex<RefCell<Queue>>,
}

embassy_time_driver::time_driver_impl!(static DRIVER: TimerDriver = TimerDriver {
    period: AtomicU32::new(0),
    alarm_at: Mutex::new(Cell::new(u64::MAX)),
    queue: Mutex::new(RefCell::new(Queue::new())),
});

/// Start TMR2 as the system time base. Call once after the clock tree is configured.
pub fn init(crm: &CRM) {
    let r = regs();
    unsafe {
        crm.apb1en().modify(|_, w| w.tmr2().set_bit());
        r.ctrl1().write(|w| w.bits(CTRL1_PMEN));
        r.div().write(|w| w.bits((APB1_TIMER_HZ / TICK_HZ - 1) as u16));
        r.pr().write(|w| w.bits(u32::MAX));
        r.c2dt().write(|w| w.bits(0x8000_0000));
        // Load the prescaler, then clear the resulting overflow flag
        r.swevt().write(|w| w.bits(SWEVT_OVF));
        r.ists().write(|w| w.bits(0));
        r.iden().write(|w| w.bits(IEN_OVF | IEN_C2));
        r.ctrl1().write(|w| w.bits(CTRL1_PMEN | CTRL1_TMREN));

        let mut nvic = cortex_m::Peripherals::steal().NVIC;
        nvic.set_priority(Interrupt::TMR2, 0x20);
        cortex_m::peripheral::NVIC::unmask(Interrupt::TMR2);
    }
}

impl TimerDriver {
    fn on_interrupt(&self) {
        let r = regs();
        critical_section::with(|cs| {
            let ists = r.ists().read().bits();
            let ien = r.iden().read().bits();
            // Clear only the flags we handle (write-0-to-clear)
            unsafe { r.ists().write(|w| w.bits(!(ists & (IEN_OVF | IEN_C1 | IEN_C2)))) };

            if ists & IEN_OVF != 0 {
                self.next_period(cs);
            }
            if ists & IEN_C2 != 0 {
                self.next_period(cs);
            }
            if ists & IEN_C1 != 0 && ien & IEN_C1 != 0 {
                self.trigger_alarm(cs);
            }
        });
    }

    fn next_period(&self, cs: CriticalSection) {
        let period = self.period.load(Ordering::Relaxed) + 1;
        self.period.store(period, Ordering::Relaxed);
        let t = (period as u64) << 31;
        let at = self.alarm_at.borrow(cs).get();
        if at < t + 0xc000_0000 {
            // Alarm falls into the next 3/4 period: arm channel 1
            let r = regs();
            unsafe { r.iden().modify(|rr, w| w.bits(rr.bits() | IEN_C1)) };
        }
    }

    fn trigger_alarm(&self, cs: CriticalSection) {
        let mut next = self.queue.borrow(cs).borrow_mut().next_expiration(self.now());
        while !self.set_alarm(cs, next) {
            next = self.queue.borrow(cs).borrow_mut().next_expiration(self.now());
        }
    }

    fn set_alarm(&self, cs: CriticalSection, timestamp: u64) -> bool {
        let r = regs();
        self.alarm_at.borrow(cs).set(timestamp);

        let t = self.now();
        if timestamp <= t {
            unsafe { r.iden().modify(|rr, w| w.bits(rr.bits() & !IEN_C1)) };
            self.alarm_at.borrow(cs).set(u64::MAX);
            return false;
        }

        unsafe {
            r.c1dt().write(|w| w.bits(timestamp as u32));
            // Clear a stale compare flag before (re-)enabling the interrupt
            r.ists().write(|w| w.bits(!IEN_C1));
        }
        let diff = timestamp - t;
        unsafe {
            r.iden().modify(|rr, w| {
                if diff < 0xc000_0000 {
                    w.bits(rr.bits() | IEN_C1)
                } else {
                    w.bits(rr.bits() & !IEN_C1)
                }
            })
        };

        // The timestamp may have been reached while the compare value was being written
        let t = self.now();
        if timestamp <= t {
            unsafe { r.iden().modify(|rr, w| w.bits(rr.bits() & !IEN_C1)) };
            self.alarm_at.borrow(cs).set(u64::MAX);
            return false;
        }
        true
    }
}

impl Driver for TimerDriver {
    fn now(&self) -> u64 {
        let r = regs();
        let period = self.period.load(Ordering::Relaxed);
        compiler_fence(Ordering::Acquire);
        let counter = r.cval().read().bits();
        calc_now(period, counter)
    }

    fn schedule_wake(&self, at: u64, waker: &core::task::Waker) {
        critical_section::with(|cs| {
            let mut queue = self.queue.borrow(cs).borrow_mut();
            if queue.schedule_wake(at, waker) {
                let mut next = queue.next_expiration(self.now());
                while !self.set_alarm(cs, next) {
                    next = queue.next_expiration(self.now());
                }
            }
        })
    }
}

#[interrupt]
fn TMR2() {
    DRIVER.on_interrupt();
}
