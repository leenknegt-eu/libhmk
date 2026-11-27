//! Deferred action ring buffer (`src/deferred_actions.c`) for Tap-Hold and Dynamic Keystroke.

pub const MAX_DEFERRED_ACTIONS: usize = 16;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum DeferredActionType {
    #[default]
    None,
    Press,
    Release,
    Tap,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct DeferredAction {
    pub kind: DeferredActionType,
    pub key: u8,
    pub keycode: u8,
    pub ticks: u8,
}

pub struct DeferredActionQueue {
    head: usize,
    size: usize,
    queue: [DeferredAction; MAX_DEFERRED_ACTIONS],
}

impl DeferredActionQueue {
    pub const fn new() -> Self {
        Self {
            head: 0,
            size: 0,
            queue: [DeferredAction {
                kind: DeferredActionType::None,
                key: 0,
                keycode: 0,
                ticks: 0,
            }; MAX_DEFERRED_ACTIONS],
        }
    }

    pub fn clear(&mut self) {
        self.head = 0;
        self.size = 0;
    }

    pub fn push(&mut self, mut action: DeferredAction, tick_rate: u8) -> bool {
        if self.size == MAX_DEFERRED_ACTIONS {
            return false;
        }
        action.ticks = tick_rate;
        let idx = (self.head + self.size) & (MAX_DEFERRED_ACTIONS - 1);
        self.queue[idx] = action;
        self.size += 1;
        true
    }

    /// Pop all ready deferred actions (ticks == 0) into `ready_out` and return their count.
    pub fn tick_and_drain(
        &mut self,
        tick_rate: u8,
        ready_out: &mut [DeferredAction; MAX_DEFERRED_ACTIONS],
    ) -> usize {
        if self.size == 0 {
            return 0;
        }
        let mut count = 0usize;
        for i in 0..self.size {
            let idx = (self.head + i) & (MAX_DEFERRED_ACTIONS - 1);
            let act = &mut self.queue[idx];
            act.ticks = act.ticks.min(tick_rate);
            if act.ticks > 0 {
                act.ticks -= 1;
                continue;
            }
            ready_out[count] = *act;
            count += 1;
        }
        self.head = (self.head + count) & (MAX_DEFERRED_ACTIONS - 1);
        self.size -= count;
        count
    }
}
