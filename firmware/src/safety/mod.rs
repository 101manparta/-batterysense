//! Safety Module — Watchdog, plausibility checks, fault detection

pub mod watchdog;
pub mod plausibility;

pub use watchdog::{WatchdogTask, WatchdogError, ExternalSupervisor};
pub use plausibility::{PlausibilityChecker, SafetyEvent, PlausibilityConfig};