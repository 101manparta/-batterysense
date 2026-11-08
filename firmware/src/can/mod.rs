//! CAN Module — CAN FD with J1939/CANopen support

pub mod j1939;
pub mod fdcan;

pub use j1939::{J1939Encoder, J1939Decoder, Pgn, Spn};
pub use fdcan::{CanFrame, CanTask, CanError};