//! CAN FD Driver Wrapper — embassy-stm32 FDCAN abstraction

use embassy_stm32::can::fdcan::{Fdcan, RxFifo, TxFifo, Frame, TxElement, RxElement};
use embassy_stm32::Peripherals;
use heapless::Vec;
use thiserror::Error;

#[derive(Debug, Error, defmt::Format)]
pub enum CanError {
    #[error("TX buffer full")]
    TxFull,
    #[error("RX buffer empty")]
    RxEmpty,
    #[error("Bus off")]
    BusOff,
    #[error("Protocol error")]
    ProtocolError,
    #[error("Invalid frame")]
    InvalidFrame,
}

#[derive(Debug, Clone, defmt::Format)]
pub struct CanFrame {
    pub id: u32,
    pub dlc: u8,
    pub data: Vec<u8, 64>,
    pub timestamp_us: u64,
    pub is_fd: bool,
    pub bitrate_switch: bool,
}

impl From<Frame> for CanFrame {
    fn from(frame: Frame) -> Self {
        let mut data = Vec::new();
        data.extend_from_slice(&frame.data[..frame.dlc as usize]).ok();
        Self {
            id: frame.id,
            dlc: frame.dlc,
            data,
            timestamp_us: embassy_time::Instant::now().as_micros(),
            is_fd: frame.fdf,
            bitrate_switch: frame.brs,
        }
    }
}

impl From<CanFrame> for Frame {
    fn from(frame: CanFrame) -> Self {
        let mut data = [0u8; 64];
        data[..frame.data.len()].copy_from_slice(&frame.data);
        Self {
            id: frame.id,
            dlc: frame.dlc,
            data,
            fdf: frame.is_fd,
            brs: frame.bitrate_switch,
            esi: false,
            rtr: false,
            xtd: true,  // Extended ID for J1939
        }
    }
}

pub struct CanTask;

impl CanTask {
    pub async fn transmit<D: embassy_stm32::can::fdcan::Instance>(
        can: &mut Fdcan<'_, D>,
        frame: &CanFrame,
    ) -> Result<(), CanError> {
        let tx_frame: Frame = frame.clone().into();
        can.write(&tx_frame).await.map_err(|_| CanError::TxFull)
    }

    pub async fn receive<D: embassy_stm32::can::fdcan::Instance>(
        can: &mut Fdcan<'_, D>,
    ) -> Result<CanFrame, CanError> {
        let frame = can.read().await.map_err(|_| CanError::RxEmpty)?;
        Ok(frame.into())
    }
}

pub fn configure_acceptance_filter<D: embassy_stm32::can::fdcan::Instance>(
    can: &mut Fdcan<'_, D>,
    filter_index: u8,
    id: u32,
    mask: u32,
) {
    // Configure standard filter element
    // In real implementation: use can.configure_filter()
}