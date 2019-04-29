pub use crate::afio::AfioExt as _stm32_hal_afio_AfioExt;
pub use crate::dma::DmaExt as _stm32_hal_dma_DmaExt;
pub use crate::flash::FlashExt as _stm32_hal_flash_FlashExt;
pub use crate::gpio::GpioExt as _stm32_hal_gpio_GpioExt;
pub use crate::hal::adc::OneShot as _embedded_hal_adc_OneShot;
pub use crate::hal::digital::StatefulOutputPin as _embedded_hal_digital_StatefulOutputPin;
pub use crate::hal::digital::ToggleableOutputPin as _embedded_hal_digital_ToggleableOutputPin;
pub use crate::hal::prelude::*;
pub use crate::pwm::PwmExt as _stm32_hal_pwm_PwmExt;
pub use crate::rcc::RccExt as _stm32_hal_rcc_RccExt;
pub use crate::dma::CircReadDma as _stm32_hal_dma_CircReadDma;
pub use crate::dma::ReadDma as _stm32_hal_dma_ReadDma;
pub use crate::dma::WriteDma as _stm32_hal_dma_WriteDma;
pub use crate::time::U32Ext as _stm32_hal_time_U32Ext;
#[cfg(feature = "stm32f103")]
pub use crate::can::TransmitMailbox as _stm32_hal_can_TransmitMailbox;
#[cfg(feature = "stm32f103")]
pub use crate::can::ReceiveFifo as _stm32_hal_can_ReceiveFifo;
