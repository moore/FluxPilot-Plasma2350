//! Plasma 2350 board pin mapping derived from the schematic and pinout PDFs.

/// LED strip screw terminal is driven through a level shifter.
/// These GPIOs are 3.3V on the RP2350; the screw terminals are 5V.
pub const LED_DATA_3V3_GPIO: u8 = 15; // DATA_3V3 -> DATA_5V
pub const LED_CLK_3V3_GPIO: u8 = 14; // CLK_3V3 -> CLK_5V

/// Onboard RGB LED (active low).
pub const LED_R_GPIO: u8 = 16;
pub const LED_G_GPIO: u8 = 17;
pub const LED_B_GPIO: u8 = 18;
pub const LED_RGB_ACTIVE_LOW: bool = true;

/// I2C header and interrupt line.
pub const I2C_INT_GPIO: u8 = 19;
pub const I2C_SDA_GPIO: u8 = 20;
pub const I2C_SCL_GPIO: u8 = 21;

/// User button (non-boot).
pub const USER_SW_GPIO: u8 = 22;

/// UART0 on the expansion header.
pub const UART0_TX_GPIO: u8 = 0;
pub const UART0_RX_GPIO: u8 = 1;

/// SP/CE connector signals (as labeled on the schematic).
pub const PWRKEY_BL_GPIO: u8 = 7;
pub const UART_TX_DC_GPIO: u8 = 8;
pub const UART_RX_CS_GPIO: u8 = 9;
pub const NETLIGHT_SCK_GPIO: u8 = 10;
pub const RESET_MOSI_GPIO: u8 = 11;
pub const SW_A_GPIO: u8 = 12;

/// Analog inputs on the expansion header.
pub const ADC0_GPIO: u8 = 26;
pub const ADC1_GPIO: u8 = 27;
pub const ADC2_GPIO: u8 = 28;
pub const ADC3_GPIO: u8 = 29;

#[derive(Debug, Clone, Copy)]
pub struct BoardPins {
    pub led_data_3v3: u8,
    pub led_clk_3v3: u8,
    pub led_r: u8,
    pub led_g: u8,
    pub led_b: u8,
    pub i2c_int: u8,
    pub i2c_sda: u8,
    pub i2c_scl: u8,
    pub user_sw: u8,
    pub uart0_tx: u8,
    pub uart0_rx: u8,
    pub adc0: u8,
    pub adc1: u8,
    pub adc2: u8,
    pub adc3: u8,
}

#[derive(Debug, Clone, Copy)]
pub struct Board {
    pub pins: BoardPins,
}

impl Board {
    pub fn new() -> Self {
        Self {
            pins: BoardPins {
                led_data_3v3: LED_DATA_3V3_GPIO,
                led_clk_3v3: LED_CLK_3V3_GPIO,
                led_r: LED_R_GPIO,
                led_g: LED_G_GPIO,
                led_b: LED_B_GPIO,
                i2c_int: I2C_INT_GPIO,
                i2c_sda: I2C_SDA_GPIO,
                i2c_scl: I2C_SCL_GPIO,
                user_sw: USER_SW_GPIO,
                uart0_tx: UART0_TX_GPIO,
                uart0_rx: UART0_RX_GPIO,
                adc0: ADC0_GPIO,
                adc1: ADC1_GPIO,
                adc2: ADC2_GPIO,
                adc3: ADC3_GPIO,
            },
        }
    }
}
