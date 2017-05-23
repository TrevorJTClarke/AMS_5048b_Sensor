/**
 * Constants for AS5048B
 */

// Default addresses for AS5048B
export const AS5048 = {
  ADDRESS: 0x40, // 0b10000 + ( A1 & A2 to GND)
  PROG_REG: 0x03,
  ADDR_REG: 0x15,
  ZEROMSB_REG: 0x16, //bits 0..7 (MOST SIGNIFICANT BIT)
  ZEROLSB_REG: 0x17, //bits 0..5 (LEAST SIGNIFICANT BIT)
  GAIN_REG: 0xFA,
  DIAG_REG: 0xFB,
  MAGNMSB_REG: 0xFC, //bits 0..7
  MAGNLSB_REG: 0xFD, //bits 0..5
  ANGLMSB_REG: 0xFE, //bits 0..7
  ANGLLSB_REG: 0xFF, //bits 0..5
  RESOLUTION: 16384.0 //14 bits
}

// Moving Exponential Average on angle - beware heavy calculation for some boards
// This is a 1st order low pass filter
// Moving average is calculated on Sine et Cosine values of the angle to provide an extrapolated accurate angle value.
export const EXP = {
  MOVAVG_N: 5,	//history length impact on moving average impact - keep in mind the moving average will be impacted by the measurement frequency too
  MOVAVG_LOOP: 1 //number of measurements before starting mobile Average - starting with a simple average - 1 allows a quick start. Value must be 1 minimum
}

//unit consts - just to make the units more readable
export const UNIT = {
  RAW: 1,
  TRN: 2,
  DEG: 3,
  RAD: 4,
  GRAD: 5,
  MOA: 6,
  SOA: 7,
  MILNATO: 8,
  MILSE: 9,
  MILRU: 10
}
