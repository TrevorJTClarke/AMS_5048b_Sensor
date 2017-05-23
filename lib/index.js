// Defaults for AS5048B
import { AS5048, EXP, UNIT } from './constants';

class AS5048B {

  constructor(i2c, addr) {
    super(i2c, addr)

    this.i2c = i2c
    this.addr = addr || AS5048.ADDRESS

    // standard internal variables
  	this.clockWise = true
  	this.chipAddress = new Uint8Array(0)
  	this.addressRegVal = new Uint8Array(0)
  	this.zeroRegVal = new Uint16Array(0)
  	this.lastAngleRaw = 0

    // TODO: CHeck need
    this.initialize()
  }

  /* Init values and overall behaviors for AS5948B use */
  initialize() {
    this.clockWise = false
  	this.lastAngleRaw = 0.0

    // 1. read zero & angle
    // 2. write zero to 0
    // 3. write zero to current value
    // 4. ready to operate
  	this.zeroRegVal = this.zeroRegRead()
  	this.addressRegVal = this.addressRegRead()
  }

  /**
   * Set / unset clock wise counting - sensor counts CCW natively
   *
   * @param cw - true: CW, false: CCW
   */
  setClockWise(cw) {
  	this.clockWise = cw
  	this.lastAngleRaw = 0.0
  }

  /**
   * Writes to OTP control register
   *
   * @param register value
   */
  progRegister(val) {
    let data = new Uint8Array(val)
    this.i2c.writeTo(AS5048.PROG_REG, data)
  }

  /* Burn values to the OTP register */
  doProg() {
    var _this = this

  	//enable special programming mode
  	_this.progRegister(0xFD)

  	//set the burn bit: enables automatic programming procedure
  	setTimeout(function(){
  	  _this.progRegister(0x08)
  	}, 10)

  	//disable special programming mode
    setTimeout(function(){
  	  _this.progRegister(0x00)
  	}, 20)
  }

  /**
   * write I2C address value (5 bits) into the address register
   *
   * @param slave address value
   *
   * @description (From Data Sheet)
   * The slave address consists of the hardware setting on pins A1,
   * A2. The MSB of the slave address (yellow) is internally inverted.
   * This means that by default the resulting data is ‘1’. A read of the
   * I²C slave address register 21 will return a ‘0’ at the MSB.
   */
  addressRegWrite(val) {
    let data = new Uint8Array(val)

  	// write the new chip address to the register
  	this.writeReg(AS5048.ADDR_REG, data)

  	// update our chip address with our 5 programmable bits
  	// the MSB is internally inverted, so we flip the leftmost bit
  	this.chipAddress = ((data << 2) | (this.chipAddress & 0b11)) ^ (1 << 6)
  }

  /* reads I2C address register value */
  addressRegRead() {
  	return this.readReg8(AS5048.ADDR_REG)
  }

  /* sets current angle as the zero position */
  setZeroPosition() {
  	let newZero = this.readReg16(AS5048.ANGLLSB_REG)
    // TODO:
    console.log('newZero', newZero)
  	this.zeroRegWrite(0x00) //Issue closed by @MechatronicsWorkman
  	// this.zeroRegWrite(newZero)
  }

  /**
   * writes the 2 bytes Zero position register value
   *
   * @param register value (16bit, broken into 8bits)
   *
   * @description (From Data Sheet)
   * Programming of the Zero Position: The absolute angle
   * position can be permanent programmed over the interface.
   * This could be useful for random placement of the magnet on
   * the rotation axis. A readout at the mechanical zero position can
   * be performed and written back into the IC. With permanent
   * programming the position is non-reversible stored in the IC.
   * This programming can be performed only once.
   * To simplify the calculation of the zero position it is only needed
   * to write the value in the IC which was read out before from the
   * angle register.
   *
   * Programming Sequence with Verification:
   * To program the zero position is needed to
   * perform following sequence:
   *  1. Write 0 into OTP zero position register to clear
   *  2. Read angle information
   *  3. Write previous read angle position into OTP zero
   * position register
   * Now the zero position is set.
   * If you want to burn it to the OTP register send:
   *  4. Set the Programming Enable bit in the OTP control
   * register
   *  5. Set the Burn bit to start the automatic programming
   * procedure
   *  6. Read angle information (equals to 0)
   *  7. Set the Verify bit to load the OTP data again into the
   * internal registers
   *  8. Read angle information (equals to 0)
   *
   * TODO: Update to utilize this flow!!
   */
  zeroRegWrite(val) {
    // adjusts bits to conform to standards
  	this.writeReg(AS5048.ZEROMSB_REG, (val >> 6))
  	this.writeReg(AS5048.ZEROLSB_REG, (val & 0x3F))
  }

  /* reads the 2 bytes Zero position register value */
  /* returns uint16_t register value trimmed on 14 bits */
  zeroRegRead() {
  	return this.readReg16(AS5048.ZEROMSB_REG)
  }

  /* reads the 2 bytes magnitude register value */
  /* returns uint16_t register value trimmed on 14 bits */
  magnitudeRead() {
  	return this.readReg16(AS5048.MAGNMSB_REG)
  }

  /* reads the 2 bytes magnitude register value */
  /* returns uint16_t register value trimmed on 14 bits */
  angleRegRead() {
  	return this.readReg16(AS5048.ANGLMSB_REG)
  }

  /* reads the 1 bytes auto gain register value */
  getAutoGain() {
  	return this.readReg8(AS5048.GAIN_REG)
  }

  /* reads the 1 bytes diagnostic register value */
  getDiagReg() {
  	return this.readReg8(AS5048.DIAG_REG)
  }

  /**
   * reads current angle value and converts it into the desired unit
   *
   * @param unit : string expressing the unit of the angle. Sensor raw value as default
   * @param newVal : have a new measurement or use the last read one. True as default
   * @return angle value converted into the desired unit
   */
  angleRead(unit, newVal) {
  	var angleRaw

  	if (newVal) {
  		if (this.clockWise) {
        // highest possible number, subtracting read number
  			angleRaw = (0b11111111111111 - this.readReg16(AS5048.ANGLLSB_REG))
  		} else {
  			angleRaw = this.readReg16(AS5048.ANGLLSB_REG)
  		}
  		this.lastAngleRaw = angleRaw
  	} else {
  		angleRaw = this.lastAngleRaw
  	}

  	return this.convertAngle(unit, angleRaw)
  }

  /**
   * Converts raw data to specified unit
   *
   * @param unit: See above list of unit definitions
   * @param angle: raw data of angle
   * @return converted angle
   */
  convertAngle(unit, angle) {

  	// convert raw sensor reading into angle unit
  	var angleConv;

  	switch (unit) {
  		case UNIT.RAW:
  			//Sensor raw measurement
  			angleConv = angle
  			break
  		case UNIT.TRN:
  			//full turn ratio
  			angleConv = (angle / AS5048.RESOLUTION)
  			break
  		case UNIT.DEG:
  			//degree
  			angleConv = (angle / AS5048.RESOLUTION) * 360.0
  			break
  		case UNIT.RAD:
  			//Radian
  			angleConv = (angle / AS5048.RESOLUTION) * 2 * Math.PI
  			break
  		case UNIT.MOA:
  			//minute of arc
  			angleConv = (angle / AS5048.RESOLUTION) * 60.0 * 360.0
  			break
  		case UNIT.SOA:
  			//second of arc
  			angleConv = (angle / AS5048.RESOLUTION) * 60.0 * 60.0 * 360.0
  			break
  		case UNIT.GRAD:
  			//grade
  			angleConv = (angle / AS5048.RESOLUTION) * 400.0
  			break
  		case UNIT.MILNATO:
  			//NATO MIL
  			angleConv = (angle / AS5048.RESOLUTION) * 6400.0
  			break
  		case UNIT.MILSE:
  			//Swedish MIL
  			angleConv = (angle / AS5048.RESOLUTION) * 6300.0
  			break
  		case UNIT.MILRU:
  			//Russian MIL
  			angleConv = (angle / AS5048.RESOLUTION) * 6000.0
  			break
  		default:
  			//no conversion => raw angle
  			angleConv = angle
  			break
  	}

  	return angleConv
  }

  // Read from register, 8 bits
  readReg8(reg, length) {
    length = length || 1

    //8 bit value got from 1 8bits register
    this.i2c.writeTo(this.addr, reg)
    let d = this.i2c.readFrom(this.addr, length)
    return d[0]
  }

  // Read from register, 16 bits consisting of
  // most significant bits and least significant bits
  readReg16(reg, length) {
    length = length || 2

  	//16 bit value got from 2 8bits registers (7..0 MSB + 5..0 LSB) => 14 bits value
    this.i2c.writeTo(this.addr, reg)
    var d = this.i2c.readFrom(this.addr, length)

    // format MSB & LSB into signed int
    var i = d[0] << 6
        i += (d[1] & 0x3F)

    return i
  }

  // TODO: Seems wrong?
  // writes to a specific register within initialized address
  writeReg(reg, data) {
    let arr = [reg].concat(data)
    let formatted = new Uint16Array(arr)
    console.log('writeReg: data', reg, data, arr, formatted)
    this.i2c.writeTo(this.addr, formatted)
  }

}
