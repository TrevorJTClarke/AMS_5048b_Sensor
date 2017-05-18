
/* Copyright (c) 2017 Trevor Clarke. @trevorjtclarke

History:
v1.0.0 - First release

See the file LICENSE for copying permission.

Module for the AMS AS5048B Magnetic Position Sensor
Only I2C is supported. (AS5048A has SPI version)

Parts of the module is based on the driver written by SOSAndroid.fr (E. Ha.):
I2Cdev device library code is placed under the MIT license
Copyright (c) 2013 SOSAndroid.fr (E. Ha.)

Software License Agreement (BSD License)

Copyright (c) 2013, SOSAndroid.fr (E. Ha.)
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.
3. Neither the name of the copyright holders nor the
names of its contributors may be used to endorse or promote products
derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/* Module constants */
var ASC = {

  // Moving Exponential Average on angle - beware heavy calculation for some boards
  // This is a 1st order low pass filter
  // Moving average is calculated on Sine et Cosine values of the angle to provide an extrapolated accurate angle value.
  EXP_MOVAVG_N: 5,	//history length impact on moving average impact - keep in mind the moving average will be impacted by the measurement frequency too
  EXP_MOVAVG_LOOP: 1, //number of measurements before starting mobile Average - starting with a simple average - 1 allows a quick start. Value must be 1 minimum

  //unit consts - just to make the units more readable
  U_RAW: 1,
  U_TRN: 2,
  U_DEG: 3,
  U_RAD: 4,
  U_GRAD: 5,
  U_MOA: 6,
  U_SOA: 7,
  U_MILNATO: 8,
  U_MILSE: 9,
  U_MILRU: 10

};

/* Register addresses */
var R = {
  // Default addresses for AS5048B
  AS5048_ADDRESS: 0x40, // 0b10000 + ( A1 & A2 to GND)
  AS5048B_PROG_REG: 0x03,
  AS5048B_ADDR_REG: 0x15,
  AS5048B_ZEROMSB_REG: 0x16, //bits 0..7 (MOST SIGNIFICANT BIT)
  AS5048B_ZEROLSB_REG: 0x17, //bits 0..5 (LEAST SIGNIFICANT BIT)
  AS5048B_GAIN_REG: 0xFA,
  AS5048B_DIAG_REG: 0xFB,
  AS5048B_MAGNMSB_REG: 0xFC, //bits 0..7
  AS5048B_MAGNLSB_REG: 0xFD, //bits 0..5
  AS5048B_ANGLMSB_REG: 0xFE, //bits 0..7
  AS5048B_ANGLLSB_REG: 0xFF, //bits 0..5
  AS5048B_RESOLUTION: 16384.0 //14 bits
};

// exports.connect = function (_i2c, _addr) {
//   return new AS5048B(_i2c, _addr);
// };

/* AS5048B Object */
function AS5048B(_i2c, _addr) {
  this.i2c = _i2c;
  this.addr = _addr || R.AS5048_ADDRESS;

  // standard internal variables
	this.clockWise = true;
	this.chipAddress = new Uint8Array(0);
	this.addressRegVal = new Uint8Array(0);
	this.zeroRegVal = new Uint16Array(0);
	this.lastAngleRaw = 0;
	this.movingAvgExpAngle = 0;
	this.movingAvgExpSin = 0;
	this.movingAvgExpCos = 0;
	this.movingAvgExpAlpha = 0;
	this.movingAvgCountLoop = 0;

  this.initialize();
}


/* Init values and overall behaviors for AS5948B use */
AS5048B.prototype.initialize = function() {
  this.clockWise = false;
	this.lastAngleRaw = 0.0;

  var tmp = this.readReg16(R.AS5048B_ANGLLSB_REG, 2);
  console.log('tmp', tmp);
  var cnvt = this.convertAngle(ASC.U_DEG, tmp);
  console.log('AS5048B_ANGLMSB_REG', cnvt);

  // // 1. read zero & angle
  // // 2. write zero to 0
  // // 3. write zero to current value
  // // 4. ready to operate
	// this.zeroRegVal = this.zeroRegRead();
	// this.addressRegVal = this.addressRegRead();
  //
	// this.resetMovingAvgExp();
};

/**
 * Set / unset clock wise counting - sensor counts CCW natively
 *
 * @param cw - true: CW, false: CCW
 */
AS5048B.prototype.setClockWise = function (cw) {
	this.clockWise = cw;
	this.lastAngleRaw = 0.0;
	this.resetMovingAvgExp();
}

/**
 * Writes to OTP control register
 *
 * @param register value
 */
AS5048B.prototype.progRegister = function (val) {
  var data = new Uint8Array(val);
  this.i2c.writeTo(R.AS5048B_PROG_REG, data);
}

/* Burn values to the OTP register */
AS5048B.prototype.doProg = function () {
  var _this = this;

	//enable special programming mode
	_this.progRegister(0xFD);

	//set the burn bit: enables automatic programming procedure
	setTimeout(function(){
	  _this.progRegister(0x08);
	}, 10);

	//disable special programming mode
  setTimeout(function(){
	  _this.progRegister(0x00);
	}, 20);
}

/**
 * write I2C address value (5 bits) into the address register
 *
 * @param register value
 */
AS5048B.prototype.addressRegWrite = function (val) {
  var data = new Uint8Array(val);

	// write the new chip address to the register
	this.writeReg(R.AS5048B_ADDR_REG, data);

	// update our chip address with our 5 programmable bits
	// the MSB is internally inverted, so we flip the leftmost bit
	// TODO: NEED HELP MAKING THIS CHANGE!
	this.chipAddress = ((regVal << 2) | (_chipAddress & 0b11)) ^ (1 << 6);
}

/* reads I2C address register value */
AS5048B.prototype.addressRegRead = function () {
	return this.readReg8(R.AS5048B_ADDR_REG);
}

/* sets current angle as the zero position */
AS5048B.prototype.setZeroPosition = function () {
  var _this = this;

	var newZero = _this.readReg16(R.AS5048B_ANGLMSB_REG);
	this.zeroRegWrite(new Uint8Array(0x00)); //Issue closed by @MechatronicsWorkman
	this.zeroRegWrite(newZero);
}

/**
 * writes the 2 bytes Zero position register value
 *
 * @param register value
 */
AS5048B.prototype.zeroRegWrite = function (val) {
  var data = new Uint8Array(val);

  // TODO: TEST!!!!!!!!!
  // adjusts bits to conform to standards
	this.writeReg(R.AS5048B_ZEROMSB_REG, (regVal >> 6));
	this.writeReg(R.AS5048B_ZEROLSB_REG, (regVal & 0x3F));
}

/* reads the 2 bytes Zero position register value */
/* returns uint16_t register value trimmed on 14 bits */
AS5048B.prototype.zeroRegRead = function () {
	return this.readReg16(R.AS5048B_ZEROMSB_REG);
}

/* reads the 2 bytes magnitude register value */
/* returns uint16_t register value trimmed on 14 bits */
AS5048B.prototype.magnitudeRead = function () {
	return this.readReg16(R.AS5048B_MAGNMSB_REG);
}

/* reads the 2 bytes magnitude register value */
/* returns uint16_t register value trimmed on 14 bits */
AS5048B.prototype.angleRegRead = function () {
	return this.readReg16(R.AS5048B_ANGLMSB_REG);
}

/* reads the 1 bytes auto gain register value */
AS5048B.prototype.getAutoGain = function () {
	return this.readReg8(R.AS5048B_GAIN_REG);
}

/* reads the 1 bytes diagnostic register value */
AS5048B.prototype.getDiagReg = function () {
	return this.readReg8(R.AS5048B_DIAG_REG);
}

/**
 * reads current angle value and converts it into the desired unit
 *
 * @param unit : string expressing the unit of the angle. Sensor raw value as default
 * @param newVal : have a new measurement or use the last read one. True as default
 * @return angle value converted into the desired unit
 */
AS5048B.prototype.angleRead = function (unit, newVal) {
	var angleRaw;

	if (newVal) {
		if (this.clockWise) {
      // TODO: NEED HELP MAKING THIS CHANGE!
      // NOTE: looks like highest possible number, subtracting read number
			angleRaw = (0b11111111111111 - this.readReg16(R.AS5048B_ANGLMSB_REG));
		} else {
			angleRaw = this.readReg16(R.AS5048B_ANGLMSB_REG);
		}
		this.lastAngleRaw = angleRaw;
	} else {
		angleRaw = this.lastAngleRaw;
	}

	return this.convertAngle(unit, angleRaw);
}

/* Performs an exponential moving average on the angle. */
/* Works on Sine and Cosine of the angle to avoid issues 0°/360° discontinuity */
AS5048B.prototype.updateMovingAvgExp = function () {

	//sine and cosine calculation on angles in radian
	var angle = this.angleRead(ASC.U_RAD, true);

	if (this.movingAvgCountLoop < ASC.EXP_MOVAVG_LOOP) {
		this.movingAvgExpSin += Math.sin(angle);
		this.movingAvgExpCos += Math.cos(angle);
		if (this.movingAvgCountLoop == (ASC.EXP_MOVAVG_LOOP - 1)) {
			this.movingAvgExpSin = this.movingAvgExpSin / ASC.EXP_MOVAVG_LOOP;
			this.movingAvgExpCos = this.movingAvgExpCos / ASC.EXP_MOVAVG_LOOP;
		}
		this.movingAvgCountLoop++;
	} else {
		var movavgexpsin = this.movingAvgExpSin + this.movingAvgExpAlpha * (Math.sin(angle) - this.movingAvgExpSin);
		var movavgexpcos = this.movingAvgExpCos + this.movingAvgExpAlpha * (Math.cos(angle) - this.movingAvgExpCos);
		this.movingAvgExpSin = movavgexpsin;
		this.movingAvgExpCos = movavgexpcos;
		this.movingAvgExpAngle = this.getExpAvgRawAngle();
	}

}

/**
 * sent back the exponential moving averaged angle in the desired unit
 *
 * @param unit : string expressing the unit of the angle. Sensor raw value as default
 * @return exponential moving averaged angle value
 */
AS5048B.prototype.getMovingAvgExp = function (unit) {
  var _this = this;
	return this.convertAngle(unit, _this.movingAvgExpAngle);
}

/* resets moving AVG */
AS5048B.prototype.resetMovingAvgExp = function () {
	this.movingAvgExpAngle = 0.0;
	this.movingAvgCountLoop = 0;
	this.movingAvgExpAlpha = 2.0 / (ASC.EXP_MOVAVG_N + 1.0);
}

/* returns average raw angle */
AS5048B.prototype.getExpAvgRawAngle = function () {
  var _this = this;
	var angle;
	var twopi = 2 * Math.PI;

	if (this.movingAvgExpSin < 0.0) {
		angle = twopi - Math.acos(_this.movingAvgExpCos);
	} else {
		angle = Math.acos(_this.movingAvgExpCos);
	}

	angle = (angle / twopi) * R.AS5048B_RESOLUTION;

	return angle;
}

/**
 * Converts raw data to specified unit
 *
 * @param unit: See above list of unit definitions
 * @param angle: raw data of angle
 * @return converted angle
 */
AS5048B.prototype.convertAngle = function (unit, angle) {

	// convert raw sensor reading into angle unit
	var angleConv;

	switch (unit) {
		case ASC.U_RAW:
			//Sensor raw measurement
			angleConv = angle;
			break;
		case ASC.U_TRN:
			//full turn ratio
			angleConv = (angle / R.AS5048B_RESOLUTION);
			break;
		case ASC.U_DEG:
			//degree
			angleConv = (angle / R.AS5048B_RESOLUTION) * 360.0;
			break;
		case ASC.U_RAD:
			//Radian
			angleConv = (angle / R.AS5048B_RESOLUTION) * 2 * Math.PI;
			break;
		case ASC.U_MOA:
			//minute of arc
			angleConv = (angle / R.AS5048B_RESOLUTION) * 60.0 * 360.0;
			break;
		case ASC.U_SOA:
			//second of arc
			angleConv = (angle / R.AS5048B_RESOLUTION) * 60.0 * 60.0 * 360.0;
			break;
		case ASC.U_GRAD:
			//grade
			angleConv = (angle / R.AS5048B_RESOLUTION) * 400.0;
			break;
		case ASC.U_MILNATO:
			//NATO MIL
			angleConv = (angle / R.AS5048B_RESOLUTION) * 6400.0;
			break;
		case ASC.U_MILSE:
			//Swedish MIL
			angleConv = (angle / R.AS5048B_RESOLUTION) * 6300.0;
			break;
		case ASC.U_MILRU:
			//Russian MIL
			angleConv = (angle / R.AS5048B_RESOLUTION) * 6000.0;
			break;
		default:
			//no conversion => raw angle
			angleConv = angle;
			break;
	}

	return angleConv;
}

// TODO: TEST!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// Read from register, 8 bits
AS5048B.prototype.readReg8 = function (reg, length) {
  length = length || 1;

  //8 bit value got from 1 8bits register
  this.i2c.writeTo(this.addr, reg);
  var d = this.i2c.readFrom(reg, length);
  console.log('readReg8:  ', d, d[0]);
  return d;
}

// Read from register, 16 bits consisting of
// most significant bits and least significant bits
AS5048B.prototype.readReg16 = function (reg, length) {
  length = length || 2;

	//16 bit value got from 2 8bits registers (7..0 MSB + 5..0 LSB) => 14 bits value
  this.i2c.writeTo(this.addr, reg);
  var d = this.i2c.readFrom(this.addr, length);

  // format MSB & LSB into signed int
  var i = d[0] << 6;
      i += (d[1] & 0x3F);

  return i;
}

// TODO: TEST!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// writes to a specific register within initialized address
AS5048B.prototype.writeReg = function (reg, data) {
  console.log('writeReg: data', reg, data);
  this.i2c.writeTo(this.addr, [reg].concat(data));
}

// ------------------------------------------------------


I2C1.setup({ scl: B6, sda: B7 });
var as = new AS5048B(I2C1);
