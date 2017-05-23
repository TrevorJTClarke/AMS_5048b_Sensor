import { AS5048B } from './lib';
import { UNIT } from './lib/constants';

I2C1.setup({ scl: B6, sda: B7 })
var AS = new AS5048B(I2C1)
var angl = AS.angleRead(UNIT.DEG, true)
// console.log('zeroRegRead', as.zeroRegRead())
// console.log('setZeroPosition', as.setZeroPosition())

setInterval(() => {
  angl = as.angleRead(UNIT.DEG, true)
  console.log('angl:', angl)
}, 2000)

// Defaults
// TODO: calibrate totalSteps, full/half/quarter
const stepPin = B4
const dirPin = B3
const direction = true
const steps = {
  full: 408,
  half: 816,
  quarter: 1632
}

// NOTE:S
// - speed in is microseconds!! use 5ms or less
// - steps needs calibration!!
// - total time needed should be based on (speed * 2)
function runSteps(totalSteps = steps.full, speed = 1) {
  digitalWrite(dirPin, direction)

  for (let x = 0; x < totalSteps; x++) {
    digitalPulse(stepPin, 1, speed)
    digitalPulse(stepPin, 0, speed)
  }

  direction = !direction
}

// Control Flow
//
// Flow:
// 1. MCU - Sends initial call to send motor to position
// 2. Motor Driver - receives position, moves, returns promise
// 3. Sensor - verifies position, acts against invalid position
//    - If within margin, accept flow
//    - If outside margin, step down and correct
// 4. Repeat
