EXAMPLE:
I2C1.setup({ scl: B6, sda: B7 });
var ams = require("AS5048B").connect(I2C1);


// Output
-read-
 chip 0x40 address 0x16
-read-
 chip 0x40 address 0xFE
-write-
 chip 0x40 address 0x16 value 0x0
-write-
 chip 0x40 address 0x17 value 0x0
-write-
 chip 0x40 address 0x16 value 0x6D
-write-
 chip 0x40 address 0x17 value 0x2E
Angle degree : -read-
 chip 0x40 address 0xFE
0.1098632812
Angle degree : -read-
 chip 0x40 address 0xFE
0.0878906250


// Guess Operations:
-read- (ZEROMSB_REG read zero)
 chip 0x40 address 0x16
-read- (ANGLMSB_REG read angle)
 chip 0x40 address 0xFE
-write- (Set ZEROMSB_REG as zero)
 chip 0x40 address 0x16 value 0x0
-write- (Set ZEROLSB_REG as zero)
 chip 0x40 address 0x17 value 0x0
-write- (Set ZEROMSB_REG as init value found)
 chip 0x40 address 0x16 value 0x6D
-write- (Set ZEROLSB_REG as init value found)
 chip 0x40 address 0x17 value 0x2E

// begin periodic reading
-read-
 chip 0x40 address 0xFE
Angle degree : 0.1098632812
-read-
 chip 0x40 address 0xFE
Angle degree : 0.0878906250
