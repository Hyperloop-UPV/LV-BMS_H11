# NEED TO:

 - Find how to: Automatic over/undervoltage and temperature detection routable to fault pin

 - GPIOs

 - supports SPI and TPL communication, not sure what tpl is

 - baude rate?
 - NSS/chip select? por software, hardware, espera, timeouts?
 - 2 lanes -> miso & mosi [YES]
 - clock polarity? low/high
SPI active low chip select (page 8)

 - Clock phase -> 1edge/first edge or 2edge
 - endianness in comparison with microcontroller endianness
 - CRC? every 8 bits or every frame?
 - Chip select mode? pulse or not pulse
 - Chip select polarity?
 - waiting times?

page 22 -> SPI interface


/* Field VPRE_UV: VPRE undervoltage detection */
MC33772C_SYS_CFG2_VPRE_UV

/* Field CT1_OV_FLT (read-only): Contains the status of the overvoltage fault for the cell terminal. Register is updated with each internal and system controller on-demand conversion cycle. */
MC33772C_CELL_OV_FLT_CT1_OV_FLT

/* Field GPIO0_H (read-only): The GPIOx_H bits detects and latches the low to high transition occurring on the GPIOx input. */
MC33772C_GPIO_STS_GPIO0_H

/* Field AN0_UT (read-only): Undertemperature detection for ANx - ANx_UT ored is provided in FAULT1_STATUS[AN_UT_FLT]. */
MC33772C_AN_OT_UT_FLT_AN0_UT

/* Field AN0_OT (read-only): Overtemperature detection for ANx - ANx_OT ored is provided in FAULT1_STATUS[AN_OT_FLT]. */
MC33772C_AN_OT_UT_FLT_AN0_OT


Pin number | Pin name | Pin function | Definition
3          | FAULT    | Output       | Fault output dependent on user defined internal or external faults. 
           |          |              | If not used, it must be left open

//Vpwr(OV_FLAG) - Vpwr overvoltage fault threshold (flag)
//Vpwr(LV_FLAG)

fault detection -> activation of fault pin = 56us

puede ser esto? Oscillator fault monitoring
(OSC fault monitoring)

overvoltage undervoltage threshold register - TH_ALL_CT
fixed point??? where's the point?? some initial value?

Overtemperature, undertemperature threshold registers – TH_ANx_OT, TH_ANx_UT
