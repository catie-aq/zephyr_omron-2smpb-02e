# Omron 2SMPB 02E Sensor Driver

Omron 2SMPB 02E digital barometric pressure sensor Zephyr OS driver.

## Hardware requirements
- Zest Sensor P-T-RH
- Any Zest Core board

## Usage
This sensor driver can be used to read the pressure and temperature from the Omron 2SMPB 02E sensor.

### Build
Use the Zest Sensor P-T-RH shield:
```bash
west build -b <board> samples -- -DSHIELD=zest_sensor_p_t_rh
```

Alternatively:

You can use the Omron 2SMPB 02E sensor with a custom board.
- Make sur you have added the 2SMPB 02E node in your board devicetree.

```dts
&i2c {
    o2smpb02e@70 {
		compatible = "omron,2smpb-02e";
		reg = <0x70>;
	};
};
```

Build the example:
```bash
west build -b <board> samples
```
