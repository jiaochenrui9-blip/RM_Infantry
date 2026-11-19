#ifndef POWER_CONFIG_H
#define POWER_CONFIG_H

/* 采样电阻按实物 1 mΩ，网表上的型号是 2 mΩ，以实物为准。 */
#define POWER_SHUNT_OHM         0.001f
#define POWER_I2C_ADDRESS       0x40U
#define POWER_UPDATE_PERIOD_MS  10U

/* 每计数 312.5 μA，覆盖该阻值在大压差档的量程；不是整板额定电流。 */
#define POWER_CURRENT_LSB_A     0.0003125f

#endif
