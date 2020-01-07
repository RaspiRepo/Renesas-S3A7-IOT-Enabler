#include "environmental_sense.h"

#define ENS210_SENS_RUN             0x21

float     g_onboard_temperature = 0.0f;
float     g_indoor_temperature  = 0.0f;
float     g_indoor_humidity     = 0.0f;
int       g_light_distance      = 0;
uint16_t  g_indoor_co2_level    = 0;

uint8_t Ens210_slave_address = 0x43;
uint8_t ens210_data_regstr   =  0x30;
uint8_t ens210_data_buf[6]   = {0x0,0x0,0x0,0x0,0x0,0x0};

char lcd_disp_str[30] = "";
char g_air_qual_status[30] = "";


//Lighting sensor
uint8_t as3935_afe_reg     = 0x00;
uint8_t estimated_dist_reg = 0x07;

uint8_t afe_setting       = 0x00;
uint8_t est_distance      = 0x00; //default out of range
float g_lightning_miles   = 0.0f;


/******************************* READING PROXIMITY/Color SENSOR START ************
 * Color Light-to-Digital Converter with Proximity Sensing.
 * The TMD3782x device will perform color temperature measurement, ambient light sensing (ALS)
 *  and proximity detection with background light rejection.
 *
 *  The color sensing feature is useful in applications such as backlight control,
 *  solid state lighting, reflected LED color sampler, or fluorescent light color
 *  temperature detection. The integrated IR blocking filter makes this device an excellent ambient
 *  light sensor, color temperature monitor, and general purpose color sensor.
 *  http://www.ams.com/ICdirect.
 *
 *  Applications include:
    • Ambient Light Sensing
    • Color Temperature Sensing
    • Cell Phone Touch Screen Disable
    • Mechanical Switch Replacement
    • Industrial Process Control
    • Medical Diagnostics.

 *  READ/WRITE OPERATION USING I2C INTERFACE
 *  Interface and control are accomplished through an I²C serial
 *  compatible interface (standard or fast mode) to a set of registers
 *  that provide access to device control functions and output data.
 *  The devices support the 7-bit I²C addressing protocol.
 *  The I²C standard provides for three types of bus transaction:
 *  read, write, and a combined protocol (Figure 22). During a write
 *  operation, the first byte written is a command byte followed by
 *  data. In a combined protocol, the first byte written is the
 *  command byte followed by reading a series of bytes. If a read
 *  command is issued, the register address from the previous
 *  command will be used for data access. Likewise, if the MSB of
 *  the command is not set, the device will write a series of bytes
 *  at the address stored in the last valid command with a register
 *  address. The command byte contains either control information
 *  or a 5-bit register address. The control commands can also be
 *  used to clear interrupts.
 */

#define I2C_TMD3782_SLAVE_ADDRESS     0x39

#define Tmd3782_command              0x80
#define Tmd3782_enable_register      0x00
#define Tmd3782_prxy_pulse_register  0x0e
#define Tmd3782_rgbc_data_register   0x14
#define Tmd3782_pxmty_data_register  0x1c
#define Tmd3782_auto_inc_proto       0x20
#define proxmity_pulse_counts        0x08

uint8_t color_data_buf[8] = {0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0};
//RGBC Data Registers (0x14 - 0x1b)
uint8_t tmd3289_cmd_read_rgbc = (uint8_t)(Tmd3782_command | Tmd3782_auto_inc_proto | Tmd3782_rgbc_data_register);

//Proximity Data Registers (0x1c - 0x1d)
uint8_t tmd3289_cmd_read_proxymity = (uint8_t)((uint8_t)Tmd3782_command | Tmd3782_auto_inc_proto | Tmd3782_pxmty_data_register);

uint16_t g_clear  = 0;
uint16_t g_red    = 0;
uint16_t g_green  = 0;
uint16_t g_blue   = 0;
uint16_t g_proximity = 10;

void tmd3872_sensor_start ();
void tmd3782_sensor_reading ();

uint16_t g_back_colour;
uint16_t g_font_colour;
int16_t g_light_rgb565 = 0x0a31;


void error_wait_delay (ssp_err_t erro_code)
/*----------------------------------------------------------------------------
    error_wait_delay    : Threadx Sleep (context switch)

    Written By          : RaspiRepo (mariya.k@gmail.com)
                          https://github.com/RaspiRepo

    Address             : Mountain View, CA 94040
    Date                : May 2017
------------------------------------------------------------------------------*/
{
    if (TX_SUCCESS != erro_code) {
        while(1);
    }
}


void write_display (int line_num, char *msg_str)
/*----------------------------------------------------------------------------
    write_display_console: Dummy display function

    Written By          : RaspiRepo (mariya.k@gmail.com)
                          https://github.com/RaspiRepo

    Address             : Mountain View, CA 94040
    Date                : May 2017
------------------------------------------------------------------------------*/
{
//    BufferLine(line_num, msg_str);
}





void display_sensor_value()
/*----------------------------------------------------------------------------
    display_sensor_value: Construct display string and calls display driver
                          to display values on LCD panel

    Written By          : RaspiRepo (mariya.k@gmail.com)
                          https://github.com/RaspiRepo

    Address             : Mountain View, CA 94040
    Date                : May 2017
------------------------------------------------------------------------------*/
{
    snprintf(lcd_disp_str, sizeof(lcd_disp_str), "Temperature : %2.3f F", g_indoor_temperature);
    write_display(3, lcd_disp_str);

    snprintf(lcd_disp_str, sizeof(lcd_disp_str), "Humidity    : %2.3f %%", g_indoor_humidity);
    write_display(4, lcd_disp_str);

    snprintf(lcd_disp_str, sizeof(lcd_disp_str), "Co2 level   : %d ppm", g_indoor_co2_level);
    write_display(5, lcd_disp_str);

    snprintf(lcd_disp_str, sizeof(lcd_disp_str), "Proximity   : %d", g_proximity);
    write_display(6, lcd_disp_str);

    snprintf(lcd_disp_str, sizeof(lcd_disp_str), "RGB565      : 0x%x", g_light_rgb565);
    write_display(7, lcd_disp_str);

    snprintf(lcd_disp_str, sizeof(lcd_disp_str), "AFE Indoor  : 0x%x", afe_setting);
    if (afe_setting == 0x1c) {
        snprintf(lcd_disp_str, sizeof(lcd_disp_str), "AFE Outdoor : 0x%x", afe_setting);
    }
    write_display(9, lcd_disp_str);

    if (est_distance == 0x3f) {
        snprintf(lcd_disp_str, sizeof(lcd_disp_str), "Lightning   : UNKWN");
    } else {
        g_lightning_miles = est_distance * 0.621371f;
        snprintf(lcd_disp_str, sizeof(lcd_disp_str), "Lightning   : %2.0f Miles", g_lightning_miles);
    }
    //update LCD screen
//    PaintText();
}





void ens210_sensor_start ()
/*----------------------------------------------------------------------------
    ens210_sensor_start: Initialize AMS ENS210 I2C sensor to mesure temperature
                         and humidity

    Written By          : RaspiRepo (mariya.k@gmail.com)
                          https://github.com/RaspiRepo

    Address             : Mountain View, CA 94040
    Date                : May 2017
------------------------------------------------------------------------------*/
{
    ssp_err_t err;

    err = g_sf_i2c_ams_en210.p_api->open(g_sf_i2c_ams_en210.p_ctrl, g_sf_i2c_ams_en210.p_cfg);

    uint8_t ens210_init_seq[] = {ENS210_SENS_RUN, 0x3, 0x3 };

     //err = g_sf_i2c_ams_en210.p_api->write(g_sf_i2c_ams_en210.p_ctrl, &Ens210_slave_address, 1, true, TX_WAIT_FOREVER);
    err = g_sf_i2c_ams_en210.p_api->write(g_sf_i2c_ams_en210.p_ctrl, (uint8_t *)&ens210_init_seq,
                                          sizeof(ens210_init_seq), false, TX_WAIT_FOREVER);
    //error_wait_delay(err);
}



void ens210_sensor_reading ()
/*----------------------------------------------------------------------------
    ens210_sensor_reading: Collect AMS ENS210 I2C sensor  temperature Kelvin
                           and humidity

    Written By          : RaspiRepo (mariya.k@gmail.com)
                          https://github.com/RaspiRepo

    Address             : Mountain View, CA 94040
    Date                : May 2017
------------------------------------------------------------------------------*/
{
    ssp_err_t err;

    err = g_sf_i2c_ams_en210.p_api->write(g_sf_i2c_ams_en210.p_ctrl, &ens210_data_regstr,
                                          sizeof(ens210_data_regstr), false, TX_WAIT_FOREVER);
    //error_wait_delay(err);

    err = g_sf_i2c_ams_en210.p_api->read(g_sf_i2c_ams_en210.p_ctrl, (uint8_t*)ens210_data_buf, 6, false, TX_WAIT_FOREVER);
    //error_wait_delay(err);

    int32_t t_val = (ens210_data_buf[2]<<16) + (ens210_data_buf[1]<<8) + (ens210_data_buf[0]<<0);
    int32_t h_val = (ens210_data_buf[5]<<16) + (ens210_data_buf[4]<<8) + (ens210_data_buf[3]<<0);

    int32_t t_data = (t_val>>0 ) & 0xffff;

    float TinK = (float)t_data / 64; // Temperature in Kelvin
    float TinC = (float)(TinK - 273.15f); // Temperature in Celsius
    g_indoor_temperature = (float)(TinC * 1.8f + 32.0f); // Temperature in Fahrenheit

    uint32_t h_data = (h_val>>0 ) & 0xffff;

    /*
    uint32_t h_valid= (h_val>>16) & 0x1;
    uint32_t h_crc = (h_val>>17) & 0x7f;
    uint32_t h_payl = (h_val>>0 ) & 0x1ffff;
    */
    g_indoor_humidity = (float)((float)h_data/512.0f); // relative humidity (in %)
}


void aiq_core_sensor_start ()
/*----------------------------------------------------------------------------
    aiq_core_sensor_start: Initialize AirQuaity Sensor

    Written By          : RaspiRepo (mariya.k@gmail.com)
                          https://github.com/RaspiRepo

    Address             : Mountain View, CA 94040
    Date                : May 2017
------------------------------------------------------------------------------*/
{
    ssp_err_t err = TX_SUCCESS;

    //open device interface instance
    err = g_sf_i2c_device_aiq_core.p_api->open(g_sf_i2c_device_aiq_core.p_ctrl, g_sf_i2c_device_aiq_core.p_cfg);
    error_wait_delay(err);
}


void aiq_sensor_reading ()
/*----------------------------------------------------------------------------
    aiq_sensor_reading  : Read current value from  AirQuaity Sensor TMD3782,
                          from cold boot takes at least 5 minutes for first
                          valid reading

    Written By          : RaspiRepo (mariya.k@gmail.com)
                          https://github.com/RaspiRepo

    Address             : Mountain View, CA 94040
    Date                : May 2017
------------------------------------------------------------------------------*/
{
    ssp_err_t err = TX_SUCCESS;

    uint8_t aiq_data_buf[] = {0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0};

    //Read 8 bytes. for each channel 2 bytes (16bi) from sensor (lower byte and higher byte)
    //Refer AMS TMD3782 Data sheet page 27
    //Clear, red, green, and blue data is stored as 16-bit values.
    err = g_sf_i2c_device_aiq_core.p_api->read(g_sf_i2c_device_aiq_core.p_ctrl, (uint8_t*)aiq_data_buf,
                                               sizeof(aiq_data_buf), false, TX_WAIT_FOREVER);
    uint32_t resistance;
    uint8_t  status = aiq_data_buf[3];
    switch (status) {
        case 0x00:
            g_indoor_co2_level = (uint16_t) ((aiq_data_buf[0] << 8) + aiq_data_buf[1]);
            resistance = (aiq_data_buf[4] << 16) | (aiq_data_buf[5] << 8) | aiq_data_buf[6];
            break;
        case 0x10:
            snprintf(g_air_qual_status, sizeof(g_air_qual_status), "RUNNING");
            break;
        case 0x01:
            snprintf(g_air_qual_status, sizeof(g_air_qual_status), "BUSY..");
            break;
        case 0x80:
            snprintf(g_air_qual_status, sizeof(g_air_qual_status), "ERROR.");
            break;
        default:
            snprintf(g_air_qual_status, sizeof(g_air_qual_status), "FATAL.");
            break;
    }

//    if (status == 0) {
//        snprintf(lcd_disp_str, sizeof(lcd_disp_str), "Co2: %d ppm", g_indoor_co2_level);
//        write_display(6, lcd_disp_str);
//    }
}


void lightning_sensor_start ()
/*----------------------------------------------------------------------------
    lightning_sensor_start:
                          Initialize Lightning Sensor AS3935

    Written By          : RaspiRepo (mariya.k@gmail.com)
                          https://github.com/RaspiRepo

    Address             : Mountain View, CA 94040
    Date                : May 2017
------------------------------------------------------------------------------*/
{
    ssp_err_t err = TX_SUCCESS;

    uint8_t as3935_start_reset[] = {0x3c, 0x96, 0x96};

    err = g_sf_i2c_lightning.p_api->open(g_sf_i2c_lightning.p_ctrl, g_sf_i2c_lightning.p_cfg);
    err = g_sf_i2c_lightning.p_api->write(g_sf_i2c_lightning.p_ctrl, (uint8_t *)&as3935_start_reset,
                                         sizeof(as3935_start_reset), false, TX_WAIT_FOREVER);
}




void lightning_sensor_reading ()
/*----------------------------------------------------------------------------
    lightning5_sensor_reading:
                          AS3935 Franklin Lightning Sensor IC: Lightning sensor
                          warns of lightning storm activity within a radius
                          of 40km

    Written By          : RaspiRepo (mariya.k@gmail.com)
                          https://github.com/RaspiRepo

    Address             : Mountain View, CA 94040
    Date                : May 2017
------------------------------------------------------------------------------*/
{
    ssp_err_t err = TX_SUCCESS;
    afe_setting = 0;

    //Issue read command
    err = g_sf_i2c_lightning.p_api->write(g_sf_i2c_lightning.p_ctrl, (uint8_t *)&as3935_afe_reg,
                                               sizeof(as3935_afe_reg), false, TX_WAIT_FOREVER);


    err = g_sf_i2c_lightning.p_api->read(g_sf_i2c_lightning.p_ctrl, (uint8_t*)&afe_setting, sizeof(afe_setting),
                                       false, TX_WAIT_FOREVER);
    if (TX_SUCCESS == err) {
        //REG0x00[5:1]   - 0x24 - means indoor :  10010
        //Outdoor 01110  - 0x1c = means outdoor
        afe_setting = afe_setting & 0x3E;
    }

    //Statistical Distance Estimation:
    //The AS3935 generates an assessment of the estimated distance to the head of an approaching storm. This assessment is done based on
    //statistical calculation. The energy of the single event (lightning) provided by the Energy Calculation block is stored in an internal memory,
    //together with timing information, in the AS3935. The events stored in the memory are then correlated with a look-up table by the statistical
    //distance estimation block, which provides a final estimation of the distance to the head of the storm. The algorithm automatically deletes events,
    //which are older than a certain time. R7=0x01 means that the storm is right overhead, while R7=0x3F is displayed when the storm is out of range.
    //This algorithm is hardwired and not accessible from outside.
    //The estimated distance is directly represented in km in the register REG0x07[5:0] (binary encoded). The distance estimation can change also if
    //no new event triggers the AS3935, as older events can be purged.
    err = g_sf_i2c_lightning.p_api->write(g_sf_i2c_lightning.p_ctrl, (uint8_t *)&estimated_dist_reg,
                                               sizeof(estimated_dist_reg), false, TX_WAIT_FOREVER);

    err = g_sf_i2c_lightning.p_api->read(g_sf_i2c_lightning.p_ctrl, (uint8_t*)&est_distance, sizeof(est_distance),
                                       false, TX_WAIT_FOREVER);
    if (TX_SUCCESS == err) {
        //use only bits[5:0]
        est_distance = est_distance & 0x3F;
    }
}



void proximity_color_sensor_start ()
/*----------------------------------------------------------------------------
    proximity_color_sensor_start:
                          Initialize Proximity and Color Sensor tmd3289

    Written By          : RaspiRepo (mariya.k@gmail.com)
                          https://github.com/RaspiRepo

    Address             : Mountain View, CA 94040
    Date                : May 2017
------------------------------------------------------------------------------*/
{
    ssp_err_t err = TX_SUCCESS;

    err = g_sf_i2c_proximity.p_api->open(g_sf_i2c_proximity.p_ctrl, g_sf_i2c_proximity.p_cfg);

    //During a write operation, the first byte written is a command byte followed by data.
    // data 0x07 : 0000 0111
    // bit 2 (1) - Proximity enable
    // bit 1 (1) - ADC enable. This bit activates the four-channel (RGBC)
    // bit 0 (1) - Power ON. This bit activates the internal oscillator to permit the timers and
    //             ADC channels to operate.
    uint8_t tmd3289_init_seq[] = {(Tmd3782_command | Tmd3782_enable_register), 0x07};

    //Issue start command for RGBC
    err = g_sf_i2c_proximity.p_api->write(g_sf_i2c_proximity.p_ctrl, (uint8_t *)&tmd3289_init_seq,
                                               sizeof(tmd3289_init_seq), false, TX_WAIT_FOREVER);
    //0x0E PPULSE R/W Proximity pulse count set Tmd3782_prxy_pulse_countregister
    //The Proximity Pulse Count Register sets the number of
    //proximity pulses that will be transmitted.
    uint8_t tmd3289_init_seq2[] = {(Tmd3782_command | Tmd3782_prxy_pulse_register), proxmity_pulse_counts};

    //start command for proximity
    err = g_sf_i2c_proximity.p_api->write(g_sf_i2c_proximity.p_ctrl, (uint8_t *)&tmd3289_init_seq2,
                                               sizeof(tmd3289_init_seq2), false, TX_WAIT_FOREVER);
}



void proximity_color_sensorr_reading ()
/*----------------------------------------------------------------------------
    proximity_color_sensorr_reading:
                          Read current value of Proximity and Color value from
                          tmd3289

    Written By          : RaspiRepo (mariya.k@gmail.com)
                          https://github.com/RaspiRepo

    Address             : Mountain View, CA 94040
    Date                : May 2017
------------------------------------------------------------------------------*/
{
    ssp_err_t err = TX_SUCCESS;

    //Issue read command
    err = g_sf_i2c_proximity.p_api->write(g_sf_i2c_proximity.p_ctrl, (uint8_t *)&tmd3289_cmd_read_rgbc,
                                               sizeof(tmd3289_cmd_read_rgbc), false, TX_WAIT_FOREVER);

    //Read 8 bytes. for each channel 2 bytes (16bi) from sensor (lower byte and higher byte)
    //Refer AMS TMD3782 Data sheet page 27
    //Clear, red, green, and blue data is stored as 16-bit values.
    err = g_sf_i2c_proximity.p_api->read(g_sf_i2c_proximity.p_ctrl, (uint8_t*)color_data_buf, sizeof(color_data_buf),
                                       false, TX_WAIT_FOREVER);
    /*
    //process RGB clear channel bytes RGBC Data Registers (0x14 - 0x1b)
    g_clear = (uint16_t) ((color_data_buf[1] << 8) + color_data_buf[0]);
    g_red   = (uint16_t) ((color_data_buf[3] << 8) + color_data_buf[2]);
    g_green = (uint16_t) ((color_data_buf[5] << 8) + color_data_buf[4]);
    g_blue  = (uint16_t) ((color_data_buf[7] << 8) + color_data_buf[6]);

    err = g_sf_i2c_proximity.p_api->write(g_sf_i2c_proximity.p_ctrl, (uint8_t *)&tmd3289_cmd_read_proxymity,
                                               sizeof(tmd3289_cmd_read_proxymity), false, TX_WAIT_FOREVER);


    color_samples[samp_idx++] = (((uint8_t)g_red & 0x1f) << 11) | (((uint8_t)g_green & 0x3f) << 6) | ((uint8_t)g_blue) & 0x1f;
    */

    g_clear = (uint16_t) ((color_data_buf[1] << 8) | color_data_buf[0]);
    g_red   = (uint16_t) ((color_data_buf[3] << 8) | color_data_buf[2]);
    g_green = (uint16_t) ((color_data_buf[5] << 8) | color_data_buf[4]);
    g_blue  = (uint16_t) ((color_data_buf[7] << 8) | color_data_buf[6]);

   /* g_red   = color_data_buf[2];
    g_green = color_data_buf[4];
    g_blue  = color_data_buf[6];
    */
    g_light_rgb565 = (g_red << 11) | (g_green & 0x3f << 5) | (g_blue & 0x1f);

    err = g_sf_i2c_proximity.p_api->read(g_sf_i2c_proximity.p_ctrl, (uint8_t *)&g_proximity,
                                              sizeof(g_proximity), false, TX_WAIT_FOREVER);
}



/* Environmental Thread entry function */
void environmental_sense_entry(void)
/*----------------------------------------------------------------------------
    environmental_sense_entry:
                          Environmental sensor reading process

    Written By          : RaspiRepo (mariya.k@gmail.com)
                          https://github.com/RaspiRepo

    Address             : Mountain View, CA 94040
    Date                : May 2017
------------------------------------------------------------------------------*/
{
    /* LED type structure */
    bsp_leds_t leds;
    /* LED state variable */
    ioport_level_t level = IOPORT_LEVEL_HIGH;

    /* Get LED information for this board */
    R_BSP_LedsGet(&leds);

    ens210_sensor_start();
    aiq_core_sensor_start();
    lightning_sensor_start();
    proximity_color_sensor_start();

    while (1) {
        ens210_sensor_reading();
        aiq_sensor_reading();
        display_sensor_value();
        lightning_sensor_reading();
        proximity_color_sensorr_reading();
        if (0 < leds.led_count) {
            if(IOPORT_LEVEL_LOW == level) {
                level = IOPORT_LEVEL_HIGH;
            } else {
                level = IOPORT_LEVEL_LOW;
            }
            g_ioport.p_api->pinWrite(leds.p_leds[2], level);
        }
        tx_thread_sleep (10);
    }
}
