#include "haw/BMP180.h"
#include "math.h"

#define REG_CONTROL 0xF4
#define REG_RESULT 0xF6

#define TEMPERATURE 0x2E
#define PRESSURE0 0x34
#define PRESSURE1 0x74
#define PRESSURE2 0xB4
#define PRESSURE3 0xF4

static inline uint8_t readBytes(struct bmp180 *self, uint8_t *values, const int8_t length)
{
    self->config.ack = i2c_write_blocking(self->i2c.instance, self->i2c.address, &values[0], 1, true);

    if (self->config.ack != PICO_ERROR_GENERIC)
    {
        i2c_read_blocking(self->i2c.instance, self->i2c.address, values, length, false);
        return 1;
    }

    return 0;
}

static inline uint8_t readInt(struct bmp180 *self, const uint8_t address, int16_t *value)
{
    uint8_t data[2];
    data[0] = address;

    if (readBytes(self, data, 2))
    {
        *value = (((int16_t)data[0] << 8 | (int16_t)data[1]));
        return 1;
    }

    *value = 0;
    return 0;
}

static inline uint8_t readUInt(struct bmp180 *self, const uint8_t address, int16_t *value)
{
    uint8_t data[2];
    data[0] = address;

    if (readBytes(self, data, 2))
    {
        *value = (((uint16_t)data[0] << 8 | (uint16_t)data[1]));
        return true;
    }

    *value = 0;
    return false;
}

static inline uint8_t writeBytes(struct bmp180 *self, const uint8_t *values, const int8_t length)
{
    self->config.ack = i2c_write_blocking(self->i2c.instance, self->i2c.address, values, length, false);

    if (self->config.ack != PICO_ERROR_GENERIC)
    {
        return 1;
    }

    return 0;
}

static inline uint8_t startTemperature(struct bmp180 *self)
{
    uint8_t data[2];

    data[0] = REG_CONTROL;
    data[1] = TEMPERATURE;

    if (writeBytes(self, data, 2))
    {
        sleep_ms(5);
        return true;
    }

    return false;
}

static inline uint8_t startPressure(struct bmp180 *self)
{
    uint8_t data[2], delay;

    data[0] = REG_CONTROL;

    switch (self->config.oversampling)
    {
    case 0:
        data[1] = PRESSURE0;
        delay = 5;
        break;
    case 1:
        data[1] = PRESSURE1;
        delay = 8;
        break;
    case 2:
        data[1] = PRESSURE2;
        delay = 14;
        break;
    case 3:
        data[1] = PRESSURE3;
        delay = 26;
        break;
    default:
        data[1] = PRESSURE0;
        delay = 5;
        break;
    }

    if (writeBytes(self, data, 2))
    {
        sleep_ms(delay);
        return 1;
    }

    return 0;
}

struct bmp180 bmp180_init(i2c_inst_t *i2c_instance)
{
    struct bmp180 bmp;
    bmp.i2c.instance = i2c_instance;
    bmp.i2c.address = BMP180_ADDRESS;

    return bmp;
}

uint8_t bmp180_begin(struct bmp180 *self)
{
    double c3, c4, b1;

    if (readInt(self, 0xAA, &self->calibration_data.AC1) &&
        readInt(self, 0xAC, &self->calibration_data.AC2) &&
        readInt(self, 0xAE, &self->calibration_data.AC3) &&
        readUInt(self, 0xB0, &self->calibration_data.AC4) &&
        readUInt(self, 0xB2, &self->calibration_data.AC5) &&
        readUInt(self, 0xB4, &self->calibration_data.AC6) &&
        readInt(self, 0xB6, &self->calibration_data.VB1) &&
        readInt(self, 0xB8, &self->calibration_data.VB2) &&
        readInt(self, 0xBA, &self->calibration_data.MB) &&
        readInt(self, 0xBC, &self->calibration_data.MC) &&
        readInt(self, 0xBE, &self->calibration_data.MD))
    {
        c3 = 160.0 * pow(2, -15) * self->calibration_data.AC3;
        c4 = pow(10, -3) * pow(2, -15) * self->calibration_data.AC4;
        b1 = pow(160, 2) * pow(2, -30) * self->calibration_data.VB1;
        self->calibration_data.c5 = (pow(2, -15) / 160) * self->calibration_data.AC5;
        self->calibration_data.c6 = self->calibration_data.AC6;
        self->calibration_data.mc = (pow(2, 11) / pow(160, 2)) * self->calibration_data.MC;
        self->calibration_data.md = self->calibration_data.MD / 160.0;
        self->calibration_data.x0 = self->calibration_data.AC1;
        self->calibration_data.x1 = 160.0 * pow(2, -13) * self->calibration_data.AC2;
        self->calibration_data.x2 = pow(160, 2) * pow(2, -25) * self->calibration_data.VB2;
        self->calibration_data.y0 = c4 * pow(2, 15);
        self->calibration_data.y1 = c4 * c3;
        self->calibration_data.y2 = c4 * b1;
        self->calibration_data.p0 = (3791.0 - 8.0) / 1600.0;
        self->calibration_data.p1 = 1.0 - 7357.0 * pow(2, -20);
        self->calibration_data.p2 = 3038.0 * 100.0 * pow(2, -36);

        return 1;
    }

    return 0;
}

uint8_t bmp180_event(struct bmp180 *self)
{
    uint8_t data_received = 1;
    self->temperature = 0.0;
    self->pressure = 0.0;
    self->temperatureF = 0.0;
    self->seaLevel = 0.0;
    self->altitude = 0.0;

    if (self->config.measTemperature)
    {
        startTemperature(self);
        uint8_t data[2];
        bool result;

        data[0] = REG_RESULT;

        result = readBytes(self, data, 2);

        if (result)
        {
            self->rawTemperature = (data[0] * 256) + data[1];
        }
        else
        {
            data_received = 0;
        }
    }

    if (self->config.measPressure)
    {
        startPressure(self);
        uint8_t data[3];
        bool result;

        data[0] = REG_RESULT;

        result = readBytes(self, data, 3);

        if (result)
        {
            self->rawPressure = (data[0] * 256.0) + data[1] + (data[2] / 256.0);
        }
        else
        {
            data_received = 0;
        }
    }

    return data_received;
}

double bmp180_get_temperature_c(struct bmp180 *self)
{
    if (self->temperature != 0)
    {
        return self->temperature;
    }
    else if (self->temperature == 0 && self->rawTemperature != 0)
    {
        double a = self->calibration_data.c5 * (self->rawTemperature - self->calibration_data.c6);
        self->temperature = a + (self->calibration_data.mc / (a + self->calibration_data.md));
        return self->temperature;
    }

    return 0.0;
}

double bmp180_get_temperature_f(struct bmp180 *self)
{
    if (self->temperatureF != 0.0)
    {
        return self->temperatureF;
    }
    else if (bmp180_get_temperature_c(self) != 0.0)
    {
        self->temperatureF = (bmp180_get_temperature_c(self) * 1.8) + 32;
        return self->temperatureF;
    }

    return 0.0;
}

double bmp180_get_pressure(struct bmp180 *self)
{
    if (self->config.measTemperature == 0)
    {
        bmp180_set_temperature_measuring(self, 1);
        bmp180_event(self);
    }

    if (self->pressure != 0.0)
    {
        return self->pressure;
    }
    else if (self->pressure == 0.0 && self->rawPressure != 0)
    {
        double s, x, y, z;
        s = bmp180_get_temperature_c(self) - 25.0;
        x = (self->calibration_data.x2 * pow(s, 2)) + (self->calibration_data.x1 * s) + self->calibration_data.x0;
        y = (self->calibration_data.y2 * pow(s, 2)) + (self->calibration_data.y1 * s) + self->calibration_data.y0;
        z = (self->rawPressure - x) / y;
        self->pressure = (self->calibration_data.p2 * pow(z, 2)) + (self->calibration_data.p1 * z) + self->calibration_data.p0;
        return self->pressure;
    }

    return 0.0;
}

double bmp180_get_sea_level(struct bmp180 *self, double current_altitude)
{
    if (self->seaLevel != 0)
    {
        return self->seaLevel;
    }

    self->seaLevel = (bmp180_get_temperature_c(self) / pow(1 - (current_altitude / 44330.0), 5.255));
    return self->seaLevel;
}

double bmp180_get_altitude(struct bmp180 *self, double fixed_pressure)
{
    if (self->altitude != 0)
    {
        return self->altitude;
    }

    self->altitude = (44330.0 * (1 - pow(bmp180_get_pressure(self) / fixed_pressure, 1 / 5.255)));
    return self->altitude;
}

void bmp180_set_temperature_measuring(struct bmp180 *self, uint8_t state)
{
    self->config.measTemperature = state;
}

void bmp180_set_pressure_measuring(struct bmp180 *self, uint8_t state)
{
    self->config.measPressure = state;
}

void bmp180_set_pressure_sampling(struct bmp180 *self, uint8_t oversampling)
{
    self->config.oversampling = oversampling;
}

int bmp180_get_error(struct bmp180 *self)
{
    return self->config.ack;
}
