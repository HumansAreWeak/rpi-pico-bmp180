#include "haw/BMP180.h"
#include "math.h"

#define REG_CONTROL 0xF4
#define REG_RESULT 0xF6

#define TEMPERATURE 0x2E
#define PRESSURE0 0x34
#define PRESSURE1 0x74
#define PRESSURE2 0xB4
#define PRESSURE3 0xF4

static inline uint8_t readBytes(BMP180 *self, uint8_t *values, const int8_t length)
{
    self->ack = i2c_write_blocking(self->instance, self->address, &values[0], 1, true);

    if (self->ack != PICO_ERROR_GENERIC)
    {
        i2c_read_blocking(self->instance, self->address, values, length, false);
        return 1;
    }

    return 0;
}

static inline uint8_t readInt(BMP180 *self, const uint8_t address, int16_t *value)
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

static inline uint8_t readUInt(BMP180 *self, const uint8_t address, int16_t *value)
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

static inline uint8_t writeBytes(BMP180 *self, const uint8_t *values, const int8_t length)
{
    self->ack = i2c_write_blocking(self->instance, self->address, values, length, false);

    if (self->ack != PICO_ERROR_GENERIC)
    {
        return 1;
    }

    return 0;
}

static inline uint8_t startTemperature(BMP180 *self)
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

static inline uint8_t startPressure(BMP180 *self)
{
    uint8_t data[2], delay;

    data[0] = REG_CONTROL;

    switch (self->oversampling)
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

BMP180 bmp180_init(i2c_inst_t *i2c_instance)
{
    BMP180 bmp;
    bmp.instance = i2c_instance;
    bmp.address = 0x77;

    return bmp;
}

uint8_t bmp180_begin(BMP180 *self)
{
    double c3, c4, b1;

    if (readInt(self, 0xAA, &self->AC1) && readInt(self, 0xAC, &self->AC2) && readInt(self, 0xAE, &self->AC3) && readUInt(self, 0xB0, &self->AC4) && readUInt(self, 0xB2, &self->AC5) && readUInt(self, 0xB4, &self->AC6) && readInt(self, 0xB6, &self->VB1) && readInt(self, 0xB8, &self->VB2) && readInt(self, 0xBA, &self->MB) && readInt(self, 0xBC, &self->MC) && readInt(self, 0xBE, &self->MD))
    {
        c3 = 160.0 * pow(2, -15) * self->AC3;
        c4 = pow(10, -3) * pow(2, -15) * self->AC4;
        b1 = pow(160, 2) * pow(2, -30) * self->VB1;
        self->c5 = (pow(2, -15) / 160) * self->AC5;
        self->c6 = self->AC6;
        self->mc = (pow(2, 11) / pow(160, 2)) * self->MC;
        self->md = self->MD / 160.0;
        self->x0 = self->AC1;
        self->x1 = 160.0 * pow(2, -13) * self->AC2;
        self->x2 = pow(160, 2) * pow(2, -25) * self->VB2;
        self->y0 = c4 * pow(2, 15);
        self->y1 = c4 * c3;
        self->y2 = c4 * b1;
        self->p0 = (3791.0 - 8.0) / 1600.0;
        self->p1 = 1.0 - 7357.0 * pow(2, -20);
        self->p2 = 3038.0 * 100.0 * pow(2, -36);

        return 1;
    }

    return 0;
}

uint8_t bmp180_event(BMP180 *self)
{
    uint8_t data_received = 1;
    self->temperature = 0.0;
    self->pressure = 0.0;
    self->temperatureF = 0.0;
    self->seaLevel = 0.0;
    self->altitude = 0.0;

    if (self->measTemperature)
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

    if (self->measPressure)
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

double bmp180_get_temperature_c(BMP180 *self)
{
    if (self->temperature != 0)
    {
        return self->temperature;
    }
    else if (self->temperature == 0 && self->rawTemperature != 0)
    {
        double a = self->c5 * (self->rawTemperature - self->c6);
        self->temperature = a + (self->mc / (a + self->md));
        return self->temperature;
    }

    return 0.0;
}

double bmp180_get_temperature_f(BMP180 *self)
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

double bmp180_get_pressure(BMP180 *self)
{
    if (self->measTemperature == 0)
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
        x = (self->x2 * pow(s, 2)) + (self->x1 * s) + self->x0;
        y = (self->y2 * pow(s, 2)) + (self->y1 * s) + self->y0;
        z = (self->rawPressure - x) / y;
        self->pressure = (self->p2 * pow(z, 2)) + (self->p1 * z) + self->p0;
        return self->pressure;
    }

    return 0.0;
}

double bmp180_get_sea_level(BMP180 *self, double current_altitude)
{
    if (self->seaLevel != 0)
    {
        return self->seaLevel;
    }

    self->seaLevel = (bmp180_get_temperature_c(self) / pow(1 - (current_altitude / 44330.0), 5.255));
    return self->seaLevel;
}

double bmp180_get_altitude(BMP180 *self, double fixed_pressure)
{
    if (self->altitude != 0)
    {
        return self->altitude;
    }

    self->altitude = (44330.0 * (1 - pow(bmp180_get_pressure(self) / fixed_pressure, 1 / 5.255)));
    return self->altitude;
}

void bmp180_set_temperature_measuring(BMP180 *self, uint8_t state)
{
    self->measTemperature = state;
}

void bmp180_set_pressure_measuring(BMP180 *self, uint8_t state)
{
    self->measPressure = state;
}

void bmp180_set_pressure_sampling(BMP180 *self, uint8_t oversampling)
{
    self->oversampling = oversampling;
}

int bmp180_get_error(BMP180 *self)
{
    return self->ack;
}
