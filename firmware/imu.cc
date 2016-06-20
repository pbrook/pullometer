#include "imu.h"
#include "debugf.h"

#include <Eigen/Eigen>
#include <math.h>

using namespace Eigen;

static void
i2c_wait()
{
    while ((I2C0->S & I2C_S_IICIF_MASK) == 0)
	/* no-op */;
    I2C0->S |= I2C_S_IICIF_MASK;
    while ((I2C0->S & I2C_S_TCF_MASK) == 0)
	/* no-op */;
}

static void
i2c_do_addr(uint8_t addr)
{
    if (I2C0->S & I2C_S_BUSY_MASK) {
	I2C0->C1 |= I2C_C1_RSTA_MASK;
    } else {
	I2C0->C1 |= I2C_C1_MST_MASK;
	I2C0->C1 |= I2C_C1_TX_MASK;
    }
    I2C0->D = addr;
    i2c_wait();
}

static void
i2c_write(uint8_t addr, const uint8_t *data, int len, bool stop)
{
    i2c_do_addr(addr << 1);
    while (len != 0) {
	I2C0->D = *data;
	data++;
	len--;
	i2c_wait();
    }
    if (stop) {
	I2C0->C1 &= ~I2C_C1_MST_MASK;
    }
}

static void
i2c_read(uint8_t addr, uint8_t *data, int len)
{
    bool dummy;
    i2c_do_addr((addr << 1) | 1);
    I2C0->C1 &= ~I2C_C1_TX_MASK;
    // Discard the first byte read to flush the data register.
    dummy = true;
    len++;
    while (len != 0) {
	len--;
	if (len == 1) {
	    // Indicate last byte by suppressing ACK.
	    I2C0->C1 |= I2C_C1_TXAK_MASK;
	} else if (len == 0) {
	    // All bytes transferred, send STOP.
	    I2C0->C1 &= ~(I2C_C1_MST_MASK | I2C_C1_TXAK_MASK);
	}
	*data = I2C0->D;
	if (len == 0)
            return;
	i2c_wait();
	if (dummy)
            dummy = false;
	else
            data++;
    }
}

uint8_t
i2c_slave::read_reg(uint8_t reg)
{
    uint8_t val;

    i2c_write(addr, &reg, 1, false);
    val = 0xff;
    i2c_read(addr, &val, 1);
    return val;
}

void
i2c_slave::read_block(uint8_t reg, uint8_t len, uint8_t *data)
{
    i2c_write(addr, &reg, 1, false);
    i2c_read(addr, data, len);
}

void
i2c_slave::write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2];

    buf[0] = reg;
    buf[1] = val;
    i2c_write(addr, buf, 2, true);
}

static inline void
i2c_delay()
{
    delay_us(10);
}

static void
i2c_debrick(void)
{
    int n;
    PORTB->PCR[3] = PORT_PCR_MUX(1);
    PORTB->PCR[4] = PORT_PCR_MUX(1);

    FPTB->PCOR = _BV(3) | _BV(4);
    // Output clocks until data line released.
    while ((FPTB->PDIR & _BV(4)) == 0) {
	FPTB->PDDR |= _BV(3);
	i2c_delay();
	FPTB->PDDR &= ~_BV(3);
	i2c_delay();
    }
    // Generate STOP condition
    FPTB->PDDR |= _BV(4);
    i2c_delay();
    FPTB->PDDR &= ~_BV(4);
    i2c_delay();

    PORTB->PCR[3] = PORT_PCR_MUX(2);
    PORTB->PCR[4] = PORT_PCR_MUX(2);
}

#define ACCEL_FACTOR 0.01

// Threshold for auto-calibration (rad/s)
#define GYRO_THRESHOLD 1.0f
#define GYRO_THRESHOLD2 (GYRO_THRESHOLD * GYRO_THRESHOLD)
#define SLOW_THRESHOLD (GYRO_THRESHOLD / 4)

#define AUTOCAL_FACTOR 0.001
#define MISALIGN_THRESHOLD 5

static float max_speed; // Actually speed squared
static int swing_count;
static Vector3f max_accel;
static Vector3f max_gyro;
float attitude;

static float accel_angle;
static float gyro_vel;

// project 3D vectors into 2D rotational plane
class RotationTransform
{
public:
    // Unit vecor along axis of rotation
    Vector3f rot_axis;
    // Used to determine x axis
    Vector3f gravity;
    // Unit vectors along axes in 2D rotational plane
    Vector3f x_axis;
    Vector3f y_axis;
    int misalign_count;
    bool calibrated;

    RotationTransform() {};

    void recalibrate();

    // Project a vector
    Vector2f project(const Vector3f &vec)
    {
	float x = x_axis.dot(vec);
	float y = y_axis.dot(vec);
	return Vector2f(x, y);
    }
};

RotationTransform rt;

void
RotationTransform::recalibrate()
{
    // Assume that maximum rotational velocity coincides with bottom-dead-center.
    Vector3f vec = max_accel.normalized();
    bool misaligned = false;

    if (calibrated) {
        if (vec.dot(gravity) < 0.5)
            misaligned = true;
        if (misalign_count < MISALIGN_THRESHOLD) {
            vec = gravity * (1 - AUTOCAL_FACTOR) + vec * AUTOCAL_FACTOR;
            vec.normalize();
        }
    }
    gravity = vec;

    vec = max_gyro.normalized();
    if (calibrated) {
	float dot = vec.dot(rot_axis);
	if (dot < 0) {
            vec = -vec;
            dot = -dot;
	}
        if (dot < 0.5)
            misaligned = true;
        if (misalign_count < MISALIGN_THRESHOLD) {
            vec = rot_axis * (1 - AUTOCAL_FACTOR) + vec * AUTOCAL_FACTOR;
            vec.normalize();
        }
    }
    rot_axis = vec;

    if (misaligned) {
        misalign_count++;
    } else {
        misalign_count = 0;
    }
    
    // Gravity should ideally be perpendicular to rotation.
    // Project it into the plane of rotation just in case.
    x_axis = gravity - rot_axis * rot_axis.dot(gravity);
    x_axis.normalize();
    y_axis = x_axis.cross(rot_axis);

    calibrated = true;
}

static Vector3f raw_accel;
static Vector3f raw_gyro;

void
imu_set_accel(float x, float y, float z)
{
    Vector2f v2;

    raw_accel = Vector3f(x, y, z);
    if (rt.calibrated) {
	v2 = rt.project(raw_accel);
	accel_angle = atan2f(v2.y(), v2.x());
    }
}

void
imu_set_gyro(float x, float y, float z)
{
    float s2;
    raw_gyro = Vector3f(x, y, z);

    s2 = raw_gyro.squaredNorm();
    if (s2 > GYRO_THRESHOLD2) {
        if (s2 > max_speed) {
            max_speed = s2;
            max_gyro = raw_gyro;
            max_accel = raw_accel;
        }
        swing_count++;
    }

    if (rt.calibrated) {
	gyro_vel = rt.rot_axis.dot(raw_gyro);
    } else {
	gyro_vel = sqrtf(s2);
    }
}

extern int gyro_z;

#define DT (0.001f * IMU_TICK_MS)
void
imu_tick(void)
{
    float err;
    static float old_gyro;

    // automatic calibration
    if (fabsf(gyro_vel) < SLOW_THRESHOLD) {
        // Ignore very short pulses
        if (swing_count > 10) {
            rt.recalibrate();
        }
	max_speed = 0;
        swing_count = 0;
    }
    if (!rt.calibrated) {
	return;
    }

    err = accel_angle - attitude;
    if (err > M_PI)
	err -= 2.0f * M_PI;
    else if (err < -M_PI)
	err += 2.0f * M_PI;
    attitude += ((gyro_vel + old_gyro) * 0.5 * DT) * (1.0f - ACCEL_FACTOR) + (err * ACCEL_FACTOR);
    old_gyro = gyro_vel;

    if (attitude > M_PI * 1.5f)
      attitude -= 2.0f * M_PI;
    if (attitude < -M_PI * 1.5f)
      attitude += 2.0f * M_PI;
#if 0
    static int n;
    if (++n >= 10) {
        debugf("filter %4d, accel %4d, gyro %6f %4d\n",
            (int)(attitude * 180 / M_PI),
            (int)(accel_angle * 180 / M_PI),
            (gyro_vel * 180 / M_PI),
            (gyro_z * 2000) / 0x8000);
        n = 0;
    }
#endif
}

void
init_imu(void)
{
    SIM->SCGC4 |= SIM_SCGC4_I2C0_MASK;
    i2c_debrick();
    I2C0->F = 0;
    I2C0->C1 = I2C_C1_IICEN_MASK;
}
