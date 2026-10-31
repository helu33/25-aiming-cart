#include "odometry.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>

#define TEST_EPSILON (1.0e-5f)

static void assert_near(float actual, float expected)
{
    assert(fabsf(actual - expected) < TEST_EPSILON);
}

int main(void)
{
    odometry_context_t odom;
    int32_t x_mm;
    int32_t y_mm;
    int32_t yaw_mrad;

    odometry_init(&odom, 0.2f);

    assert(odometry_update(&odom, 0.1f, 0.1f, 1.0f) != 0U);
    assert_near(odom.x_m, 0.1f);
    assert_near(odom.y_m, 0.0f);
    assert_near(odom.yaw_rad, 0.0f);

    odometry_reset(&odom, 0.0f, 0.0f, 0.0f);
    assert(odometry_update(&odom, -0.1f, 0.1f, 1.0f) != 0U);
    assert_near(odom.x_m, 0.0f);
    assert_near(odom.y_m, 0.0f);
    assert_near(odom.yaw_rad, 1.0f);

    odometry_reset(&odom, 0.0f, 0.0f, 0.0f);
    assert(odometry_update(&odom, 0.0f, 0.3141592654f, 1.0f) != 0U);
    assert_near(odom.x_m, 0.1f);
    assert_near(odom.y_m, 0.1f);
    assert_near(odom.yaw_rad, 1.5707963268f);

    odometry_reset(&odom, 0.0f, 0.0f, 0.0f);
    assert(odometry_update_fused(&odom, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f) != 0U);
    assert_near(odom.x_m, 0.0f);
    assert_near(odom.y_m, 0.0f);
    assert_near(odom.yaw_rad, 1.0f);

    odometry_reset(&odom, 1.2346f, -0.5005f, 0.1236f);
    odometry_pose_fixed_get(&odom, &x_mm, &y_mm, &yaw_mrad);
    assert(x_mm == 1235);
    assert(y_mm == -501);
    assert(yaw_mrad == 124);

    return 0;
}
