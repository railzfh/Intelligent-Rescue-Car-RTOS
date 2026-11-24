#include "3_Robot/Drive/straight_drive.h"

#include <cassert>
#include <cmath>
#include <cstdio>

static constexpr Struct_Straight_Drive_Config Config{0.05f, 1.0f, 5.236f, 5.236f, 0.9f};

static void Test_Distance_And_Stop()
{
    Class_Straight_Drive drive(Config, 40.0f, 40.0f);
    const std::array<float, 2> start{53.0f, -11.0f};
    const std::array<float, 2> stopped{};
    assert(drive.Start(start, stopped, 0.0f, 1000));
    assert(drive.Get_Distance_M() == 0.0f);
    drive.Update({start[0] + 5.0f, start[1] + 15.0f}, stopped, 0.0f, 0.0f, 1001);
    assert(std::fabs(drive.Get_Distance_M() - 0.5f) < 0.0001f);
    assert(drive.Is_Running());
    const auto braking = drive.Update({start[0] + 20.0f, start[1] + 20.0f},
                                      {2.0f, 2.0f}, 0.0f, 0.0f, 1002);
    assert(drive.Is_Running());
    assert(braking[0] < 0.0f && braking[1] < 0.0f);
    const auto estop = drive.Update(start, {2.0f, 2.0f}, 0.0f, 0.0f, 3002);
    assert(!drive.Is_Running());
    assert(estop[0] == 0.0f && estop[1] == 0.0f);
}

static void Test_Yaw_And_Timeout()
{
    const std::array<float, 2> stopped{};
    Class_Straight_Drive positive(Config, 40.0f, 40.0f);
    Class_Straight_Drive negative(Config, 40.0f, 40.0f);
    constexpr float start_heading = 6.28f;
    assert(positive.Start(stopped, stopped, start_heading, 0));
    assert(negative.Start(stopped, stopped, start_heading, 0));
    const auto left_turn = positive.Update(stopped, stopped, start_heading + 0.02f, 0.0f, 1);
    const auto right_turn = negative.Update(stopped, stopped, start_heading, -0.5f, 1);
    assert(left_turn[0] < left_turn[1]);
    assert(right_turn[0] > right_turn[1]);
    assert(positive.Is_Running() && negative.Is_Running());

    const auto heading_stop = positive.Update(stopped, stopped, start_heading + 0.45f, 0.0f, 2);
    assert(!positive.Is_Running());
    assert(heading_stop[0] == 0.0f && heading_stop[1] == 0.0f);
    const auto timeout = negative.Update(stopped, stopped, start_heading, 0.0f, 20000);
    assert(!negative.Is_Running());
    assert(timeout[0] == 0.0f && timeout[1] == 0.0f);

    auto unconfigured = Config;
    unconfigured.wheel_radius_m = 0.0f;
    Class_Straight_Drive missing_radius(unconfigured, 40.0f, 40.0f);
    assert(!missing_radius.Start(stopped, stopped, 0.0f, 0));
}

static void Test_Independent_Speed_Loops()
{
    Class_Wheel_LADRC right(40.0f);
    Class_Wheel_LADRC left(40.0f);
    right.Reset(0.0f);
    left.Reset(0.0f);
    assert(right.Update(0.0f, 20.0f, 0.9f) == 0.9f);
    assert(left.Update(0.0f, -20.0f, 0.9f) == -0.9f);
    right.Reset(0.0f);
    left.Reset(0.0f);

    // Different plant gains and constant loads exercise the two ESO states.
    float right_speed = 0.0f;
    float left_speed = 0.0f;
    for (int i = 0; i < 2500; ++i)
    {
        const float right_torque = right.Update(right_speed, 4.0f, 0.9f);
        const float left_torque = left.Update(left_speed, 4.0f, 0.9f);
        assert(std::fabs(right_torque) <= 0.9f && std::fabs(left_torque) <= 0.9f);
        right_speed += 0.001f * (32.0f * right_torque - 3.0f);
        left_speed += 0.001f * (48.0f * left_torque - 9.0f);
    }
    assert(std::fabs(right_speed - 4.0f) < 0.03f);
    assert(std::fabs(left_speed - 4.0f) < 0.03f);
}

static void Test_Approach_Target()
{
    auto config = Config;
    config.wheel_radius_m = 0.029f;
    Class_Straight_Drive drive(config, 40.0f, 40.0f);
    std::array<float, 2> speed{};
    std::array<float, 2> angle{};
    assert(drive.Start(angle, speed, 0.0f, 0));
    float cruise_speed = 0.0f;
    for (uint32_t ms = 1; ms < 10000 && drive.Is_Running(); ++ms)
    {
        const auto torque = drive.Update(angle, speed, 0.0f, 0.0f, ms);
        for (unsigned i = 0; i < 2; ++i)
        {
            angle[i] += speed[i] * 0.001f;
            speed[i] += 40.0f * torque[i] * 0.001f;
        }
        if (drive.Get_Distance_M() > 0.5f && drive.Get_Distance_M() < 0.6f)
        {
            cruise_speed = speed[0];
        }
    }
    assert(!drive.Is_Running());
    assert(drive.Get_Distance_M() > 0.97f && drive.Get_Distance_M() < 1.03f);
    assert(cruise_speed > 4.0f && speed[0] < 0.5f * cruise_speed);
}

int main()
{
    Test_Distance_And_Stop();
    Test_Yaw_And_Timeout();
    Test_Independent_Speed_Loops();
    Test_Approach_Target();
    std::puts("PASS: wheel LADRC, distance, yaw correction, deceleration and stop");
}
