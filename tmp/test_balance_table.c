/* 独立验证 BallControl_InterpolateHoldPwm 的查表输出（与 main.c 中实现一致）。 */
#include <stdint.h>
#include <stdio.h>

typedef struct
{
    int16_t target_x_px;
    uint16_t hold_pwm_us;
} BallBalancePoint;

static const BallBalancePoint g_ball_balance_table[] =
{
    {  95, 1490 },   /* -9 cm */
    { 168, 1470 },   /* -6 cm */
    { 212, 1443 },   /* -4 cm, 2026-08-01 static test verified */
    { 240, 1410 },   /* -3 cm */
    { 314, 1385 },   /*  0 cm */
    { 391, 1360 },   /* +3 cm */
    { 415, 1350 },   /* +4 cm, 2026-08-01 real-device test verified */
    { 470, 1325 },   /* +6 cm */
    { 542, 1295 },   /* +9 cm */
};

static int32_t AppRoundFloatToI32(float value)
{
    if (value >= 0.0f)
    {
        return (int32_t)(value + 0.5f);
    }
    return (int32_t)(value - 0.5f);
}

static uint16_t BallControl_InterpolateHoldPwm(int16_t target_x_px)
{
    uint32_t n = sizeof(g_ball_balance_table) / sizeof(g_ball_balance_table[0]);
    uint32_t i;

    if (target_x_px <= g_ball_balance_table[0].target_x_px)
    {
        return g_ball_balance_table[0].hold_pwm_us;
    }
    if (target_x_px >= g_ball_balance_table[n - 1U].target_x_px)
    {
        return g_ball_balance_table[n - 1U].hold_pwm_us;
    }

    for (i = 0U; i + 1U < n; i++)
    {
        int32_t x0 = g_ball_balance_table[i].target_x_px;
        int32_t x1 = g_ball_balance_table[i + 1U].target_x_px;

        if ((target_x_px >= x0) && (target_x_px <= x1))
        {
            int32_t pwm0 = g_ball_balance_table[i].hold_pwm_us;
            int32_t pwm1 = g_ball_balance_table[i + 1U].hold_pwm_us;
            float hold =
                (float)pwm0 +
                (float)(pwm1 - pwm0) *
                ((float)(target_x_px - x0) / (float)(x1 - x0));
            return (uint16_t)AppRoundFloatToI32(hold);
        }
    }

    return g_ball_balance_table[n - 1U].hold_pwm_us;
}

int main(void)
{
    /* 必查节点 */
    const int16_t nodes[] = {95, 168, 212, 240, 314, 391, 415, 470, 542};
    const uint16_t expect[] = {1490, 1470, 1443, 1410, 1385, 1360, 1350, 1325, 1295};
    int i;
    int fails = 0;

    for (i = 0; i < 9; i++)
    {
        uint16_t got = BallControl_InterpolateHoldPwm(nodes[i]);
        int ok = (got == expect[i]);
        printf("x=%3d -> %u (expect %u) %s\n", nodes[i], got, expect[i], ok ? "OK" : "FAIL");
        if (!ok)
        {
            fails++;
        }
    }

    /* 区间外 */
    {
        uint16_t lo = BallControl_InterpolateHoldPwm(-50);   /* x<95 */
        uint16_t hi = BallControl_InterpolateHoldPwm(600);   /* x>542 */
        printf("x=-50 -> %u (expect 1490) %s\n", lo, lo == 1490 ? "OK" : "FAIL");
        printf("x=600 -> %u (expect 1295) %s\n", hi, hi == 1295 ? "OK" : "FAIL");
        if (lo != 1490 || hi != 1295)
        {
            fails++;
        }
    }

    /* 区间内插值抽查（人工算） */
    /* x=200: 168~212, 1470 + (1443-1470)*(200-168)/(212-168)=1450.36 -> 1450 */
    {
        uint16_t g1 = BallControl_InterpolateHoldPwm(200);
        printf("x=200 -> %u (expect 1450) %s\n", g1, g1 == 1450 ? "OK" : "FAIL");
        if (g1 != 1450)
        {
            fails++;
        }
        /* x=314 已在上方 */
        /* x=95 首节点边界 == 1490 已查 */
    }

    /* 新增正方向节点后的区间内插值抽查。 */
    {
        uint16_t positive_mid = BallControl_InterpolateHoldPwm(431);
        printf("x=431 -> %u (expect 1343) %s\n",
               positive_mid,
               positive_mid == 1343 ? "OK" : "FAIL");
        if (positive_mid != 1343)
        {
            fails++;
        }
    }

    /* 第三题关键目标点：-5 cm和+5 cm。 */
    {
        uint16_t negative_5cm = BallControl_InterpolateHoldPwm(192);
        uint16_t positive_5cm = BallControl_InterpolateHoldPwm(444);
        printf("x=192 -> %u (expect 1455) %s\n",
               negative_5cm,
               negative_5cm == 1455 ? "OK" : "FAIL");
        printf("x=444 -> %u (expect 1337) %s\n",
               positive_5cm,
               positive_5cm == 1337 ? "OK" : "FAIL");
        if (negative_5cm != 1455 || positive_5cm != 1337)
        {
            fails++;
        }
    }

    printf("\n%s (%d failures)\n", fails ? "TEST FAILED" : "ALL PASS", fails);
    return fails ? 1 : 0;
}
