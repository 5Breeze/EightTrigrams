#include "low_power.h"
// 进入睡眠模式
void enter_sleep_mode(void) {
    // 配置系统进入睡眠模式
    HAL_SuspendTick();  // 暂停滴答定时器
    __HAL_RCC_PWR_CLK_ENABLE();  // 使能电源时钟
    HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);

    // 退出睡眠模式后会继续执行这里的代码
    HAL_ResumeTick();  // 恢复滴答定时器
}

// 进入停机模式
void enter_stop_mode(void) {
    // 配置RTC唤醒中断（可选）
    // HAL_RTCEx_SetWakeUpTimer(&hrtc, RTC_WAKEUPCLOCK_RTCCLK_DIV16, 1000, 0);

    // 配置系统进入停机模式
    HAL_SuspendTick();  // 暂停滴答定时器
    __HAL_RCC_PWR_CLK_ENABLE();  // 使能电源时钟
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON,PWR_SLEEPENTRY_WFI);

    // 退出停机模式后会继续执行这里的代码
    HAL_ResumeTick();  // 恢复滴答定时器
}

// 进入待机模式
void enter_standby_mode(void) {
    // 配置RTC唤醒中断（可选）
    // HAL_RTCEx_SetWakeUpTimer(&hrtc, RTC_WAKEUPCLOCK_RTCCLK_DIV16, 1000, 0);

    // 配置系统进入待机模式
    HAL_SuspendTick();  // 暂停滴答定时器
    __HAL_RCC_PWR_CLK_ENABLE();  // 使能电源时钟
	
		__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
    HAL_PWR_EnterSTANDBYMode();

    // 退出待机模式后会继续执行这里的代码
    HAL_ResumeTick();  // 恢复滴答定时器
}
