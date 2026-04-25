#include "HMC5883L_Calibration.h"
#include "HMC5883L.h"
#include <math.h>

/*
 * 这个宏表示“弧度转角度”的系数。
 *
 * 为什么需要它？
 * atan2f() 计算出来的结果单位是“弧度”，
 * 但我们平时看小车朝向角更习惯用“度”。
 *
 * 角度 = 弧度 * 57.2957795...
 */
#define HMC5883L_CALIBRATION_DEG_PER_RAD 57.2957795130823208768f

/*
 * 函数名：HMC5883L_Calibration_Max3
 * ----------------------------------------------------------------------------
 * 功能：
 *     取 3 个浮点数里面最大的那个。
 *
 * 为什么这里需要它？
 * 因为在计算 scale_x / scale_y / scale_z 之前，
 * 我们需要先找到 half_x / half_y / half_z 中最大的半径，
 * 把它当作参考半径 ref_half。
 *
 * 参数说明：
 * - a:
 *     第 1 个浮点数。
 *
 * - b:
 *     第 2 个浮点数。
 *
 * - c:
 *     第 3 个浮点数。
 *
 * 返回值：
 *     3 个数中最大的那个值。
 *
 * static 的意思：
 *     这个函数只给当前 .c 文件内部使用，
 *     头文件外面看不到它。
 */
 
HMC5883L_CalibStatus App_ReadSample(int16_t *x, int16_t *y, int16_t *z, void *user_ctx)
{
    (void)user_ctx;

		/*下面这里写你自己接收HMC5883Lx,y,z数据的函数*/
    if (HMC5883L_GetData(x,y,z) == HAL_OK) 
    {
        return HMC5883L_CALIB_STATUS_OK;
    }

    return HMC5883L_CALIB_STATUS_READ_FAILED;
}

void App_DelayMs(uint32_t delay_ms, void *user_ctx)
{
    (void)user_ctx;
		// 这里写延迟函数，如果是HAL库那么就是
    HAL_Delay(delay_ms); 
}
 
static float HMC5883L_Calibration_Max3(float a, float b, float c)
{
    /*
     * max_value：当前已经找到的最大值。
     * 一开始先默认等于 a，
     * 然后再拿 b、c 和它比较。
     */
    float max_value;

    max_value = a;
    if (b > max_value)
    {
        max_value = b;
    }
    if (c > max_value)
    {
        max_value = c;
    }

    return max_value;
}

/*
 * 函数名：HMC5883L_Calibration_SetIdentity
 * ----------------------------------------------------------------------------
 * 作用：
 *     把校准结果设成“默认不修正状态”。
 *
 * 为什么叫 Identity？
 *     在数学里，Identity 可以理解成“单位、不改变原值”的状态。
 *
 *     对这个模块来说：
 *     offset = 0
 *     scale  = 1
 *
 *     这样套进公式后：
 *     cal_x = (raw_x - 0) * 1 = raw_x
 *     cal_y = (raw_y - 0) * 1 = raw_y
 *     cal_z = (raw_z - 0) * 1 = raw_z
 *
 *     也就是“完全不改原始值”。
 */
void HMC5883L_Calibration_SetIdentity(HMC5883L_CalibrationResult *result)
{
    /*
     * result 是一个指针，指向外部传进来的校准结果变量。
     * 如果它是 0（空指针），说明外部没有给我们有效地址。
     * 这种情况下函数就直接返回，避免访问非法地址。
     */
    if (result == 0)
    {
        return;
    }

    /* 三个偏移量全部清零。 */
    result->offset_x = 0.0f;
    result->offset_y = 0.0f;
    result->offset_z = 0.0f;

    /* 三个缩放系数全部设为 1。 */
    result->scale_x = 1.0f;
    result->scale_y = 1.0f;
    result->scale_z = 1.0f;
}

/*
 * 函数名：HMC5883L_Calibration_ResetCollector
 * ----------------------------------------------------------------------------
 * 作用：
 *     清空一个采样收集器，让它回到“刚开始采样之前”的状态。
 */
void HMC5883L_Calibration_ResetCollector(HMC5883L_CalibrationCollector *collector)
{
    /*
     * 如果 collector 是空指针，说明没有合法的收集器可操作。
     * 这里直接返回即可。
     */
    if (collector == 0)
    {
        return;
    }

    /*
     * min/max 全部先清零。
     * 注意：
     * 这些值最终会在第一组样本加入时被真正初始化，
     * 所以这里清零只是“重置状态”，不是最终有效值。
     */
    collector->min_x = 0;
    collector->max_x = 0;
    collector->min_y = 0;
    collector->max_y = 0;
    collector->min_z = 0;
    collector->max_z = 0;

    /* 样本数归零，表示当前还没有加入任何样本。 */
    collector->sample_count = 0U;

    /*
     * initialized = 0 表示：
     * “还没有用第一组样本初始化过 min/max”。
     */
    collector->initialized = 0U;
}

/*
 * 函数名：HMC5883L_Calibration_AddSample
 * ----------------------------------------------------------------------------
 * 作用：
 *     往收集器中加入一组新的原始样本。
 */
HMC5883L_CalibStatus HMC5883L_Calibration_AddSample(HMC5883L_CalibrationCollector *collector,
                                                    int16_t raw_x,
                                                    int16_t raw_y,
                                                    int16_t raw_z)
{
    /*
     * collector：
     *     指向采样收集器的指针。
     *
     * raw_x / raw_y / raw_z：
     *     本次新采到的一组三轴原始数据。
     */
    if (collector == 0)
    {
        return HMC5883L_CALIB_STATUS_INVALID_PARAM;
    }

    /*
     * 如果这是第一组样本，收集器还没有初始化过，
     * 那就直接拿这组样本同时作为最小值和最大值。
     *
     * 例如：
     * 第一次采样得到 X = 123，
     * 那么此时：
     * min_x = 123
     * max_x = 123
     *
     * 后面再来的样本，才会慢慢把这个范围扩展开。
     */
    if (collector->initialized == 0U)
    {
        collector->min_x = raw_x;
        collector->max_x = raw_x;
        collector->min_y = raw_y;
        collector->max_y = raw_y;
        collector->min_z = raw_z;
        collector->max_z = raw_z;

        /*
         * sample_count = 1 表示：
         * 到目前为止已经成功收集到了 1 组样本。
         */
        collector->sample_count = 1U;

        /*
         * initialized = 1 表示：
         * min/max 已经有合法初值了。
         */
        collector->initialized = 1U;

        return HMC5883L_CALIB_STATUS_OK;
    }

    /*
     * 下面的逻辑就是“更新 X 轴范围”：
     * 如果新来的 raw_x 比当前最小值还小，就刷新最小值；
     * 如果新来的 raw_x 比当前最大值还大，就刷新最大值。
     */
    if (raw_x < collector->min_x)
    {
        collector->min_x = raw_x;
    }
    if (raw_x > collector->max_x)
    {
        collector->max_x = raw_x;
    }

    /* 更新 Y 轴范围。 */
    if (raw_y < collector->min_y)
    {
        collector->min_y = raw_y;
    }
    if (raw_y > collector->max_y)
    {
        collector->max_y = raw_y;
    }

    /* 更新 Z 轴范围。 */
    if (raw_z < collector->min_z)
    {
        collector->min_z = raw_z;
    }
    if (raw_z > collector->max_z)
    {
        collector->max_z = raw_z;
    }

    /*
     * 每成功加入一组样本，样本数加 1。
     */
    collector->sample_count++;

    return HMC5883L_CALIB_STATUS_OK;
}

/*
 * 函数名：HMC5883L_Calibration_Generate
 * ----------------------------------------------------------------------------
 * 作用：
 *     根据收集器中记录的 min/max，计算最终校准参数。
 */
HMC5883L_CalibStatus HMC5883L_Calibration_Generate(const HMC5883L_CalibrationCollector *collector,
                                                   HMC5883L_CalibrationResult *result)
{
    /*
     * half_x / half_y / half_z：
     *     分别表示 X/Y/Z 三个轴的“半径”。
     *     公式是：
     *         (最大值 - 最小值) / 2
     *
     * ref_half：
     *     三个半径里最大的那个，作为参考半径。
     */
    float half_x;
    float half_y;
    float half_z;
    float ref_half;

    /*
     * collector 是输入数据来源，result 是输出结果目标。
     * 任何一个为空都不能继续。
     */
    if ((collector == 0) || (result == 0))
    {
        return HMC5883L_CALIB_STATUS_INVALID_PARAM;
    }

    /*
     * 如果还没初始化，或者样本数还是 0，
     * 说明根本没有有效数据可算。
     */
    if ((collector->initialized == 0U) || (collector->sample_count == 0U))
    {
        return HMC5883L_CALIB_STATUS_NOT_READY;
    }

    /*
     * 计算三个轴的 offset。
     *
     * 你可以把它想成“求中心点”。
     * 例如：
     * max_x = 110
     * min_x = -90
     * 那么：
     * offset_x = (110 + -90) / 2 = 10
     *
     * 说明原始数据的中心偏到了 +10，
     * 后面应该减掉它。
     */
    result->offset_x = ((float)collector->max_x + (float)collector->min_x) * 0.5f;
    result->offset_y = ((float)collector->max_y + (float)collector->min_y) * 0.5f;
    result->offset_z = ((float)collector->max_z + (float)collector->min_z) * 0.5f;

    /*
     * 计算三个轴的半径。
     *
     * 例子：
     * max_x = 110
     * min_x = -90
     * half_x = (110 - -90) / 2 = 100
     *
     * 这个值可以理解成“X 轴摆动范围的一半”。
     */
    half_x = ((float)collector->max_x - (float)collector->min_x) * 0.5f;
    half_y = ((float)collector->max_y - (float)collector->min_y) * 0.5f;
    half_z = ((float)collector->max_z - (float)collector->min_z) * 0.5f;

    /*
     * 如果某个轴的半径太小，说明这一轴几乎没怎么动。
     * 例如只左右平转，没有上下翻动，
     * 那么 Z 轴变化可能很小。
     *
     * 这种情况下，scale 计算出来会很不可靠，
     * 所以直接返回错误。
     */
    if ((half_x < 1.0f) || (half_y < 1.0f) || (half_z < 1.0f))
    {
        return HMC5883L_CALIB_STATUS_RANGE_TOO_SMALL;
    }

    /*
     * 找出三个轴半径中最大的那个。
     * 为什么要找最大的？
     * 因为我们要拿它当“目标尺度”。
     */
    ref_half = HMC5883L_Calibration_Max3(half_x, half_y, half_z);

    /*
     * 计算三个轴的缩放系数。
     *
     * 举例：
     * 如果：
     * half_x = 100
     * half_y = 80
     * half_z = 50
     * ref_half = 100
     *
     * 那么：
     * scale_x = 100 / 100 = 1.00
     * scale_y = 100 / 80  = 1.25
     * scale_z = 100 / 50  = 2.00
     *
     * 这意味着：
     * X 不用放大，
     * Y 放大 1.25 倍，
     * Z 放大 2 倍。
     */
    result->scale_x = ref_half / half_x;
    result->scale_y = ref_half / half_y;
    result->scale_z = ref_half / half_z;

    return HMC5883L_CALIB_STATUS_OK;
}

/*
 * 函数名：HMC5883L_Calibration_RunBlocking
 * ----------------------------------------------------------------------------
 * 作用：
 *     自动完成整套阻塞式校准流程。
 */
HMC5883L_CalibStatus HMC5883L_Calibration_RunBlocking(HMC5883L_CalibrationResult *result,
                                                      uint16_t sample_count,
                                                      uint32_t sample_delay_ms,
                                                      HMC5883L_CalibrationReadSampleFn read_sample_fn,
                                                      HMC5883L_CalibrationDelayMsFn delay_ms_fn,
                                                      void *user_ctx)
{
    /*
     * collector：
     *     本地临时采样收集器。
     *     它在这个函数里负责统计 min/max/sample_count。
     *
     * status：
     *     用来保存每一步函数调用的返回状态。
     *
     * collected：
     *     for 循环里的计数变量，表示已经采集到第几组。
     *
     * raw_x / raw_y / raw_z：
     *     临时保存本次读到的一组原始数据。
     */
    HMC5883L_CalibrationCollector collector;
    HMC5883L_CalibStatus status;
    uint16_t collected;
    int16_t raw_x;
    int16_t raw_y;
    int16_t raw_z;

    /*
     * 参数合法性检查：
     * 1. result 不能为空
     * 2. sample_count 不能为 0
     * 3. read_sample_fn 不能为空
     */
    if ((result == 0) || (sample_count == 0U) || (read_sample_fn == 0))
    {
        return HMC5883L_CALIB_STATUS_INVALID_PARAM;
    }

    /*
     * 如果要求采样间隔大于 0，
     * 那么必须提供 delay_ms_fn。
     */
    if ((sample_delay_ms > 0U) && (delay_ms_fn == 0))
    {
        return HMC5883L_CALIB_STATUS_INVALID_PARAM;
    }

    /* 先把本地收集器清零。 */
    HMC5883L_Calibration_ResetCollector(&collector);

    /*
     * 先读第一组数据。
     * 为什么要单独先读一次？
     * 因为第一组数据要拿来初始化 min/max。
     */
    status = read_sample_fn(&raw_x, &raw_y, &raw_z, user_ctx);
    if (status != HMC5883L_CALIB_STATUS_OK)
    {
        return status;
    }

    /* 把第一组样本塞进收集器。 */
    status = HMC5883L_Calibration_AddSample(&collector, raw_x, raw_y, raw_z);
    if (status != HMC5883L_CALIB_STATUS_OK)
    {
        return status;
    }

    /*
     * 继续采集剩下的 sample_count - 1 组数据。
     *
     * 例如 sample_count = 300：
     * 第一组已经采过了，
     * 这里还要再采 299 组。
     */
    for (collected = 1U; collected < sample_count; collected++)
    {
        /*
         * 两次采样之间先等待一会儿，
         * 防止采样太快，读到太密或者重复的数据。
         */
        if ((sample_delay_ms > 0U) && (delay_ms_fn != 0))
        {
            delay_ms_fn(sample_delay_ms, user_ctx);
        }

        /* 再读一组新数据。 */
        status = read_sample_fn(&raw_x, &raw_y, &raw_z, user_ctx);
        if (status != HMC5883L_CALIB_STATUS_OK)
        {
            return status;
        }

        /* 把这组数据继续喂给收集器。 */
        status = HMC5883L_Calibration_AddSample(&collector, raw_x, raw_y, raw_z);
        if (status != HMC5883L_CALIB_STATUS_OK)
        {
            return status;
        }
    }

    /*
     * 当所有样本都收集完后，
     * 最后根据收集器生成真正的校准结果。
     */
    return HMC5883L_Calibration_Generate(&collector, result);
}

/*
 * 函数名：HMC5883L_Calibration_Apply
 * ----------------------------------------------------------------------------
 * 作用：
 *     用已经算好的 offset 和 scale 去修正一组原始数据。
 */
HMC5883L_CalibStatus HMC5883L_Calibration_Apply(const HMC5883L_CalibrationResult *result,
                                                int16_t raw_x,
                                                int16_t raw_y,
                                                int16_t raw_z,
                                                float *cal_x,
                                                float *cal_y,
                                                float *cal_z)
{
    /*
     * result 里装着 offset_x / scale_x 等校准参数。
     * 如果 result 是空指针，就无法进行修正。
     */
    if (result == 0)
    {
        return HMC5883L_CALIB_STATUS_INVALID_PARAM;
    }

    /*
     * cal_x / cal_y / cal_z 都允许传 0。
     * 这样做的好处是更灵活。
     *
     * 例如：
     * 只想算航向角时，很多时候只关心 X/Y，
     * 那 cal_z 就可以直接传 0。
     */
    if (cal_x != 0)
    {
        *cal_x = ((float)raw_x - result->offset_x) * result->scale_x;
    }
    if (cal_y != 0)
    {
        *cal_y = ((float)raw_y - result->offset_y) * result->scale_y;
    }
    if (cal_z != 0)
    {
        *cal_z = ((float)raw_z - result->offset_z) * result->scale_z;
    }

    return HMC5883L_CALIB_STATUS_OK;
}

/*
 * 函数名：HMC5883L_Calibration_ComputeYawDeg
 * ----------------------------------------------------------------------------
 * 作用：
 *     根据校准后的 X/Y 计算航向角，返回单位为“度”的结果。
 *
 * 计算公式本质上是：
 *     atan2f(Y, X)
 *
 * 为什么用 atan2f 而不是 atan？
 * 因为 atan2f 能正确区分四个象限，
 * 算出来的方向角更靠谱。
 *
 * 参数说明：
 * - calibrated_x：
 *     已经校准后的 X 值。
 *
 * - calibrated_y：
 *     已经校准后的 Y 值。
 *
 * 返回值：
 *     角度值，单位“度”。
 */
float HMC5883L_Calibration_ComputeYawDeg(float calibrated_x, float calibrated_y)
{
    return atan2f(calibrated_y, calibrated_x) * HMC5883L_CALIBRATION_DEG_PER_RAD;
}
