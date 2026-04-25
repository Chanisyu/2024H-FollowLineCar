#ifndef HMC5883L_CALIBRATION_H
#define HMC5883L_CALIBRATION_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ============================================================================
 * 文件名：HMC5883L_Calibration.h
 * 作用：HMC5883L 磁力计校准模块的头文件
 * ============================================================================
 *
 * 一、这个模块是干什么的？
 * ----------------------------------------------------------------------------
 * 这个模块专门用来做 HMC5883L 的“校准”。
 *
 * 为什么要校准？
 * 因为磁力计装到小车上之后，读出来的数据通常不会很准，常见原因有：
 * 1. 小车上的电机、螺丝、铁件、磁铁会影响磁场。
 * 2. 模块安装时，三个方向的灵敏度可能不完全一样。
 * 3. 周围金属环境会让原始数据整体偏掉。
 *
 * 所以我们通常不能直接用原始的 X/Y/Z。
 * 一般要先做两件事：
 * 1. 先“减去偏移量” offset，修正整体偏移。
 * 2. 再“乘缩放系数” scale，把三个轴拉到差不多的尺度。
 *
 * 这个模块做的正是这件事。
 *
 * ----------------------------------------------------------------------------
 * 二、你可以把它想成什么？
 * ----------------------------------------------------------------------------
 * 你可以把这个模块想成一个“小老师”：
 * 1. 它先让你转动小车，收集很多很多组磁力计原始数据。
 * 2. 它帮你记住 X/Y/Z 每个方向的最小值和最大值。
 * 3. 它根据这些最大值和最小值，算出校准参数。
 * 4. 以后你每次读到新的原始数据，都可以交给它修正。
 *
 * ----------------------------------------------------------------------------
 * 三、这个模块使用什么校准算法？
 * ----------------------------------------------------------------------------
 * 这个模块使用的是“最大值/最小值法”，也就是很多单片机入门项目中最常用的
 * 简化校准方式。它的流程如下：
 *
 * 第 1 步：不断采集原始数据
 *     例如：
 *     第 1 次：X=100,  Y=200,  Z=-50
 *     第 2 次：X=120,  Y=180,  Z=-70
 *     第 3 次：X=-90,  Y=260,  Z=30
 *     ...
 *
 * 第 2 步：分别记录三个轴的最小值和最大值
 *     比如：
 *     X 最小值 = -120
 *     X 最大值 =  130
 *     Y 最小值 = -200
 *     Y 最大值 =  250
 *     Z 最小值 =  -80
 *     Z 最大值 =   90
 *
 * 第 3 步：算 offset（偏移量）
 *     公式：
 *     offset_x = (max_x + min_x) / 2
 *     offset_y = (max_y + min_y) / 2
 *     offset_z = (max_z + min_z) / 2
 *
 *     这一步的含义是：
 *     “把球心找出来”，也就是让数据中心尽量回到 0 附近。
 *
 * 第 4 步：算每个轴的半径 half_x / half_y / half_z
 *     公式：
 *     half_x = (max_x - min_x) / 2
 *     half_y = (max_y - min_y) / 2
 *     half_z = (max_z - min_z) / 2
 *
 *     这一步的含义是：
 *     “看看三个方向谁大谁小”。
 *
 * 第 5 步：算 scale（缩放系数）
 *     先找三个半径里最大的那个，记作 ref_half。
 *     再算：
 *     scale_x = ref_half / half_x
 *     scale_y = ref_half / half_y
 *     scale_z = ref_half / half_z
 *
 *     这一步的含义是：
 *     “把比较小的轴放大一点，让三个方向更一致”。
 *
 * ----------------------------------------------------------------------------
 * 四、这个模块不做什么？
 * ----------------------------------------------------------------------------
 * 这个模块只负责“校准参数的计算”和“校准后的数据修正”。
 * 它不负责：
 * 1. I2C 通信
 * 2. OLED 显示
 * 3. 电机控制
 * 4. STM32 初始化
 * 5. HMC5883L 芯片配置
 *
 * 也就是说：
 * 你还是需要你自己的 HMC5883L 驱动去读取原始 X/Y/Z。
 * 这个模块只吃“原始数据”，然后吐出“校准参数”和“修正后的数据”。
 *
 * ----------------------------------------------------------------------------
 * 五、这个模块适合什么人？
 * ----------------------------------------------------------------------------
 * 这个模块非常适合：
 * 1. STM32 初学者
 * 2. 做寻迹小车、循迹车、方向车的同学
 * 3. 想要“上电后自动校准几秒”的项目
 * 4. 不想一开始就上复杂椭球拟合算法的人
 *
 * ----------------------------------------------------------------------------
 * 六、你最常用的使用流程（小白版）
 * ----------------------------------------------------------------------------
 * 方式 A：上电后阻塞式自动校准
 *     这最适合你现在的小车项目。
 *
 *     步骤如下：
 *     1. 初始化 HMC5883L
 *     2. 调用 HMC5883L_Calibration_RunBlocking(...)
 *     3. 在这几秒里手动转动小车
 *     4. 函数返回后，校准参数就已经算好了
 *     5. 后续每次读到原始数据后，调用 HMC5883L_Calibration_Apply(...)
 *     6. 再用 HMC5883L_Calibration_ComputeYawDeg(...) 算航向角
 *
 * 方式 B：增量式校准
 *     适合你自己在主循环、定时器中断、任务里慢慢喂数据。
 *
 * ----------------------------------------------------------------------------
 * 七、最常见的数据类型解释（非常重要）
 * ----------------------------------------------------------------------------
 * 1. int16_t
 *    含义：16 位有符号整数。
 *    能表示正数，也能表示负数。
 *    HMC5883L 的原始 X/Y/Z 数据通常就是这种类型。
 *    例如：-123、0、456。
 *
 * 2. uint16_t
 *    含义：16 位无符号整数。
 *    只能表示 0 和正数，不能表示负数。
 *    常用来表示“采样次数”这种肯定不会是负数的量。
 *
 * 3. uint32_t
 *    含义：32 位无符号整数。
 *    常用来表示毫秒时间、计数值等。
 *    比如延时 20ms、1000ms，就可以用它。
 *
 * 4. uint8_t
 *    含义：8 位无符号整数。
 *    通常用来当作“小标志位”。
 *    例如 initialized = 0 表示“还没初始化”
 *         initialized = 1 表示“已经初始化”
 *
 * 5. float
 *    含义：单精度浮点数，也就是可以带小数的数。
 *    比如：1.0f、-23.5f、57.29f。
 *    因为 offset 和 scale 往往不是整数，所以这里必须用 float。
 *
 * 6. void *
 *    含义：通用指针、万能指针。
 *    你可以先把它简单理解成：
 *    “一个可以传任何额外信息进去的接口位置”。
 *
 *    这个模块里有个 user_ctx 参数，类型就是 void *。
 *    它的作用是：
 *    “如果你以后想传点额外东西给回调函数，就从这里传。”
 *
 *    如果你现在完全用不到它，直接传 0 即可。
 *    在代码里常见写法：
 *        (void)user_ctx;
 *    这句的意思是：
 *        “这个参数我知道有，但我现在故意不用它。”
 *
 * 7. 指针是什么？例如 float *cal_x、int16_t *x
 *    你可以先把指针理解成：
 *    “把结果写回去的地址”。
 *
 *    例如：
 *        int16_t raw_x;
 *        HMC5883L_GetData(&raw_x, &raw_y, &raw_z);
 *
 *    这里的 &raw_x 就是把 raw_x 的地址交给函数，
 *    函数就能把读到的结果写到 raw_x 里面。
 *
 * ----------------------------------------------------------------------------
 * 八、一个完整的阻塞式使用例子
 * ----------------------------------------------------------------------------
 * static HMC5883L_CalibStatus App_ReadSample(int16_t *x, int16_t *y, int16_t *z, void *user_ctx)
 * {
 *     (void)user_ctx;
 *
 *     if (HMC5883L_GetData(x, y, z) == HAL_OK)
 *     {
 *         return HMC5883L_CALIB_STATUS_OK;
 *     }
 *
 *     return HMC5883L_CALIB_STATUS_READ_FAILED;
 * }
 *
 * static void App_DelayMs(uint32_t delay_ms, void *user_ctx)
 * {
 *     (void)user_ctx;
 *     HAL_Delay(delay_ms);
 * }
 *
 * HMC5883L_CalibrationResult calib;
 *
 * HMC5883L_Calibration_SetIdentity(&calib);
 *
 * if (HMC5883L_Calibration_RunBlocking(&calib,
 *                                      300U,
 *                                      20U,
 *                                      App_ReadSample,
 *                                      App_DelayMs,
 *                                      0) == HMC5883L_CALIB_STATUS_OK)
 * {
 *     int16_t raw_x;
 *     int16_t raw_y;
 *     int16_t raw_z;
 *     float cal_x;
 *     float cal_y;
 *     float cal_z;
 *     float yaw_deg;
 *
 *     HMC5883L_GetData(&raw_x, &raw_y, &raw_z);
 *     HMC5883L_Calibration_Apply(&calib, raw_x, raw_y, raw_z, &cal_x, &cal_y, &cal_z);
 *     yaw_deg = HMC5883L_Calibration_ComputeYawDeg(cal_x, cal_y);
 * }
 *
 * ----------------------------------------------------------------------------
 * 九、校准时的操作建议
 * ----------------------------------------------------------------------------
 * 1. 上电后先别让车乱跑，先进入校准阶段。
 * 2. 你用手拿着小车，在几秒钟内多方向转动。
 * 3. 最好不是只在平面上左右转，而是前后左右都翻一翻。
 * 4. 采样时间太短、转动范围太小，都可能让校准效果变差。
 * 5. 校准结束后，可以把 result 里的 6 个数保存下来，供下次开机使用。
 * ============================================================================
 */

/*
 * 枚举类型 enum 说明：
 * ----------------------------------------------------------------------------
 * enum 可以理解成“几个固定选项”。
 * 这里的 enum 用来表示函数执行结果。
 *
 * 为什么不用普通 int？
 * 因为 enum 写出来更清楚，一眼就知道每个数字代表什么含义。
 */
typedef enum
{
    /*
     * 0 表示成功。
     * 这是最常见的返回值。
     */
    HMC5883L_CALIB_STATUS_OK = 0,

    /*
     * -1 表示参数不合法。
     * 例如：
     * 1. 传进来的指针是空指针 0
     * 2. sample_count 写成了 0
     * 3. 要求延时但没有提供延时函数
     */
    HMC5883L_CALIB_STATUS_INVALID_PARAM = -1,

    /*
     * -2 表示“还没准备好”。
     * 一般发生在：
     * 采样收集器里还没有任何有效样本，你却提前让它生成校准结果。
     */
    HMC5883L_CALIB_STATUS_NOT_READY = -2,

    /*
     * -3 表示“变化范围太小”。
     * 这通常意味着你转得不够充分。
     * 比如某一轴几乎没怎么变，模块就没法可靠算出缩放系数。
     */
    HMC5883L_CALIB_STATUS_RANGE_TOO_SMALL = -3,

    /*
     * -4 表示“底层读取磁力计数据失败”。
     * 例如 I2C 读失败、设备没响应等。
     */
    HMC5883L_CALIB_STATUS_READ_FAILED = -4
} HMC5883L_CalibStatus;

/*
 * 结构体 struct 说明：
 * ----------------------------------------------------------------------------
 * struct 可以理解成“一个打包好的数据盒子”。
 * 这个结构体专门用来保存“最终算出来的校准参数”。
 *
 * 你以后最常见的写法是：
 *     HMC5883L_CalibrationResult hmc_calib;
 *
 * 意思是：
 *     “我创建了一个叫 hmc_calib 的变量，
 *      它专门用来保存校准结果。”
 */
typedef struct
{
    /*
     * X 轴偏移量。
     * 使用时通常这样修正：
     *     corrected_x = raw_x - offset_x
     *
     * 为什么要减它？
     * 因为原始 X 轴数据的中心可能不是 0，而是偏到了别的地方。
     */
    float offset_x;

    /*
     * Y 轴偏移量。
     * 用途和 offset_x 一样，只不过它作用于 Y 轴。
     */
    float offset_y;

    /*
     * Z 轴偏移量。
     * 如果你只拿 X/Y 算航向角，也可以暂时不太关注它，
     * 但完整校准结果里仍然会把它算出来。
     */
    float offset_z;

    /*
     * X 轴缩放系数。
     * 在减去偏移量之后，再乘这个系数。
     * 典型用法：
     *     cal_x = (raw_x - offset_x) * scale_x;
     */
    float scale_x;

    /*
     * Y 轴缩放系数。
     */
    float scale_y;

    /*
     * Z 轴缩放系数。
     */
    float scale_z;
} HMC5883L_CalibrationResult;

/*
 * 这个结构体是“采样收集器”。
 * ----------------------------------------------------------------------------
 * 它不是最终结果，而是“校准过程中的中间工作箱子”。
 *
 * 什么时候会用到它？
 * 当你不想一次性阻塞校准，而是想自己在主循环里慢慢喂数据时，
 * 就会用到它。
 *
 * 例如：
 * 1. 先 ResetCollector()
 * 2. 每来一组新数据就 AddSample()
 * 3. 最后 Generate()
 */
typedef struct
{
    /*
     * 到目前为止，X 轴采样到的最小值。
     */
    int16_t min_x;

    /*
     * 到目前为止，X 轴采样到的最大值。
     */
    int16_t max_x;

    /*
     * 到目前为止，Y 轴采样到的最小值。
     */
    int16_t min_y;

    /*
     * 到目前为止，Y 轴采样到的最大值。
     */
    int16_t max_y;

    /*
     * 到目前为止，Z 轴采样到的最小值。
     */
    int16_t min_z;

    /*
     * 到目前为止，Z 轴采样到的最大值。
     */
    int16_t max_z;

    /*
     * 当前已经收集了多少组样本。
     * 注意：这里记录的是“样本组数”，不是毫秒数。
     */
    uint32_t sample_count;

    /*
     * 这个标志表示“是否已经用第一组样本完成初始化”。
     *
     * initialized = 0：
     *     说明还没收到第一组有效样本。
     *
     * initialized = 1：
     *     说明已经至少加入过一组样本了，
     *     min/max 已经有初始值了。
     */
    uint8_t initialized;
} HMC5883L_CalibrationCollector;

/*
 * 这是“读取一组样本的回调函数类型”定义。
 * ----------------------------------------------------------------------------
 * 初学者可以把“回调函数”理解成：
 * “你把一个你自己写的函数，交给模块去调用。”
 *
 * 为什么要这样设计？
 * 因为不同工程读 HMC5883L 的方式可能不同：
 * 1. 有的工程用 HAL I2C
 * 2. 有的工程用模拟 I2C
 * 3. 有的工程封装成 HMC5883L_GetData()
 * 4. 有的工程又是别的名字
 *
 * 所以这个模块不直接写死怎么读，而是让你自己提供一个“读数据函数”。
 *
 * 参数说明：
 * - x:
 *     输出参数，函数需要把读到的 X 轴原始值写到 *x 里。
 *
 * - y:
 *     输出参数，函数需要把读到的 Y 轴原始值写到 *y 里。
 *
 * - z:
 *     输出参数，函数需要把读到的 Z 轴原始值写到 *z 里。
 *
 * - user_ctx:
 *     用户上下文指针。
 *     如果你不需要传额外信息，直接传 0 即可。
 *     如果你暂时用不到它，可以在函数里写：
 *         (void)user_ctx;
 *
 * 返回值说明：
 * - HMC5883L_CALIB_STATUS_OK
 *     表示成功读到了一组样本。
 *
 * - 其他错误码
 *     表示读取失败。
 */
typedef HMC5883L_CalibStatus (*HMC5883L_CalibrationReadSampleFn)(int16_t *x,
                                                                 int16_t *y,
                                                                 int16_t *z,
                                                                 void *user_ctx);

/*
 * 这是“毫秒延时回调函数类型”定义。
 * ----------------------------------------------------------------------------
 * 作用：
 *     在阻塞式校准时，两次采样之间暂停一会儿。
 *
 * 典型写法：
 *     static void App_DelayMs(uint32_t delay_ms, void *user_ctx)
 *     {
 *         (void)user_ctx;
 *         HAL_Delay(delay_ms);
 *     }
 *
 * 参数说明：
 * - delay_ms:
 *     需要延时多少毫秒。
 *
 * - user_ctx:
 *     用户上下文指针。
 *     如果你现在不用它，就忽略即可。
 */
typedef void (*HMC5883L_CalibrationDelayMsFn)(uint32_t delay_ms, void *user_ctx);

/*
 * 函数名：HMC5883L_Calibration_SetIdentity
 * ----------------------------------------------------------------------------
 * 功能：
 *     把一个校准结果变量初始化成“默认值”。
 *
 * 默认值是什么？
 *     offset_x = 0
 *     offset_y = 0
 *     offset_z = 0
 *     scale_x  = 1
 *     scale_y  = 1
 *     scale_z  = 1
 *
 * 这意味着：
 *     “先不做任何修正”。
 *
 * 为什么需要这个函数？
 * 1. 防止你的变量里原本是乱值。
 * 2. 让你在校准前有一组确定的初始参数。
 *
 * 参数说明：
 * - result:
 *     指向一个 HMC5883L_CalibrationResult 变量的指针。
 *     函数会把默认值写进这个变量里。
 *
 * 使用示例：
 *     HMC5883L_CalibrationResult calib;
 *     HMC5883L_Calibration_SetIdentity(&calib);
 */
void HMC5883L_Calibration_SetIdentity(HMC5883L_CalibrationResult *result);

/*
 * 函数名：HMC5883L_Calibration_ResetCollector
 * ----------------------------------------------------------------------------
 * 功能：
 *     把“采样收集器”清空，准备开始新一轮校准。
 *
 * 什么时候调用？
 *     当你要开始重新采样时，就先调它。
 *
 * 它会做什么？
 * 1. 清空 min/max
 * 2. 把 sample_count 清零
 * 3. 把 initialized 设为 0
 *
 * 参数说明：
 * - collector:
 *     指向采样收集器变量的指针。
 *
 * 使用示例：
 *     HMC5883L_CalibrationCollector collector;
 *     HMC5883L_Calibration_ResetCollector(&collector);
 */
void HMC5883L_Calibration_ResetCollector(HMC5883L_CalibrationCollector *collector);

/*
 * 函数名：HMC5883L_Calibration_AddSample
 * ----------------------------------------------------------------------------
 * 功能：
 *     向采样收集器中加入一组新的原始磁力计数据。
 *
 * 这个函数会自动做什么？
 * 1. 如果这是第一组数据，就用它初始化 min/max。
 * 2. 如果不是第一组数据，就拿它去更新 min/max。
 * 3. sample_count 自动加 1。
 *
 * 参数说明：
 * - collector:
 *     采样收集器指针。
 *     这个函数会修改它里面的 min/max/sample_count。
 *
 * - raw_x:
 *     新采到的一组原始 X 轴数据。
 *
 * - raw_y:
 *     新采到的一组原始 Y 轴数据。
 *
 * - raw_z:
 *     新采到的一组原始 Z 轴数据。
 *
 * 返回值说明：
 * - HMC5883L_CALIB_STATUS_OK
 *     表示成功加入样本。
 *
 * - HMC5883L_CALIB_STATUS_INVALID_PARAM
 *     表示 collector 是空指针。
 *
 * 使用示例：
 *     HMC5883L_Calibration_AddSample(&collector, raw_x, raw_y, raw_z);
 */
HMC5883L_CalibStatus HMC5883L_Calibration_AddSample(HMC5883L_CalibrationCollector *collector,
                                                    int16_t raw_x,
                                                    int16_t raw_y,
                                                    int16_t raw_z);

/*
 * 函数名：HMC5883L_Calibration_Generate
 * ----------------------------------------------------------------------------
 * 功能：
 *     根据采样收集器里已经收集好的 min/max 信息，生成最终校准结果。
 *
 * 换句话说：
 *     AddSample() 负责“收集数据”，
 *     Generate() 负责“根据这些数据算参数”。
 *
 * 它输出什么？
 *     一个 HMC5883L_CalibrationResult 结构体，
 *     里面有 offset_x / offset_y / offset_z / scale_x / scale_y / scale_z。
 *
 * 参数说明：
 * - collector:
 *     只读输入参数。
 *     里面必须已经至少有一组有效样本。
 *
 * - result:
 *     输出参数。
 *     函数会把计算好的校准参数写到这个结构体里。
 *
 * 返回值说明：
 * - HMC5883L_CALIB_STATUS_OK
 *     成功生成了校准结果。
 *
 * - HMC5883L_CALIB_STATUS_INVALID_PARAM
 *     collector 或 result 是空指针。
 *
 * - HMC5883L_CALIB_STATUS_NOT_READY
 *     collector 里还没有有效样本。
 *
 * - HMC5883L_CALIB_STATUS_RANGE_TOO_SMALL
 *     某一轴变化太小，说明你采样时转动不够充分，
 *     这时生成出来的 scale 会不可靠，所以函数直接报错。
 */
HMC5883L_CalibStatus HMC5883L_Calibration_Generate(const HMC5883L_CalibrationCollector *collector,
                                                   HMC5883L_CalibrationResult *result);

/*
 * 函数名：HMC5883L_Calibration_RunBlocking
 * ----------------------------------------------------------------------------
 * 功能：
 *     执行“一次完整的阻塞式校准流程”。
 *
 * 什么叫“阻塞式”？
 *     就是函数一旦开始运行，程序会一直在这里采样、等待、再采样、再等待，
 *     直到校准完成之后，函数才会返回。
 *
 * 对初学者来说，这通常是最简单最好理解的方式。
 *
 * 你可以把这个函数理解成：
 *     “帮你自动完成整套校准流程的大总管函数。”
 *
 * 它内部会做什么？
 * 1. 读取第一组数据，初始化 min/max。
 * 2. 按 sample_count 的要求继续读取后续数据。
 * 3. 每次采样之间调用 delay_ms_fn 延时。
 * 4. 最后自动计算 offset 和 scale。
 * 5. 把结果写入 result。
 *
 * 参数说明：
 * - result:
 *     输出参数。
 *     校准成功后，最终的 offset 和 scale 会写到这里。
 *
 * - sample_count:
 *     需要采样多少组数据。
 *     比如写 300，就表示采 300 次。
 *     这个值必须大于 0。
 *
 * - sample_delay_ms:
 *     两次采样之间间隔多少毫秒。
 *     比如写 20，就表示隔 20ms 采一次。
 *
 * - read_sample_fn:
 *     你提供的“读取一组 HMC5883L 原始数据”的函数。
 *
 * - delay_ms_fn:
 *     你提供的“毫秒延时函数”。
 *     如果 sample_delay_ms 大于 0，这个参数就不能是 0。
 *
 * - user_ctx:
 *     传给 read_sample_fn 和 delay_ms_fn 的额外用户参数。
 *     如果你暂时不需要，直接传 0 即可。
 *
 * 返回值说明：
 * - HMC5883L_CALIB_STATUS_OK
 *     表示整套校准流程成功结束。
 *
 * - HMC5883L_CALIB_STATUS_INVALID_PARAM
 *     参数有问题。
 *
 * - HMC5883L_CALIB_STATUS_READ_FAILED
 *     中途读取磁力计失败。
 *
 * - HMC5883L_CALIB_STATUS_RANGE_TOO_SMALL
 *     虽然采到了数据，但变化范围太小，不能可靠生成校准参数。
 *
 * 典型使用建议：
 * - 如果你想上电后自动校准 6 秒，可以写：
 *       sample_count = 300
 *       sample_delay_ms = 20
 *
 *   因为：
 *       300 * 20ms = 6000ms = 约 6 秒
 */
HMC5883L_CalibStatus HMC5883L_Calibration_RunBlocking(HMC5883L_CalibrationResult *result,
                                                      uint16_t sample_count,
                                                      uint32_t sample_delay_ms,
                                                      HMC5883L_CalibrationReadSampleFn read_sample_fn,
                                                      HMC5883L_CalibrationDelayMsFn delay_ms_fn,
                                                      void *user_ctx);

/*
 * 函数名：HMC5883L_Calibration_Apply
 * ----------------------------------------------------------------------------
 * 功能：
 *     使用已经算好的校准参数，去修正一组新的原始磁力计数据。
 *
 * 公式：
 *     cal_x = (raw_x - offset_x) * scale_x
 *     cal_y = (raw_y - offset_y) * scale_y
 *     cal_z = (raw_z - offset_z) * scale_z
 *
 * 你可以把它理解成：
 *     “把原始值洗一遍，变成更靠谱的值。”
 *
 * 参数说明：
 * - result:
 *     输入参数。
 *     里面必须装着有效的校准结果。
 *
 * - raw_x:
 *     原始 X 轴值。
 *
 * - raw_y:
 *     原始 Y 轴值。
 *
 * - raw_z:
 *     原始 Z 轴值。
 *
 * - cal_x:
 *     输出参数。
 *     如果你希望函数把校准后的 X 写出来，就传入 &变量名。
 *     如果你不关心 X，也可以传 0。
 *
 * - cal_y:
 *     输出参数。
 *     如果你希望函数把校准后的 Y 写出来，就传入 &变量名。
 *     如果你不关心 Y，也可以传 0。
 *
 * - cal_z:
 *     输出参数。
 *     如果你希望函数把校准后的 Z 写出来，就传入 &变量名。
 *     如果你不关心 Z，也可以传 0。
 *
 * 返回值说明：
 * - HMC5883L_CALIB_STATUS_OK
 *     成功执行修正。
 *
 * - HMC5883L_CALIB_STATUS_INVALID_PARAM
 *     result 是空指针。
 *
 * 使用示例：
 *     float cal_x;
 *     float cal_y;
 *     HMC5883L_Calibration_Apply(&calib, raw_x, raw_y, raw_z, &cal_x, &cal_y, 0);
 */
HMC5883L_CalibStatus HMC5883L_Calibration_Apply(const HMC5883L_CalibrationResult *result,
                                                int16_t raw_x,
                                                int16_t raw_y,
                                                int16_t raw_z,
                                                float *cal_x,
                                                float *cal_y,
                                                float *cal_z);

/*
 * 函数名：HMC5883L_Calibration_ComputeYawDeg
 * ----------------------------------------------------------------------------
 * 功能：
 *     根据“校准后的 X/Y 数据”计算航向角，单位是“度”。
 *
 * 为什么只用 X 和 Y？
 *     因为平面航向角通常就是在水平面里看的方向角，
 *     所以常见写法是 atan2f(Y, X)。
 *
 * 参数说明：
 * - calibrated_x:
 *     已经校准后的 X 轴值。
 *
 * - calibrated_y:
 *     已经校准后的 Y 轴值。
 *
 * 返回值说明：
 * - 返回一个 float 角度值，单位为度。
 * - 一般范围在 -180 到 +180 度之间。
 *
 * 注意：
 * 1. 这个函数只负责数学计算。
 * 2. 如果你的模块安装方向和别人的工程不同，
 *    可能还要自己额外加一个角度补偿。
 */
float HMC5883L_Calibration_ComputeYawDeg(float calibrated_x, float calibrated_y);

extern HMC5883L_CalibStatus App_ReadSample(int16_t *x, int16_t *y, int16_t *z, void *user_ctx);
extern void App_DelayMs(uint32_t delay_ms, void *user_ctx);

#ifdef __cplusplus
}
#endif

#endif
