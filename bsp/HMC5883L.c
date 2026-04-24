#include "HMC5883L.h"
#include "i2c.h"

int16_t hmc_x, hmc_y, hmc_z;
float hmc_x_cal, hmc_y_cal, hmc_z_cal;
float yaw_hmc;

// 鎴戦噸鍐欎簡HMC5883L鐨勬ā鍧楋紝鎶婂畠浠待恢复注释杞欢I2C+绉佹湁搴待恢复注释鐨勫舰寮忚浆鍙樻垚浜待恢复注释纭欢I2C+HAL搴待恢复注释鐨勫舰寮待恢复注释
// 鍑芥暟杩斿洖鍊奸噸鍐欎负HAL_StatusTypeDef 浣垮嚱鏁板彲浠ヨ繑鍥炴墽琛岀姸鎬待恢复注释
// 纭欢鍦板潃瀹忓畾涔夊湪.h鏂囦欢閲岋紝鍙彸閿烦杞煡鐪嬶紝纭欢鍦板潃鍦ㄥ瘎瀛樺櫒鎵嬪唽閲屽彲浠ユ煡鐪待恢复注释

/*
	鍙傛暟涓€锛氱‖浠跺唴 鐩爣鍐欏叆鍦板潃
	鍙傛暟浜岋細瑕佸啓鍏ョ殑瀛楄妭
*/

HAL_StatusTypeDef HMC5883L_Write(uint8_t addr, uint8_t dat)
{
	// Mem灏辨槸鎶待恢复注释鎵剧‖浠跺湴鍧€銆佹壘纭欢鍐呭湴鍧€銆佸啓/璇待恢复注释涓変綅涓€浣撶殑纭欢I2C鍑芥暟銆待恢复注释
	
	return HAL_I2C_Mem_Write(&hi2c2,
                             HMC5883L_ADDR,
                             addr,						// 鍐呴儴瑕佹搷浣滅殑鍦板潃
                             I2C_MEMADD_SIZE_8BIT,		// 瀵勫瓨鍣ㄦ湁澶氬
                             &dat,						
                             1,							// 瑕佹搷浣滃嚑瀛楄妭
                             10);
}

// 鍘熸潵璇诲彇鍑芥暟鏄洿鎺ヨ繑鍥炶鍙栧€硷紝鏇存敼鎴愪簡杩斿洖鎵ц鐘舵€侊紝澧炲姞浜嗕竴涓寚閽堝弬鏁帮紝鐢ㄤ簬鍌ㄥ瓨杩斿洖鍊待恢复注释

/*
	鍙傛暟涓€锛氱‖浠跺唴 鐩爣璇诲彇鍦板潃
	鍙傛暟浜岋細杩斿洖鍊煎皢瑕佸啓鍏ョ殑鍙橀噺鐨勬寚閽待恢复注释
*/

HAL_StatusTypeDef HMC5883L_Read(uint8_t addr, uint8_t *dat)
{
	    return HAL_I2C_Mem_Read(&hi2c2,
                            HMC5883L_ADDR,
                            addr,
                            I2C_MEMADD_SIZE_8BIT,
                            dat,
                            1,
                            10);
}

HAL_StatusTypeDef HMC5883L_ReadBytes(uint8_t addr, uint8_t *buf, uint16_t len)
{
    return HAL_I2C_Mem_Read(&hi2c2,
                            HMC5883L_ADDR,
                            addr,
                            I2C_MEMADD_SIZE_8BIT,
                            buf,
                            len,
                            10);
}

HAL_StatusTypeDef HMC5883L_Init()
{
	HAL_Delay(100);
	
	// 鏈€澶ц緭鍑洪€熺巼锛待恢复注释5HZ锛待恢复注释
	if (HMC5883L_Write(HMC5883L_CRA, 0x78) != HAL_OK)
        return HAL_ERROR;
	
	// 榛樿澧炵泭
	if (HMC5883L_Write(HMC5883L_CRB, 0x20) != HAL_OK)
        return HAL_ERROR;
	
	// 杩炵画娴嬮噺
	if (HMC5883L_Write(HMC5883L_MR, 0x00) != HAL_OK)
        return HAL_ERROR;

	HAL_Delay(20); 
	
	return HAL_OK;
}

HAL_StatusTypeDef HMC5883L_GetData(int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t buf[6];

    if (HMC5883L_ReadBytes(HMC5883L_DOXMR, buf, 6) != HAL_OK)
        return HAL_ERROR;

    // HMC5883L 杩炵画瀵勫瓨鍣ㄩ『搴忔槸:
    // 0x03 X_MSB
    // 0x04 X_LSB
    // 0x05 Z_MSB
    // 0x06 Z_LSB
    // 0x07 Y_MSB
    // 0x08 Y_LSB

    *x = (int16_t)((buf[0] << 8) | buf[1]);
    *z = (int16_t)((buf[2] << 8) | buf[3]);
    *y = (int16_t)((buf[4] << 8) | buf[5]);

    return HAL_OK;
}


//HAL_StatusTypeDef HMC5883L_GetData(int16_t *x, int16_t *y, int16_t *z)
//{
//	// 楂樹綅鏁版嵁鍜屼綆浣嶆暟鎹紝鑾峰彇鍚庢嫾璧锋潵
//	// 杩欓噷閲囩敤璇诲叚娆″崟瀛楄妭鍐嶆嫾璧锋潵鐨勬柟娉待恢复注释
//	// 鏇村悎鐞嗙殑鏄娇鐢≧eadByte鍑芥暟涓€娆℃€ц鍙栧叚涓瓧鑺待恢复注释
//	// 鍙互鍙傝€僲pu6050.c鐨凴eadByte鍐欐硶
//	
//	// 鎴戦噸鍐欎簡杩欎釜鍑芥暟锛屾妸鍘熸潵 鐩存帴鍦ㄥ嚱鏁伴噷淇敼鎸囧畾鍙橀噺 鍙樻垚浜待恢复注释浼犲叆鎸囬拡銆佷慨鏀规寚閽堟寚鍚戠殑鍙橀噺
//	// 杩樺鍔犱簡鐘舵€佽繑鍥炲€待恢复注释
//	uint8_t data_h, data_l;
//	if(HMC5883L_Read(HMC5883L_DOXMR, &data_h) != HAL_OK)
//		return HAL_ERROR;
//	if(HMC5883L_Read(HMC5883L_DOXLR, &data_l) != HAL_OK)
//		return HAL_ERROR;
//	*x = data_l | (data_h << 8);

//	if(HMC5883L_Read(HMC5883L_DOYMR, &data_h) != HAL_OK)
//		return HAL_ERROR;
//	if(HMC5883L_Read(HMC5883L_DOYLR, &data_l) != HAL_OK)
//		return HAL_ERROR;
//	*y = data_l | (data_h << 8);

//	if(HMC5883L_Read(HMC5883L_DOZMR, &data_h) != HAL_OK)
//		return HAL_ERROR;
//	if(HMC5883L_Read(HMC5883L_DOZLR, &data_l) != HAL_OK)
//		return HAL_ERROR;
//	*z = data_l | (data_h << 8);
//	
//	return HAL_OK;
//}




	


