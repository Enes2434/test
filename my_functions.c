/*
 * my_functions.c
 *
 *  Created on: 24 Kas 2023
 *      Author: ENES TASTEKIN
 */
#include "driverlib.h"
#include "math.h"
#include "variable.h"
#include "my_functions.h"


//=============================================================================
// REC_calibrateOffset()
// offset of the voltage and current sense
//=============================================================================
void REC_calibrateOffset(){

    REC_vR_MeasOffset_pu = 0;
    REC_vS_MeasOffset_pu = 0;
    REC_vT_MeasOffset_pu = 0;
    REC_iR_MeasOffset_pu = 0;
    REC_iS_MeasOffset_pu = 0;
    REC_iT_MeasOffset_pu = 0;

    REC_I_Aku_P_MeasOffset_pu = 0;
    REC_I_Aku_N_MeasOffset_pu = 0;
//****************************************************//
    uint32_t REC_R_offset_counter = 0;
    uint16_t rec_offset_50us_cnt = 0;
    uint32_t rec_offset_index = 0;

    float32_t rec_R_offset_sum = 0;
    float32_t rec_S_offset_sum = 0;
    float32_t rec_T_offset_sum = 0;

    float32_t rec_iR_offset_sum = 0;
    float32_t rec_iS_offset_sum = 0;
    float32_t rec_iT_offset_sum = 0;

    float32_t rec_I_Aku_P_offset_sum = 0;
    float32_t rec_I_Aku_N_offset_sum = 0;

    float32_t rec_R_offset_inst_sample = 0;
    float32_t rec_S_offset_inst_sample = 0;
    float32_t rec_T_offset_inst_sample = 0;

    float32_t rec_iR_offset_inst_sample = 0;
    float32_t rec_iS_offset_inst_sample = 0;
    float32_t rec_iT_offset_inst_sample = 0;

    float32_t rec_I_Aku_P_offset_inst_sample = 0;
    float32_t rec_I_Aku_N_offset_inst_sample = 0;

    while (REC_R_offset_counter < 100000){
        if (REC_HAL_getPWMInterruptFlag(EPWM5_BASE) == 1){
            rec_offset_50us_cnt++;
            if (rec_offset_50us_cnt >= 5){
                rec_offset_50us_cnt = 0;
                rec_R_offset_inst_sample = REC_V_R_ADC_D * REC_ADC_PU_SCALE_FACTOR;
                rec_R_offset_sum = rec_R_offset_inst_sample+ rec_R_offset_sum;

                rec_S_offset_inst_sample = REC_V_S_ADC_D * REC_ADC_PU_SCALE_FACTOR;
                rec_S_offset_sum = rec_S_offset_inst_sample+ rec_S_offset_sum;

                rec_T_offset_inst_sample = REC_V_T_ADC_D * REC_ADC_PU_SCALE_FACTOR;
                rec_T_offset_sum = rec_T_offset_inst_sample+ rec_T_offset_sum;

                rec_iR_offset_inst_sample = REC_I_R_ADC_D * REC_ADC_PU_SCALE_FACTOR;
                rec_iR_offset_sum = rec_iR_offset_inst_sample+ rec_iR_offset_sum;

                rec_iS_offset_inst_sample = REC_I_S_ADC_D * REC_ADC_PU_SCALE_FACTOR;
                rec_iS_offset_sum = rec_iS_offset_inst_sample+ rec_iS_offset_sum;

                rec_iT_offset_inst_sample = REC_I_T_ADC_D * REC_ADC_PU_SCALE_FACTOR;
                rec_iT_offset_sum = rec_iT_offset_inst_sample+ rec_iT_offset_sum;

                rec_I_Aku_P_offset_inst_sample = REC_I_AKU_P_ADC_D * REC_ADC_PU_SCALE_FACTOR;
                rec_I_Aku_P_offset_sum = rec_I_Aku_P_offset_inst_sample+ rec_I_Aku_P_offset_sum;

                rec_I_Aku_N_offset_inst_sample = REC_I_AKU_N_ADC_D * REC_ADC_PU_SCALE_FACTOR;
                rec_I_Aku_N_offset_sum = rec_I_Aku_N_offset_inst_sample+ rec_I_Aku_N_offset_sum;

                rec_offset_index++;
                if (rec_offset_index >= 20000){

                    REC_vR_MeasOffset_pu = rec_R_offset_sum / rec_offset_index;
                    REC_vS_MeasOffset_pu = rec_S_offset_sum / rec_offset_index;
                    REC_vT_MeasOffset_pu = rec_T_offset_sum / rec_offset_index;

                    REC_iR_MeasOffset_pu = rec_iR_offset_sum / rec_offset_index;
                    I_R_meas_offset_cla = REC_iR_MeasOffset_pu;
                    REC_iS_MeasOffset_pu = rec_iS_offset_sum / rec_offset_index;
                    I_S_meas_offset_cla = REC_iS_MeasOffset_pu;
                    REC_iT_MeasOffset_pu = rec_iT_offset_sum / rec_offset_index;
                    I_T_meas_offset_cla = REC_iT_MeasOffset_pu;

                    REC_I_Aku_P_MeasOffset_pu = rec_I_Aku_P_offset_sum / rec_offset_index;
                    I_battery_upper_meas_offset_cla = REC_I_Aku_P_MeasOffset_pu;
                    REC_I_Aku_N_MeasOffset_pu = rec_I_Aku_N_offset_sum / rec_offset_index;
                    I_battery_lower_meas_offset_cla = REC_I_Aku_N_MeasOffset_pu;

                    rec_offset_index = 0;
                    rec_R_offset_sum = 0;
                    rec_S_offset_sum = 0;
                    rec_T_offset_sum = 0;

                    rec_iR_offset_sum =0;
                    rec_iS_offset_sum =0;
                    rec_iT_offset_sum =0;

                    rec_I_Aku_P_offset_sum=0;
                    rec_I_Aku_N_offset_sum=0;

                }
            }
            REC_R_offset_counter++;
            REC_HAL_clearPWMInterruptFlag(EPWM5_BASE);
        }
    }
}// end of Rec_CalibrateOffset()


//=============================================================================
// Function Name : calculateCRC                           /
// Description   : None                                                      /
// Argument      : None                                                            /
// Return        : None                                                      /
//=============================================================================
uint16_t calculateCRC(uint8_t *data, uint16_t length){
    uint16_t crc = 0xFFFF;
    uint16_t i;
    uint16_t j;

    for (i = 0; i < length; i++){
        crc ^= data[i];
        for (j = 0; j < 8; j++){
            if (crc & 0x0001){
                crc >>= 1;
                crc ^= 0xA001;
            } else{
                crc >>= 1;
            }
        }
    }
    return crc;
}
//=============================================================================


//---------------------------------------------------------------------------/
// Function Name : SaveResponseBufferToMenuValue                           /
// Description   : None                                                      /
// Argument      : None                                                            /
// Return        : None
// NOT: Akım bilgilerini ön panele gönderirken akım değrinden sonra bir virgüllük değerde gönderilir
// NOT2: Kalibrasyon değerlerini 10 kat arttırıp göndererek daha hassas bir işlem yapılabilir
//---------------------------------------------------------------------------/
void SaveResponseBufferToMenuValue    (void){

    if(rec_R_pll_all_stable_true == 1){

        ResponseBuffer[0] = (uint16_t)(Vrms_Line10_R);
        ResponseBuffer[1] = (uint16_t)(Vrms_Line10_S);
        ResponseBuffer[2] = (uint16_t)(Vrms_Line10_T);
        ResponseBuffer[3] = (uint16_t)format_one_decimal_fnc(Irms_Line10_R);
        ResponseBuffer[4] = (uint16_t)format_one_decimal_fnc(Irms_Line10_S);
        ResponseBuffer[5] = (uint16_t)format_one_decimal_fnc(Irms_Line10_T);

        ResponseBuffer[6] = (uint16_t)(Vrms_inv10_R);
        ResponseBuffer[7] = (uint16_t)(Vrms_inv10_S);
        ResponseBuffer[8] = (uint16_t)(Vrms_inv10_T);
        ResponseBuffer[9] =  (uint16_t)format_one_decimal_fnc(Irms_out_R);
        ResponseBuffer[10] = (uint16_t)format_one_decimal_fnc(Irms_out_S);
        ResponseBuffer[11] = (uint16_t)format_one_decimal_fnc(Irms_out_T);

        ResponseBuffer[12] = (uint16_t)(Sout_R);
        ResponseBuffer[13] = (uint16_t)(Sout_S);
        ResponseBuffer[14] = (uint16_t)(Sout_T);
        ResponseBuffer[15] = (uint16_t)(Pout_R);
        ResponseBuffer[16] = (uint16_t)(Pout_S);
        ResponseBuffer[17] = (uint16_t)(Pout_T);

        ResponseBuffer[18] = (uint16_t)(V_DC_bar_upper_avg_20ms);
        ResponseBuffer[19] = (uint16_t)(V_DC_bar_lower_avg_20ms);
        ResponseBuffer[20] = (uint16_t)format_one_decimal_fnc((Rec_I_Aku_P_sum_avg + 1000.0f));
        ResponseBuffer[21] = (uint16_t)format_one_decimal_fnc((Rec_I_Aku_N_sum_avg + 1000.0f));
        ResponseBuffer[22] = (uint16_t)V_Battery_upper;
        ResponseBuffer[23] = (uint16_t)V_Battery_lower;

        if(Vrms_Line10_R > VAC_LINE_MIN){
            ResponseBuffer[24] = (uint16_t)VAC_line_R_freq;
        }else{
            ResponseBuffer[24] = 0.0f;
        }
        if(Vrms_inv10_R > VAC_LINE_MIN){
            ResponseBuffer[25] = (uint16_t)VAC_out_R_freq;
        }else{
            ResponseBuffer[25] = 0.0f;
        }
        ResponseBuffer[26] = (uint16_t)Vrms_bypass_R;
        ResponseBuffer[27] = (uint16_t)Vrms_bypass_S;
        ResponseBuffer[28] = (uint16_t)Vrms_bypass_T;

        ResponseBuffer[29] = ups_mode_status;
        //*** Calibration ***//
        ResponseBuffer[30] = (uint16_t)V_R_in_meas_coeff;
        ResponseBuffer[31] = (uint16_t)V_S_in_meas_coeff;
        ResponseBuffer[32] = (uint16_t)V_T_in_meas_coeff;
        ResponseBuffer[33] = (uint16_t)(I_R_in_meas_coeff);
        ResponseBuffer[34] = (uint16_t)(I_S_in_meas_coeff);
        ResponseBuffer[35] = (uint16_t)(I_T_in_meas_coeff);

        ResponseBuffer[36] = (uint16_t)V_R_out_meas_coeff;
        ResponseBuffer[37] = (uint16_t)V_S_out_meas_coeff;
        ResponseBuffer[38] = (uint16_t)V_T_out_meas_coeff;
        ResponseBuffer[39] = (uint16_t)(I_R_out_meas_coeff);
        ResponseBuffer[40] = (uint16_t)(I_S_out_meas_coeff);
        ResponseBuffer[41] = (uint16_t)(I_T_out_meas_coeff);

        ResponseBuffer[42] = (uint16_t)V_Battery_upper_coeff;
        ResponseBuffer[43] = (uint16_t)V_DC_bar_upper_meas_coeff;
        ResponseBuffer[44] = (uint16_t)V_DC_bar_lower_meas_coeff;
        ResponseBuffer[45] = (uint16_t)(I_battery_upper_meas_coeff);
        ResponseBuffer[46] = (uint16_t)(I_battery_lower_meas_coeff);

        ResponseBuffer[47] = (uint16_t)V_R_bypass_meas_coeff;
        ResponseBuffer[48] = (uint16_t)V_S_bypass_meas_coeff;
        ResponseBuffer[49] = (uint16_t)V_T_bypass_meas_coeff;

        ResponseBuffer[50] = (uint16_t)(I_R_load_meas_coeff);
        ResponseBuffer[51] = (uint16_t)(I_S_load_meas_coeff);
        ResponseBuffer[52] = (uint16_t)(I_T_load_meas_coeff);

        ResponseBuffer[53] = (uint16_t)V_R_load_meas_coeff;
        ResponseBuffer[54] = (uint16_t)V_S_load_meas_coeff;
        ResponseBuffer[55] = (uint16_t)V_T_load_meas_coeff;

        ResponseBuffer[56] = (uint16_t)Vrms_load_R;
        ResponseBuffer[57] = (uint16_t)Vrms_load_S;
        ResponseBuffer[58] = (uint16_t)Vrms_load_T;
        ResponseBuffer[59] = (uint16_t)format_one_decimal_fnc(Irms_load_R);
        ResponseBuffer[60] = (uint16_t)format_one_decimal_fnc(Irms_load_S);
        ResponseBuffer[61] = (uint16_t)format_one_decimal_fnc(Irms_load_T);

        ResponseBuffer[62] = (uint16_t)thyr_input_side_active_fl;
        ResponseBuffer[63] = thyr_bypass_side_active_fl_from_cpu2;
        ResponseBuffer[64] = inv_control_system_active_fl_from_cpu2;
        ResponseBuffer[65] = ups_eeprom_reset_compl_fl;
        ResponseBuffer[66] = Freq_bypass_R;
        ResponseBuffer[67] = Rec_phase_squence_correct_err;
    }
}


//=============================================================================
//  kalibrasyon verilerini kaydetme fonksiyonu
//=============================================================================
void SaveResponseCalibrationValue (void){

    V_R_in_meas_coeff=(float32_t)TI_Recive_Buffer[0];
    V_S_in_meas_coeff=(float32_t)TI_Recive_Buffer[1];
    V_T_in_meas_coeff=(float32_t)TI_Recive_Buffer[2];

    I_R_in_meas_coeff=(float32_t)(TI_Recive_Buffer[3]);
    I_S_in_meas_coeff=(float32_t)(TI_Recive_Buffer[4]);
    I_T_in_meas_coeff=(float32_t)(TI_Recive_Buffer[5]);

    V_R_out_meas_coeff_eeprom=(float32_t)TI_Recive_Buffer[6];
    V_S_out_meas_coeff_eeprom=(float32_t)TI_Recive_Buffer[7];
    V_T_out_meas_coeff_eeprom=(float32_t)TI_Recive_Buffer[8];

    I_R_out_meas_coeff_eeprom=(float32_t)(TI_Recive_Buffer[9]);
    I_S_out_meas_coeff_eeprom=(float32_t)(TI_Recive_Buffer[10]);
    I_T_out_meas_coeff_eeprom=(float32_t)(TI_Recive_Buffer[11]);

    V_Battery_upper_coeff_eeprom=(float32_t)TI_Recive_Buffer[12];
    V_DC_bar_upper_meas_coeff=(float32_t)TI_Recive_Buffer[13];
    V_DC_bar_lower_meas_coeff=(float32_t)TI_Recive_Buffer[14];

    I_battery_upper_meas_coeff=(float32_t)(TI_Recive_Buffer[15]);
    I_battery_lower_meas_coeff=(float32_t)(TI_Recive_Buffer[16]);

    V_R_bypass_meas_coeff_eeprom=(float32_t)TI_Recive_Buffer[17];
    V_S_bypass_meas_coeff_eeprom=(float32_t)TI_Recive_Buffer[18];
    V_T_bypass_meas_coeff_eeprom=(float32_t)TI_Recive_Buffer[19];
    //20. indis eeprom_active_fl için kullanılıyor
    I_R_load_meas_coeff_eeprom=(float32_t)(TI_Recive_Buffer[21]);
    I_S_load_meas_coeff_eeprom=(float32_t)(TI_Recive_Buffer[22]);
    I_T_load_meas_coeff_eeprom=(float32_t)(TI_Recive_Buffer[23]);

    V_R_load_meas_coeff_eeprom=(float32_t)TI_Recive_Buffer[24];
    V_S_load_meas_coeff_eeprom=(float32_t)TI_Recive_Buffer[25];
    V_T_load_meas_coeff_eeprom=(float32_t)TI_Recive_Buffer[26];

    rec_uart_comm_active_for_real_term_fl=TI_Recive_Buffer[27]; // indis rec_uart_comm_active_for_real_term_fl
    inv_uart_comm_active_for_real_term_fl=TI_Recive_Buffer[28]; // indis inv_uart_comm_active_for_real_term_fl
    rec_state_from_front_panel=TI_Recive_Buffer[29]; // rec state from frontpanel
    inv_state_from_front_panel=TI_Recive_Buffer[30]; // inv state from frontpanel
    byp_state_from_front_panel=TI_Recive_Buffer[31]; // bypas state from frontpanel
//    ups_error_count_index_from_front_panel=TI_Recive_Buffer[32]; // ups errors data count from front panel // bu veri hep okunmalı bu yüzden haberleşme var ise değer oradana alınmalı

   //rec_kontrol sistem parametrelerinin güncellenmesi
    I_R_meas_cla_coff=I_R_in_meas_coeff;
    I_S_meas_cla_coff=I_S_in_meas_coeff;
    I_T_meas_cla_coff=I_T_in_meas_coeff;
    V_DC_bar_upper_meas_cla_coff=V_DC_bar_upper_meas_coeff;
    V_DC_bar_lower_meas_cla_coff=V_DC_bar_lower_meas_coeff;
    I_battery_upper_meas_cla_coff=I_battery_upper_meas_coeff;
    I_battery_lower_meas_cla_coff=I_battery_lower_meas_coeff;

}


//=============================================================================
//=============================================================================
// Gerilim değerini istenen formata dönüştüren fonksiyon
uint16_t format_one_decimal_fnc(float32_t voltage) {

    uint16_t whole_voltage = (uint16_t)voltage; // Take the whole part
    uint16_t after_decimal_one = (uint16_t)((voltage - whole_voltage) * 10); // Get the value after the decimal point
    uint16_t after_decimal_two = (uint16_t)((voltage - whole_voltage) * 100) % 10; // Get the value after the second decimal point


    // Check the value after the second decimal point to round up or down
    if (after_decimal_two >= 5) {
        after_decimal_one++;
    } else if (after_decimal_two < 5) {
        after_decimal_one--;
    }

if(voltage >= 0.1f){
    whole_voltage = (whole_voltage * 10) + after_decimal_one;
}
    return (uint16_t)whole_voltage;
}
//=============================================================================

//=============================================================================
// Gerilim değerini istenen formata dönüştüren fonksiyon
uint16_t format_two_decimal(float32_t value) {
    uint16_t whole_value = (uint16_t)value; // Take the whole part
    uint16_t after_decimal_one = (uint16_t)((value - whole_value) * 10); // Get the value after the decimal point
    uint16_t after_decimal_two = (uint16_t)((value - whole_value) * 100) % 10; // Get the value after the second decimal point
    uint16_t after_decimal_three = (uint16_t)((uint16_t)((value - whole_value) * 1000) % 100) % 10; // Get the value after the second decimal point


    // Check the value after the second decimal point to round up or down
    if (after_decimal_three >= 5) {
        after_decimal_two++;
    } else if (after_decimal_three < 5) {
        after_decimal_two--;
    }

if(value >= 0.1f){
    whole_value = (after_decimal_one * 10) + after_decimal_two;
}
    return (uint16_t)whole_value;
}
//=============================================================================



//=============================================================================
// writeDataCPU1 - Write a pattern to an array in shared RAM CPU2 ye gönderilecek veriler
//eeprom
//=============================================================================
void writeDataCPU1(void){

    cpu1RWArray[0] = CPU1_TO_CPU2_COMM_KEY;
    cpu1RWArray[1] = Vrms_Line10_R;
    cpu1RWArray[2] = thyr_input_side_active_fl;
    cpu1RWArray[3] = (float32_t)uart_core;
    cpu1RWArray[4] = V_R_in_meas;
    cpu1RWArray[5] = V_S_in_meas;
    cpu1RWArray[6] = V_T_in_meas;
    cpu1RWArray[7] = Vrms_Line10_S;
    cpu1RWArray[8] = Vrms_Line10_T;
    cpu1RWArray[9] = (float32_t)inv_uart_comm_active;

    // NOT: Sistem ilk açıldığında eepromdan okunan verilerin cpu2 tarafına aktarılması gerekli
    // NOT: eğer ön panelden yazma işlemi aktif edilirse tekrardan cpu2 tarafındaki veriler güncellenmeli
    if(I2C_eeprom_read_active_fl==1 || ups_calibrate_active_fl == 1){
        cpu1RWArray[10] = (float32_t)24.0;
        cpu1RWArray[11] = V_R_out_meas_coeff_eeprom;
        cpu1RWArray[12] = V_S_out_meas_coeff_eeprom;
        cpu1RWArray[13] = V_T_out_meas_coeff_eeprom;

        cpu1RWArray[14] = I_R_out_meas_coeff_eeprom;
        cpu1RWArray[15] = I_S_out_meas_coeff_eeprom;
        cpu1RWArray[16] = I_T_out_meas_coeff_eeprom;

        cpu1RWArray[17] = V_Battery_upper_coeff_eeprom;

        cpu1RWArray[18] = V_R_bypass_meas_coeff_eeprom;
        cpu1RWArray[19] = V_S_bypass_meas_coeff_eeprom;
        cpu1RWArray[20] = V_T_bypass_meas_coeff_eeprom;

        cpu1RWArray[21] = I_R_load_meas_coeff_eeprom;//load
        cpu1RWArray[22] = I_S_load_meas_coeff_eeprom;
        cpu1RWArray[23] = I_T_load_meas_coeff_eeprom;

        cpu1RWArray[24] = V_R_load_meas_coeff_eeprom;
        cpu1RWArray[25] = V_S_load_meas_coeff_eeprom;
        cpu1RWArray[26] = V_T_load_meas_coeff_eeprom;
        I2C_eeprom_read_active_fl=0;
    }else{
        cpu1RWArray[10] = (float32_t)0.0;
    }
    cpu1RWArray[27] = V_DC_bar_upper_meas;
    cpu1RWArray[28] = V_DC_bar_lower_meas;
    cpu1RWArray[29] = Vrms_line1_R;
    cpu1RWArray[30] = Vrms_line1_S;
    cpu1RWArray[31] = Vrms_line1_T;
    cpu1RWArray[32] = VAC_line_R_freq;
    cpu1RWArray[33] = Rec_phase_squence_correct;

    cpu1RWArray[34] = (float32_t)panel_to_main_board_write_active;
    cpu1RWArray[35] = (float32_t)inv_state_from_front_panel;
    cpu1RWArray[36] = (float32_t)byp_state_from_front_panel;

    cpu1RWArray[37] = (float32_t)inv_err_record_fl;   // inverter sistemine arızanın vektöre yazıldığını gönderen değişken

    IPC_setFlagLtoR(IPC_CPU1_L_CPU2_R, CPU1_TO_CPU2_COMM_IPC_FLAG);
}
//=============================================================================

//=============================================================================
// readDataCPU1 - Read and compare an array from shared RAM
//// Read cpu1RArray and modify cpu2RWArray
void readDataCPU1(void){

if(IPC_isFlagBusyRtoL(IPC_CPU2_L_CPU1_R, CPU2_TO_CPU1_COMM_IPC_FLAG) == 1){

    if(cpu1RArray[0] == CPU2_TO_CPU1_COMM_KEY){
        VAC_out_R_freq=cpu1RArray[1];

        Vrms_inv10_R=cpu1RArray[2];
        Vrms_inv10_S=cpu1RArray[3];
        Vrms_inv10_T=cpu1RArray[4];
        Irms_out_R=cpu1RArray[5];
        Irms_out_S=cpu1RArray[6];
        Irms_out_T=cpu1RArray[7];

        Pout_R=cpu1RArray[8];
        Pout_S=cpu1RArray[9];
        Pout_T=cpu1RArray[10];
        Sout_R=cpu1RArray[11];
        Sout_S=cpu1RArray[12];
        Sout_T=cpu1RArray[13];

        Vrms_bypass_R=cpu1RArray[14];
        Vrms_bypass_S=cpu1RArray[15];
        Vrms_bypass_T=cpu1RArray[16];

        V_Battery_upper=cpu1RArray[17];
        V_Battery_lower=V_Battery_upper;

        ups_mode_status=(uint16_t)cpu1RArray[18];

        //kalibrasyon katsayıları ekrana gönder
        V_Battery_upper_coeff=cpu1RArray[19];//battery

        V_R_out_meas_coeff=cpu1RArray[20];//inv
        V_S_out_meas_coeff=cpu1RArray[21];
        V_T_out_meas_coeff=cpu1RArray[22];
        I_R_out_meas_coeff=cpu1RArray[23];
        I_S_out_meas_coeff=cpu1RArray[24];
        I_T_out_meas_coeff=cpu1RArray[25];

        V_R_bypass_meas_coeff=cpu1RArray[26];//bypass
        V_S_bypass_meas_coeff=cpu1RArray[27];
        V_T_bypass_meas_coeff=cpu1RArray[28];

        I_R_load_meas_coeff=cpu1RArray[29];//load
        I_S_load_meas_coeff=cpu1RArray[30];
        I_T_load_meas_coeff=cpu1RArray[31];
        V_R_load_meas_coeff=cpu1RArray[32];
        V_S_load_meas_coeff=cpu1RArray[33];
        V_T_load_meas_coeff=cpu1RArray[34];

        //load measure value
        Irms_load_R=cpu1RArray[35];
        Irms_load_S=cpu1RArray[36];
        Irms_load_T=cpu1RArray[37];
        Vrms_load_R=cpu1RArray[38];
        Vrms_load_S=cpu1RArray[39];
        Vrms_load_T=cpu1RArray[40];

        //INV system max and min values
        Vrms_inv1_R_max_cpu2=cpu1RArray[41];
        Vrms_inv1_S_max_cpu2=cpu1RArray[42];
        Vrms_inv1_T_max_cpu2=cpu1RArray[43];

        Irms_out1_R_max_cpu2=cpu1RArray[44];
        Irms_out1_S_max_cpu2=cpu1RArray[45];
        Irms_out1_T_max_cpu2=cpu1RArray[46];

        Vrms_load1_R_max_cpu2=cpu1RArray[47];
        Vrms_load1_S_max_cpu2=cpu1RArray[48];
        Vrms_load1_T_max_cpu2=cpu1RArray[49];

        Irms_load1_R_max_cpu2=cpu1RArray[50];
        Irms_load1_S_max_cpu2=cpu1RArray[51];
        Irms_load1_T_max_cpu2=cpu1RArray[52];

        Vrms_bypass1_R_max_cpu2=cpu1RArray[53];
        Vrms_bypass1_S_max_cpu2=cpu1RArray[54];
        Vrms_bypass1_T_max_cpu2=cpu1RArray[55];

        V_battery_upper_max_cpu2=cpu1RArray[56];

        V_R_inv_meas_max_cpu2=cpu1RArray[57];
        V_S_inv_meas_max_cpu2=cpu1RArray[58];
        V_T_inv_meas_max_cpu2=cpu1RArray[59];

        I_R_inv_meas_max_cpu2=cpu1RArray[60];
        I_S_inv_meas_max_cpu2=cpu1RArray[61];
        I_T_inv_meas_max_cpu2=cpu1RArray[62];

        I_R_load_meas_max_cpu2=cpu1RArray[63];
        I_S_load_meas_max_cpu2=cpu1RArray[64];
        I_T_load_meas_max_cpu2=cpu1RArray[65];

        thyr_bypass_side_active_fl_from_cpu2=(uint16_t)cpu1RArray[66];
        inv_control_system_active_fl_from_cpu2=(uint16_t)cpu1RArray[67];

        inv_err_active_for_cpu2=(uint16_t)cpu1RArray[68];
        error_code_for_inv_from_cpu2=(uint16_t)cpu1RArray[69];
        ups_inv_data_log_send_terminal_active_cpu2=(uint16_t)cpu1RArray[70];

        Freq_bypass_R=(uint16_t)cpu1RArray[71];

    }
    IPC_ackFlagRtoL(IPC_CPU2_L_CPU1_R, CPU2_TO_CPU1_COMM_IPC_FLAG);
  }
}
//=============================================================================

//=============================================================================
// Function Name : InvAddFault
// Description   : Hata ekleme fonksiyonu
//               : fonksiyonu, yeni bir hata eklendiğinde eski hataları bir aşağı kaydırarak, yeni hatayı en üste ekler.                                                      /
// Argument      : None
// Return        : None
//=============================================================================

void InvAddFault(Fault_Flag flag) {

    // Eğer en üstteki hata ile yeni hata aynı ise, ekleme yapma
    if (faultFlags[0] == flag) {
        return;
    }
    uint16_t i;
    // Yeni hata eklendiğinde eski hataları bir aşağı kaydır
    for( i = MAX_FAULT_HISTORY - 1; i > 0; i--) {
        faultFlags[i] = faultFlags[i - 1];
    }
    // Yeni hatayı en üste ekle
    faultFlags[0] = flag;
}


////=============================================================================
////  SCR SOFT START ALGORITHM for Rectifier R phase
//// " rec_R_pll_stable_timer " rectifier'in R fazı icin uygulanan pll'in 1 saniye stabile olma koşulu
//// NOT!!!!= "rec_R_pos_scr_gpio=off" pozitif taraftaki scr ile işlem bittiyse negatif bara dolana kadar scr1=off olmalı yoksa
//// İki SCR aynı anda aktif olursa rezonans oluşuyor peak akımlar artıyor soft start olmuyor
////=============================================================================
//void handle_cap_charge_fnc() {
//
//   if(cap_charge_request==1){
//    //SCR1+
//    if (rec_R_pos_bus_cap_charge_flg == 0){
//      if (rec_R_neg_to_pos_cross_flg == 1) {
//
//        if (rec_R_pos_scr_bus_1s_timer_clamp == 0) {
//          rec_R_pos_scr_bus_1s_timer++;
//        }
//
//        //
//        //scr 1 saniye %5 de ateşlensin
//        //
//        if (rec_R_sin_hold_time_cnt >= rec_R_pos_sine_hold * 0.95f && rec_R_pos_scr_bus_1s_timer_clamp == 0) {
//          rec_R_neg_to_pos_cross_flg = 0;
//          wrpin(rec_R_pos_scr_gpio, SCRStageON);
//          rec_R_pos_scr_fire_flag = 1;
//        }
//
//        //
//        //her 20ms de bir 50Hz için 20ms*50=1s
//        //
//        if (rec_R_pos_scr_bus_1s_timer == 50) {
//          rec_R_pos_scr_bus_1s_timer_clamp = 1;
//
//          if (rec_R_sin_hold_time_cnt >= rec_R_pos_sine_hold * rec_R_pos_sine_hold_scale) {
//
//            wrpin(rec_R_pos_scr_gpio, SCRStageON);
//            rec_R_pos_scr_fire_flag = 1;
//            rec_R_neg_to_pos_cross_flg = 0;
//            rec_R_pos_sine_hold_scale = rec_R_pos_sine_hold_scale - 0.02f; //0.02 1saniye
//            if (rec_R_pos_sine_hold_scale <= 0.0f) {
//              rec_R_pos_bus_cap_charge_flg = 1;
//            }
//          }
//        }
//      }
//
//      //
//      //eğer pozitif scr ateşlenmiş ise diğer cycle geçmeden ateşleme sinyali LOW olmalı (500*10us=5ms, 100*50us=5ms)
//      //
//      if (rec_R_pos_scr_fire_flag == 1) {
//        rec_R_pos_scr_low_timer++;
//      }
//
//      if (rec_R_pos_scr_low_timer >= 100) {
//        rec_R_pos_scr_fire_flag = 0;
//        rec_R_pos_scr_low_timer = 0;
//        wrpin(rec_R_pos_scr_gpio, SCRStageOFF);
//      }
//    } // positive scr is fired for R pahse of the rectifier
//
//    //
//    // sine signal negative side scr fire for R phase of the rectifier
//    ////SCR2-
//    if (rec_R_neg_bus_cap_charge_flg == 0  && rec_R_pos_bus_cap_charge_flg == 1){
//
//        wrpin(rec_R_pos_scr_gpio, SCRStageOFF);
//      //
//      //negative side scr for R phase
//      //
//      if (rec_R_pos_to_neg_cross_flg == 1){
//
//        if (rec_R_neg_scr_bus_1s_timer_clamp == 0){
//          rec_R_neg_scr_bus_1s_timer++;
//        }
//
//        if (rec_R_sin_hold_time_cnt >= rec_R_neg_sine_hold * 0.95f && rec_R_neg_scr_bus_1s_timer_clamp == 0){
//          rec_R_pos_to_neg_cross_flg = 0;
//          rec_R_neg_scr_fire_flg = 1;
//          wrpin(rec_R_neg_scr_gpio, SCRStageON);
//        }
//        if (rec_R_neg_scr_bus_1s_timer == 50) {
//          rec_R_neg_scr_bus_1s_timer_clamp = 1;
//
//          if (rec_R_sin_hold_time_cnt >= rec_R_neg_sine_hold * rec_R_neg_sine_hold_scale) {
//            wrpin(rec_R_neg_scr_gpio, SCRStageON);
//            rec_R_neg_scr_fire_flg = 1;
//            rec_R_pos_to_neg_cross_flg = 0;
//            rec_R_neg_sine_hold_scale = rec_R_neg_sine_hold_scale - 0.02f;
//            if (rec_R_neg_sine_hold_scale <= 0.0f) {
//              rec_R_neg_bus_cap_charge_flg = 1;
//            }
//          }
//        }
//      }
//
//      if (rec_R_neg_scr_fire_flg == 1) {
//        rec_R_neg_scr_low_timer++;
//      }
//
//      if (rec_R_neg_scr_low_timer >= 100) {
//        rec_R_neg_scr_fire_flg = 0;
//        rec_R_neg_scr_low_timer = 0;
//        wrpin(rec_R_neg_scr_gpio, SCRStageOFF);
//      }
//    } // negative scr is fired
//
//
//    if (rec_R_neg_bus_cap_charge_flg == 1 ) {
//      wrpin(rec_R_pos_scr_gpio, SCRStageON);
//      wrpin(rec_R_neg_scr_gpio, SCRStageON);
//      wrpin(rec_S_pos_scr_gpio, SCRStageON);
//      wrpin(rec_T_pos_scr_gpio, SCRStageON);
//          cap_charge_request=0;
//          thyr_input_side_active_fl=1;
//          rec_data_log_meas_active_fl=0;
//          I_in_30u_fault_try_return=0;
//          V_DC_bar_fast_err_try_return=0;
//          I_battery_charge_err_try_return=0;
//          rec_over_load_100_125_fault_try_return=0;
//          rec_over_load_125_150_fault_try_return=0;
//          rec_igbt_fault_try_return=0;
//      rec_R_pos_bus_cap_charge_flg = 0;
//      rec_R_neg_bus_cap_charge_flg = 0;
//      rec_R_pos_scr_bus_1s_timer_clamp = 0;
//      rec_R_neg_scr_bus_1s_timer_clamp = 0;
//      rec_R_pos_scr_bus_1s_timer = 0;
//      rec_R_neg_scr_bus_1s_timer =0;
//      rec_R_sin_hold_time_cnt = 0;
//      rec_R_pos_scr_fire_flag = 0;
//      rec_R_neg_scr_fire_flg = 0;
//
//      rec_R_pos_sine_hold_scale = 0.95f;
//      rec_R_neg_sine_hold_scale = 0.95f;
//      rec_R_pos_scr_low_timer = 0;
//      rec_R_neg_scr_low_timer = 0;
//    }
//  }
//} // end of handle_cap_charge_fnc


void stop_pwm_PFC_boost(void){
   DB_FED_PWM(REC_PWM_R_BASE,false);
   DB_RED_PWM(REC_PWM_R_BASE,false);
   pwm_duty_a(REC_PWM_R_BASE,1);
   pwm_duty_b(REC_PWM_R_BASE,1);

   DB_FED_PWM(REC_PWM_S_BASE,false);
   DB_RED_PWM(REC_PWM_S_BASE,false);
   pwm_duty_a(REC_PWM_S_BASE,1);
   pwm_duty_b(REC_PWM_S_BASE,1);

   DB_FED_PWM(REC_PWM_T_BASE,false);
   DB_RED_PWM(REC_PWM_T_BASE,false);
   pwm_duty_a(REC_PWM_T_BASE,1);
   pwm_duty_b(REC_PWM_T_BASE,1);
   PFC_boost_first_ref_set=0;
}
void turn_off_input_thyr(void){
    wrpin(rec_R_pos_scr_gpio, SCRStageOFF);
    wrpin(rec_R_neg_scr_gpio, SCRStageOFF);
    wrpin(rec_S_pos_scr_gpio, SCRStageOFF);
    wrpin(rec_T_pos_scr_gpio, SCRStageOFF);
}



//*****************************************************************************
//
// Modbus read array
//
//*****************************************************************************
uint16_t Modbus_read(uint32_t base, uint16_t * const array, uint16_t length,uint16_t com_start_byte){
    ASSERT(SCI_isBaseValid(base));
    uint16_t i;
    volatile uint16_t recive_rx_mesag;

   recive_rx_mesag = (uint16_t)(HWREGH(base + SCI_O_RXBUF) & SCI_RXBUF_SAR_M);
   if(recive_rx_mesag == com_start_byte){
       recive_rx_mesag=0;
       for(i = 1U; i < length; i++){
           // Wait until a character is available in the receive buffer.
           uart_timeout_counter=0;
           while(!SCI_isDataAvailableNonFIFO(base) ){
               // Zaman aşımını kontrol et
               if (uart_timeout_counter  > 60000) {
                   uart_intrrupt_fl_clear_cnt_debug++;
                   return 1; // bir hata döndürebilirsiniz
               }
           }
           // Return the character from the receive buffer.
           array[i] = (uint16_t)(HWREGH(base + SCI_O_RXBUF) & SCI_RXBUF_SAR_M);
       }
   }
   return 0;
}


// Arıza kaydını ekleyen fonksiyon
uint16_t addError(uint32_t errorCode, uint32_t timestamp, uint16_t ups_err_count ) {
    uint32_t combinedError = (errorCode & 0xFF) | ((timestamp & 0xFFFFFF) << 8); // ilk 8 bit arıza kodu son 24 bit zaman bilgisi
    uint32_t i;

    if (ups_err_count < UPS_MAX_ERRORS) {
        // bir önceki arızalar bir alt satıra geçiyor
        for (i = ups_err_count; i > 0; i--) {
            ups_error_vector[i] = ups_error_vector[i - 1];
        }
        // Yeni arıza kaydını en üste ekle
        ups_error_vector[0] = combinedError;
        ups_err_count++;
    } else {
        // Vektör doluysa, en eski kaydı sil ve yeni kaydı ekle
        for (i = UPS_MAX_ERRORS - 1; i > 0; i--) {
            ups_error_vector[i] = ups_error_vector[i - 1];
        }
        ups_error_vector[0] = combinedError;
    }
    return ups_err_count;
}



