/*
 * variable.c
 *
 *  Created on: 2 Kas 2023
 *      Author: ENES TASTEKIN
 */
#include "variable.h"
//#include "my_functions.h"


 //=============================================================================
 //UPS_1_V1 VARIABLE

 uint16_t i = 0;
 uint16_t j = 0;

uint16_t V_R_in_meas_debug=0;
uint16_t V_S_in_meas_debug=0;
uint16_t V_T_in_meas_debug=0;
uint16_t SCI_read_debug=0;

uint16_t rec_pwm_duty_debug=1785;
uint16_t rec_pwm_duty_debug_1=1585;
uint16_t rec_pwm_duty_debug_2=1285;
uint16_t fault_pin_current_state_debug=0;
uint16_t igbt_fault_fl_debug=0;

uint16_t fault_pin_current_state_for_rec_igbt_R=0;
uint16_t fault_pin_current_state_cnt_for_rec_igbt_R=0;
uint16_t fault_pin_last_state_for_rec_igbt_R=0;
volatile bool rec_igbt_R_err=0;
volatile bool rec_igbt_fault_try_return=0;
volatile bool rec_igbt_R_fault_persists=0;
uint16_t rec_igbt_R_fault_repeat=0;
uint16_t return_rec_igbt_fault_fault_cnt=0;
uint16_t accept_rec_igbt_fault_fault_return_cnt=0;

uint16_t fault_pin_current_state_for_rec_igbt_S=0;
uint16_t fault_pin_current_state_cnt_for_rec_igbt_S=0;
uint16_t fault_pin_last_state_for_rec_igbt_S=0;
volatile bool rec_igbt_S_err=0;
volatile bool rec_igbt_S_fault_persists=0;
uint16_t rec_igbt_S_fault_repeat=0;

uint16_t fault_pin_current_state_for_rec_igbt_T=0;
uint16_t fault_pin_current_state_cnt_for_rec_igbt_T=0;
uint16_t fault_pin_last_state_for_rec_igbt_T=0;
volatile bool rec_igbt_T_err=0;
volatile bool rec_igbt_T_fault_persists=0;
uint16_t rec_igbt_T_fault_repeat=0;

//=============================================================================
// Allocate all the below shared variables across C28x1 and C28x2 to a
// "DualCoreVariables" section. This section is allocated to RAMGS1 AND RAMGS20 in linker cmd file
//bu bölüme yazılacak değişkenler CPU1 ve CPU2 tarafından ortak kullanlıcak değişkenler olmalı
//=============================================================================
volatile float32_t cpu1RArray[256];       // Mapped to GS5 of shared RAM owned by CPU2
volatile float32_t cpu1RWArray[256];      // Mapped to GS4 of shared RAM owned by CPU1
#pragma DATA_SECTION(cpu1RArray,"SHARERAMGS5");  //CPU2
#pragma DATA_SECTION(cpu1RWArray,"SHARERAMGS4"); //CPU1

//read variables

volatile float32_t Vrms_inv10_R=0;
volatile float32_t Vrms_inv10_S=0;
volatile float32_t Vrms_inv10_T=0;
volatile float32_t Irms_out_R=0;
volatile float32_t Irms_out_S=0;
volatile float32_t Irms_out_T=0;
volatile float32_t Irms_load_R=0;
volatile float32_t Irms_load_S=0;
volatile float32_t Irms_load_T=0;
volatile float32_t Vrms_load_R=0;
volatile float32_t Vrms_load_S=0;
volatile float32_t Vrms_load_T=0;
volatile float32_t Pout_R=0;
volatile float32_t Pout_S=0;
volatile float32_t Pout_T=0;
volatile float32_t Sout_R=0;
volatile float32_t Sout_S=0;
volatile float32_t Sout_T=0;
volatile float32_t V_Battery_upper=0;
volatile float32_t V_Battery_lower=0;
volatile float32_t VAC_out_R_freq=0;
volatile float32_t Vrms_bypass_R=0;
volatile float32_t Vrms_bypass_S=0;
volatile float32_t Vrms_bypass_T=0;
volatile float32_t Freq_bypass_R=0;


float32_t v_meas_debug=0;

volatile uint16_t ups_mode_status=0;

volatile float32_t V_Battery_upper_coeff=V_BATTERY_MEAS_COFF;
volatile float32_t V_R_out_meas_coeff=V_OUT_MEAS_COEFF;
volatile float32_t V_S_out_meas_coeff=V_OUT_MEAS_COEFF;
volatile float32_t V_T_out_meas_coeff=V_OUT_MEAS_COEFF;
volatile float32_t I_R_out_meas_coeff=I_OUT_MEAS_COEFF;
volatile float32_t I_S_out_meas_coeff=I_OUT_MEAS_COEFF;
volatile float32_t I_T_out_meas_coeff=I_OUT_MEAS_COEFF;
volatile float32_t V_R_bypass_meas_coeff=V_BYPASS_MEAS_COEFF;
volatile float32_t V_S_bypass_meas_coeff=V_BYPASS_MEAS_COEFF;
volatile float32_t V_T_bypass_meas_coeff=V_BYPASS_MEAS_COEFF;
volatile float32_t I_R_load_meas_coeff=I_LOAD_MEAS_COEFF;
volatile float32_t I_S_load_meas_coeff=I_LOAD_MEAS_COEFF;
volatile float32_t I_T_load_meas_coeff=I_LOAD_MEAS_COEFF;
volatile float32_t V_R_load_meas_coeff=V_LOAD_MEAS_COEFF;
volatile float32_t V_S_load_meas_coeff=V_LOAD_MEAS_COEFF;
volatile float32_t V_T_load_meas_coeff=V_LOAD_MEAS_COEFF;

volatile float32_t V_Battery_upper_coeff_eeprom=0;
volatile float32_t V_R_out_meas_coeff_eeprom=0;
volatile float32_t V_S_out_meas_coeff_eeprom=0;
volatile float32_t V_T_out_meas_coeff_eeprom=0;
volatile float32_t I_R_out_meas_coeff_eeprom=0;
volatile float32_t I_S_out_meas_coeff_eeprom=0;
volatile float32_t I_T_out_meas_coeff_eeprom=0;
volatile float32_t V_R_bypass_meas_coeff_eeprom=0;
volatile float32_t V_S_bypass_meas_coeff_eeprom=0;
volatile float32_t V_T_bypass_meas_coeff_eeprom=0;

volatile float32_t I_R_load_meas_coeff_eeprom=0;
volatile float32_t I_S_load_meas_coeff_eeprom=0;
volatile float32_t I_T_load_meas_coeff_eeprom=0;
volatile float32_t V_R_load_meas_coeff_eeprom=0;
volatile float32_t V_S_load_meas_coeff_eeprom=0;
volatile float32_t V_T_load_meas_coeff_eeprom=0;

volatile bool I2C_eeprom_active_fl=0;
volatile bool I2C_eeprom_read_active_fl=0;
volatile bool I2C_eeprom_write_fl=0;
uint16_t I2C_eeprom_read_active_key=0;
uint16_t I2C_eeprom_read_active_fl_debug=0;
uint16_t I2C_eeprom_read_active_fl_debug2=0;

float32_t I2C_eeprom_read_debug=0;
float32_t I2C_eeprom_read_debug1=0;
float32_t I2C_eeprom_read_debug2=0;
float32_t I2C_eeprom_read_debug3=0;

bool eemprom_write_debug=0;
bool eemprom_read_startup_fl=0;
uint16_t eeprom_status=0;
uint16_t eeprom_status1=0;
uint16_t eeprom_read_debug_val=0;
uint16_t eeprom_read_debug_val2=0;
uint16_t eeprom_write_debug_val=125;
uint16_t eeprom_write_debug_val2=36;
uint16_t eeprom_write_debug_val3=24;

volatile uint16_t  rec_err_eeprom_wr_active_fl=0;
volatile uint16_t  inv_err_eeprom_wr_active_fl=0;
volatile uint16_t  inv_err_active_for_cpu2=0;
volatile uint16_t  inv_err_record_fl=0;
volatile uint16_t  error_code_for_inv_from_cpu2=0;
volatile uint16_t  ups_inv_data_log_send_terminal_active_cpu2=0;
uint16_t err_eeprom_write_debug_cnt=0;
uint16_t err_eeprom_write_debug_cnt1=0;

uint16_t ups_eeprom_err_adrr=UPS_EEPROM_ERR_DATA_ADRR; // 0x0580h 12.page
//write variables

//=============================================================================

 Fault_Mode faultMode=F_None;
 Rec_Err rec_err=None;
 Fault_Flag faultFlags[MAX_FAULT_HISTORY];

 //FLAGS
 volatile bool cap_charge_request=0;
 volatile bool thyr_input_side_active_fl=0;
 volatile bool V_Line_Slow_Low_Err=0;
 volatile bool V_Line_Slow_High_Err=0;
 volatile bool V_line_fast_Low_Err=0;
 volatile bool V_Line_fast_high_Err=0;
 volatile bool V_line_rms_range_is_OK=0;
 volatile bool V_line_freq_range_is_OK=0;
 volatile bool V_line_range_is_OK=0;
 volatile bool allow_control_syst_PFC_boost=0;
 volatile bool Vrms_line1_R_is_in_user_range=0;
 volatile bool Vrms_line1_S_is_in_user_range=0;
 volatile bool Vrms_line1_T_is_in_user_range=0;

 volatile bool Vrms_line1_R_is_in_slow_range=0;
 volatile bool Vrms_line1_S_is_in_slow_range=0;
 volatile bool Vrms_line1_T_is_in_slow_range=0;


 // rectifier işlemi için gerekli şart değişkenleri VR=220%+-20 176/265
 volatile float32_t V_Line_Slow_Limit_Min=VAC_LINE_MIN;
 volatile float32_t V_Line_Slow_Limit_Max=VAC_LINE_MAX;
 volatile float32_t V_Line_Fast_Limit_Min=VAC_LINE_MIN_FAST;
 volatile float32_t V_Line_Fast_Limit_Max=VAC_LINE_MAX_FAST;


 uint16_t accept_V_line_slow_low_cnt=0;
 uint16_t accept_V_line_slow_high_cnt=0;
 uint16_t cap_charge_request_delay_cnt=0;
 volatile uint16_t thyr_input_side_turn_off_complete_delay=0;
 uint16_t thyr_deactivate_cnt=0;

 volatile uint16_t return_from_V_line_slow_err_cnt=0;
 volatile uint16_t Return_From_V_Bus_Fault_cnt=0;

 volatile uint16_t return_from_V_line_fault_cnt_deb=0;
 volatile uint16_t accept_V_line_slow_range_fault_cnt_deb=0;
 volatile uint16_t V_Line_Slow_Err_reset_deb=0;
 volatile uint16_t V_Line_Slow_Low_Err_deb=0;
 volatile uint16_t V_Line_Slow_High_Err_deb=0;
 volatile uint16_t accept_V_line_slow_high_cnt_deb=0;
 volatile uint16_t accept_V_line_slow_low_cnt_deb=0;
 volatile uint16_t V_line_fast_Low_Err_deb=0;
 volatile uint16_t V_Line_fast_high_Err_deb=0;

 volatile uint16_t cap_charge_request_deb_1=0;


volatile float32_t V_DC_bar_under_volt_protect_activate_lim_volt_min=V_DC_BAR_START_LOW;

float32_t rec_R_phase_seq_V_hold;
float32_t rec_S_phase_seq_V_hold;
float32_t rec_T_phase_seq_V_hold;

volatile float32_t I_in_30u_protc_lim=I1_IN_MAX_30U_PROTC_LIM; //I1_IN_MAX_30U_PROTC_LIM
uint16_t accept_I_in_R_30u_fault_cnt=0;
uint16_t accept_I_in_S_30u_fault_cnt=0;
uint16_t accept_I_in_T_30u_fault_cnt=0;
volatile uint32_t return_I_in_30u_fault_cnt=0;
uint32_t accept_critical_fault_return_cnt=0;
volatile uint16_t I_in_30u_fault_repeat=0;
uint16_t I_in_30u_fault_recovered_from_repeat=0;
uint32_t return_from_I_in_30u_fault_period=RETURN_FROM_I_IN_30U_FAULT_PERIOD;
volatile bool I_in_R_30u_fault=0;
volatile bool I_in_S_30u_fault=0;
volatile bool I_in_T_30u_fault=0;
volatile bool I_in_30u_fault_persists=0;
volatile bool I_in_30u_fault_try_return=0;
volatile bool start_system_from_cap_charge=1;


uint16_t DC_bar_fast_fault_recovered_from_repeat=0;
volatile bool V_DC_bar_fast_err_try_return=0;
volatile bool DC_bar_fast_fault_persists=0;
volatile uint16_t return_DC_bar_fast_fault_cnt=0;
volatile uint16_t return_from__DC_bar_fast_fault_period=RETURN_FROM_DC_BAR_FAST_FAULT_PERIOD;
uint16_t DC_bar_fast_fault_repeat=0;
float32_t V_DC_bar_lower_fast_high_err_hold=0;
float32_t V_DC_bar_upper_fast_high_err_hold=0;
float32_t V_DC_bar_upper_fast_low_err_hold=0;
float32_t V_DC_bar_lower_fast_low_err_hold=0;


// DC_BUS UNDER VOLTAJE PROTECTİON ON OFF FLAG
volatile bool V_DC_bar_upper_fast_under_volt_protc_actv=0;
volatile bool V_DC_bar_lower_fast_under_volt_protc_actv=0;

volatile float32_t I_battery_charge_lim=I_BATTARY_CHARGE_LIM;
uint16_t I_battery_charge_accept_period=I_BATTARY_CHARGE_ACCEPT_PERIOD;
uint16_t I_lower_battery_charge_fault_accept_cnt=0;
uint16_t I_upper_battery_charge_fault_accept_cnt=0;
volatile bool I_lower_battery_charge_err=0;
volatile bool I_upper_battery_charge_err=0;
uint16_t return_I_battery_charge_fault_cnt=0;
volatile bool I_battery_charge_err_try_return=0;
volatile bool I_battery_charge_fault_persists=0;
volatile uint16_t return_from_I_battery_charge_fault_period=RETURN_FROM_I_BATTARY_CHARGE_FAULT_PERIOD;
uint16_t I_battery_charge_fault_repeat=0;
uint16_t I_battery_charge_fault_recovered_from_repeat=0;
uint16_t I_battery_cnt_for_battery_switch_control=0;
uint16_t battery_voltage_hold_for_switch_control_cnt=0;
uint16_t V_battery_cnt_for_battery_switch_control=0;
bool battery_voltage_hold_fl=0;
volatile bool battery_switch_active=0;


uint16_t VAC_line_freq_max_fault_accept_cnt=0;
uint16_t VAC_line_freq_accept_period=FREQ_LINE_ACCEPT_PERIOD;
volatile bool VAC_line_freq_max_err=0;
uint16_t return_VAC_line_freq_fault_cnt=0;
volatile float32_t VAC_line_freq_protc_max_lim=FREQ_IN_PROTC_MAX_LIM;
volatile float32_t VAC_line_freq_protc_min_lim=FREQ_IN_PROTC_MIN_LIM;

uint16_t VAC_line_freq_min_fault_accept_cnt=0;
volatile bool VAC_line_freq_min_err=0;
volatile float32_t VAC_line_freq_ret_hyst_perc=VAC_LINE_FREQ_RET_HYST_PERC;


volatile float32_t V_DC_bar_fast_limit_max=V_DC_BAR_MAX_FAST;
volatile float32_t V_DC_bar_fast_limit_min=V_DC_BAR_MIN_FAST;
volatile float32_t V_DC_bar_upper_fast_under_volt_protc_actv_thresh=(V_DC_BAR_MIN_FAST+(V_DC_BAR_NOM-V_DC_BAR_MIN_FAST)*0.8f);
volatile float32_t V_DC_bar_lower_fast_under_volt_protc_actv_thresh=(V_DC_BAR_MIN_FAST+(V_DC_BAR_NOM-V_DC_BAR_MIN_FAST)*0.8f);
uint16_t V_DC_bar_upper_fast_low_accept_cnt=0;
uint16_t V_DC_bar_upper_fast_high_accept_cnt=0;
uint16_t V_DC_bar_lower_fast_low_accept_cnt=0;
uint16_t V_DC_bar_lower_fast_high_accept_cnt=0;
uint16_t pfc_boost_start_delay_cnt=0;
uint16_t critical_fault_reset_delay_cnt=0;

volatile float32_t S_in_100_lim=S_IN_100_LIM;
volatile float32_t S_in_125_lim=S_IN_125_LIM;
volatile float32_t S_in_150_lim=S_IN_150_LIM;

volatile float32_t I_in_rms_100_lim=I_IN_RMS_100_LIM;
volatile float32_t I_in_rms_125_lim=I_IN_RMS_125_LIM;
volatile float32_t I_in_rms_150_lim=I_IN_RMS_150_LIM;
volatile bool rec_over_load_125_150_err=0;
volatile bool rec_over_load_125_150_fault_try_return=0;
volatile bool rec_over_load_125_150_fault_persists=0;
uint16_t rec_over_load_125_150_fault_repeat=0;
uint16_t rec_over_load_125_150_accept_cnt=0;
uint16_t rec_over_load_125_150_accept_period=REC_OVER_LOAD_ACCEPT_PERIOD;
uint16_t return_rec_over_load_125_150_fault_cnt=0;
uint16_t return_I_in_rms_125_150_fault_period=I_IN_RMS_125_150_RETURN_PERIOD;

volatile bool rec_over_load_100_125_err=0;
volatile bool rec_over_load_100_125_fault_try_return=0;
volatile bool rec_over_load_100_125_fault_persists=0;
uint16_t rec_over_load_100_125_fault_repeat=0;
uint32_t accept_rec_over_load_125_150_fault_return_cnt=0;
uint32_t rec_over_load_100_125_accept_cnt=0;
uint16_t I_in_rms_100_125_ovc_counting_reset_cnt=0;
uint32_t rec_over_load_100_125_accept_period=REC_OVER_LOAD_100_125_ACCEPT_PERIOD;
uint32_t return_rec_over_load_100_125_fault_cnt=0;
uint32_t accept_rec_over_load_100_125_fault_return_cnt=0;
uint32_t return_I_in_rms_100_125_fault_period=I_IN_RMS_100_125_RETURN_PERIOD;


volatile int16_t V_R_in_meas_values[REC_AC_MEAS_VECTOR_SIZE]={0};
volatile int16_t V_S_in_meas_values[REC_AC_MEAS_VECTOR_SIZE]={0};
volatile int16_t V_T_in_meas_values[REC_AC_MEAS_VECTOR_SIZE]={0};
volatile int16_t I_R_in_meas_values[REC_AC_MEAS_VECTOR_SIZE]={0};
volatile int16_t I_S_in_meas_values[REC_AC_MEAS_VECTOR_SIZE]={0};
volatile int16_t I_T_in_meas_values[REC_AC_MEAS_VECTOR_SIZE]={0};
volatile int16_t V_DC_bar_upper_meas_values[REC_DC_MEAS_VECTOR_SIZE]={0};
volatile int16_t V_DC_bar_lower_meas_values[REC_DC_MEAS_VECTOR_SIZE]={0};
volatile int16_t I_battery_upper_meas_values[REC_DC_MEAS_VECTOR_SIZE]={0};
volatile int16_t I_battery_lower_meas_values[REC_DC_MEAS_VECTOR_SIZE]={0};
uint16_t rec_data_log_last_5_cycle_active_fl=0;
volatile uint16_t rec_data_log_stop_fl=0;
uint16_t rec_data_log_last_5_cycle_cnt=0;
volatile uint16_t rec_data_log_meas_active_fl=0;
volatile uint16_t rec_data_log_eeprom_wr_active_fl=0;
volatile uint16_t rec_data_log_eeprom_wr_page=0;
volatile uint16_t rec_data_log_send_terminal_page=0;
uint16_t ups_eeprom_rec_datas_adrr=UPS_EEPROM_REC_DATAS_ADRR;

uint16_t AC_in_meas_values_index=0;
uint16_t DC_meas_values_index=0;
uint16_t input_meas_values_100us_cnt=0;

volatile uint16_t uart_intrrupt_fl_clear_cnt=0;
volatile uint16_t uart_timeout_counter=0;
volatile uint32_t uart_intrrupt_fl_clear_cnt_debug=0;


// Arıza kaydı için 32 bitlik vektör
volatile uint32_t ups_error_vector[UPS_MAX_ERRORS]={0};
uint32_t ups_eeprom_error_buffer[UPS_MAX_ERRORS]={0};
volatile uint16_t ups_error_count = 0;
volatile uint16_t ups_error_count_index = 0;
volatile uint16_t ups_err_data_full = 0;
volatile uint16_t ups_error_count_index_from_front_panel = 0;
volatile uint16_t ups_eeprom_reset_active_fl = 0;
volatile uint16_t ups_eeprom_reset_compl_fl = 0;
volatile uint16_t ups_error_vector_index = 0;
uint16_t ups_err_index_for_respose_save = 2;
uint16_t timer_cnt_for_1sec = 0;
volatile uint32_t timer_1sec_for_ups_system = 0;
volatile uint16_t ups_err_data_full_eeprom=0;
volatile uint16_t ups_error_count_index_eeprom=0;
volatile uint16_t ups_eeprom_err_adrr_debug=0;
volatile uint16_t eeprom_error_data_read_debug=0;


 //=============================================================================
 //END OF UPS_1_V1 VARIABLE

//
//-----------------------------------------------------------------------------
// Enum for build level of software and board status
//
enum REC_boardState_enum REC_boardState = PowerStageOFF;
enum REC_boardStatus_enum REC_boardStatus = boardStatus_Idle;


//****************************************//
#pragma SET_DATA_SECTION("controlVariables")

//
volatile float32_t I_R_inst_meas_cla=0;
volatile float32_t I_S_inst_meas_cla=0;
volatile float32_t I_T_inst_meas_cla=0;
volatile float32_t V_DC_bar_upper_meas_cla=0;
volatile float32_t V_DC_bar_lower_meas_cla=0;
volatile float32_t I_battery_upper_meas_cla=0;
volatile float32_t I_battery_lower_meas_cla=0;

volatile float32_t I_R_meas_offset_cla;
volatile float32_t I_S_meas_offset_cla;
volatile float32_t I_T_meas_offset_cla;
volatile float32_t I_battery_upper_meas_offset_cla;
volatile float32_t I_battery_lower_meas_offset_cla;

volatile float32_t I_R_meas_cla_coff=I_LINE_MEAS_COEFF_CLA;
volatile float32_t I_S_meas_cla_coff=I_LINE_MEAS_COEFF_CLA;
volatile float32_t I_T_meas_cla_coff=I_LINE_MEAS_COEFF_CLA;
volatile float32_t I_battery_upper_meas_cla_coff=I_BATTERY_MEAS_COEFF_CLA;
volatile float32_t I_battery_lower_meas_cla_coff=I_BATTERY_MEAS_COEFF_CLA;
volatile float32_t V_DC_bar_upper_meas_cla_coff=V_DC_BAR_MEAS_COEFF_CLA;
volatile float32_t V_DC_bar_lower_meas_cla_coff=V_DC_BAR_MEAS_COEFF_CLA;

//kontrol sistemi yapı için değişken ataması
E24000_PR_CONTROLLER REC_R_PR_control;
E24000_PR_CONTROLLER REC_S_PR_control;
E24000_PR_CONTROLLER REC_T_PR_control;

volatile float32_t Rec_PR_Control_VR_Ref_Pu=0;
volatile float32_t Rec_PR_Control_VS_Ref_Pu=0;
volatile float32_t Rec_PR_Control_VT_Ref_Pu=0;
//=============================================================================
//END OF UPS_1_V1 VARIABLE


volatile float32_t I_in_no_load_cur_lim=I_IN_NO_LOAD_CUR_LIM;
volatile float32_t I_in_no_load_cur_lim_return=I_IN_NO_LOAD_CUR_LIM_RETURN;
uint16_t accept_I_in_R_no_load_cur_cnt=0;
uint16_t accept_I_in_S_no_load_cur_cnt=0;
uint16_t accept_I_in_T_no_load_cur_cnt=0;
volatile uint16_t accept_I_in_no_load_cur_period=ACCEPT_I_IN_NO_LOAD_CUR_PERIOD;
volatile bool I_in_R_no_load_cur_fault_fl=0;
volatile bool I_in_S_no_load_cur_fault_fl=0;
volatile bool I_in_T_no_load_cur_fault_fl=0;

volatile bool pfc_boost_stable_fl=0;
uint16_t  pfc_boost_stable_accept_cnt=0;

uint16_t accept__cnt=0;


volatile bool start_control_syst_PFC_boost=0;
volatile bool Rec_phase_squence_correct=0;
volatile uint16_t Rec_phase_squence_correct_err=0;
volatile bool V_DC_bar_lower_fast_low_err=0;
volatile bool V_DC_bar_upper_fast_high_err=0;
volatile bool V_DC_bar_lower_fast_high_err=0;
volatile bool V_DC_bar_upper_fast_low_err=0;
volatile bool V_DC_bar_fast_err=0;
volatile float32_t V_DC_bar_err_hold=0;


volatile float32_t  Rec_Vdc_PM_Ref=0;
volatile float32_t  Rec_Vdc_MN_Ref=0;
volatile float32_t  Rec_Vdc_Bus_Ref_in=V_DC_BAR_NOM;
uint16_t stop_pwm_PFC_boost_deb=0;
volatile bool PFC_boost_first_ref_set=0;
//uint16_t var1=0;
//uint16_t var2=0;
//uint16_t var3=0;
//uint16_t var4=0;
//uint16_t var5=0;
//uint16_t var6=0;

//volatile float32_t  I_R_inst_val=0;
//volatile float32_t V_DC_bar_upper_meas_cla=0;
//volatile float32_t V_DC_bar_lower_meas_cla=0;



// voltaj and current sense variable

volatile uint16_t rec_vSMeas_pu=0;
volatile uint16_t rec_vTMeas_pu=0;
volatile uint16_t rec_iRMeas_pu=0;
volatile uint16_t rec_iSMeas_pu=0;
volatile uint16_t rec_iTMeas_pu=0;
volatile float32_t V_R_in_meas=0;
volatile float32_t V_S_in_meas=0;
volatile float32_t V_T_in_meas=0;
volatile float32_t I_R_in_meas=0;
volatile float32_t I_S_in_meas=0;
volatile float32_t I_T_in_meas=0;

volatile float32_t REC_vR_MeasOffset_pu;
volatile float32_t REC_vS_MeasOffset_pu;
volatile float32_t REC_vT_MeasOffset_pu;
volatile float32_t REC_iR_MeasOffset_pu;
volatile float32_t REC_iS_MeasOffset_pu;
volatile float32_t REC_iT_MeasOffset_pu;

//
// DC BUS Voltage measurement
//
volatile float32_t V_DC_bar_upper_meas=0;
volatile float32_t V_DC_bar_lower_meas=0;
volatile float32_t rec_VbusPM_Meas_pu=0;
volatile float32_t rec_VbusMN_Meas_pu=0;
volatile float32_t I_battery_upper_meas=0;
volatile float32_t I_battery_lower_meas=0;
volatile uint16_t rec_I_Aku_P_Meas_pu=0;
volatile uint16_t rec_I_Aku_N_Meas_pu=0;
volatile float32_t REC_I_Aku_P_MeasOffset_pu;
volatile float32_t REC_I_Aku_N_MeasOffset_pu;

// voltaj ve akım için donanımsal olarak ayarlanan katsayı
volatile float32_t V_R_in_meas_coeff=V_LINE_MEAS_COEFF;
volatile float32_t V_S_in_meas_coeff=V_LINE_MEAS_COEFF;
volatile float32_t V_T_in_meas_coeff=V_LINE_MEAS_COEFF;
volatile float32_t I_R_in_meas_coeff=I_LINE_MEAS_COEFF;
volatile float32_t I_S_in_meas_coeff=I_LINE_MEAS_COEFF;
volatile float32_t I_T_in_meas_coeff=I_LINE_MEAS_COEFF;
volatile float32_t I_battery_upper_meas_coeff=I_BATERY_MEAS_COEFF;
volatile float32_t I_battery_lower_meas_coeff=I_BATERY_MEAS_COEFF;
volatile float32_t V_DC_bar_upper_meas_coeff=V_DC_BAR_MEAS_COEFF;
volatile float32_t V_DC_bar_lower_meas_coeff=V_DC_BAR_MEAS_COEFF;

volatile bool rec_R_pll_all_stable_true = 0;    //cla                       // zamanlama hesaplamalari icin yazilimsal kitleme katsayileri

#pragma SET_DATA_SECTION()
//********************************************//

//
// PHASE SEQUENCE VERIFICATION Calibration
//

bool rec_R_phase_seq_V_hold_fl=0;
volatile bool rec_S_phase_seq_V_hold_fl=0;
volatile bool rec_T_phase_seq_V_hold_fl=0;
float32_t Rec_R_phase_angle_meas=0;
float32_t Rec_S_phase_angle_meas=0;
float32_t Rec_S_phase_angle_prev_meas=0;
float32_t Rec_T_phase_angle_meas=0;
float32_t Rec_T_phase_angle_prev_meas=0;


//epll block for R sine phase
// pll algoritması bir structure ile yapılandırılmıştır onun için her işlem için
// atama yapılmalı
EPLL_1PH REC_R_epll1;

float32_t rec_R_pll_stable_ratio = 0.0;      //cla
bool rec_R_neg_to_pos_cross_flg = 0;         //cla
bool rec_R_pos_to_neg_cross_flg = 0;        //cla
bool rec_R_pll_stable_flg = 0;              //cla
bool rec_R_pll_vdif_stable_flg = 0;         //cla
bool rec_R_pos_bus_cap_charge_flg = 0;
bool rec_R_neg_bus_cap_charge_flg= 0;
float32_t rec_R_pll_stable_ratio_threshold = 0.01;    //cla
uint16_t rec_R_pll_stable_count = 0;              //cla
uint16_t rec_R_pll_stable_Vdif_count = 0;   //cla
uint32_t rec_R_pll_stable_timer = 0;           //cla
uint32_t rec_R_pos_scr_bus_1s_timer =0;
uint64_t rec_R_neg_scr_bus_1s_timer=0;

 int16_t rec_R_pos_sine_hold = 0;         //cla              // Pozitifden negatife geÃ§iÅŸ anÄ±ndaki sayma deÄŸeri
 int16_t rec_R_neg_sine_hold = 0;         //cla           // Negatifden pozitife geÃ§iÅŸ anÄ±ndaki sayma deÄŸeri //cla
 uint16_t rec_R_sin_hold_time_cnt = 0;            //cla
 int8_t rec_R_pos_scr_bus_1s_timer_clamp = 0;
 int8_t rec_R_neg_scr_bus_1s_timer_clamp=0;
 float32_t rec_R_pos_sine_hold_scale = 0.95;
 float32_t rec_R_neg_sine_hold_scale = 0.95;



//// voltaj and current sense variable

volatile float32_t Vrms_line1_R=0;
volatile float32_t Vrms_line1_S=0;
volatile float32_t Vrms_line1_T=0;
volatile float32_t Irms_Line1_R=0;
volatile float32_t Irms_Line1_S=0;
volatile float32_t Irms_Line1_T=0;

volatile float32_t VAC_line_R_freq=0;
volatile float32_t VAC_line_S_freq=0;
volatile float32_t VAC_line_T_freq=0;
volatile float32_t Sin_R_meas=0;// rectifier sisteminde güç analizine gerek yoktur
volatile float32_t Sin_S_meas=0;
volatile float32_t Sin_T_meas=0;
volatile float32_t Pin_R_meas=0;
volatile float32_t Pin_S_meas=0;
volatile float32_t Pin_T_meas=0;
volatile float32_t pf_R_meas=0;
volatile float32_t pf_S_meas=0;
volatile float32_t pf_T_meas=0;

//// ups system voltaj and current sense max variable
uint32_t eeprom_write_cnt_for_ups_max=0;
volatile float32_t Vrms_line1_R_max=0;
volatile float32_t Vrms_line1_S_max=0;
volatile float32_t Vrms_line1_T_max=0;
volatile float32_t Irms_Line1_R_max=0;
volatile float32_t Irms_Line1_S_max=0;
volatile float32_t Irms_Line1_T_max=0;
//instantaneous values
volatile float32_t V_DC_bar_lower_meas_max=0;
volatile float32_t V_DC_bar_upper_meas_max=0;
volatile float32_t I_battery_lower_meas_max=0;
volatile float32_t I_battery_upper_meas_max=0;
volatile float32_t V_R_in_meas_max=0;
volatile float32_t V_S_in_meas_max=0;
volatile float32_t V_T_in_meas_max=0;
volatile float32_t I_R_in_meas_max=0;
volatile float32_t I_S_in_meas_max=0;
volatile float32_t I_T_in_meas_max=0;

volatile float32_t V_DC_bar_lower_meas_min=1000.0;
volatile float32_t V_DC_bar_upper_meas_min=1000.0;
volatile float32_t I_battery_upper_meas_min=1000.0;
volatile float32_t I_battery_lower_meas_min=1000.0;
volatile float32_t V_R_in_meas_min=0;
volatile float32_t V_S_in_meas_min=0;
volatile float32_t V_T_in_meas_min=0;
volatile float32_t I_R_in_meas_min=0;
volatile float32_t I_S_in_meas_min=0;
volatile float32_t I_T_in_meas_min=0;
uint16_t input_meas_max_min_values_cnt=0;

volatile float32_t Vrms_inv1_R_max_cpu2=0;
volatile float32_t Vrms_inv1_S_max_cpu2=0;
volatile float32_t Vrms_inv1_T_max_cpu2=0;
volatile float32_t Irms_out1_R_max_cpu2=0;
volatile float32_t Irms_out1_S_max_cpu2=0;
volatile float32_t Irms_out1_T_max_cpu2=0;
volatile float32_t Vrms_load1_R_max_cpu2=0;
volatile float32_t Vrms_load1_S_max_cpu2=0;
volatile float32_t Vrms_load1_T_max_cpu2=0;
volatile float32_t Irms_load1_R_max_cpu2=0;
volatile float32_t Irms_load1_S_max_cpu2=0;
volatile float32_t Irms_load1_T_max_cpu2=0;
volatile float32_t Vrms_bypass1_R_max_cpu2=0;
volatile float32_t Vrms_bypass1_S_max_cpu2=0;
volatile float32_t Vrms_bypass1_T_max_cpu2=0;
volatile float32_t V_battery_upper_max_cpu2=0;
volatile float32_t V_R_inv_meas_max_cpu2=0;
volatile float32_t V_S_inv_meas_max_cpu2=0;
volatile float32_t V_T_inv_meas_max_cpu2=0;
volatile float32_t I_R_inv_meas_max_cpu2=0;
volatile float32_t I_S_inv_meas_max_cpu2=0;
volatile float32_t I_T_inv_meas_max_cpu2=0;
volatile float32_t I_R_load_meas_max_cpu2=0;
volatile float32_t I_S_load_meas_max_cpu2=0;
volatile float32_t I_T_load_meas_max_cpu2=0;

volatile uint16_t thyr_bypass_side_active_fl_from_cpu2=0;
volatile uint16_t inv_control_system_active_fl_from_cpu2=0;

volatile uint16_t rec_state_from_front_panel=0;
volatile uint16_t inv_state_from_front_panel=0;
volatile uint16_t byp_state_from_front_panel=0;
volatile uint16_t ups_write_state_active_fl=0;
volatile uint16_t panel_to_main_board_write_active=0;
volatile uint16_t rec_system_state_control_fl_from_servise=0;

float32_t Vrms_line1_R_max_eeprom=0;
float32_t Vrms_line1_S_max_eeprom=0;
float32_t Vrms_line1_T_max_eeprom=0;
float32_t Irms_Line1_R_max_eeprom=0;
float32_t Irms_Line1_S_max_eeprom=0;
float32_t Irms_Line1_T_max_eeprom=0;
float32_t V_DC_bar_lower_meas_max_eeprom=0;
float32_t V_DC_bar_upper_meas_max_eeprom=0;
float32_t I_battery_lower_meas_max_eeprom=0;
float32_t I_battery_upper_meas_max_eeprom=0;
volatile bool eemprom_read_for_ups_max_val_fl=0;

volatile uint16_t rec_R_pos_scr_low_timer=0;
volatile uint16_t rec_R_neg_scr_low_timer=0;
volatile bool rec_R_pos_scr_fire_flag=0;
volatile bool rec_R_neg_scr_fire_flg=0;
volatile uint16_t thy_pos_voltage_down_timer=0;
volatile uint16_t thy_neg_voltage_down_timer=0;

//=============================================================================
// // moving average parameters
//=============================================================================

volatile bool rec_meas_mov_avg_on_flag=0;         //cla
volatile bool DC_bar_mov_avg_on_flag=0;         //cla

//parameters for DC_BUS_PM
enum REC_average_state_enum rec_vbus_avrg_active=AverageStageOFF;

volatile float32_t V_DC_bar_upper_avg_sum=0;
volatile float32_t V_DC_bar_lower_avg_sum=0;
volatile float32_t Rec_I_Aku_P_sum_avg_sum=0;
volatile float32_t Rec_I_Aku_N_sum_avg_sum=0;

volatile float32_t Rec_VbusPM_sum=0;
volatile float32_t Rec_VbusPM_sum_sample[V_DC_BAR_AVG_20MS_SAMPLE_COUNT]={0};
volatile uint16_t rec_vbus_sum_index=0;
volatile float32_t V_DC_bar_upper_avg_20ms=0;
//parameters for DC_BUS_MN
volatile float32_t Rec_VbusMN_sum=0.0;
volatile float32_t Rec_VbusMN_sum_sample[V_DC_BAR_AVG_20MS_SAMPLE_COUNT]={0};
volatile float32_t V_DC_bar_lower_avg_20ms=0;

volatile float32_t Rec_I_Aku_P_sum=0;
volatile float32_t Rec_I_Aku_P_sum_sample[V_DC_BAR_AVG_20MS_SAMPLE_COUNT]={0.0};
volatile float32_t Rec_I_Aku_P_sum_avg=0;

volatile float32_t Rec_I_Aku_N_sum=0;
volatile float32_t Rec_I_Aku_N_sum_sample[V_DC_BAR_AVG_20MS_SAMPLE_COUNT]={0};
volatile float32_t Rec_I_Aku_N_sum_avg=0;

//parameters for rec R phase
volatile float32_t Rec_VrmsR_sum=0;
volatile float32_t Rec_IrmsR_sum=0;
volatile float32_t Rec_VrmsR_sum_sample[Rec_VrmsRST_sum_count]={0};
volatile float32_t Rec_IrmsR_sum_sample[Rec_IrmsRST_sum_count]={0};
volatile uint16_t Rec_VrmsRST_sum_index=0;
volatile uint16_t Rec_IrmsRST_sum_index=0;
volatile float32_t Vrms_Line10_R=0;
volatile float32_t Irms_Line10_R=0;

//parameters for rec S phase
volatile float32_t Rec_VrmsS_sum=0;
volatile float32_t Rec_IrmsS_sum=0;
volatile float32_t Rec_VrmsS_sum_sample[Rec_VrmsRST_sum_count]={0};
volatile float32_t Rec_IrmsS_sum_sample[Rec_IrmsRST_sum_count]={0};
volatile float32_t Vrms_Line10_S=0;
volatile float32_t Irms_Line10_S=0;

//parameters for rec T phase
volatile float32_t Rec_VrmsT_sum=0;
volatile float32_t Rec_IrmsT_sum=0;
volatile float32_t Rec_VrmsT_sum_sample[Rec_VrmsRST_sum_count]={0};
volatile float32_t Rec_IrmsT_sum_sample[Rec_IrmsRST_sum_count]={0};
volatile float32_t Vrms_Line10_T=0;
volatile float32_t Irms_Line10_T=0;

uint32_t led_control_cpu1=0;

 //
 // Sine analyzer block for RMS Volt, Curr and Power measurements
 //
 POWER_MEAS_SINE_ANALYZER rec_R_sine_mains1;
 POWER_MEAS_SINE_ANALYZER rec_S_sine_mains1;
 POWER_MEAS_SINE_ANALYZER rec_T_sine_mains1;

 volatile bool rec_R_power_meas_flg=0;      //cla
 volatile bool rec_S_power_meas_flg=0;
 volatile bool rec_T_power_meas_flg=0;


 // LCD display ile oluşturulacak olan haberleşme değişkenleri
 //=============================================================================
 uint8_t comm_key_value=0;
 uint8_t ModbusTxBuffer[MODBUSMASTER_REGISTER]={0};
 uint8_t ModbusTxBuffer_debug[1]={0};
 uint8_t ModbusTxBuffer_cnt_debug=11;
 volatile bool comm_main_board_to_monitor_fl=0;
 volatile uint8_t modbus_fnc_code=4;
 volatile uint16_t modbus_com_data_count=COM_DATA_COUNT_UPS_SYS;

 volatile bool ups_calibrate_active_fl=0;
 uint16_t ResponseDataIndx=0;
 uint16_t TransDataIndx=0;
 uint16_t ResponseBuffer[COM_RESPONSE_DATA_COUNT]={0};
 volatile uint16_t err_response_buffer[ERR_16BIT_DATA_COUNT]={0};
 uint16_t TI_ReciveDataIndx=0;
 volatile uint16_t TI_Recive_Buffer[MODBUS_RECIVE_COM_DATA_COUNT];
 uint8_t   ModbusRxBuffer[MODBUS_RECIVE_MASTER_REGISTER];
 uint16_t  Modbus_receiveIndex=0;
 uint16_t  modbus_status=0;
 volatile uint16_t  ModbusEndOfDataIndx=0;
 uint16_t  sci_interrupt_cnt=0;
 uint16_t  TI_receiveIndex=0;
 uint8_t  receivedByte=0;
 uint16_t  modbus_recive_end_of_data_Indx=0;
volatile uint8_t   recieve_SlaveAddress;
volatile uint8_t   recieve_FunctionCode;
uint16_t SCI_Rx_i=0;

//
uint16_t ST_receiveIndex=0;
uint16_t ST_endOfDataIndx=0;
uint8_t ST_receivecrcLowByte=0;
uint8_t receivecrcHighByte=0;

volatile uint16_t rec_uart_comm_active_for_real_term_fl=0;
volatile bool inv_uart_comm_active_for_real_term_fl_from_cpu2=0;
volatile uint16_t inv_uart_comm_active_for_real_term_fl=0;
volatile uint16_t inv_uart_comm_active=0;
volatile uint16_t rec_uart_comm_passive_fl=0;
volatile uint16_t ups_rec_data_log_send_terminal_active=0;
volatile bool uart_comm_active_for_real_term_fl2=0;
uint16_t inv_uart_comm_passive_cnt=0;
char uartBuff[UART_BUFFER_SIZE];
char dizi[30];
int16_t length_debug=0;
uint32_t user_baudrate;
uint32_t user_config;
volatile uint16_t ups_uart_core_select_fl=0;
volatile uint16_t uart_core=1;

 //=============================================================================
 //

//EEPROM VARİABLE
uint16_t UPS_eeprom_buffer[EEPROM_NUM_OF_DATA] = {0};
//uint32_t UPS_eeprom_buffer_debug[EEPROM_NUM_OF_DATA_OF_UPS_VALUES] = {0};
//uint32_t eeprom_debug_val=100244;
bool eeprom_write_debug=0;
bool eeprom_read_debug=0;
//uint32_t eeprom_read_debug_val_24=0;
//uint32_t eeprom_read_debug_val_25=0;
//uint32_t eeprom_read_debug_val_26=0;
//uint32_t eeprom_read_debug_val_27=0;

 // globalVariablesInit()
 //
 void Rectifier_globalVariablesInit(void){

     PFC_boost_control_reset(&REC_R_PR_control);
     PFC_boost_control_config(&REC_R_PR_control,
                              Rec_Vdc_Bus_Ref,
                              Rec_Vdc_bus_scale_const,
                              Rec_DC_bus_Kp,
                              Rec_DC_bus_Ki,
                              Rec_DC_bus_pi_b0,
                              Rec_DC_bus_pi_a1,
                              Rec_PWM_TBPRD,
                              Rec_PR_const,
                              Rec_PR_Offset,
                              Rec_PR_Kp,
                              Rec_R_1_Ki,
                              Rec_R_1_b0,
                              Rec_R_1_b1,
                              Rec_R_1_b2,
                              Rec_R_1_a1,
                              Rec_R_1_a2,
                              Rec_R_3_Ki,
                              Rec_R_3_b0,
                              Rec_R_3_b1,
                              Rec_R_3_b2,
                              Rec_R_3_a1,
                              Rec_R_3_a2,
                              Rec_R_5_Ki,
                              Rec_R_5_b0,
                              Rec_R_5_b1,
                              Rec_R_5_b2,
                              Rec_R_5_a1,
                              Rec_R_5_a2,
                              Rec_R_7_Ki,
                              Rec_R_7_b0,
                              Rec_R_7_b1,
                              Rec_R_7_b2,
                              Rec_R_7_a1,
                              Rec_R_7_a2,
                              REC_I_AKU_REF,
                              REC_I_AKU_KP,
                              REC_I_AKU_KI);
//
     PFC_boost_control_reset(&REC_S_PR_control);
     PFC_boost_control_config(&REC_S_PR_control,
                              Rec_Vdc_Bus_Ref,
                              Rec_Vdc_bus_scale_const,
                              Rec_DC_bus_Kp,
                              Rec_DC_bus_Ki,
                              Rec_DC_bus_pi_b0,
                              Rec_DC_bus_pi_a1,
                              Rec_PWM_TBPRD,
                              Rec_PR_const,
                              Rec_PR_Offset,
                              Rec_PR_Kp,
                              Rec_R_1_Ki,
                              Rec_R_1_b0,
                              Rec_R_1_b1,
                              Rec_R_1_b2,
                              Rec_R_1_a1,
                              Rec_R_1_a2,
                              Rec_R_3_Ki,
                              Rec_R_3_b0,
                              Rec_R_3_b1,
                              Rec_R_3_b2,
                              Rec_R_3_a1,
                              Rec_R_3_a2,
                              Rec_R_5_Ki,
                              Rec_R_5_b0,
                              Rec_R_5_b1,
                              Rec_R_5_b2,
                              Rec_R_5_a1,
                              Rec_R_5_a2,
                              Rec_R_7_Ki,
                              Rec_R_7_b0,
                              Rec_R_7_b1,
                              Rec_R_7_b2,
                              Rec_R_7_a1,
                              Rec_R_7_a2,
                              REC_I_AKU_REF,
                              REC_I_AKU_KP,
                              REC_I_AKU_KI);
//
     PFC_boost_control_reset(&REC_T_PR_control);
     PFC_boost_control_config(&REC_T_PR_control,
                              Rec_Vdc_Bus_Ref,
                              Rec_Vdc_bus_scale_const,
                              Rec_DC_bus_Kp,
                              Rec_DC_bus_Ki,
                              Rec_DC_bus_pi_b0,
                              Rec_DC_bus_pi_a1,
                              Rec_PWM_TBPRD,
                              Rec_PR_const,
                              Rec_PR_Offset,
                              Rec_PR_Kp,
                              Rec_R_1_Ki,
                              Rec_R_1_b0,
                              Rec_R_1_b1,
                              Rec_R_1_b2,
                              Rec_R_1_a1,
                              Rec_R_1_a2,
                              Rec_R_3_Ki,
                              Rec_R_3_b0,
                              Rec_R_3_b1,
                              Rec_R_3_b2,
                              Rec_R_3_a1,
                              Rec_R_3_a2,
                              Rec_R_5_Ki,
                              Rec_R_5_b0,
                              Rec_R_5_b1,
                              Rec_R_5_b2,
                              Rec_R_5_a1,
                              Rec_R_5_a2,
                              Rec_R_7_Ki,
                              Rec_R_7_b0,
                              Rec_R_7_b1,
                              Rec_R_7_b2,
                              Rec_R_7_a1,
                              Rec_R_7_a2,
                              REC_I_AKU_REF,
                              REC_I_AKU_KP,
                              REC_I_AKU_KI);

     //epll algorithm initialization for R sine phase
     EPLL_1PH_reset(&REC_R_epll1);
     EPLL_1PH_config(&REC_R_epll1,
                     EPLL_INT_B0,
                     EPLL_INT_A1,
                     EPLL_INT_KI,
                     EPLL_FREQ_INT_KI,
                     EPLL_FREQ_KP);

     //sine analyzer initialization for R sine phase
      POWER_MEAS_SINE_ANALYZER_reset(&rec_R_sine_mains1);
      POWER_MEAS_SINE_ANALYZER_config(&rec_R_sine_mains1,
                                      VOLTAGE_ISR_20KHZ_FREQUENCY_HZ,
                                      (float32_t)0.08,
                                      (float32_t)REC_GRID_MAX_FREQ_HZ,
                                      (float32_t)REC_GRID_MIN_FREQ_HZ);
     //sine analyzer initialization for S sine phase
      POWER_MEAS_SINE_ANALYZER_reset(&rec_S_sine_mains1);
      POWER_MEAS_SINE_ANALYZER_config(&rec_S_sine_mains1,
                                      VOLTAGE_ISR_20KHZ_FREQUENCY_HZ,
                                      (float32_t)0.08,
                                      (float32_t)REC_GRID_MAX_FREQ_HZ,
                                      (float32_t)REC_GRID_MIN_FREQ_HZ);
     //sine analyzer initialization for T sine phase
      POWER_MEAS_SINE_ANALYZER_reset(&rec_T_sine_mains1);
      POWER_MEAS_SINE_ANALYZER_config(&rec_T_sine_mains1,
                                      VOLTAGE_ISR_20KHZ_FREQUENCY_HZ,
                                      (float32_t)0.08,
                                      (float32_t)REC_GRID_MAX_FREQ_HZ,
                                      (float32_t)REC_GRID_MIN_FREQ_HZ);
 }




