/* ==============================================================================
System Name:   POWER Elektronik 3 Faz 160kVA Ups Rectifier Kodu
MCU                  =   TMS320F28379D , 200MHz,
Rectifier            =   160KVA 3P ~= 350uH sok , 100uF*2 metal govdeli giris kondansatoru
===========================================================================  */

#include "board.h"
#include "device.h"
#include "driverlib.h"
#include "ipc.h"
#include "math.h"
#include "rec_hal.h"
#include "my_functions.h"
#include "variable.h"
#include "sw_prioritized_isr_levels.h"
#include "I2C_EEPROM.h"
#include "stdio.h"

void (*Alpha_State_Ptr)(void);
void A0(void);
__interrupt void CPU1_loop_fnc(void);
__interrupt void INSTRUMENTATION_ISR();
__interrupt void UPS_REC_COM_ISR(void);
#pragma CODE_SECTION(CPU1_loop_fnc,"isrcodefuncs");

void main(void) {
  REC_HAL_setupDevice();
  Board_init();
  Rec_init();
  DEVICE_DELAY_US(1000);
  IPC_clearFlagLtoR(IPC_CPU1_L_CPU2_R, IPC_FLAG_ALL);
  IPC_sync(IPC_CPU1_L_CPU2_R, IPC_FLAG31);
  Alpha_State_Ptr = &A0;
  Rectifier_globalVariablesInit();
  REC_calibrateOffset();
  EINT;
  ERTM;
  writeDataCPU1();
  SCI_getConfig(UPS_REC_UART0_ISO_BASE,DEVICE_LSPCLK_FREQ,&user_baudrate,&user_config);
  for(;;){
      (*Alpha_State_Ptr)();
  }
}//END MAIN CODE


//=============================================================================
//  STATE-MACHINE SEQUENCING AND SYNCRONIZATION FOR SLOW BACKGROUND TASKS
//=============================================================================
//--------------------------------- FRAME WORK --------------------------------
void A0(void){  // while loop function

    //    // eeproma kalibrasyon verilerinin okunması
    //    // işlemci kapanır ve tekrar açılırsa değerler eeprom bellekten alınmalı
    //    //***************************************************
        if(eemprom_read_startup_fl == 0){
            eeprom_status1=I2C_MasterReceiver_Word(SLAVE_ADRESS_EEPROM,V_R_IN_MEAS_COEFF_ADR,26,UPS_eeprom_buffer,I2C_EEPROM_BASE);
            DEVICE_DELAY_US(I2C_EEPROM_READ_DELAY);
            V_R_in_meas_coeff=(float32_t)((float32_t)UPS_eeprom_buffer[0] * 0.1f);
            V_S_in_meas_coeff=(float32_t)((float32_t)UPS_eeprom_buffer[1] * 0.1f);
            V_T_in_meas_coeff=(float32_t)((float32_t)UPS_eeprom_buffer[2] * 0.1f);
            I_R_in_meas_coeff=(float32_t)((float32_t)UPS_eeprom_buffer[3] * 0.1f);
            I_S_in_meas_coeff=(float32_t)((float32_t)UPS_eeprom_buffer[4] * 0.1f);
            I_T_in_meas_coeff=(float32_t)((float32_t)UPS_eeprom_buffer[5] * 0.1f);

            V_R_out_meas_coeff_eeprom=(float32_t)((float32_t)UPS_eeprom_buffer[6] * 0.1f);
            V_S_out_meas_coeff_eeprom=(float32_t)((float32_t)UPS_eeprom_buffer[7] * 0.1f);
            V_T_out_meas_coeff_eeprom=(float32_t)((float32_t)UPS_eeprom_buffer[8] * 0.1f);
            I_R_out_meas_coeff_eeprom=(float32_t)((float32_t)UPS_eeprom_buffer[9] * 0.1f);
            I_S_out_meas_coeff_eeprom=(float32_t)((float32_t)UPS_eeprom_buffer[10] * 0.1f);
            I_T_out_meas_coeff_eeprom=(float32_t)((float32_t)UPS_eeprom_buffer[11] * 0.1f);

            V_Battery_upper_coeff_eeprom=(float32_t)((float32_t)UPS_eeprom_buffer[12] * 0.1f);
            V_DC_bar_upper_meas_coeff=(float32_t)((float32_t)UPS_eeprom_buffer[13] * 0.1f);
            V_DC_bar_lower_meas_coeff=(float32_t)((float32_t)UPS_eeprom_buffer[14] * 0.1f);
            I_battery_upper_meas_coeff=(float32_t)((float32_t)UPS_eeprom_buffer[15] * 0.1f);
            I_battery_lower_meas_coeff=(float32_t)((float32_t)UPS_eeprom_buffer[16] * 0.1f);

            V_R_bypass_meas_coeff_eeprom=(float32_t)((float32_t)UPS_eeprom_buffer[17] * 0.1f);
            V_S_bypass_meas_coeff_eeprom=(float32_t)((float32_t)UPS_eeprom_buffer[18] * 0.1f);
            V_T_bypass_meas_coeff_eeprom=(float32_t)((float32_t)UPS_eeprom_buffer[19] * 0.1f);

            V_R_load_meas_coeff_eeprom=(float32_t)((float32_t)UPS_eeprom_buffer[20] * 0.1f);
            V_S_load_meas_coeff_eeprom=(float32_t)((float32_t)UPS_eeprom_buffer[21] * 0.1f);
            V_T_load_meas_coeff_eeprom=(float32_t)((float32_t)UPS_eeprom_buffer[22] * 0.1f);
            I_R_load_meas_coeff_eeprom=(float32_t)((float32_t)UPS_eeprom_buffer[23] * 0.1f);
            I_S_load_meas_coeff_eeprom=(float32_t)((float32_t)UPS_eeprom_buffer[24] * 0.1f);
            I_T_load_meas_coeff_eeprom=(float32_t)((float32_t)UPS_eeprom_buffer[25] * 0.1f);


            //rec kontrol sistemi okuma katsayıları
            I_R_meas_cla_coff=I_R_in_meas_coeff;
            I_S_meas_cla_coff=I_S_in_meas_coeff;
            I_T_meas_cla_coff=I_T_in_meas_coeff;
            V_DC_bar_upper_meas_cla_coff=V_DC_bar_upper_meas_coeff;
            V_DC_bar_lower_meas_cla_coff=V_DC_bar_lower_meas_coeff;
            I_battery_upper_meas_cla_coff=I_battery_upper_meas_coeff;
            I_battery_lower_meas_cla_coff=I_battery_lower_meas_coeff;

            eeprom_status1=I2C_MasterReceiver_Word(SLAVE_ADRESS_EEPROM,UPS_EEPROM_ERR_DATA_COUNT_ADRR,3,UPS_eeprom_buffer,I2C_EEPROM_BASE);
            DEVICE_DELAY_US(I2C_EEPROM_READ_DELAY);
            ups_err_data_full=UPS_eeprom_buffer[0];
            ups_error_count_index=UPS_eeprom_buffer[1];
            ups_eeprom_err_adrr=UPS_eeprom_buffer[2];
            ups_err_data_full=(ups_err_data_full > UPS_MAX_ERRORS) ? UPS_MAX_ERRORS : ups_err_data_full;
            ups_error_count=ups_err_data_full;
            if(ups_err_data_full > 0){
                eeprom_status1=I2C_MasterReceiver_DWord(SLAVE_ADRESS_EEPROM,UPS_EEPROM_ERR_DATA_ADRR,ups_err_data_full,ups_eeprom_error_buffer,I2C_EEPROM_BASE);
                DEVICE_DELAY_US(I2C_EEPROM_READ_DELAY);
                for (i = 0 ; i < ups_err_data_full; i ++){
                    ups_error_vector[i]=ups_eeprom_error_buffer[(ups_err_data_full - 1) - i];
                }
            }
            I2C_eeprom_read_active_fl=1;// bu değişken cpu2 için eepromdan okunan verileri güncellemek için kullanılıyor
            eemprom_read_startup_fl=1;
        }

     //** Eğer ön panelden yazma aktif ise yapılması gerekenler
     //** 1. ön panelden gelen verilerin kaydedilmesi ve güncellenmesi
     //** 2. ön panelden rectifier durumunu (aç/kapat) kontrol edilmesi
    if(panel_to_main_board_write_active == 1){
        uint16_t j;
        for(j = 4; j < ModbusEndOfDataIndx ; j+=2){
             TI_Recive_Buffer[TI_ReciveDataIndx] = ((uint16_t) (ModbusRxBuffer[j] << 8)) | ModbusRxBuffer[j+1]; //8 bitlik veriler 16 bite dönüştürülüyor
             TI_ReciveDataIndx++;
           }
         TI_ReciveDataIndx=0;
         SaveResponseCalibrationValue ();

        //**************
        if(rec_state_from_front_panel == 0 && thyr_input_side_active_fl == 1 && rec_system_state_control_fl_from_servise == 0){
            stop_pwm_PFC_boost();
            start_control_syst_PFC_boost=0;
            turn_off_input_thyr();
            thyr_input_side_turn_off_complete_delay=1;
            rec_system_state_control_fl_from_servise=1;
        }
        else if(rec_state_from_front_panel == 1 && thyr_input_side_active_fl == 0 && cap_charge_request == 0){
            if (V_line_range_is_OK==1) {
                cap_charge_request=1;
             }
            rec_system_state_control_fl_from_servise=0;
        }
     }

      //***************************************************
      // UPS rec data loglarının EEPROMA YAZILMASI
    if(rec_data_log_eeprom_wr_active_fl == 1 ){
        rec_data_log_eeprom_wr_page++;
      if(rec_data_log_eeprom_wr_page == 1){
        //********* VR_in ***************//
        ups_eeprom_rec_datas_adrr=UPS_EEPROM_REC_DATAS_ADRR;
        for(i=0; i < REC_AC_MEAS_VECTOR_SIZE ; i++){
            // int16_t verisini uint16_t'ye dönüştürme (eğer değer negatifse, uygun bir dönüşüm yapılmalı)
            UPS_eeprom_buffer[j] = (uint16_t)( V_R_in_meas_values[i] + 32768); // Negatif değerleri pozitif hale getiriyoruz
            j++;
            if(i < 1984){
                if(j==64){
                    eeprom_status=I2C_MasterTransmitter_Word(SLAVE_ADRESS_EEPROM,ups_eeprom_rec_datas_adrr,64,UPS_eeprom_buffer,I2C_EEPROM_BASE);
                    DEVICE_DELAY_US(6000);//6ms
                    j=0;
                    ups_eeprom_rec_datas_adrr=(uint16_t)(ups_eeprom_rec_datas_adrr + 128);
                }
            }else{
                if(j==16){
                    eeprom_status=I2C_MasterTransmitter_Word(SLAVE_ADRESS_EEPROM,ups_eeprom_rec_datas_adrr,16,UPS_eeprom_buffer,I2C_EEPROM_BASE);
                    DEVICE_DELAY_US(6000);//6ms
                    j=0;
                    ups_eeprom_rec_datas_adrr=(uint16_t)(ups_eeprom_rec_datas_adrr + 128);
                }
            }
        }
      }
      else if(rec_data_log_eeprom_wr_page == 2){
        //********* VS_in *************//
        ups_eeprom_rec_datas_adrr=UPS_EEPROM_REC_VS_DATAS_ADRR;
        for(i=0; i < REC_AC_MEAS_VECTOR_SIZE ; i++){
            UPS_eeprom_buffer[j] = (uint16_t)( V_S_in_meas_values[i] + 32768);
            j++;
            if(i < 1984){
                if(j==64){
                    eeprom_status=I2C_MasterTransmitter_Word(SLAVE_ADRESS_EEPROM,ups_eeprom_rec_datas_adrr,64,UPS_eeprom_buffer,I2C_EEPROM_BASE);
                    DEVICE_DELAY_US(6000);//6ms
                    j=0;
                    ups_eeprom_rec_datas_adrr=(uint16_t)(ups_eeprom_rec_datas_adrr + 128);
                }
            }else{
                if(j==16){
                    eeprom_status=I2C_MasterTransmitter_Word(SLAVE_ADRESS_EEPROM,ups_eeprom_rec_datas_adrr,16,UPS_eeprom_buffer,I2C_EEPROM_BASE);
                    DEVICE_DELAY_US(6000);//6ms
                    j=0;
                    ups_eeprom_rec_datas_adrr=(uint16_t)(ups_eeprom_rec_datas_adrr + 128);
                }
            }
        }
      }
      else if(rec_data_log_eeprom_wr_page == 3){
        //********* VT_in *************//
        ups_eeprom_rec_datas_adrr=UPS_EEPROM_REC_VT_DATAS_ADRR;
        for(i=0; i < REC_AC_MEAS_VECTOR_SIZE ; i++){
            UPS_eeprom_buffer[j] = (uint16_t)( V_T_in_meas_values[i] + 32768);
            j++;
            if(i < 1984){
                if(j==64){
                    eeprom_status=I2C_MasterTransmitter_Word(SLAVE_ADRESS_EEPROM,ups_eeprom_rec_datas_adrr,64,UPS_eeprom_buffer,I2C_EEPROM_BASE);
                    DEVICE_DELAY_US(6000);//6ms
                    j=0;
                    ups_eeprom_rec_datas_adrr=(uint16_t)(ups_eeprom_rec_datas_adrr + 128);
                }
            }else{
                if(j==16){
                    eeprom_status=I2C_MasterTransmitter_Word(SLAVE_ADRESS_EEPROM,ups_eeprom_rec_datas_adrr,16,UPS_eeprom_buffer,I2C_EEPROM_BASE);
                    DEVICE_DELAY_US(6000);//6ms
                    j=0;
                    ups_eeprom_rec_datas_adrr=(uint16_t)(ups_eeprom_rec_datas_adrr + 128);
                }
            }
        }
      }
      else if(rec_data_log_eeprom_wr_page == 4){
        //********* IR_in *************//
        ups_eeprom_rec_datas_adrr=UPS_EEPROM_REC_IR_DATAS_ADRR;
        for(i=0; i < REC_AC_MEAS_VECTOR_SIZE ; i++){
            UPS_eeprom_buffer[j] = (uint16_t)( I_R_in_meas_values[i] + 32768);
            j++;
            if(i < 1984){
                if(j==64){
                    eeprom_status=I2C_MasterTransmitter_Word(SLAVE_ADRESS_EEPROM,ups_eeprom_rec_datas_adrr,64,UPS_eeprom_buffer,I2C_EEPROM_BASE);
                    DEVICE_DELAY_US(6000);//6ms
                    j=0;
                    ups_eeprom_rec_datas_adrr=(uint16_t)(ups_eeprom_rec_datas_adrr + 128);
                }
            }else{
                if(j==16){
                    eeprom_status=I2C_MasterTransmitter_Word(SLAVE_ADRESS_EEPROM,ups_eeprom_rec_datas_adrr,16,UPS_eeprom_buffer,I2C_EEPROM_BASE);
                    DEVICE_DELAY_US(6000);//6ms
                    j=0;
                    ups_eeprom_rec_datas_adrr=(uint16_t)(ups_eeprom_rec_datas_adrr + 128);
                }
            }
        }
      }
      else if(rec_data_log_eeprom_wr_page == 5){
        //********* IS_in *************//
        ups_eeprom_rec_datas_adrr=UPS_EEPROM_REC_IS_DATAS_ADRR;
        for(i=0; i < REC_AC_MEAS_VECTOR_SIZE ; i++){
            UPS_eeprom_buffer[j] = (uint16_t)( I_S_in_meas_values[i] + 32768);
            j++;
            if(i < 1984){
                if(j==64){
                    eeprom_status=I2C_MasterTransmitter_Word(SLAVE_ADRESS_EEPROM,ups_eeprom_rec_datas_adrr,64,UPS_eeprom_buffer,I2C_EEPROM_BASE);
                    DEVICE_DELAY_US(6000);//6ms
                    j=0;
                    ups_eeprom_rec_datas_adrr=(uint16_t)(ups_eeprom_rec_datas_adrr + 128);
                }
            }else{
                if(j==16){
                    eeprom_status=I2C_MasterTransmitter_Word(SLAVE_ADRESS_EEPROM,ups_eeprom_rec_datas_adrr,16,UPS_eeprom_buffer,I2C_EEPROM_BASE);
                    DEVICE_DELAY_US(6000);//6ms
                    j=0;
                    ups_eeprom_rec_datas_adrr=(uint16_t)(ups_eeprom_rec_datas_adrr + 128);
                }
            }
        }
      }
      else if(rec_data_log_eeprom_wr_page == 6){
        //********* IT_in *************//
        ups_eeprom_rec_datas_adrr=UPS_EEPROM_REC_IT_DATAS_ADRR;
        for(i=0; i < REC_AC_MEAS_VECTOR_SIZE ; i++){
            UPS_eeprom_buffer[j] = (uint16_t)( I_T_in_meas_values[i] + 32768);
            j++;
            if(i < 1984){
                if(j==64){
                    eeprom_status=I2C_MasterTransmitter_Word(SLAVE_ADRESS_EEPROM,ups_eeprom_rec_datas_adrr,64,UPS_eeprom_buffer,I2C_EEPROM_BASE);
                    DEVICE_DELAY_US(6000);//6ms
                    j=0;
                    ups_eeprom_rec_datas_adrr=(uint16_t)(ups_eeprom_rec_datas_adrr + 128);
                }
            }else{
                if(j==16){
                    eeprom_status=I2C_MasterTransmitter_Word(SLAVE_ADRESS_EEPROM,ups_eeprom_rec_datas_adrr,16,UPS_eeprom_buffer,I2C_EEPROM_BASE);
                    DEVICE_DELAY_US(6000);//6ms
                    j=0;
                    ups_eeprom_rec_datas_adrr=(uint16_t)(ups_eeprom_rec_datas_adrr + 128);
                }
            }
        }
      }
      else if(rec_data_log_eeprom_wr_page == 7){
        //********* V_DC_bar_upper_meas_values *************//
      ups_eeprom_rec_datas_adrr=UPS_EEPROM_VDC_BAR_UP_DATAS_ADRR;
        for(i=0; i < REC_AC_MEAS_VECTOR_SIZE ; i++){
            UPS_eeprom_buffer[j] = (uint16_t)( V_DC_bar_upper_meas_values[i] + 32768);
            j++;
            if(i < 1984){
                if(j==64){
                    eeprom_status=I2C_MasterTransmitter_Word(SLAVE_ADRESS_EEPROM,ups_eeprom_rec_datas_adrr,64,UPS_eeprom_buffer,I2C_EEPROM_BASE);
                    DEVICE_DELAY_US(6000);//6ms
                    j=0;
                    ups_eeprom_rec_datas_adrr=(uint16_t)(ups_eeprom_rec_datas_adrr + 128);
                }
            }else{
                if(j==16){
                    eeprom_status=I2C_MasterTransmitter_Word(SLAVE_ADRESS_EEPROM,ups_eeprom_rec_datas_adrr,16,UPS_eeprom_buffer,I2C_EEPROM_BASE);
                    DEVICE_DELAY_US(6000);//6ms
                    j=0;
                    ups_eeprom_rec_datas_adrr=(uint16_t)(ups_eeprom_rec_datas_adrr + 128);
                }
            }
        }
      }
      else if(rec_data_log_eeprom_wr_page == 8){
        //********* V_DC_bar_lower_meas_values *************//
        ups_eeprom_rec_datas_adrr=UPS_EEPROM_VDC_BAR_LOW_DATAS_ADRR;
        for(i=0; i < REC_AC_MEAS_VECTOR_SIZE ; i++){
            UPS_eeprom_buffer[j] = (uint16_t)( V_DC_bar_lower_meas_values[i] + 32768);
            j++;
            if(i < 1984){
                if(j==64){
                    eeprom_status=I2C_MasterTransmitter_Word(SLAVE_ADRESS_EEPROM,ups_eeprom_rec_datas_adrr,64,UPS_eeprom_buffer,I2C_EEPROM_BASE);
                    DEVICE_DELAY_US(6000);//6ms
                    j=0;
                    ups_eeprom_rec_datas_adrr=(uint16_t)(ups_eeprom_rec_datas_adrr + 128);
                }
            }else{
                if(j==16){
                    eeprom_status=I2C_MasterTransmitter_Word(SLAVE_ADRESS_EEPROM,ups_eeprom_rec_datas_adrr,16,UPS_eeprom_buffer,I2C_EEPROM_BASE);
                    DEVICE_DELAY_US(6000);//6ms
                    j=0;
                    ups_eeprom_rec_datas_adrr=(uint16_t)(ups_eeprom_rec_datas_adrr + 128);
                }
            }
        }
      }
      else if(rec_data_log_eeprom_wr_page == 9){
        //********* I_battery_upper_meas_values *************//
            ups_eeprom_rec_datas_adrr=UPS_EEPROM_I_BAT_UP_DATAS_ADRR;
        for(i=0; i < REC_AC_MEAS_VECTOR_SIZE ; i++){
            UPS_eeprom_buffer[j] = (uint16_t)( I_battery_upper_meas_values[i] + 32768);
            j++;
            if(i < 1984){
                if(j==64){
                    eeprom_status=I2C_MasterTransmitter_Word(SLAVE_ADRESS_EEPROM,ups_eeprom_rec_datas_adrr,64,UPS_eeprom_buffer,I2C_EEPROM_BASE);
                    DEVICE_DELAY_US(6000);//6ms
                    j=0;
                    ups_eeprom_rec_datas_adrr=(uint16_t)(ups_eeprom_rec_datas_adrr + 128);
                }
            }else{
                if(j==16){
                    eeprom_status=I2C_MasterTransmitter_Word(SLAVE_ADRESS_EEPROM,ups_eeprom_rec_datas_adrr,16,UPS_eeprom_buffer,I2C_EEPROM_BASE);
                    DEVICE_DELAY_US(6000);//6ms
                    j=0;
                    ups_eeprom_rec_datas_adrr=(uint16_t)(ups_eeprom_rec_datas_adrr + 128);
                }
            }
        }
      }
      else if(rec_data_log_eeprom_wr_page == 10){
          rec_data_log_eeprom_wr_page=0;
          rec_data_log_eeprom_wr_active_fl=0;
        //********* I_battery_lower_meas_values *************//
        ups_eeprom_rec_datas_adrr=UPS_EEPROM_I_BAT_LOW_DATAS_ADRR;
        for(i=0; i < REC_AC_MEAS_VECTOR_SIZE ; i++){
            UPS_eeprom_buffer[j] = (uint16_t)( I_battery_lower_meas_values[i] + 32768);
            j++;
            if(i < 1984){
                if(j==64){
                    eeprom_status=I2C_MasterTransmitter_Word(SLAVE_ADRESS_EEPROM,ups_eeprom_rec_datas_adrr,64,UPS_eeprom_buffer,I2C_EEPROM_BASE);
                    DEVICE_DELAY_US(6000);//6ms
                    j=0;
                    ups_eeprom_rec_datas_adrr=(uint16_t)(ups_eeprom_rec_datas_adrr + 128);
                }
            }else{
                if(j==16){
                    eeprom_status=I2C_MasterTransmitter_Word(SLAVE_ADRESS_EEPROM,ups_eeprom_rec_datas_adrr,16,UPS_eeprom_buffer,I2C_EEPROM_BASE);
                    DEVICE_DELAY_US(6000);//6ms
                    j=0;
                    ups_eeprom_rec_datas_adrr=(uint16_t)(ups_eeprom_rec_datas_adrr + 128);
                }
            }
        }
      }
    }


      //***************************************************
      // UPS rec verilerinin terminala aktırımını içeren yapı
      // Rectifier sistemi durmuş ise terminalden aktarım yapılmalı
      //NOT: ups_inv_data_log_send_terminal_active_cpu2 bu değişken ile inverter data logları gönderiliyorsa rectifierden gönderme isteği olmasın ayrıca , bir kez inv verilerini gönder isteği geldi ise işlem bitene kadar bir daha veri göndermesin
      //***************************************************
    if(rec_uart_comm_active_for_real_term_fl == 1 && ups_rec_data_log_send_terminal_active == 0 && rec_uart_comm_passive_fl == 0 && inv_uart_comm_active_for_real_term_fl == 0 && thyr_input_side_active_fl == 0 && start_control_syst_PFC_boost == 0 && ups_inv_data_log_send_terminal_active_cpu2 == 0){

        EALLOW;
        SysCtl_setStandbyQualificationPeriod(2);
        SysCtl_disableMCD();
        SysCtl_selectCPUForPeripheral(SYSCTL_CPUSEL5_SCI, 3, SYSCTL_CPUSEL_CPU1);
        EDIS;
        DEVICE_DELAY_US(60);
        ups_rec_data_log_send_terminal_active=1;
        rec_uart_comm_passive_fl=1;
    }
    else if(inv_uart_comm_active_for_real_term_fl == 1 && rec_uart_comm_active_for_real_term_fl == 0 && inv_uart_comm_active == 0 && ups_rec_data_log_send_terminal_active == 0 ){

        EALLOW;
        SysCtl_setStandbyQualificationPeriod(2);
        SysCtl_disableMCD();
        SysCtl_selectCPUForPeripheral(SYSCTL_CPUSEL5_SCI, 3, SYSCTL_CPUSEL_CPU2);
        EDIS;
        DEVICE_DELAY_US(60);

        inv_uart_comm_active=1;
    }
    if(rec_uart_comm_active_for_real_term_fl == 0 && rec_uart_comm_passive_fl == 1){
        rec_uart_comm_passive_fl=0;
    }
    if(inv_uart_comm_active_for_real_term_fl == 0 && inv_uart_comm_active == 1 && ups_inv_data_log_send_terminal_active_cpu2 == 0){
        inv_uart_comm_active=0;
    }

    //*******************************************
    //NOT1: UPS'in rectifier verileri terminale gönderilirken her bir döngüde bir veri bilgisi gönderilmeli
    //********************************************
    if(ups_rec_data_log_send_terminal_active == 1){
        rec_data_log_send_terminal_page++;
         if(rec_data_log_send_terminal_page == 1){
           eeprom_status1=I2C_MasterReceiver_Word(SLAVE_ADRESS_EEPROM,UPS_EEPROM_REC_DATAS_ADRR,2000,UPS_eeprom_buffer,I2C_EEPROM_BASE);
           DEVICE_DELAY_US(I2C_EEPROM_READ_DELAY);
           length_debug = sprintf(uartBuff, "vinR=[\n");
           SCI_writeCharArray(UPS_REC_UART0_ISO_BASE, (uint8_t*)uartBuff, length_debug);

           // Gerilim değerlerini yaz
           for ( i= 0; i < REC_AC_MEAS_VECTOR_SIZE; i++) {
               length_debug = sprintf(uartBuff, "%d\n",  (int16_t)( UPS_eeprom_buffer[i] - 32768));
               SCI_writeCharArray(UPS_REC_UART0_ISO_BASE, (uint8_t*)uartBuff, length_debug);
           }
         // Kapanış mesajı
           length_debug = sprintf(uartBuff, "];\n");
           SCI_writeCharArray(UPS_REC_UART0_ISO_BASE, (uint8_t*)uartBuff, length_debug);
         }
         else if(rec_data_log_send_terminal_page == 2){
           //********************************
           eeprom_status1=I2C_MasterReceiver_Word(SLAVE_ADRESS_EEPROM,UPS_EEPROM_REC_VS_DATAS_ADRR,2000,UPS_eeprom_buffer,I2C_EEPROM_BASE);
           DEVICE_DELAY_US(I2C_EEPROM_READ_DELAY);
           // VinputS ani değerleri
           length_debug = sprintf(uartBuff, "vinS=[\n");
           SCI_writeCharArray(UPS_REC_UART0_ISO_BASE, (uint8_t*)uartBuff, length_debug);
           for ( i= 0; i < REC_AC_MEAS_VECTOR_SIZE; i++) {
               length_debug = sprintf(uartBuff, "%d\n", (int16_t)( UPS_eeprom_buffer[i] - 32768));
               SCI_writeCharArray(UPS_REC_UART0_ISO_BASE, (uint8_t*)uartBuff, length_debug);
           }
           length_debug = sprintf(uartBuff, "];\n");
           SCI_writeCharArray(UPS_REC_UART0_ISO_BASE, (uint8_t*)uartBuff, length_debug);
         }
         else if(rec_data_log_send_terminal_page == 3){
           //********************************
           eeprom_status1=I2C_MasterReceiver_Word(SLAVE_ADRESS_EEPROM,UPS_EEPROM_REC_VT_DATAS_ADRR,2000,UPS_eeprom_buffer,I2C_EEPROM_BASE);
           DEVICE_DELAY_US(I2C_EEPROM_READ_DELAY);
           // VinputT ani değerleri
           length_debug = sprintf(uartBuff, "vinT=[\n");
           SCI_writeCharArray(UPS_REC_UART0_ISO_BASE, (uint8_t*)uartBuff, length_debug);
           for ( i= 0; i < REC_AC_MEAS_VECTOR_SIZE; i++) {
               length_debug = sprintf(uartBuff, "%d\n", (int16_t)( UPS_eeprom_buffer[i] - 32768));
               SCI_writeCharArray(UPS_REC_UART0_ISO_BASE, (uint8_t*)uartBuff, length_debug);
           }
           length_debug = sprintf(uartBuff, "];\n");
           SCI_writeCharArray(UPS_REC_UART0_ISO_BASE, (uint8_t*)uartBuff, length_debug);
         }
         else if(rec_data_log_send_terminal_page == 4){
           //********************************
           eeprom_status1=I2C_MasterReceiver_Word(SLAVE_ADRESS_EEPROM,UPS_EEPROM_REC_IR_DATAS_ADRR,2000,UPS_eeprom_buffer,I2C_EEPROM_BASE);
           DEVICE_DELAY_US(I2C_EEPROM_READ_DELAY);
           // IinputR ani değerleri
           length_debug = sprintf(uartBuff, "IinR=[\n");
           SCI_writeCharArray(UPS_REC_UART0_ISO_BASE, (uint8_t*)uartBuff, length_debug);
           for ( i= 0; i < REC_AC_MEAS_VECTOR_SIZE; i++) {
               length_debug = sprintf(uartBuff, "%d\n",  (int16_t)( UPS_eeprom_buffer[i] - 32768));
               SCI_writeCharArray(UPS_REC_UART0_ISO_BASE, (uint8_t*)uartBuff, length_debug);
           }
           length_debug = sprintf(uartBuff, "];\n");
           SCI_writeCharArray(UPS_REC_UART0_ISO_BASE, (uint8_t*)uartBuff, length_debug);
         }
         else if(rec_data_log_send_terminal_page == 5){
           //********************************
           eeprom_status1=I2C_MasterReceiver_Word(SLAVE_ADRESS_EEPROM,UPS_EEPROM_REC_IS_DATAS_ADRR,2000,UPS_eeprom_buffer,I2C_EEPROM_BASE);
           DEVICE_DELAY_US(I2C_EEPROM_READ_DELAY);
           // IinputS ani değerleri
           length_debug = sprintf(uartBuff, "IinS=[\n");
           SCI_writeCharArray(UPS_REC_UART0_ISO_BASE, (uint8_t*)uartBuff, length_debug);
           for ( i= 0; i < REC_AC_MEAS_VECTOR_SIZE; i++) {
               length_debug = sprintf(uartBuff, "%d\n", (int16_t)( UPS_eeprom_buffer[i] - 32768));
               SCI_writeCharArray(UPS_REC_UART0_ISO_BASE, (uint8_t*)uartBuff, length_debug);
           }
           length_debug = sprintf(uartBuff, "];\n");
           SCI_writeCharArray(UPS_REC_UART0_ISO_BASE, (uint8_t*)uartBuff, length_debug);
         }
         else if(rec_data_log_send_terminal_page == 6){
           //********************************
           eeprom_status1=I2C_MasterReceiver_Word(SLAVE_ADRESS_EEPROM,UPS_EEPROM_REC_IT_DATAS_ADRR,2000,UPS_eeprom_buffer,I2C_EEPROM_BASE);
           DEVICE_DELAY_US(I2C_EEPROM_READ_DELAY);
           // IinputT ani değerleri
           length_debug = sprintf(uartBuff, "IinT=[\n");
           SCI_writeCharArray(UPS_REC_UART0_ISO_BASE, (uint8_t*)uartBuff, length_debug);
           for ( i= 0; i < REC_AC_MEAS_VECTOR_SIZE; i++) {
               length_debug = sprintf(uartBuff, "%d\n", (int16_t)( UPS_eeprom_buffer[i] - 32768));
               SCI_writeCharArray(UPS_REC_UART0_ISO_BASE, (uint8_t*)uartBuff, length_debug);
           }
           length_debug = sprintf(uartBuff, "];\n");
           SCI_writeCharArray(UPS_REC_UART0_ISO_BASE, (uint8_t*)uartBuff, length_debug);
         }
         else if(rec_data_log_send_terminal_page == 7){
           //********************************
           eeprom_status1=I2C_MasterReceiver_Word(SLAVE_ADRESS_EEPROM,UPS_EEPROM_VDC_BAR_UP_DATAS_ADRR,2000,UPS_eeprom_buffer,I2C_EEPROM_BASE);
           DEVICE_DELAY_US(I2C_EEPROM_READ_DELAY);
           // DC BAR UPPER ani değerleri
           length_debug = sprintf(uartBuff, "VDC_Upper =[\n");
           SCI_writeCharArray(UPS_REC_UART0_ISO_BASE, (uint8_t*)uartBuff, length_debug);
           for ( i= 0; i < REC_DC_MEAS_VECTOR_SIZE; i++) {
               length_debug = sprintf(uartBuff, "%d\n", (int16_t)( UPS_eeprom_buffer[i] - 32768));
               SCI_writeCharArray(UPS_REC_UART0_ISO_BASE, (uint8_t*)uartBuff, length_debug);
           }
           length_debug = sprintf(uartBuff, "];\n");
           SCI_writeCharArray(UPS_REC_UART0_ISO_BASE, (uint8_t*)uartBuff, length_debug);
         }
         else if(rec_data_log_send_terminal_page == 8){
           //********************************
           eeprom_status1=I2C_MasterReceiver_Word(SLAVE_ADRESS_EEPROM,UPS_EEPROM_VDC_BAR_LOW_DATAS_ADRR,2000,UPS_eeprom_buffer,I2C_EEPROM_BASE);
           DEVICE_DELAY_US(I2C_EEPROM_READ_DELAY);
           // DC BAR LOWER ani değerleri
           length_debug = sprintf(uartBuff, "VDC_Lower =[\n");
           SCI_writeCharArray(UPS_REC_UART0_ISO_BASE, (uint8_t*)uartBuff, length_debug);
           for ( i= 0; i < REC_DC_MEAS_VECTOR_SIZE; i++) {
               length_debug = sprintf(uartBuff, "%d\n", (int16_t)( UPS_eeprom_buffer[i] - 32768));
               SCI_writeCharArray(UPS_REC_UART0_ISO_BASE, (uint8_t*)uartBuff, length_debug);
           }
           length_debug = sprintf(uartBuff, "];\n");
           SCI_writeCharArray(UPS_REC_UART0_ISO_BASE, (uint8_t*)uartBuff, length_debug);
         }
         else if(rec_data_log_send_terminal_page == 9){
           //********************************
           eeprom_status1=I2C_MasterReceiver_Word(SLAVE_ADRESS_EEPROM,UPS_EEPROM_I_BAT_UP_DATAS_ADRR,2000,UPS_eeprom_buffer,I2C_EEPROM_BASE);
           DEVICE_DELAY_US(I2C_EEPROM_READ_DELAY);
           // I battery upper ani değerleri
           length_debug = sprintf(uartBuff, "Ibat_upper =[\n");
           SCI_writeCharArray(UPS_REC_UART0_ISO_BASE, (uint8_t*)uartBuff, length_debug);
           for ( i= 0; i < REC_DC_MEAS_VECTOR_SIZE; i++) {
               length_debug = sprintf(uartBuff, "%d\n", (int16_t)( UPS_eeprom_buffer[i] - 32768));
               SCI_writeCharArray(UPS_REC_UART0_ISO_BASE, (uint8_t*)uartBuff, length_debug);
           }
           length_debug = sprintf(uartBuff, "];\n");
           SCI_writeCharArray(UPS_REC_UART0_ISO_BASE, (uint8_t*)uartBuff, length_debug);
         }
         else if(rec_data_log_send_terminal_page == 10){
           //********************************
           eeprom_status1=I2C_MasterReceiver_Word(SLAVE_ADRESS_EEPROM,UPS_EEPROM_I_BAT_LOW_DATAS_ADRR,2000,UPS_eeprom_buffer,I2C_EEPROM_BASE);
           DEVICE_DELAY_US(I2C_EEPROM_READ_DELAY);
           // I battery lower ani değerleri
           length_debug = sprintf(uartBuff, "Ibat_lower =[\n");
           SCI_writeCharArray(UPS_REC_UART0_ISO_BASE, (uint8_t*)uartBuff, length_debug);
           for ( i= 0; i < REC_DC_MEAS_VECTOR_SIZE; i++) {
               length_debug = sprintf(uartBuff, "%d\n", (int16_t)( UPS_eeprom_buffer[i] - 32768));
               SCI_writeCharArray(UPS_REC_UART0_ISO_BASE, (uint8_t*)uartBuff, length_debug);
           }
           length_debug = sprintf(uartBuff, "];\n");
           SCI_writeCharArray(UPS_REC_UART0_ISO_BASE, (uint8_t*)uartBuff, length_debug);
         }
         else if(rec_data_log_send_terminal_page == 11){
             rec_data_log_send_terminal_page=0;
             ups_rec_data_log_send_terminal_active=0;

             // MAX  DEĞERLERİ
             length_debug = sprintf(uartBuff, "V_R_in_max = %d\n", (int16_t)V_R_in_meas_max);
             SCI_writeCharArray(UPS_REC_UART0_ISO_BASE, (uint8_t*)uartBuff, length_debug);

             length_debug = sprintf(uartBuff, "V_S_in_max = %d\n", (int16_t)V_S_in_meas_max);
             SCI_writeCharArray(UPS_REC_UART0_ISO_BASE, (uint8_t*)uartBuff, length_debug);

             length_debug = sprintf(uartBuff, "V_T_in_max = %d\n", (int16_t)V_T_in_meas_max);
             SCI_writeCharArray(UPS_REC_UART0_ISO_BASE, (uint8_t*)uartBuff, length_debug);

             length_debug = sprintf(uartBuff, "I_R_in_max = %d\n", (int16_t)I_R_in_meas_max);
             SCI_writeCharArray(UPS_REC_UART0_ISO_BASE, (uint8_t*)uartBuff, length_debug);

             length_debug = sprintf(uartBuff, "I_S_in_max = %d\n", (int16_t)I_S_in_meas_max);
             SCI_writeCharArray(UPS_REC_UART0_ISO_BASE, (uint8_t*)uartBuff, length_debug);

             length_debug = sprintf(uartBuff, "I_T_in_max = %d\n", (int16_t)I_T_in_meas_max);
             SCI_writeCharArray(UPS_REC_UART0_ISO_BASE, (uint8_t*)uartBuff, length_debug);

             length_debug = sprintf(uartBuff, "V_DC_up_max = %d\n", (int16_t)V_DC_bar_upper_meas_max);
             SCI_writeCharArray(UPS_REC_UART0_ISO_BASE, (uint8_t*)uartBuff, length_debug);

             length_debug = sprintf(uartBuff, "V_DC_low_max = %d\n", (int16_t)V_DC_bar_lower_meas_max);
             SCI_writeCharArray(UPS_REC_UART0_ISO_BASE, (uint8_t*)uartBuff, length_debug);

             length_debug = sprintf(uartBuff, "I_bat_up_max = %d\n", (int16_t)I_battery_upper_meas_max*100);
             SCI_writeCharArray(UPS_REC_UART0_ISO_BASE, (uint8_t*)uartBuff, length_debug);

             length_debug = sprintf(uartBuff, "I_bat_low_max = %d\n",(int16_t)I_battery_lower_meas_max*100);
             SCI_writeCharArray(UPS_REC_UART0_ISO_BASE, (uint8_t*)uartBuff, length_debug);

             // MIN  DEĞERLERİ
             length_debug = sprintf(uartBuff, "V_R_in_min = %d\n", (int16_t)V_R_in_meas_min);
             SCI_writeCharArray(UPS_REC_UART0_ISO_BASE, (uint8_t*)uartBuff, length_debug);

             length_debug = sprintf(uartBuff, "V_S_in_min = %d\n", (int16_t)V_S_in_meas_min);
             SCI_writeCharArray(UPS_REC_UART0_ISO_BASE, (uint8_t*)uartBuff, length_debug);

             length_debug = sprintf(uartBuff, "V_T_in_min = %d\n", (int16_t)V_T_in_meas_min);
             SCI_writeCharArray(UPS_REC_UART0_ISO_BASE, (uint8_t*)uartBuff, length_debug);

             length_debug = sprintf(uartBuff, "I_R_in_min = %d\n", (int16_t)I_R_in_meas_min);
             SCI_writeCharArray(UPS_REC_UART0_ISO_BASE, (uint8_t*)uartBuff, length_debug);

             length_debug = sprintf(uartBuff, "I_S_in_min = %d\n", (int16_t)I_S_in_meas_min);
             SCI_writeCharArray(UPS_REC_UART0_ISO_BASE, (uint8_t*)uartBuff, length_debug);

             length_debug = sprintf(uartBuff, "I_T_in_min = %d\n", (int16_t)I_T_in_meas_min);
             SCI_writeCharArray(UPS_REC_UART0_ISO_BASE, (uint8_t*)uartBuff, length_debug);

             length_debug = sprintf(uartBuff, "V_DC_up_min = %d\n", (int16_t)V_DC_bar_upper_meas_min);
             SCI_writeCharArray(UPS_REC_UART0_ISO_BASE, (uint8_t*)uartBuff, length_debug);

             length_debug = sprintf(uartBuff, "V_DC_low_min = %d\n", (int16_t)V_DC_bar_lower_meas_min);
             SCI_writeCharArray(UPS_REC_UART0_ISO_BASE, (uint8_t*)uartBuff, length_debug);

             length_debug = sprintf(uartBuff, "I_bat_up_min = %d\n", (int16_t)I_battery_upper_meas_min*100);
             SCI_writeCharArray(UPS_REC_UART0_ISO_BASE, (uint8_t*)uartBuff, length_debug);

             length_debug = sprintf(uartBuff, "I_bat_low_min = %d\n", (int16_t)I_battery_lower_meas_min*100);
             SCI_writeCharArray(UPS_REC_UART0_ISO_BASE, (uint8_t*)uartBuff, length_debug);
         }
    }


    //****************************************//
       TaskComm();    // NOT: Ön panele gönderilecek  veriler
    //****************************************//


       //**************************************
       // Inverterden gelen hata bilgisinin kayıtları ve Arıza kodlarının eeproma kaydedilmesi
       //**************************************
       if(inv_err_active_for_cpu2 == 1 && inv_err_record_fl == 0){
         ups_error_count=addError(error_code_for_inv_from_cpu2,timer_1sec_for_ups_system,ups_error_count);//error
         inv_err_record_fl=1;
         inv_err_eeprom_wr_active_fl=1;
       }
       else if(inv_err_active_for_cpu2 == 0){
           inv_err_record_fl=0;
       }

       if(rec_err_eeprom_wr_active_fl == 1 && inv_err_eeprom_wr_active_fl == 1){

           ups_error_count_index++;
           if(ups_err_data_full < ups_error_count_index){
//               ups_error_count=ups_error_count_index;
               ups_err_data_full=ups_error_count_index;
            }
           ups_error_count_index = (ups_error_count_index >= UPS_MAX_ERRORS) ? 0 : ups_error_count_index;

           ups_eeprom_error_buffer[0] = ups_error_vector[0];
           ups_eeprom_error_buffer[1] = ups_error_vector[1];
           eeprom_status=I2C_MasterTransmitter_DWord(SLAVE_ADRESS_EEPROM,ups_eeprom_err_adrr,2,ups_eeprom_error_buffer,I2C_EEPROM_BASE);
           DEVICE_DELAY_US(6000);//6ms
           ups_eeprom_err_adrr=ups_eeprom_err_adrr + 8;
           ups_eeprom_err_adrr=(ups_eeprom_err_adrr < 3408) ? ups_eeprom_err_adrr : 1408; //500. hatadan sonra başa dönsün

           UPS_eeprom_buffer[0] = ups_err_data_full;
           UPS_eeprom_buffer[1] = ups_error_count_index;
           UPS_eeprom_buffer[2] = ups_eeprom_err_adrr;
           eeprom_status=I2C_MasterTransmitter_Word(SLAVE_ADRESS_EEPROM,UPS_EEPROM_ERR_DATA_COUNT_ADRR,3,UPS_eeprom_buffer,I2C_EEPROM_BASE);
           DEVICE_DELAY_US(6000);//6ms

           rec_err_eeprom_wr_active_fl=0;
           inv_err_eeprom_wr_active_fl=0;
           err_eeprom_write_debug_cnt++;
       }
       else if(rec_err_eeprom_wr_active_fl == 1 || inv_err_eeprom_wr_active_fl == 1){ // herhangi biri bir ise
           ups_error_count_index++;
           if(ups_err_data_full < ups_error_count_index){
//                ups_error_count=ups_error_count_index;
                ups_err_data_full=ups_error_count_index;
             }
           ups_error_count_index = (ups_error_count_index >= UPS_MAX_ERRORS) ? 0 : ups_error_count_index;

           ups_eeprom_error_buffer[0] = ups_error_vector[0];
           eeprom_status=I2C_MasterTransmitter_DWord(SLAVE_ADRESS_EEPROM,ups_eeprom_err_adrr,1,ups_eeprom_error_buffer,I2C_EEPROM_BASE);
           DEVICE_DELAY_US(6000);//6ms
           ups_eeprom_err_adrr=ups_eeprom_err_adrr + 4;
           ups_eeprom_err_adrr=(ups_eeprom_err_adrr < 3408) ? ups_eeprom_err_adrr : 1408; //500. hatadan sonra başa dönsün

           UPS_eeprom_buffer[0] = ups_err_data_full;
           UPS_eeprom_buffer[1] = ups_error_count_index;
           UPS_eeprom_buffer[2] = ups_eeprom_err_adrr;
           eeprom_status=I2C_MasterTransmitter_Word(SLAVE_ADRESS_EEPROM,UPS_EEPROM_ERR_DATA_COUNT_ADRR,3,UPS_eeprom_buffer,I2C_EEPROM_BASE);
           DEVICE_DELAY_US(6000);//6ms

           rec_err_eeprom_wr_active_fl=0;
           inv_err_eeprom_wr_active_fl=0;
           err_eeprom_write_debug_cnt1++;
       }


        //***************************************************
        // olay hafızasına ait verilerinin ram ve eeprom'dan  silinmesi
        //***************************************************
       if(ups_eeprom_reset_active_fl == 1 && ups_eeprom_reset_compl_fl == 0){

         for(i=0; i < ups_err_data_full; i++){
           ups_error_vector[i]=0;
         }
           ups_error_count=0;
           ups_err_data_full=0;
           ups_error_count_index=0;
           ups_eeprom_err_adrr=UPS_EEPROM_ERR_DATA_ADRR;
           UPS_eeprom_buffer[0] = ups_err_data_full;
           UPS_eeprom_buffer[1] = ups_error_count_index;
           UPS_eeprom_buffer[2] = ups_eeprom_err_adrr;
           eeprom_status=I2C_MasterTransmitter_Word(SLAVE_ADRESS_EEPROM,UPS_EEPROM_ERR_DATA_COUNT_ADRR,3,UPS_eeprom_buffer,I2C_EEPROM_BASE);
           DEVICE_DELAY_US(6000);//6ms
           ups_eeprom_reset_compl_fl=1;
       }
       else if(ups_eeprom_reset_active_fl == 0 && ups_eeprom_reset_compl_fl == 1){
           ups_eeprom_reset_compl_fl=0;
       }

    //***************************************************
    // eeproma kalibrasyon verilerinin yazılması
    // servis menüsünden çıkarken eeprom kayıt işlemine evet ise
    // eğer bir kez bile kayıt yapılmış ise I2C_eeprom_write_fl=0 olur eğer kalibrasyon bölümüne tekrar girilirse bu flag yeniden aktif olur
    // tekrar eeprom kayıt istenirse yine kayıt yapılır
    //***************************************************
    if(I2C_eeprom_active_fl == 1 && I2C_eeprom_write_fl == 1){

        UPS_eeprom_buffer[0]=(uint16_t)(V_R_in_meas_coeff*10.0f);
        UPS_eeprom_buffer[1]=(uint16_t)(V_S_in_meas_coeff*10.0f);
        UPS_eeprom_buffer[2]=(uint16_t)(V_T_in_meas_coeff*10.0f);
        UPS_eeprom_buffer[3]=(uint16_t)(I_R_in_meas_coeff*10.0f);
        UPS_eeprom_buffer[4]=(uint16_t)(I_S_in_meas_coeff*10.0f);
        UPS_eeprom_buffer[5]=(uint16_t)(I_T_in_meas_coeff*10.0f);

        UPS_eeprom_buffer[6]=(uint16_t)(V_R_out_meas_coeff_eeprom*10.0f);
        UPS_eeprom_buffer[7]=(uint16_t)(V_S_out_meas_coeff_eeprom*10.0f);
        UPS_eeprom_buffer[8]=(uint16_t)(V_T_out_meas_coeff_eeprom*10.0f);
        UPS_eeprom_buffer[9]=(uint16_t)(I_R_out_meas_coeff_eeprom*10.0f);
        UPS_eeprom_buffer[10]=(uint16_t)(I_S_out_meas_coeff_eeprom*10.0f);
        UPS_eeprom_buffer[11]=(uint16_t)(I_T_out_meas_coeff_eeprom*10.0f);

        UPS_eeprom_buffer[12]=(uint16_t)(V_Battery_upper_coeff_eeprom*10.0f);
        UPS_eeprom_buffer[13]=(uint16_t)(V_DC_bar_upper_meas_coeff*10.0f);
        UPS_eeprom_buffer[14]=(uint16_t)(V_DC_bar_lower_meas_coeff*10.0f);
        UPS_eeprom_buffer[15]=(uint16_t)(I_battery_upper_meas_coeff*10.0f);
        UPS_eeprom_buffer[16]=(uint16_t)(I_battery_lower_meas_coeff*10.0f);

        UPS_eeprom_buffer[17]=(uint16_t)(V_R_bypass_meas_coeff_eeprom*10.0f);
        UPS_eeprom_buffer[18]=(uint16_t)(V_S_bypass_meas_coeff_eeprom*10.0f);
        UPS_eeprom_buffer[19]=(uint16_t)(V_T_bypass_meas_coeff_eeprom*10.0f);

        UPS_eeprom_buffer[20]=(uint16_t)(V_R_load_meas_coeff_eeprom*10.0f);
        UPS_eeprom_buffer[21]=(uint16_t)(V_S_load_meas_coeff_eeprom*10.0f);
        UPS_eeprom_buffer[22]=(uint16_t)(V_T_load_meas_coeff_eeprom*10.0f);
        UPS_eeprom_buffer[23]=(uint16_t)(I_R_load_meas_coeff_eeprom*10.0f);
        UPS_eeprom_buffer[24]=(uint16_t)(I_S_load_meas_coeff_eeprom*10.0f);
        UPS_eeprom_buffer[25]=(uint16_t)(I_T_load_meas_coeff_eeprom*10.0f);
        eeprom_status=I2C_MasterTransmitter_Word(SLAVE_ADRESS_EEPROM,V_R_IN_MEAS_COEFF_ADR,26,UPS_eeprom_buffer,I2C_EEPROM_BASE);
        DEVICE_DELAY_US(6000);//6ms

        I2C_eeprom_read_active_fl_debug=17;
        I2C_eeprom_write_fl=0;
    }

    //***************************************************
    // IGBT-REC_Desat_Fault Monitor
    //***************************************************
    if(rec_igbt_fault_try_return == 0){

        //****** IGBT-REC_R Desat_Fault Monitor -> turn off system ******//
        fault_pin_current_state_for_rec_igbt_R = rdpin(REC_R_ERR_M); // Fault pin okuma
        if(fault_pin_current_state_for_rec_igbt_R==1){
            fault_pin_current_state_cnt_for_rec_igbt_R++;
        }else{
            fault_pin_current_state_cnt_for_rec_igbt_R=0;
        }

        if (fault_pin_current_state_cnt_for_rec_igbt_R==10) {
            fault_pin_current_state_cnt_for_rec_igbt_R=0;
            rec_igbt_R_err=1;       rec_igbt_fault_try_return=1; rec_igbt_R_fault_repeat++;
            stop_pwm_PFC_boost();
            start_control_syst_PFC_boost=0;
            turn_off_input_thyr();
            thyr_input_side_turn_off_complete_delay=1;
            rec_data_log_stop_fl=1;
            return_rec_igbt_fault_fault_cnt=0;
            ups_error_count=addError(ERR_REC_IGBT_R,timer_1sec_for_ups_system,ups_error_count); rec_err_eeprom_wr_active_fl=1;//error
            if (rec_igbt_R_fault_repeat > 1) {
                rec_igbt_R_fault_persists=1;
            }
        }

        //****** IGBT-REC_S Desat_Fault Monitor -> turn off system ******//
        fault_pin_current_state_for_rec_igbt_S = rdpin(REC_S_ERR_M); // Fault pin okuma
        if(fault_pin_current_state_for_rec_igbt_S==1){
            fault_pin_current_state_cnt_for_rec_igbt_S++;
        }else{
            fault_pin_current_state_cnt_for_rec_igbt_S=0;
        }

        if (fault_pin_current_state_cnt_for_rec_igbt_S==10) {
            fault_pin_current_state_cnt_for_rec_igbt_S=0;
            rec_igbt_S_err=1;       rec_igbt_fault_try_return=1; rec_igbt_S_fault_repeat++;
            stop_pwm_PFC_boost();
            start_control_syst_PFC_boost=0;
            turn_off_input_thyr();
            thyr_input_side_turn_off_complete_delay=1;
            rec_data_log_stop_fl=1;
            return_rec_igbt_fault_fault_cnt=0;
            ups_error_count=addError(ERR_REC_IGBT_S,timer_1sec_for_ups_system,ups_error_count); rec_err_eeprom_wr_active_fl=1; //error
            if (rec_igbt_S_fault_repeat > 1) {
                rec_igbt_S_fault_persists=1;
            }
        }

        //****** IGBT-REC_T Desat_Fault Monitor -> turn off system ******//
        fault_pin_current_state_for_rec_igbt_T = rdpin(REC_T_ERR_M); // Fault pin okuma
        if(fault_pin_current_state_for_rec_igbt_T==1){
            fault_pin_current_state_cnt_for_rec_igbt_T++;
        }else{
            fault_pin_current_state_cnt_for_rec_igbt_T=0;
        }
        if (fault_pin_current_state_cnt_for_rec_igbt_T==10) {
            fault_pin_current_state_cnt_for_rec_igbt_T=0;
            rec_igbt_T_err=1;       rec_igbt_fault_try_return=1; rec_igbt_T_fault_repeat++;
            stop_pwm_PFC_boost();
            start_control_syst_PFC_boost=0;
            turn_off_input_thyr();
            thyr_input_side_turn_off_complete_delay=1;
            rec_data_log_stop_fl=1;
            return_rec_igbt_fault_fault_cnt=0;
            ups_error_count=addError(ERR_REC_IGBT_T,timer_1sec_for_ups_system,ups_error_count); rec_err_eeprom_wr_active_fl=1;//error
            if (rec_igbt_T_fault_repeat > 1) {
                rec_igbt_T_fault_persists=1;
            }
        }
    }


    //***************************************************
    // Description: bu yapı ups rectifier sisteminin arkaplan yavaş işlemleri için kullanılıyor
    // timer0 1ms period function
    //***************************************************

    if(TIMER0_PERIOD_FNC == 1){
        TIMER0_PERIOD_FNC_CLR_FL;

        if(led_control_cpu1++ >= 1000){
            led_control_cpu1=0;
            tgpin(LED4);
        }

       //NOT: sistemdeki hata oluştuğunda kaç saniye geçtiğini belirleyen sayaç, 2^24 olduğunda tekrar sıfırlanmalı
        if(timer_cnt_for_1sec++ >= 1000){
            timer_cnt_for_1sec=0;
            timer_1sec_for_ups_system++;
            timer_1sec_for_ups_system=(timer_1sec_for_ups_system > UPS_SYSTEM_TIMER_MAX_VALUE) ? 0 : timer_1sec_for_ups_system;
        }

       // NOT: son 1 saniye min max 10ms +1 veya -1 , son bir saniye anlık ölçümler , max 100V saniye ile azalacak,min 100V saniye ile yükselecek
       // NOT1:Sistem çalışmaya başladıktan sonra verilerin kapasite gibi azalıp artmalı
        if( rec_data_log_stop_fl == 0 && rec_data_log_meas_active_fl == 0 && rec_data_log_eeprom_wr_active_fl == 0){
            if(input_meas_max_min_values_cnt++ >= 10 ){
                input_meas_max_min_values_cnt=0;
                V_R_in_meas_max=V_R_in_meas_max - 1.0f;
                V_S_in_meas_max=V_S_in_meas_max - 1.0f;
                V_T_in_meas_max=V_T_in_meas_max - 1.0f;
                I_R_in_meas_max=I_R_in_meas_max - 1.0f;
                I_S_in_meas_max=I_S_in_meas_max - 1.0f;
                I_T_in_meas_max=I_T_in_meas_max - 1.0f;
                V_DC_bar_lower_meas_max=V_DC_bar_lower_meas_max - 1.0f;
                V_DC_bar_upper_meas_max=V_DC_bar_upper_meas_max - 1.0f;
                I_battery_lower_meas_max=I_battery_lower_meas_max - 1.0f;
                I_battery_upper_meas_max=I_battery_upper_meas_max - 1.0f;

                V_R_in_meas_min=V_R_in_meas_min + 1.0f;
                V_S_in_meas_min=V_S_in_meas_min + 1.0f;
                V_T_in_meas_min=V_T_in_meas_min + 1.0f;
                I_R_in_meas_min=I_R_in_meas_min + 1.0f;
                I_S_in_meas_min=I_S_in_meas_min + 1.0f;
                I_T_in_meas_min=I_T_in_meas_min + 1.0f;
                V_DC_bar_upper_meas_min=V_DC_bar_upper_meas_min + 1.0f;
                V_DC_bar_lower_meas_min=V_DC_bar_lower_meas_min + 1.0f;
                I_battery_lower_meas_min=I_battery_lower_meas_min + 1.0f;
                I_battery_upper_meas_min=I_battery_upper_meas_min + 1.0f;
            }
        }

        //****** thyristor turn off delay half cycle ******//
        if (thyr_input_side_turn_off_complete_delay==1) {  // input tristörlerinin devreden çikmasi 10ms sürecegi için.
            if(thyr_deactivate_cnt < THYR_DEACTIVATE_PERIOD) thyr_deactivate_cnt++;
            if(thyr_deactivate_cnt == THYR_DEACTIVATE_PERIOD){
                thyr_input_side_active_fl=0;
                thyr_input_side_turn_off_complete_delay=0;
                thyr_deactivate_cnt=0;
            }
        }

//        //****** battery switch active  monitor *******//
//        if(start_control_syst_PFC_boost==1 && battery_switch_active==0){
//            if(V_Battery_upper > V_BATTERY_MIN_LIM && V_Battery_upper < V_BATTERY_MAX_LIM && battery_voltage_hold_fl == 0){
//                if(battery_voltage_hold_for_switch_control_cnt < CONTROL_BATTERY_SWITCH_CNT_PERIOD) battery_voltage_hold_for_switch_control_cnt++;
//                if(battery_voltage_hold_for_switch_control_cnt == CONTROL_BATTERY_SWITCH_CNT_PERIOD){
//                    battery_voltage_hold_for_switch_control_cnt=0;
//                    Rec_Vdc_Bus_Ref_in = V_Battery_upper + 10.0f;
//                    Rec_Vdc_Bus_Ref_in=(Rec_Vdc_Bus_Ref_in >= V_DC_BAR_NOM ) ? V_DC_BAR_NOM : Rec_Vdc_Bus_Ref_in;
//                    battery_voltage_hold_fl=1;
//                }
//            }
//            if(Rec_I_Aku_P_sum_avg > 0.2f || Rec_I_Aku_N_sum_avg > 0.2f){
//                if(I_battery_cnt_for_battery_switch_control < CONTROL_BATTERY_SWITCH_CNT_PERIOD) I_battery_cnt_for_battery_switch_control++;
//                if(I_battery_cnt_for_battery_switch_control == CONTROL_BATTERY_SWITCH_CNT_PERIOD){
//                    I_battery_cnt_for_battery_switch_control=0;
//                    battery_switch_active=1;
//                    battery_voltage_hold_fl=0;
//                    Rec_Vdc_Bus_Ref_in=V_DC_BAR_NOM;
//                }
//            }
//            else{
//                I_battery_cnt_for_battery_switch_control=0;
//            }
//        }

        //******* V in rms slow range monitor ******//
        if(V_line_rms_range_is_OK == 1){
            if((Vrms_line1_R < V_Line_Slow_Limit_Min ) || (Vrms_line1_S < V_Line_Slow_Limit_Min) || (Vrms_line1_T < V_Line_Slow_Limit_Min )){
                 if(accept_V_line_slow_low_cnt < ACCEPT_V_LINE_RANGE_SLOW_FAULT_PERIOD) accept_V_line_slow_low_cnt++;
                 if(accept_V_line_slow_low_cnt == ACCEPT_V_LINE_RANGE_SLOW_FAULT_PERIOD){
                     V_Line_Slow_Low_Err=1;
                     V_line_rms_range_is_OK=0;
                     V_line_freq_range_is_OK=0;
                     stop_pwm_PFC_boost();
                     start_control_syst_PFC_boost=0;
                     turn_off_input_thyr();
                     thyr_input_side_turn_off_complete_delay=1;
                     rec_data_log_stop_fl=1;
                     ups_error_count=addError(ERR_V_LINE_RST_SLOW_LOW,timer_1sec_for_ups_system,ups_error_count); rec_err_eeprom_wr_active_fl=1;//error
                }
            }else{
                accept_V_line_slow_low_cnt=0;
            }

            if((Vrms_line1_R > V_Line_Slow_Limit_Max ) || (Vrms_line1_S > V_Line_Slow_Limit_Max ) || (Vrms_line1_T > V_Line_Slow_Limit_Max ) ){
                 if(accept_V_line_slow_high_cnt < ACCEPT_V_LINE_RANGE_SLOW_FAULT_PERIOD) accept_V_line_slow_high_cnt++;
                 if(accept_V_line_slow_high_cnt == ACCEPT_V_LINE_RANGE_SLOW_FAULT_PERIOD){
                     V_Line_Slow_High_Err=0;
                     V_line_rms_range_is_OK=0;
                     V_line_freq_range_is_OK=0;
                     stop_pwm_PFC_boost();
                     start_control_syst_PFC_boost=0;
                     turn_off_input_thyr();
                     thyr_input_side_turn_off_complete_delay=1;
                     rec_data_log_stop_fl=1;
                     ups_error_count=addError(ERR_V_LINE_RST_SLOW_HIGH,timer_1sec_for_ups_system,ups_error_count); rec_err_eeprom_wr_active_fl=1;//error
                }
            }else{
                accept_V_line_slow_high_cnt=0;
            }

            //******* input frequency monitor *******//
            if(V_line_freq_range_is_OK == 1){
                if(VAC_line_R_freq > VAC_line_freq_protc_max_lim){
                    VAC_line_freq_min_fault_accept_cnt=0;
                     if(VAC_line_freq_max_fault_accept_cnt < VAC_line_freq_accept_period) VAC_line_freq_max_fault_accept_cnt++;
                     if(VAC_line_freq_max_fault_accept_cnt == VAC_line_freq_accept_period){
                         VAC_line_freq_max_err=1;
                         V_line_freq_range_is_OK=0;
                         stop_pwm_PFC_boost();
                         start_control_syst_PFC_boost=0;
                         turn_off_input_thyr();
                         thyr_input_side_turn_off_complete_delay=1;
                         rec_data_log_stop_fl=1;
                         ups_error_count=addError(ERR_F_LINE_HIGH,timer_1sec_for_ups_system,ups_error_count); rec_err_eeprom_wr_active_fl=1;//error
                    }
                }
                else if(VAC_line_R_freq < VAC_line_freq_protc_min_lim){
                    VAC_line_freq_max_fault_accept_cnt=0;
                     if(VAC_line_freq_min_fault_accept_cnt < VAC_line_freq_accept_period) VAC_line_freq_min_fault_accept_cnt++;
                     if(VAC_line_freq_min_fault_accept_cnt == VAC_line_freq_accept_period){
                         VAC_line_freq_min_err=1;
                         V_line_freq_range_is_OK=0;
                         stop_pwm_PFC_boost();
                         start_control_syst_PFC_boost=0;
                         turn_off_input_thyr();
                         thyr_input_side_turn_off_complete_delay=1;
                         rec_data_log_stop_fl=1;
                         ups_error_count=addError(ERR_F_LINE_LOW,timer_1sec_for_ups_system,ups_error_count); rec_err_eeprom_wr_active_fl=1;//error
                    }
                }
                else{
                    VAC_line_freq_min_fault_accept_cnt=0;
                    VAC_line_freq_max_fault_accept_cnt=0;
                }
            }
            else{
                if (VAC_line_R_freq > FREQ_IN_PROTC_MIN_RETURN_LIM && VAC_line_R_freq < FREQ_IN_PROTC_MAX_RETURN_LIM){
                    if(return_VAC_line_freq_fault_cnt < RETURN_FROM_V_LINE_FREQ_FAULT_PERIOD) return_VAC_line_freq_fault_cnt++;
                    if(return_VAC_line_freq_fault_cnt == RETURN_FROM_V_LINE_FREQ_FAULT_PERIOD){
                        VAC_line_freq_min_err=0;
                        VAC_line_freq_max_err=0;
                        V_line_freq_range_is_OK=1;
                        return_VAC_line_freq_fault_cnt=0;
                        VAC_line_freq_min_fault_accept_cnt=0;
                        VAC_line_freq_max_fault_accept_cnt=0;
                    }
                }
                else{
                    return_VAC_line_freq_fault_cnt=0;
                }
            }
        }
        else{
            if (Vrms_line1_R > VAC_LINE_SLOW_MIN_RETURN_LIM && Vrms_line1_R < VAC_LINE_SLOW_MAX_RETURN_LIM) Vrms_line1_R_is_in_slow_range=1;
            else Vrms_line1_R_is_in_slow_range=0;
            if (Vrms_line1_S > VAC_LINE_SLOW_MIN_RETURN_LIM && Vrms_line1_S < VAC_LINE_SLOW_MAX_RETURN_LIM) Vrms_line1_S_is_in_slow_range=1;
            else Vrms_line1_S_is_in_slow_range=0;
            if (Vrms_line1_T > VAC_LINE_SLOW_MIN_RETURN_LIM && Vrms_line1_T < VAC_LINE_SLOW_MAX_RETURN_LIM) Vrms_line1_T_is_in_slow_range=1;
            else Vrms_line1_T_is_in_slow_range=0;

            if (Vrms_line1_R_is_in_slow_range==1 && Vrms_line1_S_is_in_slow_range==1 && Vrms_line1_T_is_in_slow_range==1) { // Vrms_line1_T 1 periodluk rms
               if(return_from_V_line_slow_err_cnt < RETURN_FROM_V_LINE_SLOW_FAULT_PERIOD) return_from_V_line_slow_err_cnt++;
               if(return_from_V_line_slow_err_cnt == RETURN_FROM_V_LINE_SLOW_FAULT_PERIOD){
                    V_line_rms_range_is_OK=1;
                    V_Line_Slow_Low_Err=0;
                    V_line_fast_Low_Err=0;
                    V_Line_Slow_High_Err=0;
                    V_Line_fast_high_Err=0;
                    return_from_V_line_slow_err_cnt=0;
                    accept_V_line_slow_low_cnt=0;
                    accept_V_line_slow_high_cnt=0;
               }
            }
            else{
                return_from_V_line_slow_err_cnt=0;
            }
        }

        if(V_line_rms_range_is_OK == 1 && V_line_freq_range_is_OK == 1){
            V_line_range_is_OK=1;
        }else{
            V_line_range_is_OK=0;
            Rec_phase_squence_correct=0;
            Rec_phase_squence_correct_err=0;
        }

        //********** we will try to start the system from  fault (over current 30u, dc bar ov fast vb...) ********//
        // eğer  sistem servisten kapatılmış ise rectifier request buradan değil servisten yapılmalı
        if(cap_charge_request == 0 && thyr_input_side_active_fl == 0 && rec_system_state_control_fl_from_servise == 0 && Rec_phase_squence_correct == 1){
            if (rec_over_load_100_125_fault_try_return==1 && rec_over_load_100_125_fault_persists == 0) {
                if(return_rec_over_load_100_125_fault_cnt < return_I_in_rms_100_125_fault_period) return_rec_over_load_100_125_fault_cnt++;
                if(return_rec_over_load_100_125_fault_cnt == return_I_in_rms_100_125_fault_period){
                   if (V_line_range_is_OK==1) {
                       cap_charge_request=1;
                       accept_rec_over_load_100_125_fault_return_cnt=0;
                   }
                }
            }
            else if (rec_over_load_125_150_fault_try_return==1 && rec_over_load_125_150_fault_persists == 0) {
                if(return_rec_over_load_125_150_fault_cnt < return_I_in_rms_125_150_fault_period) return_rec_over_load_125_150_fault_cnt++;
                if(return_rec_over_load_125_150_fault_cnt == return_I_in_rms_125_150_fault_period){
                   if (V_line_range_is_OK==1) {
                       cap_charge_request=1;
                       accept_rec_over_load_125_150_fault_return_cnt=0;
                   }
                }
            }
            else if(I_battery_charge_err_try_return==1 && I_battery_charge_fault_persists==0 ){
                if(return_I_battery_charge_fault_cnt < return_from_I_battery_charge_fault_period) return_I_battery_charge_fault_cnt++;
                if(return_I_battery_charge_fault_cnt == return_from_I_battery_charge_fault_period){ // 10 sec
                   if (V_line_range_is_OK==1) {
                       cap_charge_request=1;
                       critical_fault_reset_delay_cnt=0;
                   }
                }
            }
            else if (rec_igbt_fault_try_return==1  && rec_igbt_R_fault_persists == 0 && rec_igbt_S_fault_persists == 0 && rec_igbt_T_fault_persists == 0 ) {
                if(return_rec_igbt_fault_fault_cnt < RETURN_FROM_IGBT_FAULT_PERIOD) return_rec_igbt_fault_fault_cnt++; // 10 saniyede 1 tekrar dene
                if(return_rec_igbt_fault_fault_cnt == RETURN_FROM_IGBT_FAULT_PERIOD){
                    if (V_line_range_is_OK==1) {
                        cap_charge_request=1;
                        accept_rec_igbt_fault_fault_return_cnt=0;
                    }
                }
            }
            else if (I_in_30u_fault_try_return==1 && I_in_30u_fault_persists == 0) {
                if(return_I_in_30u_fault_cnt < return_from_I_in_30u_fault_period) return_I_in_30u_fault_cnt++; // 10 saniyede 1 tekrar dene
                if(return_I_in_30u_fault_cnt == return_from_I_in_30u_fault_period){
                   if (V_line_range_is_OK==1) {
                       cap_charge_request=1;
                       critical_fault_reset_delay_cnt=0;
                   }
                }
            }
            else if (V_DC_bar_fast_err_try_return==1 && DC_bar_fast_fault_persists == 0 ) {
                if(return_DC_bar_fast_fault_cnt < return_from__DC_bar_fast_fault_period) return_DC_bar_fast_fault_cnt++;
                if(return_DC_bar_fast_fault_cnt == return_from__DC_bar_fast_fault_period){ // 10 sec
                   if (V_line_range_is_OK==1) {
                       cap_charge_request=1;
                       critical_fault_reset_delay_cnt=0;
                   }
                }
            }

            //******* request cap charge because of ac line fault *******//
            if(cap_charge_request==0){
                if (I_in_R_no_load_cur_fault_fl==0 && I_in_S_no_load_cur_fault_fl==0 && I_in_T_no_load_cur_fault_fl==0 ) {
                    if (rec_over_load_100_125_fault_try_return==0) {
                        if (rec_over_load_125_150_fault_try_return==0) {
                            if(rec_igbt_fault_try_return == 0){
                                if(I_upper_battery_charge_err ==0 && I_lower_battery_charge_err==0){
                                    if (I_in_R_30u_fault==0 && I_in_S_30u_fault==0 && I_in_T_30u_fault==0) {
                                        if (V_DC_bar_upper_fast_high_err==0 && V_DC_bar_lower_fast_high_err==0 && V_DC_bar_lower_fast_low_err==0 && V_DC_bar_upper_fast_low_err==0) {
                                            if (V_line_range_is_OK==1) {
                                                if(cap_charge_request_delay_cnt < CAP_CHARGE_REQUEST_DELAY_PERIOD) cap_charge_request_delay_cnt++;
                                                if(cap_charge_request_delay_cnt == CAP_CHARGE_REQUEST_DELAY_PERIOD){
                                                    cap_charge_request=1;   cap_charge_request_deb_1++;
                                                    cap_charge_request_delay_cnt=0;
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        //******* start control pfc boost system *********//
        if(start_control_syst_PFC_boost==0){
            if (thyr_input_side_active_fl==1 && thyr_input_side_turn_off_complete_delay==0 ){    // cap charge completed
                if (I_in_R_no_load_cur_fault_fl==0 && I_in_S_no_load_cur_fault_fl==0 && I_in_T_no_load_cur_fault_fl==0) {
                     if (V_DC_bar_upper_meas > V_DC_BAR_START_LOW && V_DC_bar_upper_meas < V_DC_BAR_START_HIGH && V_DC_bar_lower_meas > V_DC_BAR_START_LOW  && V_DC_bar_lower_meas < V_DC_BAR_START_HIGH) {
                        if(pfc_boost_start_delay_cnt < PFC_BOOST_START_DELAY_PERIOD) pfc_boost_start_delay_cnt++;
                        if(pfc_boost_start_delay_cnt == PFC_BOOST_START_DELAY_PERIOD) { // 500ms
                            pfc_boost_start_delay_cnt=0;
                            if (PFC_boost_first_ref_set==0) {   // ref value is set only once // this code line may not be needed
                                Rec_Vdc_PM_Ref=V_DC_bar_upper_avg_20ms*1.025f;
                                Rec_Vdc_MN_Ref=V_DC_bar_lower_avg_20ms*1.025f;
                                PFC_boost_first_ref_set=1;
                            }
                            start_control_syst_PFC_boost=1;
                        }
                    }
                }
            }
            else {
                pfc_boost_start_delay_cnt=0;
            }
        }

        // if control system  active and line current exist ,  for 2sec , system is stable
        if (start_control_syst_PFC_boost==1 && pfc_boost_stable_fl==0 && Irms_Line1_R > I_in_no_load_cur_lim_return && Irms_Line1_S > I_in_no_load_cur_lim_return && Irms_Line1_T > I_in_no_load_cur_lim_return) {
            if(pfc_boost_stable_accept_cnt < PFC_BOOST_STABLE_ACCEPT_PERIOD) pfc_boost_stable_accept_cnt++;
            if(pfc_boost_stable_accept_cnt == PFC_BOOST_STABLE_ACCEPT_PERIOD) { // 2s
                pfc_boost_stable_fl=1;
                pfc_boost_stable_accept_cnt=0;
            }
        }
        else if (start_control_syst_PFC_boost==0) {
            pfc_boost_stable_fl=0; pfc_boost_stable_accept_cnt=0;
        }


        //********** battery charge over current  -> turn off system *************//
        // 02.01.2025 NOT: Akü switch aktif şartı buraya eklenmeli
        if(I_battery_charge_err_try_return == 0 && start_control_syst_PFC_boost == 1){
            if(I_battery_lower_meas > I_battery_charge_lim){
                 if(I_lower_battery_charge_fault_accept_cnt < I_battery_charge_accept_period) I_lower_battery_charge_fault_accept_cnt++;
                 if(I_lower_battery_charge_fault_accept_cnt == I_battery_charge_accept_period){
                     I_lower_battery_charge_err=1; I_battery_charge_err_try_return=1; I_battery_charge_fault_repeat++;
                     stop_pwm_PFC_boost();
                     start_control_syst_PFC_boost=0;
                     turn_off_input_thyr();
                     thyr_input_side_turn_off_complete_delay=1;
                     rec_data_log_stop_fl=1;
                     return_I_battery_charge_fault_cnt=0;
                     I_lower_battery_charge_fault_accept_cnt=0;
                     I_upper_battery_charge_fault_accept_cnt=0;
                     if (I_battery_charge_fault_repeat > 3) {
                         I_battery_charge_fault_persists=1;
                     }
                     ups_error_count=addError(ERR_I_LOWER_BATTERY_CHARGE,timer_1sec_for_ups_system,ups_error_count); rec_err_eeprom_wr_active_fl=1; //error
                 }
            }
            else{
                if(I_lower_battery_charge_fault_accept_cnt > 0) I_lower_battery_charge_fault_accept_cnt--;
            }

            if(I_battery_upper_meas > I_battery_charge_lim){
                 if(I_upper_battery_charge_fault_accept_cnt < I_battery_charge_accept_period) I_upper_battery_charge_fault_accept_cnt++;
                 if(I_upper_battery_charge_fault_accept_cnt == I_battery_charge_accept_period){
                     I_upper_battery_charge_err=1; I_battery_charge_err_try_return=1;  I_battery_charge_fault_repeat++;
                     stop_pwm_PFC_boost();
                     start_control_syst_PFC_boost=0;
                     turn_off_input_thyr();
                     thyr_input_side_turn_off_complete_delay=1;
                     rec_data_log_stop_fl=1;
                     return_I_battery_charge_fault_cnt=0;
                     I_upper_battery_charge_fault_accept_cnt=0;
                     I_lower_battery_charge_fault_accept_cnt=0;
                     if (I_battery_charge_fault_repeat > 3) {
                         I_battery_charge_fault_persists=1;
                     }
                     ups_error_count=addError(ERR_I_UPPER_BATTERY_CHARGE,timer_1sec_for_ups_system,ups_error_count); rec_err_eeprom_wr_active_fl=1; //error
                 }
            }
            else{
                if(I_upper_battery_charge_fault_accept_cnt > 0) I_upper_battery_charge_fault_accept_cnt--;
            }
        }

        //****** 100 125 ovc -> turn off system ******//
        //31.12.2024 19.06 NOT:  görünür güç sistem dahil edildi güç faktörü 1 olduğu için aktif güç kullanılmadı
       if(rec_over_load_100_125_fault_try_return == 0 && thyr_input_side_active_fl == 1){

           if(Sin_R_meas > S_in_100_lim || Sin_S_meas > S_in_100_lim || Sin_T_meas > S_in_100_lim){
               if(rec_over_load_100_125_accept_cnt < rec_over_load_100_125_accept_period) rec_over_load_100_125_accept_cnt++;
               if(rec_over_load_100_125_accept_cnt == rec_over_load_100_125_accept_period){
                   rec_over_load_100_125_err=1;  rec_over_load_100_125_fault_try_return=1; rec_over_load_100_125_fault_repeat++; return_rec_over_load_100_125_fault_cnt=0;
                   stop_pwm_PFC_boost();
                   start_control_syst_PFC_boost=0;
                   turn_off_input_thyr();
                   thyr_input_side_turn_off_complete_delay=1;
                   rec_data_log_stop_fl=1;
                   rec_over_load_100_125_accept_cnt=0;
                   if (rec_over_load_100_125_fault_repeat > 3) {
                       rec_over_load_100_125_fault_persists=1;
                     }
                   ups_error_count=addError(ERR_I_IN_RMS_100_125,timer_1sec_for_ups_system,ups_error_count);  rec_err_eeprom_wr_active_fl=1; //error
               }
           }
           else if(Irms_Line1_R > I_in_rms_100_lim || Irms_Line1_S > I_in_rms_100_lim || Irms_Line1_T > I_in_rms_100_lim){
                 if(rec_over_load_100_125_accept_cnt < rec_over_load_100_125_accept_period) rec_over_load_100_125_accept_cnt++;
                 if(rec_over_load_100_125_accept_cnt == rec_over_load_100_125_accept_period){
                     rec_over_load_100_125_err=1;  rec_over_load_100_125_fault_try_return=1; rec_over_load_100_125_fault_repeat++; return_rec_over_load_100_125_fault_cnt=0;
                     stop_pwm_PFC_boost();
                     start_control_syst_PFC_boost=0;
                     turn_off_input_thyr();
                     thyr_input_side_turn_off_complete_delay=1;
                     rec_data_log_stop_fl=1;
                     rec_over_load_100_125_accept_cnt=0;
                     if (rec_over_load_100_125_fault_repeat > 3) {
                         rec_over_load_100_125_fault_persists=1;
                       }
                     ups_error_count=addError(ERR_I_IN_RMS_100_125,timer_1sec_for_ups_system,ups_error_count);  rec_err_eeprom_wr_active_fl=1; //error
                 }
            }
            else{
                if(rec_over_load_100_125_accept_cnt > 0) rec_over_load_100_125_accept_cnt--;
            }
       }
       else{
           if(rec_over_load_100_125_accept_cnt > 0) rec_over_load_100_125_accept_cnt--;
       }

        //********* 125 150 ovc -> turn off system *********//
       if(rec_over_load_125_150_fault_try_return == 0 && thyr_input_side_active_fl == 1){

           if(Sin_R_meas > S_in_125_lim || Sin_S_meas > S_in_125_lim || Sin_T_meas > S_in_125_lim){
               if(rec_over_load_125_150_accept_cnt < rec_over_load_125_150_accept_period) rec_over_load_125_150_accept_cnt++;
               if(rec_over_load_125_150_accept_cnt == rec_over_load_125_150_accept_period){
                   rec_over_load_125_150_err=1; rec_over_load_125_150_fault_try_return=1; rec_over_load_125_150_fault_repeat++;  return_rec_over_load_125_150_fault_cnt=0;
                   stop_pwm_PFC_boost();
                   start_control_syst_PFC_boost=0;
                   turn_off_input_thyr();
                   thyr_input_side_turn_off_complete_delay=1;
                   rec_data_log_stop_fl=1;
                   rec_over_load_125_150_accept_cnt=0;
                   if (rec_over_load_125_150_fault_repeat > 3) {
                       rec_over_load_125_150_fault_persists=1;
                     }
                   ups_error_count=addError(ERR_I_IN_RMS_125_150,timer_1sec_for_ups_system,ups_error_count); rec_err_eeprom_wr_active_fl=1; //error
               }
           }
           else if(Irms_Line1_R > I_in_rms_125_lim || Irms_Line1_S > I_in_rms_125_lim || Irms_Line1_T > I_in_rms_125_lim){
                if(rec_over_load_125_150_accept_cnt < rec_over_load_125_150_accept_period) rec_over_load_125_150_accept_cnt++;
                if(rec_over_load_125_150_accept_cnt == rec_over_load_125_150_accept_period){
                    rec_over_load_125_150_err=1; rec_over_load_125_150_fault_try_return=1; rec_over_load_125_150_fault_repeat++;  return_rec_over_load_125_150_fault_cnt=0;
                    stop_pwm_PFC_boost();
                    start_control_syst_PFC_boost=0;
                    turn_off_input_thyr();
                    thyr_input_side_turn_off_complete_delay=1;
                    rec_data_log_stop_fl=1;
                    rec_over_load_125_150_accept_cnt=0;
                    if (rec_over_load_125_150_fault_repeat > 3) {
                        rec_over_load_125_150_fault_persists=1;
                      }
                    ups_error_count=addError(ERR_I_IN_RMS_125_150,timer_1sec_for_ups_system,ups_error_count); rec_err_eeprom_wr_active_fl=1; //error
                }
            }
            else{
                if(rec_over_load_125_150_accept_cnt > 0) rec_over_load_125_150_accept_cnt--;
            }
       }
       else{
           if(rec_over_load_125_150_accept_cnt > 0) rec_over_load_125_150_accept_cnt--;
       }


   //****** kritik arızadan sonra sistem çalışmaya başladı ve 1 saniye geçti. flagları gecikmesiz sıfırlarsam cap charge complete flagları 1 olmadan diğer cap charge fonksiyonuna girer. bu yüzden flagları gecikmeli sıfırla.
   //NOT: tekrar sistem çalışıp 1 saniye geçtiği için sistemimi çalışıyor olarak düşünüyorum. kodları sıfırlıyorum. eğer tekrar arıza olursa kodlar tekrar set edilebilir sorun yok.
       if (start_control_syst_PFC_boost==1) {
            if(critical_fault_reset_delay_cnt < FAULT_RESET_DELAY_CNT_PERIOD) critical_fault_reset_delay_cnt++;
            if(critical_fault_reset_delay_cnt == FAULT_RESET_DELAY_CNT_PERIOD) { // 1sec
                I_in_R_30u_fault=0;
                I_in_S_30u_fault=0;
                I_in_T_30u_fault=0;
                V_DC_bar_upper_fast_high_err=0;
                V_DC_bar_lower_fast_high_err=0;
                V_DC_bar_lower_fast_low_err=0;
                V_DC_bar_upper_fast_low_err=0;
                I_upper_battery_charge_err=0;
                I_lower_battery_charge_err=0;
            }
        }

        //****herhangi bir hata olduktan sonra, hatanın gürültümü yoksa kalıcı bir hata mı olduğunu anlamak için
        // tekrar sayaçları ve hatanın sıfırlanmasının kontrol edilmesi gerekiyor.
        if(rec_over_load_100_125_err==1 && rec_over_load_100_125_fault_try_return == 0){
            if(accept_rec_over_load_100_125_fault_return_cnt < REC_OVER_LOAD_100_125_FAULT_RETURN_PERIOD) accept_rec_over_load_100_125_fault_return_cnt++;
            if(accept_rec_over_load_100_125_fault_return_cnt == REC_OVER_LOAD_100_125_FAULT_RETURN_PERIOD){
                rec_over_load_100_125_err=0;
                rec_over_load_100_125_fault_repeat=0;
            }
        }
        if(rec_over_load_125_150_err==1 && rec_over_load_125_150_fault_try_return == 0){
            if(accept_rec_over_load_125_150_fault_return_cnt < REC_OVER_LOAD_125_150_FAULT_RETURN_PERIOD) accept_rec_over_load_125_150_fault_return_cnt++;
            if(accept_rec_over_load_125_150_fault_return_cnt == REC_OVER_LOAD_125_150_FAULT_RETURN_PERIOD){
                rec_over_load_125_150_err=0;
                rec_over_load_125_150_fault_repeat=0;
            }
        }
        if((rec_igbt_R_err==1 ||rec_igbt_S_err==1 ||rec_igbt_T_err==1) && rec_igbt_fault_try_return == 0){
            if(accept_rec_igbt_fault_fault_return_cnt < ACCEPT_FROM_IGBT_FAULT_PERIOD) accept_rec_igbt_fault_fault_return_cnt++;
            if(accept_rec_igbt_fault_fault_return_cnt == ACCEPT_FROM_IGBT_FAULT_PERIOD){
                rec_igbt_R_err=0;  rec_igbt_R_fault_repeat=0;
                rec_igbt_S_err=0;  rec_igbt_S_fault_repeat=0;
                rec_igbt_T_err=0;  rec_igbt_T_fault_repeat=0;
            }
        }

        /////////// fault olduktan sonra eğer 12 saniye geçerse ancak fault repeat sıfırlanıyor Bundan önce tekrar 30u fault olursa repeated fault oarak kabul ediliyor.
        if (start_control_syst_PFC_boost==1 ) {
            if(accept_critical_fault_return_cnt < 12000) accept_critical_fault_return_cnt++;
            if(accept_critical_fault_return_cnt == 12000){
                I_in_30u_fault_recovered_from_repeat=I_in_30u_fault_repeat; I_in_30u_fault_repeat=0;  // repeat ler sonrasında sistem düzelirse repeat sayısını bir variable da sakla.
                DC_bar_fast_fault_recovered_from_repeat=DC_bar_fast_fault_repeat; DC_bar_fast_fault_repeat=0;  // repeat ler sonrasında sistem düzelirse repeat sayısını bir variable da sakla.
                I_battery_charge_fault_recovered_from_repeat= I_battery_charge_fault_repeat;   I_battery_charge_fault_repeat=0;
            }
        }else {
         accept_critical_fault_return_cnt=0;
        }
    } //end of timer0 1ms period function
}//end of while loop


//=============================================================================
// INSTRUMENTATION_ISR - CPUTIMER2_ISR
// 20Khz ISR Code -- 50 us loop
// Timer 0  are used for background tasks, non timing critical
// Timer 2 is used for instrumentation ISR
//=============================================================================
__interrupt void INSTRUMENTATION_ISR(){
    IER &= MINT14;
    EINT;
//ISR Code here

    Rectifier_instrumentationCode();
    SaveResponseBufferToMenuValue();
    handle_cap_charge_fnc();

    // ups hata durumlarının buffer kayıt işlemi 32 bit 16 bit olarak ayrılıyor
    if(ups_error_count_index != ups_error_count_index_from_front_panel && modbus_fnc_code == MODBUS_UPS_DATA_CODE){
//        if(ups_err_data_full < ups_error_count_index){
//            ups_err_data_full=ups_error_count_index;
//        }
        err_response_buffer[0]=ups_error_count_index;
        err_response_buffer[1]=ups_err_data_full;
        for (ups_error_vector_index = 0; ups_error_vector_index < ups_err_data_full; ups_error_vector_index++){
            err_response_buffer[ups_err_index_for_respose_save] = (uint16_t)(ups_error_vector[ups_error_vector_index] & 0xFFFF);        // Düşük 16 bit
            err_response_buffer[ups_err_index_for_respose_save + 1] = (uint16_t)(ups_error_vector[ups_error_vector_index] >> 16) & 0xFFFF;  // Yüksek 16 bit
            ups_err_index_for_respose_save+=2;
        }
        ups_err_index_for_respose_save=2;
        modbus_fnc_code=(uint8_t)MODBUS_ERROR_DATA_CODE;
        modbus_com_data_count=(uint16_t)((uint16_t)(ups_err_data_full * 4U) + 4U);
    }

    //************************************************
    // NOT1:hata olduktan sonra 5 cycle daha alınmalı toplam 10 cycle verileri incelemek için yeterli 100us örnekleme yeterli
    // NOT2:rec_data_log_meas_active_fl bu değişken ölçüm yapmak için tekrardan sistemin başlandığını belirlemek için
    // NOT3: rec_data_log_eeprom_wr_active_fl ups rec data loglarının eeproma yazılması gerektiğini belirleyen ve ayrıca sistem tekrar başlasa bile eğer hala eeproma
    // yazma işlemi devam ediyorsa bu değişken yeni verilerin tutulmasını engellemek için kullanılacaktır.
    //*************************************************
    if(rec_data_log_stop_fl == 1 && rec_data_log_meas_active_fl == 0){
        rec_data_log_last_5_cycle_active_fl=1;
    }
    if(rec_data_log_last_5_cycle_active_fl == 1){
        rec_data_log_last_5_cycle_cnt++;
        if(rec_data_log_last_5_cycle_cnt == 2000){
            rec_data_log_last_5_cycle_cnt=0;
            rec_data_log_last_5_cycle_active_fl=0;
            rec_data_log_meas_active_fl=1;
            rec_data_log_stop_fl=0;
            rec_data_log_eeprom_wr_active_fl=1;
        }
    }
    if(rec_data_log_meas_active_fl == 0 && rec_data_log_eeprom_wr_active_fl == 0){
        input_meas_values_100us_cnt++;
        if(input_meas_values_100us_cnt == 2){
            input_meas_values_100us_cnt=0;
            I_R_in_meas_values[AC_in_meas_values_index]= (int16_t)I_R_in_meas*10;
            I_S_in_meas_values[AC_in_meas_values_index]= (int16_t)I_S_in_meas*10;
            I_T_in_meas_values[AC_in_meas_values_index]= (int16_t)I_T_in_meas*10;
            V_R_in_meas_values[AC_in_meas_values_index]= (int16_t)V_R_in_meas*10;
            V_S_in_meas_values[AC_in_meas_values_index]= (int16_t)V_S_in_meas*10;
            V_T_in_meas_values[AC_in_meas_values_index]= (int16_t)V_T_in_meas*10;
            V_DC_bar_upper_meas_values[DC_meas_values_index]= (int16_t)V_DC_bar_upper_meas*10;
            V_DC_bar_lower_meas_values[DC_meas_values_index]= (int16_t)V_DC_bar_lower_meas*10;
            I_battery_upper_meas_values[DC_meas_values_index]= (int16_t)I_battery_upper_meas*10;
            I_battery_lower_meas_values[DC_meas_values_index]= (int16_t)I_battery_lower_meas*10;

            AC_in_meas_values_index = (AC_in_meas_values_index + 1) % REC_AC_MEAS_VECTOR_SIZE;
            DC_meas_values_index = (DC_meas_values_index + 1) % REC_DC_MEAS_VECTOR_SIZE;
        }
    }

    if(rec_data_log_stop_fl == 0 && rec_data_log_meas_active_fl == 0 && rec_data_log_eeprom_wr_active_fl == 0){

        //****** UPS SYSTEM MAX VOLT AND CURRENT VALUE MONITOR ****//
        if(V_DC_bar_lower_meas > V_DC_bar_lower_meas_max) V_DC_bar_lower_meas_max = V_DC_bar_lower_meas;
        if(V_DC_bar_upper_meas > V_DC_bar_upper_meas_max) V_DC_bar_upper_meas_max = V_DC_bar_upper_meas;
        if(I_battery_lower_meas > I_battery_lower_meas_max) I_battery_lower_meas_max = I_battery_lower_meas;
        if(I_battery_upper_meas > I_battery_upper_meas_max) I_battery_upper_meas_max = I_battery_upper_meas;
        if(V_R_in_meas > V_R_in_meas_max) V_R_in_meas_max = V_R_in_meas;
        if(V_S_in_meas > V_S_in_meas_max) V_S_in_meas_max = V_S_in_meas;
        if(V_T_in_meas > V_T_in_meas_max) V_T_in_meas_max = V_T_in_meas;
        if(I_R_in_meas > I_R_in_meas_max) I_R_in_meas_max = I_R_in_meas;
        if(I_S_in_meas > I_S_in_meas_max) I_S_in_meas_max = I_S_in_meas;
        if(I_T_in_meas > I_T_in_meas_max) I_T_in_meas_max = I_T_in_meas;

        if(V_DC_bar_lower_meas < V_DC_bar_lower_meas_min) V_DC_bar_lower_meas_min = V_DC_bar_lower_meas;
        if(V_DC_bar_upper_meas < V_DC_bar_upper_meas_min) V_DC_bar_upper_meas_min = V_DC_bar_upper_meas;
        if(I_battery_lower_meas < I_battery_lower_meas_min) I_battery_lower_meas_min = I_battery_lower_meas;
        if(I_battery_upper_meas < I_battery_upper_meas_min) I_battery_upper_meas_min = I_battery_upper_meas;
        if(V_R_in_meas < V_R_in_meas_min) V_R_in_meas_min = V_R_in_meas;
        if(V_S_in_meas < V_S_in_meas_min) V_S_in_meas_min = V_S_in_meas;
        if(V_T_in_meas < V_T_in_meas_min) V_T_in_meas_min = V_T_in_meas;
        if(I_R_in_meas < I_R_in_meas_min) I_R_in_meas_min = I_R_in_meas;
        if(I_S_in_meas < I_S_in_meas_min) I_S_in_meas_min = I_S_in_meas;
        if(I_T_in_meas < I_T_in_meas_min) I_T_in_meas_min = I_T_in_meas;
    }

    // bu ortolamalar sadece ön panelde gösterim için kullanılmalı
     V_DC_bar_upper_avg_sum =V_DC_bar_upper_meas+V_DC_bar_upper_avg_sum;
     V_DC_bar_lower_avg_sum =V_DC_bar_lower_meas+V_DC_bar_lower_avg_sum;
     Rec_I_Aku_P_sum_avg_sum =I_battery_upper_meas+Rec_I_Aku_P_sum_avg_sum;
     Rec_I_Aku_N_sum_avg_sum =I_battery_lower_meas+Rec_I_Aku_N_sum_avg_sum;
     rec_vbus_sum_index++;
     if(rec_vbus_sum_index >= V_DC_BAR_AVG_20MS_SAMPLE_COUNT){
         V_DC_bar_upper_avg_20ms=(float32_t)(V_DC_bar_upper_avg_sum/rec_vbus_sum_index);
         V_DC_bar_lower_avg_20ms=(float32_t)(V_DC_bar_lower_avg_sum/rec_vbus_sum_index);
         Rec_I_Aku_P_sum_avg=(float32_t)(Rec_I_Aku_P_sum_avg_sum/rec_vbus_sum_index);
         Rec_I_Aku_N_sum_avg=(float32_t)(Rec_I_Aku_N_sum_avg_sum/rec_vbus_sum_index);
         rec_vbus_sum_index=0;
         V_DC_bar_lower_avg_sum=0;
         V_DC_bar_upper_avg_sum=0;
         Rec_I_Aku_P_sum_avg_sum=0;
         Rec_I_Aku_N_sum_avg_sum=0;
     }

        //******* V in rms fast range monitor********
    if(V_line_rms_range_is_OK == 1){
        if ((Vrms_line1_R < V_Line_Fast_Limit_Min) || (Vrms_line1_S < V_Line_Fast_Limit_Min) || (Vrms_line1_T < V_Line_Fast_Limit_Min)){
            stop_pwm_PFC_boost();
            start_control_syst_PFC_boost=0;
            turn_off_input_thyr();
            thyr_input_side_turn_off_complete_delay=1;
            rec_data_log_stop_fl=1;
            V_line_fast_Low_Err=1;
            V_line_rms_range_is_OK=0;
            V_line_freq_range_is_OK=0;
            ups_error_count=addError(ERR_V_LINE_RST_FAST_LOW,timer_1sec_for_ups_system,ups_error_count);  rec_err_eeprom_wr_active_fl=1;//error
        }
        else if ((Vrms_line1_R > V_Line_Fast_Limit_Max) || (Vrms_line1_S > V_Line_Fast_Limit_Max) || (Vrms_line1_T > V_Line_Fast_Limit_Max)){
            stop_pwm_PFC_boost();
            start_control_syst_PFC_boost=0;
            turn_off_input_thyr();
            thyr_input_side_turn_off_complete_delay=1;
            rec_data_log_stop_fl=1;
            V_Line_fast_high_Err=1;
            V_line_rms_range_is_OK=0;
            V_line_freq_range_is_OK=0;
            ups_error_count=addError(ERR_V_LINE_RST_FAST_HIGH,timer_1sec_for_ups_system,ups_error_count);  rec_err_eeprom_wr_active_fl=1; //error
        }
    }

    //***************************** I in no load current monitor **************************//
    if(Irms_Line1_R < I_in_no_load_cur_lim && pfc_boost_stable_fl==1 && start_control_syst_PFC_boost == 1){  // bu durum gerçekleşirse cihaz tekrar devreye girmeye çalışmasın. teknik servis kontrolü yapılmalı. tristör sürülemiyor olabilir.
        I_in_R_no_load_cur_fault_fl=1;
        stop_pwm_PFC_boost();
        start_control_syst_PFC_boost=0;
        turn_off_input_thyr();
        thyr_input_side_turn_off_complete_delay=1;
        rec_data_log_stop_fl=1;
        ups_error_count=addError(ERR_I_IN_R_NO_CUR,timer_1sec_for_ups_system,ups_error_count);  rec_err_eeprom_wr_active_fl=1;//error
    }

    if(Irms_Line1_S < I_in_no_load_cur_lim && pfc_boost_stable_fl==1 && start_control_syst_PFC_boost == 1){
        I_in_S_no_load_cur_fault_fl=1;
        stop_pwm_PFC_boost();
        start_control_syst_PFC_boost=0;
        turn_off_input_thyr();
        thyr_input_side_turn_off_complete_delay=1;
        rec_data_log_stop_fl=1;
        ups_error_count=addError(ERR_I_IN_S_NO_CUR,timer_1sec_for_ups_system,ups_error_count);  rec_err_eeprom_wr_active_fl=1;//error
    }

    if(Irms_Line1_T < I_in_no_load_cur_lim && pfc_boost_stable_fl==1 && start_control_syst_PFC_boost == 1){
        I_in_T_no_load_cur_fault_fl=1;
        stop_pwm_PFC_boost();
        start_control_syst_PFC_boost=0;
        turn_off_input_thyr();
        thyr_input_side_turn_off_complete_delay=1;
        rec_data_log_stop_fl=1;
        ups_error_count=addError(ERR_I_IN_T_NO_CUR,timer_1sec_for_ups_system,ups_error_count); rec_err_eeprom_wr_active_fl=1; //error
    }

    uart_timeout_counter++;
    uart_timeout_counter=(uart_timeout_counter > 64000) ? 64000 : uart_timeout_counter;


//end of ISR Code
    DINT;
    traceISR[traceISRIndex % TRACE_SIZE] = 2;
    traceISRIndex++;
}

//=============================================================================
// CPU1_loop_fnc
// Timer counts FOR SLOW BACKGROUND TASKS
// "rec_R_sin_hold_time_cnt" system counter for timer to use author function
// NOT1="handle_cap_charge_fnc();" SCR soft start arka plan işlemlerine alınacak
// son ölçülen loop süresi 7.3 us. 16.10.24 16:28
// son ölçülen loop süresi 5.5 us. 24.10.24 15:19 // DC BARA MOVING AVERAGE 50us KESMESINE ALINDI
//=============================================================================
__interrupt void CPU1_loop_fnc(void) {

    volatile uint16_t tempPIEIER = HWREGH(PIECTRL_BASE + PIE_O_IER3);
    IER |= M_INT3;
    IER &= MINT3;
    HWREGH(PIECTRL_BASE + PIE_O_IER3) &= MG3_5;
    Interrupt_clearACKGroup(0xFFFFU);
    __asm("  NOP");
    EINT;
    // ISR Code here

    REC_readCurrVolADCSignals();    // main ADC readings
    writeDataCPU1();
    readDataCPU1();
    rec_R_sin_hold_time_cnt++;      // pozitifte iken kaç tane örnek yapıldı, negatifte kaç tane

    if(V_DC_bar_upper_avg_20ms > V_DC_bar_upper_fast_under_volt_protc_actv_thresh && start_control_syst_PFC_boost==1 ) {
        V_DC_bar_upper_fast_under_volt_protc_actv=1;
    }
    else if(start_control_syst_PFC_boost == 0 && V_DC_bar_upper_fast_under_volt_protc_actv ==1) {
        V_DC_bar_upper_fast_under_volt_protc_actv=0;
    }

    if(V_DC_bar_lower_avg_20ms > V_DC_bar_lower_fast_under_volt_protc_actv_thresh && start_control_syst_PFC_boost==1 ) {
        V_DC_bar_lower_fast_under_volt_protc_actv=1;
    }
    else if(start_control_syst_PFC_boost == 0 && V_DC_bar_lower_fast_under_volt_protc_actv == 1 ) {
        V_DC_bar_lower_fast_under_volt_protc_actv=0;
    }

    ///////////////////// DC bar fast high-low koruma
    if(V_DC_bar_fast_err_try_return == 0){
        if(V_DC_bar_upper_meas > V_DC_bar_fast_limit_max ){
            V_DC_bar_upper_fast_low_accept_cnt=0;
            if(V_DC_bar_upper_fast_high_accept_cnt < V_DC_BAR_FAST_HIGH_ACCEPT_PERIOD) V_DC_bar_upper_fast_high_accept_cnt++;
            if(V_DC_bar_upper_fast_high_accept_cnt == V_DC_BAR_FAST_HIGH_ACCEPT_PERIOD){ // 100us
                V_DC_bar_upper_fast_high_err=1; V_DC_bar_fast_err_try_return=1; DC_bar_fast_fault_repeat++; return_DC_bar_fast_fault_cnt=0;
                stop_pwm_PFC_boost();
                start_control_syst_PFC_boost=0;
                turn_off_input_thyr();
                thyr_input_side_turn_off_complete_delay=1;
                rec_data_log_stop_fl=1;
                V_DC_bar_upper_fast_high_accept_cnt=0;
                V_DC_bar_upper_fast_low_accept_cnt=0;
                V_DC_bar_lower_fast_low_accept_cnt=0;
                V_DC_bar_lower_fast_high_accept_cnt=0;
                if (DC_bar_fast_fault_repeat > 3) {
                    DC_bar_fast_fault_persists=1;
                }
                ups_error_count=addError(ERR_V_DC_BAR_UPPER_HIGH,timer_1sec_for_ups_system,ups_error_count); rec_err_eeprom_wr_active_fl=1; //error
                if (V_DC_bar_upper_fast_high_err_hold==0) V_DC_bar_upper_fast_high_err_hold=V_DC_bar_upper_meas;
            }
        }
        else if( V_DC_bar_upper_meas < V_DC_bar_fast_limit_min && V_DC_bar_upper_fast_under_volt_protc_actv==1){
            V_DC_bar_upper_fast_high_accept_cnt=0;
            if(V_DC_bar_upper_fast_low_accept_cnt < V_DC_BAR_FAST_LOW_ACCEPT_PERIOD) V_DC_bar_upper_fast_low_accept_cnt++;
            if(V_DC_bar_upper_fast_low_accept_cnt == V_DC_BAR_FAST_LOW_ACCEPT_PERIOD){ // 100us
                V_DC_bar_upper_fast_low_err=1; V_DC_bar_fast_err_try_return=1; DC_bar_fast_fault_repeat++; return_DC_bar_fast_fault_cnt=0;
                stop_pwm_PFC_boost();
                start_control_syst_PFC_boost=0;
                turn_off_input_thyr();
                thyr_input_side_turn_off_complete_delay=1;
                rec_data_log_stop_fl=1;
                V_DC_bar_upper_fast_high_accept_cnt=0;
                V_DC_bar_upper_fast_low_accept_cnt=0;
                V_DC_bar_lower_fast_low_accept_cnt=0;
                V_DC_bar_lower_fast_high_accept_cnt=0;
                if (DC_bar_fast_fault_repeat > 3) {
                    DC_bar_fast_fault_persists=1;
                }
                ups_error_count=addError(ERR_V_DC_BAR_UPPER_LOW,timer_1sec_for_ups_system,ups_error_count); rec_err_eeprom_wr_active_fl=1;//error
                if (V_DC_bar_upper_fast_low_err_hold==0) V_DC_bar_upper_fast_low_err_hold=V_DC_bar_upper_meas;
            }
        }
        else{
            V_DC_bar_upper_fast_high_accept_cnt=0;
            V_DC_bar_upper_fast_low_accept_cnt=0;
        }

        //////////////////////////////////////

        if(V_DC_bar_lower_meas > V_DC_bar_fast_limit_max ){
            V_DC_bar_lower_fast_low_accept_cnt=0;
            if(V_DC_bar_lower_fast_high_accept_cnt < V_DC_BAR_FAST_HIGH_ACCEPT_PERIOD) V_DC_bar_lower_fast_high_accept_cnt++;
            if(V_DC_bar_lower_fast_high_accept_cnt == V_DC_BAR_FAST_HIGH_ACCEPT_PERIOD){
                V_DC_bar_lower_fast_high_err=1; V_DC_bar_fast_err_try_return=1; DC_bar_fast_fault_repeat++; return_DC_bar_fast_fault_cnt=0;
                stop_pwm_PFC_boost();
                start_control_syst_PFC_boost=0;
                turn_off_input_thyr();
                thyr_input_side_turn_off_complete_delay=1;
                rec_data_log_stop_fl=1;
                V_DC_bar_upper_fast_high_accept_cnt=0;
                V_DC_bar_upper_fast_low_accept_cnt=0;
                V_DC_bar_lower_fast_low_accept_cnt=0;
                V_DC_bar_lower_fast_high_accept_cnt=0;
                if (DC_bar_fast_fault_repeat > 3) {
                    DC_bar_fast_fault_persists=1;
                }
                ups_error_count=addError(ERR_V_DC_BAR_LOWER_HIGH,timer_1sec_for_ups_system,ups_error_count);  rec_err_eeprom_wr_active_fl=1; //error
                if (V_DC_bar_lower_fast_high_err_hold==0) V_DC_bar_lower_fast_high_err_hold=V_DC_bar_lower_meas;
            }
        }
        else if( V_DC_bar_lower_meas < V_DC_bar_fast_limit_min && V_DC_bar_lower_fast_under_volt_protc_actv==1){
            V_DC_bar_lower_fast_high_accept_cnt=0;
            if(V_DC_bar_lower_fast_low_accept_cnt < V_DC_BAR_FAST_LOW_ACCEPT_PERIOD) V_DC_bar_lower_fast_low_accept_cnt++;
            if(V_DC_bar_lower_fast_low_accept_cnt == V_DC_BAR_FAST_LOW_ACCEPT_PERIOD){
                V_DC_bar_lower_fast_low_err=1; V_DC_bar_fast_err_try_return=1; DC_bar_fast_fault_repeat++; return_DC_bar_fast_fault_cnt=0;
                stop_pwm_PFC_boost();
                start_control_syst_PFC_boost=0;
                turn_off_input_thyr();
                thyr_input_side_turn_off_complete_delay=1;
                rec_data_log_stop_fl=1;
                V_DC_bar_upper_fast_high_accept_cnt=0;
                V_DC_bar_upper_fast_low_accept_cnt=0;
                V_DC_bar_lower_fast_low_accept_cnt=0;
                V_DC_bar_lower_fast_high_accept_cnt=0;
                if (DC_bar_fast_fault_repeat > 3) {
                    DC_bar_fast_fault_persists=1;
                }
                ups_error_count=addError(ERR_V_DC_BAR_LOWER_LOW,timer_1sec_for_ups_system,ups_error_count); rec_err_eeprom_wr_active_fl=1; //error
                if (V_DC_bar_lower_fast_low_err_hold==0) V_DC_bar_lower_fast_low_err_hold=V_DC_bar_lower_meas;
            }
        }
        else{
            V_DC_bar_lower_fast_low_accept_cnt=0;
            V_DC_bar_lower_fast_high_accept_cnt=0;
        }
    }

    /////////////////////// input current monitor 30us
    if (I_R_in_meas > I_in_30u_protc_lim && I_in_30u_fault_try_return==0){
         if(accept_I_in_R_30u_fault_cnt < ACCEPT_I_IN_30U_FAULT_PERIOD) accept_I_in_R_30u_fault_cnt++; // ACCEPT_I_IN_30U_FAULT_PERIOD 3
         if(accept_I_in_R_30u_fault_cnt == ACCEPT_I_IN_30U_FAULT_PERIOD){
             I_in_R_30u_fault=1; I_in_30u_fault_try_return=1; I_in_30u_fault_repeat++; return_I_in_30u_fault_cnt=0;
             stop_pwm_PFC_boost();
             start_control_syst_PFC_boost=0;
             turn_off_input_thyr();
             thyr_input_side_turn_off_complete_delay=1;
             rec_data_log_stop_fl=1;
             accept_I_in_R_30u_fault_cnt=0;
             accept_I_in_S_30u_fault_cnt=0;
             accept_I_in_T_30u_fault_cnt=0;
             if (I_in_30u_fault_repeat > 3) {
                 I_in_30u_fault_persists=1;
             }
             ups_error_count=addError(ERR_REC_I_R_PEAK,timer_1sec_for_ups_system,ups_error_count);  rec_err_eeprom_wr_active_fl=1; //error
        }
    }
    else{
        accept_I_in_R_30u_fault_cnt=0;
    }

    if (I_S_in_meas > I_in_30u_protc_lim && I_in_30u_fault_try_return==0){
         if(accept_I_in_S_30u_fault_cnt < ACCEPT_I_IN_30U_FAULT_PERIOD) accept_I_in_S_30u_fault_cnt++;
         if(accept_I_in_S_30u_fault_cnt == ACCEPT_I_IN_30U_FAULT_PERIOD){
             I_in_S_30u_fault=1; I_in_30u_fault_try_return=1; I_in_30u_fault_repeat++; return_I_in_30u_fault_cnt=0;
             stop_pwm_PFC_boost();
             start_control_syst_PFC_boost=0;
             turn_off_input_thyr();
             thyr_input_side_turn_off_complete_delay=1;
             rec_data_log_stop_fl=1;
             accept_I_in_R_30u_fault_cnt=0;
             accept_I_in_S_30u_fault_cnt=0;
             accept_I_in_T_30u_fault_cnt=0;
             if (I_in_30u_fault_repeat > 3) {
                 I_in_30u_fault_persists=1;
             }
             ups_error_count=addError(ERR_REC_I_S_PEAK,timer_1sec_for_ups_system,ups_error_count); rec_err_eeprom_wr_active_fl=1;//error
        }
    }
    else{
        accept_I_in_S_30u_fault_cnt=0;
    }

    if (I_T_in_meas > I_in_30u_protc_lim && I_in_30u_fault_try_return==0){
         if(accept_I_in_T_30u_fault_cnt < ACCEPT_I_IN_30U_FAULT_PERIOD) accept_I_in_T_30u_fault_cnt++;
         if(accept_I_in_T_30u_fault_cnt == ACCEPT_I_IN_30U_FAULT_PERIOD){
             I_in_T_30u_fault=1; I_in_30u_fault_try_return=1; I_in_30u_fault_repeat++; return_I_in_30u_fault_cnt=0;
             stop_pwm_PFC_boost();
             start_control_syst_PFC_boost=0;
             turn_off_input_thyr();
             thyr_input_side_turn_off_complete_delay=1;
             rec_data_log_stop_fl=1;
             accept_I_in_R_30u_fault_cnt=0;
             accept_I_in_S_30u_fault_cnt=0;
             accept_I_in_T_30u_fault_cnt=0;
             if (I_in_30u_fault_repeat > 3) {
                 I_in_30u_fault_persists=1;
             }
             ups_error_count=addError(ERR_REC_I_T_PEAK,timer_1sec_for_ups_system,ups_error_count);  rec_err_eeprom_wr_active_fl=1; //error
        }
    }
    else {
        accept_I_in_T_30u_fault_cnt=0;
    }

    ////*************************************
        PLL_1PH();   // main pll function
    ////*************************************

    //******** phase squence ***************//
    //NOT: faz sırası hatalı ise ekrana bu bilgi gnderilmeli "Rec_phase_squence_correct_err"
    //NOT: Faz sırası hatalı ise cap charge işlemi olmamalı
    //NOT: Faz sırası yanlış ise rec kontrol sistemi kesinlikle başlamamalı!!!!!!!!!!!!!
    if(Rec_phase_squence_correct == 0 && V_line_range_is_OK == 1 && rec_R_pll_all_stable_true==1){
        if (Rec_R_phase_angle_meas > 0.0f && rec_R_phase_seq_V_hold_fl==0){
            rec_R_phase_seq_V_hold = V_R_in_meas;
            rec_R_phase_seq_V_hold_fl=1;
        }
        else if (Rec_R_phase_angle_meas >= 2.0944f && rec_S_phase_seq_V_hold_fl==0){
            rec_S_phase_seq_V_hold = V_S_in_meas;
            rec_S_phase_seq_V_hold_fl=1;
        }
        else if (Rec_R_phase_angle_meas >= 4.1888f && rec_T_phase_seq_V_hold_fl==0){
            rec_T_phase_seq_V_hold = V_T_in_meas;
            rec_T_phase_seq_V_hold_fl=1;

            if ((rec_R_phase_seq_V_hold > 10.0f) && (rec_S_phase_seq_V_hold > 10.0f) && (rec_T_phase_seq_V_hold > 10.0f)){
                Rec_phase_squence_correct=1;
                Rec_phase_squence_correct_err = 0;
                rec_R_phase_seq_V_hold_fl =0;
                rec_S_phase_seq_V_hold_fl=0;
                rec_T_phase_seq_V_hold_fl=0;
                rec_R_phase_seq_V_hold=0;
                rec_S_phase_seq_V_hold=0;
                rec_T_phase_seq_V_hold=0;
            } else{
                Rec_phase_squence_correct_err = 1;
                Rec_phase_squence_correct = 0;
                rec_R_phase_seq_V_hold_fl =0;
                rec_S_phase_seq_V_hold_fl=0;
                rec_T_phase_seq_V_hold_fl=0;
                rec_R_phase_seq_V_hold=0;
                rec_S_phase_seq_V_hold=0;
                rec_T_phase_seq_V_hold=0;
            }
        }
    }

// end of ISR Code here
    clearPWMInterruptFlag(EPWM5_BASE);
    clearInterrupt(INTERRUPT_ACK_GROUP3);
    DINT;
    HWREGH(PIECTRL_BASE + PIE_O_IER3) = tempPIEIER;
    traceISR[traceISRIndex % TRACE_SIZE] = 1;
    traceISRIndex++;
}


//////=============================================================================
//////  rx kesmesi ile veri alındığında okuma yapılıyor
//////  "data_count" değeri 16 bitlik verileri 8 bit halinde gönderileceğinden çarpı 2 olmalıdır
//////
//////=============================================================================
__interrupt void UPS_REC_COM_ISR(void) {

    volatile uint16_t tempPIEIER = HWREGH(PIECTRL_BASE + PIE_O_IER8);
    IER |= M_INT8;
    IER &= MINT8;
    HWREGH(PIECTRL_BASE + PIE_O_IER8) &= MG8_7;
    Interrupt_clearACKGroup(0xFFFFU);
    __asm("  NOP");
    EINT;
    //ISR Code here

      modbus_status=Modbus_read(UPS_REC_SCI_BASE,ModbusRxBuffer,sizeof(ModbusRxBuffer),MODBUS_START);
      if  (ModbusRxBuffer[1]==MODBAS_SLAVE_ID){
          ModbusRxBuffer[0]=MODBUS_START;
          ModbusEndOfDataIndx = 3 + ModbusRxBuffer[3]*2;
          uint16_t modbus_Rx_crc = calculateCRC(ModbusRxBuffer, (ModbusEndOfDataIndx + 1));
          uint8_t calc_modbus_Rx_crc_low_byte = modbus_Rx_crc & 0xFF;
          uint8_t calc_modbus_Rx_crc_high_byte = (modbus_Rx_crc >> 8) & 0xFF;
          if(calc_modbus_Rx_crc_low_byte == ModbusRxBuffer[ModbusEndOfDataIndx + 1] && calc_modbus_Rx_crc_high_byte == ModbusRxBuffer[ModbusEndOfDataIndx + 2]){
              I2C_eeprom_active_fl = (bool) ModbusRxBuffer[45];
              ups_error_count_index_from_front_panel = ((uint16_t) (ModbusRxBuffer[68] << 8)) | ModbusRxBuffer[69]; // ups errors data count from front panel
              ups_eeprom_reset_active_fl = ((uint16_t) (ModbusRxBuffer[70] << 8)) | ModbusRxBuffer[71]; // ups errors data count from front panel
              switch (ModbusRxBuffer[2]) {
                  case MODBUS_READ_CODE: // Read registerss
                      comm_main_board_to_monitor_fl=1;
                      panel_to_main_board_write_active=0;
                      ups_calibrate_active_fl=0;
                      break;
                  case MODBUS_WRITE_CODE: // Write registers
                      I2C_eeprom_read_active_fl=0;
                      ups_calibrate_active_fl=1;
                      I2C_eeprom_write_fl=1;
                      panel_to_main_board_write_active=1;
                      comm_main_board_to_monitor_fl=1;
                      break;
              }
          }
      }

   //end of ISR Code here
//   SCI_clearOverflowStatus(UPS_REC_SCI_BASE);
//   SCI_clearInterruptStatus(UPS_REC_SCI_BASE, SCI_INT_RXFF); // Clear INT flag for this RXRDY
   SCI_clearInterruptStatus(UPS_REC_SCI_BASE,SCI_INT_RXRDY_BRKDT); // Clear INT flag for this RXRDY
   Interrupt_clearACKGroup(UPS_REC_INT_mySCI_RX_INTERRUPT_ACK_GROUP);
   uart_intrrupt_fl_clear_cnt=0;
   DINT;
   HWREGH(PIECTRL_BASE + PIE_O_IER8) = tempPIEIER;
   traceISR[traceISRIndex % TRACE_SIZE] = 3;
   traceISRIndex++;
}



