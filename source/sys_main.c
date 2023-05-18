/** @file sys_main.c 
*   @brief Application main file
*   @date 11-Dec-2018
*   @version 04.07.01
*
*   This file contains an empty main function,
*   which can be used for the application.
*/

/* 
* Copyright (C) 2009-2018 Texas Instruments Incorporated - www.ti.com 
* 
* 
*  Redistribution and use in source and binary forms, with or without 
*  modification, are permitted provided that the following conditions 
*  are met:
*
*    Redistributions of source code must retain the above copyright 
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the 
*    documentation and/or other materials provided with the   
*    distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/


/* USER CODE BEGIN (0) */
#include "sci.h"
#include "adc.h"
#include "gio.h"
#include "stdlib.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END */

/* Include Files */

#include "sys_common.h"
#include "arm_math.h"
/* USER CODE BEGIN (1) */
unsigned char command[10];

char command1[3];
char command2[10];
/* USER CODE END */


/** @fn void main(void)
*   @brief Application main function
*   @note This function is empty by default.
*
*   This function is called after startup.
*   The user can use this function to implement the application.
*/

/* USER CODE BEGIN (2) */

void ADCS_Omega(arm_matrix_instance_f32 * omega, arm_matrix_instance_f32 * pDst);
void ADCS_f(arm_matrix_instance_f32 * Omega_main_arm, arm_matrix_instance_f32 * q_main_arm,arm_matrix_instance_f32 * f_main_arm);
void ADCS_h(arm_matrix_instance_f32 * q_main_arm, arm_matrix_instance_f32 * m_ref_main_arm,arm_matrix_instance_f32 * a_ref_main_arm,arm_matrix_instance_f32 * h_main_arm);
void ADCS_dhdq(arm_matrix_instance_f32 * q_main_arm, arm_matrix_instance_f32 * h_main_arm, arm_matrix_instance_f32 * dhdq_main_arm);
void ADCS_dfdq(arm_matrix_instance_f32 * Omega_main_arm, arm_matrix_instance_f32 * dfdq_main_arm);
void ADCS_q2R(arm_matrix_instance_f32 * q_main_arm, arm_matrix_instance_f32 * R_main_arm);
void ADCS_append_array(arm_matrix_instance_f32 * x_main_arm, arm_matrix_instance_f32 * y_main_arm,arm_matrix_instance_f32 * out_main_arm);
float32_t* ADCS_append_array_float32(float32_t * x, float32_t * y);
float32_t* ADCS_vector_normalized(float32_t * x, uint32_t n);

void ADCS_copyArray(float32_t ref[], float32_t dest[],uint8_t numArr);
void ADCS_MatrixInit(float32_t **matrix, uint8_t sizerow, uint8_t sizecolumn);
void ADCS_MatrixPrint(float32_t input1[][], uint8_t sizerow, uint8_t sizecolumn);
void ADCS_MatrixPrintARM(arm_matrix_instance_f32 S, uint8_t sizerow, uint8_t sizecolumn);
void ADCS_set_reference_frame(float32_t * mref, char * frame, float32_t * m_ref, float32_t * a_ref);
void ADCS_update(float32_t * q, float32_t * gyr, float32_t * acc, float32_t * mag, float32_t * mref, char * frame, arm_matrix_instance_f32 * output_quat);



/* USER CODE END */

extern float32_t eye[16] = {1.0, 0.,0.,0.,
                     0.0, 1.,0.,0.,
                     0.0, 0.,1.,0.,
                     0.0, 0.,0.,1.};
extern float32_t zeros16[16] = {0.0, 0.,0.,0.,
                     0.0, 0.,0.,0.,
                     0.0, 0.,0.,0.,
                     0.0, 0.,0.,0.};
extern float32_t zeros24[24] = {0.0, 0.,0.,0.,
                     0.0, 0.,0.,0.,
                     0.0, 0.,0.,0.,
                     0.0, 0.,0.,0.,
                     0.0, 0.,0.,0.,
                     0.0, 0.,0.,0.};
extern float32_t zeros9[9] = {0.0, 0.,0.,
                     0.0, 0.,0.,
                     0.0, 0.,0.,
                     0.0, 0.,0.};
extern float32_t zeros4[4] = {0.0, 0.,0.,0.};
extern float32_t zeros3[3] = {0.0, 0.,0.};
extern float32_t zeros6[6] = {0.0, 0.,0.,0.,0.,0.};
extern float32_t dt = 0.01;

#define NELEMS(x)  (sizeof(x) / sizeof((x)[0]))





int main(void)
{
/* USER CODE BEGIN (3) */

    sciInit(); //Initializes the SCI (UART) module


    // Defining Variable Vector
    float32_t omega[3] = {10.1, 40.2, 20.3};
    float32_t q_simple[4] = {2.1, 4.2, 3.3, 5.1};


    float32_t mag[3] = {0.91, 0.65, 0.1};
    float32_t acc[3] = {0.74, -0.42, 0.34};


    float32_t a_ref_main_float[3] = {1.2, 1.6, 0.75};
    float32_t m_ref_main_float[3] = {0.32, 2.1, 0.6};


    float32_t* magnormalized = ADCS_vector_normalized( mag, 3);
    float32_t* accnormalized = ADCS_vector_normalized( acc, 3);

    float32_t* z_function_float = ADCS_append_array_float32(magnormalized, accnormalized);

    ADCS_MatrixPrint(z_function_float,  6, 1);
    //ADCS_MatrixPrint(gyronormalized,  3, 1);



    arm_matrix_instance_f32 Omega_t_main_arm,Omega_main_arm,q_main_arm,f_main_arm,dfdq_main_arm;
    arm_matrix_instance_f32 a_ref_main_arm,m_ref_main_arm,h_main_arm;
    arm_matrix_instance_f32 dhdq_main_arm;

    arm_mat_init_f32(&q_main_arm,4,1,(float32_t *) q_simple);

    arm_mat_init_f32(&Omega_t_main_arm,4,4,(float32_t *) zeros16);
    arm_mat_init_f32(&Omega_main_arm,3,1,(float32_t *) omega);
    arm_mat_init_f32(&a_ref_main_arm,3,1,(float32_t *) a_ref_main_float);
    arm_mat_init_f32(&m_ref_main_arm,3,1,(float32_t *) m_ref_main_float);
    arm_mat_init_f32(&h_main_arm,6,1,(float32_t *) zeros6);
    arm_mat_init_f32(&dhdq_main_arm,6,4,(float32_t *) zeros24);


    ADCS_Omega(&Omega_main_arm, &Omega_t_main_arm);
    ADCS_f(&Omega_main_arm,&q_main_arm,&f_main_arm);
    ADCS_MatrixPrintARM(f_main_arm,  4, 1);

    ADCS_dfdq(&Omega_main_arm, &dfdq_main_arm);
    ADCS_MatrixPrintARM(dfdq_main_arm,  4, 4);

    ADCS_h(&q_main_arm, &m_ref_main_arm, &a_ref_main_arm, &h_main_arm);
    ADCS_MatrixPrintARM(h_main_arm,  6, 1);



    ADCS_dhdq(&q_main_arm, &h_main_arm, &dhdq_main_arm);
    ADCS_MatrixPrintARM(dhdq_main_arm,  6, 4);


    sciSend(scilinREG, 8, (unsigned char *)"END"); //Sends '0x' hex designation chars

/* USER CODE END */

    return 0;
}



void ADCS_MatrixPrintARM(arm_matrix_instance_f32 S, uint8_t sizerow, uint8_t sizecolumn) {
    uint8_t i;
    uint8_t j;

    char command1[10];
    char str1[1200] = "[";
    char strcomma[3] = ", ";
    char strnewline[3] = "\r\n";
    char strclose[3] = "]";
    for (i = 0; i < sizerow; i++) {
        for (j = 0; j < sizecolumn; j++){
            sprintf((char *)command1, "%.4f",  S.pData[i*S.numCols + j]);
            strcat(str1, command1);
            strcat(str1, strcomma);
        }
        if (i==sizerow-1){
            strcat(str1, strclose);
        }
        else{
            strcat(str1, strnewline);
        }

    }

    sciSend(scilinREG, 2, (unsigned char *)"\r\n"); //Sends new line character
    sciSend(scilinREG, 1200,(char *)str1); //Sends the ambient light sensor data
    sciSend(scilinREG, 2, (unsigned char *)"\r\n"); //Sends new line character

}

void ADCS_MatrixPrint(float32_t input1[][], uint8_t sizerow, uint8_t sizecolumn) {
    uint8_t i;
    uint8_t j;

    char command1[10];
    char str1[1200] = "[";
    char strcomma[3] = ", ";
    char strnewline[3] = "\r\n";
    char strclose[3] = "]";
    for (i = 0; i < sizerow; i++) {
        for (j = 0; j < sizecolumn; j++){
            sprintf((char *)command1, "%.4f",  *(*input1 + i*sizecolumn + j));
            strcat(str1, command1);
            strcat(str1, strcomma);
        }
        if (i==sizerow-1){
            strcat(str1, strclose);
        }
        else{
            strcat(str1, strnewline);
        }

    }

    sciSend(scilinREG, 2, (unsigned char *)"\r\n"); //Sends new line character
    sciSend(scilinREG, 1200,(char *)str1); //Sends the ambient light sensor data
    sciSend(scilinREG, 2, (unsigned char *)"\r\n"); //Sends new line character

}

void ADCS_MatrixInit(float32_t **matrix, uint8_t sizerow, uint8_t sizecolumn) {
    uint8_t i;
    matrix = (float32_t **) malloc(sizerow * sizeof(float32_t *));
    for (i = 0; i < sizerow; i++) {
            matrix[i] = (float32_t *) malloc(sizecolumn * sizeof(float32_t));
    }
}

void ADCS_copyArray(float32_t ref[], float32_t dest[],uint8_t numArr){
    uint8_t sizearray = numArr;
    uint8_t i;
    for (i = 0; i < sizearray; i++){
        dest[i] = ref[i];
    }

}

void ADCS_Omega(arm_matrix_instance_f32 * omega1, arm_matrix_instance_f32 * pDst) {
    // Create matrix instance with 4 rows and 4 columns
    float32_t *pOut = pDst->pData;
    float32_t *x1 = omega1->pData;
    float32_t omega[16] = {0.0,  -x1[0], -x1[1], -x1[2],x1[0],   0.0,  x1[2], -x1[1],x1[1], -x1[2],   0.0,  x1[0], x1[2],  x1[1], -x1[0],   0.0};
    ADCS_copyArray(omega,pOut,16);
}



void ADCS_f(arm_matrix_instance_f32 * Omega_main_arm, arm_matrix_instance_f32 * q_main_arm,arm_matrix_instance_f32 * f_main_arm){

    // 1. Extract the input and output data from dsplib variable
    float32_t* Omega_function_float = Omega_main_arm->pData;
    float32_t* q_function_float = q_main_arm->pData;
    //float32_t* f2InOmega = f_main_arm->pData;

    // 2. Define the dsplib Variable
    arm_matrix_instance_f32 Omegascale_t_function_arm,EyeMatrix,Aq,Omega_function_arm,Omega_t_function_arm,q_function_arm,f_function_arm;

    // 3. Initialize the dsplib variable
    arm_mat_init_f32(&Omegascale_t_function_arm,4,4,(float32_t *)zeros16);
    arm_mat_init_f32(&Aq,4,4,(float32_t *)zeros16);
    arm_mat_init_f32(&Omega_t_function_arm,4,4,(float32_t *)zeros16);
    arm_mat_init_f32(&Omega_function_arm,3,1,(float32_t *)Omega_function_float);
    arm_mat_init_f32(&q_function_arm,4,1,(float32_t *)q_function_float);
    arm_mat_init_f32(&f_function_arm,4,1,(float32_t *)zeros4);

    // 4. Function Calculation
    ADCS_Omega(&Omega_function_arm, &Omega_t_function_arm);
    arm_mat_scale_f32(&Omega_t_function_arm, dt * 0.5,&Omegascale_t_function_arm);
    arm_mat_init_f32(&EyeMatrix,4,4,(float32_t *)eye);
    arm_mat_add_f32(&EyeMatrix,&Omegascale_t_function_arm,&Aq);
    arm_mat_mult_f32(&Aq,&q_function_arm,&f_function_arm);

    // 5. Output replacement
    *f_main_arm = f_function_arm;

}

void ADCS_dfdq(arm_matrix_instance_f32 * Omega_main_arm, arm_matrix_instance_f32 * dfdq_main_arm){

    // 1. Extract the input and output data from dsplib variable
    float32_t* Omega_function_float = Omega_main_arm->pData;
    //float32_t* dfdq_function_float = dfdq_main_arm->pData;

    // 2. Define the dsplib Variable
    arm_matrix_instance_f32 Omega_function_arm,EyeMatrix , Omegascale_function_arm, Omega_t_scale_function_arm,dfdq_function_arm;

    // 3. Initialize the dsplib variable
    arm_mat_init_f32(&Omega_function_arm,3,1,(float32_t *)Omega_function_float);
    arm_mat_init_f32(&Omegascale_function_arm,3,1,(float32_t *)zeros3);
    arm_mat_init_f32(&Omega_t_scale_function_arm,4,4,(float32_t *)zeros16);
    arm_mat_init_f32(&EyeMatrix,4,4,(float32_t *)eye);
    arm_mat_init_f32(&dfdq_function_arm,4,4,(float32_t *)zeros16);

    // 4. Function Calculation
    arm_mat_scale_f32(&Omega_function_arm, dt * 0.5,&Omegascale_function_arm);
    ADCS_Omega(&Omegascale_function_arm, &Omega_t_scale_function_arm);
    arm_mat_add_f32(&EyeMatrix,&Omega_t_scale_function_arm,&dfdq_function_arm);

    // 5. Output replacement
    *dfdq_main_arm = dfdq_function_arm;


}

void ADCS_q2R(arm_matrix_instance_f32 * q_main_arm, arm_matrix_instance_f32 * R_main_arm) {

    // 1. Extract the input and output data from dsplib variable
    float32_t* q_function_float = q_main_arm->pData;
    float32_t* R_function_float = R_main_arm->pData;
    float32_t R[9] = {
                           1 - 2.0*(pow(q_function_float[2],2) + pow(q_function_float[3],2)),
                           2.0*(q_function_float[1]*q_function_float[2]+q_function_float[0]*q_function_float[3]),
                           2.0*(q_function_float[1]*q_function_float[3]+q_function_float[0]*q_function_float[2]),
                           2.0*(q_function_float[1]*q_function_float[2]+q_function_float[0]*q_function_float[3]),
                           1 - 2.0*(pow(q_function_float[1],2) + pow(q_function_float[3],2)),
                           2.0*(q_function_float[0]*q_function_float[1]+q_function_float[2]*q_function_float[3]),
                           2.0*(q_function_float[1]*q_function_float[3]+q_function_float[0]*q_function_float[2]),
                           2.0*(q_function_float[2]*q_function_float[3]+q_function_float[0]*q_function_float[1]),
                           1 - 2.0*(pow(q_function_float[1],2) + pow(q_function_float[2],2))
    };
    ADCS_copyArray(R,R_function_float,9);


}

void ADCS_set_reference_frame(float32_t * mref, char * frame, float32_t * m_ref, float32_t * a_ref) {

    float32_t m_ref_init[3];
    float32_t a_ref_init[3];

    if (NELEMS(mref) > 1) {
        // mref is an array
        uint8_t i;
        float32_t mref_norm_squared = 0.;
        for (i = 0; i < 3; i++) {
            mref_norm_squared += mref[i] * mref[i];
            }

        float32_t mref_norm;
        arm_sqrt_f32(mref_norm_squared, &mref_norm);
        arm_scale_f32(mref, 1 / mref_norm, m_ref_init, 3);
    } else {
        // mref is a float (in radian)
        float32_t mref_cos = arm_cos_f32(*mref);
        float32_t mref_sin = arm_sin_f32(*mref);

        if (strcmp(frame, "NED") == 0) {
            m_ref_init[0] = mref_cos;
            m_ref_init[1] = 0.;
            m_ref_init[2] = mref_sin;
        } else {
            m_ref_init[0] = 0.;
            m_ref_init[1] = mref_cos;
            m_ref_init[2] = -mref_sin;
        }
    }

    if (strcmp(frame, "NED") == 0) {
        a_ref_init[0] = 0.;
        a_ref_init[1] = 0.;
        a_ref_init[2] = -1.;
        } else {
        a_ref_init[0] = 0.;
        a_ref_init[1] = 0.;
        a_ref_init[2] = 1.;
    }
    memcpy(m_ref, m_ref_init, sizeof(m_ref_init));
    memcpy(a_ref, a_ref_init, sizeof(a_ref_init));
}

void ADCS_update(float32_t * q, float32_t * gyr, float32_t * acc, float32_t * mag, float32_t * mref, char * frame, arm_matrix_instance_f32 * output_quat) {
    // Initial quaternion q is assumed to have a norm of 1

    // 1. Current measurements
    // Normalize accelerometer vector
    uint8_t i;
    float32_t acc_norm_squared = 0.;
    for (i = 0; i < 3; i++) {
        acc_norm_squared += acc[i] * acc[i];
    }

    float32_t acc_norm;
    arm_sqrt_f32(acc_norm_squared, &acc_norm);

    if (acc_norm == 0) {
        output_quat = q;
        return;
    }

    float32_t acc_normalized[3];
    arm_scale_f32(acc, 1 / acc_norm, acc_normalized, 3);

    // Normalize magnetometer vector, assumed that its norm must be greater than 0
    uint8_t j;
    float32_t mag_norm_sqaured = 0.;
    for (j = 0; j < 3; j++) {
        mag_norm_sqaured += mag[j] * mag[j];
    }

    float32_t mag_norm;
    arm_sqrt_f32(mag_norm_sqaured, &mag_norm);

    float32_t mag_normalized[3];
    arm_scale_f32(mag, 1 / mag_norm, mag_normalized, 3);

    // Measurement vector
    arm_matrix_instance_f32 z;
    float32_t z_buf[6];
    uint8_t k;
    for (k = 0; k < 6; k++) {
        if (k < 3) {
            z_buf[k] = acc_normalized[k];
        } else {
            z_buf[k] = mag_normalized[k - 3];
        }
    }
    arm_mat_init_f32(&z, 6, 1, z_buf);

    // Measurement covariance matrix
    arm_matrix_instance_f32 R;
    // For now, assume gyro, accelerometer, and magnetometer noises
    float32_t gyr_noise = 0.3;
    float32_t acc_noise = 0.5;
    float32_t mag_noise = 0.8;
    float32_t R_buf = {
        acc_noise * acc_noise, 0., 0., 0., 0., 0.,
        0., acc_noise * acc_noise, 0., 0., 0., 0.,
        0., 0., acc_noise * acc_noise, 0., 0., 0.,
        0., 0., 0., mag_noise * mag_noise, 0., 0.,
        0., 0., 0., 0., mag_noise * mag_noise, 0.,
        0., 0., 0., 0., 0., mag_noise * mag_noise
    };
    arm_mat_init_f32(&R, 6, 6, &R_buf);

    // 2. Prediction step
    arm_matrix_instance_f32 q_t, F, FT, W, WT, Q_t, P_t,P;
    float32_t q_t_buf[4];
    float32_t F_buf[16];
    float32_t FT_buf[16];
    float32_t WT_buf[16];
    float32_t Q_t_buf[16];
    float32_t P_t_buf[16];
    arm_mat_init_f32(&q_t, 4, 1, q_t_buf);
    arm_mat_init_f32(&F, 4, 4, F_buf);
    arm_mat_init_f32(&FT, 4, 4, FT_buf);
    arm_mat_init_f32(&WT, 3, 4, WT_buf);
    arm_mat_init_f32(&Q_t, 4, 4, Q_t_buf);
    arm_mat_init_f32(&P_t, 4, 4, P_t_buf);
    ADCS_f(gyr, q, &q_t);
    ADCS_dfdq(gyr, &F);
    float32_t W_temp_buf[12] = {
        -q[1], -q[2], -q[3],
        -q[0], -q[3],  q[2],
         q[3],  q[0], -q[1],
        -q[2],  q[1],  q[0],
    };
    arm_mat_init_f32(&W, 4, 3, W_temp_buf);
    arm_mat_scale_f32(&W, 0.5 * dt, &W);
    arm_mat_trans_f32(&W, &WT);
    arm_mat_mult_f32(&W, &WT, &Q_t);
    arm_mat_scale_f32(&Q_t, 0.5 * dt * gyr_noise, &Q_t);
    arm_mat_mult_f32(&F, &P, &P_t);
    arm_mat_trans_f32(&F, &FT);
    arm_mat_mult_f32(&P_t, &FT, &P_t);
    arm_mat_add_f32(&P_t, &Q_t, &P_t);

    // 3. Correction step
    arm_matrix_instance_f32 y, m_ref, a_ref, v, H, HT, S, SI, K, KH, I4, Kv;
    float32_t y_buf[6];
    float32_t v_buf[6];
    float32_t H_buf[24];
    float32_t HT_buf[24];
    float32_t S_buf[36];
    float32_t SI_buf[36];
    float32_t K_buf[24];
    float32_t KH_buf[16];
    float32_t Kv_buf[4];
    arm_mat_init_f32(&y, 6, 1, y_buf);
    arm_mat_init_f32(&v, 6, 1, v_buf);
    arm_mat_init_f32(&H, 6, 4, H_buf);
    arm_mat_init_f32(&HT, 4, 6, HT_buf);
    arm_mat_init_f32(&S, 6, 6, S_buf);
    arm_mat_init_f32(&SI, 6, 6, SI_buf);
    arm_mat_init_f32(&K, 4, 6, K_buf);
    arm_mat_init_f32(&KH, 4, 4, KH_buf);
    arm_mat_init_f32(&Kv, 4, 1, Kv_buf);
    float32_t m_ref_data[3];
    float32_t a_ref_data[3];
    ADCS_set_reference_frame(mref, frame, m_ref_data, a_ref_data);
    arm_mat_init_f32(&m_ref, 3, 1, m_ref_data);
    arm_mat_init_f32(&a_ref, 3, 1, a_ref_data);
    ADCS_h(&q_t, &m_ref, &a_ref, &y);
    arm_mat_sub_f32(&z, &y, &v);
    ADCS_dhdq(&q_t, &y, &H);
    arm_mat_trans_f32(&H, &HT);
    arm_mat_mult_f32(&H, &P_t, &S);
    arm_mat_mult_f32(&S, &HT, &S);
    arm_mat_add_f32(&S, &R, &S);
    arm_mat_inverse_f32(&S, &SI);
    arm_mat_mult_f32(&P_t, &HT, &K);
    arm_mat_mult_f32(&K, &SI, &K);
    arm_mat_mult_f32(&K, &H, &KH);
    arm_mat_init_f32(&I4, 4, 4, eye);
    arm_mat_sub_f32(&I4, &KH, &P);
    arm_mat_mult_f32(&P, &P_t, &P);
    arm_mat_mult_f32(&K, &v, &Kv);
    arm_mat_add_f32(&q_t, &Kv, output_quat);
    float32_t *output_quat_buf = output_quat->pData;
    float32_t output_quat_norm = 0.;
    uint8_t a;
    for (a = 0; a < 4; a++) {
        output_quat_norm += output_quat_buf[a];
    }
    arm_mat_scale_f32(output_quat, 1 / output_quat_norm, output_quat);

}


void ADCS_append_array(arm_matrix_instance_f32 * x_main_arm, arm_matrix_instance_f32 * y_main_arm,arm_matrix_instance_f32 * out_main_arm){
    // 1. Extract the input and output data from dsplib variable
    float32_t* x_function_float = x_main_arm->pData;
    float32_t* y_function_float = y_main_arm->pData;
    float32_t* out_function_float = out_main_arm->pData;

    float32_t* total = malloc((NELEMS(x_function_float)+NELEMS(y_function_float)) * sizeof(float32_t)); // array to hold the result

    memcpy(total,     x_function_float, NELEMS(x_function_float) * sizeof(float32_t)); // copy 4 floats from x to total[0]...total[3]
    memcpy(total + NELEMS(x_function_float), y_function_float, NELEMS(y_function_float) * sizeof(float32_t));
    ADCS_copyArray(total,out_function_float ,(NELEMS(x_function_float)+NELEMS(y_function_float)));
}

float32_t* ADCS_append_array_float32(float32_t * x, float32_t * y){

    float32_t* z = malloc(6 * sizeof(float)); // array to hold the result

    memcpy(z,     x, 3 * sizeof(float32_t)); // copy 4 floats from x to total[0]...total[3]
    memcpy(z + 3, y, 3 * sizeof(float32_t)); // copy 4 floats from y to total[4]...total[7]
    return z;

}

void ADCS_h(arm_matrix_instance_f32 * q_main_arm, arm_matrix_instance_f32 * m_ref_main_arm,arm_matrix_instance_f32 * a_ref_main_arm,arm_matrix_instance_f32 * h_main_arm){

    // 1. Extract the input and output data from dsplib variable
    float32_t* q_function_float = q_main_arm->pData;
    float32_t* m_ref_function_float = m_ref_main_arm->pData;
    float32_t* a_ref_function_float = a_ref_main_arm->pData;


    // 2. Define the dsplib Variable
    arm_matrix_instance_f32 q_function_arm,m_ref_function_arm,a_ref_function_arm,h_function_arm;
    arm_matrix_instance_f32 R_function_arm,C_function_arm,Ca_function_arm,Cm_function_arm;

    // 3. Initialize the dsplib variable
    arm_mat_init_f32(&q_function_arm,4,1,(float32_t *) q_function_float);
    arm_mat_init_f32(&R_function_arm,3,3,(float32_t *) zeros9);
    arm_mat_init_f32(&C_function_arm,3,3,(float32_t *) zeros9);
    arm_mat_init_f32(&a_ref_function_arm,3,1,(float32_t *) a_ref_function_float);
    arm_mat_init_f32(&Ca_function_arm,3,1,(float32_t *) zeros3);
    arm_mat_init_f32(&m_ref_function_arm,3,1,(float32_t *)m_ref_function_float);
    arm_mat_init_f32(&Cm_function_arm,3,1,(float32_t *) zeros3);


    // 4. Function Calculation
    ADCS_q2R(&q_function_arm,&R_function_arm);
    arm_mat_trans_f32(&R_function_arm,&C_function_arm);
    arm_mat_mult_f32(&C_function_arm,&a_ref_function_arm,&Ca_function_arm);
    float32_t* Ca_function_float = Ca_function_arm.pData;
    float32_t Ca_function_floatcopy[3];
    ADCS_copyArray(Ca_function_float,Ca_function_floatcopy,3);
    arm_mat_mult_f32(&C_function_arm,&m_ref_function_arm,&Cm_function_arm);
    float32_t* Cm_function_float = Cm_function_arm.pData;
    float32_t* h_function_float = ADCS_append_array_float32(Cm_function_float,Ca_function_floatcopy);
    arm_mat_init_f32(&h_function_arm,6,1,(float32_t *) h_function_float);

    // 5. Output replacement
    *h_main_arm = h_function_arm;


}


void ADCS_dhdq(arm_matrix_instance_f32 * q_main_arm, arm_matrix_instance_f32 * h_main_arm, arm_matrix_instance_f32 * dhdq_main_arm) {

    // 1. Extract the input and output data from dsplib variable

    float32_t* q = q_main_arm->pData;
    float32_t qw = q[0];
    float32_t qx = q[1];
    float32_t qy = q[2];
    float32_t qz = q[3];

    float32_t* v = h_main_arm->pData;
    float32_t* dhdq_main_float = dhdq_main_arm->pData;

    float32_t dhdq_function_float[24] =  {-qy*v[2] + qz*v[1],                qy*v[1] + qz*v[2], -qw*v[2] + qx*v[1] - 2.0*qy*v[0],  qw*v[1] + qx*v[2] - 2.0*qz*v[0],
                                qx*v[2] - qz*v[0],  qw*v[2] - 2.0*qx*v[1] + qy*v[0],                qx*v[0] + qz*v[2], -qw*v[0] + qy*v[2] - 2.0*qz*v[1],
                                -qx*v[1] + qy*v[0], -qw*v[1] - 2.0*qx*v[2] + qz*v[0],  qw*v[0] - 2.0*qy*v[2] + qz*v[1],  qx*v[0] + qy*v[1],
                                -qy*v[5] + qz*v[4],                qy*v[4] + qz*v[5], -qw*v[5] + qx*v[4] - 2.0*qy*v[3],  qw*v[4] + qx*v[5] - 2.0*qz*v[3],
                                    qx*v[5] - qz*v[3],  qw*v[5] - 2.0*qx*v[4] + qy*v[3],                qx*v[3] + qz*v[5], -qw*v[3] + qy*v[5] - 2.0*qz*v[4],
                                    -qx*v[4] + qy*v[3], -qw*v[4] - 2.0*qx*v[5] + qz*v[3],  qw*v[3] - 2.0*qy*v[5] + qz*v[4],  qx*v[3] + qy*v[4]};


    //arm_mat_init_f32(&dhdq_main_arm,6,4,(float32_t *) dhdq_function_float);
    ADCS_copyArray(dhdq_function_float,dhdq_main_float,24);


}

float32_t* ADCS_vector_normalized(float32_t * x, uint32_t n){
    float32_t norm_x[1];
    arm_rms_f32(x,n,norm_x);

    arm_scale_f32(x,1./(sqrt(n)*norm_x[0]),x,n);
    return x;
}
