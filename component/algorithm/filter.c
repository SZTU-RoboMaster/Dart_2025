

#include "main.h"
#include "filter.h"


//һ�׿������˲� û��ʹ�þ������� ���㲻����
/************ һ�� �������˲�*************/

/**
  * @name   kalmanCreate
  * @brief  ����һ��һ�׿������˲���
  * @param  p:  �˲���
  *         T_Q:ϵͳ����Э����
  *         T_R:��������Э����
  *
  * @retval none
  * @attention R�̶���QԽ�󣬴���Խ���β���ֵ��Q��������ֻ�ò���ֵ
  *		       	��֮��QԽС����Խ����ģ��Ԥ��ֵ��QΪ������ֻ��ģ��Ԥ��
  */
void first_Kalman_Create(first_kalman_filter_t *p, float T_Q, float T_R)
{
    p->X_last = (float)0;
    p->P_last = 0;
    p->Q = T_Q;
    p->R = T_R;
    p->A = 1;
    p->B = 0;
    p->H = 1;
    p->X_mid = p->X_last;
}

/**
  * @name   KalmanFilter
  * @brief  �������˲���
  * @param  p:  �˲���
  *         dat:���˲�����
  * @retval �˲���������
  * @attention Z(k)��ϵͳ����,������ֵ   X(k|k)�ǿ������˲�����ֵ,����������
  *            A=1 B=0 H=1 I=1  W(K)  V(k)�Ǹ�˹������,�����ڲ���ֵ����,���Բ��ù�
  *            �����ǿ�������5�����Ĺ�ʽ
  *            һ��H'��Ϊ������,����Ϊת�þ���
  */

float first_Kalman_Filter(first_kalman_filter_t* p, float dat)
{
    p->X_mid =p->A*p->X_last;                     //�ٶȶ�Ӧ��ʽ(1)    x(k|k-1) = A*X(k-1|k-1)+B*U(k)+W(K)
    p->P_mid = p->A*p->P_last+p->Q;               //�ٶȶ�Ӧ��ʽ(2)    p(k|k-1) = A*p(k-1|k-1)*A'+Q
    p->kg = p->P_mid/(p->P_mid+p->R);             //�ٶȶ�Ӧ��ʽ(4)    kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
    p->X_now = p->X_mid+p->kg*(dat-p->X_mid);     //�ٶȶ�Ӧ��ʽ(3)    x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
    p->P_now = (1-p->kg)*p->P_mid;                //�ٶȶ�Ӧ��ʽ(5)    p(k|k) = (I-kg(k)*H)*P(k|k-1)
    p->P_last = p->P_now;                         //״̬����
    p->X_last = p->X_now;
    return p->X_now;							  //����Ԥ������x(k|k)
}

/************ һ�� �������˲�*************/



/************ ���� �������˲� *************/

float matrix_value1;
float matrix_value2;

void second_kalman_filter_init(second_kalman_filter_t *F, second_kalman_filter_init_t *I) {
    mat_init(&F->xhat, 2, 1, (float *) I->xhat_data);
    mat_init(&F->xhatminus, 2, 1, (float *) I->xhatminus_data);
    mat_init(&F->z, 2, 1, (float *) I->z_data);
    mat_init(&F->A, 2, 2, (float *) I->A_data);
    mat_init(&F->H, 2, 2, (float *) I->H_data);
    mat_init(&F->Q, 2, 2, (float *) I->Q_data);
    mat_init(&F->R, 2, 2, (float *) I->R_data);
    mat_init(&F->P, 2, 2, (float *) I->P_data);
    mat_init(&F->Pminus, 2, 2, (float *) I->Pminus_data);
    mat_init(&F->K, 2, 2, (float *) I->K_data);
    mat_init(&F->AT, 2, 2, (float *) I->AT_data);
    mat_trans(&F->A, &F->AT);
    mat_init(&F->HT, 2, 2, (float *) I->HT_data);
    mat_trans(&F->H, &F->HT);
//  matrix_value2 = F->A.pD
}


/**
  *@param �����������ṹ��
	�о��߽�Ҳ�ǿ�����ô�㵹��
  *@param �Ƕ�
  *@param �ٶ�
*/
float* second_kalman_filter_calc(second_kalman_filter_t *F, float signal1, float signal2)
{
    float TEMP_data[4] = {0, 0, 0, 0};
    float TEMP_data21[2] = {0, 0};
    mat TEMP,TEMP21;

    mat_init(&TEMP,2,2,(float *)TEMP_data);//
    mat_init(&TEMP21,2,1,(float *)TEMP_data21);//

    //��ȡ����ֵ
    F->z.pData[0] = signal1;//z(k)
    F->z.pData[1] = signal2;//z(k)

    //�����˻��ǵ�һ�������˵ڶ�������Ȼ�󽫽����ŵ���������������
    //1. xhat'(k)= A xhat(k-1)  xhat->ָ��^��x
    //��һ����������һʱ��Xhat���Ž�Ԥ�⵱ǰxhat
    mat_mult(&F->A, &F->xhat, &F->xhatminus);//  x(k|k-1) = A*X(k-1|k-1)+B*U(k)+W(K)

    //2. P'(k) = A P(k-1) AT + Q
    //�ڶ������ɹ�ȥ��Э�����������㵱ǰxhat��Э��������
    mat_mult(&F->A, &F->P, &F->Pminus);//   p(k|k-1) = A*p(k-1|k-1)*A'+Q
    mat_mult(&F->Pminus, &F->AT, &TEMP);//  p(k|k-1) = A*p(k-1|k-1)*A'+Q
    mat_add(&TEMP, &F->Q, &F->Pminus);//    p(k|k-1) = A*p(k-1|k-1)*A'+Q

    //3. K(k) = P'(k) HT / (H P'(k) HT + R)
    //�����������ݵ�ǰЭ�����������㿨��������
    //��һ����������һ�����ⶼ����K������������������һ������
    mat_mult(&F->H, &F->Pminus, &F->K);//  kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
    mat_mult(&F->K, &F->HT, &TEMP);//      kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
    mat_add(&TEMP, &F->R, &F->K);//        kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)

    mat_inv(&F->K, &F->P);	//	����ʽ��ĸȡ���ٳ�
    mat_mult(&F->Pminus, &F->HT, &TEMP);//
    mat_mult(&TEMP, &F->P, &F->K);//

    //4. xhat(k) = xhat'(k) + K(k) (z(k) - H xhat'(k))
    //���Ĳ�������QR���ۺ�Ԥ�����Ͳ���ֵ�����㵱ǰxhat���Ž�
    mat_mult(&F->H, &F->xhatminus, &TEMP21);//      x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
    mat_sub(&F->z, &TEMP21, &F->xhat);//            x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
    mat_mult(&F->K, &F->xhat, &TEMP21);//           x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
    mat_add(&F->xhatminus, &TEMP21, &F->xhat);//    x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))

    //5. P(k) = (1-K(k)H)P'(k)
    //���岽�����㵱ǰ���Ž���Э��������
    mat_mult(&F->K, &F->H, &F->P);//            p(k|k) = (I-kg(k)*H)*P(k|k-1)
    mat_sub(&F->Q, &F->P, &TEMP);//
    mat_mult(&TEMP, &F->Pminus, &F->P);

    //��ȡ���׿������ļ�������
    matrix_value1 = F->xhat.pData[0];
    matrix_value2 = F->xhat.pData[1];

    F->filtered_value[0] = F->xhat.pData[0];
    F->filtered_value[1] = F->xhat.pData[1];
    return F->filtered_value;
}


/************ ���� �������˲� *************/


