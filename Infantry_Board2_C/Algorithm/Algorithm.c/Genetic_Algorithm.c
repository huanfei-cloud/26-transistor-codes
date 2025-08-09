#include "Genetic_Algorithm.h"

#define POP_SIZE 75       // ��Ⱥ��С
#define NUM_GENERATIONS 150 // ��������
#define NUM_PARAMS 3      // PID�������� (Kp, Ki, Kd)
#define MUTATION_RATE 0.1 // �������
#define CROSSOVER_RATE 0.8 // �������

// PID������Χ
#define KP_MIN 100.0
#define KP_MAX 800.0
#define KI_MIN 0.0
#define KI_MAX 0.1
#define KD_MIN 50.0
#define KD_MAX 600.0

double monitor_Kp;
double monitor_Ki;
double monitor_Kd;


// ��Ӧ�Ⱥ���������ISE������ƽ����
double compute_fitness(double Kp, double Ki, double Kd) {
    // ģ��ϵͳ��Ӧ������ϵͳ��
    double y = 0.0;      // ϵͳ���
    double dy = 0.0;     // ϵͳ�������
    double integral = 0.0; // ������
    double error = 0.0;  // ���
    double dt = 0.01;    // ʱ�䲽��
    double setpoint = 1.0; // �ο�����
    double ise = 0.0;    // ����ƽ�����

    for (double t = 0; t < 10.0; t += dt) {
        error = setpoint - y;
        integral += error * dt;
        double u = Kp * error + Ki * integral + Kd * (error - dy); // PID����
        dy = (-2 * dy - y + u) * dt; // ϵͳ��̬����
        y += dy;
        ise += error * error * dt;
    }

    return 1.0 / (ise + 1e-6); // ��Ӧ�ȣ����ԽС����Ӧ��Խ�ߣ�
}

// ��ʼ����Ⱥ
void initialize_population(double population[POP_SIZE][NUM_PARAMS]) {
    srand(time(NULL));
    for (int i = 0; i < POP_SIZE; i++) {
        population[i][0] = KP_MIN + (double)rand() / RAND_MAX * (KP_MAX - KP_MIN); // Kp
        population[i][1] = KI_MIN + (double)rand() / RAND_MAX * (KI_MAX - KI_MIN); // Ki
        population[i][2] = KD_MIN + (double)rand() / RAND_MAX * (KD_MAX - KD_MIN); // Kd
    }
}

// ѡ�������������ѡ��
void selection(double population[POP_SIZE][NUM_PARAMS], double fitness[POP_SIZE]) {
    double new_population[POP_SIZE][NUM_PARAMS];
    for (int i = 0; i < POP_SIZE; i++) {
        int candidate1 = rand() % POP_SIZE;
        int candidate2 = rand() % POP_SIZE;
        int candidate3 = rand() % POP_SIZE;
        int winner = (fitness[candidate1] > fitness[candidate2]) ? candidate1 : candidate2;
        winner = (fitness[winner] > fitness[candidate3]) ? winner : candidate3;
        for (int j = 0; j < NUM_PARAMS; j++) {
            new_population[i][j] = population[winner][j];
        }
    }
    for (int i = 0; i < POP_SIZE; i++) {
        for (int j = 0; j < NUM_PARAMS; j++) {
            population[i][j] = new_population[i][j];
        }
    }
}

// ������������Ƚ��棩
void crossover(double population[POP_SIZE][NUM_PARAMS]) {
    for (int i = 0; i < POP_SIZE; i += 2) {
        if ((double)rand() / RAND_MAX < CROSSOVER_RATE) {
            for (int j = 0; j < NUM_PARAMS; j++) {
                if (rand() % 2 == 0) {
                    double temp = population[i][j];
                    population[i][j] = population[i + 1][j];
                    population[i + 1][j] = temp;
                }
            }
        }
    }
}

// �����������˹���죩
void mutation(double population[POP_SIZE][NUM_PARAMS]) {
    for (int i = 0; i < POP_SIZE; i++) {
        for (int j = 0; j < NUM_PARAMS; j++) {
            if ((double)rand() / RAND_MAX < MUTATION_RATE) {
                population[i][j] += (double)rand() / RAND_MAX * 0.1 - 0.05; // С��Χ�Ŷ�
                // ȷ�������ڷ�Χ��
                if (j == 0) population[i][j] = fmax(KP_MIN, fmin(KP_MAX, population[i][j]));
                if (j == 1) population[i][j] = fmax(KI_MIN, fmin(KI_MAX, population[i][j]));
                if (j == 2) population[i][j] = fmax(KD_MIN, fmin(KD_MAX, population[i][j]));
            }
        }
    }
}

// �Ŵ��㷨������
void genetic_algorithm(positionpid_t *pid_t) 
{
    double population[POP_SIZE][NUM_PARAMS]; // ��Ⱥ
    double fitness[POP_SIZE]; // ��Ӧ��
    double best_fitness = -1.0;
    double best_params[NUM_PARAMS];

    // ��ʼ����Ⱥ
    initialize_population(population);

    // �����Ż�
    for (int gen = 0; gen < NUM_GENERATIONS; gen++) {
        // ������Ӧ��
        for (int i = 0; i < POP_SIZE; i++) {
            fitness[i] = compute_fitness(population[i][0], population[i][1], population[i][2]);
            if (fitness[i] > best_fitness) {
                best_fitness = fitness[i];
                for (int j = 0; j < NUM_PARAMS; j++) {
                    best_params[j] = population[i][j];
                }
            }
        }

        // ѡ�񡢽��桢����
        selection(population, fitness);
        crossover(population);
        mutation(population);
				
				monitor_Kp = best_params[0];
				monitor_Ki = best_params[1];
				monitor_Kd = best_params[2];
        // �����ǰ���Ž�
				pid_t->Kp = best_params[0];
				pid_t->Ki = best_params[1];
				pid_t->Kd = best_params[2];

    }

}
