#include "Genetic_Algorithm.h"

#define POP_SIZE 75       // 种群大小
#define NUM_GENERATIONS 150 // 迭代次数
#define NUM_PARAMS 3      // PID参数个数 (Kp, Ki, Kd)
#define MUTATION_RATE 0.1 // 变异概率
#define CROSSOVER_RATE 0.8 // 交叉概率

// PID参数范围
#define KP_MIN 100.0
#define KP_MAX 800.0
#define KI_MIN 0.0
#define KI_MAX 0.1
#define KD_MIN 50.0
#define KD_MAX 600.0

double monitor_Kp;
double monitor_Ki;
double monitor_Kd;


// 适应度函数：计算ISE（积分平方误差）
double compute_fitness(double Kp, double Ki, double Kd) {
    // 模拟系统响应（二阶系统）
    double y = 0.0;      // 系统输出
    double dy = 0.0;     // 系统输出导数
    double integral = 0.0; // 误差积分
    double error = 0.0;  // 误差
    double dt = 0.01;    // 时间步长
    double setpoint = 1.0; // 参考输入
    double ise = 0.0;    // 积分平方误差

    for (double t = 0; t < 10.0; t += dt) {
        error = setpoint - y;
        integral += error * dt;
        double u = Kp * error + Ki * integral + Kd * (error - dy); // PID控制
        dy = (-2 * dy - y + u) * dt; // 系统动态方程
        y += dy;
        ise += error * error * dt;
    }

    return 1.0 / (ise + 1e-6); // 适应度（误差越小，适应度越高）
}

// 初始化种群
void initialize_population(double population[POP_SIZE][NUM_PARAMS]) {
    srand(time(NULL));
    for (int i = 0; i < POP_SIZE; i++) {
        population[i][0] = KP_MIN + (double)rand() / RAND_MAX * (KP_MAX - KP_MIN); // Kp
        population[i][1] = KI_MIN + (double)rand() / RAND_MAX * (KI_MAX - KI_MIN); // Ki
        population[i][2] = KD_MIN + (double)rand() / RAND_MAX * (KD_MAX - KD_MIN); // Kd
    }
}

// 选择操作（锦标赛选择）
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

// 交叉操作（均匀交叉）
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

// 变异操作（高斯变异）
void mutation(double population[POP_SIZE][NUM_PARAMS]) {
    for (int i = 0; i < POP_SIZE; i++) {
        for (int j = 0; j < NUM_PARAMS; j++) {
            if ((double)rand() / RAND_MAX < MUTATION_RATE) {
                population[i][j] += (double)rand() / RAND_MAX * 0.1 - 0.05; // 小范围扰动
                // 确保参数在范围内
                if (j == 0) population[i][j] = fmax(KP_MIN, fmin(KP_MAX, population[i][j]));
                if (j == 1) population[i][j] = fmax(KI_MIN, fmin(KI_MAX, population[i][j]));
                if (j == 2) population[i][j] = fmax(KD_MIN, fmin(KD_MAX, population[i][j]));
            }
        }
    }
}

// 遗传算法主函数
void genetic_algorithm(positionpid_t *pid_t) 
{
    double population[POP_SIZE][NUM_PARAMS]; // 种群
    double fitness[POP_SIZE]; // 适应度
    double best_fitness = -1.0;
    double best_params[NUM_PARAMS];

    // 初始化种群
    initialize_population(population);

    // 迭代优化
    for (int gen = 0; gen < NUM_GENERATIONS; gen++) {
        // 计算适应度
        for (int i = 0; i < POP_SIZE; i++) {
            fitness[i] = compute_fitness(population[i][0], population[i][1], population[i][2]);
            if (fitness[i] > best_fitness) {
                best_fitness = fitness[i];
                for (int j = 0; j < NUM_PARAMS; j++) {
                    best_params[j] = population[i][j];
                }
            }
        }

        // 选择、交叉、变异
        selection(population, fitness);
        crossover(population);
        mutation(population);
				
				monitor_Kp = best_params[0];
				monitor_Ki = best_params[1];
				monitor_Kd = best_params[2];
        // 输出当前最优解
				pid_t->Kp = best_params[0];
				pid_t->Ki = best_params[1];
				pid_t->Kd = best_params[2];

    }

}
