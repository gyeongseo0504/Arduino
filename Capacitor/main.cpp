#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include <math.h>

int main() {
    double C, V, R;
    printf("Ŀ�н��� �뷮 (C)�� �Է��ϼ���: ");
    scanf("%lf", &C);
    printf("���� (V)�� �Է��ϼ���: ");
    scanf("%lf", &V);
    printf("���� (R)�� �Է��ϼ���: ");
    scanf("%lf", &R);

    FILE* file = fopen("Capacitor charging voltage graph.csv", "w");

    fprintf(file, "Time,Charge,Current\n");

    double t;
    for (t = 0.0; t <= 10.0; t += 0.1) {
        double Q = C * V * (1 - exp(-t / (R * C)));
        double I = (V / R) * exp(-t / (R * C));
        fprintf(file, "%.2lf,%.2lf,%.2lf\n", t, Q, I);
    }

    fclose(file);
    printf("Excel ������ �����Ǿ����ϴ�.\n");
    return 0;
}