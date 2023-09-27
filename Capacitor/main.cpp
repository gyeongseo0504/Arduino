#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include <math.h>

int main() {
    double C, V, R;
    printf("커패시터 용량 (C)을 입력하세요: ");
    scanf("%lf", &C);
    printf("전압 (V)을 입력하세요: ");
    scanf("%lf", &V);
    printf("저항 (R)을 입력하세요: ");
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
    printf("Excel 파일이 생성되었습니다.\n");
    return 0;
}